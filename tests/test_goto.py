from __future__ import annotations

import threading
import time

import pytest
from fake_vehicle import APM_MODES, HOME_LAT, HOME_LON
from pymavlink import mavutil

import flypi
from flypi import drone as drone_module

mavlink = mavutil.mavlink
REPOSITION = mavlink.MAV_CMD_DO_REPOSITION
CHANGE_SPEED = mavlink.MAV_CMD_DO_CHANGE_SPEED

# Roughly 30 m north and 20 m east of the fake vehicle's home.
TARGET_LAT = HOME_LAT + 0.00027
TARGET_LON = HOME_LON + 0.00022


def airborne(fly, altitude: float = 5, **kwargs):
    drone, vehicle = fly(**kwargs)
    drone.wait_ready(timeout=5)
    drone.arm()
    drone.takeoff(altitude, timeout=10)
    return drone, vehicle


def distance_from(drone, lat: float, lon: float) -> float:
    pos = drone.telemetry.position
    return drone_module._distance_m(pos.lat, pos.lon, lat, lon)


def test_goto_ardupilot(fly):
    drone, vehicle = airborne(fly)
    drone.goto(TARGET_LAT, TARGET_LON, 8, timeout=10)

    assert drone.mode == "GUIDED"
    assert distance_from(drone, TARGET_LAT, TARGET_LON) <= 1.0
    assert drone.telemetry.position.alt_rel == pytest.approx(8, abs=1.0)
    assert vehicle.target_lat == pytest.approx(TARGET_LAT)
    assert vehicle.target_alt == pytest.approx(8)
    assert vehicle.position_targets_received == 1  # confirmed first time, no resend


def test_goto_keeps_altitude_by_default(fly):
    drone, vehicle = airborne(fly, altitude=6)
    drone.goto(TARGET_LAT, TARGET_LON, timeout=10)
    assert vehicle.target_alt == pytest.approx(6, abs=0.3)


def test_goto_relative(fly):
    drone, vehicle = airborne(fly)
    start = drone.telemetry.position
    drone.goto_relative(north=20, east=-10, up=2, timeout=10)

    end = drone.telemetry.position
    assert drone_module._distance_m(start.lat, start.lon, end.lat, end.lon) == pytest.approx(
        22.4, abs=1.0
    )
    assert end.lat > start.lat and end.lon < start.lon
    assert vehicle.target_alt == pytest.approx(start.alt_rel + 2, abs=0.1)


def test_goto_sets_groundspeed_on_ardupilot(fly):
    drone, vehicle = airborne(fly)
    drone.goto(TARGET_LAT, TARGET_LON, groundspeed=15, timeout=10)
    assert CHANGE_SPEED in vehicle.commands_received
    assert vehicle.cruise_speed == 15


def test_goto_px4_uses_acknowledged_reposition(fly):
    drone, vehicle = airborne(fly, autopilot="px4")
    drone.goto(TARGET_LAT, TARGET_LON, 7, groundspeed=12, timeout=10)

    assert REPOSITION in vehicle.commands_received
    assert vehicle.position_targets_received == 0  # no unconfirmed setpoints on PX4
    assert drone.mode == "LOITER"
    assert vehicle.cruise_speed == 12
    assert distance_from(drone, TARGET_LAT, TARGET_LON) <= 1.0
    assert drone.telemetry.position.alt_rel == pytest.approx(7, abs=1.0)


def test_lost_position_target_is_resent(fly):
    drone, vehicle = airborne(fly, drop_position_targets=1)
    drone.goto(TARGET_LAT, TARGET_LON, timeout=10)
    assert vehicle.position_targets_received == 2
    assert distance_from(drone, TARGET_LAT, TARGET_LON) <= 1.0


def test_target_echo_in_relative_frame_is_understood(fly):
    drone, vehicle = airborne(fly, echo_frame=mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT)
    drone.goto(TARGET_LAT, TARGET_LON, 9, timeout=10)
    assert vehicle.position_targets_received == 1


def test_ignored_target_without_any_target_is_rejected(fly):
    drone, vehicle = airborne(fly, ignore_position_targets=True)
    with pytest.raises(flypi.CommandRejected, match="never reported a position target") as exc:
        drone.goto(TARGET_LAT, TARGET_LON, timeout=10)
    assert exc.value.command == "SET_POSITION_TARGET_GLOBAL_INT"
    assert exc.value.result == "NOT_APPLIED"
    assert vehicle.position_targets_received == drone_module._GOTO_ATTEMPTS


def test_ignored_target_reports_the_vehicles_target(fly):
    drone, vehicle = airborne(fly)
    drone.goto_relative(north=5, timeout=10)
    vehicle.ignore_position_targets = True
    start = time.monotonic()
    with pytest.raises(flypi.CommandRejected, match="different target"):
        drone.goto(TARGET_LAT, TARGET_LON, timeout=30)
    assert time.monotonic() - start < 6  # fails fast; does not wait out the timeout


def test_goto_interrupted_by_mode_change(fly):
    drone, vehicle = airborne(fly, cruise_speed=2.0)

    def pilot_takes_over() -> None:
        time.sleep(0.5)
        vehicle.custom_mode = APM_MODES["LOITER"]
        vehicle.statustext("RC pilot switched to LOITER")

    threading.Thread(target=pilot_takes_over, daemon=True).start()
    with pytest.raises(flypi.FlightInterrupted, match="GUIDED to LOITER") as exc:
        drone.goto(TARGET_LAT, TARGET_LON, timeout=30)
    assert exc.value.mode == "LOITER"


def test_goto_without_wait_returns_at_once(fly):
    drone, _ = airborne(fly, cruise_speed=5.0)
    start = time.monotonic()
    drone.goto(TARGET_LAT, TARGET_LON, wait=False)
    assert time.monotonic() - start < 2
    assert distance_from(drone, TARGET_LAT, TARGET_LON) > 10
    drone.wait_until(
        lambda t: (
            drone_module._distance_m(t.position.lat, t.position.lon, TARGET_LAT, TARGET_LON) < 1
        ),
        timeout=15,
    )


def test_goto_timeout_is_a_wait_timeout(fly):
    drone, _ = airborne(fly, cruise_speed=1.0)
    with pytest.raises(flypi.WaitTimeout, match="arrival at"):
        drone.goto(TARGET_LAT, TARGET_LON, timeout=0.5)


def test_goto_requires_arming(fly):
    drone, _ = fly()
    drone.wait_ready(timeout=5)
    with pytest.raises(flypi.NotArmedError):
        drone.goto(TARGET_LAT, TARGET_LON, 5)


@pytest.mark.parametrize(
    "kwargs",
    [
        {"lat": 91.0},
        {"lat": float("nan")},
        {"lon": -181.0},
        {"altitude": float("inf")},
        {"groundspeed": 0},
        {"groundspeed": -2},
        {"tolerance": 0},
    ],
)
def test_goto_rejects_bad_arguments(fly, kwargs):
    drone, _ = fly()
    args = {"lat": TARGET_LAT, "lon": TARGET_LON, "altitude": 5.0} | kwargs
    lat, lon = args.pop("lat"), args.pop("lon")
    with pytest.raises(ValueError):
        drone.goto(lat, lon, **args)


def test_goto_relative_rejects_bad_offsets(fly):
    drone, _ = fly()
    with pytest.raises(ValueError, match="north"):
        drone.goto_relative(north=float("nan"))


def test_distance_and_offset_helpers_agree():
    lat, lon = drone_module._offset_position(HOME_LAT, HOME_LON, 100.0, -50.0)
    assert drone_module._distance_m(HOME_LAT, HOME_LON, lat, lon) == pytest.approx(111.8, abs=0.2)
    assert drone_module._distance_m(0, 0, 0, 1) == pytest.approx(111_319.5, rel=1e-4)
    # Offsets across the antimeridian wrap into [-180, 180).
    _, wrapped = drone_module._offset_position(0.0, 179.9999, 0.0, 100.0)
    assert -180.0 <= wrapped < -179.99
