from __future__ import annotations

import dataclasses
import time

import pytest
from fake_vehicle import free_udp_port

import flypi


def test_version_is_exposed():
    assert flypi.__version__ == "0.1.1"


def test_connect_and_read_telemetry(fly):
    drone, _ = fly()
    assert drone.autopilot == "ArduPilot"
    assert drone.vehicle_type == "QUADROTOR"
    assert drone.target_system == 1
    assert drone.target_component == 1

    t = drone.wait_until(
        lambda t: t.position.lat is not None and t.battery.voltage is not None,
        timeout=5,
        description="telemetry",
    )
    assert t.position.lat == pytest.approx(-35.363261)
    assert t.position.lon == pytest.approx(149.16523)
    assert t.position.alt_msl == pytest.approx(100.0)
    assert t.position.heading == pytest.approx(90.0)
    assert t.battery.voltage == pytest.approx(12.6)
    assert t.battery.remaining == 87
    assert t.gps.fix_type == 3
    assert t.gps.satellites == 12
    assert t.gps.has_3d_fix
    assert t.attitude.yaw == pytest.approx(90.0, abs=0.1)
    assert t.mode == "STABILIZE"
    assert t.armed is False
    assert drone.is_connected
    assert "ArduPilot QUADROTOR sysid=1" in repr(drone)


def test_telemetry_snapshot_is_immutable(fly):
    drone, _ = fly()
    t = drone.telemetry
    with pytest.raises(dataclasses.FrozenInstanceError):
        t.armed = True  # type: ignore[misc]


def test_ignores_other_systems_and_components(fly):
    drone, _ = fly(extra_heartbeats=True)
    assert drone.target_system == 1
    assert drone.target_component == 1
    time.sleep(0.5)  # let plenty of GCS and gimbal heartbeats arrive
    assert drone.mode == "STABILIZE"
    assert drone.autopilot == "ArduPilot"


def test_sends_heartbeats_to_vehicle(fly):
    _, vehicle = fly()
    time.sleep(1.5)
    assert vehicle.gcs_heartbeats_received >= 1


def test_full_flight_ardupilot(fly):
    drone, vehicle = fly()
    drone.wait_ready(timeout=5)
    drone.arm()
    assert drone.armed

    drone.takeoff(5, timeout=10)
    assert drone.mode == "GUIDED"
    assert drone.telemetry.position.alt_rel >= 4.7

    drone.land(timeout=10)
    assert drone.mode == "LAND"
    assert not drone.armed
    assert vehicle.rel_alt == 0.0


def test_full_flight_px4(fly):
    drone, vehicle = fly(autopilot="px4")
    assert drone.autopilot == "PX4"
    assert "TAKEOFF" in drone.available_modes

    drone.wait_ready(timeout=5)
    drone.arm()
    drone.takeoff(4, timeout=10)
    assert drone.telemetry.position.alt_rel >= 3.7
    drone.wait_until(lambda t: t.mode == "LOITER", timeout=5)

    drone.land(timeout=10)
    assert not drone.armed


def test_set_mode(fly):
    drone, _ = fly()
    drone.set_mode("loiter")
    assert drone.mode == "LOITER"
    drone.set_mode("LOITER")  # already in mode: no-op


def test_unknown_mode_raises(fly):
    drone, _ = fly()
    with pytest.raises(flypi.ModeError, match="Available modes"):
        drone.set_mode("WARP_SPEED")


def test_arm_rejected_reports_prearm_reason(fly):
    drone, _ = fly(prearm_ok=False)
    with pytest.raises(flypi.WaitTimeout):
        drone.wait_ready(timeout=1)
    assert not drone.is_armable

    with pytest.raises(flypi.CommandRejected) as excinfo:
        drone.arm()
    assert excinfo.value.result == "MAV_RESULT_FAILED"
    assert "PreArm: Battery below minimum" in str(excinfo.value)
    assert not drone.armed


def test_wait_ready_without_gps(fly):
    drone, _ = fly(gps_fix=1)
    with pytest.raises(flypi.WaitTimeout):
        drone.wait_ready(timeout=1)
    drone.wait_ready(timeout=2, require_gps=False)


def test_takeoff_requires_arming(fly):
    drone, _ = fly()
    with pytest.raises(flypi.NotArmedError):
        drone.takeoff(5)


@pytest.mark.parametrize("bad", [0, -3, float("nan"), float("inf")])
def test_takeoff_rejects_bad_altitude(fly, bad):
    drone, _ = fly()
    with pytest.raises(ValueError):
        drone.takeoff(bad)


def test_unacknowledged_command_times_out(fly):
    drone, _ = fly(ack_commands=False)
    start = time.monotonic()
    with pytest.raises(flypi.CommandTimeout, match="MAV_CMD_COMPONENT_ARM_DISARM"):
        drone.arm(timeout=0.3)
    assert time.monotonic() - start < 3


def test_connect_timeout_when_nothing_is_there():
    with pytest.raises(flypi.ConnectionTimeout, match="No vehicle heartbeat"):
        flypi.connect(f"udpin:127.0.0.1:{free_udp_port()}", timeout=1)


def test_connection_lost_is_detected(fly):
    drone, vehicle = fly(connect_kwargs={"heartbeat_timeout": 0.5})
    vehicle.heartbeats_enabled = False
    with pytest.raises(flypi.ConnectionLost):
        drone.wait_until(lambda t: False, timeout=5)
    assert not drone.is_connected


def test_closed_drone_refuses_commands(fly):
    drone, _ = fly()
    drone.close()
    drone.close()  # idempotent
    with pytest.raises(flypi.FlypiError, match="closed"):
        drone.wait_ready(timeout=1)


def test_context_manager_closes(make_vehicle):
    _, address = make_vehicle()
    with flypi.connect(address, timeout=5) as drone:
        assert drone.is_connected
    assert not drone.is_connected


def test_status_messages_are_collected(fly):
    drone, vehicle = fly()
    vehicle.statustext("Hello from the autopilot")
    drone.wait_until(
        lambda t: "Hello from the autopilot" in drone.status_messages,
        timeout=3,
        description="status text",
    )
