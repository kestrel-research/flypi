from __future__ import annotations

import pytest
from pymavlink import mavutil

from flypi import modes

mavlink = mavutil.mavlink


def test_autopilot_names():
    assert modes.autopilot_name(mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA) == "ArduPilot"
    assert modes.autopilot_name(mavlink.MAV_AUTOPILOT_PX4) == "PX4"
    assert modes.autopilot_name(mavlink.MAV_AUTOPILOT_GENERIC) == "GENERIC"


def test_vehicle_and_status_names():
    assert modes.vehicle_type_name(mavlink.MAV_TYPE_QUADROTOR) == "QUADROTOR"
    assert modes.system_status_name(mavlink.MAV_STATE_ACTIVE) == "ACTIVE"
    assert modes.vehicle_type_name(9999) == "MAV_TYPE(9999)"


def test_ardupilot_copter_mode_params():
    params = modes.set_mode_params(
        mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA, mavlink.MAV_TYPE_QUADROTOR, "guided"
    )
    assert params == (float(mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED), 4.0, 0.0)


def test_px4_mode_params():
    base, main, sub = modes.set_mode_params(
        mavlink.MAV_AUTOPILOT_PX4, mavlink.MAV_TYPE_QUADROTOR, "LAND"
    )
    assert main == mavutil.PX4_CUSTOM_MAIN_MODE_AUTO
    assert sub == mavutil.PX4_CUSTOM_SUB_MODE_AUTO_LAND


def test_unknown_mode_raises_key_error():
    with pytest.raises(KeyError):
        modes.set_mode_params(
            mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA, mavlink.MAV_TYPE_QUADROTOR, "NOPE"
        )


def test_available_modes():
    copter = modes.available_modes(mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA, mavlink.MAV_TYPE_QUADROTOR)
    assert {"GUIDED", "LAND", "RTL", "LOITER"} <= set(copter)
    px4 = modes.available_modes(mavlink.MAV_AUTOPILOT_PX4, mavlink.MAV_TYPE_QUADROTOR)
    assert {"TAKEOFF", "LAND", "RTL", "OFFBOARD"} <= set(px4)
