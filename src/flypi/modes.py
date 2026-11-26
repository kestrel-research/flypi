"""Flight-mode helpers for ArduPilot and PX4.

ArduPilot and PX4 encode flight modes differently in MAVLink. These helpers
hide that difference so flypi can work with plain mode names such as
``"GUIDED"`` or ``"LAND"``.
"""

from __future__ import annotations

from typing import Any

from pymavlink import mavutil

mavlink = mavutil.mavlink

ARDUPILOT = "ArduPilot"
PX4 = "PX4"

_AUTOPILOT_NAMES = {
    mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA: ARDUPILOT,
    mavlink.MAV_AUTOPILOT_PX4: PX4,
}


def autopilot_name(autopilot: int) -> str:
    """Return ``"ArduPilot"``, ``"PX4"`` or the raw MAVLink enum name."""
    if autopilot in _AUTOPILOT_NAMES:
        return _AUTOPILOT_NAMES[autopilot]
    return _enum_name("MAV_AUTOPILOT", autopilot, "MAV_AUTOPILOT_")


def vehicle_type_name(mav_type: int) -> str:
    """Return a readable vehicle type, e.g. ``"QUADROTOR"``."""
    return _enum_name("MAV_TYPE", mav_type, "MAV_TYPE_")


def system_status_name(status: int) -> str:
    """Return a readable system status, e.g. ``"ACTIVE"``."""
    return _enum_name("MAV_STATE", status, "MAV_STATE_")


def mode_from_heartbeat(heartbeat: Any) -> str:
    """Decode the current flight mode name from a HEARTBEAT message."""
    return str(mavutil.mode_string_v10(heartbeat))


def available_modes(autopilot: int, mav_type: int) -> list[str]:
    """List the flight-mode names this vehicle understands."""
    if autopilot == mavlink.MAV_AUTOPILOT_PX4:
        return sorted(mavutil.px4_map)
    mapping = mavutil.mode_mapping_byname(mav_type)
    return sorted(mapping) if mapping else []


def set_mode_params(autopilot: int, mav_type: int, mode: str) -> tuple[float, float, float]:
    """Return params 1-3 of ``MAV_CMD_DO_SET_MODE`` for the named mode.

    Raises:
        KeyError: if the mode is unknown for this autopilot/vehicle type.
    """
    name = mode.upper()
    if autopilot == mavlink.MAV_AUTOPILOT_PX4:
        base_mode, main_mode, sub_mode = mavutil.px4_map[name]
        return float(base_mode), float(main_mode), float(sub_mode)
    mapping = mavutil.mode_mapping_byname(mav_type)
    if not mapping:
        raise KeyError(name)
    return float(mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED), float(mapping[name]), 0.0


def _enum_name(enum: str, value: int, prefix: str) -> str:
    entry = mavlink.enums.get(enum, {}).get(value)
    if entry is None:
        return f"{enum}({value})"
    name = str(entry.name)
    return name[len(prefix) :] if name.startswith(prefix) else name
