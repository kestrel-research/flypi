"""Telemetry snapshots.

``Drone.telemetry`` returns a :class:`Telemetry` object: an immutable snapshot
of the latest values received from the vehicle. Values the vehicle has not
reported yet are ``None``.
"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class Position:
    """Global position.

    Attributes:
        lat: Latitude in degrees.
        lon: Longitude in degrees.
        alt_msl: Altitude above mean sea level, in metres.
        alt_rel: Altitude above the home position, in metres.
        heading: Heading in degrees (0-360, 0 = north).
    """

    lat: float | None = None
    lon: float | None = None
    alt_msl: float | None = None
    alt_rel: float | None = None
    heading: float | None = None


@dataclass(frozen=True)
class Velocity:
    """Velocity in the local North-East-Down frame, in m/s."""

    north: float | None = None
    east: float | None = None
    down: float | None = None
    groundspeed: float | None = None
    airspeed: float | None = None
    climb_rate: float | None = None


@dataclass(frozen=True)
class Attitude:
    """Vehicle attitude in degrees."""

    roll: float | None = None
    pitch: float | None = None
    yaw: float | None = None


@dataclass(frozen=True)
class Battery:
    """Main battery state.

    Attributes:
        voltage: Battery voltage in volts.
        current: Battery current in amps.
        remaining: Estimated remaining charge in percent (0-100).
    """

    voltage: float | None = None
    current: float | None = None
    remaining: int | None = None


@dataclass(frozen=True)
class GPS:
    """GPS receiver state.

    Attributes:
        fix_type: 0-1 = no fix, 2 = 2D, 3 = 3D, 4+ = DGPS/RTK.
        satellites: Number of visible satellites.
        hdop: Horizontal dilution of precision (lower is better).
    """

    fix_type: int | None = None
    satellites: int | None = None
    hdop: float | None = None

    @property
    def has_3d_fix(self) -> bool:
        return self.fix_type is not None and self.fix_type >= 3


@dataclass(frozen=True)
class Telemetry:
    """A snapshot of everything flypi knows about the vehicle."""

    armed: bool = False
    mode: str | None = None
    autopilot: str = "Unknown"
    vehicle_type: str = "Unknown"
    system_status: str | None = None
    heartbeat_age: float | None = None
    position: Position = field(default_factory=Position)
    velocity: Velocity = field(default_factory=Velocity)
    attitude: Attitude = field(default_factory=Attitude)
    battery: Battery = field(default_factory=Battery)
    gps: GPS = field(default_factory=GPS)
