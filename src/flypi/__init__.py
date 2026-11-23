"""flypi - fly ArduPilot and PX4 drones from a Raspberry Pi with a few lines of Python.

Quick start::

    import flypi

    with flypi.connect("udp:127.0.0.1:14550") as drone:
        drone.wait_ready()
        drone.arm()
        drone.takeoff(5)
        drone.land()
"""

from ._version import __version__
from .drone import DEFAULT_ADDRESS, Drone, connect
from .errors import (
    CommandRejected,
    CommandTimeout,
    ConnectionLost,
    ConnectionTimeout,
    FlypiError,
    ModeError,
    NotArmedError,
    WaitTimeout,
)
from .telemetry import GPS, Attitude, Battery, Position, Telemetry, Velocity

__all__ = [
    "DEFAULT_ADDRESS",
    "GPS",
    "Attitude",
    "Battery",
    "CommandRejected",
    "CommandTimeout",
    "ConnectionLost",
    "ConnectionTimeout",
    "Drone",
    "FlypiError",
    "ModeError",
    "NotArmedError",
    "Position",
    "Telemetry",
    "Velocity",
    "WaitTimeout",
    "__version__",
    "connect",
]
