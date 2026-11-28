"""Exceptions raised by flypi.

Every flypi error inherits from :class:`FlypiError`, so you can catch them all
with a single ``except flypi.FlypiError``.
"""

from __future__ import annotations


class FlypiError(Exception):
    """Base class for all flypi errors."""


class ConnectionTimeout(FlypiError, TimeoutError):
    """No vehicle heartbeat was received while connecting."""


class ConnectionLost(FlypiError):
    """The vehicle stopped sending heartbeats while flypi was waiting on it."""


class CommandRejected(FlypiError):
    """The vehicle answered a command with a result other than ACCEPTED.

    Attributes:
        command: MAVLink command name, e.g. ``"MAV_CMD_COMPONENT_ARM_DISARM"``.
        result: MAVLink result name, e.g. ``"MAV_RESULT_FAILED"``.
        status_messages: Recent STATUSTEXT messages from the vehicle, which
            usually explain why (for example ArduPilot ``PreArm:`` messages).
    """

    def __init__(
        self,
        command: str,
        result: str,
        status_messages: list[str] | None = None,
        *,
        detail: str | None = None,
    ):
        self.command = command
        self.result = result
        self.status_messages = list(status_messages or [])
        message = f"{command} was rejected by the vehicle ({result})"
        if detail:
            message += f". {detail}"
        if self.status_messages:
            message += ". Recent vehicle messages: " + " | ".join(self.status_messages)
        super().__init__(message)


class CommandTimeout(FlypiError, TimeoutError):
    """The vehicle never acknowledged a command."""


class WaitTimeout(FlypiError, TimeoutError):
    """A condition (mode change, arming, altitude, ...) was not reached in time."""


class ModeError(FlypiError, ValueError):
    """The requested flight mode does not exist for this vehicle."""


class FlightInterrupted(FlypiError):
    """Something other than your script took over while flypi was waiting.

    Raised when, for example, the vehicle disarms during takeoff, or switches
    out of ``LAND`` before touching down. Usual causes: a failsafe (battery,
    radio, geofence, EKF), the RC pilot changing mode, or another ground
    station. The message includes the vehicle's recent STATUSTEXT messages.

    Attributes:
        mode: The flight mode the vehicle was in when flypi noticed.
    """

    def __init__(self, message: str, mode: str | None = None):
        self.mode = mode
        super().__init__(message)


class NotArmedError(FlypiError):
    """An action needs the vehicle to be armed, but it is disarmed."""
