"""The :class:`Drone` class and :func:`connect`.

flypi keeps one background thread per connection. That thread:

* reads every MAVLink message from the vehicle and updates the telemetry;
* sends a heartbeat once a second, so the autopilot knows a controller is
  still attached (this matters for ground-station failsafes).

Your code calls simple blocking methods such as :meth:`Drone.arm` or
:meth:`Drone.takeoff`. Each one sends a command, checks the vehicle's
acknowledgement, and (optionally) waits until the vehicle has actually done it.
"""

from __future__ import annotations

import contextlib
import logging
import math
import threading
import time
from collections import deque
from collections.abc import Callable
from typing import Any

from pymavlink import mavutil

from . import modes as _modes
from .errors import (
    CommandRejected,
    CommandTimeout,
    ConnectionLost,
    ConnectionTimeout,
    FlightInterrupted,
    FlypiError,
    ModeError,
    NotArmedError,
    WaitTimeout,
)
from .telemetry import GPS, Attitude, Battery, Position, Telemetry, Velocity

mavlink = mavutil.mavlink
log = logging.getLogger("flypi")

DEFAULT_ADDRESS = "udp:127.0.0.1:14550"
"""Where ArduPilot SITL (via MAVProxy) and PX4 SITL send MAVLink by default."""

_FORCE_MAGIC = 21196  # MAV_CMD_COMPONENT_ARM_DISARM param2 that bypasses checks
_HEARTBEAT_PERIOD = 1.0
_STATUS_WINDOW = 10.0  # seconds of STATUSTEXT history attached to errors
_ARMING_WINDOW = 60.0  # how far back to look for PreArm:/Arm: reasons on a refused arm
_REJECT_GRACE = 0.5  # max seconds to wait for the vehicle to explain a rejection
_REJECT_QUIET = 0.15  # ...but stop once its messages have been quiet this long
_TEMP_REJECT_BACKOFF = 0.5  # seconds between retries of a TEMPORARILY_REJECTED command
_PROGRESS_TIMEOUT = 60.0  # hard cap on a command that keeps answering IN_PROGRESS
_STATUSTEXT_LEN = 50  # MAVLink STATUSTEXT chunk size
_UINT16_MAX = 65535
_ARMING_PREFIXES = ("PreArm:", "Arm:")
# Modes a vehicle may legitimately pass through while landing / returning home.
_TAKEOFF_MODES = frozenset({"GUIDED", "TAKEOFF"})  # ArduPilot; Plane may use TAKEOFF
_LAND_MODES = frozenset({"LAND", "QLAND"})
_RTL_MODES = frozenset({"RTL", "LAND", "QRTL", "QLAND", "SMART_RTL"})
_TEMPORARILY_REJECTED = object()  # sentinel returned by Drone._await_ack

# Messages flypi asks the vehicle to stream, with their rates in Hz.
_STREAM_RATES = {
    mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT: 5,
    mavlink.MAVLINK_MSG_ID_SYS_STATUS: 2,
    mavlink.MAVLINK_MSG_ID_GPS_RAW_INT: 2,
    mavlink.MAVLINK_MSG_ID_ATTITUDE: 10,
    mavlink.MAVLINK_MSG_ID_VFR_HUD: 4,
}


def connect(
    address: str = DEFAULT_ADDRESS,
    *,
    baud: int = 57600,
    timeout: float = 30.0,
    target_system: int | None = None,
    source_system: int = 255,
    source_component: int = mavlink.MAV_COMP_ID_MISSIONPLANNER,
    send_heartbeat: bool = True,
    request_streams: bool = True,
    heartbeat_timeout: float = 5.0,
) -> Drone:
    """Connect to a vehicle and wait for its first heartbeat.

    Args:
        address: A pymavlink connection string. Common examples:

            * ``"udp:127.0.0.1:14550"`` - listen for SITL or a MAVLink router.
            * ``"tcp:127.0.0.1:5760"`` - connect straight to ArduPilot SITL.
            * ``"/dev/serial0"`` - Raspberry Pi UART wired to the flight controller.
            * ``"/dev/ttyACM0"`` - flight controller over USB.
        baud: Baud rate for serial connections (ignored for UDP/TCP).
        timeout: Seconds to wait for the vehicle's first heartbeat.
        target_system: Only accept this MAVLink system ID. By default flypi
            uses the first autopilot it hears.
        source_system: MAVLink system ID flypi uses for itself.
        source_component: MAVLink component ID flypi uses for itself.
        send_heartbeat: Send a 1 Hz ground-station heartbeat to the vehicle.
        request_streams: Ask the vehicle to stream the telemetry flypi uses.
            Each request is confirmed; any the vehicle refuses are listed in
            :attr:`Drone.unconfirmed_streams`.
        heartbeat_timeout: Seconds without a vehicle heartbeat before waits
            raise :class:`~flypi.ConnectionLost`.

    Returns:
        A connected :class:`Drone`. Use it as a context manager (``with``) or
        call :meth:`Drone.close` when you are done.

    Raises:
        ConnectionTimeout: if no vehicle heartbeat arrives within ``timeout``.
    """
    link = mavutil.mavlink_connection(
        address,
        baud=baud,
        source_system=source_system,
        source_component=source_component,
        autoreconnect=True,
    )
    try:
        heartbeat = _wait_for_vehicle(link, address, timeout, target_system, send_heartbeat)
        return Drone(
            link,
            heartbeat,
            send_heartbeat=send_heartbeat,
            request_streams=request_streams,
            heartbeat_timeout=heartbeat_timeout,
        )
    except BaseException:
        link.close()
        raise


class Drone:
    """A connected vehicle. Create one with :func:`flypi.connect`."""

    def __init__(
        self,
        link: Any,
        heartbeat: Any,
        *,
        send_heartbeat: bool = True,
        request_streams: bool = True,
        heartbeat_timeout: float = 5.0,
    ) -> None:
        self._link = link
        self.target_system: int = heartbeat.get_srcSystem()
        self.target_component: int = heartbeat.get_srcComponent()
        link.target_system = self.target_system
        link.target_component = self.target_component

        self.heartbeat_timeout = heartbeat_timeout
        self._autopilot_id: int = heartbeat.autopilot
        self._mav_type: int = heartbeat.type
        self._send_heartbeat = send_heartbeat

        self._cond = threading.Condition(threading.RLock())
        self._send_lock = threading.Lock()
        self._closed = False

        self._armed = False
        self._mode: str | None = None
        self._system_status: str | None = None
        self._last_heartbeat = time.monotonic()
        self._position: dict[str, float | None] = {}
        self._velocity: dict[str, float | None] = {}
        self._attitude: dict[str, float | None] = {}
        self._battery: dict[str, Any] = {}
        self._gps: dict[str, Any] = {}
        self._sensors_present: int | None = None
        self._sensors_health: int | None = None
        self._acks: dict[int, Any] = {}
        self._command_locks: dict[int, threading.Lock] = {}
        self._status_messages: deque[tuple[float, str]] = deque(maxlen=100)
        self._statustext_chunks: dict[int, list[str]] = {}
        self._thread_error: BaseException | None = None
        self._source_system: int = getattr(link, "source_system", 0) or 0
        self._source_component: int = getattr(link, "source_component", 0) or 0
        self.unconfirmed_streams: list[str] = []
        """Telemetry stream requests the vehicle did not confirm (see :func:`connect`)."""

        self._handle_heartbeat(heartbeat)

        self._thread = threading.Thread(target=self._run, name="flypi-link", daemon=True)
        self._thread.start()
        if request_streams:
            try:
                self._request_streams()
            except BaseException:
                self.close()  # do not leave the link thread running behind a failed connect
                raise

    # ------------------------------------------------------------------ info

    @property
    def autopilot(self) -> str:
        """``"ArduPilot"``, ``"PX4"`` or another MAVLink autopilot name."""
        return _modes.autopilot_name(self._autopilot_id)

    @property
    def vehicle_type(self) -> str:
        """Vehicle type, e.g. ``"QUADROTOR"`` or ``"FIXED_WING"``."""
        return _modes.vehicle_type_name(self._mav_type)

    @property
    def telemetry(self) -> Telemetry:
        """An immutable snapshot of the latest telemetry."""
        with self._cond:
            return self._snapshot()

    @property
    def armed(self) -> bool:
        """``True`` if the motors are armed."""
        with self._cond:
            return self._armed

    @property
    def mode(self) -> str | None:
        """Current flight-mode name, e.g. ``"GUIDED"``."""
        with self._cond:
            return self._mode

    @property
    def is_connected(self) -> bool:
        """``True`` while heartbeats keep arriving from the vehicle."""
        with self._cond:
            return not self._closed and self._heartbeat_age() < self.heartbeat_timeout

    @property
    def is_armable(self) -> bool:
        """``True`` when the vehicle reports it is ready to arm.

        This means the autopilot's pre-arm checks pass (where it reports them,
        as ArduPilot does) and there is a 3D GPS fix with a known position.
        """
        with self._cond:
            return self._is_armable(require_gps=True)

    @property
    def available_modes(self) -> list[str]:
        """Flight-mode names this vehicle accepts in :meth:`set_mode`."""
        return _modes.available_modes(self._autopilot_id, self._mav_type)

    @property
    def status_messages(self) -> list[str]:
        """Recent STATUSTEXT messages from the vehicle, oldest first."""
        with self._cond:
            return [text for _, text in self._status_messages]

    # ------------------------------------------------------------- commands

    def wait_ready(self, timeout: float = 60.0, *, require_gps: bool = True) -> Telemetry:
        """Block until the vehicle is ready to arm.

        Args:
            timeout: Seconds to wait.
            require_gps: Also wait for a 3D GPS fix and a known position. Set
                to ``False`` for indoor, non-GPS setups.

        Raises:
            WaitTimeout: if the vehicle is not ready in time. The error lists
                the vehicle's recent messages, which usually say why.
        """
        what = "the vehicle to be ready to arm"
        if require_gps:
            what += " (pre-arm checks + 3D GPS fix)"
        return self._wait(lambda: self._is_armable(require_gps), timeout, what)

    def set_mode(self, mode: str, *, timeout: float = 5.0) -> None:
        """Change flight mode and wait until the vehicle reports it.

        Args:
            mode: Mode name such as ``"GUIDED"``, ``"LOITER"`` or ``"LAND"``.
                See :attr:`available_modes` for this vehicle's list.
            timeout: Seconds to wait for the mode change.

        Raises:
            ModeError: if this vehicle has no mode with that name.
            CommandRejected: if the vehicle refuses the mode change.
        """
        name = mode.upper()
        try:
            params = _modes.set_mode_params(self._autopilot_id, self._mav_type, name)
        except KeyError:
            raise ModeError(
                f"Unknown mode {mode!r} for this {self.autopilot} vehicle. "
                f"Available modes: {', '.join(self.available_modes) or 'none'}"
            ) from None
        if self.mode == name:
            return
        self._command_long(mavlink.MAV_CMD_DO_SET_MODE, params)
        self._wait(lambda: self._mode == name, timeout, f"mode {name}")

    def arm(self, *, force: bool = False, timeout: float = 10.0) -> None:
        """Arm the motors and wait until the vehicle reports armed.

        ArduPilot disarms again automatically if you do not take off within
        a few seconds, so call :meth:`takeoff` soon after arming.

        Args:
            force: Skip the autopilot's pre-arm safety checks. Dangerous; only
                use this on the bench with propellers removed.
            timeout: Seconds to wait for the vehicle to arm.

        Raises:
            CommandRejected: if the vehicle refuses to arm (the error includes
                its ``PreArm:`` messages).
        """
        if self.armed:
            return
        self._command_long(
            mavlink.MAV_CMD_COMPONENT_ARM_DISARM, (1, _FORCE_MAGIC if force else 0), timeout=timeout
        )
        self._wait(lambda: self._armed, timeout, "the vehicle to arm")

    def disarm(self, *, force: bool = False, timeout: float = 10.0) -> None:
        """Disarm the motors and wait until the vehicle reports disarmed.

        Args:
            force: Disarm even while flying. This stops the motors in the air.
            timeout: Seconds to wait for the vehicle to disarm.
        """
        if not self.armed:
            return
        self._command_long(
            mavlink.MAV_CMD_COMPONENT_ARM_DISARM, (0, _FORCE_MAGIC if force else 0), timeout=timeout
        )
        self._wait(lambda: not self._armed, timeout, "the vehicle to disarm")

    def takeoff(self, altitude: float, *, wait: bool = True, timeout: float = 60.0) -> None:
        """Take off vertically to ``altitude`` metres above home.

        The vehicle must already be armed (see :meth:`arm`). On ArduPilot,
        flypi switches to ``GUIDED`` mode first if needed.

        Args:
            altitude: Target height above the home position, in metres.
            wait: Block until the vehicle is close to the target altitude.
            timeout: Seconds to wait when ``wait`` is ``True``.

        Raises:
            NotArmedError: if the vehicle is not armed.
            ValueError: if ``altitude`` is not a positive number.
        """
        if not altitude > 0 or not math.isfinite(altitude):
            raise ValueError(f"altitude must be a positive number of metres, got {altitude!r}")
        if not self.armed:
            raise NotArmedError("The vehicle must be armed before takeoff. Call drone.arm() first.")

        nan = math.nan
        if self._autopilot_id == mavlink.MAV_AUTOPILOT_PX4:
            # PX4 expects the takeoff altitude above mean sea level.
            with self._cond:
                alt_msl = self._position.get("alt_msl")
                alt_rel = self._position.get("alt_rel")
            if alt_msl is None or alt_rel is None:
                raise FlypiError("No position yet; call drone.wait_ready() before takeoff.")
            target = (alt_msl - alt_rel) + altitude
            params = (0, 0, 0, nan, nan, nan, target)
        else:
            self.set_mode("GUIDED")
            params = (0, 0, 0, 0, 0, 0, altitude)
        self._command_long(mavlink.MAV_CMD_NAV_TAKEOFF, params)

        if wait:
            reached = altitude - max(0.3, 0.05 * altitude)
            # ArduPilot must stay in GUIDED; leaving it means a failsafe or the pilot
            # took over. PX4 moves through TAKEOFF to HOLD by itself, so only arming
            # is checked there.
            hold_modes = None if self._autopilot_id == mavlink.MAV_AUTOPILOT_PX4 else _TAKEOFF_MODES

            def at_altitude() -> bool:
                self._check_flight("takeoff", hold_modes)
                alt = self._position.get("alt_rel")
                return alt is not None and alt >= reached

            self._wait(at_altitude, timeout, f"altitude {altitude:g} m")

    def land(self, *, wait: bool = True, timeout: float = 120.0) -> None:
        """Land at the current position.

        Args:
            wait: Block until the vehicle has landed and disarmed.
            timeout: Seconds to wait when ``wait`` is ``True``.

        Raises:
            FlightInterrupted: if the vehicle leaves ``LAND`` mode before it
                has disarmed (for example the RC pilot took control).
        """
        self.set_mode("LAND")
        if wait:
            self._wait(
                lambda: self._landed_or_check("landing", _LAND_MODES),
                timeout,
                "landing (vehicle to disarm)",
            )

    def return_to_launch(self, *, wait: bool = False, timeout: float = 300.0) -> None:
        """Fly back to the home position and land (RTL mode).

        Args:
            wait: Block until the vehicle has landed and disarmed.
            timeout: Seconds to wait when ``wait`` is ``True``.

        Raises:
            FlightInterrupted: (only with ``wait=True``) if the vehicle leaves
                RTL before it has disarmed.
        """
        self.set_mode("RTL")
        if wait:
            self._wait(
                lambda: self._landed_or_check("return to launch", _RTL_MODES),
                timeout,
                "return to launch (vehicle to disarm)",
            )

    def wait_until(
        self,
        condition: Callable[[Telemetry], bool],
        timeout: float,
        description: str = "condition",
    ) -> Telemetry:
        """Block until ``condition(telemetry)`` returns ``True``.

        Example::

            drone.wait_until(lambda t: t.battery.remaining is not None, timeout=10)

        Returns:
            The telemetry snapshot that satisfied the condition.

        Raises:
            WaitTimeout: if the condition is not met within ``timeout`` seconds.
            ConnectionLost: if the vehicle stops sending heartbeats.
        """
        return self._wait(lambda: condition(self._snapshot()), timeout, description)

    def close(self) -> None:
        """Stop the background thread and close the MAVLink connection."""
        with self._cond:
            if self._closed:
                return
            self._closed = True
            self._cond.notify_all()
        self._thread.join(timeout=2.0)
        try:
            self._link.close()
        except Exception:  # pragma: no cover - best effort on shutdown
            log.debug("error closing MAVLink link", exc_info=True)

    def __enter__(self) -> Drone:
        return self

    def __exit__(self, *exc_info: object) -> None:
        self.close()

    def __repr__(self) -> str:
        return (
            f"<Drone {self.autopilot} {self.vehicle_type} sysid={self.target_system} "
            f"mode={self.mode} armed={self.armed}>"
        )

    # -------------------------------------------------------------- internals

    def _run(self) -> None:
        """Background thread: read messages and send heartbeats."""
        try:
            self._loop()
        except BaseException as exc:  # pragma: no cover - defensive
            log.critical("flypi link thread crashed", exc_info=True)
            with self._cond:
                self._thread_error = exc
                self._cond.notify_all()

    def _loop(self) -> None:
        next_heartbeat = 0.0
        while not self._closed:
            try:
                now = time.monotonic()
                if self._send_heartbeat and now >= next_heartbeat:
                    self._send(_send_gcs_heartbeat)
                    next_heartbeat = now + _HEARTBEAT_PERIOD
                msg = self._link.recv_match(blocking=True, timeout=0.1)
            except Exception:
                if self._closed:  # close() ran on another thread mid-read
                    break  # type: ignore[unreachable]
                log.warning("MAVLink link error; retrying", exc_info=True)
                time.sleep(0.2)
                continue
            if msg is None:
                continue
            with self._cond:
                try:
                    self._handle(msg)
                except Exception:
                    # One malformed message must never stop telemetry or heartbeats.
                    log.warning("ignoring a %s message flypi could not parse", msg, exc_info=True)
                self._cond.notify_all()

    def _send(self, fn: Callable[[Any], None]) -> None:
        try:
            with self._send_lock:
                fn(self._link.mav)
        except OSError as exc:
            raise ConnectionLost(f"Could not send to the vehicle: {exc}") from exc

    def _request_streams(self) -> None:
        """Ask for telemetry streams, confirming each request with the vehicle.

        A vehicle refusing a stream is not fatal (flypi still works on whatever
        the vehicle sends by default), so failures are logged and recorded in
        :attr:`unconfirmed_streams` instead of raised.
        """
        ts, tc = self.target_system, self.target_component
        for msg_id, hz in _STREAM_RATES.items():
            msg_name = mavlink.mavlink_map[msg_id].msgname
            try:
                self._command_long(
                    mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                    (msg_id, 1_000_000 / hz),
                    timeout=1.0,
                    retries=1,
                    explain=False,
                )
            except CommandRejected as exc:
                self.unconfirmed_streams.append(msg_name)
                log.warning("vehicle refused to stream %s at %g Hz (%s)", msg_name, hz, exc.result)
            except CommandTimeout:
                # This vehicle does not answer SET_MESSAGE_INTERVAL at all, so do not
                # spend seconds per stream finding that out again.
                remaining = [mavlink.mavlink_map[m].msgname for m in _STREAM_RATES]
                self.unconfirmed_streams.extend(remaining[remaining.index(msg_name) :])
                log.warning(
                    "vehicle does not acknowledge MAV_CMD_SET_MESSAGE_INTERVAL; "
                    "relying on its default telemetry streams"
                )
                break
        if self._autopilot_id == mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
            # Older ArduPilot firmware only understands the legacy stream request.
            # REQUEST_DATA_STREAM is a message, not a command: MAVLink has no ACK for it.
            self._send(
                lambda mav: mav.request_data_stream_send(ts, tc, mavlink.MAV_DATA_STREAM_ALL, 4, 1)
            )

    def _command_long(
        self,
        command: int,
        params: tuple[float, ...],
        *,
        timeout: float = 3.0,
        retries: int = 2,
        explain: bool = True,
    ) -> Any:
        """Send COMMAND_LONG and wait until the vehicle ACKs it as ACCEPTED.

        * No ACK within ``timeout``: resend (with MAVLink's ``confirmation``
          counter incremented), up to ``retries`` times.
        * ``IN_PROGRESS``: the vehicle is working on it. flypi never resends a
          command that is in progress; it waits for the final ACK, up to a
          hard cap (60 s).
        * ``TEMPORARILY_REJECTED``: back off and retry, like a missing ACK.
        * Any other result: raise :class:`CommandRejected`, after giving the
          vehicle a moment to send the STATUSTEXT that explains why.
        """
        name = _enum_name("MAV_CMD", command)
        padded = tuple(params) + (0,) * (7 - len(params))
        ts, tc = self.target_system, self.target_component
        with self._cond:
            lock = self._command_locks.setdefault(command, threading.Lock())
        # Two threads sending the same command would each see the other's ACK.
        with lock:
            sent_at = time.monotonic()
            for attempt in range(retries + 1):
                with self._cond:
                    self._acks.pop(command, None)
                confirmation = min(attempt, 255)

                def send(mav: Any, confirmation: int = confirmation) -> None:
                    mav.command_long_send(ts, tc, command, confirmation, *padded)

                self._send(send)
                outcome = self._await_ack(command, name, timeout, sent_at, explain)
                if outcome is not None and outcome is not _TEMPORARILY_REJECTED:
                    return outcome
                if attempt < retries:
                    log.debug("no usable ACK for %s (attempt %d); retrying", name, attempt + 1)
            if outcome is _TEMPORARILY_REJECTED:
                with self._cond:
                    raise self._rejection(
                        command, name, mavlink.MAV_RESULT_TEMPORARILY_REJECTED, sent_at, explain
                    )
            raise CommandTimeout(
                f"{name} was not acknowledged after {retries + 1} attempts "
                f"({timeout:g}s each). The vehicle is sending heartbeats but not answering "
                "commands: check the link is two-way (serial TX wired, UDP return path "
                "reachable) and that no other program is reading this connection."
            )

    def _await_ack(
        self, command: int, name: str, timeout: float, sent_at: float, explain: bool
    ) -> Any:
        """Wait for one attempt's ACK.

        Returns the ACCEPTED ACK, ``None`` if no ACK arrived, or
        ``_TEMPORARILY_REJECTED``; both of the latter mean "retry".
        Raises for a definite rejection, or a command stuck IN_PROGRESS.
        """
        deadline = time.monotonic() + timeout
        progress_cap: float | None = None
        with self._cond:
            while True:
                self._check_alive()
                ack = self._acks.pop(command, None)
                if ack is not None:
                    result = ack.result
                    if result == mavlink.MAV_RESULT_ACCEPTED:
                        return ack
                    if result == mavlink.MAV_RESULT_IN_PROGRESS:
                        now = time.monotonic()
                        if progress_cap is None:
                            progress_cap = now + max(_PROGRESS_TIMEOUT, timeout)
                            log.debug("%s is in progress", name)
                        # MAVLink does not promise regular progress updates, so after the
                        # first IN_PROGRESS keep waiting for the final ACK, up to the cap.
                        deadline = progress_cap
                        continue
                    if result == mavlink.MAV_RESULT_TEMPORARILY_REJECTED:
                        self._cond.wait(_TEMP_REJECT_BACKOFF)
                        return _TEMPORARILY_REJECTED
                    raise self._rejection(command, name, result, sent_at, explain)
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    if progress_cap is not None:
                        # Resending a command the vehicle is already executing could
                        # restart it (e.g. a calibration), so this is final.
                        raise CommandTimeout(
                            self._explain(
                                f"{name} was accepted as IN_PROGRESS but the vehicle never "
                                "reported that it finished"
                            )
                        )
                    return None
                self._cond.wait(min(remaining, 0.2))

    def _rejection(
        self, command: int, name: str, result: int, sent_at: float, explain: bool
    ) -> CommandRejected:
        """Build a CommandRejected, waiting briefly for the vehicle's explanation.

        Autopilots often send the ACK first and the reason a moment later
        (ArduPilot queues STATUSTEXT, so ``PreArm: ...`` can trail the FAILED ACK).
        """
        result_name = _enum_name("MAV_RESULT", result)
        if not explain:
            return CommandRejected(name, result_name)
        rejected_at = time.monotonic()
        deadline = rejected_at + _REJECT_GRACE
        while True:
            now = time.monotonic()
            last = self._status_messages[-1][0] if self._status_messages else 0.0
            if last >= rejected_at and now - last >= _REJECT_QUIET:
                break  # the vehicle explained itself and has gone quiet
            if now >= deadline:
                break
            self._cond.wait(min(deadline - now, 0.05))

        messages = [text for t, text in self._status_messages if t >= sent_at]
        if not messages:
            messages = self._recent_status()
        if command == mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            cutoff = time.monotonic() - _ARMING_WINDOW
            reasons = [
                text
                for t, text in self._status_messages
                if t >= cutoff and text.startswith(_ARMING_PREFIXES) and text not in messages
            ]
            messages = _dedupe(messages + reasons)
        return CommandRejected(name, result_name, messages)

    def _check_flight(self, phase: str, hold_modes: frozenset[str] | None) -> None:
        """Raise FlightInterrupted if the vehicle disarmed or left ``hold_modes``."""
        if not self._armed:
            raise FlightInterrupted(
                self._explain(f"The vehicle disarmed during {phase}"), mode=self._mode
            )
        if hold_modes is not None and self._mode not in hold_modes:
            raise FlightInterrupted(
                self._explain(
                    f"{phase.capitalize()} interrupted: the vehicle switched from GUIDED "
                    f"to {self._mode} (a failsafe, the RC pilot or another ground station)"
                ),
                mode=self._mode,
            )

    def _landed_or_check(self, phase: str, allowed_modes: frozenset[str]) -> bool:
        if not self._armed:
            return True
        if self._mode not in allowed_modes:
            raise FlightInterrupted(
                self._explain(
                    f"{phase.capitalize()} interrupted: the vehicle switched to {self._mode} "
                    "before landing (a failsafe, the RC pilot or another ground station)"
                ),
                mode=self._mode,
            )
        return False

    def _explain(self, message: str) -> str:
        recent = self._recent_status()
        if recent:
            message += ". Recent vehicle messages: " + " | ".join(recent)
        return message

    def _wait(self, predicate: Callable[[], bool], timeout: float, description: str) -> Telemetry:
        deadline = time.monotonic() + timeout
        with self._cond:
            while True:
                self._check_alive()
                if predicate():
                    return self._snapshot()
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise WaitTimeout(
                        self._explain(f"Timed out after {timeout:g}s waiting for {description}")
                    )
                self._cond.wait(min(remaining, 0.2))

    def _check_alive(self) -> None:
        if self._closed:
            raise FlypiError("This Drone connection is closed.")
        if self._thread_error is not None:
            raise ConnectionLost(
                f"flypi's link thread stopped unexpectedly ({self._thread_error!r}); "
                "telemetry and heartbeats are no longer flowing."
            ) from self._thread_error
        age = self._heartbeat_age()
        if age > self.heartbeat_timeout:
            raise ConnectionLost(
                self._explain(
                    f"No heartbeat from the vehicle for {age:.1f}s "
                    f"(heartbeat_timeout={self.heartbeat_timeout:g}s)"
                )
            )

    def _heartbeat_age(self) -> float:
        return time.monotonic() - self._last_heartbeat

    def _is_armable(self, require_gps: bool) -> bool:
        prearm = mavlink.MAV_SYS_STATUS_PREARM_CHECK
        if self._sensors_present is None:
            # No SYS_STATUS yet: we cannot know whether pre-arm checks pass.
            prearm_ok = False
        elif self._sensors_present & prearm:
            prearm_ok = bool((self._sensors_health or 0) & prearm)
        else:
            prearm_ok = True  # autopilot does not report pre-arm state (e.g. PX4)
        if not require_gps:
            return prearm_ok
        fix = self._gps.get("fix_type")
        has_fix = fix is not None and fix >= 3
        return prearm_ok and has_fix and self._position.get("lat") is not None

    def _recent_status(self) -> list[str]:
        cutoff = time.monotonic() - _STATUS_WINDOW
        return [text for t, text in self._status_messages if t >= cutoff]

    def _snapshot(self) -> Telemetry:
        return Telemetry(
            armed=self._armed,
            mode=self._mode,
            autopilot=self.autopilot,
            vehicle_type=self.vehicle_type,
            system_status=self._system_status,
            heartbeat_age=self._heartbeat_age(),
            position=Position(**self._position),
            velocity=Velocity(**self._velocity),
            attitude=Attitude(**self._attitude),
            battery=Battery(**self._battery),
            gps=GPS(**self._gps),
        )

    def _handle(self, msg: Any) -> None:
        mtype = msg.get_type()
        if mtype == "BAD_DATA" or msg.get_srcSystem() != self.target_system:
            return
        if mtype == "HEARTBEAT":
            if msg.get_srcComponent() == self.target_component:
                self._handle_heartbeat(msg)
        elif mtype == "GLOBAL_POSITION_INT":
            self._position.update(
                lat=msg.lat / 1e7,
                lon=msg.lon / 1e7,
                alt_msl=msg.alt / 1000.0,
                alt_rel=msg.relative_alt / 1000.0,
                heading=None if msg.hdg == _UINT16_MAX else msg.hdg / 100.0,
            )
            self._velocity.update(north=msg.vx / 100.0, east=msg.vy / 100.0, down=msg.vz / 100.0)
        elif mtype == "VFR_HUD":
            self._velocity.update(
                groundspeed=msg.groundspeed, airspeed=msg.airspeed, climb_rate=msg.climb
            )
        elif mtype == "ATTITUDE":
            self._attitude.update(
                roll=math.degrees(msg.roll),
                pitch=math.degrees(msg.pitch),
                yaw=math.degrees(msg.yaw) % 360.0,
            )
        elif mtype == "SYS_STATUS":
            self._sensors_present = msg.onboard_control_sensors_present
            self._sensors_health = msg.onboard_control_sensors_health
            self._battery.update(
                voltage=(
                    None if msg.voltage_battery == _UINT16_MAX else msg.voltage_battery / 1000.0
                ),
                current=None if msg.current_battery == -1 else msg.current_battery / 100.0,
                remaining=None if msg.battery_remaining == -1 else msg.battery_remaining,
            )
        elif mtype == "GPS_RAW_INT":
            self._gps.update(
                fix_type=msg.fix_type,
                satellites=None if msg.satellites_visible == 255 else msg.satellites_visible,
                hdop=None if msg.eph == _UINT16_MAX else msg.eph / 100.0,
            )
        elif mtype == "STATUSTEXT":
            self._handle_statustext(msg)
        elif mtype == "COMMAND_ACK" and self._ack_is_for_us(msg):
            self._acks[msg.command] = msg

    def _ack_is_for_us(self, msg: Any) -> bool:
        # Only the autopilot component answers our commands; a gimbal or camera on
        # the same system may ACK the same command ID for its own reasons.
        if msg.get_srcComponent() != self.target_component:
            return False
        # MAVLink 2 ACKs name their recipient. 0 means "not set" (always so on MAVLink 1).
        ack_sys = getattr(msg, "target_system", 0) or 0
        ack_comp = getattr(msg, "target_component", 0) or 0
        if ack_sys and self._source_system and ack_sys != self._source_system:
            return False  # an answer to another ground station
        return not (ack_comp and self._source_component and ack_comp != self._source_component)

    def _handle_statustext(self, msg: Any) -> None:
        raw = msg.text if isinstance(msg.text, str) else str(msg.text)
        raw = raw.rstrip("\x00")
        text_id = getattr(msg, "id", 0) or 0
        if text_id:
            # MAVLink 2 splits long texts into 50-char chunks sharing one id.
            chunks = self._statustext_chunks.setdefault(text_id, [])
            chunks.append(raw)
            if len(raw) >= _STATUSTEXT_LEN and len(chunks) < 8:
                if len(self._statustext_chunks) > 16:  # drop abandoned partial texts
                    self._statustext_chunks.pop(next(iter(self._statustext_chunks)))
                return
            raw = "".join(self._statustext_chunks.pop(text_id))
        text = raw.strip()
        if not text:
            return
        self._status_messages.append((time.monotonic(), text))
        log.info("vehicle: %s", text)

    def _handle_heartbeat(self, msg: Any) -> None:
        self._last_heartbeat = time.monotonic()
        self._armed = bool(msg.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        self._mode = _modes.mode_from_heartbeat(msg)
        self._system_status = _modes.system_status_name(msg.system_status)


def _send_gcs_heartbeat(mav: Any) -> None:
    mav.heartbeat_send(
        mavlink.MAV_TYPE_GCS, mavlink.MAV_AUTOPILOT_INVALID, 0, 0, mavlink.MAV_STATE_ACTIVE
    )


def _is_vehicle_heartbeat(msg: Any) -> bool:
    return bool(msg.autopilot != mavlink.MAV_AUTOPILOT_INVALID and msg.type != mavlink.MAV_TYPE_GCS)


def _wait_for_vehicle(
    link: Any,
    address: str,
    timeout: float,
    target_system: int | None,
    send_heartbeat: bool,
) -> Any:
    deadline = time.monotonic() + timeout
    next_heartbeat = 0.0
    while True:
        now = time.monotonic()
        if now >= deadline:
            raise ConnectionTimeout(
                f"No vehicle heartbeat on {address!r} within {timeout:g}s. "
                "Check the connection string, baud rate, wiring, and that the "
                "autopilot (or SITL) is running."
            )
        if send_heartbeat and now >= next_heartbeat:
            # OSError: e.g. a UDP listener that has not heard from anyone yet.
            with contextlib.suppress(OSError):
                _send_gcs_heartbeat(link.mav)
            next_heartbeat = now + _HEARTBEAT_PERIOD
        msg = link.recv_match(type="HEARTBEAT", blocking=True, timeout=min(0.5, deadline - now))
        if msg is None or not _is_vehicle_heartbeat(msg):
            continue
        if target_system is None or msg.get_srcSystem() == target_system:
            return msg


def _dedupe(items: list[str]) -> list[str]:
    return list(dict.fromkeys(items))


def _enum_name(enum: str, value: int) -> str:
    entry = mavlink.enums.get(enum, {}).get(value)
    return entry.name if entry is not None else f"{enum}({value})"
