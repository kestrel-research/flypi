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
from ._geo import EARTH_RADIUS as _EARTH_RADIUS
from ._geo import distance_m as _distance_m
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
_PX4_GOTO_MODES = frozenset({"LOITER"})  # PX4 flies DO_REPOSITION in Hold (AUTO.LOITER)
_GUIDED_MODES = frozenset({"GUIDED"})
_TEMPORARILY_REJECTED = object()  # sentinel returned by Drone._await_ack
_GOTO_ATTEMPTS = 3  # times a position target is sent before giving up on its echo
_ECHO_WAIT = 1.0  # seconds to wait for POSITION_TARGET_GLOBAL_INT after each attempt
_ECHO_LATLON_TOLERANCE = 20  # 1e-7 degree units (about 2 cm per unit)
# SET_POSITION_TARGET_GLOBAL_INT: use the position only; let the autopilot pick yaw.
_POSITION_ONLY_MASK = (
    mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
    | mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
    | mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
    | mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
    | mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
    | mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
    | mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
    | mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
)
_RELATIVE_FRAMES = frozenset(
    {mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT}
)
_AMSL_FRAMES = frozenset({mavlink.MAV_FRAME_GLOBAL, mavlink.MAV_FRAME_GLOBAL_INT})

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
        self._position_target: tuple[float, Any] | None = None  # (received_at, message)
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

    def goto(
        self,
        lat: float,
        lon: float,
        altitude: float | None = None,
        *,
        groundspeed: float | None = None,
        wait: bool = True,
        tolerance: float = 1.0,
        timeout: float = 120.0,
    ) -> None:
        """Fly to a global position and (by default) wait until the vehicle is there.

        The vehicle must be armed and flying (see :meth:`takeoff`). flypi checks
        that the autopilot actually took the new target, not just that the
        message was sent:

        * **ArduPilot**: switches to ``GUIDED`` if needed, sends
          ``SET_POSITION_TARGET_GLOBAL_INT`` and then reads the target back
          (``POSITION_TARGET_GLOBAL_INT``). MAVLink has no ACK for position
          targets, so the read-back is the confirmation. A target that is lost
          is resent; one the vehicle ignores raises :class:`~flypi.CommandRejected`.
        * **PX4** (experimental): sends ``MAV_CMD_DO_REPOSITION``, which PX4
          acknowledges, and flies it in Hold (``LOITER``) mode.

        Args:
            lat: Target latitude in degrees.
            lon: Target longitude in degrees.
            altitude: Target height above home in metres (the same reference as
                :meth:`takeoff`). ``None`` keeps the current altitude.
            groundspeed: Horizontal speed in m/s. On ArduPilot this is sent as
                ``MAV_CMD_DO_CHANGE_SPEED`` and stays in effect for later moves.
                ``None`` keeps the autopilot's current speed.
            wait: Block until the vehicle is within ``tolerance`` of the target.
            tolerance: Arrival radius in metres, applied horizontally and
                vertically.
            timeout: Seconds to wait for arrival when ``wait`` is ``True``.

        Raises:
            NotArmedError: if the vehicle is not armed.
            ValueError: if a coordinate, altitude, speed or tolerance is invalid.
            CommandRejected: if the vehicle refuses or ignores the target.
            FlightInterrupted: (only with ``wait=True``) if the vehicle disarms or
                leaves ``GUIDED`` (PX4: Hold) before arriving, for example
                because of a failsafe or the RC pilot.
        """
        _check_lat_lon(lat, lon)
        if altitude is not None and not math.isfinite(altitude):
            raise ValueError(f"altitude must be a finite number of metres, got {altitude!r}")
        if groundspeed is not None and not (math.isfinite(groundspeed) and groundspeed > 0):
            raise ValueError(f"groundspeed must be a positive number of m/s, got {groundspeed!r}")
        if not (math.isfinite(tolerance) and tolerance > 0):
            raise ValueError(f"tolerance must be a positive number of metres, got {tolerance!r}")
        if not self.armed:
            raise NotArmedError(
                "The vehicle must be armed and flying before goto(). "
                "Call drone.arm() and drone.takeoff() first."
            )
        with self._cond:
            alt_msl = self._position.get("alt_msl")
            alt_rel = self._position.get("alt_rel")
        if alt_msl is None or alt_rel is None:
            raise FlypiError("No position yet; call drone.wait_ready() before goto().")
        target_alt = alt_rel if altitude is None else altitude

        hold_modes: frozenset[str]
        if self._autopilot_id == mavlink.MAV_AUTOPILOT_PX4:
            hold_modes = _PX4_GOTO_MODES
            self._goto_px4(lat, lon, (alt_msl - alt_rel) + target_alt, groundspeed)
            # The ACK can arrive before the heartbeat that shows the new mode.
            self._wait(lambda: self._mode in hold_modes, 5.0, "PX4 to enter Hold (LOITER)")
        else:
            hold_modes = _GUIDED_MODES
            self.set_mode("GUIDED")
            if groundspeed is not None:
                self._command_long(mavlink.MAV_CMD_DO_CHANGE_SPEED, (1, groundspeed, -1, 0))
            self._goto_ardupilot(lat, lon, target_alt)

        if wait:

            def arrived() -> bool:
                self._check_flight("goto", hold_modes)
                cur_lat = self._position.get("lat")
                cur_lon = self._position.get("lon")
                cur_alt = self._position.get("alt_rel")
                if cur_lat is None or cur_lon is None or cur_alt is None:
                    return False
                return (
                    _distance_m(cur_lat, cur_lon, lat, lon) <= tolerance
                    and abs(cur_alt - target_alt) <= tolerance
                )

            self._wait(
                arrived,
                timeout,
                f"arrival at {lat:.7f}, {lon:.7f}, {target_alt:g} m (within {tolerance:g} m)",
            )

    def goto_relative(
        self,
        north: float = 0.0,
        east: float = 0.0,
        up: float = 0.0,
        *,
        groundspeed: float | None = None,
        wait: bool = True,
        tolerance: float = 1.0,
        timeout: float = 120.0,
    ) -> None:
        """Fly a distance in metres from the current position.

        Example::

            drone.goto_relative(north=10, east=-5)   # 10 m north, 5 m west
            drone.goto_relative(up=3)                # climb 3 m

        Offsets are measured from where the vehicle is when you call this,
        along true north and east. Everything else works as in :meth:`goto`.
        """
        for name, value in (("north", north), ("east", east), ("up", up)):
            if not math.isfinite(value):
                raise ValueError(f"{name} must be a finite number of metres, got {value!r}")
        with self._cond:
            lat = self._position.get("lat")
            lon = self._position.get("lon")
            alt_rel = self._position.get("alt_rel")
        if lat is None or lon is None or alt_rel is None:
            raise FlypiError("No position yet; call drone.wait_ready() before goto_relative().")
        target_lat, target_lon = _offset_position(lat, lon, north, east)
        self.goto(
            target_lat,
            target_lon,
            alt_rel + up,
            groundspeed=groundspeed,
            wait=wait,
            tolerance=tolerance,
            timeout=timeout,
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

    def _goto_ardupilot(self, lat: float, lon: float, alt_rel: float) -> None:
        """Send a GUIDED position target and confirm it by reading it back."""
        lat_i, lon_i = round(lat * 1e7), round(lon * 1e7)
        ts, tc = self.target_system, self.target_component

        def send(mav: Any) -> None:
            mav.set_position_target_global_int_send(
                0,
                ts,
                tc,
                mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                _POSITION_ONLY_MASK,
                lat_i,
                lon_i,
                alt_rel,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
            )

        name = "SET_POSITION_TARGET_GLOBAL_INT"
        sent_at = time.monotonic()
        echo: Any = None
        for attempt in range(_GOTO_ATTEMPTS):
            attempt_at = time.monotonic()
            self._send(send)
            # Ask for the target straight away instead of waiting for a stream.
            # ArduPilot answers FAILED while it has no position target; that is
            # handled below like a missing echo.
            with contextlib.suppress(CommandRejected):
                self._command_long(
                    mavlink.MAV_CMD_REQUEST_MESSAGE,
                    (mavlink.MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT,),
                    timeout=1.0,
                    retries=1,
                    explain=False,
                )
            echo = self._wait_for_position_target(attempt_at, _ECHO_WAIT)
            if echo is not None and self._target_matches(echo, lat_i, lon_i, alt_rel):
                return
            log.debug("position target not confirmed (attempt %d); resending", attempt + 1)

        with self._cond:
            messages = [text for t, text in self._status_messages if t >= sent_at]
        if echo is None:
            detail = (
                "The vehicle never reported a position target. ArduPilot only holds one "
                "in GUIDED mode while flying: check the vehicle has taken off"
            )
        else:
            detail = (
                "The vehicle kept reporting a different target, so it ignored this one "
                f"(it reports {echo.lat_int / 1e7:.7f}, {echo.lon_int / 1e7:.7f}, "
                f"alt {echo.alt:g} m in frame {_enum_name('MAV_FRAME', echo.coordinate_frame)}). "
                "Another program may be sending targets, or the target is outside a fence"
            )
        raise CommandRejected(name, "NOT_APPLIED", messages, detail=detail)

    def _goto_px4(self, lat: float, lon: float, alt_msl: float, groundspeed: float | None) -> None:
        nan = math.nan
        self._command_int(
            mavlink.MAV_CMD_DO_REPOSITION,
            mavlink.MAV_FRAME_GLOBAL,
            (
                -1 if groundspeed is None else groundspeed,
                mavlink.MAV_DO_REPOSITION_FLAGS_CHANGE_MODE,
                0,
                nan,  # yaw: let PX4 decide
            ),
            round(lat * 1e7),
            round(lon * 1e7),
            alt_msl,
        )

    def _wait_for_position_target(self, since: float, timeout: float) -> Any:
        """Return the first POSITION_TARGET_GLOBAL_INT received after ``since``, or None."""
        deadline = time.monotonic() + timeout
        with self._cond:
            while True:
                self._check_alive()
                target = self._position_target
                if target is not None and target[0] >= since:
                    return target[1]
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._cond.wait(min(remaining, 0.1))

    def _target_matches(self, echo: Any, lat_i: int, lon_i: int, alt_rel: float) -> bool:
        if abs(echo.lat_int - lat_i) > _ECHO_LATLON_TOLERANCE:
            return False
        if abs(echo.lon_int - lon_i) > _ECHO_LATLON_TOLERANCE:
            return False
        frame = echo.coordinate_frame
        if frame in _RELATIVE_FRAMES:
            return bool(abs(echo.alt - alt_rel) <= 0.5)
        if frame in _AMSL_FRAMES:
            with self._cond:
                alt_msl = self._position.get("alt_msl")
                rel = self._position.get("alt_rel")
            if alt_msl is None or rel is None:
                return True
            return bool(abs(echo.alt - ((alt_msl - rel) + alt_rel)) <= 1.0)
        return True  # terrain frames: only the horizontal position can be compared

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
        padded = tuple(params) + (0,) * (7 - len(params))
        ts, tc = self.target_system, self.target_component

        def make_send(confirmation: int) -> Callable[[Any], None]:
            def send(mav: Any) -> None:
                mav.command_long_send(ts, tc, command, confirmation, *padded)

            return send

        return self._command(command, make_send, timeout, retries, explain)

    def _command_int(
        self,
        command: int,
        frame: int,
        params: tuple[float, float, float, float],
        x: int,
        y: int,
        z: float,
        *,
        timeout: float = 3.0,
        retries: int = 2,
        explain: bool = True,
    ) -> Any:
        """Send COMMAND_INT (for commands carrying a lat/lon) and wait for its ACK.

        Retries work as in :meth:`_command_long`; COMMAND_INT has no
        ``confirmation`` counter, so resends are identical.
        """
        ts, tc = self.target_system, self.target_component

        def make_send(confirmation: int) -> Callable[[Any], None]:
            def send(mav: Any) -> None:
                mav.command_int_send(ts, tc, frame, command, 0, 0, *params, x, y, z)

            return send

        return self._command(command, make_send, timeout, retries, explain)

    def _command(
        self,
        command: int,
        make_send: Callable[[int], Callable[[Any], None]],
        timeout: float,
        retries: int,
        explain: bool,
    ) -> Any:
        name = _enum_name("MAV_CMD", command)
        with self._cond:
            lock = self._command_locks.setdefault(command, threading.Lock())
        # Two threads sending the same command would each see the other's ACK.
        with lock:
            sent_at = time.monotonic()
            for attempt in range(retries + 1):
                with self._cond:
                    self._acks.pop(command, None)
                self._send(make_send(min(attempt, 255)))
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
            # ArduPilot's takeoff also allows TAKEOFF (Plane), but GUIDED is the mode
            # the user asked for, so name that one.
            expected = "GUIDED" if "GUIDED" in hold_modes else "/".join(sorted(hold_modes))
            raise FlightInterrupted(
                self._explain(
                    f"{phase.capitalize()} interrupted: the vehicle switched from {expected} "
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
        elif mtype == "POSITION_TARGET_GLOBAL_INT":
            self._position_target = (time.monotonic(), msg)
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


def _check_lat_lon(lat: float, lon: float) -> None:
    if not (math.isfinite(lat) and -90.0 <= lat <= 90.0):
        raise ValueError(f"lat must be between -90 and 90 degrees, got {lat!r}")
    if not (math.isfinite(lon) and -180.0 <= lon <= 180.0):
        raise ValueError(f"lon must be between -180 and 180 degrees, got {lon!r}")


def _offset_position(lat: float, lon: float, north: float, east: float) -> tuple[float, float]:
    """The point ``north``/``east`` metres from ``lat``/``lon`` (flat-earth, fine below ~10 km)."""
    new_lat = lat + math.degrees(north / _EARTH_RADIUS)
    new_lon = lon + math.degrees(east / (_EARTH_RADIUS * math.cos(math.radians(lat))))
    new_lon = (new_lon + 180.0) % 360.0 - 180.0
    return new_lat, new_lon


def _dedupe(items: list[str]) -> list[str]:
    return list(dict.fromkeys(items))


def _enum_name(enum: str, value: int) -> str:
    entry = mavlink.enums.get(enum, {}).get(value)
    return entry.name if entry is not None else f"{enum}({value})"
