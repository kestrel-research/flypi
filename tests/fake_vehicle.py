"""A tiny MAVLink vehicle emulator for tests.

It speaks real MAVLink over UDP loopback, so flypi is tested end to end through
pymavlink and real sockets, without needing ArduPilot or PX4 SITL installed.
It emulates just enough of an ArduCopter- or PX4-style multicopter to exercise
connecting, telemetry, mode changes, arming, takeoff and landing.
"""

from __future__ import annotations

import socket
import threading
import time

from pymavlink import mavutil

mavlink = mavutil.mavlink

APM_MODES = {"STABILIZE": 0, "GUIDED": 4, "LOITER": 5, "RTL": 6, "LAND": 9}
PX4_AUTO = mavutil.PX4_CUSTOM_MAIN_MODE_AUTO
PX4_POSCTL = mavutil.PX4_CUSTOM_MAIN_MODE_POSCTL
PX4_SUB_TAKEOFF = mavutil.PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF
PX4_SUB_LOITER = mavutil.PX4_CUSTOM_SUB_MODE_AUTO_LOITER
PX4_SUB_LAND = mavutil.PX4_CUSTOM_SUB_MODE_AUTO_LAND
PX4_SUB_RTL = mavutil.PX4_CUSTOM_SUB_MODE_AUTO_RTL


def free_udp_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def px4_custom_mode(main: int, sub: int = 0) -> int:
    return (main << 16) | (sub << 24)


class FakeVehicle(threading.Thread):
    def __init__(
        self,
        port: int,
        *,
        autopilot: str = "ardupilot",
        prearm_ok: bool = True,
        gps_fix: int = 3,
        ack_commands: bool = True,
        extra_heartbeats: bool = False,
        rate_hz: float = 25.0,
        climb_rate: float = 10.0,
        descent_rate: float = 8.0,
    ) -> None:
        super().__init__(daemon=True)
        self.port = port
        self.px4 = autopilot == "px4"
        self.prearm_ok = prearm_ok
        self.gps_fix = gps_fix
        self.ack_commands = ack_commands
        self.extra_heartbeats = extra_heartbeats
        self.period = 1.0 / rate_hz
        self.climb_rate = climb_rate
        self.descent_rate = descent_rate

        self.armed = False
        self.custom_mode = px4_custom_mode(PX4_POSCTL) if self.px4 else APM_MODES["STABILIZE"]
        self.home_amsl = 100.0
        self.rel_alt = 0.0
        self.target_alt: float | None = None
        self.heartbeats_enabled = True
        self.gcs_heartbeats_received = 0
        self.commands_received: list[int] = []

        self._halt = threading.Event()
        self.conn = mavutil.mavlink_connection(
            f"udpout:127.0.0.1:{port}", source_system=1, source_component=1
        )
        self._gcs = mavlink.MAVLink(self.conn, srcSystem=255, srcComponent=190)
        self._gimbal = mavlink.MAVLink(self.conn, srcSystem=1, srcComponent=154)

    # ------------------------------------------------------------ lifecycle

    def stop(self) -> None:
        self._halt.set()
        self.join(timeout=2)
        self.conn.close()

    def run(self) -> None:
        last = time.monotonic()
        while not self._halt.is_set():
            now = time.monotonic()
            self._physics(now - last)
            last = now
            try:
                self._send_telemetry()
                self._read_incoming()
            except OSError:
                if self._halt.is_set():
                    break
            time.sleep(self.period)

    # -------------------------------------------------------------- helpers

    @property
    def mode_is_landing(self) -> bool:
        if self.px4:
            return self.custom_mode in (
                px4_custom_mode(PX4_AUTO, PX4_SUB_LAND),
                px4_custom_mode(PX4_AUTO, PX4_SUB_RTL),
            )
        return self.custom_mode in (APM_MODES["LAND"], APM_MODES["RTL"])

    def statustext(self, text: str) -> None:
        self.conn.mav.statustext_send(mavlink.MAV_SEVERITY_CRITICAL, text.encode())

    def _physics(self, dt: float) -> None:
        if not self.armed:
            return
        if self.mode_is_landing:
            self.target_alt = None
            self.rel_alt = max(0.0, self.rel_alt - self.descent_rate * dt)
            if self.rel_alt == 0.0:
                self.armed = False
                self.statustext("Disarming motors")
        elif self.target_alt is not None:
            self.rel_alt = min(self.target_alt, self.rel_alt + self.climb_rate * dt)
            if self.px4 and self.rel_alt >= self.target_alt:
                self.custom_mode = px4_custom_mode(PX4_AUTO, PX4_SUB_LOITER)

    def _send_telemetry(self) -> None:
        mav = self.conn.mav
        if self.extra_heartbeats:
            # Noise that flypi must ignore: another GCS and a gimbal.
            self._gcs.heartbeat_send(mavlink.MAV_TYPE_GCS, mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            self._gimbal.heartbeat_send(
                mavlink.MAV_TYPE_GIMBAL, mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0
            )
        if self.heartbeats_enabled:
            base = mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            if self.armed:
                base |= mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            mav.heartbeat_send(
                mavlink.MAV_TYPE_QUADROTOR,
                mavlink.MAV_AUTOPILOT_PX4 if self.px4 else mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
                base,
                self.custom_mode,
                mavlink.MAV_STATE_ACTIVE if self.armed else mavlink.MAV_STATE_STANDBY,
            )
        prearm = 0 if self.px4 else mavlink.MAV_SYS_STATUS_PREARM_CHECK
        health = prearm if self.prearm_ok else 0
        mav.sys_status_send(prearm, prearm, health, 500, 12600, 1050, 87, 0, 0, 0, 0, 0, 0)
        mav.gps_raw_int_send(0, self.gps_fix, -353632610, 1491652300, 584000, 80, 120, 0, 0, 12)
        mav.global_position_int_send(
            0,
            -353632610,
            1491652300,
            int((self.home_amsl + self.rel_alt) * 1000),
            int(self.rel_alt * 1000),
            0,
            0,
            0,
            9000,
        )
        mav.attitude_send(0, 0.01, -0.02, 1.5708, 0, 0, 0)
        mav.vfr_hud_send(0.0, 0.0, 90, 30, self.home_amsl + self.rel_alt, 0.0)

    def _read_incoming(self) -> None:
        while True:
            msg = self.conn.recv_match(blocking=False)
            if msg is None:
                return
            mtype = msg.get_type()
            if mtype == "HEARTBEAT" and msg.type == mavlink.MAV_TYPE_GCS:
                self.gcs_heartbeats_received += 1
            elif mtype == "COMMAND_LONG" and msg.target_system == 1:
                self.commands_received.append(msg.command)
                result = self._handle_command(msg)
                if self.ack_commands:
                    self.conn.mav.command_ack_send(msg.command, result)

    def _handle_command(self, msg) -> int:
        cmd = msg.command
        ok, failed = mavlink.MAV_RESULT_ACCEPTED, mavlink.MAV_RESULT_FAILED
        if cmd == mavlink.MAV_CMD_SET_MESSAGE_INTERVAL:
            return ok
        if cmd == mavlink.MAV_CMD_DO_SET_MODE:
            if self.px4:
                self.custom_mode = px4_custom_mode(int(msg.param2), int(msg.param3))
                return ok
            if int(msg.param2) in APM_MODES.values():
                self.custom_mode = int(msg.param2)
                return ok
            return mavlink.MAV_RESULT_DENIED
        if cmd == mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            forced = int(msg.param2) == 21196
            if msg.param1 == 1:
                if not forced and not self.prearm_ok:
                    self.statustext("PreArm: Battery below minimum arming voltage")
                    return failed
                if not forced and self.gps_fix < 3:
                    self.statustext("PreArm: Need 3D Fix")
                    return failed
                self.armed = True
                return ok
            if self.rel_alt > 0.1 and not forced:
                return failed
            self.armed = False
            return ok
        if cmd == mavlink.MAV_CMD_NAV_TAKEOFF:
            if not self.armed:
                return failed
            if self.px4:
                self.target_alt = msg.param7 - self.home_amsl
                self.custom_mode = px4_custom_mode(PX4_AUTO, PX4_SUB_TAKEOFF)
                return ok
            if self.custom_mode != APM_MODES["GUIDED"]:
                return failed
            self.target_alt = msg.param7
            return ok
        return mavlink.MAV_RESULT_UNSUPPORTED
