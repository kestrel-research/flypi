"""Tests for the 0.1.1 reliability guarantees.

Each test pins down one way a real autopilot or link misbehaves, and checks
flypi turns it into either success or a clear, specific error - never a hang
or a silent failure.
"""

from __future__ import annotations

import threading
import time
from types import SimpleNamespace

import pytest

import flypi
from flypi import drone as drone_module

ARM = drone_module.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
SET_MODE = drone_module.mavlink.MAV_CMD_DO_SET_MODE
INTERVAL = drone_module.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL


# --------------------------------------------------------------- confirmation


def test_lost_ack_is_retried_with_incrementing_confirmation(fly):
    drone, vehicle = fly(drop_acks={ARM: 1})
    drone.arm(timeout=1)
    assert drone.armed
    arm_confirmations = [c for cmd, c in vehicle.confirmations if cmd == ARM]
    assert arm_confirmations[:2] == [0, 1]


def test_temporarily_rejected_is_retried(fly):
    drone, vehicle = fly(temp_rejects={ARM: 1})
    drone.arm()
    assert drone.armed
    assert vehicle.commands_received.count(ARM) == 2


def test_temporarily_rejected_every_time_raises_rejected(fly):
    drone, _ = fly(temp_rejects={ARM: 99})
    with pytest.raises(flypi.CommandRejected) as excinfo:
        drone.arm()
    assert excinfo.value.result == "MAV_RESULT_TEMPORARILY_REJECTED"


def test_in_progress_waits_for_final_ack_without_resending(fly):
    # The command takes longer than the per-attempt ACK timeout (1 s).
    drone, vehicle = fly(in_progress={ARM: 1.5})
    drone.arm(timeout=1)
    assert drone.armed
    assert vehicle.commands_received.count(ARM) == 1


def test_command_stuck_in_progress_times_out_instead_of_hanging(fly, monkeypatch):
    monkeypatch.setattr(drone_module, "_PROGRESS_TIMEOUT", 1.0)
    drone, vehicle = fly(in_progress={ARM: float("inf")})
    start = time.monotonic()
    with pytest.raises(flypi.CommandTimeout, match="IN_PROGRESS"):
        drone.arm(timeout=0.5)
    assert time.monotonic() - start < 3
    assert vehicle.commands_received.count(ARM) == 1  # never resent


def test_timeout_message_says_what_to_check(fly):
    drone, _ = fly(ack_commands=False, no_interval_ack=True)
    with pytest.raises(flypi.CommandTimeout, match="two-way"):
        drone.arm(timeout=0.2)


def test_gimbal_ack_is_not_mistaken_for_the_autopilots(fly):
    drone, _ = fly(gimbal_acks=True, ack_delay=0.2)
    drone.arm()  # the gimbal says FAILED at once; the autopilot says ACCEPTED 0.2 s later
    assert drone.armed


def test_ack_addressed_to_another_gcs_is_ignored(fly):
    drone, _ = fly()
    mine = SimpleNamespace(
        get_srcComponent=lambda: 1, target_system=drone._source_system, target_component=0
    )
    other = SimpleNamespace(get_srcComponent=lambda: 1, target_system=42, target_component=0)
    mavlink1 = SimpleNamespace(get_srcComponent=lambda: 1)
    assert drone._ack_is_for_us(mine)
    assert drone._ack_is_for_us(mavlink1)
    assert not drone._ack_is_for_us(other)


def test_concurrent_commands_from_two_threads(fly):
    drone, _ = fly()
    errors: list[BaseException] = []

    def worker(mode: str) -> None:
        try:
            for _ in range(3):
                drone.set_mode(mode)
                drone.set_mode("STABILIZE")
        except BaseException as exc:  # pragma: no cover - reported below
            errors.append(exc)

    def reader() -> None:
        for _ in range(500):
            t = drone.telemetry
            assert t.position.lat is None or -90 <= t.position.lat <= 90

    threads = [
        threading.Thread(target=worker, args=("LOITER",)),
        threading.Thread(target=worker, args=("GUIDED",)),
        threading.Thread(target=reader),
    ]
    for t in threads:
        t.start()
    for t in threads:
        t.join(timeout=30)
    assert not errors


def test_stream_requests_are_confirmed(fly):
    drone, vehicle = fly()
    assert vehicle.commands_received.count(INTERVAL) == len(drone_module._STREAM_RATES)
    assert drone.unconfirmed_streams == []


def test_refused_stream_requests_are_recorded_not_fatal(fly):
    drone, _ = fly(refuse_interval=True)
    assert "GLOBAL_POSITION_INT" in drone.unconfirmed_streams
    drone.wait_ready(timeout=5)  # still flyable on the default streams


def test_unanswered_stream_requests_do_not_stall_connect(make_vehicle):
    _, address = make_vehicle(no_interval_ack=True)
    start = time.monotonic()
    with flypi.connect(address, timeout=5) as drone:
        assert len(drone.unconfirmed_streams) == len(drone_module._STREAM_RATES)
    assert time.monotonic() - start < 4  # gives up after the first stream, not all five


# ---------------------------------------------------------------- clear errors


def test_prearm_reason_arriving_after_the_ack_is_included(fly):
    drone, _ = fly(prearm_ok=False, late_prearm=True)
    with pytest.raises(flypi.CommandRejected) as excinfo:
        drone.arm()
    assert "PreArm: Battery below minimum" in str(excinfo.value)
    assert excinfo.value.status_messages[0].startswith("PreArm:")


def test_older_prearm_reasons_are_attached_to_a_refused_arm(fly, monkeypatch):
    # ArduPilot repeats failing pre-arm checks about every 30 s while disarmed, so
    # the reason may have arrived well before the arm attempt.
    monkeypatch.setattr(drone_module, "_STATUS_WINDOW", 0.1)
    monkeypatch.setattr(drone_module, "_REJECT_GRACE", 0.1)
    drone, vehicle = fly()
    vehicle.statustext("PreArm: Compass not calibrated")
    drone.wait_until(lambda t: drone.status_messages, timeout=3)
    time.sleep(0.3)
    vehicle.prearm_ok = False
    vehicle.late_prearm = True  # its fresh reason arrives too late for the grace window
    with pytest.raises(flypi.CommandRejected, match="Compass not calibrated"):
        drone.arm()


def test_chunked_statustext_is_reassembled(fly):
    drone, _ = fly()
    first = "PreArm: " + "x" * 42  # exactly one full 50-char chunk
    assert len(first) == 50
    with drone._cond:
        drone._handle_statustext(SimpleNamespace(text=first, id=7))
        assert drone.status_messages == []  # waiting for the rest
        drone._handle_statustext(SimpleNamespace(text=" check failed", id=7))
    assert drone.status_messages == [first + " check failed"]


# ---------------------------------------------------- lost links, no hanging


def test_link_thread_survives_a_malformed_message(fly, monkeypatch):
    drone, _ = fly()
    original = drone._handle
    calls = {"n": 0}

    def flaky(msg):
        calls["n"] += 1
        if calls["n"] == 3:
            raise RuntimeError("malformed message")
        original(msg)

    monkeypatch.setattr(drone, "_handle", flaky)
    time.sleep(0.5)
    assert drone._thread.is_alive()
    assert drone.is_connected
    drone.set_mode("LOITER")


def test_dead_link_thread_raises_connection_lost(fly):
    drone, _ = fly()
    with drone._cond:
        drone._thread_error = RuntimeError("boom")
    with pytest.raises(flypi.ConnectionLost, match="link thread stopped"):
        drone.wait_until(lambda t: False, timeout=5)


def test_connection_lost_during_command(fly):
    drone, vehicle = fly(connect_kwargs={"heartbeat_timeout": 0.5}, ack_commands=False)
    vehicle.heartbeats_enabled = False
    start = time.monotonic()
    with pytest.raises(flypi.ConnectionLost, match="No heartbeat"):
        drone.arm(timeout=5)
    assert time.monotonic() - start < 2


def test_connection_lost_during_takeoff(fly):
    drone, vehicle = fly(connect_kwargs={"heartbeat_timeout": 0.5}, climb_rate=0.5)
    drone.wait_ready(timeout=5)
    drone.arm()
    drone.takeoff(20, wait=False)
    vehicle.heartbeats_enabled = False
    with pytest.raises(flypi.ConnectionLost):
        drone.wait_until(lambda t: False, timeout=10)


# ------------------------------------------------------- interrupted flights


def test_failsafe_mode_change_interrupts_takeoff(fly):
    drone, vehicle = fly(climb_rate=1.0)
    drone.wait_ready(timeout=5)
    drone.arm()

    def failsafe() -> None:
        time.sleep(0.5)
        vehicle.custom_mode = 6  # RTL, as a battery failsafe would do
        vehicle.statustext("Battery 1 is low 10.2V used 0 mAh")

    threading.Thread(target=failsafe, daemon=True).start()
    with pytest.raises(flypi.FlightInterrupted, match="GUIDED to RTL") as excinfo:
        drone.takeoff(30, timeout=20)
    assert excinfo.value.mode == "RTL"
    assert "Battery 1 is low" in str(excinfo.value)


def test_disarm_during_takeoff_is_flight_interrupted(fly):
    drone, vehicle = fly(climb_rate=1.0)
    drone.wait_ready(timeout=5)
    drone.arm()
    threading.Timer(0.5, lambda: setattr(vehicle, "armed", False)).start()
    with pytest.raises(flypi.FlightInterrupted, match="disarmed during takeoff"):
        drone.takeoff(30, timeout=20)


def test_pilot_taking_over_during_landing_raises(fly):
    drone, vehicle = fly(descent_rate=1.0)
    drone.wait_ready(timeout=5)
    drone.arm()
    drone.takeoff(5, timeout=10)
    threading.Timer(0.5, lambda: setattr(vehicle, "custom_mode", 5)).start()  # LOITER
    with pytest.raises(flypi.FlightInterrupted, match="switched to LOITER"):
        drone.land(timeout=20)


def test_flight_interrupted_is_a_flypi_error():
    assert issubclass(flypi.FlightInterrupted, flypi.FlypiError)
