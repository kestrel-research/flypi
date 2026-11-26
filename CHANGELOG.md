# Changelog

All notable changes to flypi are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and flypi uses
[Semantic Versioning](https://semver.org/).

## [0.1.1] - 2026-11-26

Reliability release. No breaking changes: everything that worked in 0.1.0 works the
same way, and a few failure cases now raise clearer errors sooner.

### Added
- `flypi.FlightInterrupted` (a `FlypiError`): raised when the vehicle disarms during
  `takeoff()`, leaves `GUIDED` during an ArduPilot takeoff, or leaves `LAND`/`RTL`
  before landing. Before, these waited out the full timeout. It has a `.mode` attribute.
- `Drone.unconfirmed_streams`: telemetry streams the vehicle refused or did not confirm.
- Type checking with `mypy --strict` in CI.

### Fixed
- Commands answered `IN_PROGRESS` are no longer resent once the wait times out,
  because resending could restart them. They now wait for the final ACK, capped at 60 s.
- `MAV_RESULT_TEMPORARILY_REJECTED` is retried with a short back-off instead of
  failing at once.
- A `COMMAND_ACK` from another component on the same system (e.g. a gimbal), or one
  addressed to another ground station, is no longer taken as the autopilot's answer.
- Two threads sending the same command can no longer see each other's ACK.
- `CommandRejected` now includes `PreArm:`/`Arm:` reasons that arrive just *after* the
  rejection (ArduPilot queues them), plus recent pre-arm reasons from the last 60 s.
- Long STATUSTEXT messages split into MAVLink 2 chunks are reassembled.
- A message flypi cannot parse no longer kills the link thread. If the thread does
  stop, waits raise `ConnectionLost` saying so.
- Socket errors while sending are raised as `ConnectionLost`, not a bare `OSError`.
- Telemetry stream requests (`MAV_CMD_SET_MESSAGE_INTERVAL`) are now confirmed.
  Refusals are logged rather than ignored, and a vehicle that never answers them
  delays `connect()` by about 2 s at most.

### Changed
- `CommandTimeout`, `ConnectionLost` and `WaitTimeout` messages now say what to check
  and include the vehicle's recent messages.

## [0.1.0] - 2026-11-24

### Added
- `flypi.connect()` for UDP, TCP and serial MAVLink links. It waits for the
  autopilot heartbeat and ignores GCS, gimbal and camera heartbeats.
- `Drone` with a background link thread: live telemetry and a 1 Hz GCS heartbeat.
- Immutable `Telemetry` snapshots: position, velocity, attitude, battery, GPS,
  mode, armed state and heartbeat age.
- `wait_ready()`, `set_mode()`, `arm()`, `disarm()`, `takeoff()`, `land()`,
  `return_to_launch()` and `wait_until()`.
- Acknowledged commands with retries, and clear errors (`CommandRejected` includes
  the autopilot's own `PreArm:` messages).
- ArduPilot support, plus experimental PX4 support.

[0.1.1]: https://github.com/kestrel-research/flypi/compare/v0.1.0...v0.1.1
[0.1.0]: https://github.com/kestrel-research/flypi/releases/tag/v0.1.0
