# Changelog

All notable changes to flypi are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and flypi uses
[Semantic Versioning](https://semver.org/).

## [0.1.0] - Unreleased

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
