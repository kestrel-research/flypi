# flypi

**Fly ArduPilot and PX4 drones from a Raspberry Pi with a few lines of Python.**

[![CI](https://github.com/kestrel-research/flypi/actions/workflows/ci.yml/badge.svg)](https://github.com/kestrel-research/flypi/actions/workflows/ci.yml)
[![PyPI](https://img.shields.io/pypi/v/flypi.svg)](https://pypi.org/project/flypi/)
[![Python](https://img.shields.io/pypi/pyversions/flypi.svg)](https://pypi.org/project/flypi/)
[![License](https://img.shields.io/badge/license-Apache--2.0-blue.svg)](LICENSE)

flypi is a small, modern, high-level MAVLink library for companion computers.
It is built on [pymavlink](https://github.com/ArduPilot/pymavlink), has no other
dependencies, and installs cleanly on a Raspberry Pi.

```python
import flypi

with flypi.connect("udp:127.0.0.1:14550") as drone:
    drone.wait_ready()  # pre-arm checks pass + 3D GPS fix
    drone.arm()
    drone.takeoff(5)  # metres above home; waits until it gets there
    print(drone.telemetry.position)
    drone.land()  # waits until landed and disarmed
```

> **Status: alpha (v0.1).** The API may still change before 1.0.
> ArduPilot Copter is the primary target. PX4 support is experimental.

## Install

```bash
pip install flypi
```

Python 3.10 or newer. Works on Raspberry Pi OS (Bookworm and later), Linux, macOS and Windows.

## Connecting

`flypi.connect()` takes a [pymavlink connection string](https://mavlink.io/en/mavgen_python/#connection_string):

| Setup | Address |
|---|---|
| ArduPilot SITL via MAVProxy, or PX4 SITL | `"udp:127.0.0.1:14550"` (default) |
| ArduPilot SITL directly | `"tcp:127.0.0.1:5760"` |
| Pi UART (GPIO 14/15) wired to the flight controller | `"/dev/serial0"` with `baud=921600` (match your `SERIALx_BAUD`) |
| Flight controller over USB | `"/dev/ttyACM0"` |

```python
drone = flypi.connect("/dev/serial0", baud=921600, timeout=30)
```

flypi waits for the autopilot's heartbeat. It ignores heartbeats from ground
stations, gimbals and cameras. If nothing is heard within `timeout`, you get a
`flypi.ConnectionTimeout` that tells you what to check.

## What you can do in v0.1

| Call | What it does |
|---|---|
| `drone.telemetry` | Immutable snapshot: `armed`, `mode`, `position`, `velocity`, `attitude`, `battery`, `gps`, `heartbeat_age` |
| `drone.wait_ready(require_gps=True)` | Wait until the vehicle is ready to arm |
| `drone.set_mode("LOITER")` | Change flight mode and confirm it |
| `drone.arm()` / `drone.disarm()` | Arm or disarm and confirm it |
| `drone.takeoff(altitude)` | Vertical takeoff to `altitude` metres above home |
| `drone.land()` | Land at the current position, wait for disarm |
| `drone.return_to_launch()` | Switch to RTL |
| `drone.wait_until(cond, timeout)` | Wait for any telemetry condition |
| `drone.status_messages` | Recent messages from the autopilot (e.g. `PreArm:` reasons) |
| `drone.available_modes` | Mode names this vehicle accepts |

Every command checks the vehicle's acknowledgement. When the autopilot refuses
something, flypi raises an error that includes the autopilot's own explanation:

```text
flypi.errors.CommandRejected: MAV_CMD_COMPONENT_ARM_DISARM was rejected by the
vehicle (MAV_RESULT_FAILED). Recent vehicle messages: PreArm: Need 3D Fix
```

All errors inherit from `flypi.FlypiError`.

### Telemetry example

```python
t = drone.telemetry
print(t.mode, t.armed)
print(f"{t.position.lat:.6f}, {t.position.lon:.6f} @ {t.position.alt_rel:.1f} m")
print(f"Battery {t.battery.voltage:.1f} V ({t.battery.remaining}%)")
print(f"GPS fix {t.gps.fix_type}, {t.gps.satellites} satellites")
```

Values the vehicle has not reported yet are `None`.

## Try it in simulation first

Always test in software-in-the-loop (SITL) simulation before flying real hardware.

**ArduPilot SITL** (see the [ArduPilot SITL docs](https://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html)):

```bash
sim_vehicle.py -v ArduCopter --console --map
# in another terminal
python examples/first_flight.py
```

**PX4 SITL** (see the [PX4 simulation docs](https://docs.px4.io/main/en/simulation/)):

```bash
make px4_sitl gz_x500
python examples/first_flight.py
```

## How it works

Each connection runs one background thread. The thread reads every MAVLink
message from the vehicle to keep telemetry current, and sends a 1 Hz
ground-station heartbeat so the autopilot knows a controller is attached. Your
code calls ordinary blocking methods. No `asyncio` is needed.

Because flypi heartbeats as a ground station, an autopilot with a GCS failsafe
enabled (`FS_GCS_ENABLE` on ArduPilot) will treat your script stopping
mid-flight as a lost ground station and run its failsafe action. That is
usually what you want on a companion computer.

## Safety

Drones can seriously injure people and damage property. Test in SITL first, then
with propellers removed. Fly only where it is safe and legal, follow your local
aviation rules, and always keep a manual RC override ready. flypi is provided
"as is", without warranty. See the [LICENSE](LICENSE).

## Development

```bash
git clone https://github.com/kestrel-research/flypi
cd flypi
pip install -e ".[dev]"
pytest
ruff check .
```

The test suite runs flypi end to end over real UDP sockets against a small
MAVLink vehicle emulator (`tests/fake_vehicle.py`), so it needs no simulator.

## Roadmap

- `goto(lat, lon, alt)` and velocity control in GUIDED/OFFBOARD
- Missions: upload, start, monitor
- Parameters: read and write
- Camera capture helpers for the Raspberry Pi camera
- A companion research library (simulation, RL and navigation) built on flypi

## License

Apache-2.0 © Kestrel Research contributors. Part of
[Kestrel Research](https://github.com/kestrel-research).
