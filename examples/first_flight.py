"""First flight: connect, take off to 5 m, hover, and land.

Run it against SITL first:

    sim_vehicle.py -v ArduCopter          # or: make px4_sitl gz_x500
    python examples/first_flight.py

To use a different connection, pass it as the first argument:

    python examples/first_flight.py tcp:127.0.0.1:5760
    python examples/first_flight.py /dev/serial0
"""

import sys
import time

import flypi

ADDRESS = sys.argv[1] if len(sys.argv) > 1 else flypi.DEFAULT_ADDRESS
ALTITUDE_M = 5

print(f"Connecting to {ADDRESS} ...")
with flypi.connect(ADDRESS) as drone:
    print(f"Connected: {drone}")

    print("Waiting for pre-arm checks and GPS ...")
    drone.wait_ready(timeout=120)

    print("Arming ...")
    drone.arm()

    print(f"Taking off to {ALTITUDE_M} m ...")
    drone.takeoff(ALTITUDE_M)

    for _ in range(5):
        t = drone.telemetry
        print(
            f"  alt {t.position.alt_rel:5.1f} m | "
            f"battery {t.battery.voltage or 0:4.1f} V | mode {t.mode}"
        )
        time.sleep(1)

    print("Landing ...")
    drone.land()
    print("Landed and disarmed.")
