"""Fly a small square with goto_relative(), then return home and land.

Run it against SITL first:

    sim_vehicle.py -v ArduCopter          # or: make px4_sitl gz_x500
    python examples/goto.py

To use a different connection, pass it as the first argument:

    python examples/goto.py tcp:127.0.0.1:5760
"""

import sys

import flypi

ADDRESS = sys.argv[1] if len(sys.argv) > 1 else flypi.DEFAULT_ADDRESS
ALTITUDE_M = 10
SIDE_M = 20

print(f"Connecting to {ADDRESS} ...")
with flypi.connect(ADDRESS) as drone:
    print(f"Connected: {drone}")
    drone.wait_ready(timeout=120)
    home = drone.telemetry.position

    drone.arm()
    print(f"Taking off to {ALTITUDE_M} m ...")
    drone.takeoff(ALTITUDE_M)

    try:
        for north, east in ((SIDE_M, 0), (0, SIDE_M), (-SIDE_M, 0), (0, -SIDE_M)):
            print(f"Flying {north:+} m north, {east:+} m east ...")
            drone.goto_relative(north=north, east=east, groundspeed=5)
            p = drone.telemetry.position
            print(f"  at {p.lat:.7f}, {p.lon:.7f}, {p.alt_rel:.1f} m")

        print("Back over the take-off point ...")
        drone.goto(home.lat, home.lon)
    except flypi.FlightInterrupted as e:
        print(f"Something else took over (now in {e.mode}); leaving it in charge: {e}")
        raise SystemExit(1) from e

    print("Landing ...")
    drone.land()
    print("Landed and disarmed.")
