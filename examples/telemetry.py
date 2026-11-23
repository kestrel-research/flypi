"""Print live telemetry once a second. Safe to run: it never arms the vehicle.

python examples/telemetry.py                    # SITL on udp:127.0.0.1:14550
python examples/telemetry.py /dev/serial0       # Pi wired to a flight controller
"""

import sys
import time

import flypi

ADDRESS = sys.argv[1] if len(sys.argv) > 1 else flypi.DEFAULT_ADDRESS

with flypi.connect(ADDRESS) as drone:
    print(f"Connected: {drone}  (Ctrl+C to stop)")
    try:
        while True:
            t = drone.telemetry
            pos, bat, gps = t.position, t.battery, t.gps
            print(
                f"{t.mode or '?':>10} | armed={t.armed!s:5} | "
                f"lat={pos.lat} lon={pos.lon} alt={pos.alt_rel} m | "
                f"bat={bat.voltage} V {bat.remaining}% | "
                f"gps fix={gps.fix_type} sats={gps.satellites}"
            )
            time.sleep(1)
    except KeyboardInterrupt:
        pass
