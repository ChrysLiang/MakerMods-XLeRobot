"""Re-measure range limits for specific joints, rejecting glitch reads, and patch
the LeRobot calibration JSON.

Read-only against the hardware -- it never writes to a servo. It edits the
calibration file; LeRobot writes that to EEPROM next time you restore it.

Glitch rejection: positions are sampled continuously and passed through a
median-of-5 filter, so an isolated corrupt read (the 0 / 4095 spikes we saw on
the gravity-loaded joints) cannot reach the recorded min/max.

Homing offsets are left untouched -- those came out correct.

Usage:  python fix_ranges.py
"""

import json
import pathlib
import shutil
import statistics
import sys
import time
from collections import deque

import serial

from calib_paths import FOLLOWER_CAL as CAL

PORTS = {"bus1": "COM6", "bus2": "COM4"}

# joints to re-measure: name -> (port, servo id, hint)
TARGETS = [
    ("left_arm_shoulder_lift", "bus1", 2, "raise/lower the left upper arm -- SUPPORT ITS WEIGHT"),
    ("left_arm_elbow_flex", "bus1", 3, "bend the left elbow -- SUPPORT ITS WEIGHT"),
    ("head_motor_1", "bus1", 7, "pan the head left/right -- ONLY as far as you want it to travel"),
    ("head_motor_2", "bus1", 8, "tilt the head -- sweep EVENLY above and below level"),
    ("right_arm_shoulder_lift", "bus2", 2, "raise/lower the right upper arm -- SUPPORT ITS WEIGHT"),
    ("right_arm_elbow_flex", "bus2", 3, "bend the right elbow -- SUPPORT ITS WEIGHT"),
    ("right_arm_wrist_flex", "bus2", 4, "bend the right wrist up/down"),
]

WINDOW = 5  # median filter width; an isolated spike cannot survive this


def read_pos(port, sid):
    chk = (~(sid + 4 + 0x02 + 56 + 2)) & 0xFF
    port.reset_input_buffer()
    port.write(bytes([0xFF, 0xFF, sid, 0x04, 0x02, 56, 2, chk]))
    reply = port.read(8)
    if len(reply) < 8 or reply[0:2] != b"\xff\xff" or reply[2] != sid:
        return None
    return int.from_bytes(reply[5:7], "little")


def record(port, sid, seconds=25.0):
    """Sample for `seconds`, returning filtered (min, max) and the raw spread."""
    buf, filtered, raw = deque(maxlen=WINDOW), [], []
    end = time.time() + seconds
    while time.time() < end:
        pos = read_pos(port, sid)
        if pos is not None:
            raw.append(pos)
            buf.append(pos)
            if len(buf) == WINDOW:
                filtered.append(statistics.median(buf))
        if filtered:
            print(
                f"    range {min(filtered):>4} - {max(filtered):<4}"
                f"   [{end - time.time():>4.1f}s]",
                end="\r",
            )
        time.sleep(0.03)
    print(" " * 60, end="\r")
    if not filtered:
        return None, None, 0, 0
    return min(filtered), max(filtered), min(raw), max(raw)


def main():
    if not CAL.is_file():
        print(f"No calibration file at {CAL}")
        return 1
    cal = json.loads(CAL.read_text())

    print(f"Calibration: {CAL}")
    print(f"Re-measuring {len(TARGETS)} joints. Homing offsets are left alone.\n")
    print("Move ONLY the named joint during each recording. Move smoothly and")
    print("support the arm's weight so it cannot drop under gravity.\n")

    results = {}
    for name, bus, sid, hint in TARGETS:
        if name not in cal:
            print(f"!! {name} not in calibration file, skipping")
            continue
        old = cal[name]
        span_old = old["range_max"] - old["range_min"]
        print(f"\n--- {name}  (id {sid} on {PORTS[bus]})")
        print(f"    now: {old['range_min']}..{old['range_max']}  span {span_old}")
        print(f"    {hint}")
        input("    press ENTER, then move the joint through its full range (25 s)...")

        with serial.Serial(PORTS[bus], 1_000_000, timeout=0.05, write_timeout=0.2) as port:
            lo, hi, raw_lo, raw_hi = record(port, sid)

        if lo is None:
            print("    no samples -- is the board powered?")
            continue
        span = hi - lo
        print(f"    filtered: {lo}..{hi}  span {span} ({span * 360 / 4096:.0f} deg)")
        if (raw_lo, raw_hi) != (lo, hi):
            print(f"    (raw was {raw_lo}..{raw_hi} -- glitches rejected)")
        if span < 200:
            print("    WARNING: barely moved. Re-run this joint.")
        elif span > 3900:
            print("    WARNING: still a full turn. Re-run this joint.")
        else:
            results[name] = (lo, hi)
            print("    accepted")

    if not results:
        print("\nNothing accepted; calibration file unchanged.")
        return 1

    backup = CAL.with_suffix(".json.bak")
    shutil.copy2(CAL, backup)
    for name, (lo, hi) in results.items():
        cal[name]["range_min"] = int(lo)
        cal[name]["range_max"] = int(hi)
    CAL.write_text(json.dumps(cal, indent=4))

    print(f"\nBackup: {backup}")
    print(f"Updated {len(results)} joints in {CAL}")
    print("\nNow run calibrate_xlerobot.py and press ENTER (do NOT type 'c')")
    print("to restore this file -- that writes the corrected limits to EEPROM.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
