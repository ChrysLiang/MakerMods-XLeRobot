"""Watch the head servos' positions for 20 s while you move the head by hand.

Reports the travel actually observed and whether it crosses the 0/4095 wrap.
Read-only. Torque must be off (it is -- health.py showed torq=0 everywhere).

Usage:  python sweep.py [COM6] [7 8]
"""

import sys
import time

import serial

DEVICE = sys.argv[1] if len(sys.argv) > 1 else "COM6"
IDS = [int(x) for x in sys.argv[2:]] or [7, 8]
DURATION = 60.0


def read_pos(port, sid):
    chk = (~(sid + 4 + 0x02 + 56 + 2)) & 0xFF
    port.reset_input_buffer()
    port.write(bytes([0xFF, 0xFF, sid, 0x04, 0x02, 56, 2, chk]))
    reply = port.read(8)
    if len(reply) < 8 or reply[0:2] != b"\xff\xff" or reply[2] != sid:
        return None
    return int.from_bytes(reply[5:7], "little")


def main():
    print(f"Move the head through its FULL travel now -- {DURATION:.0f} seconds.\n")
    seen = {sid: [] for sid in IDS}

    with serial.Serial(DEVICE, 1_000_000, timeout=0.05, write_timeout=0.2) as port:
        end = time.time() + DURATION
        while time.time() < end:
            line = []
            for sid in IDS:
                pos = read_pos(port, sid)
                if pos is not None:
                    seen[sid].append(pos)
                if seen[sid]:
                    lo, hi = min(seen[sid]), max(seen[sid])
                    line.append(f"id{sid}: now {pos if pos is not None else '----':>4}  range {lo:>4}-{hi:<4}")
            print("  " + " | ".join(line) + f"  [{end - time.time():>4.1f}s]", end="\r")
            time.sleep(0.05)

    print("\n\nresults:")
    for sid, samples in seen.items():
        if not samples:
            print(f"  id {sid}: no samples")
            continue
        lo, hi = min(samples), max(samples)
        moved = hi - lo
        print(f"  id {sid}: {lo} .. {hi}   travel {moved} counts ({moved * 360 / 4096:.0f} deg)")

        if moved < 50:
            print("      -> barely moved; was this joint included in the sweep?")
            continue
        # A joint straddling the wrap shows samples clustered at BOTH extremes
        near_zero = any(p < 200 for p in samples)
        near_max = any(p > 3895 for p in samples)
        if near_zero and near_max:
            print("      -> CROSSES THE 0/4095 WRAP. Re-seat the servo horn so the")
            print("         joint's neutral pose sits near 2048.")
        elif lo < 100:
            print("      -> travel starts at a mechanical stop near 0; usable range is")
            print(f"         {lo}..{hi}. Fine as long as commands stay inside that.")
        else:
            print("      -> clear of the wrap. No action needed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
