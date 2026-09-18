"""Scan one servo bus: which IDs answer, and how reliably.

Read-only -- sends PING only, never a move or write command.

Usage:  python scan_bus.py COM6
"""

import sys

import serial


def ping(port, sid):
    port.reset_input_buffer()
    port.write(bytes([0xFF, 0xFF, sid, 0x02, 0x01, (~(sid + 3)) & 0xFF]))
    reply = port.read(6)
    return len(reply) >= 6 and reply[0:2] == b"\xff\xff" and reply[2] == sid


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        return 2
    device = sys.argv[1]

    with serial.Serial(device, 1_000_000, timeout=0.02, write_timeout=0.2) as port:
        found = [i for i in range(254) if ping(port, i)]
        print(f"{device} IDs found: {found}")

        if not found:
            print("\nNothing answered. Check DC power to this board and the 3-pin cable.")
            return 1

        print("\nstability (10 pings each):")
        flaky = []
        for sid in found:
            hits = sum(ping(port, sid) for _ in range(10))
            if hits < 10:
                flaky.append(sid)
            print(f"  id {sid:>3}: {hits}/10{'   <-- FLAKY' if hits < 10 else ''}")

    if flaky:
        print(f"\nFlaky IDs {flaky} -- likely two servos sharing an address.")
        return 1
    print("\nAll responding IDs are stable.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
