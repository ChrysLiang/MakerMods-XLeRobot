"""Pre-flight check for the MakerMods XLeRobot bus servo boards.

Pins each driver board by its CH343 serial number, pings every servo ID on its
bus, and prints found-vs-expected. Read-only: sends PING only, never a move
command, so it is safe to run with DC power live.

Usage:  python check_boards.py
"""

import sys

import serial
import serial.tools.list_ports as list_ports

# Board map for this robot, keyed by USB-serial number (stable across reboots
# because the CH343 reports a unique serial). Fill in as boards are identified.
BOARDS = {
    "5B8E114926": ("Follower - right arm + wheels", [1, 2, 3, 4, 5, 6, 7, 8, 9]),
    "5B61037159": ("Follower - left arm + head", [1, 2, 3, 4, 5, 6, 7, 8]),
    "5B8E113064": ("Leader L", [1, 2, 3, 4, 5, 6]),
    "5B91044284": ("Leader R", [1, 2, 3, 4, 5, 6]),
}

BAUDS = (1_000_000, 115_200)  # STS3215 ships at 1 Mbps; 115200 is the fallback
SCAN_IDS = range(1, 21)


def ping(port, servo_id):
    """Feetech SCS/STS ping: FF FF <id> 02 01 <checksum>."""
    packet = bytes([0xFF, 0xFF, servo_id, 0x02, 0x01, (~(servo_id + 3)) & 0xFF])
    port.reset_input_buffer()
    port.write(packet)
    reply = port.read(6)
    return len(reply) >= 6 and reply[0:2] == b"\xff\xff" and reply[2] == servo_id


def scan(device):
    """Return (baud, found_ids) for the first baud rate that answers."""
    for baud in BAUDS:
        try:
            with serial.Serial(device, baud, timeout=0.03, write_timeout=0.2) as port:
                found = [i for i in SCAN_IDS if ping(port, i)]
        except serial.SerialException as exc:
            return None, f"open failed: {exc}"
        if found:
            return baud, found
    return None, []


def main():
    ports = {p.serial_number: p.device for p in list_ports.comports()}
    if not ports:
        print("No serial ports found. Check USB cabling and hubs.")
        return 1

    rows, ok = [], True
    for sn, (name, expected) in BOARDS.items():
        device = ports.get(sn)
        if device is None:
            rows.append((f"SN {sn}", name, "-", "PORT MISSING"))
            ok = False
            continue

        baud, found = scan(device)
        if isinstance(found, str):
            rows.append((device, name, "-", found))
            ok = False
        elif not found:
            rows.append((device, name, "-", "NO SERVOS - check DC power / bus cable"))
            ok = False
        elif found == expected:
            rows.append((device, name, f"{baud//1000}k", f"OK  {found}"))
        else:
            missing = [i for i in expected if i not in found]
            extra = [i for i in found if i not in expected]
            detail = f"MISMATCH found={found}"
            if missing:
                detail += f" missing={missing}"
            if extra:
                detail += f" unexpected={extra}"
            rows.append((device, name, f"{baud//1000}k", detail))
            ok = False

    unmapped = set(ports) - set(BOARDS)
    width = max(len(r[1]) for r in rows)
    print(f"\n{'Port':<6} {'Board':<{width}} {'Baud':<6} Result")
    print("-" * (20 + width))
    for device, name, baud, result in rows:
        print(f"{device:<6} {name:<{width}} {baud:<6} {result}")

    for sn in sorted(unmapped):
        print(f"\nUnmapped port {ports[sn]} (SN {sn}) - add it to BOARDS.")

    print("\nALL CHECKS PASSED" if ok and not unmapped else "\nISSUES ABOVE - do not run teleop yet")
    return 0 if ok and not unmapped else 1


if __name__ == "__main__":
    sys.exit(main())
