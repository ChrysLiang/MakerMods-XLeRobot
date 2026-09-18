"""Emergency torque-off: disable Torque_Enable on every follower servo.

Uses raw serial writes rather than the LeRobot bus, so one unresponsive servo
cannot abort the whole shutdown. Also zeroes wheel Goal_Velocity first, in case
the wheels were left in VELOCITY mode.

Usage:  python safe_off.py
"""

import sys

import serial
import serial.tools.list_ports as list_ports

BOARDS = {
    "5B8E114926": ("COM4 right arm + wheels", [1, 2, 3, 4, 5, 6, 7, 8, 9], [7, 8, 9]),
    "5B61037159": ("COM6 left arm + head", [1, 2, 3, 4, 5, 6, 7, 8], []),
}

TORQUE_ENABLE = 40
GOAL_VELOCITY = 46


def write(port, sid, addr, value, size=1):
    data = list(value.to_bytes(size, "little"))
    length = size + 3
    chk = (~(sid + length + 0x03 + addr + sum(data))) & 0xFF
    port.reset_input_buffer()
    port.write(bytes([0xFF, 0xFF, sid, length, 0x03, addr, *data, chk]))
    reply = port.read(6)
    return len(reply) >= 6 and reply[0:2] == b"\xff\xff" and reply[2] == sid


def main():
    ports = {p.serial_number: p.device for p in list_ports.comports()}
    failed = []

    for sn, (label, ids, wheels) in BOARDS.items():
        device = ports.get(sn)
        print(f"\n{label} ({device or 'PORT MISSING'})")
        if device is None:
            failed.append(f"{label}: port missing")
            continue
        try:
            with serial.Serial(device, 1_000_000, timeout=0.05, write_timeout=0.3) as port:
                for sid in wheels:
                    ok = write(port, sid, GOAL_VELOCITY, 0, size=2)
                    print(f"  id {sid:>2} goal_velocity=0 {'ok' if ok else 'NO REPLY'}")
                for sid in ids:
                    ok = write(port, sid, TORQUE_ENABLE, 0)
                    print(f"  id {sid:>2} torque off      {'ok' if ok else 'NO REPLY'}")
                    if not ok:
                        failed.append(f"{label} id {sid}")
        except serial.SerialException as exc:
            print(f"  could not open: {exc}")
            failed.append(f"{label}: {exc}")

    if failed:
        print("\nDID NOT CONFIRM:")
        for f in failed:
            print(f"  - {f}")
        print("\nIf anything is still stiff or warm, switch the power strip OFF.")
        return 1
    print("\nAll servos confirmed torque-off and free to move by hand.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
