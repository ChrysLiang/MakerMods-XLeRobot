"""Health snapshot for every servo: supply voltage, temperature, torque state,
position limits and homing offset.

Read-only. Register addresses from lerobot/motors/feetech/tables.py (STS series).

Usage:  python health.py
"""

import sys

import serial
import serial.tools.list_ports as list_ports

BOARDS = {
    "5B8E114926": ("COM4 Follower right arm + wheels", [1, 2, 3, 4, 5, 6, 7, 8, 9], 12.0),
    "5B61037159": ("COM6 Follower left arm + head", [1, 2, 3, 4, 5, 6, 7, 8], 12.0),
    "5B8E113064": ("COM5 Leader L", [1, 2, 3, 4, 5, 6], 7.4),
    "5B91044284": ("COM3 Leader R", [1, 2, 3, 4, 5, 6], 7.4),
}

REGS = {
    "min_lim": (9, 2),
    "max_lim": (11, 2),
    "homing": (31, 2),
    "torque": (40, 1),
    "pos": (56, 2),
    "volt": (62, 1),  # units of 0.1 V
    "temp": (63, 1),  # degrees C
}


def read(port, sid, addr, size):
    chk = (~(sid + 4 + 0x02 + addr + size)) & 0xFF
    port.reset_input_buffer()
    port.write(bytes([0xFF, 0xFF, sid, 0x04, 0x02, addr, size, chk]))
    reply = port.read(6 + size)
    if len(reply) < 6 + size or reply[0:2] != b"\xff\xff" or reply[2] != sid:
        return None
    return int.from_bytes(reply[5 : 5 + size], "little")


def main():
    ports = {p.serial_number: p.device for p in list_ports.comports()}
    notes = []

    for sn, (label, ids, nominal) in BOARDS.items():
        device = ports.get(sn)
        print(f"\n{label}   (expect ~{nominal} V)")
        if device is None:
            notes.append(f"{label}: port missing")
            continue
        print(f"  {'id':>3} {'volt':>6} {'temp':>5} {'torq':>5} {'pos':>6} {'min':>6} {'max':>6} {'homing':>7}")
        with serial.Serial(device, 1_000_000, timeout=0.05, write_timeout=0.2) as port:
            for sid in ids:
                v = {k: read(port, sid, a, s) for k, (a, s) in REGS.items()}
                if any(x is None for x in v.values()):
                    print(f"  {sid:>3}  READ FAILED")
                    notes.append(f"{label} id {sid}: read failed")
                    continue
                volts = v["volt"] / 10.0
                print(
                    f"  {sid:>3} {volts:>5.1f}V {v['temp']:>4}C {v['torque']:>5} "
                    f"{v['pos']:>6} {v['min_lim']:>6} {v['max_lim']:>6} {v['homing']:>7}"
                )
                if abs(volts - nominal) > nominal * 0.15:
                    notes.append(f"{label} id {sid}: supply {volts:.1f} V vs nominal {nominal} V")
                if v["temp"] > 50:
                    notes.append(f"{label} id {sid}: {v['temp']} C -- hot")
                if v["pos"] < 100 or v["pos"] > 3995:
                    notes.append(f"{label} id {sid}: position {v['pos']} is within 100 counts of the 0/4095 wrap")

    print("\n" + ("No issues flagged." if not notes else "Flagged:"))
    for n in notes:
        print(f"  - {n}")
    return 0 if not notes else 1


if __name__ == "__main__":
    sys.exit(main())
