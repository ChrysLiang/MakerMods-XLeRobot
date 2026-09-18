"""Read Operating_Mode and Present_Position from every servo on every board.

Read-only: uses the Feetech READ instruction, never a write. Safe with DC live.
Register addresses taken from lerobot/motors/feetech/tables.py (STS series):
    Operating_Mode    (33, 1)   0=POSITION 1=VELOCITY 2=PWM 3=STEP
    Present_Position  (56, 2)   raw encoder counts, 0..4095

Usage:  python read_regs.py
"""

import sys

import serial
import serial.tools.list_ports as list_ports

BOARDS = {
    "5B8E114926": ("COM4 Follower right arm + wheels", [1, 2, 3, 4, 5, 6, 7, 8, 9]),
    "5B61037159": ("COM6 Follower left arm + head", [1, 2, 3, 4, 5, 6, 7, 8]),
    "5B8E113064": ("COM5 Leader L", [1, 2, 3, 4, 5, 6]),
    "5B91044284": ("COM3 Leader R", [1, 2, 3, 4, 5, 6]),
}

MODES = {0: "POSITION", 1: "VELOCITY", 2: "PWM", 3: "STEP"}


def read(port, sid, addr, size):
    """Feetech READ: FF FF <id> 04 02 <addr> <size> <checksum>."""
    chk = (~(sid + 4 + 0x02 + addr + size)) & 0xFF
    port.reset_input_buffer()
    port.write(bytes([0xFF, 0xFF, sid, 0x04, 0x02, addr, size, chk]))
    reply = port.read(6 + size)
    if len(reply) < 6 + size or reply[0:2] != b"\xff\xff" or reply[2] != sid:
        return None
    data = reply[5 : 5 + size]
    return int.from_bytes(data, "little")


def main():
    ports = {p.serial_number: p.device for p in list_ports.comports()}
    bad = 0

    for sn, (label, ids) in BOARDS.items():
        device = ports.get(sn)
        print(f"\n{label}  ({device or 'PORT MISSING'})")
        if device is None:
            bad += 1
            continue
        print(f"  {'id':>3}  {'mode':<9} {'raw pos':>7}  {'deg':>7}")
        with serial.Serial(device, 1_000_000, timeout=0.05, write_timeout=0.2) as port:
            for sid in ids:
                mode = read(port, sid, 33, 1)
                pos = read(port, sid, 56, 2)
                if mode is None or pos is None:
                    print(f"  {sid:>3}  READ FAILED")
                    bad += 1
                    continue
                deg = pos * 360.0 / 4096.0
                warn = "  <-- near end of range" if pos < 100 or pos > 3995 else ""
                print(f"  {sid:>3}  {MODES.get(mode, mode):<9} {pos:>7} {deg:>7.1f}{warn}")

    print("\nAll registers read OK" if not bad else f"\n{bad} read problem(s) above")
    return 0 if not bad else 1


if __name__ == "__main__":
    sys.exit(main())
