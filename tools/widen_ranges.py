"""Un-pin joints whose resting position sits on a range boundary.

A joint calibrated with a one-sided sweep ends up with range_max equal to its
reference pose. At rest it then normalizes to a constant +100 (or -100), so
moving the leader that direction does nothing and the follower never responds.

For every joint where the CURRENT position is within HEADROOM counts of a
boundary, this pushes that boundary out by EXTEND counts -- applied identically
to leader and follower, so the shared range (and the 1:1 mirror) is preserved.

Widening past the physical travel is harmless here: the follower is only ever
commanded to normalized values the leader actually produces, and the leader
cannot exceed its own mechanical limits.

Grippers are skipped -- they keep their own ranges by design.

Usage:  python widen_ranges.py          (dry run)
        python widen_ranges.py --write
"""

import json
import pathlib
import shutil
import sys

import serial

from calib_paths import FOLLOWER_CAL, LEADER_CAL, ROBOT_DIR, LEADER_DIR  # noqa: F401

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
SIDES = {
    "left": {"id": "leader_left", "prefix": "left_arm_", "lport": "COM5", "fport": "COM6"},
    "right": {"id": "leader_right", "prefix": "right_arm_", "lport": "COM3", "fport": "COM4"},
}
HEADROOM = 100  # counts; closer than this to a boundary counts as pinned
EXTEND = 300    # counts to push the boundary out by


def read_pos(port, sid):
    chk = (~(sid + 4 + 0x02 + 56 + 2)) & 0xFF
    port.reset_input_buffer()
    port.write(bytes([0xFF, 0xFF, sid, 0x04, 0x02, 56, 2, chk]))
    r = port.read(8)
    if len(r) < 8 or r[0:2] != b"\xff\xff" or r[2] != sid:
        return None
    return int.from_bytes(r[5:7], "little")


def read_all(device, ids):
    out = {}
    with serial.Serial(device, 1_000_000, timeout=0.05, write_timeout=0.2) as p:
        for name, sid in ids.items():
            out[name] = read_pos(p, sid)
    return out


def main():
    write = "--write" in sys.argv[1:]
    fdata = json.loads(FOLLOWER_CAL.read_text())
    changes = {}

    for side, cfg in SIDES.items():
        lpath = LEADER_CAL / f"{cfg['id']}.json"
        ldata = json.loads(lpath.read_text())
        ids = {j: ldata[j]["id"] for j in JOINTS}

        lead_pos = read_all(cfg["lport"], ids)
        foll_pos = read_all(cfg["fport"], ids)

        print(f"\n--- {side}")
        print(f"{'joint':<15}{'lead pos':>9}{'foll pos':>9}{'range':>14}{'headroom':>19}")
        for joint in JOINTS:
            fjoint = cfg["prefix"] + joint
            lo, hi = ldata[joint]["range_min"], ldata[joint]["range_max"]
            positions = [p for p in (lead_pos[joint], foll_pos[joint]) if p is not None]
            if not positions:
                print(f"{joint:<15}  no reply")
                continue

            head_max = hi - max(positions)
            head_min = min(positions) - lo
            new_lo, new_hi = lo, hi
            note = ""
            if head_max < HEADROOM:
                new_hi = max(positions) + EXTEND
                note += f"  max {hi}->{new_hi}"
            if head_min < HEADROOM:
                new_lo = min(positions) - EXTEND
                note += f"  min {lo}->{new_lo}"

            print(
                f"{joint:<15}{lead_pos[joint] if lead_pos[joint] is not None else '-':>9}"
                f"{foll_pos[joint] if foll_pos[joint] is not None else '-':>9}"
                f"{f'{lo}-{hi}':>14}{f'{head_min} / {head_max}':>19}{note}"
            )
            if (new_lo, new_hi) != (lo, hi):
                changes[(side, joint)] = (max(0, new_lo), min(4095, new_hi))

    if not changes:
        print("\nEvery joint has headroom at both ends. Nothing to do.")
        return 0

    print(f"\n{len(changes)} joint(s) pinned against a boundary:")
    for (side, joint), (lo, hi) in changes.items():
        print(f"  {side:<6} {joint:<15} -> {lo}-{hi}")

    if not write:
        print("\nDry run. Re-run with --write to apply:")
        print("  python widen_ranges.py --write")
        return 0

    for side, cfg in SIDES.items():
        lpath = LEADER_CAL / f"{cfg['id']}.json"
        ldata = json.loads(lpath.read_text())
        touched = False
        for (s, joint), (lo, hi) in changes.items():
            if s != side:
                continue
            ldata[joint]["range_min"], ldata[joint]["range_max"] = lo, hi
            fj = cfg["prefix"] + joint
            fdata[fj]["range_min"], fdata[fj]["range_max"] = lo, hi
            touched = True
        if touched:
            shutil.copy2(lpath, lpath.with_suffix(".json.bak"))
            lpath.write_text(json.dumps(ldata, indent=4))
            print(f"written: {lpath}")

    shutil.copy2(FOLLOWER_CAL, FOLLOWER_CAL.with_suffix(".json.bak"))
    FOLLOWER_CAL.write_text(json.dumps(fdata, indent=4))
    print(f"written: {FOLLOWER_CAL}")
    print("\nEEPROM is refreshed from these files when teleop connects.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
