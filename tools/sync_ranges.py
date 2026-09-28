"""Give a leader/follower arm pair identical range limits.

For each joint the new range is the INTERSECTION of the two measured ranges:

    range_min = max(leader_min, follower_min)
    range_max = min(leader_max, follower_max)

Intersection, not union, on purpose: normalization clamps commands to the range,
so a range inside both arms' real travel can never drive either into a hard stop.
A range wider than the follower's travel would stall it against its stop.

With identical ranges AND zeros that correspond (pair_calibrate.py phase 1), one
unit of leader motion equals one unit of follower motion -- a true 1:1 mirror.

THE GRIPPER IS DELIBERATELY EXCLUDED. It normalizes 0-100 over its own
closed..open travel, and the leader's trigger and the follower's jaw are
different mechanisms. Each keeps its own range so fully-closed maps to
fully-closed, which is what you actually want.

Run pair_calibrate.py for the side first. Requires --write to modify anything.

Usage:  python sync_ranges.py left [--write]
        python sync_ranges.py right [--write]
"""

import json
import pathlib
import shutil
import sys

from calib_paths import FOLLOWER_CAL, LEADER_CAL, ROBOT_DIR, LEADER_DIR  # noqa: F401

SYNC_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
SIDES = {
    "left": {"id": "leader_left", "prefix": "left_arm_", "port": "COM5", "bus": "bus1"},
    "right": {"id": "leader_right", "prefix": "right_arm_", "port": "COM3", "bus": "bus2"},
}
MIN_USABLE_SPAN = 200


def main():
    if len(sys.argv) < 2 or sys.argv[1] not in SIDES:
        print(__doc__)
        return 2
    side = sys.argv[1]
    write = "--write" in sys.argv[2:]
    cfg = SIDES[side]

    lpath = LEADER_CAL / f"{cfg['id']}.json"
    ldata = json.loads(lpath.read_text())
    fdata = json.loads(FOLLOWER_CAL.read_text())

    print(f"side: {side}   leader: {lpath.name}\n")
    print(f"{'joint':<15}{'leader':>13}{'follower':>13}{'-> common':>14}{'span':>7}")

    plan, problems = {}, []
    for joint in SYNC_JOINTS:
        fjoint = cfg["prefix"] + joint
        if joint not in ldata or fjoint not in fdata:
            problems.append(f"{joint}: missing from a calibration file")
            continue
        lo = max(ldata[joint]["range_min"], fdata[fjoint]["range_min"])
        hi = min(ldata[joint]["range_max"], fdata[fjoint]["range_max"])
        span = hi - lo
        flag = ""
        if span < MIN_USABLE_SPAN:
            flag = "  <-- TOO NARROW"
            problems.append(f"{joint}: overlap only {span} counts")
        else:
            plan[joint] = (lo, hi)
        print(
            f"{joint:<15}"
            f"{f'{ldata[joint]['range_min']}-{ldata[joint]['range_max']}':>13}"
            f"{f'{fdata[fjoint]['range_min']}-{fdata[fjoint]['range_max']}':>13}"
            f"{f'{lo}-{hi}':>14}{span:>7}{flag}"
        )

    gj = cfg["prefix"] + "gripper"
    if "gripper" in ldata and gj in fdata:
        print(
            f"\n{'gripper':<15}"
            f"{f'{ldata['gripper']['range_min']}-{ldata['gripper']['range_max']}':>13}"
            f"{f'{fdata[gj]['range_min']}-{fdata[gj]['range_max']}':>13}"
            f"{'(left as-is)':>14}"
        )

    if problems:
        print("\nproblems:")
        for p in problems:
            print(f"  - {p}")
        print("\nA tiny overlap usually means the two arms were swept over")
        print("different travel, or a range is still a glitched 0-4095.")
        print("Re-run pair_calibrate.py for this side before syncing.")

    if not plan:
        print("\nnothing to write.")
        return 1

    if not write:
        print(f"\nDry run. Re-run with --write to apply:")
        print(f"  python sync_ranges.py {side} --write")
        return 0

    for joint, (lo, hi) in plan.items():
        ldata[joint]["range_min"], ldata[joint]["range_max"] = lo, hi
        fj = cfg["prefix"] + joint
        fdata[fj]["range_min"], fdata[fj]["range_max"] = lo, hi

    for path, data in ((lpath, ldata), (FOLLOWER_CAL, fdata)):
        shutil.copy2(path, path.with_suffix(".json.bak"))
        path.write_text(json.dumps(data, indent=4))
        print(f"written: {path}")

    # Push to EEPROM so servos and files agree.
    try:
        from lerobot.motors import MotorCalibration
        from lerobot.robots.xlerobot import XLerobot, XLerobotConfig
        from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig

        robot = XLerobot(XLerobotConfig(id="xlerobot", max_relative_target=None, calibration_dir=ROBOT_DIR))
        leader = SO101Leader(SO101LeaderConfig(port=cfg["port"], id=cfg["id"], use_degrees=False, calibration_dir=LEADER_DIR))
        bus = getattr(robot, cfg["bus"])
        leader.bus.connect()
        bus.connect()
        try:
            leader.bus.write_calibration({j: MotorCalibration(**ldata[j]) for j in ldata})
            bus.write_calibration({n: MotorCalibration(**fdata[n]) for n in bus.motors if n in fdata})
            print("EEPROM updated on both arms.")
        finally:
            leader.bus.disconnect(True)
            bus.disconnect(True)
    except Exception as exc:
        print(f"\nFiles written, but EEPROM update failed: {type(exc).__name__}: {exc}")
        print("The files are what teleop reads, so this is not fatal.")

    print("\nDone. Re-run teleop and check the alignment table -- the diffs")
    print("should now be small when the arms are genuinely in the same pose.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
