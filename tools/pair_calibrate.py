"""Calibrate one leader/follower ARM PAIR so their normalized frames correspond.

Two phases, in this order (it matters):

  1. ZERO. You pose both arms the same way; set_half_turn_homings() makes that
     pose read 2048 on BOTH arms. Their zeros now correspond.
     This must come first because set_half_turn_homings() calls
     reset_calibration(), which wipes the min/max limits.

  2. RANGE. Each joint is measured on both arms in one window -- sweep the
     leader's joint, then the follower's. Samples pass through a median-of-5
     filter, so the corrupt reads that pinned shoulder_lift and elbow_flex to
     0-4095 three times cannot get through.

Afterwards run sync_ranges.py to intersect each pair's ranges, which is what
makes one unit of leader motion equal one unit of follower motion.

The follower's head and wheel entries are preserved untouched.

Usage:  python pair_calibrate.py left
        python pair_calibrate.py right
"""

import json
import pathlib
import shutil
import statistics
import sys
import time
from collections import deque

from lerobot.robots.xlerobot import XLerobot, XLerobotConfig
from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig

CAL = pathlib.Path.home() / ".cache/huggingface/lerobot/calibration"
FOLLOWER_CAL = CAL / "robots/xlerobot/xlerobot.json"
LEADER_CAL = CAL / "teleoperators/so_leader"

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]

SIDES = {
    "left": {"port": "COM5", "id": "leader_left", "bus": "bus1", "prefix": "left_arm_"},
    "right": {"port": "COM3", "id": "leader_right", "bus": "bus2", "prefix": "right_arm_"},
}

WINDOW = 5
RECORD_SECONDS = 30.0
NUM_RETRY = 5


def save_json(path, data):
    if path.is_file():
        shutil.copy2(path, path.with_suffix(".json.bak"))
    path.write_text(json.dumps(data, indent=4))


def record_pair(leader_bus, follower_bus, joint, follower_joint):
    """One window; returns filtered (min,max) for leader and follower."""
    buf = {"lead": deque(maxlen=WINDOW), "foll": deque(maxlen=WINDOW)}
    filt = {"lead": [], "foll": []}
    raw = {"lead": [], "foll": []}

    end = time.time() + RECORD_SECONDS
    while time.time() < end:
        for tag, bus, name in (("lead", leader_bus, joint), ("foll", follower_bus, follower_joint)):
            try:
                pos = bus.sync_read("Present_Position", [name], normalize=False, num_retry=NUM_RETRY)[name]
            except Exception:
                continue
            raw[tag].append(pos)
            buf[tag].append(pos)
            if len(buf[tag]) == WINDOW:
                filt[tag].append(statistics.median(buf[tag]))
        if filt["lead"] and filt["foll"]:
            print(
                f"    leader {min(filt['lead']):>4}-{max(filt['lead']):<4}"
                f"   follower {min(filt['foll']):>4}-{max(filt['foll']):<4}"
                f"   [{end - time.time():>4.1f}s]",
                end="\r",
            )
        time.sleep(0.02)
    print(" " * 78, end="\r")

    out = {}
    for tag in ("lead", "foll"):
        if not filt[tag]:
            out[tag] = None
        else:
            out[tag] = (min(filt[tag]), max(filt[tag]), min(raw[tag]), max(raw[tag]))
    return out


def main():
    if len(sys.argv) != 2 or sys.argv[1] not in SIDES:
        print(__doc__)
        return 2
    side = sys.argv[1]
    cfg = SIDES[side]
    prefix = cfg["prefix"]
    follower_joints = [prefix + j for j in JOINTS]

    robot = XLerobot(XLerobotConfig(id="xlerobot", max_relative_target=None))
    leader = SO101Leader(SO101LeaderConfig(port=cfg["port"], id=cfg["id"], use_degrees=False))

    follower_bus = getattr(robot, cfg["bus"])
    leader.bus.connect()
    follower_bus.connect()
    print(f"leader {cfg['port']} | follower {follower_bus.port} | side: {side}")

    try:
        # ---- Phase 1: zero -------------------------------------------------
        print("\n" + "=" * 68)
        print("PHASE 1 -- ZERO")
        print("=" * 68)
        print("Pose the LEADER and FOLLOWER arms the SAME way. Match them by eye,")
        print("joint by joint: same shoulder angle, same elbow, same wrist.")
        print("Torque is off on both, so move them freely.")
        print("\nThe more carefully you match this pose, the better the follower")
        print("will track your hand. Take your time.")
        input("\npress ENTER when both arms are in the same pose...")

        lead_homing = leader.bus.set_half_turn_homings(JOINTS)
        foll_homing = follower_bus.set_half_turn_homings(follower_joints)
        print("zeroed -- both arms now read ~2048 at that pose.")

        # ---- Phase 2: ranges -----------------------------------------------
        print("\n" + "=" * 68)
        print("PHASE 2 -- RANGE  (both arms, one joint at a time)")
        print("=" * 68)

        lead_min, lead_max, foll_min, foll_max = {}, {}, {}, {}
        for joint in JOINTS:
            fjoint = prefix + joint
            while True:
                print(f"\n--- {joint}")
                print("    sweep the LEADER's joint to both mechanical stops,")
                print("    then the FOLLOWER's joint to both of its stops.")
                input(f"    press ENTER to record for {RECORD_SECONDS:.0f}s...")
                res = record_pair(leader.bus, follower_bus, joint, fjoint)

                ok = True
                for tag, label in (("lead", "leader"), ("foll", "follower")):
                    if res[tag] is None:
                        print(f"    {label}: no samples")
                        ok = False
                        continue
                    lo, hi, rlo, rhi = res[tag]
                    span = hi - lo
                    note = ""
                    if (rlo, rhi) != (lo, hi):
                        note = f"  (raw {rlo}-{rhi}, glitches rejected)"
                    print(f"    {label:<9} {lo:>4}-{hi:<4} span {span:>4} ({span * 360 / 4096:>3.0f} deg){note}")
                    if span < 200:
                        print(f"    -> {label} barely moved")
                        ok = False
                    elif span > 3900:
                        print(f"    -> {label} recorded a full turn")
                        ok = False
                if ok:
                    lead_min[joint], lead_max[joint] = int(res["lead"][0]), int(res["lead"][1])
                    foll_min[fjoint], foll_max[fjoint] = int(res["foll"][0]), int(res["foll"][1])
                    break
                if input("    retry this joint? [Y/n] ").strip().lower() == "n":
                    lead_min[joint], lead_max[joint] = int(res["lead"][0]), int(res["lead"][1])
                    foll_min[fjoint], foll_max[fjoint] = int(res["foll"][0]), int(res["foll"][1])
                    break

        # ---- Write ----------------------------------------------------------
        lpath = LEADER_CAL / f"{cfg['id']}.json"
        ldata = json.loads(lpath.read_text()) if lpath.is_file() else {}
        for joint in JOINTS:
            ldata.setdefault(joint, {"id": JOINTS.index(joint) + 1, "drive_mode": 0})
            ldata[joint]["homing_offset"] = int(lead_homing[joint])
            ldata[joint]["range_min"] = lead_min[joint]
            ldata[joint]["range_max"] = lead_max[joint]
        save_json(lpath, ldata)

        fdata = json.loads(FOLLOWER_CAL.read_text())
        for joint in JOINTS:
            fjoint = prefix + joint
            fdata[fjoint]["homing_offset"] = int(foll_homing[fjoint])
            fdata[fjoint]["range_min"] = foll_min[fjoint]
            fdata[fjoint]["range_max"] = foll_max[fjoint]
        save_json(FOLLOWER_CAL, fdata)

        # Push to EEPROM so the servos agree with the files.
        from lerobot.motors import MotorCalibration

        leader.bus.write_calibration(
            {j: MotorCalibration(**ldata[j]) for j in JOINTS}
        )
        follower_bus.write_calibration(
            {n: MotorCalibration(**fdata[n]) for n in follower_bus.motors if n in fdata}
        )

        print(f"\nwritten: {lpath}")
        print(f"written: {FOLLOWER_CAL}")
        print(f"\n{'joint':<16}{'leader':>14}{'follower':>16}")
        for joint in JOINTS:
            fjoint = prefix + joint
            print(
                f"{joint:<16}{f'{lead_min[joint]}-{lead_max[joint]}':>14}"
                f"{f'{foll_min[fjoint]}-{foll_max[fjoint]}':>16}"
            )
        print(f"\nNow run:  python sync_ranges.py {side}")
        return 0

    except KeyboardInterrupt:
        print("\naborted -- nothing written.")
        return 1
    finally:
        for fn in (
            lambda: leader.bus.disconnect(True),
            lambda: follower_bus.disconnect(True),
        ):
            try:
                fn()
            except Exception:
                pass
        print("disconnected, torque disabled.")


if __name__ == "__main__":
    sys.exit(main())
