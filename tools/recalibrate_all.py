"""Full XLeRobot recalibration with glitch-resistant range recording.

Same structure as XLerobot.calibrate(), with one change: LeRobot's
record_ranges_of_motion() takes a plain min/max over raw samples, so a single
corrupt read pins a joint's range to 0 or 4095. That is what happened to
shoulder_lift and elbow_flex on both arms. Here every sample passes through a
median-of-5 filter first, and each joint is recorded on its own with a sanity
check and the option to retry.

Homing offsets and the EEPROM write still go through LeRobot's own code
(set_half_turn_homings / write_calibration).

Run this in a real terminal -- it prompts.

Usage:  python recalibrate_all.py
"""

import statistics
import sys
import time
from collections import deque

from calib_paths import ROBOT_DIR
from lerobot.motors import MotorCalibration
from lerobot.motors.feetech import OperatingMode
from lerobot.robots.xlerobot import XLerobot, XLerobotConfig

WINDOW = 5  # median filter width -- an isolated spike cannot survive this
RECORD_SECONDS = 25.0

HINTS = {
    "shoulder_pan": "rotate the whole arm about the vertical axis",
    "shoulder_lift": "raise/lower the upper arm -- SUPPORT ITS WEIGHT",
    "elbow_flex": "bend the elbow -- SUPPORT ITS WEIGHT",
    "wrist_flex": "bend the wrist up/down",
    "wrist_roll": "rotate the gripper about the forearm axis",
    "gripper": "open and close the gripper fully",
    "head_motor_1": "pan the head left/right -- only as far as you want it to go",
    "head_motor_2": "tilt the head -- sweep EVENLY above and below level",
}


def hint_for(name):
    for key, text in HINTS.items():
        if name.endswith(key) or name == key:
            return text
    return "move this joint through its range"


def record_joint(bus, name):
    """Median-filtered min/max for one joint. Returns (lo, hi, raw_lo, raw_hi)."""
    buf, filtered, raw = deque(maxlen=WINDOW), [], []
    end = time.time() + RECORD_SECONDS
    while time.time() < end:
        pos = bus.sync_read("Present_Position", [name], normalize=False, num_retry=5)[name]
        raw.append(pos)
        buf.append(pos)
        if len(buf) == WINDOW:
            filtered.append(statistics.median(buf))
        if filtered:
            print(
                f"    range {min(filtered):>4} - {max(filtered):<4}   [{end - time.time():>4.1f}s]",
                end="\r",
            )
        time.sleep(0.03)
    print(" " * 60, end="\r")
    if not filtered:
        return None, None, None, None
    return min(filtered), max(filtered), min(raw), max(raw)


def record_with_retry(bus, name):
    while True:
        print(f"\n--- {name}")
        print(f"    {hint_for(name)}")
        input("    press ENTER, then move the joint through its full range (25 s)...")
        lo, hi, raw_lo, raw_hi = record_joint(bus, name)

        if lo is None:
            print("    no samples read.")
        else:
            span = hi - lo
            print(f"    filtered: {lo}..{hi}  span {span} ({span * 360 / 4096:.0f} deg)")
            if (raw_lo, raw_hi) != (lo, hi):
                print(f"    raw was {raw_lo}..{raw_hi} -- glitches rejected")
            if span < 200:
                print("    REJECTED: barely moved.")
            elif span > 3900:
                print("    REJECTED: full turn recorded.")
            else:
                return int(lo), int(hi)

        if input("    retry this joint? [Y/n] ").strip().lower() == "n":
            return int(lo), int(hi)


def calibrate_bus(bus, joints, label):
    print(f"\n{'=' * 60}\n{label}\n{'=' * 60}")
    bus.disable_torque()
    for name in joints:
        bus.write("Operating_Mode", name, OperatingMode.POSITION.value)

    input(f"\nMove {label} to the MIDDLE of their range of motion, then press ENTER...")
    homing = bus.set_half_turn_homings(joints)
    print("homing offsets written; every joint now reads ~2048 at this pose.")

    mins, maxes = {}, {}
    for name in joints:
        mins[name], maxes[name] = record_with_retry(bus, name)
    return homing, mins, maxes


def main():
    robot = XLerobot(XLerobotConfig(id="xlerobot", calibration_dir=ROBOT_DIR))
    robot.bus1.connect()
    robot.bus2.connect()
    print(f"bus1 {robot.bus1.port} | bus2 {robot.bus2.port}")

    left_joints = robot.left_arm_motors + robot.head_motors
    homing1, mins1, maxes1 = calibrate_bus(robot.bus1, left_joints, "LEFT ARM + HEAD")

    right_joints = robot.right_arm_motors
    homing2, mins2, maxes2 = calibrate_bus(robot.bus2, right_joints, "RIGHT ARM")

    # Wheels are continuous rotation: full range, no homing. Same as upstream.
    for name in robot.base_motors:
        homing2[name] = 0
        mins2[name] = 0
        maxes2[name] = 4095

    cal1 = {
        name: MotorCalibration(
            id=motor.id, drive_mode=0, homing_offset=homing1[name],
            range_min=mins1[name], range_max=maxes1[name],
        )
        for name, motor in robot.bus1.motors.items()
    }
    cal2 = {
        name: MotorCalibration(
            id=motor.id, drive_mode=0, homing_offset=homing2[name],
            range_min=mins2[name], range_max=maxes2[name],
        )
        for name, motor in robot.bus2.motors.items()
    }

    robot.bus1.write_calibration(cal1)
    robot.bus2.write_calibration(cal2)
    robot.calibration = {**cal1, **cal2}
    robot._save_calibration()
    print(f"\nCalibration written to EEPROM and saved to {robot.calibration_fpath}")

    print(f"\n{'joint':<26}{'homing':>8}{'min':>7}{'max':>7}{'span':>7}")
    for name, c in robot.calibration.items():
        print(f"{name:<26}{c.homing_offset:>8}{c.range_min:>7}{c.range_max:>7}{c.range_max - c.range_min:>7}")

    robot.bus1.disconnect()
    robot.bus2.disconnect()
    print("\ndone -- torque disabled, disconnected.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
