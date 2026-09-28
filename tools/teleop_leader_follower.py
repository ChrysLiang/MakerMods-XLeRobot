"""Leader-follower teleoperation: two SO-101 leader arms drive the XLeRobot arms.

XLeRobot ships no leader-follower example (keyboard/Xbox/JoyCon/VR only), and
lerobot-teleoperate cannot target the 'xlerobot' type because
make_robot_from_config has no branch for it. So this wires LeRobot's SO101Leader
teleoperators straight to XLerobot.send_action().

Head and base are not commanded here -- arms only.

WHY NOT robot.connect():
  XLerobot.configure() calls enable_torque() with num_retry=0, so a single
  dropped Feetech status packet aborts the whole connect (that is what failed on
  right_arm_gripper). robust_configure() below is a faithful copy of
  XLerobot.configure() -- same registers, same values, same order -- with
  num_retry added. Doing the bus setup here also skips the interactive
  "restore calibration?" prompt on every retry.

SAFETY
  * max_relative_target clamps per-step joint motion, so a leader/follower
    mismatch ramps instead of snapping. Six follower joints currently have
    range 0-4095, which mis-scales their motion; this clamp is the guard.
  * Wheel Goal_Velocity is zeroed right after torque comes on.
  * You align the leaders to the follower BEFORE streaming starts.
  * Everything runs inside try/finally, so torque is always released.

Keep a hand on the power strip switch. Ctrl+C stops and disables torque.

Usage:  python teleop_leader_follower.py
"""

import sys
import time
import traceback

from calib_paths import ROBOT_DIR, LEADER_DIR
from lerobot.motors.feetech import OperatingMode
from lerobot.robots.xlerobot import XLerobot, XLerobotConfig
from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig

FPS = 30
MAX_RELATIVE_TARGET = 2  # normalized units per step; raise only once proven
NUM_RETRY = 5

LEADER_LEFT_PORT = "COM5"
LEADER_RIGHT_PORT = "COM3"

# Joints NOT commanded -- they stay where they are, holding torque.
#
# right_arm_wrist_flex was excluded while the follower had a glitched 4-4094 range
# against a correct leader, which would have commanded a full turn on a ~180 deg
# joint. After pair_calibrate/sync_ranges on the right side both arms share
# 37-2050, so the mismatch is gone and the exclusion is no longer needed.
EXCLUDE_JOINTS = set()


def robust_configure(robot):
    """XLerobot.configure() with num_retry on every write."""
    robot.bus1.disable_torque(num_retry=NUM_RETRY)
    robot.bus1.configure_motors()
    robot.bus2.disable_torque(num_retry=NUM_RETRY)
    robot.bus2.configure_motors()

    for bus, names in (
        (robot.bus1, robot.left_arm_motors + robot.head_motors),
        (robot.bus2, robot.right_arm_motors),
    ):
        for name in names:
            bus.write("Operating_Mode", name, OperatingMode.POSITION.value, num_retry=NUM_RETRY)
            bus.write("P_Coefficient", name, 16, num_retry=NUM_RETRY)
            bus.write("I_Coefficient", name, 0, num_retry=NUM_RETRY)
            bus.write("D_Coefficient", name, 43, num_retry=NUM_RETRY)

    for name in robot.base_motors:
        robot.bus2.write("Operating_Mode", name, OperatingMode.VELOCITY.value, num_retry=NUM_RETRY)

    robot.bus1.enable_torque(num_retry=NUM_RETRY)
    robot.bus2.enable_torque(num_retry=NUM_RETRY)


def alignment_table(robot, left_action, right_action):
    follower_left = robot.bus1.sync_read("Present_Position", robot.left_arm_motors, num_retry=NUM_RETRY)
    follower_right = robot.bus2.sync_read("Present_Position", robot.right_arm_motors, num_retry=NUM_RETRY)

    rows, worst = [], 0.0
    for side, action, follower, prefix in (
        ("L", left_action, follower_left, "left_arm_"),
        ("R", right_action, follower_right, "right_arm_"),
    ):
        for key, lead_val in action.items():
            joint = key.removesuffix(".pos")
            foll_val = follower.get(prefix + joint)
            if foll_val is None:
                continue
            diff = lead_val - foll_val
            excluded = (prefix + joint) in EXCLUDE_JOINTS
            if not excluded:
                worst = max(worst, abs(diff))
            rows.append((side, joint, lead_val, foll_val, diff, excluded))

    print(f"\n  {'':<2} {'joint':<16}{'leader':>9}{'follower':>10}{'diff':>9}")
    for side, joint, lead_val, foll_val, diff, excluded in rows:
        if excluded:
            flag = "  (not commanded)"
        else:
            flag = "  <-- align" if abs(diff) > 15 else ""
        print(f"  {side:<2} {joint:<16}{lead_val:>9.1f}{foll_val:>10.1f}{diff:>9.1f}{flag}")
    print(f"\n  largest mismatch: {worst:.1f}")
    return worst


def main():
    # max_relative_target is deliberately left None: XLerobot.send_action() has a
    # bug in that branch -- it indexes present_pos (keys without ".pos") using goal
    # keys (with ".pos"), raising KeyError. Since it defaults to None upstream, the
    # branch has evidently never been exercised. We clamp in the loop below instead.
    robot = XLerobot(XLerobotConfig(id="xlerobot", max_relative_target=None, calibration_dir=ROBOT_DIR))
    left = SO101Leader(SO101LeaderConfig(port=LEADER_LEFT_PORT, id="leader_left", use_degrees=False, calibration_dir=LEADER_DIR))
    right = SO101Leader(SO101LeaderConfig(port=LEADER_RIGHT_PORT, id="leader_right", use_degrees=False, calibration_dir=LEADER_DIR))

    print(f"follower bus1 {robot.bus1.port} / bus2 {robot.bus2.port}")
    print(f"leaders: left {LEADER_LEFT_PORT}, right {LEADER_RIGHT_PORT}")

    if not robot.calibration:
        print("No follower calibration loaded -- aborting.")
        return 1

    try:
        # Leaders stay torque-free; nothing moves yet.
        left.connect()
        right.connect()
        print("leaders connected.")

        print("\n" + "!" * 64)
        print("Next step ENERGIZES all 17 follower servos. The arms will stiffen")
        print("and hold position. Clear the workspace, support the arms, and keep")
        print("a hand on the power strip switch.")
        print("!" * 64)
        input("press ENTER to energize, or Ctrl+C to abort...")

        robot.bus1.connect()
        robot.bus2.connect()
        robot.bus1.write_calibration({k: v for k, v in robot.calibration.items() if k in robot.bus1.motors})
        robot.bus2.write_calibration({k: v for k, v in robot.calibration.items() if k in robot.bus2.motors})
        robust_configure(robot)
        robot.stop_base()
        print("follower energized, base velocity zeroed.")

        while True:
            worst = alignment_table(robot, left.get_action(), right.get_action())
            if worst > 15:
                print("\n  Move the LEADER arms to match the follower's pose.")
            choice = input("\n  ENTER to start teleop, 'r' to re-check, 'q' to quit: ").strip().lower()
            if choice == "q":
                return 0
            if choice != "r":
                break

        print(f"\nteleop at {FPS} Hz, max {MAX_RELATIVE_TARGET} units/step. Ctrl+C to stop.\n")
        period = 1.0 / FPS
        loop = 0
        while True:
            t0 = time.perf_counter()

            present = {
                **robot.bus1.sync_read("Present_Position", robot.left_arm_motors, num_retry=NUM_RETRY),
                **robot.bus2.sync_read("Present_Position", robot.right_arm_motors, num_retry=NUM_RETRY),
            }

            action = {}
            for prefix, leader in (("left_arm_", left), ("right_arm_", right)):
                for key, val in leader.get_action().items():
                    joint = prefix + key.removesuffix(".pos")
                    if joint in EXCLUDE_JOINTS:
                        continue
                    # Clamp per step: never command further than MAX_RELATIVE_TARGET
                    # from where the joint actually is, so a mismatch ramps in
                    # rather than snapping.
                    current = present.get(joint)
                    if current is not None:
                        val = min(current + MAX_RELATIVE_TARGET, max(current - MAX_RELATIVE_TARGET, val))
                    action[joint + ".pos"] = val

            robot.send_action(action)

            loop += 1
            if loop % FPS == 0:
                print(f"  {loop // FPS:>4}s running", end="\r")
            time.sleep(max(0.0, period - (time.perf_counter() - t0)))

    except KeyboardInterrupt:
        print("\n\nstopping...")
    except Exception:
        print("\n\nerror:")
        traceback.print_exc()
    finally:
        for label, fn in (
            ("stop_base", lambda: robot.stop_base()),
            ("bus1 torque off", lambda: robot.bus1.disable_torque(num_retry=NUM_RETRY)),
            ("bus2 torque off", lambda: robot.bus2.disable_torque(num_retry=NUM_RETRY)),
            ("bus1 close", lambda: robot.bus1.disconnect(False)),
            ("bus2 close", lambda: robot.bus2.disconnect(False)),
            ("leader L close", lambda: left.disconnect()),
            ("leader R close", lambda: right.disconnect()),
        ):
            try:
                fn()
            except Exception as exc:
                print(f"  cleanup: {label} failed ({type(exc).__name__})")
        print("shutdown complete -- torque disabled.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
