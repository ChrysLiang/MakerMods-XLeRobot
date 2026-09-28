"""Run XLeRobot's calibration. Must be run in an interactive terminal -- it prompts.

Writes homing offsets and range limits to servo EEPROM, then saves a calibration
JSON. This is what fixes the head servos' 0/4095 wrap: whatever position the head
is in when you press ENTER at the "middle of their range" prompt becomes 2048.

Usage:  python calibrate_xlerobot.py
"""

import logging

from calib_paths import ROBOT_DIR
from lerobot.robots.xlerobot import XLerobot, XLerobotConfig

logging.basicConfig(level=logging.INFO, format="%(levelname)s  %(message)s")

cfg = XLerobotConfig(id="xlerobot", calibration_dir=ROBOT_DIR)
robot = XLerobot(cfg)

print(f"\nbus1 (left arm + head) : {cfg.port1}")
print(f"bus2 (right arm + base): {cfg.port2}")
print(f"calibration will be saved to: {robot.calibration_fpath}\n")

robot.connect(calibrate=True)
print(f"\nis_calibrated: {robot.is_calibrated}")
robot.disconnect()
print("done -- torque disabled, disconnected.")
