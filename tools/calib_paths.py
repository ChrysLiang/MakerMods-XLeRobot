r"""Where these tools read and write calibration.

Repo-local by default, so calibration travels with the checkout rather than
living in ~/.cache/huggingface/lerobot/calibration. That keeps the robot's
measured ranges under version control alongside the code that produced them.

LeRobot resolves calibration as:

    calibration_dir   = config.calibration_dir or HF_LEROBOT_CALIBRATION/<kind>/<name>
    calibration_fpath = calibration_dir / f"{config.id}.json"

so passing calibration_dir= on the config is all that is needed to redirect it.
Scripts here pass ROBOT_DIR / LEADER_DIR accordingly.

Override the location with the XLEROBOT_CALIBRATION environment variable, e.g.
to fall back to LeRobot's default:

    $env:XLEROBOT_CALIBRATION = "$HOME\.cache\huggingface\lerobot\calibration"
"""

import os
import pathlib

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent

CALIBRATION_ROOT = pathlib.Path(
    os.environ.get("XLEROBOT_CALIBRATION", str(REPO_ROOT / "calibration"))
)

# Directories handed to LeRobot as calibration_dir=
ROBOT_DIR = CALIBRATION_ROOT / "robots" / "xlerobot"
LEADER_DIR = CALIBRATION_ROOT / "teleoperators" / "so_leader"

# Individual files, for the tools that edit JSON directly
FOLLOWER_CAL = ROBOT_DIR / "xlerobot.json"
LEADER_CAL = LEADER_DIR  # + f"{leader_id}.json"


def describe() -> str:
    where = "env XLEROBOT_CALIBRATION" if "XLEROBOT_CALIBRATION" in os.environ else "repo"
    return f"calibration ({where}): {CALIBRATION_ROOT}"
