# XLeRobot bring-up and calibration tools

Helper scripts for bringing up the MakerMods XLeRobot: verifying the servo buses,
calibrating leader/follower arm pairs, and running leader-follower teleoperation.

## Environment

These scripts need the `lerobot` conda environment — LeRobot 0.6.1, installed
editable from `C:\02_FYP\lerobot`, with XLeRobot's `robots/xlerobot` and
`model/SO101Robot.py` copied into its source tree.

**`ModuleNotFoundError: No module named 'lerobot'` means you are on the wrong
interpreter**, not that anything is broken. The system Python
(`C:\Python313\python.exe`) does not have LeRobot; only the conda env does.

Either activate the env:

```
conda activate lerobot
python tools\check_boards.py
```

(`conda activate` needs conda's PowerShell hook — if it fails, run
`conda init powershell` once and reopen the terminal.)

Or call the env's interpreter directly, which needs no activation:

```
& "C:\Users\liangnanyi\.conda\envs\lerobot\python.exe" tools\check_boards.py
```

Only four scripts actually import LeRobot and therefore need the env:

    calibrate_xlerobot.py   pair_calibrate.py
    recalibrate_all.py      teleop_leader_follower.py

The other ten need nothing but `pyserial`:

    check_boards.py  scan_bus.py    health.py       read_regs.py   sweep.py
    diag_joint.py    safe_off.py    widen_ranges.py fix_ranges.py  sync_ranges.py

`safe_off.py` is deliberately in that group — the emergency stop must work from
any interpreter, without the env. `sync_ranges.py` imports LeRobot lazily, inside
a try, only to push the new limits to EEPROM; without the env it still writes the
calibration files, which is what teleop actually reads.

## Hardware map

| Port | Serial (CH343) | Board |
| ---- | -------------- | ----- |
| COM6 | 5B61037159 | follower bus1 — left arm (1-6) + head (7, 8) |
| COM4 | 5B8E114926 | follower bus2 — right arm (1-6) + wheels (7, 8, 9) |
| COM5 | 5B8E113064 | leader left (1-6) |
| COM3 | 5B91044284 | leader right (1-6) |

Ports are pinned by USB serial number, so they survive replugging and reboots.

## Safety

**`safe_off.py`** — emergency torque-off for all 17 follower servos. Uses raw
serial writes rather than the LeRobot bus, so one unresponsive servo cannot block
the shutdown. Also zeroes wheel `Goal_Velocity`.

```
python tools/safe_off.py
```

The power strip switch is faster. Use it first if anything is moving unexpectedly.

## Diagnostics (read-only, safe with power on)

| Script | Purpose |
| ------ | ------- |
| `check_boards.py` | Pings all four boards, checks every expected servo ID answers. Run this first, always. |
| `scan_bus.py COM6` | Full 0-253 ID scan on one bus plus a 10-ping stability test. Flaky IDs mean two servos share an address. |
| `health.py` | Supply voltage, temperature, torque state, position limits and homing offset for every servo. |
| `read_regs.py` | Operating mode and present position for every servo. |
| `sweep.py` | Watches chosen joints for 60 s while you move them by hand; reports travel and flags 0/4095 wrap. |
| `diag_joint.py` | Deep dive on specific joints: position vs range, last commanded goal, torque, load, temperature, error flags. |

## Calibration

Run in this order. Order matters: `set_half_turn_homings()` calls
`reset_calibration()`, which wipes the min/max limits — so zeroing must precede
range measurement.

**1. `pair_calibrate.py left|right`** — calibrates one leader/follower pair.
   - *Phase 1 (zero):* you pose both arms identically; both are homed so that
     pose reads 2048. This is what makes their normalized frames correspond.
   - *Phase 2 (range):* each joint measured on both arms in one 30 s window.

**2. `sync_ranges.py left|right [--write]`** — gives the pair identical range
   limits, using the *intersection* of the two measured ranges so neither arm can
   be commanded into a hard stop. Grippers are excluded by design. Dry run
   without `--write`.

**3. `widen_ranges.py [--write]`** — finds joints resting on a range boundary and
   pushes that boundary out. A joint pinned at its limit normalizes to a constant
   ±100 and will not respond to the leader at all.

Other calibration helpers:

- `calibrate_xlerobot.py` — runs XLeRobot's own stock calibration. Press ENTER to
  restore from file, or `c` to recalibrate manually.
- `fix_ranges.py` — re-measures a named list of joints with glitch rejection and
  patches the calibration JSON. Does not write to servos.
- `recalibrate_all.py` — full follower recalibration with filtered range recording.

### Known issue: glitched range recording

LeRobot's `record_ranges_of_motion()` takes a plain `min()`/`max()` over raw
samples with no outlier rejection, so a single corrupt read pins a joint to
`0-4095`. This reliably affects the gravity-loaded joints (`shoulder_lift`,
`elbow_flex`) because releasing the arm lets it drop fast, and fast back-driving
corrupts Feetech position reads.

The scripts here use a median-of-5 filter to reject isolated spikes. For sustained
bursts a median filter is not enough — velocity-based rejection (discard any
sample more than ~200 counts from the last accepted one) would be needed.

## Teleoperation

**`teleop_leader_follower.py`** — two SO-101 leader arms drive the XLeRobot arms.

```
python tools/teleop_leader_follower.py
```

Head and base are not commanded; arms only.

Notes on why it is written the way it is:

- `lerobot-teleoperate` cannot target `--robot.type=xlerobot` — LeRobot's
  `make_robot_from_config()` is a hardcoded if/elif chain with no `xlerobot`
  branch. XLeRobot's own 16 examples are keyboard/Xbox/JoyCon/VR only, none
  leader-follower. Hence a custom script.
- `robust_configure()` replaces `XLerobot.configure()`, which calls
  `enable_torque()` with `num_retry=0` — one dropped Feetech status packet aborts
  the whole connect.
- `max_relative_target` is left `None` because that branch of
  `XLerobot.send_action()` has a bug: it indexes `present_pos` (keys without
  `.pos`) using goal keys (with `.pos`), raising `KeyError`. The per-step clamp is
  applied in the teleop loop instead.

Tunables at the top of the file: `MAX_RELATIVE_TARGET` (start at 2, raise to 5
then 10 once tracking is proven), `FPS`, and `EXCLUDE_JOINTS`.

## Typical bring-up sequence

```
python tools/check_boards.py                  # all four boards, all servos
python tools/health.py                        # voltages, temperatures, limits
python tools/pair_calibrate.py left
python tools/sync_ranges.py left --write
python tools/pair_calibrate.py right
python tools/sync_ranges.py right --write
python tools/widen_ranges.py --write
python tools/teleop_leader_follower.py
```

## Calibration file locations

Calibration lives **in the repo**, not in `~/.cache`, so the robot's measured
ranges are version-controlled alongside the code that produced them:

```
calibration/robots/xlerobot/xlerobot.json              17 motors (12 arm + 2 head + 3 wheel)
calibration/teleoperators/so_leader/leader_left.json    6 motors  (COM5)
calibration/teleoperators/so_leader/leader_right.json   6 motors  (COM3)
```

`tools/calib_paths.py` defines these paths and every tool imports them, so the
calibration tools and teleop can never drift apart. LeRobot resolves calibration
as `calibration_dir / f"{id}.json"`, so the scripts simply pass
`calibration_dir=` on each config.

To fall back to LeRobot's default location, set an environment variable:

```powershell
$env:XLEROBOT_CALIBRATION = "$HOME\.cache\huggingface\lerobot\calibration"
```

Scripts that modify these files write a `.json.bak` alongside first — one deep,
overwritten each time, so copy a file by hand if you want a lasting snapshot.

Note that `~/.cache/.../robots/so_follower/arm_a.json` (if present) belongs to a
different robot class and is never read by `XLerobot`.
