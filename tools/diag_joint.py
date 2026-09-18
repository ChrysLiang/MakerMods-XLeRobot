"""Diagnose stuck joints: position vs range, torque, last goal, load, errors."""
import json, pathlib, serial

CAL = pathlib.Path.home()/".cache/huggingface/lerobot/calibration"
foll = json.load(open(CAL/"robots/xlerobot/xlerobot.json"))
lead = {"L": json.load(open(CAL/"teleoperators/so_leader/leader_left.json")),
        "R": json.load(open(CAL/"teleoperators/so_leader/leader_right.json"))}

TARGETS = [
    ("L elbow_flex leader",   "COM5", 3, lead["L"]["elbow_flex"]),
    ("L elbow_flex follower", "COM6", 3, foll["left_arm_elbow_flex"]),
    ("R wrist_flex leader",   "COM3", 4, lead["R"]["wrist_flex"]),
    ("R wrist_flex follower", "COM4", 4, foll["right_arm_wrist_flex"]),
    ("L wrist_flex follower", "COM6", 4, foll["left_arm_wrist_flex"]),  # working joint, for contrast
]
REGS = {"min_eeprom":(9,2), "max_eeprom":(11,2), "torque":(40,1),
        "goal":(42,2), "pos":(56,2), "load":(60,2), "temp":(63,1), "status":(65,1)}

def read(port, sid, addr, size):
    chk = (~(sid + 4 + 0x02 + addr + size)) & 0xFF
    port.reset_input_buffer()
    port.write(bytes([0xFF,0xFF,sid,0x04,0x02,addr,size,chk]))
    r = port.read(6+size)
    if len(r) < 6+size or r[0:2] != b"\xff\xff" or r[2] != sid: return None
    return int.from_bytes(r[5:5+size],"little")

for label, dev, sid, cal in TARGETS:
    try:
        with serial.Serial(dev, 1_000_000, timeout=0.05, write_timeout=0.2) as p:
            v = {k: read(p, sid, a, s) for k,(a,s) in REGS.items()}
    except Exception as e:
        print(f"{label}: open failed: {e}\n"); continue
    if v["pos"] is None:
        print(f"{label}: no reply\n"); continue
    lo, hi = cal["range_min"], cal["range_max"]
    print(f"{label}  ({dev} id{sid})")
    print(f"   file range  {lo}-{hi}       eeprom limits {v['min_eeprom']}-{v['max_eeprom']}")
    print(f"   position {v['pos']}   goal {v['goal']}   torque {v['torque']}"
          f"   load {v['load']}   temp {v['temp']}C   status 0x{v['status']:02x}")
    if v["pos"] >= hi:   print(f"   *** position {v['pos']} is AT/ABOVE range max {hi} -> normalizes to +100 and stops changing")
    elif v["pos"] <= lo: print(f"   *** position {v['pos']} is AT/BELOW range min {lo} -> normalizes to -100 and stops changing")
    if not (lo < 2048 < hi): print(f"   *** homing reference 2048 is OUTSIDE {lo}-{hi}")
    if v["status"]: print(f"   *** servo error flags set: 0x{v['status']:02x}")
    print()
