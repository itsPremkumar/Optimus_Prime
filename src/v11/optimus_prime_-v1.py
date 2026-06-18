if 'TARGET_MODULE' not in globals():
    TARGET_MODULE = "ALL"
if 'EXPORT_STL' not in globals():
    EXPORT_STL = False
if 'EXPORT_STEP' not in globals():
    EXPORT_STEP = False
if 'EXPORT_URDF' not in globals():
    EXPORT_URDF = False
if 'CAPTURE_SCREENSHOTS' not in globals():
    CAPTURE_SCREENSHOTS = False
if 'GENERATE_DOCS' not in globals():
    GENERATE_DOCS = True

import adsk.core, adsk.fusion, traceback, math, os, csv, json, datetime, time

_ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

# ─── Output directories ─────────────────────────────────────────────────────
try:
    SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
except NameError:
    SCRIPT_DIR = os.getcwd()
PROJECT_DIR    = os.path.dirname(os.path.dirname(SCRIPT_DIR))
_OUTPUT_DIR    = os.path.join(PROJECT_DIR, "output")
LOG_DIR        = globals().get("LOG_DIR")        or os.path.join(_OUTPUT_DIR, "logs")
SCREENSHOT_DIR = globals().get("SCREENSHOT_DIR") or os.path.join(_OUTPUT_DIR, "screenshots")
EXPORT_DIR     = globals().get("EXPORT_DIR")     or os.path.join(_OUTPUT_DIR, "exports")
DOCS_DIR       = os.path.join(_OUTPUT_DIR, "build_docs")
LOG_FILE       = os.path.join(LOG_DIR,     f"optimus_v12_{_ts}.txt")
BOM_FILE       = os.path.join(_OUTPUT_DIR, f"BOM_v12_{_ts}.csv")
GUIDE_FILE     = os.path.join(DOCS_DIR,    f"assembly_guide_v12_{_ts}.txt")
BUILD_DOC_FILE = os.path.join(DOCS_DIR,    f"BuildGuide_v12_{_ts}.md")
STOP_FLAG      = os.path.join(_OUTPUT_DIR, "stop.flag")

# ═════════════════════════════════════════════════════════════════════════════
# CONFIGURATION  — ALL DIMENSIONS IN cm (Fusion 360 native unit)
# ═════════════════════════════════════════════════════════════════════════════

# ── Skeleton Z-heights ──────────────────────────────────────────────────────
CLEARANCE    = 0.060
FDM_TOL      = 0.015
ANKLE_CTR    = 3.8
SHIN_CTR     = 9.3
KNEE_CTR     = 14.8
THIGH_CTR    = 20.3
PELVIS_CTR   = 30.5
WAIST_CTR    = 32.5
TORSO_CTR    = 36.0
SHOULDER_CTR = 41.5
HEAD_CTR     = 47.5
HIP_X        = 5.8
SHOULDER_X   = 13.0
ELBOW_Z      = 35.0
WRIST_Z      = 29.0
HIP_JOINT_Z  = 26.8
NECK_JOINT_Z = 44.5

# ── Physical / FDM print parameters ─────────────────────────────────────────
WALL_S       = 0.30
WALL_P       = 0.20
WALL_F       = 0.15
WALL_THIN    = 0.12
CHAMFER_D    = 0.10
SUPPORT_ANG  = 45.0
LAYER_H      = 0.02
NOZZLE_D     = 0.04

# ── Fastener dimensions (cm) ────────────────────────────────────────────────
M3_CLR_R     = 0.160
M3_PILOT_R   = 0.125
M3_HEAD_R    = 0.285
M3_HEAD_H    = 0.300
M3_NUT_CIR   = 0.320
M3_NUT_H     = 0.240
M3_WASHER_OD = 0.350
M3_WASHER_ID = 0.165
INSERT_R     = 0.235
INSERT_H     = 0.500
BOSS_R       = 0.350
ALIGN_PIN_R  = 0.100
ALIGN_SOCK_R = 0.115
GROMMET_R    = 0.175
GROMMET_SLOT = 0.25

SCREW_LENGTHS = {
    "M2x6": 0.60, "M2x8": 0.80, "M2x10": 1.00,
    "M2.5x6": 0.60, "M2.5x8": 0.80, "M2.5x10": 1.00,
    "M3x6": 0.60, "M3x8": 0.80, "M3x10": 1.00, "M3x12": 1.20,
    "M3x14": 1.40, "M3x16": 1.60, "M3x18": 1.80, "M3x20": 2.00,
    "M3x25": 2.50, "M3x30": 3.00,
}
SCREW_GRIP_RATIO = 0.65

# ── Bearing parameters ───────────────────────────────────────────────────────
BEARING_PRESS   = 0.002
BEARING_GLUE    = 0.015
BEARING_RET_LIP = 0.025

# ── Servo horn / coupler parameters ─────────────────────────────────────────
COUPLER_HUB_RADIUS_MG996R = 0.45
COUPLER_SPLINE_RADIUS     = 0.38
COUPLER_LENGTH            = 1.0
SETSCREW_POCKET_R         = 0.14

SERVO_COUPLER = {
    "DS3225MG": {"spline_r": 0.30, "hub_r": 0.70, "hub_h": 0.60, "set_screw": True, "bore_r": 0.16, "flange_od": 1.60},
    "DS3218":   {"spline_r": 0.28, "hub_r": 0.65, "hub_h": 0.55, "set_screw": True, "bore_r": 0.16, "flange_od": 1.50},
    "MG996R":   {"spline_r": 0.25, "hub_r": 0.60, "hub_h": 0.50, "set_screw": True, "bore_r": 0.16, "flange_od": 1.40},
    "MG90S":    {"spline_r": 0.18, "hub_r": 0.45, "hub_h": 0.40, "set_screw": False, "bore_r": 0.12, "flange_od": 1.00},
    "DS04-NFC": {"spline_r": 0.15, "hub_r": 0.40, "hub_h": 0.35, "set_screw": False, "bore_r": 0.10, "flange_od": 0.85},
}

# ── Tendon system ───────────────────────────────────────────────────────────
TENDON_GROOVE_W = 0.15
TENDON_GROOVE_D = 0.12
TENDON_ANCHOR_R = 0.12
TENDON_DIA      = 0.04
IDLER_PULLEY_R  = 0.25
DRUM_RADIUS     = 0.35

# ── Electronics footprints (cm) ─────────────────────────────────────────────
RPI0_L,  RPI0_W,  RPI0_H  = 6.50, 3.00, 0.20
PCA_L,   PCA_W,   PCA_H   = 6.25, 2.54, 0.18
ESP32_L, ESP32_W, ESP32_H = 3.80, 2.58, 0.15
IMU_L,   IMU_W,   IMU_H   = 2.10, 1.60, 0.12
LIPO_L,  LIPO_W,  LIPO_H  = 7.00, 3.20, 1.80
LIPO_SLACK              = 0.30
XT60_W,  XT60_H_SLOT       = 1.60, 1.30
BEC_L,   BEC_W,   BEC_H   = 3.50, 2.20, 0.80
SWITCH_R                 = 0.60
FUSE_HOLDER_L, FUSE_HOLDER_W = 2.50, 1.20
LED_R_5MM                = 0.260
LED_R_RING               = 0.600

# ── Finger geometry (cm) ─────────────────────────────────────────────────────
FING_W       = 0.52
FING_H       = 0.48
FING_GAP     = 0.10
THUMB_W      = 0.65
THUMB_H      = 0.58
FING_PP      = [1.40, 1.60, 1.70, 1.55]
FING_MP      = [1.00, 1.20, 1.30, 1.15]
FING_DP      = [0.80, 0.90, 0.95, 0.88]
FING_NAMES   = ["Pinky", "Ring", "Middle", "Index"]
FING_X_OFF   = [-1.10, -0.37,  0.37,  1.10]
THUMB_PP_L   = 1.40
THUMB_DP_L   = 1.00
PALM_BOTTOM_OFFSET = 2.50

# ── Servo specs ──────────────────────────────────────────────────────────────
SERVO_SPECS = {
    "hip_hd": {"name": "DS3225MG", "rated": 25.0, "stall": 35.0, "mass_g":  60, "spline": 0.29},
    "waist":  {"name": "DS3218",   "rated": 20.0, "stall": 25.0, "mass_g":  55, "spline": 0.29},
    "std":    {"name": "MG996R",   "rated":  9.4, "stall": 11.5, "mass_g":  55, "spline": 0.29},
    "micro":  {"name": "MG90S",    "rated":  1.8, "stall":  2.2, "mass_g":  13, "spline": 0.24},
    "digit":  {"name": "DS04-NFC", "rated":  1.8, "stall":  2.2, "mass_g":   9, "spline": 0.24},
}

# ── ROM limits (deg) ─────────────────────────────────────────────────────────
JOINT_LIMITS = {
    "Waist_Cluster":      {"pitch": (-45,  60), "yaw": (-15,  15), "roll": (-15,  15)},
    "Neck_Cluster":       {"pitch": (-90,  45), "yaw": (-20,  20), "roll": (-20,  20)},
    "L_Hip_Cluster":      {"pitch": (-30,  30), "yaw": (-95,  95), "roll": (-30,  30)},
    "R_Hip_Cluster":      {"pitch": (-30,  30), "yaw": (-95,  95), "roll": (-30,  30)},
    "L_Knee":             {"pitch": (  0, 135)},
    "R_Knee":             {"pitch": (  0, 135)},
    "L_Ankle_Cluster":    {"pitch": (-20,  20), "yaw": (-30,  95), "roll": (-20,  20)},
    "R_Ankle_Cluster":    {"pitch": (-20,  20), "yaw": (-30,  95), "roll": (-20,  20)},
    "L_Shoulder_Cluster": {"pitch": (-175, 60), "yaw": (-90,  90), "roll": (-90,  90)},
    "R_Shoulder_Cluster": {"pitch": (-175, 60), "yaw": (-90,  90), "roll": (-90,  90)},
    "L_Elbow":            {"pitch": (  0, 150)},
    "R_Elbow":            {"pitch": (  0, 150)},
    "L_Wrist":            {"pitch": (  0,  90), "roll": (-180, 180)},
    "R_Wrist":            {"pitch": (  0, 135), "roll": (-180, 180)},
    "Blaster_Fold":       {"pitch": (-90,   0)},
    "L_Pinky_MCP":  {"pitch": (-5, 85)}, "R_Pinky_MCP":  {"pitch": (-5, 85)},
    "L_Ring_MCP":   {"pitch": (-5, 85)}, "R_Ring_MCP":   {"pitch": (-5, 85)},
    "L_Middle_MCP": {"pitch": (-5, 85)}, "R_Middle_MCP": {"pitch": (-5, 85)},
    "L_Index_MCP":  {"pitch": (-5, 85)}, "R_Index_MCP":  {"pitch": (-5, 85)},
    "L_Thumb_CMC":  {"pitch": (-20, 60), "yaw": (-30, 30)},
    "R_Thumb_CMC":  {"pitch": (-20, 60), "yaw": (-30, 30)},
}

SPLIT_KEYS = {"Shell", "Link", "Main", "Armor", "Core", "Pod", "Palm",
              "Block", "Sole", "Plate", "Bay", "Collar", "Cover", "Door",
              "Frame", "Skeleton", "Bracket"}
SKIP_TAGS  = {"Marker", "Pivot", "MtA", "MtB", "Axle_Pivot", "Horn",
              "Pin", "_Vis", "Boss", "Insert", "Nut", "Snap", "Coupler",
              "Tendon", "Pulley", "CableClip", "Stop", "AlignJig", "Clip",
              "Cutter", "Temp", "Groove", "Channel", "Wire"}
MERGE_TAGS = {"Pin", "Boss", "Insert", "Nut", "Snap", "Standoff", "Retainer"}

# ── Advanced mechanical constants ────────────────────────────────────────────
SHAFT_DIAM_MAJOR, SHAFT_DIAM_MED, SHAFT_DIAM_MINOR, SHAFT_DIAM_PIN = 0.50, 0.40, 0.30, 0.20
FLANGE_BRG_MAJOR_ID, FLANGE_BRG_MAJOR_OD, FLANGE_BRG_MAJOR_W = 0.50, 1.00, 0.40
FLANGE_BRG_MED_ID, FLANGE_BRG_MED_OD, FLANGE_BRG_MED_W = 0.40, 0.80, 0.32
FLANGE_BRG_MINOR_ID, FLANGE_BRG_MINOR_OD, FLANGE_BRG_MINOR_W = 0.30, 0.60, 0.26
FLANGE_BRG_FLANGE_OD, FLANGE_BRG_FLANGE_T = 1.20, 0.08
BEARING_BLOCK_W, BEARING_BLOCK_H = 0.80, 0.80
SET_SCREW_R, SET_SCREW_L = 0.145, 0.40
EXTRUSION_W, EXTRUSION_SLOT_W, EXTRUSION_SLOT_D = 2.00, 0.60, 0.30
EXTRUSION_T_SLOT_R, EXTRUSION_CENTER_BORE = 0.35, 0.30
FRAME_BRACKET_T, FRAME_BRACKET_W = 0.30, 1.20
WORM_MODULE, WORM_START, WORM_ADDENDUM, WORM_DEDDENDUM = 0.10, 1, 0.10, 0.12
GEAR_RATIO_WAIST, GEAR_RATIO_HIP, GEAR_RATIO_KNEE = 30, 4, 3
PLANET_STAGE_H, PLANET_STAGE_R, PLANET_SUN_R, PLANET_GEAR_R = 1.20, 1.40, 0.30, 0.45
LATCH_SOLENOID_D, LATCH_SOLENOID_L = 1.20, 3.00
LATCH_HOOK_L, LATCH_HOOK_D, LATCH_POST_D, LATCH_POST_H = 1.20, 0.30, 0.40, 1.00
ALIGN_CAM_H, ALIGN_CAM_W, ALIGN_CAM_ANGLE = 0.60, 0.80, 20
QR_PIN_D, QR_COLLAR_D, QR_COLLAR_L, QR_DETENT_BALL_R = 0.60, 1.40, 0.80, 0.125
QR_SPRING_D, QR_SPRING_L = 0.30, 0.60
SPINE_SEG_W, SPINE_SEG_H, SPINE_SEG_D = 2.00, 0.80, 1.50
SPINE_NERVE_CH_D, SPINE_BALL_D, SPINE_SOCKET_D = 0.80, 0.50, 0.55
BELLOWS_WALL, BELLOWS_RIB_H = 0.08, 0.15


# ═════════════════════════════════════════════════════════════════════════════
# LOGGER
# ═════════════════════════════════════════════════════════════════════════════
class Logger:
    _buffer = []
    _count  = 0

    @classmethod
    def log(cls, msg, level="INFO"):
        ts = datetime.datetime.now().strftime("%H:%M:%S")
        line = f"[{ts}] [{level}] {msg}\n"
        safe = line.encode("ascii", "replace").decode("ascii")
        print(safe, end="", flush=True)
        cls._buffer.append(line)
        cls._count += 1
        if cls._count >= 20:
            cls.flush()

    @classmethod
    def flush(cls):
        if not cls._buffer: return
        try:
            os.makedirs(LOG_DIR, exist_ok=True)
            with open(LOG_FILE, "a", encoding="utf-8") as f:
                f.write("".join(cls._buffer))
        except Exception: pass
        cls._buffer.clear()
        cls._count = 0

    @classmethod
    def print_note(cls, msg):
        cls.log(msg, "NOTE")


# ═════════════════════════════════════════════════════════════════════════════
# BOM TRACKER  (with aggregation from gem.py)
# ═════════════════════════════════════════════════════════════════════════════
class BOM:
    _rows = []
    _fasteners = []

    @classmethod
    def add(cls, category, part_name, qty, note=""):
        cls._rows.append({"Category": category, "Part": part_name, "Qty": qty, "Note": note})

    @classmethod
    def register_fastener(cls, f_type, length, needed_grip, note):
        cls._fasteners.append({"Type": f_type, "Len": length, "Grip": needed_grip, "Loc": note})

    @classmethod
    def save_csv(cls, path):
        if not cls._rows: return
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, "w", newline="", encoding="utf-8") as f:
                writer = csv.DictWriter(f, fieldnames=["Category", "Part", "Qty", "Note"])
                writer.writeheader()
                agg = {}
                for r in cls._rows:
                    k = (r["Category"], r["Part"])
                    if k not in agg:
                        agg[k] = {"Qty": 0, "Note": set()}
                    agg[k]["Qty"] += r["Qty"]
                    if r["Note"]: agg[k]["Note"].add(r["Note"])
                for (cat, part), data in agg.items():
                    notes = " | ".join(data["Note"])
                    writer.writerow({"Category": cat, "Part": part, "Qty": data["Qty"], "Note": notes})
            Logger.log(f"BOM saved -> {path}")
        except Exception as e:
            Logger.log(f"BOM save failed: {e}", "WARN")

    @classmethod
    def summary(cls):
        total = sum(r["Qty"] for r in cls._rows)
        Logger.log(f"BOM total line items: {len(cls._rows)}  total parts: {total}")

    @classmethod
    def by_category(cls):
        cats = {}
        for r in cls._rows:
            cats.setdefault(r["Category"], []).append(r)
        return cats

    @classmethod
    def check_fastener_lengths(cls):
        Logger.log("--- FASTENER VERIFICATION ---")
        for f in cls._fasteners:
            if f["Len"] < f["Grip"] + 0.2:
                Logger.log(f"WARNING: {f['Type']}x{f['Len']*10} at {f['Loc']} too short (needs {f['Grip']*10}mm)", "WARN")
            elif f["Len"] > f["Grip"] + 0.6:
                Logger.log(f"NOTE: {f['Type']}x{f['Len']*10} at {f['Loc']} longer than grip ({f['Grip']*10}mm)")


# ═════════════════════════════════════════════════════════════════════════════
# BUILD GUIDE  (from qwen.py — text-based assembly guide)
# ═════════════════════════════════════════════════════════════════════════════
class BuildGuide:
    _entries = []

    @classmethod
    def add(cls, section, content):
        cls._entries.append((section, content))

    @classmethod
    def generate(cls):
        lines = []
        lines.append("=" * 65)
        lines.append("OPTIMUS PRIME G1 v12 — PHYSICAL BUILD ASSEMBLY GUIDE")
        lines.append(f"Generated: {datetime.datetime.now().strftime('%Y-%m-%d %H:%M')}")
        lines.append("=" * 65)
        lines.append("")
        lines.append("--- PRINTED PART SUMMARY ---")
        for p in BOM.by_category().get("Printed Part", []):
            lines.append(f"  {p['Qty']}x {p['Part']:35s} {p['Note']}")
        lines.append("")
        lines.append("--- ASSEMBLY ORDER ---")
        for section, content in cls._entries:
            lines.append(f"\n  [{section}]")
            for line in content.split("\n"):
                lines.append(f"    {line.strip()}")
        return "\n".join(lines)

    @classmethod
    def save(cls, path):
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, "w", encoding="utf-8") as f:
                f.write(cls.generate())
            Logger.log(f"Build guide saved -> {path}")
        except Exception as e:
            Logger.log(f"Build guide save failed: {e}", "WARN")


# ═════════════════════════════════════════════════════════════════════════════
# FASTENER VALIDATOR  (from qwen.py — screw length vs grip validation)
# ═════════════════════════════════════════════════════════════════════════════
class FastenerValidator:
    _warnings = []

    @classmethod
    def clear(cls):
        cls._warnings = []

    @classmethod
    def check(cls, screw_name, grip_length, nut_depth, material_thickness, location=""):
        if screw_name not in SCREW_LENGTHS:
            cls._warnings.append(f"Unknown screw: {screw_name} at {location}")
            return False
        total_len = SCREW_LENGTHS[screw_name]
        screw_d = float(screw_name.split("x")[0].replace("M", "")) / 10.0
        min_grip = screw_d * SCREW_GRIP_RATIO
        available_grip = total_len - material_thickness - 0.05
        if available_grip < min_grip:
            cls._warnings.append(f"  [{screw_name} @ {location}] TOO SHORT")
            return False
        if available_grip > nut_depth + 0.10:
            cls._warnings.append(f"  [{screw_name} @ {location}] MAY BOTTOM OUT")
            return False
        return True

    @classmethod
    def report(cls):
        for w in cls._warnings:
            Logger.log(w, "WARN")
        if not cls._warnings:
            Logger.log("Fastener validation: [OK] All lengths acceptable")


# ═════════════════════════════════════════════════════════════════════════════
# BUILD DOCS  (from kimi.py — Markdown build documentation)
# ═════════════════════════════════════════════════════════════════════════════
PRINT_NOTES = []
PRINT_WARNINGS = []

class BuildDocs:
    _sections = []
    _fastener_map = []
    _print_orient = {}

    @classmethod
    def add_section(cls, title, content):
        cls._sections.append((title, content))

    @classmethod
    def add_fastener(cls, location, screw_spec, engagement_mm, status):
        cls._fastener_map.append({
            "Location": location, "Spec": screw_spec,
            "Engagement": engagement_mm, "Status": status
        })

    @classmethod
    def set_print_orientation(cls, part_name, orientation, supports, material):
        cls._print_orient[part_name] = {
            "orientation": orientation,
            "supports": supports,
            "material": material
        }

    @classmethod
    def generate(cls, path):
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, "w", encoding="utf-8") as f:
                f.write("# Optimus Prime G1 v12.0 — Build & Assembly Guide\n")
                f.write(f"Generated: {datetime.datetime.now().strftime('%Y-%m-%d %H:%M')}\n\n")
                f.write("## Critical Build Warnings\n\n")
                if PRINT_WARNINGS:
                    for w in PRINT_WARNINGS:
                        f.write(f"- **{w}**\n")
                else:
                    f.write("- No critical warnings.\n")
                f.write("\n")
                f.write("## Print Orientation Guide\n\n")
                f.write("| Part | Orientation | Supports | Material |\n")
                f.write("|------|-------------|----------|----------|\n")
                for part, info in cls._print_orient.items():
                    f.write(f"| {part} | {info['orientation']} | {info['supports']} | {info['material']} |\n")
                f.write("\n")
                f.write("## Assembly Order\n\n")
                f.write("1. **Print all parts** — verify tolerances on test pieces first\n")
                f.write("2. **Install heat-set inserts** — use soldering iron at 200°C\n")
                f.write("3. **Press bearings** — use vise or arbor press, check fit\n")
                f.write("4. **Install servos** — verify horn alignment at neutral\n")
                f.write("5. **Route wiring** — use cable clips, avoid pinching\n")
                f.write("6. **Assemble legs** — hip to thigh to knee to shin to ankle to foot\n")
                f.write("7. **Assemble arms** — shoulder to upper arm to elbow to forearm to hand\n")
                f.write("8. **Assemble torso** — install electronics before closing shell\n")
                f.write("9. **Mount head** — connect neck servos and camera cable\n")
                f.write("10. **System test** — power on, verify all joints move freely\n")
                f.write("11. **Install covers** — battery door last for access\n\n")
                f.write("## Fastener Verification\n\n")
                f.write("| Location | Screw Spec | Engagement (mm) | Status |\n")
                f.write("|----------|------------|-----------------|--------|\n")
                for fm in cls._fastener_map:
                    icon = "OK" if fm["Status"] == "OK" else "WARN"
                    f.write(f"| {fm['Location']} | {fm['Spec']} | {fm['Engagement']:.1f} | {icon} |\n")
                f.write("\n")
                for title, content in cls._sections:
                    f.write(f"## {title}\n\n{content}\n\n")
                f.write("---\n*End of build guide. Refer to BOM CSV for hardware quantities.*\n")
            Logger.log(f"Build guide (md) -> {path}")
        except Exception as e:
            Logger.log(f"Build guide (md) failed: {e}", "WARN")


# ═════════════════════════════════════════════════════════════════════════════
# PRINTABILITY HELPERS  (from kimi.py)
# ═════════════════════════════════════════════════════════════════════════════
def add_print_orientation_note(part_name, orientation, support_req):
    Logger.print_note(f"PRINT: {part_name} — orient {orientation}, support: {support_req}")

def printability_review(comp, body_name, overhang_angle):
    if overhang_angle > SUPPORT_ANG:
        msg = f"{comp.name}/{body_name}: overhang {overhang_angle:.0f} exceeds {SUPPORT_ANG:.0f} — supports recommended"
        Logger.log(f"  PRINT WARNING: {msg}", "WARN")
        PRINT_WARNINGS.append(msg)
        BuildDocs.set_print_orientation(body_name, "reorient or add supports", "required", "PLA/PETG")

def fastener_verify(location, screw_dia_mm, screw_len_mm, material_thickness_mm, nut_trap_depth_mm=0, min_engagement=4.0):
    engagement = screw_len_mm - material_thickness_mm
    if nut_trap_depth_mm > 0:
        engagement = min(engagement, nut_trap_depth_mm)
    status = "OK"
    if engagement < min_engagement:
        status = "SHORT"
        Logger.log(f"  FASTENER WARNING: {location} engagement {engagement:.1f}mm < {min_engagement}mm", "WARN")
    elif engagement > screw_len_mm * 0.9:
        status = "LONG"
        Logger.log(f"  FASTENER WARNING: {location} may bottom out", "WARN")
    spec = f"M{screw_dia_mm:.1f}x{screw_len_mm:.0f}"
    BuildDocs.add_fastener(location, spec, engagement, status)
    return status


# ═════════════════════════════════════════════════════════════════════════════
# MAIN ENTRY POINT
# ═════════════════════════════════════════════════════════════════════════════
def run(context):
    global TARGET_MODULE, EXPORT_STL, EXPORT_STEP, EXPORT_URDF, EXPORT_DIR

    app = ui = None
    Logger.log("=" * 65)
    Logger.log("EXECUTION START  v12.0 — Optimus Prime G1 Ultimate Physical Build")
    Logger.log("=" * 65)

    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        try:
            app.preferences.generalPreferences.defaultModelingOrientation = \
                adsk.core.DefaultModelingOrientations.ZUpModelingOrientation
        except Exception: pass

        doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
        design = adsk.fusion.Design.cast(app.activeProduct)
        root = design.rootComponent

        # ── APPEARANCES ───────────────────────────────────────────────────
        app_lib = None
        for i in range(app.materialLibraries.count):
            lib = app.materialLibraries.item(i)
            if "Appearance" in lib.name:
                app_lib = lib; break

        def _copy_ap(query):
            if not app_lib: return None
            for i in range(app_lib.appearances.count):
                ap = app_lib.appearances.item(i)
                if query.lower() in ap.name.lower():
                    try: return design.appearances.addByCopy(ap)
                    except: return ap
            return None

        def get_ap(primary, *fallbacks):
            ap = _copy_ap(primary)
            if ap: return ap
            for fb in fallbacks:
                ap = _copy_ap(fb)
                if ap: return ap
            return None

        op_red        = get_ap("Paint - Metallic (Red)",       "Steel - Painted (Red)")
        op_blue       = get_ap("Paint - Metallic (Blue)",      "Steel - Painted (Blue)")
        chrome        = get_ap("Chrome",                        "Steel - Polished")
        dark_metal    = get_ap("Steel - Flat",                  "Plastic - Matte (Black)")
        rubber_blk    = get_ap("Rubber",                        "Plastic - Matte (Black)")
        glass_clr     = get_ap("Glass - Window",                "Acrylic - Clear")
        grey_plastic  = get_ap("Plastic - Matte (Grey)",        "ABS Plastic")
        dark_grey     = get_ap("Plastic - Matte (Dark Grey)",   "Plastic - Matte (Grey)")
        white_pla     = get_ap("Plastic - Glossy (White)",      "Nylon - White")
        black_plastic = get_ap("Plastic - Matte (Black)",       "Rubber")
        gold_met      = get_ap("Gold",                          "Brass")
        yellow_met    = get_ap("Paint - Metallic (Yellow)",     "Gold")
        op_blue_glass = get_ap("Acrylic - Blue Transparent",    "Glass - Window")
        orange_pla    = get_ap("Plastic - Glossy (Orange)",     "Nylon - White")
        trans_orange  = get_ap("Acrylic - Orange Transparent",  "Glass - Window")

        # ── COMPONENT REGISTRY & PRIMITIVES ──────────────────────────────
        comps_list = []
        occs = {}
        jig_list = []

        def new_component(name):
            occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
            comp = occ.component; comp.name = name
            comps_list.append(comp); occs[name] = occ; return comp

        def new_jig(name):
            occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
            comp = occ.component; comp.name = f"JIG_{name}"
            jig_list.append(comp); return comp

        def set_ap(body, ap):
            if body and ap:
                try: body.appearance = ap
                except: pass

        def box(comp, name, cx, cy, cz, lx, ly, lz, ap=None):
            temp = adsk.fusion.TemporaryBRepManager.get()
            obb = adsk.core.OrientedBoundingBox3D.create(
                adsk.core.Point3D.create(cx, cy, cz),
                adsk.core.Vector3D.create(1, 0, 0),
                adsk.core.Vector3D.create(0, 1, 0), lx, ly, lz)
            shape = temp.createBox(obb)
            bf = comp.features.baseFeatures.add(); bf.startEdit()
            body = comp.bRepBodies.add(shape, bf); bf.finishEdit()
            body.name = name; set_ap(body, ap); return body

        def cyl(comp, name, cx, cy, cz, r, h, axis, ap=None):
            temp = adsk.fusion.TemporaryBRepManager.get()
            ax = {"x": (1,0,0), "y": (0,1,0), "z": (0,0,1)}[axis]
            p1 = adsk.core.Point3D.create(cx-ax[0]*h/2, cy-ax[1]*h/2, cz-ax[2]*h/2)
            p2 = adsk.core.Point3D.create(cx+ax[0]*h/2, cy+ax[1]*h/2, cz+ax[2]*h/2)
            shape = temp.createCylinderOrCone(p1, r, p2, r)
            bf = comp.features.baseFeatures.add(); bf.startEdit()
            body = comp.bRepBodies.add(shape, bf); bf.finishEdit()
            body.name = name; set_ap(body, ap); return body

        def cone_shape(comp, name, cx, cy, cz, r1, r2, h, axis, ap=None):
            temp = adsk.fusion.TemporaryBRepManager.get()
            ax = {"x": (1,0,0), "y": (0,1,0), "z": (0,0,1)}[axis]
            p1 = adsk.core.Point3D.create(cx-ax[0]*h/2, cy-ax[1]*h/2, cz-ax[2]*h/2)
            p2 = adsk.core.Point3D.create(cx+ax[0]*h/2, cy+ax[1]*h/2, cz+ax[2]*h/2)
            shape = temp.createCylinderOrCone(p1, r1, p2, r2)
            bf = comp.features.baseFeatures.add(); bf.startEdit()
            body = comp.bRepBodies.add(shape, bf); bf.finishEdit()
            body.name = name; set_ap(body, ap); return body

        def marker(comp, name, cx, cy, cz, size=0.22):
            return box(comp, name, cx, cy, cz, size, size, size, white_pla)

        # ── Boolean cut / merge / split helpers ──────────────────────────
        def cut_cavity(comp, tool_body, keep_tool=False):
            tools = adsk.core.ObjectCollection.create()
            tools.add(tool_body)
            for b in list(comp.bRepBodies):
                if b == tool_body: continue
                if b.name and any(t in b.name for t in SKIP_TAGS): continue
                try:
                    ci = comp.features.combineFeatures.createInput(b, tools)
                    ci.operation = adsk.fusion.CombineOperation.CutFeatureOperation
                    ci.isKeepToolBodies = True
                    comp.features.combineFeatures.add(ci)
                except: pass
            if not keep_tool:
                try:
                    tool_body.isLightBulbOn = False
                    if "_Vis" not in tool_body.name: tool_body.name += "_Vis"
                except: pass

        def merge_bodies(comp, target_body, source_body):
            try:
                tools = adsk.core.ObjectCollection.create(); tools.add(source_body)
                ci = comp.features.combineFeatures.createInput(target_body, tools)
                ci.operation = adsk.fusion.CombineOperation.JoinFeatureOperation
                ci.isKeepToolBodies = False
                comp.features.combineFeatures.add(ci); return target_body
            except:
                try: source_body.isLightBulbOn = False
                except: pass
            return target_body

        def split_halves(comp, body, axis="y", offset=0.0):
            try:
                planes = comp.constructionPlanes; pi = planes.createInput()
                ref = (root.xYConstructionPlane if axis == "z" else
                       root.xZConstructionPlane if axis == "y" else root.yZConstructionPlane)
                pi.setByOffset(ref, adsk.core.ValueInput.createByReal(offset))
                sp = planes.add(pi)
                split_input = comp.features.splitBodyFeatures.createInput(body, sp, True)
                comp.features.splitBodyFeatures.add(split_input)
            except: pass

        def merge_fasteners_to_halves(comp, axis="y", offset=0.0):
            def _y(b):
                try: return b.boundingBox.centerPoint.y
                except: return offset
            shells = [b for b in comp.bRepBodies if b.name and any(k in b.name for k in SPLIT_KEYS)]
            left_shells = [b for b in shells if _y(b) < offset]
            right_shells = [b for b in shells if _y(b) >= offset]
            fasteners = [b for b in comp.bRepBodies if b.name and any(f in b.name for f in MERGE_TAGS)]
            if fasteners and left_shells:
                tools = adsk.core.ObjectCollection.create()
                for f in fasteners:
                    if _y(f) < offset: tools.add(f)
                if tools.count > 0:
                    try:
                        ci = comp.features.combineFeatures.createInput(left_shells[0], tools)
                        ci.operation = adsk.fusion.CombineOperation.JoinFeatureOperation
                        ci.isKeepToolBodies = False
                        comp.features.combineFeatures.add(ci)
                    except: pass
            if fasteners and right_shells:
                tools = adsk.core.ObjectCollection.create()
                for f in fasteners:
                    if _y(f) >= offset: tools.add(f)
                if tools.count > 0:
                    try:
                        ci = comp.features.combineFeatures.createInput(right_shells[0], tools)
                        ci.operation = adsk.fusion.CombineOperation.JoinFeatureOperation
                        ci.isKeepToolBodies = False
                        comp.features.combineFeatures.add(ci)
                    except: pass

        # ── FASTENER HELPERS ─────────────────────────────────────────────
        def m3_boss(comp, tag, cx, cy, cz, axis="z", depth=0.80):
            boss = cyl(comp, f"{tag}_Boss", cx, cy, cz, BOSS_R, depth, axis, dark_metal)
            boss.name = f"{tag}_Boss"
            cut_cavity(comp, cyl(comp, f"{tag}_Insert", cx, cy, cz, INSERT_R, INSERT_H, axis))
            BOM.add("Fastener", "M3 heat-set insert (Voron M3x5)", 1, f"boss at ({cx:.1f},{cy:.1f},{cz:.1f}) in {comp.name}")

        def captive_nut(comp, tag, cx, cy, cz, axis="z", bolt_len=1.0):
            cut_cavity(comp, cyl(comp, f"{tag}_NutTrap", cx, cy, cz, M3_NUT_CIR, M3_NUT_H, axis))
            ax = {"x": (1,0,0), "y": (0,1,0), "z": (0,0,1)}[axis]
            cut_cavity(comp, cyl(comp, f"{tag}_NutBore", cx, cy, cz, M3_CLR_R, bolt_len, axis))
            BOM.add("Fastener", "M3 hex nut", 1, comp.name)
            BOM.add("Fastener", f"M3x{int(bolt_len*10)} SHCS", 1, comp.name)
            BOM.register_fastener("M3", bolt_len, bolt_len - 0.2, f"captive_nut {tag}")

        def snap_clip(comp, tag, cx, cy, cz, span_x=1.2, axis_latch="z"):
            for sign in [-1, 1]:
                arm_cx = cx + sign * span_x * 0.5
                box(comp, f"{tag}_SnapArm_{sign}", arm_cx, cy, cz, WALL_F, 0.40, 1.20, grey_plastic)
                box(comp, f"{tag}_SnapHead_{sign}", arm_cx, cy, cz + 0.55, WALL_F + 0.10, 0.50, 0.28, grey_plastic)

        def align_pin(comp, tag, cx, cy, cz, axis="z", height=0.40):
            cyl(comp, f"{tag}_AlignPin", cx, cy, cz, ALIGN_PIN_R, height, axis, chrome)

        def align_socket(comp, tag, cx, cy, cz, axis="z", depth=0.45):
            cut_cavity(comp, cyl(comp, f"{tag}_AlignSock", cx, cy, cz, ALIGN_SOCK_R, depth, axis))

        def grommet_hole(comp, tag, cx, cy, cz, axis="y"):
            cut_cavity(comp, cyl(comp, f"{tag}_Grommet", cx, cy, cz, GROMMET_R, 0.80, axis))
            BOM.add("Hardware", "Wire grommet 3.5mm", 1, comp.name)

        def screw_hole(comp, cx, cy, cz, axis="y", length=3.0):
            cut_cavity(comp, cyl(comp, "ScrewHole", cx, cy, cz, M3_PILOT_R, length, axis))
            BOM.add("Fastener", f"M3x{int(length*10)} self-tap", 1, comp.name)

        def magnet_pocket(comp, tag, cx, cy, cz, axis="z"):
            cut_cavity(comp, cyl(comp, f"{tag}_MagPkt", cx, cy, cz, 0.32 + FDM_TOL, 0.35, axis))
            BOM.add("Hardware", "Magnet D6x3 mm N35", 1, comp.name)

        def wire_channel(comp, tag, cx, cy, cz, r, h, axis):
            cut_cavity(comp, cyl(comp, f"{tag}_Wire", cx, cy, cz, r, h, axis))

        def led_pocket_5mm(comp, tag, cx, cy, cz, axis="y"):
            cut_cavity(comp, cyl(comp, f"{tag}_LED5", cx, cy, cz, LED_R_5MM, 0.85, axis))
            BOM.add("Electronics", "LED 5 mm (colour TBD)", 1, comp.name)

        def led_ring_pocket(comp, tag, cx, cy, cz, axis="z"):
            cut_cavity(comp, cyl(comp, f"{tag}_LEDRing", cx, cy, cz, LED_R_RING, 0.30, axis))
            BOM.add("Electronics", "WS2812 5050 LED ring 12 mm", 1, comp.name)

        # ── PHYSICAL FEATURE HELPERS (v11 merge) ─────────────────────────
        def horn_coupler(comp, tag, cx, cy, cz, axis, servo_type="MG996R"):
            spec = SERVO_COUPLER.get(servo_type, SERVO_COUPLER["MG996R"])
            hr, hh, sr, br = spec["hub_r"], spec["hub_h"], spec["spline_r"], spec["bore_r"]
            hub = cyl(comp, f"{tag}_CouplerHub", cx, cy, cz, hr, hh, axis, chrome)
            hub.name = f"{tag}_CouplerHub"
            cut_cavity(comp, cyl(comp, f"{tag}_SplineCut", cx, cy, cz, sr + 0.02, hh + 0.05, axis))
            if spec["set_screw"]:
                ss_r, ss_len = br * 0.9, hr * 1.2
                if axis == "x": ss_body = cyl(comp, f"{tag}_SetScrew", cx, cy+hr*0.5, cz, ss_r, ss_len, "y")
                elif axis == "y": ss_body = cyl(comp, f"{tag}_SetScrew", cx+hr*0.5, cy, cz, ss_r, ss_len, "x")
                else: ss_body = cyl(comp, f"{tag}_SetScrew", cx, cy+hr*0.5, cz, ss_r, ss_len, "y")
                cut_cavity(comp, cyl(comp, f"{tag}_Bore", cx, cy, cz, br+0.01, hh+0.10, axis))
                BOM.add("Fastener", "M3x4 set screw (cup point)", 1, f"coupler {tag}")
            BOM.add("Printed", f"Coupler {servo_type} ({tag})", 1, f"PETG")

        def tendon_groove(comp, tag, cx, cy, cz, length, axis="z"):
            cut_cavity(comp, box(comp, f"{tag}_TendonGroove", cx, cy, cz,
                                 TENDON_GROOVE_W if axis != "x" else length,
                                 TENDON_GROOVE_W if axis != "y" else length,
                                 TENDON_GROOVE_W if axis != "z" else length))

        def tendon_anchor(comp, tag, cx, cy, cz, axis="z"):
            cyl(comp, f"{tag}_Anchor", cx, cy, cz, TENDON_ANCHOR_R, 0.2, axis, chrome)
            BOM.add("Hardware", "Tendon: 0.4 mm Dyneema", 30, "cm per finger")

        def idler_pulley(comp, tag, cx, cy, cz, axis="x"):
            cyl(comp, f"{tag}_Idler", cx, cy, cz, IDLER_PULLEY_R, 0.5, axis, dark_grey)

        def servo_drum(comp, tag, cx, cy, cz, axis="x"):
            cyl(comp, f"{tag}_Drum", cx, cy, cz, DRUM_RADIUS, 0.8, axis, grey_plastic)
            BOM.add("Printed", "Servo tendon drum", 1, f"{tag} in {comp.name}")

        def cover_plate(comp, tag, cx, cy, cz, lx, ly, lz, axis="y",
                         boss_positions=None, magnet_positions=None, snap_positions=None):
            cover = box(comp, f"{tag}_Cover", cx, cy, cz, lx, ly, lz, dark_grey)
            if boss_positions:
                for (bx, bz) in boss_positions:
                    m3_boss(comp, f"{tag}CvrBoss_{bx}_{bz}", cx+bx, cy, cz+bz)
            if magnet_positions:
                for (mx, mz) in magnet_positions:
                    magnet_pocket(comp, f"{tag}CvrMag_{mx}_{mz}", cx+mx, cy, cz+mz)
            if snap_positions:
                for (sx, sz) in snap_positions:
                    snap_clip(comp, f"{tag}CvrSnap_{sx}_{sz}", cx+sx, cy, cz+sz)
            BOM.add("Printed", f"Access cover {tag}", 1, f"{comp.name}")

        def bearing_fit(comp, tag, cx, cy, cz, axis="x", ro=1.10, w=0.60, fit_type="press"):
            tol = -BEARING_PRESS if fit_type == "press" else (BEARING_GLUE if fit_type == "glue" else 0)
            cavity_r = ro + tol
            cyl(comp, f"{tag}_BO", cx, cy, cz, ro, w, axis, chrome)
            cyl(comp, f"{tag}_BI", cx, cy, cz, ro*0.58, w*0.80, axis, dark_grey)
            cyl(comp, f"{tag}_BB", cx, cy, cz, ro*0.32, w*1.10, axis, chrome)
            temp = adsk.fusion.TemporaryBRepManager.get()
            ax = {"x": (1,0,0), "y": (0,1,0), "z": (0,0,1)}[axis]
            half = w/2.0 + 0.05
            p1 = adsk.core.Point3D.create(cx-ax[0]*half, cy-ax[1]*half, cz-ax[2]*half)
            p2 = adsk.core.Point3D.create(cx+ax[0]*half, cy+ax[1]*half, cz+ax[2]*half)
            cs = temp.createCylinderOrCone(p1, cavity_r, p2, cavity_r)
            bf = comp.features.baseFeatures.add(); bf.startEdit()
            cb = comp.bRepBodies.add(cs, bf); bf.finishEdit()
            cb.name = f"{tag}_BearingCavity"; cut_cavity(comp, cb)
            if fit_type in ("lip", "press"):
                lip_w, lip_r = BEARING_RET_LIP, ro - 0.02
                lip_p1 = adsk.core.Point3D.create(cx-ax[0]*(half+0.01), cy-ax[1]*(half+0.01), cz-ax[2]*(half+0.01))
                lip_p2 = adsk.core.Point3D.create(cx-ax[0]*(half+0.01-lip_w), cy-ax[1]*(half+0.01-lip_w), cz-ax[2]*(half+0.01-lip_w))
                lip = temp.createCylinderOrCone(lip_p1, lip_r, lip_p2, lip_r)
                bf2 = comp.features.baseFeatures.add(); bf2.startEdit()
                lip_body = comp.bRepBodies.add(lip, bf2); bf2.finishEdit()
                lip_body.name = f"{tag}_Retainer"; cut_cavity(comp, lip_body)
            BOM.add("Bearing", f"Bearing {int(ro*2*10)}x{int(w*10)} mm", 1, f"{fit_type}-fit, {comp.name}")

        def bearing(comp, tag, cx, cy, cz, axis="x", ro=1.10, w=0.60):
            bearing_fit(comp, tag, cx, cy, cz, axis, ro, w, fit_type="press")

        def cable_clip(comp, tag, cx, cy, cz, axis="y"):
            clip_body = box(comp, f"{tag}_Clip", cx, cy, cz, 0.30, 0.40, 0.30, grey_plastic)
            ch = box(comp, f"{tag}_ClipCh", cx, cy, cz, 0.10, 0.35, 0.26)
            cut_cavity(comp, ch)
            BOM.add("Printed", "Cable clip", 1, f"{tag}")

        def wiring_hub(comp, tag, cx, cy, cz):
            hub_comp = new_component(f"{tag}_WiringHub")
            box(hub_comp, f"{tag}_HubBase", cx, cy, cz, 2.0, 1.0, 1.5, dark_grey)
            for wi in range(4):
                wx = cx + (wi - 1.5) * 0.5
                box(hub_comp, f"{tag}_Route_{wi}", wx, cy+0.3, cz, 0.30, 0.25, 0.50)
            BOM.add("Printed", "Wiring hub", 1, f"{comp.name}")

        def hard_stop(comp, tag, cx, cy, cz, axis, angle_deg):
            box(comp, f"{tag}_Stop", cx, cy, cz, 0.5, 0.5, 0.5, dark_metal)
            BOM.add("Printed", f"Mechanical stop {tag}", 1, f"{angle_deg:.0f} deg limit")

        def transformation_latch(comp, tag, cx, cy, cz, axis="z"):
            magnet_pocket(comp, f"{tag}_LatchMag", cx, cy, cz, axis)
            align_pin(comp, f"{tag}_LatchPin", cx, cy, cz, axis)

        def assembly_jig(comp, tag, cx, cy, cz, lx, ly, lz):
            box(comp, f"{tag}_Jig", cx, cy, cz, lx, ly, lz, white_pla)
            BOM.add("Printed", f"Assembly jig {tag}", 1, f"{comp.name}")

        def alignment_jig(name, cx, cy, cz, lx, ly, lz, pin_positions=None):
            jig = new_jig(f"Align_{name}")
            box(jig, f"Jig_{name}_Base", cx, cy, cz, lx, ly, lz, white_pla)
            if pin_positions:
                for pi, (px, py, pz) in enumerate(pin_positions):
                    cyl(jig, f"Pin_{pi}", px, py, pz, ALIGN_PIN_R, 0.50, "z", white_pla)
            add_print_orientation_note(f"Jig_{name}", "flat on build plate", "none")
            BOM.add("Printed", f"Assembly jig {name}", 1, f"PLA")
            return jig

        def wear_plate(comp, tag, cx, cy, cz, lx, ly, lz, ap=None):
            ap = ap or white_pla
            plate = box(comp, f"{tag}_WearPlate", cx, cy, cz, lx, ly, lz, ap)
            for wx in [cx - lx*0.3, cx + lx*0.3]:
                if abs(wx - cx) > 0.4: screw_hole(comp, wx, cy, cz, axis="z", length=0.60)
            BOM.add("Printed", f"Wear plate {tag}", 1, f"replaceable, {comp.name}")

        def lock_pin(comp, tag, cx, cy, cz, axis="z", ap=None):
            ap = ap or chrome
            cyl(comp, f"{tag}_LockPin", cx, cy, cz, 0.18, 0.80, axis, ap)
            cyl(comp, f"{tag}_LockPinHandle", cx, cy, cz, 0.30, 0.15, axis, op_red)
            BOM.add("Hardware", "Lock pin spring (3x10mm)", 1, comp.name)
            BOM.add("Printed", f"Lock pin housing {tag}", 1, f"PETG")

        def mechanical_stop(comp, tag, cx, cy, cz, axis, angle_deg, ap=None):
            ap = ap or dark_metal
            box(comp, f"{tag}_MechStop", cx, cy, cz, 0.30, 0.60, 0.30, ap)
            BOM.add("Printed", f"Mechanical stop {tag}", 1, f"{angle_deg:.0f} deg")

        def servo_mount_reinforcement(comp, tag, cx, cy, cz, lx, ly, lz):
            for ri in range(3):
                rx = cx + (ri - 1) * lx * 0.25
                box(comp, f"{tag}_Rib_{ri}", rx, cy, cz, 0.20, ly*1.1, 0.20, dark_metal)
            BOM.add("Printed", f"Servo mount reinforcement {tag}", 1, comp.name)

        # ── ELECTRONICS BAY HELPERS ──────────────────────────────────────
        def rpi_zero_bay(comp, tag, cx, cy, cz):
            cut_cavity(comp, box(comp, f"{tag}_RPiBay", cx, cy, cz, RPI0_L + 0.25, RPI0_W + 0.25, RPI0_H + 0.35))
            cut_cavity(comp, box(comp, f"{tag}_SDSlot", cx - RPI0_L*0.5 - 0.15, cy, cz + 0.5, 0.30, 0.80, 0.40))
            cut_cavity(comp, box(comp, f"{tag}_USBSlot", cx + RPI0_L*0.5 + 0.15, cy, cz - 0.3, 0.30, 1.20, 0.60))
            for sx, sz in [(-2.90, -1.15), (+2.90, -1.15), (-2.90, +1.15), (+2.90, +1.15)]:
                cyl(comp, f"{tag}_RpiStdoff_{sx}_{sz}", cx+sx, cy, cz+sz, 0.14, RPI0_H+0.55, "y", dark_metal)
            BOM.add("Electronics", "Raspberry Pi Zero 2W", 1, comp.name)
            BOM.add("Fastener", "M2.5x11 mm brass standoff", 4, comp.name)

        def pca9685_bay(comp, tag, cx, cy, cz):
            cut_cavity(comp, box(comp, f"{tag}_PCABay", cx, cy, cz, PCA_L + 0.20, PCA_W + 0.20, PCA_H + 0.30))
            for sx, sz in [(-3.00, -1.08), (+3.00, -1.08), (-3.00, +1.08), (+3.00, +1.08)]:
                cyl(comp, f"{tag}_PCAStdoff_{sx}_{sz}", cx+sx, cy, cz+sz, 0.14, PCA_H+0.50, "y", dark_metal)
            BOM.add("Electronics", "PCA9685 16-ch servo driver", 1, comp.name)

        def lipo_bay(comp, tag, cx, cy, cz):
            cut_cavity(comp, box(comp, f"{tag}_LipoBay", cx, cy, cz, LIPO_L + LIPO_SLACK, LIPO_H + LIPO_SLACK, LIPO_W + LIPO_SLACK))
            cut_cavity(comp, box(comp, f"{tag}_XT60Slot", cx, cy + LIPO_H/2 + 0.15, cz, XT60_W + 0.10, 0.50, XT60_H_SLOT + 0.10))
            BOM.add("Electronics", "2S 1300 mAh 7.4V LiPo battery", 1, comp.name)
            BOM.add("Electronics", "XT60-F panel-mount connector", 1, comp.name)

        def imu_pocket(comp, tag, cx, cy, cz, axis="z"):
            cut_cavity(comp, box(comp, f"{tag}_IMUPkt", cx, cy, cz, IMU_L + 0.20, IMU_W + 0.20, IMU_H + 0.30))
            BOM.add("Electronics", "MPU-6050 IMU breakout", 1, comp.name)

        def esp32_cam_pocket(comp, tag, cx, cy, cz, lens_axis="y"):
            cut_cavity(comp, box(comp, f"{tag}_ESP32Bay", cx, cy, cz, ESP32_L + 0.20, ESP32_H + 0.30, ESP32_W + 0.20))
            cut_cavity(comp, cyl(comp, f"{tag}_LensHole", cx, cy - (ESP32_H/2 + 0.30), cz, 0.150, 0.60, lens_axis))
            BOM.add("Electronics", "ESP32-CAM module (OV2640)", 1, comp.name)

        def bec_bay(comp, tag, cx, cy, cz):
            cut_cavity(comp, box(comp, f"{tag}_BECBay", cx, cy, cz, BEC_L + 0.30, BEC_W + 0.20, BEC_H + 0.20))
            BOM.add("Electronics", "5V 5A BEC regulator", 1, comp.name)

        def fuse_pocket(comp, tag, cx, cy, cz):
            cut_cavity(comp, box(comp, f"{tag}_FusePkt", cx, cy, cz, FUSE_HOLDER_L + 0.30, FUSE_HOLDER_W + 0.20, 0.60))
            BOM.add("Electronics", "ATO inline fuse holder + 10A fuse", 1, comp.name)

        def cutoff_switch_pocket(comp, tag, cx, cy, cz):
            cut_cavity(comp, cyl(comp, f"{tag}_SwitchHole", cx, cy, cz, SWITCH_R + 0.02, 0.80, "y"))
            BOM.add("Electronics", "SPST toggle switch 15A+ (emergency cutoff)", 1, comp.name)

        # ── JOINT BUILDERS ───────────────────────────────────────────────
        def _make_joint_geometry(cx, cy, cz):
            try:
                cpi = root.constructionPoints.createInput()
                cpi.setByPoint(adsk.core.Point3D.create(cx, cy, cz))
                cp = root.constructionPoints.add(cpi); cp.isLightBulbOn = False
                return adsk.fusion.JointGeometry.createByPoint(cp)
            except: pass
            try:
                sketch = root.sketches.add(root.xYConstructionPlane); sketch.isVisible = False
                s_pt = sketch.sketchPoints.add(adsk.core.Point3D.create(cx, cy, cz))
                return adsk.fusion.JointGeometry.createByPoint(s_pt)
            except: return None

        def revolute_joint(name, occ1, occ2, cx, cy, cz, axis_str):
            if not (occ1 and occ2): return
            try:
                geom = _make_joint_geometry(cx, cy, cz)
                ji = root.asBuiltJoints.createInput(occ1, occ2, geom)
                av = {"x": adsk.core.Vector3D.create(1,0,0),
                      "y": adsk.core.Vector3D.create(0,1,0),
                      "z": adsk.core.Vector3D.create(0,0,1)}[axis_str]
                ji.setAsRevoluteJointMotion(adsk.fusion.JointDirections.CustomJointDirection, av)
                j = root.asBuiltJoints.add(ji); j.name = name
            except: pass

        def ball_joint(name, occ1, occ2, cx, cy, cz):
            if not (occ1 and occ2): return
            try:
                geom = _make_joint_geometry(cx, cy, cz)
                ji = root.asBuiltJoints.createInput(occ1, occ2, geom)
                ji.setAsBallJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection,
                                        adsk.fusion.JointDirections.XAxisJointDirection)
                j = root.asBuiltJoints.add(ji); j.name = name
            except: pass

        def rigid_joint(name, occ1, occ2):
            if not (occ1 and occ2): return
            try:
                geom = _make_joint_geometry(0, 0, 0)
                ji = root.asBuiltJoints.createInput(occ1, occ2, geom)
                ji.setAsRigidJointMotion()
                j = root.asBuiltJoints.add(ji); j.name = name
            except: pass

        # ── SERVO HELPERS ────────────────────────────────────────────────
        def servo_hardware(comp, tag, cx, cy, cz, axis, mg996):
            if mg996: fd, fw, hr, pd, sd = 2.4, 0.5, 0.7, 0.125, 0.15
            else: fd, fw, hr, pd, sd = 1.35, 0.0, 0.4, 0.10, 0.10
            if axis == "x": hx,hy,hz = cx+2.40 if mg996 else cx+1.40, cy, cz+1.05 if mg996 else cz+0.50; fx,fy,fz = cx+0.95 if mg996 else cx+0.45, cy, cz
            elif axis == "z": hx,hy,hz = cx-1.10 if mg996 else cx-0.50, cy, cz+2.40 if mg996 else cz+1.40; fx,fy,fz = cx, cy, cz+0.95 if mg996 else cz+0.45
            else: hx,hy,hz = cx, cy+2.40 if mg996 else cy+1.40, cz+1.05 if mg996 else cz+0.50; fx,fy,fz = cx, cy+0.95 if mg996 else cy+0.45, cz
            for d1 in [-fd, fd]:
                for d2 in ([-fw, fw] if fw > 0 else [0]):
                    if axis == "x": c = cyl(comp, f"{tag}_FlgS_{d1}_{d2}", fx, fy+d2, fz+d1, sd, 1.5, "x")
                    elif axis == "z": c = cyl(comp, f"{tag}_FlgS_{d1}_{d2}", fx+d1, fy+d2, fz, sd, 1.5, "z")
                    else: c = cyl(comp, f"{tag}_FlgS_{d1}_{d2}", fx+d1, fy, fz+d2, sd, 1.5, "y")
                    cut_cavity(comp, c)
            for d in [-hr, hr]:
                if axis == "x": c1, c2 = cyl(comp, f"{tag}_Hrn1_{d}", hx, hy+d, hz, pd, 1.5, "x"), cyl(comp, f"{tag}_Hrn2_{d}", hx, hy, hz+d, pd, 1.5, "x")
                elif axis == "z": c1, c2 = cyl(comp, f"{tag}_Hrn1_{d}", hx+d, hy, hz, pd, 1.5, "z"), cyl(comp, f"{tag}_Hrn2_{d}", hx, hy+d, hz, pd, 1.5, "z")
                else: c1, c2 = cyl(comp, f"{tag}_Hrn1_{d}", hx+d, hy, hz, pd, 1.5, "y"), cyl(comp, f"{tag}_Hrn2_{d}", hx, hy, hz+d, pd, 1.5, "y")
                cut_cavity(comp, c1); cut_cavity(comp, c2)
            grommet_hole(comp, tag, cx, cy, cz + (0.5 if axis != "z" else 1.0))

        def add_mg996r(comp, tag, cx, cy, cz, axis="x"):
            cl = CLEARANCE
            if axis == "x":
                box(comp, f"{tag}_VisBody", cx, cy, cz, 4.05, 2.00, 4.20, grey_plastic)
                box(comp, f"{tag}_VisEars", cx+0.95, cy, cz, 0.30, 2.20, 5.80, dark_grey)
                cyl(comp, f"{tag}_VisHorn", cx+2.40, cy, cz+1.05, 0.95, 0.22, "x", white_pla)
                marker(comp, f"{tag}_Pivot", cx+2.40, cy, cz+1.05)
                cut_cavity(comp, box(comp, f"{tag}_CB", cx, cy, cz, 4.05+cl, 2.00+cl, 4.20+cl))
                cut_cavity(comp, box(comp, f"{tag}_CE", cx+0.95, cy, cz, 0.30+cl, 2.20+cl, 5.80+cl))
            elif axis == "z":
                box(comp, f"{tag}_VisBody", cx, cy, cz, 4.05, 2.00, 4.20, grey_plastic)
                box(comp, f"{tag}_VisEars", cx, cy, cz+0.95, 5.80, 2.20, 0.30, dark_grey)
                cyl(comp, f"{tag}_VisHorn", cx-1.10, cy, cz+2.40, 0.95, 0.22, "z", white_pla)
                marker(comp, f"{tag}_Pivot", cx-1.10, cy, cz+2.40)
                cut_cavity(comp, box(comp, f"{tag}_CB", cx, cy, cz, 4.05+cl, 2.00+cl, 4.20+cl))
                cut_cavity(comp, box(comp, f"{tag}_CE", cx, cy, cz+0.95, 5.80+cl, 2.20+cl, 0.30+cl))
            else:
                box(comp, f"{tag}_VisBody", cx, cy, cz, 4.05, 4.20, 2.00, grey_plastic)
                box(comp, f"{tag}_VisEars", cx, cy+0.95, cz, 4.05, 0.30, 2.20, dark_grey)
                cyl(comp, f"{tag}_VisHorn", cx, cy+2.40, cz+1.05, 0.95, 0.22, "y", white_pla)
                marker(comp, f"{tag}_Pivot", cx, cy+2.40, cz+1.05)
                cut_cavity(comp, box(comp, f"{tag}_CB", cx, cy, cz, 4.05+cl, 4.20+cl, 2.00+cl))
                cut_cavity(comp, box(comp, f"{tag}_CE", cx, cy+0.95, cz, 4.05+cl, 0.30+cl, 2.20+cl))
            servo_hardware(comp, tag, cx, cy, cz, axis, True)
            servo_type = "DS3225MG" if "hip" in tag.lower() or "Hip" in tag else "MG996R"
            horn_coupler(comp, tag, cx, cy, cz, axis, servo_type)
            BOM.add("Servo", "MG996R 11 kg.cm servo", 1, comp.name)

        def add_mg90s(comp, tag, cx, cy, cz, axis="x"):
            cl = CLEARANCE
            if axis == "x":
                box(comp, f"{tag}_VisBody", cx, cy, cz, 2.30, 1.20, 2.30, op_blue)
                box(comp, f"{tag}_VisEars", cx+0.45, cy, cz, 0.20, 1.30, 3.20, op_blue)
                cyl(comp, f"{tag}_VisHorn", cx+1.40, cy, cz+0.50, 0.55, 0.18, "x", white_pla)
                marker(comp, f"{tag}_Pivot", cx+1.40, cy, cz+0.50)
                cut_cavity(comp, box(comp, f"{tag}_CB", cx, cy, cz, 2.30+cl, 1.20+cl, 2.30+cl))
                cut_cavity(comp, box(comp, f"{tag}_CE", cx+0.45, cy, cz, 0.20+cl, 1.30+cl, 3.20+cl))
            elif axis == "z":
                box(comp, f"{tag}_VisBody", cx, cy, cz, 2.30, 1.20, 2.30, op_blue)
                box(comp, f"{tag}_VisEars", cx, cy, cz+0.45, 3.20, 1.30, 0.20, op_blue)
                cyl(comp, f"{tag}_VisHorn", cx-0.50, cy, cz+1.40, 0.55, 0.18, "z", white_pla)
                marker(comp, f"{tag}_Pivot", cx-0.50, cy, cz+1.40)
                cut_cavity(comp, box(comp, f"{tag}_CB", cx, cy, cz, 2.30+cl, 1.20+cl, 2.30+cl))
                cut_cavity(comp, box(comp, f"{tag}_CE", cx, cy, cz+0.45, 3.20+cl, 1.30+cl, 0.20+cl))
            else:
                box(comp, f"{tag}_VisBody", cx, cy, cz, 2.30, 2.30, 1.20, op_blue)
                box(comp, f"{tag}_VisEars", cx, cy+0.45, cz, 3.20, 0.20, 1.30, op_blue)
                cyl(comp, f"{tag}_VisHorn", cx, cy+1.40, cz+0.50, 0.55, 0.18, "y", white_pla)
                marker(comp, f"{tag}_Pivot", cx, cy+1.40, cz+0.50)
                cut_cavity(comp, box(comp, f"{tag}_CB", cx, cy, cz, 2.30+cl, 2.30+cl, 1.20+cl))
                cut_cavity(comp, box(comp, f"{tag}_CE", cx, cy+0.45, cz, 3.20+cl, 0.20+cl, 1.30+cl))
            servo_hardware(comp, tag, cx, cy, cz, axis, False)
            horn_coupler(comp, tag, cx, cy, cz, axis, "MG90S")
            BOM.add("Servo", "MG90S micro servo", 1, comp.name)

        def add_ds04_nfc(comp, tag, cx, cy, cz, axis="x"):
            cl, dim = CLEARANCE, 2.00
            box(comp, f"{tag}_VisBody", cx, cy, cz, dim, 0.80, dim, grey_plastic)
            cyl(comp, f"{tag}_VisHorn", cx+0.80, cy, cz+0.40, 0.40, 0.15, axis, white_pla)
            marker(comp, f"{tag}_Pivot", cx+0.80, cy, cz+0.40)
            cut_cavity(comp, box(comp, f"{tag}_CB", cx, cy, cz, dim+cl, 0.80+cl, dim+cl))
            BOM.add("Servo", "DS04-NFC 9g digital servo", 1, comp.name)
            horn_coupler(comp, tag, cx, cy, cz, axis, "DS04-NFC")

        def tt_wheel(comp, tag, cx, cy, cz, side=1):
            cl = CLEARANCE
            box(comp, f"{tag}_VisGB", cx, cy, cz, 2.30, 5.20, 1.90, yellow_met)
            cyl(comp, f"{tag}_VisMot", cx, cy-1.5, cz, 0.90, 2.10, "y", chrome)
            cyl(comp, f"{tag}_VisShaft", cx+side*1.75, cy, cz, 0.20, 3.50, "x", chrome)
            cyl(comp, f"{tag}_VisHub", cx+side*3.25, cy, cz, 0.80, 2.60, "x", dark_metal)
            cyl(comp, f"{tag}_VisTire", cx+side*3.25, cy, cz, 3.25, 2.60, "x", rubber_blk)
            cyl(comp, f"{tag}_VisRim", cx+side*3.25, cy, cz, 2.20, 2.65, "x", chrome)
            marker(comp, f"{tag}_Axle_Pivot", cx+side*3.25, cy, cz, 0.18)
            cut_cavity(comp, box(comp, f"{tag}_CGB", cx, cy, cz, 2.30+cl, 5.20+cl, 1.90+cl))
            cut_cavity(comp, box(comp, f"{tag}_CDS", cx+side*3.25, cy, cz, 2.7+cl, 0.54+cl, 0.36+cl))
            BOM.add("Drive", "TT gear-motor 3V-6V", 1, comp.name)
            BOM.add("Drive", "65 mm rubber tyre + wheel", 1, comp.name)

        def u_bracket(comp, tag, cx, cy, cz, lx, ly, lz, ap=None):
            ap = ap or chrome
            box(comp, f"{tag}_BB", cx, cy, cz, 0.45, ly, lz, ap)
            box(comp, f"{tag}_BL", cx+lx*0.45, cy+ly*0.35, cz, lx*0.55, 0.40, lz, ap)
            box(comp, f"{tag}_BR", cx+lx*0.45, cy-ly*0.35, cz, lx*0.55, 0.40, lz, ap)
            cyl(comp, f"{tag}_Pin", cx+lx*0.50, cy, cz, 0.18, ly*0.85, "y", chrome)
            BOM.add("Printed", f"U-bracket {tag}", 1, f"PETG")

        def structural_rib(comp, tag, cx, cy, cz, lx, ly, lz):
            box(comp, f"{tag}_Rib", cx, cy, cz, lx*0.12, ly, lz, dark_metal)
            for ri in range(3):
                rx = cx + (ri - 1) * lx * 0.25
                box(comp, f"{tag}_Rib_{ri}", rx, cy, cz, 0.15, ly*0.9, 0.15, dark_metal)

        # ── ADVANCED MECHANICAL HELPERS (from qwen.py) ────────────────────
        def metal_frame_channel(comp, name, cx, cy, cz, length, axis="z"):
            ch = box(comp, name, cx, cy, cz, EXTRUSION_W, EXTRUSION_W, length, dark_metal)
            if axis == "x": slot = box(comp, f"{name}_Slot", cx, cy+EXTRUSION_SLOT_D, cz, length, EXTRUSION_SLOT_W, EXTRUSION_SLOT_D*2, dark_metal)
            elif axis == "y": slot = box(comp, f"{name}_Slot", cx+EXTRUSION_SLOT_D, cy, cz, EXTRUSION_SLOT_W, length, EXTRUSION_SLOT_D*2, dark_metal)
            else: slot = box(comp, f"{name}_Slot", cx+EXTRUSION_SLOT_D, cy, cz, EXTRUSION_SLOT_W, EXTRUSION_SLOT_D*2, length, dark_metal)
            cut_cavity(comp, slot)
            BOM.add("Frame", f"2020 extrusion {int(length*10)}mm", 1, name)
            return ch

        def frame_mount_bracket(comp, name, cx, cy, cz, axis="z"):
            b = box(comp, name, cx, cy, cz, FRAME_BRACKET_W, FRAME_BRACKET_T, FRAME_BRACKET_W, dark_metal)
            for bx in [cx - FRAME_BRACKET_W*0.3, cx + FRAME_BRACKET_W*0.3]:
                screw_hole(comp, bx, cy, cz, axis=axis, length=FRAME_BRACKET_W)
            BOM.add("Frame", f"L-bracket {name}", 2, "M3x8 + T-nut")
            return b

        def steel_shaft_block(comp, name, cx, cy, cz, axis, shaft_diam, length, bearing_count=2):
            bw2 = max(BEARING_BLOCK_W, shaft_diam + 0.30)
            bl = box(comp, name, cx, cy, cz, bw2, BEARING_BLOCK_H, length, dark_metal)
            bore = cyl(comp, f"{name}_Bore", cx, cy, cz, shaft_diam/2 + CLEARANCE, length + 0.20, axis)
            cut_cavity(comp, bore)
            ss_off = shaft_diam * 0.6
            ssx = ss_off if axis != "x" else 0; ssy = ss_off if axis != "y" else 0; ssz = ss_off if axis != "z" else 0
            ss = cyl(comp, f"{name}_SetScrew", cx+ssx, cy+ssy, cz+ssz, SET_SCREW_R, SET_SCREW_L, axis, dark_metal)
            merge_bodies(comp, bl, ss)
            return bl

        def worm_gear_housing(comp, name, cx, cy, cz, axis, ratio=GEAR_RATIO_WAIST):
            hw, hl = 1.80, 2.20
            h = box(comp, name, cx, cy, cz, hw, hl, hl, dark_metal)
            sr = 0.30 + CLEARANCE
            worm_brg = cyl(comp, f"{name}_WormBrg", cx, cy-0.8, cz, sr, hl, "y"); cut_cavity(comp, worm_brg)
            wheel_brg = cyl(comp, f"{name}_WheelBrg", cx, cy+0.2, cz, sr*ratio/10+0.1, hw, "x"); cut_cavity(comp, wheel_brg)
            BOM.add("Gearbox", f"Worm set {ratio}:1", 1, name)
            return h

        def planetary_gear_stage(comp, name, cx, cy, cz, axis, reduction=GEAR_RATIO_HIP):
            p = box(comp, name, cx, cy, cz, PLANET_STAGE_R*2, PLANET_STAGE_H, PLANET_STAGE_R*2, dark_metal)
            sr = PLANET_SUN_R + CLEARANCE
            sun = cyl(comp, f"{name}_Sun", cx, cy, cz, sr, PLANET_STAGE_H+0.1, axis); cut_cavity(comp, sun)
            for gi in range(3):
                gx = cx + (PLANET_SUN_R + PLANET_GEAR_R + 0.05) * 0.7 * (1 if axis == "z" else 0)
                gy = cy + (PLANET_SUN_R + PLANET_GEAR_R + 0.05) * 0.7 * (1 if axis == "y" else 0)
                gz = cz + (PLANET_SUN_R + PLANET_GEAR_R + 0.05) * 0.7 * (1 if axis == "x" else 0)
                pg = cyl(comp, f"{name}_PG{gi}", gx, gy, gz, PLANET_GEAR_R, PLANET_STAGE_H+0.1, axis); cut_cavity(comp, pg)
            BOM.add("Gearbox", f"Planetary stage {reduction}:1", 1, name)
            return p

        def transformation_latch_hook(comp, name, cx, cy, cz, axis="z"):
            hk = box(comp, name, cx, cy, cz, LATCH_HOOK_D, LATCH_HOOK_D, LATCH_HOOK_L, dark_metal)
            hp = box(comp, f"{name}_Mount", cx, cy, cz, LATCH_HOOK_D+0.20, 0.10, 0.10, dark_metal)
            merge_bodies(comp, hk, hp)
            BOM.add("Hardware", f"Latch hook {name}", 1, "PLA/PETG")
            return hk

        def transformation_latch_post(comp, name, cx, cy, cz, axis="z"):
            p = cyl(comp, name, cx, cy, cz, LATCH_POST_D/2, LATCH_POST_H, axis, chrome)
            mp = box(comp, f"{name}_Base", cx, cy, cz-LATCH_POST_H/2, LATCH_POST_D+0.20, LATCH_POST_D+0.20, 0.10, dark_metal)
            merge_bodies(comp, p, mp)
            return p

        def solenoid_mount(comp, name, cx, cy, cz, axis="z"):
            s = cyl(comp, name, cx, cy, cz, LATCH_SOLENOID_D/2 + CLEARANCE, LATCH_SOLENOID_L, axis)
            cut_cavity(comp, s)
            wire_exit = box(comp, f"{name}_Wire", cx, cy, cz-LATCH_SOLENOID_L/2, 0.20, 0.20, 0.30, dark_metal)
            merge_bodies(comp, s, wire_exit)
            BOM.add("Electronics", f"12V locking solenoid {int(LATCH_SOLENOID_D*10)}x{int(LATCH_SOLENOID_L*10)}mm", 1, name)
            return s

        def quick_release_coupler(comp, name, cx, cy, cz, axis="z", side=1):
            m_h = box(comp, f"{name}_Male", cx, cy, cz, QR_PIN_D, QR_PIN_D, QR_COLLAR_L, chrome)
            f_h = box(comp, f"{name}_Female", cx+QR_COLLAR_D*side, cy, cz, QR_COLLAR_D, QR_COLLAR_D, QR_COLLAR_L, dark_metal)
            detent = cyl(comp, f"{name}_Detent", cx+QR_COLLAR_D*side*0.5, cy, cz, QR_DETENT_BALL_R, QR_COLLAR_L*0.3, axis, chrome)
            merge_bodies(comp, f_h, detent)
            BOM.add("Hardware", f"Quick-release coupler {name}", 1, "PLA/PETG")
            return m_h, f_h

        def spine_vertebra(comp, name, cx, cy, cz, axis="z"):
            v = box(comp, name + "_Seg", cx, cy, cz, SPINE_SEG_W, SPINE_SEG_H, SPINE_SEG_D, dark_metal)
            ball = cyl(comp, name + "_Ball", cx, cy+SPINE_SEG_H/2, cz, SPINE_BALL_D/2, 0.30, axis, chrome)
            socket = cyl(comp, name + "_Socket", cx, cy-SPINE_SEG_H/2, cz, SPINE_SOCKET_D/2, 0.20, axis, dark_metal)
            cut_cavity(comp, socket)
            nerve = box(comp, name + "_Nerve", cx, cy, cz, SPINE_NERVE_CH_D, SPINE_SEG_H, SPINE_NERVE_CH_D)
            cut_cavity(comp, nerve)
            BOM.add("Printed", f"Spine vertebra {name}", 1, "PETG")
            return v

        # ═════════════════════════════════════════════════════════════════
        # COMPONENT BUILDING — BODY SECTIONS
        # ═════════════════════════════════════════════════════════════════

        # ── 1. TORSO ─────────────────────────────────────────────────────
        torso = new_component("OP_Torso")

        box(torso, "Torso_Shell",    0,    0,   TORSO_CTR,        10.4, 8.6, 12.2, op_red)
        box(torso, "Torso_Side_L",  -5.6,  0,   TORSO_CTR,         0.5, 7.8, 11.2, op_red)
        box(torso, "Torso_Side_R",   5.6,  0,   TORSO_CTR,         0.5, 7.8, 11.2, op_red)

        box(torso, "CWin_Frame_L",  -2.3, -4.20, TORSO_CTR+2.5,   2.8, 0.32, 3.2, op_blue)
        box(torso, "CWin_Frame_R",   2.3, -4.20, TORSO_CTR+2.5,   2.8, 0.32, 3.2, op_blue)
        box(torso, "Chest_Win_L",   -2.3, -4.35, TORSO_CTR+2.5,   2.4, 0.20, 2.8, glass_clr)
        box(torso, "Chest_Win_R",    2.3, -4.35, TORSO_CTR+2.5,   2.4, 0.20, 2.8, glass_clr)
        box(torso, "Chest_Win_Div",  0,   -4.25, TORSO_CTR+2.5,   0.40, 0.22, 3.2, chrome)

        for gz_offset, gw in [(-0.2, 7.4), (-1.0, 7.0), (-1.8, 6.6), (-2.6, 6.2)]:
            box(torso, f"Grille_{int(gz_offset*10)}", 0, -4.40, TORSO_CTR+gz_offset, gw, 0.22, 0.30, chrome)

        box(torso, "Headlight_L",   -4.4, -4.50, TORSO_CTR-1.2,  1.8, 0.28, 2.0, glass_clr)
        box(torso, "Headlight_R",    4.4, -4.50, TORSO_CTR-1.2,  1.8, 0.28, 2.0, glass_clr)
        box(torso, "Front_Bumper",   0,   -5.8,  TORSO_CTR-4.4,  10.0, 2.0,  1.8, chrome)
        box(torso, "Hood_Crease_L", -2.5, -4.60, TORSO_CTR-2.8,   0.5, 0.35, 3.0, op_red)
        box(torso, "Hood_Crease_R",  2.5, -4.60, TORSO_CTR-2.8,   0.5, 0.35, 3.0, op_red)
        box(torso, "Ind_L",         -3.8, -5.00, TORSO_CTR-3.8,   0.6, 0.25, 0.5, yellow_met)
        box(torso, "Ind_R",          3.8, -5.00, TORSO_CTR-3.8,   0.6, 0.25, 0.5, yellow_met)

        box(torso, "Chest_Plate",    0,   -4.20, TORSO_CTR+0.5,   8.4, 0.32, 4.0, chrome)
        cyl(torso, "Badge_Ring",     0,   -4.55, TORSO_CTR+0.5,   0.80, 0.12, "y", op_red)
        led_ring_pocket(torso, "Badge",  0, -4.60, TORSO_CTR+0.5, "y")

        box(torso, "Inner_Frame",    0,    0,    TORSO_CTR+1.5,   7.4, 6.0,  8.2, dark_metal)
        box(torso, "Spine_Beam",     0,    0,    TORSO_CTR+1.5,   1.8, 1.8,  8.2, chrome)
        cyl(torso, "Spine_Cyl",      0,    0,    TORSO_CTR+1.5,  1.10, 4.5,  "z", chrome)

        box(torso, "LipoBay_Shell",  0,    3.2, TORSO_CTR-2.0,   7.6, 4.4,  5.6, black_plastic)
        lipo_bay(torso, "Main", 0, 2.8, TORSO_CTR-2.0)

        box(torso, "RPi_Shell",      0,    2.8, TORSO_CTR+1.8,   7.0, 3.6,  2.8, black_plastic)
        rpi_zero_bay(torso, "Main",  0,    3.2, TORSO_CTR+1.8)

        box(torso, "PCA_Shell",      0,    2.8, TORSO_CTR+4.2,   6.8, 3.2,  2.2, black_plastic)
        pca9685_bay(torso, "Main",   0,    3.0, TORSO_CTR+4.2)

        box(torso, "BEC_Shell",      0,    2.0, TORSO_CTR-2.8,   4.2, 2.6,  1.2, black_plastic)
        bec_bay(torso, "Main", 0, 2.0, TORSO_CTR-2.8)

        box(torso, "Fuse_Shell",     0,    3.8, TORSO_CTR-2.0,   1.6, 1.4,  1.0, black_plastic)
        fuse_pocket(torso, "Main", 0, 3.8, TORSO_CTR-2.0)

        box(torso, "Cable_L",       -3.4,  0.6,  TORSO_CTR,      0.55, 1.0, 10.0, dark_grey)
        box(torso, "Cable_R",        3.4,  0.6,  TORSO_CTR,      0.55, 1.0, 10.0, dark_grey)
        cable_clip(torso, "Spine_A", 0, 1.0, TORSO_CTR-2.0)
        cable_clip(torso, "Spine_B", 0, 1.0, TORSO_CTR+2.0)

        box(torso, "Collar_L",      -8.0,  0,    SHOULDER_CTR-1.0, 5.5, 3.4, 3.0, chrome)
        box(torso, "Collar_R",       8.0,  0,    SHOULDER_CTR-1.0, 5.5, 3.4, 3.0, chrome)

        box(torso, "TF_Flap_L",     -5.40,-0.2,  TORSO_CTR+3.0,  0.40, 6.6,  6.2, op_red)
        box(torso, "TF_Flap_R",      5.40,-0.2,  TORSO_CTR+3.0,  0.40, 6.6,  6.2, op_red)
        box(torso, "TF_BackTop",     0,    5.0,  TORSO_CTR+5.2,  8.2, 0.38,  5.2, op_blue)

        for bx_off, bz_off in [(-3.2, 4.8), (3.2, 4.8), (-3.2, -4.8), (3.2, -4.8)]:
            m3_boss(torso, f"Torso_B{bx_off}_{bz_off}", bx_off, 0, TORSO_CTR+bz_off)
        align_pin(torso, "TorsoSplit_L", -5.0, 0, TORSO_CTR)
        align_pin(torso, "TorsoSplit_R",  5.0, 0, TORSO_CTR)

        u_bracket(torso, "Waist_Brkt",  0, 0, WAIST_CTR,     4.0, 4.2, 3.4)
        add_mg996r(torso, "Waist_Yaw",   0, 0, WAIST_CTR,     "z")
        bearing(torso,   "Waist_Brg",   0, 0, WAIST_CTR+0.5, "z", 1.30, 0.65)
        u_bracket(torso, "WaistP_Brkt", 0, 0, WAIST_CTR-2.5, 4.0, 4.2, 3.4)
        add_mg996r(torso, "Waist_Pitch", 0, 0, WAIST_CTR-2.5, "x")
        bearing(torso,   "WaistP_Brg",  0, 0, WAIST_CTR-2.0, "x", 1.30, 0.65)
        magnet_pocket(torso, "WLock_F", 0, -2.0, WAIST_CTR-3.0)
        magnet_pocket(torso, "WLock_R", 0,  2.0, WAIST_CTR-3.0)

        u_bracket(torso, "Neck_Brkt",   0, 0, NECK_JOINT_Z,  3.2, 2.8, 3.0)
        add_mg996r(torso, "Neck_Pitch",  0, 0, NECK_JOINT_Z,  "x")
        wire_channel(torso, "Spine",    0, 0, TORSO_CTR,     0.6, 20.0, "z")

        # ── 2. HEAD ──────────────────────────────────────────────────────
        head = new_component("OP_Head")
        HC = HEAD_CTR

        box(head, "Helm_Main",      0,     0,    HC+0.8,  5.6, 5.2, 5.0, op_blue)
        box(head, "Helm_Forehead",  0,    -2.30, HC+2.6,  5.0, 0.42, 1.8, op_blue)
        box(head, "Helm_Top",       0,     0,    HC+3.4,  4.8, 4.4, 0.90, op_blue)

        cone_shape(head, "Ear_L",        -3.30,  0,    HC+1.8,  0.50, 0.70, 4.8, "x", op_blue)
        cone_shape(head, "Ear_R",         3.30,  0,    HC+1.8,  0.70, 0.50, 4.8, "x", op_blue)
        box(head, "EarTop_L",      -3.05,  0,    HC+3.5,  1.40, 4.4, 0.62, op_blue)
        box(head, "EarTop_R",       3.05,  0,    HC+3.5,  1.40, 4.4, 0.62, op_blue)

        box(head, "Crest_Main",     0,    -0.30, HC+3.85, 1.05, 0.68, 3.6, chrome)
        box(head, "Crest_Stripe",   0,    -0.30, HC+3.95, 0.55, 0.36, 2.9, op_blue)

        box(head, "Face_Plate",     0,    -2.50, HC+0.5,  3.6, 0.38, 3.2, chrome)
        box(head, "Chin_Guard",     0,    -2.60, HC-0.9,  3.0, 0.38, 1.8, chrome)
        box(head, "Chin_L",        -1.35, -2.52, HC-0.4,  0.38, 0.32, 2.2, chrome)
        box(head, "Chin_R",         1.35, -2.52, HC-0.4,  0.38, 0.32, 2.2, chrome)

        box(head, "Visor_Frame",    0,    -2.55, HC+1.35, 3.8, 0.30, 1.25, op_blue)
        box(head, "Visor",          0,    -2.68, HC+1.35, 3.0, 0.16, 0.92, op_blue_glass)
        for vx in [-0.85, 0.0, 0.85]:
            led_pocket_5mm(head, f"Visor_LED_{int(vx*100)}", vx, -2.80, HC+1.35, "y")

        box(head, "Nose_Bridge",    0,    -2.60, HC+1.95, 0.72, 0.22, 0.72, chrome)
        box(head, "Mouth_Plate",    0,    -2.55, HC+0.10, 2.4, 0.22, 1.10, dark_grey)
        for mz in [-0.32, 0.0, 0.32]:
            box(head, f"MouthGrill_{int(mz*100)}", 0, -2.62, HC+0.10+mz, 1.8, 0.12, 0.18, chrome)

        box(head, "Stk_A",          0,     0,    HC-1.8,  0.80, 0.50, 0.50, chrome)
        box(head, "Stk_Tip",        0,     0,    HC-2.4,  0.20, 0.50, 0.50, chrome)

        box(head, "Cheek_L",       -2.50, -2.50, HC-0.2,  0.32, 0.40, 2.2, op_blue)
        box(head, "Cheek_R",        2.50, -2.50, HC-0.2,  0.32, 0.40, 2.2, op_blue)
        box(head, "Jowl_L",        -1.90, -2.55, HC-1.0,  0.50, 0.32, 1.4, op_blue)
        box(head, "Jowl_R",         1.90, -2.55, HC-1.0,  0.50, 0.32, 1.4, op_blue)

        m3_boss(head, "Helm_LR", -2.2, 0, HC+0.8)
        m3_boss(head, "Helm_RR",  2.2, 0, HC+0.8)
        m3_boss(head, "Helm_LF", -2.2, 0, HC-1.2)
        m3_boss(head, "Helm_RF",  2.2, 0, HC-1.2)

        esp32_cam_pocket(head, "Face", 0, -2.30, HC+0.5, "y")

        # ── 3. PELVIS ────────────────────────────────────────────────────
        pelvis = new_component("OP_Pelvis")

        box(pelvis, "Pelvis_Main",   0,  0,    PELVIS_CTR,      8.0, 6.4, 3.4, op_red)
        box(pelvis, "Pelvis_Belt_L", -HIP_X-1.2, 0, PELVIS_CTR-1.0, 2.8, 4.0, 2.8, op_red)
        box(pelvis, "Pelvis_Belt_R",  HIP_X+1.2, 0, PELVIS_CTR-1.0, 2.8, 4.0, 2.8, op_red)

        box(pelvis, "Hip_Pod_L",    -HIP_X-2.4, 0, PELVIS_CTR-0.4, 3.0, 2.4, 5.2, op_blue)
        box(pelvis, "Hip_Pod_R",     HIP_X+2.4, 0, PELVIS_CTR-0.4, 3.0, 2.4, 5.2, op_blue)

        box(pelvis, "Skirt_L",      -4.4, -3.60, PELVIS_CTR-0.2, 0.32, 3.2, 3.8, chrome)
        box(pelvis, "Skirt_R",       4.4, -3.60, PELVIS_CTR-0.2, 0.32, 3.2, 3.8, chrome)
        box(pelvis, "Skirt_F",       0,   -3.80, PELVIS_CTR-0.2, 3.0, 0.32, 3.8, chrome)

        imu_pocket(pelvis, "Main", 0, 0, PELVIS_CTR)
        m3_boss(pelvis, "Pelvis_FL", -3.6, 0, PELVIS_CTR-1.0)
        m3_boss(pelvis, "Pelvis_FR",  3.6, 0, PELVIS_CTR-1.0)
        m3_boss(pelvis, "Pelvis_RL", -3.6, 0, PELVIS_CTR+1.0)
        m3_boss(pelvis, "Pelvis_RR",  3.6, 0, PELVIS_CTR+1.0)

        u_bracket(pelvis, "HipL_Brkt", -HIP_X, 0, HIP_JOINT_Z, 3.6, 4.0, 3.8)
        add_mg996r(pelvis, "HipL_Yaw",  -HIP_X, 0, HIP_JOINT_Z, "z")
        bearing(pelvis, "HipL_Brg", -HIP_X, 0, HIP_JOINT_Z+0.5, "z", 1.30, 0.65)

        u_bracket(pelvis, "HipR_Brkt",  HIP_X, 0, HIP_JOINT_Z, 3.6, 4.0, 3.8)
        add_mg996r(pelvis, "HipR_Yaw",   HIP_X, 0, HIP_JOINT_Z, "z")
        bearing(pelvis, "HipR_Brg",  HIP_X, 0, HIP_JOINT_Z+0.5, "z", 1.30, 0.65)

        # ── 4. LEGS ──────────────────────────────────────────────────────
        def build_leg(side, sx):
            ax_sgn = -1 if side == "L" else 1
            leg = new_component(f"OP_Leg_{side}")

            # Thigh
            box(leg, "Thigh_Main", sx, 0, THIGH_CTR, 4.2, 3.8, 8.2, op_red)
            box(leg, "Thigh_Armor_L", sx-2.4, 0, THIGH_CTR, 0.60, 3.6, 7.4, op_red)
            box(leg, "Thigh_Armor_R", sx+2.4, 0, THIGH_CTR, 0.60, 3.6, 7.4, op_red)
            box(leg, "Thigh_Pod", sx, 0, THIGH_CTR-2.5, 2.8, 2.4, 2.2, op_blue)

            u_bracket(leg, f"HipPitch_{side}_Brkt", sx, 0, THIGH_CTR, 4.2, 2.8, 3.2)
            add_mg996r(leg, f"HipPitch_{side}", sx, 0, THIGH_CTR, "x")
            bearing(leg, f"HipPitch_{side}_Brg", sx, 0, THIGH_CTR+0.4, "x", 1.20, 0.60)

            m3_boss(leg, f"Thigh_F_{side}", sx-2.0, 0, THIGH_CTR+2.5)
            m3_boss(leg, f"Thigh_R_{side}", sx+2.0, 0, THIGH_CTR+2.5)

            # Shin
            box(leg, "Shin_Main", sx, 0, SHIN_CTR, 3.8, 3.4, 9.4, op_red)
            box(leg, "Shin_Armor_L", sx-2.2, 0, SHIN_CTR, 0.50, 3.2, 8.6, op_red)
            box(leg, "Shin_Armor_R", sx+2.2, 0, SHIN_CTR, 0.50, 3.2, 8.6, op_red)

            u_bracket(leg, f"Knee_{side}_Brkt", sx, 0, KNEE_CTR, 3.8, 3.4, 2.4)
            add_mg996r(leg, f"Knee_{side}", sx, 0, KNEE_CTR, "x")
            bearing(leg, f"Knee_{side}_Brg", sx, 0, KNEE_CTR+0.4, "x", 1.20, 0.60)

            m3_boss(leg, f"Shin_F_{side}", sx-1.8, 0, SHIN_CTR+2.0)
            m3_boss(leg, f"Shin_R_{side}", sx+1.8, 0, SHIN_CTR+2.0)

            # Foot
            box(leg, "Foot_Main", sx, 0, ANKLE_CTR, 4.6, 2.2, 2.0, op_red)
            box(leg, "Heel_Spur", sx-2.2, 0, ANKLE_CTR+1.2, 0.30, 0.60, 0.80, chrome)
            box(leg, "Toe_Cap", sx+2.2, 0, ANKLE_CTR-1.0, 0.80, 0.80, 0.60, chrome)
            box(leg, "Sole_Grip", sx, 0, ANKLE_CTR, 4.6, 0.30, 2.0, rubber_blk)

            u_bracket(leg, f"Ankle_{side}_Brkt", sx, 0, ANKLE_CTR, 3.6, 2.6, 1.8)
            add_mg90s(leg, f"Ankle_{side}", sx, 0, ANKLE_CTR, "x")
            bearing(leg, f"Ankle_{side}_Brg", sx, 0, ANKLE_CTR+0.3, "x", 0.90, 0.50)

            return leg

        build_leg("L", -HIP_X)
        build_leg("R", HIP_X)

        # ── 5. ARMS ──────────────────────────────────────────────────────
        def build_arm(side, sx):
            ax_sgn = -1 if side == "L" else 1
            arm = new_component(f"OP_Arm_{side}")

            # Upper arm
            box(arm, "UA_Main", sx, 0, SHOULDER_CTR-2.0, 4.4, 3.0, 6.0, op_red)
            box(arm, "UA_Pod", sx+ax_sgn*2.2, 0, SHOULDER_CTR-2.0, 1.2, 2.6, 4.6, op_blue)

            u_bracket(arm, f"Shoulder_{side}_Brkt", sx, 0, SHOULDER_CTR-2.0, 4.4, 3.2, 2.2)
            add_mg996r(arm, f"Shoulder_{side}", sx, 0, SHOULDER_CTR-2.0, "y")
            bearing(arm, f"Shoulder_{side}_Brg", sx, 0, SHOULDER_CTR-1.8, "y", 1.30, 0.65)

            m3_boss(arm, f"UA_F_{side}", sx+ax_sgn*2.0, 0, SHOULDER_CTR-4.0)
            m3_boss(arm, f"UA_R_{side}", sx+ax_sgn*2.0, 0, SHOULDER_CTR)

            # Forearm
            box(arm, "FA_Main", sx, 0, ELBOW_Z+2.0, 3.8, 2.8, 5.4, op_red)
            box(arm, "FA_Armor_L", sx-2.2*ax_sgn, 0, ELBOW_Z+2.0, 0.50, 2.6, 4.8, op_red)
            box(arm, "FA_Armor_R", sx+2.2*ax_sgn, 0, ELBOW_Z+2.0, 0.50, 2.6, 4.8, op_red)

            u_bracket(arm, f"Elbow_{side}_Brkt", sx, 0, ELBOW_Z, 3.8, 2.8, 2.0)
            add_mg996r(arm, f"Elbow_{side}", sx, 0, ELBOW_Z, "x")
            bearing(arm, f"Elbow_{side}_Brg", sx, 0, ELBOW_Z+0.4, "x", 1.20, 0.60)

            m3_boss(arm, f"FA_F_{side}", sx+ax_sgn*1.8, 0, ELBOW_Z+0.5)
            m3_boss(arm, f"FA_R_{side}", sx+ax_sgn*1.8, 0, ELBOW_Z+3.5)

            return arm

        build_arm("L", -SHOULDER_X)
        build_arm("R", SHOULDER_X)

        # ── 6. HANDS & FINGERS ──────────────────────────────────────────
        def build_hand(side, hx):
            ax_sgn = -1 if side == "L" else 1
            hand = new_component(f"OP_Hand_{side}")

            # Palm
            box(hand, "Palm_Main", hx, 0, WRIST_Z, 2.8, 1.8, 3.2, dark_grey)
            box(hand, "Palm_Back", hx+ax_sgn*1.6, 0, WRIST_Z, 0.40, 1.6, 2.8, dark_grey)

            add_mg90s(hand, f"Wrist_{side}", hx, 0, WRIST_Z, "x")
            add_ds04_nfc(hand, f"Tendon_{side}", hx, 0.5, WRIST_Z-2.0, "x")

            # Fingers
            for fi, (fname, fxo, pp_l, mp_l, dp_l) in enumerate(
                    zip(FING_NAMES, FING_X_OFF, FING_PP, FING_MP, FING_DP)):
                fx = hx + ax_sgn * fxo
                mcp_z = WRIST_Z - PALM_BOTTOM_OFFSET
                pp_cz = mcp_z - pp_l/2
                mp_cz = mcp_z - pp_l - mp_l/2
                dp_cz = mcp_z - pp_l - mp_l - dp_l/2

                finger = new_component(f"OP_{fname}_{side}")
                box(finger, f"{fname}_PP", fx, -1.15, pp_cz, FING_W, FING_H, pp_l, op_red)
                box(finger, f"{fname}_MP", fx, -1.15, mp_cz, FING_W*0.85, FING_H*0.85, mp_l, op_red)
                box(finger, f"{fname}_DP", fx, -1.15, dp_cz, FING_W*0.70, FING_H*0.70, dp_l, op_red)
                marker(finger, f"{fname}_MCP_Pivot", fx, -1.15, mcp_z)
                marker(finger, f"{fname}_PIP_Pivot", fx, -1.15, mcp_z - pp_l)
                tendon_groove(finger, fname, fx, -1.15, pp_cz, pp_l, "z")
                tendon_anchor(finger, f"{fname}_Anchor", fx, -1.15, dp_cz - dp_l/2 - 0.05)

            # Thumb
            tx = hx + ax_sgn * 1.70
            thumb = new_component(f"OP_Thumb_{side}")
            thumb_mcp_z = WRIST_Z - PALM_BOTTOM_OFFSET
            box(thumb, f"Thumb_PP", tx, 0.4, thumb_mcp_z - THUMB_PP_L/2, THUMB_W, THUMB_H, THUMB_PP_L, op_red)
            box(thumb, f"Thumb_DP", tx, 0.4, thumb_mcp_z - THUMB_PP_L - THUMB_DP_L/2, THUMB_W*0.80, THUMB_H*0.80, THUMB_DP_L, op_red)
            marker(thumb, "Thumb_CMC_Pivot", tx, 0.4, thumb_mcp_z)
            tendon_groove(thumb, f"{side}_Thumb", tx, 0.4, thumb_mcp_z - THUMB_PP_L/2, THUMB_PP_L + THUMB_DP_L, "z")
            tendon_anchor(thumb, f"{side}_ThumbAnchor", tx, 0.4, thumb_mcp_z - THUMB_PP_L - THUMB_DP_L - 0.1)

            return hand

        build_hand("L", -SHOULDER_X)
        build_hand("R", SHOULDER_X)

        # ── 7. BACKPACK ──────────────────────────────────────────────────
        backpack = new_component("OP_Backpack")

        box(backpack, "BP_Main",     0,     3.6,  TORSO_CTR+4.5,  8.2, 4.2, 6.0, op_blue)
        box(backpack, "BP_TopFlap",  0,     3.8,  TORSO_CTR+7.6,  7.2, 3.6, 0.80, op_blue)
        box(backpack, "BP_Strut_L", -4.0,   3.0,  TORSO_CTR+5.0,  0.60, 1.0, 6.4, dark_metal)
        box(backpack, "BP_Strut_R",  4.0,   3.0,  TORSO_CTR+5.0,  0.60, 1.0, 6.4, dark_metal)

        for bz in [TORSO_CTR+3.8, TORSO_CTR+4.8, TORSO_CTR+5.8, TORSO_CTR+6.8]:
            box(backpack, f"BP_Vent_{int(bz*10)}", 0, 5.0, bz, 3.8, 0.18, 0.50, dark_grey)

        m3_boss(backpack, "BP_TL", -3.6, 3.2, TORSO_CTR+4.8)
        m3_boss(backpack, "BP_TR",  3.6, 3.2, TORSO_CTR+4.8)
        m3_boss(backpack, "BP_BL", -3.6, 3.2, TORSO_CTR+7.2)
        m3_boss(backpack, "BP_BR",  3.6, 3.2, TORSO_CTR+7.2)

        # ── 8. SPINE ─────────────────────────────────────────────────────
        spine_seg_count = 5
        for si in range(spine_seg_count):
            sz = TORSO_CTR + 1.5 + si * (SPINE_SEG_D + 0.2)
            spine_vertebra(torso, f"V{si}", 0, 0, sz)

        # ── 9. STEER PODS ────────────────────────────────────────────────
        steer_pod = new_component("OP_SteerPod_L")
        box(steer_pod, "Pod_Main", -HIP_X-3.8, 0, PELVIS_CTR-0.5, 2.0, 1.6, 2.8, dark_grey)
        cyl(steer_pod, "Pod_Jet", -HIP_X-3.8, 0, PELVIS_CTR-2.0, 0.60, 0.80, "z", chrome)
        led_pocket_5mm(steer_pod, "Pod_LED", -HIP_X-3.8, 0, PELVIS_CTR-2.6, "y")

        steer_pod = new_component("OP_SteerPod_R")
        box(steer_pod, "Pod_Main", HIP_X+3.8, 0, PELVIS_CTR-0.5, 2.0, 1.6, 2.8, dark_grey)
        cyl(steer_pod, "Pod_Jet", HIP_X+3.8, 0, PELVIS_CTR-2.0, 0.60, 0.80, "z", chrome)
        led_pocket_5mm(steer_pod, "Pod_LED", HIP_X+3.8, 0, PELVIS_CTR-2.6, "y")

        # ── 10. SHIELDS ──────────────────────────────────────────────────
        shield = new_component("OP_Shield_L")
        box(shield, "Shield_Main", -SHOULDER_X-4.0, -1.5, SHOULDER_CTR-2.0, 6.0, 0.50, 8.0, op_blue)
        box(shield, "Shield_Edge", -SHOULDER_X-6.0, -1.5, SHOULDER_CTR-2.0, 2.2, 0.60, 7.2, op_red)
        cyl(shield, "Shield_Rivet_1", -SHOULDER_X-3.5, -1.5, SHOULDER_CTR+1.5, 0.30, 0.15, "y", chrome)
        cyl(shield, "Shield_Rivet_2", -SHOULDER_X-3.5, -1.5, SHOULDER_CTR-5.5, 0.30, 0.15, "y", chrome)

        shield = new_component("OP_Shield_R")
        box(shield, "Shield_Main", SHOULDER_X+4.0, -1.5, SHOULDER_CTR-2.0, 6.0, 0.50, 8.0, op_blue)
        box(shield, "Shield_Edge", SHOULDER_X+6.0, -1.5, SHOULDER_CTR-2.0, 2.2, 0.60, 7.2, op_red)
        cyl(shield, "Shield_Rivet_1", SHOULDER_X+3.5, -1.5, SHOULDER_CTR+1.5, 0.30, 0.15, "y", chrome)
        cyl(shield, "Shield_Rivet_2", SHOULDER_X+3.5, -1.5, SHOULDER_CTR-5.5, 0.30, 0.15, "y", chrome)

        # ── 11. ION BLASTER ──────────────────────────────────────────────
        blaster = new_component("OP_IonBlaster")
        box(blaster, "Blaster_Body", SHOULDER_X+4.0, -1.0, SHOULDER_CTR-6.0, 5.0, 3.6, 8.5, dark_grey)
        box(blaster, "Blaster_Barrel", SHOULDER_X+4.0, -1.0, SHOULDER_CTR-10.5, 3.2, 2.4, 3.6, chrome)
        cyl(blaster, "Blaster_Muzzle", SHOULDER_X+4.0, -1.0, SHOULDER_CTR-12.0, 0.80, 0.30, "z", chrome)
        box(blaster, "Blaster_Grip", SHOULDER_X+4.0, -2.8, SHOULDER_CTR-5.5, 2.0, 1.2, 1.2, op_red)
        box(blaster, "Blaster_Scope", SHOULDER_X+5.8, -1.0, SHOULDER_CTR-8.5, 1.2, 0.60, 2.2, dark_metal)
        led_pocket_5mm(blaster, "Blaster_LED", SHOULDER_X+4.0, -1.0, SHOULDER_CTR-12.2, "z")

        BOM.add("Printed", "Ion Blaster assembly", 1, "PLA/PETG")
        add_print_orientation_note("IonBlaster", "barrel vertical", "yes for undercuts")

        # ═════════════════════════════════════════════════════════════════
        # POST-PROCESSING
        # ═════════════════════════════════════════════════════════════════

        Logger.log("Adding access covers...")
        cover_plate(torso, "LipoBay", 0, 2.8, TORSO_CTR-2.0, 7.4, 0.25, 4.8,
                    boss_positions=[(-3.0,-1.5),(3.0,-1.5),(-3.0,1.5),(3.0,1.5)])
        cover_plate(torso, "RPiBay", 0, 3.2, TORSO_CTR+1.8, 6.8, 0.25, 2.4,
                    boss_positions=[(-2.9,-0.95),(2.9,-0.95),(-2.9,0.95),(2.9,0.95)])
        cover_plate(torso, "PCABay", 0, 3.0, TORSO_CTR+4.2, 6.6, 0.25, 2.0,
                    boss_positions=[(-3.0,-0.88),(3.0,-0.88),(-3.0,0.88),(3.0,0.88)])

        magnet_pocket(head, "FaceCover_L", -1.5, -2.5, HEAD_CTR+0.5, "y")
        magnet_pocket(head, "FaceCover_R",  1.5, -2.5, HEAD_CTR+0.5, "y")

        cover_plate(pelvis, "IMUCover", 0, 0, PELVIS_CTR, 2.4, 0.25, 1.8,
                    boss_positions=[(-0.9,-0.65),(0.9,-0.65),(-0.9,0.65),(0.9,0.65)])

        Logger.log("Merging fasteners to split halves...")
        for comp in comps_list:
            to_split = [b for b in comp.bRepBodies if b.name and any(k in b.name for k in SPLIT_KEYS)]
            for b in to_split:
                split_halves(comp, b, "y", 0.0)
            merge_fasteners_to_halves(comp)

        for z_pos in [TORSO_CTR, THIGH_CTR, SHIN_CTR]:
            cable_clip(torso, f"SpineClip_{int(z_pos*10)}", 0, 0, z_pos)

        assembly_jig(torso, "TorsoAlign", 0, 0, TORSO_CTR, 10.8, 1.0, 1.0)

        transformation_latch(torso, "RobotLock_F", 0, -2.0, TORSO_CTR-1.0, "y")
        transformation_latch(torso, "TruckLock_R", 0, 2.0, TORSO_CTR+1.0, "y")

        for side, sx in [("L", -SHOULDER_X), ("R", SHOULDER_X)]:
            m = -1 if side=="L" else 1
            hand = occs.get(f"OP_Hand_{side}").component
            for px in [-0.8, 0.8]:
                idler_pulley(hand, f"{side}_Pulley_{px:.1f}", sx+m*px, -1.3, WRIST_Z-2.0, "x")
            servo_drum(hand, f"{side}_Drum", sx, 0.5, WRIST_Z-2.2, "x")
            for fi, (fname, fxo, pp_l, mp_l, dp_l) in enumerate(
                    zip(FING_NAMES, FING_X_OFF, FING_PP, FING_MP, FING_DP)):
                fx = sx + m * fxo
                mcp_z = WRIST_Z - PALM_BOTTOM_OFFSET
                pp_cz = mcp_z - pp_l/2
                tendon_groove(occs[f"OP_{fname}_{side}"].component, fname, fx, -1.15, pp_cz, pp_l, "z")
                tendon_anchor(occs[f"OP_{fname}_{side}"].component, f"{fname}_Anchor",
                              fx, -1.15, mcp_z - pp_l - mp_l - dp_l - 0.1)
            tx = sx + m * 1.70
            thumb_comp = occs[f"OP_Thumb_{side}"].component
            tendon_groove(thumb_comp, f"{side}_Thumb", tx, 0.4, WRIST_Z - PALM_BOTTOM_OFFSET - THUMB_PP_L/2, THUMB_PP_L+THUMB_DP_L, "z")
            tendon_anchor(thumb_comp, f"{side}_ThumbAnchor", tx, 0.4, WRIST_Z - PALM_BOTTOM_OFFSET - THUMB_PP_L - THUMB_DP_L - 0.15)

        hard_stop(torso, "WaistPitchLimit", 0, 0, WAIST_CTR-2.5, "x", 0)

        box(torso, "PowerSwitchSlot", 0, 4.0, TORSO_CTR-2.0, 1.5, 1.2, 1.0, dark_grey)
        BOM.add("Electronics", "Rocker switch SPST", 1, "torso")

        # ═════════════════════════════════════════════════════════════════
        # OVERHANG ANALYSIS
        # ═════════════════════════════════════════════════════════════════
        Logger.log("--- OVERHANG ANALYSIS ---")
        overhang_parts = ["Stk_A_", "StkTip_", "Chin_Guard", "Heel_Spur",
                          "Toe_Cap", "Sh_Pad_Outer", "EarTop_", "BP_TopFlap"]
        for c in comps_list:
            for body in c.bRepBodies:
                if any(p in (body.name or "") for p in overhang_parts):
                    Logger.log(f"  WARNING: {c.name}::{body.name} likely needs supports.")

        # ═════════════════════════════════════════════════════════════════
        # GENERATE BUILD DOCUMENTATION
        # ═════════════════════════════════════════════════════════════════
        BuildGuide.add("Torso", "Assemble battery, RPi, PCA9685 into bays. Secure with M3 screws.")
        BuildGuide.add("Legs", "Hip to thigh to shin to foot. Verify MG996R centering.")
        BuildGuide.add("Arms", "Shoulder to upper arm to elbow to forearm. Route wires through clips.")
        BuildGuide.add("Head", "Install ESP32-CAM, connect to neck harness.")
        BuildGuide.add("Hands", "Thread tendons through finger grooves, tie to anchors.")
        BuildGuide.add("Final", "Full ROM test, then close all covers.")
        BuildGuide.save(GUIDE_FILE)

        BuildDocs.set_print_orientation("Torso Shell", "split on Y=0 plane", "minimal", "PETG")
        BuildDocs.set_print_orientation("Leg Parts", "upright (Z-axis up)", "minimal", "PETG")
        BuildDocs.set_print_orientation("Finger Segments", "flat on build plate", "none", "PLA")
        BuildDocs.generate(BUILD_DOC_FILE)

        BOM.check_fastener_lengths()
        BOM.save_csv(BOM_FILE)
        BOM.summary()
        FastenerValidator.report()

        # ═════════════════════════════════════════════════════════════════
        # JOINTS & KINEMATICS
        # ═════════════════════════════════════════════════════════════════
        # Waist
        revolute_joint("Waist_Yaw", occs.get("OP_Torso"), occs.get("OP_Torso"),
                       0, 0, WAIST_CTR, "z")
        revolute_joint("Waist_Pitch", occs.get("OP_Torso"), occs.get("OP_Torso"),
                       0, 0, WAIST_CTR-2.5, "x")

        # Neck
        revolute_joint("Neck_Pitch", occs.get("OP_Torso"), occs.get("OP_Head"),
                       0, 0, NECK_JOINT_Z, "x")

        # Hips
        for side, sx in [("L", -HIP_X), ("R", HIP_X)]:
            revolute_joint(f"Hip_{side}_Yaw", occs.get("OP_Pelvis"),
                           occs.get(f"OP_Leg_{side}"), sx, 0, HIP_JOINT_Z, "z")
            revolute_joint(f"Hip_{side}_Pitch", occs.get(f"OP_Leg_{side}"),
                           occs.get(f"OP_Leg_{side}"), sx, 0, THIGH_CTR, "x")

        # Knees
        for side, sx in [("L", -HIP_X), ("R", HIP_X)]:
            revolute_joint(f"Knee_{side}", occs.get(f"OP_Leg_{side}"),
                           occs.get(f"OP_Leg_{side}"), sx, 0, KNEE_CTR, "x")

        # Ankles
        for side, sx in [("L", -HIP_X), ("R", HIP_X)]:
            revolute_joint(f"Ankle_{side}", occs.get(f"OP_Leg_{side}"),
                           occs.get(f"OP_Leg_{side}"), sx, 0, ANKLE_CTR, "x")

        # Shoulders
        for side, sx in [("L", -SHOULDER_X), ("R", SHOULDER_X)]:
            revolute_joint(f"Shoulder_{side}", occs.get("OP_Torso"),
                           occs.get(f"OP_Arm_{side}"), sx, 0, SHOULDER_CTR-2.0, "y")

        # Elbows
        for side, sx in [("L", -SHOULDER_X), ("R", SHOULDER_X)]:
            revolute_joint(f"Elbow_{side}", occs.get(f"OP_Arm_{side}"),
                           occs.get(f"OP_Arm_{side}"), sx, 0, ELBOW_Z, "x")

        # Wrists
        for side, sx in [("L", -SHOULDER_X), ("R", SHOULDER_X)]:
            revolute_joint(f"Wrist_{side}", occs.get(f"OP_Hand_{side}"),
                           occs.get(f"OP_Hand_{side}"), sx, 0, WRIST_Z, "x")

        Logger.log(f"Total components: {len(comps_list)}")
        Logger.log("v12 script completed successfully.")

    except Exception:
        Logger.log(f"FATAL ERROR:\n{traceback.format_exc()}", "ERROR")
    finally:
        Logger.flush()
