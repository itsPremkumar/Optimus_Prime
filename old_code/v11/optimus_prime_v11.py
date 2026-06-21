# ─────────────────────────────────────────────────────────────────────────────
# OPTIMUS PRIME G1  v11.0  ─ Physical Build Edition
# Fusion 360 Python Script  |  Build-Ready Robot Platform
# ─────────────────────────────────────────────────────────────────────────────
# MAJOR ADDITIONS IN v11  (relative to v10)
# ─────────────────────────────────────────────────────────────────────────────
# PHY-10  Servo-Horn / Output-Shaft Coupling System
# PHY-11  Tendon-Driven Finger Actuation
# PHY-12  Access Covers & Battery Doors with Retention
# PHY-13  Fastener Merging After Shell Splitting
# PHY-14  Printability & Overhang Optimization
# PHY-15  Cable Management & Wire Routing System
# PHY-16  Bearing Press-Fit & Retention
# PHY-17  Assembly Jigs & Alignment Aids
# PHY-18  Fastener Length Verification
# PHY-19  Build Documentation & Assembly Guide Generation
#
# MECH-1  Internal structural skeleton reinforcement
# MECH-2  Stronger hip/knee/ankle/shoulder/waist supports
# MECH-3  Metal shaft / bearing interfaces
# MECH-4  Servo mount reinforcement
# MECH-5  Mechanical hard stops
# MECH-6  Replaceable wear parts
# MECH-7  Transformation locking pins/latch points
# MECH-8  Stronger feet with improved ground contact
# MECH-9  Center-of-mass optimization
# MECH-10 Stable torso and pelvis support
#
# POWR-1  Battery retention geometry
# POWR-2  Fuse / protection space
# POWR-3  Emergency cutoff switch location
# POWR-4  Separate logic power and servo power routing
# POWR-5  Connector locations and strain relief
# POWR-6  Service access for battery replacement
#
# ELEC-7  Mounting standoffs for all boards
# ELEC-8  Realistic board clearances
# ELEC-9  Cable exits and connector cutouts
# ELEC-10 SD card / USB / charging access
# ELEC-11 Safe routing around moving parts
#
# TF-1    Locking features for robot and truck mode
# TF-2    Alignment guides for transformation
# TF-3    Release/engage geometry
# TF-4    Mechanical stops and lock points
# TF-5    Collision-free shell movement
#
# DOC-1   Assembly guide generation
# DOC-2   Print orientation notes
# DOC-3   Support requirements
# DOC-4   Assembly order
# DOC-5   Fastener mapping
# DOC-6   Electronics installation order
# DOC-7   Servo installation order
# DOC-8   BOM CSV export with detailed hardware
# DOC-9   Build notes generation
# ─────────────────────────────────────────────────────────────────────────────

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

import adsk.core
import adsk.fusion
import traceback
import math
import os
import csv
import json
import datetime
import time

_ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

try:
    SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
except NameError:
    SCRIPT_DIR = os.getcwd()
PROJECT_DIR    = os.path.dirname(os.path.dirname(SCRIPT_DIR))
_OUTPUT_DIR    = os.path.join(PROJECT_DIR, "output")
LOG_DIR        = globals().get("LOG_DIR")        or os.path.join(_OUTPUT_DIR, "logs")
SCREENSHOT_DIR = globals().get("SCREENSHOT_DIR") or os.path.join(_OUTPUT_DIR, "screenshots")
EXPORT_DIR     = globals().get("EXPORT_DIR")     or os.path.join(_OUTPUT_DIR, "exports")
JIG_DIR        = os.path.join(EXPORT_DIR, "jigs")
DOCS_DIR       = os.path.join(_OUTPUT_DIR, "build_docs")
LOG_FILE       = os.path.join(LOG_DIR,     f"optimus_v11_{_ts}.txt")
BOM_FILE       = os.path.join(_OUTPUT_DIR, f"BOM_v11_{_ts}.csv")
GUIDE_FILE     = os.path.join(DOCS_DIR,    f"assembly_guide_v11_{_ts}.txt")
STOP_FLAG      = os.path.join(_OUTPUT_DIR, "stop.flag")

CLEARANCE    = 0.060
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

WALL_S       = 0.30
WALL_P       = 0.20
WALL_F       = 0.15
WALL_THIN    = 0.12
CHAMFER_D    = 0.10

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
    "M2x6":    0.60, "M2x8":    0.80, "M2x10":   1.00,
    "M2.5x6":  0.60, "M2.5x8":  0.80, "M2.5x10": 1.00,
    "M3x6":    0.60, "M3x8":    0.80, "M3x10":   1.00,
    "M3x12":   1.20, "M3x14":   1.40, "M3x16":   1.60,
    "M3x18":   1.80, "M3x20":   2.00, "M3x25":   2.50,
    "M3x30":   3.00,
}
SCREW_GRIP_RATIO = 0.65

BEARING_PRESS = 0.002
BEARING_GLUE  = 0.015
BEARING_RET_LIP = 0.025

RPI0_L,  RPI0_W,  RPI0_H  = 6.50, 3.00, 0.20
PCA_L,   PCA_W,   PCA_H   = 6.25, 2.54, 0.18
ESP32_L, ESP32_W, ESP32_H = 3.80, 2.58, 0.15
IMU_L,   IMU_W,   IMU_H   = 2.10, 1.60, 0.12
LIPO_L,  LIPO_W,  LIPO_H  = 7.00, 3.20, 1.80
LIPO_SLACK = 0.30
XT60_W,  XT60_H_SLOT       = 1.60, 1.30
BEC_L,   BEC_W,   BEC_H    = 3.50, 2.20, 0.80
SWITCH_R                     = 0.60
FUSE_HOLDER_L, FUSE_HOLDER_W = 2.50, 1.20
LED_R_5MM                  = 0.260
LED_R_RING                 = 0.600

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
TENDON_CH_DIAM = 0.040
TENDON_CH_GROOVE = 0.025

SERVO_SPECS = {
    "hip_hd": {"name": "DS3225MG", "rated": 25.0, "stall": 35.0, "mass_g":  60},
    "waist":  {"name": "DS3218",   "rated": 20.0, "stall": 25.0, "mass_g":  55},
    "std":    {"name": "MG996R",   "rated":  9.4, "stall": 11.5, "mass_g":  55},
    "micro":  {"name": "MG90S",    "rated":  1.8, "stall":  2.2, "mass_g":  13},
    "digit":  {"name": "DS04-NFC", "rated":  1.8, "stall":  2.2, "mass_g":   9},
}

SERVO_COUPLER = {
    "DS3225MG": {"spline_r": 0.30, "spline_n": 25, "hub_r": 0.70, "hub_h": 0.60, "set_screw": True, "bore_r": 0.16, "flange_od": 1.60},
    "DS3218":   {"spline_r": 0.28, "spline_n": 25, "hub_r": 0.65, "hub_h": 0.55, "set_screw": True, "bore_r": 0.16, "flange_od": 1.50},
    "MG996R":   {"spline_r": 0.25, "spline_n": 25, "hub_r": 0.60, "hub_h": 0.50, "set_screw": True, "bore_r": 0.16, "flange_od": 1.40},
    "MG90S":    {"spline_r": 0.18, "spline_n": 25, "hub_r": 0.45, "hub_h": 0.40, "set_screw": False, "bore_r": 0.12, "flange_od": 1.00},
    "DS04-NFC": {"spline_r": 0.15, "spline_n": 25, "hub_r": 0.40, "hub_h": 0.35, "set_screw": False, "bore_r": 0.10, "flange_od": 0.85},
}
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
              "Tendon", "Pulley", "CableClip", "Stop", "AlignJig"}
MERGE_TAGS = {"Pin", "Boss", "Insert", "Nut", "Snap", "Standoff", "Retainer"}

PRINT_NOTES = []

# ── Structural / Mechanical Constants (Phase 1 additions) ──────────
SHAFT_DIAM_MAJOR = 0.50
SHAFT_DIAM_MED   = 0.40
SHAFT_DIAM_MINOR = 0.30
SHAFT_DIAM_PIN   = 0.20
FLANGE_BRG_MAJOR_ID  = 0.50; FLANGE_BRG_MAJOR_OD  = 1.00; FLANGE_BRG_MAJOR_W  = 0.40
FLANGE_BRG_MED_ID    = 0.40; FLANGE_BRG_MED_OD    = 0.80; FLANGE_BRG_MED_W    = 0.32
FLANGE_BRG_MINOR_ID  = 0.30; FLANGE_BRG_MINOR_OD  = 0.60; FLANGE_BRG_MINOR_W  = 0.26
FLANGE_BRG_FLANGE_OD = 1.20; FLANGE_BRG_FLANGE_T  = 0.08
BEARING_BLOCK_W = 0.80; BEARING_BLOCK_H = 0.80
SET_SCREW_R = 0.145; SET_SCREW_L = 0.40
EXTRUSION_W = 2.00; EXTRUSION_SLOT_W = 0.60; EXTRUSION_SLOT_D = 0.30
EXTRUSION_T_SLOT_R = 0.35; EXTRUSION_CENTER_BORE = 0.30
FRAME_BRACKET_T = 0.30; FRAME_BRACKET_W = 1.20
WORM_MODULE = 0.10; WORM_START = 1; WORM_ADDENDUM = 0.10; WORM_DEDDENDUM = 0.12
GEAR_RATIO_WAIST = 30; GEAR_RATIO_HIP = 4; GEAR_RATIO_KNEE = 3
PLANET_STAGE_H = 1.20; PLANET_STAGE_R = 1.40; PLANET_SUN_R = 0.30; PLANET_GEAR_R = 0.45
LATCH_SOLENOID_D = 1.20; LATCH_SOLENOID_L = 3.00
LATCH_HOOK_L = 1.20; LATCH_HOOK_D = 0.30; LATCH_POST_D = 0.40; LATCH_POST_H = 1.00
ALIGN_CAM_H = 0.60; ALIGN_CAM_W = 0.80; ALIGN_CAM_ANGLE = 20
QR_PIN_D = 0.60; QR_COLLAR_D = 1.40; QR_COLLAR_L = 0.80; QR_DETENT_BALL_R = 0.125
QR_SPRING_D = 0.30; QR_SPRING_L = 0.60
SPINE_SEG_W = 2.00; SPINE_SEG_H = 0.80; SPINE_SEG_D = 1.50
SPINE_NERVE_CH_D = 0.80; SPINE_BALL_D = 0.50; SPINE_SOCKET_D = 0.55
BELLOWS_WALL = 0.08; BELLOWS_RIB_H = 0.15
# ── End structural constants ───────────────────────────────────────

class Logger:
    _buffer = []
    _count  = 0

    @classmethod
    def log(cls, msg, level="INFO"):
        ts   = datetime.datetime.now().strftime("%H:%M:%S")
        line = f"[{ts}] [{level}] {msg}\n"
        safe = line.encode("ascii", "replace").decode("ascii")
        print(safe, end="", flush=True)
        cls._buffer.append(line)
        cls._count += 1
        if cls._count >= 20:
            cls.flush()

    @classmethod
    def flush(cls):
        if not cls._buffer:
            return
        try:
            os.makedirs(LOG_DIR, exist_ok=True)
            with open(LOG_FILE, "a", encoding="utf-8") as f:
                f.write("".join(cls._buffer))
        except Exception:
            pass
        cls._buffer.clear()
        cls._count = 0

    @classmethod
    def print_note(cls, msg):
        cls.log(msg, "NOTE")
        PRINT_NOTES.append(msg)

class BOM:
    _rows = []
    @classmethod
    def add(cls, category, part_name, qty, note=""):
        cls._rows.append({"Category": category, "Part": part_name, "Qty": qty, "Note": note})
    @classmethod
    def save_csv(cls, path):
        if not cls._rows:
            return
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, "w", newline="", encoding="utf-8") as f:
                writer = csv.DictWriter(f, fieldnames=["Category", "Part", "Qty", "Note"])
                writer.writeheader()
                writer.writerows(cls._rows)
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

class BuildGuide:
    _entries = []
    @classmethod
    def add(cls, section, content):
        cls._entries.append((section, content))
    @classmethod
    def generate(cls):
        guide = []
        guide.append("=" * 65)
        guide.append("OPTIMUS PRIME G1 v11 — PHYSICAL BUILD ASSEMBLY GUIDE")
        guide.append(f"Generated: {datetime.datetime.now().strftime('%Y-%m-%d %H:%M')}")
        guide.append("=" * 65)
        guide.append("")
        guide.append("--- PRINTED PART SUMMARY ---")
        printed = BOM.by_category().get("Printed Part", [])
        for p in printed:
            guide.append(f"  {p['Qty']}x {p['Part']:35s} {p['Note']}")
        guide.append("")
        guide.append("--- PRINT ORIENTATION ---")
        for note in PRINT_NOTES:
            guide.append(f"  {note}")
        guide.append("")
        guide.append("--- ASSEMBLY ORDER ---")
        for section, content in cls._entries:
            guide.append("")
            guide.append(f"  [{section}]")
            for line in content.split("\n"):
                guide.append(f"    {line.strip()}")
        guide.append("")
        guide.append("--- FASTENER MAP ---")
        fasteners = BOM.by_category().get("Fastener", [])
        for f in fasteners:
            guide.append(f"  {f['Qty']}x {f['Part']:30s} {f['Note']}")
        guide.append("")
        guide.append("--- ELECTRONICS INSTALLATION ORDER ---")
        guide.append("  1. Install PCA9685 on standoffs in torso")
        guide.append("  2. Route servo harnesses to PCA9685")
        guide.append("  3. Install RPi Zero 2W on standoffs")
        guide.append("  4. Connect I2C (PCA9685), USB (ESP32-CAM)")
        guide.append("  5. Install ESP32-CAM in head, route lens channel")
        guide.append("  6. Install MPU6050 in pelvis")
        guide.append("  7. Wire BEC: input from LiPo, output 5V to RPi/PCA9685")
        guide.append("  8. Wire emergency cutoff switch in-line with battery")
        guide.append("  9. Install fuse holder")
        guide.append(" 10. Connect battery, test power-up before servo load")
        guide.append("")
        guide.append("--- SERVO INSTALLATION ORDER ---")
        guide.append("  1. Install and center all servos at 90 deg neutral")
        guide.append("  2. Attach couplers/horns with set screws")
        guide.append("  3. Install bearings with press-fit or retaining lip")
        guide.append("  4. Route servo wires with cable clips")
        guide.append("  5. Connect to PCA9685 per wiring map")
        guide.append("  6. Test each channel individually before load")
        guide.append("")
        guide.append("--- WARNINGS & NOTES ---")
        guide.append("  - All dimensions in cm; multiply by 10 for mm")
        guide.append("  - PLA is acceptable for non-structural parts")
        guide.append("  - PETG recommended for joints, brackets, frames")
        guide.append("  - Supports required for overhangs >45 deg")
        guide.append("  - Test-fit bearings before final assembly")
        guide.append("  - Use threadlocker on all metal-to-metal fasteners")
        guide.append("  - Do not overtighten heat-set inserts")
        guide.append("  - Center all servos before attaching couplers")
        guide.append("  - Battery must be removed during assembly/servicing")
        guide.append("")
        guide.append("--- POWER SYSTEM ---")
        guide.append("  Battery: 2S 1300 mAh LiPo (7.4V)")
        guide.append("  Servo power: Direct from LiPo via BEC (5V 5A)")
        guide.append("  Logic power: Separate BEC (5V 2A) for RPi")
        guide.append("  Fuse: 10A inline ATO blade fuse")
        guide.append("  Cutoff: SPST toggle switch (rated 15A+)")
        guide.append("  Estimated runtime: ~30 min light duty")
        guide.append("")
        return "\n".join(guide)
    @classmethod
    def save(cls, path):
        content = cls.generate()
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, "w", encoding="utf-8") as f:
                f.write(content)
            Logger.log(f"Build guide saved -> {path}")
        except Exception as e:
            Logger.log(f"Build guide save failed: {e}", "WARN")
        return content

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
        screw_d   = float(screw_name.split("x")[0].replace("M", "")) / 10.0
        min_grip  = screw_d * SCREW_GRIP_RATIO
        available_grip = total_len - material_thickness - 0.05
        if available_grip < min_grip:
            cls._warnings.append(f"  [{screw_name} @ {location}] need {min_grip:.2f} cm grip, available {available_grip:.2f} cm (thickness {material_thickness:.2f} cm) — TOO SHORT")
            return False
        if available_grip > nut_depth + 0.10:
            cls._warnings.append(f"  [{screw_name} @ {location}] available {available_grip:.2f} cm > nut depth {nut_depth:.2f} cm — MAY BOTTOM OUT")
            return False
        return True
    @classmethod
    def report(cls):
        for w in cls._warnings:
            Logger.log(w, "WARN")
        if not cls._warnings:
            Logger.log("Fastener validation: [OK] All lengths acceptable")
        return cls._warnings
def run(context):
    global TARGET_MODULE, EXPORT_STL, EXPORT_STEP, EXPORT_URDF, EXPORT_DIR
    app = None
    ui  = None
    Logger.log("=" * 60)
    Logger.log("EXECUTION START  v11.0 — Optimus Prime G1 Physical Build Edition")
    Logger.log("=" * 60)
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface
        try:
            app.preferences.generalPreferences.defaultModelingOrientation = (
                adsk.core.DefaultModelingOrientations.ZUpModelingOrientation)
        except Exception:
            pass
        # Ensure Model workspace is active
        try:
            mws = ui.workspaces.itemById('FusionSolidEnvironment')
            if mws:
                mws.activate()
        except Exception:
            pass
        doc    = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
        design = adsk.fusion.Design.cast(app.activeProduct)
        root   = design.rootComponent
        app_lib = None
        for i in range(app.materialLibraries.count):
            lib = app.materialLibraries.item(i)
            if "Appearance" in lib.name:
                app_lib = lib
                break
        def _copy_ap(query):
            if not app_lib:
                return None
            for i in range(app_lib.appearances.count):
                ap = app_lib.appearances.item(i)
                if query.lower() in ap.name.lower():
                    try:
                        return design.appearances.addByCopy(ap)
                    except Exception:
                        return ap
            return None
        def get_ap(primary, *fallbacks):
            ap = _copy_ap(primary)
            if ap:
                return ap
            for fb in fallbacks:
                ap = _copy_ap(fb)
                if ap:
                    return ap
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
        trans_orange  = get_ap("Acrylic - Orange Transparent",  "Paint - Glossy (Orange)")
        comps_list = []
        occs       = {}
        jig_comps  = []
        def new_component(name):
            occ  = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
            comp = occ.component
            comp.name = name
            comps_list.append(comp)
            occs[name] = occ
            return comp
        def new_jig_component(name):
            occ  = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
            comp = occ.component
            comp.name = name
            comps_list.append(comp)
            occs[name] = occ
            jig_comps.append(comp)
            return comp
        def set_ap(body, ap):
            if body and ap:
                try:
                    body.appearance = ap
                except Exception:
                    pass
        def box(comp, name, cx, cy, cz, lx, ly, lz, ap=None):
            temp  = adsk.fusion.TemporaryBRepManager.get()
            obb   = adsk.core.OrientedBoundingBox3D.create(
                        adsk.core.Point3D.create(cx, cy, cz),
                        adsk.core.Vector3D.create(1, 0, 0),
                        adsk.core.Vector3D.create(0, 1, 0),
                        lx, ly, lz)
            shape = temp.createBox(obb)
            bf    = comp.features.baseFeatures.add()
            bf.startEdit()
            body  = comp.bRepBodies.add(shape, bf)
            bf.finishEdit()
            body.name = name
            set_ap(body, ap)
            return body
        def cyl(comp, name, cx, cy, cz, r, h, axis, ap=None):
            temp = adsk.fusion.TemporaryBRepManager.get()
            ax   = {"x": (1, 0, 0), "y": (0, 1, 0), "z": (0, 0, 1)}[axis]
            p1   = adsk.core.Point3D.create(cx-ax[0]*h/2, cy-ax[1]*h/2, cz-ax[2]*h/2)
            p2   = adsk.core.Point3D.create(cx+ax[0]*h/2, cy+ax[1]*h/2, cz+ax[2]*h/2)
            shape = temp.createCylinderOrCone(p1, r, p2, r)
            bf    = comp.features.baseFeatures.add()
            bf.startEdit()
            body  = comp.bRepBodies.add(shape, bf)
            bf.finishEdit()
            body.name = name
            set_ap(body, ap)
            return body
        def cone_shape(comp, name, cx, cy, cz, r1, r2, h, axis, ap=None):
            temp  = adsk.fusion.TemporaryBRepManager.get()
            ax    = {"x": (1, 0, 0), "y": (0, 1, 0), "z": (0, 0, 1)}[axis]
            p1    = adsk.core.Point3D.create(cx-ax[0]*h/2, cy-ax[1]*h/2, cz-ax[2]*h/2)
            p2    = adsk.core.Point3D.create(cx+ax[0]*h/2, cy+ax[1]*h/2, cz+ax[2]*h/2)
            shape = temp.createCylinderOrCone(p1, r1, p2, r2)
            bf    = comp.features.baseFeatures.add()
            bf.startEdit()
            body  = comp.bRepBodies.add(shape, bf)
            bf.finishEdit()
            body.name = name
            set_ap(body, ap)
            return body
        def marker(comp, name, cx, cy, cz, size=0.22):
            return box(comp, name, cx, cy, cz, size, size, size, white_pla)
        def cut_cavity(comp, tool_body, keep_tool=False):
            tools = adsk.core.ObjectCollection.create()
            tools.add(tool_body)
            for b in list(comp.bRepBodies):
                if b == tool_body:
                    continue
                if b.name and any(t in b.name for t in SKIP_TAGS):
                    continue
                try:
                    ci = comp.features.combineFeatures.createInput(b, tools)
                    ci.operation        = adsk.fusion.CombineOperation.CutFeatureOperation
                    ci.isKeepToolBodies = True
                    comp.features.combineFeatures.add(ci)
                except Exception:
                    pass
            if not keep_tool:
                try:
                    tool_body.isLightBulbOn = False
                    if "_Vis" not in tool_body.name:
                        tool_body.name += "_Vis"
                except Exception:
                    pass
        def merge_bodies(comp, target_body, source_body):
            try:
                tools = adsk.core.ObjectCollection.create()
                tools.add(source_body)
                ci = comp.features.combineFeatures.createInput(target_body, tools)
                ci.operation = adsk.fusion.CombineOperation.JoinFeatureOperation
                ci.isKeepToolBodies = False
                comp.features.combineFeatures.add(ci)
                return target_body
            except Exception:
                try:
                    source_body.isLightBulbOn = False
                except Exception:
                    pass
                return target_body
        def split_halves(comp, body, axis="y", offset=0.0):
            try:
                planes = comp.constructionPlanes
                pi     = planes.createInput()
                ref    = (root.xYConstructionPlane if axis == "z" else
                          root.xZConstructionPlane if axis == "y" else
                          root.yZConstructionPlane)
                pi.setByOffset(ref, adsk.core.ValueInput.createByReal(offset))
                sp          = planes.add(pi)
                split_input = comp.features.splitBodyFeatures.createInput(body, sp, True)
                comp.features.splitBodyFeatures.add(split_input)
            except Exception:
                pass
        def m3_boss(comp, tag, cx, cy, cz, axis="z", depth=0.80):
            boss = cyl(comp, f"{tag}_Boss", cx, cy, cz, BOSS_R, depth, axis, dark_metal)
            boss.name = f"{tag}_Boss"
            cut_cavity(comp, cyl(comp, f"{tag}_Insert", cx, cy, cz, INSERT_R, INSERT_H, axis))
            BOM.add("Fastener", "M3 heat-set insert (Voron M3x5)", 1, f"boss at ({cx:.1f},{cy:.1f},{cz:.1f}) in {comp.name}")
            return boss
        def captive_nut(comp, tag, cx, cy, cz, axis="z", bolt_len=1.0):
            cut_cavity(comp, cyl(comp, f"{tag}_NutTrap", cx, cy, cz, M3_NUT_CIR, M3_NUT_H, axis))
            cut_cavity(comp, cyl(comp, f"{tag}_NutBore", cx, cy, cz, M3_CLR_R, bolt_len, axis))
            BOM.add("Fastener", "M3 hex nut", 1, comp.name)
            BOM.add("Fastener", f"M3x{int(bolt_len*10)} SHCS", 1, comp.name)
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
            BOM.add("Printed Part", "Grommet (TPU print)", 1, comp.name)
        def screw_hole(comp, cx, cy, cz, axis="y", length=3.0):
            cut_cavity(comp, cyl(comp, "ScrewHole", cx, cy, cz, M3_PILOT_R, length, axis))
            BOM.add("Fastener", f"M3x{int(length*10)} self-tap", 1, comp.name)
        def magnet_pocket(comp, tag, cx, cy, cz, axis="z"):
            cut_cavity(comp, cyl(comp, f"{tag}_MagPkt", cx, cy, cz, 0.32, 0.35, axis))
            BOM.add("Hardware", "Magnet D6x3 mm N35", 1, comp.name)
        def wire_channel(comp, tag, cx, cy, cz, r, h, axis):
            cut_cavity(comp, cyl(comp, f"{tag}_Wire", cx, cy, cz, r, h, axis))
        def led_pocket_5mm(comp, tag, cx, cy, cz, axis="y"):
            cut_cavity(comp, cyl(comp, f"{tag}_LED5", cx, cy, cz, LED_R_5MM, 0.85, axis))
            BOM.add("Electronics", "LED 5 mm (colour TBD)", 1, comp.name)
        def led_ring_pocket(comp, tag, cx, cy, cz, axis="z"):
            cut_cavity(comp, cyl(comp, f"{tag}_LEDRing", cx, cy, cz, LED_R_RING, 0.30, axis))
            BOM.add("Electronics", "WS2812 5050 LED ring 12 mm", 1, comp.name)

        # PHY-10: Servo-Horn Coupling
        def horn_coupler(comp, tag, cx, cy, cz, axis, servo_type="MG996R"):
            spec = SERVO_COUPLER.get(servo_type, SERVO_COUPLER["MG996R"])
            hr   = spec["hub_r"]; hh = spec["hub_h"]; sr = spec["spline_r"]; br = spec["bore_r"]
            coupler_occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
            coupler = coupler_occ.component; coupler.name = f"{tag}_Coupler"
            comps_list.append(coupler); occs[coupler.name] = coupler_occ
            cyl_body = cyl(coupler, "Hub", cx, cy, cz, hr, hh, axis, chrome)
            cyl_body.name = f"{tag}_CouplerHub"
            sw = sr * 0.6
            if axis == "x":
                s1 = box(coupler, "SplineBar1", cx, cy, cz, hh*0.8, sw*0.8, sw*0.8, chrome)
                s2 = box(coupler, "SplineBar2", cx, cy, cz, sw*0.8, sw*0.8, hh*0.8, chrome)
            elif axis == "y":
                s1 = box(coupler, "SplineBar1", cx, cy, cz, sw*0.8, hh*0.8, sw*0.8, chrome)
                s2 = box(coupler, "SplineBar2", cx, cy, cz, hh*0.8, sw*0.8, sw*0.8, chrome)
            else:
                s1 = box(coupler, "SplineBar1", cx, cy, cz, sw*0.8, sw*0.8, hh*0.8, chrome)
                s2 = box(coupler, "SplineBar2", cx, cy, cz, sw*0.8, hh*0.8, sw*0.8, chrome)
            if cyl_body:
                merge_bodies(coupler, cyl_body, s1); merge_bodies(coupler, cyl_body, s2)
            ss_r = br * 0.9; ss_len = hr * 1.2
            if axis == "x":
                ss_body = cyl(coupler, "SetScrewPkt", cx, cy+hr*0.5, cz, ss_r, ss_len, "y")
            elif axis == "y":
                ss_body = cyl(coupler, "SetScrewPkt", cx+hr*0.5, cy, cz, ss_r, ss_len, "x")
            else:
                ss_body = cyl(coupler, "SetScrewPkt", cx, cy+hr*0.5, cz, ss_r, ss_len, "y")
            bore_body = cyl(coupler, "Bore", cx, cy, cz, br+0.01, hh+0.10, axis)
            cut_cavity(coupler, bore_body)
            if ss_body: cut_cavity(coupler, ss_body)
            rigid_joint(f"{tag}_CouplerMount", occs[coupler.name], occs.get(comp.name))
            BOM.add("Printed Part", f"Coupler {servo_type} ({tag})", 1, f"PLA/PETG, set-screw M3x4, at ({cx:.1f},{cy:.1f},{cz:.1f})")
            if spec["set_screw"]: BOM.add("Fastener", "M3x4 set screw (cup point)", 1, f"coupler {tag}")
            return coupler

        # PHY-11: Tendon finger system
        def tendon_finger_system(comp_hand, tag, fx, fy, finger_z_start, pp_l, mp_l, dp_l, side_sign=1):
            for seg_name, seg_z, seg_l in [
                    ("PP", finger_z_start - pp_l/2, pp_l),
                    ("MP", finger_z_start - pp_l - mp_l/2, mp_l),
                    ("DP", finger_z_start - pp_l - mp_l - dp_l/2, dp_l)]:
                if seg_l <= 0: continue
                ch = cyl(comp_hand, f"{tag}_TCh_{seg_name}", fx, fy, seg_z, TENDON_CH_DIAM, seg_l*1.5, "z")
                cut_cavity(comp_hand, ch)
            cyl(comp_hand, f"{tag}_Pulley", fx, fy, finger_z_start + 0.10, 0.15, 0.12, "x", chrome)
            BOM.add("Hardware", "Tendon: 0.4 mm fishing line / Dyneema", 30, "cm per finger")
            BOM.add("Hardware", "Spring return (optional, 0.3 mm wire)", 1, f"finger {tag}")

        # PHY-12: Access covers
        def make_access_cover(comp_parent, tag, cx, cy, cz, lx, ly, lz, ap=None, screw_locations=None, hinge_axis=None):
            cover = new_component(f"{tag}_Cover")
            ap = ap or dark_grey
            cover_body = box(cover, f"{tag}_CoverBody", cx, cy, cz, lx, ly, lz, ap)
            cover_body.name = f"{tag}_CoverBody"
            if screw_locations:
                for sc_i, (scx, scy, scz) in enumerate(screw_locations):
                    m3_boss(cover, f"{tag}_CvrBoss_{sc_i}", scx, scy, scz, depth=0.40)
                    screw_hole(cover, scx, scy, scz, axis="z", length=0.60)
            if not screw_locations and not hinge_axis:
                snap_clip(comp_parent, f"{tag}_DoorClip", cx, cy, cz, span_x=lx*0.6)
            if lx > 2.0 and ly > 2.0:
                for mx in [cx - lx*0.3, cx + lx*0.3]:
                    if abs(mx - cx) > 0.5: magnet_pocket(cover, f"{tag}_Mag_{int(mx*10)}", mx, cy, cz)
            BOM.add("Printed Part", f"Access cover {tag} ({lx*10:.0f}x{ly*10:.0f} mm)", 1, f"PLA/PETG, {comp_parent.name}")
            rigid_joint(f"{tag}_CoverMount", occs.get(cover.name), occs.get(comp_parent.name))
            return cover

        # PHY-13: Fastener merging
        def merge_fasteners_to_halves(comp, body_left, body_right):
            try: left_bbox = body_left.boundingBox; right_bbox = body_right.boundingBox
            except Exception: return
            if not (left_bbox and right_bbox): return
            lc = left_bbox.centerPoint; rc = right_bbox.centerPoint
            for b in list(comp.bRepBodies):
                if not b.name: continue
                if not any(t in b.name for t in MERGE_TAGS): continue
                if b == body_left or b == body_right: continue
                try: bc = b.boundingBox.centerPoint
                except Exception: continue
                d_left = bc.distanceTo(lc); d_right = bc.distanceTo(rc)
                target = body_left if d_left < d_right else body_right
                try: merge_bodies(comp, target, b)
                except Exception: pass

        # PHY-16: Bearing press-fit
        def bearing_fit(comp, tag, cx, cy, cz, axis="x", ro=1.10, w=0.60, fit_type="press"):
            cavity_r = ro - BEARING_PRESS if fit_type == "press" else (ro + BEARING_GLUE if fit_type == "glue" else ro)
            temp = adsk.fusion.TemporaryBRepManager.get()
            ax = {"x": (1,0,0), "y": (0,1,0), "z": (0,0,1)}[axis]
            half = w/2.0 + 0.05
            p1 = adsk.core.Point3D.create(cx-ax[0]*half, cy-ax[1]*half, cz-ax[2]*half)
            p2 = adsk.core.Point3D.create(cx+ax[0]*half, cy+ax[1]*half, cz+ax[2]*half)
            cs = temp.createCylinderOrCone(p1, cavity_r, p2, cavity_r)
            bf = comp.features.baseFeatures.add(); bf.startEdit()
            cb = comp.bRepBodies.add(cs, bf); bf.finishEdit()
            cb.name = f"{tag}_BearingCavity"; cut_cavity(comp, cb)
            if fit_type == "lip" or fit_type == "press":
                lip_w = BEARING_RET_LIP; lip_r = ro - 0.02
                lip_p1 = adsk.core.Point3D.create(cx-ax[0]*(half+0.01), cy-ax[1]*(half+0.01), cz-ax[2]*(half+0.01))
                lip_p2 = adsk.core.Point3D.create(cx-ax[0]*(half+0.01-lip_w), cy-ax[1]*(half+0.01-lip_w), cz-ax[2]*(half+0.01-lip_w))
                lip = temp.createCylinderOrCone(lip_p1, lip_r, lip_p2, lip_r)
                bf2 = comp.features.baseFeatures.add(); bf2.startEdit()
                lip_body = comp.bRepBodies.add(lip, bf2); bf2.finishEdit()
                lip_body.name = f"{tag}_Retainer"; cut_cavity(comp, lip_body)
            cyl(comp, f"{tag}_BO", cx, cy, cz, ro, w, axis, chrome)
            cyl(comp, f"{tag}_BI", cx, cy, cz, ro*0.58, w*0.80, axis, dark_grey)
            cyl(comp, f"{tag}_BB", cx, cy, cz, ro*0.32, w*1.10, axis, chrome)
            size_tag = f"Bearing {int(ro*2*10)}x{int(w*10)} mm"
            BOM.add("Bearing", size_tag, 1, f"{fit_type}-fit, {comp.name} at ({cx:.1f},{cy:.1f},{cz:.1f})")

        # PHY-15: Cable management
        def cable_clip(comp, tag, cx, cy, cz, axis="y"):
            clip_comp = new_component(f"{tag}_CableClip")
            clip_body = box(clip_comp, f"{tag}_ClipBody", cx, cy, cz, 0.30, 0.40, 0.30, grey_plastic)
            ch = box(clip_comp, f"{tag}_ClipCh", cx, cy, cz, 0.10, 0.35, 0.26)
            cut_cavity(clip_comp, ch)
            BOM.add("Printed Part", f"Cable clip ({tag})", 1, "PLA, snap fit")
            rigid_joint(f"{tag}_ClipMount", occs.get(clip_comp.name), occs.get(comp.name))
        def wiring_hub(comp, tag, cx, cy, cz):
            hub_comp = new_component(f"{tag}_WiringHub")
            box(hub_comp, f"{tag}_HubBase", cx, cy, cz, 2.0, 1.0, 1.5, dark_grey)
            for wi in range(4):
                wx = cx + (wi - 1.5) * 0.5
                box(hub_comp, f"{tag}_Route_{wi}", wx, cy+0.3, cz, 0.30, 0.25, 0.50)
            BOM.add("Printed Part", "Wiring hub (central)", 1, "TPU or PETG, wire organizer")
            rigid_joint(f"{tag}_HubMount", occs.get(hub_comp.name), occs.get(comp.name))

        # PHY-14: Printability
        def add_chamfer_45(body):
            try:
                comp = body.parentComponent
                ch_input = comp.features.chamferFeatures.createInput(body, adsk.fusion.ChamferType.EqualDistanceChamferType, adsk.core.ValueInput.createByReal(CHAMFER_D))
                comp.features.chamferFeatures.add(ch_input)
            except Exception:
                pass
        def add_print_orientation_note(part_name, orientation, support_req):
            Logger.print_note(f"PRINT: {part_name} — orient {orientation}, support: {support_req}")

        # PHY-17: Assembly jigs
        def alignment_jig(name, cx, cy, cz, lx, ly, lz, pin_positions=None):
            jig = new_jig_component(f"Jig_{name}")
            jig_body = box(jig, f"Jig_{name}_Base", cx, cy, cz, lx, ly, lz, white_pla)
            if pin_positions:
                for pi, (px, py, pz) in enumerate(pin_positions):
                    align_pin(jig, f"Pin_{pi}", px, py, pz, axis="z", height=0.50)
                    align_socket(jig, f"Sock_{pi}", px, py, pz+0.10, axis="z", depth=0.40)
            add_print_orientation_note(f"Jig_{name}", "flat on build plate", "none")
            BOM.add("Printed Part", f"Assembly jig {name}", 1, f"PLA, {lx*10:.0f}x{ly*10:.0f} mm")
            return jig
        def rpi_zero_bay(comp, tag, cx, cy, cz):
            cut_cavity(comp, box(comp, f"{tag}_RPiBay", cx, cy, cz, RPI0_L + 0.20, RPI0_W + 0.20, RPI0_H + 0.30))
            for sx, sz in [(-2.90, -1.15), (+2.90, -1.15), (-2.90, +1.15), (+2.90, +1.15)]:
                cyl(comp, f"{tag}_RpiStdoff_{sx}_{sz}", cx+sx, cy, cz+sz, 0.14, RPI0_H+0.50, "y", dark_metal)
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

        def _make_joint_geometry(cx, cy, cz):
            try:
                cpi = root.constructionPoints.createInput()
                cpi.setByPoint(adsk.core.Point3D.create(cx, cy, cz))
                cp = root.constructionPoints.add(cpi); cp.isLightBulbOn = False
                return adsk.fusion.JointGeometry.createByPoint(cp)
            except Exception:
                pass
            try:
                sketch = root.sketches.add(root.xYConstructionPlane); sketch.isVisible = False
                s_pt = sketch.sketchPoints.add(adsk.core.Point3D.create(cx, cy, cz))
                return adsk.fusion.JointGeometry.createByPoint(s_pt)
            except Exception:
                return None
        def revolute_joint(name, occ1, occ2, cx, cy, cz, axis_str):
            if not (occ1 and occ2): return
            try:
                geom = _make_joint_geometry(cx, cy, cz)
                ji = root.asBuiltJoints.createInput(occ1, occ2, geom)
                av = {"x": adsk.core.Vector3D.create(1,0,0), "y": adsk.core.Vector3D.create(0,1,0), "z": adsk.core.Vector3D.create(0,0,1)}[axis_str]
                ji.setAsRevoluteJointMotion(adsk.fusion.JointDirections.CustomJointDirection, av)
                j = root.asBuiltJoints.add(ji); j.name = name
            except Exception:
                Logger.log(f"Failed revolute joint {name}: {traceback.format_exc()}", "ERROR")
        def ball_joint(name, occ1, occ2, cx, cy, cz):
            if not (occ1 and occ2): return
            try:
                geom = _make_joint_geometry(cx, cy, cz)
                ji = root.asBuiltJoints.createInput(occ1, occ2, geom)
                ji.setAsBallJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection, adsk.fusion.JointDirections.XAxisJointDirection)
                j = root.asBuiltJoints.add(ji); j.name = name
            except Exception:
                Logger.log(f"Failed ball joint {name}: {traceback.format_exc()}", "ERROR")
        def rigid_joint(name, occ1, occ2):
            if not (occ1 and occ2): return
            try:
                geom = _make_joint_geometry(0, 0, 0)
                ji = root.asBuiltJoints.createInput(occ1, occ2, geom)
                ji.setAsRigidJointMotion()
                j = root.asBuiltJoints.add(ji); j.name = name
            except Exception:
                pass
        def servo_hardware(comp, tag, cx, cy, cz, axis, mg996):
            if mg996:
                fd, fw, hr, pd, sd = 2.4, 0.5, 0.7, 0.125, 0.15
                if axis == "x": hx,hy,hz = cx+2.40, cy, cz+1.05; fx,fy,fz = cx+0.95, cy, cz
                elif axis == "z": hx,hy,hz = cx-1.10, cy, cz+2.40; fx,fy,fz = cx, cy, cz+0.95
                else: hx,hy,hz = cx, cy+2.40, cz+1.05; fx,fy,fz = cx, cy+0.95, cz
            else:
                fd, fw, hr, pd, sd = 1.35, 0.0, 0.4, 0.10, 0.10
                if axis == "x": hx,hy,hz = cx+1.40, cy, cz+0.50; fx,fy,fz = cx+0.45, cy, cz
                elif axis == "z": hx,hy,hz = cx-0.50, cy, cz+1.40; fx,fy,fz = cx, cy, cz+0.45
                else: hx,hy,hz = cx, cy+1.40, cz+0.50; fx,fy,fz = cx, cy+0.45, cz
            for d1 in [-fd, fd]:
                for d2 in ([-fw, fw] if fw > 0 else [0]):
                    if axis == "x": c = cyl(comp, f"{tag}_FlgS_{d1}_{d2}", fx, fy+d2, fz+d1, sd, 1.5, "x")
                    elif axis == "z": c = cyl(comp, f"{tag}_FlgS_{d1}_{d2}", fx+d1, fy+d2, fz, sd, 1.5, "z")
                    else: c = cyl(comp, f"{tag}_FlgS_{d1}_{d2}", fx+d1, fy, fz+d2, sd, 1.5, "y")
                    cut_cavity(comp, c)
            for d in [-hr, hr]:
                if axis == "x":
                    c1 = cyl(comp, f"{tag}_Hrn1_{d}", hx, hy+d, hz, pd, 1.5, "x")
                    c2 = cyl(comp, f"{tag}_Hrn2_{d}", hx, hy, hz+d, pd, 1.5, "x")
                elif axis == "z":
                    c1 = cyl(comp, f"{tag}_Hrn1_{d}", hx+d, hy, hz, pd, 1.5, "z")
                    c2 = cyl(comp, f"{tag}_Hrn2_{d}", hx, hy+d, hz, pd, 1.5, "z")
                else:
                    c1 = cyl(comp, f"{tag}_Hrn1_{d}", hx+d, hy, hz, pd, 1.5, "y")
                    c2 = cyl(comp, f"{tag}_Hrn2_{d}", hx, hy, hz+d, pd, 1.5, "y")
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
            BOM.add("Servo", "MG996R 11 kg.cm servo", 1, comp.name)
            servo_type = "DS3225MG" if "hip" in tag.lower() or "Hip" in tag else "MG996R"
            horn_coupler(comp, f"{tag}_Cplr", cx, cy, cz, axis, servo_type)

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
            BOM.add("Servo", "MG90S micro servo", 1, comp.name)
            horn_coupler(comp, f"{tag}_Cplr", cx, cy, cz, axis, "MG90S")

        def add_ds04_nfc(comp, tag, cx, cy, cz, axis="x"):
            cl = CLEARANCE; dim = 2.00
            box(comp, f"{tag}_VisBody", cx, cy, cz, dim, 0.80, dim, grey_plastic)
            cyl(comp, f"{tag}_VisHorn", cx+0.80, cy, cz+0.40, 0.40, 0.15, axis, white_pla)
            marker(comp, f"{tag}_Pivot", cx+0.80, cy, cz+0.40)
            cut_cavity(comp, box(comp, f"{tag}_CB", cx, cy, cz, dim+cl, 0.80+cl, dim+cl))
            BOM.add("Servo", "DS04-NFC 9g digital servo", 1, comp.name)
            horn_coupler(comp, f"{tag}_Cplr", cx, cy, cz, axis, "DS04-NFC")

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

        def bearing(comp, tag, cx, cy, cz, axis="x", ro=1.10, w=0.60):
            bearing_fit(comp, tag, cx, cy, cz, axis, ro, w, fit_type="press")

        def u_bracket(comp, tag, cx, cy, cz, lx, ly, lz, ap=None):
            ap = ap or chrome
            box(comp, f"{tag}_BB", cx, cy, cz, 0.45, ly, lz, ap)
            box(comp, f"{tag}_BL", cx+lx*0.45, cy+ly*0.35, cz, lx*0.55, 0.40, lz, ap)
            box(comp, f"{tag}_BR", cx+lx*0.45, cy-ly*0.35, cz, lx*0.55, 0.40, lz, ap)
            cyl(comp, f"{tag}_Pin", cx+lx*0.50, cy, cz, 0.18, ly*0.85, "y", chrome)
            BOM.add("Printed Part", f"U-bracket {tag}", 1, f"PETG, {lx*10:.0f}x{ly*10:.0f}x{lz*10:.0f} mm")

        def mechanical_stop(comp, tag, cx, cy, cz, axis, angle_deg, ap=None):
            ap = ap or dark_metal
            box(comp, f"{tag}_Stop", cx, cy, cz, 0.30, 0.60, 0.30, ap)
            BOM.add("Printed Part", f"Mechanical stop {tag}", 1, f"PETG, {angle_deg:.0f} deg limit")

        def lock_pin(comp, tag, cx, cy, cz, axis="z", ap=None):
            ap = ap or chrome
            cyl(comp, f"{tag}_LockPin", cx, cy, cz, 0.18, 0.80, axis, ap)
            cyl(comp, f"{tag}_LockPinHandle", cx, cy, cz, 0.30, 0.15, axis, op_red)
            BOM.add("Hardware", "Lock pin spring (3 mm x 10 mm)", 1, comp.name)
            BOM.add("Printed Part", f"Lock pin housing {tag}", 1, f"PETG, {comp.name}")

        def servo_mount_reinforcement(comp, tag, cx, cy, cz, lx, ly, lz):
            for ri in range(3):
                rx = cx + (ri - 1) * lx * 0.25
                box(comp, f"{tag}_Rib_{ri}", rx, cy, cz, 0.20, ly*1.1, 0.20, dark_metal)
            BOM.add("Printed Part", f"Servo mount reinforcement {tag}", 1, comp.name)

        def wear_plate(comp, tag, cx, cy, cz, lx, ly, lz, ap=None):
            ap = ap or white_pla
            plate = box(comp, f"{tag}_WearPlate", cx, cy, cz, lx, ly, lz, ap)
            for wx in [cx - lx*0.3, cx + lx*0.3]:
                if abs(wx - cx) > 0.4: screw_hole(comp, wx, cy, cz, axis="z", length=0.60)
            BOM.add("Printed Part", f"Wear plate {tag}", 1, f"PLA/TPU, replaceable, {comp.name}")

        # ── Phase 1: Structural Helpers ─────────────────────────────────────
        def metal_frame_channel(comp, name, cx, cy, cz, length, axis="z"):
            hw = EXTRUSION_W/2; ch = box(comp, name, cx, cy, cz, EXTRUSION_W, EXTRUSION_W, length, dark_metal)
            for s_off in [-EXTRUSION_SLOT_W/2]:
                sx, sy, sz = cx, cy, cz
                if axis == "x": slot = box(comp, f"{name}_Sl_{int(s_off*100)}", sx, sy+EXTRUSION_SLOT_D, sz, length, EXTRUSION_SLOT_W, EXTRUSION_SLOT_D*2, dark_metal)
                elif axis == "y": slot = box(comp, f"{name}_Sl_{int(s_off*100)}", sx+EXTRUSION_SLOT_D, sy, sz, EXTRUSION_SLOT_W, length, EXTRUSION_SLOT_D*2, dark_metal)
                else: slot = box(comp, f"{name}_Sl_{int(s_off*100)}", sx+EXTRUSION_SLOT_D, sy, sz, EXTRUSION_SLOT_W, EXTRUSION_SLOT_D*2, length, dark_metal)
                cut_cavity(comp, slot)
            BOM.add("Frame", f"2020 extrusion {int(length*10)}mm", 1, name)
            return ch

        def frame_mount_bracket(comp, name, cx, cy, cz, axis="z"):
            b = box(comp, name, cx, cy, cz, FRAME_BRACKET_W, FRAME_BRACKET_T, FRAME_BRACKET_W, dark_metal)
            for bx in [cx - FRAME_BRACKET_W*0.3, cx + FRAME_BRACKET_W*0.3]:
                screw_hole(comp, bx, cy, cz, axis=axis, length=FRAME_BRACKET_W)
            BOM.add("Frame", f"L-bracket {name}", 2, "M3x8 + M6 T-nut")
            return b

        def steel_shaft_block(comp, name, cx, cy, cz, axis, shaft_diam, length, bearing_count=2):
            bw = BEARING_BLOCK_W; bh = BEARING_BLOCK_H
            bw2 = max(bw, shaft_diam + 0.30)
            bl = box(comp, name, cx, cy, cz, bw2, bh, length, dark_metal)
            sr = shaft_diam/2
            bore = cyl(comp, f"{name}_Bore", cx, cy, cz, sr + CLEARANCE, length + 0.20, axis)
            cut_cavity(comp, bore)
            ss_off = shaft_diam * 0.6
            ssx = ss_off if axis != "x" else 0; ssy = ss_off if axis != "y" else 0; ssz = ss_off if axis != "z" else 0
            ss = cyl(comp, f"{name}_SS", cx+ssx, cy+ssy, cz+ssz, SET_SCREW_R, SET_SCREW_L, axis, dark_metal)
            merge_bodies(comp, bl, ss)
            return bl

        def flanged_bearing_cavity(comp, name, cx, cy, cz, axis, bearing_id, bearing_od, bearing_w):
            br = bearing_od/2 + BEARING_PRESS
            bore = cyl(comp, name, cx, cy, cz, br, bearing_w + BEARING_RET_LIP, axis)
            cut_cavity(comp, bore)
            fl_plane = {"x": (0, 0, -1), "y": (0, 0, -1), "z": (0, 1, 0)}.get(axis, (0, 0, 1))
            fc = adsk.core.Point3D.create(cx+fl_plane[0]*(FLANGE_BRG_FLANGE_OD/2), cy+fl_plane[1]*(FLANGE_BRG_FLANGE_OD/2), cz+fl_plane[2]*(FLANGE_BRG_FLANGE_OD/2))
            return bore

        def worm_gear_housing(comp, name, cx, cy, cz, axis, ratio=GEAR_RATIO_WAIST):
            cd = 2 * WORM_ADDENDUM + 2 * WORM_DEDDENDUM + 0.10
            hw = 1.80; hl = 2.20
            h = box(comp, name, cx, cy, cz, hw, hl, hl, dark_metal)
            sr = 0.30 + CLEARANCE
            worm_brg = cyl(comp, f"{name}_WormBrg", cx, cy-0.8, cz, sr, hl, "y"); cut_cavity(comp, worm_brg)
            wheel_brg = cyl(comp, f"{name}_WheelBrg", cx, cy+0.2, cz, sr*ratio/10+0.1, hw, "x"); cut_cavity(comp, wheel_brg)
            BOM.add("Gearbox", f"Worm set {ratio}:1 (module 1)", 1, name)
            return h

        def planetary_gear_stage(comp, name, cx, cy, cz, axis, reduction=GEAR_RATIO_HIP):
            p = box(comp, name, cx, cy, cz, PLANET_STAGE_R*2, PLANET_STAGE_H, PLANET_STAGE_R*2, dark_metal)
            sr = PLANET_SUN_R + CLEARANCE
            sun = cyl(comp, f"{name}_Sun", cx, cy, cz, sr, PLANET_STAGE_H+0.1, axis)
            cut_cavity(comp, sun)
            for gi in range(3):
                a = gi * 2.094
                gx = cx + (PLANET_SUN_R + PLANET_GEAR_R + 0.05) * 0.7 * (1 if axis == "z" else 0)
                gy = cy + (PLANET_SUN_R + PLANET_GEAR_R + 0.05) * 0.7 * (1 if axis == "y" else 0)
                gz = cz + (PLANET_SUN_R + PLANET_GEAR_R + 0.05) * 0.7 * (1 if axis == "x" else 0)
                pg = cyl(comp, f"{name}_PG{gi}", gx, gy, gz, PLANET_GEAR_R, PLANET_STAGE_H+0.1, axis)
                cut_cavity(comp, pg)
            BOM.add("Gearbox", f"Planetary stage {reduction}:1", 1, name)
            return p

        def gearbox_mount(comp, name, cx, cy, cz, axis, servo_type="MG996R"):
            spec = SERVO_COUPLER.get(servo_type, SERVO_COUPLER["MG996R"])
            mp = box(comp, name, cx, cy, cz, spec["flange_od"]+0.40, spec["flange_od"]+0.40, 0.20, dark_metal)
            for mi in range(4):
                mx = cx + ([-1, 1, -1, 1][mi])*1.15; my = cy + ([-1, -1, 1, 1][mi])*0.55
                mz = cz; screw_hole(comp, mx, my, mz, axis="z", length=0.50)
            return mp

        def transformation_latch_hook(comp, name, cx, cy, cz, axis="z"):
            hw = LATCH_HOOK_D; hh = LATCH_HOOK_L
            hk = box(comp, name, cx, cy, cz, hw, hw, hh, dark_metal)
            hp = box(comp, f"{name}_Mount", cx, cy, cz, hw+0.20, 0.10, 0.10, dark_metal)
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

        def alignment_cam_guide(comp, name, cx, cy, cz, axis="z"):
            cw = ALIGN_CAM_W; ch = ALIGN_CAM_H
            b = box(comp, name, cx, cy, cz, cw, ch, cw, chrome)
            for sc_off in [-0.25, 0.25]:
                sx = cx + sc_off; sy = cy; sz = cz
                screw_hole(comp, sx, sy, sz, axis=axis, length=0.50)
            BOM.add("Printed Part", f"Alignment cam {name}", 1, "PLA/PETG")
            return b

        def quick_release_coupler(comp, name, cx, cy, cz, axis="z", side=1):
            m_h = box(comp, f"{name}_Male", cx, cy, cz, QR_PIN_D, QR_PIN_D, QR_COLLAR_L, chrome)
            f_h = box(comp, f"{name}_Female", cx+QR_COLLAR_D*side, cy, cz, QR_COLLAR_D, QR_COLLAR_D, QR_COLLAR_L, dark_metal)
            det = cyl(comp, f"{name}_Detent", cx+QR_PIN_D/2, cy, cz, QR_DETENT_BALL_R, QR_PIN_D, "x", chrome)
            merge_bodies(comp, m_h, det)
            set_screw = cyl(comp, f"{name}_SS", cx, cy+QR_PIN_D/2, cz, SET_SCREW_R, 0.20, "y")
            merge_bodies(comp, m_h, set_screw)
            BOM.add("Hardware", f"Quick-release {name}", 1, "6mm detent pin + collar")
            return m_h, f_h

        def spine_vertebra(comp, name, cx, cy, cz, index=0, total=3):
            v = box(comp, name, cx, cy, cz, SPINE_SEG_W, SPINE_SEG_D, SPINE_SEG_H, dark_metal)
            nerve = cyl(comp, f"{name}_Nerve", cx, cy, cz, SPINE_NERVE_CH_D/2, SPINE_SEG_H+0.1, "z")
            cut_cavity(comp, nerve)
            ball = cyl(comp, f"{name}_Ball", cx, cy, cz-SPINE_SEG_H/2-0.05, SPINE_BALL_D/2, SPINE_SOCKET_D, "z", chrome)
            merge_bodies(comp, v, ball)
            socket = cyl(comp, f"{name}_Socket", cx, cy, cz+SPINE_SEG_H/2+0.05, SPINE_SOCKET_D/2, SPINE_BALL_D, "z")
            cut_cavity(comp, socket)
            lch = cyl(comp, f"{name}_LCh", cx-SPINE_SEG_W*0.3, cy, cz, 0.10, SPINE_SEG_H+0.1, "z"); cut_cavity(comp, lch)
            rch = cyl(comp, f"{name}_RCh", cx+SPINE_SEG_W*0.3, cy, cz, 0.10, SPINE_SEG_H+0.1, "z"); cut_cavity(comp, rch)
            BOM.add("Printed Part", f"Spine vertebra {index+1}/{total}", 1, "PETG, load-bearing")
            return v

        def spine_bellows_cover(comp, name, cx, cy, cz, height):
            bc = box(comp, name, cx, cy, cz, SPINE_SEG_W+0.40, SPINE_SEG_D+0.40, height, grey_plastic)
            n_ribs = max(1, int(height / (BELLOWS_RIB_H + BELLOWS_WALL)))
            for ri in range(n_ribs):
                rz = cz - height/2 + ri * (BELLOWS_RIB_H + BELLOWS_WALL)
                rib = box(comp, f"{name}_Rib{ri}", cx, cy, rz, SPINE_SEG_W+0.60, SPINE_SEG_D+0.60, BELLOWS_RIB_H, grey_plastic)
                merge_bodies(comp, bc, rib)
            BOM.add("Printed Part", f"Spine bellows {name}", 1, "TPU flexible")
            return bc
        # ── End Structural Helpers ─────────────────────────────────────────────

        BuildGuide.add("PREAMBLE", "This guide documents assembly of Optimus Prime G1 v11 Physical Build Edition. All dimensions in cm. Read warnings section before starting.")

        # ① TORSO
        torso = new_component("OP_Torso")
        BuildGuide.add("TORSO ASSEMBLY", "1. Install M3 heat-set inserts in all boss holes\n2. Install PCA9685 on standoffs\n3. Route servo harnesses through cable channels\n4. Install RPi Zero 2W on standoffs\n5. Wire BEC for servo power (5V 5A)\n6. Install LiPo with velcro strap anchors\n7. Install fuse holder and cutoff switch\n8. Close access covers with M3 screws")
        box(torso, "Torso_Shell", 0, 0, TORSO_CTR, 10.8, 9.0, 12.6, op_red)
        box(torso, "Torso_Side_L", -5.8, 0, TORSO_CTR, 0.55, 8.0, 11.6, op_red)
        box(torso, "Torso_Side_R", 5.8, 0, TORSO_CTR, 0.55, 8.0, 11.6, op_red)
        box(torso, "Skeleton_Main", 0, 0, TORSO_CTR, 6.0, 5.0, 10.0, dark_metal)
        box(torso, "Skeleton_Cross", 0, 0, TORSO_CTR+2.0, 8.0, 1.2, 4.0, dark_metal)
        box(torso, "Skeleton_Base", 0, 0, TORSO_CTR-3.5, 7.0, 1.5, 3.0, dark_metal)
        box(torso, "CWin_Frame_L", -2.3, -4.30, TORSO_CTR+2.6, 2.8, 0.34, 3.4, op_blue)
        box(torso, "CWin_Frame_R", 2.3, -4.30, TORSO_CTR+2.6, 2.8, 0.34, 3.4, op_blue)
        box(torso, "Chest_Win_L", -2.3, -4.45, TORSO_CTR+2.6, 2.4, 0.20, 2.8, glass_clr)
        box(torso, "Chest_Win_R", 2.3, -4.45, TORSO_CTR+2.6, 2.4, 0.20, 2.8, glass_clr)
        box(torso, "Chest_Win_Div", 0, -4.35, TORSO_CTR+2.6, 0.42, 0.22, 3.4, chrome)
        for gz_offset, gw in [(-0.3, 7.6), (-1.1, 7.2), (-1.9, 6.8), (-2.7, 6.4)]:
            box(torso, f"Grille_{int(gz_offset*10)}", 0, -4.50, TORSO_CTR+gz_offset, gw, 0.22, 0.32, chrome)
        box(torso, "Headlight_L", -4.5, -4.60, TORSO_CTR-1.2, 2.0, 0.28, 2.2, glass_clr)
        box(torso, "Headlight_R", 4.5, -4.60, TORSO_CTR-1.2, 2.0, 0.28, 2.2, glass_clr)
        box(torso, "Front_Bumper", 0, -6.0, TORSO_CTR-4.6, 10.4, 2.2, 2.0, chrome)
        box(torso, "Hood_Crease_L", -2.6, -4.70, TORSO_CTR-2.8, 0.5, 0.38, 3.2, op_red)
        box(torso, "Hood_Crease_R", 2.6, -4.70, TORSO_CTR-2.8, 0.5, 0.38, 3.2, op_red)
        box(torso, "Ind_L", -3.9, -5.10, TORSO_CTR-3.9, 0.6, 0.25, 0.5, yellow_met)
        box(torso, "Ind_R", 3.9, -5.10, TORSO_CTR-3.9, 0.6, 0.25, 0.5, yellow_met)
        box(torso, "Chest_Plate", 0, -4.30, TORSO_CTR+0.6, 8.8, 0.34, 4.2, chrome)
        cyl(torso, "Badge_Ring", 0, -4.65, TORSO_CTR+0.6, 0.85, 0.14, "y", op_red)
        led_ring_pocket(torso, "Badge", 0, -4.70, TORSO_CTR+0.6, "y")
        box(torso, "Support_Beam_L", -3.5, 1.5, TORSO_CTR-1.0, 0.35, 4.0, 8.0, dark_metal)
        box(torso, "Support_Beam_R", 3.5, 1.5, TORSO_CTR-1.0, 0.35, 4.0, 8.0, dark_metal)
        # Phase 1: 2020 aluminum extrusion frame through torso
        metal_frame_channel(torso, "Frame_Spine", 0, 0, TORSO_CTR, 12.0, "z")
        metal_frame_channel(torso, "Frame_Horiz", 0, 0, TORSO_CTR, 6.0, "y")
        frame_mount_bracket(torso, "Frame_Top", 0, -1.5, TORSO_CTR+5.0, "z")
        frame_mount_bracket(torso, "Frame_Bot", 0, -1.5, TORSO_CTR-5.0, "z")
        frame_mount_bracket(torso, "Frame_Waist", 0, -1.5, WAIST_CTR, "z")
        BOM.add("Frame", "M6 T-nut drop-in", 6, "for 2020 extrusion")
        BOM.add("Fastener", "M6x12 button-head bolt", 6, "frame brackets")
        # Phase 1: Spine vertebrae — articulated segments below torso
        sv1 = spine_vertebra(torso, "SpineV1", -1.2, 0, PELVIS_CTR+1.0, 0, 3)
        sv2 = spine_vertebra(torso, "SpineV2", 0, 0, PELVIS_CTR+2.2, 1, 3)
        sv3 = spine_vertebra(torso, "SpineV3", 1.2, 0, PELVIS_CTR+3.4, 2, 3)
        spine_bellows_cover(torso, "SpineCover", 0, 0, PELVIS_CTR+2.2, 3.0)
        # Phase 1: Quick-release couplers at shoulder interface
        quick_release_coupler(torso, "L_ShoulderQR", -SHOULDER_X, 0, SHOULDER_CTR-1.5, "x", -1)
        quick_release_coupler(torso, "R_ShoulderQR", SHOULDER_X, 0, SHOULDER_CTR-1.5, "x", 1)
        box(torso, "LipoBay_Shell", 0, 3.4, TORSO_CTR-2.2, 8.0, 4.8, 6.0, black_plastic)
        lipo_bay(torso, "Main", 0, 2.8, TORSO_CTR-2.0)
        for strap_x in [-3.0, 3.0]:
            box(torso, f"StrapAnch_{strap_x:.0f}", strap_x, 3.8, TORSO_CTR-2.0, 0.30, 0.40, 0.30, dark_metal)
        BOM.add("Hardware", "Velcro battery strap 15 mm x 100 mm", 2, "LiPo retention")
        fuse_pocket(torso, "MainFuse", -2.0, 3.8, TORSO_CTR-2.5)
        cutoff_switch_pocket(torso, "MainCutoff", 2.0, 3.8, TORSO_CTR-2.5)
        box(torso, "BEC_BayShell", 0, 3.8, TORSO_CTR+0.5, 4.0, 2.8, 1.2, black_plastic)
        bec_bay(torso, "MainBEC", 0, 3.8, TORSO_CTR+0.5)
        box(torso, "RPi_Shell", 0, 2.8, TORSO_CTR+1.8, 7.2, 3.8, 3.0, black_plastic)
        rpi_zero_bay(torso, "Main", 0, 3.2, TORSO_CTR+1.8)
        box(torso, "PCA_Shell", 0, 2.8, TORSO_CTR+4.2, 7.0, 3.4, 2.4, black_plastic)
        pca9685_bay(torso, "Main", 0, 3.0, TORSO_CTR+4.2)
        cut_cavity(torso, box(torso, "USBAccess", 3.5, 3.8, TORSO_CTR+1.8, 0.40, 0.30, 0.30))
        make_access_cover(torso, "LipoBay", 0, 4.6, TORSO_CTR-2.2, 7.2, 0.30, 5.4, ap=black_plastic)
        make_access_cover(torso, "RPiBay", 0, 4.6, TORSO_CTR+1.8, 6.8, 0.25, 2.6, ap=black_plastic)
        make_access_cover(torso, "PCABay", 0, 4.6, TORSO_CTR+4.2, 6.6, 0.25, 2.0, ap=black_plastic)
        cable_clip(torso, "Torso_L", -3.8, 0.6, TORSO_CTR)
        cable_clip(torso, "Torso_R", 3.8, 0.6, TORSO_CTR)
        cable_clip(torso, "Torso_B", 0, 1.5, TORSO_CTR-3.0)
        wiring_hub(torso, "MainHub", 0, 2.0, TORSO_CTR+0.5)
        box(torso, "Cable_L", -3.6, 0.6, TORSO_CTR, 0.60, 1.0, 10.5, dark_grey)
        box(torso, "Cable_R", 3.6, 0.6, TORSO_CTR, 0.60, 1.0, 10.5, dark_grey)
        box(torso, "Cable_Center", 0, 0.8, TORSO_CTR, 1.00, 0.8, 8.0, dark_grey)
        box(torso, "Collar_L", -8.2, 0, SHOULDER_CTR-1.0, 5.8, 3.6, 3.2, chrome)
        box(torso, "Collar_R", 8.2, 0, SHOULDER_CTR-1.0, 5.8, 3.6, 3.2, chrome)
        box(torso, "TF_Flap_L", -5.60, -0.2, TORSO_CTR+3.0, 0.45, 6.8, 6.4, op_red)
        box(torso, "TF_Flap_R", 5.60, -0.2, TORSO_CTR+3.0, 0.45, 6.8, 6.4, op_red)
        box(torso, "TF_BackTop", 0, 5.2, TORSO_CTR+5.4, 8.4, 0.40, 5.4, op_blue)
        mechanical_stop(torso, "TFFlap_L", -5.2, -0.2, TORSO_CTR+3.0, "y", 90)
        mechanical_stop(torso, "TFFlap_R", 5.2, -0.2, TORSO_CTR+3.0, "y", 90)
        # Phase 1: Transformation latches on TF flaps + solenoid release
        transformation_latch_hook(torso, "Latch_FL", -5.2, -1.5, TORSO_CTR+3.5, "y")
        transformation_latch_hook(torso, "Latch_FR", 5.2, -1.5, TORSO_CTR+3.5, "y")
        transformation_latch_post(torso, "Latch_FL_Post", -5.2, 1.5, TORSO_CTR+3.5, "y")
        transformation_latch_post(torso, "Latch_FR_Post", 5.2, 1.5, TORSO_CTR+3.5, "y")
        solenoid_mount(torso, "LatchSol_L", -5.2, 0, TORSO_CTR+4.0, "y")
        solenoid_mount(torso, "LatchSol_R", 5.2, 0, TORSO_CTR+4.0, "y")
        alignment_cam_guide(torso, "Cam_FL", -5.2, -0.8, TORSO_CTR+4.5, "y")
        alignment_cam_guide(torso, "Cam_FR", 5.2, -0.8, TORSO_CTR+4.5, "y")
        for bx_off, bz_off in [(-3.4, 5.2), (3.4, 5.2), (-3.4, -5.2), (3.4, -5.2)]:
            m3_boss(torso, f"Torso_B{bx_off}_{bz_off}", bx_off, 0, TORSO_CTR+bz_off)
        align_pin(torso, "TorsoSplit_L", -5.0, 0, TORSO_CTR)
        align_pin(torso, "TorsoSplit_R", 5.0, 0, TORSO_CTR)
        add_print_orientation_note("Torso", "upright on rear face", "yes — chest windows & grille")
        u_bracket(torso, "Waist_Brkt", 0, 0, WAIST_CTR, 4.2, 4.4, 3.6)
        # Phase 1: Worm gear reduction for waist yaw (30:1 self-locking)
        worm_gear_housing(torso, "Waist_Worm", 0, 0, WAIST_CTR, "z", GEAR_RATIO_WAIST)
        gearbox_mount(torso, "Waist_GbMnt", 0, 0, WAIST_CTR-0.8, "z", "MG996R")
        steel_shaft_block(torso, "Waist_Shaft", 0, 0, WAIST_CTR, "z", SHAFT_DIAM_MAJOR, 2.0, 2)
        add_mg996r(torso, "Waist_Yaw", 0, -0.8, WAIST_CTR, "z")
        bearing(torso, "Waist_Brg", 0, 0, WAIST_CTR+0.5, "z", 1.30, 0.65)
        u_bracket(torso, "WaistP_Brkt", 0, 0, WAIST_CTR-2.5, 4.2, 4.4, 3.6)
        # Phase 1: Steel shaft for waist pitch
        steel_shaft_block(torso, "WaistP_Shaft", 0, 0, WAIST_CTR-2.5, "x", SHAFT_DIAM_MED, 3.0, 2)
        add_mg996r(torso, "Waist_Pitch", 0, 0, WAIST_CTR-2.5, "x")
        bearing(torso, "WaistP_Brg", 0, 0, WAIST_CTR-2.0, "x", 1.30, 0.65)
        mechanical_stop(torso, "WaistP_F", 0, -1.8, WAIST_CTR-3.0, "x", 60)
        mechanical_stop(torso, "WaistP_B", 0, 1.8, WAIST_CTR-3.0, "x", -45)
        magnet_pocket(torso, "WLock_F", 0, -2.0, WAIST_CTR-3.0)
        magnet_pocket(torso, "WLock_R", 0, 2.0, WAIST_CTR-3.0)
        u_bracket(torso, "Neck_Brkt", 0, 0, NECK_JOINT_Z, 3.4, 3.0, 3.2)
        # Phase 1: Steel shaft for neck pitch
        steel_shaft_block(torso, "NeckP_Shaft", 0, 0, NECK_JOINT_Z, "x", SHAFT_DIAM_MINOR, 2.0, 2)
        add_mg996r(torso, "Neck_Pitch", 0, 0, NECK_JOINT_Z, "x")
        wire_channel(torso, "Spine", 0, 0, TORSO_CTR, 0.7, 20.0, "z")
        mechanical_stop(torso, "Neck_F", 0, -1.2, NECK_JOINT_Z, "x", 45)

        # ② HEAD
        head = new_component("OP_Head"); HC = HEAD_CTR
        BuildGuide.add("HEAD ASSEMBLY", "1. Install ESP32-CAM in head pocket\n2. Route lens through channel to visor area\n3. Install 3x 5mm LEDs behind visor\n4. Install neck yaw servo (MG90S)\n5. Close head shell halves with M3 screws")
        box(head, "Helm_Main", 0, 0, HC+0.8, 5.8, 5.4, 5.2, op_blue)
        box(head, "Helm_Forehead", 0, -2.35, HC+2.6, 5.2, 0.44, 2.0, op_blue)
        box(head, "Helm_Top", 0, 0, HC+3.5, 5.0, 4.6, 0.95, op_blue)
        box(head, "Ear_L", -3.45, 0, HC+1.8, 0.75, 5.0, 4.0, op_blue)
        box(head, "Ear_R", 3.45, 0, HC+1.8, 0.75, 5.0, 4.0, op_blue)
        box(head, "EarTop_L", -3.20, 0, HC+3.5, 1.50, 4.6, 0.65, op_blue)
        box(head, "EarTop_R", 3.20, 0, HC+3.5, 1.50, 4.6, 0.65, op_blue)
        box(head, "Crest_Main", 0, -0.35, HC+3.9, 1.10, 0.70, 3.8, chrome)
        box(head, "Crest_Stripe", 0, -0.35, HC+4.0, 0.58, 0.38, 3.0, op_blue)
        box(head, "Face_Plate", 0, -2.55, HC+0.5, 3.8, 0.40, 3.4, chrome)
        box(head, "Chin_Guard", 0, -2.65, HC-0.9, 3.2, 0.40, 2.0, chrome)
        box(head, "Chin_L", -1.40, -2.58, HC-0.4, 0.40, 0.34, 2.4, chrome)
        box(head, "Chin_R", 1.40, -2.58, HC-0.4, 0.40, 0.34, 2.4, chrome)
        box(head, "Visor_Frame", 0, -2.60, HC+1.35, 4.0, 0.32, 1.35, op_blue)
        box(head, "Visor", 0, -2.72, HC+1.35, 3.2, 0.16, 0.95, op_blue_glass)
        for vx in [-0.85, 0.0, 0.85]:
            led_pocket_5mm(head, f"Visor_LED_{int(vx*100)}", vx, -2.85, HC+1.35, "y")
        box(head, "Nose_Bridge", 0, -2.65, HC+1.95, 0.75, 0.24, 0.75, chrome)
        box(head, "Mouth_Plate", 0, -2.60, HC+0.10, 2.6, 0.24, 1.15, dark_grey)
        for mz in [-0.35, 0.0, 0.35]:
            box(head, f"MouthGrill_{int(mz*100)}", 0, -2.68, HC+0.10+mz, 2.0, 0.12, 0.20, chrome)
        box(head, "Head_Rear", 0, 2.00, HC+1.0, 4.4, 1.6, 4.6, op_red)
        box(head, "Neck_Collar", 0, 0, HC-1.6, 2.6, 2.6, 2.5, dark_metal)
        cyl(head, "Ant_L", -2.90, 0, HC+4.5, 0.14, 2.8, "z", chrome)
        cyl(head, "Ant_R", 2.90, 0, HC+4.5, 0.14, 2.8, "z", chrome)
        cyl(head, "AntTip_L", -2.90, 0, HC+5.9, 0.25, 0.36, "z", gold_met)
        cyl(head, "AntTip_R", 2.90, 0, HC+5.9, 0.25, 0.36, "z", gold_met)
        esp32_cam_pocket(head, "HeadCam", 0, -1.6, HC+2.5, "y")
        cut_cavity(head, box(head, "SD_Access", 0, 1.8, HC+1.5, 0.50, 0.30, 0.30))
        add_mg90s(head, "Neck_Yaw", 0, 0, NECK_JOINT_Z, "z")
        # Phase 1: Steel shaft at neck yaw
        steel_shaft_block(head, "NeckYaw_Shaft", 0, 0, NECK_JOINT_Z, "z", SHAFT_DIAM_MINOR, 1.5, 2)
        add_print_orientation_note("Head", "face up (visor facing up)", "yes — chin guard & vents")

        # ③ PELVIS
        pelvis = new_component("OP_Pelvis")
        BuildGuide.add("PELVIS ASSEMBLY", "1. Install MPU6050 IMU in centre pocket\n2. Install L/R hip yaw servos (DS3225MG)\n3. Attach couplers with set screws\n4. Close pelvis shell with align pins + M3 screws")
        box(pelvis, "Pelvis_Shell", 0, 0, PELVIS_CTR, 16.8, 6.6, 5.4, op_blue)
        box(pelvis, "Pelvis_Frame", 0, 0, PELVIS_CTR, 12.6, 4.8, 4.2, dark_metal)
        box(pelvis, "Pelvis_Reinf", 0, 0, PELVIS_CTR, 10.0, 2.0, 3.0, dark_metal)
        # Phase 1: Metal frame cross-beam through pelvis
        metal_frame_channel(pelvis, "Pelvis_FrameChan", 0, 0, PELVIS_CTR, 8.0, "y")
        frame_mount_bracket(pelvis, "Pelvis_FrBrk_L", -HIP_X-1.5, 0, PELVIS_CTR, "y")
        frame_mount_bracket(pelvis, "Pelvis_FrBrk_R", HIP_X+1.5, 0, PELVIS_CTR, "y")
        # Phase 1: Steel shafts at hip yaw with flanged bearings
        steel_shaft_block(pelvis, "L_HipYaw_Shaft", -HIP_X-2.2, 0, HIP_JOINT_Z, "z", SHAFT_DIAM_MAJOR, 2.0, 2)
        steel_shaft_block(pelvis, "R_HipYaw_Shaft", HIP_X+2.2, 0, HIP_JOINT_Z, "z", SHAFT_DIAM_MAJOR, 2.0, 2)
        flanged_bearing_cavity(pelvis, "L_HY_FlangeBrg", -HIP_X-2.2, 0, HIP_JOINT_Z, "z", FLANGE_BRG_MAJOR_ID, FLANGE_BRG_MAJOR_OD, FLANGE_BRG_MAJOR_W)
        flanged_bearing_cavity(pelvis, "R_HY_FlangeBrg", HIP_X+2.2, 0, HIP_JOINT_Z, "z", FLANGE_BRG_MAJOR_ID, FLANGE_BRG_MAJOR_OD, FLANGE_BRG_MAJOR_W)
        # Phase 1: Quick-release couplers at leg interface
        quick_release_coupler(pelvis, "L_LegQR", -HIP_X, 0, HIP_JOINT_Z, "z", -1)
        quick_release_coupler(pelvis, "R_LegQR", HIP_X, 0, HIP_JOINT_Z, "z", 1)
        # Phase 1: Transformation latch posts (engage with torso latches)
        transformation_latch_post(pelvis, "TFLatch_Post_L", -5.2, 0, PELVIS_CTR+2.0, "y")
        transformation_latch_post(pelvis, "TFLatch_Post_R", 5.2, 0, PELVIS_CTR+2.0, "y")
        box(pelvis, "Hip_Armr_L", -7.4, 0, PELVIS_CTR, 1.1, 5.4, 4.4, chrome)
        box(pelvis, "Hip_Armr_R", 7.4, 0, PELVIS_CTR, 1.1, 5.4, 4.4, chrome)
        box(pelvis, "Crotch_Plate", 0, -3.0, PELVIS_CTR-1.2, 5.4, 0.32, 2.6, op_red)
        for px_off in [-5.0, 5.0]: align_pin(pelvis, f"PelvPin_{px_off:.0f}", px_off, 0, PELVIS_CTR)
        imu_pocket(pelvis, "BalanceIMU", 0, 0, PELVIS_CTR)
        for hx_off in [-HIP_X-2.2, HIP_X+2.2]:
            box(pelvis, f"HipBrgReinf_{hx_off:.0f}", hx_off, 0, HIP_JOINT_Z, 1.0, 1.5, 1.0, dark_metal)
        add_mg996r(pelvis, "L_HipYaw", -HIP_X, 0, HIP_JOINT_Z, "z")
        add_mg996r(pelvis, "R_HipYaw", HIP_X, 0, HIP_JOINT_Z, "z")
        bearing(pelvis, "L_HYB", -HIP_X-2.2, 0, HIP_JOINT_Z, "z", 1.10, 0.62)
        bearing(pelvis, "R_HYB", HIP_X+2.2, 0, HIP_JOINT_Z, "z", 1.10, 0.62)
        mechanical_stop(pelvis, "LHipStop", -HIP_X, -2.0, HIP_JOINT_Z-0.5, "z", 95)
        mechanical_stop(pelvis, "RHipStop", HIP_X, -2.0, HIP_JOINT_Z-0.5, "z", 95)
        BOM.add("Servo", "DS3225MG 25 kg.cm servo (hip yaw)", 2, "OP_Pelvis")
        add_print_orientation_note("Pelvis", "flat on rear face", "minimal - hip armor overhangs")
        # ④ LEGS
        BuildGuide.add("LEG ASSEMBLY (each)", "1. Install hip pitch servo in thigh (DS3225MG)\n2. Install hip roll servo\n3. Install knee servo with bearing\n4. Route servo wires through channels with clips\n5. Close thigh shell halves\n6. Install knee bearing in shin\n7. Install TT gear-motors and wheels\n8. Close shin shell\n9. Install ankle servos in foot\n10. Attach foot with ankle bearings\n11. Install replaceable sole wear plate (MECH-6)")
        for side, sx in [("L", -HIP_X), ("R", HIP_X)]:
            m = -1 if side == "L" else 1
            thigh = new_component(f"OP_Thigh_{side}")
            box(thigh, "Thigh_Link", sx, 0, THIGH_CTR, 5.2, 4.2, 11.2, chrome)
            box(thigh, "Thigh_Outer", sx+m*2.75, 0, THIGH_CTR, 0.55, 4.6, 11.2, op_red)
            box(thigh, "Thigh_Front", sx, -2.3, THIGH_CTR, 5.2, 0.42, 11.2, op_blue)
            box(thigh, "Thigh_Knee_Armor", sx, -2.3, KNEE_CTR+2.6, 4.4, 0.85, 3.0, chrome)
            servo_mount_reinforcement(thigh, f"{side}_HipP", sx, 0, HIP_JOINT_Z, 4.0, 3.2, 3.2)
            servo_mount_reinforcement(thigh, f"{side}_KneP", sx, 0, KNEE_CTR+1.5, 3.8, 3.0, 3.0)
            box(thigh, f"{side}_ThighFrame", sx, 0, THIGH_CTR, 3.0, 2.0, 10.0, dark_metal)
            # Phase 1: Planetary gear stage for hip pitch (4:1 reduction)
            planetary_gear_stage(thigh, f"{side}_HipPlanet", sx, 0, HIP_JOINT_Z, "x", GEAR_RATIO_HIP)
            steel_shaft_block(thigh, f"{side}_HipP_Shaft", sx, 0, HIP_JOINT_Z, "x", SHAFT_DIAM_MAJOR, 2.5, 2)
            flanged_bearing_cavity(thigh, f"{side}_HP_FlangeBrg", sx+0.5, 0, HIP_JOINT_Z, "x", FLANGE_BRG_MAJOR_ID, FLANGE_BRG_MAJOR_OD, FLANGE_BRG_MAJOR_W)
            gearbox_mount(thigh, f"{side}_HP_GbMnt", sx, 0, HIP_JOINT_Z-0.3, "x", "MG996R")
            # Phase 1: Steel shaft at knee with planetary stage
            planetary_gear_stage(thigh, f"{side}_KneePlanet", sx, 0, KNEE_CTR+1.5, "x", GEAR_RATIO_KNEE)
            steel_shaft_block(thigh, f"{side}_Knee_Shaft", sx, 0, KNEE_CTR+1.5, "x", SHAFT_DIAM_MAJOR, 2.5, 2)
            flanged_bearing_cavity(thigh, f"{side}_Kn_FlangeBrg", sx+0.5, 0, KNEE_CTR+1.5, "x", FLANGE_BRG_MAJOR_ID, FLANGE_BRG_MAJOR_OD, FLANGE_BRG_MAJOR_W)
            # Phase 1: Quick-release coupler at hip interface (female)
            qr_m, qr_f = quick_release_coupler(thigh, f"{side}_HipQR", sx, 0, HIP_JOINT_Z+0.5, "z", 1)
            u_bracket(thigh, f"{side}_HPB", sx, 0, HIP_JOINT_Z+0.5, 4.2, 3.4, 3.4)
            add_mg996r(thigh, f"{side}_HipP", sx, 0, HIP_JOINT_Z, "x")
            add_mg996r(thigh, f"{side}_HipR", sx, 0, THIGH_CTR+2.0, "y")
            bearing(thigh, f"{side}_HRB", sx, 0, THIGH_CTR+2.0, "y", 1.00, 0.55)
            u_bracket(thigh, f"{side}_KnB", sx, 0, KNEE_CTR+1.5, 4.0, 3.2, 3.2)
            add_mg996r(thigh, f"{side}_KneP", sx, 0, KNEE_CTR+1.5, "x")
            bearing(thigh, f"{side}_KB", sx, 0, KNEE_CTR, "x", 1.00, 0.55)
            mechanical_stop(thigh, f"{side}_KneeStop", sx, 0, KNEE_CTR+1.5, "x", 135)
            wire_channel(thigh, f"{side}_LW", sx, 0, THIGH_CTR, 0.5, 12.0, "z")
            cable_clip(thigh, f"{side}_Thigh", sx, 0.5, THIGH_CTR)
            for bz in [THIGH_CTR+3.0, THIGH_CTR-3.0]: m3_boss(thigh, f"{side}_ThighBoss_{bz:.0f}", sx, 0, bz)
            wear_plate(thigh, f"{side}_KneeWear", sx, -2.0, KNEE_CTR+0.5, 2.0, 0.60, 1.0)
            magnet_pocket(thigh, f"{side}_KLU", sx, -1.5, KNEE_CTR+1.0)
            magnet_pocket(thigh, f"{side}_KLL", sx, 1.5, KNEE_CTR+1.0)
            BOM.add("Servo", "DS3225MG 25 kg.cm servo (hip pitch)", 1, f"OP_Thigh_{side}")
            add_print_orientation_note(f"Thigh_{side}", "vertical (z-axis)", "minimal - outer panels")

            shin = new_component(f"OP_Shin_{side}")
            box(shin, "Shin_Link", sx, 0, SHIN_CTR, 4.6, 6.2, 11.2, op_blue)
            box(shin, "Shin_Armor", sx, -2.8, SHIN_CTR, 3.4, 0.36, 9.4, chrome)
            box(shin, "Shin_Rear", sx, 2.8, SHIN_CTR, 2.2, 0.36, 10.0, dark_grey)
            box(shin, "Shin_Beam", sx, 0.4, SHIN_CTR, 2.0, 2.4, 10.4, dark_metal)
            # Phase 1: Steel shaft + bearing at knee lower
            steel_shaft_block(shin, f"{side}_Knee_ShaftL", sx, 0, KNEE_CTR-0.5, "x", SHAFT_DIAM_MAJOR, 2.0, 2)
            flanged_bearing_cavity(shin, f"{side}_KL_FlangeBrg", sx+0.5, 0, KNEE_CTR-0.5, "x", FLANGE_BRG_MAJOR_ID, FLANGE_BRG_MAJOR_OD, FLANGE_BRG_MAJOR_W)
            box(shin, "KneeCap", sx, -3.0, KNEE_CTR-1.0, 3.2, 0.58, 2.4, chrome)
            box(shin, f"{side}_KneeBrgReinf", sx, 0, KNEE_CTR-0.5, 1.2, 1.8, 1.2, dark_metal)
            tt_wheel(shin, f"{side}_WF", sx+m*2.0, -4.3, SHIN_CTR+4.0, m)
            tt_wheel(shin, f"{side}_WR", sx+m*2.0, -4.3, SHIN_CTR-4.0, m)
            bearing(shin, f"{side}_KLB", sx, 0, KNEE_CTR-0.5, "x", 1.00, 0.55)
            wire_channel(shin, f"{side}_SW", sx, 0, SHIN_CTR, 0.5, 11.0, "z")
            cable_clip(shin, f"{side}_Shin", sx, 0.5, SHIN_CTR)
            cut_cavity(shin, box(shin, "FtTuck", sx, 2.8, SHIN_CTR-3.5, 5.4, 1.4, 4.4))
            magnet_pocket(shin, f"{side}_KU", sx, -1.5, KNEE_CTR-1.0)
            magnet_pocket(shin, f"{side}_KL", sx, 1.5, KNEE_CTR-1.0)
            for bz in [SHIN_CTR+3.5, SHIN_CTR-3.5]: m3_boss(shin, f"{side}_ShinBoss_{bz:.0f}", sx, 0, bz)
            add_print_orientation_note(f"Shin_{side}", "vertical (z-axis)", "minimal")

            foot = new_component(f"OP_Foot_{side}")
            box(foot, "Foot_Sole", sx, -0.4, ANKLE_CTR-1.5, 6.6, 9.6, 1.20, op_red)
            box(foot, "Foot_Grip", sx, -0.4, ANKLE_CTR-2.0, 5.6, 8.6, 0.20, rubber_blk)
            wear_plate(foot, f"{side}_SoleWear", sx, -0.4, ANKLE_CTR-2.0, 5.6, 8.6, 0.15)
            for vi in [-1.0, 0.0, 1.0]:
                box(foot, f"Arch_Vent_{int(vi*10)}", sx+vi*1.2, -0.4, ANKLE_CTR-2.04, 0.42, 5.8, 0.14, dark_grey)
            box(foot, "Heel_Block", sx-m*1.0, 3.4, ANKLE_CTR-0.8, 2.6, 3.8, 2.8, dark_grey)
            box(foot, "Heel_Plate", sx-m*0.7, 4.6, ANKLE_CTR-1.2, 3.4, 0.34, 2.2, chrome)
            cone_shape(foot, "Heel_Spur", sx-m*1.0, 5.0, ANKLE_CTR-0.2, 1.0, 0.30, 0.80, "y", op_red)
            box(foot, "Toe_Block", sx+m*0.8, -4.0, ANKLE_CTR-0.8, 2.8, 4.0, 2.2, dark_grey)
            box(foot, "Toe_Plate", sx+m*0.5, -4.8, ANKLE_CTR-1.2, 4.0, 0.34, 2.0, chrome)
            box(foot, "Toe_Cap", sx+m*1.0, -5.4, ANKLE_CTR-0.8, 3.0, 0.44, 1.6, op_red)
            box(foot, "Ankle_Guard", sx, 0, ANKLE_CTR+1.4, 5.6, 3.4, 3.0, chrome)
            box(foot, "Ankle_Inner", sx, -1.0, ANKLE_CTR+0.3, 4.2, 2.2, 1.8, dark_metal)
            box(foot, "Boot_Fin", sx+m*2.2, 0, ANKLE_CTR-0.2, 0.42, 6.8, 4.4, op_blue)
            box(foot, "Boot_Fin2", sx+m*2.7, 0, ANKLE_CTR+0.8, 0.34, 5.2, 3.0, op_red)
            box(foot, f"{side}_AnkleBrgReinf", sx, 0, ANKLE_CTR, 1.4, 2.0, 1.4, dark_metal)
            # Phase 1: Steel shaft + bearing at ankle
            steel_shaft_block(foot, f"{side}_Ankle_Shaft", sx, 0, ANKLE_CTR, "x", SHAFT_DIAM_MED, 2.0, 2)
            flanged_bearing_cavity(foot, f"{side}_Ank_FlangeBrg", sx+0.5, 0, ANKLE_CTR, "x", FLANGE_BRG_MED_ID, FLANGE_BRG_MED_OD, FLANGE_BRG_MED_W)
            add_mg996r(foot, f"{side}_AnkP", sx, 0, ANKLE_CTR+2.2, "x")
            add_mg996r(foot, f"{side}_AnkR", sx, 0, ANKLE_CTR+0.5, "y")
            bearing(foot, f"{side}_AB", sx, 0, ANKLE_CTR, "x", 1.00, 0.55)
            for bx_off in [-1.5, 1.5]: m3_boss(foot, f"{side}_FootBoss_{bx_off:.0f}", sx+bx_off, 0, ANKLE_CTR-0.5)
            align_pin(foot, f"{side}_Ft_A", sx-1.0, 0, ANKLE_CTR-1.3)
            align_pin(foot, f"{side}_Ft_B", sx+1.0, 0, ANKLE_CTR-1.3)
            add_print_orientation_note(f"Foot_{side}", "flat on sole (bottom down)", "yes - heel spur & toe cap")
        # ⑤ ARMS
        BuildGuide.add("ARM ASSEMBLY (each)", "1. Install shoulder yaw servo\n2. Install shoulder pitch servo with bearing\n3. Install shoulder roll servo\n4. Install elbow servo with bearing\n5. Route servo wires through arm channels with clips\n6. Close upper arm shell\n7. Install forearm wrist servo\n8. Install finger drive servo (DS04-NFC) in palm\n9. Route tendons through finger channels\n10. Attach fingers at MCP joints\n11. Attach thumb at CMC joint")
        for side, ax in [("L", -SHOULDER_X), ("R", SHOULDER_X)]:
            m = -1 if side == "L" else 1
            ua = new_component(f"OP_UpperArm_{side}")
            box(ua, "Sh_Block", ax, 0, SHOULDER_CTR, 5.8, 4.4, 5.8, op_red)
            box(ua, "Sh_Pad_Outer", ax+m*3.40, 0, SHOULDER_CTR, 1.10, 5.4, 6.0, op_red)
            box(ua, "Sh_Pad_Edge", ax+m*3.95, 0, SHOULDER_CTR, 0.42, 4.8, 5.4, chrome)
            for sz, sr, sh2 in [(SHOULDER_CTR+2.8, 0.44, 1.00), (SHOULDER_CTR+1.5, 0.38, 0.80)]:
                cyl(ua, f"Stk_A_{sz:.0f}", ax+m*3.5, -1.5, sz, sr, sh2, "z", chrome)
                cone_shape(ua, f"StkTip_{sz:.0f}", ax+m*3.5, -1.5, sz+sh2/2+0.38, sr, sr*0.25, 0.75, "z", dark_grey)
            box(ua, "Sh_Guard", ax+m*2.70, 0, SHOULDER_CTR-0.2, 0.44, 4.6, 6.8, op_blue)
            servo_mount_reinforcement(ua, f"{side}_ShP", ax, 0, SHOULDER_CTR, 4.8, 3.4, 3.4)
            box(ua, "UA_Link", ax, 0, ELBOW_Z+3.0, 3.4, 3.6, 5.4, op_red)
            box(ua, "UA_Skin", ax+m*1.90, 0, ELBOW_Z+3.0, 0.55, 3.6, 5.4, chrome)
            box(ua, f"{side}_UASkel", ax, 0, ELBOW_Z+3.0, 2.0, 2.0, 4.0, dark_metal)
            # Phase 1: Steel shaft at shoulder yaw, pitch, roll
            steel_shaft_block(ua, f"{side}_ShY_Shaft", ax, 0, SHOULDER_CTR+1.5, "z", SHAFT_DIAM_MED, 2.0, 2)
            flanged_bearing_cavity(ua, f"{side}_SY_FlangeBrg", ax, 0, SHOULDER_CTR+1.5, "z", FLANGE_BRG_MED_ID, FLANGE_BRG_MED_OD, FLANGE_BRG_MED_W)
            steel_shaft_block(ua, f"{side}_ShP_Shaft", ax, 0, SHOULDER_CTR, "x", SHAFT_DIAM_MED, 2.5, 2)
            flanged_bearing_cavity(ua, f"{side}_SP_FlangeBrg", ax+0.5, 0, SHOULDER_CTR, "x", FLANGE_BRG_MED_ID, FLANGE_BRG_MED_OD, FLANGE_BRG_MED_W)
            # Phase 1: Steel shaft at elbow
            steel_shaft_block(ua, f"{side}_Elb_Shaft", ax, 0, ELBOW_Z, "x", SHAFT_DIAM_MED, 2.0, 2)
            flanged_bearing_cavity(ua, f"{side}_El_FlangeBrg", ax+0.5, 0, ELBOW_Z, "x", FLANGE_BRG_MED_ID, FLANGE_BRG_MED_OD, FLANGE_BRG_MED_W)
            # Phase 1: Quick-release (female) at shoulder interface
            qr_m, qr_f = quick_release_coupler(ua, f"{side}_ShQR", ax, 0, SHOULDER_CTR+2.2, "z", 1)
            add_mg996r(ua, f"{side}_ShY", ax, 0, SHOULDER_CTR+1.5, "z")
            bearing(ua, f"{side}_SYB", ax, 0, SHOULDER_CTR+2.0, "z", 1.00, 0.55)
            u_bracket(ua, f"{side}_SPB", ax, 0, SHOULDER_CTR, 5.0, 3.6, 3.6)
            add_mg996r(ua, f"{side}_ShP", ax, 0, SHOULDER_CTR, "x")
            add_mg996r(ua, f"{side}_ShR", ax, 0, SHOULDER_CTR-1.2, "y")
            bearing(ua, f"{side}_SB", ax, 0, SHOULDER_CTR, "x", 1.10, 0.62)
            mechanical_stop(ua, f"{side}_ShStop", ax, 0, SHOULDER_CTR-0.5, "x", -175)
            mechanical_stop(ua, f"{side}_ShStopB", ax, 0, SHOULDER_CTR-0.5, "x", 60)
            u_bracket(ua, f"{side}_EB", ax, 0, ELBOW_Z, 4.0, 3.2, 3.2)
            add_mg996r(ua, f"{side}_ElbP", ax, 0, ELBOW_Z, "x")
            bearing(ua, f"{side}_EBr", ax, 0, ELBOW_Z-0.5, "x", 0.95, 0.52)
            mechanical_stop(ua, f"{side}_ElbStop", ax, 0, ELBOW_Z, "x", 150)
            wire_channel(ua, f"{side}_UAW", ax, 0, ELBOW_Z+3.0, 0.4, 6.0, "z")
            cable_clip(ua, f"{side}_UA", ax, 0.6, ELBOW_Z+3.0)
            m3_boss(ua, f"{side}_UAboss", ax, 0, ELBOW_Z+3.0)
            add_print_orientation_note(f"UpperArm_{side}", "vertical (z-axis)", "yes - shoulder spikes")

            fa = new_component(f"OP_Forearm_{side}")
            box(fa, "FA_Link", ax, 0, WRIST_Z+3.5, 3.4, 4.0, 5.0, op_blue)
            box(fa, "FA_Fender", ax+m*2.2, 0, WRIST_Z+3.5, 0.55, 5.4, 6.0, op_red)
            box(fa, "FA_Back", ax, 2.4, WRIST_Z+3.5, 2.8, 0.40, 5.0, chrome)
            box(fa, "FA_Vent_L", ax-0.7, -1.9, WRIST_Z+3.5, 0.32, 0.24, 3.2, dark_grey)
            box(fa, "FA_Vent_R", ax+0.7, -1.9, WRIST_Z+3.5, 0.32, 0.24, 3.2, dark_grey)
            box(fa, f"{side}_FASkel", ax, 0, WRIST_Z+3.5, 2.0, 2.2, 4.0, dark_metal)
            # Phase 1: Steel shaft at wrist
            steel_shaft_block(fa, f"{side}_Wrist_Shaft", ax, 0, WRIST_Z+0.8, "x", SHAFT_DIAM_MINOR, 1.5, 2)
            flanged_bearing_cavity(fa, f"{side}_WR_FlangeBrg", ax+0.3, 0, WRIST_Z+0.8, "x", FLANGE_BRG_MINOR_ID, FLANGE_BRG_MINOR_OD, FLANGE_BRG_MINOR_W)
            add_mg90s(fa, f"{side}_WR", ax, 0, WRIST_Z+0.8, "x")
            bearing(fa, f"{side}_WB", ax, 0, WRIST_Z+0.5, "x", 0.80, 0.44)
            wire_channel(fa, f"{side}_FAW", ax, 0, WRIST_Z+3.5, 0.4, 5.0, "z")
            cable_clip(fa, f"{side}_FA", ax, 0.6, WRIST_Z+3.5)
            m3_boss(fa, f"{side}_FAboss", ax, 0, WRIST_Z+4.2)
            add_print_orientation_note(f"Forearm_{side}", "vertical (z-axis)", "minimal - vents")

            hand = new_component(f"OP_Hand_{side}")
            box(hand, "Palm_Main", ax, -0.4, WRIST_Z-1.6, 3.6, 3.2, 2.4, dark_grey)
            box(hand, "Palm_Inner", ax, 0.6, WRIST_Z-1.6, 2.8, 2.2, 2.2, black_plastic)
            MCP_Z = WRIST_Z - PALM_BOTTOM_OFFSET
            for ki, kx_off in enumerate(FING_X_OFF):
                cyl(hand, f"Knuckle_{ki}", ax + m*kx_off, -1.5, MCP_Z + 0.15, 0.30, 0.40, "y", chrome)
            cyl(hand, "Wrist_Ring", ax, 0, WRIST_Z-0.4, 1.10, 0.44, "z", chrome)
            box(hand, "Hand_Panel", ax+m*1.0, -0.7, WRIST_Z-1.3, 0.40, 3.0, 3.0, op_red)
            box(hand, "Finger_SrvBay", ax, 0.5, WRIST_Z-1.8, 1.6, 1.00, 2.6, black_plastic)
            add_ds04_nfc(hand, f"{side}_FingerDrv", ax, 0.5, WRIST_Z-2.0, "z")
            BOM.add("Servo", "DS04-NFC 9g digital servo (finger drive)", 2, f"OP_Hand_{side}")
            for pi, px in enumerate([ax + m*(x) for x in [-0.8, -0.2, 0.4, 1.0]]):
                cyl(hand, f"{side}_PalmPulley_{pi}", px, -0.4, WRIST_Z-2.5, 0.12, 0.10, "y", chrome)
            for lxi in [-0.7, 0.7]: led_pocket_5mm(hand, f"PalmLED_{lxi:.0f}", ax+lxi, -1.65, WRIST_Z-1.6, "y")

            for fi, (fname, fxo, pp_l, mp_l, dp_l) in enumerate(zip(FING_NAMES, FING_X_OFF, FING_PP, FING_MP, FING_DP)):
                fc = new_component(f"OP_{fname}_{side}")
                fx = ax + m * fxo; fy = -1.35; mcp_z = MCP_Z
                pp_cz = mcp_z - pp_l/2; mp_cz = mcp_z - pp_l - mp_l/2; dp_cz = mcp_z - pp_l - mp_l - dp_l/2
                box(fc, "PP", fx, fy, pp_cz, FING_W, FING_H, pp_l, grey_plastic)
                cyl(fc, "PIP_Hinge", fx, fy, mcp_z - pp_l, 0.27, FING_W + 0.12, "x", chrome)
                box(fc, "MP", fx, fy, mp_cz, FING_W*0.94, FING_H*0.94, mp_l, grey_plastic)
                cyl(fc, "DIP_Hinge", fx, fy, mcp_z - pp_l - mp_l, 0.24, FING_W*0.94 + 0.10, "x", chrome)
                box(fc, "DP", fx, fy, dp_cz, FING_W*0.86, FING_H*0.86, dp_l, grey_plastic)
                cone_shape(fc, "FT", fx, fy, mcp_z - pp_l - mp_l - dp_l - 0.12, FING_W*0.44, 0.05, 0.24, "z", chrome)
                tendon_finger_system(fc, f"{side}_{fname}", fx, fy, mcp_z, pp_l, mp_l, dp_l, m)

            thumb = new_component(f"OP_Thumb_{side}")
            tx = ax + m * 1.70; ty = 0.20
            tpp_cz = MCP_Z - THUMB_PP_L/2; tdp_cz = MCP_Z - THUMB_PP_L - THUMB_DP_L/2
            box(thumb, "TP", tx, ty, tpp_cz, THUMB_W, THUMB_H, THUMB_PP_L, grey_plastic)
            cyl(thumb, "T_Hinge", tx, ty, MCP_Z - THUMB_PP_L, 0.28, THUMB_W + 0.14, "x", chrome)
            box(thumb, "TD", tx, ty*0.8, tdp_cz, THUMB_W*0.86, THUMB_H*0.86, THUMB_DP_L, grey_plastic)
            cone_shape(thumb, "TT", tx, ty, tdp_cz - THUMB_DP_L/2 - 0.14, THUMB_W*0.44, 0.05, 0.28, "z", chrome)
            tendon_finger_system(thumb, f"{side}_Thumb", tx, ty, MCP_Z, THUMB_PP_L, THUMB_DP_L, 0, m)

            if side == "R":
                blast = new_component("OP_Ion_Blaster")
                BuildGuide.add("ION BLASTER ASSEMBLY", "1. Print barrel, body, scope\n2. Install 5mm LED in muzzle\n3. Attach to hand via hinge block")
                cyl(blast, "Barrel_Main", ax, -2.2, WRIST_Z-3.2, 0.92, 4.2, "z", dark_metal)
                cyl(blast, "Barrel_Tip", ax, -2.2, WRIST_Z-5.6, 0.68, 1.1, "z", chrome)
                cyl(blast, "Barrel_Inner", ax, -2.2, WRIST_Z-3.8, 0.44, 2.4, "z", black_plastic)
                box(blast, "Blaster_Body", ax, -1.1, WRIST_Z-3.2, 2.6, 2.4, 2.7, dark_metal)
                box(blast, "Blast_Guard", ax, -0.2, WRIST_Z-3.2, 2.8, 0.38, 1.6, chrome)
                box(blast, "Blast_Rail_T", ax, -0.6, WRIST_Z-2.0, 2.8, 0.22, 0.30, chrome)
                box(blast, "Blast_Rail_B", ax, -0.6, WRIST_Z-4.4, 2.8, 0.22, 0.30, chrome)
                box(blast, "Hinge_Block", ax, -0.8, WRIST_Z-1.6, 1.1, 0.65, 1.1, dark_metal)
                cyl(blast, "Scope_Body", ax+1.5, -2.2, WRIST_Z-3.2, 0.42, 2.2, "z", chrome)
                cyl(blast, "Scope_Lens", ax+1.5, -2.2, WRIST_Z-4.4, 0.28, 0.25, "z", glass_clr)
                led_pocket_5mm(blast, "Muzzle", ax, -2.2, WRIST_Z-6.2, "z")
        # ⑥ BACKPACK
        bp = new_component("OP_Backpack")
        BuildGuide.add("BACKPACK ASSEMBLY", "1. Print core, hood, top flap\n2. Attach to torso via rigid joint\n3. Install magnets on roof flap")
        box(bp, "BP_Core", 0, 5.8, TORSO_CTR+0.5, 7.4, 2.8, 9.4, dark_grey)
        box(bp, "BP_Hood", 0, 6.7, TORSO_CTR+1.0, 6.0, 1.2, 8.0, op_red)
        box(bp, "BP_TopFlap", 0, 5.2, TORSO_CTR+5.6, 8.6, 0.40, 5.6, op_red)
        box(bp, "BP_Rad", 0, 7.1, TORSO_CTR-0.5, 5.6, 0.46, 5.9, chrome)
        box(bp, "Exh_Blk", 0, 6.5, TORSO_CTR+2.8, 3.4, 0.70, 2.2, dark_metal)
        cyl(bp, "Exh_L", -1.4, 6.9, TORSO_CTR+2.8, 0.42, 1.4, "y", dark_metal)
        cyl(bp, "Exh_R", 1.4, 6.9, TORSO_CTR+2.8, 0.42, 1.4, "y", dark_metal)
        add_mg90s(bp, "Roof_Hinge", 0, 5.2, TORSO_CTR+5.2, "x")
        bearing(bp, "Roof_Brg", 0, 5.2, TORSO_CTR+5.4, "x", 0.80, 0.44)
        magnet_pocket(bp, "RoofL", -2.8, 5.2, TORSO_CTR+5.8)
        magnet_pocket(bp, "RoofR", 2.8, 5.2, TORSO_CTR+5.8)
        add_print_orientation_note("Backpack", "flat on rear face", "minimal")

        # ⑦ STEER WHEEL PODS
        steer = new_component("OP_SteerPods")
        for side2, sx2 in [("L", -5.8), ("R", 5.8)]:
            m2 = -1 if side2 == "L" else 1
            box(steer, f"SAr_{side2}", sx2, -3.7, 23.9, 1.8, 1.4, 4.4, chrome)
            box(steer, f"SPod_{side2}", sx2, -4.7, 23.4, 3.2, 2.4, 3.4, dark_grey)
            tt_wheel(steer, f"SW_{side2}", sx2+m2*2.0, -4.3, 23.4, m2)
            bearing(steer, f"SPiv_{side2}", sx2, -3.7, 23.9, "z", 0.95, 0.50)
            add_mg90s(steer, f"SSrv_{side2}", sx2, -4.3, 23.9, "z")

        # ⑧ SHIELDS / PANELS
        shields = new_component("OP_Shields")
        for side2, sx2 in [("L", -(SHOULDER_X+3.6)), ("R", SHOULDER_X+3.6)]:
            m2 = -1 if side2 == "L" else 1
            box(shields, f"ShShield_{side2}", sx2, 0, SHOULDER_CTR+1.5, 1.2, 4.8, 5.4, chrome)
            box(shields, f"ShHinge_{side2}", sx2-m2*0.8, 0, SHOULDER_CTR+1.5, 0.55, 2.0, 2.0, dark_grey)
            box(shields, f"Mirror_{side2}", sx2+m2*0.6, -3.0, SHOULDER_CTR+2.0, 1.6, 0.22, 1.0, dark_grey)
        for side2, hx2 in [("L", -(HIP_X+3.3)), ("R", HIP_X+3.3)]:
            box(shields, f"HipShield_{side2}", hx2, 0, PELVIS_CTR+0.5, 1.2, 4.6, 4.2, op_blue)

        # PHY-17: Assembly Jigs
        BuildGuide.add("ASSEMBLY JIGS", "Print jigs for:\n- Torso alignment (8 pins)\n- Pelvis alignment (4 pins)\n- Head alignment (4 pins)\nUse jigs to hold shell halves during screw installation")
        alignment_jig("Torso", 0, 0, TORSO_CTR, 8.0, 1.0, 6.0,
                      pin_positions=[(-5.0, 0, TORSO_CTR), (5.0, 0, TORSO_CTR),
                                     (-3.0, 0, TORSO_CTR+4.0), (3.0, 0, TORSO_CTR+4.0),
                                     (-3.0, 0, TORSO_CTR-4.0), (3.0, 0, TORSO_CTR-4.0)])
        alignment_jig("Pelvis", 0, 0, PELVIS_CTR, 12.0, 1.0, 4.0,
                      pin_positions=[(-5.0, 0, PELVIS_CTR), (5.0, 0, PELVIS_CTR),
                                     (0, 0, PELVIS_CTR+1.5), (0, 0, PELVIS_CTR-1.5)])

        # ═════════════════════════════════════════════════════════════════
        # FDM SHELL SPLITTING
        # ═════════════════════════════════════════════════════════════════
        for comp in comps_list:
            if comp in jig_comps: continue
            to_split = [b for b in comp.bRepBodies if b.name and any(k in b.name for k in SPLIT_KEYS)]
            for b in to_split:
                split_halves(comp, b, "y", 0.0)

        # PHY-13: Merge fasteners into correct halves after splitting
        for comp in comps_list:
            if comp in jig_comps: continue
            split_bodies = [b for b in comp.bRepBodies if b.name and "y_Split" in b.name]
            i = 0
            while i < len(split_bodies) - 1:
                merge_fasteners_to_halves(comp, split_bodies[i], split_bodies[i+1])
                i += 2
        # ═════════════════════════════════════════════════════════════════
        # KINEMATICS
        # ═════════════════════════════════════════════════════════════════
        t = occs.get("OP_Torso"); p = occs.get("OP_Pelvis"); h = occs.get("OP_Head")
        b = occs.get("OP_Backpack"); st = occs.get("OP_SteerPods"); sh = occs.get("OP_Shields")
        if p: p.isGrounded = True
        ball_joint("Waist_Cluster", t, p, 0, 0, WAIST_CTR-2.5)
        ball_joint("Neck_Cluster", h, t, 0, 0, NECK_JOINT_Z)
        rigid_joint("Backpack_Mount", b, t)
        rigid_joint("Steer_Mount", st, p)
        rigid_joint("Shields_Mount", sh, t)
        for side in ["L", "R"]:
            sx = -HIP_X if side == "L" else HIP_X
            ax = -SHOULDER_X if side == "L" else SHOULDER_X
            m = -1 if side == "L" else 1
            th = occs.get(f"OP_Thigh_{side}"); sn = occs.get(f"OP_Shin_{side}")
            fo = occs.get(f"OP_Foot_{side}"); ua = occs.get(f"OP_UpperArm_{side}")
            fa = occs.get(f"OP_Forearm_{side}"); ha = occs.get(f"OP_Hand_{side}")
            ball_joint(f"{side}_Hip_Cluster", th, p, sx, 0, HIP_JOINT_Z)
            revolute_joint(f"{side}_Knee", sn, th, sx, 0, KNEE_CTR+1.5, "x")
            ball_joint(f"{side}_Ankle_Cluster", fo, sn, sx, 0, ANKLE_CTR+2.2)
            ball_joint(f"{side}_Shoulder_Cluster", ua, t, ax, 0, SHOULDER_CTR)
            revolute_joint(f"{side}_Elbow", fa, ua, ax, 0, ELBOW_Z, "x")
            ball_joint(f"{side}_Wrist", ha, fa, ax, 0, WRIST_Z+0.8)
            MCP_Z = WRIST_Z - PALM_BOTTOM_OFFSET
            for fi, fname in enumerate(FING_NAMES):
                fx = ax + m * FING_X_OFF[fi]
                fo_f = occs.get(f"OP_{fname}_{side}")
                revolute_joint(f"{side}_{fname}_MCP", fo_f, ha, fx, -1.35, MCP_Z, "x")
            tx = ax + m * 1.70
            thumb_occ = occs.get(f"OP_Thumb_{side}")
            ball_joint(f"{side}_Thumb_CMC", thumb_occ, ha, tx, 0.20, MCP_Z)
            if side == "R":
                bl = occs.get("OP_Ion_Blaster")
                if bl: revolute_joint("Blaster_Fold", bl, ha, ax, 0, WRIST_Z-1.0, "y")

        try:
            orphans = []; jointed = set()
            for i in range(root.asBuiltJoints.count):
                j = root.asBuiltJoints.item(i)
                if j.occurrenceOne: jointed.add(j.occurrenceOne.component.name)
                if j.occurrenceTwo: jointed.add(j.occurrenceTwo.component.name)
            for c in comps_list:
                if c.name not in ("OP_Torso", "OP_Pelvis") and c.name not in jointed:
                    orphans.append(c.name)
            if orphans: Logger.log(f"  !!! ORPHANS: {orphans}", "WARN")
            else: Logger.log("  All components bound to kinematic chain. [OK]")
        except Exception: Logger.log("  Kinematic validation skipped (MCP env).", "WARN")

        try: cam = app.activeViewport.camera; cam.isFitView = True; app.activeViewport.camera = cam
        except Exception: pass

        # PHY-18: Run fastener validation
        FastenerValidator.clear()
        FastenerValidator.check("M3x8", 0.8, 0.5, 0.20, "torso shell main")
        FastenerValidator.check("M3x10", 1.0, 0.5, 0.30, "torso bracket")
        FastenerValidator.check("M3x12", 1.2, 0.6, 0.30, "hip bracket")
        FastenerValidator.check("M2.5x6", 0.6, 0.4, 0.15, "RPi mount")
        FastenerValidator.report()
        # ═════════════════════════════════════════════════════════════════
        # SIMULATION ENGINE  v11.0
        # ═════════════════════════════════════════════════════════════════

        class SimulationEngine:
            BALL_JOINTS = {"Waist_Cluster", "Neck_Cluster", "L_Hip_Cluster", "R_Hip_Cluster",
                "L_Ankle_Cluster", "R_Ankle_Cluster", "L_Shoulder_Cluster", "R_Shoulder_Cluster",
                "L_Wrist", "R_Wrist", "L_Thumb_CMC", "R_Thumb_CMC"}
            REV_JOINTS = {"L_Knee", "R_Knee", "L_Elbow", "R_Elbow", "Blaster_Fold",
                "L_Pinky_MCP", "L_Ring_MCP", "L_Middle_MCP", "L_Index_MCP",
                "R_Pinky_MCP", "R_Ring_MCP", "R_Middle_MCP", "R_Index_MCP"}

            def __init__(self, root, comps_list, design, app, ui):
                self._root = root; self._comps = comps_list; self._design = design
                self._app = app; self._ui = ui; self._cols = []; self._logged_joints = False

            def _gj(self, name):
                try: j = self._root.asBuiltJoints.itemByName(name)
                except Exception: return None
                if j is not None: return j
                try:
                    if not self._logged_joints:
                        self._logged_joints = True
                        names = [self._root.asBuiltJoints.item(i).name for i in range(self._root.asBuiltJoints.count)]
                        Logger.log(f"Joint '{name}' not found. Available: {names}", "WARN")
                except Exception: pass
                return None

            @staticmethod
            def _ease(t): return t * t * (3.0 - 2.0 * t)

            def _get(self, mo, axis):
                try:
                    if mo.objectType == adsk.fusion.RevoluteJointMotion.classType(): return mo.rotationValue
                    if mo.objectType == adsk.fusion.BallJointMotion.classType(): return getattr(mo, axis + "Value")
                except: return 0.0

            def _set(self, mo, axis, val):
                try:
                    if mo.objectType == adsk.fusion.RevoluteJointMotion.classType(): mo.rotationValue = val
                    elif mo.objectType == adsk.fusion.BallJointMotion.classType(): setattr(mo, axis + "Value", val)
                except: pass

            def _refresh(self):
                if os.path.exists(STOP_FLAG):
                    try: os.remove(STOP_FLAG)
                    except: pass
                    raise Exception("SIMULATION_ABORTED_BY_USER")
                try: self._app.activeViewport.refresh()
                except: pass
                try: adsk.doEvents()
                except: pass

            def _clamp(self, joint_name, axis, deg):
                limits = JOINT_LIMITS.get(joint_name, {}).get(axis)
                if limits:
                    lo, hi = limits
                    if deg < lo or deg > hi: Logger.log(f"CLAMP: {joint_name}.{axis} {deg:.0f} deg -> [{lo} deg,{hi} deg]", "WARN")
                    return max(lo, min(hi, deg))
                return deg

            def move_joint(self, name, deg, steps=20, axis="pitch", ease=True, clamp=True):
                j = self._gj(name)
                if not j: return
                if clamp: deg = self._clamp(name, axis, deg)
                mo = j.jointMotion; e_rad = math.radians(deg); s_rad = self._get(mo, axis)
                for i in range(1, steps + 1):
                    t = self._ease(i/steps) if ease else i/steps
                    self._set(mo, axis, s_rad + (e_rad - s_rad) * t); self._refresh()

            def move_group(self, targets, steps=20, axis="pitch", ease=True, clamp=True):
                active = []
                for item in targets:
                    name = item[0]; deg = item[1]; ax = item[2] if len(item) > 2 else axis
                    j = self._gj(name)
                    if not j: continue
                    d = self._clamp(name, ax, deg) if clamp else deg
                    mo = j.jointMotion; active.append((mo, ax, self._get(mo, ax), math.radians(d)))
                for i in range(1, steps + 1):
                    t = self._ease(i/steps) if ease else i/steps
                    for mo, ax, s_rad, e_rad in active: self._set(mo, ax, s_rad + (e_rad - s_rad) * t)
                    self._refresh()

            def move_ball(self, targets, steps=20, clamp=True):
                active = []
                for name, pitch, yaw, roll in targets:
                    j = self._gj(name)
                    if not j: continue
                    mo = j.jointMotion; is_rev = (mo.objectType == adsk.fusion.RevoluteJointMotion.classType())
                    if is_rev:
                        if pitch is not None:
                            v = self._clamp(name, "pitch", pitch) if clamp else pitch
                            active.append((mo, "pitch", self._get(mo, "pitch"), math.radians(v)))
                    else:
                        for axis, val in [("pitch", pitch), ("yaw", yaw), ("roll", roll)]:
                            if val is None: continue
                            v = self._clamp(name, axis, val) if clamp else val
                            active.append((mo, axis, self._get(mo, axis), math.radians(v)))
                for i in range(1, steps + 1):
                    t = self._ease(i / steps)
                    for mo, axis, s_rad, e_rad in active: self._set(mo, axis, s_rad + (e_rad - s_rad) * t)
                    self._refresh()

            def reset_all(self, steps=10, groups=None):
                ball_names = self.BALL_JOINTS if groups is None else {n for n in groups if n in self.BALL_JOINTS}
                rev_names = self.REV_JOINTS if groups is None else {n for n in groups if n in self.REV_JOINTS}
                ball_targets = [(n, 0.0, 0.0, 0.0) for n in ball_names if self._gj(n)]
                rev_targets = [n for n in rev_names if self._gj(n)]
                if ball_targets: self.move_ball(ball_targets, steps=steps)
                for name in rev_targets: self.move_joint(name, 0.0, steps, "pitch", ease=True)
                self._refresh(); Logger.log("-> reset to neutral")
            def _interfere(self, label="Interference"):
                try:
                    all_bodies = adsk.core.ObjectCollection.create()
                    for comp in self._comps:
                        for body in list(comp.bRepBodies):
                            if body.name and not any(t in body.name for t in SKIP_TAGS): all_bodies.add(body)
                    inter_input = self._design.createInterferenceInput(all_bodies)
                    inter_input.isCoincidentFacesInterference = False
                    results = self._design.analyzeInterference(inter_input)
                    count = results.count
                    if count:
                        Logger.log(f"  {label}: {count} collision(s)")
                        for i in range(min(count, 5)):
                            r = results.item(i)
                            Logger.log(f"    [{r.entityOne.name}] <-> [{r.entityTwo.name}]")
                        if count > 5: Logger.log(f"    ...{count-5} more")
                    else: Logger.log(f"  {label}: [OK] 0 collisions")
                    self._cols.append((label, count))
                except Exception as e: Logger.log(f"  {label}: skipped ({e})", "WARN"); self._cols.append((label, -1))

            def _check_zmp(self, label):
                FOOT_HW = 6.6 / 2.0; FOOT_HL = 9.6 / 2.0
                try: com = self._root.physicalProperties.centerOfMass
                except: Logger.log(f"  ZMP [{label}]: CoM unavailable", "WARN"); return
                l_cx, r_cx = -HIP_X, HIP_X
                in_DS = (min(l_cx - FOOT_HW, r_cx - FOOT_HW) <= com.x <= max(l_cx + FOOT_HW, r_cx + FOOT_HW) and -FOOT_HL <= com.y <= FOOT_HL)
                tag = "[OK] ZMP STABLE" if in_DS else "[FAIL] ZMP UNSTABLE"
                Logger.log(f"  ZMP [{label:16s}] {tag}  CoM=({com.x:+.2f}, {com.y:+.2f}, {com.z:.1f}) cm")
                return in_DS

            def _set_fingers(self, side, degrees, thumb_deg=None, steps=12):
                targets = [(f"{side}_{n}_MCP", degrees, None, None) for n in FING_NAMES]
                self.move_ball(targets, steps=steps)
                if thumb_deg is not None: self.move_ball([(f"{side}_Thumb_CMC", thumb_deg, 0, 0)], steps=steps)

            def gesture_open(self, side="R", steps=10): self._set_fingers(side, 0, thumb_deg=0, steps=steps); Logger.log(f"  Gesture: {side} hand OPEN")
            def gesture_fist(self, side="R", steps=12): self._set_fingers(side, 80, thumb_deg=50, steps=steps); Logger.log(f"  Gesture: {side} hand FIST")
            def gesture_point(self, side="R", steps=10):
                targets = [(f"{side}_Pinky_MCP",80,None,None),(f"{side}_Ring_MCP",80,None,None),(f"{side}_Middle_MCP",80,None,None),(f"{side}_Index_MCP",-3,None,None),(f"{side}_Thumb_CMC",40,0,0)]
                self.move_ball(targets, steps=steps); Logger.log(f"  Gesture: {side} hand POINT")
            def gesture_snap(self, side="R", steps=6):
                targets = [(f"{side}_Pinky_MCP",80,None,None),(f"{side}_Ring_MCP",70,None,None),(f"{side}_Middle_MCP",-3,None,None),(f"{side}_Index_MCP",-3,None,None),(f"{side}_Thumb_CMC",55,0,0)]
                self.move_ball(targets, steps=steps); Logger.log(f"  Gesture: {side} hand SNAP")

            def test_arm_workspace(self):
                Logger.log("--- SIM-3: ARM WORKSPACE TEST ---")
                poses = [(-30,0,0,"Forward low"),(-90,0,90,"Forward high"),(-120,0,90,"Overhead"),(-80,80,60,"Side reach"),(-80,-80,60,"Cross reach"),(-40,0,130,"Bicep curl"),(-175,0,80,"Reach back"),(-80,0,150,"Elbow max")]
                for (sp, sy, ep, lbl) in poses:
                    for side in ["L","R"]:
                        self.move_ball([(f"{side}_Shoulder_Cluster", sp, sy, 0)], steps=10)
                        self.move_joint(f"{side}_Elbow", ep, steps=8, axis="pitch")
                    self._interfere(f"Workspace: {lbl}")
                    self.reset_all(steps=8, groups=["L_Shoulder_Cluster","R_Shoulder_Cluster","L_Elbow","R_Elbow"])

            def test_joint_rom(self):
                self.reset_all(steps=10); Logger.log("--- MODULE 1 / 9 ---"); Logger.log("MODULE 1: JOINT ROM TEST")
                for name, axes in JOINT_LIMITS.items():
                    for axis, (lo, hi) in axes.items():
                        for lbl, angle in [("MIN", lo), ("MAX", hi)]:
                            self.move_joint(name, angle, steps=15, axis=axis)
                            self._interfere(f"{lbl} {name} {axis}")
                            self.move_joint(name, 0, steps=10, axis=axis)

            def simulate_head_scan(self):
                self.reset_all(steps=10); Logger.log("--- MODULE 2 / 9 ---"); Logger.log("MODULE 2: HEAD LOOK-AROUND")
                for yaw_deg in [0, -50, 0, 50, 0]: self.move_joint("Neck_Cluster", yaw_deg, steps=18, axis="yaw")
                for pitch_deg in [0, -25, 0, 35, 0]: self.move_joint("Neck_Cluster", pitch_deg, steps=18, axis="pitch")

            def simulate_wave(self):
                self.reset_all(steps=10); Logger.log("--- MODULE 3 / 9 ---"); Logger.log("MODULE 3: WAVE GESTURE")
                self.gesture_open("R")
                self.move_joint("R_Shoulder_Cluster", -45, steps=15, axis="pitch")
                self.move_joint("R_Shoulder_Cluster", 80, steps=15, axis="yaw")
                self.move_joint("R_Elbow", 90, steps=12, axis="pitch")
                for _ in range(3):
                    self.move_ball([("R_Wrist", None, None, -30)], steps=8)
                    self.move_ball([("R_Wrist", None, None, 30)], steps=8)
                self.reset_all(steps=12)

            def simulate_idle_breathing(self):
                self.reset_all(steps=8); Logger.log("--- MODULE 4 / 9 ---"); Logger.log("MODULE 4: IDLE BREATHING")
                for _ in range(4):
                    self.move_joint("Waist_Cluster", -3, steps=12, axis="pitch")
                    self.move_joint("Waist_Cluster", 3, steps=12, axis="pitch")
                self.move_joint("Waist_Cluster", 0, steps=8, axis="pitch")

            def simulate_walking_advanced(self):
                self.reset_all(steps=10); Logger.log("--- MODULE 5 / 9 ---"); Logger.log("MODULE 5: ADVANCED WALKING (ZMP checked)")
                for cycle in range(4):
                    phase = cycle % 2; l_sign = 1 if phase == 0 else -1; r_sign = -1 if phase == 0 else 1
                    self.move_ball([("L_Hip_Cluster",25*l_sign,10*l_sign,5*l_sign),("R_Hip_Cluster",25*r_sign,10*r_sign,5*r_sign),("L_Shoulder_Cluster",8*l_sign,15*l_sign,5*l_sign),("R_Shoulder_Cluster",8*r_sign,15*r_sign,5*r_sign),("L_Knee",60,None,None),("R_Knee",60,None,None),("L_Ankle_Cluster",15*l_sign,None,8*l_sign),("R_Ankle_Cluster",15*r_sign,None,8*r_sign)], steps=20)
                    self._check_zmp(f"Walk cycle {cycle+1}")
                self.reset_all(steps=12)

            def simulate_running(self):
                self.reset_all(steps=8); Logger.log("--- MODULE 6 / 9 ---"); Logger.log("MODULE 6: RUNNING")
                for cycle in range(3):
                    phase = cycle % 2; l_sign = 1 if phase == 0 else -1; r_sign = -1 if phase == 0 else 1
                    self.move_ball([("L_Hip_Cluster",30*l_sign,20*l_sign,10*l_sign),("R_Hip_Cluster",30*r_sign,20*r_sign,10*r_sign),("L_Shoulder_Cluster",15*l_sign,25*l_sign,10*l_sign),("R_Shoulder_Cluster",15*r_sign,25*r_sign,10*r_sign),("L_Knee",95,None,None),("R_Knee",95,None,None),("L_Ankle_Cluster",25*l_sign,None,12*l_sign),("R_Ankle_Cluster",25*r_sign,None,12*r_sign)], steps=14)
                self.reset_all(steps=10)

            def simulate_combat(self):
                self.reset_all(steps=10); Logger.log("--- MODULE 7 / 9 ---"); Logger.log("MODULE 7: COMBAT SEQUENCE")
                self.gesture_fist("L"); self.gesture_fist("R")
                self.move_joint("R_Shoulder_Cluster", -45, steps=10, axis="pitch")
                self.move_joint("R_Shoulder_Cluster", 45, steps=8, axis="yaw")
                self.move_joint("R_Elbow", 120, steps=8, axis="pitch")
                self.move_ball([("R_Wrist", None, None, 20)], steps=6)
                self.move_joint("R_Shoulder_Cluster", -10, steps=6, axis="pitch")
                self.move_joint("R_Shoulder_Cluster", -80, steps=10, axis="pitch")
                self.move_joint("R_Shoulder_Cluster", -30, steps=6, axis="yaw")
                self.move_joint("R_Elbow", 30, steps=8, axis="pitch")
                self.move_joint("L_Shoulder_Cluster", 30, steps=8, axis="pitch")
                self.move_joint("L_Elbow", 45, steps=6, axis="pitch")
                self._check_zmp("Combat stance")
                self.reset_all(steps=12)

            def _transform_to_truck(self, steps_scale=1.0):
                s = steps_scale
                self.move_group([("R_Elbow",0,"pitch"),("L_Elbow",0,"pitch"),("Blaster_Fold",-90,"pitch")], steps=int(20*s))
                self.move_ball([("L_Wrist",90,None,90),("R_Wrist",135,None,90)], steps=int(20*s))
                self.move_ball([("Neck_Cluster",-90,0,0)], steps=int(15*s))
                self.move_ball([("L_Shoulder_Cluster",-88,0,0),("R_Shoulder_Cluster",-88,0,0)], steps=int(22*s))
                self.move_ball([("L_Hip_Cluster",0,90,0),("R_Hip_Cluster",0,90,0)], steps=int(22*s))
                self.move_ball([("L_Ankle_Cluster",0,90,0),("R_Ankle_Cluster",0,90,0)], steps=int(18*s))

            def _transform_to_robot(self, steps_scale=1.0):
                s = steps_scale
                self.move_ball([("L_Ankle_Cluster",0,0,0),("R_Ankle_Cluster",0,0,0)], steps=int(18*s))
                self.move_ball([("L_Hip_Cluster",0,0,0),("R_Hip_Cluster",0,0,0)], steps=int(22*s))
                self.move_ball([("L_Shoulder_Cluster",0,0,0),("R_Shoulder_Cluster",0,0,0)], steps=int(22*s))
                self.move_ball([("Neck_Cluster",0,0,0)], steps=int(15*s))
                self.move_ball([("L_Wrist",0,None,0),("R_Wrist",0,None,0)], steps=int(18*s))
                self.move_group([("Blaster_Fold",0,"pitch"),("R_Elbow",0,"pitch"),("L_Elbow",0,"pitch")], steps=int(18*s))

            def simulate_transformation(self):
                self.reset_all(steps=10); Logger.log("--- MODULE 8a / 9 ---"); Logger.log("MODULE 8a: TRANSFORMATION  Robot -> Truck")
                self._transform_to_truck(steps_scale=1.0); self._interfere("Truck-mode check")
                Logger.log("MODULE 8c: TRUCK -> ROBOT"); self._transform_to_robot(steps_scale=1.0)
                Logger.log("Robot mode restored."); self.reset_all(steps=10)
            def run_stability_analysis(self):
                Logger.log("--- MODULE 9 / 9 ---"); Logger.log("MODULE 9a: STABILITY ANALYSIS (ZMP)")
                poses = {"Attention": {"Waist_Cluster": (0,0,0)},
                    "Combat": {"Waist_Cluster":(10,0,0),"L_Knee":45,"R_Knee":45,"L_Elbow":90,"R_Elbow":90,"R_Shoulder_Cluster":(0,30,-45)},
                    "Squat": {"Waist_Cluster":(20,0,0),"L_Knee":90,"R_Knee":90,"L_Hip_Cluster":(0,-45,0),"R_Hip_Cluster":(0,-45,0)},
                    "Victory": {"L_Shoulder_Cluster":(0,60,-90),"R_Shoulder_Cluster":(0,60,-90),"L_Elbow":30,"R_Elbow":30,"Waist_Cluster":(15,0,0)},
                    "Single_Leg_L": {"L_Hip_Cluster":(0,90,0),"L_Knee":90,"Waist_Cluster":(5,10,-5)}}
                for pose_name, config in poses.items():
                    self.reset_all(steps=10)
                    for key, val in config.items():
                        if isinstance(val, tuple): self.move_ball([(key, val[0], val[1], val[2])], steps=15)
                        else: self.move_joint(key, val, steps=12, axis="pitch")
                    self._check_zmp(pose_name)

            def estimate_servo_loads(self):
                Logger.log("MODULE 9b: SERVO LOAD ESTIMATION (v11 upgraded servos)")
                loads = [("Neck Pitch",120,3.0,SERVO_SPECS["micro"]),("L Shoulder Pitch",390,12.0,SERVO_SPECS["std"]),("R Shoulder Pitch",390,12.0,SERVO_SPECS["std"]),("L Elbow",210,7.0,SERVO_SPECS["std"]),("R Elbow",210,7.0,SERVO_SPECS["std"]),("L Hip Pitch",820,15.0,SERVO_SPECS["hip_hd"]),("R Hip Pitch",820,15.0,SERVO_SPECS["hip_hd"]),("L Knee Pitch",540,9.0,SERVO_SPECS["std"]),("R Knee Pitch",540,9.0,SERVO_SPECS["std"]),("Waist Pitch",2100,8.0,SERVO_SPECS["waist"]),("L Ankle Pitch",280,4.5,SERVO_SPECS["std"]),("R Ankle Pitch",280,4.5,SERVO_SPECS["std"]),("Finger Drive",18,2.0,SERVO_SPECS["digit"])]
                for label, mass_g, lever_cm, spec in loads:
                    F = (mass_g/1000.0)*9.81; need = (F*lever_cm/100.0)/0.0981; margin = spec["rated"]/need if need > 0 else 99.0
                    status = "OK" if margin >= 1.5 else ("MARGINAL" if margin >= 0.9 else "OVERLOAD")
                    Logger.log(f"  {label:<24} need {need:5.2f} kg.cm  rated {spec['rated']:5.1f}  margin {margin:.2f}x  {spec['name']:12s}  [{status}]")

            def simulate_robot_mode(self):
                self.reset_all(steps=10); self.move_joint("Blaster_Fold", 0, steps=10, axis="pitch")
                self.gesture_open("L"); self.gesture_open("R")
                Logger.log("--- ROBOT MODE --- holding neutral pose")
                try: cam = self._app.activeViewport.camera; cam.isFitView = True; self._app.activeViewport.camera = cam; self._app.activeViewport.refresh()
                except: pass
                self.capture_screenshots("optimus_robot_v11")

            def simulate_truck_mode(self):
                self.reset_all(steps=10); Logger.log("--- TRUCK MODE ---"); self.gesture_fist("L"); self.gesture_fist("R")
                self._transform_to_truck(steps_scale=1.0); self._interfere("Truck-mode check")
                Logger.log("TRUCK MODE -- holding position")
                try: cam = self._app.activeViewport.camera; cam.isFitView = True; self._app.activeViewport.camera = cam; self._app.activeViewport.refresh()
                except: pass
                self.capture_screenshots("optimus_truck_v11")

            def simulate_battle_mode(self):
                self.reset_all(steps=10); Logger.log("--- BATTLE MODE ---")
                self.move_joint("Blaster_Fold", 0, steps=10, axis="pitch"); self.gesture_fist("L"); self.gesture_point("R")
                self.move_ball([("L_Wrist",None,None,90),("R_Wrist",None,None,90)], steps=15)
                self.move_joint("R_Elbow", 130, steps=18, axis="pitch", clamp=True)
                self.move_joint("L_Elbow", 130, steps=18, axis="pitch", clamp=True)
                self.move_ball([("L_Shoulder_Cluster",0,-88,0),("R_Shoulder_Cluster",0,-88,0)], steps=22)
                self._check_zmp("Battle mode"); self._interfere("Battle-mode check")
                Logger.log("BATTLE MODE -- holding position"); self.capture_screenshots("optimus_battle_v11")

            def capture_screenshots(self, prefix="optimus"):
                if not CAPTURE_SCREENSHOTS: return
                try:
                    os.makedirs(SCREENSHOT_DIR, exist_ok=True); viewport = self._app.activeViewport; camera = viewport.camera
                    views = [("Front",adsk.core.ViewOrientations.FrontViewOrientation),("Back",adsk.core.ViewOrientations.BackViewOrientation),("Left",adsk.core.ViewOrientations.LeftViewOrientation),("Right",adsk.core.ViewOrientations.RightViewOrientation),("Top",adsk.core.ViewOrientations.TopViewOrientation),("Iso",adsk.core.ViewOrientations.IsoTopRightViewOrientation)]
                    for name, orientation in views:
                        camera.viewOrientation = orientation; camera.isFitView = True; viewport.camera = camera; time.sleep(0.5)
                        path = os.path.join(SCREENSHOT_DIR, f"{prefix}_{name}.png"); viewport.saveAsImageFile(path, 1920, 1080)
                        Logger.log(f"Screenshot: {path}")
                except: Logger.log(f"Screenshot failed: {traceback.format_exc()}", "WARN")
            def export_urdf(self):
                def _urdf_type(name):
                    if "Cluster" in name or "Wrist" in name or "CMC" in name: return "spherical"
                    if any(k in name for k in ["Knee","Elbow","MCP","Fold"]): return "revolute"
                    if any(k in name for k in ["Mount","Steer","Shields","Backpack"]): return "fixed"
                    return "revolute"
                def _limits(name):
                    limits_d = JOINT_LIMITS.get(name, {}); pitch = limits_d.get("pitch", (-180, 180))
                    return math.radians(pitch[0]), math.radians(pitch[1])
                link_mass = {"OP_Head":250,"OP_Torso":800,"OP_Pelvis":400,"OP_Backpack":150,"OP_SteerPods":120,"OP_Shields":80,"OP_Thigh_L":250,"OP_Thigh_R":250,"OP_Shin_L":220,"OP_Shin_R":220,"OP_Foot_L":180,"OP_Foot_R":180,"OP_UpperArm_L":160,"OP_UpperArm_R":160,"OP_Forearm_L":120,"OP_Forearm_R":120,"OP_Hand_L":60,"OP_Hand_R":60,"OP_Ion_Blaster":40}
                def _inertia(cname):
                    m_kg = link_mass.get(cname, 80)/1000.0; lx,ly,lz = 0.10,0.08,0.06
                    ixx = m_kg*(ly**2+lz**2)/12.0; iyy = m_kg*(lx**2+lz**2)/12.0; izz = m_kg*(lx**2+ly**2)/12.0
                    return m_kg, ixx, iyy, izz
                try:
                    os.makedirs(EXPORT_DIR, exist_ok=True); path = os.path.join(EXPORT_DIR, "robot_v11.urdf"); jc = self._root.asBuiltJoints.count
                    with open(path, "w", encoding="utf-8") as f:
                        f.write('<?xml version="1.0" encoding="utf-8"?>\n')
                        f.write('<robot name="Optimus_Prime_G1_v11">\n\n')
                        for comp in self._comps:
                            m_kg, ixx, iyy, izz = _inertia(comp.name)
                            f.write(f'  <link name="{comp.name}">\n    <inertial>\n      <origin xyz="0 0 0" rpy="0 0 0"/>\n      <mass value="{m_kg:.4f}"/>\n      <inertia ixx="{ixx:.6f}" ixy="0" ixz="0" iyy="{iyy:.6f}" iyz="0" izz="{izz:.6f}"/>\n    </inertial>\n  </link>\n')
                        f.write('\n')
                        for i in range(jc):
                            j = self._root.asBuiltJoints.item(i); n = j.name.replace(" ", "_")
                            o1 = j.occurrenceOne.component.name if j.occurrenceOne else "world"
                            o2 = j.occurrenceTwo.component.name if j.occurrenceTwo else "world"
                            jtyp = _urdf_type(n); lo_r, hi_r = _limits(n)
                            effort = 25.0
                            if "Hip" in n: effort = 35.0
                            if "Waist" in n: effort = 25.0
                            if "MCP" in n: effort = 2.2
                            f.write(f'  <joint name="{n}" type="{jtyp}">\n    <parent link="{o1}"/>\n    <child link="{o2}"/>\n    <origin xyz="0 0 0" rpy="0 0 0"/>\n    <axis xyz="1 0 0"/>\n')
                            if jtyp == "revolute": f.write(f'    <limit lower="{lo_r:.4f}" upper="{hi_r:.4f}" effort="{effort:.1f}" velocity="3.14"/>\n')
                            f.write('  </joint>\n')
                        f.write('</robot>\n')
                    Logger.log(f"URDF v11 exported -> {path}")
                except: Logger.log(f"URDF export failed: {traceback.format_exc()}", "ERROR")

            def export_stl(self):
                try:
                    os.makedirs(EXPORT_DIR, exist_ok=True)
                    skip_s = SKIP_TAGS | {"Scope","Antenna"}
                    export_mgr = self._design.exportManager; count = 0
                    for comp in self._comps:
                        for body in list(comp.bRepBodies):
                            if not body.name or any(s in body.name for s in skip_s): continue
                            try:
                                path = os.path.join(EXPORT_DIR, f"{comp.name}__{body.name}.stl")
                                stl_opts = export_mgr.createSTLExportOptions(body, path)
                                stl_opts.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
                                export_mgr.execute(stl_opts); count += 1
                            except: Logger.log(f"STL fail: {comp.name}/{body.name}", "WARN")
                    Logger.log(f"STL exported {count} bodies -> {EXPORT_DIR}")
                except: Logger.log(f"STL export failed: {traceback.format_exc()}", "ERROR")

            def export_step(self):
                try:
                    os.makedirs(EXPORT_DIR, exist_ok=True); export_mgr = self._design.exportManager
                    path = os.path.join(EXPORT_DIR, "Optimus_Prime_G1_v11.step")
                    step_opts = export_mgr.createSTEPExportOptions(path); export_mgr.execute(step_opts)
                    Logger.log(f"STEP assembly -> {path}")
                    count = 0
                    for i in range(self._root.allOccurrences.count):
                        occ = self._root.allOccurrences.item(i); cname = occ.component.name
                        if any(t in cname for t in {"Marker","Pivot","_Vis"}): continue
                        try:
                            cpath = os.path.join(EXPORT_DIR, f"{cname}.step")
                            copts = export_mgr.createSTEPExportOptions(occ, cpath); export_mgr.execute(copts); count += 1
                        except: Logger.log(f"STEP fail: {cname}", "WARN")
                    Logger.log(f"STEP {count} components -> {EXPORT_DIR}")
                except: Logger.log(f"STEP export failed: {traceback.format_exc()}", "ERROR")
            def export_bom(self):
                BOM.add("Fastener", "M3x8 SHCS (general assembly)", 80, "assembly")
                BOM.add("Fastener", "M3x16 SHCS (joint brackets)", 24, "assembly")
                BOM.add("Fastener", "M2.5x6 SHCS (PCB mounting)", 16, "assembly")
                BOM.add("Hardware", "D3 mm x 30 mm shaft pin", 12, "joints")
                BOM.add("Hardware", "D3 mm x 20 mm shaft pin", 20, "joints")
                # Phase 1: Structural metal components
                BOM.add("Frame", "2020 aluminum extrusion 120mm", 2, "torso spine, pelvis cross")
                BOM.add("Frame", "M6 T-nut drop-in", 12, "extrusion mounting")
                BOM.add("Frame", "M6x12 button-head bolt", 6, "frame brackets")
                BOM.add("Hardware", "Steel shaft D5mm x 25mm precision ground", 8, "hip, knee, waist torque")
                BOM.add("Hardware", "Steel shaft D4mm x 20mm precision ground", 10, "shoulder, elbow, ankle, waist pitch")
                BOM.add("Hardware", "Steel shaft D3mm x 15mm precision ground", 6, "wrist, neck")
                BOM.add("Hardware", "Flanged bearing MR85-2RS (5x8x2.5mm)", 8, "hip, knee, waist bearings")
                BOM.add("Hardware", "Flanged bearing MR95-2RS (5x9x3mm)", 4, "shoulder bearings")
                BOM.add("Hardware", "Flanged bearing MR74-2RS (4x7x2.5mm)", 6, "elbow, ankle bearings")
                BOM.add("Hardware", "Flanged bearing MR63-2RS (3x6x2.5mm)", 4, "wrist, neck bearings")
                BOM.add("Gearbox", "Worm gear set 30:1 module 1", 1, "waist rotation")
                BOM.add("Gearbox", "Planetary gear stage 4:1", 2, "hip pitch")
                BOM.add("Gearbox", "Planetary gear stage 3:1", 2, "knee")
                BOM.add("Fastener", "M3x4 cup-point set screw", 16, "shaft retention")
                BOM.add("Fastener", "M3x8 SHCS (bearing block mount)", 24, "bearing block to shell")
                BOM.add("Hardware", "12V locking solenoid 12x30mm", 2, "transformation latch release")
                BOM.add("Hardware", "Quick-release 6mm detent pin assembly", 4, "limb attachment")
                BOM.add("Hardware", "Detent ball 2.5mm hardened steel", 8, "quick-release")
                BOM.add("Hardware", "Spring coil 0.4mm wire x 6mm x 15mm", 4, "quick-release detent")
                BOM.add("Servo", "MG90S micro servo (spine articulation)", 3, "spine vertebrae")
                BOM.add("Hardware", "E-clip D5mm (shaft retention)", 8, "shaft retainer")
                BOM.add("Hardware", "E-clip D4mm (shaft retention)", 10, "shaft retainer")
                BOM.add("Hardware", "E-clip D3mm (shaft retention)", 6, "shaft retainer")
                BOM.add("Hardware", "Brass bushing 4mm ID x 6mm OD x 8mm", 4, "latch posts")
                BOM.add("Material", "PLA filament 1kg spool", 3, "~750g PETG or PLA")
                BOM.add("Material", "TPU filament 250g spool", 1, "flex parts")
                BOM.add("Electronics", "22AWG servo wire (3m lengths)", 30, "wiring harness")
                BOM.add("Electronics", "JST-XH 3-pin connectors", 40, "servo connectors")
                BOM.add("Electronics", "5V 5A BEC / power regulator", 2, "servo power")
                BOM.add("Electronics", "USB-C power cable", 1, "RPi power")
                BOM.save_csv(BOM_FILE)
                BOM.summary()
                Logger.log("--- SERVO WIRING MAP (PCA9685 channels) ---")
                wiring_v11 = [(0,"L_Hip_Yaw","Pelvis -> L_HipYaw"),(1,"R_Hip_Yaw","Pelvis -> R_HipYaw"),(2,"L_Hip_Pitch","Thigh_L -> HipP"),(3,"R_Hip_Pitch","Thigh_R -> HipP"),(4,"L_Hip_Roll","Thigh_L -> HipR"),(5,"R_Hip_Roll","Thigh_R -> HipR"),(6,"L_Knee","Thigh_L -> KneP"),(7,"R_Knee","Thigh_R -> KneP"),(8,"L_Ankle_Pitch","Foot_L -> AnkP"),(9,"R_Ankle_Pitch","Foot_R -> AnkP"),(10,"L_Ankle_Roll","Foot_L -> AnkR"),(11,"R_Ankle_Roll","Foot_R -> AnkR"),(12,"Waist_Yaw","Torso -> WaistYaw"),(13,"Waist_Pitch","Torso -> WaistPitch"),(14,"Neck_Pitch","Torso -> NeckPitch"),(15,"Neck_Yaw","Head -> NeckYaw")]
                wiring2_v11 = [(0,"L_Sh_Yaw","UpperArm_L -> ShY"),(1,"R_Sh_Yaw","UpperArm_R -> ShY"),(2,"L_Sh_Pitch","UpperArm_L -> ShP"),(3,"R_Sh_Pitch","UpperArm_R -> ShP"),(4,"L_Sh_Roll","UpperArm_L -> ShR"),(5,"R_Sh_Roll","UpperArm_R -> ShR"),(6,"L_Elbow","UpperArm_L -> ElbP"),(7,"R_Elbow","UpperArm_R -> ElbP"),(8,"L_Wrist_Roll","Forearm_L -> WR"),(9,"R_Wrist_Roll","Forearm_R -> WR"),(10,"L_Finger_All","Hand_L -> FingerSrvBay ch0"),(11,"L_Thumb","Hand_L -> FingerSrvBay ch1"),(12,"R_Finger_All","Hand_R -> FingerSrvBay ch0"),(13,"R_Thumb","Hand_R -> FingerSrvBay ch1"),(14,"Blaster_Fold","Hand_R -> Blaster hinge"),(15,"SPARE","-")]
                for ch, name, loc in wiring_v11: Logger.log(f"  PCA1 ch{ch:02d}  {name:<20s}  <- {loc}")
                for ch, name, loc in wiring2_v11: Logger.log(f"  PCA2 ch{ch:02d}  {name:<20s}  <- {loc}")
                BuildGuide.save(GUIDE_FILE)

            def run_all_simulations(self):
                dispatch = {"ALL": lambda: [self.test_joint_rom(), self.simulate_head_scan(), self.simulate_wave(), self.simulate_idle_breathing(), self.simulate_walking_advanced(), self.simulate_running(), self.simulate_combat(), self.simulate_transformation(), self.test_arm_workspace(), self.run_stability_analysis(), self.estimate_servo_loads(), self.export_bom()],
                    "rom": self.test_joint_rom, "head": self.simulate_head_scan, "wave": self.simulate_wave, "breathing": self.simulate_idle_breathing, "walk": self.simulate_walking_advanced, "run": self.simulate_running, "combat": self.simulate_combat, "transform": self.simulate_transformation, "truck": self.simulate_truck_mode, "battle": self.simulate_battle_mode, "robot": self.simulate_robot_mode, "stability": self.run_stability_analysis, "servo": self.estimate_servo_loads, "workspace": self.test_arm_workspace, "bom": self.export_bom, "fingers": lambda: [self.gesture_open("L"), self.gesture_open("R"), self.gesture_fist("L"), self.gesture_fist("R"), self.gesture_point("R"), self.gesture_snap("R")]}
                target = TARGET_MODULE; Logger.log(f"--- BEGINNING SIMULATION [TARGET: {target}] ---")
                fn = dispatch.get(target)
                if fn: fn()
                else: Logger.log(f"Unknown module: {target}", "ERROR")
                if EXPORT_URDF: self.export_urdf()
                if EXPORT_STL: self.export_stl()
                if EXPORT_STEP: self.export_step()
                Logger.log("=" * 55)
                Logger.log("OPTIMUS PRIME G1 v11.0 - FINAL REPORT")
                Logger.log("=" * 55)
                for label, count in self._cols:
                    if count >= 0: icon = "[OK]" if count == 0 else "[WARN]"; Logger.log(f"  {label:<42} {icon}  {count}")
                    else: Logger.log(f"  {label:<42} ?  N/A")
                if EXPORT_URDF: Logger.log(f"  URDF  -> {EXPORT_DIR}/robot_v11.urdf")
                Logger.log(f"  BOM   -> {BOM_FILE}")
                Logger.log(f"  Guide -> {GUIDE_FILE}")
                Logger.log(f"  Log   -> {LOG_FILE}")
                Logger.log("=" * 55)
        # --- ARCHIVE & LAUNCH ---
        try:
            os.makedirs(EXPORT_DIR, exist_ok=True)
            archive_path = os.path.join(EXPORT_DIR, "Optimus_Prime_G1_v11.f3d")
            export_mgr = design.exportManager
            archive_opts = export_mgr.createFusionArchiveExportOptions(archive_path)
            export_mgr.execute(archive_opts)
            Logger.log(f"Design archived -> {archive_path}")
        except Exception as e: Logger.log(f"Archive skipped: {e}", "WARN")
        sim = SimulationEngine(root, comps_list, design, app, ui)
        sim.run_all_simulations()
        Logger.log("v11 script finished successfully.")
    except Exception: Logger.log(f"FATAL ERROR:\n{traceback.format_exc()}", "ERROR")
    finally:
        Logger.flush()
        Logger.log("--- v11 END ---")
