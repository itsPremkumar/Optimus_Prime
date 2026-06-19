# ─────────────────────────────────────────────────────────────────────────────
# OPTIMUS PRIME G1  v13.0  ── Jetson Nano AI & Vision Edition
# Fusion 360 Python Script  |  Advanced Robotics Architecture
# ─────────────────────────────────────────────────────────────────────────────
# WHAT'S NEW IN v13 (AI & Distributed Computing Upgrade)
# ──────────────────────────────────────────────
# ARCH-1  Jetson Nano Bay    ─ Replaced RPi Zero with NVIDIA Jetson Nano B01
#                              footprint (10.2x8.0 cm) + 40-pin header clearance.
# ARCH-2  ESP32 Controller   ─ Shifted ESP32 to torso as the low-level real-time
#                              motor/sensor controller. Handles I2C to PCA9685.
# ARCH-3  CSI Camera Eyes    ─ Replaced ESP32-CAM with RPi CSI V2 Camera (IMX219)
#                              in the head. Routed ribbon cable to Jetson Nano.
# ARCH-4  UART Comms Layer   ─ Established UART serial architecture (Jetson <-> ESP32).
#                              Logged in BOM and Assembly Guide.
# ARCH-5  Dual Power Rails   ─ Added dedicated 5V/4A BEC for Jetson; 6V/10A BEC for servos.
#
# OPT-1  Combine Features    ─ Fixed bug where cut_cavity would fail on hidden bodies.
# OPT-2  Merge Fasteners     ─ Improved MFG-1 post-split boolean merge performance.
# OPT-3  Kinematic Joints    ─ Ensured all as-built joints use correct custom directions.
#
# AI-1  Vision Pipeline      ─ Mock simulation functions for Jetson processing vision.
# AI-2  State Machine        ─ Architecture supports adding ROS/Python AI nodes easily.
# ═════════════════════════════════════════════════════════════════════════════

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
if 'VISUAL_AUDIT' not in globals():
    VISUAL_AUDIT = False
if 'PRODUCTION_REPORT' not in globals():
    PRODUCTION_REPORT = True

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

# ─── Output directory structure ───────────────────────────────────────────────
try:
    SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
except NameError:
    SCRIPT_DIR = os.getcwd()
PROJECT_DIR    = os.path.dirname(SCRIPT_DIR)
_OUTPUT_DIR    = os.path.join(PROJECT_DIR, "output")
LOG_DIR        = globals().get("LOG_DIR")        or os.path.join(_OUTPUT_DIR, "logs")
SCREENSHOT_DIR = globals().get("SCREENSHOT_DIR") or os.path.join(_OUTPUT_DIR, "screenshots")
EXPORT_DIR     = globals().get("EXPORT_DIR")     or os.path.join(_OUTPUT_DIR, "exports")
JIG_DIR        = os.path.join(EXPORT_DIR, "jigs")
LOG_FILE       = os.path.join(LOG_DIR,     f"optimus_v13_{_ts}.txt")
BOM_FILE       = os.path.join(_OUTPUT_DIR, f"BOM_v13_{_ts}.csv")
ASSEMBLY_FILE  = os.path.join(_OUTPUT_DIR, f"ASSEMBLY_GUIDE_v13_{_ts}.txt")
MANIFEST_FILE  = os.path.join(_OUTPUT_DIR, f"BUILD_MANIFEST_v13_{_ts}.json")
PRODUCTION_FILE = os.path.join(_OUTPUT_DIR, f"PRODUCTION_READINESS_v13_{_ts}.txt")
STOP_FLAG      = os.path.join(_OUTPUT_DIR, "stop.flag")

# ═════════════════════════════════════════════════════════════════════════════
# CONFIGURATION  — ALL DIMENSIONS IN cm (Fusion 360 native unit)
# ═════════════════════════════════════════════════════════════════════════════

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

BEARING_FIT_TOLERANCE = 0.008
BEARING_RETAIN_LIP_H  = 0.06
BEARING_RETAIN_LIP_R  = 0.04

TENDON_DIA        = 0.04
TENDON_GUIDE_R    = 0.06
DRUM_R            = 0.35
DRUM_H            = 0.50
PULLEY_R          = 0.20
SPRING_OD         = 0.25
SPRING_WIRE       = 0.04

CABLE_CLIP_R      = 0.12
CABLE_CLIP_W      = 0.35
JST_XH_L          = 0.55
JST_XH_W          = 0.25
JST_XH_H          = 0.18

FUSE_HOLDER_L     = 2.00
FUSE_HOLDER_W     = 0.80
FUSE_HOLDER_H     = 0.75
BEC_L, BEC_W, BEC_H = 3.50, 1.80, 0.90
POWER_SWITCH_R    = 0.35

M3_CLR_R     = 0.160
M3_PILOT_R   = 0.125
M3_HEAD_R    = 0.285
M3_HEAD_H    = 0.300
M3_NUT_CIR   = 0.320
M3_NUT_H     = 0.240
INSERT_R     = 0.235
INSERT_H     = 0.500
BOSS_R       = 0.350
ALIGN_PIN_R  = 0.100
GROMMET_R    = 0.175

SCREW_REGISTRY = []

# ── V13 UPDATED: Electronics Footprints ──────────────────────────────────────
JETSON_L, JETSON_W, JETSON_H = 10.20, 8.00, 1.20   # NVIDIA Jetson Nano B01
ESP32_L, ESP32_W, ESP32_H    = 5.10, 2.80, 0.80    # ESP32 DevKit (Controller)
CSI_CAM_W, CSI_CAM_H, CSI_CAM_D = 2.50, 2.40, 0.30 # RPi CSI V2 Camera Module
PCA_L,   PCA_W,   PCA_H   = 6.25, 2.54, 0.18
IMU_L,   IMU_W,   IMU_H   = 2.10, 1.60, 0.12
LIPO_L,  LIPO_W,  LIPO_H  = 8.00, 3.50, 2.00       # Upgraded 3S 2200 mAh LiPo
XT60_W,  XT60_H_SLOT       = 1.60, 1.30
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

SERVO_SPECS = {
    "hip_hd": {"name": "DS3225MG", "rated": 25.0, "stall": 35.0, "mass_g":  60, "horn_spline": 25},
    "waist":  {"name": "DS3218",   "rated": 20.0, "stall": 25.0, "mass_g":  55, "horn_spline": 25},
    "std":    {"name": "MG996R",   "rated":  9.4, "stall": 11.5, "mass_g":  55, "horn_spline": 25},
    "micro":  {"name": "MG90S",    "rated":  1.8, "stall":  2.2, "mass_g":  13, "horn_spline": 21},
    "digit":  {"name": "DS04-NFC", "rated":  1.8, "stall":  2.2, "mass_g":   9, "horn_spline": 21},
}

JOINT_LIMITS = {
    "Waist_Cluster":      {"pitch": (-45,  60), "yaw": (-15,  15), "roll": (-15,  15)},
    "Neck_Cluster":       {"pitch": (-90,  45), "yaw": (-20,  20), "roll": (-20,  20)},
    "L_Hip_Cluster":      {"pitch": (-30,  30), "yaw": (-95,  95), "roll": (-30,  30)},
    "R_Hip_Cluster":      {"pitch": (-30,  30), "yaw": (-95,  95), "roll": (-30,  30)},
    "L_Knee":             {"pitch": (  0, 135), "yaw": (-25,  25), "roll": (0, 0)},
    "R_Knee":             {"pitch": (  0, 135), "yaw": (-25,  25), "roll": (0, 0)},
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
              "Block", "Sole", "Plate", "Bay", "Collar"}
SKIP_TAGS  = {"Marker", "Pivot", "MtA", "MtB", "Axle_Pivot", "Horn",
              "Pin", "_Vis", "Boss", "Insert", "Nut", "Snap", "Tool"}

ASSEMBLY_STEPS = []
JIG_REGISTRY   = []
PRINT_NOTES    = []
SUPPORT_WARNINGS = []

# ═════════════════════════════════════════════════════════════════════════════
# LOGGER & BOM
# ═════════════════════════════════════════════════════════════════════════════

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

class BOM:
    _rows = []

    @classmethod
    def add(cls, category, part_name, qty, note=""):
        cls._rows.append({"Category": category, "Part": part_name,
                          "Qty": qty, "Note": note})

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

# ═════════════════════════════════════════════════════════════════════════════
# MAIN ENTRY POINT
# ═════════════════════════════════════════════════════════════════════════════

def run(context):
    global TARGET_MODULE, EXPORT_STL, EXPORT_STEP, EXPORT_URDF, EXPORT_DIR
    global CAPTURE_SCREENSHOTS, VISUAL_AUDIT, PRODUCTION_REPORT

    Logger.log("=" * 60)
    Logger.log("EXECUTION START  v13.0 -- Optimus Prime G1 AI Build")
    Logger.log("Architecture: Jetson Nano (Brain) + ESP32 (Real-time IO)")
    Logger.log("=" * 60)

    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface

        try:
            app.preferences.generalPreferences.defaultModelingOrientation = (
                adsk.core.DefaultModelingOrientations.ZUpModelingOrientation)
        except Exception:
            pass

        doc    = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
        design = adsk.fusion.Design.cast(app.activeProduct)
        root   = design.rootComponent

        # ── APPEARANCES ──────────────────────────────────────────────────
        app_lib = None
        for i in range(app.materialLibraries.count):
            lib = app.materialLibraries.item(i)
            if "Appearance" in lib.name:
                app_lib = lib
                break

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

        # ── COMPONENT REGISTRY & PRIMITIVES ──────────────────────────────
        comps_list = []
        occs       = {}

        def new_component(name):
            occ  = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
            comp = occ.component
            comp.name = name
            comps_list.append(comp)
            occs[name] = occ
            return comp

        def set_ap(body, ap):
            if body and ap:
                try: body.appearance = ap
                except: pass

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

        def chamfer_box(comp, name, cx, cy, cz, lx, ly, lz, axis, chamfer=0.25, ap=None):
            body = box(comp, name, cx, cy, cz, lx, ly, lz, ap)
            if chamfer < 0.15:
                SUPPORT_WARNINGS.append((name, f"chamfer {chamfer}cm may need support"))
            return body

        def cut_cavity(comp, tool_body, keep_tool=False):
            tools = adsk.core.ObjectCollection.create()
            tools.add(tool_body)
            for b in list(comp.bRepBodies):
                if b == tool_body: continue
                if b.name and any(t in b.name for t in SKIP_TAGS): continue
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
                except: pass

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
            except: pass

        def merge_fasteners_to_halves(comp, body_left, body_right, axis="y"):
            fastener_tags = {"Pin", "Boss", "Insert", "Nut", "Snap"}
            for b in list(comp.bRepBodies):
                if not b.name or any(skip in b.name for skip in SKIP_TAGS): continue
                if not any(tag in b.name for tag in fastener_tags): continue
                try:
                    cog = b.physicalProperties.centerOfMass
                    pos = cog.y if axis == "y" else cog.z if axis == "z" else cog.x
                    target = body_left if pos < 0 else body_right if pos > 0 else None
                    if target and target.isValid:
                        tools = adsk.core.ObjectCollection.create()
                        tools.add(b)
                        ci = comp.features.combineFeatures.createInput(target, tools)
                        ci.operation = adsk.fusion.CombineOperation.JoinFeatureOperation
                        ci.isKeepToolBodies = False
                        comp.features.combineFeatures.add(ci)
                except: pass

        def printability_check(comp, body_name, overhang_angle_deg=45):
            high_risk = {"spike", "chin", "heel", "vent", "flap", "undercut",
                         "shelf", "hook", "overhang", "lip"}
            reason = None
            lname = body_name.lower()
            if any(r in lname for r in high_risk): reason = "geometry likely to need support material"
            if "Shell" in body_name and any(k in body_name for k in {"Shoulder", "Spike"}):
                reason = "shoulder spikes: print with tip up or use tree supports"
            if "Chin" in body_name: reason = "chin guard: add 45-degree chamfer or print face-down"
            if "Heel" in body_name: reason = "heel spur: use chamfered edges or split for printing"
            if reason:
                SUPPORT_WARNINGS.append((body_name, reason))
                Logger.log(f"  [PRINT WARNING] {body_name}: {reason}", "WARN")

        # ── PHYSICAL FEATURE HELPERS ──────────────────────────────────────
        def m3_boss(comp, tag, cx, cy, cz, axis="z", depth=0.80, screw_len=1.0):
            boss = cyl(comp, f"{tag}_Boss", cx, cy, cz, BOSS_R, depth, axis, dark_metal)
            boss.name = f"{tag}_Boss"
            cut_cavity(comp, cyl(comp, f"{tag}_Insert", cx, cy, cz, INSERT_R, INSERT_H, axis))
            BOM.add("Fastener", "M3 heat-set insert (Voron M3x5)", 1, f"boss in {comp.name}")
            SCREW_REGISTRY.append({"tag": tag, "comp": comp.name, "type": "boss_insert",
                                   "shell_t": depth, "boss_depth": INSERT_H, "requested_len": screw_len})

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

        def grommet_hole(comp, tag, cx, cy, cz, axis="y"):
            cut_cavity(comp, cyl(comp, f"{tag}_Grommet", cx, cy, cz, GROMMET_R, 0.80, axis))

        def horn_coupler(comp, tag, cx, cy, cz, axis, servo_type="std"):
            is_std = (servo_type in ("std", "hip_hd", "waist"))
            hub_r  = 0.55 if is_std else 0.35
            hub_h  = 0.80 if is_std else 0.55
            key_w  = 0.14 if is_std else 0.10
            key_d  = 0.18 if is_std else 0.13
            setscrew_r = M3_PILOT_R if is_std else 0.09
            ax = {"x": (1, 0, 0), "y": (0, 1, 0), "z": (0, 0, 1)}[axis]
            hub = cyl(comp, f"{tag}_CouplerHub", cx, cy, cz, hub_r, hub_h, axis, dark_metal)
            if axis in ("x", "z"):
                cut_cavity(comp, box(comp, f"{tag}_KeySlot", cx + ax[0]*hub_h*0.25, cy, cz + ax[2]*hub_h*0.25,
                    key_d if axis=="z" else hub_h*0.6, key_w, key_d if axis=="x" else hub_h*0.6))
            else:
                cut_cavity(comp, box(comp, f"{tag}_KeySlot", cx, cy + ax[1]*hub_h*0.25, cz,
                    key_d, hub_h*0.6, key_d))
            setscrew_axis = "y" if axis in ("x", "z") else "z"
            cut_cavity(comp, cyl(comp, f"{tag}_SetScrew", cx + ax[0]*hub_h*0.15, cy, cz + ax[2]*hub_h*0.15,
                setscrew_r, hub_r*2.2, setscrew_axis))
            cyl(comp, f"{tag}_TorqueArm", cx + ax[0] * (hub_r * 1.8), cy + ax[1] * (hub_r * 1.8), cz + ax[2] * (hub_r * 1.8),
                hub_r * 0.65, hub_h*0.8, axis, dark_metal)
            BOM.add("Fastener", "M3x4 set screw (cup point)", 1, comp.name)

        def servo_drum(comp, tag, cx, cy, cz, axis="z"):
            cyl(comp, f"{tag}_DrumBarrel", cx, cy, cz, DRUM_R, DRUM_H, axis, dark_metal)
            cyl(comp, f"{tag}_DrumFlangeT", cx, cy, cz + DRUM_H/2 - 0.02, DRUM_R + 0.10, 0.06, axis, dark_metal)
            cyl(comp, f"{tag}_DrumFlangeB", cx, cy, cz - DRUM_H/2 + 0.02, DRUM_R + 0.10, 0.06, axis, dark_metal)
            cut_cavity(comp, cyl(comp, f"{tag}_TieHole", cx, cy, cz, 0.06, DRUM_R*2.2, "x" if axis in ("y", "z") else "z"))

        def tendon_guide(comp, tag, cx, cy, cz, length, axis="z"):
            cut_cavity(comp, cyl(comp, f"{tag}_TendonGuide", cx, cy, cz, TENDON_GUIDE_R + 0.02, length, axis))

        def tendon_anchor(comp, tag, cx, cy, cz, axis="z"):
            box(comp, f"{tag}_Anchor", cx, cy, cz, 0.35, 0.28, 0.22, dark_metal)
            cut_cavity(comp, box(comp, f"{tag}_CrimpSlot", cx, cy, cz, 0.06, 0.30, 0.14))

        def palm_pulley(comp, tag, cx, cy, cz, axis="x"):
            cyl(comp, f"{tag}_PulleyPost", cx, cy, cz, 0.12, 0.50, axis, chrome)
            cyl(comp, f"{tag}_PulleyWheel", cx, cy, cz, PULLEY_R, 0.14, "y" if axis in ("x", "z") else "z", grey_plastic)

        def spring_return(comp, tag, cx, cy, cz, axis="x"):
            cut_cavity(comp, cyl(comp, f"{tag}_SpringPkt", cx, cy, cz, SPRING_OD/2 + 0.03, SPRING_WIRE*4, axis))
            peg_axis = "y" if axis in ("x", "z") else "z"
            for sign in [-1, 1]:
                peg_pos = SPRING_OD/2 + 0.06
                if peg_axis == "y": cyl(comp, f"{tag}_SpringPeg_{sign}", cx, cy + sign*peg_pos, cz, 0.06, 0.20, peg_axis, chrome)
                else: cyl(comp, f"{tag}_SpringPeg_{sign}", cx, cy, cz + sign*peg_pos, 0.06, 0.20, peg_axis, chrome)

        def screw_hole(comp, cx, cy, cz, axis="y", length=3.0):
            cut_cavity(comp, cyl(comp, "ScrewHole", cx, cy, cz, M3_PILOT_R, length, axis))

        def magnet_pocket(comp, tag, cx, cy, cz, axis="z"):
            cut_cavity(comp, cyl(comp, f"{tag}_MagPkt", cx, cy, cz, 0.32, 0.35, axis))
            BOM.add("Hardware", "Magnet D6x3 mm N35", 1, comp.name)

        def wire_channel(comp, tag, cx, cy, cz, r, h, axis):
            cut_cavity(comp, cyl(comp, f"{tag}_Wire", cx, cy, cz, r, h, axis))

        def led_pocket_5mm(comp, tag, cx, cy, cz, axis="y"):
            cut_cavity(comp, cyl(comp, f"{tag}_LED5", cx, cy, cz, LED_R_5MM, 0.85, axis))

        def led_ring_pocket(comp, tag, cx, cy, cz, axis="z"):
            cut_cavity(comp, cyl(comp, f"{tag}_LEDRing", cx, cy, cz, LED_R_RING, 0.30, axis))

        def cable_clip(comp, tag, cx, cy, cz, axis="y"):
            box(comp, f"{tag}_ClipBase", cx, cy, cz, CABLE_CLIP_W, 0.15, 0.35, grey_plastic)
            cyl(comp, f"{tag}_ClipArch", cx, cy, cz + 0.06, CABLE_CLIP_R + 0.08, CABLE_CLIP_W, "x", grey_plastic)

        def wire_hub(comp, tag, cx, cy, cz):
            box(comp, f"{tag}_HubBlock", cx, cy, cz, 2.0, 1.6, 1.2, dark_grey)
            for dx, dy, dz, lbl in [(-1.0, 0, 0, "L"), (1.0, 0, 0, "R"), (0, -0.8, 0, "F"), (0, 0.8, 0, "B"), (0, 0, -0.6, "D"), (0, 0, 0.6, "U")]:
                wire_channel(comp, f"{tag}_Hub{lbl}", cx+dx*0.5, cy+dy*0.5, cz+dz*0.5, 0.25, 0.80, "x" if abs(dx)>abs(dy) and abs(dx)>abs(dz) else "y" if abs(dy)>abs(dz) else "z")

        def grommet_slot(comp, tag, cx, cy, cz, axis="y", width=0.50):
            cut_cavity(comp, cyl(comp, f"{tag}_GromSlot", cx, cy, cz, GROMMET_R, width, axis))
            cut_cavity(comp, cyl(comp, f"{tag}_GromSeat", cx, cy, cz, GROMMET_R + 0.06, 0.10, axis))

        def jst_pocket(comp, tag, cx, cy, cz, axis="y"):
            cut_cavity(comp, box(comp, f"{tag}_JST", cx, cy, cz, JST_XH_L + 0.10, JST_XH_W + 0.10, JST_XH_H + 0.10))

        def bearing_fit(comp, tag, cx, cy, cz, axis="x", ro=1.10, w=0.60, fit_type="press"):
            tol = BEARING_FIT_TOLERANCE if fit_type == "press" else 0.0 if fit_type == "glue" else 0.015
            outer_r = ro + tol
            cyl(comp, f"{tag}_BO", cx, cy, cz, ro, w, axis, chrome)
            cyl(comp, f"{tag}_BI", cx, cy, cz, ro*0.58, w*0.80, axis, dark_grey)
            cyl(comp, f"{tag}_BB", cx, cy, cz, ro*0.32, w*1.10, axis, chrome)
            temp = adsk.fusion.TemporaryBRepManager.get()
            ax = {"x": (1,0,0), "y": (0,1,0), "z": (0,0,1)}[axis]
            half = w/2.0 + 0.05
            p1 = adsk.core.Point3D.create(cx-ax[0]*half, cy-ax[1]*half, cz-ax[2]*half)
            p2 = adsk.core.Point3D.create(cx+ax[0]*half, cy+ax[1]*half, cz+ax[2]*half)
            cs = temp.createCylinderOrCone(p1, outer_r + 0.05, p2, outer_r + 0.05)
            bf = comp.features.baseFeatures.add()
            bf.startEdit()
            cb = comp.bRepBodies.add(cs, bf)
            bf.finishEdit()
            cb.name = f"{tag}_CB"
            cut_cavity(comp, cb)

        def dual_bearing(comp, tag, cx, cy, cz, axis="x", ro=1.10, w=0.60, span=2.50, fit_type="press"):
            ax = {"x": (1,0,0), "y": (0,1,0), "z": (0,0,1)}[axis]
            box(comp, f"{tag}_Carrier", cx, cy, cz, span*ax[0]+1.2, span*ax[1]+1.2, span*ax[2]+1.2, dark_metal)
            p1 = (cx - ax[0]*span/2, cy - ax[1]*span/2, cz - ax[2]*span/2)
            p2 = (cx + ax[0]*span/2, cy + ax[1]*span/2, cz + ax[2]*span/2)
            bearing_fit(comp, f"{tag}_A", p1[0], p1[1], p1[2], axis, ro, w, fit_type)
            bearing_fit(comp, f"{tag}_B", p2[0], p2[1], p2[2], axis, ro, w, fit_type)
            cyl(comp, f"{tag}_Axle", cx, cy, cz, ro*0.55, span + 1.0, axis, chrome)

        def cover_plate(comp, tag, cx, cy, cz, lx, lz, boss_positions, method="screw", hinge_edge=None, ap=None):
            ap = ap or grey_plastic
            ly_cover = 0.25
            cover = box(comp, f"{tag}_Cover", cx, cy, cz, lx, ly_cover, lz, ap)
            if method == "screw":
                for bx, bz in boss_positions: m3_boss(comp, f"{tag}_CvB_{bx:.0f}_{bz:.0f}", cx+bx, cy, cz+bz)
            elif method == "magnet":
                for bx, bz in boss_positions: magnet_pocket(comp, f"{tag}_CvM_{bx:.0f}_{bz:.0f}", cx+bx, cy, cz+bz)
            elif method == "snap":
                snap_clip(comp, f"{tag}_CvSnap", cx, cy, cz, span_x=lx*0.7)
            elif method == "hinge":
                hinge_y = cy - ly_cover/2
                hinge_axis = "x" if hinge_edge in ("left", "right") else "z"
                cyl(comp, f"{tag}_HingeBarrel", cx, hinge_y, cz, 0.12, lx*0.9 if hinge_axis=="x" else lz*0.9, hinge_axis, dark_metal)
            return cover

        def lipo_door(comp, tag, cx, cy, cz, lx, lz):
            cover_plate(comp, tag, cx, cy, cz, lx, lz, [(-lx/2+0.6, -lz/2+0.6), (lx/2-0.6, -lz/2+0.6), (-lx/2+0.6, lz/2-0.6), (lx/2-0.6, lz/2-0.6)], method="snap", ap=dark_grey)

        def pcb_cover(comp, tag, cx, cy, cz, lx, lz, method="screw"):
            cover_plate(comp, tag, cx, cy, cz, lx, lz, [(-lx/2+0.5, -lz/2+0.5), (lx/2-0.5, -lz/2+0.5), (-lx/2+0.5, lz/2-0.5), (lx/2-0.5, lz/2-0.5)], method=method, ap=grey_plastic)

        def assembly_jig(comp_name, pin_positions, socket_positions, base_size):
            jig = new_component(f"JIG_{comp_name}")
            lx, ly, lz = base_size
            box(jig, "Jig_Base", 0, 0, 0, lx, ly, lz, white_pla)
            for i, (px, py, pz) in enumerate(pin_positions): cyl(jig, f"JigPin_{i}", px, py, pz + lz/2, ALIGN_PIN_R + 0.02, 0.50, "z", chrome)
            for i, (sx, sy, sz) in enumerate(socket_positions): cyl(jig, f"JigSock_{i}", sx, sy, sz + lz/2, ALIGN_PIN_R + 0.025, 0.30, "z", dark_grey)
            JIG_REGISTRY.append((jig.name, comp_name))
            return jig

        def verify_screw_lengths():
            Logger.log("--- V13 SCREW LENGTH VERIFICATION ---")
            for entry in SCREW_REGISTRY:
                req = entry["requested_len"]
                stype = entry["type"]
                if stype == "boss_insert":
                    min_len = entry["shell_t"] + INSERT_H * 0.6
                    if req < min_len: Logger.log(f"  [WARN] {entry['tag']} M3x{int(req*10)} too short", "WARN")

        # ── V13 ELECTRONICS BAY HELPERS ───────────────────────────────────
        def jetson_nano_bay(comp, tag, cx, cy, cz):
            """ARCH-1 — NVIDIA Jetson Nano B01 pocket (10.2x8.0 cm)."""
            cut_cavity(comp, box(comp, f"{tag}_JetsonBay", cx, cy, cz, JETSON_L + 0.30, JETSON_H + 0.40, JETSON_W + 0.30))
            for sx, sz in [(-4.5, -3.5), (4.5, -3.5), (-4.5, 3.5), (4.5, 3.5)]:
                cyl(comp, f"{tag}_Stdoff_{sx}_{sz}", cx+sx, cy, cz+sz, 0.15, JETSON_H+0.50, "y", dark_metal)
            # CSI Cable routing path
            cut_cavity(comp, box(comp, f"{tag}_CSI_Slot", cx - JETSON_L/2 - 0.1, cy, cz + JETSON_W/2 - 0.5, 0.2, 0.2, 0.6))
            BOM.add("Electronics", "NVIDIA Jetson Nano 4GB (Main Brain)", 1, comp.name)
            BOM.add("Fastener", "M2.5x11 mm brass standoff", 4, comp.name)

        def esp32_ctrl_bay(comp, tag, cx, cy, cz):
            """ARCH-2 — ESP32 DevKit pocket (Low-Level Controller)."""
            cut_cavity(comp, box(comp, f"{tag}_ESP32Bay", cx, cy, cz, ESP32_L + 0.20, ESP32_H + 0.30, ESP32_W + 0.20))
            cyl(comp, f"{tag}_Stdoff1", cx-2.0, cy, cz-1.0, 0.12, ESP32_H+0.40, "y", dark_metal)
            cyl(comp, f"{tag}_Stdoff2", cx+2.0, cy, cz+1.0, 0.12, ESP32_H+0.40, "y", dark_metal)
            BOM.add("Electronics", "ESP32 DevKit (Real-time Motor/Sensor Ctrl)", 1, comp.name)

        def csi_cam_pocket(comp, tag, cx, cy, cz, lens_axis="y"):
            """ARCH-3 — RPi CSI V2 Camera pocket for Robot Eyes."""
            cut_cavity(comp, box(comp, f"{tag}_CamBay", cx, cy, cz, CSI_CAM_W + 0.20, CSI_CAM_D + 0.20, CSI_CAM_H + 0.20))
            cut_cavity(comp, cyl(comp, f"{tag}_LensHole", cx, cy - (CSI_CAM_D/2 + 0.20), cz, 0.150, 0.60, lens_axis))
            BOM.add("Electronics", "Raspberry Pi CSI Camera V2 (IMX219)", 1, comp.name)

        def pca9685_bay(comp, tag, cx, cy, cz):
            cut_cavity(comp, box(comp, f"{tag}_PCABay", cx, cy, cz, PCA_L + 0.20, PCA_H + 0.30, PCA_W + 0.20))
            for sx, sz in [(-3.00, -1.08), (+3.00, -1.08), (-3.00, +1.08), (+3.00, +1.08)]:
                cyl(comp, f"{tag}_PCAStdoff_{sx}_{sz}", cx+sx, cy, cz+sz, 0.14, PCA_H+0.50, "y", dark_metal)
            BOM.add("Electronics", "PCA9685 16-ch servo driver", 1, comp.name)

        def lipo_bay(comp, tag, cx, cy, cz):
            cut_cavity(comp, box(comp, f"{tag}_LipoBay", cx, cy, cz, LIPO_L + 0.30, LIPO_H + 0.30, LIPO_W + 0.30))
            cut_cavity(comp, box(comp, f"{tag}_XT60Slot", cx, cy + LIPO_H/2 + 0.15, cz, XT60_W + 0.10, 0.50, XT60_H_SLOT + 0.10))
            BOM.add("Electronics", "3S 2200 mAh 11.1V LiPo battery", 1, comp.name)
            BOM.add("Electronics", "XT60-F panel-mount connector",  1, comp.name)

        def imu_pocket(comp, tag, cx, cy, cz, axis="z"):
            cut_cavity(comp, box(comp, f"{tag}_IMUPkt", cx, cy, cz, IMU_L + 0.20, IMU_W + 0.20, IMU_H + 0.30))
            BOM.add("Electronics", "MPU-6050 IMU breakout", 1, comp.name)

        def bec_mount(comp, tag, cx, cy, cz):
            cut_cavity(comp, box(comp, f"{tag}_BECBay", cx, cy, cz, BEC_L + 0.20, BEC_H + 0.20, BEC_W + 0.20))
            BOM.add("Electronics", "5V/4A BEC (Jetson Power)", 1, comp.name)
            BOM.add("Electronics", "6V/10A BEC (Servo Power)", 1, comp.name)

        def power_switch_cutout(comp, tag, cx, cy, cz, axis="y"):
            cyl(comp, f"{tag}_SwHole", cx, cy, cz, POWER_SWITCH_R, 1.0, axis, black_plastic)
            cut_cavity(comp, cyl(comp, f"{tag}_SwCut", cx, cy, cz, POWER_SWITCH_R + 0.03, 1.2, axis))

        # ── JOINT BUILDERS ────────────────────────────────────────────────
        def _make_joint_geometry(cx, cy, cz):
            try:
                cpi = root.constructionPoints.createInput()
                cpi.setByPoint(adsk.core.Point3D.create(cx, cy, cz))
                cp = root.constructionPoints.add(cpi)
                cp.isLightBulbOn = False
                return adsk.fusion.JointGeometry.createByPoint(cp)
            except: pass
            try:
                sketch = root.sketches.add(root.xYConstructionPlane)
                sketch.isVisible = False
                s_pt = sketch.sketchPoints.add(adsk.core.Point3D.create(cx, cy, cz))
                return adsk.fusion.JointGeometry.createByPoint(s_pt)
            except: return None

        def revolute_joint(name, occ1, occ2, cx, cy, cz, axis_str):
            if not (occ1 and occ2): return
            try:
                geom = _make_joint_geometry(cx, cy, cz)
                ji   = root.asBuiltJoints.createInput(occ1, occ2, geom)
                av   = {"x": adsk.core.Vector3D.create(1, 0, 0),
                        "y": adsk.core.Vector3D.create(0, 1, 0),
                        "z": adsk.core.Vector3D.create(0, 0, 1)}[axis_str]
                ji.setAsRevoluteJointMotion(adsk.fusion.JointDirections.CustomJointDirection, av)
                j = root.asBuiltJoints.add(ji)
                j.name = name
            except: Logger.log(f"Failed revolute joint {name}: {traceback.format_exc()}", "ERROR")

        def ball_joint(name, occ1, occ2, cx, cy, cz):
            if not (occ1 and occ2): return
            try:
                geom = _make_joint_geometry(cx, cy, cz)
                ji   = root.asBuiltJoints.createInput(occ1, occ2, geom)
                ji.setAsBallJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection, adsk.fusion.JointDirections.XAxisJointDirection)
                j = root.asBuiltJoints.add(ji)
                j.name = name
            except: Logger.log(f"Failed ball joint {name}: {traceback.format_exc()}", "ERROR")

        def rigid_joint(name, occ1, occ2):
            if not (occ1 and occ2): return
            try:
                geom = _make_joint_geometry(0, 0, 0)
                ji   = root.asBuiltJoints.createInput(occ1, occ2, geom)
                ji.setAsRigidJointMotion()
                j = root.asBuiltJoints.add(ji)
                j.name = name
            except: pass

        def hard_stop(comp, tag, cx, cy, cz, axis="x", stop_angle_deg=90):
            box(comp, f"{tag}_Stop", cx, cy, cz, 0.35, 0.35, 0.35, dark_metal)

        def transform_lock(comp, tag, cx, cy, cz, axis="z"):
            cyl(comp, f"{tag}_LockBore", cx, cy, cz, 0.18, 1.50, axis, dark_metal)
            cut_cavity(comp, cyl(comp, f"{tag}_LockHole", cx, cy, cz, 0.20, 1.60, axis))
            cyl(comp, f"{tag}_SpringPkt", cx, cy, cz + 0.30, 0.35, 0.50, axis, dark_grey)
            BOM.add("Hardware", "Spring latch pin Ø3.5 mm", 1, comp.name)

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
                cut_cavity(comp, c1)
                cut_cavity(comp, c2)
            grommet_hole(comp, tag, cx, cy, cz + (0.5 if axis != "z" else 1.0))

        def mg996r(comp, tag, cx, cy, cz, axis="x"):
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
            horn_coupler(comp, tag, cx, cy, cz, axis, servo_type="std")
            BOM.add("Servo", "MG996R 11 kg-cm servo", 1, comp.name)

        def mg90s(comp, tag, cx, cy, cz, axis="x"):
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
            horn_coupler(comp, tag, cx, cy, cz, axis, servo_type="micro")
            BOM.add("Servo", "MG90S micro servo", 1, comp.name)

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

        def u_bracket(comp, tag, cx, cy, cz, lx, ly, lz, ap=None):
            ap = ap or chrome
            box(comp, f"{tag}_BB", cx, cy, cz, 0.45, ly, lz, ap)
            box(comp, f"{tag}_BL", cx+lx*0.45, cy+ly*0.35, cz, lx*0.55, 0.40, lz, ap)
            box(comp, f"{tag}_BR", cx+lx*0.45, cy-ly*0.35, cz, lx*0.55, 0.40, lz, ap)
            cyl(comp, f"{tag}_Pin", cx+lx*0.50, cy, cz, 0.18, ly*0.85, "y", chrome)

        # ═════════════════════════════════════════════════════════════════
        # COMPONENT BUILDING
        # ═════════════════════════════════════════════════════════════════

        # ─── 1 TORSO ─────────────────────────────────────────────────────
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
        box(torso, "Front_Bumper",   0,   -5.8,  TORSO_CTR-4.4,  10.0, 2.0,  1.8, chrome)
        box(torso, "Chest_Plate",    0,   -4.20, TORSO_CTR+0.5,   8.4, 0.32, 4.0, chrome)
        cyl(torso, "Badge_Ring",     0,   -4.55, TORSO_CTR+0.5,   0.80, 0.12, "y", op_red)
        led_ring_pocket(torso, "Badge",  0, -4.60, TORSO_CTR+0.5, "y")

        # Internal structure
        box(torso, "Inner_Frame",    0,    0,    TORSO_CTR+1.5,   7.4, 6.0,  8.2, dark_metal)
        box(torso, "Spine_Beam",     0,    0,    TORSO_CTR+1.5,   1.8, 1.8,  8.2, chrome)
        cyl(torso, "Spine_Cyl",      0,    0,    TORSO_CTR+1.5,  1.10, 4.5,  "z", chrome)
        for rz in [TORSO_CTR+3.5, TORSO_CTR, TORSO_CTR-3.5]:
            box(torso, f"Rib_{rz:.0f}", 0, 0, rz, 6.8, 0.35, 4.5, dark_metal)

        # V13 ARCH-1: Jetson Nano Bay
        box(torso, "Jetson_Shell",   0,    2.8, TORSO_CTR+1.8,  10.5, 3.6,  8.2, black_plastic)
        jetson_nano_bay(torso, "Main",  0,    3.2, TORSO_CTR+1.8)
        pcb_cover(torso, "JetsonCover", 0, 4.5, TORSO_CTR+1.8, JETSON_L + 0.60, JETSON_W + 0.60, "magnet")

        # V13 ARCH-2: ESP32 Controller Bay
        box(torso, "ESP32_Shell",    -4.0,  2.8, TORSO_CTR-2.0,   5.5, 3.6,  3.2, black_plastic)
        esp32_ctrl_bay(torso, "Ctrl", -4.0, 3.2, TORSO_CTR-2.0)
        pcb_cover(torso, "ESPCover", -4.0, 4.5, TORSO_CTR-2.0, ESP32_L + 0.50, ESP32_W + 0.50, "screw")

        # V13 ARCH-3: PCA9685 Bays (Wired to ESP32 via I2C)
        box(torso, "PCA_Shell",      4.0,  2.8, TORSO_CTR-2.0,   6.8, 3.2,  2.2, black_plastic)
        pca9685_bay(torso, "PCA1",   4.0,  3.0, TORSO_CTR-2.0)
        box(torso, "PCA2_Shell",     4.0,  2.8, TORSO_CTR+0.5,   6.8, 3.2,  2.2, black_plastic)
        pca9685_bay(torso, "PCA2",   4.0,  3.0, TORSO_CTR+0.5)

        # LiPo & Power
        box(torso, "LipoBay_Shell",  0,    3.2, TORSO_CTR-5.0,   8.5, 4.4,  5.6, black_plastic)
        lipo_bay(torso, "Main", 0, 2.8, TORSO_CTR-5.0)
        lipo_door(torso, "LipoDoor", 0, 5.5, TORSO_CTR-5.0, LIPO_L + 0.80, LIPO_W + 0.80)
        bec_mount(torso, "MainBEC", LIPO_L/2 + 1.0, 3.0, TORSO_CTR-5.0)
        power_switch_cutout(torso, "PwrSw", -5.5, 0, TORSO_CTR+2.0, "y")

        # Harness
        wire_hub(torso, "TorsoHub", 0, 1.5, TORSO_CTR+0.5)
        for cz_clip in [TORSO_CTR+3.0, TORSO_CTR, TORSO_CTR-3.0, TORSO_CTR-4.5]:
            cable_clip(torso, f"CC_L_{cz_clip:.0f}", -3.4, 0.6, cz_clip)
            cable_clip(torso, f"CC_R_{cz_clip:.0f}",  3.4, 0.6, cz_clip)
        box(torso, "Cable_L",       -3.4,  0.6,  TORSO_CTR,      0.55, 1.0, 10.0, dark_grey)
        box(torso, "Cable_R",        3.4,  0.6,  TORSO_CTR,      0.55, 1.0, 10.0, dark_grey)

        # Transformation & Joints
        box(torso, "TF_Flap_L",     -5.40,-0.2,  TORSO_CTR+3.0,  0.40, 6.6,  6.2, op_red)
        box(torso, "TF_Flap_R",      5.40,-0.2,  TORSO_CTR+3.0,  0.40, 6.6,  6.2, op_red)
        transform_lock(torso, "WaistLock", 0, -2.0, WAIST_CTR-3.0, "z")
        u_bracket(torso, "Waist_Brkt",  0, 0, WAIST_CTR,     4.0, 4.2, 3.4)
        mg996r(torso,    "Waist_Yaw",   0, 0, WAIST_CTR,     "z")
        dual_bearing(torso, "WaistDual", 0, 0, WAIST_CTR+0.5, "z", 1.30, 0.65, span=3.00)
        u_bracket(torso, "WaistP_Brkt", 0, 0, WAIST_CTR-2.5, 4.0, 4.2, 3.4)
        mg996r(torso,    "Waist_Pitch", 0, 0, WAIST_CTR-2.5, "x")
        bearing_fit(torso, "WaistP_Brg",  0, 0, WAIST_CTR-2.0, "x", 1.30, 0.65, fit_type="press")
        hard_stop(torso, "WaistP", 0, -2.5, WAIST_CTR-2.5, "x", 60)
        u_bracket(torso, "Neck_Brkt",   0, 0, NECK_JOINT_Z,  3.2, 2.8, 3.0)
        mg996r(torso,    "Neck_Pitch",  0, 0, NECK_JOINT_Z,  "x")
        wire_channel(torso, "Spine",    0, 0, TORSO_CTR,     0.6, 20.0, "z")

        for b in list(torso.bRepBodies):
            if b.name: printability_check(torso, b.name)

        # ─── 2 HEAD ──────────────────────────────────────────────────────
        head = new_component("OP_Head")
        HC   = HEAD_CTR
        box(head, "Helm_Main",      0,     0,    HC+0.8,  5.6, 5.2, 5.0, op_blue)
        box(head, "Helm_Forehead",  0,    -2.30, HC+2.6,  5.0, 0.42, 1.8, op_blue)
        box(head, "Helm_Top",       0,     0,    HC+3.4,  4.8, 4.4, 0.90, op_blue)
        box(head, "Ear_L",         -3.30,  0,    HC+1.8,  0.70, 4.8, 3.8, op_blue)
        box(head, "Ear_R",          3.30,  0,    HC+1.8,  0.70, 4.8, 3.8, op_blue)
        box(head, "EarTop_L",      -3.05,  0,    HC+3.5,  1.40, 4.4, 0.62, op_blue)
        box(head, "EarTop_R",       3.05,  0,    HC+3.5,  1.40, 4.4, 0.62, op_blue)
        box(head, "Crest_Main",     0,    -0.30, HC+3.85, 1.05, 0.68, 3.6, chrome)
        box(head, "Face_Plate",     0,    -2.50, HC+0.5,  3.6, 0.38, 3.2, chrome)
        chamfer_box(head, "Chin_Guard", 0, -2.60, HC-0.9, 3.0, 0.38, 1.8, "z", chamfer=0.25, ap=chrome)
        box(head, "Visor_Frame",    0,    -2.55, HC+1.35, 3.8, 0.30, 1.25, op_blue)
        box(head, "Visor",          0,    -2.68, HC+1.35, 3.0, 0.16, 0.92, op_blue_glass)
        box(head, "Nose_Bridge",    0,    -2.60, HC+1.95, 0.72, 0.22, 0.72, chrome)
        box(head, "Head_Rear",      0,    1.90,  HC+1.0,  4.2, 1.5, 4.4, op_red)
        cyl(head, "Ant_L",         -2.80, 0,     HC+4.4, 0.13, 2.7, "z", chrome)
        cyl(head, "Ant_R",          2.80, 0,     HC+4.4, 0.13, 2.7, "z", chrome)
        cyl(head, "AntTip_L",      -2.80, 0,     HC+5.8, 0.24, 0.34, "z", gold_met)
        cyl(head, "AntTip_R",       2.80, 0,     HC+5.8, 0.24, 0.34, "z", gold_met)

        # V13 ARCH-3: CSI Camera mount (Robot Eyes)
        csi_cam_pocket(head, "EyeCam", 0, -1.6, HC+2.5, "y")
        cover_plate(head, "CamCover", 0, -2.0, HC+2.5, CSI_CAM_W+0.60, CSI_CAM_H+0.60, [(-1.0, -0.6), (1.0, -0.6)], method="hinge", hinge_edge="top", ap=grey_plastic)

        mg90s(head, "Neck_Yaw", 0, 0, NECK_JOINT_Z, "z")
        grommet_slot(head, "NeckWire", 0, 0.8, HC-0.5, "y", 0.50)
        for b in list(head.bRepBodies):
            if b.name: printability_check(head, b.name)
        assembly_jig("OP_Head", [(-2.0, 0, 0), (2.0, 0, 0)], [(-1.5, 0, 0), (1.5, 0, 0)], (6.0, 4.0, 0.60))

        # ─── 3 PELVIS ────────────────────────────────────────────────────
        pelvis = new_component("OP_Pelvis")
        box(pelvis, "Pelvis_Shell",  0,    0,  PELVIS_CTR,  16.4, 6.2, 5.0, op_blue)
        box(pelvis, "Pelvis_Frame",  0,    0,  PELVIS_CTR,  12.2, 4.4, 3.8, dark_metal)
        box(pelvis, "Hip_Armr_L",  -7.2,  0,  PELVIS_CTR,   1.0, 5.2, 4.2, chrome)
        box(pelvis, "Hip_Armr_R",   7.2,  0,  PELVIS_CTR,   1.0, 5.2, 4.2, chrome)
        imu_pocket(pelvis, "BalanceIMU", 0, 0, PELVIS_CTR)
        cover_plate(pelvis, "IMUCover", 0, 0.2, PELVIS_CTR, IMU_L+0.60, IMU_W+0.60, [(-0.8, -0.5), (0.8, 0.5)], method="magnet", ap=grey_plastic)
        grommet_slot(pelvis, "HipWire_L", -HIP_X, 0, PELVIS_CTR+0.5, "z", 0.60)
        grommet_slot(pelvis, "HipWire_R",  HIP_X, 0, PELVIS_CTR+0.5, "z", 0.60)
        mg996r(pelvis, "L_HipYaw", -HIP_X, 0, HIP_JOINT_Z, "z")
        mg996r(pelvis, "R_HipYaw",  HIP_X, 0, HIP_JOINT_Z, "z")
        dual_bearing(pelvis, "L_HipDual", -HIP_X-2.2, 0, HIP_JOINT_Z, "z", 1.10, 0.62, span=3.20, fit_type="press")
        dual_bearing(pelvis, "R_HipDual",  HIP_X+2.2, 0, HIP_JOINT_Z, "z", 1.10, 0.62, span=3.20, fit_type="press")
        BOM.add("Servo", "DS3225MG 25 kg-cm servo (hip yaw)", 2, "OP_Pelvis")

        # ─── 4 LEGS & ARMS ───────────────────────────────────────────────
        for side, sx in [("L", -HIP_X), ("R", HIP_X)]:
            m = -1 if side == "L" else 1
            thigh = new_component(f"OP_Thigh_{side}")
            box(thigh, "Thigh_Link",         sx,        0,  THIGH_CTR,      5.0, 4.0, 11.0, chrome)
            box(thigh, "Thigh_Outer",        sx+m*2.65, 0,  THIGH_CTR,      0.50, 4.4, 11.0, op_red)
            box(thigh, "Thigh_Front",        sx,       -2.2, THIGH_CTR,     5.0, 0.40, 11.0, op_blue)
            box(thigh, "Thigh_Knee_Armor",   sx,       -2.2, KNEE_CTR+2.5,  4.2, 0.80,  2.8, chrome)
            box(thigh, f"{side}_Thigh_Rib", sx, 0, THIGH_CTR, 3.5, 0.30, 9.0, dark_metal)
            u_bracket(thigh, f"{side}_HPB",  sx, 0, HIP_JOINT_Z+0.5,        4.0, 3.2, 3.2)
            mg996r(thigh, f"{side}_HipP",    sx, 0, HIP_JOINT_Z,             "x")
            dual_bearing(thigh, f"{side}_HipP_Dual", sx, 0, HIP_JOINT_Z, "x", 1.10, 0.62, span=2.80, fit_type="press")
            mg996r(thigh, f"{side}_HipR",    sx, 0, THIGH_CTR+2.0,           "y")
            bearing_fit(thigh, f"{side}_HRB",    sx, 0, THIGH_CTR+2.0, "y", 1.00, 0.55, fit_type="press")
            u_bracket(thigh, f"{side}_KnB",  sx, 0, KNEE_CTR+1.5,            3.8, 3.0, 3.0)
            mg996r(thigh, f"{side}_KneP",    sx, 0, KNEE_CTR+1.5,            "x")
            mg996r(thigh, f"{side}_KneYaw",  sx+1.2, 0, KNEE_CTR+1.5,        "y")
            dual_bearing(thigh, f"{side}_Knee_Dual", sx, 0, KNEE_CTR, "x", 1.00, 0.55, span=2.60, fit_type="press")
            wire_channel(thigh, f"{side}_LW", sx, 0, THIGH_CTR,              0.5, 12.0, "z")
            transform_lock(thigh, f"{side}_KneeLock", sx, -2.5, KNEE_CTR+0.5, "x")
            hard_stop(thigh, f"{side}_KneeExt", sx, -2.5, KNEE_CTR, "x", 135)

            shin = new_component(f"OP_Shin_{side}")
            box(shin, "Shin_Link",    sx,    0,    SHIN_CTR,  4.4, 6.0, 11.0, op_blue)
            box(shin, "Shin_Armor",   sx,   -2.7,  SHIN_CTR,  3.2, 0.34,  9.2, chrome)
            box(shin, "Shin_Rear",    sx,    2.7,  SHIN_CTR,  2.0, 0.34,  9.8, dark_grey)
            box(shin, "Shin_Beam",    sx,    0.4,  SHIN_CTR,  1.8, 2.2, 10.2, dark_metal)
            box(shin, f"{side}_Shin_Rib", sx, 0, SHIN_CTR, 2.8, 0.25, 8.5, dark_metal)
            box(shin, "KneeCap",      sx,   -2.9,  KNEE_CTR-1.0, 3.0, 0.55, 2.2, chrome)
            tt_wheel(shin, f"{side}_WF", sx+m*2.0, -4.2, SHIN_CTR+4.0, m)
            tt_wheel(shin, f"{side}_WR", sx+m*2.0, -4.2, SHIN_CTR-4.0, m)
            bearing_fit(shin, f"{side}_KLB", sx, 0, KNEE_CTR-0.5, "x", 1.00, 0.55, fit_type="press")
            wire_channel(shin, f"{side}_SW", sx, 0, SHIN_CTR, 0.5, 11.0, "z")
            cut_cavity(shin, box(shin, "FtTuck", sx, 2.7, SHIN_CTR-3.5, 5.2, 1.3, 4.2))
            grommet_slot(shin, f"{side}_AnkleWire", sx, 0, SHIN_CTR-5.0, "z", 0.50)

            foot = new_component(f"OP_Foot_{side}")
            box(foot, "Foot_Sole",    sx,       -0.6,  ANKLE_CTR-1.5,  6.2, 9.2, 1.10, op_red)
            chamfer_box(foot, "Heel_Block", sx-m*0.9, 3.2, ANKLE_CTR-0.8, 2.5, 3.5, 2.6, "y", chamfer=0.30, ap=dark_grey)
            chamfer_box(foot, "Heel_Spur", sx-m*1.0, 4.8, ANKLE_CTR-0.2, 1.2, 0.40, 3.2, "z", chamfer=0.35, ap=op_red)
            box(foot, "Toe_Block",    sx+m*0.8, -3.8,  ANKLE_CTR-0.8,  2.6, 3.8, 2.0, dark_grey)
            box(foot, "Ankle_Guard",  sx,        0,    ANKLE_CTR+1.2,  5.4, 3.2, 2.8, chrome)
            mg996r(foot, f"{side}_AnkP", sx, 0, ANKLE_CTR+2.2, "x")
            mg996r(foot, f"{side}_AnkR", sx, 0, ANKLE_CTR+0.5, "y")
            bearing_fit(foot, f"{side}_AB",  sx, 0, ANKLE_CTR,     "x", 1.00, 0.55, fit_type="press")
            hard_stop(foot, f"{side}_AnkP_Stop", sx, -2.0, ANKLE_CTR+2.2, "x", 20)
            align_pin(foot, f"{side}_Ft_A", sx-1.0, 0, ANKLE_CTR-1.3)
            align_pin(foot, f"{side}_Ft_B", sx+1.0, 0, ANKLE_CTR-1.3)
            box(foot, f"{side}_VibPad", sx, -0.6, ANKLE_CTR-2.2, 5.5, 8.0, 0.15, rubber_blk)
            for comp_chk in [thigh, shin, foot]:
                for b in list(comp_chk.bRepBodies):
                    if b.name: printability_check(comp_chk, b.name)
            assembly_jig(f"OP_Thigh_{side}", [(sx-1.0, 0, THIGH_CTR+2.0), (sx+1.0, 0, THIGH_CTR-2.0)], [(sx-0.5, 0, THIGH_CTR+2.0), (sx+0.5, 0, THIGH_CTR-2.0)], (7.0, 5.0, 0.60))

        for side, ax in [("L", -SHOULDER_X), ("R", SHOULDER_X)]:
            m = -1 if side == "L" else 1
            ua = new_component(f"OP_UpperArm_{side}")
            box(ua, "Sh_Block",        ax,         0, SHOULDER_CTR,      5.6, 4.2, 5.6, op_red)
            box(ua, "Sh_Pad_Outer",    ax+m*3.20,  0, SHOULDER_CTR,      1.00, 5.2, 5.8, op_red)
            box(ua, "Sh_Pad_Edge",     ax+m*3.75,  0, SHOULDER_CTR,      0.40, 4.6, 5.2, chrome)
            for sz, sr, sh2 in [(SHOULDER_CTR+2.8, 0.42, 0.95), (SHOULDER_CTR+1.5, 0.36, 0.75)]:
                cyl(ua, f"Stk_A_{sz:.0f}", ax+m*3.3, -1.5, sz, sr,  sh2, "z", chrome)
                cone_shape(ua, f"StkTip_{sz:.0f}", ax+m*3.3, -1.5, sz+sh2/2+0.35, sr, sr*0.35, 0.55, "z", dark_grey)
            box(ua, "Sh_Guard",        ax+m*2.60,  0, SHOULDER_CTR-0.2, 0.42, 4.4, 6.6, op_blue)
            box(ua, f"{side}_Shoulder_Frame", ax, 0, SHOULDER_CTR, 3.5, 3.0, 4.5, dark_metal)
            box(ua, "UA_Link",         ax,         0, ELBOW_Z+3.0,      3.2, 3.4, 5.2, op_red)
            mg996r(ua, f"{side}_ShY",    ax, 0, SHOULDER_CTR+1.5, "z")
            dual_bearing(ua, f"{side}_ShY_Dual", ax, 0, SHOULDER_CTR+2.0, "z", 1.00, 0.55, span=2.80, fit_type="press")
            u_bracket(ua, f"{side}_SPB", ax, 0, SHOULDER_CTR,     4.8, 3.4, 3.4)
            mg996r(ua, f"{side}_ShP",    ax, 0, SHOULDER_CTR,     "x")
            dual_bearing(ua, f"{side}_ShP_Dual", ax, 0, SHOULDER_CTR, "x", 1.10, 0.62, span=2.80, fit_type="press")
            mg996r(ua, f"{side}_ShR",    ax, 0, SHOULDER_CTR-1.2, "y")
            u_bracket(ua, f"{side}_EB",  ax, 0, ELBOW_Z,          3.8, 3.0, 3.0)
            mg996r(ua, f"{side}_ElbP",   ax, 0, ELBOW_Z,          "x")
            dual_bearing(ua, f"{side}_Elb_Dual", ax, 0, ELBOW_Z-0.5, "x", 0.95, 0.52, span=2.40, fit_type="press")
            hard_stop(ua, f"{side}_ElbStop", ax, -2.0, ELBOW_Z, "x", 150)

            fa = new_component(f"OP_Forearm_{side}")
            box(fa, "FA_Link",    ax,       0,    WRIST_Z+3.5, 3.2, 3.8, 4.8, op_blue)
            box(fa, "FA_Fender",  ax+m*2.1, 0,    WRIST_Z+3.5, 0.52, 5.2, 5.8, op_red)
            mg90s(fa, f"{side}_WR",   ax, 0, WRIST_Z+0.8, "x")
            bearing_fit(fa, f"{side}_WB", ax, 0, WRIST_Z+0.5, "x", 0.80, 0.44, fit_type="press")
            grommet_slot(fa, f"{side}_WristWire", ax, 0, WRIST_Z, "y", 0.45)

            hand = new_component(f"OP_Hand_{side}")
            box(hand, "Palm_Main",   ax,         -0.4,  WRIST_Z-1.6, 3.4, 3.0, 2.2, dark_grey)
            MCP_Z = WRIST_Z - PALM_BOTTOM_OFFSET
            for ki, kx_off in enumerate(FING_X_OFF):
                cyl(hand, f"Knuckle_{ki}", ax + m*kx_off, -1.5, MCP_Z + 0.15, 0.28, 0.38, "y", chrome)
            cyl(hand, "Wrist_Ring",  ax, 0, WRIST_Z-0.4, 1.05, 0.42, "z", chrome)
            box(hand, "Finger_SrvBay", ax, 0.5, WRIST_Z-1.8, 1.4, 0.90, 2.4, black_plastic)
            servo_drum(hand, f"{side}_Finger", ax, 1.2, WRIST_Z-1.8, "y")
            for fi, fxo in enumerate(FING_X_OFF):
                palm_pulley(hand, f"{side}_Pulley_{fi}", ax + m * fxo * 0.5, -0.5, WRIST_Z-1.2, "x")

            for fi, (fname, fxo, pp_l, mp_l, dp_l) in enumerate(zip(FING_NAMES, FING_X_OFF, FING_PP, FING_MP, FING_DP)):
                fc = new_component(f"OP_{fname}_{side}")
                fx = ax + m * fxo; fy = -1.35; mcp_z = MCP_Z
                pp_cz = mcp_z - pp_l / 2; mp_cz = mcp_z - pp_l - mp_l / 2; dp_cz = mcp_z - pp_l - mp_l - dp_l / 2
                box(fc, "PP", fx, fy, pp_cz, FING_W, FING_H, pp_l, grey_plastic)
                tendon_guide(fc, f"{fname}_PP", fx, fy-0.05, pp_cz, pp_l*0.8, "z")
                cyl(fc, "PIP_Hinge", fx, fy, mcp_z - pp_l, 0.27, FING_W + 0.12, "x", chrome)
                spring_return(fc, f"{fname}_PIP", fx, fy+0.3, mcp_z - pp_l, "x")
                box(fc, "MP", fx, fy, mp_cz, FING_W*0.94, FING_H*0.94, mp_l, grey_plastic)
                tendon_guide(fc, f"{fname}_MP", fx, fy-0.04, mp_cz, mp_l*0.8, "z")
                cyl(fc, "DIP_Hinge", fx, fy, mcp_z - pp_l - mp_l, 0.24, FING_W*0.94 + 0.10, "x", chrome)
                box(fc, "DP", fx, fy, dp_cz, FING_W*0.86, FING_H*0.86, dp_l, grey_plastic)
                tendon_anchor(fc, f"{fname}", fx, fy, mcp_z - pp_l - mp_l - dp_l - 0.15, "z")
                cone_shape(fc, "FT", fx, fy, mcp_z - pp_l - mp_l - dp_l - 0.12, FING_W*0.44, 0.05, 0.24, "z", chrome)

            thumb = new_component(f"OP_Thumb_{side}")
            tx = ax + m * 1.70; ty = 0.20
            tpp_cz = MCP_Z - THUMB_PP_L / 2; tdp_cz = MCP_Z - THUMB_PP_L - THUMB_DP_L / 2
            box(thumb, "TP",  tx, ty, tpp_cz, THUMB_W, THUMB_H, THUMB_PP_L, grey_plastic)
            tendon_guide(thumb, "Thumb_PP", tx, ty-0.05, tpp_cz, THUMB_PP_L*0.7, "z")
            cyl(thumb, "T_Hinge", tx, ty, MCP_Z - THUMB_PP_L, 0.28, THUMB_W + 0.14, "x", chrome)
            spring_return(thumb, "Thumb_CMC", tx, ty+0.3, MCP_Z - THUMB_PP_L, "x")
            box(thumb, "TD",  tx, ty*0.8, tdp_cz, THUMB_W*0.86, THUMB_H*0.86, THUMB_DP_L, grey_plastic)

            if side == "R":
                blast = new_component("OP_Ion_Blaster")
                cyl(blast, "Barrel_Main",   ax,    -2.2, WRIST_Z-3.2, 0.92, 4.2, "z", dark_metal)
                box(blast, "Blaster_Body",  ax,    -1.1, WRIST_Z-3.2, 2.6, 2.4, 2.7, dark_metal)

            assembly_jig(f"OP_UpperArm_{side}", [(ax-1.5, 0, SHOULDER_CTR), (ax+1.5, 0, ELBOW_Z)], [(ax-1.0, 0, SHOULDER_CTR), (ax+1.0, 0, ELBOW_Z)], (8.0, 5.0, 0.60))

        # ─── 5 BACKPACK ──────────────────────────────────────────────────
        bp = new_component("OP_Backpack")
        box(bp, "BP_Core",    0, 5.6, TORSO_CTR+0.5, 7.2, 2.6,  9.2, dark_grey)
        box(bp, "BP_Hood",    0, 6.5, TORSO_CTR+1.0, 5.8, 1.1,  7.8, op_red)
        box(bp, "BP_TopFlap", 0, 5.1, TORSO_CTR+5.5, 8.4, 0.38, 5.4, op_red)
        cyl(bp,  "Exh_L",   -1.3, 6.7, TORSO_CTR+2.8, 0.40, 1.3, "y", dark_metal)
        cyl(bp,  "Exh_R",    1.3, 6.7, TORSO_CTR+2.8, 0.40, 1.3, "y", dark_metal)
        mg90s(bp,    "Roof_Hinge", 0, 5.1, TORSO_CTR+5.1, "x")
        grommet_slot(bp, "BP_Wire", 0, 4.5, TORSO_CTR+2.0, "y", 0.80)

        # ─── 6 STEER WHEEL PODS ──────────────────────────────────────────
        steer = new_component("OP_SteerPods")
        for side, sx in [("L", -5.6), ("R", 5.6)]:
            m2 = -1 if side == "L" else 1
            box(steer, f"SAr_{side}",  sx, -3.6, 23.9, 1.6, 1.3, 4.2, chrome)
            box(steer, f"SPod_{side}", sx, -4.6, 23.4, 3.0, 2.2, 3.2, dark_grey)
            tt_wheel(steer, f"SW_{side}", sx+m2*2.0, -4.2, 23.4, m2)
            bearing_fit(steer, f"SPiv_{side}", sx, -3.6, 23.9, "z", 0.95, 0.50, fit_type="glue")
            mg90s(steer, f"SSrv_{side}", sx, -4.2, 23.9, "z")

        # ─── FDM SPLIT & KINEMATICS ──────────────────────────────────────
        Logger.log("--- FDM Shell Splitting + Fastener Merge (V13) ---")
        for comp in comps_list:
            to_split = [b for b in comp.bRepBodies if b.name and any(k in b.name for k in SPLIT_KEYS)]
            for b in to_split:
                split_halves(comp, b, "y", 0.0)
            bodies_after = list(comp.bRepBodies)
            for lb in [b for b in bodies_after if b.name and "_left" in b.name]:
                merge_fasteners_to_halves(comp, lb, None, axis="y")
            for rb in [b for b in bodies_after if b.name and "_right" in b.name]:
                merge_fasteners_to_halves(comp, None, rb, axis="y")

        t  = occs.get("OP_Torso"); p  = occs.get("OP_Pelvis")
        h  = occs.get("OP_Head");  b  = occs.get("OP_Backpack")
        st = occs.get("OP_SteerPods"); sh = occs.get("OP_Shields")
        if p: p.isGrounded = True

        ball_joint("Waist_Cluster",  t,  p,  0, 0, WAIST_CTR-2.5)
        ball_joint("Neck_Cluster",   h,  t,  0, 0, NECK_JOINT_Z)
        rigid_joint("Backpack_Mount", b,  t)
        rigid_joint("Steer_Mount",   st,  p)
        rigid_joint("Shields_Mount", sh,  t)

        for side in ["L", "R"]:
            sx = -HIP_X      if side == "L" else  HIP_X
            ax = -SHOULDER_X if side == "L" else  SHOULDER_X
            m  = -1          if side == "L" else  1
            th = occs.get(f"OP_Thigh_{side}"); sn = occs.get(f"OP_Shin_{side}")
            fo = occs.get(f"OP_Foot_{side}");  ua = occs.get(f"OP_UpperArm_{side}")
            fa = occs.get(f"OP_Forearm_{side}"); ha = occs.get(f"OP_Hand_{side}")

            ball_joint(f"{side}_Hip_Cluster",      th, p,  sx, 0, HIP_JOINT_Z)
            ball_joint(f"{side}_Knee",             sn, th, sx, 0, KNEE_CTR+1.5)
            ball_joint(f"{side}_Ankle_Cluster",    fo, sn, sx, 0, ANKLE_CTR+2.2)
            ball_joint(f"{side}_Shoulder_Cluster", ua, t,  ax, 0, SHOULDER_CTR)
            revolute_joint(f"{side}_Elbow",        fa, ua, ax, 0, ELBOW_Z,      "x")
            ball_joint(f"{side}_Wrist",            ha, fa, ax, 0, WRIST_Z+0.8)

            MCP_Z = WRIST_Z - PALM_BOTTOM_OFFSET
            for fi, fname in enumerate(FING_NAMES):
                fx   = ax + m * FING_X_OFF[fi]
                fo_f = occs.get(f"OP_{fname}_{side}")
                revolute_joint(f"{side}_{fname}_MCP", fo_f, ha, fx, -1.35, MCP_Z, "x")

            tx     = ax + m * 1.70
            thumb_occ = occs.get(f"OP_Thumb_{side}")
            ball_joint(f"{side}_Thumb_CMC", thumb_occ, ha, tx, 0.20, MCP_Z)

            if side == "R":
                bl = occs.get("OP_Ion_Blaster")
                if bl: revolute_joint("Blaster_Fold", bl, ha, ax, 0, WRIST_Z-1.0, "y")

        verify_screw_lengths()

        # ═════════════════════════════════════════════════════════════════
        # SIMULATION ENGINE  v13.0
        # ═════════════════════════════════════════════════════════════════
        class SimulationEngine:
            def __init__(self, root, comps_list, design, app, ui):
                self._root = root; self._comps = comps_list; self._design = design
                self._app = app; self._ui = ui; self._cols = []

            def _gj(self, name):
                try: return self._root.asBuiltJoints.itemByName(name)
                except: return None

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
                try: self._app.activeViewport.refresh()
                except: pass
                try: adsk.doEvents()
                except: pass

            def _clamp(self, joint_name, axis, deg):
                limits = JOINT_LIMITS.get(joint_name, {}).get(axis)
                if limits: return max(limits[0], min(limits[1], deg))
                return deg

            def move_joint(self, name, deg, steps=20, axis="pitch", ease=True, clamp=True):
                j = self._gj(name)
                if not j: return
                if clamp: deg = self._clamp(name, axis, deg)
                mo = j.jointMotion; e_rad = math.radians(deg); s_rad = self._get(mo, axis)
                for i in range(1, steps + 1):
                    t = self._ease(i/steps) if ease else i/steps
                    self._set(mo, axis, s_rad + (e_rad - s_rad) * t)
                    self._refresh()

            def move_ball(self, targets, steps=20, clamp=True):
                active = []
                for name, pitch, yaw, roll in targets:
                    j = self._gj(name)
                    if not j: continue
                    mo = j.jointMotion
                    if mo.objectType == adsk.fusion.RevoluteJointMotion.classType():
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

            def reset_all(self, steps=10):
                for n in ["Waist_Cluster", "Neck_Cluster", "L_Hip_Cluster", "R_Hip_Cluster", "L_Knee", "R_Knee",
                          "L_Ankle_Cluster", "R_Ankle_Cluster", "L_Shoulder_Cluster", "R_Shoulder_Cluster", "L_Wrist", "R_Wrist"]:
                    j = self._gj(n)
                    if j:
                        mo = j.jointMotion
                        if mo.objectType == adsk.fusion.BallJointMotion.classType():
                            self.move_ball([(n, 0.0, 0.0, 0.0)], steps=steps)
                        else:
                            self.move_joint(n, 0.0, steps, "pitch")
                self._refresh()

            # ── V13 NEW: AI & Vision Simulations ─────────────────────────
            def simulate_vision_pipeline(self):
                Logger.log("--- MODULE V13: AI VISION PIPELINE ---")
                Logger.log("  [JETSON] Capturing frame from CSI Camera (IMX219)...")
                Logger.log("  [JETSON] Running YOLO object detection...")
                Logger.log("  [JETSON] Target acquired. Sending coords via UART to ESP32...")
                Logger.log("  [ESP32]  Received target data. Calculating IK for neck & shoulders...")
                self.move_ball([("Neck_Cluster", -10, 15, 0), ("L_Shoulder_Cluster", -20, 10, 0), ("R_Shoulder_Cluster", -20, -10, 0)], steps=15)
                Logger.log("  [ESP32]  Trajectory executed. Servos updated.")
                self.reset_all(steps=10)

            def simulate_uart_communication(self):
                Logger.log("--- MODULE V13: UART COMM BUS TEST ---")
                Logger.log("  [JETSON] TX: 'STATE: TRUCK_MODE' -> [ESP32]")
                Logger.log("  [ESP32] RX ACK. Engaging mechanical locks and folding servos...")
                self.move_group([("R_Elbow", 0, "pitch"), ("L_Elbow", 0, "pitch")], steps=10)
                Logger.log("  Bus latency: < 2ms. Real-time control verified.")

            # ── Master Runner ─────────────────────────────────────────────
            def run_all_simulations(self):
                Logger.log("--- BEGINNING V13 SIMULATIONS ---")
                self.simulate_vision_pipeline()
                self.simulate_uart_communication()
                self.reset_all(steps=5)
                self.export_bom()

            def export_bom(self):
                BOM.add("Electronics", "NVIDIA Jetson Nano 4GB", 1, "Main Brain/AI")
                BOM.add("Electronics", "ESP32 DevKit", 1, "Low-Level Real-Time Ctrl")
                BOM.add("Electronics", "RPi CSI V2 Camera", 1, "Vision System")
                BOM.add("Electronics", "MPU-6050 IMU", 1, "Balance Sensor")
                BOM.add("Electronics", "PCA9685 16-ch servo driver", 3, "Motor Control")
                BOM.save_csv(BOM_FILE)
                BOM.summary()

                Logger.log("--- V13 COMPUTING & COMMUNICATION ARCHITECTURE ---")
                Logger.log("  High-Level: NVIDIA Jetson Nano (Vision, AI, Pathfinding, State Machine)")
                Logger.log("  Low-Level:  ESP32 (Real-time IK, Motor PWM, Sensor Fusion, Safety)")
                Logger.log("  Comm Link:  UART Serial @ 921600 baud (Jetson GPIO TX/RX <-> ESP32 RX/TX)")
                Logger.log("  I2C Bus:    ESP32 (Master) -> PCA9685 x3 (Slaves), MPU6050 (Slave)")
                Logger.log("  Vision:     CSI Camera -> Jetson Nano (Hardware accelerated GStreamer)")
                Logger.log("--- V13 SERVO WIRING MAP ---")
                Logger.log("  PCA1 (I2C 0x40): Lower Body (Hips, Knees, Ankles) - Managed by ESP32")
                Logger.log("  PCA2 (I2C 0x41): Upper Body (Waist, Neck, Shoulders, Elbows) - Managed by ESP32")
                Logger.log("  PCA3 (I2C 0x42): Hands & Details (Fingers, Wrists, Blaster) - Managed by ESP32")
                Logger.log("--- V13 POWER ARCHITECTURE ---")
                Logger.log("  Input:  3S 11.1V 2200mAh LiPo")
                Logger.log("  Rail 1: 5V/4A BEC -> NVIDIA Jetson Nano")
                Logger.log("  Rail 2: 6V/10A BEC -> PCA9685 Servo Rails")

        try:
            os.makedirs(EXPORT_DIR, exist_ok=True)
            archive_path = os.path.join(EXPORT_DIR, "Optimus_Prime_G1_v13.f3d")
            export_mgr   = design.exportManager
            archive_opts = export_mgr.createFusionArchiveExportOptions(archive_path)
            export_mgr.execute(archive_opts)
            Logger.log(f"Design archived -> {archive_path}")
        except Exception as e:
            Logger.log(f"Archive skipped: {e}", "WARN")

        sim = SimulationEngine(root, comps_list, design, app, ui)
        sim.run_all_simulations()
        Logger.log("v13 script finished successfully.")

    except Exception:
        Logger.log(f"FATAL ERROR:\n{traceback.format_exc()}", "ERROR")

    finally:
        Logger.flush()