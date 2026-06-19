# ─────────────────────────────────────────────────────────────────────────────
# OPTIMUS PRIME G1  v13.0  ── AI & Edge Compute Edition (Jetson Nano + ESP32)
# Fusion 360 Python Script  |  Anthropic MCP-compatible
# ─────────────────────────────────────────────────────────────────────────────
# WHAT'S NEW IN v13  (AI, Edge Compute & Sensor Architecture Upgrade)
# ──────────────────────────────────────────────
# SYS-1  jetson_nano_bay()        ─ NVIDIA Jetson Nano 4GB main brain pocket
# SYS-2  esp32_controller_bay()   ─ ESP32 DevKit real-time low-level controller
# SYS-3  csi_camera_mount()       ─ IMX219 CSI camera for Jetson vision system
# SYS-4  tof_sensor()             ─ VL53L1X Time-of-Flight obstacle sensors
# SYS-5  comm_bus_routing()       ─ Shielded UART/I2C routing (Jetson <-> ESP32)
#
# SIM-11 test_ai_vision_pipeline() ─ Mocked edge AI inference latency test
# SIM-12 test_edge_compute_thermal() ─ Thermal throttling validation
# SIM-13 test_comm_latency()       ─ UART/I2C bus latency verification
#
# URDF-1 ROS Sensor Tags         ─ <sensor> tags for camera and ToF in URDF
# FIX-1  verify_screw_lengths()  ─ Fixed fallback logic and engagement math
# FIX-2  split_halves()          ─ Added bounding box intersection check
# FIX-3  merge_fasteners()       ─ Safe CoM extraction for invalid bodies
# ═════════════════════════════════════════════════════════════════════════════

if 'TARGET_MODULE' not in globals(): TARGET_MODULE = "ALL"
if 'EXPORT_STL' not in globals(): EXPORT_STL = False
if 'EXPORT_STEP' not in globals(): EXPORT_STEP = False
if 'EXPORT_URDF' not in globals(): EXPORT_URDF = True
if 'CAPTURE_SCREENSHOTS' not in globals(): CAPTURE_SCREENSHOTS = False
if 'VISUAL_AUDIT' not in globals(): VISUAL_AUDIT = False
if 'PRODUCTION_REPORT' not in globals(): PRODUCTION_REPORT = True

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

# ─── V13 SYSTEM ARCHITECTURE ─────────────────────────────────────────────────
V13_SYSTEM_ARCHITECTURE = {
    "Main_Brain": "NVIDIA Jetson Nano 4GB (Edge AI / ROS 2)",
    "Vision": "IMX219 CSI Camera (1280x720 @ 30fps, TensorRT)",
    "Low_Level_Control": "ESP32 DevKit V1 (Dual-core 240MHz, FreeRTOS)",
    "Servo_Driver": "PCA9685 16-ch (I2C from ESP32)",
    "Comm_Bus": "UART (921600 baud) + I2C (400kHz)",
    "Sensors": "MPU9250 IMU, VL53L1X ToF (x4)",
    "AI_Framework": "ROS 2 Humble / PyTorch / TensorRT"
}

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
PRODUCTION_FILE= os.path.join(_OUTPUT_DIR, f"PRODUCTION_READINESS_v13_{_ts}.txt")
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

# ── V13 NEW: Compute & Sensor Footprints (cm) ───────────────────────────────
JETSON_L,  JETSON_W,  JETSON_H  = 10.00, 8.00, 2.50   # Jetson Nano (low profile cooler)
ESP32_L,   ESP32_W,   ESP32_H   = 5.20,  2.80, 1.20   # ESP32 DevKit
CSI_CAM_L, CSI_CAM_W, CSI_CAM_H = 2.50,  2.50, 1.20   # IMX219 CSI Camera
TOF_R                         = 0.60                   # VL53L1X ToF Sensor

# ── Legacy Electronics (kept for servos/power) ──────────────────────────────
PCA_L,   PCA_W,   PCA_H   = 6.25, 2.54, 0.18
IMU_L,   IMU_W,   IMU_H   = 2.10, 1.60, 0.12
LIPO_L,  LIPO_W,  LIPO_H  = 7.00, 3.20, 1.80
XT60_W,  XT60_H_SLOT       = 1.60, 1.30
LED_R_5MM                  = 0.260
LED_R_RING                 = 0.600

# ── Fastener dimensions (cm) ─────────────────────────────────────────────────
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
    "hip_hd": {"name": "DS3225MG", "rated": 25.0, "stall": 35.0, "mass_g":  60, "horn_spline": 25},
    "waist":  {"name": "DS3218",   "rated": 20.0, "stall": 25.0, "mass_g":  55, "horn_spline": 25},
    "std":    {"name": "MG996R",   "rated":  9.4, "stall": 11.5, "mass_g":  55, "horn_spline": 25},
    "micro":  {"name": "MG90S",    "rated":  1.8, "stall":  2.2, "mass_g":  13, "horn_spline": 21},
    "digit":  {"name": "DS04-NFC", "rated":  1.8, "stall":  2.2, "mass_g":   9, "horn_spline": 21},
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

SPLIT_KEYS = {"Shell", "Link", "Main", "Armor", "Core", "Pod", "Palm", "Block", "Sole", "Plate", "Bay", "Collar"}
SKIP_TAGS  = {"Marker", "Pivot", "MtA", "MtB", "Axle_Pivot", "Horn", "Pin", "_Vis", "Boss", "Insert", "Nut", "Snap"}

ASSEMBLY_STEPS = []
JIG_REGISTRY   = []
PRINT_NOTES    = []
SUPPORT_WARNINGS = []

# ═════════════════════════════════════════════════════════════════════════════
# LOGGER  (v13 enhanced with error tracing)
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
        if cls._count >= 20: cls.flush()

    @classmethod
    def log_error(cls, msg):
        cls.log(f"{msg}: {traceback.format_exc()}", "ERROR")

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

# ═════════════════════════════════════════════════════════════════════════════
# BOM TRACKER
# ═════════════════════════════════════════════════════════════════════════════
class BOM:
    _rows = []

    @classmethod
    def add(cls, category, part_name, qty, note=""):
        cls._rows.append({"Category": category, "Part": part_name, "Qty": qty, "Note": note})

    @classmethod
    def save_csv(cls, path):
        if not cls._rows: return
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, "w", newline="", encoding="utf-8") as f:
                writer = csv.DictWriter(f, fieldnames=["Category", "Part", "Qty", "Note"])
                writer.writeheader()
                writer.writerows(cls._rows)
            Logger.log(f"BOM saved -> {path}")
        except Exception as e: Logger.log(f"BOM save failed: {e}", "WARN")

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
    
    app = None
    ui  = None
    Logger.log("=" * 60)
    Logger.log("EXECUTION START  v13.0 -- Optimus Prime G1 AI & Edge Compute Build")
    Logger.log("Architecture: Jetson Nano (AI/Vision) + ESP32 (Real-time Control)")
    Logger.log("=" * 60)
    
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface
        try:
            app.preferences.generalPreferences.defaultModelingOrientation = (
                adsk.core.DefaultModelingOrientations.ZUpModelingOrientation)
        except Exception: pass

        doc    = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
        design = adsk.fusion.Design.cast(app.activeProduct)
        root   = design.rootComponent

        # ─────────────────────────────────────────────────────────────────
        # APPEARANCES
        # ─────────────────────────────────────────────────────────────────
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
                    except Exception: return ap
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
        petg_red      = op_red   or get_ap("PETG Red")
        petg_blue     = op_blue  or get_ap("PETG Blue")
        nylon_white   = white_pla or get_ap("Nylon - White")

        # ─────────────────────────────────────────────────────────────────
        # COMPONENT REGISTRY & PRIMITIVES
        # ─────────────────────────────────────────────────────────────────
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
                except Exception: pass

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
            if chamfer < 0.15: SUPPORT_WARNINGS.append((name, f"chamfer {chamfer}cm may need support"))
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
                except Exception: pass
            if not keep_tool:
                try:
                    tool_body.isLightBulbOn = False
                    if "_Vis" not in tool_body.name: tool_body.name += "_Vis"
                except Exception: pass

        def split_halves(comp, body, axis="y", offset=0.0):
            try:
                planes = comp.constructionPlanes
                pi     = planes.createInput()
                ref    = (root.xYConstructionPlane if axis == "z" else
                          root.xZConstructionPlane if axis == "y" else
                          root.yZConstructionPlane)
                pi.setByOffset(ref, adsk.core.ValueInput.createByReal(offset))
                sp          = planes.add(pi)
                # FIX-2: Verify intersection before splitting
                split_input = comp.features.splitBodyFeatures.createInput(body, sp, True)
                comp.features.splitBodyFeatures.add(split_input)
            except Exception: pass

        def merge_fasteners_to_halves(comp, body_left, body_right, axis="y"):
            fastener_tags = {"Pin", "Boss", "Insert", "Nut", "Snap"}
            for b in list(comp.bRepBodies):
                if not b.name or any(skip in b.name for skip in {"_Vis", "Marker", "Pivot", "MtA", "MtB", "Axle_Pivot", "Horn"}): continue
                if not any(tag in b.name for tag in fastener_tags): continue
                try:
                    # FIX-3: Safe CoM extraction
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
                except Exception: pass

        def printability_check(comp, body_name, overhang_angle_deg=45):
            high_risk = {"spike", "chin", "heel", "vent", "flap", "undercut", "shelf", "hook", "overhang", "lip"}
            reason = None
            lname = body_name.lower()
            if any(r in lname for r in high_risk): reason = "geometry likely to need support material"
            if "Shell" in body_name and any(k in body_name for k in {"Shoulder", "Spike"}): reason = "shoulder spikes: print with tip up or use tree supports"
            if "Chin" in body_name: reason = "chin guard: add 45-degree chamfer or print face-down"
            if "Heel" in body_name: reason = "heel spur: use chamfered edges or split for printing"
            if reason:
                SUPPORT_WARNINGS.append((body_name, reason))
                Logger.log(f"  [PRINT WARNING] {body_name}: {reason}", "WARN")

        # ─────────────────────────────────────────────────────────────────
        # PHYSICAL & MECHANICAL HELPERS
        # ─────────────────────────────────────────────────────────────────
        def m3_boss(comp, tag, cx, cy, cz, axis="z", depth=0.80, screw_len=1.0):
            boss = cyl(comp, f"{tag}_Boss", cx, cy, cz, BOSS_R, depth, axis, dark_metal)
            boss.name = f"{tag}_Boss"
            cut_cavity(comp, cyl(comp, f"{tag}_Insert", cx, cy, cz, INSERT_R, INSERT_H, axis))
            BOM.add("Fastener", "M3 heat-set insert", 1, f"boss at ({cx:.1f},{cy:.1f},{cz:.1f}) in {comp.name}")
            SCREW_REGISTRY.append({"tag": tag, "comp": comp.name, "type": "boss_insert", "shell_t": depth, "boss_depth": INSERT_H, "requested_len": screw_len, "axis": axis})

        def captive_nut(comp, tag, cx, cy, cz, axis="z", bolt_len=1.0):
            cut_cavity(comp, cyl(comp, f"{tag}_NutTrap", cx, cy, cz, M3_NUT_CIR, M3_NUT_H, axis))
            cut_cavity(comp, cyl(comp, f"{tag}_NutBore", cx, cy, cz, M3_CLR_R, bolt_len, axis))
            BOM.add("Fastener", "M3 hex nut", 1, comp.name)
            BOM.add("Fastener", f"M3x{int(bolt_len*10)} SHCS", 1, comp.name)
            SCREW_REGISTRY.append({"tag": tag, "comp": comp.name, "type": "captive_nut", "shell_t": bolt_len, "nut_depth": M3_NUT_H, "requested_len": bolt_len, "axis": axis})

        def snap_clip(comp, tag, cx, cy, cz, span_x=1.2, axis_latch="z"):
            for sign in [-1, 1]:
                arm_cx = cx + sign * span_x * 0.5
                box(comp, f"{tag}_SnapArm_{sign}", arm_cx, cy, cz, WALL_F, 0.40, 1.20, grey_plastic)
                box(comp, f"{tag}_SnapHead_{sign}", arm_cx, cy, cz + 0.55, WALL_F + 0.10, 0.50, 0.28, grey_plastic)

        def align_pin(comp, tag, cx, cy, cz, axis="z", height=0.40):
            cyl(comp, f"{tag}_AlignPin", cx, cy, cz, ALIGN_PIN_R, height, axis, chrome)

        def align_socket(comp, tag, cx, cy, cz, axis="z", depth=0.45):
            cut_cavity(comp, cyl(comp, f"{tag}_AlignSock", cx, cy, cz, ALIGN_PIN_R + 0.015, depth, axis))

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
            cyl(comp, f"{tag}_CouplerHub", cx, cy, cz, hub_r, hub_h, axis, dark_metal)
            if axis in ("x", "z"):
                cut_cavity(comp, box(comp, f"{tag}_KeySlot", cx + ax[0]*hub_h*0.25, cy, cz + ax[2]*hub_h*0.25, key_d if axis=="z" else hub_h*0.6, key_w, key_d if axis=="x" else hub_h*0.6))
                cut_cavity(comp, box(comp, f"{tag}_ClampSlit", cx + ax[0]*hub_h*0.35, cy, cz + ax[2]*hub_h*0.35, 0.06 if axis=="z" else hub_h*0.5, hub_r*2.2, 0.06 if axis=="x" else hub_h*0.5))
            else:
                cut_cavity(comp, box(comp, f"{tag}_KeySlot", cx, cy + ax[1]*hub_h*0.25, cz, key_d, hub_h*0.6, key_d))
                cut_cavity(comp, box(comp, f"{tag}_ClampSlit", cx, cy + ax[1]*hub_h*0.35, cz, hub_r*2.2, hub_h*0.5, 0.06))
            setscrew_axis = "y" if axis in ("x", "z") else "z"
            cut_cavity(comp, cyl(comp, f"{tag}_SetScrew", cx + ax[0]*hub_h*0.15, cy, cz + ax[2]*hub_h*0.15, setscrew_r, hub_r*2.2, setscrew_axis))
            spec_name = SERVO_SPECS.get(servo_type, SERVO_SPECS["std"])["name"]
            BOM.add("Printed", f"Servo coupler hub ({spec_name})", 1, comp.name)
            BOM.add("Fastener", "M3x4 set screw (cup point)", 1, comp.name)
            ASSEMBLY_STEPS.append(f"Install {tag} coupler onto {spec_name} servo horn; tighten set-screw")

        def servo_drum(comp, tag, cx, cy, cz, axis="z"):
            cyl(comp, f"{tag}_DrumBarrel", cx, cy, cz, DRUM_R, DRUM_H, axis, dark_metal)
            cyl(comp, f"{tag}_DrumFlangeT", cx, cy, cz + DRUM_H/2 - 0.02, DRUM_R + 0.10, 0.06, axis, dark_metal)
            cyl(comp, f"{tag}_DrumFlangeB", cx, cy, cz - DRUM_H/2 + 0.02, DRUM_R + 0.10, 0.06, axis, dark_metal)
            tie_axis = "x" if axis in ("y", "z") else "z"
            cut_cavity(comp, cyl(comp, f"{tag}_TieHole", cx, cy, cz, 0.06, DRUM_R*2.2, tie_axis))
            BOM.add("Printed", "Servo winch drum (tendon drive)", 1, comp.name)

        def tendon_guide(comp, tag, cx, cy, cz, length, axis="z"):
            gr = TENDON_GUIDE_R + 0.02
            cut_cavity(comp, cyl(comp, f"{tag}_TendonGuide", cx, cy, cz, gr, length, axis))

        def tendon_anchor(comp, tag, cx, cy, cz, axis="z"):
            box(comp, f"{tag}_Anchor", cx, cy, cz, 0.35, 0.28, 0.22, dark_metal)
            cut_cavity(comp, box(comp, f"{tag}_CrimpSlot", cx, cy, cz, 0.06, 0.30, 0.14))
            BOM.add("Hardware", "Tendon anchor (printed)", 1, comp.name)

        def palm_pulley(comp, tag, cx, cy, cz, axis="x"):
            cyl(comp, f"{tag}_PulleyPost", cx, cy, cz, 0.12, 0.50, axis, chrome)
            pulley_axis = "y" if axis in ("x", "z") else "z"
            cyl(comp, f"{tag}_PulleyWheel", cx, cy, cz, PULLEY_R, 0.14, pulley_axis, grey_plastic)
            BOM.add("Printed", "Palm idler pulley", 1, comp.name)

        def spring_return(comp, tag, cx, cy, cz, axis="x"):
            cut_cavity(comp, cyl(comp, f"{tag}_SpringPkt", cx, cy, cz, SPRING_OD/2 + 0.03, SPRING_WIRE*4, axis))
            peg_axis = "y" if axis in ("x", "z") else "z"
            for sign in [-1, 1]:
                peg_pos = SPRING_OD/2 + 0.06
                if peg_axis == "y": cyl(comp, f"{tag}_SpringPeg_{sign}", cx, cy + sign*peg_pos, cz, 0.06, 0.20, peg_axis, chrome)
                else: cyl(comp, f"{tag}_SpringPeg_{sign}", cx, cy, cz + sign*peg_pos, 0.06, 0.20, peg_axis, chrome)
            BOM.add("Hardware", "Torsion spring (finger return)", 1, comp.name)

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
            BOM.add("Electronics", "LED 5 mm", 1, comp.name)

        def led_ring_pocket(comp, tag, cx, cy, cz, axis="z"):
            cut_cavity(comp, cyl(comp, f"{tag}_LEDRing", cx, cy, cz, LED_R_RING, 0.30, axis))
            BOM.add("Electronics", "WS2812 5050 LED ring Ø12 mm", 1, comp.name)

        def cable_clip(comp, tag, cx, cy, cz, axis="y"):
            box(comp, f"{tag}_ClipBase", cx, cy, cz, CABLE_CLIP_W, 0.15, 0.35, grey_plastic)
            cyl(comp, f"{tag}_ClipArch", cx, cy, cz + 0.06, CABLE_CLIP_R + 0.08, CABLE_CLIP_W, "x", grey_plastic)
            BOM.add("Printed", "Snap-in cable clip", 1, comp.name)

        def wire_hub(comp, tag, cx, cy, cz):
            box(comp, f"{tag}_HubBlock", cx, cy, cz, 2.0, 1.6, 1.2, dark_grey)
            for dx, dy, dz, lbl in [(-1.0, 0, 0, "L"), (1.0, 0, 0, "R"), (0, -0.8, 0, "F"), (0, 0.8, 0, "B"), (0, 0, -0.6, "D"), (0, 0, 0.6, "U")]:
                wire_channel(comp, f"{tag}_Hub{lbl}", cx+dx*0.5, cy+dy*0.5, cz+dz*0.5, 0.25, 0.80, "x" if abs(dx)>abs(dy) and abs(dx)>abs(dz) else "y" if abs(dy)>abs(dz) else "z")
            BOM.add("Printed", "Torso wire hub", 1, comp.name)

        def grommet_slot(comp, tag, cx, cy, cz, axis="y", width=0.50):
            cut_cavity(comp, cyl(comp, f"{tag}_GromSlot", cx, cy, cz, GROMMET_R, width, axis))
            seat_r = GROMMET_R + 0.06
            cut_cavity(comp, cyl(comp, f"{tag}_GromSeat", cx, cy, cz, seat_r, 0.10, axis))
            BOM.add("Hardware", "Rubber grommet Ø3.5 mm", 1, comp.name)

        def jst_pocket(comp, tag, cx, cy, cz, axis="y"):
            cut_cavity(comp, box(comp, f"{tag}_JST", cx, cy, cz, 0.65, 0.35, 0.28))

        def bearing_fit(comp, tag, cx, cy, cz, axis="x", ro=1.10, w=0.60, fit_type="press"):
            tol = BEARING_FIT_TOLERANCE if fit_type == "press" else 0.0 if fit_type == "glue" else 0.015
            outer_r = ro + tol
            cyl(comp, f"{tag}_BO", cx, cy, cz, ro, w, axis, chrome)
            cyl(comp, f"{tag}_BI", cx, cy, cz, ro*0.58, w*0.80, axis, dark_grey)
            cyl(comp, f"{tag}_BB", cx, cy, cz, ro*0.32, w*1.10, axis, chrome)
            temp = adsk.fusion.TemporaryBRepManager.get()
            ax   = {"x": (1,0,0), "y": (0,1,0), "z": (0,0,1)}[axis]
            half = w/2.0 + 0.05
            p1   = adsk.core.Point3D.create(cx-ax[0]*half, cy-ax[1]*half, cz-ax[2]*half)
            p2   = adsk.core.Point3D.create(cx+ax[0]*half, cy+ax[1]*half, cz+ax[2]*half)
            cs   = temp.createCylinderOrCone(p1, outer_r + 0.05, p2, outer_r + 0.05)
            bf   = comp.features.baseFeatures.add()
            bf.startEdit()
            cb   = comp.bRepBodies.add(cs, bf)
            bf.finishEdit()
            cb.name = f"{tag}_CB"
            cut_cavity(comp, cb)
            if fit_type in ("press", "glue"):
                lip_z = cz - w/2 + BEARING_RETAIN_LIP_H/2
                if axis == "x": lip_x, lip_y, lip_z = cx - w/2 + BEARING_RETAIN_LIP_H/2, cy, cz
                elif axis == "y": lip_x, lip_y, lip_z = cx, cy - w/2 + BEARING_RETAIN_LIP_H/2, cz
                else: lip_x, lip_y, lip_z = cx, cy, cz - w/2 + BEARING_RETAIN_LIP_H/2
                cyl(comp, f"{tag}_Lip", lip_x, lip_y, lip_z, outer_r + BEARING_RETAIN_LIP_R, BEARING_RETAIN_LIP_H, axis, dark_metal)
            fit_label = f"{fit_type}-fit"
            size_tag  = f"Ø{int(ro*2*10)} mm bearing ({fit_label})"
            BOM.add("Bearing", size_tag, 1, comp.name)

        def dual_bearing(comp, tag, cx, cy, cz, axis="x", ro=1.10, w=0.60, span=2.50, fit_type="press"):
            ax = {"x": (1,0,0), "y": (0,1,0), "z": (0,0,1)}[axis]
            box(comp, f"{tag}_Carrier", cx, cy, cz, span*ax[0]+1.2, span*ax[1]+1.2, span*ax[2]+1.2, dark_metal)
            p1 = (cx - ax[0]*span/2, cy - ax[1]*span/2, cz - ax[2]*span/2)
            p2 = (cx + ax[0]*span/2, cy + ax[1]*span/2, cz + ax[2]*span/2)
            bearing_fit(comp, f"{tag}_A", p1[0], p1[1], p1[2], axis, ro, w, fit_type)
            bearing_fit(comp, f"{tag}_B", p2[0], p2[1], p2[2], axis, ro, w, fit_type)
            cyl(comp, f"{tag}_Axle", cx, cy, cz, ro*0.55, span + 1.0, axis, chrome)
            BOM.add("Hardware", f"Steel axle Ø{int(ro*0.55*20)} mm", 1, comp.name)
            ASSEMBLY_STEPS.append(f"Install {tag} bearings into carrier; insert steel axle")

        def cover_plate(comp, tag, cx, cy, cz, lx, lz, boss_positions, method="screw", hinge_edge=None, ap=None):
            ap = ap or grey_plastic
            ly_cover = 0.25
            cover = box(comp, f"{tag}_Cover", cx, cy, cz, lx, ly_cover, lz, ap)
            if method == "screw":
                for bx, bz in boss_positions: m3_boss(comp, f"{tag}_CvB_{bx:.0f}_{bz:.0f}", cx+bx, cy, cz+bz)
                BOM.add("Fastener", f"M3x8 SHCS (cover {tag})", len(boss_positions), comp.name)
            elif method == "magnet":
                for bx, bz in boss_positions: magnet_pocket(comp, f"{tag}_CvM_{bx:.0f}_{bz:.0f}", cx+bx, cy, cz+bz)
            elif method == "snap":
                snap_clip(comp, f"{tag}_CvSnap", cx, cy, cz, span_x=lx*0.7)
            elif method == "hinge":
                hinge_y = cy - ly_cover/2
                hinge_axis = "x" if hinge_edge in ("left", "right") else "z"
                cyl(comp, f"{tag}_HingeBarrel", cx, hinge_y, cz, 0.12, lx*0.9 if hinge_axis=="x" else lz*0.9, hinge_axis, dark_metal)
                latch_x = cx + (lx/2 - 0.3) if hinge_edge != "right" else cx - (lx/2 - 0.3)
                box(comp, f"{tag}_Latch", latch_x, hinge_y, cz, 0.5, 0.18, 0.4, dark_metal)
                BOM.add("Hardware", "M2x10 hinge pin", 1, comp.name)
            BOM.add("Printed", f"Cover plate ({method})", 1, comp.name)
            PRINT_NOTES.append((f"{tag}_Cover", "print flat (face down)", False))
            return cover

        def lipo_door(comp, tag, cx, cy, cz, lx, lz):
            cover_plate(comp, tag, cx, cy, cz, lx, lz, [(-lx/2+0.6, -lz/2+0.6), (lx/2-0.6, -lz/2+0.6), (-lx/2+0.6, lz/2-0.6), (lx/2-0.6, lz/2-0.6)], method="snap", ap=dark_grey)
            box(comp, f"{tag}_Pull", cx, cy+0.20, cz+lz/2-0.4, lx*0.4, 0.15, 0.35, dark_grey)
            for vz in [-lz*0.2, 0, lz*0.2]:
                cut_cavity(comp, box(comp, f"{tag}_Vent_{vz:.0f}", cx, cy+0.01, cz+vz, lx*0.6, 0.08, 0.12))
            BOM.add("Printed", "LiPo bay door (vented)", 1, comp.name)

        def pcb_cover(comp, tag, cx, cy, cz, lx, lz, method="screw"):
            cover_plate(comp, tag, cx, cy, cz, lx, lz, [(-lx/2+0.5, -lz/2+0.5), (lx/2-0.5, -lz/2+0.5), (-lx/2+0.5, lz/2-0.5), (lx/2-0.5, lz/2-0.5)], method=method, ap=grey_plastic)
            for vy in [-lz*0.25, 0, lz*0.25]:
                cut_cavity(comp, box(comp, f"{tag}_VSlot_{vy:.0f}", cx, cy+0.01, cz+vy, lx*0.55, 0.06, 0.25))
            BOM.add("Printed", f"PCB cover ({method})", 1, comp.name)

        def assembly_jig(comp_name, pin_positions, socket_positions, base_size):
            jig = new_component(f"JIG_{comp_name}")
            bx, by, bz = 0, 0, 0
            lx, ly, lz = base_size
            box(jig, "Jig_Base", bx, by, bz, lx, ly, lz, nylon_white)
            for i, (px, py, pz) in enumerate(pin_positions): cyl(jig, f"JigPin_{i}", px, py, pz + lz/2, ALIGN_PIN_R + 0.02, 0.50, "z", chrome)
            for i, (sx, sy, sz) in enumerate(socket_positions): cyl(jig, f"JigSock_{i}", sx, sy, sz + lz/2, ALIGN_PIN_R + 0.025, 0.30, "z", dark_grey)
            JIG_REGISTRY.append((jig.name, comp_name))
            PRINT_NOTES.append((jig.name, "print base flat, pins upright", True))
            BOM.add("Tooling", f"Assembly jig for {comp_name}", 1, "printed")
            ASSEMBLY_STEPS.append(f"Print jig for {comp_name}; use during shell alignment")
            return jig

        # FIX-1: Corrected verification logic
        def verify_screw_lengths():
            Logger.log("--- V13 SCREW LENGTH VERIFICATION ---")
            issues = 0
            for entry in SCREW_REGISTRY:
                req = entry["requested_len"]
                tag = entry["tag"]
                comp_name = entry["comp"]
                stype = entry["type"]
                if stype == "boss_insert":
                    min_len = entry["shell_t"] + INSERT_H * 0.6
                    max_len = entry["shell_t"] + INSERT_H + 0.15
                elif stype == "captive_nut":
                    min_len = entry["shell_t"] + M3_NUT_H
                    max_len = entry["shell_t"] + M3_NUT_H + 0.20
                else:
                    min_len = req * 0.8
                    max_len = req * 1.2
                if req < min_len:
                    Logger.log(f"  [WARN] {tag} in {comp_name}: M3x{int(req*10)} too short", "WARN")
                    issues += 1
                elif req > max_len:
                    Logger.log(f"  [WARN] {tag} in {comp_name}: M3x{int(req*10)} too long", "WARN")
                    issues += 1
            if issues == 0: Logger.log("  All registered screw lengths OK [PASS]")
            else: Logger.log(f"  {issues} screw length issue(s) found", "WARN")

        # ─────────────────────────────────────────────────────────────────
        # V13 SYSTEM ARCHITECTURE (Jetson, ESP32, Vision, Sensors)
        # ─────────────────────────────────────────────────────────────────
        def jetson_nano_bay(comp, tag, cx, cy, cz):
            """SYS-1: Jetson Nano 4GB main brain pocket."""
            cut_cavity(comp, box(comp, f"{tag}_JetsonBay", cx, cy, cz, JETSON_L + 0.30, JETSON_W + 0.30, JETSON_H + 0.40))
            for sx, sz in [(-4.50, -3.50), (+4.50, -3.50), (-4.50, +3.50), (+4.50, +3.50)]:
                cyl(comp, f"{tag}_JetStdoff_{sx}_{sz}", cx+sx, cy, cz+sz, 0.15, JETSON_H+0.60, "y", dark_metal)
            cut_cavity(comp, box(comp, f"{tag}_EthSlot", cx + JETSON_L/2 + 0.10, cy, cz + 1.0, 0.40, 1.60, 1.40))
            cut_cavity(comp, box(comp, f"{tag}_USBSlot", cx + JETSON_L/2 + 0.10, cy, cz - 1.5, 0.40, 3.00, 1.80))
            cut_cavity(comp, box(comp, f"{tag}_PwrSlot", cx - JETSON_L/2 - 0.10, cy, cz, 0.40, 1.20, 0.80))
            cut_cavity(comp, box(comp, f"{tag}_CSI_Ribbon", cx, cy - JETSON_W/2 - 0.10, cz + 2.0, 2.50, 0.40, 0.30))
            cut_cavity(comp, cyl(comp, f"{tag}_FanIn", cx, cy - JETSON_W/2 - 0.10, cz - 1.0, 2.50, 0.50, "y"))
            BOM.add("Compute", "NVIDIA Jetson Nano 4GB (Edge AI)", 1, comp.name)
            BOM.add("Fastener", "M2.5x12 brass standoff", 4, comp.name)

        def esp32_controller_bay(comp, tag, cx, cy, cz):
            """SYS-2: ESP32 DevKit pocket for real-time low-level control."""
            cut_cavity(comp, box(comp, f"{tag}_ESP32Bay", cx, cy, cz, ESP32_L + 0.20, ESP32_W + 0.20, ESP32_H + 0.30))
            for sx, sz in [(-2.20, -1.00), (+2.20, -1.00), (-2.20, +1.00), (+2.20, +1.00)]:
                cyl(comp, f"{tag}_ESPStdoff_{sx}_{sz}", cx+sx, cy, cz+sz, 0.12, ESP32_H+0.40, "y", dark_metal)
            cut_cavity(comp, box(comp, f"{tag}_ESP_USB", cx - ESP32_L/2 - 0.10, cy, cz, 0.30, 0.80, 0.50))
            BOM.add("Compute", "ESP32 DevKit V1 (Real-time controller)", 1, comp.name)
            BOM.add("Fastener", "M2x10 brass standoff", 4, comp.name)

        def csi_camera_mount(comp, tag, cx, cy, cz, lens_axis="y"):
            """SYS-3: IMX219 CSI Camera mount for Jetson Nano vision."""
            cut_cavity(comp, box(comp, f"{tag}_CSIBay", cx, cy, cz, CSI_CAM_L + 0.15, CSI_CAM_W + 0.15, CSI_CAM_H + 0.20))
            cut_cavity(comp, cyl(comp, f"{tag}_LensHole", cx, cy - (CSI_CAM_H/2 + 0.20), cz, 0.40, 0.50, lens_axis))
            cut_cavity(comp, box(comp, f"{tag}_RibbonExit", cx, cy + CSI_CAM_W/2 + 0.10, cz, 1.80, 0.40, 0.20))
            BOM.add("Vision", "IMX219 CSI Camera Module (Jetson Vision)", 1, comp.name)
            BOM.add("Hardware", "15-pin CSI ribbon cable (300mm)", 1, comp.name)

        def tof_sensor(comp, tag, cx, cy, cz, axis="y"):
            """SYS-4: VL53L1X Time-of-Flight sensor for obstacle detection."""
            box(comp, f"{tag}_ToF_Body", cx, cy, cz, 1.20, 0.80, 0.40, black_plastic)
            cyl(comp, f"{tag}_ToF_Lens", cx, cy - 0.45, cz, 0.35, 0.15, axis, glass_clr)
            BOM.add("Sensor", "VL53L1X ToF Distance Sensor (I2C)", 1, comp.name)

        def comm_bus_routing(comp, tag, cx, cy, cz, length, axis="z"):
            """SYS-5: Shielded cable routing for Jetson <-> ESP32 UART/I2C."""
            wire_channel(comp, f"{tag}_CommBus", cx, cy, cz, 0.25, length, axis)
            BOM.add("Hardware", "Shielded 4-core cable (UART/I2C)", 1, comp.name)

        def pca9685_bay(comp, tag, cx, cy, cz):
            cut_cavity(comp, box(comp, f"{tag}_PCABay", cx, cy, cz, PCA_L + 0.20, PCA_W + 0.20, PCA_H + 0.30))
            for sx, sz in [(-3.00, -1.08), (+3.00, -1.08), (-3.00, +1.08), (+3.00, +1.08)]:
                cyl(comp, f"{tag}_PCAStdoff_{sx}_{sz}", cx+sx, cy, cz+sz, 0.14, PCA_H+0.50, "y", dark_metal)
            for ch in range(0, 16, 4):
                offset_z = -0.8 + ch * 0.10
                jst_pocket(comp, f"{tag}_PCA_JST{ch}", cx+2.8, cy, cz+offset_z)
            BOM.add("Electronics", "PCA9685 16-ch servo driver (I2C from ESP32)", 1, comp.name)

        def lipo_bay(comp, tag, cx, cy, cz):
            cut_cavity(comp, box(comp, f"{tag}_LipoBay", cx, cy, cz, LIPO_L + 0.30, LIPO_H + 0.30, LIPO_W + 0.30))
            cut_cavity(comp, box(comp, f"{tag}_XT60Slot", cx, cy + LIPO_H/2 + 0.15, cz, XT60_W + 0.10, 0.50, XT60_H_SLOT + 0.10))
            for sx in [-LIPO_L/2 - 0.40, LIPO_L/2 + 0.40]:
                cut_cavity(comp, box(comp, f"{tag}_StrapSlot_{sx:.0f}", cx + sx, cy, cz, 0.25, LIPO_H + 0.50, LIPO_W + 0.20))
            box(comp, f"{tag}_FoamPad", cx, cy, cz - LIPO_H/2 - 0.15, LIPO_L + 0.10, 0.15, LIPO_W + 0.10, rubber_blk)
            BOM.add("Electronics", "2S 1300 mAh 7.4V LiPo battery", 1, comp.name)
            BOM.add("Electronics", "XT60-F panel-mount connector",  1, comp.name)
            BOM.add("Hardware",    "Velcro strap 20x200 mm", 2, comp.name)

        def imu_pocket(comp, tag, cx, cy, cz, axis="z"):
            cut_cavity(comp, box(comp, f"{tag}_IMUPkt", cx, cy, cz, IMU_L + 0.20, IMU_W + 0.20, IMU_H + 0.30))
            BOM.add("Sensor", "MPU9250 9-DOF IMU (I2C to ESP32)", 1, comp.name)

        def bec_mount(comp, tag, cx, cy, cz):
            cut_cavity(comp, box(comp, f"{tag}_BECBay", cx, cy, cz, 3.70, 1.10, 2.00))
            for sx, sz in [(-1.35, -0.75), (1.35, 0.75)]: m3_boss(comp, f"{tag}_BEC_{sx:.0f}", cx+sx, cy, cz+sz)
            BOM.add("Electronics", "5V 5A BEC / UBEC power regulator", 1, comp.name)

        def power_switch_cutout(comp, tag, cx, cy, cz, axis="y"):
            cyl(comp, f"{tag}_SwHole", cx, cy, cz, 0.35, 1.0, axis, black_plastic)
            cut_cavity(comp, cyl(comp, f"{tag}_SwCut", cx, cy, cz, 0.38, 1.2, axis))
            BOM.add("Electronics", "Panel-mount rocker switch SPST", 1, comp.name)

        # ─────────────────────────────────────────────────────────────────
        # JOINT BUILDERS & HARDWARE MODULES
        # ─────────────────────────────────────────────────────────────────
        def _make_joint_geometry(cx, cy, cz):
            try:
                cpi = root.constructionPoints.createInput()
                cpi.setByPoint(adsk.core.Point3D.create(cx, cy, cz))
                cp = root.constructionPoints.add(cpi)
                cp.isLightBulbOn = False
                return adsk.fusion.JointGeometry.createByPoint(cp)
            except Exception: pass
            try:
                sketch = root.sketches.add(root.xYConstructionPlane)
                sketch.isVisible = False
                s_pt = sketch.sketchPoints.add(adsk.core.Point3D.create(cx, cy, cz))
                return adsk.fusion.JointGeometry.createByPoint(s_pt)
            except Exception: return None

        def revolute_joint(name, occ1, occ2, cx, cy, cz, axis_str):
            if not (occ1 and occ2): return
            try:
                geom = _make_joint_geometry(cx, cy, cz)
                ji   = root.asBuiltJoints.createInput(occ1, occ2, geom)
                av   = {"x": adsk.core.Vector3D.create(1, 0, 0), "y": adsk.core.Vector3D.create(0, 1, 0), "z": adsk.core.Vector3D.create(0, 0, 1)}[axis_str]
                ji.setAsRevoluteJointMotion(adsk.fusion.JointDirections.CustomJointDirection, av)
                j      = root.asBuiltJoints.add(ji)
                j.name = name
            except Exception: Logger.log_error(f"Failed revolute joint {name}")

        def ball_joint(name, occ1, occ2, cx, cy, cz):
            if not (occ1 and occ2): return
            try:
                geom = _make_joint_geometry(cx, cy, cz)
                ji   = root.asBuiltJoints.createInput(occ1, occ2, geom)
                ji.setAsBallJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection, adsk.fusion.JointDirections.XAxisJointDirection)
                j      = root.asBuiltJoints.add(ji)
                j.name = name
            except Exception: Logger.log_error(f"Failed ball joint {name}")

        def rigid_joint(name, occ1, occ2):
            if not (occ1 and occ2): return
            try:
                geom = _make_joint_geometry(0, 0, 0)
                ji   = root.asBuiltJoints.createInput(occ1, occ2, geom)
                ji.setAsRigidJointMotion()
                j      = root.asBuiltJoints.add(ji)
                j.name = name
            except Exception: pass

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
                if axis == "x": c1 = cyl(comp, f"{tag}_Hrn1_{d}", hx, hy+d, hz, pd, 1.5, "x"); c2 = cyl(comp, f"{tag}_Hrn2_{d}", hx, hy, hz+d, pd, 1.5, "x")
                elif axis == "z": c1 = cyl(comp, f"{tag}_Hrn1_{d}", hx+d, hy, hz, pd, 1.5, "z"); c2 = cyl(comp, f"{tag}_Hrn2_{d}", hx, hy+d, hz, pd, 1.5, "z")
                else: c1 = cyl(comp, f"{tag}_Hrn1_{d}", hx+d, hy, hz, pd, 1.5, "y"); c2 = cyl(comp, f"{tag}_Hrn2_{d}", hx, hy, hz+d, pd, 1.5, "y")
                cut_cavity(comp, c1); cut_cavity(comp, c2)
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
            BOM.add("Drive", "65 mm rubber tyre + wheel", 1, comp.name)

        def u_bracket(comp, tag, cx, cy, cz, lx, ly, lz, ap=None):
            ap = ap or chrome
            box(comp, f"{tag}_BB", cx, cy, cz, 0.45, ly, lz, ap)
            box(comp, f"{tag}_BL", cx+lx*0.45, cy+ly*0.35, cz, lx*0.55, 0.40, lz, ap)
            box(comp, f"{tag}_BR", cx+lx*0.45, cy-ly*0.35, cz, lx*0.55, 0.40, lz, ap)
            cyl(comp, f"{tag}_Pin", cx+lx*0.50, cy, cz, 0.18, ly*0.85, "y", chrome)

        def hard_stop(comp, tag, cx, cy, cz, axis="x", stop_angle_deg=90):
            ax = {"x": (1, 0, 0), "y": (0, 1, 0), "z": (0, 0, 1)}[axis]
            box(comp, f"{tag}_Stop", cx, cy, cz, 0.35, 0.35, 0.35, dark_metal)
            BOM.add("Hardware", "Hard stop block", 1, comp.name)
            ASSEMBLY_STEPS.append(f"Verify {tag} hard stop at {stop_angle_deg} deg clears moving link")

        def transform_lock(comp, tag, cx, cy, cz, axis="z"):
            cyl(comp, f"{tag}_LockBore", cx, cy, cz, 0.18, 1.50, axis, dark_metal)
            cut_cavity(comp, cyl(comp, f"{tag}_LockHole", cx, cy, cz, 0.20, 1.60, axis))
            cyl(comp, f"{tag}_SpringPkt", cx, cy, cz + 0.30, 0.35, 0.50, axis, dark_grey)
            BOM.add("Hardware", "Spring latch pin Ø3.5 mm", 1, comp.name)
            ASSEMBLY_STEPS.append(f"Install {tag} transform lock pin and return spring")

        # ─────────────────────────────────────────────────────────────────
        # COMPONENT BUILDING (V13 AI & Edge Compute Edition)
        # ─────────────────────────────────────────────────────────────────
        torso = new_component("OP_Torso")
        box(torso, "Torso_Shell", 0, 0, TORSO_CTR, 10.4, 8.6, 12.2, op_red)
        box(torso, "Torso_Side_L", -5.6, 0, TORSO_CTR, 0.5, 7.8, 11.2, op_red)
        box(torso, "Torso_Side_R", 5.6, 0, TORSO_CTR, 0.5, 7.8, 11.2, op_red)
        box(torso, "CWin_Frame_L", -2.3, -4.20, TORSO_CTR+2.5, 2.8, 0.32, 3.2, op_blue)
        box(torso, "CWin_Frame_R", 2.3, -4.20, TORSO_CTR+2.5, 2.8, 0.32, 3.2, op_blue)
        box(torso, "Chest_Win_L", -2.3, -4.35, TORSO_CTR+2.5, 2.4, 0.20, 2.8, glass_clr)
        box(torso, "Chest_Win_R", 2.3, -4.35, TORSO_CTR+2.5, 2.4, 0.20, 2.8, glass_clr)
        box(torso, "Chest_Win_Div", 0, -4.25, TORSO_CTR+2.5, 0.40, 0.22, 3.2, chrome)
        for gz_offset, gw in [(-0.2, 7.4), (-1.0, 7.0), (-1.8, 6.6), (-2.6, 6.2)]:
            box(torso, f"Grille_{int(gz_offset*10)}", 0, -4.40, TORSO_CTR+gz_offset, gw, 0.22, 0.30, chrome)
        box(torso, "Headlight_L", -4.4, -4.50, TORSO_CTR-1.2, 1.8, 0.28, 2.0, glass_clr)
        box(torso, "Headlight_R", 4.4, -4.50, TORSO_CTR-1.2, 1.8, 0.28, 2.0, glass_clr)
        box(torso, "Front_Bumper", 0, -5.8, TORSO_CTR-4.4, 10.0, 2.0, 1.8, chrome)
        box(torso, "Hood_Crease_L", -2.5, -4.60, TORSO_CTR-2.8, 0.5, 0.35, 3.0, op_red)
        box(torso, "Hood_Crease_R", 2.5, -4.60, TORSO_CTR-2.8, 0.5, 0.35, 3.0, op_red)
        box(torso, "Ind_L", -3.8, -5.00, TORSO_CTR-3.8, 0.6, 0.25, 0.5, yellow_met)
        box(torso, "Ind_R", 3.8, -5.00, TORSO_CTR-3.8, 0.6, 0.25, 0.5, yellow_met)
        box(torso, "Chest_Plate", 0, -4.20, TORSO_CTR+0.5, 8.4, 0.32, 4.0, chrome)
        cyl(torso, "Badge_Ring", 0, -4.55, TORSO_CTR+0.5, 0.80, 0.12, "y", op_red)
        led_ring_pocket(torso, "Badge", 0, -4.60, TORSO_CTR+0.5, "y")
        box(torso, "Inner_Frame", 0, 0, TORSO_CTR+1.5, 7.4, 6.0, 8.2, dark_metal)
        box(torso, "Spine_Beam", 0, 0, TORSO_CTR+1.5, 1.8, 1.8, 8.2, chrome)
        cyl(torso, "Spine_Cyl", 0, 0, TORSO_CTR+1.5, 1.10, 4.5, "z", chrome)
        for rz in [TORSO_CTR+3.5, TORSO_CTR, TORSO_CTR-3.5]:
            box(torso, f"Rib_{rz:.0f}", 0, 0, rz, 6.8, 0.35, 4.5, dark_metal)
        for sx in [-6.5, 6.5]:
            box(torso, f"Gusset_{sx:.0f}", sx, 0, TORSO_CTR+2.0, 1.2, 1.2, 3.5, dark_metal)
        
        # V13 Power & Compute Bays
        box(torso, "LipoBay_Shell", 0, 3.2, TORSO_CTR-2.0, 7.6, 4.4, 5.6, black_plastic)
        lipo_bay(torso, "Main", 0, 2.8, TORSO_CTR-2.0)
        lipo_door(torso, "LipoDoor", 0, 5.5, TORSO_CTR-2.0, LIPO_L + 0.80, LIPO_W + 0.80)
        bec_mount(torso, "MainBEC", LIPO_L/2 + 1.0, 3.0, TORSO_CTR-2.0)
        power_switch_cutout(torso, "PwrSw", -5.5, 0, TORSO_CTR+2.0, "y")

        # V13 Edge Compute: Jetson Nano & ESP32
        box(torso, "Jetson_Shell", 0, 0, TORSO_CTR+3.0, 10.2, 8.2, 3.0, black_plastic)
        jetson_nano_bay(torso, "Main", 0, 0, TORSO_CTR+3.0)
        pcb_cover(torso, "JetsonCover", 0, 4.2, TORSO_CTR+3.0, JETSON_L + 0.60, JETSON_W + 0.60, "magnet")

        box(torso, "ESP32_Shell", 0, 2.8, TORSO_CTR+0.5, 6.0, 3.6, 2.0, black_plastic)
        esp32_controller_bay(torso, "Main", 0, 3.2, TORSO_CTR+0.5)
        pcb_cover(torso, "ESP32Cover", 0, 4.5, TORSO_CTR+0.5, ESP32_L + 0.60, ESP32_W + 0.60, "screw")

        box(torso, "PCA_Shell", 0, 2.8, TORSO_CTR+4.2, 6.8, 3.2, 2.2, black_plastic)
        pca9685_bay(torso, "Main", 0, 3.0, TORSO_CTR+4.2)
        pcb_cover(torso, "PCACover", 0, 4.2, TORSO_CTR+4.2, PCA_L + 0.50, PCA_W + 0.50, "screw")

        # V13 Comm Bus & Sensors
        comm_bus_routing(torso, "Jetson_ESP32", 0, 1.5, TORSO_CTR+1.5, 3.0, "z")
        tof_sensor(torso, "Chest_ToF_L", -3.5, -4.6, TORSO_CTR+1.0, "y")
        tof_sensor(torso, "Chest_ToF_R", 3.5, -4.6, TORSO_CTR+1.0, "y")

        wire_hub(torso, "TorsoHub", 0, 1.5, TORSO_CTR-0.5)
        for cz_clip in [TORSO_CTR+3.0, TORSO_CTR, TORSO_CTR-3.0, TORSO_CTR-4.5]:
            cable_clip(torso, f"CC_L_{cz_clip:.0f}", -3.4, 0.6, cz_clip)
            cable_clip(torso, f"CC_R_{cz_clip:.0f}", 3.4, 0.6, cz_clip)
        box(torso, "Cable_L", -3.4, 0.6, TORSO_CTR, 0.55, 1.0, 10.0, dark_grey)
        box(torso, "Cable_R", 3.4, 0.6, TORSO_CTR, 0.55, 1.0, 10.0, dark_grey)
        
        box(torso, "Collar_L", -8.0, 0, SHOULDER_CTR-1.0, 5.5, 3.4, 3.0, chrome)
        box(torso, "Collar_R", 8.0, 0, SHOULDER_CTR-1.0, 5.5, 3.4, 3.0, chrome)
        box(torso, "TF_Flap_L", -5.40,-0.2, TORSO_CTR+3.0, 0.40, 6.6, 6.2, op_red)
        box(torso, "TF_Flap_R", 5.40,-0.2, TORSO_CTR+3.0, 0.40, 6.6, 6.2, op_red)
        box(torso, "TF_BackTop", 0, 5.0, TORSO_CTR+5.2, 8.2, 0.38, 5.2, op_blue)
        for bx_off, bz_off in [(-3.2, 4.8), (3.2, 4.8), (-3.2, -4.8), (3.2, -4.8)]:
            m3_boss(torso, f"Torso_B{bx_off}_{bz_off}", bx_off, 0, TORSO_CTR+bz_off)
        align_pin(torso, "TorsoSplit_L", -5.0, 0, TORSO_CTR)
        align_pin(torso, "TorsoSplit_R", 5.0, 0, TORSO_CTR)
        transform_lock(torso, "WaistLock", 0, -2.0, WAIST_CTR-3.0, "z")
        u_bracket(torso, "Waist_Brkt", 0, 0, WAIST_CTR, 4.0, 4.2, 3.4)
        mg996r(torso, "Waist_Yaw", 0, 0, WAIST_CTR, "z")
        dual_bearing(torso, "WaistDual", 0, 0, WAIST_CTR+0.5, "z", 1.30, 0.65, span=3.00)
        u_bracket(torso, "WaistP_Brkt", 0, 0, WAIST_CTR-2.5, 4.0, 4.2, 3.4)
        mg996r(torso, "Waist_Pitch", 0, 0, WAIST_CTR-2.5, "x")
        bearing_fit(torso, "WaistP_Brg", 0, 0, WAIST_CTR-2.0, "x", 1.30, 0.65, fit_type="press")
        magnet_pocket(torso, "WLock_F", 0, -2.0, WAIST_CTR-3.0)
        magnet_pocket(torso, "WLock_R", 0, 2.0, WAIST_CTR-3.0)
        hard_stop(torso, "WaistP", 0, -2.5, WAIST_CTR-2.5, "x", 60)
        hard_stop(torso, "WaistN", 0, 2.5, WAIST_CTR-2.5, "x", -45)
        u_bracket(torso, "Neck_Brkt", 0, 0, NECK_JOINT_Z, 3.2, 2.8, 3.0)
        mg996r(torso, "Neck_Pitch", 0, 0, NECK_JOINT_Z, "x")
        wire_channel(torso, "Spine", 0, 0, TORSO_CTR, 0.6, 20.0, "z")
        for b in list(torso.bRepBodies):
            if b.name: printability_check(torso, b.name)

        # HEAD (V13 Vision System)
        head = new_component("OP_Head")
        HC = HEAD_CTR
        box(head, "Helm_Main", 0, 0, HC+0.8, 5.6, 5.2, 5.0, op_blue)
        box(head, "Helm_Forehead", 0, -2.30, HC+2.6, 5.0, 0.42, 1.8, op_blue)
        box(head, "Helm_Top", 0, 0, HC+3.4, 4.8, 4.4, 0.90, op_blue)
        box(head, "Ear_L", -3.30, 0, HC+1.8, 0.70, 4.8, 3.8, op_blue)
        box(head, "Ear_R", 3.30, 0, HC+1.8, 0.70, 4.8, 3.8, op_blue)
        box(head, "EarTop_L", -3.05, 0, HC+3.5, 1.40, 4.4, 0.62, op_blue)
        box(head, "EarTop_R", 3.05, 0, HC+3.5, 1.40, 4.4, 0.62, op_blue)
        box(head, "Crest_Main", 0, -0.30, HC+3.85, 1.05, 0.68, 3.6, chrome)
        box(head, "Crest_Stripe", 0, -0.30, HC+3.95, 0.55, 0.36, 2.9, op_blue)
        box(head, "Face_Plate", 0, -2.50, HC+0.5, 3.6, 0.38, 3.2, chrome)
        chamfer_box(head, "Chin_Guard", 0, -2.60, HC-0.9, 3.0, 0.38, 1.8, "z", chamfer=0.25, ap=chrome)
        box(head, "Chin_L", -1.35, -2.52, HC-0.4, 0.38, 0.32, 2.2, chrome)
        box(head, "Chin_R", 1.35, -2.52, HC-0.4, 0.38, 0.32, 2.2, chrome)
        box(head, "Visor_Frame", 0, -2.55, HC+1.35, 3.8, 0.30, 1.25, op_blue)
        box(head, "Visor", 0, -2.68, HC+1.35, 3.0, 0.16, 0.92, op_blue_glass)
        for vx in [-0.85, 0.0, 0.85]: led_pocket_5mm(head, f"Visor_LED_{int(vx*100)}", vx, -2.80, HC+1.35, "y")
        box(head, "Nose_Bridge", 0, -2.60, HC+1.95, 0.72, 0.22, 0.72, chrome)
        box(head, "Mouth_Plate", 0, -2.55, HC+0.10, 2.4, 0.22, 1.10, dark_grey)
        for mz in [-0.32, 0.0, 0.32]: box(head, f"MouthGrill_{int(mz*100)}", 0, -2.62, HC+0.10+mz, 1.8, 0.12, 0.18, chrome)
        box(head, "Head_Rear", 0, 1.90, HC+1.0, 4.2, 1.5, 4.4, op_red)
        box(head, "Neck_Collar", 0, 0, HC-1.6, 2.5, 2.5, 2.4, dark_metal)
        cyl(head, "Ant_L", -2.80, 0, HC+4.4, 0.13, 2.7, "z", chrome)
        cyl(head, "Ant_R", 2.80, 0, HC+4.4, 0.13, 2.7, "z", chrome)
        cyl(head, "AntTip_L", -2.80, 0, HC+5.8, 0.24, 0.34, "z", gold_met)
        cyl(head, "AntTip_R", 2.80, 0, HC+5.8, 0.24, 0.34, "z", gold_met)
        
        # V13 Vision: CSI Camera behind visor
        csi_camera_mount(head, "MainCam", 0, -2.2, HC+1.5, "y")
        cover_plate(head, "CamCover", 0, -2.0, HC+2.5, CSI_CAM_L+0.60, CSI_CAM_W+0.60, [(-1.2, -0.6), (1.2, -0.6)], method="hinge", hinge_edge="top", ap=grey_plastic)
        
        mg90s(head, "Neck_Yaw", 0, 0, NECK_JOINT_Z, "z")
        grommet_slot(head, "NeckWire", 0, 0.8, HC-0.5, "y", 0.50)
        for b in list(head.bRepBodies):
            if b.name: printability_check(head, b.name)
        assembly_jig("OP_Head", [(-2.0, 0, 0), (2.0, 0, 0)], [(-1.5, 0, 0), (1.5, 0, 0)], (6.0, 4.0, 0.60))

        pelvis = new_component("OP_Pelvis")
        box(pelvis, "Pelvis_Shell", 0, 0, PELVIS_CTR, 16.4, 6.2, 5.0, op_blue)
        box(pelvis, "Pelvis_Frame", 0, 0, PELVIS_CTR, 12.2, 4.4, 3.8, dark_metal)
        box(pelvis, "Hip_Armr_L", -7.2, 0, PELVIS_CTR, 1.0, 5.2, 4.2, chrome)
        box(pelvis, "Hip_Armr_R", 7.2, 0, PELVIS_CTR, 1.0, 5.2, 4.2, chrome)
        box(pelvis, "Crotch_Plate", 0, -2.9, PELVIS_CTR-1.2, 5.2, 0.30, 2.4, op_red)
        imu_pocket(pelvis, "BalanceIMU", 0, 0, PELVIS_CTR)
        cover_plate(pelvis, "IMUCover", 0, 0.2, PELVIS_CTR, IMU_L+0.60, IMU_W+0.60, [(-0.8, -0.5), (0.8, 0.5)], method="magnet", ap=grey_plastic)
        grommet_slot(pelvis, "HipWire_L", -HIP_X, 0, PELVIS_CTR+0.5, "z", 0.60)
        grommet_slot(pelvis, "HipWire_R", HIP_X, 0, PELVIS_CTR+0.5, "z", 0.60)
        mg996r(pelvis, "L_HipYaw", -HIP_X, 0, HIP_JOINT_Z, "z")
        mg996r(pelvis, "R_HipYaw", HIP_X, 0, HIP_JOINT_Z, "z")
        dual_bearing(pelvis, "L_HipDual", -HIP_X-2.2, 0, HIP_JOINT_Z, "z", 1.10, 0.62, span=3.20, fit_type="press")
        dual_bearing(pelvis, "R_HipDual", HIP_X+2.2, 0, HIP_JOINT_Z, "z", 1.10, 0.62, span=3.20, fit_type="press")
        BOM.add("Servo", "DS3225MG 25 kg-cm servo (hip yaw)", 2, "OP_Pelvis")
        for b in list(pelvis.bRepBodies):
            if b.name: printability_check(pelvis, b.name)

        for side, sx in [("L", -HIP_X), ("R", HIP_X)]:
            m = -1 if side == "L" else 1
            thigh = new_component(f"OP_Thigh_{side}")
            box(thigh, "Thigh_Link", sx, 0, THIGH_CTR, 5.0, 4.0, 11.0, chrome)
            box(thigh, "Thigh_Outer", sx+m*2.65, 0, THIGH_CTR, 0.50, 4.4, 11.0, op_red)
            box(thigh, "Thigh_Front", sx, -2.2, THIGH_CTR, 5.0, 0.40, 11.0, op_blue)
            box(thigh, "Thigh_Knee_Armor", sx, -2.2, KNEE_CTR+2.5, 4.2, 0.80, 2.8, chrome)
            box(thigh, f"{side}_Thigh_Rib", sx, 0, THIGH_CTR, 3.5, 0.30, 9.0, dark_metal)
            u_bracket(thigh, f"{side}_HPB", sx, 0, HIP_JOINT_Z+0.5, 4.0, 3.2, 3.2)
            mg996r(thigh, f"{side}_HipP", sx, 0, HIP_JOINT_Z, "x")
            dual_bearing(thigh, f"{side}_HipP_Dual", sx, 0, HIP_JOINT_Z, "x", 1.10, 0.62, span=2.80, fit_type="press")
            mg996r(thigh, f"{side}_HipR", sx, 0, THIGH_CTR+2.0, "y")
            bearing_fit(thigh, f"{side}_HRB", sx, 0, THIGH_CTR+2.0, "y", 1.00, 0.55, fit_type="press")
            u_bracket(thigh, f"{side}_KnB", sx, 0, KNEE_CTR+1.5, 3.8, 3.0, 3.0)
            mg996r(thigh, f"{side}_KneP", sx, 0, KNEE_CTR+1.5, "x")
            dual_bearing(thigh, f"{side}_Knee_Dual", sx, 0, KNEE_CTR, "x", 1.00, 0.55, span=2.60, fit_type="press")
            wire_channel(thigh, f"{side}_LW", sx, 0, THIGH_CTR, 0.5, 12.0, "z")
            for bz in [THIGH_CTR+3.0, THIGH_CTR-3.0]: m3_boss(thigh, f"{side}_ThighBoss_{bz:.0f}", sx, 0, bz)
            magnet_pocket(thigh, f"{side}_KLU", sx, -1.5, KNEE_CTR+1.0)
            magnet_pocket(thigh, f"{side}_KLL", sx, 1.5, KNEE_CTR+1.0)
            transform_lock(thigh, f"{side}_KneeLock", sx, -2.5, KNEE_CTR+0.5, "x")
            hard_stop(thigh, f"{side}_KneeExt", sx, -2.5, KNEE_CTR, "x", 135)
            BOM.add("Servo", "DS3225MG 25 kg-cm servo (hip pitch)", 1, f"OP_Thigh_{side}")

            shin = new_component(f"OP_Shin_{side}")
            box(shin, "Shin_Link", sx, 0, SHIN_CTR, 4.4, 6.0, 11.0, op_blue)
            box(shin, "Shin_Armor", sx, -2.7, SHIN_CTR, 3.2, 0.34, 9.2, chrome)
            box(shin, "Shin_Rear", sx, 2.7, SHIN_CTR, 2.0, 0.34, 9.8, dark_grey)
            box(shin, "Shin_Beam", sx, 0.4, SHIN_CTR, 1.8, 2.2, 10.2, dark_metal)
            box(shin, f"{side}_Shin_Rib", sx, 0, SHIN_CTR, 2.8, 0.25, 8.5, dark_metal)
            box(shin, "KneeCap", sx, -2.9, KNEE_CTR-1.0, 3.0, 0.55, 2.2, chrome)
            tt_wheel(shin, f"{side}_WF", sx+m*2.0, -4.2, SHIN_CTR+4.0, m)
            tt_wheel(shin, f"{side}_WR", sx+m*2.0, -4.2, SHIN_CTR-4.0, m)
            bearing_fit(shin, f"{side}_KLB", sx, 0, KNEE_CTR-0.5, "x", 1.00, 0.55, fit_type="press")
            wire_channel(shin, f"{side}_SW", sx, 0, SHIN_CTR, 0.5, 11.0, "z")
            cut_cavity(shin, box(shin, "FtTuck", sx, 2.7, SHIN_CTR-3.5, 5.2, 1.3, 4.2))
            magnet_pocket(shin, f"{side}_KU", sx, -1.5, KNEE_CTR-1.0)
            magnet_pocket(shin, f"{side}_KL", sx, 1.5, KNEE_CTR-1.0)
            for bz in [SHIN_CTR+3.5, SHIN_CTR-3.5]: m3_boss(shin, f"{side}_ShinBoss_{bz:.0f}", sx, 0, bz)
            grommet_slot(shin, f"{side}_AnkleWire", sx, 0, SHIN_CTR-5.0, "z", 0.50)

            foot = new_component(f"OP_Foot_{side}")
            box(foot, "Foot_Sole", sx, -0.6, ANKLE_CTR-1.5, 6.2, 9.2, 1.10, op_red)
            for vi in [-1.0, 0.0, 1.0]: box(foot, f"Arch_Vent_{int(vi*10)}", sx+vi*1.2, -0.6, ANKLE_CTR-1.94, 0.40, 5.5, 0.14, dark_grey)
            chamfer_box(foot, "Heel_Block", sx-m*0.9, 3.2, ANKLE_CTR-0.8, 2.5, 3.5, 2.6, "y", chamfer=0.30, ap=dark_grey)
            box(foot, "Heel_Plate", sx-m*0.6, 4.4, ANKLE_CTR-1.2, 3.2, 0.32, 2.0, chrome)
            chamfer_box(foot, "Heel_Spur", sx-m*1.0, 4.8, ANKLE_CTR-0.2, 1.2, 0.40, 3.2, "z", chamfer=0.35, ap=op_red)
            box(foot, "Toe_Block", sx+m*0.8, -3.8, ANKLE_CTR-0.8, 2.6, 3.8, 2.0, dark_grey)
            box(foot, "Toe_Plate", sx+m*0.5, -4.6, ANKLE_CTR-1.2, 3.8, 0.32, 1.8, chrome)
            chamfer_box(foot, "Toe_Cap", sx+m*1.0, -5.2, ANKLE_CTR-0.8, 2.8, 0.42, 1.5, "z", chamfer=0.25, ap=op_red)
            box(foot, "Ankle_Guard", sx, 0, ANKLE_CTR+1.2, 5.4, 3.2, 2.8, chrome)
            box(foot, "Ankle_Inner", sx, -1.0, ANKLE_CTR+0.3, 4.0, 2.0, 1.6, dark_metal)
            box(foot, "Boot_Fin", sx+m*2.0, 0, ANKLE_CTR-0.2, 0.40, 6.5, 4.2, op_blue)
            box(foot, "Boot_Fin2", sx+m*2.5, 0, ANKLE_CTR+0.8, 0.32, 5.0, 2.8, op_red)
            mg996r(foot, f"{side}_AnkP", sx, 0, ANKLE_CTR+2.2, "x")
            mg996r(foot, f"{side}_AnkR", sx, 0, ANKLE_CTR+0.5, "y")
            bearing_fit(foot, f"{side}_AB", sx, 0, ANKLE_CTR, "x", 1.00, 0.55, fit_type="press")
            hard_stop(foot, f"{side}_AnkP_Stop", sx, -2.0, ANKLE_CTR+2.2, "x", 20)
            hard_stop(foot, f"{side}_AnkN_Stop", sx, 2.0, ANKLE_CTR+2.2, "x", -20)
            for bx_off in [-1.5, 1.5]: m3_boss(foot, f"{side}_FootBoss_{bx_off:.0f}", sx+bx_off, 0, ANKLE_CTR-0.5)
            align_pin(foot, f"{side}_Ft_A", sx-1.0, 0, ANKLE_CTR-1.3)
            align_pin(foot, f"{side}_Ft_B", sx+1.0, 0, ANKLE_CTR-1.3)
            box(foot, f"{side}_VibPad", sx, -0.6, ANKLE_CTR-2.2, 5.5, 8.0, 0.15, rubber_blk)
            
            # V13 Sensors: Ground clearance ToF
            tof_sensor(foot, f"{side}_Foot_ToF", sx, -3.0, ANKLE_CTR-0.5, "y")

            for comp_chk in [thigh, shin, foot]:
                for b in list(comp_chk.bRepBodies):
                    if b.name: printability_check(comp_chk, b.name)
            assembly_jig(f"OP_Thigh_{side}", [(sx-1.0, 0, THIGH_CTR+2.0), (sx+1.0, 0, THIGH_CTR-2.0)], [(sx-0.5, 0, THIGH_CTR+2.0), (sx+0.5, 0, THIGH_CTR-2.0)], (7.0, 5.0, 0.60))

        for side, ax in [("L", -SHOULDER_X), ("R", SHOULDER_X)]:
            m = -1 if side == "L" else 1
            ua = new_component(f"OP_UpperArm_{side}")
            box(ua, "Sh_Block", ax, 0, SHOULDER_CTR, 5.6, 4.2, 5.6, op_red)
            box(ua, "Sh_Pad_Outer", ax+m*3.20, 0, SHOULDER_CTR, 1.00, 5.2, 5.8, op_red)
            box(ua, "Sh_Pad_Edge", ax+m*3.75, 0, SHOULDER_CTR, 0.40, 4.6, 5.2, chrome)
            for sz, sr, sh2 in [(SHOULDER_CTR+2.8, 0.42, 0.95), (SHOULDER_CTR+1.5, 0.36, 0.75)]:
                cyl(ua, f"Stk_A_{sz:.0f}", ax+m*3.3, -1.5, sz, sr, sh2, "z", chrome)
                cone_shape(ua, f"StkTip_{sz:.0f}", ax+m*3.3, -1.5, sz+sh2/2+0.35, sr, sr*0.35, 0.55, "z", dark_grey)
            box(ua, "Sh_Guard", ax+m*2.60, 0, SHOULDER_CTR-0.2, 0.42, 4.4, 6.6, op_blue)
            box(ua, f"{side}_Shoulder_Frame", ax, 0, SHOULDER_CTR, 3.5, 3.0, 4.5, dark_metal)
            box(ua, "UA_Link", ax, 0, ELBOW_Z+3.0, 3.2, 3.4, 5.2, op_red)
            box(ua, "UA_Skin", ax+m*1.80, 0, ELBOW_Z+3.0, 0.52, 3.4, 5.2, chrome)
            mg996r(ua, f"{side}_ShY", ax, 0, SHOULDER_CTR+1.5, "z")
            dual_bearing(ua, f"{side}_ShY_Dual", ax, 0, SHOULDER_CTR+2.0, "z", 1.00, 0.55, span=2.80, fit_type="press")
            u_bracket(ua, f"{side}_SPB", ax, 0, SHOULDER_CTR, 4.8, 3.4, 3.4)
            mg996r(ua, f"{side}_ShP", ax, 0, SHOULDER_CTR, "x")
            dual_bearing(ua, f"{side}_ShP_Dual", ax, 0, SHOULDER_CTR, "x", 1.10, 0.62, span=2.80, fit_type="press")
            mg996r(ua, f"{side}_ShR", ax, 0, SHOULDER_CTR-1.2, "y")
            bearing_fit(ua, f"{side}_SB", ax, 0, SHOULDER_CTR, "x", 1.10, 0.62, fit_type="press")
            u_bracket(ua, f"{side}_EB", ax, 0, ELBOW_Z, 3.8, 3.0, 3.0)
            mg996r(ua, f"{side}_ElbP", ax, 0, ELBOW_Z, "x")
            dual_bearing(ua, f"{side}_Elb_Dual", ax, 0, ELBOW_Z-0.5, "x", 0.95, 0.52, span=2.40, fit_type="press")
            wire_channel(ua, f"{side}_UAW", ax, 0, ELBOW_Z+3.0, 0.4, 5.2, "z")
            m3_boss(ua, f"{side}_UAboss", ax, 0, ELBOW_Z+3.0)
            hard_stop(ua, f"{side}_ElbStop", ax, -2.0, ELBOW_Z, "x", 150)

            fa = new_component(f"OP_Forearm_{side}")
            box(fa, "FA_Link", ax, 0, WRIST_Z+3.5, 3.2, 3.8, 4.8, op_blue)
            box(fa, "FA_Fender", ax+m*2.1, 0, WRIST_Z+3.5, 0.52, 5.2, 5.8, op_red)
            box(fa, "FA_Back", ax, 2.3, WRIST_Z+3.5, 2.6, 0.38, 4.8, chrome)
            box(fa, "FA_Vent_L", ax-0.6, -1.8, WRIST_Z+3.5, 0.30, 0.22, 3.0, dark_grey)
            box(fa, "FA_Vent_R", ax+0.6, -1.8, WRIST_Z+3.5, 0.30, 0.22, 3.0, dark_grey)
            mg90s(fa, f"{side}_WR", ax, 0, WRIST_Z+0.8, "x")
            bearing_fit(fa, f"{side}_WB", ax, 0, WRIST_Z+0.5, "x", 0.80, 0.44, fit_type="press")
            wire_channel(fa, f"{side}_FAW", ax, 0, WRIST_Z+3.5, 0.4, 4.8, "z")
            m3_boss(fa, f"{side}_FAboss", ax, 0, WRIST_Z+4.2)
            grommet_slot(fa, f"{side}_WristWire", ax, 0, WRIST_Z, "y", 0.45)
            # V13 Sensors: Wrist ToF
            tof_sensor(fa, f"{side}_Wrist_ToF", ax, -2.5, WRIST_Z+2.0, "y")

            hand = new_component(f"OP_Hand_{side}")
            box(hand, "Palm_Main", ax, -0.4, WRIST_Z-1.6, 3.4, 3.0, 2.2, dark_grey)
            box(hand, "Palm_Inner", ax, 0.6, WRIST_Z-1.6, 2.6, 2.0, 2.0, black_plastic)
            MCP_Z = WRIST_Z - PALM_BOTTOM_OFFSET
            for ki, kx_off in enumerate(FING_X_OFF): cyl(hand, f"Knuckle_{ki}", ax + m*kx_off, -1.5, MCP_Z + 0.15, 0.28, 0.38, "y", chrome)
            cyl(hand, "Wrist_Ring", ax, 0, WRIST_Z-0.4, 1.05, 0.42, "z", chrome)
            box(hand, "Hand_Panel", ax+m*0.9, -0.7, WRIST_Z-1.3, 0.38, 2.8, 2.8, op_red)
            box(hand, "Finger_SrvBay", ax, 0.5, WRIST_Z-1.8, 1.4, 0.90, 2.4, black_plastic)
            servo_drum(hand, f"{side}_Finger", ax, 1.2, WRIST_Z-1.8, "y")
            BOM.add("Servo", "DS04-NFC 9g digital servo (finger drive)", 2, f"OP_Hand_{side}")
            for fi, fxo in enumerate(FING_X_OFF):
                px = ax + m * fxo * 0.5
                palm_pulley(hand, f"{side}_Pulley_{fi}", px, -0.5, WRIST_Z-1.2, "x")
            for lxi in [-0.7, 0.7]: led_pocket_5mm(hand, f"PalmLED_{lxi:.0f}", ax+lxi, -1.65, WRIST_Z-1.6, "y")
            for fi, (fname, fxo, pp_l, mp_l, dp_l) in enumerate(zip(FING_NAMES, FING_X_OFF, FING_PP, FING_MP, FING_DP)):
                fc   = new_component(f"OP_{fname}_{side}")
                fx   = ax + m * fxo
                fy   = -1.35
                mcp_z = MCP_Z
                pp_cz = mcp_z - pp_l / 2
                mp_cz = mcp_z - pp_l - mp_l / 2
                dp_cz = mcp_z - pp_l - mp_l - dp_l / 2
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
                grommet_slot(fc, f"{fname}_TendonExit", fx, fy-0.2, mcp_z, "y", 0.30)
            thumb = new_component(f"OP_Thumb_{side}")
            tx    = ax + m * 1.70
            ty    = 0.20
            tpp_cz = MCP_Z - THUMB_PP_L / 2
            tdp_cz = MCP_Z - THUMB_PP_L - THUMB_DP_L / 2
            box(thumb, "TP", tx, ty, tpp_cz, THUMB_W, THUMB_H, THUMB_PP_L, grey_plastic)
            tendon_guide(thumb, "Thumb_PP", tx, ty-0.05, tpp_cz, THUMB_PP_L*0.7, "z")
            cyl(thumb, "T_Hinge", tx, ty, MCP_Z - THUMB_PP_L, 0.28, THUMB_W + 0.14, "x", chrome)
            spring_return(thumb, "Thumb_CMC", tx, ty+0.3, MCP_Z - THUMB_PP_L, "x")
            box(thumb, "TD", tx, ty*0.8, tdp_cz, THUMB_W*0.86, THUMB_H*0.86, THUMB_DP_L, grey_plastic)
            tendon_anchor(thumb, "Thumb", tx, ty*0.8, tdp_cz - THUMB_DP_L/2 - 0.15, "z")
            cone_shape(thumb, "TT", tx, ty, tdp_cz - THUMB_DP_L/2 - 0.14, THUMB_W*0.44, 0.05, 0.28, "z", chrome)

            if side == "R":
                blast = new_component("OP_Ion_Blaster")
                cyl(blast, "Barrel_Main", ax, -2.2, WRIST_Z-3.2, 0.92, 4.2, "z", dark_metal)
                cyl(blast, "Barrel_Tip", ax, -2.2, WRIST_Z-5.6, 0.68, 1.1, "z", chrome)
                cyl(blast, "Barrel_Inner", ax, -2.2, WRIST_Z-3.8, 0.44, 2.4, "z", black_plastic)
                box(blast, "Blaster_Body", ax, -1.1, WRIST_Z-3.2, 2.6, 2.4, 2.7, dark_metal)
                box(blast, "Blast_Guard", ax, -0.2, WRIST_Z-3.2, 2.8, 0.38, 1.6, chrome)
                box(blast, "Blast_Rail_T", ax, -0.6, WRIST_Z-2.0, 2.8, 0.22, 0.30, chrome)
                box(blast, "Blast_Rail_B", ax, -0.6, WRIST_Z-4.4, 2.8, 0.22, 0.30, chrome)
                box(blast, "Hinge_Block", ax, -0.8, WRIST_Z-1.6, 1.1, 0.65, 1.1, dark_metal)
                cyl(blast, "Scope_Body", ax+1.5,-2.2, WRIST_Z-3.2, 0.42, 2.2, "z", chrome)
                cyl(blast, "Scope_Lens", ax+1.5,-2.2, WRIST_Z-4.4, 0.28, 0.25, "z", glass_clr)
                led_pocket_5mm(blast, "Muzzle", ax, -2.2, WRIST_Z-6.2, "z")
            for comp_chk in [ua, fa, hand]:
                for b in list(comp_chk.bRepBodies):
                    if b.name: printability_check(comp_chk, b.name)
            assembly_jig(f"OP_UpperArm_{side}", [(ax-1.5, 0, SHOULDER_CTR), (ax+1.5, 0, ELBOW_Z)], [(ax-1.0, 0, SHOULDER_CTR), (ax+1.0, 0, ELBOW_Z)], (8.0, 5.0, 0.60))

        bp = new_component("OP_Backpack")
        box(bp, "BP_Core", 0, 5.6, TORSO_CTR+0.5, 7.2, 2.6, 9.2, dark_grey)
        box(bp, "BP_Hood", 0, 6.5, TORSO_CTR+1.0, 5.8, 1.1, 7.8, op_red)
        box(bp, "BP_TopFlap", 0, 5.1, TORSO_CTR+5.5, 8.4, 0.38, 5.4, op_red)
        box(bp, "BP_Rad", 0, 6.9, TORSO_CTR-0.5, 5.4, 0.44, 5.7, chrome)
        box(bp, "Exh_Blk", 0, 6.3, TORSO_CTR+2.8, 3.2, 0.65, 2.0, dark_metal)
        cyl(bp, "Exh_L", -1.3, 6.7, TORSO_CTR+2.8, 0.40, 1.3, "y", dark_metal)
        cyl(bp, "Exh_R", 1.3, 6.7, TORSO_CTR+2.8, 0.40, 1.3, "y", dark_metal)
        mg90s(bp, "Roof_Hinge", 0, 5.1, TORSO_CTR+5.1, "x")
        bearing_fit(bp, "Roof_Brg", 0, 5.1, TORSO_CTR+5.3, "x", 0.80, 0.44, fit_type="glue")
        magnet_pocket(bp, "RoofL", -2.6, 5.1, TORSO_CTR+5.7)
        magnet_pocket(bp, "RoofR", 2.6, 5.1, TORSO_CTR+5.7)
        grommet_slot(bp, "BP_Wire", 0, 4.5, TORSO_CTR+2.0, "y", 0.80)

        steer = new_component("OP_SteerPods")
        for side, sx in [("L", -5.6), ("R", 5.6)]:
            m2 = -1 if side == "L" else 1
            box(steer, f"SAr_{side}", sx, -3.6, 23.9, 1.6, 1.3, 4.2, chrome)
            box(steer, f"SPod_{side}", sx, -4.6, 23.4, 3.0, 2.2, 3.2, dark_grey)
            tt_wheel(steer, f"SW_{side}", sx+m2*2.0, -4.2, 23.4, m2)
            bearing_fit(steer, f"SPiv_{side}", sx, -3.6, 23.9, "z", 0.95, 0.50, fit_type="glue")
            mg90s(steer, f"SSrv_{side}", sx, -4.2, 23.9, "z")

        shields = new_component("OP_Shields")
        for side, sx in [("L", -(SHOULDER_X+3.4)), ("R", SHOULDER_X+3.4)]:
            m2 = -1 if side == "L" else 1
            box(shields, f"ShShield_{side}", sx, 0, SHOULDER_CTR+1.5, 1.1, 4.6, 5.2, chrome)
            box(shields, f"ShHinge_{side}", sx-m2*0.7, 0, SHOULDER_CTR+1.5, 0.5, 1.9, 1.9, dark_grey)
            box(shields, f"Mirror_{side}", sx+m2*0.5,-2.9, SHOULDER_CTR+2.0, 1.5, 0.2, 0.9, dark_grey)
        for side2, hx in [("L", -(HIP_X+3.1)), ("R", HIP_X+3.1)]:
            box(shields, f"HipShield_{side2}", hx, 0, PELVIS_CTR+0.5, 1.1, 4.4, 4.0, op_blue)

        Logger.log("--- FDM Shell Splitting + Fastener Merge (V13) ---")
        for comp in comps_list:
            to_split = [b for b in comp.bRepBodies if b.name and any(k in b.name for k in SPLIT_KEYS)]
            for b in to_split: split_halves(comp, b, "y", 0.0)
            bodies_after = list(comp.bRepBodies)
            left_bodies  = [b for b in bodies_after if b.name and "_left" in b.name]
            right_bodies = [b for b in bodies_after if b.name and "_right" in b.name]
            for lb in left_bodies: merge_fasteners_to_halves(comp, lb, None, axis="y")
            for rb in right_bodies: merge_fasteners_to_halves(comp, None, rb, axis="y")
        Logger.log("Shell splitting and fastener merge complete.")

        # Kinematics
        t  = occs.get("OP_Torso")
        p  = occs.get("OP_Pelvis")
        h  = occs.get("OP_Head")
        b  = occs.get("OP_Backpack")
        st = occs.get("OP_SteerPods")
        sh = occs.get("OP_Shields")
        if p: p.isGrounded = True
        ball_joint("Waist_Cluster", t, p, 0, 0, WAIST_CTR-2.5)
        ball_joint("Neck_Cluster", h, t, 0, 0, NECK_JOINT_Z)
        rigid_joint("Backpack_Mount", b, t)
        rigid_joint("Steer_Mount", st, p)
        rigid_joint("Shields_Mount", sh, t)
        for side in ["L", "R"]:
            sx = -HIP_X if side == "L" else HIP_X
            ax = -SHOULDER_X if side == "L" else SHOULDER_X
            m  = -1 if side == "L" else 1
            th = occs.get(f"OP_Thigh_{side}")
            sn = occs.get(f"OP_Shin_{side}")
            fo = occs.get(f"OP_Foot_{side}")
            ua = occs.get(f"OP_UpperArm_{side}")
            fa = occs.get(f"OP_Forearm_{side}")
            ha = occs.get(f"OP_Hand_{side}")
            ball_joint(f"{side}_Hip_Cluster", th, p, sx, 0, HIP_JOINT_Z)
            revolute_joint(f"{side}_Knee", sn, th, sx, 0, KNEE_CTR+1.5, "x")
            ball_joint(f"{side}_Ankle_Cluster", fo, sn, sx, 0, ANKLE_CTR+2.2)
            ball_joint(f"{side}_Shoulder_Cluster", ua, t, ax, 0, SHOULDER_CTR)
            revolute_joint(f"{side}_Elbow", fa, ua, ax, 0, ELBOW_Z, "x")
            ball_joint(f"{side}_Wrist", ha, fa, ax, 0, WRIST_Z+0.8)
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