# ─────────────────────────────────────────────────────────────────────────────
# OPTIMUS PRIME G1  v13.0  ── Physically Buildable Prototype Edition
# Fusion 360 Python Script  |  Anthropic MCP-compatible
# ─────────────────────────────────────────────────────────────────────────────
# WHAT'S NEW IN v13 (AI & Vision Architecture Upgrade)
# ──────────────────────────────────────────────
# ELEC-1  jetson_nano_bay() ─ Main high-level AI brain, replacing RPi Zero
#                             Includes heatsink clearance and M3 standoffs
# ELEC-5  jetcam_pocket()   ─ IMX219 CSI camera pocket in the head
# ELEC-7  esp32_bay()       ─ Real-time low-level sub-controller bay
# CAB-5   csi_ribbon_slot() ─ Neck routing for camera CSI ribbon cable
# MECH-6  Thermal Vents     ─ Additional exhaust venting for Jetson cooling
# ARCH    Comm Layer        ─ Physical routing for Jetson(USB) -> ESP32 -> PCA9685
# CODE    Robust Booleans   ─ Enhanced try/except handling on all combine/cut ops
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

WALL_S       = 0.30    # 3.0 mm structural shell wall (min for FDM strength)
WALL_P       = 0.20    # 2.0 mm partition / internal wall
WALL_F       = 0.15    # 1.5 mm snap-fit cantilever arm

BEARING_FIT_TOLERANCE = 0.008   # 0.08 mm diametral interference for press-fit
BEARING_RETAIN_LIP_H  = 0.06    # 0.6 mm shallow retaining lip height
BEARING_RETAIN_LIP_R  = 0.04    # 0.4 mm lip protrusion

TENDON_DIA        = 0.04       # 0.4 mm braided fishing line / Dyneema
TENDON_GUIDE_R    = 0.06       # groove radius
DRUM_R            = 0.35       # servo winch drum radius
DRUM_H            = 0.50       # drum height
PULLEY_R          = 0.20       # palm idler pulley radius
SPRING_OD         = 0.25       # torsion spring outer diameter
SPRING_WIRE       = 0.04       # spring wire diameter

CABLE_CLIP_R      = 0.12       # snap-in wire clip radius
CABLE_CLIP_W      = 0.35       # clip saddle width
JST_XH_L          = 0.55       # JST-XH 3-pin body length
JST_XH_W          = 0.25       # JST-XH body width
JST_XH_H          = 0.18       # JST-XH body height
CSI_RIBBON_W      = 1.60       # 16mm CSI ribbon cable width
CSI_RIBBON_T      = 0.15       # Cable clearance thickness

FUSE_HOLDER_L     = 2.00
FUSE_HOLDER_W     = 0.80
FUSE_HOLDER_H     = 0.75
BEC_L, BEC_W, BEC_H = 3.50, 1.80, 0.90
POWER_SWITCH_R    = 0.35

M3_CLR_R     = 0.160   # M3 clearance bore radius  (Ø3.2 mm)
M3_PILOT_R   = 0.125   # M3 self-tap pilot radius  (Ø2.5 mm)
M3_HEAD_R    = 0.285   # M3 socket-cap head radius
M3_HEAD_H    = 0.300   # M3 socket-cap head height
M3_NUT_CIR   = 0.320   # M3 hex-nut circumscribed radius
M3_NUT_H     = 0.240   # M3 hex-nut thickness
INSERT_R     = 0.235   # M3 heat-set insert OD radius (Ø4.7 mm — tight fit)
INSERT_H     = 0.500   # M3 heat-set insert height
BOSS_R       = 0.350   # Boss cylinder outer radius  (Ø7.0 mm)
ALIGN_PIN_R  = 0.100   # Ø2.0 mm alignment pin radius
GROMMET_R    = 0.175   # Ø3.5 mm servo wire-exit grommet radius

SCREW_REGISTRY = []    

# ── Electronics footprints V13 (cm) ──────────────────────────────────────────
JETSON_L, JETSON_W, JETSON_H = 10.00, 8.00, 3.20  # NVIDIA Jetson Nano + Heatsink clearance
JETCAM_L, JETCAM_W, JETCAM_H =  2.50, 2.40, 0.90  # IMX219 Jetson compatible camera
ESP32_L,  ESP32_W,  ESP32_H  =  5.20, 2.80, 1.40  # ESP32 DevKitC Sub-controller
PCA_L,    PCA_W,    PCA_H    =  6.25, 2.54, 0.18  # PCA9685 servo driver
IMU_L,    IMU_W,    IMU_H    =  2.10, 1.60, 0.12  # MPU-6050 breakout
LIPO_L,   LIPO_W,   LIPO_H   =  7.00, 3.20, 1.80  # 2S 1300 mAh LiPo
XT60_W,   XT60_H_SLOT        =  1.60, 1.30        # XT60-F connector slot
LED_R_5MM                    = 0.260              # Ø5 mm LED pocket radius
LED_R_RING                   = 0.600              # Ø12 mm LED ring pocket outer radius

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
              "Block", "Sole", "Plate", "Bay", "Collar"}
SKIP_TAGS  = {"Marker", "Pivot", "MtA", "MtB", "Axle_Pivot", "Horn",
              "Pin", "_Vis", "Boss", "Insert", "Nut", "Snap"}

ASSEMBLY_STEPS = []
JIG_REGISTRY   = []
PRINT_NOTES    = []    
SUPPORT_WARNINGS = []  

# ═════════════════════════════════════════════════════════════════════════════
# LOGGER
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


# ═════════════════════════════════════════════════════════════════════════════
# BOM TRACKER
# ═════════════════════════════════════════════════════════════════════════════

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

    app = None
    ui  = None

    Logger.log("=" * 60)
    Logger.log("EXECUTION START  v13.0 -- Optimus Prime G1 AI Vision Build")
    Logger.log("V13 Upgrades: NVIDIA Jetson Nano architecture, ESP32 Sub-Controller,")
    Logger.log("              IMX219 JetCam integration, robust boolean operations.")
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
        chrome        = get_ap("Chrome",                       "Steel - Polished")
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
        petg_red      = op_red   or get_ap("PETG Red", "Steel - Painted (Red)")
        petg_blue     = op_blue  or get_ap("PETG Blue", "Steel - Painted (Blue)")
        nylon_white   = white_pla or get_ap("Nylon - White", "Plastic - Glossy (White)")
        pcb_green     = get_ap("Green (Matte)", "Plastic - Matte (Green)")

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

        def chamfer_box(comp, name, cx, cy, cz, lx, ly, lz, axis, chamfer=0.25, ap=None):
            body = box(comp, name, cx, cy, cz, lx, ly, lz, ap)
            if chamfer < 0.15:
                SUPPORT_WARNINGS.append((name, f"chamfer {chamfer}cm may need support"))
            return body

        # ── Robust Boolean cavity cutter ──────────────────────────────────
        def cut_cavity(comp, tool_body, keep_tool=False):
            if not tool_body or not tool_body.isValid:
                return
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
                except Exception as e:
                    Logger.log(f"Combine cut failed on {b.name} with {tool_body.name}: {e}", "WARN")
            if not keep_tool:
                try:
                    tool_body.isLightBulbOn = False
                    if "_Vis" not in tool_body.name:
                        tool_body.name += "_Vis"
                except Exception:
                    pass

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
            except Exception as e:
                Logger.log(f"Split halves failed on {body.name}: {e}", "WARN")

        def merge_fasteners_to_halves(comp, body_left, body_right, axis="y"):
            fastener_tags = {"Pin", "Boss", "Insert", "Nut", "Snap"}
            for b in list(comp.bRepBodies):
                if not b.name or any(skip in b.name for skip in {"_Vis", "Marker", "Pivot", "MtA", "MtB", "Axle_Pivot", "Horn"}):
                    continue
                if not any(tag in b.name for tag in fastener_tags):
                    continue
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
                except Exception:
                    pass

        def printability_check(comp, body_name, overhang_angle_deg=45):
            high_risk = {"spike", "chin", "heel", "vent", "flap", "undercut",
                         "shelf", "hook", "overhang", "lip"}
            reason = None
            lname = body_name.lower()
            if any(r in lname for r in high_risk):
                reason = "geometry likely to need support material"
            if "Shell" in body_name and any(k in body_name for k in {"Shoulder", "Spike"}):
                reason = "shoulder spikes: print with tip up or use tree supports"
            if "Chin" in body_name:
                reason = "chin guard: add 45-degree chamfer or print face-down"
            if "Heel" in body_name:
                reason = "heel spur: use chamfered edges or split for printing"
            if reason:
                SUPPORT_WARNINGS.append((body_name, reason))

        # ─────────────────────────────────────────────────────────────────
        # PHYSICAL FEATURE HELPERS
        # ─────────────────────────────────────────────────────────────────

        def m3_boss(comp, tag, cx, cy, cz, axis="z", depth=0.80, screw_len=1.0):
            boss = cyl(comp, f"{tag}_Boss", cx, cy, cz, BOSS_R, depth, axis, dark_metal)
            boss.name = f"{tag}_Boss"
            cut_cavity(comp, cyl(comp, f"{tag}_Insert", cx, cy, cz, INSERT_R, INSERT_H, axis))
            BOM.add("Fastener", "M3 heat-set insert (Voron M3x5)", 1, f"boss at ({cx:.1f},{cy:.1f},{cz:.1f})")
            SCREW_REGISTRY.append({
                "tag": tag, "comp": comp.name, "type": "boss_insert",
                "shell_t": depth, "boss_depth": INSERT_H,
                "requested_len": screw_len, "axis": axis,
                "cx": cx, "cy": cy, "cz": cz,
            })

        def captive_nut(comp, tag, cx, cy, cz, axis="z", bolt_len=1.0):
            cut_cavity(comp, cyl(comp, f"{tag}_NutTrap", cx, cy, cz, M3_NUT_CIR, M3_NUT_H, axis))
            cut_cavity(comp, cyl(comp, f"{tag}_NutBore", cx, cy, cz, M3_CLR_R, bolt_len, axis))
            BOM.add("Fastener", "M3 hex nut", 1, comp.name)
            BOM.add("Fastener", f"M3x{int(bolt_len*10)} SHCS", 1, comp.name)
            SCREW_REGISTRY.append({
                "tag": tag, "comp": comp.name, "type": "captive_nut",
                "shell_t": bolt_len, "nut_depth": M3_NUT_H,
                "requested_len": bolt_len, "axis": axis,
                "cx": cx, "cy": cy, "cz": cz,
            })

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
            clamp_w = 0.10 if is_std else 0.08
            setscrew_r = M3_PILOT_R if is_std else 0.09
            ax = {"x": (1, 0, 0), "y": (0, 1, 0), "z": (0, 0, 1)}[axis]

            hub = cyl(comp, f"{tag}_CouplerHub", cx, cy, cz, hub_r, hub_h, axis, dark_metal)

            if axis in ("x", "z"):
                cut_cavity(comp, box(comp, f"{tag}_KeySlot",
                    cx + ax[0]*hub_h*0.25, cy, cz + ax[2]*hub_h*0.25,
                    key_d if axis=="z" else hub_h*0.6, key_w, key_d if axis=="x" else hub_h*0.6))
                cut_cavity(comp, box(comp, f"{tag}_ClampSlit",
                    cx + ax[0]*hub_h*0.35, cy, cz + ax[2]*hub_h*0.35,
                    0.06 if axis=="z" else hub_h*0.5, hub_r*2.2, 0.06 if axis=="x" else hub_h*0.5))
            else:
                cut_cavity(comp, box(comp, f"{tag}_KeySlot", cx, cy + ax[1]*hub_h*0.25, cz, key_d, hub_h*0.6, key_d))
                cut_cavity(comp, box(comp, f"{tag}_ClampSlit", cx, cy + ax[1]*hub_h*0.35, cz, hub_r*2.2, hub_h*0.5, 0.06))

            setscrew_axis = "y" if axis in ("x", "z") else "z"
            cut_cavity(comp, cyl(comp, f"{tag}_SetScrew", cx + ax[0]*hub_h*0.15, cy, cz + ax[2]*hub_h*0.15, setscrew_r, hub_r*2.2, setscrew_axis))

            arm_len = hub_r * 1.8
            cyl(comp, f"{tag}_TorqueArm", cx + ax[0] * arm_len, cy + ax[1] * arm_len, cz + ax[2] * arm_len, hub_r * 0.65, hub_h*0.8, axis, dark_metal)

            spec_name = SERVO_SPECS.get(servo_type, SERVO_SPECS["std"])["name"]
            BOM.add("Printed", f"Servo coupler hub ({spec_name})", 1, comp.name)
            BOM.add("Fastener", "M3x4 set screw (cup point)", 1, comp.name)

        def servo_drum(comp, tag, cx, cy, cz, axis="z"):
            cyl(comp, f"{tag}_DrumBarrel", cx, cy, cz, DRUM_R, DRUM_H, axis, dark_metal)
            cyl(comp, f"{tag}_DrumFlangeT", cx, cy, cz + DRUM_H/2 - 0.02, DRUM_R + 0.10, 0.06, axis, dark_metal)
            cyl(comp, f"{tag}_DrumFlangeB", cx, cy, cz - DRUM_H/2 + 0.02, DRUM_R + 0.10, 0.06, axis, dark_metal)
            tie_axis = "x" if axis in ("y", "z") else "z"
            cut_cavity(comp, cyl(comp, f"{tag}_TieHole", cx, cy, cz, 0.06, DRUM_R*2.2, tie_axis))

        def tendon_guide(comp, tag, cx, cy, cz, length, axis="z"):
            cut_cavity(comp, cyl(comp, f"{tag}_TendonGuide", cx, cy, cz, TENDON_GUIDE_R + 0.02, length, axis))

        def tendon_anchor(comp, tag, cx, cy, cz, axis="z"):
            box(comp, f"{tag}_Anchor", cx, cy, cz, 0.35, 0.28, 0.22, dark_metal)
            cut_cavity(comp, box(comp, f"{tag}_CrimpSlot", cx, cy, cz, 0.06, 0.30, 0.14))

        def palm_pulley(comp, tag, cx, cy, cz, axis="x"):
            cyl(comp, f"{tag}_PulleyPost", cx, cy, cz, 0.12, 0.50, axis, chrome)
            pulley_axis = "y" if axis in ("x", "z") else "z"
            cyl(comp, f"{tag}_PulleyWheel", cx, cy, cz, PULLEY_R, 0.14, pulley_axis, grey_plastic)

        def spring_return(comp, tag, cx, cy, cz, axis="x"):
            cut_cavity(comp, cyl(comp, f"{tag}_SpringPkt", cx, cy, cz, SPRING_OD/2 + 0.03, SPRING_WIRE*4, axis))
            peg_axis = "y" if axis in ("x", "z") else "z"
            for sign in [-1, 1]:
                peg_pos = SPRING_OD/2 + 0.06
                px, py, pz = (cx, cy + sign*peg_pos, cz) if peg_axis == "y" else (cx, cy, cz + sign*peg_pos)
                cyl(comp, f"{tag}_SpringPeg_{sign}", px, py, pz, 0.06, 0.20, peg_axis, chrome)

        def screw_hole(comp, cx, cy, cz, axis="y", length=3.0):
            cut_cavity(comp, cyl(comp, "ScrewHole", cx, cy, cz, M3_PILOT_R, length, axis))

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

        def wire_hub(comp, tag, cx, cy, cz):
            box(comp, f"{tag}_HubBlock", cx, cy, cz, 2.0, 1.6, 1.2, dark_grey)
            for dx, dy, dz, lbl in [(-1.0, 0, 0, "L"), (1.0, 0, 0, "R"), (0, -0.8, 0, "F"), (0, 0.8, 0, "B"), (0, 0, -0.6, "D"), (0, 0, 0.6, "U")]:
                wax = "x" if abs(dx)>abs(dy) and abs(dx)>abs(dz) else "y" if abs(dy)>abs(dz) else "z"
                wire_channel(comp, f"{tag}_Hub{lbl}", cx+dx*0.5, cy+dy*0.5, cz+dz*0.5, 0.25, 0.80, wax)

        def grommet_slot(comp, tag, cx, cy, cz, axis="y", width=0.50):
            cut_cavity(comp, cyl(comp, f"{tag}_GromSlot", cx, cy, cz, GROMMET_R, width, axis))
            cut_cavity(comp, cyl(comp, f"{tag}_GromSeat", cx, cy, cz, GROMMET_R + 0.06, 0.10, axis))

        def jst_pocket(comp, tag, cx, cy, cz, axis="y"):
            cut_cavity(comp, box(comp, f"{tag}_JST", cx, cy, cz, JST_XH_L + 0.10, JST_XH_W + 0.10, JST_XH_H + 0.10))

        # ── V13 CAB-5: CSI Ribbon Cable routing ───────────────────────────
        def csi_ribbon_slot(comp, tag, cx, cy, cz, axis="z", length=1.0):
            """CAB-5 — V13 specific: Route for IMX219 CSI camera ribbon cable."""
            ax_map = {"x": (length, CSI_RIBBON_W, CSI_RIBBON_T),
                      "y": (CSI_RIBBON_W, length, CSI_RIBBON_T),
                      "z": (CSI_RIBBON_W, CSI_RIBBON_T, length)}
            lx, ly, lz = ax_map[axis]
            cut_cavity(comp, box(comp, f"{tag}_CSI_Ribbon", cx, cy, cz, lx, ly, lz))

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
                lip_x, lip_y, lip_z = cx, cy, cz
                if axis == "x": lip_x = cx - w/2 + BEARING_RETAIN_LIP_H/2
                elif axis == "y": lip_y = cy - w/2 + BEARING_RETAIN_LIP_H/2
                else: lip_z = cz - w/2 + BEARING_RETAIN_LIP_H/2
                cyl(comp, f"{tag}_Lip", lip_x, lip_y, lip_z, outer_r + BEARING_RETAIN_LIP_R, BEARING_RETAIN_LIP_H, axis, dark_metal)
            BOM.add("Bearing", f"Ø{int(ro*2*10)} mm bearing ({fit_type}-fit)", 1, comp.name)

        def dual_bearing(comp, tag, cx, cy, cz, axis="x", ro=1.10, w=0.60, span=2.50, fit_type="press"):
            ax = {"x": (1,0,0), "y": (0,1,0), "z": (0,0,1)}[axis]
            box(comp, f"{tag}_Carrier", cx, cy, cz, span*ax[0]+1.2, span*ax[1]+1.2, span*ax[2]+1.2, dark_metal)
            p1 = (cx - ax[0]*span/2, cy - ax[1]*span/2, cz - ax[2]*span/2)
            p2 = (cx + ax[0]*span/2, cy + ax[1]*span/2, cz + ax[2]*span/2)
            bearing_fit(comp, f"{tag}_A", p1[0], p1[1], p1[2], axis, ro, w, fit_type)
            bearing_fit(comp, f"{tag}_B", p2[0], p2[1], p2[2], axis, ro, w, fit_type)
            cyl(comp, f"{tag}_Axle", cx, cy, cz, ro*0.55, span + 1.0, axis, chrome)
            BOM.add("Hardware", f"Steel axle Ø{int(ro*0.55*20)} mm x {int((span+1.0)*10)} mm", 1, comp.name)

        def cover_plate(comp, tag, cx, cy, cz, lx, lz, boss_positions, method="screw", hinge_edge=None, ap=None):
            ap = ap or grey_plastic
            ly_cover = 0.25
            cover = box(comp, f"{tag}_Cover", cx, cy, cz, lx, ly_cover, lz, ap)
            if method == "screw":
                for bx, bz in boss_positions:
                    m3_boss(comp, f"{tag}_CvB_{bx:.0f}_{bz:.0f}", cx+bx, cy, cz+bz)
                BOM.add("Fastener", f"M3x8 SHCS (cover {tag})", len(boss_positions), comp.name)
            elif method == "magnet":
                for bx, bz in boss_positions:
                    magnet_pocket(comp, f"{tag}_CvM_{bx:.0f}_{bz:.0f}", cx+bx, cy, cz+bz)
            elif method == "snap":
                snap_clip(comp, f"{tag}_CvSnap", cx, cy, cz, span_x=lx*0.7)
            elif method == "hinge":
                hinge_y = cy - ly_cover/2
                hinge_axis = "x" if hinge_edge in ("left", "right") else "z"
                cyl(comp, f"{tag}_HingeBarrel", cx, hinge_y, cz, 0.12, lx*0.9 if hinge_axis=="x" else lz*0.9, hinge_axis, dark_metal)
                latch_x = cx + (lx/2 - 0.3) if hinge_edge != "right" else cx - (lx/2 - 0.3)
                box(comp, f"{tag}_Latch", latch_x, hinge_y, cz, 0.5, 0.18, 0.4, dark_metal)
                BOM.add("Hardware", "M2x10 hinge pin (steel)", 1, comp.name)
            BOM.add("Printed", f"Cover plate ({method})", 1, comp.name)
            PRINT_NOTES.append((f"{tag}_Cover", "print flat (face down)", False))
            return cover

        def pcb_cover(comp, tag, cx, cy, cz, lx, lz, method="screw"):
            cover_plate(comp, tag, cx, cy, cz, lx, lz,
                        [(-lx/2+0.5, -lz/2+0.5), (lx/2-0.5, -lz/2+0.5),
                         (-lx/2+0.5, lz/2-0.5), (lx/2-0.5, lz/2-0.5)],
                        method=method, ap=grey_plastic)
            for vy in [-lz*0.25, 0, lz*0.25]:
                cut_cavity(comp, box(comp, f"{tag}_VSlot_{vy:.0f}", cx, cy+0.01, cz+vy, lx*0.55, 0.06, 0.25))

        def assembly_jig(comp_name, pin_positions, socket_positions, base_size):
            jig = new_component(f"JIG_{comp_name}")
            lx, ly, lz = base_size
            box(jig, "Jig_Base", 0, 0, 0, lx, ly, lz, nylon_white)
            for i, (px, py, pz) in enumerate(pin_positions):
                cyl(jig, f"JigPin_{i}", px, py, pz + lz/2, ALIGN_PIN_R + 0.02, 0.50, "z", chrome)
            for i, (sx, sy, sz) in enumerate(socket_positions):
                cyl(jig, f"JigSock_{i}", sx, sy, sz + lz/2, ALIGN_PIN_R + 0.025, 0.30, "z", dark_grey)
            JIG_REGISTRY.append((jig.name, comp_name))
            BOM.add("Tooling", f"Assembly jig for {comp_name}", 1, "printed")
            return jig

        def verify_screw_lengths():
            Logger.log("--- V13 SCREW LENGTH VERIFICATION ---")
            issues = 0
            for entry in SCREW_REGISTRY:
                req, tag, comp_name, stype = entry["requested_len"], entry["tag"], entry["comp"], entry["type"]
                if stype == "boss_insert":
                    min_len, max_len = entry["shell_t"] + INSERT_H * 0.6, entry["shell_t"] + INSERT_H + 0.15
                elif stype == "captive_nut":
                    min_len, max_len = entry["shell_t"] + M3_NUT_H, entry["shell_t"] + M3_NUT_H + 0.20
                else:
                    min_len, max_len = req * 0.5, req * 2.0
                if req < min_len or req > max_len:
                    Logger.log(f"  [WARN] {tag} in {comp_name}: M3x{int(req*10)} mismatch.", "WARN")
                    issues += 1
            if issues == 0: Logger.log("  All registered screw lengths OK [PASS]")

        def write_assembly_guide():
            try:
                os.makedirs(os.path.dirname(ASSEMBLY_FILE), exist_ok=True)
                with open(ASSEMBLY_FILE, "w", encoding="utf-8") as f:
                    f.write("=" * 60 + "\n")
                    f.write("  OPTIMUS PRIME G1 v13.0  ASSEMBLY GUIDE\n")
                    f.write("  AI Vision & Jetson Nano Architecture Edition\n")
                    f.write("=" * 60 + "\n\n")

                    f.write("--- ELECTRONICS INSTALL ORDER (V13 UPGRADE) ---\n")
                    elec_order = [
                        "1. Mount NVIDIA Jetson Nano into primary torso bay using 4x M3 screws",
                        "2. Connect Jetson heatsink fan to internal GPIO header",
                        "3. Mount ESP32 DevKitC into lower torso sub-controller bay",
                        "4. Route USB/UART data cable from Jetson Nano to ESP32",
                        "5. Mount IMX219 Camera into head pocket",
                        "6. Route CSI Ribbon Cable down neck through CSI slot to Jetson Nano",
                        "7. Mount PCA9685 boards (x2) adjacent to ESP32",
                        "8. Connect I2C bus from ESP32 -> PCA9685 boards",
                        "9. Install MPU-6050 IMU into pelvis and wire I2C to ESP32",
                        "10. Connect high-current 5V BECs to Jetson (Barrel/GPIO) and Servo Rails",
                        "11. Install main 2S LiPo battery in rear bay",
                    ]
                    for eo in elec_order:
                        f.write(f"  {eo}\n")
                Logger.log(f"Assembly guide written -> {ASSEMBLY_FILE}")
            except Exception as e:
                Logger.log(f"Assembly guide failed: {e}", "WARN")

        # ─────────────────────────────────────────────────────────────────
        # V13 ELECTRONICS BAY HELPERS
        # ─────────────────────────────────────────────────────────────────

        def jetson_nano_bay(comp, tag, cx, cy, cz):
            """ELEC-1 (V13) — NVIDIA Jetson Nano Bay (100x80x32 mm) w/ Cooling and Standoffs."""
            # Main pocket
            cut_cavity(comp, box(comp, f"{tag}_JetsonBay",
                                 cx, cy, cz,
                                 JETSON_L + 0.40, JETSON_W + 0.40, JETSON_H + 0.50))
            
            # Mounting Standoffs (M3 for Jetson Nano carrier board approx 87x58mm)
            for sx, sz in [(-4.35, -2.90), (+4.35, -2.90), (-4.35, +2.90), (+4.35, +2.90)]:
                cyl(comp, f"{tag}_JetsonStdoff_{sx}_{sz}",
                    cx+sx, cy, cz+sz, 0.18, JETSON_H, "y", dark_metal)
                # Screw holes down center of standoffs
                screw_hole(comp, cx+sx, cy, cz+sz, "y", length=0.8)

            # Ports Access (Rear/Bottom IO)
            cut_cavity(comp, box(comp, f"{tag}_IO_Access",
                cx - JETSON_L/2 - 0.20, cy, cz, 0.50, JETSON_W*0.6, JETSON_H*0.8))
            
            # V13 MECH-6: Active cooling exhaust vent (for Jetson Heatsink)
            cut_cavity(comp, box(comp, f"{tag}_ThermalVent",
                cx, cy + JETSON_W/2 + 0.20, cz, JETSON_L*0.5, 0.50, JETSON_H*0.6))
            
            BOM.add("Electronics", "NVIDIA Jetson Nano 4GB", 1, comp.name)
            BOM.add("Fastener", "M3x6 machine screws (Jetson mount)", 4, comp.name)

        def esp32_bay(comp, tag, cx, cy, cz):
            """ELEC-7 (V13) — ESP32 DevKitC sub-controller pocket."""
            cut_cavity(comp, box(comp, f"{tag}_ESP32Bay",
                                 cx, cy, cz,
                                 ESP32_L + 0.20, ESP32_H + 0.30, ESP32_W + 0.20))
            # USB Access Slot
            cut_cavity(comp, box(comp, f"{tag}_ESP_USB",
                cx - ESP32_L/2 - 0.20, cy, cz, 0.40, ESP32_H, 0.60))
            BOM.add("Electronics", "ESP32 DevKitC V4", 1, comp.name)

        def jetcam_pocket(comp, tag, cx, cy, cz, lens_axis="y"):
            """ELEC-5 (V13) — IMX219 Jetson compatible camera pocket (25x24x9mm)."""
            cut_cavity(comp, box(comp, f"{tag}_JetCamBay",
                                 cx, cy, cz,
                                 JETCAM_L + 0.10, JETCAM_H + 0.10, JETCAM_W + 0.10))
            # Lens barrel cutout
            cut_cavity(comp, cyl(comp, f"{tag}_LensHole",
                                 cx, cy - (JETCAM_H/2 + 0.30), cz,
                                 0.40, 0.80, lens_axis))
            # CSI Ribbon Cable exit slot (downwards)
            csi_ribbon_slot(comp, f"{tag}_CSI_Exit", cx, cy, cz - JETCAM_W/2 - 0.2, "z", 0.5)
            BOM.add("Electronics", "IMX219 CSI Camera Module 8MP", 1, comp.name)
            BOM.add("Electronics", "15-pin to 22-pin CSI ribbon cable (30cm)", 1, comp.name)

        def pca9685_bay(comp, tag, cx, cy, cz):
            cut_cavity(comp, box(comp, f"{tag}_PCABay", cx, cy, cz, PCA_L + 0.20, PCA_W + 0.20, PCA_H + 0.30))
            for sx, sz in [(-3.00, -1.08), (+3.00, -1.08), (-3.00, +1.08), (+3.00, +1.08)]:
                cyl(comp, f"{tag}_PCAStdoff_{sx}_{sz}", cx+sx, cy, cz+sz, 0.14, PCA_H+0.50, "y", dark_metal)
            BOM.add("Electronics", "PCA9685 16-ch servo driver", 1, comp.name)

        def imu_pocket(comp, tag, cx, cy, cz, axis="z"):
            cut_cavity(comp, box(comp, f"{tag}_IMUPkt", cx, cy, cz, IMU_L + 0.20, IMU_W + 0.20, IMU_H + 0.30))
            BOM.add("Electronics", "MPU-6050 IMU breakout", 1, comp.name)

        # ─────────────────────────────────────────────────────────────────
        # JOINT BUILDERS
        # ─────────────────────────────────────────────────────────────────

        def _make_joint_geometry(cx, cy, cz):
            try:
                cpi = root.constructionPoints.createInput()
                cpi.setByPoint(adsk.core.Point3D.create(cx, cy, cz))
                cp = root.constructionPoints.add(cpi)
                cp.isLightBulbOn = False
                return adsk.fusion.JointGeometry.createByPoint(cp)
            except Exception:
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
                av   = {"x": adsk.core.Vector3D.create(1, 0, 0), "y": adsk.core.Vector3D.create(0, 1, 0), "z": adsk.core.Vector3D.create(0, 0, 1)}[axis_str]
                ji.setAsRevoluteJointMotion(adsk.fusion.JointDirections.CustomJointDirection, av)
                j    = root.asBuiltJoints.add(ji)
                j.name = name
            except: pass

        def ball_joint(name, occ1, occ2, cx, cy, cz):
            if not (occ1 and occ2): return
            try:
                geom = _make_joint_geometry(cx, cy, cz)
                ji   = root.asBuiltJoints.createInput(occ1, occ2, geom)
                ji.setAsBallJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection, adsk.fusion.JointDirections.XAxisJointDirection)
                j    = root.asBuiltJoints.add(ji)
                j.name = name
            except: pass

        def rigid_joint(name, occ1, occ2):
            if not (occ1 and occ2): return
            try:
                geom = _make_joint_geometry(0, 0, 0)
                ji   = root.asBuiltJoints.createInput(occ1, occ2, geom)
                ji.setAsRigidJointMotion()
                j    = root.asBuiltJoints.add(ji)
                j.name = name
            except: pass

        def mg996r(comp, tag, cx, cy, cz, axis="x"):
            # Simplified for length, adds correct hardware footprint
            box(comp, f"{tag}_Body", cx, cy, cz, 4.05, 2.00, 4.20, grey_plastic)
            marker(comp, f"{tag}_Pivot", cx, cy, cz)
            cut_cavity(comp, box(comp, f"{tag}_CB", cx, cy, cz, 4.05+CLEARANCE, 2.00+CLEARANCE, 4.20+CLEARANCE))
            horn_coupler(comp, tag, cx, cy, cz, axis, servo_type="std")
            BOM.add("Servo", "MG996R 11 kg-cm servo", 1, comp.name)

        def mg90s(comp, tag, cx, cy, cz, axis="x"):
            box(comp, f"{tag}_Body", cx, cy, cz, 2.30, 1.20, 2.30, op_blue)
            marker(comp, f"{tag}_Pivot", cx, cy, cz)
            cut_cavity(comp, box(comp, f"{tag}_CB", cx, cy, cz, 2.30+CLEARANCE, 1.20+CLEARANCE, 2.30+CLEARANCE))
            horn_coupler(comp, tag, cx, cy, cz, axis, servo_type="micro")
            BOM.add("Servo", "MG90S micro servo", 1, comp.name)

        def u_bracket(comp, tag, cx, cy, cz, lx, ly, lz, ap=None):
            ap = ap or chrome
            box(comp, f"{tag}_BB",  cx, cy, cz, 0.45, ly, lz, ap)
            box(comp, f"{tag}_BL",  cx+lx*0.45, cy+ly*0.35, cz, lx*0.55, 0.40, lz, ap)
            box(comp, f"{tag}_BR",  cx+lx*0.45, cy-ly*0.35, cz, lx*0.55, 0.40, lz, ap)
            cyl(comp, f"{tag}_Pin", cx+lx*0.50, cy, cz, 0.18, ly*0.85, "y", chrome)

        def hard_stop(comp, tag, cx, cy, cz, axis="x", stop_angle_deg=90):
            box(comp, f"{tag}_Stop", cx, cy, cz, 0.35, 0.35, 0.35, dark_metal)

        def transform_lock(comp, tag, cx, cy, cz, axis="z"):
            cyl(comp, f"{tag}_LockBore", cx, cy, cz, 0.18, 1.50, axis, dark_metal)
            cut_cavity(comp, cyl(comp, f"{tag}_LockHole", cx, cy, cz, 0.20, 1.60, axis))
            BOM.add("Hardware", "Spring latch pin Ø3.5 mm (steel)", 1, comp.name)

        # ═════════════════════════════════════════════════════════════════
        # COMPONENT BUILDING  (V13 AI Architecture Edition)
        # ═════════════════════════════════════════════════════════════════

        # ─────────────────────────────────────────────────────────────────
        # 1 TORSO (V13: Jetson Nano Integration)
        # ─────────────────────────────────────────────────────────────────
        torso = new_component("OP_Torso")

        # Main structural shell (Expanded slightly for Jetson)
        box(torso, "Torso_Shell",    0,    0,   TORSO_CTR,        11.4, 9.6, 13.2, op_red)
        box(torso, "Torso_Side_L",  -6.1,  0,   TORSO_CTR,         0.5, 8.8, 12.2, op_red)
        box(torso, "Torso_Side_R",   6.1,  0,   TORSO_CTR,         0.5, 8.8, 12.2, op_red)

        # Chest windows
        box(torso, "Chest_Win_L",   -2.3, -4.85, TORSO_CTR+2.5,   2.4, 0.20, 2.8, glass_clr)
        box(torso, "Chest_Win_R",    2.3, -4.85, TORSO_CTR+2.5,   2.4, 0.20, 2.8, glass_clr)

        # Chest Plate + Badge
        box(torso, "Chest_Plate",    0,   -4.70, TORSO_CTR+0.5,   8.4, 0.32, 4.0, chrome)
        led_ring_pocket(torso, "Badge",  0, -5.10, TORSO_CTR+0.5, "y")

        # Internal Skeleton
        box(torso, "Inner_Frame",    0,    0,    TORSO_CTR+1.5,   8.4, 7.0,  9.2, dark_metal)
        box(torso, "Spine_Beam",     0,    0,    TORSO_CTR+1.5,   1.8, 1.8,  8.2, chrome)

        # V13 ARCHITECTURE: Jetson Nano Bay (Upper Torso/Backpack Area)
        box(torso, "Jetson_Shell",   0,    3.0,  TORSO_CTR+2.0,   10.5, 4.0, 8.5, black_plastic)
        jetson_nano_bay(torso, "MainBrain", 0, 3.5, TORSO_CTR+2.0)
        pcb_cover(torso, "JetsonCover", 0, 5.0, TORSO_CTR+2.0, JETSON_L + 0.60, JETSON_W + 0.60, "screw")

        # V13 ARCHITECTURE: ESP32 Sub-controller Bay (Lower Torso)
        box(torso, "ESP32_Shell",    0,    2.8,  TORSO_CTR-3.0,   6.0,  3.0, 3.5, black_plastic)
        esp32_bay(torso, "SubCtrl",  0,    3.0,  TORSO_CTR-3.0)
        
        # V13 ARCHITECTURE: PCA9685 Bay
        pca9685_bay(torso, "PWM",    3.5,  2.8,  TORSO_CTR-3.0)

        # V13 CAB-5: CSI Ribbon routing from neck down to Jetson
        csi_ribbon_slot(torso, "Neck_CSI", 0, 0, TORSO_CTR+5.0, "y", 2.0)

        # Shoulder collars
        box(torso, "Collar_L",      -8.5,  0,    SHOULDER_CTR-1.0, 5.5, 3.4, 3.0, chrome)
        box(torso, "Collar_R",       8.5,  0,    SHOULDER_CTR-1.0, 5.5, 3.4, 3.0, chrome)

        # Waist servo cluster
        u_bracket(torso, "Waist_Brkt",  0, 0, WAIST_CTR,     4.0, 4.2, 3.4)
        mg996r(torso,    "Waist_Yaw",   0, 0, WAIST_CTR,     "z")
        dual_bearing(torso, "WaistDual", 0, 0, WAIST_CTR+0.5, "z", 1.30, 0.65, span=3.00)

        # ─────────────────────────────────────────────────────────────────
        # 2 HEAD (V13: JetCam Vision Integration)
        # ─────────────────────────────────────────────────────────────────
        head = new_component("OP_Head")
        HC   = HEAD_CTR

        box(head, "Helm_Main",      0,     0,    HC+0.8,  5.6, 5.2, 5.0, op_blue)
        box(head, "Face_Plate",     0,    -2.50, HC+0.5,  3.6, 0.38, 3.2, chrome)
        chamfer_box(head, "Chin",   0,    -2.60, HC-0.9,  3.0, 0.38, 1.8, "z", chamfer=0.25, ap=chrome)
        
        # V13 ELEC-5: IMX219 Camera Pocket in Forehead/Crest
        jetcam_pocket(head, "MainCam", 0, -2.1, HC+2.5, "y")
        
        # Visor & LEDs
        box(head, "Visor",          0,    -2.68, HC+1.35, 3.0, 0.16, 0.92, op_blue_glass)
        for vx in [-0.85, 0.85]:
            led_pocket_5mm(head, f"Visor_LED_{int(vx*100)}", vx, -2.80, HC+1.35, "y")

        # Neck yaw servo
        mg90s(head, "Neck_Yaw", 0, 0, NECK_JOINT_Z, "z")
        
        # V13 CAB-5: CSI ribbon cable channel down the neck
        csi_ribbon_slot(head, "CSI_Route", 0, -1.0, HC-0.5, "z", 2.0)

        # ─────────────────────────────────────────────────────────────────
        # 3 PELVIS
        # ─────────────────────────────────────────────────────────────────
        pelvis = new_component("OP_Pelvis")
        box(pelvis, "Pelvis_Shell",  0,    0,  PELVIS_CTR,  16.4, 6.2, 5.0, op_blue)
        box(pelvis, "Hip_Armr_L",  -7.2,  0,  PELVIS_CTR,   1.0, 5.2, 4.2, chrome)
        box(pelvis, "Hip_Armr_R",   7.2,  0,  PELVIS_CTR,   1.0, 5.2, 4.2, chrome)

        imu_pocket(pelvis, "BalanceIMU", 0, 0, PELVIS_CTR)
        
        mg996r(pelvis, "L_HipYaw", -HIP_X, 0, HIP_JOINT_Z, "z")
        mg996r(pelvis, "R_HipYaw",  HIP_X, 0, HIP_JOINT_Z, "z")
        dual_bearing(pelvis, "L_HipDual", -HIP_X-2.2, 0, HIP_JOINT_Z, "z", 1.10, 0.62, span=3.20)
        dual_bearing(pelvis, "R_HipDual",  HIP_X+2.2, 0, HIP_JOINT_Z, "z", 1.10, 0.62, span=3.20)

        # ─────────────────────────────────────────────────────────────────
        # 4 LEGS
        # ─────────────────────────────────────────────────────────────────
        for side, sx in [("L", -HIP_X), ("R", HIP_X)]:
            m = -1 if side == "L" else 1

            thigh = new_component(f"OP_Thigh_{side}")
            box(thigh, "Thigh_Link",         sx,        0,  THIGH_CTR,      5.0, 4.0, 11.0, chrome)
            u_bracket(thigh, f"{side}_HPB",  sx, 0, HIP_JOINT_Z+0.5,        4.0, 3.2, 3.2)
            mg996r(thigh, f"{side}_HipP",    sx, 0, HIP_JOINT_Z,            "x")
            mg996r(thigh, f"{side}_HipR",    sx, 0, THIGH_CTR+2.0,          "y")
            mg996r(thigh, f"{side}_KneP",    sx, 0, KNEE_CTR+1.5,           "x")

            shin = new_component(f"OP_Shin_{side}")
            box(shin, "Shin_Link",    sx,    0,    SHIN_CTR,  4.4, 6.0, 11.0, op_blue)
            box(shin, "KneeCap",      sx,   -2.9,  KNEE_CTR-1.0, 3.0, 0.55, 2.2, chrome)
            
            foot = new_component(f"OP_Foot_{side}")
            box(foot, "Foot_Sole",    sx,       -0.6,  ANKLE_CTR-1.5,  6.2, 9.2, 1.10, op_red)
            chamfer_box(foot, "Toe_Cap", sx+m*1.0, -5.2, ANKLE_CTR-0.8, 2.8, 0.42, 1.5, "z", chamfer=0.25, ap=op_red)
            mg996r(foot, f"{side}_AnkP", sx, 0, ANKLE_CTR+2.2, "x")
            mg996r(foot, f"{side}_AnkR", sx, 0, ANKLE_CTR+0.5, "y")

        # ─────────────────────────────────────────────────────────────────
        # 5 ARMS & HANDS (Tendon Driven)
        # ─────────────────────────────────────────────────────────────────
        for side, ax in [("L", -SHOULDER_X), ("R", SHOULDER_X)]:
            m = -1 if side == "L" else 1

            ua = new_component(f"OP_UpperArm_{side}")
            box(ua, "Sh_Block",        ax,         0, SHOULDER_CTR,      5.6, 4.2, 5.6, op_red)
            box(ua, "UA_Link",         ax,         0, ELBOW_Z+3.0,      3.2, 3.4, 5.2, op_red)
            mg996r(ua, f"{side}_ShY",    ax, 0, SHOULDER_CTR+1.5, "z")
            mg996r(ua, f"{side}_ShP",    ax, 0, SHOULDER_CTR,     "x")
            mg996r(ua, f"{side}_ShR",    ax, 0, SHOULDER_CTR-1.2, "y")
            mg996r(ua, f"{side}_ElbP",   ax, 0, ELBOW_Z,          "x")

            fa = new_component(f"OP_Forearm_{side}")
            box(fa, "FA_Link",    ax,       0,    WRIST_Z+3.5, 3.2, 3.8, 4.8, op_blue)
            mg90s(fa, f"{side}_WR",   ax, 0, WRIST_Z+0.8, "x")

            hand = new_component(f"OP_Hand_{side}")
            box(hand, "Palm_Main",   ax,         -0.4,  WRIST_Z-1.6, 3.4, 3.0, 2.2, dark_grey)
            servo_drum(hand, f"{side}_Finger", ax, 1.2, WRIST_Z-1.8, "y")

            MCP_Z = WRIST_Z - PALM_BOTTOM_OFFSET
            for fi, (fname, fxo, pp_l, mp_l, dp_l) in enumerate(zip(FING_NAMES, FING_X_OFF, FING_PP, FING_MP, FING_DP)):
                fc   = new_component(f"OP_{fname}_{side}")
                fx   = ax + m * fxo
                fy   = -1.35
                pp_cz = MCP_Z - pp_l / 2
                box(fc, "PP", fx, fy, pp_cz, FING_W, FING_H, pp_l, grey_plastic)
                tendon_guide(fc, f"{fname}_PP", fx, fy-0.05, pp_cz, pp_l*0.8, "z")

        # ─────────────────────────────────────────────────────────────────
        # FDM SHELL SPLITTING
        # ─────────────────────────────────────────────────────────────────
        for comp in comps_list:
            to_split = [b for b in comp.bRepBodies if b.name and any(k in b.name for k in SPLIT_KEYS)]
            for b in to_split:
                split_halves(comp, b, "y", 0.0)

        # ─────────────────────────────────────────────────────────────────
        # KINEMATICS
        # ─────────────────────────────────────────────────────────────────
        t  = occs.get("OP_Torso")
        p  = occs.get("OP_Pelvis")
        h  = occs.get("OP_Head")

        if p: p.isGrounded = True

        ball_joint("Waist_Cluster",  t,  p,  0, 0, WAIST_CTR-2.5)
        ball_joint("Neck_Cluster",   h,  t,  0, 0, NECK_JOINT_Z)

        for side in ["L", "R"]:
            sx = -HIP_X      if side == "L" else  HIP_X
            ax = -SHOULDER_X if side == "L" else  SHOULDER_X
            th = occs.get(f"OP_Thigh_{side}")
            sn = occs.get(f"OP_Shin_{side}")
            fo = occs.get(f"OP_Foot_{side}")
            ua = occs.get(f"OP_UpperArm_{side}")
            fa = occs.get(f"OP_Forearm_{side}")
            ha = occs.get(f"OP_Hand_{side}")

            ball_joint(f"{side}_Hip_Cluster",      th, p,  sx, 0, HIP_JOINT_Z)
            revolute_joint(f"{side}_Knee",         sn, th, sx, 0, KNEE_CTR+1.5, "x")
            ball_joint(f"{side}_Ankle_Cluster",    fo, sn, sx, 0, ANKLE_CTR+2.2)
            ball_joint(f"{side}_Shoulder_Cluster", ua, t,  ax, 0, SHOULDER_CTR)
            revolute_joint(f"{side}_Elbow",        fa, ua, ax, 0, ELBOW_Z,      "x")
            ball_joint(f"{side}_Wrist",            ha, fa, ax, 0, WRIST_Z+0.8)

        # ── Documentation & Checks ────────────────────────────────────────
        verify_screw_lengths()
        write_assembly_guide()

        # ═════════════════════════════════════════════════════════════════
        # ARCHIVE & MANIFEST
        # ═════════════════════════════════════════════════════════════════
        try:
            os.makedirs(EXPORT_DIR, exist_ok=True)
            archive_path = os.path.join(EXPORT_DIR, "Optimus_Prime_G1_v13.f3d")
            export_mgr   = design.exportManager
            archive_opts = export_mgr.createFusionArchiveExportOptions(archive_path)
            export_mgr.execute(archive_opts)
            Logger.log(f"Design archived -> {archive_path}")
        except Exception as e:
            Logger.log(f"Archive skipped: {e}", "WARN")

        manifest = {
            "version": "v13.0",
            "architecture": "Jetson Nano + ESP32 Sub-controller",
            "vision": "IMX219 JetCam",
            "bom_lines": BOM._rows
        }
        with open(MANIFEST_FILE, "w", encoding="utf-8") as f:
            json.dump(manifest, f, indent=2)

        BOM.save_csv(BOM_FILE)
        Logger.log("v13 AI Vision Architecture script finished successfully.")

    except Exception:
        Logger.log(f"FATAL ERROR:\n{traceback.format_exc()}", "ERROR")

    finally:
        Logger.flush()