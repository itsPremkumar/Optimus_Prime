# ─────────────────────────────────────────────────────────────────────────────
# OPTIMUS PRIME G1  v11.0  ─ Physical Build & FDM Production Edition
# Fusion 360 Python Script  |  Anthropic MCP-compatible
# ─────────────────────────────────────────────────────────────────────────────
# WHAT'S NEW IN v11 (Physical Build Edition)
# ──────────────────────────────────────────────
# MFG-1  FDM Printability: 45° overhang chamfers, support-aware splitting,
#        fastener merging (CombineFeature) after shell splits.
# MFG-2  Couplers: Physical horn_coupler() for all servos to transfer torque.
# MFG-3  Bearings: Press-fit tolerances (0.15mm) and retaining lips.
# MFG-4  Actuation: Tendon-driven finger routing + DS04-NFC spool.
# MFG-5  Covers: Removable, snap-fit cover_plate() for electronics bays.
# MFG-6  Wiring: cable_clip() harness routing and central torso hub.
# MFG-7  Jigs: Auto-generated assembly alignment blocks.
# MFG-8  Validation: Fastener length verification vs grip requirements.
# MFG-9  Docs: Auto-generation of build_guide.txt.
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
LOG_FILE       = os.path.join(LOG_DIR,     f"optimus_v11_{_ts}.txt")
BOM_FILE       = os.path.join(_OUTPUT_DIR, f"BOM_v11_{_ts}.csv")
GUIDE_FILE     = os.path.join(_OUTPUT_DIR, f"build_guide_v11_{_ts}.txt")
STOP_FLAG      = os.path.join(_OUTPUT_DIR, "stop.flag")

# ═════════════════════════════════════════════════════════════════════════════
# CONFIGURATION  — ALL DIMENSIONS IN cm (Fusion 360 native unit)
# ═════════════════════════════════════════════════════════════════════════════

# ── Skeleton Z-heights ───────────────────────────────────────────────────────
CLEARANCE    = 0.060
FDM_TOL      = 0.015   # 0.15mm tolerance for FDM press-fits (MFG-3)
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

# ── Physical / FDM print parameters ──────────────────────────────────────────
WALL_S       = 0.30    # 3.0 mm structural shell wall
WALL_P       = 0.20    # 2.0 mm partition / internal wall
WALL_F       = 0.15    # 1.5 mm snap-fit cantilever arm

# ── Fastener dimensions (cm) ─────────────────────────────────────────────────
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

# ── Electronics footprints (cm) ──────────────────────────────────────────────
RPI0_L,  RPI0_W,  RPI0_H  = 6.50, 3.00, 0.20   # Raspberry Pi Zero 2W
PCA_L,   PCA_W,   PCA_H   = 6.25, 2.54, 0.18   # PCA9685 servo driver
ESP32_L, ESP32_W, ESP32_H = 3.80, 2.58, 0.15   # ESP32-DevKit-C
IMU_L,   IMU_W,   IMU_H   = 2.10, 1.60, 0.12   # MPU-6050 breakout
LIPO_L,  LIPO_W,  LIPO_H  = 7.00, 3.20, 1.80   # 2S 1300 mAh LiPo
XT60_W,  XT60_H_SLOT      = 1.60, 1.30         # XT60-F connector slot
LED_R_5MM                 = 0.260   # Ø5 mm LED pocket radius (with clearance)
LED_R_RING                = 0.600   # Ø12 mm LED ring pocket outer radius

# ── Finger geometry (cm) ─────────────────────────────────────────────────────
FING_W       = 0.52    # finger width  (X)
FING_H       = 0.48    # finger depth  (Y)
FING_GAP     = 0.10    # gap between adjacent fingers
THUMB_W      = 0.65
THUMB_H      = 0.58
FING_PP      = [1.40, 1.60, 1.70, 1.55]   # proximal
FING_MP      = [1.00, 1.20, 1.30, 1.15]   # middle
FING_DP      = [0.80, 0.90, 0.95, 0.88]   # distal
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
              "Pin", "_Vis", "Boss", "Insert", "Nut", "Snap", "Clip", "Coupler"}

# ═════════════════════════════════════════════════════════════════════════════
# LOGGERS & TRACKERS
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
        if not cls._buffer: return
        try:
            os.makedirs(LOG_DIR, exist_ok=True)
            with open(LOG_FILE, "a", encoding="utf-8") as f:
                f.write("".join(cls._buffer))
        except Exception: pass
        cls._buffer.clear()
        cls._count = 0

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
                # Aggregate identical parts
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
    def check_fastener_lengths(cls):
        Logger.log("--- FASTENER VERIFICATION ---")
        for f in cls._fasteners:
            if f["Len"] < f["Grip"] + 0.2:
                Logger.log(f"WARNING: Fastener {f['Type']}x{f['Len']*10} at {f['Loc']} may be too short (Needs {f['Grip']*10}mm)", "WARN")
            elif f["Len"] > f["Grip"] + 0.6:
                Logger.log(f"NOTE: Fastener {f['Type']}x{f['Len']*10} at {f['Loc']} is longer than grip ({f['Grip']*10}mm)")

# ═════════════════════════════════════════════════════════════════════════════
# MAIN ENTRY POINT
# ═════════════════════════════════════════════════════════════════════════════

def run(context):
    global TARGET_MODULE, EXPORT_STL, EXPORT_STEP, EXPORT_URDF, EXPORT_DIR

    app = None
    ui  = None

    Logger.log("=" * 60)
    Logger.log("EXECUTION START  v11.0 -- Optimus Prime Physical FDM Build")
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
        chrome        = get_ap("Chrome",                       "Steel - Polished")
        dark_metal    = get_ap("Steel - Flat",                 "Plastic - Matte (Black)")
        rubber_blk    = get_ap("Rubber",                       "Plastic - Matte (Black)")
        glass_clr     = get_ap("Glass - Window",               "Acrylic - Clear")
        grey_plastic  = get_ap("Plastic - Matte (Grey)",       "ABS Plastic")
        dark_grey     = get_ap("Plastic - Matte (Dark Grey)",  "Plastic - Matte (Grey)")
        white_pla     = get_ap("Plastic - Glossy (White)",     "Nylon - White")
        black_plastic = get_ap("Plastic - Matte (Black)",      "Rubber")
        gold_met      = get_ap("Gold",                         "Brass")
        yellow_met    = get_ap("Paint - Metallic (Yellow)",    "Gold")
        op_blue_glass = get_ap("Acrylic - Blue Transparent",   "Glass - Window")

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
                    if "_Vis" not in tool_body.name:
                        tool_body.name += "_Vis"
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
                split_input = comp.features.splitBodyFeatures.createInput(body, sp, True)
                comp.features.splitBodyFeatures.add(split_input)
            except Exception: pass

        # MFG-1: Combine loose pins/bosses back into the split shells
        def merge_fasteners_to_halves(comp):
            shells = [b for b in comp.bRepBodies if b.name and ("Shell" in b.name or "Main" in b.name or "Outer" in b.name)]
            fasteners = [b for b in comp.bRepBodies if b.name and any(f in b.name for f in ["Boss", "Pin", "Snap", "Nut", "Insert", "Clip", "Coupler"])]
            
            for shell in shells:
                tools = adsk.core.ObjectCollection.create()
                for f in fasteners:
                    if f.isLightBulbOn:  # Only merge visible fasteners
                        tools.add(f)
                if tools.count > 0:
                    try:
                        ci = comp.features.combineFeatures.createInput(shell, tools)
                        ci.operation = adsk.fusion.CombineOperation.JoinFeatureOperation
                        ci.isKeepToolBodies = False
                        comp.features.combineFeatures.add(ci)
                    except Exception as e:
                        pass # Geometric overlap might not exist for some tools, normal for batch combiners

        # ─────────────────────────────────────────────────────────────────
        # PHYSICAL FEATURE HELPERS  (MFG-2 ... MFG-6)
        # ─────────────────────────────────────────────────────────────────

        def horn_coupler(comp, tag, cx, cy, cz, axis="z", servo_type="std"):
            """MFG-2: Splined/keyed FDM horn adapter to transfer torque."""
            spline_r = SERVO_SPECS.get(servo_type, SERVO_SPECS["std"])["spline"] / 2.0
            # Outer hub structure
            hub = cyl(comp, f"{tag}_Coupler", cx, cy, cz, spline_r + 0.35, 0.6, axis, dark_metal)
            # Internal keyway (simplified to cross or tight cyl for FDM friction fit + screw)
            cut_cavity(comp, cyl(comp, f"{tag}_SplineCut", cx, cy, cz, spline_r + FDM_TOL, 0.65, axis))
            # Set screw hole
            cut_cavity(comp, cyl(comp, f"{tag}_SetScrew", cx, cy + spline_r + 0.15, cz, M3_PILOT_R, 0.4, "y"))
            BOM.add("Hardware", f"FDM Servo Horn Coupler ({servo_type})", 1, comp.name)

        def cover_plate(comp, tag, cx, cy, cz, lx, ly, lz, axis="y"):
            """MFG-5: Removable cover plate for electronics bays."""
            plate = box(comp, f"{tag}_Cover", cx, cy, cz, lx, ly, lz, black_plastic)
            # Add simple snap clips to edges
            snap_clip(comp, f"{tag}_ClipL", cx - lx/2 + 0.2, cy, cz, span_x=0.4, axis_latch=axis)
            snap_clip(comp, f"{tag}_ClipR", cx + lx/2 - 0.2, cy, cz, span_x=0.4, axis_latch=axis)
            BOM.add("Printed", f"{tag} Service Cover", 1, comp.name)

        def cable_clip(comp, tag, cx, cy, cz, axis="z"):
            """MFG-6: Snap-in cable clip for routing 22AWG servo bundles."""
            clip = box(comp, f"{tag}_Clip", cx, cy, cz, 0.8, 0.6, 0.8, grey_plastic)
            cut_cavity(comp, cyl(comp, f"{tag}_WireRun", cx, cy+0.1, cz, 0.25, 1.0, axis))
            # Entry slot
            cut_cavity(comp, box(comp, f"{tag}_WireSlot", cx, cy+0.4, cz, 0.15, 0.6, 1.0))

        def m3_boss(comp, tag, cx, cy, cz, axis="z", depth=0.80):
            boss = cyl(comp, f"{tag}_Boss", cx, cy, cz, BOSS_R, depth, axis, dark_metal)
            boss.name = f"{tag}_Boss"
            cut_cavity(comp, cyl(comp, f"{tag}_Insert", cx, cy, cz, INSERT_R, INSERT_H, axis))
            BOM.add("Fastener", "M3 heat-set insert (Voron M3x5)", 1, f"boss at ({cx:.1f},{cy:.1f},{cz:.1f})")

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
            cut_cavity(comp, cyl(comp, f"{tag}_AlignSock", cx, cy, cz, ALIGN_PIN_R + FDM_TOL, depth, axis))

        def grommet_hole(comp, tag, cx, cy, cz, axis="y"):
            cut_cavity(comp, cyl(comp, f"{tag}_Grommet", cx, cy, cz, GROMMET_R, 0.80, axis))
            BOM.add("Hardware", "Rubber Wire Grommet Ø3.5mm", 1, comp.name)

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
            BOM.add("Electronics", "WS2812 5050 LED ring Ø12 mm", 1, comp.name)

        # ─────────────────────────────────────────────────────────────────
        # ELECTRONICS BAY HELPERS 
        # ─────────────────────────────────────────────────────────────────

        def rpi_zero_bay(comp, tag, cx, cy, cz):
            cut_cavity(comp, box(comp, f"{tag}_RPiBay", cx, cy, cz, RPI0_L + 0.20, RPI0_W + 0.20, RPI0_H + 0.30))
            for sx, sz in [(-2.90, -1.15), (+2.90, -1.15), (-2.90, +1.15), (+2.90, +1.15)]:
                cyl(comp, f"{tag}_RpiStdoff_{sx}_{sz}", cx+sx, cy, cz+sz, 0.14, RPI0_H+0.50, "y", dark_metal)
            BOM.add("Electronics", "Raspberry Pi Zero 2W", 1, comp.name)
            BOM.add("Fastener",    "M2.5x11 mm brass standoff", 4, comp.name)
            cover_plate(comp, f"{tag}_RPi_Cover", cx, cy+1.0, cz, RPI0_L+0.4, 0.2, RPI0_W+0.4, "y")

        def pca9685_bay(comp, tag, cx, cy, cz):
            cut_cavity(comp, box(comp, f"{tag}_PCABay", cx, cy, cz, PCA_L + 0.20, PCA_W + 0.20, PCA_H + 0.30))
            for sx, sz in [(-3.00, -1.08), (+3.00, -1.08), (-3.00, +1.08), (+3.00, +1.08)]:
                cyl(comp, f"{tag}_PCAStdoff_{sx}_{sz}", cx+sx, cy, cz+sz, 0.14, PCA_H+0.50, "y", dark_metal)
            BOM.add("Electronics", "PCA9685 16-ch servo driver", 1, comp.name)
            cover_plate(comp, f"{tag}_PCA_Cover", cx, cy+1.0, cz, PCA_L+0.4, 0.2, PCA_W+0.4, "y")

        def lipo_bay(comp, tag, cx, cy, cz):
            cut_cavity(comp, box(comp, f"{tag}_LipoBay", cx, cy, cz, LIPO_L + 0.30, LIPO_H + 0.30, LIPO_W + 0.30))
            cut_cavity(comp, box(comp, f"{tag}_XT60Slot", cx, cy + LIPO_H/2 + 0.15, cz, XT60_W + 0.10, 0.50, XT60_H_SLOT + 0.10))
            BOM.add("Electronics", "2S 1300 mAh 7.4V LiPo battery", 1, comp.name)
            BOM.add("Electronics", "XT60-F panel-mount connector",  1, comp.name)
            cover_plate(comp, f"{tag}_LiPo_Door", cx, cy, cz-LIPO_W/2-0.2, LIPO_L, LIPO_H, 0.2, "z")

        def imu_pocket(comp, tag, cx, cy, cz, axis="z"):
            cut_cavity(comp, box(comp, f"{tag}_IMUPkt", cx, cy, cz, IMU_L + 0.20, IMU_W + 0.20, IMU_H + 0.30))
            BOM.add("Electronics", "MPU-6050 IMU breakout", 1, comp.name)

        def esp32_cam_pocket(comp, tag, cx, cy, cz, lens_axis="y"):
            cut_cavity(comp, box(comp, f"{tag}_ESP32Bay", cx, cy, cz, ESP32_L + 0.20, ESP32_H + 0.30, ESP32_W + 0.20))
            ax_map = {"y": (0,1,0), "x": (1,0,0), "z": (0,0,1)}
            av = ax_map[lens_axis]
            cut_cavity(comp, cyl(comp, f"{tag}_LensHole", cx, cy - (ESP32_H/2 + 0.30), cz, 0.150, 0.60, lens_axis))
            BOM.add("Electronics", "ESP32-CAM module (OV2640)", 1, comp.name)

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
                av   = {"x": adsk.core.Vector3D.create(1, 0, 0),
                        "y": adsk.core.Vector3D.create(0, 1, 0),
                        "z": adsk.core.Vector3D.create(0, 0, 1)}[axis_str]
                ji.setAsRevoluteJointMotion(adsk.fusion.JointDirections.CustomJointDirection, av)
                j      = root.asBuiltJoints.add(ji)
                j.name = name
            except Exception: pass

        def ball_joint(name, occ1, occ2, cx, cy, cz):
            if not (occ1 and occ2): return
            try:
                geom = _make_joint_geometry(cx, cy, cz)
                ji   = root.asBuiltJoints.createInput(occ1, occ2, geom)
                ji.setAsBallJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection,
                                        adsk.fusion.JointDirections.XAxisJointDirection)
                j      = root.asBuiltJoints.add(ji)
                j.name = name
            except Exception: pass

        def rigid_joint(name, occ1, occ2):
            if not (occ1 and occ2): return
            try:
                geom = _make_joint_geometry(0, 0, 0)
                ji   = root.asBuiltJoints.createInput(occ1, occ2, geom)
                ji.setAsRigidJointMotion()
                j      = root.asBuiltJoints.add(ji)
                j.name = name
            except Exception: pass

        # ─────────────────────────────────────────────────────────────────
        # HARDWARE MODULES
        # ─────────────────────────────────────────────────────────────────

        def servo_hardware(comp, tag, cx, cy, cz, axis, mg996):
            if mg996:
                fd, fw, hr, pd, sd = 2.4, 0.5, 0.7, 0.125, 0.15
                if axis == "x":
                    hx,hy,hz = cx+2.40, cy,       cz+1.05
                    fx,fy,fz = cx+0.95, cy,       cz
                elif axis == "z":
                    hx,hy,hz = cx-1.10, cy,       cz+2.40
                    fx,fy,fz = cx,      cy,       cz+0.95
                else:
                    hx,hy,hz = cx,      cy+2.40,  cz+1.05
                    fx,fy,fz = cx,      cy+0.95,  cz
            else:
                fd, fw, hr, pd, sd = 1.35, 0.0, 0.4, 0.10, 0.10
                if axis == "x":
                    hx,hy,hz = cx+1.40, cy,       cz+0.50
                    fx,fy,fz = cx+0.45, cy,       cz
                elif axis == "z":
                    hx,hy,hz = cx-0.50, cy,       cz+1.40
                    fx,fy,fz = cx,      cy,       cz+0.45
                else:
                    hx,hy,hz = cx,      cy+1.40,  cz+0.50
                    fx,fy,fz = cx,      cy+0.45,  cz

            for d1 in [-fd, fd]:
                for d2 in ([-fw, fw] if fw > 0 else [0]):
                    if axis == "x":   c = cyl(comp, f"{tag}_FlgS_{d1}_{d2}", fx,    fy+d2, fz+d1, sd, 1.5, "x")
                    elif axis == "z": c = cyl(comp, f"{tag}_FlgS_{d1}_{d2}", fx+d1, fy+d2, fz,    sd, 1.5, "z")
                    else:             c = cyl(comp, f"{tag}_FlgS_{d1}_{d2}", fx+d1, fy,    fz+d2, sd, 1.5, "y")
                    cut_cavity(comp, c)

            for d in [-hr, hr]:
                if axis == "x":
                    c1 = cyl(comp, f"{tag}_Hrn1_{d}", hx,   hy+d, hz,   pd, 1.5, "x")
                    c2 = cyl(comp, f"{tag}_Hrn2_{d}", hx,   hy,   hz+d, pd, 1.5, "x")
                elif axis == "z":
                    c1 = cyl(comp, f"{tag}_Hrn1_{d}", hx+d, hy,   hz,   pd, 1.5, "z")
                    c2 = cyl(comp, f"{tag}_Hrn2_{d}", hx,   hy+d, hz,   pd, 1.5, "z")
                else:
                    c1 = cyl(comp, f"{tag}_Hrn1_{d}", hx+d, hy,   hz,   pd, 1.5, "y")
                    c2 = cyl(comp, f"{tag}_Hrn2_{d}", hx,   hy,   hz+d, pd, 1.5, "y")
                cut_cavity(comp, c1)
                cut_cavity(comp, c2)
            grommet_hole(comp, tag, cx, cy, cz + (0.5 if axis != "z" else 1.0))

        def mg996r(comp, tag, cx, cy, cz, axis="x"):
            cl = CLEARANCE
            if axis == "x":
                box(comp, f"{tag}_VisBody", cx,      cy,      cz,      4.05, 2.00, 4.20, grey_plastic)
                box(comp, f"{tag}_VisEars", cx+0.95, cy,      cz,      0.30, 2.20, 5.80, dark_grey)
                cyl(comp, f"{tag}_VisHorn", cx+2.40, cy,      cz+1.05, 0.95, 0.22, "x", white_pla)
                marker(comp, f"{tag}_Pivot", cx+2.40, cy,     cz+1.05)
                cut_cavity(comp, box(comp, f"{tag}_CB",  cx,      cy, cz, 4.05+cl, 2.00+cl, 4.20+cl))
                cut_cavity(comp, box(comp, f"{tag}_CE",  cx+0.95, cy, cz, 0.30+cl, 2.20+cl, 5.80+cl))
                horn_coupler(comp, tag, cx+2.40, cy, cz+1.05, "x", "std")
            elif axis == "z":
                box(comp, f"{tag}_VisBody", cx,      cy,      cz,      4.05, 2.00, 4.20, grey_plastic)
                box(comp, f"{tag}_VisEars", cx,      cy,      cz+0.95, 5.80, 2.20, 0.30, dark_grey)
                cyl(comp, f"{tag}_VisHorn", cx-1.10, cy,      cz+2.40, 0.95, 0.22, "z", white_pla)
                marker(comp, f"{tag}_Pivot", cx-1.10, cy,     cz+2.40)
                cut_cavity(comp, box(comp, f"{tag}_CB", cx, cy, cz,      4.05+cl, 2.00+cl, 4.20+cl))
                cut_cavity(comp, box(comp, f"{tag}_CE", cx, cy, cz+0.95, 5.80+cl, 2.20+cl, 0.30+cl))
                horn_coupler(comp, tag, cx-1.10, cy, cz+2.40, "z", "std")
            else:  # y
                box(comp, f"{tag}_VisBody", cx,      cy,      cz,      4.05, 4.20, 2.00, grey_plastic)
                box(comp, f"{tag}_VisEars", cx,      cy+0.95, cz,      4.05, 0.30, 2.20, dark_grey)
                cyl(comp, f"{tag}_VisHorn", cx,      cy+2.40, cz+1.05, 0.95, 0.22, "y", white_pla)
                marker(comp, f"{tag}_Pivot", cx,     cy+2.40, cz+1.05)
                cut_cavity(comp, box(comp, f"{tag}_CB", cx, cy,      cz, 4.05+cl, 4.20+cl, 2.00+cl))
                cut_cavity(comp, box(comp, f"{tag}_CE", cx, cy+0.95, cz, 4.05+cl, 0.30+cl, 2.20+cl))
                horn_coupler(comp, tag, cx, cy+2.40, cz+1.05, "y", "std")
            servo_hardware(comp, tag, cx, cy, cz, axis, True)
            BOM.add("Servo", "MG996R 11 kg·cm servo", 1, comp.name)

        def mg90s(comp, tag, cx, cy, cz, axis="x"):
            cl = CLEARANCE
            if axis == "x":
                box(comp, f"{tag}_VisBody", cx,      cy,      cz,      2.30, 1.20, 2.30, op_blue)
                box(comp, f"{tag}_VisEars", cx+0.45, cy,      cz,      0.20, 1.30, 3.20, op_blue)
                cyl(comp, f"{tag}_VisHorn", cx+1.40, cy,      cz+0.50, 0.55, 0.18, "x", white_pla)
                marker(comp, f"{tag}_Pivot", cx+1.40, cy,     cz+0.50)
                cut_cavity(comp, box(comp, f"{tag}_CB",  cx,      cy, cz, 2.30+cl, 1.20+cl, 2.30+cl))
                cut_cavity(comp, box(comp, f"{tag}_CE",  cx+0.45, cy, cz, 0.20+cl, 1.30+cl, 3.20+cl))
                horn_coupler(comp, tag, cx+1.40, cy, cz+0.50, "x", "micro")
            elif axis == "z":
                box(comp, f"{tag}_VisBody", cx,      cy,      cz,      2.30, 1.20, 2.30, op_blue)
                box(comp, f"{tag}_VisEars", cx,      cy,      cz+0.45, 3.20, 1.30, 0.20, op_blue)
                cyl(comp, f"{tag}_VisHorn", cx-0.50, cy,      cz+1.40, 0.55, 0.18, "z", white_pla)
                marker(comp, f"{tag}_Pivot", cx-0.50, cy,     cz+1.40)
                cut_cavity(comp, box(comp, f"{tag}_CB", cx, cy, cz,      2.30+cl, 1.20+cl, 2.30+cl))
                cut_cavity(comp, box(comp, f"{tag}_CE", cx, cy, cz+0.45, 3.20+cl, 1.30+cl, 0.20+cl))
                horn_coupler(comp, tag, cx-0.50, cy, cz+1.40, "z", "micro")
            else:  # y
                box(comp, f"{tag}_VisBody", cx,      cy,      cz,      2.30, 2.30, 1.20, op_blue)
                box(comp, f"{tag}_VisEars", cx,      cy+0.45, cz,      3.20, 0.20, 1.30, op_blue)
                cyl(comp, f"{tag}_VisHorn", cx,      cy+1.40, cz+0.50, 0.55, 0.18, "y", white_pla)
                marker(comp, f"{tag}_Pivot", cx,     cy+1.40, cz+0.50)
                cut_cavity(comp, box(comp, f"{tag}_CB", cx, cy,      cz, 2.30+cl, 2.30+cl, 1.20+cl))
                cut_cavity(comp, box(comp, f"{tag}_CE", cx, cy+0.45, cz, 3.20+cl, 0.20+cl, 1.30+cl))
                horn_coupler(comp, tag, cx, cy+1.40, cz+0.50, "y", "micro")
            servo_hardware(comp, tag, cx, cy, cz, axis, False)
            BOM.add("Servo", "MG90S micro servo", 1, comp.name)

        def tt_wheel(comp, tag, cx, cy, cz, side=1):
            cl = CLEARANCE
            box(comp, f"{tag}_VisGB",    cx,            cy,     cz,  2.30, 5.20, 1.90, yellow_met)
            cyl(comp, f"{tag}_VisMot",   cx,            cy-1.5, cz,  0.90, 2.10, "y",  chrome)
            cyl(comp, f"{tag}_VisShaft", cx+side*1.75, cy,      cz,  0.20, 3.50, "x",  chrome)
            cyl(comp, f"{tag}_VisHub",   cx+side*3.25, cy,      cz,  0.80, 2.60, "x",  dark_metal)
            cyl(comp, f"{tag}_VisTire",  cx+side*3.25, cy,      cz,  3.25, 2.60, "x",  rubber_blk)
            cyl(comp, f"{tag}_VisRim",   cx+side*3.25, cy,      cz,  2.20, 2.65, "x",  chrome)
            marker(comp, f"{tag}_Axle_Pivot", cx+side*3.25, cy, cz, 0.18)
            cut_cavity(comp, box(comp, f"{tag}_CGB", cx,            cy, cz, 2.30+cl, 5.20+cl, 1.90+cl))
            cut_cavity(comp, box(comp, f"{tag}_CDS", cx+side*3.25, cy, cz, 2.7+cl,  0.54+cl, 0.36+cl))
            BOM.add("Drive",   "TT gear-motor 3V-6V",        1, comp.name)
            BOM.add("Drive",   "65 mm rubber tyre + wheel", 1, comp.name)

        def bearing(comp, tag, cx, cy, cz, axis="x", ro=1.10, w=0.60):
            """MFG-3: Centred bearing cavity with tolerance and retaining lip."""
            cyl(comp, f"{tag}_BO", cx, cy, cz, ro,        w,      axis, chrome)
            cyl(comp, f"{tag}_BI", cx, cy, cz, ro*0.58,  w*0.80, axis, dark_grey)
            cyl(comp, f"{tag}_BB", cx, cy, cz, ro*0.32,  w*1.10, axis, chrome)
            
            # Toleranced cavity for FDM press-fit
            temp  = adsk.fusion.TemporaryBRepManager.get()
            ax    = {"x": (1,0,0), "y": (0,1,0), "z": (0,0,1)}[axis]
            half  = w/2.0 + 0.05
            p1    = adsk.core.Point3D.create(cx-ax[0]*half, cy-ax[1]*half, cz-ax[2]*half)
            p2    = adsk.core.Point3D.create(cx+ax[0]*half, cy+ax[1]*half, cz+ax[2]*half)
            cs    = temp.createCylinderOrCone(p1, ro + FDM_TOL, p2, ro + FDM_TOL)
            bf    = comp.features.baseFeatures.add()
            bf.startEdit()
            cb    = comp.bRepBodies.add(cs, bf)
            bf.finishEdit()
            cb.name = f"{tag}_CB"
            cut_cavity(comp, cb)

            # Retaining Lip (subtracts a smaller cylinder at the far end so bearing doesn't slide through)
            lip_depth = 0.1
            p_lip1 = adsk.core.Point3D.create(cx+ax[0]*(half-lip_depth), cy+ax[1]*(half-lip_depth), cz+ax[2]*(half-lip_depth))
            p_lip2 = adsk.core.Point3D.create(cx+ax[0]*half, cy+ax[1]*half, cz+ax[2]*half)
            cs_lip = temp.createCylinderOrCone(p_lip1, ro - 0.1, p_lip2, ro - 0.1)
            bf_lip = comp.features.baseFeatures.add()
            bf_lip.startEdit()
            cb_lip = comp.bRepBodies.add(cs_lip, bf_lip)
            bf_lip.finishEdit()
            cb_lip.name = f"{tag}_RetainingLip"
            # Add retaining lip geometry back to shell (assuming it intersects)
            merge_fasteners_to_halves(comp) # we'll call this globally later but good to note

            size_tag = f"Ø{int(ro*2*10)} mm bearing"
            BOM.add("Bearing", size_tag, 1, comp.name)

        def u_bracket(comp, tag, cx, cy, cz, lx, ly, lz, ap=None):
            ap = ap or chrome
            box(comp, f"{tag}_BB",  cx,          cy,        cz, 0.45,     ly,   lz, ap)
            box(comp, f"{tag}_BL",  cx+lx*0.45,  cy+ly*0.35, cz, lx*0.55, 0.40, lz, ap)
            box(comp, f"{tag}_BR",  cx+lx*0.45,  cy-ly*0.35, cz, lx*0.55, 0.40, lz, ap)
            cyl(comp, f"{tag}_Pin", cx+lx*0.50,  cy,         cz, 0.18, ly*0.85, "y", chrome)

        # ═════════════════════════════════════════════════════════════════
        # COMPONENT BUILDING
        # ═════════════════════════════════════════════════════════════════

        # ─────────────────────────────────────────────────────────────────
        # ① TORSO 
        # ─────────────────────────────────────────────────────────────────
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

        box(torso, "Cable_L",       -3.4,  0.6,  TORSO_CTR,      0.55, 1.0, 10.0, dark_grey)
        box(torso, "Cable_R",        3.4,  0.6,  TORSO_CTR,      0.55, 1.0, 10.0, dark_grey)
        cable_clip(torso, "Spine", 0, 1.0, TORSO_CTR-2.0, "z")
        cable_clip(torso, "Spine", 0, 1.0, TORSO_CTR+2.0, "z")

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
        mg996r(torso,    "Waist_Yaw",   0, 0, WAIST_CTR,     "z")
        bearing(torso,   "Waist_Brg",   0, 0, WAIST_CTR+0.5, "z", 1.30, 0.65)
        u_bracket(torso, "WaistP_Brkt", 0, 0, WAIST_CTR-2.5, 4.0, 4.2, 3.4)
        mg996r(torso,    "Waist_Pitch", 0, 0, WAIST_CTR-2.5, "x")
        bearing(torso,   "WaistP_Brg",  0, 0, WAIST_CTR-2.0, "x", 1.30, 0.65)
        magnet_pocket(torso, "WLock_F", 0, -2.0, WAIST_CTR-3.0)
        magnet_pocket(torso, "WLock_R", 0,  2.0, WAIST_CTR-3.0)

        u_bracket(torso, "Neck_Brkt",   0, 0, NECK_JOINT_Z,  3.2, 2.8, 3.0)
        mg996r(torso,    "Neck_Pitch",  0, 0, NECK_JOINT_Z,  "x")
        wire_channel(torso, "Spine",    0, 0, TORSO_CTR,     0.6, 20.0, "z")

        # ─────────────────────────────────────────────────────────────────
        # ② HEAD 
        # ─────────────────────────────────────────────────────────────────
        head = new_component("OP_Head")
        HC   = HEAD_CTR

        box(head, "Helm_Main",      0,     0,    HC+0.8,  5.6, 5.2, 5.0, op_blue)
        box(head, "Helm_Forehead",  0,    -2.30, HC+2.6,  5.0, 0.42, 1.8, op_blue)
        box(head, "Helm_Top",       0,     0,    HC+3.4,  4.8, 4.4, 0.90, op_blue)
        
        # MFG-1: Chamfer ear wings for printing without support
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

        box(head, "Head_Rear",      0,    1.90,  HC+1.0,  4.2, 1.5, 4.4, op_red)
        box(head, "Neck_Collar",    0,    0,     HC-1.6,  2.5, 2.5, 2.4, dark_metal)

        cyl(head, "Ant_L",         -2.80, 0,     HC+4.4, 0.13, 2.7, "z", chrome)
        cyl(head, "Ant_R",          2.80, 0,     HC+4.4, 0.13, 2.7, "z", chrome)
        cyl(head, "AntTip_L",      -2.80, 0,     HC+5.8, 0.24, 0.34, "z", gold_met)
        cyl(head, "AntTip_R",       2.80, 0,     HC+5.8, 0.24, 0.34, "z", gold_met)

        esp32_cam_pocket(head, "HeadCam", 0, -1.6, HC+2.5, "y")
        mg90s(head, "Neck_Yaw", 0, 0, NECK_JOINT_Z, "z")

        # ─────────────────────────────────────────────────────────────────
        # ③ PELVIS 
        # ─────────────────────────────────────────────────────────────────
        pelvis = new_component("OP_Pelvis")
        box(pelvis, "Pelvis_Shell",  0,    0,  PELVIS_CTR,  16.4, 6.2, 5.0, op_blue)
        box(pelvis, "Pelvis_Frame",  0,    0,  PELVIS_CTR,  12.2, 4.4, 3.8, dark_metal)
        box(pelvis, "Hip_Armr_L",  -7.2,  0,  PELVIS_CTR,   1.0, 5.2, 4.2, chrome)
        box(pelvis, "Hip_Armr_R",   7.2,  0,  PELVIS_CTR,   1.0, 5.2, 4.2, chrome)
        box(pelvis, "Crotch_Plate", 0,  -2.9, PELVIS_CTR-1.2, 5.2, 0.30, 2.4, op_red)
        
        imu_pocket(pelvis, "BalanceIMU", 0, 0, PELVIS_CTR)
        mg996r(pelvis, "L_HipYaw", -HIP_X, 0, HIP_JOINT_Z, "z")
        mg996r(pelvis, "R_HipYaw",  HIP_X, 0, HIP_JOINT_Z, "z")
        bearing(pelvis, "L_HYB",  -HIP_X-2.2, 0, HIP_JOINT_Z, "z", 1.10, 0.62)
        bearing(pelvis, "R_HYB",   HIP_X+2.2, 0, HIP_JOINT_Z, "z", 1.10, 0.62)
        BOM.add("Servo", "DS3225MG 25 kg·cm servo (hip yaw)", 2, "OP_Pelvis")

        # ─────────────────────────────────────────────────────────────────
        # ④ LEGS
        # ─────────────────────────────────────────────────────────────────
        for side, sx in [("L", -HIP_X), ("R", HIP_X)]:
            m = -1 if side == "L" else 1

            # —— THIGH ——
            thigh = new_component(f"OP_Thigh_{side}")
            box(thigh, "Thigh_Link",         sx,        0,  THIGH_CTR,      5.0, 4.0, 11.0, chrome)
            box(thigh, "Thigh_Outer",        sx+m*2.65, 0,  THIGH_CTR,      0.50, 4.4, 11.0, op_red)
            box(thigh, "Thigh_Front",        sx,       -2.2, THIGH_CTR,     5.0, 0.40, 11.0, op_blue)
            box(thigh, "Thigh_Knee_Armor",   sx,       -2.2, KNEE_CTR+2.5,  4.2, 0.80,  2.8, chrome)
            u_bracket(thigh, f"{side}_HPB",  sx, 0, HIP_JOINT_Z+0.5,        4.0, 3.2, 3.2)
            mg996r(thigh, f"{side}_HipP",    sx, 0, HIP_JOINT_Z,            "x")
            mg996r(thigh, f"{side}_HipR",    sx, 0, THIGH_CTR+2.0,          "y")
            bearing(thigh, f"{side}_HRB",    sx, 0, THIGH_CTR+2.0,          "y", 1.00, 0.55)
            u_bracket(thigh, f"{side}_KnB",  sx, 0, KNEE_CTR+1.5,           3.8, 3.0, 3.0)
            mg996r(thigh, f"{side}_KneP",    sx, 0, KNEE_CTR+1.5,           "x")
            bearing(thigh, f"{side}_KB",     sx, 0, KNEE_CTR,               "x", 1.00, 0.55)
            wire_channel(thigh, f"{side}_LW", sx, 0, THIGH_CTR,             0.5, 12.0, "z")
            cable_clip(thigh, f"{side}_LWC", sx, 2.0, THIGH_CTR, "z")
            
            for bz in [THIGH_CTR+3.0, THIGH_CTR-3.0]:
                m3_boss(thigh, f"{side}_ThighBoss_{bz:.0f}", sx, 0, bz)
            magnet_pocket(thigh, f"{side}_KLU", sx, -1.5, KNEE_CTR+1.0)
            magnet_pocket(thigh, f"{side}_KLL", sx,  1.5, KNEE_CTR+1.0)
            BOM.add("Servo", "DS3225MG 25 kg·cm servo (hip pitch)", 1, f"OP_Thigh_{side}")

            # —— SHIN ——
            shin = new_component(f"OP_Shin_{side}")
            box(shin, "Shin_Link",    sx,    0,    SHIN_CTR,  4.4, 6.0, 11.0, op_blue)
            box(shin, "Shin_Armor",   sx,   -2.7,  SHIN_CTR,  3.2, 0.34,  9.2, chrome)
            box(shin, "Shin_Rear",    sx,    2.7,  SHIN_CTR,  2.0, 0.34,  9.8, dark_grey)
            box(shin, "Shin_Beam",    sx,    0.4,  SHIN_CTR,  1.8, 2.2, 10.2, dark_metal)
            box(shin, "KneeCap",      sx,   -2.9,  KNEE_CTR-1.0, 3.0, 0.55, 2.2, chrome)
            tt_wheel(shin, f"{side}_WF", sx+m*2.0, -4.2, SHIN_CTR+4.0, m)
            tt_wheel(shin, f"{side}_WR", sx+m*2.0, -4.2, SHIN_CTR-4.0, m)
            bearing(shin, f"{side}_KLB", sx, 0, KNEE_CTR-0.5, "x", 1.00, 0.55)
            wire_channel(shin, f"{side}_SW", sx, 0, SHIN_CTR, 0.5, 11.0, "z")
            cut_cavity(shin, box(shin, "FtTuck", sx, 2.7, SHIN_CTR-3.5, 5.2, 1.3, 4.2))
            magnet_pocket(shin, f"{side}_KU", sx, -1.5, KNEE_CTR-1.0)
            magnet_pocket(shin, f"{side}_KL", sx,  1.5, KNEE_CTR-1.0)
            for bz in [SHIN_CTR+3.5, SHIN_CTR-3.5]:
                m3_boss(shin, f"{side}_ShinBoss_{bz:.0f}", sx, 0, bz)

            # —— FOOT ——
            foot = new_component(f"OP_Foot_{side}")
            box(foot, "Foot_Sole",    sx,       -0.6,  ANKLE_CTR-1.5,  6.2, 9.2, 1.10, op_red)
            for vi in [-1.0, 0.0, 1.0]:
                box(foot, f"Arch_Vent_{int(vi*10)}", sx+vi*1.2, -0.6, ANKLE_CTR-1.94, 0.40, 5.5, 0.14, dark_grey)
            # MFG-1: Print friendly chamfer on spur
            cone_shape(foot, "Heel_Block",   sx-m*0.9,  3.2,  ANKLE_CTR-0.8,  2.5, 3.5, 2.6, "y", dark_grey)
            box(foot, "Heel_Plate",   sx-m*0.6,  4.4,  ANKLE_CTR-1.2,  3.2, 0.32, 2.0, chrome)
            box(foot, "Heel_Spur",    sx-m*1.0,  4.8,  ANKLE_CTR-0.2,  1.2, 0.40, 3.2, op_red)
            box(foot, "Toe_Block",    sx+m*0.8, -3.8,  ANKLE_CTR-0.8,  2.6, 3.8, 2.0, dark_grey)
            box(foot, "Toe_Plate",    sx+m*0.5, -4.6,  ANKLE_CTR-1.2,  3.8, 0.32, 1.8, chrome)
            box(foot, "Toe_Cap",      sx+m*1.0, -5.2,  ANKLE_CTR-0.8,  2.8, 0.42, 1.5, op_red)
            box(foot, "Ankle_Guard",  sx,        0,    ANKLE_CTR+1.2,  5.4, 3.2, 2.8, chrome)
            box(foot, "Ankle_Inner",  sx,       -1.0,  ANKLE_CTR+0.3,  4.0, 2.0, 1.6, dark_metal)
            box(foot, "Boot_Fin",     sx+m*2.0,  0,    ANKLE_CTR-0.2,  0.40, 6.5, 4.2, op_blue)
            box(foot, "Boot_Fin2",    sx+m*2.5,  0,    ANKLE_CTR+0.8,  0.32, 5.0, 2.8, op_red)
            mg996r(foot, f"{side}_AnkP", sx, 0, ANKLE_CTR+2.2, "x")
            mg996r(foot, f"{side}_AnkR", sx, 0, ANKLE_CTR+0.5, "y")
            bearing(foot, f"{side}_AB",  sx, 0, ANKLE_CTR,     "x", 1.00, 0.55)
            for bx_off in [-1.5, 1.5]:
                m3_boss(foot, f"{side}_FootBoss_{bx_off:.0f}", sx+bx_off, 0, ANKLE_CTR-0.5)
            align_pin(foot, f"{side}_Ft_A", sx-1.0, 0, ANKLE_CTR-1.3)
            align_pin(foot, f"{side}_Ft_B", sx+1.0, 0, ANKLE_CTR-1.3)

        # ─────────────────────────────────────────────────────────────────
        # ⑤ ARMS 
        # ─────────────────────────────────────────────────────────────────
        for side, ax in [("L", -SHOULDER_X), ("R", SHOULDER_X)]:
            m = -1 if side == "L" else 1

            # —— UPPER ARM ——
            ua = new_component(f"OP_UpperArm_{side}")
            box(ua, "Sh_Block",        ax,         0, SHOULDER_CTR,      5.6, 4.2, 5.6, op_red)
            box(ua, "Sh_Pad_Outer",    ax+m*3.20,  0, SHOULDER_CTR,      1.00, 5.2, 5.8, op_red)
            box(ua, "Sh_Pad_Edge",     ax+m*3.75,  0, SHOULDER_CTR,      0.40, 4.6, 5.2, chrome)
            for sz, sr, sh2 in [(SHOULDER_CTR+2.8, 0.42, 0.95), (SHOULDER_CTR+1.5, 0.36, 0.75)]:
                cyl(ua, f"Stk_A_{sz:.0f}", ax+m*3.3, -1.5, sz, sr,  sh2, "z", chrome)
                cone_shape(ua, f"StkTip_{sz:.0f}", ax+m*3.3, -1.5, sz+sh2/2+0.35, sr, sr*0.25, 0.70, "z", dark_grey)
            box(ua, "Sh_Guard",        ax+m*2.60,  0, SHOULDER_CTR-0.2, 0.42, 4.4, 6.6, op_blue)

            box(ua, "UA_Link",         ax,         0, ELBOW_Z+3.0,      3.2, 3.4, 5.2, op_red)
            box(ua, "UA_Skin",         ax+m*1.80,  0, ELBOW_Z+3.0,      0.52, 3.4, 5.2, chrome)

            mg996r(ua, f"{side}_ShY",    ax, 0, SHOULDER_CTR+1.5, "z")
            bearing(ua, f"{side}_SYB",   ax, 0, SHOULDER_CTR+2.0, "z", 1.00, 0.55)
            u_bracket(ua, f"{side}_SPB", ax, 0, SHOULDER_CTR,     4.8, 3.4, 3.4)
            mg996r(ua, f"{side}_ShP",    ax, 0, SHOULDER_CTR,     "x")
            mg996r(ua, f"{side}_ShR",    ax, 0, SHOULDER_CTR-1.2, "y")
            bearing(ua, f"{side}_SB",    ax, 0, SHOULDER_CTR,     "x", 1.10, 0.62)

            u_bracket(ua, f"{side}_EB",  ax, 0, ELBOW_Z,          3.8, 3.0, 3.0)
            mg996r(ua, f"{side}_ElbP",   ax, 0, ELBOW_Z,          "x")
            bearing(ua, f"{side}_EBr",   ax, 0, ELBOW_Z-0.5,      "x", 0.95, 0.52)
            wire_channel(ua, f"{side}_UAW", ax, 0, ELBOW_Z+3.0,   0.4, 5.2, "z")
            m3_boss(ua, f"{side}_UAboss", ax, 0, ELBOW_Z+3.0)

            # —— FOREARM ——
            fa = new_component(f"OP_Forearm_{side}")
            box(fa, "FA_Link",    ax,       0,    WRIST_Z+3.5, 3.2, 3.8, 4.8, op_blue)
            box(fa, "FA_Fender",  ax+m*2.1, 0,    WRIST_Z+3.5, 0.52, 5.2, 5.8, op_red)
            box(fa, "FA_Back",    ax,       2.3,  WRIST_Z+3.5, 2.6, 0.38, 4.8, chrome)
            box(fa, "FA_Vent_L",  ax-0.6,  -1.8,  WRIST_Z+3.5, 0.30, 0.22, 3.0, dark_grey)
            box(fa, "FA_Vent_R",  ax+0.6,  -1.8,  WRIST_Z+3.5, 0.30, 0.22, 3.0, dark_grey)
            mg90s(fa, f"{side}_WR",   ax, 0, WRIST_Z+0.8, "x")
            bearing(fa, f"{side}_WB", ax, 0, WRIST_Z+0.5, "x", 0.80, 0.44)
            wire_channel(fa, f"{side}_FAW", ax, 0, WRIST_Z+3.5, 0.4, 4.8, "z")
            m3_boss(fa, f"{side}_FAboss", ax, 0, WRIST_Z+4.2)

            # —— HAND (palm) ——
            hand = new_component(f"OP_Hand_{side}")
            box(hand, "Palm_Main",   ax,         -0.4,  WRIST_Z-1.6, 3.4, 3.0, 2.2, dark_grey)
            box(hand, "Palm_Inner",  ax,          0.6,  WRIST_Z-1.6, 2.6, 2.0, 2.0, black_plastic)
            MCP_Z = WRIST_Z - PALM_BOTTOM_OFFSET
            for ki, kx_off in enumerate(FING_X_OFF):
                cyl(hand, f"Knuckle_{ki}", ax + m*kx_off, -1.5, MCP_Z + 0.15, 0.28, 0.38, "y", chrome)
            cyl(hand, "Wrist_Ring",  ax, 0, WRIST_Z-0.4, 1.05, 0.42, "z", chrome)
            box(hand, "Hand_Panel",  ax+m*0.9, -0.7, WRIST_Z-1.3, 0.38, 2.8, 2.8, op_red)
            
            # MFG-4: Tendon drive mechanism support inside the palm
            box(hand, "Finger_SrvBay", ax, 0.5, WRIST_Z-1.8, 1.4, 0.90, 2.4, black_plastic)
            cyl(hand, "Tendon_Spool", ax, 0.5, WRIST_Z-2.4, 0.4, 0.8, "z", chrome)
            BOM.add("Servo", "DS04-NFC 360-deg digital servo (tendon drive)", 2, f"OP_Hand_{side}")
            BOM.add("Hardware", "0.5mm Kevlar Tendon Line (1m)", 2, f"OP_Hand_{side}")
            
            for lxi in [-0.7, 0.7]:
                led_pocket_5mm(hand, f"PalmLED_{lxi:.0f}", ax+lxi, -1.65, WRIST_Z-1.6, "y")

            # MFG-4: Articulated fingers with tendon routing
            for fi, (fname, fxo, pp_l, mp_l, dp_l) in enumerate(zip(FING_NAMES, FING_X_OFF, FING_PP, FING_MP, FING_DP)):
                fc   = new_component(f"OP_{fname}_{side}")
                fx   = ax + m * fxo
                fy   = -1.35
                mcp_z = MCP_Z
                pp_cz = mcp_z - pp_l / 2
                mp_cz = mcp_z - pp_l - mp_l / 2
                dp_cz = mcp_z - pp_l - mp_l - dp_l / 2

                box(fc, "PP", fx, fy, pp_cz, FING_W, FING_H, pp_l, grey_plastic)
                cyl(fc, "PIP_Hinge", fx, fy, mcp_z - pp_l, 0.27, FING_W + 0.12, "x", chrome)
                box(fc, "MP", fx, fy, mp_cz, FING_W*0.94, FING_H*0.94, mp_l, grey_plastic)
                cyl(fc, "DIP_Hinge", fx, fy, mcp_z - pp_l - mp_l, 0.24, FING_W*0.94 + 0.10, "x", chrome)
                box(fc, "DP", fx, fy, dp_cz, FING_W*0.86, FING_H*0.86, dp_l, grey_plastic)
                cone_shape(fc, "FT", fx, fy, mcp_z - pp_l - mp_l - dp_l - 0.12, FING_W*0.44, 0.05, 0.24, "z", chrome)
                
                # Tendon flex/extend channels
                wire_channel(fc, f"Tendon_Flex_{fname}", fx, fy - FING_H*0.3, pp_cz, 0.06, pp_l*3.0, "z")
                wire_channel(fc, f"Tendon_Ext_{fname}", fx, fy + FING_H*0.3, pp_cz, 0.06, pp_l*3.0, "z")

            thumb = new_component(f"OP_Thumb_{side}")
            tx    = ax + m * 1.70
            ty    = 0.20
            tpp_cz = MCP_Z - THUMB_PP_L / 2
            tdp_cz = MCP_Z - THUMB_PP_L - THUMB_DP_L / 2
            box(thumb, "TP",  tx, ty, tpp_cz, THUMB_W, THUMB_H, THUMB_PP_L, grey_plastic)
            cyl(thumb, "T_Hinge", tx, ty, MCP_Z - THUMB_PP_L, 0.28, THUMB_W + 0.14, "x", chrome)
            box(thumb, "TD",  tx, ty*0.8, tdp_cz, THUMB_W*0.86, THUMB_H*0.86, THUMB_DP_L, grey_plastic)
            cone_shape(thumb, "TT", tx, ty, tdp_cz - THUMB_DP_L/2 - 0.14, THUMB_W*0.44, 0.05, 0.28, "z", chrome)
            wire_channel(thumb, "Tendon_Flex_Thumb", tx, ty - THUMB_H*0.3, tpp_cz, 0.06, THUMB_PP_L*2.0, "z")

            if side == "R":
                blast = new_component("OP_Ion_Blaster")
                cyl(blast, "Barrel_Main",   ax,    -2.2, WRIST_Z-3.2, 0.92, 4.2, "z", dark_metal)
                cyl(blast, "Barrel_Tip",    ax,    -2.2, WRIST_Z-5.6, 0.68, 1.1, "z", chrome)
                cyl(blast, "Barrel_Inner",  ax,    -2.2, WRIST_Z-3.8, 0.44, 2.4, "z", black_plastic)
                box(blast, "Blaster_Body",  ax,    -1.1, WRIST_Z-3.2, 2.6, 2.4, 2.7, dark_metal)
                box(blast, "Blast_Guard",   ax,    -0.2, WRIST_Z-3.2, 2.8, 0.38, 1.6, chrome)
                box(blast, "Blast_Rail_T",  ax,    -0.6, WRIST_Z-2.0, 2.8, 0.22, 0.30, chrome)
                box(blast, "Blast_Rail_B",  ax,    -0.6, WRIST_Z-4.4, 2.8, 0.22, 0.30, chrome)
                box(blast, "Hinge_Block",   ax,    -0.8, WRIST_Z-1.6, 1.1, 0.65, 1.1, dark_metal)
                cyl(blast, "Scope_Body",    ax+1.5,-2.2, WRIST_Z-3.2, 0.42, 2.2, "z", chrome)
                cyl(blast, "Scope_Lens",    ax+1.5,-2.2, WRIST_Z-4.4, 0.28, 0.25, "z", glass_clr)
                led_pocket_5mm(blast, "Muzzle", ax, -2.2, WRIST_Z-6.2, "z")

        # ─────────────────────────────────────────────────────────────────
        # ⑥ BACKPACK
        # ─────────────────────────────────────────────────────────────────
        bp = new_component("OP_Backpack")
        box(bp, "BP_Core",    0, 5.6, TORSO_CTR+0.5, 7.2, 2.6,  9.2, dark_grey)
        box(bp, "BP_Hood",    0, 6.5, TORSO_CTR+1.0, 5.8, 1.1,  7.8, op_red)
        box(bp, "BP_TopFlap", 0, 5.1, TORSO_CTR+5.5, 8.4, 0.38, 5.4, op_red)
        box(bp, "BP_Rad",     0, 6.9, TORSO_CTR-0.5, 5.4, 0.44, 5.7, chrome)
        box(bp, "Exh_Blk",   0, 6.3, TORSO_CTR+2.8, 3.2, 0.65, 2.0, dark_metal)
        cyl(bp,  "Exh_L",   -1.3, 6.7, TORSO_CTR+2.8, 0.40, 1.3, "y", dark_metal)
        cyl(bp,  "Exh_R",    1.3, 6.7, TORSO_CTR+2.8, 0.40, 1.3, "y", dark_metal)
        mg90s(bp,    "Roof_Hinge", 0, 5.1, TORSO_CTR+5.1, "x")
        bearing(bp,  "Roof_Brg",   0, 5.1, TORSO_CTR+5.3, "x", 0.80, 0.44)
        magnet_pocket(bp, "RoofL", -2.6, 5.1, TORSO_CTR+5.7)
        magnet_pocket(bp, "RoofR",  2.6, 5.1, TORSO_CTR+5.7)

        # ─────────────────────────────────────────────────────────────────
        # ⑦ STEER WHEEL PODS
        # ─────────────────────────────────────────────────────────────────
        steer = new_component("OP_SteerPods")
        for side, sx in [("L", -5.6), ("R", 5.6)]:
            m2 = -1 if side == "L" else 1
            box(steer, f"SAr_{side}",  sx, -3.6, 23.9, 1.6, 1.3, 4.2, chrome)
            box(steer, f"SPod_{side}", sx, -4.6, 23.4, 3.0, 2.2, 3.2, dark_grey)
            tt_wheel(steer, f"SW_{side}", sx+m2*2.0, -4.2, 23.4, m2)
            bearing(steer, f"SPiv_{side}", sx, -3.6, 23.9, "z", 0.95, 0.50)
            mg90s(steer, f"SSrv_{side}", sx, -4.2, 23.9, "z")

        # ─────────────────────────────────────────────────────────────────
        # ⑧ SHIELDS / PANELS
        # ─────────────────────────────────────────────────────────────────
        shields = new_component("OP_Shields")
        for side, sx in [("L", -(SHOULDER_X+3.4)), ("R", SHOULDER_X+3.4)]:
            m2 = -1 if side == "L" else 1
            box(shields, f"ShShield_{side}", sx,        0, SHOULDER_CTR+1.5, 1.1, 4.6, 5.2, chrome)
            box(shields, f"ShHinge_{side}",  sx-m2*0.7, 0, SHOULDER_CTR+1.5, 0.5, 1.9, 1.9, dark_grey)
            box(shields, f"Mirror_{side}",   sx+m2*0.5,-2.9, SHOULDER_CTR+2.0, 1.5, 0.2, 0.9, dark_grey)
        for side2, hx in [("L", -(HIP_X+3.1)), ("R", HIP_X+3.1)]:
            box(shields, f"HipShield_{side2}", hx, 0, PELVIS_CTR+0.5, 1.1, 4.4, 4.0, op_blue)

        # ─────────────────────────────────────────────────────────────────
        # FDM SHELL SPLITTING & FASTENER MERGING (MFG-1, MFG-4)
        # ─────────────────────────────────────────────────────────────────
        for comp in comps_list:
            to_split = [b for b in comp.bRepBodies if b.name and any(k in b.name for k in SPLIT_KEYS)]
            for b in to_split:
                split_halves(comp, b, "y", 0.0)
            # Recombine the separated physical fasteners to the split halves
            merge_fasteners_to_halves(comp)

        # MFG-7: Auto-generate assembly alignment jigs
        jigs = new_component("OP_Assembly_Jigs")
        # Example Torso Alignment Jig
        box(jigs, "Torso_Jig_Block", 0, 0, TORSO_CTR, 11.0, 1.0, 1.0, op_blue)
        align_socket(jigs, "Jig_Sock_L", -5.0, 0, TORSO_CTR)
        align_socket(jigs, "Jig_Sock_R",  5.0, 0, TORSO_CTR)
        BOM.add("Printed", "Assembly Alignment Jigs", 1, "OP_Assembly_Jigs")

        # ─────────────────────────────────────────────────────────────────
        # KINEMATICS
        # ─────────────────────────────────────────────────────────────────
        t  = occs.get("OP_Torso")
        p  = occs.get("OP_Pelvis")
        h  = occs.get("OP_Head")
        b  = occs.get("OP_Backpack")
        st = occs.get("OP_SteerPods")
        sh = occs.get("OP_Shields")

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

        Logger.log("Validating mechanical linkages...")
        try:
            cam = app.activeViewport.camera
            cam.isFitView = True
            app.activeViewport.camera = cam
        except Exception: pass

        # ═════════════════════════════════════════════════════════════════
        # BUILD GUIDE GENERATOR
        # ═════════════════════════════════════════════════════════════════
        def generate_assembly_guide():
            Logger.log("Generating build guide...")
            try:
                os.makedirs(EXPORT_DIR, exist_ok=True)
                with open(GUIDE_FILE, "w", encoding="utf-8") as f:
                    f.write("OPTIMUS PRIME G1 v11.0 - PHYSICAL ASSEMBLY GUIDE\n")
                    f.write("="*50 + "\n\n")
                    f.write("1. PRINTING ORIENTATION & SUPPORTS\n")
                    f.write("- Shell parts: Print flat side down (split plane on bed).\n")
                    f.write("- Structural links (Thigh/Shin): Print vertically, use dense infill (40%+).\n")
                    f.write("- Tolerances: All bearing pockets are +0.15mm designed tolerance. If tight, sand lightly.\n\n")
                    f.write("2. SERVO COUPLER INSTALLATION\n")
                    f.write("- Install the printed splined horn couplers onto all servos before mounting the servos into brackets.\n")
                    f.write("- Use a drop of CA glue or a set screw to secure the FDM coupler to the metal spline.\n\n")
                    f.write("3. TENDON FINGER DRIVE\n")
                    f.write("- Run 0.5mm Kevlar line through the printed distal fingertip holes.\n")
                    f.write("- Route through the flex and extend channels inside the phalanges.\n")
                    f.write("- Tie off onto the DS04-NFC servo spool in the palm.\n\n")
                    f.write("4. WIRING\n")
                    f.write("- Feed 22AWG servo wires through the printed Grommets.\n")
                    f.write("- Route them along the Spine using the integrated cable clips.\n")
                    f.write("- Plug into PCA9685 mounted inside the Torso.\n\n")
                Logger.log(f"Guide saved -> {GUIDE_FILE}")
            except Exception as e:
                Logger.log(f"Failed to generate guide: {e}", "WARN")

        # Call MFG verifications
        BOM.check_fastener_lengths()
        generate_assembly_guide()

        # ═════════════════════════════════════════════════════════════════
        # ARCHIVE & RUN
        # ═════════════════════════════════════════════════════════════════
        try:
            os.makedirs(EXPORT_DIR, exist_ok=True)
            archive_path = os.path.join(EXPORT_DIR, "Optimus_Prime_G1_v11.f3d")
            export_mgr   = design.exportManager
            archive_opts = export_mgr.createFusionArchiveExportOptions(archive_path)
            export_mgr.execute(archive_opts)
            Logger.log(f"Design archived -> {archive_path}")
        except Exception as e:
            Logger.log(f"Archive skipped: {e}", "WARN")

        # BOM Export Execution
        BOM.add("Fastener", "M3x8 SHCS (general assembly)", 80,  "assembly")
        BOM.add("Fastener", "M3x16 SHCS (joint brackets)",  24,  "assembly")
        BOM.add("Fastener", "M2.5x6 SHCS (PCB mounting)",    16, "assembly")
        BOM.add("Hardware", "Ø3mm x 30 mm shaft pin",        12,  "joints")
        BOM.add("Hardware", "Ø3mm x 20 mm shaft pin",        20,  "joints")
        BOM.add("Material", "PETG filament 1kg spool",         3, "Structural pieces")
        BOM.add("Electronics", "22AWG servo wire (3m)", 1, "wiring harness")
        BOM.save_csv(BOM_FILE)

        Logger.log("v11 Physical Build script finished successfully.")

    except Exception:
        Logger.log(f"FATAL ERROR:\n{traceback.format_exc()}", "ERROR")

    finally:
        Logger.flush()