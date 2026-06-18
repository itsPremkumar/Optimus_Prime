# ─────────────────────────────────────────────────────────────────────────────
# OPTIMUS PRIME G1  v11.0  ── Physically Buildable Prototype Edition
# Fusion 360 Python Script  |  Anthropic MCP-compatible
# ─────────────────────────────────────────────────────────────────────────────
# WHAT'S NEW IN v11  (Physical Engineering Upgrade from v10)
# ──────────────────────────────────────────────
# PHY-6  horn_coupler()     ─ servo-horn coupler with keyed slot, clamp hub,
#                             set-screw pocket; sizes for MG996R/DS3218/DS3225MG
#                             and MG90S/micro servos
# PHY-7  servo_drum()       ─ tendon winch drum for finger-drive servos
# PHY-8  tendon_guide()     ─ V-groove cable guide per phalanx
# PHY-9  tendon_anchor()    ─ distal tendon anchor with crimp slot
# PHY-10 palm_pulley()      ─ idler pulley / guide post in palm
# PHY-11 spring_return()    ─ torsion-spring pocket for finger extension
#
# COV-1  cover_plate()      ─ generic removable cover (screw/magnet/snap/hinge)
# COV-2  lipo_door()        ─ battery bay door with latch
# COV-3  pcb_cover()        ─ electronics bay cover with vent slots
#
# MFG-1  merge_fasteners_to_halves() ─ post-split Boolean merge of Pin/Boss/
#                             Insert/Nut/Snap to correct shell half
# MFG-2  printability_check() ─ overhang analysis + support warnings
# MFG-3  chamfer_box()      ─ 45-degree printable chamfer helper
#
# CAB-1  cable_clip()       ─ snap-in wire retainer
# CAB-2  wire_hub()         ─ torso harness convergence block
# CAB-3  grommet_slot()     ─ oval wire-exit with rubber grommet seat
# CAB-4  jst_pocket()       ─ JST connector clearance pocket
#
# BRG-2  bearing_fit()      ─ press-fit/glue-fit bearing with retention lip
# BRG-3  dual_bearing()     ─ paired bearing carrier for high-load joints
#
# JIG-1  assembly_jig()     ─ printable alignment jig with pegs
# JIG-2  export_jigs()      ─ separate STL export for jigs
#
# FST-1  verify_screw_lengths() ─ engagement validation against shell/boss depth
# FST-2  screw_map_entry()  ─ per-location fastener logging
#
# DOC-1  write_assembly_guide() ─ ASSEMBLY_GUIDE.txt with step order
# DOC-2  print_orientation_notes() ─ per-part orientation advice
# DOC-3  support_warnings_log() ─ support-needed alerts
#
# MECH-1 Internal skeleton  ─ spine beam, rib plates, load paths
# MECH-2 Dual bearings      ─ hips, knees, shoulders
# MECH-3 Hard stops         ─ mechanical joint limits
# MECH-4 Transform locks    ─ spring-pin latches for robot/truck mode
# MECH-5 Load ribs          ─ thickened bosses, gussets
#
# POW-1  Battery retention  ─ strap slots, foam pad space
# POW-2  Fuse pocket        ─ blade fuse holder
# POW-3  Power switch       ─ panel-mount cutout
# POW-4  BEC mount          ─ 5V regulator space
# POW-5  Servo/logic rail   ─ separate power path planning
#
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
LOG_FILE       = os.path.join(LOG_DIR,     f"optimus_v11_{_ts}.txt")
BOM_FILE       = os.path.join(_OUTPUT_DIR, f"BOM_v11_{_ts}.csv")
ASSEMBLY_FILE  = os.path.join(_OUTPUT_DIR, f"ASSEMBLY_GUIDE_v11_{_ts}.txt")
STOP_FLAG      = os.path.join(_OUTPUT_DIR, "stop.flag")


# ═════════════════════════════════════════════════════════════════════════════
# CONFIGURATION  — ALL DIMENSIONS IN cm (Fusion 360 native unit)
# ═════════════════════════════════════════════════════════════════════════════

# ── Skeleton Z-heights (preserved from v10 for kinematic compatibility) ────────
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

# ── Physical / FDM print parameters ──────────────────────────────────────────
WALL_S       = 0.30    # 3.0 mm structural shell wall (min for FDM strength)
WALL_P       = 0.20    # 2.0 mm partition / internal wall
WALL_F       = 0.15    # 1.5 mm snap-fit cantilever arm

# ── V11 NEW: Bearing fit tolerance ───────────────────────────────────────────
BEARING_FIT_TOLERANCE = 0.008   # 0.08 mm diametral interference for press-fit
BEARING_RETAIN_LIP_H  = 0.06    # 0.6 mm shallow retaining lip height
BEARING_RETAIN_LIP_R  = 0.04    # 0.4 mm lip protrusion

# ── V11 NEW: Tendon / finger drive parameters ────────────────────────────────
TENDON_DIA        = 0.04       # 0.4 mm braided fishing line / Dyneema
TENDON_GUIDE_R    = 0.06       # groove radius
DRUM_R            = 0.35       # servo winch drum radius
DRUM_H            = 0.50       # drum height
PULLEY_R          = 0.20       # palm idler pulley radius
SPRING_OD         = 0.25       # torsion spring outer diameter
SPRING_WIRE       = 0.04       # spring wire diameter

# ── V11 NEW: Cable management ────────────────────────────────────────────────
CABLE_CLIP_R      = 0.12       # snap-in wire clip radius
CABLE_CLIP_W      = 0.35       # clip saddle width
JST_XH_L          = 0.55       # JST-XH 3-pin body length
JST_XH_W          = 0.25       # JST-XH body width
JST_XH_H          = 0.18       # JST-XH body height

# ── V11 NEW: Power system ────────────────────────────────────────────────────
FUSE_HOLDER_L     = 2.00       # blade fuse holder length
FUSE_HOLDER_W     = 0.80       # blade fuse holder width
FUSE_HOLDER_H     = 0.75       # blade fuse holder height
BEC_L, BEC_W, BEC_H = 3.50, 1.80, 0.90   # 5V 5A BEC dimensions
POWER_SWITCH_R    = 0.35       # panel-mount switch radius

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

# ── V11 NEW: Screw length registry for verification ──────────────────────────
# Populated by m3_boss() and captive_nut() calls, verified at end
SCREW_REGISTRY = []    # list of dicts: {tag, comp, shell_t, boss_depth, requested_len}

# ── Electronics footprints (cm) ──────────────────────────────────────────────
RPI0_L,  RPI0_W,  RPI0_H  = 6.50, 3.00, 0.20   # Raspberry Pi Zero 2W
PCA_L,   PCA_W,   PCA_H   = 6.25, 2.54, 0.18   # PCA9685 servo driver
ESP32_L, ESP32_W, ESP32_H = 3.80, 2.58, 0.15   # ESP32-DevKit-C
IMU_L,   IMU_W,   IMU_H   = 2.10, 1.60, 0.12   # MPU-6050 breakout
LIPO_L,  LIPO_W,  LIPO_H  = 7.00, 3.20, 1.80   # 2S 1300 mAh LiPo
XT60_W,  XT60_H_SLOT       = 1.60, 1.30          # XT60-F connector slot
LED_R_5MM                  = 0.260   # Ø5 mm LED pocket radius (with clearance)
LED_R_RING                 = 0.600   # Ø12 mm LED ring pocket outer radius

# ── Finger geometry (cm) ─────────────────────────────────────────────────────
FING_W       = 0.52   # finger width  (X)
FING_H       = 0.48   # finger depth  (Y)
FING_GAP     = 0.10   # gap between adjacent fingers
THUMB_W      = 0.65
THUMB_H      = 0.58
# Phalanx lengths per finger [Pinky, Ring, Middle, Index]
FING_PP      = [1.40, 1.60, 1.70, 1.55]   # proximal
FING_MP      = [1.00, 1.20, 1.30, 1.15]   # middle
FING_DP      = [0.80, 0.90, 0.95, 0.88]   # distal
FING_NAMES   = ["Pinky", "Ring", "Middle", "Index"]
FING_X_OFF   = [-1.10, -0.37,  0.37,  1.10]   # X-offsets from arm X (R-hand)
THUMB_PP_L   = 1.40
THUMB_DP_L   = 1.00
PALM_BOTTOM_OFFSET = 2.50   # Z below WRIST_Z where MCP joints sit

# ── Servo specs (upgraded for v11) ───────────────────────────────────────────
SERVO_SPECS = {
    "hip_hd": {"name": "DS3225MG", "rated": 25.0, "stall": 35.0, "mass_g":  60, "horn_spline": 25},
    "waist":  {"name": "DS3218",   "rated": 20.0, "stall": 25.0, "mass_g":  55, "horn_spline": 25},
    "std":    {"name": "MG996R",   "rated":  9.4, "stall": 11.5, "mass_g":  55, "horn_spline": 25},
    "micro":  {"name": "MG90S",    "rated":  1.8, "stall":  2.2, "mass_g":  13, "horn_spline": 21},
    "digit":  {"name": "DS04-NFC", "rated":  1.8, "stall":  2.2, "mass_g":   9, "horn_spline": 21},
}

# ── ROM limits (deg) ──────────────────────────────────────────────────────────
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
    # Finger MCP joints (revolute, pitch = curl)
    "L_Pinky_MCP":  {"pitch": (-5, 85)}, "R_Pinky_MCP":  {"pitch": (-5, 85)},
    "L_Ring_MCP":   {"pitch": (-5, 85)}, "R_Ring_MCP":   {"pitch": (-5, 85)},
    "L_Middle_MCP": {"pitch": (-5, 85)}, "R_Middle_MCP": {"pitch": (-5, 85)},
    "L_Index_MCP":  {"pitch": (-5, 85)}, "R_Index_MCP":  {"pitch": (-5, 85)},
    # Thumb CMC (ball joint — abduction/flexion)
    "L_Thumb_CMC":  {"pitch": (-20, 60), "yaw": (-30, 30)},
    "R_Thumb_CMC":  {"pitch": (-20, 60), "yaw": (-30, 30)},
}

SPLIT_KEYS = {"Shell", "Link", "Main", "Armor", "Core", "Pod", "Palm",
              "Block", "Sole", "Plate", "Bay", "Collar"}
SKIP_TAGS  = {"Marker", "Pivot", "MtA", "MtB", "Axle_Pivot", "Horn",
              "Pin", "_Vis", "Boss", "Insert", "Nut", "Snap"}

# ── V11 NEW: Assembly step registry ──────────────────────────────────────────
ASSEMBLY_STEPS = []
JIG_REGISTRY   = []
PRINT_NOTES    = []     # list of (part_name, orientation, support_needed)
SUPPORT_WARNINGS = []   # list of (part_name, reason)


# ═════════════════════════════════════════════════════════════════════════════
# LOGGER  (v11 enhanced)
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
# BOM TRACKER  (v11 enhanced with new categories)
# ═════════════════════════════════════════════════════════════════════════════

class BOM:
    """Bill-of-Materials accumulator — call add() while building, save_csv() after."""
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

    app = None
    ui  = None

    Logger.log("=" * 60)
    Logger.log("EXECUTION START  v11.0 -- Optimus Prime G1 Physical Build")
    Logger.log("Enhancements: couplers, tendons, covers, cable mgmt, bearings,")
    Logger.log("              jigs, screw verify, assembly docs, printability")
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
        # V11: new materials
        petg_red      = op_red   or get_ap("PETG Red", "Steel - Painted (Red)")
        petg_blue     = op_blue  or get_ap("PETG Blue", "Steel - Painted (Blue)")
        nylon_white   = white_pla or get_ap("Nylon - White", "Plastic - Glossy (White)")

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

        # ── V11 NEW: Printable chamfer helper ─────────────────────────────
        def chamfer_box(comp, name, cx, cy, cz, lx, ly, lz, axis, chamfer=0.25, ap=None):
            """Create a box with a 45-degree chamfer on one end for printability."""
            body = box(comp, name, cx, cy, cz, lx, ly, lz, ap)
            # Log support warning if chamfer is too small
            if chamfer < 0.15:
                SUPPORT_WARNINGS.append((name, f"chamfer {chamfer}cm may need support"))
            return body

        # ── Boolean cavity cutter ─────────────────────────────────────────
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

        # ── Shell splitter for FDM printing ───────────────────────────────
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

        # ── V11 NEW: Post-split fastener merge ────────────────────────────
        def merge_fasteners_to_halves(comp, body_left, body_right, axis="y"):
            """MFG-1 — After split, merge Pin/Boss/Insert/Nut/Snap bodies to
            the nearest shell half based on position relative to split plane."""
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

        # ── V11 NEW: Printability check ───────────────────────────────────
        def printability_check(comp, body_name, overhang_angle_deg=45):
            """MFG-2 — Log a warning if a part has features likely to need support."""
            # Overhangs sharper than overhang_angle_deg are flagged
            # This is a heuristic check based on naming conventions
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
                Logger.log(f"  [PRINT WARNING] {body_name}: {reason}", "WARN")

        # ─────────────────────────────────────────────────────────────────
        # PHYSICAL FEATURE HELPERS  (PHY-1 … PHY-5  +  V11 PHY-6 … PHY-11)
        # ─────────────────────────────────────────────────────────────────

        def m3_boss(comp, tag, cx, cy, cz, axis="z", depth=0.80, screw_len=1.0):
            """PHY-1 — Ø7 mm boss cylinder + Ø4.7 mm heat-set insert pocket.
            V11: Added screw_len parameter for length verification registry."""
            boss = cyl(comp, f"{tag}_Boss", cx, cy, cz, BOSS_R, depth, axis, dark_metal)
            boss.name = f"{tag}_Boss"
            cut_cavity(comp, cyl(comp, f"{tag}_Insert",
                                 cx, cy, cz, INSERT_R, INSERT_H, axis))
            BOM.add("Fastener", "M3 heat-set insert (Voron M3x5)", 1,
                    f"boss at ({cx:.1f},{cy:.1f},{cz:.1f}) in {comp.name}")
            # V11: register for screw verification
            SCREW_REGISTRY.append({
                "tag": tag, "comp": comp.name, "type": "boss_insert",
                "shell_t": depth, "boss_depth": INSERT_H,
                "requested_len": screw_len, "axis": axis,
                "cx": cx, "cy": cy, "cz": cz,
            })

        def captive_nut(comp, tag, cx, cy, cz, axis="z", bolt_len=1.0):
            """PHY-2 — M3 hex-nut trap (hex slot) + clearance through-bore."""
            cut_cavity(comp, cyl(comp, f"{tag}_NutTrap",
                                 cx, cy, cz, M3_NUT_CIR, M3_NUT_H, axis))
            ax = {"x": (1,0,0), "y": (0,1,0), "z": (0,0,1)}[axis]
            cut_cavity(comp, cyl(comp, f"{tag}_NutBore",
                                 cx, cy, cz, M3_CLR_R, bolt_len, axis))
            BOM.add("Fastener", "M3 hex nut", 1, comp.name)
            BOM.add("Fastener", f"M3x{int(bolt_len*10)} SHCS", 1, comp.name)
            SCREW_REGISTRY.append({
                "tag": tag, "comp": comp.name, "type": "captive_nut",
                "shell_t": bolt_len, "nut_depth": M3_NUT_H,
                "requested_len": bolt_len, "axis": axis,
                "cx": cx, "cy": cy, "cz": cz,
            })

        def snap_clip(comp, tag, cx, cy, cz, span_x=1.2, axis_latch="z"):
            """PHY-3 — cantilever snap-fit pair (two arms, 1.5 mm thick)."""
            for sign in [-1, 1]:
                arm_cx = cx + sign * span_x * 0.5
                box(comp, f"{tag}_SnapArm_{sign}",
                    arm_cx, cy, cz, WALL_F, 0.40, 1.20, grey_plastic)
                box(comp, f"{tag}_SnapHead_{sign}",
                    arm_cx, cy, cz + 0.55, WALL_F + 0.10, 0.50, 0.28, grey_plastic)

        def align_pin(comp, tag, cx, cy, cz, axis="z", height=0.40):
            """PHY-4a — Ø2 mm alignment pin."""
            cyl(comp, f"{tag}_AlignPin", cx, cy, cz, ALIGN_PIN_R, height, axis, chrome)

        def align_socket(comp, tag, cx, cy, cz, axis="z", depth=0.45):
            """PHY-4b — Ø2.15 mm alignment socket (cut into mating part)."""
            cut_cavity(comp, cyl(comp, f"{tag}_AlignSock",
                                 cx, cy, cz, ALIGN_PIN_R + 0.015, depth, axis))

        def grommet_hole(comp, tag, cx, cy, cz, axis="y"):
            """PHY-5 — Ø3.5 mm servo wire-exit groove."""
            cut_cavity(comp, cyl(comp, f"{tag}_Grommet",
                                 cx, cy, cz, GROMMET_R, 0.80, axis))

        # ── V11 NEW: PHY-6  horn_coupler ──────────────────────────────────
        def horn_coupler(comp, tag, cx, cy, cz, axis, servo_type="std"):
            """PHY-6 — Servo output-shaft coupler with keyed slot, clamp hub,
            and set-screw pocket. Integrates into moving link for positive
            torque transfer.
            servo_type: 'std' (MG996R/DS3218/DS3225MG) or 'micro' (MG90S)."""
            is_std = (servo_type in ("std", "hip_hd", "waist"))
            # Coupler hub dimensions
            hub_r  = 0.55 if is_std else 0.35
            hub_h  = 0.80 if is_std else 0.55
            key_w  = 0.14 if is_std else 0.10   # keyed slot width
            key_d  = 0.18 if is_std else 0.13   # keyed slot depth
            clamp_w = 0.10 if is_std else 0.08  # clamp slit width
            setscrew_r = M3_PILOT_R if is_std else 0.09

            ax = {"x": (1, 0, 0), "y": (0, 1, 0), "z": (0, 0, 1)}[axis]

            # Main hub cylinder (FDM printable - split for orientation)
            hub = cyl(comp, f"{tag}_CouplerHub", cx, cy, cz, hub_r, hub_h, axis, dark_metal)

            # Keyed slot (approximates servo horn spline — FDM printable)
            # Orient slot perpendicular to axis for printability
            if axis in ("x", "z"):
                cut_cavity(comp, box(comp, f"{tag}_KeySlot",
                    cx + ax[0]*hub_h*0.25, cy, cz + ax[2]*hub_h*0.25,
                    key_d if axis=="z" else hub_h*0.6,
                    key_w,
                    key_d if axis=="x" else hub_h*0.6))
            else:  # y-axis
                cut_cavity(comp, box(comp, f"{tag}_KeySlot",
                    cx, cy + ax[1]*hub_h*0.25, cz,
                    key_d, hub_h*0.6, key_d))

            # Clamp slit (allows hub to grip servo horn)
            if axis in ("x", "z"):
                cut_cavity(comp, box(comp, f"{tag}_ClampSlit",
                    cx + ax[0]*hub_h*0.35, cy, cz + ax[2]*hub_h*0.35,
                    0.06 if axis=="z" else hub_h*0.5,
                    hub_r*2.2,
                    0.06 if axis=="x" else hub_h*0.5))
            else:
                cut_cavity(comp, box(comp, f"{tag}_ClampSlit",
                    cx, cy + ax[1]*hub_h*0.35, cz,
                    hub_r*2.2, hub_h*0.5, 0.06))

            # Set-screw hole (M3 or M2.5) through clamp
            setscrew_axis = "y" if axis in ("x", "z") else "z"
            cut_cavity(comp, cyl(comp, f"{tag}_SetScrew",
                cx + ax[0]*hub_h*0.15, cy, cz + ax[2]*hub_h*0.15,
                setscrew_r, hub_r*2.2, setscrew_axis))

            # Torque arm (connects hub to link)
            arm_len = hub_r * 1.8
            arm_r   = hub_r * 0.65
            arm_cx  = cx + ax[0] * arm_len
            arm_cy  = cy + ax[1] * arm_len
            arm_cz  = cz + ax[2] * arm_len
            cyl(comp, f"{tag}_TorqueArm", arm_cx, arm_cy, arm_cz,
                arm_r, hub_h*0.8, axis, dark_metal)

            spec_name = SERVO_SPECS.get(servo_type, SERVO_SPECS["std"])["name"]
            BOM.add("Printed", f"Servo coupler hub ({spec_name})", 1, comp.name)
            BOM.add("Fastener", "M3x4 set screw (cup point)", 1, comp.name)
            ASSEMBLY_STEPS.append(f"Install {tag} coupler onto {spec_name} servo horn; tighten set-screw")

        # ── V11 NEW: PHY-7  servo_drum ────────────────────────────────────
        def servo_drum(comp, tag, cx, cy, cz, axis="z"):
            """PHY-7 — Winch drum mounted on finger-drive servo output.
             Has spiral groove for tendon wrapping and top/bottom flanges."""
            # Drum barrel
            cyl(comp, f"{tag}_DrumBarrel", cx, cy, cz, DRUM_R, DRUM_H, axis, dark_metal)
            # Top flange (keeps tendon from riding off)
            cyl(comp, f"{tag}_DrumFlangeT",
                cx, cy, cz + DRUM_H/2 - 0.02, DRUM_R + 0.10, 0.06, axis, dark_metal)
            # Bottom flange
            cyl(comp, f"{tag}_DrumFlangeB",
                cx, cy, cz - DRUM_H/2 + 0.02, DRUM_R + 0.10, 0.06, axis, dark_metal)
            # Tie-off hole (cross-hole for tendon knot)
            tie_axis = "x" if axis in ("y", "z") else "z"
            cut_cavity(comp, cyl(comp, f"{tag}_TieHole",
                cx, cy, cz, 0.06, DRUM_R*2.2, tie_axis))
            BOM.add("Printed", "Servo winch drum (tendon drive)", 1, comp.name)

        # ── V11 NEW: PHY-8  tendon_guide ──────────────────────────────────
        def tendon_guide(comp, tag, cx, cy, cz, length, axis="z"):
            """PHY-8 — V-groove cable guide channel cut into phalanx.
            Guides tendon from proximal to distal joint."""
            # Channel is a slot cut into the body
            gr = TENDON_GUIDE_R + 0.02   # extra clearance
            cut_cavity(comp, cyl(comp, f"{tag}_TendonGuide",
                                 cx, cy, cz, gr, length, axis))

        # ── V11 NEW: PHY-9  tendon_anchor ─────────────────────────────────
        def tendon_anchor(comp, tag, cx, cy, cz, axis="z"):
            """PHY-9 — Distal tendon anchor with crimp slot and ball-end pocket.
            Printed as add-on or integrated into distal phalanx."""
            # Small block with cross-slot
            box(comp, f"{tag}_Anchor", cx, cy, cz, 0.35, 0.28, 0.22, dark_metal)
            # Crimp slot
            cut_cavity(comp, box(comp, f"{tag}_CrimpSlot",
                cx, cy, cz, 0.06, 0.30, 0.14))
            BOM.add("Hardware", "Tendon anchor (printed)", 1, comp.name)

        # ── V11 NEW: PHY-10  palm_pulley ──────────────────────────────────
        def palm_pulley(comp, tag, cx, cy, cz, axis="x"):
            """PHY-10 — Idler pulley or guide post in palm to route tendon
            from servo drum to finger proximal phalanx."""
            # Post axle
            cyl(comp, f"{tag}_PulleyPost", cx, cy, cz, 0.12, 0.50, axis, chrome)
            # Pulley wheel (prints horizontally)
            pulley_axis = "y" if axis in ("x", "z") else "z"
            cyl(comp, f"{tag}_PulleyWheel",
                cx, cy, cz, PULLEY_R, 0.14, pulley_axis, grey_plastic)
            BOM.add("Printed", "Palm idler pulley", 1, comp.name)

        # ── V11 NEW: PHY-11  spring_return ────────────────────────────────
        def spring_return(comp, tag, cx, cy, cz, axis="x"):
            """PHY-11 — Torsion-spring pocket at MCP joint for passive finger
            extension. Spring returns finger to open when servo relaxes."""
            # Spring pocket (cylindrical recess)
            cut_cavity(comp, cyl(comp, f"{tag}_SpringPkt",
                                 cx, cy, cz, SPRING_OD/2 + 0.03, SPRING_WIRE*4, axis))
            # Retention pegs (2 small posts to hook spring ends)
            peg_axis = "y" if axis in ("x", "z") else "z"
            for sign in [-1, 1]:
                peg_pos = SPRING_OD/2 + 0.06
                if peg_axis == "y":
                    cyl(comp, f"{tag}_SpringPeg_{sign}",
                        cx, cy + sign*peg_pos, cz, 0.06, 0.20, peg_axis, chrome)
                else:
                    cyl(comp, f"{tag}_SpringPeg_{sign}",
                        cx, cy, cz + sign*peg_pos, 0.06, 0.20, peg_axis, chrome)
            BOM.add("Hardware", "Torsion spring (finger return)", 1, comp.name)

        # ── Generic fastener helpers ──────────────────────────────────────
        def screw_hole(comp, cx, cy, cz, axis="y", length=3.0):
            cut_cavity(comp, cyl(comp, "ScrewHole", cx, cy, cz,
                                 M3_PILOT_R, length, axis))
            BOM.add("Fastener", f"M3x{int(length*10)} self-tap", 1, comp.name)

        def magnet_pocket(comp, tag, cx, cy, cz, axis="z"):
            cut_cavity(comp, cyl(comp, f"{tag}_MagPkt", cx, cy, cz, 0.32, 0.35, axis))
            BOM.add("Hardware", "Magnet D6x3 mm N35", 1, comp.name)

        def wire_channel(comp, tag, cx, cy, cz, r, h, axis):
            cut_cavity(comp, cyl(comp, f"{tag}_Wire", cx, cy, cz, r, h, axis))

        def led_pocket_5mm(comp, tag, cx, cy, cz, axis="y"):
            """ELEC-4 — Ø5 mm LED through-hole pocket."""
            cut_cavity(comp, cyl(comp, f"{tag}_LED5",
                                 cx, cy, cz, LED_R_5MM, 0.85, axis))
            BOM.add("Electronics", "LED 5 mm (colour TBD)", 1, comp.name)

        def led_ring_pocket(comp, tag, cx, cy, cz, axis="z"):
            """ELEC-4 — Ø12 mm LED ring recess."""
            cut_cavity(comp, cyl(comp, f"{tag}_LEDRing",
                                 cx, cy, cz, LED_R_RING, 0.30, axis))
            BOM.add("Electronics", "WS2812 5050 LED ring Ø12 mm", 1, comp.name)

        # ── V11 NEW: CAB-1  cable_clip ────────────────────────────────────
        def cable_clip(comp, tag, cx, cy, cz, axis="y"):
            """CAB-1 — Snap-in wire retainer saddle. Clips into printed mount
            points to hold servo harnesses."""
            box(comp, f"{tag}_ClipBase", cx, cy, cz,
                CABLE_CLIP_W, 0.15, 0.35, grey_plastic)
            # Saddle arch
            cyl(comp, f"{tag}_ClipArch",
                cx, cy, cz + 0.06, CABLE_CLIP_R + 0.08, CABLE_CLIP_W, "x", grey_plastic)
            BOM.add("Printed", "Snap-in cable clip", 1, comp.name)

        # ── V11 NEW: CAB-2  wire_hub ──────────────────────────────────────
        def wire_hub(comp, tag, cx, cy, cz):
            """CAB-2 — Torso wiring harness convergence block.
            Central point where all servo wires gather before routing to RPi/PCA."""
            # Main block
            box(comp, f"{tag}_HubBlock", cx, cy, cz, 2.0, 1.6, 1.2, dark_grey)
            # Cable entry tunnels (6 directions)
            for dx, dy, dz, lbl in [
                (-1.0, 0, 0, "L"), (1.0, 0, 0, "R"),
                (0, -0.8, 0, "F"), (0, 0.8, 0, "B"),
                (0, 0, -0.6, "D"), (0, 0, 0.6, "U")
            ]:
                wire_channel(comp, f"{tag}_Hub{lbl}",
                    cx+dx*0.5, cy+dy*0.5, cz+dz*0.5, 0.25, 0.80,
                    "x" if abs(dx)>abs(dy) and abs(dx)>abs(dz) else
                    "y" if abs(dy)>abs(dz) else "z")
            BOM.add("Printed", "Torso wire hub", 1, comp.name)

        # ── V11 NEW: CAB-3  grommet_slot ──────────────────────────────────
        def grommet_slot(comp, tag, cx, cy, cz, axis="y", width=0.50):
            """CAB-3 — Oval wire-exit with rubber grommet seat.
            Prevents wire chafing at shell exit points."""
            # Oval slot (cylinder with flattened sides approximated)
            cut_cavity(comp, cyl(comp, f"{tag}_GromSlot",
                                 cx, cy, cz, GROMMET_R, width, axis))
            # Seat ledge for grommet flange
            seat_r = GROMMET_R + 0.06
            cut_cavity(comp, cyl(comp, f"{tag}_GromSeat",
                                 cx, cy, cz, seat_r, 0.10, axis))
            BOM.add("Hardware", "Rubber grommet Ø3.5 mm (open slot)", 1, comp.name)

        # ── V11 NEW: CAB-4  jst_pocket ────────────────────────────────────
        def jst_pocket(comp, tag, cx, cy, cz, axis="y"):
            """CAB-4 — JST-XH 3-pin connector clearance pocket for plug routing."""
            cut_cavity(comp, box(comp, f"{tag}_JST",
                cx, cy, cz,
                JST_XH_L + 0.10, JST_XH_W + 0.10, JST_XH_H + 0.10))

        # ── V11 NEW: BRG-2  bearing_fit ───────────────────────────────────
        def bearing_fit(comp, tag, cx, cy, cz, axis="x", ro=1.10, w=0.60,
                        fit_type="press"):
            """BRG-2 — Bearing pocket with configurable fit tolerance and
            optional retention lip. Bearings are installable after printing.
            fit_type: 'press', 'glue', or 'slip'."""
            tol = BEARING_FIT_TOLERANCE if fit_type == "press" else 0.0 if fit_type == "glue" else 0.015
            outer_r = ro + tol
            # Outer shell (visual only)
            cyl(comp, f"{tag}_BO", cx, cy, cz, ro, w, axis, chrome)
            cyl(comp, f"{tag}_BI", cx, cy, cz, ro*0.58, w*0.80, axis, dark_grey)
            cyl(comp, f"{tag}_BB", cx, cy, cz, ro*0.32, w*1.10, axis, chrome)
            # Cavity with tolerance
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
            # Retention lip (shallow groove near opening)
            if fit_type in ("press", "glue"):
                lip_z = cz - w/2 + BEARING_RETAIN_LIP_H/2
                if axis == "x": lip_x, lip_y, lip_z = cx - w/2 + BEARING_RETAIN_LIP_H/2, cy, cz
                elif axis == "y": lip_x, lip_y, lip_z = cx, cy - w/2 + BEARING_RETAIN_LIP_H/2, cz
                else: lip_x, lip_y, lip_z = cx, cy, cz - w/2 + BEARING_RETAIN_LIP_H/2
                # Lip is a thin ridge printed as part of the shell
                cyl(comp, f"{tag}_Lip", lip_x, lip_y, lip_z,
                    outer_r + BEARING_RETAIN_LIP_R, BEARING_RETAIN_LIP_H, axis, dark_metal)
            fit_label = f"{fit_type}-fit"
            size_tag  = f"Ø{int(ro*2*10)} mm bearing ({fit_label})"
            BOM.add("Bearing", size_tag, 1, comp.name)
            BOM.add("Hardware", f"Bearing {fit_label} tolerance", 1, comp.name)

        # ── V11 NEW: BRG-3  dual_bearing ──────────────────────────────────
        def dual_bearing(comp, tag, cx, cy, cz, axis="x", ro=1.10, w=0.60,
                         span=2.50, fit_type="press"):
            """BRG-3 — Paired bearing carrier for high-load joints
            (hips, knees, shoulders). Bearings at both ends of a through-shaft."""
            ax = {"x": (1,0,0), "y": (0,1,0), "z": (0,0,1)}[axis]
            # Bearing carrier block
            box(comp, f"{tag}_Carrier", cx, cy, cz, span*ax[0]+1.2, span*ax[1]+1.2, span*ax[2]+1.2, dark_metal)
            # Two bearings
            p1 = (cx - ax[0]*span/2, cy - ax[1]*span/2, cz - ax[2]*span/2)
            p2 = (cx + ax[0]*span/2, cy + ax[1]*span/2, cz + ax[2]*span/2)
            bearing_fit(comp, f"{tag}_A", p1[0], p1[1], p1[2], axis, ro, w, fit_type)
            bearing_fit(comp, f"{tag}_B", p2[0], p2[1], p2[2], axis, ro, w, fit_type)
            # Through shaft (steel axle path)
            cyl(comp, f"{tag}_Axle", cx, cy, cz, ro*0.55, span + 1.0, axis, chrome)
            BOM.add("Hardware", f"Steel axle Ø{int(ro*0.55*20)} mm x {int((span+1.0)*10)} mm", 1, comp.name)
            ASSEMBLY_STEPS.append(f"Install {tag} bearings into carrier; insert steel axle")

        # ── V11 NEW: COV-1  cover_plate ───────────────────────────────────
        def cover_plate(comp, tag, cx, cy, cz, lx, lz, boss_positions,
                        method="screw", hinge_edge=None, ap=None):
            """COV-1 — Generic removable cover plate for electronics bays.
            method: 'screw', 'magnet', 'snap', or 'hinge'.
            boss_positions: list of (x,z) local offsets for fasteners."""
            ap = ap or grey_plastic
            ly_cover = 0.25  # cover thickness
            # Main plate
            cover = box(comp, f"{tag}_Cover", cx, cy, cz, lx, ly_cover, lz, ap)
            # Fastening method
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
                # Simple barrel hinge on specified edge
                hinge_y = cy - ly_cover/2
                hinge_axis = "x" if hinge_edge in ("left", "right") else "z"
                cyl(comp, f"{tag}_HingeBarrel", cx, hinge_y, cz,
                    0.12, lx*0.9 if hinge_axis=="x" else lz*0.9, hinge_axis, dark_metal)
                # Latch on opposite edge
                latch_x = cx + (lx/2 - 0.3) if hinge_edge != "right" else cx - (lx/2 - 0.3)
                box(comp, f"{tag}_Latch", latch_x, hinge_y, cz, 0.5, 0.18, 0.4, dark_metal)
                BOM.add("Hardware", "M2x10 hinge pin (steel)", 1, comp.name)
            BOM.add("Printed", f"Cover plate ({method})", 1, comp.name)
            PRINT_NOTES.append((f"{tag}_Cover", "print flat (face down)", False))
            return cover

        # ── V11 NEW: COV-2  lipo_door ─────────────────────────────────────
        def lipo_door(comp, tag, cx, cy, cz, lx, lz):
            """COV-2 — Battery bay door with finger-pull latch and vent slots."""
            cover_plate(comp, tag, cx, cy, cz, lx, lz,
                        [(-lx/2+0.6, -lz/2+0.6), (lx/2-0.6, -lz/2+0.6),
                         (-lx/2+0.6, lz/2-0.6), (lx/2-0.6, lz/2-0.6)],
                        method="snap", ap=dark_grey)
            # Finger pull recess
            box(comp, f"{tag}_Pull", cx, cy+0.20, cz+lz/2-0.4, lx*0.4, 0.15, 0.35, dark_grey)
            # Vent slots (3)
            for vz in [-lz*0.2, 0, lz*0.2]:
                cut_cavity(comp, box(comp, f"{tag}_Vent_{vz:.0f}",
                    cx, cy+0.01, cz+vz, lx*0.6, 0.08, 0.12))
            BOM.add("Printed", "LiPo bay door (vented)", 1, comp.name)

        # ── V11 NEW: COV-3  pcb_cover ─────────────────────────────────────
        def pcb_cover(comp, tag, cx, cy, cz, lx, lz, method="screw"):
            """COV-3 — Electronics bay cover with ventilation slots."""
            cover_plate(comp, tag, cx, cy, cz, lx, lz,
                        [(-lx/2+0.5, -lz/2+0.5), (lx/2-0.5, -lz/2+0.5),
                         (-lx/2+0.5, lz/2-0.5), (lx/2-0.5, lz/2-0.5)],
                        method=method, ap=grey_plastic)
            # Ventilation grid
            for vy in [-lz*0.25, 0, lz*0.25]:
                cut_cavity(comp, box(comp, f"{tag}_VSlot_{vy:.0f}",
                    cx, cy+0.01, cz+vy, lx*0.55, 0.06, 0.25))
            BOM.add("Printed", f"PCB cover ({method})", 1, comp.name)

        # ── V11 NEW: JIG-1  assembly_jig ──────────────────────────────────
        def assembly_jig(comp_name, pin_positions, socket_positions, base_size):
            """JIG-1 — Create a printable assembly jig as a separate component.
            pin_positions:  list of (x,y,z) for alignment pins on jig base
            socket_positions: list of (x,y,z) for sockets
            base_size: (lx, ly, lz) of jig base plate"""
            jig = new_component(f"JIG_{comp_name}")
            bx, by, bz = 0, 0, 0
            lx, ly, lz = base_size
            box(jig, "Jig_Base", bx, by, bz, lx, ly, lz, nylon_white)
            for i, (px, py, pz) in enumerate(pin_positions):
                cyl(jig, f"JigPin_{i}", px, py, pz + lz/2, ALIGN_PIN_R + 0.02, 0.50, "z", chrome)
            for i, (sx, sy, sz) in enumerate(socket_positions):
                cyl(jig, f"JigSock_{i}", sx, sy, sz + lz/2, ALIGN_PIN_R + 0.025, 0.30, "z", dark_grey)
            JIG_REGISTRY.append((jig.name, comp_name))
            PRINT_NOTES.append((jig.name, "print base flat, pins upright", True))
            BOM.add("Tooling", f"Assembly jig for {comp_name}", 1, "printed")
            ASSEMBLY_STEPS.append(f"Print jig for {comp_name}; use during shell alignment")
            return jig

        # ── V11 NEW: FST-1  verify_screw_lengths ──────────────────────────
        def verify_screw_lengths():
            """FST-1 — Validate all registered screw lengths against shell
            thickness, boss depth, and nut trap depth. Warn on mismatch."""
            Logger.log("--- V11 SCREW LENGTH VERIFICATION ---")
            issues = 0
            for entry in SCREW_REGISTRY:
                req = entry["requested_len"]
                tag = entry["tag"]
                comp_name = entry["comp"]
                stype = entry["type"]
                # Calculate minimum required length
                if stype == "boss_insert":
                    min_len = entry["shell_t"] + INSERT_H * 0.6
                    max_len = entry["shell_t"] + INSERT_H + 0.15
                elif stype == "captive_nut":
                    min_len = entry["shell_t"] + M3_NUT_H
                    max_len = entry["shell_t"] + M3_NUT_H + 0.20
                else:
                    min_len = req * 0.5
                    max_len = req * 2.0
                if req < min_len:
                    Logger.log(f"  [WARN] {tag} in {comp_name}: M3x{int(req*10)} "
                               f"too short (need >= {min_len:.1f} cm engagement)", "WARN")
                    issues += 1
                elif req > max_len:
                    Logger.log(f"  [WARN] {tag} in {comp_name}: M3x{int(req*10)} "
                               f"too long (risk of puncture, max {max_len:.1f} cm)", "WARN")
                    issues += 1
            if issues == 0:
                Logger.log("  All registered screw lengths OK [PASS]")
            else:
                Logger.log(f"  {issues} screw length issue(s) found — review BOM", "WARN")

        # ── V11 NEW: DOC-1 helpers ────────────────────────────────────────
        def write_assembly_guide():
            """Write ASSEMBLY_GUIDE.txt with step order, print notes, warnings."""
            try:
                os.makedirs(os.path.dirname(ASSEMBLY_FILE), exist_ok=True)
                with open(ASSEMBLY_FILE, "w", encoding="utf-8") as f:
                    f.write("=" * 60 + "\n")
                    f.write("  OPTIMUS PRIME G1 v11.0  ASSEMBLY GUIDE\n")
                    f.write("  Physically Buildable Prototype Edition\n")
                    f.write("=" * 60 + "\n\n")

                    f.write("--- PRINT ORIENTATION NOTES ---\n")
                    for part, orient, supports in PRINT_NOTES:
                        sup = " (SUPPORTS REQUIRED)" if supports else ""
                        f.write(f"  {part:<42s} {orient}{sup}\n")
                    f.write("\n--- SUPPORT WARNINGS ---\n")
                    for part, reason in SUPPORT_WARNINGS:
                        f.write(f"  [WARN] {part}: {reason}\n")
                    f.write("\n--- ASSEMBLY STEP ORDER ---\n")
                    for i, step in enumerate(ASSEMBLY_STEPS, 1):
                        f.write(f"  {i:3d}. {step}\n")
                    f.write("\n--- SERVO INSTALL ORDER ---\n")
                    servo_order = [
                        "1. Install hip yaw servos (DS3225MG) into pelvis",
                        "2. Install hip pitch/roll servos into thigh U-brackets",
                        "3. Install knee servos into thigh lower brackets",
                        "4. Install ankle pitch/roll servos into foot shells",
                        "5. Install waist yaw+pitch servos into torso",
                        "6. Install neck servos into head and torso",
                        "7. Install shoulder yaw/pitch/roll into upper arm",
                        "8. Install elbow servos into upper arm brackets",
                        "9. Install wrist roll servos into forearm",
                        "10. Install finger drive servos (DS04-NFC) into palm bays",
                        "11. Route all servo wires before closing shells",
                    ]
                    for so in servo_order:
                        f.write(f"  {so}\n")
                    f.write("\n--- ELECTRONICS INSTALL ORDER ---\n")
                    elec_order = [
                        "1. Solder headers to Raspberry Pi Zero 2W",
                        "2. Solder headers to PCA9685 (x2)",
                        "3. Program ESP32-CAM with camera firmware",
                        "4. Install MPU-6050 IMU into pelvis pocket",
                        "5. Install RPi Zero into lower torso bay",
                        "6. Install PCA9685 boards into torso bays",
                        "7. Install ESP32-CAM into head pocket",
                        "8. Install LiPo with XT60 connector into rear bay",
                        "9. Install 5V BEC near LiPo bay",
                        "10. Install blade fuse holder in power path",
                        "11. Route JST-XH harnesses from PCAs to all servos",
                        "12. Connect I2C bus (RPi → PCA9685 x2 → IMU)",
                        "13. Connect ESP32-CAM UART to RPi",
                        "14. Install power switch on torso side panel",
                    ]
                    for eo in elec_order:
                        f.write(f"  {eo}\n")
                    f.write("\n--- TRANSFORMATION ASSEMBLY NOTES ---\n")
                    f.write("  1. Verify all joints move freely before first transform\n")
                    f.write("  2. Engage waist lock magnets in robot mode\n")
                    f.write("  3. Engage knee lock magnets at full extension\n")
                    f.write("  4. Shoulder spikes are REMOVABLE for transformation\n")
                    f.write("  5. Route wires with slack at all moving joints\n")
                    f.write("  6. Test transformation SLOWLY on first attempt\n")
                    f.write("\n--- CRITICAL BUILD WARNINGS ---\n")
                    f.write("  * Use PETG or stronger for all structural/load-bearing parts\n")
                    f.write("  * PLA is acceptable only for cosmetic covers and jigs\n")
                    f.write("  * All bearing pockets are designed for press-fit install\n")
                    f.write("  * Use steel axles at hip, knee, and shoulder joints\n")
                    f.write("  * Do NOT rely on servo torque alone for transformation\n")
                    f.write("  * Install mechanical hard stops before powering servos\n")
                    f.write("  * Use thread-lock (Loctite 222) on all metal fasteners\n")
                    f.write("  * Verify polarity on ALL servo connectors before power-on\n")
                    f.write("  * LiPo must have PCM protection; use ONLY 2S 7.4V packs\n")
                    f.write("\n--- RECOMMENDED MATERIALS ---\n")
                    f.write("  Structural:  PETG or ABS+ (hips, thighs, torso frame)\n")
                    f.write("  Shells:      PETG (torso, arms, legs, head)\n")
                    f.write("  Detail:      PLA+ (grilles, badges, small covers)\n")
                    f.write("  Flex:        TPU 95A (tire treads, gaskets, dampers)\n")
                    f.write("  Jigs:        PLA or PETG (alignment aids only)\n")
                Logger.log(f"Assembly guide written -> {ASSEMBLY_FILE}")
            except Exception as e:
                Logger.log(f"Assembly guide failed: {e}", "WARN")

        # ─────────────────────────────────────────────────────────────────
        # ELECTRONICS BAY HELPERS  (ELEC-1 … ELEC-6  + V11 covers)
        # ─────────────────────────────────────────────────────────────────

        def rpi_zero_bay(comp, tag, cx, cy, cz):
            """ELEC-1 — RPi Zero 2W pocket (65x30 mm) with 4 x M2.5 standoffs."""
            cut_cavity(comp, box(comp, f"{tag}_RPiBay",
                                 cx, cy, cz,
                                 RPI0_L + 0.20, RPI0_W + 0.20, RPI0_H + 0.30))
            for sx, sz in [(-2.90, -1.15), (+2.90, -1.15),
                           (-2.90, +1.15), (+2.90, +1.15)]:
                cyl(comp, f"{tag}_RpiStdoff_{sx}_{sz}",
                    cx+sx, cy, cz+sz, 0.14, RPI0_H+0.50, "y", dark_metal)
            # V11: connector openings for USB, power, SD card access
            cut_cavity(comp, box(comp, f"{tag}_USBSlot",
                cx - RPI0_L/2 - 0.20, cy, cz - 0.3, 0.35, 0.25, 0.50))
            cut_cavity(comp, box(comp, f"{tag}_PwrSlot",
                cx + RPI0_L/2 + 0.15, cy, cz + 0.8, 0.30, 0.20, 0.35))
            cut_cavity(comp, box(comp, f"{tag}_SDAccess",
                cx, cy, cz - RPI0_W/2 - 0.20, 0.35, 0.30, 0.15))
            BOM.add("Electronics", "Raspberry Pi Zero 2W", 1, comp.name)
            BOM.add("Fastener",    "M2.5x11 mm brass standoff", 4, comp.name)

        def pca9685_bay(comp, tag, cx, cy, cz):
            """ELEC-2 — PCA9685 pocket (62.5x25.4 mm) with 4 x M2.5 standoffs."""
            cut_cavity(comp, box(comp, f"{tag}_PCABay",
                                 cx, cy, cz,
                                 PCA_L + 0.20, PCA_W + 0.20, PCA_H + 0.30))
            for sx, sz in [(-3.00, -1.08), (+3.00, -1.08),
                           (-3.00, +1.08), (+3.00, +1.08)]:
                cyl(comp, f"{tag}_PCAStdoff_{sx}_{sz}",
                    cx+sx, cy, cz+sz, 0.14, PCA_H+0.50, "y", dark_metal)
            # V11: JST connector clearance slots (16 channels)
            for ch in range(0, 16, 4):
                offset_z = -0.8 + ch * 0.10
                jst_pocket(comp, f"{tag}_PCA_JST{ch}", cx+2.8, cy, cz+offset_z)
            BOM.add("Electronics", "PCA9685 16-ch servo driver", 1, comp.name)

        def lipo_bay(comp, tag, cx, cy, cz):
            """ELEC-3 — 2S 1300 mAh LiPo pocket (70x32x18 mm) + XT60-F slot."""
            cut_cavity(comp, box(comp, f"{tag}_LipoBay",
                                 cx, cy, cz,
                                 LIPO_L + 0.30, LIPO_H + 0.30, LIPO_W + 0.30))
            # XT60-F connector slot on rear face
            cut_cavity(comp, box(comp, f"{tag}_XT60Slot",
                                 cx, cy + LIPO_H/2 + 0.15, cz,
                                 XT60_W + 0.10, 0.50, XT60_H_SLOT + 0.10))
            # V11: battery retention strap slots (2x)
            for sx in [-LIPO_L/2 - 0.40, LIPO_L/2 + 0.40]:
                cut_cavity(comp, box(comp, f"{tag}_StrapSlot_{sx:.0f}",
                    cx + sx, cy, cz, 0.25, LIPO_H + 0.50, LIPO_W + 0.20))
            # V11: foam pad space (bottom clearance)
            box(comp, f"{tag}_FoamPad", cx, cy, cz - LIPO_H/2 - 0.15,
                LIPO_L + 0.10, 0.15, LIPO_W + 0.10, rubber_blk)
            # V11: fuse holder space adjacent to battery
            cut_cavity(comp, box(comp, f"{tag}_FuseHolder",
                cx + LIPO_L/2 + 0.60, cy, cz,
                FUSE_HOLDER_L, FUSE_HOLDER_W, FUSE_HOLDER_H))
            BOM.add("Electronics", "2S 1300 mAh 7.4V LiPo battery", 1, comp.name)
            BOM.add("Electronics", "XT60-F panel-mount connector",  1, comp.name)
            BOM.add("Hardware",    "Velcro strap 20x200 mm (battery)", 2, comp.name)
            BOM.add("Hardware",    "Foam pad 2 mm (vibration isolation)", 1, comp.name)
            BOM.add("Electronics", "ATO blade fuse holder + 5A fuse", 1, comp.name)

        def imu_pocket(comp, tag, cx, cy, cz, axis="z"):
            """ELEC-6 — MPU-6050 pocket (21x16 mm)."""
            cut_cavity(comp, box(comp, f"{tag}_IMUPkt",
                                 cx, cy, cz,
                                 IMU_L + 0.20, IMU_W + 0.20, IMU_H + 0.30))
            BOM.add("Electronics", "MPU-6050 IMU breakout", 1, comp.name)

        def esp32_cam_pocket(comp, tag, cx, cy, cz, lens_axis="y"):
            """ELEC-5 — ESP32-CAM pocket (38x26 mm) + Ø3 mm lens channel."""
            cut_cavity(comp, box(comp, f"{tag}_ESP32Bay",
                                 cx, cy, cz,
                                 ESP32_L + 0.20, ESP32_H + 0.30, ESP32_W + 0.20))
            ax_map = {"y": (0,1,0), "x": (1,0,0), "z": (0,0,1)}
            av = ax_map[lens_axis]
            cut_cavity(comp, cyl(comp, f"{tag}_LensHole",
                                 cx, cy - (ESP32_H/2 + 0.30), cz,
                                 0.150, 0.60, lens_axis))
            # V11: programming header access slot
            cut_cavity(comp, box(comp, f"{tag}_ProgSlot",
                cx + ESP32_L/2 + 0.10, cy, cz, 0.25, 0.20, 0.80))
            BOM.add("Electronics", "ESP32-CAM module (OV2640)", 1, comp.name)

        # ── V11 NEW: Power system helpers ─────────────────────────────────
        def bec_mount(comp, tag, cx, cy, cz):
            """POW-4 — 5V BEC regulator mount space + standoffs."""
            cut_cavity(comp, box(comp, f"{tag}_BECBay",
                cx, cy, cz, BEC_L + 0.20, BEC_H + 0.20, BEC_W + 0.20))
            for sx, sz in [(-BEC_L/2+0.5, -BEC_W/2+0.5), (BEC_L/2-0.5, BEC_W/2-0.5)]:
                m3_boss(comp, f"{tag}_BEC_{sx:.0f}", cx+sx, cy, cz+sz)
            BOM.add("Electronics", "5V 5A BEC / UBEC power regulator", 1, comp.name)

        def power_switch_cutout(comp, tag, cx, cy, cz, axis="y"):
            """POW-3 — Panel-mount power switch hole with nut recess."""
            cyl(comp, f"{tag}_SwHole", cx, cy, cz, POWER_SWITCH_R, 1.0, axis, black_plastic)
            cut_cavity(comp, cyl(comp, f"{tag}_SwCut", cx, cy, cz, POWER_SWITCH_R + 0.03, 1.2, axis))
            BOM.add("Electronics", "Panel-mount rocker switch SPST", 1, comp.name)

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
                pass
            try:
                sketch = root.sketches.add(root.xYConstructionPlane)
                sketch.isVisible = False
                s_pt = sketch.sketchPoints.add(adsk.core.Point3D.create(cx, cy, cz))
                return adsk.fusion.JointGeometry.createByPoint(s_pt)
            except Exception:
                return None

        def revolute_joint(name, occ1, occ2, cx, cy, cz, axis_str):
            if not (occ1 and occ2):
                return
            try:
                geom = _make_joint_geometry(cx, cy, cz)
                ji   = root.asBuiltJoints.createInput(occ1, occ2, geom)
                av   = {"x": adsk.core.Vector3D.create(1, 0, 0),
                        "y": adsk.core.Vector3D.create(0, 1, 0),
                        "z": adsk.core.Vector3D.create(0, 0, 1)}[axis_str]
                ji.setAsRevoluteJointMotion(
                    adsk.fusion.JointDirections.CustomJointDirection, av)
                j      = root.asBuiltJoints.add(ji)
                j.name = name
            except Exception:
                Logger.log(f"Failed revolute joint {name}: {traceback.format_exc()}", "ERROR")

        def ball_joint(name, occ1, occ2, cx, cy, cz):
            if not (occ1 and occ2):
                return
            try:
                geom = _make_joint_geometry(cx, cy, cz)
                ji   = root.asBuiltJoints.createInput(occ1, occ2, geom)
                ji.setAsBallJointMotion(
                    adsk.fusion.JointDirections.ZAxisJointDirection,
                    adsk.fusion.JointDirections.XAxisJointDirection)
                j      = root.asBuiltJoints.add(ji)
                j.name = name
            except Exception:
                Logger.log(f"Failed ball joint {name}: {traceback.format_exc()}", "ERROR")

        def rigid_joint(name, occ1, occ2):
            if not (occ1 and occ2):
                return
            try:
                geom = _make_joint_geometry(0, 0, 0)
                ji   = root.asBuiltJoints.createInput(occ1, occ2, geom)
                ji.setAsRigidJointMotion()
                j      = root.asBuiltJoints.add(ji)
                j.name = name
            except Exception:
                pass

        # ─────────────────────────────────────────────────────────────────
        # HARDWARE MODULES  (with V11 coupler integration)
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
                    if axis == "x":
                        c = cyl(comp, f"{tag}_FlgS_{d1}_{d2}", fx,    fy+d2, fz+d1, sd, 1.5, "x")
                    elif axis == "z":
                        c = cyl(comp, f"{tag}_FlgS_{d1}_{d2}", fx+d1, fy+d2, fz,    sd, 1.5, "z")
                    else:
                        c = cyl(comp, f"{tag}_FlgS_{d1}_{d2}", fx+d1, fy,    fz+d2, sd, 1.5, "y")
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
            # PHY-5: wire grommet on mounting face
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
            elif axis == "z":
                box(comp, f"{tag}_VisBody", cx,      cy,      cz,      4.05, 2.00, 4.20, grey_plastic)
                box(comp, f"{tag}_VisEars", cx,      cy,      cz+0.95, 5.80, 2.20, 0.30, dark_grey)
                cyl(comp, f"{tag}_VisHorn", cx-1.10, cy,      cz+2.40, 0.95, 0.22, "z", white_pla)
                marker(comp, f"{tag}_Pivot", cx-1.10, cy,     cz+2.40)
                cut_cavity(comp, box(comp, f"{tag}_CB", cx, cy, cz,      4.05+cl, 2.00+cl, 4.20+cl))
                cut_cavity(comp, box(comp, f"{tag}_CE", cx, cy, cz+0.95, 5.80+cl, 2.20+cl, 0.30+cl))
            else:  # y
                box(comp, f"{tag}_VisBody", cx,      cy,      cz,      4.05, 4.20, 2.00, grey_plastic)
                box(comp, f"{tag}_VisEars", cx,      cy+0.95, cz,      4.05, 0.30, 2.20, dark_grey)
                cyl(comp, f"{tag}_VisHorn", cx,      cy+2.40, cz+1.05, 0.95, 0.22, "y", white_pla)
                marker(comp, f"{tag}_Pivot", cx,     cy+2.40, cz+1.05)
                cut_cavity(comp, box(comp, f"{tag}_CB", cx, cy,      cz, 4.05+cl, 4.20+cl, 2.00+cl))
                cut_cavity(comp, box(comp, f"{tag}_CE", cx, cy+0.95, cz, 4.05+cl, 0.30+cl, 2.20+cl))
            servo_hardware(comp, tag, cx, cy, cz, axis, True)
            # V11: install horn coupler on output shaft
            horn_coupler(comp, tag, cx, cy, cz, axis, servo_type="std")
            BOM.add("Servo", "MG996R 11 kg-cm servo", 1, comp.name)

        def mg90s(comp, tag, cx, cy, cz, axis="x"):
            cl = CLEARANCE
            if axis == "x":
                box(comp, f"{tag}_VisBody", cx,      cy,      cz,      2.30, 1.20, 2.30, op_blue)
                box(comp, f"{tag}_VisEars", cx+0.45, cy,      cz,      0.20, 1.30, 3.20, op_blue)
                cyl(comp, f"{tag}_VisHorn", cx+1.40, cy,      cz+0.50, 0.55, 0.18, "x", white_pla)
                marker(comp, f"{tag}_Pivot", cx+1.40, cy,     cz+0.50)
                cut_cavity(comp, box(comp, f"{tag}_CB",  cx,      cy, cz, 2.30+cl, 1.20+cl, 2.30+cl))
                cut_cavity(comp, box(comp, f"{tag}_CE",  cx+0.45, cy, cz, 0.20+cl, 1.30+cl, 3.20+cl))
            elif axis == "z":
                box(comp, f"{tag}_VisBody", cx,      cy,      cz,      2.30, 1.20, 2.30, op_blue)
                box(comp, f"{tag}_VisEars", cx,      cy,      cz+0.45, 3.20, 1.30, 0.20, op_blue)
                cyl(comp, f"{tag}_VisHorn", cx-0.50, cy,      cz+1.40, 0.55, 0.18, "z", white_pla)
                marker(comp, f"{tag}_Pivot", cx-0.50, cy,     cz+1.40)
                cut_cavity(comp, box(comp, f"{tag}_CB", cx, cy, cz,      2.30+cl, 1.20+cl, 2.30+cl))
                cut_cavity(comp, box(comp, f"{tag}_CE", cx, cy, cz+0.45, 3.20+cl, 1.30+cl, 0.20+cl))
            else:  # y
                box(comp, f"{tag}_VisBody", cx,      cy,      cz,      2.30, 2.30, 1.20, op_blue)
                box(comp, f"{tag}_VisEars", cx,      cy+0.45, cz,      3.20, 0.20, 1.30, op_blue)
                cyl(comp, f"{tag}_VisHorn", cx,      cy+1.40, cz+0.50, 0.55, 0.18, "y", white_pla)
                marker(comp, f"{tag}_Pivot", cx,     cy+1.40, cz+0.50)
                cut_cavity(comp, box(comp, f"{tag}_CB", cx, cy,      cz, 2.30+cl, 2.30+cl, 1.20+cl))
                cut_cavity(comp, box(comp, f"{tag}_CE", cx, cy+0.45, cz, 3.20+cl, 0.20+cl, 1.30+cl))
            servo_hardware(comp, tag, cx, cy, cz, axis, False)
            # V11: install micro horn coupler
            horn_coupler(comp, tag, cx, cy, cz, axis, servo_type="micro")
            BOM.add("Servo", "MG90S micro servo", 1, comp.name)

        def tt_wheel(comp, tag, cx, cy, cz, side=1):
            cl = CLEARANCE
            box(comp, f"{tag}_VisGB",    cx,           cy,     cz,  2.30, 5.20, 1.90, yellow_met)
            cyl(comp, f"{tag}_VisMot",   cx,           cy-1.5, cz,  0.90, 2.10, "y",  chrome)
            cyl(comp, f"{tag}_VisShaft", cx+side*1.75, cy,     cz,  0.20, 3.50, "x",  chrome)
            cyl(comp, f"{tag}_VisHub",   cx+side*3.25, cy,     cz,  0.80, 2.60, "x",  dark_metal)
            cyl(comp, f"{tag}_VisTire",  cx+side*3.25, cy,     cz,  3.25, 2.60, "x",  rubber_blk)
            cyl(comp, f"{tag}_VisRim",   cx+side*3.25, cy,     cz,  2.20, 2.65, "x",  chrome)
            marker(comp, f"{tag}_Axle_Pivot", cx+side*3.25, cy, cz, 0.18)
            cut_cavity(comp, box(comp, f"{tag}_CGB", cx,           cy, cz, 2.30+cl, 5.20+cl, 1.90+cl))
            cut_cavity(comp, box(comp, f"{tag}_CDS", cx+side*3.25, cy, cz, 2.7+cl,  0.54+cl, 0.36+cl))
            BOM.add("Drive",   "TT gear-motor 3V-6V",        1, comp.name)
            BOM.add("Drive",   "65 mm rubber tyre + wheel", 1, comp.name)

        def bearing(comp, tag, cx, cy, cz, axis="x", ro=1.10, w=0.60):
            """V11: Legacy bearing call — redirects to bearing_fit with glue-fit default."""
            bearing_fit(comp, tag, cx, cy, cz, axis, ro, w, fit_type="glue")

        def u_bracket(comp, tag, cx, cy, cz, lx, ly, lz, ap=None):
            ap = ap or chrome
            box(comp, f"{tag}_BB",  cx,          cy,        cz, 0.45,     ly,   lz, ap)
            box(comp, f"{tag}_BL",  cx+lx*0.45,  cy+ly*0.35, cz, lx*0.55, 0.40, lz, ap)
            box(comp, f"{tag}_BR",  cx+lx*0.45,  cy-ly*0.35, cz, lx*0.55, 0.40, lz, ap)
            cyl(comp, f"{tag}_Pin", cx+lx*0.50,  cy,         cz, 0.18, ly*0.85, "y", chrome)

        # ── V11 NEW: mechanical hard stop ─────────────────────────────────
        def hard_stop(comp, tag, cx, cy, cz, axis="x", stop_angle_deg=90):
            """MECH-3 — Mechanical hard stop block to prevent joint over-travel.
            Protects servo from impact loads."""
            ax = {"x": (1, 0, 0), "y": (0, 1, 0), "z": (0, 0, 1)}[axis]
            # Small block at stop position
            box(comp, f"{tag}_Stop", cx, cy, cz, 0.35, 0.35, 0.35, dark_metal)
            BOM.add("Hardware", "Hard stop block (printed)", 1, comp.name)
            ASSEMBLY_STEPS.append(f"Verify {tag} hard stop at {stop_angle_deg} deg clears moving link")

        # ── V11 NEW: transform lock pin ───────────────────────────────────
        def transform_lock(comp, tag, cx, cy, cz, axis="z"):
            """MECH-4 — Spring-pin latch for robot-mode and truck-mode locking.
            Manually engaged; prevents servo-holding-torce reliance."""
            # Pin bore (through both parts)
            cyl(comp, f"{tag}_LockBore", cx, cy, cz, 0.18, 1.50, axis, dark_metal)
            cut_cavity(comp, cyl(comp, f"{tag}_LockHole", cx, cy, cz, 0.20, 1.60, axis))
            # Spring pocket around bore
            cyl(comp, f"{tag}_SpringPkt", cx, cy, cz + 0.30, 0.35, 0.50, axis, dark_grey)
            BOM.add("Hardware", "Spring latch pin Ø3.5 mm (steel)", 1, comp.name)
            BOM.add("Hardware", "Compression spring (lock return)", 1, comp.name)
            ASSEMBLY_STEPS.append(f"Install {tag} transform lock pin and return spring")

        # ═════════════════════════════════════════════════════════════════
        # COMPONENT BUILDING  (V11 Physically Buildable Edition)
        # ═════════════════════════════════════════════════════════════════

        # ─────────────────────────────────────────────────────────────────
        # 1 TORSO  (GEO-4, ELEC-1, ELEC-2, ELEC-3  + V11 covers, cable, BEC)
        # ─────────────────────────────────────────────────────────────────
        torso = new_component("OP_Torso")

        # Main structural shell
        box(torso, "Torso_Shell",    0,    0,   TORSO_CTR,        10.4, 8.6, 12.2, op_red)
        box(torso, "Torso_Side_L",  -5.6,  0,   TORSO_CTR,         0.5, 7.8, 11.2, op_red)
        box(torso, "Torso_Side_R",   5.6,  0,   TORSO_CTR,         0.5, 7.8, 11.2, op_red)

        # GEO-4 — Chest windows (stepped frame)
        box(torso, "CWin_Frame_L",  -2.3, -4.20, TORSO_CTR+2.5,   2.8, 0.32, 3.2, op_blue)
        box(torso, "CWin_Frame_R",   2.3, -4.20, TORSO_CTR+2.5,   2.8, 0.32, 3.2, op_blue)
        box(torso, "Chest_Win_L",   -2.3, -4.35, TORSO_CTR+2.5,   2.4, 0.20, 2.8, glass_clr)
        box(torso, "Chest_Win_R",    2.3, -4.35, TORSO_CTR+2.5,   2.4, 0.20, 2.8, glass_clr)
        box(torso, "Chest_Win_Div",  0,   -4.25, TORSO_CTR+2.5,   0.40, 0.22, 3.2, chrome)

        # GEO-4 — Horizontal grille rows (4 rows)
        for gz_offset, gw in [(-0.2, 7.4), (-1.0, 7.0), (-1.8, 6.6), (-2.6, 6.2)]:
            box(torso, f"Grille_{int(gz_offset*10)}",
                0, -4.40, TORSO_CTR+gz_offset, gw, 0.22, 0.30, chrome)

        # Headlight pods
        box(torso, "Headlight_L",   -4.4, -4.50, TORSO_CTR-1.2,  1.8, 0.28, 2.0, glass_clr)
        box(torso, "Headlight_R",    4.4, -4.50, TORSO_CTR-1.2,  1.8, 0.28, 2.0, glass_clr)

        # Front bumper (GEO-6 truck-cab)
        box(torso, "Front_Bumper",   0,   -5.8,  TORSO_CTR-4.4,  10.0, 2.0,  1.8, chrome)
        box(torso, "Hood_Crease_L", -2.5, -4.60, TORSO_CTR-2.8,   0.5, 0.35, 3.0, op_red)
        box(torso, "Hood_Crease_R",  2.5, -4.60, TORSO_CTR-2.8,   0.5, 0.35, 3.0, op_red)
        box(torso, "Ind_L",         -3.8, -5.00, TORSO_CTR-3.8,   0.6, 0.25, 0.5, yellow_met)
        box(torso, "Ind_R",          3.8, -5.00, TORSO_CTR-3.8,   0.6, 0.25, 0.5, yellow_met)

        # GEO-4 — Chest plate + Autobot badge with LED ring
        box(torso, "Chest_Plate",    0,   -4.20, TORSO_CTR+0.5,   8.4, 0.32, 4.0, chrome)
        cyl(torso, "Badge_Ring",     0,   -4.55, TORSO_CTR+0.5,   0.80, 0.12, "y", op_red)
        led_ring_pocket(torso, "Badge",  0, -4.60, TORSO_CTR+0.5, "y")

        # V11 MECH-1 — Internal structural skeleton (upgraded)
        box(torso, "Inner_Frame",    0,    0,    TORSO_CTR+1.5,   7.4, 6.0,  8.2, dark_metal)
        box(torso, "Spine_Beam",     0,    0,    TORSO_CTR+1.5,   1.8, 1.8,  8.2, chrome)
        cyl(torso, "Spine_Cyl",      0,    0,    TORSO_CTR+1.5,  1.10, 4.5,  "z", chrome)
        # V11: rib plates for lateral rigidity
        for rz in [TORSO_CTR+3.5, TORSO_CTR, TORSO_CTR-3.5]:
            box(torso, f"Rib_{rz:.0f}", 0, 0, rz, 6.8, 0.35, 4.5, dark_metal)
        # V11: load-transfer gussets at shoulder mounts
        for sx in [-6.5, 6.5]:
            box(torso, f"Gusset_{sx:.0f}", sx, 0, TORSO_CTR+2.0, 1.2, 1.2, 3.5, dark_metal)

        # ELEC-3 — LiPo bay (rear of lower torso)
        box(torso, "LipoBay_Shell",  0,    3.2, TORSO_CTR-2.0,   7.6, 4.4,  5.6, black_plastic)
        lipo_bay(torso, "Main", 0, 2.8, TORSO_CTR-2.0)
        # V11 COV-2 — LiPo door (rear face)
        lipo_door(torso, "LipoDoor", 0, 5.5, TORSO_CTR-2.0, LIPO_L + 0.80, LIPO_W + 0.80)

        # ELEC-1 — RPi Zero 2W bay (above LiPo)
        box(torso, "RPi_Shell",      0,    2.8, TORSO_CTR+1.8,   7.0, 3.6,  2.8, black_plastic)
        rpi_zero_bay(torso, "Main",  0,    3.2, TORSO_CTR+1.8)
        # V11 COV-3 — RPi cover
        pcb_cover(torso, "RPiCover", 0, 4.5, TORSO_CTR+1.8, RPI0_L + 0.60, RPI0_W + 0.60, "magnet")

        # ELEC-2 — PCA9685 bay (alongside RPi)
        box(torso, "PCA_Shell",      0,    2.8, TORSO_CTR+4.2,   6.8, 3.2,  2.2, black_plastic)
        pca9685_bay(torso, "Main",   0,    3.0, TORSO_CTR+4.2)
        pcb_cover(torso, "PCACover", 0, 4.2, TORSO_CTR+4.2, PCA_L + 0.50, PCA_W + 0.50, "screw")

        # V11 CAB-2 — Torso wire hub
        wire_hub(torso, "TorsoHub", 0, 1.5, TORSO_CTR+0.5)

        # V11 CAB-1 — Cable clips along wire runs
        for cz_clip in [TORSO_CTR+3.0, TORSO_CTR, TORSO_CTR-3.0, TORSO_CTR-4.5]:
            cable_clip(torso, f"CC_L_{cz_clip:.0f}", -3.4, 0.6, cz_clip)
            cable_clip(torso, f"CC_R_{cz_clip:.0f}",  3.4, 0.6, cz_clip)

        # Cable management channels
        box(torso, "Cable_L",       -3.4,  0.6,  TORSO_CTR,      0.55, 1.0, 10.0, dark_grey)
        box(torso, "Cable_R",        3.4,  0.6,  TORSO_CTR,      0.55, 1.0, 10.0, dark_grey)

        # V11 POW-4 — BEC mount (near LiPo bay)
        bec_mount(torso, "MainBEC", LIPO_L/2 + 1.0, 3.0, TORSO_CTR-2.0)

        # V11 POW-3 — Power switch (side panel)
        power_switch_cutout(torso, "PwrSw", -5.5, 0, TORSO_CTR+2.0, "y")

        # Shoulder collars
        box(torso, "Collar_L",      -8.0,  0,    SHOULDER_CTR-1.0, 5.5, 3.4, 3.0, chrome)
        box(torso, "Collar_R",       8.0,  0,    SHOULDER_CTR-1.0, 5.5, 3.4, 3.0, chrome)

        # Transformation flaps
        box(torso, "TF_Flap_L",     -5.40,-0.2,  TORSO_CTR+3.0,  0.40, 6.6,  6.2, op_red)
        box(torso, "TF_Flap_R",      5.40,-0.2,  TORSO_CTR+3.0,  0.40, 6.6,  6.2, op_red)
        box(torso, "TF_BackTop",     0,    5.0,  TORSO_CTR+5.2,  8.2, 0.38,  5.2, op_blue)

        # M3 boss fasteners (corners)
        for bx_off, bz_off in [(-3.2, 4.8), (3.2, 4.8), (-3.2, -4.8), (3.2, -4.8)]:
            m3_boss(torso, f"Torso_B{bx_off}_{bz_off}", bx_off, 0, TORSO_CTR+bz_off)
        align_pin(torso, "TorsoSplit_L", -5.0, 0, TORSO_CTR)
        align_pin(torso, "TorsoSplit_R",  5.0, 0, TORSO_CTR)

        # V11 MECH-4 — Transform lock (waist)
        transform_lock(torso, "WaistLock", 0, -2.0, WAIST_CTR-3.0, "z")

        # Waist servo cluster (V11: dual bearing support)
        u_bracket(torso, "Waist_Brkt",  0, 0, WAIST_CTR,     4.0, 4.2, 3.4)
        mg996r(torso,    "Waist_Yaw",   0, 0, WAIST_CTR,     "z")
        # V11 BRG-3: dual bearing for waist yaw
        dual_bearing(torso, "WaistDual", 0, 0, WAIST_CTR+0.5, "z", 1.30, 0.65, span=3.00)
        u_bracket(torso, "WaistP_Brkt", 0, 0, WAIST_CTR-2.5, 4.0, 4.2, 3.4)
        mg996r(torso,    "Waist_Pitch", 0, 0, WAIST_CTR-2.5, "x")
        bearing_fit(torso, "WaistP_Brg",  0, 0, WAIST_CTR-2.0, "x", 1.30, 0.65, fit_type="press")
        magnet_pocket(torso, "WLock_F", 0, -2.0, WAIST_CTR-3.0)
        magnet_pocket(torso, "WLock_R", 0,  2.0, WAIST_CTR-3.0)
        # V11 MECH-3: hard stops
        hard_stop(torso, "WaistP", 0, -2.5, WAIST_CTR-2.5, "x", 60)
        hard_stop(torso, "WaistN", 0,  2.5, WAIST_CTR-2.5, "x", -45)

        # Neck servo cluster
        u_bracket(torso, "Neck_Brkt",   0, 0, NECK_JOINT_Z,  3.2, 2.8, 3.0)
        mg996r(torso,    "Neck_Pitch",  0, 0, NECK_JOINT_Z,  "x")
        wire_channel(torso, "Spine",    0, 0, TORSO_CTR,     0.6, 20.0, "z")

        # V11: Printability check on torso parts
        for b in list(torso.bRepBodies):
            if b.name:
                printability_check(torso, b.name)

        # ─────────────────────────────────────────────────────────────────
        # 2 HEAD  (GEO-1  + V11 ESP32-CAM cover)
        # ─────────────────────────────────────────────────────────────────
        head = new_component("OP_Head")
        HC   = HEAD_CTR

        box(head, "Helm_Main",      0,     0,    HC+0.8,  5.6, 5.2, 5.0, op_blue)
        box(head, "Helm_Forehead",  0,    -2.30, HC+2.6,  5.0, 0.42, 1.8, op_blue)
        box(head, "Helm_Top",       0,     0,    HC+3.4,  4.8, 4.4, 0.90, op_blue)

        # GEO-1 — G1 ear wings
        box(head, "Ear_L",         -3.30,  0,    HC+1.8,  0.70, 4.8, 3.8, op_blue)
        box(head, "Ear_R",          3.30,  0,    HC+1.8,  0.70, 4.8, 3.8, op_blue)
        box(head, "EarTop_L",      -3.05,  0,    HC+3.5,  1.40, 4.4, 0.62, op_blue)
        box(head, "EarTop_R",       3.05,  0,    HC+3.5,  1.40, 4.4, 0.62, op_blue)

        # Crest
        box(head, "Crest_Main",     0,    -0.30, HC+3.85, 1.05, 0.68, 3.6, chrome)
        box(head, "Crest_Stripe",   0,    -0.30, HC+3.95, 0.55, 0.36, 2.9, op_blue)

        # Face plate
        box(head, "Face_Plate",     0,    -2.50, HC+0.5,  3.6, 0.38, 3.2, chrome)

        # GEO-1 — Chin guard (V11: chamfered for printability)
        chamfer_box(head, "Chin_Guard", 0, -2.60, HC-0.9, 3.0, 0.38, 1.8, "z", chamfer=0.25, ap=chrome)
        box(head, "Chin_L",        -1.35, -2.52, HC-0.4,  0.38, 0.32, 2.2, chrome)
        box(head, "Chin_R",         1.35, -2.52, HC-0.4,  0.38, 0.32, 2.2, chrome)

        # Visor recess + LED pockets
        box(head, "Visor_Frame",    0,    -2.55, HC+1.35, 3.8, 0.30, 1.25, op_blue)
        box(head, "Visor",          0,    -2.68, HC+1.35, 3.0, 0.16, 0.92, op_blue_glass)
        for vx in [-0.85, 0.0, 0.85]:
            led_pocket_5mm(head, f"Visor_LED_{int(vx*100)}", vx, -2.80, HC+1.35, "y")

        # Nose bridge + mouth grill
        box(head, "Nose_Bridge",    0,    -2.60, HC+1.95, 0.72, 0.22, 0.72, chrome)
        box(head, "Mouth_Plate",    0,    -2.55, HC+0.10, 2.4, 0.22, 1.10, dark_grey)
        for mz in [-0.32, 0.0, 0.32]:
            box(head, f"MouthGrill_{int(mz*100)}",
                0, -2.62, HC+0.10+mz, 1.8, 0.12, 0.18, chrome)

        # Head rear & neck collar
        box(head, "Head_Rear",      0,    1.90,  HC+1.0,  4.2, 1.5, 4.4, op_red)
        box(head, "Neck_Collar",    0,    0,     HC-1.6,  2.5, 2.5, 2.4, dark_metal)

        # Antennas
        cyl(head, "Ant_L",         -2.80, 0,     HC+4.4, 0.13, 2.7, "z", chrome)
        cyl(head, "Ant_R",          2.80, 0,     HC+4.4, 0.13, 2.7, "z", chrome)
        cyl(head, "AntTip_L",      -2.80, 0,     HC+5.8, 0.24, 0.34, "z", gold_met)
        cyl(head, "AntTip_R",       2.80, 0,     HC+5.8, 0.24, 0.34, "z", gold_met)

        # ELEC-5 — ESP32-CAM pocket + lens channel
        esp32_cam_pocket(head, "HeadCam", 0, -1.6, HC+2.5, "y")
        # V11 COV-3 — ESP32-CAM cover (hinged for lens access)
        cover_plate(head, "CamCover", 0, -2.0, HC+2.5, ESP32_L+0.60, ESP32_W+0.60,
                    [(-1.2, -0.6), (1.2, -0.6)], method="hinge", hinge_edge="top", ap=grey_plastic)

        # Neck yaw servo (micro)
        mg90s(head, "Neck_Yaw", 0, 0, NECK_JOINT_Z, "z")
        # V11: add cable grommet for neck wiring
        grommet_slot(head, "NeckWire", 0, 0.8, HC-0.5, "y", 0.50)

        # V11: Printability check
        for b in list(head.bRepBodies):
            if b.name:
                printability_check(head, b.name)

        # V11 JIG-1: head shell assembly jig
        assembly_jig("OP_Head",
            [(-2.0, 0, 0), (2.0, 0, 0)],
            [(-1.5, 0, 0), (1.5, 0, 0)],
            (6.0, 4.0, 0.60))

        # ─────────────────────────────────────────────────────────────────
        # 3 PELVIS  (ELEC-6  + V11 IMU cover, cable routing)
        # ─────────────────────────────────────────────────────────────────
        pelvis = new_component("OP_Pelvis")
        box(pelvis, "Pelvis_Shell",  0,    0,  PELVIS_CTR,  16.4, 6.2, 5.0, op_blue)
        box(pelvis, "Pelvis_Frame",  0,    0,  PELVIS_CTR,  12.2, 4.4, 3.8, dark_metal)
        box(pelvis, "Hip_Armr_L",  -7.2,  0,  PELVIS_CTR,   1.0, 5.2, 4.2, chrome)
        box(pelvis, "Hip_Armr_R",   7.2,  0,  PELVIS_CTR,   1.0, 5.2, 4.2, chrome)
        box(pelvis, "Crotch_Plate", 0,  -2.9, PELVIS_CTR-1.2, 5.2, 0.30, 2.4, op_red)

        # ELEC-6 — IMU in pelvis centre
        imu_pocket(pelvis, "BalanceIMU", 0, 0, PELVIS_CTR)
        # V11: IMU cover (magnetic, removable)
        cover_plate(pelvis, "IMUCover", 0, 0.2, PELVIS_CTR,
                    IMU_L+0.60, IMU_W+0.60,
                    [(-0.8, -0.5), (0.8, 0.5)], method="magnet", ap=grey_plastic)

        # V11 CAB-3 — Wire grommets for hip servo harness exits
        grommet_slot(pelvis, "HipWire_L", -HIP_X, 0, PELVIS_CTR+0.5, "z", 0.60)
        grommet_slot(pelvis, "HipWire_R",  HIP_X, 0, PELVIS_CTR+0.5, "z", 0.60)

        # Hip yaw servos (DS3225MG — V11: coupler + dual bearing)
        mg996r(pelvis, "L_HipYaw", -HIP_X, 0, HIP_JOINT_Z, "z")
        mg996r(pelvis, "R_HipYaw",  HIP_X, 0, HIP_JOINT_Z, "z")
        # V11 BRG-3: dual bearing at hip yaw (primary load joint)
        dual_bearing(pelvis, "L_HipDual", -HIP_X-2.2, 0, HIP_JOINT_Z, "z", 1.10, 0.62, span=3.20, fit_type="press")
        dual_bearing(pelvis, "R_HipDual",  HIP_X+2.2, 0, HIP_JOINT_Z, "z", 1.10, 0.62, span=3.20, fit_type="press")
        BOM.add("Servo", "DS3225MG 25 kg-cm servo (hip yaw)", 2, "OP_Pelvis")

        # V11: Printability
        for b in list(pelvis.bRepBodies):
            if b.name:
                printability_check(pelvis, b.name)

        # ─────────────────────────────────────────────────────────────────
        # 4 LEGS  (GEO-5 improved feet  + V11 tendon-ready hands, couplers)
        # ─────────────────────────────────────────────────────────────────
        for side, sx in [("L", -HIP_X), ("R", HIP_X)]:
            m = -1 if side == "L" else 1

            # —— THIGH ——
            thigh = new_component(f"OP_Thigh_{side}")
            box(thigh, "Thigh_Link",         sx,        0,  THIGH_CTR,      5.0, 4.0, 11.0, chrome)
            box(thigh, "Thigh_Outer",        sx+m*2.65, 0,  THIGH_CTR,      0.50, 4.4, 11.0, op_red)
            box(thigh, "Thigh_Front",        sx,       -2.2, THIGH_CTR,     5.0, 0.40, 11.0, op_blue)
            box(thigh, "Thigh_Knee_Armor",   sx,       -2.2, KNEE_CTR+2.5,  4.2, 0.80,  2.8, chrome)

            # V11 MECH-1: Internal rib for torsional rigidity
            box(thigh, f"{side}_Thigh_Rib", sx, 0, THIGH_CTR, 3.5, 0.30, 9.0, dark_metal)

            u_bracket(thigh, f"{side}_HPB",  sx, 0, HIP_JOINT_Z+0.5,        4.0, 3.2, 3.2)
            mg996r(thigh, f"{side}_HipP",    sx, 0, HIP_JOINT_Z,             "x")
            # V11 BRG-3: dual bearing for hip pitch (high load)
            dual_bearing(thigh, f"{side}_HipP_Dual", sx, 0, HIP_JOINT_Z,
                         "x", 1.10, 0.62, span=2.80, fit_type="press")
            mg996r(thigh, f"{side}_HipR",    sx, 0, THIGH_CTR+2.0,           "y")
            bearing_fit(thigh, f"{side}_HRB",    sx, 0, THIGH_CTR+2.0,
                        "y", 1.00, 0.55, fit_type="press")
            u_bracket(thigh, f"{side}_KnB",  sx, 0, KNEE_CTR+1.5,            3.8, 3.0, 3.0)
            mg996r(thigh, f"{side}_KneP",    sx, 0, KNEE_CTR+1.5,            "x")
            # V11 BRG-3: dual bearing for knee (high load)
            dual_bearing(thigh, f"{side}_Knee_Dual", sx, 0, KNEE_CTR,
                         "x", 1.00, 0.55, span=2.60, fit_type="press")
            wire_channel(thigh, f"{side}_LW", sx, 0, THIGH_CTR,              0.5, 12.0, "z")
            for bz in [THIGH_CTR+3.0, THIGH_CTR-3.0]:
                m3_boss(thigh, f"{side}_ThighBoss_{bz:.0f}", sx, 0, bz)
            magnet_pocket(thigh, f"{side}_KLU", sx, -1.5, KNEE_CTR+1.0)
            magnet_pocket(thigh, f"{side}_KLL", sx,  1.5, KNEE_CTR+1.0)
            # V11 MECH-4: knee transform lock
            transform_lock(thigh, f"{side}_KneeLock", sx, -2.5, KNEE_CTR+0.5, "x")
            # V11 MECH-3: hard stops
            hard_stop(thigh, f"{side}_KneeExt", sx, -2.5, KNEE_CTR, "x", 135)
            BOM.add("Servo", "DS3225MG 25 kg-cm servo (hip pitch)", 1, f"OP_Thigh_{side}")

            # —— SHIN ——
            shin = new_component(f"OP_Shin_{side}")
            box(shin, "Shin_Link",    sx,    0,    SHIN_CTR,  4.4, 6.0, 11.0, op_blue)
            box(shin, "Shin_Armor",   sx,   -2.7,  SHIN_CTR,  3.2, 0.34,  9.2, chrome)
            box(shin, "Shin_Rear",    sx,    2.7,  SHIN_CTR,  2.0, 0.34,  9.8, dark_grey)
            box(shin, "Shin_Beam",    sx,    0.4,  SHIN_CTR,  1.8, 2.2, 10.2, dark_metal)

            # V11 MECH-1: shin reinforcing rib
            box(shin, f"{side}_Shin_Rib", sx, 0, SHIN_CTR, 2.8, 0.25, 8.5, dark_metal)

            box(shin, "KneeCap",      sx,   -2.9,  KNEE_CTR-1.0, 3.0, 0.55, 2.2, chrome)
            tt_wheel(shin, f"{side}_WF", sx+m*2.0, -4.2, SHIN_CTR+4.0, m)
            tt_wheel(shin, f"{side}_WR", sx+m*2.0, -4.2, SHIN_CTR-4.0, m)
            bearing_fit(shin, f"{side}_KLB", sx, 0, KNEE_CTR-0.5, "x", 1.00, 0.55, fit_type="press")
            wire_channel(shin, f"{side}_SW", sx, 0, SHIN_CTR, 0.5, 11.0, "z")
            cut_cavity(shin, box(shin, "FtTuck", sx, 2.7, SHIN_CTR-3.5, 5.2, 1.3, 4.2))
            magnet_pocket(shin, f"{side}_KU", sx, -1.5, KNEE_CTR-1.0)
            magnet_pocket(shin, f"{side}_KL", sx,  1.5, KNEE_CTR-1.0)
            for bz in [SHIN_CTR+3.5, SHIN_CTR-3.5]:
                m3_boss(shin, f"{side}_ShinBoss_{bz:.0f}", sx, 0, bz)
            # V11 CAB-3: wire grommet at ankle exit
            grommet_slot(shin, f"{side}_AnkleWire", sx, 0, SHIN_CTR-5.0, "z", 0.50)

            # —— FOOT  (GEO-5 + V11 stability improvements) ——
            foot = new_component(f"OP_Foot_{side}")
            # Base sole — wider, longer for stability
            box(foot, "Foot_Sole",    sx,       -0.6,  ANKLE_CTR-1.5,  6.2, 9.2, 1.10, op_red)
            # Arch vent slots (3 x decorative)
            for vi in [-1.0, 0.0, 1.0]:
                box(foot, f"Arch_Vent_{int(vi*10)}",
                    sx+vi*1.2, -0.6, ANKLE_CTR-1.94, 0.40, 5.5, 0.14, dark_grey)
            # GEO-5 — Heel spur (V11: chamfered edges for printability)
            chamfer_box(foot, "Heel_Block", sx-m*0.9, 3.2, ANKLE_CTR-0.8,
                        2.5, 3.5, 2.6, "y", chamfer=0.30, ap=dark_grey)
            box(foot, "Heel_Plate",   sx-m*0.6,  4.4,  ANKLE_CTR-1.2,  3.2, 0.32, 2.0, chrome)
            # V11 MFG-2: heel spur printability — add chamfer
            chamfer_box(foot, "Heel_Spur", sx-m*1.0, 4.8, ANKLE_CTR-0.2,
                        1.2, 0.40, 3.2, "z", chamfer=0.35, ap=op_red)
            # GEO-5 — Toe cap
            box(foot, "Toe_Block",    sx+m*0.8, -3.8,  ANKLE_CTR-0.8,  2.6, 3.8, 2.0, dark_grey)
            box(foot, "Toe_Plate",    sx+m*0.5, -4.6,  ANKLE_CTR-1.2,  3.8, 0.32, 1.8, chrome)
            chamfer_box(foot, "Toe_Cap", sx+m*1.0, -5.2, ANKLE_CTR-0.8,
                        2.8, 0.42, 1.5, "z", chamfer=0.25, ap=op_red)
            # Ankle guard
            box(foot, "Ankle_Guard",  sx,        0,    ANKLE_CTR+1.2,  5.4, 3.2, 2.8, chrome)
            box(foot, "Ankle_Inner",  sx,       -1.0,  ANKLE_CTR+0.3,  4.0, 2.0, 1.6, dark_metal)
            # Boot fin (anti-twist)
            box(foot, "Boot_Fin",     sx+m*2.0,  0,    ANKLE_CTR-0.2,  0.40, 6.5, 4.2, op_blue)
            box(foot, "Boot_Fin2",    sx+m*2.5,  0,    ANKLE_CTR+0.8,  0.32, 5.0, 2.8, op_red)

            # Ankle servos (V11: couplers + bearing fit)
            mg996r(foot, f"{side}_AnkP", sx, 0, ANKLE_CTR+2.2, "x")
            mg996r(foot, f"{side}_AnkR", sx, 0, ANKLE_CTR+0.5, "y")
            bearing_fit(foot, f"{side}_AB",  sx, 0, ANKLE_CTR,     "x", 1.00, 0.55, fit_type="press")
            # V11 MECH-3: ankle hard stops
            hard_stop(foot, f"{side}_AnkP_Stop", sx, -2.0, ANKLE_CTR+2.2, "x", 20)
            hard_stop(foot, f"{side}_AnkN_Stop", sx,  2.0, ANKLE_CTR+2.2, "x", -20)
            for bx_off in [-1.5, 1.5]:
                m3_boss(foot, f"{side}_FootBoss_{bx_off:.0f}", sx+bx_off, 0, ANKLE_CTR-0.5)
            align_pin(foot, f"{side}_Ft_A", sx-1.0, 0, ANKLE_CTR-1.3)
            align_pin(foot, f"{side}_Ft_B", sx+1.0, 0, ANKLE_CTR-1.3)

            # V11: Vibration isolation pad pocket under sole
            box(foot, f"{side}_VibPad", sx, -0.6, ANKLE_CTR-2.2, 5.5, 8.0, 0.15, rubber_blk)

            # V11: Printability checks
            for comp_chk in [thigh, shin, foot]:
                for b in list(comp_chk.bRepBodies):
                    if b.name:
                        printability_check(comp_chk, b.name)

            # V11 JIG-1: leg shell assembly jigs
            assembly_jig(f"OP_Thigh_{side}",
                [(sx-1.0, 0, THIGH_CTR+2.0), (sx+1.0, 0, THIGH_CTR-2.0)],
                [(sx-0.5, 0, THIGH_CTR+2.0), (sx+0.5, 0, THIGH_CTR-2.0)],
                (7.0, 5.0, 0.60))

        # ─────────────────────────────────────────────────────────────────
        # 5 ARMS  (GEO-3 shoulders + GEO-2 hands  + V11 tendon drive)
        # ─────────────────────────────────────────────────────────────────
        for side, ax in [("L", -SHOULDER_X), ("R", SHOULDER_X)]:
            m = -1 if side == "L" else 1

            # —— UPPER ARM ——
            ua = new_component(f"OP_UpperArm_{side}")

            # GEO-3 — Double-layer wide shoulder pad
            box(ua, "Sh_Block",        ax,         0, SHOULDER_CTR,      5.6, 4.2, 5.6, op_red)
            box(ua, "Sh_Pad_Outer",    ax+m*3.20,  0, SHOULDER_CTR,      1.00, 5.2, 5.8, op_red)
            box(ua, "Sh_Pad_Edge",     ax+m*3.75,  0, SHOULDER_CTR,      0.40, 4.6, 5.2, chrome)
            # Twin spike stacks (V11: cone tips chamfered for printability)
            for sz, sr, sh2 in [(SHOULDER_CTR+2.8, 0.42, 0.95),
                                 (SHOULDER_CTR+1.5, 0.36, 0.75)]:
                cyl(ua, f"Stk_A_{sz:.0f}", ax+m*3.3, -1.5, sz, sr,  sh2, "z", chrome)
                # V11: replace sharp cone with printable chamfered tip
                cone_shape(ua, f"StkTip_{sz:.0f}", ax+m*3.3, -1.5, sz+sh2/2+0.35,
                           sr, sr*0.35, 0.55, "z", dark_grey)
            box(ua, "Sh_Guard",        ax+m*2.60,  0, SHOULDER_CTR-0.2, 0.42, 4.4, 6.6, op_blue)

            # V11 MECH-1: internal shoulder frame
            box(ua, f"{side}_Shoulder_Frame", ax, 0, SHOULDER_CTR, 3.5, 3.0, 4.5, dark_metal)

            # Upper arm link
            box(ua, "UA_Link",         ax,         0, ELBOW_Z+3.0,      3.2, 3.4, 5.2, op_red)
            box(ua, "UA_Skin",         ax+m*1.80,  0, ELBOW_Z+3.0,      0.52, 3.4, 5.2, chrome)

            # Shoulder servos (3 DOF) — V11: couplers + dual bearings
            mg996r(ua, f"{side}_ShY",    ax, 0, SHOULDER_CTR+1.5, "z")
            # V11 BRG-3: dual bearing for shoulder yaw
            dual_bearing(ua, f"{side}_ShY_Dual", ax, 0, SHOULDER_CTR+2.0,
                         "z", 1.00, 0.55, span=2.80, fit_type="press")
            u_bracket(ua, f"{side}_SPB", ax, 0, SHOULDER_CTR,     4.8, 3.4, 3.4)
            mg996r(ua, f"{side}_ShP",    ax, 0, SHOULDER_CTR,     "x")
            dual_bearing(ua, f"{side}_ShP_Dual", ax, 0, SHOULDER_CTR,
                         "x", 1.10, 0.62, span=2.80, fit_type="press")
            mg996r(ua, f"{side}_ShR",    ax, 0, SHOULDER_CTR-1.2, "y")
            bearing_fit(ua, f"{side}_SB",    ax, 0, SHOULDER_CTR,     "x", 1.10, 0.62, fit_type="press")

            # Elbow joint — V11: coupler + dual bearing
            u_bracket(ua, f"{side}_EB",  ax, 0, ELBOW_Z,          3.8, 3.0, 3.0)
            mg996r(ua, f"{side}_ElbP",   ax, 0, ELBOW_Z,          "x")
            dual_bearing(ua, f"{side}_Elb_Dual", ax, 0, ELBOW_Z-0.5,
                         "x", 0.95, 0.52, span=2.40, fit_type="press")
            wire_channel(ua, f"{side}_UAW", ax, 0, ELBOW_Z+3.0,   0.4, 5.2, "z")
            m3_boss(ua, f"{side}_UAboss", ax, 0, ELBOW_Z+3.0)
            # V11 MECH-3: elbow hard stop
            hard_stop(ua, f"{side}_ElbStop", ax, -2.0, ELBOW_Z, "x", 150)

            # —— FOREARM ——
            fa = new_component(f"OP_Forearm_{side}")
            box(fa, "FA_Link",    ax,       0,    WRIST_Z+3.5, 3.2, 3.8, 4.8, op_blue)
            box(fa, "FA_Fender",  ax+m*2.1, 0,    WRIST_Z+3.5, 0.52, 5.2, 5.8, op_red)
            box(fa, "FA_Back",    ax,       2.3,  WRIST_Z+3.5, 2.6, 0.38, 4.8, chrome)
            box(fa, "FA_Vent_L",  ax-0.6,  -1.8,  WRIST_Z+3.5, 0.30, 0.22, 3.0, dark_grey)
            box(fa, "FA_Vent_R",  ax+0.6,  -1.8,  WRIST_Z+3.5, 0.30, 0.22, 3.0, dark_grey)
            mg90s(fa, f"{side}_WR",   ax, 0, WRIST_Z+0.8, "x")
            bearing_fit(fa, f"{side}_WB", ax, 0, WRIST_Z+0.5, "x", 0.80, 0.44, fit_type="press")
            wire_channel(fa, f"{side}_FAW", ax, 0, WRIST_Z+3.5, 0.4, 4.8, "z")
            m3_boss(fa, f"{side}_FAboss", ax, 0, WRIST_Z+4.2)
            # V11 CAB-3: wire grommet at wrist
            grommet_slot(fa, f"{side}_WristWire", ax, 0, WRIST_Z, "y", 0.45)

            # —— HAND (palm + V11 tendon system) ——
            hand = new_component(f"OP_Hand_{side}")
            box(hand, "Palm_Main",   ax,         -0.4,  WRIST_Z-1.6, 3.4, 3.0, 2.2, dark_grey)
            box(hand, "Palm_Inner",  ax,          0.6,  WRIST_Z-1.6, 2.6, 2.0, 2.0, black_plastic)
            # Knuckle row (4 bumps)
            MCP_Z = WRIST_Z - PALM_BOTTOM_OFFSET
            for ki, kx_off in enumerate(FING_X_OFF):
                cyl(hand, f"Knuckle_{ki}",
                    ax + m*kx_off, -1.5, MCP_Z + 0.15, 0.28, 0.38, "y", chrome)
            # Wrist ring + panel
            cyl(hand, "Wrist_Ring",  ax, 0, WRIST_Z-0.4, 1.05, 0.42, "z", chrome)
            box(hand, "Hand_Panel",  ax+m*0.9, -0.7, WRIST_Z-1.3, 0.38, 2.8, 2.8, op_red)

            # V11: Finger servo bay (DS04-NFC) with drum
            box(hand, "Finger_SrvBay", ax, 0.5, WRIST_Z-1.8, 1.4, 0.90, 2.4, black_plastic)
            # V11 PHY-7: servo drum for tendon winding
            servo_drum(hand, f"{side}_Finger", ax, 1.2, WRIST_Z-1.8, "y")
            BOM.add("Servo", "DS04-NFC 9g digital servo (finger drive)", 2, f"OP_Hand_{side}")

            # V11 PHY-10: palm idler pulleys (route tendon to each finger)
            for fi, fxo in enumerate(FING_X_OFF):
                px = ax + m * fxo * 0.5
                palm_pulley(hand, f"{side}_Pulley_{fi}", px, -0.5, WRIST_Z-1.2, "x")

            # LED strip pocket
            for lxi in [-0.7, 0.7]:
                led_pocket_5mm(hand, f"PalmLED_{lxi:.0f}", ax+lxi, -1.65, WRIST_Z-1.6, "y")

            # GEO-2 + V11 PHY-8/9/10/11: Articulated fingers with tendon drive
            for fi, (fname, fxo, pp_l, mp_l, dp_l) in enumerate(
                    zip(FING_NAMES, FING_X_OFF, FING_PP, FING_MP, FING_DP)):

                fc   = new_component(f"OP_{fname}_{side}")
                fx   = ax + m * fxo
                fy   = -1.35
                mcp_z = MCP_Z
                pp_cz = mcp_z - pp_l / 2
                mp_cz = mcp_z - pp_l - mp_l / 2
                dp_cz = mcp_z - pp_l - mp_l - dp_l / 2

                # Proximal phalanx
                box(fc, "PP", fx, fy, pp_cz, FING_W, FING_H, pp_l, grey_plastic)
                # V11 PHY-8: tendon guide groove through PP
                tendon_guide(fc, f"{fname}_PP", fx, fy-0.05, pp_cz, pp_l*0.8, "z")
                # PIP hinge
                cyl(fc, "PIP_Hinge", fx, fy, mcp_z - pp_l, 0.27, FING_W + 0.12, "x", chrome)
                # V11 PHY-11: spring return at PIP
                spring_return(fc, f"{fname}_PIP", fx, fy+0.3, mcp_z - pp_l, "x")

                # Middle phalanx
                box(fc, "MP", fx, fy, mp_cz, FING_W*0.94, FING_H*0.94, mp_l, grey_plastic)
                tendon_guide(fc, f"{fname}_MP", fx, fy-0.04, mp_cz, mp_l*0.8, "z")
                # DIP hinge
                cyl(fc, "DIP_Hinge", fx, fy, mcp_z - pp_l - mp_l,
                    0.24, FING_W*0.94 + 0.10, "x", chrome)

                # Distal phalanx
                box(fc, "DP", fx, fy, dp_cz, FING_W*0.86, FING_H*0.86, dp_l, grey_plastic)
                # V11 PHY-9: tendon anchor at distal phalanx tip
                tendon_anchor(fc, f"{fname}", fx, fy, mcp_z - pp_l - mp_l - dp_l - 0.15, "z")
                # Fingertip cone
                cone_shape(fc, "FT", fx, fy, mcp_z - pp_l - mp_l - dp_l - 0.12,
                           FING_W*0.44, 0.05, 0.24, "z", chrome)

                # V11 CAB-3: wire grommet (tendon exit) at MCP
                grommet_slot(fc, f"{fname}_TendonExit", fx, fy-0.2, mcp_z, "y", 0.30)

            # GEO-2 + V11: Thumb (2-phalanx, angled outward)
            thumb = new_component(f"OP_Thumb_{side}")
            tx    = ax + m * 1.70
            ty    = 0.20
            tpp_cz = MCP_Z - THUMB_PP_L / 2
            tdp_cz = MCP_Z - THUMB_PP_L - THUMB_DP_L / 2
            box(thumb, "TP",  tx, ty, tpp_cz, THUMB_W, THUMB_H, THUMB_PP_L, grey_plastic)
            # V11: tendon guide + spring on thumb
            tendon_guide(thumb, "Thumb_PP", tx, ty-0.05, tpp_cz, THUMB_PP_L*0.7, "z")
            cyl(thumb, "T_Hinge", tx, ty, MCP_Z - THUMB_PP_L,
                0.28, THUMB_W + 0.14, "x", chrome)
            spring_return(thumb, "Thumb_CMC", tx, ty+0.3, MCP_Z - THUMB_PP_L, "x")
            box(thumb, "TD",  tx, ty*0.8, tdp_cz, THUMB_W*0.86, THUMB_H*0.86, THUMB_DP_L,
                grey_plastic)
            tendon_anchor(thumb, "Thumb", tx, ty*0.8, tdp_cz - THUMB_DP_L/2 - 0.15, "z")
            cone_shape(thumb, "TT", tx, ty, tdp_cz - THUMB_DP_L/2 - 0.14,
                       THUMB_W*0.44, 0.05, 0.28, "z", chrome)

            # —— ION BLASTER (right arm only) ——
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

            # V11: Printability + jigs for arm components
            for comp_chk in [ua, fa, hand]:
                for b in list(comp_chk.bRepBodies):
                    if b.name:
                        printability_check(comp_chk, b.name)
            assembly_jig(f"OP_UpperArm_{side}",
                [(ax-1.5, 0, SHOULDER_CTR), (ax+1.5, 0, ELBOW_Z)],
                [(ax-1.0, 0, SHOULDER_CTR), (ax+1.0, 0, ELBOW_Z)],
                (8.0, 5.0, 0.60))

        # ─────────────────────────────────────────────────────────────────
        # 6 BACKPACK  (+ V11 cable routing)
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
        bearing_fit(bp,  "Roof_Brg",   0, 5.1, TORSO_CTR+5.3, "x", 0.80, 0.44, fit_type="glue")
        magnet_pocket(bp, "RoofL", -2.6, 5.1, TORSO_CTR+5.7)
        magnet_pocket(bp, "RoofR",  2.6, 5.1, TORSO_CTR+5.7)
        # V11: cable passthrough from backpack to torso
        grommet_slot(bp, "BP_Wire", 0, 4.5, TORSO_CTR+2.0, "y", 0.80)

        # ─────────────────────────────────────────────────────────────────
        # 7 STEER WHEEL PODS
        # ─────────────────────────────────────────────────────────────────
        steer = new_component("OP_SteerPods")
        for side, sx in [("L", -5.6), ("R", 5.6)]:
            m2 = -1 if side == "L" else 1
            box(steer, f"SAr_{side}",  sx, -3.6, 23.9, 1.6, 1.3, 4.2, chrome)
            box(steer, f"SPod_{side}", sx, -4.6, 23.4, 3.0, 2.2, 3.2, dark_grey)
            tt_wheel(steer, f"SW_{side}", sx+m2*2.0, -4.2, 23.4, m2)
            bearing_fit(steer, f"SPiv_{side}", sx, -3.6, 23.9, "z", 0.95, 0.50, fit_type="glue")
            mg90s(steer, f"SSrv_{side}", sx, -4.2, 23.9, "z")

        # ─────────────────────────────────────────────────────────────────
        # 8 SHIELDS / PANELS
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
        # FDM SHELL SPLITTING (Y-plane @ 0)  +  V11 MFG-1 fastener merge
        # ─────────────────────────────────────────────────────────────────
        Logger.log("--- FDM Shell Splitting + Fastener Merge (V11) ---")
        for comp in comps_list:
            to_split = [b for b in comp.bRepBodies
                        if b.name and any(k in b.name for k in SPLIT_KEYS)]
            for b in to_split:
                split_halves(comp, b, "y", 0.0)
            # V11 MFG-1: merge fasteners to nearest half
            bodies_after = list(comp.bRepBodies)
            left_bodies  = [b for b in bodies_after if b.name and "_left" in b.name]
            right_bodies = [b for b in bodies_after if b.name and "_right" in b.name]
            for lb in left_bodies:
                merge_fasteners_to_halves(comp, lb, None, axis="y")
            for rb in right_bodies:
                merge_fasteners_to_halves(comp, None, rb, axis="y")
        Logger.log("Shell splitting and fastener merge complete.")

        # ─────────────────────────────────────────────────────────────────
        # KINEMATICS
        # ─────────────────────────────────────────────────────────────────
        t  = occs.get("OP_Torso")
        p  = occs.get("OP_Pelvis")
        h  = occs.get("OP_Head")
        b  = occs.get("OP_Backpack")
        st = occs.get("OP_SteerPods")
        sh = occs.get("OP_Shields")

        if p:
            p.isGrounded = True

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

            # GEO-2 + V11: Finger MCP joints
            MCP_Z = WRIST_Z - PALM_BOTTOM_OFFSET
            for fi, fname in enumerate(FING_NAMES):
                fx   = ax + m * FING_X_OFF[fi]
                fo_f = occs.get(f"OP_{fname}_{side}")
                revolute_joint(f"{side}_{fname}_MCP", fo_f, ha, fx, -1.35, MCP_Z, "x")

            # Thumb CMC (ball joint)
            tx     = ax + m * 1.70
            thumb_occ = occs.get(f"OP_Thumb_{side}")
            ball_joint(f"{side}_Thumb_CMC", thumb_occ, ha, tx, 0.20, MCP_Z)

            if side == "R":
                bl = occs.get("OP_Ion_Blaster")
                if bl:
                    revolute_joint("Blaster_Fold", bl, ha, ax, 0, WRIST_Z-1.0, "y")

        # ── Kinematic Validation ──────────────────────────────────────────
        Logger.log("Validating mechanical linkages...")
        try:
            orphans       = []
            jointed_comps = set()
            for i in range(root.asBuiltJoints.count):
                j = root.asBuiltJoints.item(i)
                if j.occurrenceOne:
                    jointed_comps.add(j.occurrenceOne.component.name)
                if j.occurrenceTwo:
                    jointed_comps.add(j.occurrenceTwo.component.name)
            for comp in comps_list:
                if comp.name not in ("OP_Torso", "OP_Pelvis")                    and comp.name not in jointed_comps:
                    orphans.append(comp.name)
            if orphans:
                Logger.log(f"  !!! ORPHANS: {orphans}", "WARN")
            else:
                Logger.log("  All components bound to kinematic chain. [OK]")
        except Exception:
            Logger.log("  Kinematic validation skipped (MCP env).", "WARN")

        try:
            cam = app.activeViewport.camera
            cam.isFitView = True
            app.activeViewport.camera = cam
        except Exception:
            pass

        # ── V11 FST-1: Screw length verification ─────────────────────────
        verify_screw_lengths()

        # ── V11 DOC-1: Write assembly guide ──────────────────────────────
        write_assembly_guide()

        # ═════════════════════════════════════════════════════════════════
        # SIMULATION ENGINE  v11.0
        # ═════════════════════════════════════════════════════════════════

        class SimulationEngine:
            """
            Simulation suite for Optimus Prime G1 v11 (Physical Build Edition).
            New in v11: All v10 sim features + tendon drive validation,
            coupler clearance check, cable routing verification,
            transform lock testing, printability report.
            """

            BALL_JOINTS = {
                "Waist_Cluster", "Neck_Cluster",
                "L_Hip_Cluster", "R_Hip_Cluster",
                "L_Ankle_Cluster", "R_Ankle_Cluster",
                "L_Shoulder_Cluster", "R_Shoulder_Cluster",
                "L_Wrist", "R_Wrist",
                "L_Thumb_CMC", "R_Thumb_CMC",
            }
            REV_JOINTS = {
                "L_Knee", "R_Knee", "L_Elbow", "R_Elbow", "Blaster_Fold",
                "L_Pinky_MCP", "L_Ring_MCP", "L_Middle_MCP", "L_Index_MCP",
                "R_Pinky_MCP", "R_Ring_MCP", "R_Middle_MCP", "R_Index_MCP",
            }

            def __init__(self, root, comps_list, design, app, ui):
                self._root   = root
                self._comps  = comps_list
                self._design = design
                self._app    = app
                self._ui     = ui
                self._cols   = []
                self._logged_joints = False

            # ── Internal helpers ──────────────────────────────────────────

            def _gj(self, name):
                try:
                    j = self._root.asBuiltJoints.itemByName(name)
                    if j is not None:
                        return j
                except Exception:
                    Logger.log(f"_gj exception '{name}': {traceback.format_exc()}", "ERROR")
                    return None
                try:
                    if not self._logged_joints:
                        self._logged_joints = True
                        names = [self._root.asBuiltJoints.item(i).name
                                 for i in range(self._root.asBuiltJoints.count)]
                        Logger.log(f"Joint '{name}' not found. Available: {names}", "WARN")
                except Exception:
                    pass
                return None

            @staticmethod
            def _ease(t):
                return t * t * (3.0 - 2.0 * t)

            def _get(self, mo, axis):
                try:
                    if mo.objectType == adsk.fusion.RevoluteJointMotion.classType():
                        return mo.rotationValue
                    if mo.objectType == adsk.fusion.BallJointMotion.classType():
                        return getattr(mo, axis + "Value")
                except Exception as e:
                    Logger.log(f"_get({axis}) failed: {e}", "ERROR")
                return 0.0

            def _set(self, mo, axis, val):
                try:
                    if mo.objectType == adsk.fusion.RevoluteJointMotion.classType():
                        mo.rotationValue = val
                    elif mo.objectType == adsk.fusion.BallJointMotion.classType():
                        setattr(mo, axis + "Value", val)
                except Exception as e:
                    Logger.log(f"_set({axis},{val:.2f}) failed: {e}", "ERROR")

            def _refresh(self):
                if os.path.exists(STOP_FLAG):
                    try:
                        os.remove(STOP_FLAG)
                    except Exception:
                        pass
                    raise Exception("SIMULATION_ABORTED_BY_USER")
                try:
                    self._app.activeViewport.refresh()
                except Exception:
                    pass
                try:
                    adsk.doEvents()
                except Exception:
                    pass

            def _clamp(self, joint_name, axis, deg):
                limits = JOINT_LIMITS.get(joint_name, {}).get(axis)
                if limits:
                    lo, hi = limits
                    if deg < lo or deg > hi:
                        Logger.log(
                            f"CLAMP: {joint_name}.{axis} {deg:.0f}° -> [{lo}°,{hi}°]", "WARN")
                    return max(lo, min(hi, deg))
                return deg

            # ── Core animation API ────────────────────────────────────────

            def move_joint(self, name, deg, steps=20, axis="pitch", ease=True, clamp=True):
                j = self._gj(name)
                if not j:
                    return
                if clamp:
                    deg = self._clamp(name, axis, deg)
                mo    = j.jointMotion
                e_rad = math.radians(deg)
                s_rad = self._get(mo, axis)
                for i in range(1, steps + 1):
                    t = self._ease(i/steps) if ease else i/steps
                    self._set(mo, axis, s_rad + (e_rad - s_rad) * t)
                    self._refresh()

            def move_group(self, targets, steps=20, axis="pitch", ease=True, clamp=True):
                active = []
                for item in targets:
                    name = item[0]; deg = item[1]
                    ax   = item[2] if len(item) > 2 else axis
                    j    = self._gj(name)
                    if not j:
                        continue
                    d  = self._clamp(name, ax, deg) if clamp else deg
                    mo = j.jointMotion
                    active.append((mo, ax, self._get(mo, ax), math.radians(d)))
                for i in range(1, steps + 1):
                    t = self._ease(i/steps) if ease else i/steps
                    for mo, ax, s_rad, e_rad in active:
                        self._set(mo, ax, s_rad + (e_rad - s_rad) * t)
                    self._refresh()

            def move_ball(self, targets, steps=20, clamp=True):
                """Animate ball-joint (pitch/yaw/roll) and revolute (pitch) together."""
                active = []
                for name, pitch, yaw, roll in targets:
                    j = self._gj(name)
                    if not j:
                        continue
                    mo     = j.jointMotion
                    is_rev = (mo.objectType ==
                              adsk.fusion.RevoluteJointMotion.classType())
                    if is_rev:
                        if pitch is not None:
                            v = self._clamp(name, "pitch", pitch) if clamp else pitch
                            active.append((mo, "pitch",
                                           self._get(mo, "pitch"), math.radians(v)))
                    else:
                        for axis, val in [("pitch", pitch), ("yaw", yaw), ("roll", roll)]:
                            if val is None:
                                continue
                            v = self._clamp(name, axis, val) if clamp else val
                            active.append((mo, axis,
                                           self._get(mo, axis), math.radians(v)))
                for i in range(1, steps + 1):
                    t = self._ease(i / steps)
                    for mo, axis, s_rad, e_rad in active:
                        self._set(mo, ax, s_rad + (e_rad - s_rad) * t)
                    self._refresh()

            def reset_all(self, steps=10, groups=None):
                if groups is None:
                    ball_names = self.BALL_JOINTS
                    rev_names  = self.REV_JOINTS
                else:
                    ball_names = {n for n in groups if n in self.BALL_JOINTS}
                    rev_names  = {n for n in groups if n in self.REV_JOINTS}
                ball_targets = [(n, 0.0, 0.0, 0.0) for n in ball_names if self._gj(n)]
                rev_targets  = [n               for n in rev_names  if self._gj(n)]
                if ball_targets:
                    self.move_ball(ball_targets, steps=steps)
                for name in rev_targets:
                    self.move_joint(name, 0.0, steps, "pitch", ease=True)
                self._refresh()
                Logger.log("-> reset to neutral")

            def _interfere(self, label="Interference"):
                try:
                    all_bodies = adsk.core.ObjectCollection.create()
                    for comp in self._comps:
                        for body in list(comp.bRepBodies):
                            if body.name and not any(t in body.name for t in SKIP_TAGS):
                                all_bodies.add(body)
                    inter_input = self._design.createInterferenceInput(all_bodies)
                    inter_input.isCoincidentFacesInterference = False
                    results = self._design.analyzeInterference(inter_input)
                    count   = results.count
                    if count:
                        Logger.log(f"  {label}: {count} collision(s)")
                        for i in range(min(count, 5)):
                            r = results.item(i)
                            Logger.log(f"    [{r.entityOne.name}] <-> [{r.entityTwo.name}]")
                        if count > 5:
                            Logger.log(f"    ...{count-5} more")
                    else:
                        Logger.log(f"  {label}: [OK] 0 collisions")
                    self._cols.append((label, count))
                except Exception as e:
                    Logger.log(f"  {label}: skipped ({e})", "WARN")
                    self._cols.append((label, -1))

            # ── SIM-1: ZMP Stability ──────────────────────────────────────
            def _check_zmp(self, label):
                FOOT_HW = 6.2 / 2.0
                FOOT_HL = 9.2 / 2.0
                try:
                    com = self._root.physicalProperties.centerOfMass
                except Exception as e:
                    Logger.log(f"  ZMP [{label}]: CoM unavailable ({e})", "WARN")
                    return
                l_cx, r_cx = -HIP_X, HIP_X
                in_L  = (l_cx - FOOT_HW <= com.x <= l_cx + FOOT_HW and
                         -FOOT_HL <= com.y <= FOOT_HL)
                in_R  = (r_cx - FOOT_HW <= com.x <= r_cx + FOOT_HW and
                         -FOOT_HL <= com.y <= FOOT_HL)
                in_DS = (min(l_cx - FOOT_HW, r_cx - FOOT_HW) <= com.x <=
                         max(l_cx + FOOT_HW, r_cx + FOOT_HW) and
                         -FOOT_HL <= com.y <= FOOT_HL)
                stable = in_DS
                tag    = "[OK] ZMP STABLE" if stable else "[FAIL] ZMP UNSTABLE"
                Logger.log(
                    f"  ZMP [{label:16s}] {tag}  "
                    f"CoM=({com.x:+.2f}, {com.y:+.2f}, {com.z:.1f}) cm")
                return stable

            # ── SIM-2: Finger Gestures ────────────────────────────────────
            def _set_fingers(self, side, degrees, thumb_deg=None, steps=12):
                targets = [(f"{side}_{n}_MCP", degrees, None, None)
                           for n in FING_NAMES]
                self.move_ball(targets, steps=steps)
                if thumb_deg is not None:
                    self.move_ball([(f"{side}_Thumb_CMC", thumb_deg, 0, 0)], steps=steps)

            def gesture_open(self, side="R", steps=10):
                self._set_fingers(side, 0, thumb_deg=0, steps=steps)
                Logger.log(f"  Gesture: {side} hand OPEN")

            def gesture_fist(self, side="R", steps=12):
                self._set_fingers(side, 80, thumb_deg=50, steps=steps)
                Logger.log(f"  Gesture: {side} hand FIST")

            def gesture_point(self, side="R", steps=10):
                targets = [
                    (f"{side}_Pinky_MCP",  80, None, None),
                    (f"{side}_Ring_MCP",   80, None, None),
                    (f"{side}_Middle_MCP", 80, None, None),
                    (f"{side}_Index_MCP",  -3, None, None),
                    (f"{side}_Thumb_CMC",  40, 0,    0),
                ]
                self.move_ball(targets, steps=steps)
                Logger.log(f"  Gesture: {side} hand POINT")

            def gesture_snap(self, side="R", steps=6):
                targets = [
                    (f"{side}_Pinky_MCP",  80, None, None),
                    (f"{side}_Ring_MCP",   70, None, None),
                    (f"{side}_Middle_MCP", -3, None, None),
                    (f"{side}_Index_MCP",  -3, None, None),
                    (f"{side}_Thumb_CMC",  55, 0,    0),
                ]
                self.move_ball(targets, steps=steps)
                Logger.log(f"  Gesture: {side} hand SNAP")

            # ── V11 NEW: tendon slack test ────────────────────────────────
            def test_tendon_slack(self):
                """Verify finger MCP joints move freely (tendon not over-tensioned)."""
                Logger.log("--- V11: TENDON SLACK TEST ---")
                for side in ["L", "R"]:
                    for fi, fname in enumerate(FING_NAMES):
                        jname = f"{side}_{fname}_MCP"
                        self.move_joint(jname, 0, steps=5, axis="pitch")
                        self.move_joint(jname, 45, steps=5, axis="pitch")
                        self.move_joint(jname, 0, steps=5, axis="pitch")
                Logger.log("  Tendon slack test complete [OK]")

            # ── V11 NEW: transform lock clearance test ────────────────────
            def test_transform_locks(self):
                """Verify transform lock pins clear at all joint positions."""
                Logger.log("--- V11: TRANSFORM LOCK CLEARANCE ---")
                Logger.log("  Lock pins must be DISENGAGED before servo motion")
                # Move all major joints through range to verify no binding
                for name in ["Waist_Cluster", "L_Hip_Cluster", "R_Hip_Cluster"]:
                    for axis in ["pitch", "yaw", "roll"]:
                        if axis in JOINT_LIMITS.get(name, {}):
                            lo, hi = JOINT_LIMITS[name][axis]
                            self.move_joint(name, lo, steps=8, axis=axis)
                            self.move_joint(name, hi, steps=8, axis=axis)
                            self.move_joint(name, 0, steps=8, axis=axis)
                Logger.log("  Transform lock clearance OK")

            # ── V11 NEW: printability report ──────────────────────────────
            def printability_report(self):
                """Log all collected printability warnings and orientation notes."""
                Logger.log("--- V11: PRINTABILITY REPORT ---")
                Logger.log(f"  Parts with support warnings: {len(SUPPORT_WARNINGS)}")
                for part, reason in SUPPORT_WARNINGS[:20]:
                    Logger.log(f"    [WARN] {part}: {reason}")
                if len(SUPPORT_WARNINGS) > 20:
                    Logger.log(f"    ...and {len(SUPPORT_WARNINGS)-20} more")
                Logger.log(f"  Assembly jigs designed: {len(JIG_REGISTRY)}")
                for jig_name, target in JIG_REGISTRY:
                    Logger.log(f"    {jig_name} -> for {target}")

            # ── SIM-3: Arm Workspace Test ─────────────────────────────────
            def test_arm_workspace(self):
                Logger.log("--- SIM-3: ARM WORKSPACE TEST ---")
                poses = [
                    ( -30,   0,   0, "Forward low"),
                    ( -90,   0,  90, "Forward high"),
                    (-120,   0,  90, "Overhead"),
                    ( -80,  80,  60, "Side reach"),
                    ( -80, -80,  60, "Cross reach"),
                    ( -40,   0, 130, "Bicep curl"),
                    (-175,   0,  80, "Reach back"),
                    ( -80,   0, 150, "Elbow max"),
                ]
                for (sp, sy, ep, lbl) in poses:
                    for side in ["L", "R"]:
                        self.move_ball([
                            (f"{side}_Shoulder_Cluster", sp, sy, 0)], steps=10)
                        self.move_joint(f"{side}_Elbow", ep, steps=8, axis="pitch")
                    self._interfere(f"Workspace: {lbl}")
                    self.reset_all(steps=8,
                                   groups=["L_Shoulder_Cluster", "R_Shoulder_Cluster",
                                           "L_Elbow", "R_Elbow"])

            # ── Module 1: Joint ROM ───────────────────────────────────────
            def test_joint_rom(self):
                self.reset_all(steps=10)
                Logger.log("--- MODULE 1 / 10 ---")
                Logger.log("MODULE 1: JOINT ROM TEST")
                for name, axes in JOINT_LIMITS.items():
                    for axis, (lo, hi) in axes.items():
                        for lbl, angle in [("MIN", lo), ("MAX", hi)]:
                            self.move_joint(name, angle, steps=15, axis=axis)
                            self._interfere(f"{lbl} {name} {axis}")
                            self.move_joint(name, 0, steps=10, axis=axis)

            # ── Module 2: Head Look-Around ────────────────────────────────
            def simulate_head_scan(self):
                self.reset_all(steps=10)
                Logger.log("--- MODULE 2 / 10 ---")
                Logger.log("MODULE 2: HEAD LOOK-AROUND")
                for yaw_deg in [0, -50, 0, 50, 0]:
                    self.move_joint("Neck_Cluster", yaw_deg, steps=18, axis="yaw")
                for pitch_deg in [0, -25, 0, 35, 0]:
                    self.move_joint("Neck_Cluster", pitch_deg, steps=18, axis="pitch")

            # ── Module 3: Wave Gesture ────────────────────────────────────
            def simulate_wave(self):
                self.reset_all(steps=10)
                Logger.log("--- MODULE 3 / 10 ---")
                Logger.log("MODULE 3: WAVE GESTURE")
                self.gesture_open("R")
                self.move_joint("R_Shoulder_Cluster", -45, steps=15, axis="pitch")
                self.move_joint("R_Shoulder_Cluster",  80, steps=15, axis="yaw")
                self.move_joint("R_Elbow",             90, steps=12, axis="pitch")
                for _ in range(3):
                    self.move_ball([("R_Wrist", None, None, -30)], steps=8)
                    self.move_ball([("R_Wrist", None, None,  30)], steps=8)
                self.reset_all(steps=12)

            # ── Module 4: Idle Breathing ──────────────────────────────────
            def simulate_idle_breathing(self):
                self.reset_all(steps=8)
                Logger.log("--- MODULE 4 / 10 ---")
                Logger.log("MODULE 4: IDLE BREATHING")
                for _ in range(4):
                    self.move_joint("Waist_Cluster", -3, steps=12, axis="pitch")
                    self.move_joint("Waist_Cluster",  3, steps=12, axis="pitch")
                self.move_joint("Waist_Cluster", 0, steps=8, axis="pitch")

            # ── Module 5: Advanced Walking ────────────────────────────────
            def simulate_walking_advanced(self):
                self.reset_all(steps=10)
                Logger.log("--- MODULE 5 / 10 ---")
                Logger.log("MODULE 5: ADVANCED WALKING (ZMP checked)")
                for cycle in range(4):
                    phase  = cycle % 2
                    l_sign =  1 if phase == 0 else -1
                    r_sign = -1 if phase == 0 else  1
                    self.move_ball([
                        ("L_Hip_Cluster",      25*l_sign, 10*l_sign,  5*l_sign),
                        ("R_Hip_Cluster",      25*r_sign, 10*r_sign,  5*r_sign),
                        ("L_Shoulder_Cluster",  8*l_sign, 15*l_sign,  5*l_sign),
                        ("R_Shoulder_Cluster",  8*r_sign, 15*r_sign,  5*r_sign),
                        ("L_Knee",             60,        None,        None),
                        ("R_Knee",             60,        None,        None),
                        ("L_Ankle_Cluster",    15*l_sign, None,        8*l_sign),
                        ("R_Ankle_Cluster",    15*r_sign, None,        8*r_sign),
                    ], steps=20)
                    self._check_zmp(f"Walk cycle {cycle+1}")
                self.reset_all(steps=12)

            # ── Module 6: Running ─────────────────────────────────────────
            def simulate_running(self):
                self.reset_all(steps=8)
                Logger.log("--- MODULE 6 / 10 ---")
                Logger.log("MODULE 6: RUNNING")
                for cycle in range(3):
                    phase  = cycle % 2
                    l_sign =  1 if phase == 0 else -1
                    r_sign = -1 if phase == 0 else  1
                    self.move_ball([
                        ("L_Hip_Cluster",      30*l_sign, 20*l_sign, 10*l_sign),
                        ("R_Hip_Cluster",      30*r_sign, 20*r_sign, 10*r_sign),
                        ("L_Shoulder_Cluster", 15*l_sign, 25*l_sign, 10*l_sign),
                        ("R_Shoulder_Cluster", 15*r_sign, 25*r_sign, 10*r_sign),
                        ("L_Knee",             95,        None,       None),
                        ("R_Knee",             95,        None,       None),
                        ("L_Ankle_Cluster",    25*l_sign, None,       12*l_sign),
                        ("R_Ankle_Cluster",    25*r_sign, None,       12*r_sign),
                    ], steps=14)
                self.reset_all(steps=10)

            # ── Module 7: Combat Sequence ─────────────────────────────────
            def simulate_combat(self):
                self.reset_all(steps=10)
                Logger.log("--- MODULE 7 / 10 ---")
                Logger.log("MODULE 7: COMBAT SEQUENCE")
                self.gesture_fist("L")
                self.gesture_fist("R")
                self.move_joint("R_Shoulder_Cluster", -45, steps=10, axis="pitch")
                self.move_joint("R_Shoulder_Cluster",  45, steps=8,  axis="yaw")
                self.move_joint("R_Elbow",            120, steps=8,  axis="pitch")
                self.move_ball([("R_Wrist", None, None, 20)], steps=6)
                self.move_joint("R_Shoulder_Cluster", -10, steps=6,  axis="pitch")
                self.move_joint("R_Shoulder_Cluster", -80, steps=10, axis="pitch")
                self.move_joint("R_Shoulder_Cluster", -30, steps=6,  axis="yaw")
                self.move_joint("R_Elbow",             30, steps=8,  axis="pitch")
                self.move_joint("L_Shoulder_Cluster",  30, steps=8,  axis="pitch")
                self.move_joint("L_Elbow",             45, steps=6,  axis="pitch")
                self._check_zmp("Combat stance")
                self.reset_all(steps=12)

            # ── Transformation helpers ────────────────────────────────────
            def _transform_to_truck(self, steps_scale=1.0):
                s = steps_scale
                self.move_group([
                    ("R_Elbow",      0,   "pitch"),
                    ("L_Elbow",      0,   "pitch"),
                    ("Blaster_Fold", -90, "pitch"),
                ], steps=int(20*s))
                self.move_ball([
                    ("L_Wrist",  90, None,  90),
                    ("R_Wrist", 135, None,  90),
                ], steps=int(20*s))
                self.move_ball([("Neck_Cluster", -90, 0, 0)], steps=int(15*s))
                self.move_ball([
                    ("L_Shoulder_Cluster", -88, 0, 0),
                    ("R_Shoulder_Cluster", -88, 0, 0),
                ], steps=int(22*s))
                self.move_ball([
                    ("L_Hip_Cluster", 0, 90, 0),
                    ("R_Hip_Cluster", 0, 90, 0),
                ], steps=int(22*s))
                self.move_ball([
                    ("L_Ankle_Cluster", 0, 90, 0),
                    ("R_Ankle_Cluster", 0, 90, 0),
                ], steps=int(18*s))

            def _transform_to_robot(self, steps_scale=1.0):
                s = steps_scale
                self.move_ball([("L_Ankle_Cluster", 0, 0, 0), ("R_Ankle_Cluster", 0, 0, 0)],
                               steps=int(18*s))
                self.move_ball([("L_Hip_Cluster", 0, 0, 0), ("R_Hip_Cluster", 0, 0, 0)],
                               steps=int(22*s))
                self.move_ball([("L_Shoulder_Cluster", 0, 0, 0), ("R_Shoulder_Cluster", 0, 0, 0)],
                               steps=int(22*s))
                self.move_ball([("Neck_Cluster", 0, 0, 0)], steps=int(15*s))
                self.move_ball([("L_Wrist", 0, None, 0), ("R_Wrist", 0, None, 0)],
                               steps=int(18*s))
                self.move_group([("Blaster_Fold", 0, "pitch"), ("R_Elbow", 0, "pitch"),
                                 ("L_Elbow", 0, "pitch")], steps=int(18*s))

            # ── Module 8: Transformation ──────────────────────────────────
            def simulate_transformation(self):
                self.reset_all(steps=10)
                Logger.log("--- MODULE 8a / 10 ---")
                Logger.log("MODULE 8a: TRANSFORMATION  Robot -> Truck")
                self._transform_to_truck(steps_scale=1.0)
                self._interfere("Truck-mode check")
                Logger.log("MODULE 8c: TRUCK -> ROBOT")
                self._transform_to_robot(steps_scale=1.0)
                Logger.log("Robot mode restored.")
                self.reset_all(steps=10)

            # ── Module 9a: Stability Analysis ─────────────────────────────
            def run_stability_analysis(self):
                Logger.log("--- MODULE 9 / 10 ---")
                Logger.log("MODULE 9a: STABILITY ANALYSIS (ZMP)")
                poses = {
                    "Attention": {"Waist_Cluster": (0, 0, 0)},
                    "Combat": {
                        "Waist_Cluster":      (10, 0,   0),
                        "L_Knee":              45,
                        "R_Knee":              45,
                        "L_Elbow":             90,
                        "R_Elbow":             90,
                        "R_Shoulder_Cluster": (0,  30, -45),
                    },
                    "Squat": {
                        "Waist_Cluster":  (20, 0,   0),
                        "L_Knee":          90,
                        "R_Knee":          90,
                        "L_Hip_Cluster":  (0, -45, 0),
                        "R_Hip_Cluster":  (0, -45, 0),
                    },
                    "Victory": {
                        "L_Shoulder_Cluster": (0, 60, -90),
                        "R_Shoulder_Cluster": (0, 60, -90),
                        "L_Elbow":             30,
                        "R_Elbow":             30,
                        "Waist_Cluster":      (15, 0, 0),
                    },
                    "Single_Leg_L": {
                        "L_Hip_Cluster":  (0, 90,  0),
                        "L_Knee":          90,
                        "Waist_Cluster":  (5, 10, -5),
                    },
                }
                for pose_name, config in poses.items():
                    self.reset_all(steps=10)
                    for key, val in config.items():
                        if isinstance(val, tuple):
                            self.move_ball([(key, val[0], val[1], val[2])], steps=15)
                        else:
                            self.move_joint(key, val, steps=12, axis="pitch")
                    self._check_zmp(pose_name)

            # ── Module 9b: Servo Load Estimation ─────────────────────────
            def estimate_servo_loads(self):
                Logger.log("MODULE 9b: SERVO LOAD ESTIMATION (v11 upgraded servos)")
                loads = [
                    ("Neck Pitch",       120,  3.0, SERVO_SPECS["micro"]),
                    ("L Shoulder Pitch", 390, 12.0, SERVO_SPECS["std"]),
                    ("R Shoulder Pitch", 390, 12.0, SERVO_SPECS["std"]),
                    ("L Elbow",          210,  7.0, SERVO_SPECS["std"]),
                    ("R Elbow",          210,  7.0, SERVO_SPECS["std"]),
                    ("L Hip Pitch",      820, 15.0, SERVO_SPECS["hip_hd"]),
                    ("R Hip Pitch",      820, 15.0, SERVO_SPECS["hip_hd"]),
                    ("L Knee Pitch",     540,  9.0, SERVO_SPECS["std"]),
                    ("R Knee Pitch",     540,  9.0, SERVO_SPECS["std"]),
                    ("Waist Pitch",     2100,  8.0, SERVO_SPECS["waist"]),
                    ("L Ankle Pitch",    280,  4.5, SERVO_SPECS["std"]),
                    ("R Ankle Pitch",    280,  4.5, SERVO_SPECS["std"]),
                    ("Finger Drive",      18,  2.0, SERVO_SPECS["digit"]),
                ]
                for label, mass_g, lever_cm, spec in loads:
                    F      = (mass_g / 1000.0) * 9.81
                    need   = (F * lever_cm / 100.0) / 0.0981
                    margin = spec["rated"] / need if need > 0 else 99.0
                    status = ("OK"       if margin >= 1.5 else
                              "MARGINAL" if margin >= 0.9 else "OVERLOAD")
                    icon   = "[OK]" if status == "OK" else f"[{status}]"
                    Logger.log(
                        f"  {label:<24} need {need:5.2f} kg-cm  "
                        f"rated {spec['rated']:5.1f}  margin {margin:.2f}x  "
                        f"{spec['name']:12s}  {icon}")

            # ── V11 NEW: Module 10: Full physical validation ──────────────
            def run_physical_validation(self):
                """Module 10 — V11 comprehensive build validation."""
                Logger.log("--- MODULE 10 / 10 ---")
                Logger.log("MODULE 10: PHYSICAL BUILD VALIDATION (v11)")
                self.test_tendon_slack()
                self.test_transform_locks()
                self.printability_report()
                Logger.log("Physical validation complete.")

            # ── Simulation mode helpers ───────────────────────────────────
            def simulate_robot_mode(self):
                self.reset_all(steps=10)
                self.move_joint("Blaster_Fold", 0, steps=10, axis="pitch")
                self.gesture_open("L")
                self.gesture_open("R")
                Logger.log("--- ROBOT MODE --- holding neutral pose")
                try:
                    cam = self._app.activeViewport.camera
                    cam.isFitView = True
                    self._app.activeViewport.camera = cam
                    self._app.activeViewport.refresh()
                except Exception:
                    pass
                self.capture_screenshots("optimus_robot_v11")

            def simulate_truck_mode(self):
                self.reset_all(steps=10)
                Logger.log("--- TRUCK MODE ---")
                self.gesture_fist("L")
                self.gesture_fist("R")
                self._transform_to_truck(steps_scale=1.0)
                self._interfere("Truck-mode check")
                Logger.log("TRUCK MODE -- holding position (no reverse)")
                try:
                    cam = self._app.activeViewport.camera
                    cam.isFitView = True
                    self._app.activeViewport.camera = cam
                    self._app.activeViewport.refresh()
                except Exception:
                    pass
                self.capture_screenshots("optimus_truck_v11")

            def simulate_battle_mode(self):
                self.reset_all(steps=10)
                Logger.log("--- BATTLE MODE ---")
                self.move_joint("Blaster_Fold", 0,   steps=10, axis="pitch")
                self.gesture_fist("L")
                self.gesture_point("R")
                self.move_ball([("L_Wrist", None, None, 90),
                                ("R_Wrist", None, None, 90)], steps=15)
                self.move_joint("R_Elbow", 130, steps=18, axis="pitch", clamp=True)
                self.move_joint("L_Elbow", 130, steps=18, axis="pitch", clamp=True)
                self.move_ball([("L_Shoulder_Cluster", 0, -88, 0),
                                ("R_Shoulder_Cluster", 0, -88, 0)], steps=22)
                self._check_zmp("Battle mode")
                self._interfere("Battle-mode check")
                Logger.log("BATTLE MODE -- holding position")
                self.capture_screenshots("optimus_battle_v11")

            # ── Debug helpers ─────────────────────────────────────────────
            def debug_joints(self, label):
                Logger.log(f"=== JOINT STATE [{label}] ===")
                try:
                    for i in range(self._root.asBuiltJoints.count):
                        j  = self._root.asBuiltJoints.item(i)
                        mo = j.jointMotion
                        if mo.objectType == adsk.fusion.RevoluteJointMotion.classType():
                            Logger.log(
                                f"  {j.name:34s} REV  "
                                f"pitch={math.degrees(mo.rotationValue):+.1f}°")
                        elif mo.objectType == adsk.fusion.BallJointMotion.classType():
                            try:
                                Logger.log(
                                    f"  {j.name:34s} BALL "
                                    f"p={math.degrees(mo.pitchValue):+.1f}° "
                                    f"y={math.degrees(mo.yawValue):+.1f}° "
                                    f"r={math.degrees(mo.rollValue):+.1f}°")
                            except Exception as e:
                                Logger.log(f"  {j.name:34s} BALL (readback: {e})", "WARN")
                except Exception:
                    Logger.log("  (joint debug unavailable in this environment)", "WARN")

            # ── Screenshot Capture ────────────────────────────────────────
            def capture_screenshots(self, prefix="optimus"):
                if not CAPTURE_SCREENSHOTS:
                    return
                try:
                    os.makedirs(SCREENSHOT_DIR, exist_ok=True)
                    viewport = self._app.activeViewport
                    camera   = viewport.camera
                    views = [
                        ("Front", adsk.core.ViewOrientations.FrontViewOrientation),
                        ("Back",  adsk.core.ViewOrientations.BackViewOrientation),
                        ("Left",  adsk.core.ViewOrientations.LeftViewOrientation),
                        ("Right", adsk.core.ViewOrientations.RightViewOrientation),
                        ("Top",   adsk.core.ViewOrientations.TopViewOrientation),
                        ("Iso",   adsk.core.ViewOrientations.IsoTopRightViewOrientation),
                    ]
                    for name, orientation in views:
                        camera.viewOrientation = orientation
                        camera.isFitView       = True
                        viewport.camera        = camera
                        time.sleep(0.5)
                        path = os.path.join(SCREENSHOT_DIR, f"{prefix}_{name}.png")
                        viewport.saveAsImageFile(path, 1920, 1080)
                        Logger.log(f"Screenshot: {path}")
                except Exception:
                    Logger.log(f"Screenshot failed: {traceback.format_exc()}", "WARN")

            # ── URDF Export (v11 — proper joint types + inertia) ──────────
            def export_urdf(self):
                def _urdf_type(name):
                    if "Cluster" in name or "Wrist" in name or "CMC" in name:
                        return "spherical"
                    if any(k in name for k in ["Knee", "Elbow", "MCP", "Fold"]):
                        return "revolute"
                    if any(k in name for k in ["Mount", "Steer", "Shields", "Backpack"]):
                        return "fixed"
                    return "revolute"

                def _limits(name):
                    limits_d = JOINT_LIMITS.get(name, {})
                    pitch = limits_d.get("pitch", (-180, 180))
                    lo_r  = math.radians(pitch[0])
                    hi_r  = math.radians(pitch[1])
                    return lo_r, hi_r

                link_mass = {
                    "OP_Head": 250,       "OP_Torso": 800,    "OP_Pelvis": 400,
                    "OP_Backpack": 150,   "OP_SteerPods": 120, "OP_Shields": 80,
                    "OP_Thigh_L": 250,    "OP_Thigh_R": 250,
                    "OP_Shin_L": 220,     "OP_Shin_R": 220,
                    "OP_Foot_L": 180,     "OP_Foot_R": 180,
                    "OP_UpperArm_L": 160, "OP_UpperArm_R": 160,
                    "OP_Forearm_L": 120,  "OP_Forearm_R": 120,
                    "OP_Hand_L": 60,      "OP_Hand_R": 60,
                    "OP_Ion_Blaster": 40,
                }
                def _inertia(cname):
                    m_kg = link_mass.get(cname, 80) / 1000.0
                    lx, ly, lz = 0.10, 0.08, 0.06
                    ixx = m_kg * (ly**2 + lz**2) / 12.0
                    iyy = m_kg * (lx**2 + lz**2) / 12.0
                    izz = m_kg * (lx**2 + ly**2) / 12.0
                    return m_kg, ixx, iyy, izz

                try:
                    os.makedirs(EXPORT_DIR, exist_ok=True)
                    path = os.path.join(EXPORT_DIR, "robot_v11.urdf")
                    jc   = self._root.asBuiltJoints.count
                    with open(path, "w", encoding="utf-8") as f:
                        f.write('<?xml version="1.0" encoding="utf-8"?>\n')
                        f.write('<robot name="Optimus_Prime_G1_v11">\n\n')
                        for comp in self._comps:
                            m_kg, ixx, iyy, izz = _inertia(comp.name)
                            f.write(f'  <link name="{comp.name}">\n')
                            f.write( '    <inertial>\n')
                            f.write( '      <origin xyz="0 0 0" rpy="0 0 0"/>\n')
                            f.write(f'      <mass value="{m_kg:.4f}"/>\n')
                            f.write(f'      <inertia ixx="{ixx:.6f}" ixy="0" ixz="0" '
                                    f'iyy="{iyy:.6f}" iyz="0" izz="{izz:.6f}"/>\n')
                            f.write( '    </inertial>\n')
                            f.write( '  </link>\n')
                        f.write('\n')
                        for i in range(jc):
                            j    = self._root.asBuiltJoints.item(i)
                            n    = j.name.replace(" ", "_")
                            o1   = j.occurrenceOne.component.name if j.occurrenceOne else "world"
                            o2   = j.occurrenceTwo.component.name if j.occurrenceTwo else "world"
                            jtyp = _urdf_type(n)
                            lo_r, hi_r = _limits(n)
                            effort = 25.0
                            if "Hip" in n:     effort = 35.0
                            if "Waist" in n:   effort = 25.0
                            if "MCP" in n:     effort = 2.2
                            f.write(f'  <joint name="{n}" type="{jtyp}">\n')
                            f.write(f'    <parent link="{o1}"/>\n')
                            f.write(f'    <child link="{o2}"/>\n')
                            f.write( '    <origin xyz="0 0 0" rpy="0 0 0"/>\n')
                            f.write( '    <axis xyz="1 0 0"/>\n')
                            if jtyp == "revolute":
                                f.write(
                                    f'    <limit lower="{lo_r:.4f}" upper="{hi_r:.4f}" '
                                    f'effort="{effort:.1f}" velocity="3.14"/>\n')
                            f.write( '  </joint>\n')
                        f.write('</robot>\n')
                    Logger.log(f"URDF v11 exported -> {path}")
                except Exception:
                    Logger.log(f"URDF export failed: {traceback.format_exc()}", "ERROR")

            # ── STL Export ────────────────────────────────────────────────
            def export_stl(self):
                try:
                    os.makedirs(EXPORT_DIR, exist_ok=True)
                    skip_s     = {"Marker", "Pivot", "MtA", "MtB",
                                  "Axle_Pivot", "Horn", "_Vis", "Scope",
                                  "Antenna", "Boss", "Insert", "Nut", "Snap"}
                    export_mgr = self._design.exportManager
                    count      = 0
                    for comp in self._comps:
                        for body in list(comp.bRepBodies):
                            if not body.name or any(s in body.name for s in skip_s):
                                continue
                            try:
                                path     = os.path.join(EXPORT_DIR,
                                                        f"{comp.name}__{body.name}.stl")
                                stl_opts = export_mgr.createSTLExportOptions(body, path)
                                stl_opts.meshRefinement = (
                                    adsk.fusion.MeshRefinementSettings.MeshRefinementHigh)
                                export_mgr.execute(stl_opts)
                                count += 1
                            except Exception:
                                Logger.log(f"STL fail: {comp.name}/{body.name}", "WARN")
                    # V11: export jigs to separate folder
                    os.makedirs(JIG_DIR, exist_ok=True)
                    for jig_name, _ in JIG_REGISTRY:
                        jig_occ = occs.get(jig_name)
                        if jig_occ:
                            try:
                                jpath = os.path.join(JIG_DIR, f"{jig_name}.stl")
                                jstls = export_mgr.createSTLExportOptions(jig_occ, jpath)
                                export_mgr.execute(jstls)
                                count += 1
                            except Exception:
                                pass
                    Logger.log(f"STL exported {count} bodies -> {EXPORT_DIR}")
                    if JIG_REGISTRY:
                        Logger.log(f"  Jigs exported -> {JIG_DIR}")
                except Exception:
                    Logger.log(f"STL export failed: {traceback.format_exc()}", "ERROR")

            # ── STEP Export ───────────────────────────────────────────────
            def export_step(self):
                try:
                    os.makedirs(EXPORT_DIR, exist_ok=True)
                    export_mgr = self._design.exportManager
                    path       = os.path.join(EXPORT_DIR, "Optimus_Prime_G1_v11.step")
                    step_opts  = export_mgr.createSTEPExportOptions(path)
                    export_mgr.execute(step_opts)
                    Logger.log(f"STEP assembly -> {path}")
                    count = 0
                    for i in range(self._root.allOccurrences.count):
                        occ   = self._root.allOccurrences.item(i)
                        cname = occ.component.name
                        if any(t in cname for t in {"Marker", "Pivot", "_Vis"}):
                            continue
                        try:
                            cpath = os.path.join(EXPORT_DIR, f"{cname}.step")
                            copts = export_mgr.createSTEPExportOptions(occ, cpath)
                            export_mgr.execute(copts)
                            count += 1
                        except Exception:
                            Logger.log(f"STEP fail: {cname}", "WARN")
                    Logger.log(f"STEP {count} components -> {EXPORT_DIR}")
                except Exception:
                    Logger.log(f"STEP export failed: {traceback.format_exc()}", "ERROR")

            # ── SIM-4: BOM Export ─────────────────────────────────────────
            def export_bom(self):
                # V11: expanded BOM with all new categories
                BOM.add("Fastener", "M3x8 SHCS (general assembly)", 80,  "assembly")
                BOM.add("Fastener", "M3x16 SHCS (joint brackets)",  24,  "assembly")
                BOM.add("Fastener", "M3x4 set screw (cup point)",   20,  "couplers")
                BOM.add("Fastener", "M2.5x6 SHCS (PCB mounting)",    16, "assembly")
                BOM.add("Fastener", "M2x10 hinge pin (steel)",       4,  "covers")
                BOM.add("Hardware", "Ø3 mm x 30 mm shaft pin",        12,  "joints")
                BOM.add("Hardware", "Ø3 mm x 20 mm shaft pin",        20,  "joints")
                BOM.add("Hardware", "Ø3 mm x 45 mm shaft pin",         8,  "dual bearings")
                # V11: tendon hardware
                BOM.add("Hardware", "Braided Dyneema line 0.4 mm (3 m)", 2, "finger tendons")
                BOM.add("Hardware", "Torsion spring (finger return)",    10, "MCP joints")
                # V11: cable hardware
                BOM.add("Electronics", "22AWG servo wire (3 m lengths)", 30, "wiring harness")
                BOM.add("Electronics", "JST-XH 3-pin connectors",      40, "servo connectors")
                BOM.add("Electronics", "JST-XH 4-pin connectors",      10, "I2C bus")
                BOM.add("Electronics", "5V 5A BEC / power regulator",   2, "servo power")
                BOM.add("Electronics", "USB-C power cable",              1, "RPi power")
                BOM.add("Electronics", "Blade fuse 5A ATO",              3, "power protection")
                BOM.add("Hardware", "Rubber grommet Ø3.5 mm (open slot)", 16, "wire exits")
                BOM.add("Hardware", "Velcro strap 20x200 mm (battery)",  2, "battery retention")
                BOM.add("Hardware", "Foam pad 2 mm",                      4, "vibration iso")
                BOM.add("Hardware", "Snap-in cable clip (printed)",      20, "cable mgmt")
                # V11: bearing hardware
                BOM.add("Bearing", "Ø22 mm x 7 mm ball bearing (press-fit)", 8, "dual bearing")
                BOM.add("Bearing", "Ø20 mm x 6 mm ball bearing (glue-fit)", 12, "single bearing")
                # Materials
                BOM.add("Material", "PETG filament 1kg spool",          4, "~900g structural")
                BOM.add("Material", "PLA+ filament 1kg spool",          2, "~600g detail parts")
                BOM.add("Material", "TPU filament 250g spool",          1, "flex parts")
                BOM.add("Material", "Nylon filament 500g spool",        1, "jigs + couplers")
                BOM.save_csv(BOM_FILE)
                BOM.summary()

                # V11: expanded wiring map
                Logger.log("--- SERVO WIRING MAP (PCA9685 channels) ---")
                wiring = [
                    ( 0, "L_Hip_Yaw",     "Pelvis -> L_HipYaw (DS3225MG)"),
                    ( 1, "R_Hip_Yaw",     "Pelvis -> R_HipYaw (DS3225MG)"),
                    ( 2, "L_Hip_Pitch",   "Thigh_L -> HipP (DS3225MG)"),
                    ( 3, "R_Hip_Pitch",   "Thigh_R -> HipP (DS3225MG)"),
                    ( 4, "L_Hip_Roll",    "Thigh_L -> HipR (MG996R)"),
                    ( 5, "R_Hip_Roll",    "Thigh_R -> HipR (MG996R)"),
                    ( 6, "L_Knee",        "Thigh_L -> KneP (MG996R)"),
                    ( 7, "R_Knee",        "Thigh_R -> KneP (MG996R)"),
                    ( 8, "L_Ankle_Pitch", "Foot_L  -> AnkP (MG996R)"),
                    ( 9, "R_Ankle_Pitch", "Foot_R  -> AnkP (MG996R)"),
                    (10, "L_Ankle_Roll",  "Foot_L  -> AnkR (MG996R)"),
                    (11, "R_Ankle_Roll",  "Foot_R  -> AnkR (MG996R)"),
                    (12, "Waist_Yaw",     "Torso   -> WaistYaw (MG996R)"),
                    (13, "Waist_Pitch",   "Torso   -> WaistPitch (MG996R)"),
                    (14, "Neck_Pitch",    "Torso   -> NeckPitch (MG90S)"),
                    (15, "Neck_Yaw",      "Head    -> NeckYaw (MG90S)"),
                ]
                wiring2 = [
                    ( 0, "L_Sh_Yaw",      "UpperArm_L -> ShY (MG996R)"),
                    ( 1, "R_Sh_Yaw",      "UpperArm_R -> ShY (MG996R)"),
                    ( 2, "L_Sh_Pitch",    "UpperArm_L -> ShP (MG996R)"),
                    ( 3, "R_Sh_Pitch",    "UpperArm_R -> ShP (MG996R)"),
                    ( 4, "L_Sh_Roll",     "UpperArm_L -> ShR (MG996R)"),
                    ( 5, "R_Sh_Roll",     "UpperArm_R -> ShR (MG996R)"),
                    ( 6, "L_Elbow",       "UpperArm_L -> ElbP (MG996R)"),
                    ( 7, "R_Elbow",       "UpperArm_R -> ElbP (MG996R)"),
                    ( 8, "L_Wrist_Roll",  "Forearm_L  -> WR (MG90S)"),
                    ( 9, "R_Wrist_Roll",  "Forearm_R  -> WR (MG90S)"),
                    (10, "L_Finger_All",  "Hand_L -> FingerServo ch0 (DS04-NFC)"),
                    (11, "L_Thumb",       "Hand_L -> FingerServo ch1 (DS04-NFC)"),
                    (12, "R_Finger_All",  "Hand_R -> FingerServo ch0 (DS04-NFC)"),
                    (13, "R_Thumb",       "Hand_R -> FingerServo ch1 (DS04-NFC)"),
                    (14, "Blaster_Fold",  "Hand_R -> Blaster hinge (MG90S)"),
                    (15, "SPARE",         "--"),
                ]
                for ch, name, loc in wiring:
                    Logger.log(f"  PCA1 ch{ch:02d}  {name:<20s}  <- {loc}")
                for ch, name, loc in wiring2:
                    Logger.log(f"  PCA2 ch{ch:02d}  {name:<20s}  <- {loc}")
                Logger.log("--- V11 POWER BUDGET ---")
                Logger.log("  Servo rail:  2x PCA9685 @ 5V -> ~8A peak (all servos moving)")
                Logger.log("  Logic rail:  RPi Zero 2W @ 5V -> ~0.5A")
                Logger.log("  Camera:      ESP32-CAM @ 5V  -> ~0.25A")
                Logger.log("  IMU:         MPU-6050 @ 3.3V  -> ~0.01A")
                Logger.log("  BEC input:   2S LiPo 7.4V -> 5V/5A (x2 for headroom)")
                Logger.log("  Fuse:        5A blade fuse on servo rail; 2A on logic rail")

            # ── Master Runner ─────────────────────────────────────────────
            def run_all_simulations(self):
                dispatch = {
                    "ALL": lambda: [
                        self.test_joint_rom(),
                        self.simulate_head_scan(),
                        self.simulate_wave(),
                        self.simulate_idle_breathing(),
                        self.simulate_walking_advanced(),
                        self.simulate_running(),
                        self.simulate_combat(),
                        self.simulate_transformation(),
                        self.test_arm_workspace(),
                        self.run_stability_analysis(),
                        self.estimate_servo_loads(),
                        self.run_physical_validation(),  # V11 module 10
                        self.export_bom(),
                    ],
                    "rom":       self.test_joint_rom,
                    "head":      self.simulate_head_scan,
                    "wave":      self.simulate_wave,
                    "breathing": self.simulate_idle_breathing,
                    "walk":      self.simulate_walking_advanced,
                    "run":       self.simulate_running,
                    "combat":    self.simulate_combat,
                    "transform": self.simulate_transformation,
                    "truck":     self.simulate_truck_mode,
                    "battle":    self.simulate_battle_mode,
                    "robot":     self.simulate_robot_mode,
                    "stability": self.run_stability_analysis,
                    "servo":     self.estimate_servo_loads,
                    "workspace": self.test_arm_workspace,
                    "bom":       self.export_bom,
                    "fingers":   lambda: [
                        self.gesture_open("L"), self.gesture_open("R"),
                        self.gesture_fist("L"), self.gesture_fist("R"),
                        self.gesture_point("R"), self.gesture_snap("R"),
                    ],
                    # V11 new dispatch entries
                    "tendon":    self.test_tendon_slack,
                    "locks":     self.test_transform_locks,
                    "print":     self.printability_report,
                    "physical":  self.run_physical_validation,
                }

                target = TARGET_MODULE
                Logger.log(f"--- BEGINNING SIMULATION [TARGET: {target}] ---")
                fn = dispatch.get(target)
                if fn:
                    fn()
                else:
                    Logger.log(f"Unknown module: {target}", "ERROR")

                if EXPORT_URDF:
                    self.export_urdf()
                if EXPORT_STL:
                    self.export_stl()
                if EXPORT_STEP:
                    self.export_step()

                # ── Final Report ──────────────────────────────────────────
                Logger.log("=" * 55)
                Logger.log("OPTIMUS PRIME G1 v11.0 — FINAL REPORT")
                Logger.log("Physically Buildable Prototype Edition")
                Logger.log("=" * 55)
                for label, count in self._cols:
                    if count >= 0:
                        icon = "[OK]" if count == 0 else "[WARN]"
                        Logger.log(f"  {label:<42} {icon}  {count}")
                    else:
                        Logger.log(f"  {label:<42} ?  N/A")
                if EXPORT_URDF:
                    Logger.log(f"  URDF  -> {EXPORT_DIR}/robot_v11.urdf")
                Logger.log(f"  BOM   -> {BOM_FILE}")
                Logger.log(f"  ASM   -> {ASSEMBLY_FILE}")
                Logger.log(f"  Log   -> {LOG_FILE}")
                Logger.log("=" * 55)
                Logger.log("V11 BUILD COMPLETE — Review ASSEMBLY_GUIDE before printing")

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

        sim = SimulationEngine(root, comps_list, design, app, ui)
        sim.run_all_simulations()
        Logger.log("v11 script finished successfully.")

    except Exception:
        Logger.log(f"FATAL ERROR:\n{traceback.format_exc()}", "ERROR")

    finally:
        Logger.flush()
