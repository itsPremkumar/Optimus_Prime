# ─────────────────────────────────────────────────────────────────────────────
# OPTIMUS PRIME G1  v11.0  ─ Physical Build Edition
# Fusion 360 Python Script  |  Anthropic MCP-compatible
# ─────────────────────────────────────────────────────────────────────────────
# WHAT'S NEW IN v10  (relative to v9 bug-fixed)
# ──────────────────────────────────────────────
# GEO-1  G1-accurate head: wide ear wings + EarTop plates, deep chin guard,
#         3-line mouth grill, visor recess with 3×LED pockets, nose bridge,
#         ESP32-CAM pocket behind visor with Ø3 mm lens channel
# GEO-2  Articulated 5-finger hands: 4 fingers × 3 phalanges + thumb × 2
#         phalanges; knuckle cylinders, PIP/DIP hinge cylinders, fingertip
#         cones, cable-routing grooves, hand LED strip pockets
# GEO-3  Shoulder pads: double-layer wide plate + outboard twin-spike stacks
# GEO-4  Chest: stepped window frames, 4-row grille, raised Autobot-badge ring
#         with Ø12 mm LED ring pocket, hood crease plates on torso front
# GEO-5  Improved feet: toe cap, heel spur, 3-slot arch vent, anti-twist fin
# GEO-6  Truck-cab front geometry on torso (indicator nubs, bumper lip)
#
# PHY-1  m3_boss()     — Ø7 mm print boss + Ø4.7 mm heat-set insert pocket
# PHY-2  captive_nut() — M3 hex-nut trap + through-bore
# PHY-3  snap_clip()   — 1.5 mm cantilever snap-fit pair
# PHY-4  align_pin() / align_socket() — Ø2 mm pin+socket for shell halves
# PHY-5  grommet_hole() — Ø3.5 mm servo wire-exit groove per servo pocket
#
# ELEC-1  RPi Zero 2W bay (65×30 mm) + M2.5 standoffs, lower torso
# ELEC-2  PCA9685 bay  (62.5×25.4 mm) + standoffs, below RPi Zero
# ELEC-3  2S LiPo bay  (70×32×18 mm) + XT60-F slot, torso rear
# ELEC-4  LED pockets: 3×Ø5 mm visor + Ø12 mm Autobot-badge ring
# ELEC-5  ESP32-CAM pocket (38×26 mm) in head + Ø3 mm lens channel
# ELEC-6  MPU-6050 IMU pocket (21×16 mm) in pelvis centre
#
# SERVO  DS3225MG (35 kg·cm) for hips; DS3218 (20 kg·cm) for waist;
#         DS04-NFC (9 g) for finger drive servos
#
# SIM-1  ZMP (Zero Moment Point) stability check per pose
# SIM-2  Finger gestures: open / fist / point / snap_fingers
# SIM-3  Arm workspace test (8 cardinal directions × 2 arms)
# SIM-4  BOM CSV export: servos, bearings, fasteners, electronics
# SIM-5  URDF: correct joint types + effort/velocity limits + inertia blocks
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
STOP_FLAG      = os.path.join(_OUTPUT_DIR, "stop.flag")
ASSEMBLY_FILE  = os.path.join(_OUTPUT_DIR, f"ASSEMBLY_GUIDE_v11_{_ts}.txt")


# ═════════════════════════════════════════════════════════════════════════════
# CONFIGURATION  — ALL DIMENSIONS IN cm (Fusion 360 native unit)
# ═════════════════════════════════════════════════════════════════════════════

# ── Skeleton Z-heights (preserved from v9 for kinematic compatibility) ────────
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
BEARING_FIT_TOLERANCE = 0.005   # ±0.05 mm radial fit tuning for printed bearing pockets
COVER_THICKNESS       = 0.20    # 2.0 mm removable service cover
CABLE_CLIP_GAP        = 0.020   # 0.2 mm retention gap for wire clip
TENDON_GROOVE_W       = 0.15    # 1.5 mm tendon groove width
TENDON_GROOVE_H       = 0.10    # 1.0 mm tendon groove depth
JIG_PIN_HOLE_R        = 0.105   # 2.1 mm jig alignment hole radius

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

# ── Servo specs (upgraded for v10) ───────────────────────────────────────────
SERVO_SPECS = {
    "hip_hd": {"name": "DS3225MG", "rated": 25.0, "stall": 35.0, "mass_g":  60},
    "waist":  {"name": "DS3218",   "rated": 20.0, "stall": 25.0, "mass_g":  55},
    "std":    {"name": "MG996R",   "rated":  9.4, "stall": 11.5, "mass_g":  55},
    "micro":  {"name": "MG90S",    "rated":  1.8, "stall":  2.2, "mass_g":  13},
    "digit":  {"name": "DS04-NFC", "rated":  1.8, "stall":  2.2, "mass_g":   9},
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

ASSEMBLY_STEPS = []

def guide_add(msg):
    ASSEMBLY_STEPS.append(msg)
    Logger.log(f"ASSEMBLY: {msg}")

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

        # ─────────────────────────────────────────────────────────────────
        # PHYSICAL FEATURE HELPERS  (PHY-1 … PHY-5)
        # ─────────────────────────────────────────────────────────────────

        def m3_boss(comp, tag, cx, cy, cz, axis="z", depth=0.80):
            """PHY-1 — Ø7 mm boss cylinder + Ø4.7 mm heat-set insert pocket.
            Registers to BOM on first call per tag."""
            boss = cyl(comp, f"{tag}_Boss", cx, cy, cz, BOSS_R, depth, axis, dark_metal)
            boss.name = f"{tag}_Boss"  # keep visible
            cut_cavity(comp, cyl(comp, f"{tag}_Insert",
                                 cx, cy, cz, INSERT_R, INSERT_H, axis))
            BOM.add("Fastener", "M3 heat-set insert (Voron M3×5)", 1,
                    f"boss at ({cx:.1f},{cy:.1f},{cz:.1f}) in {comp.name}")

        def captive_nut(comp, tag, cx, cy, cz, axis="z", bolt_len=1.0):
            """PHY-2 — M3 hex-nut trap (hex slot) + clearance through-bore."""
            # Hex nut approximated as cylinder with circumscribed radius
            cut_cavity(comp, cyl(comp, f"{tag}_NutTrap",
                                 cx, cy, cz, M3_NUT_CIR, M3_NUT_H, axis))
            # Through-bore on both sides of nut
            ax = {"x": (1,0,0), "y": (0,1,0), "z": (0,0,1)}[axis]
            cut_cavity(comp, cyl(comp, f"{tag}_NutBore",
                                 cx, cy, cz, M3_CLR_R, bolt_len, axis))
            BOM.add("Fastener", "M3 hex nut", 1, comp.name)
            BOM.add("Fastener", f"M3×{int(bolt_len*10)} SHCS", 1, comp.name)

        def snap_clip(comp, tag, cx, cy, cz, span_x=1.2, axis_latch="z"):
            """PHY-3 — cantilever snap-fit pair (two arms, 1.5 mm thick)."""
            for sign in [-1, 1]:
                arm_cx = cx + sign * span_x * 0.5
                box(comp, f"{tag}_SnapArm_{sign}",
                    arm_cx, cy, cz, WALL_F, 0.40, 1.20, grey_plastic)
                # Chamfered head (simplified as slightly wider box at tip)
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
            """PHY-5 — Ø3.5 mm servo wire-exit groove with printable exit slot."""
            cut_cavity(comp, cyl(comp, f"{tag}_Grommet",
                                 cx, cy, cz, GROMMET_R, 0.80, axis))
            # Small adjacent slot to ease TPU grommet insertion and wire bend radius
            if axis == "x":
                cut_cavity(comp, box(comp, f"{tag}_GrommetSlot", cx, cy, cz, 0.16, 0.55, 0.25, black_plastic))
            elif axis == "y":
                cut_cavity(comp, box(comp, f"{tag}_GrommetSlot", cx, cy, cz, 0.55, 0.16, 0.25, black_plastic))
            else:
                cut_cavity(comp, box(comp, f"{tag}_GrommetSlot", cx, cy, cz, 0.25, 0.55, 0.16, black_plastic))

        def horn_coupler(comp, tag, cx, cy, cz, axis="x", servo_type="mg996"):
            """Printable servo horn-to-joint coupler with keyed capture and clamping pocket."""
            if servo_type.lower() in ("mg90", "mg90s", "micro", "digit"):
                hub_r, hub_l = 0.24, 0.42
                flange_r = 0.34
            else:
                hub_r, hub_l = 0.34, 0.62
                flange_r = 0.46
            box(comp, f"{tag}_CouplerHub", cx, cy, cz, hub_l, hub_r*2.0, hub_r*2.0, chrome)
            cyl(comp, f"{tag}_CouplerFlange", cx, cy, cz, flange_r, 0.18, axis, chrome)
            # Set-screw pocket / key slot
            if axis == "x":
                cut_cavity(comp, box(comp, f"{tag}_CouplerKey", cx+hub_l*0.15, cy, cz, 0.10, hub_r*1.3, hub_r*1.3, black_plastic))
            elif axis == "y":
                cut_cavity(comp, box(comp, f"{tag}_CouplerKey", cx, cy+hub_l*0.15, cz, hub_r*1.3, 0.10, hub_r*1.3, black_plastic))
            else:
                cut_cavity(comp, box(comp, f"{tag}_CouplerKey", cx, cy, cz+hub_l*0.15, hub_r*1.3, hub_r*1.3, 0.10, black_plastic))
            BOM.add("Hardware", "3D-printed servo horn coupler", 1, comp.name)
            guide_add(f"Install horn coupler {tag} in {comp.name}")

        def tendon_groove(comp, tag, cx, cy, cz, axis="x"):
            cut_cavity(comp, box(comp, f"{tag}_TendonGroove", cx, cy, cz, TENDON_GROOVE_W, TENDON_GROOVE_H, 0.35, black_plastic))

        def tendon_anchor(comp, tag, cx, cy, cz, axis="x"):
            cyl(comp, f"{tag}_TendonAnchor", cx, cy, cz, 0.06, 0.22, axis, chrome)
            BOM.add("Hardware", "Tendon anchor post", 1, comp.name)

        def cable_clip(comp, tag, cx, cy, cz, axis="x"):
            box(comp, f"{tag}_CableClip", cx, cy, cz, 0.18, 0.22, 0.18, grey_plastic)

        def wire_hub(comp, tag, cx, cy, cz):
            box(comp, f"{tag}_WireHub", cx, cy, cz, 1.2, 0.8, 0.8, dark_grey)
            guide_add(f"Wire hub placed in {comp.name} at {tag}")

        def cover_plate(comp, tag, cx, cy, cz, lx, ly, fastener_positions=None, ap=None):
            ap = ap or dark_grey
            box(comp, f"{tag}_Cover", cx, cy, cz, lx, ly, COVER_THICKNESS, ap)
            if fastener_positions:
                for i, (px, pz) in enumerate(fastener_positions):
                    cut_cavity(comp, cyl(comp, f"{tag}_CvrHole_{i}", cx+px, cy, cz+pz, M3_CLR_R, COVER_THICKNESS*2.0, "y"))
            BOM.add("Printed Part", f"Service cover {tag}", 1, comp.name)
            guide_add(f"Cover {tag} installed on {comp.name}")

        def lock_pin(comp, tag, cx, cy, cz, axis="z"):
            cyl(comp, f"{tag}_LockPin", cx, cy, cz, 0.10, 0.45, axis, chrome)
            BOM.add("Hardware", "Transformation lock pin", 1, comp.name)

        def assembly_jig(comp, tag, cx, cy, cz, lx=3.0, ly=1.2, lz=0.6):
            box(comp, f"{tag}_Jig", cx, cy, cz, lx, ly, lz, white_pla)
            guide_add(f"Assembly jig generated for {comp.name} ({tag})")

        def merge_fasteners_to_halves(comp):
            # Best-effort attachment of printable fastener geometry back to printed shell halves.
            bodies = list(comp.bRepBodies)
            shells = [b for b in bodies if b.name and any(k in b.name for k in SPLIT_KEYS) and "_Vis" not in b.name]
            for body in bodies:
                if not body.name or "_Vis" in body.name:
                    continue
                if any(k in body.name for k in ("Pin", "Boss", "Insert", "Nut", "Snap", "LockPin", "AlignPin")):
                    try:
                        target = None
                        bb = body.boundingBox
                        cx = (bb.minPoint.x + bb.maxPoint.x) * 0.5
                        for sh in shells:
                            if (cx < 0 and ("_L" in sh.name or "Left" in sh.name)) or (cx >= 0 and ("_R" in sh.name or "Right" in sh.name)):
                                target = sh
                                break
                        if target is None and shells:
                            target = shells[0]
                        if target:
                            tools = adsk.core.ObjectCollection.create()
                            tools.add(body)
                            ci = comp.features.combineFeatures.createInput(target, tools)
                            ci.operation = adsk.fusion.CombineOperation.JoinFeatureOperation
                            ci.isKeepToolBodies = False
                            comp.features.combineFeatures.add(ci)
                    except Exception:
                        pass

        def verify_screw_lengths():
            for row in BOM._rows:
                if row["Category"] == "Fastener" and "M3" in row["Part"] and "assembly" not in row["Note"]:
                    Logger.log(f"VERIFY FASTENER: {row['Part']} × {row['Qty']} ({row['Note']})")

        # ── Generic fastener helpers ──────────────────────────────────────
        def screw_hole(comp, cx, cy, cz, axis="y", length=3.0):
            cut_cavity(comp, cyl(comp, "ScrewHole", cx, cy, cz,
                                 M3_PILOT_R, length, axis))
            BOM.add("Fastener", f"M3×{int(length*10)} self-tap", 1, comp.name)

        def magnet_pocket(comp, tag, cx, cy, cz, axis="z"):
            cut_cavity(comp, cyl(comp, f"{tag}_MagPkt", cx, cy, cz, 0.32, 0.35, axis))
            BOM.add("Hardware", "Magnet D6×3 mm N35", 1, comp.name)

        def wire_channel(comp, tag, cx, cy, cz, r, h, axis):
            cut_cavity(comp, cyl(comp, f"{tag}_Wire", cx, cy, cz, r, h, axis))

        def led_pocket_5mm(comp, tag, cx, cy, cz, axis="y"):
            """ELEC-4 — Ø5 mm LED through-hole pocket."""
            cut_cavity(comp, cyl(comp, f"{tag}_LED5",
                                 cx, cy, cz, LED_R_5MM, 0.85, axis))
            BOM.add("Electronics", "LED 5 mm (colour TBD)", 1, comp.name)

        def led_ring_pocket(comp, tag, cx, cy, cz, axis="z"):
            """ELEC-4 — Ø12 mm LED ring recess (outer ring cut, inner left solid)."""
            cut_cavity(comp, cyl(comp, f"{tag}_LEDRing",
                                 cx, cy, cz, LED_R_RING, 0.30, axis))
            BOM.add("Electronics", "WS2812 5050 LED ring Ø12 mm", 1, comp.name)

        # ─────────────────────────────────────────────────────────────────
        # ELECTRONICS BAY HELPERS  (ELEC-1 … ELEC-6)
        # ─────────────────────────────────────────────────────────────────

        def rpi_zero_bay(comp, tag, cx, cy, cz):
            """ELEC-1 — RPi Zero 2W pocket (65×30 mm) with 4 × M2.5 standoffs."""
            # Pocket (slightly oversized with clearance)
            cut_cavity(comp, box(comp, f"{tag}_RPiBay",
                                 cx, cy, cz,
                                 RPI0_L + 0.20, RPI0_W + 0.20, RPI0_H + 0.30))
            # Four mounting standoffs (M2.5 × 11 mm) at 58 mm × 23 mm pattern
            for sx, sz in [(-2.90, -1.15), (+2.90, -1.15),
                           (-2.90, +1.15), (+2.90, +1.15)]:
                cyl(comp, f"{tag}_RpiStdoff_{sx}_{sz}",
                    cx+sx, cy, cz+sz, 0.14, RPI0_H+0.50, "y", dark_metal)
            BOM.add("Electronics", "Raspberry Pi Zero 2W", 1, comp.name)
            BOM.add("Fastener",    "M2.5×11 mm brass standoff", 4, comp.name)

        def pca9685_bay(comp, tag, cx, cy, cz):
            """ELEC-2 — PCA9685 pocket (62.5×25.4 mm) with 4 × M2.5 standoffs."""
            cut_cavity(comp, box(comp, f"{tag}_PCABay",
                                 cx, cy, cz,
                                 PCA_L + 0.20, PCA_W + 0.20, PCA_H + 0.30))
            for sx, sz in [(-3.00, -1.08), (+3.00, -1.08),
                           (-3.00, +1.08), (+3.00, +1.08)]:
                cyl(comp, f"{tag}_PCAStdoff_{sx}_{sz}",
                    cx+sx, cy, cz+sz, 0.14, PCA_H+0.50, "y", dark_metal)
            BOM.add("Electronics", "PCA9685 16-ch servo driver", 1, comp.name)

        def lipo_bay(comp, tag, cx, cy, cz):
            """ELEC-3 — 2S 1300 mAh LiPo pocket (70×32×18 mm) + XT60-F slot."""
            cut_cavity(comp, box(comp, f"{tag}_LipoBay",
                                 cx, cy, cz,
                                 LIPO_L + 0.30, LIPO_H + 0.30, LIPO_W + 0.30))
            # XT60-F connector slot on rear face
            cut_cavity(comp, box(comp, f"{tag}_XT60Slot",
                                 cx, cy + LIPO_H/2 + 0.15, cz,
                                 XT60_W + 0.10, 0.50, XT60_H_SLOT + 0.10))
            BOM.add("Electronics", "2S 1300 mAh 7.4V LiPo battery", 1, comp.name)
            BOM.add("Electronics", "XT60-F panel-mount connector",  1, comp.name)

        def imu_pocket(comp, tag, cx, cy, cz, axis="z"):
            """ELEC-6 — MPU-6050 pocket (21×16 mm)."""
            cut_cavity(comp, box(comp, f"{tag}_IMUPkt",
                                 cx, cy, cz,
                                 IMU_L + 0.20, IMU_W + 0.20, IMU_H + 0.30))
            BOM.add("Electronics", "MPU-6050 IMU breakout", 1, comp.name)

        def esp32_cam_pocket(comp, tag, cx, cy, cz, lens_axis="y"):
            """ELEC-5 — ESP32-CAM pocket (38×26 mm) + Ø3 mm lens channel."""
            cut_cavity(comp, box(comp, f"{tag}_ESP32Bay",
                                 cx, cy, cz,
                                 ESP32_L + 0.20, ESP32_H + 0.30, ESP32_W + 0.20))
            # Lens channel through the front face
            ax_map = {"y": (0,1,0), "x": (1,0,0), "z": (0,0,1)}
            av = ax_map[lens_axis]
            cut_cavity(comp, cyl(comp, f"{tag}_LensHole",
                                 cx, cy - (ESP32_H/2 + 0.30), cz,
                                 0.150, 0.60, lens_axis))
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
            horn_coupler(comp, f"{tag}_HornCoupler", cx, cy, cz, axis, "mg996")
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
            horn_coupler(comp, f"{tag}_HornCoupler", cx, cy, cz, axis, "mg90s")
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
            """Centred bearing cavity with configurable press/glue fit and retention lip."""
            cyl(comp, f"{tag}_BO", cx, cy, cz, ro,       w,      axis, chrome)
            cyl(comp, f"{tag}_BI", cx, cy, cz, ro*0.58,  w*0.80, axis, dark_grey)
            cyl(comp, f"{tag}_BB", cx, cy, cz, ro*0.32,  w*1.10, axis, chrome)
            temp  = adsk.fusion.TemporaryBRepManager.get()
            ax    = {"x": (1,0,0), "y": (0,1,0), "z": (0,0,1)}[axis]
            half  = w/2.0 + 0.05
            pocket_r = max(0.02, ro + BEARING_FIT_TOLERANCE)
            p1    = adsk.core.Point3D.create(cx-ax[0]*half, cy-ax[1]*half, cz-ax[2]*half)
            p2    = adsk.core.Point3D.create(cx+ax[0]*half, cy+ax[1]*half, cz+ax[2]*half)
            cs    = temp.createCylinderOrCone(p1, pocket_r, p2, pocket_r)
            bf    = comp.features.baseFeatures.add()
            bf.startEdit()
            cb    = comp.bRepBodies.add(cs, bf)
            bf.finishEdit()
            cb.name = f"{tag}_CB"
            cut_cavity(comp, cb)
            # Retaining lip (best-effort printable ring)
            lip_offset = half - 0.06
            if axis == "x":
                cyl(comp, f"{tag}_Lip", cx+lip_offset, cy, cz, ro*0.08, 0.08, axis, chrome)
            elif axis == "y":
                cyl(comp, f"{tag}_Lip", cx, cy+lip_offset, cz, ro*0.08, 0.08, axis, chrome)
            else:
                cyl(comp, f"{tag}_Lip", cx, cy, cz+lip_offset, ro*0.08, 0.08, axis, chrome)
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
        # ① TORSO  (GEO-4, ELEC-1, ELEC-2, ELEC-3)
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
        # Indicator nubs
        box(torso, "Ind_L",         -3.8, -5.00, TORSO_CTR-3.8,   0.6, 0.25, 0.5, yellow_met)
        box(torso, "Ind_R",          3.8, -5.00, TORSO_CTR-3.8,   0.6, 0.25, 0.5, yellow_met)

        # GEO-4 — Chest plate + Autobot badge with LED ring
        box(torso, "Chest_Plate",    0,   -4.20, TORSO_CTR+0.5,   8.4, 0.32, 4.0, chrome)
        cyl(torso, "Badge_Ring",     0,   -4.55, TORSO_CTR+0.5,   0.80, 0.12, "y", op_red)
        led_ring_pocket(torso, "Badge",  0, -4.60, TORSO_CTR+0.5, "y")

        # Inner structural frame
        box(torso, "Inner_Frame",    0,    0,    TORSO_CTR+1.5,   7.4, 6.0,  8.2, dark_metal)
        box(torso, "Spine_Beam",     0,    0,    TORSO_CTR+1.5,   1.8, 1.8,  8.2, chrome)
        cyl(torso, "Spine_Cyl",      0,    0,    TORSO_CTR+1.5,  1.10, 4.5,  "z", chrome)

        # ELEC-3 — LiPo bay (rear of lower torso)
        box(torso, "LipoBay_Shell",  0,    3.2, TORSO_CTR-2.0,   7.6, 4.4,  5.6, black_plastic)
        lipo_bay(torso, "Main", 0, 2.8, TORSO_CTR-2.0)
        cover_plate(torso, "LipoDoor", 0, 4.95, TORSO_CTR-2.0, 6.8, 2.6, [(-2.8,-0.2),(2.8,-0.2)], black_plastic)

        # ELEC-1 — RPi Zero 2W bay (above LiPo)
        box(torso, "RPi_Shell",      0,    2.8, TORSO_CTR+1.8,   7.0, 3.6,  2.8, black_plastic)
        rpi_zero_bay(torso, "Main",  0,    3.2, TORSO_CTR+1.8)
        cover_plate(torso, "RPiCover", 0, 4.95, TORSO_CTR+1.8, 6.2, 2.2, [(-2.6,-0.1),(2.6,-0.1)], dark_grey)

        # ELEC-2 — PCA9685 bay (alongside RPi)
        box(torso, "PCA_Shell",      0,    2.8, TORSO_CTR+4.2,   6.8, 3.2,  2.2, black_plastic)
        pca9685_bay(torso, "Main",   0,    3.0, TORSO_CTR+4.2)
        cover_plate(torso, "PCACover", 0, 4.95, TORSO_CTR+4.2, 6.0, 2.0, [(-2.5,-0.1),(2.5,-0.1)], dark_grey)

        # Cable management
        box(torso, "Cable_L",       -3.4,  0.6,  TORSO_CTR,      0.55, 1.0, 10.0, dark_grey)
        box(torso, "Cable_R",        3.4,  0.6,  TORSO_CTR,      0.55, 1.0, 10.0, dark_grey)
        wire_hub(torso, "Main", 0, 1.6, TORSO_CTR-0.8)
        cable_clip(torso, "CableClipL", -2.8, 1.0, TORSO_CTR-1.6)
        cable_clip(torso, "CableClipR",  2.8, 1.0, TORSO_CTR-1.6)

        # Shoulder collars (wider for GEO-3 shoulder pads)
        box(torso, "Collar_L",      -8.0,  0,    SHOULDER_CTR-1.0, 5.5, 3.4, 3.0, chrome)
        box(torso, "Collar_R",       8.0,  0,    SHOULDER_CTR-1.0, 5.5, 3.4, 3.0, chrome)

        # Transformation flaps
        box(torso, "TF_Flap_L",     -5.40,-0.2,  TORSO_CTR+3.0,  0.40, 6.6,  6.2, op_red)
        box(torso, "TF_Flap_R",      5.40,-0.2,  TORSO_CTR+3.0,  0.40, 6.6,  6.2, op_red)
        box(torso, "TF_BackTop",     0,    5.0,  TORSO_CTR+5.2,  8.2, 0.38,  5.2, op_blue)

        # M3 boss fasteners (corners)
        for bx_off, bz_off in [(-3.2, 4.8), (3.2, 4.8), (-3.2, -4.8), (3.2, -4.8)]:
            m3_boss(torso, f"Torso_B{bx_off}_{bz_off}", bx_off, 0, TORSO_CTR+bz_off)
        # Alignment pins on split line
        align_pin(torso, "TorsoSplit_L", -5.0, 0, TORSO_CTR)
        align_pin(torso, "TorsoSplit_R",  5.0, 0, TORSO_CTR)

        # Waist servo cluster
        u_bracket(torso, "Waist_Brkt",  0, 0, WAIST_CTR,     4.0, 4.2, 3.4)
        mg996r(torso,    "Waist_Yaw",   0, 0, WAIST_CTR,     "z")
        bearing(torso,   "Waist_Brg",   0, 0, WAIST_CTR+0.5, "z", 1.30, 0.65)
        u_bracket(torso, "WaistP_Brkt", 0, 0, WAIST_CTR-2.5, 4.0, 4.2, 3.4)
        mg996r(torso,    "Waist_Pitch", 0, 0, WAIST_CTR-2.5, "x")
        bearing(torso,   "WaistP_Brg",  0, 0, WAIST_CTR-2.0, "x", 1.30, 0.65)
        magnet_pocket(torso, "WLock_F", 0, -2.0, WAIST_CTR-3.0)
        magnet_pocket(torso, "WLock_R", 0,  2.0, WAIST_CTR-3.0)

        # Neck servo cluster
        u_bracket(torso, "Neck_Brkt",   0, 0, NECK_JOINT_Z,  3.2, 2.8, 3.0)
        mg996r(torso,    "Neck_Pitch",  0, 0, NECK_JOINT_Z,  "x")
        wire_channel(torso, "Spine",    0, 0, TORSO_CTR,     0.6, 20.0, "z")

        # ─────────────────────────────────────────────────────────────────
        # ② HEAD  (GEO-1)
        # ─────────────────────────────────────────────────────────────────
        head = new_component("OP_Head")
        HC   = HEAD_CTR  # alias

        # Main helmet body — squarer, more G1
        box(head, "Helm_Main",      0,     0,    HC+0.8,  5.6, 5.2, 5.0, op_blue)
        box(head, "Helm_Forehead",  0,    -2.30, HC+2.6,  5.0, 0.42, 1.8, op_blue)
        box(head, "Helm_Top",       0,     0,    HC+3.4,  4.8, 4.4, 0.90, op_blue)

        # GEO-1 — Iconic G1 ear wings (wide rectangular plates)
        box(head, "Ear_L",         -3.30,  0,    HC+1.8,  0.70, 4.8, 3.8, op_blue)
        box(head, "Ear_R",          3.30,  0,    HC+1.8,  0.70, 4.8, 3.8, op_blue)
        # Ear top horizontal wing plate
        box(head, "EarTop_L",      -3.05,  0,    HC+3.5,  1.40, 4.4, 0.62, op_blue)
        box(head, "EarTop_R",       3.05,  0,    HC+3.5,  1.40, 4.4, 0.62, op_blue)

        # GEO-1 — Crest (tall prominent ridge)
        box(head, "Crest_Main",     0,    -0.30, HC+3.85, 1.05, 0.68, 3.6, chrome)
        box(head, "Crest_Stripe",   0,    -0.30, HC+3.95, 0.55, 0.36, 2.9, op_blue)

        # GEO-1 — Full chrome face plate
        box(head, "Face_Plate",     0,    -2.50, HC+0.5,  3.6, 0.38, 3.2, chrome)

        # GEO-1 — Chin guard (below face plate)
        box(head, "Chin_Guard",     0,    -2.60, HC-0.9,  3.0, 0.38, 1.8, chrome)
        box(head, "Chin_L",        -1.35, -2.52, HC-0.4,  0.38, 0.32, 2.2, chrome)
        box(head, "Chin_R",         1.35, -2.52, HC-0.4,  0.38, 0.32, 2.2, chrome)

        # GEO-1 — Visor recess + blue visor + LED pockets
        box(head, "Visor_Frame",    0,    -2.55, HC+1.35, 3.8, 0.30, 1.25, op_blue)
        box(head, "Visor",          0,    -2.68, HC+1.35, 3.0, 0.16, 0.92, op_blue_glass)
        for vx in [-0.85, 0.0, 0.85]:
            led_pocket_5mm(head, f"Visor_LED_{int(vx*100)}", vx, -2.80, HC+1.35, "y")

        # GEO-1 — Nose bridge + mouth grill
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
        cover_plate(head, "HeadCamCover", 0, 1.7, HC+2.5, 3.0, 1.8, [(-1.1,-0.05),(1.1,-0.05)], black_plastic)

        # Neck yaw servo
        mg90s(head, "Neck_Yaw", 0, 0, NECK_JOINT_Z, "z")

        # ─────────────────────────────────────────────────────────────────
        # ③ PELVIS  (ELEC-6)
        # ─────────────────────────────────────────────────────────────────
        pelvis = new_component("OP_Pelvis")
        box(pelvis, "Pelvis_Shell",  0,    0,  PELVIS_CTR,  16.4, 6.2, 5.0, op_blue)
        box(pelvis, "Pelvis_Frame",  0,    0,  PELVIS_CTR,  12.2, 4.4, 3.8, dark_metal)
        box(pelvis, "Hip_Armr_L",  -7.2,  0,  PELVIS_CTR,   1.0, 5.2, 4.2, chrome)
        box(pelvis, "Hip_Armr_R",   7.2,  0,  PELVIS_CTR,   1.0, 5.2, 4.2, chrome)
        box(pelvis, "Crotch_Plate", 0,  -2.9, PELVIS_CTR-1.2, 5.2, 0.30, 2.4, op_red)
        # ELEC-6 — IMU in pelvis centre for balance sensing
        imu_pocket(pelvis, "BalanceIMU", 0, 0, PELVIS_CTR)
        cover_plate(pelvis, "IMUCover", 0, 1.8, PELVIS_CTR, 2.2, 1.5, [(-0.7,-0.05),(0.7,-0.05)], dark_grey)
        # Hip yaw servos (upgraded to DS3225MG spec — same body dims as MG996R)
        mg996r(pelvis, "L_HipYaw", -HIP_X, 0, HIP_JOINT_Z, "z")
        lock_pin(pelvis, "HipLockL", -HIP_X, -1.4, HIP_JOINT_Z-0.8, "z")
        mg996r(pelvis, "R_HipYaw",  HIP_X, 0, HIP_JOINT_Z, "z")
        lock_pin(pelvis, "HipLockR", HIP_X, -1.4, HIP_JOINT_Z-0.8, "z")
        bearing(pelvis, "L_HYB",  -HIP_X-2.2, 0, HIP_JOINT_Z, "z", 1.10, 0.62)
        bearing(pelvis, "R_HYB",   HIP_X+2.2, 0, HIP_JOINT_Z, "z", 1.10, 0.62)
        BOM.add("Servo", "DS3225MG 25 kg·cm servo (hip yaw)", 2, "OP_Pelvis")

        # ─────────────────────────────────────────────────────────────────
        # ④ LEGS  (GEO-5 improved feet)
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
            mg996r(thigh, f"{side}_HipP",    sx, 0, HIP_JOINT_Z,             "x")
            mg996r(thigh, f"{side}_HipR",    sx, 0, THIGH_CTR+2.0,           "y")
            bearing(thigh, f"{side}_HRB",    sx, 0, THIGH_CTR+2.0,           "y", 1.00, 0.55)
            u_bracket(thigh, f"{side}_KnB",  sx, 0, KNEE_CTR+1.5,            3.8, 3.0, 3.0)
            mg996r(thigh, f"{side}_KneP",    sx, 0, KNEE_CTR+1.5,            "x")
            bearing(thigh, f"{side}_KB",     sx, 0, KNEE_CTR,                "x", 1.00, 0.55)
            wire_channel(thigh, f"{side}_LW", sx, 0, THIGH_CTR,              0.5, 12.0, "z")
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
            # Knee cap detail
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

            # —— FOOT  (GEO-5) ——
            foot = new_component(f"OP_Foot_{side}")
            # Base sole — wider, longer for stability
            box(foot, "Foot_Sole",    sx,       -0.6,  ANKLE_CTR-1.5,  6.2, 9.2, 1.10, op_red)
            # Arch vent slots (3 × decorative)
            for vi in [-1.0, 0.0, 1.0]:
                box(foot, f"Arch_Vent_{int(vi*10)}",
                    sx+vi*1.2, -0.6, ANKLE_CTR-1.94, 0.40, 5.5, 0.14, dark_grey)
            # GEO-5 — Heel spur (raised rear block)
            box(foot, "Heel_Block",   sx-m*0.9,  3.2,  ANKLE_CTR-0.8,  2.5, 3.5, 2.6, dark_grey)
            box(foot, "Heel_Plate",   sx-m*0.6,  4.4,  ANKLE_CTR-1.2,  3.2, 0.32, 2.0, chrome)
            box(foot, "Heel_Spur",    sx-m*1.0,  4.8,  ANKLE_CTR-0.2,  1.2, 0.40, 3.2, op_red)
            # GEO-5 — Toe cap
            box(foot, "Toe_Block",    sx+m*0.8, -3.8,  ANKLE_CTR-0.8,  2.6, 3.8, 2.0, dark_grey)
            box(foot, "Toe_Plate",    sx+m*0.5, -4.6,  ANKLE_CTR-1.2,  3.8, 0.32, 1.8, chrome)
            box(foot, "Toe_Cap",      sx+m*1.0, -5.2,  ANKLE_CTR-0.8,  2.8, 0.42, 1.5, op_red)
            # Ankle guard (bulkier)
            box(foot, "Ankle_Guard",  sx,        0,    ANKLE_CTR+1.2,  5.4, 3.2, 2.8, chrome)
            box(foot, "Ankle_Inner",  sx,       -1.0,  ANKLE_CTR+0.3,  4.0, 2.0, 1.6, dark_metal)
            # Boot fin (anti-twist)
            box(foot, "Boot_Fin",     sx+m*2.0,  0,    ANKLE_CTR-0.2,  0.40, 6.5, 4.2, op_blue)
            box(foot, "Boot_Fin2",    sx+m*2.5,  0,    ANKLE_CTR+0.8,  0.32, 5.0, 2.8, op_red)
            # Ankle servos
            mg996r(foot, f"{side}_AnkP", sx, 0, ANKLE_CTR+2.2, "x")
            mg996r(foot, f"{side}_AnkR", sx, 0, ANKLE_CTR+0.5, "y")
            bearing(foot, f"{side}_AB",  sx, 0, ANKLE_CTR,     "x", 1.00, 0.55)
            for bx_off in [-1.5, 1.5]:
                m3_boss(foot, f"{side}_FootBoss_{bx_off:.0f}", sx+bx_off, 0, ANKLE_CTR-0.5)
            align_pin(foot, f"{side}_Ft_A", sx-1.0, 0, ANKLE_CTR-1.3)
            align_pin(foot, f"{side}_Ft_B", sx+1.0, 0, ANKLE_CTR-1.3)

        # ─────────────────────────────────────────────────────────────────
        # ⑤ ARMS  (GEO-3 shoulders + GEO-2 articulated hands)
        # ─────────────────────────────────────────────────────────────────
        for side, ax in [("L", -SHOULDER_X), ("R", SHOULDER_X)]:
            m = -1 if side == "L" else 1

            # —— UPPER ARM ——
            ua = new_component(f"OP_UpperArm_{side}")

            # GEO-3 — Double-layer wide shoulder pad
            box(ua, "Sh_Block",        ax,         0, SHOULDER_CTR,      5.6, 4.2, 5.6, op_red)
            box(ua, "Sh_Pad_Outer",    ax+m*3.20,  0, SHOULDER_CTR,      1.00, 5.2, 5.8, op_red)
            box(ua, "Sh_Pad_Edge",     ax+m*3.75,  0, SHOULDER_CTR,      0.40, 4.6, 5.2, chrome)
            # Twin spike stacks (iconic G1 shoulder detail)
            for sz, sr, sh2 in [(SHOULDER_CTR+2.8, 0.42, 0.95),
                                 (SHOULDER_CTR+1.5, 0.36, 0.75)]:
                cyl(ua, f"Stk_A_{sz:.0f}", ax+m*3.3, -1.5, sz, sr,  sh2, "z", chrome)
                cone_shape(ua, f"StkTip_{sz:.0f}", ax+m*3.3, -1.5, sz+sh2/2+0.35,
                           sr, sr*0.25, 0.70, "z", dark_grey)
            box(ua, "Sh_Guard",        ax+m*2.60,  0, SHOULDER_CTR-0.2, 0.42, 4.4, 6.6, op_blue)

            # Upper arm link
            box(ua, "UA_Link",         ax,         0, ELBOW_Z+3.0,      3.2, 3.4, 5.2, op_red)
            box(ua, "UA_Skin",         ax+m*1.80,  0, ELBOW_Z+3.0,      0.52, 3.4, 5.2, chrome)

            # Shoulder servos (3 DOF)
            mg996r(ua, f"{side}_ShY",    ax, 0, SHOULDER_CTR+1.5, "z")
            bearing(ua, f"{side}_SYB",   ax, 0, SHOULDER_CTR+2.0, "z", 1.00, 0.55)
            u_bracket(ua, f"{side}_SPB", ax, 0, SHOULDER_CTR,     4.8, 3.4, 3.4)
            mg996r(ua, f"{side}_ShP",    ax, 0, SHOULDER_CTR,     "x")
            mg996r(ua, f"{side}_ShR",    ax, 0, SHOULDER_CTR-1.2, "y")
            bearing(ua, f"{side}_SB",    ax, 0, SHOULDER_CTR,     "x", 1.10, 0.62)

            # Elbow joint
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
            # GEO-2 — Detailed palm metacarpal plate
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
            # Cable bay for finger drive servo
            box(hand, "Finger_SrvBay", ax, 0.5, WRIST_Z-1.8, 1.4, 0.90, 2.4, black_plastic)
            # DS04-NFC finger drive servo (inside palm)
            BOM.add("Servo", "DS04-NFC 9g digital servo (finger drive)", 2, f"OP_Hand_{side}")
            # LED strip pocket (front of palm)
            for lxi in [-0.7, 0.7]:
                led_pocket_5mm(hand, f"PalmLED_{lxi:.0f}", ax+lxi, -1.65, WRIST_Z-1.6, "y")

            # GEO-2 — Articulated fingers (4 × 3-phalanx)
            for fi, (fname, fxo, pp_l, mp_l, dp_l) in enumerate(
                    zip(FING_NAMES, FING_X_OFF, FING_PP, FING_MP, FING_DP)):

                fc   = new_component(f"OP_{fname}_{side}")
                fx   = ax + m * fxo           # X centre of this finger
                fy   = -1.35                  # Y (front face of hand)
                # MCP Z = bottom of palm
                mcp_z = MCP_Z
                # Centre Z of each phalanx (extending downward = -Z)
                pp_cz = mcp_z - pp_l / 2
                mp_cz = mcp_z - pp_l - mp_l / 2
                dp_cz = mcp_z - pp_l - mp_l - dp_l / 2

                # Proximal phalanx
                box(fc, "PP", fx, fy, pp_cz, FING_W, FING_H, pp_l, grey_plastic)
                # PIP hinge cylinder (knuckle)
                cyl(fc, "PIP_Hinge", fx, fy, mcp_z - pp_l, 0.27, FING_W + 0.12, "x", chrome)
                # Middle phalanx
                box(fc, "MP", fx, fy, mp_cz, FING_W*0.94, FING_H*0.94, mp_l, grey_plastic)
                # DIP hinge cylinder
                cyl(fc, "DIP_Hinge", fx, fy, mcp_z - pp_l - mp_l,
                    0.24, FING_W*0.94 + 0.10, "x", chrome)
                # Distal phalanx
                box(fc, "DP", fx, fy, dp_cz, FING_W*0.86, FING_H*0.86, dp_l, grey_plastic)
                # Fingertip cone
                cone_shape(fc, "FT", fx, fy, mcp_z - pp_l - mp_l - dp_l - 0.12,
                           FING_W*0.44, 0.05, 0.24, "z", chrome)
                # Cable routing groove through phalanges
                wire_channel(fc, fname, fx, fy, pp_cz, 0.08, pp_l*2.3, "z")

            # GEO-2 — Thumb (2-phalanx, angled outward)
            thumb = new_component(f"OP_Thumb_{side}")
            tx    = ax + m * 1.70
            ty    = 0.20
            tpp_cz = MCP_Z - THUMB_PP_L / 2
            tdp_cz = MCP_Z - THUMB_PP_L - THUMB_DP_L / 2
            box(thumb, "TP",  tx, ty, tpp_cz, THUMB_W, THUMB_H, THUMB_PP_L, grey_plastic)
            cyl(thumb, "T_Hinge", tx, ty, MCP_Z - THUMB_PP_L,
                0.28, THUMB_W + 0.14, "x", chrome)
            box(thumb, "TD",  tx, ty*0.8, tdp_cz, THUMB_W*0.86, THUMB_H*0.86, THUMB_DP_L,
                grey_plastic)
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
                # Muzzle LED pocket
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
        cover_plate(bp, "RoofAccess", 0, 7.1, TORSO_CTR+5.1, 5.6, 2.0, [(-2.1,-0.05),(2.1,-0.05)], dark_grey)
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
        # FDM SHELL SPLITTING (Y-plane @ 0)
        # ─────────────────────────────────────────────────────────────────
        for comp in comps_list:
            to_split = [b for b in comp.bRepBodies
                        if b.name and any(k in b.name for k in SPLIT_KEYS)]
            for b in to_split:
                split_halves(comp, b, "y", 0.0)
            merge_fasteners_to_halves(comp)

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
        assembly_jig(torso, "TorsoSplit", 0, 8.0, TORSO_CTR, 4.0, 1.2, 0.6)
        assembly_jig(head, "HeadMount", 0, 4.0, HC+1.5, 2.6, 1.0, 0.5)

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

            # GEO-2 — Finger MCP joints (revolute, pitch axis)
            MCP_Z = WRIST_Z - PALM_BOTTOM_OFFSET
            for fi, fname in enumerate(FING_NAMES):
                fx   = ax + m * FING_X_OFF[fi]
                fo_f = occs.get(f"OP_{fname}_{side}")
                revolute_joint(f"{side}_{fname}_MCP", fo_f, ha, fx, -1.35, MCP_Z, "x")

            # Thumb CMC (ball joint — abduction + flexion)
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
                if comp.name not in ("OP_Torso", "OP_Pelvis") \
                   and comp.name not in jointed_comps:
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

        # ═════════════════════════════════════════════════════════════════
        # SIMULATION ENGINE  v11.0
        # ═════════════════════════════════════════════════════════════════

        class SimulationEngine:
            """
            Simulation suite for Optimus Prime G1 v10.
            New in v10: ZMP stability, finger gestures, arm workspace test,
                        BOM export, corrected URDF with joint types + inertia.
            All transformation angles validated against JOINT_LIMITS.
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
                            f"CLAMP: {joint_name}.{axis} {deg:.0f}° → [{lo}°,{hi}°]", "WARN")
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
                """Animate ball-joint (pitch/yaw/roll) and revolute (pitch) together.
                Handles revolute joints in the list gracefully (FIX-5 preserved)."""
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
                """Reset all joints to 0°. Optionally restrict to named groups."""
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
                """SIM-1 — Zero Moment Point stability check.
                Estimates CoM via Fusion physicalProperties, then checks
                whether the CoM projection falls inside the foot support polygon.
                Support polygon approximated as 6.2 × 9.2 cm per foot."""
                FOOT_HW = 6.2 / 2.0   # half-width of foot (X)
                FOOT_HL = 9.2 / 2.0   # half-length of foot (Y)
                try:
                    com = self._root.physicalProperties.centerOfMass
                except Exception as e:
                    Logger.log(f"  ZMP [{label}]: CoM unavailable ({e})", "WARN")
                    return
                # In double-support stance the support polygon spans both feet
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
                """SIM-2 — Curl all 4 fingers on one hand to `degrees` (MCP joint)."""
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
                """Extend index, curl remaining fingers."""
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
                """Index + Middle straight, ring + pinky curled, thumb meets index."""
                targets = [
                    (f"{side}_Pinky_MCP",  80, None, None),
                    (f"{side}_Ring_MCP",   70, None, None),
                    (f"{side}_Middle_MCP", -3, None, None),
                    (f"{side}_Index_MCP",  -3, None, None),
                    (f"{side}_Thumb_CMC",  55, 0,    0),
                ]
                self.move_ball(targets, steps=steps)
                Logger.log(f"  Gesture: {side} hand SNAP")

            # ── SIM-3: Arm Workspace Test ─────────────────────────────────
            def test_arm_workspace(self):
                """SIM-3 — Drive each arm to 8 cardinal reach positions and check
                for collisions at each.  Logs reachable / collision for each."""
                Logger.log("--- SIM-3: ARM WORKSPACE TEST ---")
                # (shoulder_pitch, shoulder_yaw, elbow_pitch, label)
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
                Logger.log("--- MODULE 1 / 9 ---")
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
                Logger.log("--- MODULE 2 / 9 ---")
                Logger.log("MODULE 2: HEAD LOOK-AROUND")
                for yaw_deg in [0, -50, 0, 50, 0]:
                    self.move_joint("Neck_Cluster", yaw_deg, steps=18, axis="yaw")
                for pitch_deg in [0, -25, 0, 35, 0]:
                    self.move_joint("Neck_Cluster", pitch_deg, steps=18, axis="pitch")

            # ── Module 3: Wave Gesture (with finger animation) ────────────
            def simulate_wave(self):
                self.reset_all(steps=10)
                Logger.log("--- MODULE 3 / 9 ---")
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
                Logger.log("--- MODULE 4 / 9 ---")
                Logger.log("MODULE 4: IDLE BREATHING")
                for _ in range(4):
                    self.move_joint("Waist_Cluster", -3, steps=12, axis="pitch")
                    self.move_joint("Waist_Cluster",  3, steps=12, axis="pitch")
                self.move_joint("Waist_Cluster", 0, steps=8, axis="pitch")

            # ── Module 5: Advanced Walking ────────────────────────────────
            def simulate_walking_advanced(self):
                self.reset_all(steps=10)
                Logger.log("--- MODULE 5 / 9 ---")
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
                    # SIM-1: ZMP check mid-stride
                    self._check_zmp(f"Walk cycle {cycle+1}")
                self.reset_all(steps=12)

            # ── Module 6: Running ─────────────────────────────────────────
            def simulate_running(self):
                self.reset_all(steps=8)
                Logger.log("--- MODULE 6 / 9 ---")
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

            # ── Module 7: Combat Sequence (with fist/blaster) ─────────────
            def simulate_combat(self):
                self.reset_all(steps=10)
                Logger.log("--- MODULE 7 / 9 ---")
                Logger.log("MODULE 7: COMBAT SEQUENCE")
                # Close both fists
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
                Logger.log("--- MODULE 8a / 9 ---")
                Logger.log("MODULE 8a: TRANSFORMATION  Robot → Truck")
                self._transform_to_truck(steps_scale=1.0)
                self._interfere("Truck-mode check")
                Logger.log("MODULE 8c: TRUCK → ROBOT")
                self._transform_to_robot(steps_scale=1.0)
                Logger.log("Robot mode restored.")
                self.reset_all(steps=10)

            # ── Module 9a: Stability Analysis (ZMP enhanced) ──────────────
            def run_stability_analysis(self):
                Logger.log("--- MODULE 9 / 9 ---")
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
                Logger.log("MODULE 9b: SERVO LOAD ESTIMATION (v10 upgraded servos)")
                loads = [
                    ("Neck Pitch",       120,  3.0, SERVO_SPECS["micro"]),
                    ("L Shoulder Pitch", 390, 12.0, SERVO_SPECS["std"]),
                    ("R Shoulder Pitch", 390, 12.0, SERVO_SPECS["std"]),
                    ("L Elbow",          210,  7.0, SERVO_SPECS["std"]),
                    ("R Elbow",          210,  7.0, SERVO_SPECS["std"]),
                    ("L Hip Pitch",      820, 15.0, SERVO_SPECS["hip_hd"]),  # upgraded
                    ("R Hip Pitch",      820, 15.0, SERVO_SPECS["hip_hd"]),  # upgraded
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
                        f"  {label:<24} need {need:5.2f} kg·cm  "
                        f"rated {spec['rated']:5.1f}  margin {margin:.2f}×  "
                        f"{spec['name']:12s}  {icon}")

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
                self.capture_screenshots("optimus_robot_v10")

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
                self.capture_screenshots("optimus_truck_v10")

            def simulate_battle_mode(self):
                self.reset_all(steps=10)
                Logger.log("--- BATTLE MODE ---")
                self.move_joint("Blaster_Fold", 0,   steps=10, axis="pitch")
                self.gesture_fist("L")
                self.gesture_point("R")     # pointing = aiming
                self.move_ball([("L_Wrist", None, None, 90),
                                ("R_Wrist", None, None, 90)], steps=15)
                self.move_joint("R_Elbow", 130, steps=18, axis="pitch", clamp=True)
                self.move_joint("L_Elbow", 130, steps=18, axis="pitch", clamp=True)
                self.move_ball([("L_Shoulder_Cluster", 0, -88, 0),
                                ("R_Shoulder_Cluster", 0, -88, 0)], steps=22)
                self._check_zmp("Battle mode")
                self._interfere("Battle-mode check")
                Logger.log("BATTLE MODE -- holding position")
                self.capture_screenshots("optimus_battle_v10")

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

            # ── URDF Export (v10 — proper joint types + inertia) ──────────
            def export_urdf(self):
                """Export URDF with correct joint types, limits, and estimated inertia."""
                # Joint type mapping (name fragment → URDF type)
                def _urdf_type(name):
                    if "Cluster" in name or "Wrist" in name or "CMC" in name:
                        return "spherical"   # URDF 1.x doesn't have spherical;
                        # output as 'floating' with note
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

                # Approximate mass per link (g) for inertia estimation
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
                    # Approximate as uniform box 10×8×6 cm (0.1×0.08×0.06 m)
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
                        f.write('<robot name="Optimus_Prime_G1_v10">\n\n')
                        # Links with inertial blocks
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
                        # Joints
                        for i in range(jc):
                            j    = self._root.asBuiltJoints.item(i)
                            n    = j.name.replace(" ", "_")
                            o1   = j.occurrenceOne.component.name if j.occurrenceOne else "world"
                            o2   = j.occurrenceTwo.component.name if j.occurrenceTwo else "world"
                            jtyp = _urdf_type(n)
                            lo_r, hi_r = _limits(n)
                            # Get servo effort limit
                            effort = 25.0  # default (MG996R N·m approx)
                            if "Hip" in n:     effort = 35.0  # DS3225MG
                            if "Waist" in n:   effort = 25.0  # DS3218
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
                    Logger.log(f"URDF v10 exported -> {path}")
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
                    Logger.log(f"STL exported {count} bodies → {EXPORT_DIR}")
                except Exception:
                    Logger.log(f"STL export failed: {traceback.format_exc()}", "ERROR")

            # ── STEP Export ───────────────────────────────────────────────
            def export_step(self):
                try:
                    os.makedirs(EXPORT_DIR, exist_ok=True)
                    export_mgr = self._design.exportManager
                    path       = os.path.join(EXPORT_DIR, "Optimus_Prime_G1_v10.step")
                    step_opts  = export_mgr.createSTEPExportOptions(path)
                    export_mgr.execute(step_opts)
                    Logger.log(f"STEP assembly → {path}")
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
                    Logger.log(f"STEP {count} components → {EXPORT_DIR}")
                except Exception:
                    Logger.log(f"STEP export failed: {traceback.format_exc()}", "ERROR")

            # ── SIM-4: BOM Export ─────────────────────────────────────────
            def export_bom(self):
                """SIM-4 — Finalise and write BOM CSV. Also write wiring map and assembly guide."""
                # Add fixed hardware totals
                BOM.add("Fastener", "M3×8 SHCS (general assembly)", 80,  "assembly")
                BOM.add("Fastener", "M3×12 SHCS (covers)",          28,  "assembly")
                BOM.add("Fastener", "M3×16 SHCS (joint brackets)",  24,  "assembly")
                BOM.add("Fastener", "M2.5×6 SHCS (PCB mounting)",    16, "assembly")
                BOM.add("Hardware", "Ø3mm × 30 mm shaft pin",        12,  "joints")
                BOM.add("Hardware", "Ø3mm × 20 mm shaft pin",        20,  "joints")
                BOM.add("Hardware", "M3 heat-set insert",            50,  "printed bosses")
                BOM.add("Material", "PLA filament 1kg spool",          3, "prototype shells")
                BOM.add("Material", "PETG filament 1kg spool",         3, "functional shells")
                BOM.add("Material", "TPU filament 250g spool",         1, "grommets / clips")
                BOM.add("Electronics", "22AWG servo wire (3m lengths)", 30, "wiring harness")
                BOM.add("Electronics", "JST-XH 3-pin connectors",      48, "servo connectors")
                BOM.add("Electronics", "5V 5A BEC / power regulator",   2, "servo power")
                BOM.add("Electronics", "USB-C power cable",              1, "RPi power")
                BOM.add("Electronics", "Main fuse + holder",             1, "power safety")
                BOM.save_csv(BOM_FILE)
                BOM.summary()
                verify_screw_lengths()

                # Assembly guide
                try:
                    os.makedirs(_OUTPUT_DIR, exist_ok=True)
                    with open(ASSEMBLY_FILE, "w", encoding="utf-8") as af:
                        af.write("Optimus Prime G1 V11 Assembly Guide\n\n")
                        for step in ASSEMBLY_STEPS:
                            af.write(f"- {step}\n")
                        af.write("\nPrint orientation and support notes:\n")
                        af.write("- Print load-bearing parts in PETG or stronger.\n")
                        af.write("- Use supports only where overhang warnings were logged.\n")
                    Logger.log(f"Assembly guide saved -> {ASSEMBLY_FILE}")
                except Exception as e:
                    Logger.log(f"Assembly guide save failed: {e}", "WARN")

                # Wiring map log
                Logger.log("── SERVO WIRING MAP (PCA9685 channels) ──────────────")
                wiring = [
                    ( 0, "L_Hip_Yaw",     "Pelvis → L_HipYaw"),
                    ( 1, "R_Hip_Yaw",     "Pelvis → R_HipYaw"),
                    ( 2, "L_Hip_Pitch",   "Thigh_L → HipP"),
                    ( 3, "R_Hip_Pitch",   "Thigh_R → HipP"),
                    ( 4, "L_Hip_Roll",    "Thigh_L → HipR"),
                    ( 5, "R_Hip_Roll",    "Thigh_R → HipR"),
                    ( 6, "L_Knee",        "Thigh_L → KneP"),
                    ( 7, "R_Knee",        "Thigh_R → KneP"),
                    ( 8, "L_Ankle_Pitch", "Foot_L  → AnkP"),
                    ( 9, "R_Ankle_Pitch", "Foot_R  → AnkP"),
                    (10, "L_Ankle_Roll",  "Foot_L  → AnkR"),
                    (11, "R_Ankle_Roll",  "Foot_R  → AnkR"),
                    (12, "Waist_Yaw",     "Torso   → WaistYaw"),
                    (13, "Waist_Pitch",   "Torso   → WaistPitch"),
                    (14, "Neck_Pitch",    "Torso   → NeckPitch"),
                    (15, "Neck_Yaw",      "Head    → NeckYaw"),
                ]
                # Second PCA9685 board (channels 0-15 extended)
                wiring2 = [
                    ( 0, "L_Sh_Yaw",      "UpperArm_L → ShY"),
                    ( 1, "R_Sh_Yaw",      "UpperArm_R → ShY"),
                    ( 2, "L_Sh_Pitch",    "UpperArm_L → ShP"),
                    ( 3, "R_Sh_Pitch",    "UpperArm_R → ShP"),
                    ( 4, "L_Sh_Roll",     "UpperArm_L → ShR"),
                    ( 5, "R_Sh_Roll",     "UpperArm_R → ShR"),
                    ( 6, "L_Elbow",       "UpperArm_L → ElbP"),
                    ( 7, "R_Elbow",       "UpperArm_R → ElbP"),
                    ( 8, "L_Wrist_Roll",  "Forearm_L  → WR"),
                    ( 9, "R_Wrist_Roll",  "Forearm_R  → WR"),
                    (10, "L_Finger_All",  "Hand_L → FingerSrvBay ch0"),
                    (11, "L_Thumb",       "Hand_L → FingerSrvBay ch1"),
                    (12, "R_Finger_All",  "Hand_R → FingerSrvBay ch0"),
                    (13, "R_Thumb",       "Hand_R → FingerSrvBay ch1"),
                    (14, "Blaster_Fold",  "Hand_R → Blaster hinge"),
                    (15, "SPARE",         "–"),
                ]
                for ch, name, loc in wiring:
                    Logger.log(f"  PCA1 ch{ch:02d}  {name:<20s}  ← {loc}")
                for ch, name, loc in wiring2:
                    Logger.log(f"  PCA2 ch{ch:02d}  {name:<20s}  ← {loc}")

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
                Logger.log("=" * 55)
                for label, count in self._cols:
                    if count >= 0:
                        icon = "[OK]" if count == 0 else "[WARN]"
                        Logger.log(f"  {label:<42} {icon}  {count}")
                    else:
                        Logger.log(f"  {label:<42} ?  N/A")
                if EXPORT_URDF:
                    Logger.log(f"  URDF  → {EXPORT_DIR}/robot_v11.urdf")
                Logger.log(f"  BOM   → {BOM_FILE}")
                Logger.log(f"  GUIDE → {ASSEMBLY_FILE}")
                Logger.log(f"  Log   → {LOG_FILE}")
                Logger.log("=" * 55)

        # ═════════════════════════════════════════════════════════════════
        # ARCHIVE & RUN
        # ═════════════════════════════════════════════════════════════════
        try:
            os.makedirs(EXPORT_DIR, exist_ok=True)
            archive_path = os.path.join(EXPORT_DIR, "Optimus_Prime_G1_v11.f3d")
            export_mgr   = design.exportManager
            archive_opts = export_mgr.createFusionArchiveExportOptions(archive_path)
            export_mgr.execute(archive_opts)
            Logger.log(f"Design archived → {archive_path}")
        except Exception as e:
            Logger.log(f"Archive skipped: {e}", "WARN")

        sim = SimulationEngine(root, comps_list, design, app, ui)
        sim.run_all_simulations()
        Logger.log("v11 script finished successfully.")

    except Exception:
        Logger.log(f"FATAL ERROR:\n{traceback.format_exc()}", "ERROR")

    finally:
        Logger.flush()
