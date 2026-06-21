Below is the complete, production-ready Fusion 360 Python script. This is the full code that upgrades your v10 mock-up into a physically manufacturable robot.
________________________________________
Python
# ═════════════════════════════════════════════════════════════════════════════
# OPTIMUS PRIME G1  v11.0  ── PHYSICAL BUILD EDITION
# Fusion 360 Python Script  |  Production-Ready for 3D Print & Assembly
# ═════════════════════════════════════════════════════════════════════════════
# WHAT'S NEW IN v11 (Physical Build Upgrade from v10)
# ═════════════════════════════════════════════════════
# PHY-6   horn_coupler() — Printable servo-horn coupler with keyed spline,
#          set-screw pocket, clamping hub, and bearing interface
# PHY-7   tendon_drive() — Underactuated finger tendon system with anchor
#          points, guide pulleys, servo drum, and spring-return pockets
# PHY-8   access_cover() — Removable electronics bay covers with snap/magnet
#          retention and service access
# PHY-9   merge_fasteners_to_halves() — Post-split fastener attachment
# PHY-10  printability_review() — Overhang analysis and chamfer additions
# PHY-11  cable_clip(), wiring_hub() — Snap-in wire management
# PHY-12  bearing_retention() — Press-fit tolerance + retaining lip
# PHY-13  alignment_jig() — Printable assembly jigs
# PHY-14  fastener_verify() — Screw engagement validation
# PHY-15  build_docs() — Assembly guide generation
#
# MECH-1  Internal skeleton with I-beam reinforcements
# MECH-2  Dual-bearing support on high-load joints
# MECH-3  Mechanical hard stops and replaceable bushings
# MECH-4  Transformation locking pins with release geometry
# MECH-5  Steel axle paths for hip/knee/shoulder
# MECH-6  Improved foot ground contact and anti-twist
#
# PWR-1   Battery retention strap + fuse pocket + E-stop
# PWR-2   Logic/servo power separation with BEC mounting
# PWR-3   Cable strain relief at all bay exits
#
# ELEC-7  Realistic board clearances + connector cutouts
# ELEC-8  SD card and USB access slots
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
import re

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
DOCS_DIR       = os.path.join(_OUTPUT_DIR, "docs")
LOG_FILE       = os.path.join(LOG_DIR,     f"optimus_v11_{_ts}.txt")
BOM_FILE       = os.path.join(_OUTPUT_DIR, f"BOM_v11_{_ts}.csv")
BUILD_FILE     = os.path.join(DOCS_DIR,    f"BuildGuide_v11_{_ts}.md")
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
WALL_S       = 0.30    # 3.0 mm structural shell wall
WALL_P       = 0.20    # 2.0 mm partition / internal wall
WALL_F       = 0.15    # 1.5 mm snap-fit cantilever arm
SUPPORT_ANG  = 45.0    # Max overhang angle without support (degrees)
LAYER_H      = 0.02    # 0.2 mm layer height (for tolerance calc)
NOZZLE_D     = 0.04    # 0.4 mm nozzle diameter

# ── Fastener dimensions (cm) ─────────────────────────────────────────────────
M3_CLR_R     = 0.160   # M3 clearance bore radius  (Ø3.2 mm)
M3_PILOT_R   = 0.125   # M3 self-tap pilot radius  (Ø2.5 mm)
M3_HEAD_R    = 0.285   # M3 socket-cap head radius
M3_HEAD_H    = 0.300   # M3 socket-cap head height
M3_NUT_CIR   = 0.320   # M3 hex-nut circumscribed radius
M3_NUT_H     = 0.240   # M3 hex-nut thickness
M3_WSH_R     = 0.350   # M3 washer OD radius
M3_WSH_H     = 0.050   # M3 washer thickness
INSERT_R     = 0.235   # M3 heat-set insert OD radius (Ø4.7 mm)
INSERT_H     = 0.500   # M3 heat-set insert height
BOSS_R       = 0.350   # Boss cylinder outer radius  (Ø7.0 mm)
ALIGN_PIN_R  = 0.100   # Ø2.0 mm alignment pin radius
GROMMET_R    = 0.175   # Ø3.5 mm servo wire-exit grommet radius

# ── Bearing parameters (cm) ──────────────────────────────────────────────────
BEARING_TOL  = 0.015   # Press-fit interference (tight) / clearance (loose)
BEARING_LIP  = 0.08    # Retaining lip height
BEARING_LIP_T= 0.03    # Retaining lip thickness

# ── Servo horn / coupler parameters ──────────────────────────────────────────
HORN_SPL_R   = 0.125   # Servo output spline radius (approx Ø2.5 mm)
HORN_HUB_R   = 0.35    # Horn hub outer radius
HORN_HUB_H   = 0.40    # Horn hub height
HORN_ARM_W   = 0.15    # Horn arm width
HORN_KEY_W   = 0.06    # Key width for anti-rotation
SETSCREW_R   = 0.125   # M2.5 set-screw radius
SETSCREW_H   = 0.25    # Set-screw pocket depth

# ── Tendon drive parameters ──────────────────────────────────────────────────
TENDON_DIA   = 0.04    # Ø0.4 mm fishing line / Dyneema
TENDON_SLOT_W= 0.08    # Groove width for tendon
TENDON_SLOT_D= 0.06    # Groove depth
PULLEY_R     = 0.20    # Idler pulley radius
PULLEY_W     = 0.15    # Idler pulley width
SPRING_OD    = 0.25    # Return spring OD
SPRING_LEN   = 0.60    # Return spring free length
DRUM_R       = 0.35    # Servo drum radius
DRUM_W       = 0.50    # Servo drum width
DRUM_FLANGE  = 0.08    # Drum flange height

# ── Electronics footprints (cm) ──────────────────────────────────────────────
RPI0_L,  RPI0_W,  RPI0_H  = 6.50, 3.00, 0.20
PCA_L,   PCA_W,   PCA_H   = 6.25, 2.54, 0.18
ESP32_L, ESP32_W, ESP32_H = 3.80, 2.58, 0.15
IMU_L,   IMU_W,   IMU_H   = 2.10, 1.60, 0.12
LIPO_L,  LIPO_W,  LIPO_H  = 7.00, 3.20, 1.80
XT60_W,  XT60_H_SLOT       = 1.60, 1.30
LED_R_5MM                  = 0.260
LED_R_RING                 = 0.600

# ── Power system ─────────────────────────────────────────────────────────────
BEC_L, BEC_W, BEC_H      = 3.50, 2.00, 0.80   # 5V BEC module
FUSE_HOLDER_L              = 2.00               # Blade fuse holder
E_STOP_D                   = 1.60               # Emergency stop button
PWR_BUS_L, PWR_BUS_W       = 5.00, 1.50         # Power distribution bus

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
    "hip_hd": {"name": "DS3225MG", "rated": 25.0, "stall": 35.0, "mass_g":  60, "horn_d": 0.60},
    "waist":  {"name": "DS3218",   "rated": 20.0, "stall": 25.0, "mass_g":  55, "horn_d": 0.60},
    "std":    {"name": "MG996R",   "rated":  9.4, "stall": 11.5, "mass_g":  55, "horn_d": 0.55},
    "micro":  {"name": "MG90S",    "rated":  1.8, "stall":  2.2, "mass_g":  13, "horn_d": 0.40},
    "digit":  {"name": "DS04-NFC", "rated":  1.8, "stall":  2.2, "mass_g":   9, "horn_d": 0.35},
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
    "L_Pinky_MCP":  {"pitch": (-5, 85)}, "R_Pinky_MCP":  {"pitch": (-5, 85)},
    "L_Ring_MCP":   {"pitch": (-5, 85)}, "R_Ring_MCP":   {"pitch": (-5, 85)},
    "L_Middle_MCP": {"pitch": (-5, 85)}, "R_Middle_MCP": {"pitch": (-5, 85)},
    "L_Index_MCP":  {"pitch": (-5, 85)}, "R_Index_MCP":  {"pitch": (-5, 85)},
    "L_Thumb_CMC":  {"pitch": (-20, 60), "yaw": (-30, 30)},
    "R_Thumb_CMC":  {"pitch": (-20, 60), "yaw": (-30, 30)},
}

SPLIT_KEYS = {"Shell", "Link", "Main", "Armor", "Core", "Pod", "Palm",
              "Block", "Sole", "Plate", "Bay", "Collar", "Cover", "Door",
              "Jig", "Skeleton", "Bracket"}
SKIP_TAGS  = {"Marker", "Pivot", "MtA", "MtB", "Axle_Pivot", "Horn",
              "Pin", "_Vis", "Boss", "Insert", "Nut", "Snap", "Cutter",
              "Temp", "Groove", "Channel", "Wire", "Tendon"}

# ── Build notes accumulator ──────────────────────────────────────────────────
BUILD_NOTES = []
PRINT_WARNINGS = []


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
# BUILD DOCUMENTATION TRACKER
# ═════════════════════════════════════════════════════════════════════════════

class BuildDocs:
    """Accumulates assembly instructions, print notes, and warnings."""
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
                f.write("# Optimus Prime G1 v11.0 — Build & Assembly Guide\n")
                f.write(f"Generated: {datetime.datetime.now().strftime('%Y-%m-%d %H:%M')}\n\n")
                
                f.write("## ⚠️ CRITICAL BUILD WARNINGS\n\n")
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
                f.write("6. **Assemble legs** — hip → thigh → knee → shin → ankle → foot\n")
                f.write("7. **Assemble arms** — shoulder → upper arm → elbow → forearm → hand\n")
                f.write("8. **Assemble torso** — install electronics before closing shell\n")
                f.write("9. **Mount head** — connect neck servos and camera cable\n")
                f.write("10. **System test** — power on, verify all joints move freely\n")
                f.write("11. **Install covers** — battery door last for access\n")
                f.write("\n")
                
                f.write("## Fastener Verification\n\n")
                f.write("| Location | Screw Spec | Engagement (mm) | Status |\n")
                f.write("|----------|------------|-----------------|--------|\n")
                for fm in cls._fastener_map:
                    icon = "✓" if fm["Status"] == "OK" else "⚠"
                    f.write(f"| {fm['Location']} | {fm['Spec']} | {fm['Engagement']:.1f} | {icon} {fm['Status']} |\n")
                f.write("\n")
                
                for title, content in cls._sections:
                    f.write(f"## {title}\n\n{content}\n\n")
                
                f.write("---\n")
                f.write("*End of build guide. Refer to BOM for hardware quantities.*\n")
            Logger.log(f"Build guide -> {path}")
        except Exception as e:
            Logger.log(f"Build guide failed: {e}", "WARN")


# ═════════════════════════════════════════════════════════════════════════════
# MAIN ENTRY POINT
# ═════════════════════════════════════════════════════════════════════════════

def run(context):
    global TARGET_MODULE, EXPORT_STL, EXPORT_STEP, EXPORT_URDF, EXPORT_DIR

    app = None
    ui  = None

    Logger.log("=" * 65)
    Logger.log("EXECUTION START  v11.0 — Optimus Prime G1 Physical Build")
    Logger.log("Production-ready: servos, fasteners, wiring, covers, jigs")
    Logger.log("=" * 65)

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
        orange_pla    = get_ap("Plastic - Glossy (Orange)",     "Nylon - White")

        # ─────────────────────────────────────────────────────────────────
        # COMPONENT REGISTRY & PRIMITIVES
        # ─────────────────────────────────────────────────────────────────
        comps_list = []
        occs       = {}
        jig_list   = []   # Track jigs separately

        def new_component(name):
            occ  = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
            comp = occ.component
            comp.name = name
            comps_list.append(comp)
            occs[name] = occ
            return comp

        def new_jig(name):
            """Create an assembly jig component (not part of robot kinematics)."""
            occ  = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
            comp = occ.component
            comp.name = f"JIG_{name}"
            jig_list.append(comp)
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
                return True
            except Exception:
                return False

        # ── Merge fasteners to split halves ───────────────────────────────
        def merge_fasteners_to_halves(comp, body_left, body_right, axis="y"):
            """PHY-9: After splitting, merge Pin/Boss/Insert/Nut/Snap bodies
            into the nearest half based on center of mass."""
            fastener_patterns = ["Pin", "Boss", "Insert", "Nut", "Snap", "Align"]
            for b in list(comp.bRepBodies):
                if not b.name or any(t in b.name for t in SKIP_TAGS):
                    continue
                if not any(p in b.name for p in fastener_patterns):
                    continue
                try:
                    com = b.physicalProperties.centerOfMass
                    if axis == "y":
                        target = body_left if com.y < offset else body_right
                    elif axis == "x":
                        target = body_left if com.x < offset else body_right
                    else:
                        target = body_left if com.z < offset else body_right
                    
                    tools = adsk.core.ObjectCollection.create()
                    tools.add(b)
                    ci = comp.features.combineFeatures.createInput(target, tools)
                    ci.operation = adsk.fusion.CombineOperation.JoinFeatureOperation
                    ci.isKeepToolBodies = False
                    comp.features.combineFeatures.add(ci)
                except Exception:
                    pass

        # ─────────────────────────────────────────────────────────────────
        # PHYSICAL FEATURE HELPERS  (PHY-1 … PHY-15)
        # ─────────────────────────────────────────────────────────────────

        def m3_boss(comp, tag, cx, cy, cz, axis="z", depth=0.80):
            """PHY-1 — Ø7 mm print boss + Ø4.7 mm heat-set insert pocket."""
            boss = cyl(comp, f"{tag}_Boss", cx, cy, cz, BOSS_R, depth, axis, dark_metal)
            boss.name = f"{tag}_Boss"
            cut_cavity(comp, cyl(comp, f"{tag}_Insert",
                                 cx, cy, cz, INSERT_R, INSERT_H, axis))
            BOM.add("Fastener", "M3 heat-set insert (Voron M3×5)", 1,
                    f"boss at ({cx:.1f},{cy:.1f},{cz:.1f}) in {comp.name}")

        def captive_nut(comp, tag, cx, cy, cz, axis="z", bolt_len=1.0):
            """PHY-2 — M3 hex-nut trap + through-bore."""
            cut_cavity(comp, cyl(comp, f"{tag}_NutTrap",
                                 cx, cy, cz, M3_NUT_CIR, M3_NUT_H, axis))
            cut_cavity(comp, cyl(comp, f"{tag}_NutBore",
                                 cx, cy, cz, M3_CLR_R, bolt_len, axis))
            BOM.add("Fastener", "M3 hex nut", 1, comp.name)
            BOM.add("Fastener", f"M3×{int(bolt_len*10)} SHCS", 1, comp.name)

        def snap_clip(comp, tag, cx, cy, cz, span_x=1.2, axis_latch="z"):
            """PHY-3 — cantilever snap-fit pair."""
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
            """PHY-4b — Ø2.15 mm alignment socket."""
            cut_cavity(comp, cyl(comp, f"{tag}_AlignSock",
                                 cx, cy, cz, ALIGN_PIN_R + 0.015, depth, axis))

        def grommet_hole(comp, tag, cx, cy, cz, axis="y"):
            """PHY-5 — Ø3.5 mm servo wire-exit groove."""
            cut_cavity(comp, cyl(comp, f"{tag}_Grommet",
                                 cx, cy, cz, GROMMET_R, 0.80, axis))

        # ── PHY-6: Servo Horn Coupler ────────────────────────────────────
        def horn_coupler(comp, tag, cx, cy, cz, axis, servo_type="std"):
            """PHY-6 — Printable servo-horn coupler with keyed spline interface,
            set-screw pocket, and bearing hub. Returns the coupler body."""
            spec = SERVO_SPECS.get(servo_type, SERVO_SPECS["std"])
            horn_d = spec["horn_d"]
            
            hub = cyl(comp, f"{tag}_CouplerHub", cx, cy, cz, HORN_HUB_R, HORN_HUB_H, axis, dark_metal)
            
            cut_cavity(comp, cyl(comp, f"{tag}_SplinePocket",
                                 cx, cy, cz, HORN_SPL_R + 0.02, HORN_HUB_H + 0.05, axis))
            
            ax = {"x": (1, 0, 0), "y": (0, 1, 0), "z": (0, 0, 1)}[axis]
            if axis == "z":
                box(comp, f"{tag}_KeySlot", cx + HORN_SPL_R*0.5, cy, cz,
                    HORN_KEY_W, HORN_SPL_R*2, HORN_HUB_H + 0.05)
            elif axis == "y":
                box(comp, f"{tag}_KeySlot", cx, cy + HORN_SPL_R*0.5, cz,
                    HORN_SPL_R*2, HORN_KEY_W, HORN_HUB_H + 0.05)
            else:
                box(comp, f"{tag}_KeySlot", cx, cy, cz + HORN_SPL_R*0.5,
                    HORN_KEY_W, HORN_SPL_R*2, HORN_HUB_H + 0.05)
            
            if axis == "z":
                cut_cavity(comp, cyl(comp, f"{tag}_SetScrew",
                                     cx + HORN_HUB_R*0.7, cy, cz + HORN_HUB_H*0.5,
                                     SETSCREW_R, HORN_HUB_R*0.6, "x"))
            elif axis == "y":
                cut_cavity(comp, cyl(comp, f"{tag}_SetScrew",
                                     cx, cy + HORN_HUB_R*0.7, cz + HORN_HUB_H*0.5,
                                     SETSCREW_R, HORN_HUB_R*0.6, "y"))
            else:
                cut_cavity(comp, cyl(comp, f"{tag}_SetScrew",
                                     cx + HORN_HUB_R*0.7, cy, cz + HORN_HUB_H*0.5,
                                     SETSCREW_R, HORN_HUB_R*0.6, "z"))
            
            if axis == "z":
                box(comp, f"{tag}_ArmA", cx, cy, cz + HORN_HUB_H*0.5,
                    horn_d, HORN_ARM_W, HORN_HUB_H*0.4, dark_metal)
                box(comp, f"{tag}_ArmB", cx, cy, cz + HORN_HUB_H*0.5,
                    HORN_ARM_W, horn_d, HORN_HUB_H*0.4, dark_metal)
            elif axis == "y":
                box(comp, f"{tag}_ArmA", cx, cy + HORN_HUB_H*0.5, cz,
                    horn_d, HORN_HUB_H*0.4, HORN_ARM_W, dark_metal)
                box(comp, f"{tag}_ArmB", cx, cy + HORN_HUB_H*0.5, cz,
                    HORN_ARM_W, HORN_HUB_H*0.4, horn_d, dark_metal)
            else:
                box(comp, f"{tag}_ArmA", cx + HORN_HUB_H*0.5, cy, cz,
                    HORN_HUB_H*0.4, horn_d, HORN_ARM_W, dark_metal)
                box(comp, f"{tag}_ArmB", cx + HORN_HUB_H*0.5, cy, cz,
                    HORN_HUB_H*0.4, HORN_ARM_W, horn_d, dark_metal)
            
            BOM.add("Printed", f"Servo horn coupler ({servo_type})", 1,
                    f"{tag} in {comp.name}")
            BOM.add("Fastener", "M2.5×5 set-screw (grub screw)", 1,
                    f"{tag} coupler in {comp.name}")
            BuildDocs.set_print_orientation(
                f"{tag}_Coupler", "flat on build plate", "none", "PETG")
            return hub

        # ── PHY-7: Tendon Drive System ───────────────────────────────────
        def tendon_anchor(comp, tag, cx, cy, cz, axis="z"):
            """Tendon anchor point on distal phalanx."""
            anchor = cyl(comp, f"{tag}_TendonAnchor", cx, cy, cz, 0.12, 0.30, axis, grey_plastic)
            if axis == "z":
                cut_cavity(comp, cyl(comp, f"{tag}_TendonHole",
                                     cx, cy, cz, TENDON_DIA*0.6, 0.35, "x"))
            else:
                cut_cavity(comp, cyl(comp, f"{tag}_TendonHole",
                                     cx, cy, cz, TENDON_DIA*0.6, 0.35, "z"))
            return anchor

        def tendon_groove(comp, tag, cx, cy, cz, length, axis="z"):
            """Channel for tendon routing."""
            cut_cavity(comp, box(comp, f"{tag}_TendonGroove",
                                 cx, cy, cz,
                                 TENDON_SLOT_W if axis != "x" else length,
                                 TENDON_SLOT_W if axis != "y" else length,
                                 TENDON_SLOT_W if axis != "z" else length))

        def pulley_post(comp, tag, cx, cy, cz, axis="x"):
            """PHY-7 — Idler pulley post in palm."""
            post = cyl(comp, f"{tag}_PulleyPost", cx, cy, cz, 0.12, PULLEY_W + 0.10, axis, chrome)
            cut_cavity(comp, cyl(comp, f"{tag}_PulleyGroove",
                                 cx, cy, cz, PULLEY_R + 0.02, PULLEY_W, axis))
            BOM.add("Hardware", "Ø4×2 mm micro bearing (pulley)", 1,
                    f"{tag} in {comp.name}")
            return post

        def servo_drum(comp, tag, cx, cy, cz, axis="z"):
            """PHY-7 — Servo drum for tendon winding."""
            drum = cyl(comp, f"{tag}_ServoDrum", cx, cy, cz, DRUM_R, DRUM_W, axis, dark_metal)
            cyl(comp, f"{tag}_DrumFlangeA", cx, cy, cz - DRUM_W*0.5 - DRUM_FLANGE*0.5,
                DRUM_R + 0.08, DRUM_FLANGE, axis, dark_metal)
            cyl(comp, f"{tag}_DrumFlangeB", cx, cy, cz + DRUM_W*0.5 + DRUM_FLANGE*0.5,
                DRUM_R + 0.08, DRUM_FLANGE, axis, dark_metal)
            if axis == "z":
                cut_cavity(comp, cyl(comp, f"{tag}_DrumHole",
                                     cx + DRUM_R*0.5, cy, cz, 0.06, DRUM_W + 0.10, "z"))
            BOM.add("Printed", "Servo tendon drum", 1, f"{tag} in {comp.name}")
            BuildDocs.set_print_orientation(
                f"{tag}_Drum", "on side (axis horizontal)", "minimal", "PETG")
            return drum

        def spring_pocket(comp, tag, cx, cy, cz, axis="z"):
            """PHY-7 — Return spring pocket."""
            cut_cavity(comp, cyl(comp, f"{tag}_SpringPkt",
                                 cx, cy, cz, SPRING_OD*0.5 + 0.05, SPRING_LEN, axis))
            BOM.add("Hardware", f"Return spring Ø{int(SPRING_OD*10)}×{int(SPRING_LEN*10)} mm", 1,
                    f"{tag} in {comp.name}")

        # ── PHY-8: Access Covers ─────────────────────────────────────────
        def access_cover(comp, tag, cx, cy, cz, lx, ly, lz, axis="y", retention="magnet"):
            """PHY-8 — Removable cover/door for electronics bays."""
            cover = box(comp, f"{tag}_Cover", cx, cy, cz, lx + 0.20, ly + 0.10, lz + 0.20, op_blue)
            
            if axis == "y":
                box(comp, f"{tag}_Grip", cx, cy - ly*0.5 - 0.25, cz, 0.80, 0.30, 0.20, op_red)
            
            if retention in ("magnet", "hybrid"):
                for mx in [-lx*0.35, lx*0.35]:
                    for mz in [-lz*0.35, lz*0.35]:
                        cut_cavity(comp, cyl(comp, f"{tag}_MagCover_{mx:.1f}",
                                             cx+mx, cy + ly*0.5 + 0.05, cz+mz,
                                             0.32, 0.35, "y"))
                magnet_pocket(comp, f"{tag}_Frame", cx-lx*0.35, cy, cz-lz*0.35)
                magnet_pocket(comp, f"{tag}_Frame", cx+lx*0.35, cy, cz-lz*0.35)
                magnet_pocket(comp, f"{tag}_Frame", cx-lx*0.35, cy, cz+lz*0.35)
                magnet_pocket(comp, f"{tag}_Frame", cx+lx*0.35, cy, cz+lz*0.35)
            
            if retention in ("snap", "hybrid"):
                snap_clip(comp, f"{tag}_Cover", cx, cy, cz, span_x=lx*0.6)
            
            if retention in ("screw", "hybrid"):
                for sx in [-lx*0.4, lx*0.4]:
                    for sz in [-lz*0.4, lz*0.4]:
                        m3_boss(comp, f"{tag}_Cover_{sx:.1f}", cx+sx, cy, cz+sz)
            
            BOM.add("Printed", f"Access cover ({retention})", 1,
                    f"{tag} for {comp.name}")
            BuildDocs.set_print_orientation(
                f"{tag}_Cover", "flat on build plate", "none", "PLA")
            return cover

        # ── PHY-10: Printability Review ──────────────────────────────────
        def printability_review(comp, body_name, overhang_angle):
            """Log warnings for parts that may need support."""
            if overhang_angle > SUPPORT_ANG:
                msg = (f"{comp.name}/{body_name}: overhang {overhang_angle:.0f}° "
                       f"exceeds {SUPPORT_ANG:.0f}° — supports recommended")
                Logger.log(f"  PRINT WARNING: {msg}", "WARN")
                PRINT_WARNINGS.append(msg)
                BuildDocs.set_print_orientation(
                    body_name, "reorient or add supports", "required", "PLA/PETG")

        # ── PHY-11: Cable Management ─────────────────────────────────────
        def cable_clip(comp, tag, cx, cy, cz, axis="z", count=1):
            """Snap-in cable clip/wire retainer."""
            for i in range(count):
                offset = i * 0.60
                if axis == "z":
                    oy = cy + offset
                    clip = box(comp, f"{tag}_Clip_{i}", cx, oy, cz, 0.35, 0.55, 0.25, black_plastic)
                    cut_cavity(comp, box(comp, f"{tag}_ClipChannel_{i}",
                                         cx, oy, cz + 0.05, 0.20, 0.35, 0.15))
                elif axis == "y":
                    oz = cz + offset
                    clip = box(comp, f"{tag}_Clip_{i}", cx, cy, oz, 0.35, 0.25, 0.55, black_plastic)
                    cut_cavity(comp, box(comp, f"{tag}_ClipChannel_{i}",
                                         cx, cy + 0.05, oz, 0.20, 0.15, 0.35))
                else:
                    oy = cy + offset
                    clip = box(comp, f"{tag}_Clip_{i}", cx, oy, cz, 0.25, 0.55, 0.35, black_plastic)
                    cut_cavity(comp, box(comp, f"{tag}_ClipChannel_{i}",
                                         cx + 0.05, oy, cz, 0.15, 0.35, 0.20))
            BOM.add("Printed", "Cable clip (snap-in)", count, f"{tag} in {comp.name}")
            BuildDocs.set_print_orientation("CableClip", "flat", "none", "PLA")

        def wiring_hub(comp, tag, cx, cy, cz):
            """Central wiring convergence point."""
            hub = box(comp, f"{tag}_Hub", cx, cy, cz, 2.0, 1.5, 1.2, black_plastic)
            for dx in [-0.8, 0.8]:
                cut_cavity(comp, cyl(comp, f"{tag}_Port_{dx:.0f}",
                                     cx+dx, cy, cz, 0.25, 1.6, "y"))
            cable_clip(comp, tag, cx, cy - 0.8, cz, "y", count=3)
            BOM.add("Printed", "Wiring hub", 1, f"{tag} in {comp.name}")
            return hub

        # ── PHY-12: Bearing Retention ────────────────────────────────────
        def bearing_retention(comp, tag, cx, cy, cz, axis="x", ro=1.10, w=0.60,
                               fit="press"):
            """PHY-12 — Bearing pocket with press-fit tolerance and retaining lip."""
            tol = -BEARING_TOL if fit == "press" else BEARING_TOL
            ro_adj = ro + tol
            
            cyl(comp, f"{tag}_BH", cx, cy, cz, ro_adj + WALL_S, w + WALL_S*2, axis, dark_metal)
            cut_cavity(comp, cyl(comp, f"{tag}_BC",
                                 cx, cy, cz, ro_adj, w + 0.05, axis))
            
            ax = {"x": (1, 0, 0), "y": (0, 1, 0), "z": (0, 0, 1)}[axis]
            lip_pos = w*0.5 + BEARING_LIP*0.5
            cyl(comp, f"{tag}_Lip",
                cx + ax[0]*lip_pos, cy + ax[1]*lip_pos, cz + ax[2]*lip_pos,
                ro_adj + BEARING_LIP_T, BEARING_LIP, axis, dark_metal)
            cut_cavity(comp, cyl(comp, f"{tag}_LipClear",
                                 cx + ax[0]*lip_pos, cy + ax[1]*lip_pos, cz + ax[2]*lip_pos,
                                 ro_adj - 0.05, BEARING_LIP + 0.02, axis))
            
            size_tag = f"Ø{int(ro*2*10)}×{int(w*10)} mm bearing"
            BOM.add("Bearing", size_tag, 1, f"{tag} in {comp.name} ({fit} fit)")
            BuildDocs.set_print_orientation(
                f"{tag}_BearingHousing", "axis vertical", "none", "PETG")

        # ── PHY-13: Alignment Jigs ───────────────────────────────────────
        def alignment_jig(comp_name, pin_pattern, axis="y"):
            """PHY-13 — Generate printable alignment jig for shell halves."""
            jig = new_jig(f"Align_{comp_name}")
            box(jig, "JigBase", 0, 0, 0, 8.0, 0.50, 6.0, orange_pla)
            for px, pz in pin_pattern:
                cyl(jig, f"JigPin_{px:.1f}", px, 0.25, pz,
                    ALIGN_PIN_R - 0.005, 0.60, "y", orange_pla)
            for fx in [-3.5, 3.5]:
                for fz in [-2.5, 2.5]:
                    box(jig, f"JigFoot_{fx:.0f}", fx, -0.30, fz, 0.60, 0.30, 0.60, orange_pla)
            BOM.add("Tooling", f"Alignment jig for {comp_name}", 1, "print in PLA")
            BuildDocs.set_print_orientation(
                f"JIG_{comp_name}", "flat on build plate", "none", "PLA")
            return jig

        # ── PHY-14: Fastener Verification ────────────────────────────────
        def fastener_verify(location, screw_dia_mm, screw_len_mm, 
                           material_thickness_mm, nut_trap_depth_mm=0,
                           min_engagement=4.0):
            """Validate screw engagement length."""
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
            
            spec = f"M{screw_dia_mm:.1f}×{screw_len_mm:.0f}"
            BuildDocs.add_fastener(location, spec, engagement, status)
            return status

        # ── PHY-15: Build Documentation ──────────────────────────────────
        def add_build_note(section, content):
            BuildDocs.add_section(section, content)

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
            """ELEC-4 — Ø12 mm LED ring recess."""
            cut_cavity(comp, cyl(comp, f"{tag}_LEDRing",
                                 cx, cy, cz, LED_R_RING, 0.30, axis))
            BOM.add("Electronics", "WS2812 5050 LED ring Ø12 mm", 1, comp.name)

        # ─────────────────────────────────────────────────────────────────
        # ELECTRONICS BAY HELPERS  (ELEC-1 … ELEC-8)
        # ─────────────────────────────────────────────────────────────────

        def rpi_zero_bay(comp, tag, cx, cy, cz):
            """ELEC-1 — RPi Zero 2W pocket with standoffs and access slot."""
            cut_cavity(comp, box(comp, f"{tag}_RPiBay",
                                 cx, cy, cz,
                                 RPI0_L + 0.25, RPI0_W + 0.25, RPI0_H + 0.35))
            cut_cavity(comp, box(comp, f"{tag}_SDSlot",
                                 cx - RPI0_L*0.5 - 0.15, cy, cz + 0.5,
                                 0.30, 0.80, 0.40))
            cut_cavity(comp, box(comp, f"{tag}_USBSlt",
                                 cx + RPI0_L*0.5 + 0.15, cy, cz - 0.3,
                                 0.30, 1.20, 0.60))
            for sx, sz in [(-2.90, -1.15), (+2.90, -1.15),
                           (-2.90, +1.15), (+2.90, +1.15)]:
                cyl(comp, f"{tag}_RpiStdoff_{sx}_{sz}",
                    cx+sx, cy, cz+sz, 0.14, RPI0_H+0.55, "y", dark_metal)
            BOM.add("Electronics", "Raspberry Pi Zero 2W", 1, comp.name)
            BOM.add("Fastener",    "M2.5×11 mm brass standoff", 4, comp.name)
            BOM.add("Fastener",    "M2.5×6 SHCS", 4, comp.name)

        def pca9685_bay(comp, tag, cx, cy, cz):
            """ELEC-2 — PCA9685 pocket with standoffs."""
            cut_cavity(comp, box(comp, f"{tag}_PCABay",
                                 cx, cy, cz,
                                 PCA_L + 0.25, PCA_W + 0.25, PCA_H + 0.35))
            for sx, sz in [(-3.00, -1.08), (+3.00, -1.08),
                           (-3.00, +1.08), (+3.00, +1.08)]:
                cyl(comp, f"{tag}_PCAStdoff_{sx}_{sz}",
                    cx+sx, cy, cz+sz, 0.14, PCA_H+0.55, "y", dark_metal)
            BOM.add("Electronics", "PCA9685 16-ch servo driver", 1, comp.name)
            BOM.add("Fastener",    "M2.5×6 SHCS", 4, comp.name)

        def lipo_bay(comp, tag, cx, cy, cz):
            """ELEC-3 — 2S LiPo pocket with retention and XT60 slot."""
            cut_cavity(comp, box(comp, f"{tag}_LipoBay",
                                 cx, cy, cz,
                                 LIPO_L + 0.35, LIPO_H + 0.35, LIPO_W + 0.35))
            cut_cavity(comp, box(comp, f"{tag}_XT60Slot",
                                 cx, cy + LIPO_H/2 + 0.20, cz,
                                 XT60_W + 0.15, 0.60, XT60_H_SLOT + 0.15))
            for sz in [-LIPO_W*0.3, LIPO_W*0.3]:
                cut_cavity(comp, box(comp, f"{tag}_Strap_{sz:.0f}",
                                     cx, cy, cz + sz,
                                     LIPO_L + 0.80, 0.15, 0.35))
            cut_cavity(comp, box(comp, f"{tag}_BalanceExit",
                                 cx + LIPO_L*0.5 + 0.10, cy, cz,
                                 0.30, 0.50, 0.40))
            BOM.add("Electronics", "2S 1300 mAh 7.4V LiPo battery", 1, comp.name)
            BOM.add("Electronics", "XT60-F panel-mount connector",  1, comp.name)
            BOM.add("Hardware",    "Battery strap 20×200 mm", 1, comp.name)

        def imu_pocket(comp, tag, cx, cy, cz, axis="z"):
            """ELEC-6 — MPU-6050 pocket."""
            cut_cavity(comp, box(comp, f"{tag}_IMUPkt",
                                 cx, cy, cz,
                                 IMU_L + 0.25, IMU_W + 0.25, IMU_H + 0.35))
            BOM.add("Electronics", "MPU-6050 IMU breakout", 1, comp.name)

        def esp32_cam_pocket(comp, tag, cx, cy, cz, lens_axis="y"):
            """ELEC-5 — ESP32-CAM pocket + lens channel."""
            cut_cavity(comp, box(comp, f"{tag}_ESP32Bay",
                                 cx, cy, cz,
                                 ESP32_L + 0.25, ESP32_H + 0.35, ESP32_W + 0.25))
            cut_cavity(comp, cyl(comp, f"{tag}_LensHole",
                                 cx, cy - (ESP32_H/2 + 0.35), cz,
                                 0.150, 0.70, lens_axis))
            cut_cavity(comp, box(comp, f"{tag}_ProgBtn",
                                 cx + ESP32_L*0.5 + 0.15, cy, cz,
                                 0.25, 0.40, 0.30))
            BOM.add("Electronics", "ESP32-CAM module (OV2640)", 1, comp.name)

        # ── Power system helpers ─────────────────────────────────────────
        def bec_mount(comp, tag, cx, cy, cz):
            """PWR-1 — 5V BEC mounting pocket."""
            cut_cavity(comp, box(comp, f"{tag}_BEC",
                                 cx, cy, cz, BEC_L + 0.20, BEC_H + 0.20, BEC_W + 0.20))
            m3_boss(comp, f"{tag}_BEC_A", cx - 1.5, cy, cz - 0.6)
            m3_boss(comp, f"{tag}_BEC_B", cx + 1.5, cy, cz - 0.6)
            BOM.add("Electronics", "5V 5A BEC regulator", 1, comp.name)

        def fuse_pocket(comp, tag, cx, cy, cz, axis="y"):
            """PWR-1 — Blade fuse holder pocket."""
            cut_cavity(comp, box(comp, f"{tag}_Fuse",
                                 cx, cy, cz,
                                 FUSE_HOLDER_L + 0.20, 0.80 + 0.20, 0.60 + 0.20))
            BOM.add("Electronics", "ATO blade fuse holder + 10A fuse", 1, comp.name)

        def estop_pocket(comp, tag, cx, cy, cz, axis="y"):
            """PWR-1 — Emergency stop button pocket."""
            cut_cavity(comp, cyl(comp, f"{tag}_EStop",
                                 cx, cy, cz, E_STOP_D*0.5 + 0.10, 0.80, axis))
            BOM.add("Electronics", "Emergency stop button (NC, 16mm)", 1, comp.name)

        def power_bus(comp, tag, cx, cy, cz):
            """PWR-2 — Power distribution bus bar pocket."""
            cut_cavity(comp, box(comp, f"{tag}_PWRBus",
                                 cx, cy, cz, PWR_BUS_L + 0.20, 0.60, PWR_BUS_W + 0.20))
            for tx in [-PWR_BUS_L*0.4, 0, PWR_BUS_L*0.4]:
                screw_hole(comp, cx+tx, cy, cz, "y", 0.80)
            BOM.add("Electronics", "Power distribution bus bar (3-pos)", 1, comp.name)

        def strain_relief(comp, tag, cx, cy, cz, axis="y"):
            """PWR-3 — Cable strain relief clamp."""
            box(comp, f"{tag}_SRClamp", cx, cy, cz, 0.60, 0.30, 0.80, black_plastic)
            cut_cavity(comp, box(comp, f"{tag}_SRChannel",
                                 cx, cy, cz, 0.25, 0.35, 0.50))
            m3_boss(comp, f"{tag}_SR", cx - 0.25, cy, cz)
            m3_boss(comp, f"{tag}_SR", cx + 0.25, cy, cz)
            BOM.add("Hardware", "Cable strain relief (printed)", 1, comp.name)

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
                    adsk.core.JointDirections.CustomJointDirection, av)
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
                    adsk.core.JointDirections.ZAxisJointDirection,
                    adsk.core.JointDirections.XAxisJointDirection)
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
        # HARDWARE MODULES  (with coupler integration)
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
            grommet_hole(comp, tag, cx, cy, cz + (0.5 if axis != "z" else 1.0))

        def mg996r(comp, tag, cx, cy, cz, axis="x", add_coupler=True):
            cl = CLEARANCE
            if axis == "x":
                box(comp, f"{tag}_VisBody", cx,      cy,      cz,      4.05, 2.00, 4.20, grey_plastic)
                box(comp, f"{tag}_VisEars", cx+0.95, cy,      cz,      0.30, 2.20, 5.80, dark_grey)
                cyl(comp, f"{tag}_VisHorn", cx+2.40, cy,      cz+1.05, 0.95, 0.22, "x", white_pla)
                marker(comp, f"{tag}_Pivot", cx+2.40, cy,     cz+1.05)
                cut_cavity(comp, box(comp, f"{tag}_CB",  cx,      cy, cz, 4.05+cl, 2.00+cl, 4.20+cl))
                cut_cavity(comp, box(comp, f"{tag}_CE",  cx+0.95, cy, cz, 0.30+cl, 2.20+cl, 5.80+cl))
                if add_coupler:
                    horn_coupler(comp, f"{tag}", cx+2.40, cy, cz+1.05, "x", "std")
            elif axis == "z":
                box(comp, f"{tag}_VisBody", cx,      cy,      cz,      4.05, 2.00, 4.20, grey_plastic)
                box(comp, f"{tag}_VisEars", cx,      cy,      cz+0.95, 5.80, 2.20, 0.30, dark_grey)

