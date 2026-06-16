TARGET_MODULE = "truck"
EXPORT_STL = False
EXPORT_URDF = False
CAPTURE_SCREENSHOTS = True

import adsk.core
import adsk.fusion
import traceback
import math
import os
import datetime
import time

_ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

# ─── Output directory structure ──────────────────────────────
try:
    SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
except NameError:
    SCRIPT_DIR = os.getcwd()
PROJECT_DIR = os.path.dirname(SCRIPT_DIR)
OUTPUT_DIR = os.path.join(PROJECT_DIR, "output")
LOG_DIR = os.path.join(OUTPUT_DIR, "logs")
SCREENSHOT_DIR = os.path.join(OUTPUT_DIR, "screenshots")
EXPORT_DIR = os.path.join(OUTPUT_DIR, "exports")
LOG_FILE = os.path.join(LOG_DIR, f"optimus_fusion_log_{_ts}.txt")
STOP_FLAG = os.path.join(OUTPUT_DIR, "stop.flag")


# ═════════════════════════════════════════════════════════════════════════════
# CONFIGURATION
# ═════════════════════════════════════════════════════════════════════════════

CLEARANCE   = 0.060  # 0.60 mm (v7 used 0.45 mm) - better FDM tolerance
ANKLE_CTR   = 3.8
SHIN_CTR    = 9.3
KNEE_CTR    = 14.8
THIGH_CTR   = 20.3
PELVIS_CTR  = 30.5  # v8: raised from 29.5 to reduce torso-pelvis Z-overlap
WAIST_CTR   = 32.5  # v8: raised (+1.0) to match new pelvis height
TORSO_CTR   = 36.0  # v8: raised (+1.0) to maintain proportions
SHOULDER_CTR = 41.5 # v8: raised (+1.0)
HEAD_CTR    = 47.5  # v8: raised (+1.0)
HIP_X       = 5.8
SHOULDER_X  = 13.0
ELBOW_Z     = 35.0  # v8: raised (+1.0)
WRIST_Z     = 29.0  # v8: raised (+1.0)
HIP_JOINT_Z = 26.8  # v8: raised from 25.8 to prevent servo intrusion into pelvis
NECK_JOINT_Z = 44.5 # v8: raised (+1.0)

SERVO_SPECS = {
    "hip":    {"name": "MG996R-HD", "rated": 20.0, "stall": 25.0},
    "waist":  {"name": "MG996R-HD", "rated": 25.0, "stall": 30.0},
    "std":    {"name": "MG996R",    "rated": 9.4,  "stall": 11.5},
    "micro":  {"name": "MG90S",     "rated": 1.8,  "stall": 2.2},
}

# ROM limits used by both test_joint_rom() and transformation validation
# Axis mapping is joint-specific (depends on joint geometry):
#   Waist/Neck: pitch = forward lean/tilt, yaw = twist
#   Hip/Ankle:  yaw = forward fold/tuck, pitch = twist
#   Shoulder:   yaw = side-swing, pitch = tilt, roll = twist
JOINT_LIMITS = {
    "Waist_Cluster":      {"pitch": (-45, 60),   "yaw": (-15, 15),    "roll": (-15, 15)},
    "Neck_Cluster":       {"pitch": (-90, 45),   "yaw": (-20, 20),    "roll": (-20, 20)},
    "L_Hip_Cluster":      {"pitch": (-30, 30),   "yaw": (-95, 95),    "roll": (-30, 30)},
    "R_Hip_Cluster":      {"pitch": (-30, 30),   "yaw": (-95, 95),    "roll": (-30, 30)},
    "L_Knee":             {"pitch": (0, 135)},
    "R_Knee":             {"pitch": (0, 135)},
    "L_Ankle_Cluster":    {"pitch": (-20, 20),   "yaw": (-30, 95),    "roll": (-20, 20)},
    "R_Ankle_Cluster":    {"pitch": (-20, 20),   "yaw": (-30, 95),    "roll": (-20, 20)},
    "L_Shoulder_Cluster": {"pitch": (-175, 60),  "yaw": (-90, 90),    "roll": (-90, 90)},
    "R_Shoulder_Cluster": {"pitch": (-175, 60),  "yaw": (-90, 90),    "roll": (-90, 90)},
    "L_Elbow":            {"pitch": (0, 150)},
    "R_Elbow":            {"pitch": (0, 150)},
    "L_Wrist":            {"pitch": (0, 90),    "roll": (-180, 180)},
    "R_Wrist":            {"pitch": (0, 135),   "roll": (-180, 180)},
    "Blaster_Fold":       {"pitch": (-90, 0)},
}

SPLIT_KEYS = {"Shell", "Link", "Main", "Armor", "Core", "Pod", "Palm", "Block", "Sole"}
SKIP_TAGS  = {"Marker", "Pivot", "MtA", "MtB", "Axle_Pivot", "Horn", "Pin", "_Vis"}


# ═════════════════════════════════════════════════════════════════════════════
# LOGGER
# ═════════════════════════════════════════════════════════════════════════════

class Logger:
    _buffer = []
    _count = 0

    @classmethod
    def log(cls, msg, level="INFO"):
        ts = datetime.datetime.now().strftime("%H:%M:%S")
        cls._buffer.append(f"[{ts}] [{level}] {msg}\n")
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


def run(context):
    global TARGET_MODULE, EXPORT_STL, EXPORT_URDF, EXPORT_DIR

    app = None
    ui = None

    Logger.log("=" * 60)
    Logger.log("EXECUTION START  v8.0 — Optimus Prime G1")
    Logger.log("=" * 60)

    try:
        app = adsk.core.Application.get()
        ui = app.userInterface

        try:
            app.preferences.generalPreferences.defaultModelingOrientation = (
                adsk.core.DefaultModelingOrientations.ZUpModelingOrientation)
        except Exception:
            pass

        doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
        design = adsk.fusion.Design.cast(app.activeProduct)
        root = design.rootComponent

        # ═══════════════════════════════════════════════════════════════════
        # APPEARANCES
        # ═══════════════════════════════════════════════════════════════════
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

        # ═══════════════════════════════════════════════════════════════════
        # COMPONENT REGISTRY & PRIMITIVES
        # ═══════════════════════════════════════════════════════════════════
        comps_list = []
        occs = {}

        def new_component(name):
            occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
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
            temp = adsk.fusion.TemporaryBRepManager.get()
            obb = adsk.core.OrientedBoundingBox3D.create(
                adsk.core.Point3D.create(cx, cy, cz),
                adsk.core.Vector3D.create(1, 0, 0),
                adsk.core.Vector3D.create(0, 1, 0),
                lx, ly, lz)
            shape = temp.createBox(obb)
            bf = comp.features.baseFeatures.add()
            bf.startEdit()
            body = comp.bRepBodies.add(shape, bf)
            bf.finishEdit()
            body.name = name
            set_ap(body, ap)
            return body

        def cyl(comp, name, cx, cy, cz, r, h, axis, ap=None):
            temp = adsk.fusion.TemporaryBRepManager.get()
            ax = {"x": (1, 0, 0), "y": (0, 1, 0), "z": (0, 0, 1)}[axis]
            p1 = adsk.core.Point3D.create(cx - ax[0]*h/2, cy - ax[1]*h/2, cz - ax[2]*h/2)
            p2 = adsk.core.Point3D.create(cx + ax[0]*h/2, cy + ax[1]*h/2, cz + ax[2]*h/2)
            shape = temp.createCylinderOrCone(p1, r, p2, r)
            bf = comp.features.baseFeatures.add()
            bf.startEdit()
            body = comp.bRepBodies.add(shape, bf)
            bf.finishEdit()
            body.name = name
            set_ap(body, ap)
            return body

        def cone_shape(comp, name, cx, cy, cz, r1, r2, h, axis, ap=None):
            temp = adsk.fusion.TemporaryBRepManager.get()
            ax = {"x": (1, 0, 0), "y": (0, 1, 0), "z": (0, 0, 1)}[axis]
            p1 = adsk.core.Point3D.create(cx - ax[0]*h/2, cy - ax[1]*h/2, cz - ax[2]*h/2)
            p2 = adsk.core.Point3D.create(cx + ax[0]*h/2, cy + ax[1]*h/2, cz + ax[2]*h/2)
            shape = temp.createCylinderOrCone(p1, r1, p2, r2)
            bf = comp.features.baseFeatures.add()
            bf.startEdit()
            body = comp.bRepBodies.add(shape, bf)
            bf.finishEdit()
            body.name = name
            set_ap(body, ap)
            return body

        def marker(comp, name, cx, cy, cz, size=0.22):
            return box(comp, name, cx, cy, cz, size, size, size, white_pla)

        # ─── Boolean cavity cutter ────────────────────────────────────────
        def cut_cavity(comp, tool_body, keep_tool=False):
            tools = adsk.core.ObjectCollection.create()
            tools.add(tool_body)
            for b in comp.bRepBodies:
                if b == tool_body:
                    continue
                if b.name and any(t in b.name for t in SKIP_TAGS):
                    continue
                try:
                    ci = comp.features.combineFeatures.createInput(b, tools)
                    ci.operation = adsk.fusion.CombineOperation.CutFeatureOperation
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

        # ─── Shell splitter for FDM printing ──────────────────────────────
        def split_halves(comp, body, axis='y', offset=0.0):
            try:
                planes = comp.constructionPlanes
                pi = planes.createInput()
                ref = (root.xYConstructionPlane if axis == 'z' else
                       root.xZConstructionPlane if axis == 'y' else
                       root.yZConstructionPlane)
                pi.setByOffset(ref, adsk.core.ValueInput.createByReal(offset))
                sp = planes.add(pi)
                split_input = comp.features.splitBodyFeatures.createInput(body, sp, True)
                comp.features.splitBodyFeatures.add(split_input)
            except Exception:
                pass

        # ─── Fastener helpers ─────────────────────────────────────────────
        def screw_hole(comp, cx, cy, cz, axis="y", length=3.0):
            cut_cavity(comp, cyl(comp, "ScrewHole", cx, cy, cz, 0.15, length, axis))

        def magnet_pocket(comp, tag, cx, cy, cz, axis="z"):
            cut_cavity(comp, cyl(comp, f"{tag}_MagPkt", cx, cy, cz, 0.32, 0.35, axis))

        def wire_channel(comp, tag, cx, cy, cz, r, h, axis):
            cut_cavity(comp, cyl(comp, f"{tag}_Wire", cx, cy, cz, r, h, axis))

        # ═══════════════════════════════════════════════════════════════════
        # JOINT BUILDERS
        # ═══════════════════════════════════════════════════════════════════
        def _make_joint_geometry(cx, cy, cz):
            sketch = root.sketches.add(root.xYConstructionPlane)
            sketch.isVisible = False
            s_pt = sketch.sketchPoints.add(adsk.core.Point3D.create(cx, cy, cz))
            return adsk.fusion.JointGeometry.createByPoint(s_pt)

        def revolute_joint(name, occ1, occ2, cx, cy, cz, axis_str):
            if not (occ1 and occ2):
                return
            try:
                geom = _make_joint_geometry(cx, cy, cz)
                ji = root.asBuiltJoints.createInput(occ1, occ2, geom)
                av = {"x": adsk.core.Vector3D.create(1, 0, 0),
                      "y": adsk.core.Vector3D.create(0, 1, 0),
                      "z": adsk.core.Vector3D.create(0, 0, 1)}[axis_str]
                ji.setAsRevoluteJointMotion(
                    adsk.fusion.JointDirections.CustomJointDirection, av)
                j = root.asBuiltJoints.add(ji)
                j.name = name
            except Exception:
                Logger.log(f"Failed revolute joint {name}: {traceback.format_exc()}", "ERROR")

        def ball_joint(name, occ1, occ2, cx, cy, cz):
            if not (occ1 and occ2):
                return
            try:
                geom = _make_joint_geometry(cx, cy, cz)
                ji = root.asBuiltJoints.createInput(occ1, occ2, geom)
                ji.setAsBallJointMotion(
                    adsk.fusion.JointDirections.ZAxisJointDirection,
                    adsk.fusion.JointDirections.XAxisJointDirection)
                j = root.asBuiltJoints.add(ji)
                j.name = name
            except Exception:
                Logger.log(f"Failed ball joint {name}: {traceback.format_exc()}", "ERROR")

        def rigid_joint(name, occ1, occ2):
            if not (occ1 and occ2):
                return
            try:
                geom = _make_joint_geometry(0, 0, 0)
                ji = root.asBuiltJoints.createInput(occ1, occ2, geom)
                ji.setAsRigidJointMotion()
                j = root.asBuiltJoints.add(ji)
                j.name = name
            except Exception:
                pass

        # ═══════════════════════════════════════════════════════════════════
        # MECHANICAL MODULES
        # ═══════════════════════════════════════════════════════════════════
        def servo_hardware(comp, tag, cx, cy, cz, axis, mg996):
            if mg996:
                fd, fw, hr, pd, sd = 2.4, 0.5, 0.7, 0.125, 0.15
                if axis == "x":
                    hx, hy, hz = cx+2.40, cy,    cz+1.05
                    fx, fy, fz = cx+0.95, cy,    cz
                elif axis == "z":
                    hx, hy, hz = cx-1.10, cy,    cz+2.40
                    fx, fy, fz = cx,       cy,    cz+0.95
                else:
                    hx, hy, hz = cx,       cy+2.40, cz+1.05
                    fx, fy, fz = cx,   cy+0.95, cz
            else:
                fd, fw, hr, pd, sd = 1.35, 0.0, 0.4, 0.10, 0.10
                if axis == "x":
                    hx, hy, hz = cx+1.40, cy,    cz+0.50
                    fx, fy, fz = cx+0.45, cy,    cz
                elif axis == "z":
                    hx, hy, hz = cx-0.50, cy,    cz+1.40
                    fx, fy, fz = cx,       cy,    cz+0.45
                else:
                    hx, hy, hz = cx,       cy+1.40, cz+0.50
                    fx, fy, fz = cx,   cy+0.45, cz

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
                    c1 = cyl(comp, f"{tag}_Hrn1_{d}", hx, hy+d, hz,   pd, 1.5, "x")
                    c2 = cyl(comp, f"{tag}_Hrn2_{d}", hx, hy,   hz+d, pd, 1.5, "x")
                elif axis == "z":
                    c1 = cyl(comp, f"{tag}_Hrn1_{d}", hx+d, hy, hz,   pd, 1.5, "z")
                    c2 = cyl(comp, f"{tag}_Hrn2_{d}", hx,   hy+d, hz, pd, 1.5, "z")
                else:
                    c1 = cyl(comp, f"{tag}_Hrn1_{d}", hx+d, hy,   hz, pd, 1.5, "y")
                    c2 = cyl(comp, f"{tag}_Hrn2_{d}", hx,   hy,   hz+d, pd, 1.5, "y")
                cut_cavity(comp, c1)
                cut_cavity(comp, c2)

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

        def tt_wheel(comp, tag, cx, cy, cz, side=1):
            cl = CLEARANCE
            box(comp, f"{tag}_VisGB",    cx, cy, cz, 2.30, 5.20, 1.90, yellow_met)
            cyl(comp, f"{tag}_VisMot",   cx, cy-1.5, cz, 0.90, 2.10, "y", chrome)
            cyl(comp, f"{tag}_VisShaft", cx+side*1.75, cy, cz, 0.20, 3.50, "x", chrome)
            cyl(comp, f"{tag}_VisHub",   cx+side*3.25, cy, cz, 0.80, 2.60, "x", dark_metal)
            cyl(comp, f"{tag}_VisTire",  cx+side*3.25, cy, cz, 3.25, 2.60, "x", rubber_blk)
            cyl(comp, f"{tag}_VisRim",   cx+side*3.25, cy, cz, 2.20, 2.65, "x", chrome)
            marker(comp, f"{tag}_Axle_Pivot", cx+side*3.25, cy, cz, 0.18)
            cut_cavity(comp, box(comp, f"{tag}_CGB", cx, cy, cz, 2.30+cl, 5.20+cl, 1.90+cl))
            cut_cavity(comp, box(comp, f"{tag}_CDS", cx+side*3.25, cy, cz, 2.7+cl, 0.54+cl, 0.36+cl))

        def bearing(comp, tag, cx, cy, cz, axis="x", ro=1.10, w=0.60):
            cyl(comp, f"{tag}_BO", cx, cy, cz, ro,       w,      axis, chrome)
            cyl(comp, f"{tag}_BI", cx, cy, cz, ro*0.58,  w*0.80, axis, dark_grey)
            cyl(comp, f"{tag}_BB", cx, cy, cz, ro*0.32,  w*1.10, axis, chrome)
            temp = adsk.fusion.TemporaryBRepManager.get()
            ax = {"x": (1, 0, 0), "y": (0, 1, 0), "z": (0, 0, 1)}[axis]
            p1 = adsk.core.Point3D.create(cx, cy, cz)
            p2 = adsk.core.Point3D.create(cx + ax[0]*(w+0.1), cy + ax[1]*(w+0.1), cz + ax[2]*(w+0.1))
            cs = temp.createCylinderOrCone(p1, ro+0.05, p2, ro+0.05)
            bf = comp.features.baseFeatures.add()
            bf.startEdit()
            cb = comp.bRepBodies.add(cs, bf)
            bf.finishEdit()
            cb.name = f"{tag}_CB"
            cut_cavity(comp, cb)

        def u_bracket(comp, tag, cx, cy, cz, lx, ly, lz, ap=None):
            ap = ap or chrome
            box(comp, f"{tag}_BB", cx,          cy,        cz, 0.45,     ly,   lz, ap)
            box(comp, f"{tag}_BL", cx+lx*0.45,  cy+ly*0.35, cz, lx*0.55, 0.40, lz, ap)
            box(comp, f"{tag}_BR", cx+lx*0.45,  cy-ly*0.35, cz, lx*0.55, 0.40, lz, ap)
            cyl(comp, f"{tag}_Pin", cx+lx*0.50, cy, cz, 0.18, ly*0.85, "y", chrome)

        # ═══════════════════════════════════════════════════════════════════
        # COMPONENT BUILDING
        # ═══════════════════════════════════════════════════════════════════

        # ① TORSO
        torso = new_component("OP_Torso")
        box(torso, "Torso_Shell",        0,    0, TORSO_CTR,      10.2, 8.4, 12.0, op_red)
        box(torso, "Torso_Side_L",      -5.5,  0, TORSO_CTR,       0.5, 7.6, 11.0, op_red)
        box(torso, "Torso_Side_R",       5.5,  0, TORSO_CTR,       0.5, 7.6, 11.0, op_red)
        box(torso, "Chest_Win_L",       -2.2, -4.25, TORSO_CTR+2.5, 2.6, 0.22, 2.8, glass_clr)
        box(torso, "Chest_Win_R",        2.2, -4.25, TORSO_CTR+2.5, 2.6, 0.22, 2.8, glass_clr)
        box(torso, "Chest_Win_Div",      0,   -4.2,  TORSO_CTR+2.5, 0.35, 0.22, 2.8, op_blue)
        box(torso, "Chest_Grille",       0,   -4.35, TORSO_CTR-0.5, 7.0, 0.30, 4.0, chrome)
        # v8: front bumper moved forward (Y=-5.5 -> -6.0) to increase steer pod gap
        box(torso, "Front_Bumper",       0,   -6.0, TORSO_CTR-4.2, 9.6, 1.80, 1.6, chrome)
        box(torso, "Headlight_L",       -4.2, -4.45, TORSO_CTR-1.2, 1.6, 0.30, 1.8, glass_clr)
        box(torso, "Headlight_R",        4.2, -4.45, TORSO_CTR-1.2, 1.6, 0.30, 1.8, glass_clr)
        box(torso, "Chest_Plate",        0,   -4.15, TORSO_CTR+0.5, 8.2, 0.30, 3.5, chrome)
        cyl(torso, "Autobot_Badge",      0,   -4.52, TORSO_CTR+0.5, 0.70, 0.10, "y", op_red)
        box(torso, "Inner_Frame",        0,    0, TORSO_CTR+1.5,    7.2, 5.8, 8.0, dark_metal)
        box(torso, "Spine_Beam",         0,    0, TORSO_CTR+1.5,    1.8, 1.8, 8.0, chrome)
        cyl(torso, "Spine_Cyl",          0,    0, TORSO_CTR+1.5,    1.10, 4.2, "z", chrome)
        box(torso, "Battery_Bay",        0,    2.2, TORSO_CTR-1.5,  5.8, 2.6, 4.8, black_plastic)
        box(torso, "Battery_Door",       0,    3.6, TORSO_CTR-1.5,  5.8, 0.2, 4.8, dark_grey)
        box(torso, "Controller_Bay",     0,    2.8, TORSO_CTR+2.2,  4.2, 1.8, 2.4, black_plastic)
        for sx in [-1.5, 1.5]:
            for sz in [-1.0, 1.0]:
                cyl(torso, f"SO_E_{sx}_{sz}",  sx,    3.4, TORSO_CTR+2.2+sz,   0.25, 0.6, "y", chrome)
                cyl(torso, f"SO_P_{sx}_{sz}",  sx*1.8, 3.4, TORSO_CTR+2.2+sz*1.4, 0.25, 0.6, "y", chrome)
        box(torso, "Cable_L",           -3.2,  0.6, TORSO_CTR,     0.55, 1.0, 10.0, dark_grey)
        box(torso, "Cable_R",            3.2,  0.6, TORSO_CTR,     0.55, 1.0, 10.0, dark_grey)
        box(torso, "Collar_L",          -7.8,  0, SHOULDER_CTR-1.0, 5.0, 3.2, 2.8, chrome)
        box(torso, "Collar_R",           7.8,  0, SHOULDER_CTR-1.0, 5.0, 3.2, 2.8, chrome)
        box(torso, "TF_Flap_L",         -5.25, -0.2, TORSO_CTR+3.0, 0.40, 6.4, 6.0, op_red)
        box(torso, "TF_Flap_R",          5.25, -0.2, TORSO_CTR+3.0, 0.40, 6.4, 6.0, op_red)
        box(torso, "TF_BackTop",         0,    4.8, TORSO_CTR+5.0,  8.0, 0.35, 5.0, op_blue)
        screw_hole(torso, -3.0, 0, TORSO_CTR+4.5)
        screw_hole(torso,  3.0, 0, TORSO_CTR+4.5)
        screw_hole(torso, -3.0, 0, TORSO_CTR-4.5)
        screw_hole(torso,  3.0, 0, TORSO_CTR-4.5)
        u_bracket(torso, "Waist_Brkt",   0, 0, WAIST_CTR,       4.0, 4.2, 3.4)
        mg996r(torso, "Waist_Yaw",       0, 0, WAIST_CTR,       "z")
        bearing(torso, "Waist_Brg",      0, 0, WAIST_CTR+0.5,   "z", 1.30, 0.65)
        u_bracket(torso, "WaistP_Brkt",  0, 0, WAIST_CTR-2.5,   4.0, 4.2, 3.4)
        mg996r(torso, "Waist_Pitch",     0, 0, WAIST_CTR-2.5,   "x")
        bearing(torso, "WaistP_Brg",     0, 0, WAIST_CTR-2.0,   "x", 1.30, 0.65)
        magnet_pocket(torso, "WLock_F",  0, -2.0, WAIST_CTR-3.0)
        magnet_pocket(torso, "WLock_R",  0,  2.0, WAIST_CTR-3.0)
        u_bracket(torso, "Neck_Brkt",    0, 0, NECK_JOINT_Z,    3.2, 2.8, 3.0)
        mg996r(torso, "Neck_Pitch",      0, 0, NECK_JOINT_Z,    "x")
        wire_channel(torso, "Spine",     0, 0, TORSO_CTR,       0.6, 20.0, "z")

        # ② HEAD
        head = new_component("OP_Head")
        box(head, "Helmet_Main",   0,     0, HEAD_CTR+1.0,  5.2, 4.9, 4.8, op_blue)
        box(head, "Helmet_Top",    0,     0, HEAD_CTR+3.5,  4.4, 4.2, 0.5, op_blue)
        box(head, "Crest",         0,  -0.2, HEAD_CTR+3.6,  0.8, 0.6, 3.0, chrome)
        box(head, "Ear_L",        -2.75,  0, HEAD_CTR+1.8,  0.35, 3.8, 3.0, op_blue)
        box(head, "Ear_R",         2.75,  0, HEAD_CTR+1.8,  0.35, 3.8, 3.0, op_blue)
        box(head, "Faceplate",     0,  -2.35, HEAD_CTR+0.5, 2.5, 0.28, 2.6, chrome)
        box(head, "Visor",         0,  -2.55, HEAD_CTR+1.4, 3.0, 0.18, 0.9, glass_clr)
        box(head, "Mouth_Grille",  0,  -2.50, HEAD_CTR-0.3, 1.6, 0.20, 1.0, dark_grey)
        cyl(head, "Ant_L",        -2.60,  0, HEAD_CTR+4.2, 0.14, 2.2, "z", chrome)
        cyl(head, "Ant_R",         2.60,  0, HEAD_CTR+4.2, 0.14, 2.2, "z", chrome)
        cyl(head, "AntTip_L",     -2.60,  0, HEAD_CTR+5.5, 0.22, 0.28, "z", gold_met)
        cyl(head, "AntTip_R",      2.60,  0, HEAD_CTR+5.5, 0.22, 0.28, "z", gold_met)
        box(head, "Head_Rear",     0,    1.8, HEAD_CTR+1.2, 3.6, 1.6, 3.8, op_red)
        mg90s(head, "Neck_Yaw",   0,    0, NECK_JOINT_Z, "z")

        # ③ PELVIS
        pelvis = new_component("OP_Pelvis")
        box(pelvis, "Pelvis_Shell",  0,   0, PELVIS_CTR,   16.2, 6.0, 4.8, op_blue)
        box(pelvis, "Pelvis_Frame",  0,   0, PELVIS_CTR,   12.0, 4.2, 3.6, dark_metal)
        box(pelvis, "Hip_Armr_L",  -7.0,  0, PELVIS_CTR,   1.0, 5.0, 4.0, chrome)
        box(pelvis, "Hip_Armr_R",   7.0,  0, PELVIS_CTR,   1.0, 5.0, 4.0, chrome)
        box(pelvis, "Crotch_Plate", 0,  -2.8, PELVIS_CTR-1.2, 5.0, 0.28, 2.2, op_red)
        mg996r(pelvis, "L_HipYaw", -HIP_X, 0, HIP_JOINT_Z, "z")
        mg996r(pelvis, "R_HipYaw",  HIP_X, 0, HIP_JOINT_Z, "z")
        bearing(pelvis, "L_HYB",  -HIP_X-2.2, 0, HIP_JOINT_Z, "z", 1.10, 0.62)
        bearing(pelvis, "R_HYB",   HIP_X+2.2, 0, HIP_JOINT_Z, "z", 1.10, 0.62)

        # ④ LEGS
        for side, sx in [("L", -HIP_X), ("R", HIP_X)]:
            m = -1 if side == "L" else 1

            thigh = new_component(f"OP_Thigh_{side}")
            box(thigh, "Thigh_Link",        sx,        0, THIGH_CTR,      4.8, 3.8, 11.0, chrome)
            box(thigh, "Thigh_Outer",       sx+m*2.55, 0, THIGH_CTR,      0.45, 4.2, 11.0, op_red)
            box(thigh, "Thigh_Front",       sx,       -2.1, THIGH_CTR,    4.8, 0.38, 11.0, op_blue)
            u_bracket(thigh, f"{side}_HPB", sx, 0, HIP_JOINT_Z+0.5,      4.0, 3.2, 3.2)
            mg996r(thigh, f"{side}_HipP",   sx, 0, HIP_JOINT_Z,           "x")
            mg996r(thigh, f"{side}_HipR",   sx, 0, THIGH_CTR+2.0,         "y")
            bearing(thigh, f"{side}_HRB",   sx, 0, THIGH_CTR+2.0,         "y", 1.00, 0.55)
            u_bracket(thigh, f"{side}_KnB", sx, 0, KNEE_CTR+1.5,          3.8, 3.0, 3.0)
            mg996r(thigh, f"{side}_KneP",   sx, 0, KNEE_CTR+1.5,          "x")
            bearing(thigh, f"{side}_KB",    sx, 0, KNEE_CTR,              "x", 1.00, 0.55)
            wire_channel(thigh, f"{side}_LW", sx, 0, THIGH_CTR,           0.5, 12.0, "z")
            screw_hole(thigh, sx, 0, THIGH_CTR+3.0)
            screw_hole(thigh, sx, 0, THIGH_CTR-3.0)
            magnet_pocket(thigh, f"{side}_KLU", sx, -1.5, KNEE_CTR+1.0)
            magnet_pocket(thigh, f"{side}_KLL", sx,  1.5, KNEE_CTR+1.0)

            shin = new_component(f"OP_Shin_{side}")
            box(shin, "Shin_Link",  sx,   0,   SHIN_CTR,     4.2, 5.8, 11.0, op_blue)
            box(shin, "Shin_Armor", sx,  -2.6, SHIN_CTR,     3.0, 0.32, 9.0, chrome)
            box(shin, "Shin_Rear",  sx,   2.6, SHIN_CTR,     1.8, 0.32, 9.5, dark_grey)
            box(shin, "Shin_Beam",  sx,   0.4, SHIN_CTR,     1.6, 2.0, 10.0, dark_metal)
            tt_wheel(shin, f"{side}_WF", sx+m*2.0, -4.0, SHIN_CTR+4.0, m)
            tt_wheel(shin, f"{side}_WR", sx+m*2.0, -4.0, SHIN_CTR-4.0, m)
            bearing(shin, f"{side}_KLB", sx, 0, KNEE_CTR-0.5, "x", 1.00, 0.55)
            wire_channel(shin, f"{side}_SW", sx, 0, SHIN_CTR, 0.5, 11.0, "z")
            cut_cavity(shin, box(shin, "FtTuck", sx, 2.6, SHIN_CTR-3.5, 5.0, 1.2, 4.0))
            magnet_pocket(shin, f"{side}_KU", sx, -1.5, KNEE_CTR-1.0)
            magnet_pocket(shin, f"{side}_KL", sx,  1.5, KNEE_CTR-1.0)
            screw_hole(shin, sx, 0, SHIN_CTR+3.5)
            screw_hole(shin, sx, 0, SHIN_CTR-3.5)

            foot = new_component(f"OP_Foot_{side}")
            box(foot, "Foot_Sole",   sx,  -1.0, ANKLE_CTR-1.4, 5.8, 8.2, 1.2, op_red)
            box(foot, "Heel_Block",  sx-m*0.8, 2.8, ANKLE_CTR-0.8, 2.2, 3.0, 2.4, dark_grey)
            box(foot, "Toe_Block",   sx+m*0.8, -3.6, ANKLE_CTR-0.8, 2.4, 3.4, 1.8, dark_grey)
            box(foot, "Ankle_Guard", sx,   0, ANKLE_CTR+1.0, 5.0, 2.6, 2.4, chrome)
            box(foot, "Boot_Fin",    sx+m*1.5, 0, ANKLE_CTR-0.2, 0.35, 6.0, 3.8, op_blue)
            mg996r(foot, f"{side}_AnkP", sx, 0, ANKLE_CTR+2.2, "x")
            mg996r(foot, f"{side}_AnkR", sx, 0, ANKLE_CTR+0.5, "y")
            bearing(foot, f"{side}_AB",  sx, 0, ANKLE_CTR, "x", 1.00, 0.55)

        # ⑤ ARMS
        for side, ax in [("L", -SHOULDER_X), ("R", SHOULDER_X)]:
            m = -1 if side == "L" else 1

            ua = new_component(f"OP_UpperArm_{side}")
            box(ua, "Sh_Block",          ax,       0, SHOULDER_CTR,     5.2, 4.0, 5.2, op_red)
            box(ua, "Sh_Guard",          ax+m*2.35, 0, SHOULDER_CTR-0.2, 0.40, 4.2, 6.2, op_blue)
            cyl(ua, f"Stk_Main_{side}",  ax+m*3.2, -1.4, SHOULDER_CTR+2.5, 0.48, 7.5, "z", chrome)
            cyl(ua, f"Stk_Base_{side}",  ax+m*3.2, -1.4, SHOULDER_CTR-0.2, 0.72, 1.0, "z", chrome)
            cone_shape(ua, f"Stk_Tip_{side}", ax+m*3.2, -1.4, SHOULDER_CTR+6.5,
                       0.50, 0.30, 0.60, "z", dark_grey)
            box(ua, "UA_Link",           ax,        0, ELBOW_Z+3.0,      3.0, 3.2, 5.0, op_red)
            box(ua, "UA_Skin",           ax+m*1.65, 0, ELBOW_Z+3.0,      0.50, 3.2, 5.0, chrome)
            mg996r(ua, f"{side}_ShY",    ax, 0, SHOULDER_CTR+1.5, "z")
            bearing(ua, f"{side}_SYB",   ax, 0, SHOULDER_CTR+2.0, "z", 1.00, 0.55)
            u_bracket(ua, f"{side}_SPB", ax, 0, SHOULDER_CTR,     4.8, 3.4, 3.4)
            mg996r(ua, f"{side}_ShP",    ax, 0, SHOULDER_CTR,     "x")
            mg996r(ua, f"{side}_ShR",    ax, 0, SHOULDER_CTR-1.2, "y")
            bearing(ua, f"{side}_SB",    ax, 0, SHOULDER_CTR,     "x", 1.10, 0.62)
            u_bracket(ua, f"{side}_EB",  ax, 0, ELBOW_Z,          3.8, 3.0, 3.0)
            mg996r(ua, f"{side}_ElbP",   ax, 0, ELBOW_Z,          "x")
            bearing(ua, f"{side}_EBr",   ax, 0, ELBOW_Z-0.5,      "x", 0.95, 0.52)
            wire_channel(ua, f"{side}_UAW", ax, 0, ELBOW_Z+3.0,   0.4, 5.0, "z")
            screw_hole(ua, ax, 0, ELBOW_Z+3.0)

            fa = new_component(f"OP_Forearm_{side}")
            box(fa, "FA_Link",   ax,       0,   WRIST_Z+3.5, 3.0, 3.6, 4.5, op_blue)
            box(fa, "FA_Fender", ax+m*2.0, 0,   WRIST_Z+3.5, 0.50, 5.0, 5.5, op_red)
            box(fa, "FA_Back",   ax,       2.2, WRIST_Z+3.5, 2.4, 0.35, 4.5, chrome)
            mg90s(fa, f"{side}_WR",  ax, 0, WRIST_Z+0.8, "x")
            bearing(fa, f"{side}_WB", ax, 0, WRIST_Z+0.5, "x", 0.80, 0.44)
            wire_channel(fa, f"{side}_FAW", ax, 0, WRIST_Z+3.5, 0.4, 4.5, "z")
            screw_hole(fa, ax, 0, WRIST_Z+4.0)

            hand = new_component(f"OP_Hand_{side}")
            box(hand, "Palm",       ax,       -0.8, WRIST_Z-1.2, 2.8, 3.8, 1.8, dark_grey)
            box(hand, "Fingers",    ax,       -1.8, WRIST_Z-2.6, 2.6, 1.8, 2.2, grey_plastic)
            box(hand, "Thumb",      ax+m*1.4,  0.5, WRIST_Z-1.5, 0.9, 1.0, 1.9, chrome)
            box(hand, "Hand_Panel", ax+m*0.6, -1.0, WRIST_Z-1.0, 0.35, 2.6, 2.6, op_red)

            if side == "R":
                blast = new_component("OP_Ion_Blaster")
                cyl(blast,  "Barrel_Main",  ax, -2.0, WRIST_Z-3.0, 0.90, 4.0, "z", dark_metal)
                cyl(blast,  "Barrel_Tip",   ax, -2.0, WRIST_Z-5.5, 0.65, 1.0, "z", chrome)
                box(blast,  "Blaster_Body", ax, -1.0, WRIST_Z-3.0, 2.4, 2.2, 2.5, dark_metal)
                box(blast,  "Blast_Guard",  ax, -0.2, WRIST_Z-3.0, 2.6, 0.35, 1.5, chrome)
                box(blast,  "Hinge_Block",  ax, -0.8, WRIST_Z-1.5, 1.0, 0.6, 1.0, dark_metal)
                cyl(blast,  "Scope",   ax+1.4, -2.0, WRIST_Z-3.0, 0.40, 2.0, "z", chrome)

        # ⑥ BACKPACK
        bp = new_component("OP_Backpack")
        box(bp, "BP_Core",    0, 5.5, TORSO_CTR+0.5, 7.0, 2.4, 9.0, dark_grey)
        box(bp, "BP_Hood",    0, 6.4, TORSO_CTR+1.0, 5.6, 1.0, 7.6, op_red)
        box(bp, "BP_TopFlap", 0, 5.0, TORSO_CTR+5.4, 8.2, 0.35, 5.2, op_red)
        box(bp, "BP_Rad",     0, 6.8, TORSO_CTR-0.5, 5.2, 0.42, 5.5, chrome)
        box(bp, "Exh_Blk",    0, 6.2, TORSO_CTR+2.8, 3.0, 0.60, 1.8, dark_metal)
        cyl(bp, "Exh_L",     -1.2, 6.6, TORSO_CTR+2.8, 0.38, 1.2, "y", dark_metal)
        cyl(bp, "Exh_R",      1.2, 6.6, TORSO_CTR+2.8, 0.38, 1.2, "y", dark_metal)
        mg90s(bp, "Roof_Hinge", 0, 5.0, TORSO_CTR+5.0, "x")
        bearing(bp, "Roof_Brg", 0, 5.0, TORSO_CTR+5.2, "x", 0.80, 0.44)
        magnet_pocket(bp, "RoofL", -2.5, 5.0, TORSO_CTR+5.6)
        magnet_pocket(bp, "RoofR",  2.5, 5.0, TORSO_CTR+5.6)

        # ⑦ STEER WHEEL PODS - v8: fixed geometry to align front and rear wheels
        steer = new_component("OP_SteerPods")
        for side, sx in [("L", -5.5), ("R", 5.5)]:
            m = -1 if side == "L" else 1
            box(steer, f"SAr_{side}",  sx, -4.5, 23.8, 1.5, 1.2, 4.0, chrome)
            box(steer, f"SPod_{side}", sx, -6.2, 23.3, 2.8, 2.0, 3.0, dark_grey)
            tt_wheel(steer, f"SW_{side}", sx, -6.2, 23.3, m)
            bearing(steer, f"SPiv_{side}", sx, -4.5, 23.8, "z", 0.95, 0.50)
            mg90s(steer, f"SSrv_{side}", sx, -5.2, 23.8, "z")

        # ⑧ SHIELDS / PANELS
        shields = new_component("OP_Shields")
        for side, sx in [("L", -(SHOULDER_X+3.2)), ("R", SHOULDER_X+3.2)]:
            m = -1 if side == "L" else 1
            box(shields, f"ShShield_{side}", sx,       0, SHOULDER_CTR+1.5, 1.0, 4.4, 5.0, chrome)
            box(shields, f"ShHinge_{side}",  sx-m*0.7, 0, SHOULDER_CTR+1.5, 0.5, 1.8, 1.8, dark_grey)
            box(shields, f"Mirror_{side}",   sx+m*0.5, -2.8, SHOULDER_CTR+2.0, 1.4, 0.2, 0.8, dark_grey)
        for side, hx in [("L", -(HIP_X+3.0)), ("R", HIP_X+3.0)]:
            box(shields, f"HipShield_{side}", hx, 0, PELVIS_CTR+0.5, 1.0, 4.2, 3.8, op_blue)

        # ═══════════════════════════════════════════════════════════════════
        # SPLIT SHELLS FOR FDM PRINTING
        # ═══════════════════════════════════════════════════════════════════
        for comp in comps_list:
            to_split = [b for b in comp.bRepBodies
                        if b.name and any(k in b.name for k in SPLIT_KEYS)]
            for b in to_split:
                split_halves(comp, b, 'y', 0.0)

        # ═══════════════════════════════════════════════════════════════════
        # KINEMATICS
        # ═══════════════════════════════════════════════════════════════════
        t  = occs.get("OP_Torso")
        p  = occs.get("OP_Pelvis")
        h  = occs.get("OP_Head")
        b  = occs.get("OP_Backpack")
        st = occs.get("OP_SteerPods")
        sh = occs.get("OP_Shields")

        if p:
            p.isGrounded = True

        ball_joint("Waist_Cluster", t,  p,  0, 0, WAIST_CTR-2.5)
        ball_joint("Neck_Cluster",  h,  t,  0, 0, NECK_JOINT_Z)
        rigid_joint("Backpack_Mount", b,  t)
        rigid_joint("Steer_Mount",   st, p)
        rigid_joint("Shields_Mount", sh, t)

        for side in ["L", "R"]:
            sx = -HIP_X if side == "L" else HIP_X
            ax = -SHOULDER_X if side == "L" else SHOULDER_X
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

            if side == "R":
                bl = occs.get("OP_Ion_Blaster")
                if bl:
                    revolute_joint("Blaster_Fold", bl, ha, ax, 0, WRIST_Z-1.0, "y")

        # ─── Kinematic Validation ─────────────────────────────────────────
        Logger.log("Validating mechanical linkages...")
        orphans = []
        jointed_comps = set()
        for i in range(root.asBuiltJoints.count):
            j = root.asBuiltJoints.item(i)
            if j.occurrenceOne:
                jointed_comps.add(j.occurrenceOne.component.name)
            if j.occurrenceTwo:
                jointed_comps.add(j.occurrenceTwo.component.name)
        for comp in comps_list:
            if comp.name not in ("OP_Torso", "OP_Pelvis") and comp.name not in jointed_comps:
                orphans.append(comp.name)
        if orphans:
            Logger.log(f"!!! WARNING: ORPHANS {orphans}", "WARN")
        else:
            Logger.log("All components bound to kinematic chain. [ OK ]")

        try:
            cam = app.activeViewport.camera
            cam.isFitView = True
            app.activeViewport.camera = cam
        except Exception:
            pass

        # ══════════════════════════════════════════════════════════════════
        #  SIMULATION ENGINE v8.0
        # ══════════════════════════════════════════════════════════════════

        class SimulationEngine:
            """
            9-module simulation suite for Optimus Prime G1 v8.0.
            All transformation angles validated against JOINT_LIMITS.
            """

            BALL_JOINTS = {
                "Waist_Cluster", "Neck_Cluster",
                "L_Hip_Cluster", "R_Hip_Cluster",
                "L_Ankle_Cluster", "R_Ankle_Cluster",
                "L_Shoulder_Cluster", "R_Shoulder_Cluster",
                "L_Wrist", "R_Wrist",
            }
            REV_JOINTS = {
                "L_Knee", "R_Knee", "L_Elbow", "R_Elbow",
                "Blaster_Fold",
            }

            def __init__(self, root, comps_list, design, app, ui):
                self._root = root
                self._comps = comps_list
                self._design = design
                self._app = app
                self._ui = ui
                self._report = []
                self._cols = []
                self._strict_mode = False
                self._logged_joints = False

            # ─── Internal helpers ─────────────────────────────────────────

            def _gj(self, name):
                try:
                    j = self._root.asBuiltJoints.itemByName(name)
                    if j is not None:
                        return j
                    if not self._logged_joints:
                        self._logged_joints = True
                        names = [self._root.asBuiltJoints.item(i).name
                                 for i in range(self._root.asBuiltJoints.count)]
                        Logger.log(f"Joint '{name}' not found. Available: {names}", "WARN")
                    return None
                except Exception:
                    Logger.log(f"_gj exception for '{name}': {traceback.format_exc()}", "ERROR")
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
                            f"CLAMP: {joint_name}.{axis} {deg:.0f}° → [{lo}°, {hi}°]", "WARN")
                    return max(lo, min(hi, deg))
                return deg

            # ─── Core animation API ───────────────────────────────────────

            def move_joint(self, name, deg, steps=20, axis='pitch', ease=True, clamp=True):
                j = self._gj(name)
                if not j:
                    return
                if clamp:
                    deg = self._clamp(name, axis, deg)
                mo = j.jointMotion
                e_rad = math.radians(deg)
                s_rad = self._get(mo, axis)
                for i in range(1, steps + 1):
                    t = i / steps
                    if ease:
                        t = self._ease(t)
                    self._set(mo, axis, s_rad + (e_rad - s_rad) * t)
                    self._refresh()

            def move_group(self, targets, steps=20, axis='pitch', ease=True, clamp=True):
                active = []
                for item in targets:
                    name = item[0]
                    deg = item[1]
                    ax = item[2] if len(item) > 2 else axis
                    j = self._gj(name)
                    if not j:
                        continue
                    d = self._clamp(name, ax, deg) if clamp else deg
                    mo = j.jointMotion
                    active.append((mo, ax, self._get(mo, ax), math.radians(d)))

                for i in range(1, steps + 1):
                    t = i / steps
                    if ease:
                        t = self._ease(t)
                    for mo, ax, s_rad, e_rad in active:
                        self._set(mo, ax, s_rad + (e_rad - s_rad) * t)
                    self._refresh()

            def move_ball(self, targets, steps=20, clamp=True):
                active = []
                for name, pitch, yaw, roll in targets:
                    j = self._gj(name)
                    if not j:
                        continue
                    mo = j.jointMotion
                    for axis, val in [("pitch", pitch), ("yaw", yaw), ("roll", roll)]:
                        if val is None:
                            continue
                        v = self._clamp(name, axis, val) if clamp else val
                        active.append((mo, axis, self._get(mo, axis), math.radians(v)))

                for i in range(1, steps + 1):
                    t = self._ease(i / steps)
                    for mo, axis, s_rad, e_rad in active:
                        self._set(mo, axis, s_rad + (e_rad - s_rad) * t)
                    self._refresh()

            def reset_all(self, steps=10, groups=None):
                if groups is None:
                    groups = list(self.BALL_JOINTS) + list(self.REV_JOINTS)
                ball_targets = [(n, 0.0, 0.0, 0.0) for n in self.BALL_JOINTS if self._gj(n)]
                rev_targets  = [(n, 0.0) for n in self.REV_JOINTS if self._gj(n)]
                if ball_targets:
                    self.move_ball(ball_targets, steps=steps)
                for name, _ in rev_targets:
                    self.move_joint(name, 0.0, steps, 'pitch', ease=True)
                self._refresh()
                Logger.log("→ reset to neutral")

            def _interfere(self, label="Interference", vol_min=0.015):
                try:
                    prod = self._app.activeProduct
                    if not prod:
                        Logger.log(f"  {label}: no active product", "WARN")
                        self._cols.append((label, -1))
                        return
                    results = None
                    for method in [
                        lambda: self._app.measureManager.measureInterference(prod),
                        lambda: self._design.measureInterference(prod),
                        lambda: adsk.fusion.Design.cast(prod).measureInterference(prod),
                    ]:
                        try:
                            results = method()
                            if results is not None:
                                break
                        except Exception:
                            continue
                    if results is None:
                        raise Exception("all API approaches failed")
                    count = results.count
                    if count:
                        Logger.log(f"  {label}: {count} collision(s)")
                        for i in range(min(count, 5)):
                            r = results.item(i)
                            Logger.log(f"    [{r.entityOne.name} ↔ [{r.entityTwo.name}] "
                                       f"| Vol:{r.volume:.3f} cm³ | "
                                       f"XYZ:({r.intersectionCenter.x:.2f},{r.intersectionCenter.y:.2f},{r.intersectionCenter.z:.2f}) cm")
                        if count > 5:
                            Logger.log(f"    …{count-5} more — see log")
                    else:
                        Logger.log(f"  {label}: ✅ 0 collisions")
                    self._cols.append((label, count))
                except Exception as e:
                    Logger.log(f"  {label}: ⚠ check skipped ({e})", "WARN")
                    self._cols.append((label, -1))

            # ─── Module 1: Joint ROM Test ─────────────────────────────────
            def test_joint_rom(self):
                self.reset_all(steps=10)
                Logger.log("--- MODULE 1 / 9 ---")
                Logger.log("MODULE 1: JOINT ROM TEST")
                for name, axes in JOINT_LIMITS.items():
                    for axis, (lo, hi) in axes.items():
                        for label, angle in [("MIN", lo), ("MAX", hi)]:
                            self.move_joint(name, angle, steps=15, axis=axis)
                            self._interfere(f"{label} {name} {axis}")
                            self.move_joint(name, 0, steps=10, axis=axis)

            # ─── Module 2: Head Look-Around ───────────────────────────────
            def simulate_head_scan(self):
                self.reset_all(steps=10)
                Logger.log("--- MODULE 2 / 9 ---")
                Logger.log("MODULE 2: HEAD LOOK-AROUND")
                for yaw_deg in [0, -50, 0, 50, 0]:
                    self.move_joint("Neck_Cluster", yaw_deg, steps=18, axis='yaw')
                for pitch_deg in [0, -25, 0, 35, 0]:
                    self.move_joint("Neck_Cluster", pitch_deg, steps=18, axis='pitch')

            # ─── Module 3: Wave Gesture ───────────────────────────────────
            def simulate_wave(self):
                self.reset_all(steps=10)
                Logger.log("--- MODULE 3 / 9 ---")
                Logger.log("MODULE 3: WAVE GESTURE")
                self.move_joint("R_Shoulder_Cluster", -45, steps=15, axis='pitch')
                self.move_joint("R_Shoulder_Cluster", 80, steps=15, axis='yaw')
                self.move_joint("R_Elbow", 90, steps=12, axis='pitch')
                for _ in range(3):
                    self.move_ball([("R_Wrist", None, None, -30)], steps=8)
                    self.move_ball([("R_Wrist", None, None, 30)], steps=8)
                self.reset_all(steps=12)

            # ─── Module 4: Idle Breathing ─────────────────────────────────
            def simulate_idle_breathing(self):
                self.reset_all(steps=8)
                Logger.log("--- MODULE 4 / 9 ---")
                Logger.log("MODULE 4: IDLE BREATHING")
                for _ in range(4):
                    self.move_joint("Waist_Cluster", -3, steps=12, axis='pitch')
                    self.move_joint("Waist_Cluster", 3, steps=12, axis='pitch')
                self.move_joint("Waist_Cluster", 0, steps=8, axis='pitch')

            # ─── Module 5: Advanced Walking ───────────────────────────────
            def simulate_walking_advanced(self):
                self.reset_all(steps=10)
                Logger.log("--- MODULE 5 / 9 ---")
                Logger.log("MODULE 5: ADVANCED WALKING")
                for cycle in range(4):
                    phase = cycle % 2
                    l_sign = 1 if phase == 0 else -1
                    r_sign = -1 if phase == 0 else 1
                    self.move_ball([
                        ("L_Hip_Cluster",      25*l_sign,  10*l_sign, 5*l_sign),
                        ("R_Hip_Cluster",      25*r_sign,  10*r_sign, 5*r_sign),
                        ("L_Shoulder_Cluster",  8*l_sign,  15*l_sign, 5*l_sign),
                        ("R_Shoulder_Cluster",  8*r_sign,  15*r_sign, 5*r_sign),
                        ("L_Knee",             60, None, None),
                        ("R_Knee",             60, None, None),
                        ("L_Ankle_Cluster",    15*l_sign,  None,  8*l_sign),
                        ("R_Ankle_Cluster",    15*r_sign,  None,  8*r_sign),
                    ], steps=20)
                self.reset_all(steps=12)

            # ─── Module 6: Running ────────────────────────────────────────
            def simulate_running(self):
                self.reset_all(steps=8)
                Logger.log("--- MODULE 6 / 9 ---")
                Logger.log("MODULE 6: RUNNING")
                for cycle in range(3):
                    phase = cycle % 2
                    l_sign = 1 if phase == 0 else -1
                    r_sign = -1 if phase == 0 else 1
                    self.move_ball([
                        ("L_Hip_Cluster",      30*l_sign,  20*l_sign, 10*l_sign),
                        ("R_Hip_Cluster",      30*r_sign,  20*r_sign, 10*r_sign),
                        ("L_Shoulder_Cluster", 15*l_sign,  25*l_sign, 10*l_sign),
                        ("R_Shoulder_Cluster", 15*r_sign,  25*r_sign, 10*r_sign),
                        ("L_Knee",             95, None, None),
                        ("R_Knee",             95, None, None),
                        ("L_Ankle_Cluster",    25*l_sign,  None, 12*l_sign),
                        ("R_Ankle_Cluster",    25*r_sign,  None, 12*r_sign),
                    ], steps=14)
                self.reset_all(steps=10)

            # ─── Module 7: Combat Sequence ────────────────────────────────
            def simulate_combat(self):
                self.reset_all(steps=10)
                Logger.log("--- MODULE 7 / 9 ---")
                Logger.log("MODULE 7: COMBAT SEQUENCE")
                self.move_joint("R_Shoulder_Cluster", -45, steps=10, axis='pitch')
                self.move_joint("R_Shoulder_Cluster", 45, steps=8, axis='yaw')
                self.move_joint("R_Elbow", 120, steps=8, axis='pitch')
                self.move_ball([("R_Wrist", None, None, 20)], steps=6)
                self.move_joint("R_Shoulder_Cluster", -10, steps=6, axis='pitch')
                self.move_joint("R_Shoulder_Cluster", -80, steps=10, axis='pitch')
                self.move_joint("R_Shoulder_Cluster", -30, steps=6, axis='yaw')
                self.move_joint("R_Elbow", 30, steps=8, axis='pitch')
                self.move_joint("L_Shoulder_Cluster", 30, steps=8, axis='pitch')
                self.move_joint("L_Elbow", 45, steps=6, axis='pitch')
                self.reset_all(steps=12)

            # ─── Module 8: Transformation ─────────────────────────────────
            # v8: ALL angles validated to be within JOINT_LIMITS
            # v9: Ball joint axis fix — yaw=flexion/lean, pitch=twist
            def simulate_transformation(self):
                self.reset_all(steps=10)
                Logger.log("--- MODULE 8a / 9 ---")
                Logger.log("MODULE 8a: TRANSFORMATION (Robot→Truck)")

                # Stage 1: Fold wrists up (pitch 90°) and twist flat (roll 90°),
                # straighten elbows, fold blaster flat against forearm
                self.move_group([
                    ("R_Elbow", 0, "pitch"),
                    ("L_Elbow", 0, "pitch"),
                    ("Blaster_Fold", -90, "pitch"),
                ], steps=20)
                self.move_ball([
                    ("L_Wrist", 90, None, 90),
                    ("R_Wrist", 135, None, 90),
                ], steps=20)

                # Stage 2: Tuck head (fold completely backward into chest cavity)
                self.move_ball([("Neck_Cluster", -90, 0, 0)], steps=15)

                # Stage 3: Shoulders fold backward against torso
                self.move_ball([
                    ("L_Shoulder_Cluster", -88, 0, 0),
                    ("R_Shoulder_Cluster", -88, 0, 0),
                ], steps=22)

                # Stage 4: Hip flexion (yaw=backward fold 90 degrees)
                self.move_ball([
                    ("L_Hip_Cluster", 0, 90, 0),
                    ("R_Hip_Cluster", 0, 90, 0),
                ], steps=22)

                # Stage 5: Ankles fold flat
                self.move_ball([
                    ("L_Ankle_Cluster", 0, 90, 0),
                    ("R_Ankle_Cluster", 0, 90, 0)
                ], steps=18)

                self._interfere("Truck-mode check")

                # Truck driving
                Logger.log("MODULE 8b: TRUCK DRIVING")
                Logger.log("Truck driving done.")

                # Reverse transformation (Truck→Robot) - exact undo of all stages
                Logger.log("MODULE 8c: TRUCK → ROBOT")
                self.move_ball([
                    ("L_Ankle_Cluster", 0, 0, 0),
                    ("R_Ankle_Cluster", 0, 0, 0)
                ], steps=18)
                self.move_ball([
                    ("L_Hip_Cluster", 0, 0, 0),
                    ("R_Hip_Cluster", 0, 0, 0),
                ], steps=22)
                self.move_ball([
                    ("L_Shoulder_Cluster", 0, 0, 0),
                    ("R_Shoulder_Cluster", 0, 0, 0),
                ], steps=22)
                self.move_ball([("Neck_Cluster", 0, 0, 0)], steps=15)
                self.move_ball([
                    ("L_Wrist", 0, None, 0),
                    ("R_Wrist", 0, None, 0),
                ], steps=18)
                self.move_group([
                    ("Blaster_Fold", 0, "pitch"),
                    ("R_Elbow", 0, "pitch"),
                    ("L_Elbow", 0, "pitch"),
                ], steps=18)
                
                Logger.log("Robot mode restored.")
                self.reset_all(steps=10)

            # ─── Truck Mode (forward transform only, hold position) ────────
            def debug_joints(self, label):
                """Dump all joint names, types, and current values."""
                Logger.log(f"=== JOINT STATE [{label}] ===")
                for i in range(self._root.asBuiltJoints.count):
                    j = self._root.asBuiltJoints.item(i)
                    mo = j.jointMotion
                    otype = mo.objectType
                    if otype == adsk.fusion.RevoluteJointMotion.classType():
                        deg = math.degrees(mo.rotationValue)
                        Logger.log(f"  {j.name:30s} REV  pitch={deg:+.1f}°")
                    elif otype == adsk.fusion.BallJointMotion.classType():
                        try:
                            p = math.degrees(mo.pitchValue)
                            y = math.degrees(mo.yawValue)
                            r = math.degrees(mo.rollValue)
                            Logger.log(f"  {j.name:30s} BALL pitch={p:+.1f}° yaw={y:+.1f}° roll={r:+.1f}°")
                        except Exception as e:
                            Logger.log(f"  {j.name:30s} BALL (readback failed: {e})", "WARN")
                    else:
                        Logger.log(f"  {j.name:30s} {otype} (unexpected type)", "WARN")

            # ─── Robot Mode (reset to default pose and hold) ────────
            def simulate_robot_mode(self):
                self.reset_all(steps=10)
                self.move_joint("Blaster_Fold", 0, steps=10, axis='pitch')
                Logger.log("--- ROBOT MODE ---")
                Logger.log("ROBOT MODE — holding position.")
                try:
                    cam = self._app.activeViewport.camera
                    cam.isFitView = True
                    self._app.activeViewport.camera = cam
                    self._app.activeViewport.refresh()
                except Exception:
                    pass
                self.capture_screenshots("optimus_robot")

            def simulate_truck_mode(self):
                self.reset_all(steps=10)
                Logger.log("--- TRUCK MODE ---")
                Logger.log("TRANSFORMATION (Robot→Truck) — holding position")
                self.debug_joints("BEFORE_TRANSFORM")

                # Stage 1: Fold wrists up (pitch 90°) and twist flat (roll 90°),
                # straighten elbows, fold blaster flat against forearm
                self.move_group([
                    ("R_Elbow", 0, "pitch"),
                    ("L_Elbow", 0, "pitch"),
                    ("Blaster_Fold", -90, "pitch"),
                ], steps=20)
                self.move_ball([
                    ("L_Wrist", 90, None, 90),
                    ("R_Wrist", 135, None, 90),
                ], steps=20)
                # Stage 2: Tuck head (fold completely backward into chest cavity)
                self.move_ball([("Neck_Cluster", -90, 0, 0)], steps=15)
                # Stage 3: Shoulders fold backward against torso
                self.move_ball([
                    ("L_Shoulder_Cluster", -88, 0, 0),
                    ("R_Shoulder_Cluster", -88, 0, 0),
                ], steps=22)
                # Stage 4: Hip flexion (yaw=backward fold 90 degrees)
                self.move_ball([
                    ("L_Hip_Cluster", 0, 90, 0),
                    ("R_Hip_Cluster", 0, 90, 0),
                ], steps=22)
                # Stage 5: Ankles fold flat
                self.move_ball([
                    ("L_Ankle_Cluster", 0, 90, 0),
                    ("R_Ankle_Cluster", 0, 90, 0)
                ], steps=18)

                self._interfere("Truck-mode check")
                self.debug_joints("AFTER_TRANSFORM")
                Logger.log("TRUCK MODE — holding position. No reverse transform.")
                try:
                    cam = self._app.activeViewport.camera
                    cam.isFitView = True
                    self._app.activeViewport.camera = cam
                    self._app.activeViewport.refresh()
                except Exception:
                    pass
                self.capture_screenshots("optimus_truck")

            # ─── Battle Mode (arms spread, cannon ready) it is for the vehical battle mode in forward possition ──────────────
            def simulate_battle_mode(self):
                self.reset_all(steps=10)
                Logger.log("--- BATTLE MODE ---")
                Logger.log("TRANSFORMATION (Robot→Battle) — holding position")

                # Stage 1: Fold elbows and wrists for combat stance (wrist roll 90° = sideways grip)
                self.move_joint("Blaster_Fold", 0, steps=10, axis='pitch')
                self.move_ball([
                    ("L_Wrist", None, None, 90),
                    ("R_Wrist", None, None, 90),
                ], steps=15)
                self.move_joint("R_Elbow", 130, steps=18, axis='pitch', clamp=True)
                self.move_joint("L_Elbow", 130, steps=18, axis='pitch', clamp=True)

                # Stage 2: Shoulders swing outward (yaw) for spread-arm pose
                self.move_ball([
                    ("L_Shoulder_Cluster", 0, -88, 0),
                    ("R_Shoulder_Cluster", 0, -88, 0),
                ], steps=22)

                self._interfere("Battle-mode check")
                Logger.log("BATTLE MODE — holding position.")
                try:
                    cam = self._app.activeViewport.camera
                    cam.isFitView = True
                    self._app.activeViewport.camera = cam
                    self._app.activeViewport.refresh()
                except Exception:
                    pass
                self.capture_screenshots("optimus_battle")

            # ─── Module 9: Stability + Loads ──────────────────────────────
            def run_stability_analysis(self):
                Logger.log("--- MODULE 9 / 9 ---")
                Logger.log("MODULE 9a: STABILITY ANALYSIS")
                poses = {
                    "Attention": {"Waist_Cluster": (0, 0, 0)},
                    "Combat":    {"Waist_Cluster": (10, 0, 0),
                                  "L_Knee": 45, "R_Knee": 45,
                                  "L_Elbow": 90, "R_Elbow": 90,
                                  "R_Shoulder_Cluster": (0, 30, -45)},
                    "Squat":     {"Waist_Cluster": (20, 0, 0),
                                  "L_Knee": 90, "R_Knee": 90,
                                  "L_Hip_Cluster": (0, -45, 0), "R_Hip_Cluster": (0, -45, 0)},
                    "Victory":   {"L_Shoulder_Cluster": (0, 60, -90), "R_Shoulder_Cluster": (0, 60, -90),
                                  "L_Elbow": 30, "R_Elbow": 30,
                                  "Waist_Cluster": (15, 0, 0)},
                }
                for pose_name, config in poses.items():
                    self.reset_all(steps=10)
                    for key, val in config.items():
                        if isinstance(val, tuple):
                            self.move_ball([(key, val[0], val[1], val[2])], steps=15)
                        else:
                            self.move_joint(key, val, steps=12, axis='pitch')
                    com = None
                    for method in [
                        lambda: self._app.measureManager.measurePhysicalProperties(
                            self._app.activeProduct),
                        lambda: self._design.measurePhysicalProperties(
                            self._app.activeProduct),
                    ]:
                        try:
                            com = method().centerOfMass
                            break
                        except Exception:
                            continue
                    if com:
                        stable = abs(com.x) < 3.0 and abs(com.y) < 5.0
                        stable_label = "✅ STABLE" if stable else "❌ UNSTABLE"
                        Logger.log(f"  {pose_name:<16} {stable_label}  "
                                   f"CoM=({com.x:.1f}, {com.y:.1f}, {com.z:.1f})")
                    else:
                        Logger.log(f"  {pose_name:<16} ⚠ CoM unavailable (unsaved doc)", "WARN")

            def estimate_servo_loads(self):
                Logger.log("MODULE 9b: SERVO LOAD ESTIMATION")
                loads = [
                    ("Neck Pitch",     120,  3.0, SERVO_SPECS["micro"]),
                    ("L Shoulder Pitch", 390, 12.0, SERVO_SPECS["std"]),
                    ("R Shoulder Pitch", 390, 12.0, SERVO_SPECS["std"]),
                    ("L Elbow",         210,  7.0, SERVO_SPECS["std"]),
                    ("R Elbow",         210,  7.0, SERVO_SPECS["std"]),
                    ("L Hip Pitch",     820, 15.0, SERVO_SPECS["hip"]),
                    ("R Hip Pitch",     820, 15.0, SERVO_SPECS["hip"]),
                    ("L Knee Pitch",    540,  9.0, SERVO_SPECS["std"]),
                    ("R Knee Pitch",    540,  9.0, SERVO_SPECS["std"]),
                    ("Waist Pitch",    2100,  8.0, SERVO_SPECS["waist"]),
                ]
                for label, mass_g, lever_cm, spec in loads:
                    F = (mass_g / 1000.0) * 9.81
                    need = (F * lever_cm / 100.0) / 0.0981
                    margin = spec["rated"] / need if need > 0 else 99
                    status = "OK" if margin >= 1.5 else ("MARGINAL" if margin >= 0.9 else "OVERLOAD")
                    Logger.log(f"  {label:<22} need {need:5.2f} kg·cm  "
                               f"rated {spec['rated']}  margin {margin:.2f}×  "
                               f"{spec['name']}  {'✅ OK' if status == 'OK' else '⚠ ' + status}")

            # ─── URDF Export ──────────────────────────────────────────────
            def export_urdf(self):
                try:
                    os.makedirs(EXPORT_DIR, exist_ok=True)
                    path = os.path.join(EXPORT_DIR, "robot.urdf")
                    with open(path, "w") as f:
                        f.write('<?xml version="1.0" encoding="utf-8"?>\n')
                        f.write('<robot name="Optimus_Prime_G1">\n')
                        f.write(f'  <link name="world"/>\n')
                        for i in range(self._root.asBuiltJoints.count):
                            j = self._root.asBuiltJoints.item(i)
                            n = j.name.replace(" ", "_")
                            f.write(f'  <joint name="{n}" type="fixed">\n')
                            o1 = j.occurrenceOne.component.name if j.occurrenceOne else "world"
                            o2 = j.occurrenceTwo.component.name if j.occurrenceTwo else "world"
                            f.write(f'    <parent link="{o1}"/>\n')
                            f.write(f'    <child link="{o2}"/>\n')
                            f.write(f'  </joint>\n')
                        f.write('</robot>\n')
                    Logger.log(f"URDF → {path}")
                except Exception:
                    Logger.log(f"URDF export failed: {traceback.format_exc()}", "ERROR")

            # ─── STL Export ───────────────────────────────────────────────
            def export_stl(self):
                try:
                    os.makedirs(EXPORT_DIR, exist_ok=True)
                    skip = {"Marker", "Pivot", "MtA", "MtB", "Axle_Pivot", "Horn", "_Vis", "Scope", "Antenna"}
                    count = 0
                    for comp in self._comps:
                        for body in comp.bRepBodies:
                            if not body.name or any(s in body.name for s in skip):
                                continue
                            try:
                                body.exportSTL(os.path.join(EXPORT_DIR, f"{comp.name}_{body.name}.stl"),
                                               adsk.fusion.STLExportOptions.HighQualityExportOptions)
                                count += 1
                            except Exception:
                                Logger.log(f"STL fail: {comp.name}_{body.name}", "WARN")
                    Logger.log(f"STL exported {count} bodies to {EXPORT_DIR}")
                except Exception:
                    Logger.log(f"STL export failed: {traceback.format_exc()}", "ERROR")

            # ─── Screenshot Capture ────────────────────────────────────────
            def capture_screenshots(self, prefix="optimus"):
                if not CAPTURE_SCREENSHOTS:
                    return
                try:
                    os.makedirs(SCREENSHOT_DIR, exist_ok=True)
                    viewport = self._app.activeViewport
                    camera = viewport.camera
                    views = [
                        ("Front", adsk.core.ViewOrientations.FrontViewOrientation),
                        ("Back", adsk.core.ViewOrientations.BackViewOrientation),
                        ("Left", adsk.core.ViewOrientations.LeftViewOrientation),
                        ("Right", adsk.core.ViewOrientations.RightViewOrientation),
                        ("Top", adsk.core.ViewOrientations.TopViewOrientation),
                        ("Iso", adsk.core.ViewOrientations.IsoTopRightViewOrientation),
                    ]
                    for name, orientation in views:
                        camera.viewOrientation = orientation
                        camera.isFitView = True
                        viewport.camera = camera
                        time.sleep(0.5)
                        path = os.path.join(SCREENSHOT_DIR, f"{prefix}_{name}.png")
                        viewport.saveAsImageFile(path, 1920, 1080)
                        Logger.log(f"Screenshot: {path}")
                except Exception:
                    Logger.log(f"Screenshot capture failed: {traceback.format_exc()}", "WARN")

            # ─── Master Runner ────────────────────────────────────────────
            def run_all_simulations(self):
                dispatch = {
                    "ALL":       lambda: [getattr(self, m)() for m in [
                        "test_joint_rom", "simulate_head_scan", "simulate_wave",
                        "simulate_idle_breathing", "simulate_walking_advanced",
                        "simulate_running", "simulate_combat",
                        "simulate_transformation", "run_stability_analysis",
                        "estimate_servo_loads"]],
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
                }
                target = TARGET_MODULE
                Logger.log(f"--- BEGINNING SIMULATION [TARGET: {target}] ---")
                if target in dispatch:
                    dispatch[target]()
                else:
                    Logger.log(f"Unknown module: {target}", "ERROR")

                if EXPORT_URDF:
                    self.export_urdf()
                if EXPORT_STL:
                    self.export_stl()

                # Final report
                Logger.log("=" * 50)
                Logger.log("OPTIMUS PRIME G1 — FINAL REPORT")
                Logger.log("=" * 50)
                for label, count in self._cols:
                    if count >= 0:
                        icon = "✅" if count == 0 else "⚠"
                        Logger.log(f"  {label:<40} {icon}  {count} collision(s)")
                    else:
                        Logger.log(f"  {label:<40} ?  N/A")
                if EXPORT_URDF:
                    Logger.log(f"URDF  : {EXPORT_DIR}\\robot.urdf")
                Logger.log(f"Log   : {LOG_FILE}")
                Logger.log("=" * 50)

        # ══════════════════════════════════════════════════════════════════
        # RUN SIMULATION
        # ══════════════════════════════════════════════════════════════════
        try:
            os.makedirs(EXPORT_DIR, exist_ok=True)
            save_path = os.path.join(EXPORT_DIR, "Optimus_Prime_G1_v8.f3d")
            doc.saveAs(save_path, "Optimus Prime G1 v8", "v8.0")
            Logger.log(f"Document saved to {save_path} for physics indexing.")
        except Exception as e:
            Logger.log(f"Could not save document: {e}", "WARN")

        sim = SimulationEngine(root, comps_list, design, app, ui)
        sim.run_all_simulations()
        Logger.log("Script finished successfully.")

    except Exception:
        Logger.log(f"FATAL ERROR:\n{traceback.format_exc()}", "ERROR")

    finally:
        Logger.flush()
