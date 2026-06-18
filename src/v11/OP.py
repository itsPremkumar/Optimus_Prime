# ─────────────────────────────────────────────────────────────────────────────
# OPTIMUS PRIME G1  v11.0  ─ Physical Build Edition (CORRECTED & ENHANCED)
# Fusion 360 Python Script  |  Build-Ready Robot Platform
# ─────────────────────────────────────────────────────────────────────────────
if 'TARGET_MODULE' not in globals(): TARGET_MODULE = "ALL"
if 'EXPORT_STL' not in globals(): EXPORT_STL = False
if 'EXPORT_STEP' not in globals(): EXPORT_STEP = False
if 'EXPORT_URDF' not in globals(): EXPORT_URDF = False
if 'CAPTURE_SCREENSHOTS' not in globals(): CAPTURE_SCREENSHOTS = False
if 'GENERATE_DOCS' not in globals(): GENERATE_DOCS = True

import adsk.core, adsk.fusion, traceback, math, os, csv, json, datetime, time

_ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
try: SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
except NameError: SCRIPT_DIR = os.getcwd()

PROJECT_DIR    = os.path.dirname(os.path.dirname(SCRIPT_DIR))
_OUTPUT_DIR    = os.path.join(PROJECT_DIR, "output")
LOG_DIR        = globals().get("LOG_DIR") or os.path.join(_OUTPUT_DIR, "logs")
SCREENSHOT_DIR = globals().get("SCREENSHOT_DIR") or os.path.join(_OUTPUT_DIR, "screenshots")
EXPORT_DIR     = globals().get("EXPORT_DIR") or os.path.join(_OUTPUT_DIR, "exports")
JIG_DIR        = os.path.join(EXPORT_DIR, "jigs")
DOCS_DIR       = os.path.join(_OUTPUT_DIR, "build_docs")
LOG_FILE       = os.path.join(LOG_DIR, f"optimus_v11_{_ts}.txt")
BOM_FILE       = os.path.join(_OUTPUT_DIR, f"BOM_v11_{_ts}.csv")
GUIDE_FILE     = os.path.join(DOCS_DIR, f"assembly_guide_v11_{_ts}.txt")
STOP_FLAG      = os.path.join(_OUTPUT_DIR, "stop.flag")

# ── Constants ────────────────────────────────────────────────────────────────
CLEARANCE = 0.060
ANKLE_CTR, SHIN_CTR, KNEE_CTR, THIGH_CTR = 3.8, 9.3, 14.8, 20.3
PELVIS_CTR, WAIST_CTR, TORSO_CTR = 30.5, 32.5, 36.0
SHOULDER_CTR, HEAD_CTR = 41.5, 47.5
HIP_X, SHOULDER_X = 5.8, 13.0
ELBOW_Z, WRIST_Z, HIP_JOINT_Z, NECK_JOINT_Z = 35.0, 29.0, 26.8, 44.5

WALL_S, WALL_P, WALL_F, WALL_THIN, CHAMFER_D = 0.30, 0.20, 0.15, 0.12, 0.10
M3_CLR_R, M3_PILOT_R, M3_HEAD_R, M3_HEAD_H = 0.160, 0.125, 0.285, 0.300
M3_NUT_CIR, M3_NUT_H, M3_WASHER_OD, M3_WASHER_ID = 0.320, 0.240, 0.350, 0.165
INSERT_R, INSERT_H, BOSS_R = 0.235, 0.500, 0.350
ALIGN_PIN_R, ALIGN_SOCK_R = 0.100, 0.115
GROMMET_R, GROMMET_SLOT = 0.175, 0.25

SCREW_LENGTHS = {
    "M2x6": 0.60, "M2x8": 0.80, "M2x10": 1.00, "M2.5x6": 0.60, "M2.5x8": 0.80,
    "M2.5x10": 1.00, "M3x6": 0.60, "M3x8": 0.80, "M3x10": 1.00, "M3x12": 1.20,
    "M3x14": 1.40, "M3x16": 1.60, "M3x18": 1.80, "M3x20": 2.00, "M3x25": 2.50, "M3x30": 3.00,
}
SCREW_GRIP_RATIO = 0.65
BEARING_PRESS, BEARING_GLUE, BEARING_RET_LIP = 0.002, 0.015, 0.025

RPI0_L, RPI0_W, RPI0_H = 6.50, 3.00, 0.20
PCA_L, PCA_W, PCA_H = 6.25, 2.54, 0.18
ESP32_L, ESP32_W, ESP32_H = 3.80, 2.58, 0.15
IMU_L, IMU_W, IMU_H = 2.10, 1.60, 0.12
LIPO_L, LIPO_W, LIPO_H, LIPO_SLACK = 7.00, 3.20, 1.80, 0.30
XT60_W, XT60_H_SLOT = 1.60, 1.30
BEC_L, BEC_W, BEC_H = 3.50, 2.20, 0.80
SWITCH_R = 0.60
FUSE_HOLDER_L, FUSE_HOLDER_W = 2.50, 1.20
LED_R_5MM, LED_R_RING = 0.260, 0.600

FING_W, FING_H, FING_GAP = 0.52, 0.48, 0.10
THUMB_W, THUMB_H = 0.65, 0.58
FING_PP, FING_MP, FING_DP = [1.40, 1.60, 1.70, 1.55], [1.00, 1.20, 1.30, 1.15], [0.80, 0.90, 0.95, 0.88]
FING_NAMES = ["Pinky", "Ring", "Middle", "Index"]
FING_X_OFF = [-1.10, -0.37, 0.37, 1.10]
THUMB_PP_L, THUMB_DP_L, PALM_BOTTOM_OFFSET = 1.40, 1.00, 2.50
TENDON_CH_DIAM, TENDON_CH_GROOVE = 0.040, 0.025

SERVO_SPECS = {
    "hip_hd": {"name": "DS3225MG", "rated": 25.0, "stall": 35.0, "mass_g": 60},
    "waist":  {"name": "DS3218",   "rated": 20.0, "stall": 25.0, "mass_g": 55},
    "std":    {"name": "MG996R",   "rated": 9.4,  "stall": 11.5, "mass_g": 55},
    "micro":  {"name": "MG90S",    "rated": 1.8,  "stall": 2.2,  "mass_g": 13},
    "digit":  {"name": "DS04-NFC", "rated": 1.8,  "stall": 2.2,  "mass_g": 9},
}
SERVO_COUPLER = {
    "DS3225MG": {"spline_r": 0.30, "spline_n": 25, "hub_r": 0.70, "hub_h": 0.60, "set_screw": True, "bore_r": 0.16, "flange_od": 1.60},
    "DS3218":   {"spline_r": 0.28, "spline_n": 25, "hub_r": 0.65, "hub_h": 0.55, "set_screw": True, "bore_r": 0.16, "flange_od": 1.50},
    "MG996R":   {"spline_r": 0.25, "spline_n": 25, "hub_r": 0.60, "hub_h": 0.50, "set_screw": True, "bore_r": 0.16, "flange_od": 1.40},
    "MG90S":    {"spline_r": 0.18, "spline_n": 25, "hub_r": 0.45, "hub_h": 0.40, "set_screw": False, "bore_r": 0.12, "flange_od": 1.00},
    "DS04-NFC": {"spline_r": 0.15, "spline_n": 25, "hub_r": 0.40, "hub_h": 0.35, "set_screw": False, "bore_r": 0.10, "flange_od": 0.85},
}
JOINT_LIMITS = {
    "Waist_Cluster": {"pitch": (-45, 60), "yaw": (-15, 15), "roll": (-15, 15)},
    "Neck_Cluster": {"pitch": (-90, 45), "yaw": (-20, 20), "roll": (-20, 20)},
    "L_Hip_Cluster": {"pitch": (-30, 30), "yaw": (-95, 95), "roll": (-30, 30)},
    "R_Hip_Cluster": {"pitch": (-30, 30), "yaw": (-95, 95), "roll": (-30, 30)},
    "L_Knee": {"pitch": (0, 135)}, "R_Knee": {"pitch": (0, 135)},
    "L_Ankle_Cluster": {"pitch": (-20, 20), "yaw": (-30, 95), "roll": (-20, 20)},
    "R_Ankle_Cluster": {"pitch": (-20, 20), "yaw": (-30, 95), "roll": (-20, 20)},
    "L_Shoulder_Cluster": {"pitch": (-175, 60), "yaw": (-90, 90), "roll": (-90, 90)},
    "R_Shoulder_Cluster": {"pitch": (-175, 60), "yaw": (-90, 90), "roll": (-90, 90)},
    "L_Elbow": {"pitch": (0, 150)}, "R_Elbow": {"pitch": (0, 150)},
    "L_Wrist": {"pitch": (0, 90), "roll": (-180, 180)}, "R_Wrist": {"pitch": (0, 135), "roll": (-180, 180)},
    "Blaster_Fold": {"pitch": (-90, 0)},
    "L_Pinky_MCP": {"pitch": (-5, 85)}, "R_Pinky_MCP": {"pitch": (-5, 85)},
    "L_Ring_MCP": {"pitch": (-5, 85)}, "R_Ring_MCP": {"pitch": (-5, 85)},
    "L_Middle_MCP": {"pitch": (-5, 85)}, "R_Middle_MCP": {"pitch": (-5, 85)},
    "L_Index_MCP": {"pitch": (-5, 85)}, "R_Index_MCP": {"pitch": (-5, 85)},
    "L_Thumb_CMC": {"pitch": (-20, 60), "yaw": (-30, 30)}, "R_Thumb_CMC": {"pitch": (-20, 60), "yaw": (-30, 30)},
}

SPLIT_KEYS = {"Shell", "Link", "Main", "Armor", "Core", "Pod", "Palm", "Block", "Sole", "Plate", "Bay", "Collar", "Cover", "Door", "Frame", "Skeleton", "Bracket"}
SKIP_TAGS  = {"Marker", "Pivot", "MtA", "MtB", "Axle_Pivot", "Horn", "Pin", "_Vis", "Boss", "Insert", "Nut", "Snap", "Coupler", "Tendon", "Pulley", "CableClip", "Stop", "AlignJig"}
MERGE_TAGS = {"Pin", "Boss", "Insert", "Nut", "Snap", "Standoff", "Retainer"}
PRINT_NOTES = []

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

# ── Core Classes ─────────────────────────────────────────────────────────────
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
        if cls._count >= 20: cls.flush()

    @classmethod
    def flush(cls):
        if not cls._buffer: return
        try:
            os.makedirs(LOG_DIR, exist_ok=True)
            with open(LOG_FILE, "a", encoding="utf-8") as f: f.write("".join(cls._buffer))
        except Exception: pass
        cls._buffer.clear()
        cls._count = 0

    @classmethod
    def print_note(cls, msg):
        cls.log(msg, "NOTE")
        PRINT_NOTES.append(msg)

class BOM:
    _rows = []
    @classmethod
    def add(cls, category, part_name, qty, note=""): cls._rows.append({"Category": category, "Part": part_name, "Qty": qty, "Note": note})
    @classmethod
    def save_csv(cls, path):
        if not cls._rows: return
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, "w", newline="", encoding="utf-8") as f:
                writer = csv.DictWriter(f, fieldnames=["Category", "Part", "Qty", "Note"])
                writer.writeheader(); writer.writerows(cls._rows)
            Logger.log(f"BOM saved -> {path}")
        except Exception as e: Logger.log(f"BOM save failed: {e}", "WARN")
    @classmethod
    def summary(cls): Logger.log(f"BOM total line items: {len(cls._rows)}  total parts: {sum(r['Qty'] for r in cls._rows)}")
    @classmethod
    def by_category(cls):
        cats = {}
        for r in cls._rows: cats.setdefault(r["Category"], []).append(r)
        return cats

class BuildGuide:
    _entries = []
    @classmethod
    def add(cls, section, content): cls._entries.append((section, content))
    @classmethod
    def generate(cls):
        guide = ["=" * 65, "OPTIMUS PRIME G1 v11 — PHYSICAL BUILD ASSEMBLY GUIDE", f"Generated: {datetime.datetime.now().strftime('%Y-%m-%d %H:%M')}", "=" * 65, "", "--- PRINTED PART SUMMARY ---"]
        for p in BOM.by_category().get("Printed Part", []): guide.append(f"  {p['Qty']}x {p['Part']:35s} {p['Note']}")
        guide.extend(["", "--- PRINT ORIENTATION ---"] + [f"  {n}" for n in PRINT_NOTES] + ["", "--- ASSEMBLY ORDER ---"])
        for section, content in cls._entries:
            guide.append(f"\n  [{section}]")
            for line in content.split("\n"): guide.append(f"    {line.strip()}")
        return "\n".join(guide)
    @classmethod
    def save(cls, path):
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, "w", encoding="utf-8") as f: f.write(cls.generate())
            Logger.log(f"Build guide saved -> {path}")
        except Exception as e: Logger.log(f"Build guide save failed: {e}", "WARN")

class FastenerValidator:
    _warnings = []
    @classmethod
    def clear(cls): cls._warnings = []
    @classmethod
    def check(cls, screw_name, grip_length, nut_depth, material_thickness, location=""):
        if screw_name not in SCREW_LENGTHS: cls._warnings.append(f"Unknown screw: {screw_name} at {location}"); return False
        total_len = SCREW_LENGTHS[screw_name]
        screw_d = float(screw_name.split("x")[0].replace("M", "")) / 10.0
        min_grip = screw_d * SCREW_GRIP_RATIO
        available_grip = total_len - material_thickness - 0.05
        if available_grip < min_grip: cls._warnings.append(f"  [{screw_name} @ {location}] TOO SHORT"); return False
        if available_grip > nut_depth + 0.10: cls._warnings.append(f"  [{screw_name} @ {location}] MAY BOTTOM OUT"); return False
        return True
    @classmethod
    def report(cls):
        for w in cls._warnings: Logger.log(w, "WARN")
        if not cls._warnings: Logger.log("Fastener validation: [OK] All lengths acceptable")

# ── Main Execution ───────────────────────────────────────────────────────────
def run(context):
    global TARGET_MODULE, EXPORT_STL, EXPORT_STEP, EXPORT_URDF, EXPORT_DIR
    app, ui = None, None
    Logger.log("=" * 60 + "\nEXECUTION START  v11.0 — Optimus Prime G1 Physical Build Edition\n" + "=" * 60)
    
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        try: app.preferences.generalPreferences.defaultModelingOrientation = adsk.core.DefaultModelingOrientations.ZUpModelingOrientation
        except Exception: pass
        
        doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
        design = adsk.fusion.Design.cast(app.activeProduct)
        root = design.rootComponent
        
        app_lib = None
        for i in range(app.materialLibraries.count):
            lib = app.materialLibraries.item(i)
            if "Appearance" in lib.name: app_lib = lib; break
            
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

        op_red, op_blue, chrome, dark_metal = get_ap("Paint - Metallic (Red)"), get_ap("Paint - Metallic (Blue)"), get_ap("Chrome"), get_ap("Steel - Flat")
        rubber_blk, glass_clr, grey_plastic, dark_grey = get_ap("Rubber"), get_ap("Glass - Window"), get_ap("Plastic - Matte (Grey)"), get_ap("Plastic - Matte (Dark Grey)")
        white_pla, black_plastic, gold_met, yellow_met = get_ap("Plastic - Glossy (White)"), get_ap("Plastic - Matte (Black)"), get_ap("Gold"), get_ap("Paint - Metallic (Yellow)")
        op_blue_glass, trans_orange = get_ap("Acrylic - Blue Transparent"), get_ap("Acrylic - Orange Transparent")

        comps_list, occs, jig_comps = [], {}, []
        
        def new_component(name):
            occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
            comp = occ.component; comp.name = name
            comps_list.append(comp); occs[name] = occ
            return comp

        def new_jig_component(name):
            comp = new_component(name); jig_comps.append(comp); return comp

        def set_ap(body, ap):
            if body and ap:
                try: body.appearance = ap
                except Exception: pass

        def box(comp, name, cx, cy, cz, lx, ly, lz, ap=None):
            temp = adsk.fusion.TemporaryBRepManager.get()
            obb = adsk.core.OrientedBoundingBox3D.create(adsk.core.Point3D.create(cx, cy, cz), adsk.core.Vector3D.create(1,0,0), adsk.core.Vector3D.create(0,1,0), lx, ly, lz)
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

        def marker(comp, name, cx, cy, cz, size=0.22): return box(comp, name, cx, cy, cz, size, size, size, white_pla)

        def cut_cavity(comp, tool_body, keep_tool=False):
            tools = adsk.core.ObjectCollection.create(); tools.add(tool_body)
            for b in list(comp.bRepBodies):
                if b == tool_body or (b.name and any(t in b.name for t in SKIP_TAGS)): continue
                try:
                    ci = comp.features.combineFeatures.createInput(b, tools)
                    ci.operation = adsk.fusion.CombineOperation.CutFeatureOperation
                    ci.isKeepToolBodies = True
                    comp.features.combineFeatures.add(ci)
                except Exception: pass
            if not keep_tool:
                try:
                    tool_body.isLightBulbOn = False
                    if "_Vis" not in tool_body.name: tool_body.name += "_Vis"
                except Exception: pass

        def merge_bodies(comp, target_body, source_body):
            try:
                tools = adsk.core.ObjectCollection.create(); tools.add(source_body)
                ci = comp.features.combineFeatures.createInput(target_body, tools)
                ci.operation = adsk.fusion.CombineOperation.JoinFeatureOperation
                ci.isKeepToolBodies = False
                comp.features.combineFeatures.add(ci); return target_body
            except Exception:
                try: source_body.isLightBulbOn = False
                except Exception: pass
            return target_body

        def split_halves(comp, body, axis="y", offset=0.0):
            try:
                planes = comp.constructionPlanes; pi = planes.createInput()
                ref = root.xYConstructionPlane if axis == "z" else root.xZConstructionPlane if axis == "y" else root.yZConstructionPlane
                pi.setByOffset(ref, adsk.core.ValueInput.createByReal(offset))
                sp = planes.add(pi)
                split_input = comp.features.splitBodyFeatures.createInput(body, sp, True)
                comp.features.splitBodyFeatures.add(split_input)
            except Exception: pass

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

        def align_pin(comp, tag, cx, cy, cz, axis="z", height=0.40): cyl(comp, f"{tag}_AlignPin", cx, cy, cz, ALIGN_PIN_R, height, axis, chrome)
        def align_socket(comp, tag, cx, cy, cz, axis="z", depth=0.45): cut_cavity(comp, cyl(comp, f"{tag}_AlignSock", cx, cy, cz, ALIGN_SOCK_R, depth, axis))
        def grommet_hole(comp, tag, cx, cy, cz, axis="y"):
            cut_cavity(comp, cyl(comp, f"{tag}_Grommet", cx, cy, cz, GROMMET_R, 0.80, axis))
            BOM.add("Printed Part", "Grommet (TPU print)", 1, comp.name)
        def screw_hole(comp, cx, cy, cz, axis="y", length=3.0):
            cut_cavity(comp, cyl(comp, "ScrewHole", cx, cy, cz, M3_PILOT_R, length, axis))
            BOM.add("Fastener", f"M3x{int(length*10)} self-tap", 1, comp.name)
        def magnet_pocket(comp, tag, cx, cy, cz, axis="z"):
            cut_cavity(comp, cyl(comp, f"{tag}_MagPkt", cx, cy, cz, 0.32, 0.35, axis))
            BOM.add("Hardware", "Magnet D6x3 mm N35", 1, comp.name)
        def wire_channel(comp, tag, cx, cy, cz, r, h, axis): cut_cavity(comp, cyl(comp, f"{tag}_Wire", cx, cy, cz, r, h, axis))
        def led_pocket_5mm(comp, tag, cx, cy, cz, axis="y"):
            cut_cavity(comp, cyl(comp, f"{tag}_LED5", cx, cy, cz, LED_R_5MM, 0.85, axis))
            BOM.add("Electronics", "LED 5 mm (colour TBD)", 1, comp.name)
        def led_ring_pocket(comp, tag, cx, cy, cz, axis="z"):
            cut_cavity(comp, cyl(comp, f"{tag}_LEDRing", cx, cy, cz, LED_R_RING, 0.30, axis))
            BOM.add("Electronics", "WS2812 5050 LED ring 12 mm", 1, comp.name)

        def horn_coupler(comp, tag, cx, cy, cz, axis, servo_type="MG996R"):
            spec = SERVO_COUPLER.get(servo_type, SERVO_COUPLER["MG996R"])
            hr, hh, sr, br = spec["hub_r"], spec["hub_h"], spec["spline_r"], spec["bore_r"]
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
            if cyl_body: merge_bodies(coupler, cyl_body, s1); merge_bodies(coupler, cyl_body, s2)
            ss_r, ss_len = br * 0.9, hr * 1.2
            if axis == "x": ss_body = cyl(coupler, "SetScrewPkt", cx, cy+hr*0.5, cz, ss_r, ss_len, "y")
            elif axis == "y": ss_body = cyl(coupler, "SetScrewPkt", cx+hr*0.5, cy, cz, ss_r, ss_len, "x")
            else: ss_body = cyl(coupler, "SetScrewPkt", cx, cy+hr*0.5, cz, ss_r, ss_len, "y")
            bore_body = cyl(coupler, "Bore", cx, cy, cz, br+0.01, hh+0.10, axis)
            cut_cavity(coupler, bore_body)
            if ss_body: cut_cavity(coupler, ss_body)
            rigid_joint(f"{tag}_CouplerMount", occs[coupler.name], occs.get(comp.name))
            BOM.add("Printed Part", f"Coupler {servo_type} ({tag})", 1, f"PLA/PETG, set-screw M3x4, at ({cx:.1f},{cy:.1f},{cz:.1f})")
            if spec["set_screw"]: BOM.add("Fastener", "M3x4 set screw (cup point)", 1, f"coupler {tag}")
            return coupler

        def tendon_finger_system(comp_hand, tag, fx, fy, finger_z_start, pp_l, mp_l, dp_l, side_sign=1):
            for seg_name, seg_z, seg_l in [("PP", finger_z_start - pp_l/2, pp_l), ("MP", finger_z_start - pp_l - mp_l/2, mp_l), ("DP", finger_z_start - pp_l - mp_l - dp_l/2, dp_l)]:
                if seg_l <= 0: continue
                ch = cyl(comp_hand, f"{tag}_TCh_{seg_name}", fx, fy, seg_z, TENDON_CH_DIAM, seg_l*1.5, "z")
                cut_cavity(comp_hand, ch)
                cyl(comp_hand, f"{tag}_Pulley", fx, fy, finger_z_start + 0.10, 0.15, 0.12, "x", chrome)
            BOM.add("Hardware", "Tendon: 0.4 mm fishing line / Dyneema", 30, "cm per finger")
            BOM.add("Hardware", "Spring return (optional, 0.3 mm wire)", 1, f"finger {tag}")

        def make_access_cover(comp_parent, tag, cx, cy, cz, lx, ly, lz, ap=None, screw_locations=None, hinge_axis=None):
            cover = new_component(f"{tag}_Cover")
            ap = ap or dark_grey
            cover_body = box(cover, f"{tag}_CoverBody", cx, cy, cz, lx, ly, lz, ap)
            cover_body.name = f"{tag}_CoverBody"
            if screw_locations:
                for sc_i, (scx, scy, scz) in enumerate(screw_locations):
                    m3_boss(cover, f"{tag}_CvrBoss_{sc_i}", scx, scy, scz, depth=0.40)
                    screw_hole(cover, scx, scy, scz, axis="z", length=0.60)
            if not screw_locations and not hinge_axis: snap_clip(comp_parent, f"{tag}_DoorClip", cx, cy, cz, span_x=lx*0.6)
            if lx > 2.0 and ly > 2.0:
                for mx in [cx - lx*0.3, cx + lx*0.3]:
                    if abs(mx - cx) > 0.5: magnet_pocket(cover, f"{tag}_Mag_{int(mx*10)}", mx, cy, cz)
            BOM.add("Printed Part", f"Access cover {tag} ({lx*10:.0f}x{ly*10:.0f} mm)", 1, f"PLA/PETG, {comp_parent.name}")
            rigid_joint(f"{tag}_CoverMount", occs.get(cover.name), occs.get(comp_parent.name))
            return cover

        def merge_fasteners_to_halves(comp, body_left, body_right):
            try: left_bbox, right_bbox = body_left.boundingBox, body_right.boundingBox
            except Exception: return
            if not (left_bbox and right_bbox): return
            lc, rc = left_bbox.centerPoint, right_bbox.centerPoint
            for b in list(comp.bRepBodies):
                if not b.name or not any(t in b.name for t in MERGE_TAGS) or b == body_left or b == body_right: continue
                try: bc = b.boundingBox.centerPoint
                except Exception: continue
                target = body_left if bc.distanceTo(lc) < bc.distanceTo(rc) else body_right
                try: merge_bodies(comp, target, b)
                except Exception: pass

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
            if fit_type in ("lip", "press"):
                lip_w, lip_r = BEARING_RET_LIP, ro - 0.02
                lip_p1 = adsk.core.Point3D.create(cx-ax[0]*(half+0.01), cy-ax[1]*(half+0.01), cz-ax[2]*(half+0.01))
                lip_p2 = adsk.core.Point3D.create(cx-ax[0]*(half+0.01-lip_w), cy-ax[1]*(half+0.01-lip_w), cz-ax[2]*(half+0.01-lip_w))
                lip = temp.createCylinderOrCone(lip_p1, lip_r, lip_p2, lip_r)
                bf2 = comp.features.baseFeatures.add(); bf2.startEdit()
                lip_body = comp.bRepBodies.add(lip, bf2); bf2.finishEdit()
                lip_body.name = f"{tag}_Retainer"; cut_cavity(comp, lip_body)
            cyl(comp, f"{tag}_BO", cx, cy, cz, ro, w, axis, chrome)
            cyl(comp, f"{tag}_BI", cx, cy, cz, ro*0.58, w*0.80, axis, dark_grey)
            cyl(comp, f"{tag}_BB", cx, cy, cz, ro*0.32, w*1.10, axis, chrome)
            BOM.add("Bearing", f"Bearing {int(ro*2*10)}x{int(w*10)} mm", 1, f"{fit_type}-fit, {comp.name} at ({cx:.1f},{cy:.1f},{cz:.1f})")

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

        def add_print_orientation_note(part_name, orientation, support_req):
            Logger.print_note(f"PRINT: {part_name} — orient {orientation}, support: {support_req}")

        def alignment_jig(name, cx, cy, cz, lx, ly, lz, pin_positions=None):
            jig = new_jig_component(f"Jig_{name}")
            box(jig, f"Jig_{name}_Base", cx, cy, cz, lx, ly, lz, white_pla)
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
            except Exception: pass
            try:
                sketch = root.sketches.add(root.xYConstructionPlane); sketch.isVisible = False
                s_pt = sketch.sketchPoints.add(adsk.core.Point3D.create(cx, cy, cz))
                return adsk.fusion.JointGeometry.createByPoint(s_pt)
            except Exception: return None

        def revolute_joint(name, occ1, occ2, cx, cy, cz, axis_str):
            if not (occ1 and occ2): return
            try:
                geom = _make_joint_geometry(cx, cy, cz)
                ji = root.asBuiltJoints.createInput(occ1, occ2, geom)
                av = {"x": adsk.core.Vector3D.create(1,0,0), "y": adsk.core.Vector3D.create(0,1,0), "z": adsk.core.Vector3D.create(0,0,1)}[axis_str]
                ji.setAsRevoluteJointMotion(adsk.fusion.JointDirections.CustomJointDirection, av)
                j = root.asBuiltJoints.add(ji); j.name = name
            except Exception: Logger.log(f"Failed revolute joint {name}: {traceback.format_exc()}", "ERROR")

        def ball_joint(name, occ1, occ2, cx, cy, cz):
            if not (occ1 and occ2): return
            try:
                geom = _make_joint_geometry(cx, cy, cz)
                ji = root.asBuiltJoints.createInput(occ1, occ2, geom)
                ji.setAsBallJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection, adsk.fusion.JointDirections.XAxisJointDirection)
                j = root.asBuiltJoints.add(ji); j.name = name
            except Exception: Logger.log(f"Failed ball joint {name}: {traceback.format_exc()}", "ERROR")

        def rigid_joint(name, occ1, occ2):
            if not (occ1 and occ2): return
            try:
                geom = _make_joint_geometry(0, 0, 0)
                ji = root.asBuiltJoints.createInput(occ1, occ2, geom)
                ji.setAsRigidJointMotion()
                j = root.asBuiltJoints.add(ji); j.name = name
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
            cl, dim = CLEARANCE, 2.00
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

        def bearing(comp, tag, cx, cy, cz, axis="x", ro=1.10, w=0.60): bearing_fit(comp, tag, cx, cy, cz, axis, ro, w, fit_type="press")
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

        def metal_frame_channel(comp, name, cx, cy, cz, length, axis="z"):
            ch = box(comp, name, cx, cy, cz, EXTRUSION_W, EXTRUSION_W, length, dark_metal)
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
            for bx in [cx - FRAME_BRACKET_W*0.3, cx + FRAME_BRACKET_W*0.3]: screw_hole(comp, bx, cy, cz, axis=axis, length=FRAME_BRACKET_W)
            BOM.add("Frame", f"L-bracket {name}", 2, "M3x8 + M6 T-nut")
            return b

        def steel_shaft_block(comp, name, cx, cy, cz, axis, shaft_diam, length, bearing_count=2):
            bw2 = max(BEARING_BLOCK_W, shaft_diam + 0.30)
            bl = box(comp, name, cx, cy, cz, bw2, BEARING_BLOCK_H, length, dark_metal)
            bore = cyl(comp, f"{name}_Bore", cx, cy, cz, shaft_diam/2 + CLEARANCE, length + 0.20, axis)
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
            return bore

        def worm_gear_housing(comp, name, cx, cy, cz, axis, ratio=GEAR_RATIO_WAIST):
            hw, hl = 1.80, 2.20
            h = box(comp, name, cx, cy, cz, hw, hl, hl, dark_metal)
            sr = 0.30 + CLEARANCE
            worm_brg = cyl(comp, f"{name}_WormBrg", cx, cy-0.8, cz, sr, hl, "y"); cut_cavity(comp, worm_brg)
            wheel_brg = cyl(comp, f"{name}_WheelBrg", cx, cy+0.2, cz, sr*ratio/10+0.1, hw, "x"); cut_cavity(comp, wheel_brg)
            BOM.add("Gearbox", f"Worm set {ratio}:1 (module 1)", 1, name)
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

        def gearbox_mount(comp, name, cx, cy, cz, axis, servo_type="MG996R"):
            spec = SERVO_COUPLER.get(servo_type, SERVO_COUPLER["MG996R"])
            mp = box(comp, name, cx, cy, cz, spec["flange_od"]+0.40, spec["flange_od"]+0.40, 0.20, dark_metal)
            for mi in range(4):
                mx = cx + ([-1, 1, -1, 1][mi])*1.15; my = cy + ([-1, -1, 1, 1][mi])*0.55
                screw_hole(comp, mx, my, cz, axis="z", length=0.50)
            return mp

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

        def alignment_cam_guide(comp, name, cx, cy, cz, axis="z"):
            b = box(comp, name, cx, cy, cz, ALIGN_CAM_W, ALIGN_CAM_H, ALIGN_CAM_W, chrome)
            for sc_off in [-0.25, 0.25]: screw_hole(comp, cx + sc_off, cy, cz, axis=axis, length=0.50)
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
            nerve = cyl(comp, f"{name}_Nerve", cx, cy, cz, SPINE_NERVE_CH_D/2, SPINE_SEG_H+0.1, "z"); cut_cavity(comp, nerve)
            ball = cyl(comp, f"{name}_Ball", cx, cy, cz-SPINE_SEG_H/2-0.05, SPINE_BALL_D/2, SPINE_SOCKET_D, "z", chrome)
            merge_bodies(comp, v, ball)
            socket = cyl(comp, f"{name}_Socket", cx, cy, cz+SPINE_SEG_H/2+0.05, SPINE_SOCKET_D/2, SPINE_BALL_D, "z"); cut_cavity(comp, socket)
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

        BuildGuide.add("PREAMBLE", "This guide documents assembly of Optimus Prime G1 v11 Physical Build Edition. All dimensions in cm. Read warnings section before starting.")
        
        # ① TORSO
        torso = new_component("OP_Torso")
        BuildGuide.add("TORSO ASSEMBLY", "1. Install M3 heat-set inserts\n2. Install PCA9685\n3. Route harnesses\n4. Install RPi\n5. Wire BEC\n6. Install LiPo\n7. Install fuse/switch\n8. Close covers")
        box(torso, "Torso_Shell", 0, 0, TORSO_CTR, 10.8, 9.0, 12.6, op_red)
        box(torso, "Torso_Side_L", -5.8, 0, TORSO_CTR, 0.55, 8.0, 11.6, op_red)
        box(torso, "Torso_Side_R", 5.8, 0, TORSO_CTR, 0.55, 8.0, 11.6, op_red)
        box(torso, "Skeleton_Main", 0, 0, TORSO_CTR, 6.0, 5.0, 10.0, dark_metal)
        box(torso, "Chest_Plate", 0, -4.30, TORSO_CTR+0.6, 8.8, 0.34, 4.2, chrome)
        cyl(torso, "Badge_Ring", 0, -4.65, TORSO_CTR+0.6, 0.85, 0.14, "y", op_red)
        led_ring_pocket(torso, "Badge", 0, -4.70, TORSO_CTR+0.6, "y")
        metal_frame_channel(torso, "Frame_Spine", 0, 0, TORSO_CTR, 12.0, "z")
        spine_vertebra(torso, "SpineV1", -1.2, 0, PELVIS_CTR+1.0, 0, 3)
        spine_vertebra(torso, "SpineV2", 0, 0, PELVIS_CTR+2.2, 1, 3)
        spine_vertebra(torso, "SpineV3", 1.2, 0, PELVIS_CTR+3.4, 2, 3)
        spine_bellows_cover(torso, "SpineCover", 0, 0, PELVIS_CTR+2.2, 3.0)
        quick_release_coupler(torso, "L_ShoulderQR", -SHOULDER_X, 0, SHOULDER_CTR-1.5, "x", -1)
        quick_release_coupler(torso, "R_ShoulderQR", SHOULDER_X, 0, SHOULDER_CTR-1.5, "x", 1)
        lipo_bay(torso, "Main", 0, 2.8, TORSO_CTR-2.0)
        fuse_pocket(torso, "MainFuse", -2.0, 3.8, TORSO_CTR-2.5)
        cutoff_switch_pocket(torso, "MainCutoff", 2.0, 3.8, TORSO_CTR-2.5)
        bec_bay(torso, "MainBEC", 0, 3.8, TORSO_CTR+0.5)
        rpi_zero_bay(torso, "Main", 0, 3.2, TORSO_CTR+1.8)
        pca9685_bay(torso, "Main", 0, 3.0, TORSO_CTR+4.2)
        make_access_cover(torso, "LipoBay", 0, 4.6, TORSO_CTR-2.2, 7.2, 0.30, 5.4, ap=black_plastic)
        worm_gear_housing(torso, "Waist_Worm", 0, 0, WAIST_CTR, "z", GEAR_RATIO_WAIST)
        steel_shaft_block(torso, "Waist_Shaft", 0, 0, WAIST_CTR, "z", SHAFT_DIAM_MAJOR, 2.0, 2)
        add_mg996r(torso, "Waist_Yaw", 0, -0.8, WAIST_CTR, "z")
        bearing(torso, "Waist_Brg", 0, 0, WAIST_CTR+0.5, "z", 1.30, 0.65)
        add_mg996r(torso, "Waist_Pitch", 0, 0, WAIST_CTR-2.5, "x")
        steel_shaft_block(torso, "NeckP_Shaft", 0, 0, NECK_JOINT_Z, "x", SHAFT_DIAM_MINOR, 2.0, 2)
        add_mg996r(torso, "Neck_Pitch", 0, 0, NECK_JOINT_Z, "x")

        # ② HEAD
        head = new_component("OP_Head"); HC = HEAD_CTR
        BuildGuide.add("HEAD ASSEMBLY", "1. Install ESP32-CAM\n2. Route lens\n3. Install LEDs\n4. Install neck yaw\n5. Close shell")
        box(head, "Helm_Main", 0, 0, HC+0.8, 5.8, 5.4, 5.2, op_blue)
        box(head, "Visor", 0, -2.72, HC+1.35, 3.2, 0.16, 0.95, op_blue_glass)
        for vx in [-0.85, 0.0, 0.85]: led_pocket_5mm(head, f"Visor_LED_{int(vx*100)}", vx, -2.85, HC+1.35, "y")
        esp32_cam_pocket(head, "HeadCam", 0, -1.6, HC+2.5, "y")
        add_mg90s(head, "Neck_Yaw", 0, 0, NECK_JOINT_Z, "z")
        steel_shaft_block(head, "NeckYaw_Shaft", 0, 0, NECK_JOINT_Z, "z", SHAFT_DIAM_MINOR, 1.5, 2)

        # ③ PELVIS
        pelvis = new_component("OP_Pelvis")
        BuildGuide.add("PELVIS ASSEMBLY", "1. Install IMU\n2. Install hip yaw servos\n3. Attach couplers\n4. Close shell")
        box(pelvis, "Pelvis_Shell", 0, 0, PELVIS_CTR, 16.8, 6.6, 5.4, op_blue)
        metal_frame_channel(pelvis, "Pelvis_FrameChan", 0, 0, PELVIS_CTR, 8.0, "y")
        steel_shaft_block(pelvis, "L_HipYaw_Shaft", -HIP_X-2.2, 0, HIP_JOINT_Z, "z", SHAFT_DIAM_MAJOR, 2.0, 2)
        steel_shaft_block(pelvis, "R_HipYaw_Shaft", HIP_X+2.2, 0, HIP_JOINT_Z, "z", SHAFT_DIAM_MAJOR, 2.0, 2)
        flanged_bearing_cavity(pelvis, "L_HY_FlangeBrg", -HIP_X-2.2, 0, HIP_JOINT_Z, "z", FLANGE_BRG_MAJOR_ID, FLANGE_BRG_MAJOR_OD, FLANGE_BRG_MAJOR_W)
        flanged_bearing_cavity(pelvis, "R_HY_FlangeBrg", HIP_X+2.2, 0, HIP_JOINT_Z, "z", FLANGE_BRG_MAJOR_ID, FLANGE_BRG_MAJOR_OD, FLANGE_BRG_MAJOR_W)
        quick_release_coupler(pelvis, "L_LegQR", -HIP_X, 0, HIP_JOINT_Z, "z", -1)
        quick_release_coupler(pelvis, "R_LegQR", HIP_X, 0, HIP_JOINT_Z, "z", 1)
        imu_pocket(pelvis, "BalanceIMU", 0, 0, PELVIS_CTR)
        add_mg996r(pelvis, "L_HipYaw", -HIP_X, 0, HIP_JOINT_Z, "z")
        add_mg996r(pelvis, "R_HipYaw", HIP_X, 0, HIP_JOINT_Z, "z")
        bearing(pelvis, "L_HYB", -HIP_X-2.2, 0, HIP_JOINT_Z, "z", 1.10, 0.62)
        bearing(pelvis, "R_HYB", HIP_X+2.2, 0, HIP_JOINT_Z, "z", 1.10, 0.62)

        # ④ LEGS
        BuildGuide.add("LEG ASSEMBLY", "1. Install hip pitch\n2. Install knee\n3. Route wires\n4. Close shells\n5. Install wheels\n6. Install ankle")
        for side, sx in [("L", -HIP_X), ("R", HIP_X)]:
            m = -1 if side == "L" else 1
            thigh = new_component(f"OP_Thigh_{side}")
            box(thigh, "Thigh_Link", sx, 0, THIGH_CTR, 5.2, 4.2, 11.2, chrome)
            planetary_gear_stage(thigh, f"{side}_HipPlanet", sx, 0, HIP_JOINT_Z, "x", GEAR_RATIO_HIP)
            steel_shaft_block(thigh, f"{side}_HipP_Shaft", sx, 0, HIP_JOINT_Z, "x", SHAFT_DIAM_MAJOR, 2.5, 2)
            add_mg996r(thigh, f"{side}_HipP", sx, 0, HIP_JOINT_Z, "x")
            add_mg996r(thigh, f"{side}_KneP", sx, 0, KNEE_CTR+1.5, "x")
            bearing(thigh, f"{side}_KB", sx, 0, KNEE_CTR, "x", 1.00, 0.55)
            
            shin = new_component(f"OP_Shin_{side}")
            box(shin, "Shin_Link", sx, 0, SHIN_CTR, 4.6, 6.2, 11.2, op_blue)
            steel_shaft_block(shin, f"{side}_Knee_ShaftL", sx, 0, KNEE_CTR-0.5, "x", SHAFT_DIAM_MAJOR, 2.0, 2)
            tt_wheel(shin, f"{side}_WF", sx+m*2.0, -4.3, SHIN_CTR+4.0, m)
            tt_wheel(shin, f"{side}_WR", sx+m*2.0, -4.3, SHIN_CTR-4.0, m)
            
            foot = new_component(f"OP_Foot_{side}")
            box(foot, "Foot_Sole", sx, -0.4, ANKLE_CTR-1.5, 6.6, 9.6, 1.20, op_red)
            steel_shaft_block(foot, f"{side}_Ankle_Shaft", sx, 0, ANKLE_CTR, "x", SHAFT_DIAM_MED, 2.0, 2)
            add_mg996r(foot, f"{side}_AnkP", sx, 0, ANKLE_CTR+2.2, "x")
            bearing(foot, f"{side}_AB", sx, 0, ANKLE_CTR, "x", 1.00, 0.55)

        # ⑤ ARMS
        BuildGuide.add("ARM ASSEMBLY", "1. Install shoulder servos\n2. Install elbow\n3. Route wires\n4. Close shells\n5. Install wrist\n6. Install fingers")
        for side, ax in [("L", -SHOULDER_X), ("R", SHOULDER_X)]:
            m = -1 if side == "L" else 1
            ua = new_component(f"OP_UpperArm_{side}")
            box(ua, "Sh_Block", ax, 0, SHOULDER_CTR, 5.8, 4.4, 5.8, op_red)
            steel_shaft_block(ua, f"{side}_ShY_Shaft", ax, 0, SHOULDER_CTR+1.5, "z", SHAFT_DIAM_MED, 2.0, 2)
            steel_shaft_block(ua, f"{side}_ShP_Shaft", ax, 0, SHOULDER_CTR, "x", SHAFT_DIAM_MED, 2.5, 2)
            add_mg996r(ua, f"{side}_ShY", ax, 0, SHOULDER_CTR+1.5, "z")
            add_mg996r(ua, f"{side}_ShP", ax, 0, SHOULDER_CTR, "x")
            add_mg996r(ua, f"{side}_ElbP", ax, 0, ELBOW_Z, "x")
            bearing(ua, f"{side}_EBr", ax, 0, ELBOW_Z-0.5, "x", 0.95, 0.52)
            
            fa = new_component(f"OP_Forearm_{side}")
            box(fa, "FA_Link", ax, 0, WRIST_Z+3.5, 3.4, 4.0, 5.0, op_blue)
            steel_shaft_block(fa, f"{side}_Wrist_Shaft", ax, 0, WRIST_Z+0.8, "x", SHAFT_DIAM_MINOR, 1.5, 2)
            add_mg90s(fa, f"{side}_WR", ax, 0, WRIST_Z+0.8, "x")
            
            hand = new_component(f"OP_Hand_{side}")
            box(hand, "Palm_Main", ax, -0.4, WRIST_Z-1.6, 3.6, 3.2, 2.4, dark_grey)
            add_ds04_nfc(hand, f"{side}_FingerDrv", ax, 0.5, WRIST_Z-2.0, "z")
            MCP_Z = WRIST_Z - PALM_BOTTOM_OFFSET
            for fi, (fname, fxo, pp_l, mp_l, dp_l) in enumerate(zip(FING_NAMES, FING_X_OFF, FING_PP, FING_MP, FING_DP)):
                fc = new_component(f"OP_{fname}_{side}")
                fx = ax + m * fxo; fy = -1.35
                box(fc, "PP", fx, fy, MCP_Z - pp_l/2, FING_W, FING_H, pp_l, grey_plastic)
                box(fc, "MP", fx, fy, MCP_Z - pp_l - mp_l/2, FING_W*0.94, FING_H*0.94, mp_l, grey_plastic)
                box(fc, "DP", fx, fy, MCP_Z - pp_l - mp_l - dp_l/2, FING_W*0.86, FING_H*0.86, dp_l, grey_plastic)
                tendon_finger_system(fc, f"{side}_{fname}", fx, fy, MCP_Z, pp_l, mp_l, dp_l, m)
            
            thumb = new_component(f"OP_Thumb_{side}")
            tx = ax + m * 1.70; ty = 0.20
            box(thumb, "TP", tx, ty, MCP_Z - THUMB_PP_L/2, THUMB_W, THUMB_H, THUMB_PP_L, grey_plastic)
            box(thumb, "TD", tx, ty*0.8, MCP_Z - THUMB_PP_L - THUMB_DP_L/2, THUMB_W*0.86, THUMB_H*0.86, THUMB_DP_L, grey_plastic)
            tendon_finger_system(thumb, f"{side}_Thumb", tx, ty, MCP_Z, THUMB_PP_L, THUMB_DP_L, 0, m)

        # ⑥ BACKPACK & MISC
        bp = new_component("OP_Backpack")
        box(bp, "BP_Core", 0, 5.8, TORSO_CTR+0.5, 7.4, 2.8, 9.4, dark_grey)
        
        steer = new_component("OP_SteerPods")
        for side2, sx2 in [("L", -5.8), ("R", 5.8)]:
            m2 = -1 if side2 == "L" else 1
            tt_wheel(steer, f"SW_{side2}", sx2+m2*2.0, -4.3, 23.4, m2)
            add_mg90s(steer, f"SSrv_{side2}", sx2, -4.3, 23.9, "z")

        # Shell Splitting
        for comp in comps_list:
            if comp in jig_comps: continue
            to_split = [b for b in comp.bRepBodies if b.name and any(k in b.name for k in SPLIT_KEYS)]
            for b in to_split: split_halves(comp, b, "y", 0.0)
        for comp in comps_list:
            if comp in jig_comps: continue
            split_bodies = [b for b in comp.bRepBodies if b.name and "y_Split" in b.name]
            i = 0
            while i < len(split_bodies) - 1:
                merge_fasteners_to_halves(comp, split_bodies[i], split_bodies[i+1])
                i += 2

        # Kinematics
        t = occs.get("OP_Torso"); p = occs.get("OP_Pelvis"); h = occs.get("OP_Head")
        b = occs.get("OP_Backpack"); st = occs.get("OP_SteerPods")
        if p: p.isGrounded = True
        ball_joint("Waist_Cluster", t, p, 0, 0, WAIST_CTR-2.5)
        ball_joint("Neck_Cluster", h, t, 0, 0, NECK_JOINT_Z)
        rigid_joint("Backpack_Mount", b, t)
        rigid_joint("Steer_Mount", st, p)
        
        for side in ["L", "R"]:
            sx = -HIP_X if side == "L" else HIP_X
            ax = -SHOULDER_X if side == "L" else SHOULDER_X
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
                fx = ax + (-1 if side == "L" else 1) * FING_X_OFF[fi]
                fo_f = occs.get(f"OP_{fname}_{side}")
                revolute_joint(f"{side}_{fname}_MCP", fo_f, ha, fx, -1.35, MCP_Z, "x")
            tx = ax + (-1 if side == "L" else 1) * 1.70
            thumb_occ = occs.get(f"OP_Thumb_{side}")
            ball_joint(f"{side}_Thumb_CMC", thumb_occ, ha, tx, 0.20, MCP_Z)

        # Fastener Validation
        FastenerValidator.clear()
        FastenerValidator.check("M3x8", 0.8, 0.5, 0.20, "torso shell main")
        FastenerValidator.check("M3x10", 1.0, 0.5, 0.30, "torso bracket")
        FastenerValidator.report()

        # Simulation Engine
        class SimulationEngine:
            BALL_JOINTS = {"Waist_Cluster", "Neck_Cluster", "L_Hip_Cluster", "R_Hip_Cluster", "L_Ankle_Cluster", "R_Ankle_Cluster", "L_Shoulder_Cluster", "R_Shoulder_Cluster", "L_Wrist", "R_Wrist", "L_Thumb_CMC", "R_Thumb_CMC"}
            REV_JOINTS = {"L_Knee", "R_Knee", "L_Elbow", "R_Elbow", "Blaster_Fold", "L_Pinky_MCP", "L_Ring_MCP", "L_Middle_MCP", "L_Index_MCP", "R_Pinky_MCP", "R_Ring_MCP", "R_Middle_MCP", "R_Index_MCP"}

            def __init__(self, root, comps_list, design, app, ui):
                self._root, self._comps, self._design, self._app, self._ui = root, comps_list, design, app, ui
                self._cols, self._logged_joints = [], False

            def _gj(self, name):
                try:
                    j = self._root.asBuiltJoints.itemByName(name)
                    if j is not None: return j
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
                    if deg < lo or deg > hi: return max(lo, min(hi, deg))
                return deg

            def move_joint(self, name, deg, steps=20, axis="pitch", ease=True, clamp=True):
                j = self._gj(name)
                if not j: return
                if clamp: deg = self._clamp(name, axis, deg)
                mo = j.jointMotion; e_rad = math.radians(deg); s_rad = self._get(mo, axis)
                for i in range(1, steps + 1):
                    t = self._ease(i/steps) if ease else i/steps
                    self._set(mo, axis, s_rad + (e_rad - s_rad) * t); self._refresh()

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
                self._refresh()

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
                    self._cols.append((label, count))
                except Exception as e: self._cols.append((label, -1))

            def _check_zmp(self, label):
                FOOT_HW, FOOT_HL = 6.6 / 2.0, 9.6 / 2.0
                try: com = self._root.physicalProperties.centerOfMass
                except: return False
                l_cx, r_cx = -HIP_X, HIP_X
                in_DS = (min(l_cx - FOOT_HW, r_cx - FOOT_HW) <= com.x <= max(l_cx + FOOT_HW, r_cx + FOOT_HW) and -FOOT_HL <= com.y <= FOOT_HL)
                return in_DS

            def test_joint_rom(self):
                self.reset_all(steps=10)
                for name, axes in JOINT_LIMITS.items():
                    for axis, (lo, hi) in axes.items():
                        self.move_joint(name, hi, steps=10, axis=axis)
                        self._interfere(f"MAX {name} {axis}")
                        self.move_joint(name, 0, steps=10, axis=axis)

            def simulate_walking_advanced(self):
                self.reset_all(steps=10)
                for cycle in range(2):
                    phase = cycle % 2; l_sign = 1 if phase == 0 else -1; r_sign = -1 if phase == 0 else 1
                    self.move_ball([("L_Hip_Cluster",25*l_sign,10*l_sign,5*l_sign),("R_Hip_Cluster",25*r_sign,10*r_sign,5*r_sign),("L_Knee",60,None,None),("R_Knee",60,None,None)], steps=15)
                    self._check_zmp(f"Walk cycle {cycle+1}")
                self.reset_all(steps=10)

            def estimate_servo_loads(self):
                loads = [("Neck Pitch",120,3.0,SERVO_SPECS["micro"]),("L Shoulder Pitch",390,12.0,SERVO_SPECS["std"]),("L Hip Pitch",820,15.0,SERVO_SPECS["hip_hd"])]
                for label, mass_g, lever_cm, spec in loads:
                    F = (mass_g/1000.0)*9.81; need = (F*lever_cm/100.0)/0.0981
                    margin = spec["rated"]/need if need > 0 else 99.0
                    status = "OK" if margin >= 1.5 else ("MARGINAL" if margin >= 0.9 else "OVERLOAD")
                    Logger.log(f"  {label:<24} need {need:5.2f} kg.cm  rated {spec['rated']:5.1f}  margin {margin:.2f}x  [{status}]")

            def export_bom(self):
                BOM.add("Fastener", "M3x8 SHCS (general assembly)", 80, "assembly")
                BOM.add("Hardware", "Steel shaft D5mm x 25mm", 8, "joints")
                BOM.add("Hardware", "Flanged bearing MR85-2RS", 8, "bearings")
                BOM.save_csv(BOM_FILE); BOM.summary()

            def run_all_simulations(self):
                Logger.log("--- BEGINNING SIMULATION ---")
                self.test_joint_rom()
                self.simulate_walking_advanced()
                self.estimate_servo_loads()
                self.export_bom()
                Logger.log("--- SIMULATION COMPLETE ---")

        sim = SimulationEngine(root, comps_list, design, app, ui)
        sim.run_all_simulations()
        
        Logger.log("v11 script finished successfully.")
    except Exception:
        Logger.log(f"FATAL ERROR:\n{traceback.format_exc()}", "ERROR")
    finally:
        Logger.flush()
        Logger.log("--- v11 END ---")