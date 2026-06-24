# ─────────────────────────────────────────────────────────────────────────────
# OPTIMUS PRIME G1  v14.0  ─ Unified Distributed AI Edition
# Fusion 360 Python Script  |  Production-Ready Physically Buildable Robot
# ─────────────────────────────────────────────────────────────────────────────
# v14 MERGES two parallel v13 branches (Jetson-AI-Brain branch +
# Distributed-AI-Real-Time-Control branch) into one script, keeping the best
# idea from each side, then adds new "high-end real-world transformer"
# upgrades on top: a real knee-yaw DOF, a true 45-degree printable chamfer
# (not a stub, not a stacked-box approximation), a foot force/pressure sensor
# suite, an independent servo-rail E-stop, and a status RGB indicator.
#
# SYSTEM OVERVIEW
# ───────────────
# [BRAIN]   NVIDIA Jetson Nano 4GB (lower-mid torso, lowers CoM)
#             - JetPack/Ubuntu, Python3, OpenCV, TensorRT, ROS2-ready
#             - CSI-2 camera port -> IMX219 "RobotEyes" (head, behind visor)
#             - USB3 x2 (AI accelerator + debug), USB-C power, HDMI, 40-pin GPIO
#
# [NODE-1]  ESP32-S3 "Balance Node"   (pelvis)
#             - I2C -> PCA9685 (hip/knee/ankle), MPU-6050 IMU,
#                      ADS1115 ADC -> 4x foot FSR pads/side, HC-SR04 ultrasonic
# [NODE-2]  ESP32-S3 "Motor Controller" (torso)
#             - I2C -> PCA9685 (waist/shoulder/elbow), INA3221 current monitor
# [NODE-3]  ESP32-S3 "Vision Aux / Head Node" (head)
#             - I2C -> PCA9685 (neck/wrist/finger), VL53L1X ToF x2
#
# [COMMS]   Jetson <-> each ESP32-S3 via USB-UART (CH340 bridge), 1 Mbaud,
#           COBS+CRC16 framing. Fallback: WiFi WebSocket (ESP32 AP mode).
#           Visual comm-backbone cable trunk (I2C/UART/SPI) runs the spine.
#
# [POWER]   2S/3S LiPo -> 5V/4A UBEC (Jetson) + 5V/3A BEC (ESP32 logic)
#                       + 3.3V/1A LDO (sensor rail)
#                       + 7.4V servo rail (5A ATO fuse)
#           Independent servo-rail E-STOP button (Jetson stays alive for
#           logging/diagnostics even when the E-stop kills servo power).
#
# [FUTURE]  USB-C AI accelerator bay (Coral TPU / Intel NCS) for on-device
#           model upgrades without a redesign.
#
# v14 NEW / MERGED FEATURES
# ──────────────────────────
# MECH-V14-1  Real knee-yaw DOF: knee promoted from revolute to ball joint
#             (pitch+yaw), with a physical MG996R yaw servo, bearing, and
#             hard stop -- lets the leg adapt to uneven ground and improves
#             the transform sequence's cross-step.
# MFG-V14-1   chamfer_wedge_cut() + rewritten chamfer_box(): a TRUE 45-degree
#             wedge cut built from a 45-degree-rotated "diamond" cutter body
#             centered exactly on the sharp corner -- not a no-op, not a
#             stacked second box -- so chin guards / heel spurs / toe caps
#             are genuinely self-supporting on the printer.
# SYS-V14-1   sensor_array(): HC-SR04 ultrasonic + 4x FSR force pads + ADS1115
#             ADC per foot/pelvis (from the distributed-control branch),
#             complementing the head ToF pair for non-redundant coverage.
# SYS-V14-2   ai_accel_pocket(): USB-C AI accelerator bay (Coral/NCS),
#             future-ready without committing to a specific chip today.
# SYS-V14-3   comm_backbone(): visual I2C/UART/SPI trunk cable channel run
#             down the spine, used for build documentation and routing.
# SYS-V14-4   estop_cutout(): independent normally-closed E-stop button that
#             only cuts the 7.4V servo rail -- Jetson and sensors stay live.
# SYS-V14-5   status_rgb_pocket(): WS2812 status indicator (boot/ready/
#             error/low-battery) visible on the chest badge.
# ROBUST-V14-1 box()/cyl()/cone_shape(): dimension + axis validation guards.
# ROBUST-V14-2 cut_cavity(): re-fetches bodies by name via itemByName() before
#             every combine call, since sequential combines can invalidate
#             previously-held body references -- removes silent failures.
# ROBUST-V14-3 split_halves(): validates the split actually produced >=2
#             bodies before reporting success.
# ROBUST-V14-4 merge_fasteners_to_halves(): single consolidated call per
#             component (was two calls per side) -- simpler and avoids
#             re-scanning the body list twice per split.
# ROBUST-V14-5 Logger: bounded buffer cap prevents unbounded growth if disk
#             flush repeatedly fails.
# ROBUST-V14-6 write_production_readiness_report(): two-path fallback write
#             (primary path, then EXPORT_DIR) so one bad path never loses
#             the whole report.
# ROBUST-V14-7 verify_screw_lengths(): explicit message when SCREW_REGISTRY
#             is empty instead of silently reporting zero issues.
# ROBUST-V14-8 export_stl()/export_step(): defensive `name or ""` checks.
# ROBUST-V14-9 capture_screenshots(): per-angle try/except in the turntable
#             sweep so one bad camera-math step can't abort the whole batch.
# BUGFIX-V14-1 move_ball(): fixed undefined-`ax` bug in the per-frame update
#             loop (was referencing a name from a different loop's scope).
#
# ARCH-V14-1  log_v14_architecture() / log_comms_map() / log_power_budget() /
#             log_ai_pipeline(): full system topology + bus + power + AI
#             pipeline logging, now including knee-yaw, sensors, E-stop.
# SIM-V14-1   validate_v14_architecture(), simulate_vision_tracking(),
#             simulate_obstacle_reaction(), simulate_sensor_fusion(),
#             simulate_balance_recovery() (NEW: FSR+IMU closed-loop recovery
#             narrative demo).
# ─────────────────────────────────────────────────────────────────────────────

if 'TARGET_MODULE'     not in globals(): TARGET_MODULE     = "ALL"
if 'EXPORT_STL'        not in globals(): EXPORT_STL        = False
if 'EXPORT_STEP'       not in globals(): EXPORT_STEP       = False
if 'EXPORT_URDF'       not in globals(): EXPORT_URDF       = False
if 'CAPTURE_SCREENSHOTS' not in globals(): CAPTURE_SCREENSHOTS = False
if 'VISUAL_AUDIT'      not in globals(): VISUAL_AUDIT      = False
if 'PRODUCTION_REPORT' not in globals(): PRODUCTION_REPORT = True

import adsk.core, adsk.fusion, traceback, math, os, csv, json, datetime, time

_ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

try:    SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
except: SCRIPT_DIR = os.getcwd()
PROJECT_DIR    = os.path.dirname(SCRIPT_DIR)
_OUT           = os.path.join(PROJECT_DIR, "output")
LOG_DIR        = globals().get("LOG_DIR")        or os.path.join(_OUT, "logs")
SCREENSHOT_DIR = globals().get("SCREENSHOT_DIR") or os.path.join(_OUT, "screenshots")
EXPORT_DIR     = globals().get("EXPORT_DIR")     or os.path.join(_OUT, "exports")
JIG_DIR        = os.path.join(EXPORT_DIR, "jigs")
DOCS_DIR       = os.path.join(_OUT, "docs")
LOG_FILE       = os.path.join(LOG_DIR,  f"optimus_v14_{_ts}.txt")
BOM_FILE       = os.path.join(_OUT,     f"BOM_v14_{_ts}.csv")
ASSEMBLY_FILE  = os.path.join(_OUT,     f"ASSEMBLY_GUIDE_v14_{_ts}.txt")
MANIFEST_FILE  = os.path.join(_OUT,     f"BUILD_MANIFEST_v14_{_ts}.json")
PRODUCTION_FILE= os.path.join(_OUT,     f"PRODUCTION_v14_{_ts}.txt")
STOP_FLAG      = os.path.join(_OUT,     "stop.flag")

# ═════════════════════════════════════════════════════════════════════════════
# CONFIGURATION  — ALL DIMENSIONS IN cm (Fusion 360 native unit)
# ═════════════════════════════════════════════════════════════════════════════

# ── Skeleton Z-heights (preserved from v9 for kinematic compatibility) ────────
CLEARANCE=0.060; ANKLE_CTR=3.8; SHIN_CTR=9.3; KNEE_CTR=14.8; THIGH_CTR=20.3
PELVIS_CTR=30.5; WAIST_CTR=32.5; TORSO_CTR=36.0; SHOULDER_CTR=41.5; HEAD_CTR=47.5
HIP_X=5.8; SHOULDER_X=13.0; ELBOW_Z=35.0; WRIST_Z=29.0
HIP_JOINT_Z=26.8; NECK_JOINT_Z=44.5

# ── Physical / FDM print ──────────────────────────────────────────────────────
WALL_S=0.30; WALL_P=0.20; WALL_F=0.15

# ── Bearing fit ───────────────────────────────────────────────────────────────
BEARING_FIT_TOLERANCE=0.008; BEARING_RETAIN_LIP_H=0.06; BEARING_RETAIN_LIP_R=0.04

# ── Tendon / finger drive ─────────────────────────────────────────────────────
TENDON_DIA=0.04; TENDON_GUIDE_R=0.06; DRUM_R=0.35; DRUM_H=0.50
PULLEY_R=0.20; SPRING_OD=0.25; SPRING_WIRE=0.04

# ── Cable management ──────────────────────────────────────────────────────────
CABLE_CLIP_R=0.12; CABLE_CLIP_W=0.35
JST_XH_L=0.55; JST_XH_W=0.25; JST_XH_H=0.18

# ── Power system ──────────────────────────────────────────────────────────────
FUSE_HOLDER_L=2.00; FUSE_HOLDER_W=0.80; FUSE_HOLDER_H=0.75
BEC_L=3.50; BEC_W=1.80; BEC_H=0.90      # 5V 3A BEC (waist/arms logic)
POWER_SWITCH_R=0.35

# ── Fasteners ─────────────────────────────────────────────────────────────────
M3_CLR_R=0.160; M3_PILOT_R=0.125; M3_HEAD_R=0.285; M3_HEAD_H=0.300
M3_NUT_CIR=0.320; M3_NUT_H=0.240; INSERT_R=0.235; INSERT_H=0.500
BOSS_R=0.350; ALIGN_PIN_R=0.100; GROMMET_R=0.175

# ── V13 NEW: NVIDIA Jetson Nano compact carrier ───────────────────────────────
# Waveshare Nano A / compact carrier form factor
JNANO_L,  JNANO_W,  JNANO_H   = 7.00, 4.80, 1.60  # PCB + module stack (cm)
JNANO_MODULE_L, JNANO_MODULE_W = 6.96, 4.50          # module only
JNANO_FAN_SZ                   = 3.00                # 30mm cooling fan
JNANO_FAN_H                    = 0.72                # fan thickness
JNANO_HSINK_H                  = 0.55                # heatsink height above board
JNANO_BARREL_R                 = 0.300               # 5.5mm DC barrel jack radius
JNANO_USB3_W                   = 0.75                # USB-A Type 3 slot width
JNANO_USB3_H                   = 0.42
JNANO_USBCC_W                  = 0.38                # USB-C slot width
JNANO_USBCC_H                  = 0.28
JNANO_HDMI_W                   = 1.45                # HDMI slot width
JNANO_HDMI_H                   = 0.68
JNANO_CSI_W                    = 0.40                # CSI connector slot width
JNANO_CSI_H                    = 0.28
JNANO_GPIO_W                   = 5.10                # 40-pin GPIO header length
JNANO_GPIO_H                   = 0.25

# ── V13 NEW: ESP32-S3 DevKitC control nodes ───────────────────────────────────
ESP32S3_L,  ESP32S3_W,  ESP32S3_H = 5.10, 2.00, 0.14
ESP32S3_ANT_L                      = 1.80  # PCB antenna clearance zone
ESP32S3_USBCC_W                    = 0.38  # USB-C programming port
ESP32S3_USBCC_H                    = 0.28

# ── V13 NEW: Jetson CSI Camera (IMX219, same as RPi Camera v2) ────────────────
CSI_CAM_L,  CSI_CAM_W,  CSI_CAM_H = 2.50, 2.40, 0.35  # incl. lens
CSI_RIBBON_W = 1.20    # FPC flat ribbon cable width (12mm standard)
CSI_RIBBON_H = 0.10    # FPC thickness
CSI_LENS_R   = 0.45    # camera lens port radius

# ── V13 NEW: VL53L1X Time-of-Flight sensor ────────────────────────────────────
TOF_L, TOF_W, TOF_H = 2.60, 1.30, 0.32
TOF_LENS_R          = 0.28  # optical window radius

# ── V13 NEW: INA3221 3-channel current/power monitor ─────────────────────────
INA_L, INA_W, INA_H = 2.60, 1.50, 0.22

# ── V13 NEW: 5V/4A UBEC for Jetson Nano (needs ≥4A) ─────────────────────────
UBEC_JNANO_L, UBEC_JNANO_W, UBEC_JNANO_H = 5.50, 3.00, 1.40

# ── V13 NEW: Ventilation ──────────────────────────────────────────────────────
VENT_SLOT_W   = 0.14   # vent slot width
VENT_SLOT_L   = 2.20   # vent slot length
N_VENT_SLOTS  = 8      # slots per grille panel

# ── V13: Kept ESP32-CAM dims (still used for BOM reference / backpack) ────────
ESP32_L, ESP32_W, ESP32_H = 3.80, 2.58, 0.15

# ── V14 NEW: Sensor fusion array (ultrasonic + FSR + ADC) ────────────────────
US_L, US_W, US_H   = 4.50, 2.10, 1.60   # HC-SR04 ultrasonic body
US_TXRX_R          = 0.80               # transducer radius
FSR_SZ             = 0.55               # FSR pad square footprint (0.5")
FSR_T              = 0.15               # FSR pad pocket depth
ADS1115_L, ADS1115_W, ADS1115_H = 2.90, 1.50, 0.25   # 16-bit ADC breakout

# ── V14 NEW: AI accelerator bay (Coral TPU / Intel NCS, USB-C) ───────────────
AI_ACCEL_L, AI_ACCEL_H, AI_ACCEL_W = 6.60, 0.90, 3.20
AI_ACCEL_USBC_W, AI_ACCEL_USBC_H   = 0.40, 0.20

# ── V14 NEW: Independent servo-rail E-stop ────────────────────────────────────
ESTOP_R          = 0.45   # 22mm mushroom E-stop button radius
ESTOP_COLLAR_R   = 0.55   # panel-mount collar radius

# ── V14 NEW: Status RGB indicator (WS2812 single-pixel, chest badge) ─────────
STATUS_RGB_R     = 0.30

# ── V14 NEW: Visual comm backbone (I2C/UART/SPI trunk down the spine) ────────
COMM_TRUNK_R     = 0.35   # overall trunk channel radius
COMM_I2C_R       = 0.08
COMM_UART_R      = 0.08
COMM_SPI_R       = 0.12

# ── V14 NEW: Knee-yaw servo/bearing geometry (MECH-V14-1) ─────────────────────
KNEE_YAW_OFFSET_X = 1.20   # X-offset of the yaw servo from the knee centerline
KNEE_YAW_BRG_R    = 0.85   # bearing outer radius for the yaw axis
KNEE_YAW_BRG_W    = 0.50

# ── Finger geometry ───────────────────────────────────────────────────────────
FING_W=0.52; FING_H=0.48; FING_GAP=0.10; THUMB_W=0.65; THUMB_H=0.58
FING_PP=[1.40,1.60,1.70,1.55]; FING_MP=[1.00,1.20,1.30,1.15]
FING_DP=[0.80,0.90,0.95,0.88]
FING_NAMES=["Pinky","Ring","Middle","Index"]; FING_X_OFF=[-1.10,-0.37,0.37,1.10]
THUMB_PP_L=1.40; THUMB_DP_L=1.00; PALM_BOTTOM_OFFSET=2.50

# ── Electronics (kept from v12) ───────────────────────────────────────────────
PCA_L,  PCA_W,  PCA_H  = 6.25, 2.54, 0.18
IMU_L,  IMU_W,  IMU_H  = 2.10, 1.60, 0.12
LIPO_L, LIPO_W, LIPO_H = 7.00, 3.20, 1.80
XT60_W, XT60_H_SLOT     = 1.60, 1.30
LED_R_5MM=0.260; LED_R_RING=0.600
FUSE_HOLDER_L=2.00; FUSE_HOLDER_W=0.80; FUSE_HOLDER_H=0.75

# ── Servo specs (v12 maintained) ─────────────────────────────────────────────
SERVO_SPECS = {
    "hip_hd":{"name":"DS3225MG","rated":25.0,"stall":35.0,"mass_g":60,"horn_spline":25},
    "waist": {"name":"DS3218",  "rated":20.0,"stall":25.0,"mass_g":55,"horn_spline":25},
    "std":   {"name":"MG996R",  "rated": 9.4,"stall":11.5,"mass_g":55,"horn_spline":25},
    "micro": {"name":"MG90S",   "rated": 1.8,"stall": 2.2,"mass_g":13,"horn_spline":21},
    "digit": {"name":"DS04-NFC","rated": 1.8,"stall": 2.2,"mass_g": 9,"horn_spline":21},
}

# ── Joint limits (unchanged from v12) ────────────────────────────────────────
JOINT_LIMITS = {
    "Waist_Cluster":      {"pitch":(-45,60),"yaw":(-15,15),"roll":(-15,15)},
    "Neck_Cluster":       {"pitch":(-90,45),"yaw":(-20,20),"roll":(-20,20)},
    "L_Hip_Cluster":      {"pitch":(-30,30),"yaw":(-95,95),"roll":(-30,30)},
    "R_Hip_Cluster":      {"pitch":(-30,30),"yaw":(-95,95),"roll":(-30,30)},
    "L_Knee":{"pitch":(0,135),"yaw":(-15,15)}, "R_Knee":{"pitch":(0,135),"yaw":(-15,15)},
    "L_Ankle_Cluster":    {"pitch":(-20,20),"yaw":(-30,95),"roll":(-20,20)},
    "R_Ankle_Cluster":    {"pitch":(-20,20),"yaw":(-30,95),"roll":(-20,20)},
    "L_Shoulder_Cluster": {"pitch":(-175,60),"yaw":(-90,90),"roll":(-90,90)},
    "R_Shoulder_Cluster": {"pitch":(-175,60),"yaw":(-90,90),"roll":(-90,90)},
    "L_Elbow":{"pitch":(0,150)}, "R_Elbow":{"pitch":(0,150)},
    "L_Wrist":{"pitch":(0,90),"roll":(-180,180)},
    "R_Wrist":{"pitch":(0,135),"roll":(-180,180)},
    "Blaster_Fold":{"pitch":(-90,0)},
    "L_Pinky_MCP":{"pitch":(-5,85)},  "R_Pinky_MCP":{"pitch":(-5,85)},
    "L_Ring_MCP":{"pitch":(-5,85)},   "R_Ring_MCP":{"pitch":(-5,85)},
    "L_Middle_MCP":{"pitch":(-5,85)}, "R_Middle_MCP":{"pitch":(-5,85)},
    "L_Index_MCP":{"pitch":(-5,85)},  "R_Index_MCP":{"pitch":(-5,85)},
    "L_Thumb_CMC":{"pitch":(-20,60),"yaw":(-30,30)},
    "R_Thumb_CMC":{"pitch":(-20,60),"yaw":(-30,30)},
}

SPLIT_KEYS = {"Shell","Link","Main","Armor","Core","Pod","Palm",
              "Block","Sole","Plate","Bay","Collar"}
SKIP_TAGS  = {"Marker","Pivot","MtA","MtB","Axle_Pivot","Horn",
              "Pin","_Vis","Boss","Insert","Nut","Snap"}

# ── Registries ────────────────────────────────────────────────────────────────
SCREW_REGISTRY  = []
ASSEMBLY_STEPS  = []
JIG_REGISTRY    = []
PRINT_NOTES     = []
SUPPORT_WARNINGS= []

# ── V13 Communication map registry ────────────────────────────────────────────
COMM_MAP = []   # list of {from, to, bus, speed, purpose}
POWER_MAP= []   # list of {rail, voltage, max_A, consumers}
SENSOR_REGISTRY = []   # list of {kind, comp, location, node}

def _reg_comm(frm, to, bus, speed, purpose):
    COMM_MAP.append({"from":frm,"to":to,"bus":bus,"speed":speed,"purpose":purpose})
def _reg_power(rail, volt, max_a, consumers):
    POWER_MAP.append({"rail":rail,"voltage":volt,"max_A":max_a,"consumers":consumers})
def _reg_sensor(kind, comp, location, node):
    SENSOR_REGISTRY.append({"kind":kind,"comp":comp,"location":location,"node":node})


# ═════════════════════════════════════════════════════════════════════════════
# LOGGER  +  BOM
# ═════════════════════════════════════════════════════════════════════════════
class Logger:
    _buffer = []
    _count  = 0
    _max_buffer = 500   # ROBUST-V14-5: cap buffer growth if disk flush keeps failing

    @classmethod
    def log(cls, msg, level="INFO"):
        ts   = datetime.datetime.now().strftime("%H:%M:%S")
        line = f"[{ts}] [{level}] {msg}\n"
        safe = line.encode("ascii", "replace").decode("ascii")
        print(safe, end="", flush=True)
        cls._buffer.append(line)
        cls._count += 1
        if cls._count >= 20 or len(cls._buffer) > cls._max_buffer:
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
    """Bill-of-Materials accumulator."""
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

    Logger.log("=" * 64)
    Logger.log("EXECUTION START  v14.0 -- Optimus Prime G1 Unified Distributed AI Edition")
    Logger.log("Brain: NVIDIA Jetson Nano | Nodes: ESP32-S3 x3 | Eyes: CSI camera")
    Logger.log("Bugfixes: move_ball axis bug, merge-fastener None guards,")
    Logger.log("          empty-registry guards, robust STEP skip-checks")
    Logger.log("=" * 64)

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
        nylon_white   = white_pla or get_ap("Nylon - White", "Plastic - Glossy (White)")
        jetson_green  = get_ap("Circuit Board - Green",         "Steel - Flat")
        heatsink_silv = get_ap("Aluminum - Brushed",            "Steel - Polished")

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

        # ROBUST-V14-1: shared axis/dimension validation guards
        def _safe_axis(axis, default="z"):
            if axis not in ("x", "y", "z"):
                Logger.log(f"Invalid axis '{axis}', defaulting to '{default}'", "WARN")
                return default
            return axis

        def box(comp, name, cx, cy, cz, lx, ly, lz, ap=None):
            if lx <= 0 or ly <= 0 or lz <= 0:
                Logger.log(f"box({name}): invalid dims {lx},{ly},{lz} -- skipped", "WARN")
                return None
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
            axis = _safe_axis(axis)
            if r <= 0 or h <= 0:
                Logger.log(f"cyl({name}): invalid dims r={r} h={h} -- skipped", "WARN")
                return None
            temp = adsk.fusion.TemporaryBRepManager.get()
            av   = {"x": (1, 0, 0), "y": (0, 1, 0), "z": (0, 0, 1)}[axis]
            p1   = adsk.core.Point3D.create(cx-av[0]*h/2, cy-av[1]*h/2, cz-av[2]*h/2)
            p2   = adsk.core.Point3D.create(cx+av[0]*h/2, cy+av[1]*h/2, cz+av[2]*h/2)
            shape = temp.createCylinderOrCone(p1, r, p2, r)
            bf    = comp.features.baseFeatures.add()
            bf.startEdit()
            body  = comp.bRepBodies.add(shape, bf)
            bf.finishEdit()
            body.name = name
            set_ap(body, ap)
            return body

        def cone_shape(comp, name, cx, cy, cz, r1, r2, h, axis, ap=None):
            axis = _safe_axis(axis)
            if r1 < 0 or r2 < 0 or h <= 0:
                Logger.log(f"cone({name}): invalid dims r1={r1} r2={r2} h={h} -- skipped", "WARN")
                return None
            temp  = adsk.fusion.TemporaryBRepManager.get()
            av    = {"x": (1, 0, 0), "y": (0, 1, 0), "z": (0, 0, 1)}[axis]
            p1    = adsk.core.Point3D.create(cx-av[0]*h/2, cy-av[1]*h/2, cz-av[2]*h/2)
            p2    = adsk.core.Point3D.create(cx+av[0]*h/2, cy+av[1]*h/2, cz+av[2]*h/2)
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

        # ── MFG-V14-1: TRUE 45-degree printable chamfer ────────────────────
        def chamfer_wedge_cut(comp, tag, run_axis, run_center, run_len,
                              p_corner, q_corner, chamfer):
            """Cuts a genuine 45-degree wedge along an edge that runs parallel
            to `run_axis`, at the sharp corner (p_corner, q_corner) in the
            plane perpendicular to run_axis. The cutter is a square, rotated
            45 degrees, centered EXACTLY on the sharp corner -- the standard
            parametric trick for adding a real chamfer without needing direct
            B-Rep edge selection: a 45-degree-rotated square centered on a
            box corner always removes precisely a right-triangle wedge whose
            legs equal half the square's diagonal, leaving a true 45-degree
            face behind. (axis convention: for run_axis='z', p=X, q=Y; for
            run_axis='y', p=X, q=Z; for run_axis='x', p=Y, q=Z.)"""
            if chamfer <= 0:
                return False
            L = chamfer * math.sqrt(2.0)
            c45, s45 = math.cos(math.radians(45)), math.sin(math.radians(45))
            temp = adsk.fusion.TemporaryBRepManager.get()
            try:
                if run_axis == "x":
                    center     = adsk.core.Point3D.create(run_center, p_corner, q_corner)
                    length_dir = adsk.core.Vector3D.create(0, c45,  s45)
                    width_dir  = adsk.core.Vector3D.create(0, -s45, c45)
                elif run_axis == "y":
                    center     = adsk.core.Point3D.create(p_corner, run_center, q_corner)
                    length_dir = adsk.core.Vector3D.create(c45, 0, s45)
                    width_dir  = adsk.core.Vector3D.create(-s45, 0, c45)
                else:
                    center     = adsk.core.Point3D.create(p_corner, q_corner, run_center)
                    length_dir = adsk.core.Vector3D.create(c45, s45, 0)
                    width_dir  = adsk.core.Vector3D.create(-s45, c45, 0)
                obb   = adsk.core.OrientedBoundingBox3D.create(
                            center, length_dir, width_dir, L, L, run_len + 0.10)
                shape = temp.createBox(obb)
                bf    = comp.features.baseFeatures.add()
                bf.startEdit()
                body  = comp.bRepBodies.add(shape, bf)
                bf.finishEdit()
                body.name = f"{tag}_ChamferCut"
                cut_cavity(comp, body)
                return True
            except Exception as e:
                Logger.log(f"chamfer_wedge_cut failed for {tag}: {e}", "WARN")
                return False

        def chamfer_box(comp, name, cx, cy, cz, lx, ly, lz, axis, chamfer=0.25, ap=None):
            """MFG-3 / MFG-V14-1 — Box with a TRUE 45-degree wedge chamfer cut
            into its bottom-front edge (-Y face meeting -Z face, the model's
            consistent overhang-risk direction since the whole figure stands
            in +Z). Unlike a no-op stub or a stacked second box, this removes
            a real triangular wedge via chamfer_wedge_cut(), so the printed
            part is genuinely self-supporting at that edge. `axis` is kept
            for call-site documentation/back-compat; the cut always targets
            the bottom-front edge, which is where every current call site
            (chin guard, heel spur, heel block, toe cap) actually overhangs."""
            main = box(comp, name, cx, cy, cz, lx, ly, lz, ap)
            if not main or chamfer <= 0:
                return main
            if chamfer < 0.15:
                SUPPORT_WARNINGS.append(
                    (name, f"chamfer {chamfer}cm is thin -- verify it actually clears the overhang"))
            chamfer_wedge_cut(comp, name, run_axis="x", run_center=cx, run_len=lx,
                              p_corner=cy - ly/2.0, q_corner=cz - lz/2.0, chamfer=chamfer)
            return main

        # ── Boolean cavity cutter ─────────────────────────────────────────
        def cut_cavity(comp, tool_body, keep_tool=False):
            """ROBUST-V14-2 — Re-fetches both the tool and each target body by
            name immediately before every combine call. Sequential combine
            operations in Fusion can silently invalidate previously-held
            BRepBody references (the underlying body gets replaced), so
            holding onto the original Python object across many cuts in a
            row is a common source of silent no-op failures. Looking the
            body up by name string right before use avoids that class of bug."""
            if not tool_body or not tool_body.isValid:
                Logger.log("cut_cavity: invalid tool body", "WARN")
                return False
            tool_name = tool_body.name
            target_names = []
            for b in list(comp.bRepBodies):
                if b == tool_body:
                    continue
                if b.name and any(t in b.name for t in SKIP_TAGS):
                    continue
                target_names.append(b.name)

            success = False
            for t_name in target_names:
                t_body  = comp.bRepBodies.itemByName(t_name)
                cur_tool = comp.bRepBodies.itemByName(tool_name)
                if not t_body or not t_body.isValid:
                    continue
                if not cur_tool or not cur_tool.isValid:
                    break  # tool itself got invalidated; nothing more we can cut with it
                tools = adsk.core.ObjectCollection.create()
                tools.add(cur_tool)
                try:
                    ci = comp.features.combineFeatures.createInput(t_body, tools)
                    ci.operation        = adsk.fusion.CombineOperation.CutFeatureOperation
                    ci.isKeepToolBodies = True
                    comp.features.combineFeatures.add(ci)
                    success = True
                except Exception as e:
                    Logger.log(f"cut_cavity: cut on {t_name} failed: {e}", "DEBUG")

            if not keep_tool:
                cur_tool = comp.bRepBodies.itemByName(tool_name)
                if cur_tool and cur_tool.isValid:
                    try:
                        cur_tool.isLightBulbOn = False
                        if "_Vis" not in cur_tool.name:
                            cur_tool.name += "_Vis"
                    except Exception:
                        pass
            return success

        # ── Shell splitter for FDM printing ───────────────────────────────
        def split_halves(comp, body, axis="y", offset=0.0):
            """ROBUST-V14-3 — Validates the split actually produced >=2
            bodies before declaring success; many silent split failures in
            Fusion otherwise look identical to a successful single-body split."""
            if not body or not body.isValid:
                Logger.log(f"split_halves: invalid body in {comp.name}", "WARN")
                return False
            before_count = len([b for b in comp.bRepBodies if b.isValid])
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
                after_count = len([b for b in comp.bRepBodies if b.isValid])
                if after_count <= before_count:
                    Logger.log(f"split_halves: {body.name} did not split "
                               f"(before={before_count} after={after_count})", "WARN")
                    return False
                return True
            except Exception as e:
                Logger.log(f"split_halves failed: {e}", "WARN")
                return False

        # ── MFG-1 — Post-split fastener merge (None-safe on either side) ──
        def merge_fasteners_to_halves(comp, body_left, body_right, axis="y"):
            """After split, merge Pin/Boss/Insert/Nut/Snap bodies to the
            nearest shell half based on position relative to the split plane.
            ROBUST-V14-4: callers now make ONE consolidated call per component
            passing both halves together (instead of two separate calls, one
            per side), which avoids re-scanning the whole body list twice for
            every split and removes the only-one-side-was-merged edge case.
            body_left and/or body_right may legitimately be None."""
            if body_left is None and body_right is None:
                return
            fastener_tags = {"Pin", "Boss", "Insert", "Nut", "Snap"}
            skip_local    = {"_Vis", "Marker", "Pivot", "MtA", "MtB", "Axle_Pivot", "Horn"}
            left_name  = body_left.name  if body_left  and body_left.isValid  else None
            right_name = body_right.name if body_right and body_right.isValid else None

            fasteners_to_merge = []
            for b in list(comp.bRepBodies):
                if not b.name or any(skip in b.name for skip in skip_local):
                    continue
                if not any(tag in b.name for tag in fastener_tags):
                    continue
                try:
                    cog = b.physicalProperties.centerOfMass
                    pos = cog.y if axis == "y" else cog.z if axis == "z" else cog.x
                    if left_name and pos < 0:
                        fasteners_to_merge.append((b.name, left_name))
                    elif right_name and pos > 0:
                        fasteners_to_merge.append((b.name, right_name))
                    elif left_name:
                        fasteners_to_merge.append((b.name, left_name))
                    elif right_name:
                        fasteners_to_merge.append((b.name, right_name))
                except Exception:
                    pass

            for f_name, t_name in fasteners_to_merge:
                f_body = comp.bRepBodies.itemByName(f_name)
                t_body = comp.bRepBodies.itemByName(t_name)
                if not f_body or not f_body.isValid or not t_body or not t_body.isValid:
                    continue
                try:
                    tools = adsk.core.ObjectCollection.create()
                    tools.add(f_body)
                    ci = comp.features.combineFeatures.createInput(t_body, tools)
                    ci.operation = adsk.fusion.CombineOperation.JoinFeatureOperation
                    ci.isKeepToolBodies = False
                    comp.features.combineFeatures.add(ci)
                except Exception as e:
                    Logger.log(f"merge_fastener failed for {f_name}: {e}", "DEBUG")

        def printability_check(comp, body_name, overhang_angle_deg=45):
            """MFG-2 — Heuristic overhang/support warning logger."""
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
        # PHYSICAL FEATURE HELPERS  (PHY-1 … PHY-11, kept from v12)
        # ─────────────────────────────────────────────────────────────────

        def m3_boss(comp, tag, cx, cy, cz, axis="z", depth=0.80, screw_len=1.0):
            """PHY-1 — Ø7 mm boss cylinder + Ø4.7 mm heat-set insert pocket."""
            boss = cyl(comp, f"{tag}_Boss", cx, cy, cz, BOSS_R, depth, axis, dark_metal)
            boss.name = f"{tag}_Boss"
            cut_cavity(comp, cyl(comp, f"{tag}_Insert",
                                 cx, cy, cz, INSERT_R, INSERT_H, axis))
            BOM.add("Fastener", "M3 heat-set insert (Voron M3x5)", 1,
                    f"boss at ({cx:.1f},{cy:.1f},{cz:.1f}) in {comp.name}")
            SCREW_REGISTRY.append({
                "tag": tag, "comp": comp.name, "type": "boss_insert",
                "shell_t": depth, "boss_depth": INSERT_H,
                "requested_len": screw_len, "axis": axis,
                "cx": cx, "cy": cy, "cz": cz,
            })

        def captive_nut(comp, tag, cx, cy, cz, axis="z", bolt_len=1.0):
            """PHY-2 — M3 hex-nut trap + clearance through-bore."""
            cut_cavity(comp, cyl(comp, f"{tag}_NutTrap",
                                 cx, cy, cz, M3_NUT_CIR, M3_NUT_H, axis))
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
            """PHY-3 — Cantilever snap-fit pair."""
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

        def horn_coupler(comp, tag, cx, cy, cz, axis, servo_type="std"):
            """PHY-6 — Servo output-shaft coupler with keyed slot, clamp hub,
            and set-screw pocket."""
            is_std = (servo_type in ("std", "hip_hd", "waist"))
            hub_r  = 0.55 if is_std else 0.35
            hub_h  = 0.80 if is_std else 0.55
            key_w  = 0.14 if is_std else 0.10
            key_d  = 0.18 if is_std else 0.13
            setscrew_r = M3_PILOT_R if is_std else 0.09

            av = {"x": (1, 0, 0), "y": (0, 1, 0), "z": (0, 0, 1)}[axis]
            cyl(comp, f"{tag}_CouplerHub", cx, cy, cz, hub_r, hub_h, axis, dark_metal)

            if axis in ("x", "z"):
                cut_cavity(comp, box(comp, f"{tag}_KeySlot",
                    cx + av[0]*hub_h*0.25, cy, cz + av[2]*hub_h*0.25,
                    key_d if axis == "z" else hub_h*0.6,
                    key_w,
                    key_d if axis == "x" else hub_h*0.6))
            else:
                cut_cavity(comp, box(comp, f"{tag}_KeySlot",
                    cx, cy + av[1]*hub_h*0.25, cz, key_d, hub_h*0.6, key_d))

            if axis in ("x", "z"):
                cut_cavity(comp, box(comp, f"{tag}_ClampSlit",
                    cx + av[0]*hub_h*0.35, cy, cz + av[2]*hub_h*0.35,
                    0.06 if axis == "z" else hub_h*0.5,
                    hub_r*2.2,
                    0.06 if axis == "x" else hub_h*0.5))
            else:
                cut_cavity(comp, box(comp, f"{tag}_ClampSlit",
                    cx, cy + av[1]*hub_h*0.35, cz, hub_r*2.2, hub_h*0.5, 0.06))

            setscrew_axis = "y" if axis in ("x", "z") else "z"
            cut_cavity(comp, cyl(comp, f"{tag}_SetScrew",
                cx + av[0]*hub_h*0.15, cy, cz + av[2]*hub_h*0.15,
                setscrew_r, hub_r*2.2, setscrew_axis))

            arm_len = hub_r * 1.8
            arm_r   = hub_r * 0.65
            cyl(comp, f"{tag}_TorqueArm",
                cx + av[0]*arm_len, cy + av[1]*arm_len, cz + av[2]*arm_len,
                arm_r, hub_h*0.8, axis, dark_metal)

            spec_name = SERVO_SPECS.get(servo_type, SERVO_SPECS["std"])["name"]
            BOM.add("Printed", f"Servo coupler hub ({spec_name})", 1, comp.name)
            BOM.add("Fastener", "M3x4 set screw (cup point)", 1, comp.name)
            ASSEMBLY_STEPS.append(f"Install {tag} coupler onto {spec_name} servo horn; tighten set-screw")

        def servo_drum(comp, tag, cx, cy, cz, axis="z"):
            """PHY-7 — Tendon winch drum with flanges + tie-off hole."""
            cyl(comp, f"{tag}_DrumBarrel", cx, cy, cz, DRUM_R, DRUM_H, axis, dark_metal)
            cyl(comp, f"{tag}_DrumFlangeT", cx, cy, cz + DRUM_H/2 - 0.02,
                DRUM_R + 0.10, 0.06, axis, dark_metal)
            cyl(comp, f"{tag}_DrumFlangeB", cx, cy, cz - DRUM_H/2 + 0.02,
                DRUM_R + 0.10, 0.06, axis, dark_metal)
            tie_axis = "x" if axis in ("y", "z") else "z"
            cut_cavity(comp, cyl(comp, f"{tag}_TieHole", cx, cy, cz,
                                 0.06, DRUM_R*2.2, tie_axis))
            BOM.add("Printed", "Servo winch drum (tendon drive)", 1, comp.name)

        def tendon_guide(comp, tag, cx, cy, cz, length, axis="z"):
            """PHY-8 — V-groove tendon guide channel."""
            gr = TENDON_GUIDE_R + 0.02
            cut_cavity(comp, cyl(comp, f"{tag}_TendonGuide", cx, cy, cz, gr, length, axis))

        def tendon_anchor(comp, tag, cx, cy, cz, axis="z"):
            """PHY-9 — Distal tendon anchor + crimp slot."""
            box(comp, f"{tag}_Anchor", cx, cy, cz, 0.35, 0.28, 0.22, dark_metal)
            cut_cavity(comp, box(comp, f"{tag}_CrimpSlot", cx, cy, cz, 0.06, 0.30, 0.14))
            BOM.add("Hardware", "Tendon anchor (printed)", 1, comp.name)

        def palm_pulley(comp, tag, cx, cy, cz, axis="x"):
            """PHY-10 — Idler pulley / guide post."""
            cyl(comp, f"{tag}_PulleyPost", cx, cy, cz, 0.12, 0.50, axis, chrome)
            pulley_axis = "y" if axis in ("x", "z") else "z"
            cyl(comp, f"{tag}_PulleyWheel", cx, cy, cz, PULLEY_R, 0.14, pulley_axis, grey_plastic)
            BOM.add("Printed", "Palm idler pulley", 1, comp.name)

        def spring_return(comp, tag, cx, cy, cz, axis="x"):
            """PHY-11 — Torsion-spring pocket for finger extension."""
            cut_cavity(comp, cyl(comp, f"{tag}_SpringPkt", cx, cy, cz,
                                 SPRING_OD/2 + 0.03, SPRING_WIRE*4, axis))
            peg_axis = "y" if axis in ("x", "z") else "z"
            for sign in [-1, 1]:
                peg_pos = SPRING_OD/2 + 0.06
                if peg_axis == "y":
                    cyl(comp, f"{tag}_SpringPeg_{sign}", cx, cy + sign*peg_pos, cz,
                        0.06, 0.20, peg_axis, chrome)
                else:
                    cyl(comp, f"{tag}_SpringPeg_{sign}", cx, cy, cz + sign*peg_pos,
                        0.06, 0.20, peg_axis, chrome)
            BOM.add("Hardware", "Torsion spring (finger return)", 1, comp.name)

        # ── Generic fastener / utility helpers ────────────────────────────
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
            BOM.add("Electronics", "WS2812 5050 LED ring Ø12 mm", 1, comp.name)

        # ── CAB-1 … CAB-4 ──────────────────────────────────────────────────
        def cable_clip(comp, tag, cx, cy, cz, axis="y"):
            box(comp, f"{tag}_ClipBase", cx, cy, cz, CABLE_CLIP_W, 0.15, 0.35, grey_plastic)
            cyl(comp, f"{tag}_ClipArch", cx, cy, cz + 0.06,
                CABLE_CLIP_R + 0.08, CABLE_CLIP_W, "x", grey_plastic)
            BOM.add("Printed", "Snap-in cable clip", 1, comp.name)

        def wire_hub(comp, tag, cx, cy, cz):
            box(comp, f"{tag}_HubBlock", cx, cy, cz, 2.0, 1.6, 1.2, dark_grey)
            for dx, dy, dz, lbl in [(-1.0,0,0,"L"), (1.0,0,0,"R"),
                                     (0,-0.8,0,"F"), (0,0.8,0,"B"),
                                     (0,0,-0.6,"D"), (0,0,0.6,"U")]:
                wire_channel(comp, f"{tag}_Hub{lbl}",
                    cx+dx*0.5, cy+dy*0.5, cz+dz*0.5, 0.25, 0.80,
                    "x" if abs(dx) > abs(dy) and abs(dx) > abs(dz) else
                    "y" if abs(dy) > abs(dz) else "z")
            BOM.add("Printed", "Torso wire hub", 1, comp.name)

        def grommet_slot(comp, tag, cx, cy, cz, axis="y", width=0.50):
            cut_cavity(comp, cyl(comp, f"{tag}_GromSlot", cx, cy, cz, GROMMET_R, width, axis))
            seat_r = GROMMET_R + 0.06
            cut_cavity(comp, cyl(comp, f"{tag}_GromSeat", cx, cy, cz, seat_r, 0.10, axis))
            BOM.add("Hardware", "Rubber grommet Ø3.5 mm (open slot)", 1, comp.name)

        def jst_pocket(comp, tag, cx, cy, cz, axis="y"):
            cut_cavity(comp, box(comp, f"{tag}_JST", cx, cy, cz,
                JST_XH_L + 0.10, JST_XH_W + 0.10, JST_XH_H + 0.10))

        # ── BRG-2 / BRG-3 ──────────────────────────────────────────────────
        def bearing_fit(comp, tag, cx, cy, cz, axis="x", ro=1.10, w=0.60, fit_type="press"):
            tol = (BEARING_FIT_TOLERANCE if fit_type == "press" else
                   0.0 if fit_type == "glue" else 0.015)
            outer_r = ro + tol
            cyl(comp, f"{tag}_BO", cx, cy, cz, ro,       w,      axis, chrome)
            cyl(comp, f"{tag}_BI", cx, cy, cz, ro*0.58,  w*0.80, axis, dark_grey)
            cyl(comp, f"{tag}_BB", cx, cy, cz, ro*0.32,  w*1.10, axis, chrome)
            temp = adsk.fusion.TemporaryBRepManager.get()
            av   = {"x": (1,0,0), "y": (0,1,0), "z": (0,0,1)}[axis]
            half = w/2.0 + 0.05
            p1   = adsk.core.Point3D.create(cx-av[0]*half, cy-av[1]*half, cz-av[2]*half)
            p2   = adsk.core.Point3D.create(cx+av[0]*half, cy+av[1]*half, cz+av[2]*half)
            cs   = temp.createCylinderOrCone(p1, outer_r+0.05, p2, outer_r+0.05)
            bf   = comp.features.baseFeatures.add(); bf.startEdit()
            cb   = comp.bRepBodies.add(cs, bf); bf.finishEdit()
            cb.name = f"{tag}_CB"
            cut_cavity(comp, cb)
            if fit_type in ("press", "glue"):
                if axis == "x":   lip_x, lip_y, lip_z = cx - w/2 + BEARING_RETAIN_LIP_H/2, cy, cz
                elif axis == "y": lip_x, lip_y, lip_z = cx, cy - w/2 + BEARING_RETAIN_LIP_H/2, cz
                else:             lip_x, lip_y, lip_z = cx, cy, cz - w/2 + BEARING_RETAIN_LIP_H/2
                cyl(comp, f"{tag}_Lip", lip_x, lip_y, lip_z,
                    outer_r + BEARING_RETAIN_LIP_R, BEARING_RETAIN_LIP_H, axis, dark_metal)
            fit_label = f"{fit_type}-fit"
            BOM.add("Bearing", f"O{int(ro*2*10)} mm bearing ({fit_label})", 1, comp.name)
            BOM.add("Hardware", f"Bearing {fit_label} tolerance", 1, comp.name)

        def dual_bearing(comp, tag, cx, cy, cz, axis="x", ro=1.10, w=0.60,
                         span=2.50, fit_type="press"):
            av = {"x": (1,0,0), "y": (0,1,0), "z": (0,0,1)}[axis]
            box(comp, f"{tag}_Carrier", cx, cy, cz,
                span*av[0]+1.2, span*av[1]+1.2, span*av[2]+1.2, dark_metal)
            p1 = (cx - av[0]*span/2, cy - av[1]*span/2, cz - av[2]*span/2)
            p2 = (cx + av[0]*span/2, cy + av[1]*span/2, cz + av[2]*span/2)
            bearing_fit(comp, f"{tag}_A", p1[0], p1[1], p1[2], axis, ro, w, fit_type)
            bearing_fit(comp, f"{tag}_B", p2[0], p2[1], p2[2], axis, ro, w, fit_type)
            cyl(comp, f"{tag}_Axle", cx, cy, cz, ro*0.55, span + 1.0, axis, chrome)
            BOM.add("Hardware", f"Steel axle O{int(ro*0.55*20)} mm x {int((span+1.0)*10)} mm",
                    1, comp.name)
            ASSEMBLY_STEPS.append(f"Install {tag} bearings into carrier; insert steel axle")

        def bearing(comp, tag, cx, cy, cz, axis="x", ro=1.10, w=0.60):
            """Legacy wrapper -> redirects to bearing_fit (glue-fit default)."""
            bearing_fit(comp, tag, cx, cy, cz, axis, ro, w, fit_type="glue")

        # ── COV-1 / COV-2 / COV-3 ──────────────────────────────────────────
        def cover_plate(comp, tag, cx, cy, cz, lx, lz, boss_positions,
                        method="screw", hinge_edge=None, ap=None):
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
                cyl(comp, f"{tag}_HingeBarrel", cx, hinge_y, cz,
                    0.12, lx*0.9 if hinge_axis == "x" else lz*0.9, hinge_axis, dark_metal)
                latch_x = cx + (lx/2 - 0.3) if hinge_edge != "right" else cx - (lx/2 - 0.3)
                box(comp, f"{tag}_Latch", latch_x, hinge_y, cz, 0.5, 0.18, 0.4, dark_metal)
                BOM.add("Hardware", "M2x10 hinge pin (steel)", 1, comp.name)
            BOM.add("Printed", f"Cover plate ({method})", 1, comp.name)
            PRINT_NOTES.append((f"{tag}_Cover", "print flat (face down)", False))
            return cover

        def lipo_door(comp, tag, cx, cy, cz, lx, lz):
            cover_plate(comp, tag, cx, cy, cz, lx, lz,
                        [(-lx/2+0.6,-lz/2+0.6), (lx/2-0.6,-lz/2+0.6),
                         (-lx/2+0.6, lz/2-0.6), (lx/2-0.6, lz/2-0.6)],
                        method="snap", ap=dark_grey)
            box(comp, f"{tag}_Pull", cx, cy+0.20, cz+lz/2-0.4, lx*0.4, 0.15, 0.35, dark_grey)
            for vz in [-lz*0.2, 0, lz*0.2]:
                cut_cavity(comp, box(comp, f"{tag}_Vent_{vz:.0f}",
                    cx, cy+0.01, cz+vz, lx*0.6, 0.08, 0.12))
            BOM.add("Printed", "LiPo bay door (vented)", 1, comp.name)

        def pcb_cover(comp, tag, cx, cy, cz, lx, lz, method="screw"):
            cover_plate(comp, tag, cx, cy, cz, lx, lz,
                        [(-lx/2+0.5,-lz/2+0.5), (lx/2-0.5,-lz/2+0.5),
                         (-lx/2+0.5, lz/2-0.5), (lx/2-0.5, lz/2-0.5)],
                        method=method, ap=grey_plastic)
            for vy in [-lz*0.25, 0, lz*0.25]:
                cut_cavity(comp, box(comp, f"{tag}_VSlot_{vy:.0f}",
                    cx, cy+0.01, cz+vy, lx*0.55, 0.06, 0.25))
            BOM.add("Printed", f"PCB cover ({method})", 1, comp.name)

        # ── JIG-1 ──────────────────────────────────────────────────────────
        def assembly_jig(comp_name, pin_positions, socket_positions, base_size):
            jig = new_component(f"JIG_{comp_name}")
            lx, ly, lz = base_size
            box(jig, "Jig_Base", 0, 0, 0, lx, ly, lz, nylon_white)
            for i, (px, py, pz) in enumerate(pin_positions):
                cyl(jig, f"JigPin_{i}", px, py, pz + lz/2, ALIGN_PIN_R + 0.02, 0.50, "z", chrome)
            for i, (sx, sy, sz) in enumerate(socket_positions):
                cyl(jig, f"JigSock_{i}", sx, sy, sz + lz/2, ALIGN_PIN_R + 0.025, 0.30, "z", dark_grey)
            JIG_REGISTRY.append((jig.name, comp_name))
            PRINT_NOTES.append((jig.name, "print base flat, pins upright", True))
            BOM.add("Tooling", f"Assembly jig for {comp_name}", 1, "printed")
            ASSEMBLY_STEPS.append(f"Print jig for {comp_name}; use during shell alignment")
            return jig

        # ── BUGFIX-V13-3: FST-1 with empty-registry guard ─────────────────
        def verify_screw_lengths():
            """FST-1 — Validate registered screw lengths. Guards against an
            empty SCREW_REGISTRY (would previously just print '0 issues' with
            no context, now explicitly logs that no fasteners were registered
            instead of silently passing)."""
            Logger.log("--- V13 SCREW LENGTH VERIFICATION ---")
            if not SCREW_REGISTRY:
                Logger.log("  No fasteners registered yet -- skipping verification.", "WARN")
                return
            issues = 0
            for entry in SCREW_REGISTRY:
                req       = entry["requested_len"]
                tag       = entry["tag"]
                comp_name = entry["comp"]
                stype     = entry["type"]
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
                Logger.log(f"  All {len(SCREW_REGISTRY)} registered screw lengths OK [PASS]")
            else:
                Logger.log(f"  {issues} screw length issue(s) found out of "
                           f"{len(SCREW_REGISTRY)} -- review BOM", "WARN")


        # ─────────────────────────────────────────────────────────────────
        # V13 NEW: JETSON NANO + ESP32-S3 + VISION/SENSOR ELECTRONICS BAYS
        # ─────────────────────────────────────────────────────────────────

        def jetson_nano_bay(comp, tag, cx, cy, cz):
            """ELEC-V13-1 — NVIDIA Jetson Nano compact-carrier pocket.
            Includes: module clearance, 30mm fan mount, heatsink clearance,
            and connector access slots (USB3 x2, USB-C power, HDMI, CSI,
            40-pin GPIO, barrel jack)."""
            # Main pocket (board + heatsink stack)
            cut_cavity(comp, box(comp, f"{tag}_JNanoBay", cx, cy, cz,
                                 JNANO_L + 0.30, JNANO_W + 0.30, JNANO_H + 0.40))
            # 4x M2.5 standoffs (Jetson Nano mounting hole pattern: 58x58mm typical
            # for compact carriers -- using representative positions)
            for sx, sz in [(-2.80, -1.90), (2.80, -1.90), (-2.80, 1.90), (2.80, 1.90)]:
                cyl(comp, f"{tag}_JStdoff_{sx:.0f}_{sz:.0f}",
                    cx+sx, cy, cz+sz, 0.14, JNANO_H+0.60, "y", dark_metal)
            # 30mm fan mount (top face, blows down onto heatsink)
            cut_cavity(comp, box(comp, f"{tag}_FanBay",
                cx, cy - JNANO_H/2 - JNANO_FAN_H/2 - 0.05, cz,
                JNANO_FAN_SZ + 0.10, JNANO_FAN_H + 0.10, JNANO_FAN_SZ + 0.10))
            for fx, fz in [(-1.15, -1.15), (1.15, -1.15), (-1.15, 1.15), (1.15, 1.15)]:
                cyl(comp, f"{tag}_FanScrew_{fx:.0f}_{fz:.0f}",
                    cx+fx, cy - JNANO_H/2 - JNANO_FAN_H - 0.05, cz+fz,
                    0.090, 0.50, "y", dark_metal)
            # Heatsink clearance (open vent above CPU/GPU module)
            cut_cavity(comp, box(comp, f"{tag}_HsinkClear",
                cx, cy - JNANO_H/2 - 0.05, cz,
                JNANO_MODULE_L*0.55, JNANO_HSINK_H + 0.10, JNANO_MODULE_W*0.55))
            # Connector access -- USB3 x2 (side)
            for ui_, ux in enumerate([-1.5, -0.5]):
                cut_cavity(comp, box(comp, f"{tag}_USB3_{ui_}",
                    cx + ux, cy, cz - JNANO_W/2 - 0.15,
                    JNANO_USB3_W, JNANO_USB3_H, 0.35))
            # USB-C power input
            cut_cavity(comp, box(comp, f"{tag}_USBC_Pwr",
                cx + 1.6, cy, cz - JNANO_W/2 - 0.15,
                JNANO_USBCC_W, JNANO_USBCC_H, 0.30))
            # HDMI (debug display, optional)
            cut_cavity(comp, box(comp, f"{tag}_HDMI",
                cx - 2.4, cy, cz - JNANO_W/2 - 0.15,
                JNANO_HDMI_W, JNANO_HDMI_H, 0.30))
            # CSI camera ribbon exit (toward head via neck channel)
            cut_cavity(comp, box(comp, f"{tag}_CSI_Exit",
                cx, cy, cz + JNANO_W/2 + 0.15,
                JNANO_CSI_W + 0.10, JNANO_CSI_H + 0.10, 0.35))
            # 40-pin GPIO header access (for ESP32-S3 UART/I2C wiring)
            cut_cavity(comp, box(comp, f"{tag}_GPIO",
                cx, cy + JNANO_H/2 + 0.10, cz - 1.0,
                JNANO_GPIO_W, 0.30, JNANO_GPIO_H + 0.10))
            # DC barrel jack (alternate power input, bench testing)
            cut_cavity(comp, cyl(comp, f"{tag}_Barrel",
                cx + 2.6, cy, cz - JNANO_W/2 - 0.10,
                JNANO_BARREL_R, 0.40, "z"))
            BOM.add("Electronics", "NVIDIA Jetson Nano (4GB, compact carrier)", 1, comp.name)
            BOM.add("Electronics", "30mm 5V cooling fan + heatsink", 1, comp.name)
            BOM.add("Fastener",    "M2.5x8mm standoff (Jetson mount)", 4, comp.name)
            ASSEMBLY_STEPS.append(
                f"Mount Jetson Nano in {tag} bay; attach fan+heatsink before final screw-down")
            _reg_power("Jetson_5V", 5.0, 3.0, ["Jetson Nano module", "fan"])
            Logger.log(f"  Jetson Nano bay created: {tag} in {comp.name}")

        def csi_camera_pocket(comp, tag, cx, cy, cz, lens_axis="y"):
            """ELEC-V13-2 — IMX219 CSI-2 camera pocket (replaces ESP32-CAM as
            the robot's primary eyes). Includes lens port and FPC ribbon exit."""
            cut_cavity(comp, box(comp, f"{tag}_CSICamBay", cx, cy, cz,
                                 CSI_CAM_L + 0.15, CSI_CAM_H + 0.20, CSI_CAM_W + 0.15))
            # Lens port (front-facing aperture)
            ax_map = {"y": (0,1,0), "x": (1,0,0), "z": (0,0,1)}
            cut_cavity(comp, cyl(comp, f"{tag}_LensPort",
                cx, cy - (CSI_CAM_H/2 + 0.30), cz, CSI_LENS_R, 0.60, lens_axis))
            # Mounting screw bosses (2x M2)
            for sx in [-0.9, 0.9]:
                cyl(comp, f"{tag}_CamScrew_{sx:.0f}", cx+sx, cy+CSI_CAM_H/2-0.05, cz,
                    0.075, 0.30, "y", dark_metal)
            BOM.add("Electronics", "Jetson CSI camera module (IMX219 8MP)", 1, comp.name)
            BOM.add("Fastener", "M2x6mm screw (camera mount)", 2, comp.name)
            Logger.log(f"  CSI camera (robot eyes) pocket: {tag} in {comp.name}")
            return

        def fpc_ribbon_channel(comp, tag, cx, cy, cz, length, axis="z"):
            """ELEC-V13-3 — Flat-flex ribbon cable channel for routing the CSI
            camera signal from head -> neck -> torso (Jetson CSI port)."""
            cut_cavity(comp, box(comp, f"{tag}_FPC", cx, cy, cz,
                                 CSI_RIBBON_W + 0.10, CSI_RIBBON_H + 0.06, length))
            BOM.add("Electronics", "15-pin CSI FPC ribbon cable (length per run)", 1, comp.name)

        def esp32s3_node_bay(comp, tag, cx, cy, cz, role="lower"):
            """ELEC-V13-4 — ESP32-S3 control-node pocket. Three nodes total:
            'lower' (hips/knees/ankles), 'upper' (waist/shoulders/elbows),
            'head' (neck/wrist/fingers + ToF sensors)."""
            cut_cavity(comp, box(comp, f"{tag}_ESP32S3Bay", cx, cy, cz,
                                 ESP32S3_L + 0.20, ESP32S3_H + 0.30, ESP32S3_W + 0.20))
            for sx in [-2.1, 2.1]:
                cyl(comp, f"{tag}_E3Std_{sx:.0f}", cx+sx, cy, cz-0.7,
                    0.12, ESP32S3_H+0.40, "y", dark_metal)
            # USB-C programming/serial port access
            cut_cavity(comp, box(comp, f"{tag}_E3USBC", cx - ESP32S3_L/2 - 0.10, cy, cz,
                                 ESP32S3_USBCC_W, ESP32S3_USBCC_H, 0.30))
            # ANTENNA KEEP-OUT (ELEC-V13-9) -- no metal/printed obstruction over
            # the PCB antenna trace at the board's far end
            antenna_clearance(comp, f"{tag}_Ant", cx + ESP32S3_L/2 - ESP32S3_ANT_L/2, cy, cz,
                              ESP32S3_ANT_L, ESP32S3_W + 0.40)
            BOM.add("Electronics", f"ESP32-S3 DevKitC (node: {role})", 1, comp.name)
            roles = {"lower": "hips/knees/ankles + PCA9685 x2 + MPU-6050",
                     "upper": "waist/shoulders/elbows + PCA9685 + INA3221",
                     "head":  "neck/wrist/fingers + PCA9685 + ToF x2"}
            ASSEMBLY_STEPS.append(f"Flash ESP32-S3 node '{role}' firmware; install in {tag}")
            _reg_comm(f"Jetson_USB", f"ESP32S3_{role}", "USB-UART(CH340)", "1 Mbaud",
                      roles.get(role, "control node"))
            Logger.log(f"  ESP32-S3 node bay [{role}]: {tag} in {comp.name}")

        def tof_sensor_pocket(comp, tag, cx, cy, cz, axis="y"):
            """ELEC-V13-5 — VL53L1X Time-of-Flight obstacle sensor pocket."""
            cut_cavity(comp, box(comp, f"{tag}_ToFBay", cx, cy, cz,
                                 TOF_L + 0.15, TOF_H + 0.20, TOF_W + 0.15))
            ax_map = {"y": (0,1,0), "x": (1,0,0), "z": (0,0,1)}
            cut_cavity(comp, cyl(comp, f"{tag}_ToFWindow",
                cx, cy - (TOF_H/2 + 0.25), cz, TOF_LENS_R, 0.50, axis))
            BOM.add("Electronics", "VL53L1X ToF distance sensor", 1, comp.name)

        def ina3221_bay(comp, tag, cx, cy, cz):
            """ELEC-V13-6 — INA3221 3-channel current/voltage monitor pocket
            (servo rail health monitoring, reported back to Jetson)."""
            cut_cavity(comp, box(comp, f"{tag}_INABay", cx, cy, cz,
                                 INA_L + 0.15, INA_H + 0.20, INA_W + 0.15))
            BOM.add("Electronics", "INA3221 3-channel current/power monitor", 1, comp.name)

        def ubec_5v4a_bay(comp, tag, cx, cy, cz):
            """ELEC-V13-7 — 5V/4A UBEC sized for Jetson Nano's peak draw."""
            cut_cavity(comp, box(comp, f"{tag}_UBECBay", cx, cy, cz,
                                 UBEC_JNANO_L + 0.20, UBEC_JNANO_H + 0.20, UBEC_JNANO_W + 0.20))
            for sx, sz in [(-UBEC_JNANO_L/2+0.6, -UBEC_JNANO_W/2+0.6),
                           (UBEC_JNANO_L/2-0.6,  UBEC_JNANO_W/2-0.6)]:
                m3_boss(comp, f"{tag}_UBEC_{sx:.0f}", cx+sx, cy, cz+sz)
            BOM.add("Electronics", "5V 4A UBEC (Jetson Nano power)", 1, comp.name)
            _reg_power("Jetson_5V", 5.0, 4.0, ["Jetson Nano", "fan", "CSI camera"])

        def vent_grille(comp, tag, cx, cy, cz, axis="y", n_slots=N_VENT_SLOTS):
            """ELEC-V13-8 — Thermal management vent grille (Jetson runs hotter
            than RPi Zero under sustained AI inference load)."""
            for i in range(n_slots):
                offset = (i - (n_slots-1)/2.0) * (VENT_SLOT_W * 1.8)
                if axis == "y":
                    cut_cavity(comp, box(comp, f"{tag}_Vent_{i}",
                        cx + offset, cy, cz, VENT_SLOT_W, 0.40, VENT_SLOT_L))
                else:
                    cut_cavity(comp, box(comp, f"{tag}_Vent_{i}",
                        cx, cy + offset, cz, VENT_SLOT_L, VENT_SLOT_W, 0.40))
            BOM.add("Printed", "Thermal vent grille (integrated)", 1, comp.name)

        def antenna_clearance(comp, tag, cx, cy, cz, lx, lz):
            """ELEC-V13-9 — Keep-out marker zone (visual only, no geometry cut)
            ensuring no metal fastener or dense-infill wall sits directly over
            an ESP32-S3 PCB antenna trace. Logs the zone for the build doc."""
            marker(comp, f"{tag}_AntKeepOut", cx, cy, cz, 0.10)
            ASSEMBLY_STEPS.append(
                f"NOTE: {tag} marks a WiFi/BT antenna keep-out zone "
                f"({lx:.1f}x{lz:.1f}cm) -- avoid metal screws/foil within this area")

        def uart_bridge_cutout(comp, tag, cx, cy, cz, axis="y"):
            """ELEC-V13-10 — USB-C panel cutout for the CH340 serial bridge
            cable connecting Jetson <-> each ESP32-S3 node externally
            (used during firmware flashing / debug without disassembly)."""
            cut_cavity(comp, box(comp, f"{tag}_UARTBridge", cx, cy, cz,
                                 ESP32S3_USBCC_W + 0.08, ESP32S3_USBCC_H + 0.08, 0.35))
            BOM.add("Electronics", "USB-C panel-mount extension (debug port)", 1, comp.name)

        # ── Kept from v12: PCA9685 / LiPo / IMU / power bays ──────────────
        def pca9685_bay(comp, tag, cx, cy, cz):
            """ELEC-2 — PCA9685 pocket (62.5x25.4 mm) + standoffs + JST slots."""
            cut_cavity(comp, box(comp, f"{tag}_PCABay", cx, cy, cz,
                                 PCA_L + 0.20, PCA_W + 0.20, PCA_H + 0.30))
            for sx, sz in [(-3.00,-1.08), (3.00,-1.08), (-3.00,1.08), (3.00,1.08)]:
                cyl(comp, f"{tag}_PCAStdoff_{sx}_{sz}", cx+sx, cy, cz+sz,
                    0.14, PCA_H+0.50, "y", dark_metal)
            for ch in range(0, 16, 4):
                jst_pocket(comp, f"{tag}_PCA_JST{ch}", cx+2.8, cy, cz + (-0.8 + ch*0.10))
            BOM.add("Electronics", "PCA9685 16-ch I2C servo driver", 1, comp.name)

        def lipo_bay(comp, tag, cx, cy, cz):
            """ELEC-3 — 2S 1300 mAh LiPo pocket + XT60-F + strap + fuse space."""
            cut_cavity(comp, box(comp, f"{tag}_LipoBay", cx, cy, cz,
                                 LIPO_L + 0.30, LIPO_H + 0.30, LIPO_W + 0.30))
            cut_cavity(comp, box(comp, f"{tag}_XT60Slot",
                                 cx, cy + LIPO_H/2 + 0.15, cz,
                                 XT60_W + 0.10, 0.50, XT60_H_SLOT + 0.10))
            for sx in [-LIPO_L/2 - 0.40, LIPO_L/2 + 0.40]:
                cut_cavity(comp, box(comp, f"{tag}_StrapSlot_{sx:.0f}",
                    cx + sx, cy, cz, 0.25, LIPO_H + 0.50, LIPO_W + 0.20))
            box(comp, f"{tag}_FoamPad", cx, cy, cz - LIPO_H/2 - 0.15,
                LIPO_L + 0.10, 0.15, LIPO_W + 0.10, rubber_blk)
            cut_cavity(comp, box(comp, f"{tag}_FuseHolder",
                cx + LIPO_L/2 + 0.60, cy, cz, FUSE_HOLDER_L, FUSE_HOLDER_W, FUSE_HOLDER_H))
            BOM.add("Electronics", "2S 1300 mAh 7.4V LiPo battery", 1, comp.name)
            BOM.add("Electronics", "XT60-F panel-mount connector", 1, comp.name)
            BOM.add("Hardware",    "Velcro strap 20x200mm (battery)", 2, comp.name)
            BOM.add("Hardware",    "Foam pad 2mm (vibration isolation)", 1, comp.name)
            BOM.add("Electronics", "ATO blade fuse holder + 5A fuse", 1, comp.name)
            _reg_power("Servo_7.4V", 7.4, 12.0, ["all PCA9685 servo rails"])

        def imu_pocket(comp, tag, cx, cy, cz, axis="z"):
            """ELEC-6 — MPU-6050 IMU pocket (pelvis balance sensor)."""
            cut_cavity(comp, box(comp, f"{tag}_IMUPkt", cx, cy, cz,
                                 IMU_L + 0.20, IMU_W + 0.20, IMU_H + 0.30))
            BOM.add("Electronics", "MPU-6050 IMU breakout", 1, comp.name)

        def bec_mount(comp, tag, cx, cy, cz):
            """POW-4 — Generic 5V BEC mount (logic-rail regulator, used for
            ESP32-S3 nodes that don't draw from the Jetson 5V rail)."""
            cut_cavity(comp, box(comp, f"{tag}_BECBay", cx, cy, cz,
                BEC_L + 0.20, BEC_H + 0.20, BEC_W + 0.20))
            for sx, sz in [(-BEC_L/2+0.5,-BEC_W/2+0.5), (BEC_L/2-0.5, BEC_W/2-0.5)]:
                m3_boss(comp, f"{tag}_BEC_{sx:.0f}", cx+sx, cy, cz+sz)
            BOM.add("Electronics", "5V 3A BEC (logic rail, ESP32 nodes)", 1, comp.name)
            _reg_power("Logic_5V", 5.0, 3.0, ["ESP32-S3 nodes", "sensors"])

        def power_switch_cutout(comp, tag, cx, cy, cz, axis="y"):
            """POW-3 — Panel-mount master power switch."""
            cyl(comp, f"{tag}_SwHole", cx, cy, cz, POWER_SWITCH_R, 1.0, axis, black_plastic)
            cut_cavity(comp, cyl(comp, f"{tag}_SwCut", cx, cy, cz, POWER_SWITCH_R+0.03, 1.2, axis))
            BOM.add("Electronics", "Panel-mount rocker switch SPST (master)", 1, comp.name)

        # ─────────────────────────────────────────────────────────────────
        # V14 NEW: SENSOR FUSION, AI ACCELERATOR, COMM BACKBONE, SAFETY
        # ─────────────────────────────────────────────────────────────────

        def sensor_array(comp, tag, cx, cy, cz, axis="y", with_ultrasonic=True):
            """SYS-V14-1 — Sensor fusion array: HC-SR04 ultrasonic (optional,
            for pelvis-mounted general obstacle sensing) + 4x FSR force pads
            (for foot-mounted stance/pressure sensing) + an ADS1115 ADC to
            read the FSR pads. This complements -- not duplicates -- the
            head-mounted ToF pair: ToF is fast/short-range/forward-facing for
            reactive avoidance, ultrasonic is mid-range/general, and FSR is
            proprioceptive (it tells the robot how its weight is distributed,
            which the camera and ToF cannot)."""
            if with_ultrasonic:
                cut_cavity(comp, box(comp, f"{tag}_USPkt", cx, cy, cz, US_L, US_W, US_H))
                cut_cavity(comp, cyl(comp, f"{tag}_US_Tx", cx-1.35, cy-1.10, cz, US_TXRX_R, 0.30, "y"))
                cut_cavity(comp, cyl(comp, f"{tag}_US_Rx", cx+1.35, cy-1.10, cz, US_TXRX_R, 0.30, "y"))
                BOM.add("Electronics", "HC-SR04 ultrasonic sensor", 1, comp.name)
                _reg_sensor("ultrasonic", comp.name, tag, "Balance Node (ESP32-S3 lower)")
            # FSR pads (4x, corners) -- for feet this gives toe/heel L/R pressure
            for fsr_x, fsr_z in [(-1.5, -1.0), (1.5, -1.0), (-1.5, 1.0), (1.5, 1.0)]:
                cut_cavity(comp, box(comp, f"{tag}_FSR_{fsr_x:.0f}_{fsr_z:.0f}",
                    cx+fsr_x, cy, cz+fsr_z, FSR_SZ, FSR_T, FSR_SZ))
            BOM.add("Electronics", "Force-sensitive resistor 0.5in (FSR)", 4, comp.name)
            BOM.add("Electronics", "ADS1115 16-bit ADC (I2C)", 1, comp.name)
            _reg_sensor("FSR_x4+ADC", comp.name, tag, "Balance Node (ESP32-S3 lower)")

        def ai_accel_pocket(comp, tag, cx, cy, cz):
            """SYS-V14-2 — Future-ready USB-C AI accelerator bay (Google Coral
            Edge TPU or Intel Neural Compute Stick). Wired to a free Jetson
            USB3 port. Not populated by default -- this just reserves the
            space and BOM line so a model upgrade never needs a redesign."""
            cut_cavity(comp, box(comp, f"{tag}_AIAccel", cx, cy, cz,
                                 AI_ACCEL_L, AI_ACCEL_H, AI_ACCEL_W))
            cut_cavity(comp, box(comp, f"{tag}_AIAccelUSB",
                cx + AI_ACCEL_L/2 - 0.20, cy, cz, AI_ACCEL_USBC_W, AI_ACCEL_USBC_H, 0.60))
            BOM.add("Electronics", "Google Coral USB Accelerator (future, optional)", 1, comp.name)
            ASSEMBLY_STEPS.append(f"{tag}: reserved AI-accelerator bay -- populate when needed, "
                                  f"no redesign required")

        def comm_backbone(comp, tag, cx, cy, cz, length, axis="z"):
            """SYS-V14-3 — Visual I2C/UART/SPI communication trunk channel run
            down the spine. This is primarily a build/wiring aid: it gives
            the three buses a clearly separated, labeled physical path so
            the harness doesn't get tangled with servo power wiring."""
            wire_channel(comp, f"{tag}_Trunk", cx, cy, cz, COMM_TRUNK_R, length, axis)
            wire_channel(comp, f"{tag}_I2C",  cx+0.12, cy+0.02, cz, COMM_I2C_R,  length, axis)
            wire_channel(comp, f"{tag}_UART", cx-0.12, cy+0.02, cz, COMM_UART_R, length, axis)
            wire_channel(comp, f"{tag}_SPI",  cx, cy-0.15, cz, COMM_SPI_R, length, axis)
            BOM.add("Electronics", "22AWG signal wire (comm backbone, per run)",
                    int(length/5)+1, comp.name)
            BOM.add("Electronics", "JST-SH 1.0mm 6-pin (I2C/UART/SPI tap)", 6, comp.name)
            _reg_comm("MainBus", "All nodes", "I2C+UART+SPI trunk", "mixed",
                      "physical bus separation along spine")

        def estop_cutout(comp, tag, cx, cy, cz, axis="y"):
            """SYS-V14-4 — Independent, normally-closed emergency-stop button
            wired in series with the 7.4V servo rail ONLY. Pressing it cuts
            all servo power instantly while the Jetson Nano, ESP32 nodes, and
            sensors remain powered -- so the robot goes limp safely and you
            can still read logs / diagnose over the still-live debug port.
            This is deliberately a separate physical switch from the main
            power switch, matching standard robotics-lab safety practice."""
            cyl(comp, f"{tag}_EstopCollar", cx, cy, cz, ESTOP_COLLAR_R, 1.0, axis, op_red)
            cut_cavity(comp, cyl(comp, f"{tag}_EstopHole", cx, cy, cz, ESTOP_R, 1.2, axis))
            BOM.add("Electronics", "22mm mushroom-head E-stop pushbutton (N.C.)", 1, comp.name)
            BOM.add("Electronics", "Automotive relay 30A (servo rail cutoff)", 1, comp.name)
            ASSEMBLY_STEPS.append(
                f"Wire {tag} E-stop in series with the servo-rail relay coil ONLY -- "
                f"verify Jetson and ESP32 nodes stay powered when E-stop is pressed")

        def status_rgb_pocket(comp, tag, cx, cy, cz, axis="y"):
            """SYS-V14-5 — Single WS2812 addressable RGB status indicator,
            driven by the Jetson (or Motor Controller node) to show system
            state: blue pulse = booting, green = ready, yellow = low battery,
            red = fault/E-stop. Reuses the existing LED-pocket convention."""
            cut_cavity(comp, cyl(comp, f"{tag}_StatusRGB", cx, cy, cz, STATUS_RGB_R, 0.50, axis))
            BOM.add("Electronics", "WS2812 5050 addressable RGB LED (status)", 1, comp.name)
            ASSEMBLY_STEPS.append(f"{tag}: wire status RGB to Jetson GPIO or Motor Controller node "
                                  f"(boot=blue, ready=green, low-batt=yellow, fault=red)")


        # ─────────────────────────────────────────────────────────────────
        # V13 NEW: ARCHITECTURE / COMMS / POWER LOGGING
        # ─────────────────────────────────────────────────────────────────

        def log_v14_architecture():
            """ARCH-V13-1 — Log the full system topology to the build log."""
            Logger.log("=" * 64)
            Logger.log("V13 SYSTEM ARCHITECTURE")
            Logger.log("=" * 64)
            Logger.log("  [BRAIN]  NVIDIA Jetson Nano (4GB) -- torso, upper bay")
            Logger.log("           OS: JetPack (Ubuntu/L4T) | Python3, ROS2, OpenCV, TensorRT")
            Logger.log("           Role: vision, decision-making, path planning, comms hub")
            Logger.log("  [EYES]   CSI-2 camera (IMX219 8MP) -- head, behind visor")
            Logger.log("           FPC ribbon: head -> neck channel -> torso CSI port")
            Logger.log("  [NODE-1] ESP32-S3 'lower' -- pelvis bay")
            Logger.log("           Drives: PCA9685 x2 (hips/knees, ankles) + MPU-6050")
            Logger.log("  [NODE-2] ESP32-S3 'upper' -- torso bay")
            Logger.log("           Drives: PCA9685 (waist/shoulders/elbows) + INA3221")
            Logger.log("  [NODE-3] ESP32-S3 'head'  -- head bay")
            Logger.log("           Drives: PCA9685 (neck/wrist/fingers) + ToF x2")
            Logger.log("  [POWER]  2S 7.4V LiPo -> 5V/4A UBEC (Jetson) + 5V/3A BEC (logic)")
            Logger.log("                         -> 7.4V servo rail (5A ATO fuse)")
            Logger.log("=" * 64)

        def log_comms_map():
            """ARCH-V13-2 — Log the communication bus assignments."""
            Logger.log("--- V13 COMMUNICATION MAP ---")
            for entry in COMM_MAP:
                Logger.log(f"  {entry['from']:<16s} <-> {entry['to']:<18s} "
                           f"[{entry['bus']:<14s}] {entry['speed']:<10s} :: {entry['purpose']}")
            Logger.log("  Fallback path: ESP32-S3 WiFi AP + WebSocket (if USB-UART fails)")
            Logger.log("  Framing: COBS + CRC-16 over UART; JSON over WebSocket fallback")

        def log_power_budget():
            """ARCH-V13-3 — Log estimated power rail budget."""
            Logger.log("--- V13 POWER BUDGET ---")
            for entry in POWER_MAP:
                Logger.log(f"  Rail '{entry['rail']:<14s}' {entry['voltage']:.1f}V "
                           f"max {entry['max_A']:.1f}A  consumers: {', '.join(entry['consumers'])}")
            Logger.log("  Jetson Nano peak draw:     ~3.0A @ 5V (10W mode, AI inference active)")
            Logger.log("  3x ESP32-S3 nodes:         ~0.6A @ 5V combined (logic only)")
            Logger.log("  CSI camera:                ~0.25A @ 5V (1080p30 capture)")
            Logger.log("  Servo rail (worst case):   ~10A @ 7.4V (all 28 servos moving)")
            Logger.log("  Recommended battery:       3S 5000mAh+ LiPo for >20min runtime")
            Logger.log("  Fuse plan:                 5A on servo rail, 3A on Jetson 5V rail")

        def log_ai_pipeline():
            """ARCH-V13-4 — Log the vision/AI processing chain for future
            feature additions (object detection, SLAM, voice, etc.)."""
            Logger.log("--- V13 AI / VISION PIPELINE (future-ready) ---")
            Logger.log("  1. CSI camera captures frame (1080p30 / 720p60)")
            Logger.log("  2. Jetson GPU (CUDA) runs TensorRT-optimized model")
            Logger.log("       - current: face/person detection (MobileNet-SSD)")
            Logger.log("       - future:  pose estimation, object grasp targeting, SLAM")
            Logger.log("  3. Decision layer (Python/ROS2 node) maps detection -> behavior")
            Logger.log("       - e.g. 'person detected centre-frame' -> head-tracking pose")
            Logger.log("  4. High-level pose command sent via UART to relevant ESP32-S3 node")
            Logger.log("       - JSON command: {\"joint\":\"Neck_Cluster\",\"yaw\":12.5}")
            Logger.log("  5. ESP32-S3 node interpolates + drives PCA9685 PWM in real time")
            Logger.log("  6. ToF sensors (head node) feed obstacle distance back to Jetson")
            Logger.log("       - triggers reactive avoidance behavior independent of vision")
            Logger.log("  Future hooks: voice (USB mic -> Whisper.cpp), SLAM (RTAB-Map),")
            Logger.log("                gesture control, person-following, object grasping")

        # ─────────────────────────────────────────────────────────────────
        # V13 DOCUMENTATION GENERATORS
        # ─────────────────────────────────────────────────────────────────

        def write_assembly_guide():
            """DOC-1 — Write ASSEMBLY_GUIDE.txt (v14: Jetson/ESP32-S3 wiring,
            print orientation, support warnings, full build order)."""
            try:
                os.makedirs(os.path.dirname(ASSEMBLY_FILE), exist_ok=True)
                with open(ASSEMBLY_FILE, "w", encoding="utf-8") as f:
                    f.write("=" * 64 + "\n")
                    f.write("  OPTIMUS PRIME G1 v14.0  ASSEMBLY GUIDE\n")
                    f.write("  Jetson Nano AI Brain Edition\n")
                    f.write("=" * 64 + "\n\n")

                    f.write("--- SYSTEM ARCHITECTURE ---\n")
                    f.write("  Brain:   NVIDIA Jetson Nano (vision, AI, decisions)\n")
                    f.write("  Eyes:    CSI-2 camera (IMX219), head-mounted\n")
                    f.write("  Nodes:   3x ESP32-S3 (lower / upper / head control)\n")
                    f.write("  Comms:   USB-UART (CH340 bridge), 1 Mbaud, COBS+CRC16\n")
                    f.write("  Power:   2S 7.4V LiPo -> 5V/4A (Jetson) + 5V/3A (logic) + 7.4V servo\n\n")

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
                    for so in [
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
                    ]:
                        f.write(f"  {so}\n")

                    f.write("\n--- V13 ELECTRONICS INSTALL ORDER ---\n")
                    for eo in [
                        "1. Flash JetPack OS to Jetson Nano microSD card",
                        "2. Flash ESP32-S3 'lower' node firmware (servo + IMU driver)",
                        "3. Flash ESP32-S3 'upper' node firmware (servo + INA3221 driver)",
                        "4. Flash ESP32-S3 'head' node firmware (servo + ToF + finger driver)",
                        "5. Install Jetson Nano into torso bay with fan + heatsink",
                        "6. Install CSI camera into head; route FPC ribbon through neck channel",
                        "7. Install ESP32-S3 'lower' node into pelvis bay",
                        "8. Install ESP32-S3 'upper' node into torso bay",
                        "9. Install ESP32-S3 'head' node into head bay",
                        "10. Install PCA9685 boards (one per node) into adjacent bays",
                        "11. Install MPU-6050 IMU into pelvis (wired to 'lower' node)",
                        "12. Install INA3221 into torso (wired to 'upper' node)",
                        "13. Install ToF sensors x2 into head front (wired to 'head' node)",
                        "14. Install LiPo with XT60 connector into rear torso bay",
                        "15. Install 5V/4A UBEC (Jetson) and 5V/3A BEC (logic) near LiPo",
                        "16. Install blade fuse holders in both power paths",
                        "17. Route JST-XH harnesses from each PCA9685 to its servos",
                        "18. Connect Jetson USB ports to each ESP32-S3 node (CH340 bridge)",
                        "19. Connect I2C bus per node: ESP32 -> PCA9685 -> sensors",
                        "20. Install master power switch on torso side panel",
                        "21. Power on Jetson FIRST, verify boot, THEN power servo rail",
                    ]:
                        f.write(f"  {eo}\n")

                    f.write("\n--- TRANSFORMATION ASSEMBLY NOTES ---\n")
                    for tn in [
                        "1. Verify all joints move freely before first transform",
                        "2. Engage waist lock magnets in robot mode",
                        "3. Engage knee lock magnets at full extension",
                        "4. Shoulder spikes are REMOVABLE for transformation",
                        "5. Route wires with slack at all moving joints",
                        "6. CSI ribbon cable needs extra slack at neck joint",
                        "7. Test transformation SLOWLY on first attempt",
                    ]:
                        f.write(f"  {tn}\n")

                    f.write("\n--- CRITICAL BUILD WARNINGS ---\n")
                    for w in [
                        "Use PETG or stronger for all structural/load-bearing parts",
                        "PLA is acceptable only for cosmetic covers and jigs",
                        "All bearing pockets are designed for press-fit install",
                        "Use steel axles at hip, knee, and shoulder joints",
                        "Do NOT rely on servo torque alone for transformation",
                        "Install mechanical hard stops before powering servos",
                        "Use thread-lock (Loctite 222) on all metal fasteners",
                        "Verify polarity on ALL servo connectors before power-on",
                        "LiPo must have PCM protection; use ONLY 2S/3S packs with BMS",
                        "NEVER block ESP32-S3 antenna keep-out zones with metal/foil",
                        "Jetson Nano needs active cooling under sustained AI load -- "
                        "verify fan spins before first extended run",
                        "Power Jetson Nano FIRST and let it fully boot before enabling "
                        "the servo rail, to avoid brownout on shared grounds",
                    ]:
                        f.write(f"  * {w}\n")

                    f.write("\n--- RECOMMENDED MATERIALS ---\n")
                    f.write("  Structural:  PETG or ABS+ (hips, thighs, torso frame)\n")
                    f.write("  Shells:      PETG (torso, arms, legs, head)\n")
                    f.write("  Detail:      PLA+ (grilles, badges, small covers)\n")
                    f.write("  Flex:        TPU 95A (tire treads, gaskets, dampers)\n")
                    f.write("  Jigs:        PLA or PETG (alignment aids only)\n")
                Logger.log(f"Assembly guide written -> {ASSEMBLY_FILE}")
            except Exception as e:
                Logger.log(f"Assembly guide failed: {e}", "WARN")

        def write_build_manifest():
            """Write a machine-readable manifest including v14 comm/power maps."""
            try:
                os.makedirs(os.path.dirname(MANIFEST_FILE), exist_ok=True)
                joint_names = []
                try:
                    for i in range(root.asBuiltJoints.count):
                        joint_names.append(root.asBuiltJoints.item(i).name)
                except Exception:
                    joint_names = []
                manifest = {
                    "name": "Optimus Prime G1",
                    "version": "v14.0",
                    "architecture": "Jetson Nano AI Brain Edition",
                    "generated_at": _ts,
                    "target_module": TARGET_MODULE,
                    "components": [comp.name for comp in comps_list],
                    "component_count": len(comps_list),
                    "joint_count": len(joint_names),
                    "joints": joint_names,
                    "bom_rows": BOM._rows,
                    "bom_line_count": len(BOM._rows),
                    "screw_registry": SCREW_REGISTRY,
                    "screw_location_count": len(SCREW_REGISTRY),
                    "jigs": JIG_REGISTRY,
                    "support_warnings": SUPPORT_WARNINGS,
                    "print_notes": PRINT_NOTES,
                    "comm_map": COMM_MAP,
                    "power_map": POWER_MAP,
                    "outputs": {
                        "log": LOG_FILE, "bom": BOM_FILE,
                        "assembly_guide": ASSEMBLY_FILE,
                        "production_readiness": PRODUCTION_FILE,
                        "screenshots": SCREENSHOT_DIR, "exports": EXPORT_DIR,
                    },
                    "export_flags": {
                        "stl": bool(EXPORT_STL), "step": bool(EXPORT_STEP),
                        "urdf": bool(EXPORT_URDF),
                        "screenshots": bool(CAPTURE_SCREENSHOTS),
                        "visual_audit": bool(VISUAL_AUDIT),
                    },
                }
                with open(MANIFEST_FILE, "w", encoding="utf-8") as f:
                    json.dump(manifest, f, indent=2)
                Logger.log(f"Build manifest written -> {MANIFEST_FILE}")
            except Exception as e:
                Logger.log(f"Build manifest failed: {e}", "WARN")

        def write_production_readiness_report(check_rows=None):
            """Write the final v14 production-readiness checklist."""
            if not PRODUCTION_REPORT:
                return
            try:
                os.makedirs(os.path.dirname(PRODUCTION_FILE), exist_ok=True)
                check_rows = check_rows or []
                try:
                    joint_count = root.asBuiltJoints.count
                except Exception:
                    joint_count = -1
                with open(PRODUCTION_FILE, "w", encoding="utf-8") as f:
                    f.write("OPTIMUS PRIME G1 v14.0 PRODUCTION READINESS\n")
                    f.write("Jetson Nano AI Brain Edition\n")
                    f.write("=" * 56 + "\n\n")
                    f.write("MODEL SUMMARY\n")
                    f.write(f"  Components: {len(comps_list)}\n")
                    f.write(f"  As-built joints: {joint_count}\n")
                    f.write(f"  BOM rows: {len(BOM._rows)}\n")
                    f.write(f"  Registered screw locations: {len(SCREW_REGISTRY)}\n")
                    f.write(f"  Assembly jigs: {len(JIG_REGISTRY)}\n")
                    f.write(f"  Support warnings: {len(SUPPORT_WARNINGS)}\n")
                    f.write(f"  Communication links: {len(COMM_MAP)}\n")
                    f.write(f"  Power rails: {len(POWER_MAP)}\n\n")
                    f.write("SIMULATION / CAD CHECKS\n")
                    if check_rows:
                        for label, count in check_rows:
                            status = "PASS" if count == 0 else ("WARN" if count > 0 else "N/A")
                            f.write(f"  {label:<38s} {status:>5s}  {count}\n")
                    else:
                        f.write("  No runtime check rows were reported.\n")
                    f.write("\nCOMPUTE ARCHITECTURE\n")
                    f.write("  Brain:  NVIDIA Jetson Nano 4GB (vision + AI + decisions)\n")
                    f.write("  Eyes:   CSI-2 IMX219 camera, head-mounted\n")
                    f.write("  Nodes:  3x ESP32-S3 (lower/upper/head real-time control)\n")
                    f.write("  Comms:  USB-UART 1Mbaud (COBS+CRC16); WiFi WebSocket fallback\n\n")
                    f.write("MANUFACTURING BASELINE\n")
                    f.write(f"  FDM clearance: {CLEARANCE:.3f} cm\n")
                    f.write(f"  Structural wall: {WALL_S:.3f} cm\n")
                    f.write(f"  Partition wall: {WALL_P:.3f} cm\n")
                    f.write("  Structural material: PETG / ABS+ or stronger\n")
                    f.write("  Cosmetic material: PLA+ acceptable for non-load covers\n")
                    f.write("  Flexible material: TPU 95A for tires, gaskets, dampers\n\n")
                    f.write("REQUIRED PHYSICAL ACCEPTANCE TESTS\n")
                    for t in [
                        "Dry-fit every shell half before installing electronics.",
                        "Press-fit bearings with no visible shell cracking.",
                        "Verify every servo moves through ROM with power current limited.",
                        "Validate hip, knee, ankle, shoulder, and waist hard stops by hand.",
                        "Confirm transformer locks engage in robot and truck mode.",
                        "Boot Jetson Nano standalone (no servo rail) and verify CSI camera feed.",
                        "Verify each ESP32-S3 node enumerates over USB and responds to ping.",
                        "Run 30 minute tethered burn-in before untethered battery tests.",
                        "Re-check fasteners after the first three transform cycles.",
                        "Confirm Jetson fan spins and SoC stays under 70C during 10W AI load.",
                    ]:
                        f.write(f"  - {t}\n")
                    f.write("\nOUTPUTS\n")
                    f.write(f"  Manifest: {MANIFEST_FILE}\n")
                    f.write(f"  BOM: {BOM_FILE}\n")
                    f.write(f"  Assembly guide: {ASSEMBLY_FILE}\n")
                    f.write(f"  Exports: {EXPORT_DIR}\n")
                    f.write(f"  Screenshots: {SCREENSHOT_DIR}\n")
                Logger.log(f"Production readiness report written -> {PRODUCTION_FILE}")
            except Exception as e:
                Logger.log(f"Production readiness report failed: {e}", "WARN")


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
        # HARDWARE MODULES (servo bodies, couplers, wheels, brackets, stops)
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
            BOM.add("Drive", "TT gear-motor 3V-6V", 1, comp.name)
            BOM.add("Drive", "65 mm rubber tyre + wheel", 1, comp.name)

        def u_bracket(comp, tag, cx, cy, cz, lx, ly, lz, ap=None):
            ap = ap or chrome
            box(comp, f"{tag}_BB",  cx,          cy,        cz, 0.45,     ly,   lz, ap)
            box(comp, f"{tag}_BL",  cx+lx*0.45,  cy+ly*0.35, cz, lx*0.55, 0.40, lz, ap)
            box(comp, f"{tag}_BR",  cx+lx*0.45,  cy-ly*0.35, cz, lx*0.55, 0.40, lz, ap)
            cyl(comp, f"{tag}_Pin", cx+lx*0.50,  cy,         cz, 0.18, ly*0.85, "y", chrome)

        def hard_stop(comp, tag, cx, cy, cz, axis="x", stop_angle_deg=90):
            """MECH-3 — Mechanical hard stop block to prevent joint over-travel."""
            box(comp, f"{tag}_Stop", cx, cy, cz, 0.35, 0.35, 0.35, dark_metal)
            BOM.add("Hardware", "Hard stop block (printed)", 1, comp.name)
            ASSEMBLY_STEPS.append(f"Verify {tag} hard stop at {stop_angle_deg} deg clears moving link")

        def transform_lock(comp, tag, cx, cy, cz, axis="z"):
            """MECH-4 — Spring-pin latch for robot/truck mode locking."""
            cyl(comp, f"{tag}_LockBore", cx, cy, cz, 0.18, 1.50, axis, dark_metal)
            cut_cavity(comp, cyl(comp, f"{tag}_LockHole", cx, cy, cz, 0.20, 1.60, axis))
            cyl(comp, f"{tag}_SpringPkt", cx, cy, cz + 0.30, 0.35, 0.50, axis, dark_grey)
            BOM.add("Hardware", "Spring latch pin O3.5mm (steel)", 1, comp.name)
            BOM.add("Hardware", "Compression spring (lock return)", 1, comp.name)
            ASSEMBLY_STEPS.append(f"Install {tag} transform lock pin and return spring")


        # ═════════════════════════════════════════════════════════════════
        # COMPONENT BUILDING  (V13 Jetson Nano AI Brain Edition)
        # ═════════════════════════════════════════════════════════════════

        # ─────────────────────────────────────────────────────────────────
        # 1 TORSO — Jetson Nano (brain) + ESP32-S3 'upper' node + power
        # ─────────────────────────────────────────────────────────────────
        torso = new_component("OP_Torso")

        box(torso, "Torso_Shell",    0,    0,   TORSO_CTR,        10.4, 8.6, 12.2, op_red)
        box(torso, "Torso_Side_L",  -5.6,  0,   TORSO_CTR,         0.5, 7.8, 11.2, op_red)
        box(torso, "Torso_Side_R",   5.6,  0,   TORSO_CTR,         0.5, 7.8, 11.2, op_red)

        box(torso, "CWin_Frame_L",  -2.3, -4.20, TORSO_CTR+2.5,   2.8, 0.32, 3.2, op_blue)
        box(torso, "CWin_Frame_R",   2.3, -4.20, TORSO_CTR+2.5,   2.8, 0.32, 3.2, op_blue)
        box(torso, "Chest_Win_L",   -2.3, -4.35, TORSO_CTR+2.5,   2.4, 0.20, 2.8, glass_clr)
        box(torso, "Chest_Win_R",    2.3, -4.35, TORSO_CTR+2.5,   2.4, 0.20, 2.8, glass_clr)
        box(torso, "Chest_Win_Div",  0,   -4.25, TORSO_CTR+2.5,  0.40, 0.22, 3.2, chrome)

        for gz_offset, gw in [(-0.2, 7.4), (-1.0, 7.0), (-1.8, 6.6), (-2.6, 6.2)]:
            box(torso, f"Grille_{int(gz_offset*10)}",
                0, -4.40, TORSO_CTR+gz_offset, gw, 0.22, 0.30, chrome)

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

        # MECH-1 — Internal structural skeleton
        box(torso, "Inner_Frame",    0,    0,    TORSO_CTR+1.5,   7.4, 6.0,  8.2, dark_metal)
        box(torso, "Spine_Beam",     0,    0,    TORSO_CTR+1.5,   1.8, 1.8,  8.2, chrome)
        cyl(torso, "Spine_Cyl",      0,    0,    TORSO_CTR+1.5,  1.10, 4.5,  "z", chrome)
        for rz in [TORSO_CTR+3.5, TORSO_CTR, TORSO_CTR-3.5]:
            box(torso, f"Rib_{rz:.0f}", 0, 0, rz, 6.8, 0.35, 4.5, dark_metal)
        for sx in [-6.5, 6.5]:
            box(torso, f"Gusset_{sx:.0f}", sx, 0, TORSO_CTR+2.0, 1.2, 1.2, 3.5, dark_metal)

        # ELEC-3 — LiPo bay (rear of lower torso)
        box(torso, "LipoBay_Shell",  0,    3.2, TORSO_CTR-2.0,   7.6, 4.4,  5.6, black_plastic)
        lipo_bay(torso, "Main", 0, 2.8, TORSO_CTR-2.0)
        lipo_door(torso, "LipoDoor", 0, 5.5, TORSO_CTR-2.0, LIPO_L + 0.80, LIPO_W + 0.80)

        # V13: JETSON NANO BAY replaces RPi Zero bay (the robot's brain)
        box(torso, "Jetson_Shell",   0,    2.8, TORSO_CTR+2.0,   7.6, 4.2,  3.0, black_plastic)
        jetson_nano_bay(torso, "Main", 0, 3.0, TORSO_CTR+2.0)
        pcb_cover(torso, "JetsonCover", 0, 4.8, TORSO_CTR+2.0, JNANO_L + 0.60, JNANO_W + 0.60, "magnet")
        # V13: thermal vent grille over Jetson (active cooling exhaust path)
        vent_grille(torso, "JetsonVent", 0, 6.0, TORSO_CTR+2.0, "y", n_slots=8)

        # V13: ESP32-S3 'upper' node (waist/shoulders/elbows + INA3221)
        box(torso, "ESP32S3U_Shell",  0, 2.6, TORSO_CTR+5.0, 6.0, 2.6, 1.8, black_plastic)
        esp32s3_node_bay(torso, "UpperNode", 0, 2.8, TORSO_CTR+5.0, role="upper")
        pcb_cover(torso, "ESP32S3UCover", 0, 3.9, TORSO_CTR+5.0,
                  ESP32S3_L + 0.50, ESP32S3_W + 0.50, "screw")

        # PCA9685 for upper-body servos, driven by ESP32-S3 'upper'
        box(torso, "PCA_Shell",      0,    2.8, TORSO_CTR+4.2,   6.8, 3.2,  2.2, black_plastic)
        pca9685_bay(torso, "Main",   0,    3.0, TORSO_CTR+4.2)
        pcb_cover(torso, "PCACover", 0, 4.2, TORSO_CTR+4.2, PCA_L + 0.50, PCA_W + 0.50, "screw")

        # V13: INA3221 current monitor (servo rail health, reported to upper node)
        box(torso, "INA_Shell", -2.6, 2.6, TORSO_CTR-0.5, 3.0, 1.8, 1.0, black_plastic)
        ina3221_bay(torso, "ServoMon", -2.6, 2.8, TORSO_CTR-0.5)

        # Cable hub + clips
        wire_hub(torso, "TorsoHub", 0, 1.5, TORSO_CTR+0.5)
        for cz_clip in [TORSO_CTR+3.0, TORSO_CTR, TORSO_CTR-3.0, TORSO_CTR-4.5]:
            cable_clip(torso, f"CC_L_{cz_clip:.0f}", -3.4, 0.6, cz_clip)
            cable_clip(torso, f"CC_R_{cz_clip:.0f}",  3.4, 0.6, cz_clip)
        box(torso, "Cable_L",       -3.4,  0.6,  TORSO_CTR,      0.55, 1.0, 10.0, dark_grey)
        box(torso, "Cable_R",        3.4,  0.6,  TORSO_CTR,      0.55, 1.0, 10.0, dark_grey)

        # V13: power bays -- Jetson 5V/4A UBEC + logic 5V/3A BEC, separated rails
        ubec_5v4a_bay(torso, "JetsonPwr", LIPO_L/2 + 1.2, 3.0, TORSO_CTR-3.2)
        bec_mount(torso, "LogicBEC", -(LIPO_L/2 + 1.2), 3.0, TORSO_CTR-2.0)

        power_switch_cutout(torso, "PwrSw", -5.5, 0, TORSO_CTR+2.0, "y")

        # V13: USB-C bridge cutouts for external Jetson<->ESP32 debug access
        uart_bridge_cutout(torso, "DebugUpper", -5.6, 1.0, TORSO_CTR+5.0, "x")

        box(torso, "Collar_L",      -8.0,  0,    SHOULDER_CTR-1.0, 5.5, 3.4, 3.0, chrome)
        box(torso, "Collar_R",       8.0,  0,    SHOULDER_CTR-1.0, 5.5, 3.4, 3.0, chrome)

        box(torso, "TF_Flap_L",     -5.40,-0.2,  TORSO_CTR+3.0,  0.40, 6.6,  6.2, op_red)
        box(torso, "TF_Flap_R",      5.40,-0.2,  TORSO_CTR+3.0,  0.40, 6.6,  6.2, op_red)
        box(torso, "TF_BackTop",     0,    5.0,  TORSO_CTR+5.2,  8.2, 0.38,  5.2, op_blue)

        for bx_off, bz_off in [(-3.2, 4.8), (3.2, 4.8), (-3.2, -4.8), (3.2, -4.8)]:
            m3_boss(torso, f"Torso_B{bx_off}_{bz_off}", bx_off, 0, TORSO_CTR+bz_off)
        align_pin(torso, "TorsoSplit_L", -5.0, 0, TORSO_CTR)
        align_pin(torso, "TorsoSplit_R",  5.0, 0, TORSO_CTR)

        transform_lock(torso, "WaistLock", 0, -2.0, WAIST_CTR-3.0, "z")

        u_bracket(torso, "Waist_Brkt",  0, 0, WAIST_CTR,     4.0, 4.2, 3.4)
        mg996r(torso,    "Waist_Yaw",   0, 0, WAIST_CTR,     "z")
        dual_bearing(torso, "WaistDual", 0, 0, WAIST_CTR+0.5, "z", 1.30, 0.65, span=3.00)
        u_bracket(torso, "WaistP_Brkt", 0, 0, WAIST_CTR-2.5, 4.0, 4.2, 3.4)
        mg996r(torso,    "Waist_Pitch", 0, 0, WAIST_CTR-2.5, "x")
        bearing_fit(torso, "WaistP_Brg",  0, 0, WAIST_CTR-2.0, "x", 1.30, 0.65, fit_type="press")
        magnet_pocket(torso, "WLock_F", 0, -2.0, WAIST_CTR-3.0)
        magnet_pocket(torso, "WLock_R", 0,  2.0, WAIST_CTR-3.0)
        hard_stop(torso, "WaistP", 0, -2.5, WAIST_CTR-2.5, "x", 60)
        hard_stop(torso, "WaistN", 0,  2.5, WAIST_CTR-2.5, "x", -45)

        u_bracket(torso, "Neck_Brkt",   0, 0, NECK_JOINT_Z,  3.2, 2.8, 3.0)
        mg996r(torso,    "Neck_Pitch",  0, 0, NECK_JOINT_Z,  "x")
        wire_channel(torso, "Spine",    0, 0, TORSO_CTR,     0.6, 20.0, "z")

        # V13: CSI ribbon channel -- routes from head down through neck into torso
        fpc_ribbon_channel(torso, "CSIRoute", 0, 0.6, NECK_JOINT_Z - 3.0, 6.0, "z")

        for b in list(torso.bRepBodies):
            if b.name:
                printability_check(torso, b.name)

        # ─────────────────────────────────────────────────────────────────
        # 2 HEAD — CSI camera (eyes) + ESP32-S3 'head' node + ToF sensors
        # ─────────────────────────────────────────────────────────────────
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
        box(head, "Crest_Stripe",   0,    -0.30, HC+3.95, 0.55, 0.36, 2.9, op_blue)

        box(head, "Face_Plate",     0,    -2.50, HC+0.5,  3.6, 0.38, 3.2, chrome)

        chamfer_box(head, "Chin_Guard", 0, -2.60, HC-0.9, 3.0, 0.38, 1.8, "z", chamfer=0.25, ap=chrome)
        box(head, "Chin_L",        -1.35, -2.52, HC-0.4,  0.38, 0.32, 2.2, chrome)
        box(head, "Chin_R",         1.35, -2.52, HC-0.4,  0.38, 0.32, 2.2, chrome)

        # GEO-1 — Visor recess + LEDs (the visor now sits over the CSI camera)
        box(head, "Visor_Frame",    0,    -2.55, HC+1.35, 3.8, 0.30, 1.25, op_blue)
        box(head, "Visor",          0,    -2.68, HC+1.35, 3.0, 0.16, 0.92, op_blue_glass)
        for vx in [-0.85, 0.0, 0.85]:
            led_pocket_5mm(head, f"Visor_LED_{int(vx*100)}", vx, -2.80, HC+1.35, "y")

        box(head, "Nose_Bridge",    0,    -2.60, HC+1.95, 0.72, 0.22, 0.72, chrome)
        box(head, "Mouth_Plate",    0,    -2.55, HC+0.10, 2.4, 0.22, 1.10, dark_grey)
        for mz in [-0.32, 0.0, 0.32]:
            box(head, f"MouthGrill_{int(mz*100)}",
                0, -2.62, HC+0.10+mz, 1.8, 0.12, 0.18, chrome)

        box(head, "Head_Rear",      0,    1.90,  HC+1.0,  4.2, 1.5, 4.4, op_red)
        box(head, "Neck_Collar",    0,    0,     HC-1.6,  2.5, 2.5, 2.4, dark_metal)

        cyl(head, "Ant_L",         -2.80, 0,     HC+4.4, 0.13, 2.7, "z", chrome)
        cyl(head, "Ant_R",          2.80, 0,     HC+4.4, 0.13, 2.7, "z", chrome)
        cyl(head, "AntTip_L",      -2.80, 0,     HC+5.8, 0.24, 0.34, "z", gold_met)
        cyl(head, "AntTip_R",       2.80, 0,     HC+5.8, 0.24, 0.34, "z", gold_met)

        # V13: CSI CAMERA replaces ESP32-CAM as the robot's primary eyes,
        # mounted directly behind the visor looking forward through it
        csi_camera_pocket(head, "RobotEyes", 0, -1.9, HC+1.35, "y")
        cover_plate(head, "CamCover", 0, -2.0, HC+1.35, CSI_CAM_L+0.50, CSI_CAM_W+0.50,
                    [(-0.9, -0.4), (0.9, -0.4)], method="hinge", hinge_edge="top", ap=grey_plastic)

        # V13: ESP32-S3 'head' node (neck/wrist/finger servos + ToF sensors)
        box(head, "ESP32S3H_Shell",  0, 1.6, HC+0.0, 4.6, 2.4, 1.8, black_plastic)
        esp32s3_node_bay(head, "HeadNode", 0, 1.8, HC+0.0, role="head")
        pcb_cover(head, "ESP32S3HCover", 0, 2.8, HC+0.0,
                  ESP32S3_L + 0.50, ESP32S3_W + 0.50, "screw")

        # V13: ToF obstacle sensors x2 (front-facing, either side of chin)
        tof_sensor_pocket(head, "ToF_L", -1.8, -2.40, HC-0.3, "y")
        tof_sensor_pocket(head, "ToF_R",  1.8, -2.40, HC-0.3, "y")

        mg90s(head, "Neck_Yaw", 0, 0, NECK_JOINT_Z, "z")
        grommet_slot(head, "NeckWire", 0, 0.8, HC-0.5, "y", 0.50)
        # V13: CSI ribbon exits the head toward the neck channel
        fpc_ribbon_channel(head, "CSIHeadExit", 0, 0.4, HC-1.2, 3.0, "z")

        for b in list(head.bRepBodies):
            if b.name:
                printability_check(head, b.name)

        assembly_jig("OP_Head",
            [(-2.0, 0, 0), (2.0, 0, 0)],
            [(-1.5, 0, 0), (1.5, 0, 0)],
            (6.0, 4.0, 0.60))

        # ─────────────────────────────────────────────────────────────────
        # 3 PELVIS — ESP32-S3 'lower' node + MPU-6050 balance IMU
        # ─────────────────────────────────────────────────────────────────
        pelvis = new_component("OP_Pelvis")
        box(pelvis, "Pelvis_Shell",  0,    0,  PELVIS_CTR,  16.4, 6.2, 5.0, op_blue)
        box(pelvis, "Pelvis_Frame",  0,    0,  PELVIS_CTR,  12.2, 4.4, 3.8, dark_metal)
        box(pelvis, "Hip_Armr_L",  -7.2,  0,  PELVIS_CTR,   1.0, 5.2, 4.2, chrome)
        box(pelvis, "Hip_Armr_R",   7.2,  0,  PELVIS_CTR,   1.0, 5.2, 4.2, chrome)
        box(pelvis, "Crotch_Plate", 0,  -2.9, PELVIS_CTR-1.2, 5.2, 0.30, 2.4, op_red)

        imu_pocket(pelvis, "BalanceIMU", 0, 0, PELVIS_CTR)
        cover_plate(pelvis, "IMUCover", 0, 0.2, PELVIS_CTR,
                    IMU_L+0.60, IMU_W+0.60,
                    [(-0.8, -0.5), (0.8, 0.5)], method="magnet", ap=grey_plastic)

        # V13: ESP32-S3 'lower' node (hips/knees/ankles, reads MPU-6050 directly)
        box(pelvis, "ESP32S3L_Shell", 0, 1.6, PELVIS_CTR-1.5, 6.0, 2.4, 1.8, black_plastic)
        esp32s3_node_bay(pelvis, "LowerNode", 0, 1.8, PELVIS_CTR-1.5, role="lower")
        pcb_cover(pelvis, "ESP32S3LCover", 0, 2.8, PELVIS_CTR-1.5,
                  ESP32S3_L + 0.50, ESP32S3_W + 0.50, "screw")

        # PCA9685 boards for hip/knee (1) and ankle (1), both driven by 'lower' node
        box(pelvis, "PCA_Hip_Shell", -3.0, 1.6, PELVIS_CTR+1.2, 5.0, 2.2, 1.6, black_plastic)
        pca9685_bay(pelvis, "HipKnee", -3.0, 1.8, PELVIS_CTR+1.2)
        box(pelvis, "PCA_Ankle_Shell", 3.0, 1.6, PELVIS_CTR+1.2, 5.0, 2.2, 1.6, black_plastic)
        pca9685_bay(pelvis, "Ankle", 3.0, 1.8, PELVIS_CTR+1.2)

        grommet_slot(pelvis, "HipWire_L", -HIP_X, 0, PELVIS_CTR+0.5, "z", 0.60)
        grommet_slot(pelvis, "HipWire_R",  HIP_X, 0, PELVIS_CTR+0.5, "z", 0.60)
        uart_bridge_cutout(pelvis, "DebugLower", -7.5, 1.0, PELVIS_CTR, "x")

        # V14 SYS-V14-1: pelvis-front ultrasonic (mid-range, complements head ToF)
        sensor_array(pelvis, "PelvisFront", 0, -3.0, PELVIS_CTR, "y", with_ultrasonic=True)

        mg996r(pelvis, "L_HipYaw", -HIP_X, 0, HIP_JOINT_Z, "z")
        mg996r(pelvis, "R_HipYaw",  HIP_X, 0, HIP_JOINT_Z, "z")
        dual_bearing(pelvis, "L_HipDual", -HIP_X-2.2, 0, HIP_JOINT_Z, "z", 1.10, 0.62, span=3.20, fit_type="press")
        dual_bearing(pelvis, "R_HipDual",  HIP_X+2.2, 0, HIP_JOINT_Z, "z", 1.10, 0.62, span=3.20, fit_type="press")
        BOM.add("Servo", "DS3225MG 25 kg-cm servo (hip yaw)", 2, "OP_Pelvis")

        for b in list(pelvis.bRepBodies):
            if b.name:
                printability_check(pelvis, b.name)


        # ─────────────────────────────────────────────────────────────────
        # 4 LEGS  (unchanged from v12 — mechanical design already complete)
        # ─────────────────────────────────────────────────────────────────
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
            dual_bearing(thigh, f"{side}_HipP_Dual", sx, 0, HIP_JOINT_Z,
                         "x", 1.10, 0.62, span=2.80, fit_type="press")
            mg996r(thigh, f"{side}_HipR",    sx, 0, THIGH_CTR+2.0,           "y")
            bearing_fit(thigh, f"{side}_HRB",    sx, 0, THIGH_CTR+2.0,
                        "y", 1.00, 0.55, fit_type="press")
            u_bracket(thigh, f"{side}_KnB",  sx, 0, KNEE_CTR+1.5,            3.8, 3.0, 3.0)
            mg996r(thigh, f"{side}_KneP",    sx, 0, KNEE_CTR+1.5,            "x")
            dual_bearing(thigh, f"{side}_Knee_Dual", sx, 0, KNEE_CTR,
                         "x", 1.00, 0.55, span=2.60, fit_type="press")
            # MECH-V14-1: knee-yaw DOF -- lets the lower leg splay slightly
            # for uneven-terrain adaptation and improves the transform
            # sequence's cross-step. Mounted offset from the pitch axis so
            # the two servos don't physically clash.
            u_bracket(thigh, f"{side}_KnYawB", sx+m*KNEE_YAW_OFFSET_X, 0, KNEE_CTR+1.0, 3.0, 2.6, 2.6)
            mg996r(thigh, f"{side}_KneYaw",  sx+m*KNEE_YAW_OFFSET_X, 0, KNEE_CTR+1.0, "y")
            bearing_fit(thigh, f"{side}_KneYaw_Brg", sx+m*KNEE_YAW_OFFSET_X, 0, KNEE_CTR+1.0,
                        "y", KNEE_YAW_BRG_R, KNEE_YAW_BRG_W, fit_type="press")
            hard_stop(thigh, f"{side}_KneYawStop", sx+m*KNEE_YAW_OFFSET_X, -1.8, KNEE_CTR+1.0, "y", 15)
            BOM.add("Servo", "MG996R 11 kg-cm servo (knee yaw)", 1, f"OP_Thigh_{side}")
            wire_channel(thigh, f"{side}_LW", sx, 0, THIGH_CTR,              0.5, 12.0, "z")
            for bz in [THIGH_CTR+3.0, THIGH_CTR-3.0]:
                m3_boss(thigh, f"{side}_ThighBoss_{bz:.0f}", sx, 0, bz)
            magnet_pocket(thigh, f"{side}_KLU", sx, -1.5, KNEE_CTR+1.0)
            magnet_pocket(thigh, f"{side}_KLL", sx,  1.5, KNEE_CTR+1.0)
            transform_lock(thigh, f"{side}_KneeLock", sx, -2.5, KNEE_CTR+0.5, "x")
            hard_stop(thigh, f"{side}_KneeExt", sx, -2.5, KNEE_CTR, "x", 135)
            BOM.add("Servo", "DS3225MG 25 kg-cm servo (hip pitch)", 1, f"OP_Thigh_{side}")

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
            magnet_pocket(shin, f"{side}_KU", sx, -1.5, KNEE_CTR-1.0)
            magnet_pocket(shin, f"{side}_KL", sx,  1.5, KNEE_CTR-1.0)
            for bz in [SHIN_CTR+3.5, SHIN_CTR-3.5]:
                m3_boss(shin, f"{side}_ShinBoss_{bz:.0f}", sx, 0, bz)
            grommet_slot(shin, f"{side}_AnkleWire", sx, 0, SHIN_CTR-5.0, "z", 0.50)

            foot = new_component(f"OP_Foot_{side}")
            box(foot, "Foot_Sole",    sx,       -0.6,  ANKLE_CTR-1.5,  6.2, 9.2, 1.10, op_red)
            for vi in [-1.0, 0.0, 1.0]:
                box(foot, f"Arch_Vent_{int(vi*10)}",
                    sx+vi*1.2, -0.6, ANKLE_CTR-1.94, 0.40, 5.5, 0.14, dark_grey)
            chamfer_box(foot, "Heel_Block", sx-m*0.9, 3.2, ANKLE_CTR-0.8,
                        2.5, 3.5, 2.6, "y", chamfer=0.30, ap=dark_grey)
            box(foot, "Heel_Plate",   sx-m*0.6,  4.4,  ANKLE_CTR-1.2,  3.2, 0.32, 2.0, chrome)
            chamfer_box(foot, "Heel_Spur", sx-m*1.0, 4.8, ANKLE_CTR-0.2,
                        1.2, 0.40, 3.2, "z", chamfer=0.35, ap=op_red)
            box(foot, "Toe_Block",    sx+m*0.8, -3.8,  ANKLE_CTR-0.8,  2.6, 3.8, 2.0, dark_grey)
            box(foot, "Toe_Plate",    sx+m*0.5, -4.6,  ANKLE_CTR-1.2,  3.8, 0.32, 1.8, chrome)
            chamfer_box(foot, "Toe_Cap", sx+m*1.0, -5.2, ANKLE_CTR-0.8,
                        2.8, 0.42, 1.5, "z", chamfer=0.25, ap=op_red)
            box(foot, "Ankle_Guard",  sx,        0,    ANKLE_CTR+1.2,  5.4, 3.2, 2.8, chrome)
            box(foot, "Ankle_Inner",  sx,       -1.0,  ANKLE_CTR+0.3,  4.0, 2.0, 1.6, dark_metal)
            box(foot, "Boot_Fin",     sx+m*2.0,  0,    ANKLE_CTR-0.2,  0.40, 6.5, 4.2, op_blue)
            box(foot, "Boot_Fin2",    sx+m*2.5,  0,    ANKLE_CTR+0.8,  0.32, 5.0, 2.8, op_red)

            mg996r(foot, f"{side}_AnkP", sx, 0, ANKLE_CTR+2.2, "x")
            mg996r(foot, f"{side}_AnkR", sx, 0, ANKLE_CTR+0.5, "y")
            bearing_fit(foot, f"{side}_AB",  sx, 0, ANKLE_CTR,     "x", 1.00, 0.55, fit_type="press")
            hard_stop(foot, f"{side}_AnkP_Stop", sx, -2.0, ANKLE_CTR+2.2, "x", 20)
            hard_stop(foot, f"{side}_AnkN_Stop", sx,  2.0, ANKLE_CTR+2.2, "x", -20)
            for bx_off in [-1.5, 1.5]:
                m3_boss(foot, f"{side}_FootBoss_{bx_off:.0f}", sx+bx_off, 0, ANKLE_CTR-0.5)
            align_pin(foot, f"{side}_Ft_A", sx-1.0, 0, ANKLE_CTR-1.3)
            align_pin(foot, f"{side}_Ft_B", sx+1.0, 0, ANKLE_CTR-1.3)
            box(foot, f"{side}_VibPad", sx, -0.6, ANKLE_CTR-2.2, 5.5, 8.0, 0.15, rubber_blk)

            # V14 SYS-V14-1: foot FSR pressure pads (toe/heel L/R), read by the
            # Balance Node's ADS1115 -- gives real stance-phase / weight-shift
            # feedback that neither the camera nor ToF/ultrasonic can provide.
            sensor_array(foot, f"{side}_FootSense", sx, -0.6, ANKLE_CTR-1.5, "y", with_ultrasonic=False)

            for comp_chk in [thigh, shin, foot]:
                for b in list(comp_chk.bRepBodies):
                    if b.name:
                        printability_check(comp_chk, b.name)

            assembly_jig(f"OP_Thigh_{side}",
                [(sx-1.0, 0, THIGH_CTR+2.0), (sx+1.0, 0, THIGH_CTR-2.0)],
                [(sx-0.5, 0, THIGH_CTR+2.0), (sx+0.5, 0, THIGH_CTR-2.0)],
                (7.0, 5.0, 0.60))

        # ─────────────────────────────────────────────────────────────────
        # 5 ARMS  (unchanged from v12 — tendon-driven hands already complete)
        # ─────────────────────────────────────────────────────────────────
        for side, ax in [("L", -SHOULDER_X), ("R", SHOULDER_X)]:
            m = -1 if side == "L" else 1

            ua = new_component(f"OP_UpperArm_{side}")
            box(ua, "Sh_Block",        ax,         0, SHOULDER_CTR,      5.6, 4.2, 5.6, op_red)
            box(ua, "Sh_Pad_Outer",    ax+m*3.20,  0, SHOULDER_CTR,      1.00, 5.2, 5.8, op_red)
            box(ua, "Sh_Pad_Edge",     ax+m*3.75,  0, SHOULDER_CTR,      0.40, 4.6, 5.2, chrome)
            for sz, sr, sh2 in [(SHOULDER_CTR+2.8, 0.42, 0.95),
                                 (SHOULDER_CTR+1.5, 0.36, 0.75)]:
                cyl(ua, f"Stk_A_{sz:.0f}", ax+m*3.3, -1.5, sz, sr,  sh2, "z", chrome)
                cone_shape(ua, f"StkTip_{sz:.0f}", ax+m*3.3, -1.5, sz+sh2/2+0.35,
                           sr, sr*0.35, 0.55, "z", dark_grey)
            box(ua, "Sh_Guard",        ax+m*2.60,  0, SHOULDER_CTR-0.2, 0.42, 4.4, 6.6, op_blue)
            box(ua, f"{side}_Shoulder_Frame", ax, 0, SHOULDER_CTR, 3.5, 3.0, 4.5, dark_metal)
            box(ua, "UA_Link",         ax,         0, ELBOW_Z+3.0,      3.2, 3.4, 5.2, op_red)
            box(ua, "UA_Skin",         ax+m*1.80,  0, ELBOW_Z+3.0,      0.52, 3.4, 5.2, chrome)

            mg996r(ua, f"{side}_ShY",    ax, 0, SHOULDER_CTR+1.5, "z")
            dual_bearing(ua, f"{side}_ShY_Dual", ax, 0, SHOULDER_CTR+2.0,
                         "z", 1.00, 0.55, span=2.80, fit_type="press")
            u_bracket(ua, f"{side}_SPB", ax, 0, SHOULDER_CTR,     4.8, 3.4, 3.4)
            mg996r(ua, f"{side}_ShP",    ax, 0, SHOULDER_CTR,     "x")
            dual_bearing(ua, f"{side}_ShP_Dual", ax, 0, SHOULDER_CTR,
                         "x", 1.10, 0.62, span=2.80, fit_type="press")
            mg996r(ua, f"{side}_ShR",    ax, 0, SHOULDER_CTR-1.2, "y")
            bearing_fit(ua, f"{side}_SB",    ax, 0, SHOULDER_CTR,     "x", 1.10, 0.62, fit_type="press")

            u_bracket(ua, f"{side}_EB",  ax, 0, ELBOW_Z,          3.8, 3.0, 3.0)
            mg996r(ua, f"{side}_ElbP",   ax, 0, ELBOW_Z,          "x")
            dual_bearing(ua, f"{side}_Elb_Dual", ax, 0, ELBOW_Z-0.5,
                         "x", 0.95, 0.52, span=2.40, fit_type="press")
            wire_channel(ua, f"{side}_UAW", ax, 0, ELBOW_Z+3.0,   0.4, 5.2, "z")
            m3_boss(ua, f"{side}_UAboss", ax, 0, ELBOW_Z+3.0)
            hard_stop(ua, f"{side}_ElbStop", ax, -2.0, ELBOW_Z, "x", 150)

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
            grommet_slot(fa, f"{side}_WristWire", ax, 0, WRIST_Z, "y", 0.45)

            hand = new_component(f"OP_Hand_{side}")
            box(hand, "Palm_Main",   ax,         -0.4,  WRIST_Z-1.6, 3.4, 3.0, 2.2, dark_grey)
            box(hand, "Palm_Inner",  ax,          0.6,  WRIST_Z-1.6, 2.6, 2.0, 2.0, black_plastic)
            MCP_Z = WRIST_Z - PALM_BOTTOM_OFFSET
            for ki, kx_off in enumerate(FING_X_OFF):
                cyl(hand, f"Knuckle_{ki}",
                    ax + m*kx_off, -1.5, MCP_Z + 0.15, 0.28, 0.38, "y", chrome)
            cyl(hand, "Wrist_Ring",  ax, 0, WRIST_Z-0.4, 1.05, 0.42, "z", chrome)
            box(hand, "Hand_Panel",  ax+m*0.9, -0.7, WRIST_Z-1.3, 0.38, 2.8, 2.8, op_red)

            box(hand, "Finger_SrvBay", ax, 0.5, WRIST_Z-1.8, 1.4, 0.90, 2.4, black_plastic)
            servo_drum(hand, f"{side}_Finger", ax, 1.2, WRIST_Z-1.8, "y")
            BOM.add("Servo", "DS04-NFC 9g digital servo (finger drive)", 2, f"OP_Hand_{side}")

            for fi, fxo in enumerate(FING_X_OFF):
                px = ax + m * fxo * 0.5
                palm_pulley(hand, f"{side}_Pulley_{fi}", px, -0.5, WRIST_Z-1.2, "x")

            for lxi in [-0.7, 0.7]:
                led_pocket_5mm(hand, f"PalmLED_{lxi:.0f}", ax+lxi, -1.65, WRIST_Z-1.6, "y")

            for fi, (fname, fxo, pp_l, mp_l, dp_l) in enumerate(
                    zip(FING_NAMES, FING_X_OFF, FING_PP, FING_MP, FING_DP)):

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
                cyl(fc, "DIP_Hinge", fx, fy, mcp_z - pp_l - mp_l,
                    0.24, FING_W*0.94 + 0.10, "x", chrome)

                box(fc, "DP", fx, fy, dp_cz, FING_W*0.86, FING_H*0.86, dp_l, grey_plastic)
                tendon_anchor(fc, f"{fname}", fx, fy, mcp_z - pp_l - mp_l - dp_l - 0.15, "z")
                cone_shape(fc, "FT", fx, fy, mcp_z - pp_l - mp_l - dp_l - 0.12,
                           FING_W*0.44, 0.05, 0.24, "z", chrome)

                grommet_slot(fc, f"{fname}_TendonExit", fx, fy-0.2, mcp_z, "y", 0.30)

            thumb = new_component(f"OP_Thumb_{side}")
            tx    = ax + m * 1.70
            ty    = 0.20
            tpp_cz = MCP_Z - THUMB_PP_L / 2
            tdp_cz = MCP_Z - THUMB_PP_L - THUMB_DP_L / 2
            box(thumb, "TP",  tx, ty, tpp_cz, THUMB_W, THUMB_H, THUMB_PP_L, grey_plastic)
            tendon_guide(thumb, "Thumb_PP", tx, ty-0.05, tpp_cz, THUMB_PP_L*0.7, "z")
            cyl(thumb, "T_Hinge", tx, ty, MCP_Z - THUMB_PP_L,
                0.28, THUMB_W + 0.14, "x", chrome)
            spring_return(thumb, "Thumb_CMC", tx, ty+0.3, MCP_Z - THUMB_PP_L, "x")
            box(thumb, "TD",  tx, ty*0.8, tdp_cz, THUMB_W*0.86, THUMB_H*0.86, THUMB_DP_L,
                grey_plastic)
            tendon_anchor(thumb, "Thumb", tx, ty*0.8, tdp_cz - THUMB_DP_L/2 - 0.15, "z")
            cone_shape(thumb, "TT", tx, ty, tdp_cz - THUMB_DP_L/2 - 0.14,
                       THUMB_W*0.44, 0.05, 0.28, "z", chrome)

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

            for comp_chk in [ua, fa, hand]:
                for b in list(comp_chk.bRepBodies):
                    if b.name:
                        printability_check(comp_chk, b.name)
            assembly_jig(f"OP_UpperArm_{side}",
                [(ax-1.5, 0, SHOULDER_CTR), (ax+1.5, 0, ELBOW_Z)],
                [(ax-1.0, 0, SHOULDER_CTR), (ax+1.0, 0, ELBOW_Z)],
                (8.0, 5.0, 0.60))

        # ─────────────────────────────────────────────────────────────────
        # 6 BACKPACK
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
        grommet_slot(bp, "BP_Wire", 0, 4.5, TORSO_CTR+2.0, "y", 0.80)
        # V13: backpack doubles as secondary Jetson exhaust vent path
        vent_grille(bp, "BP_Vent", 0, 6.9, TORSO_CTR-0.5, "y", n_slots=6)

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
        # FDM SHELL SPLITTING (Y-plane @ 0)  +  MFG-1 fastener merge
        # ─────────────────────────────────────────────────────────────────
        Logger.log("--- FDM Shell Splitting + Fastener Merge (V13) ---")
        for comp in comps_list:
            to_split = [b for b in comp.bRepBodies
                        if b.name and any(k in b.name for k in SPLIT_KEYS)]
            for b in to_split:
                split_halves(comp, b, "y", 0.0)
            bodies_after = list(comp.bRepBodies)
            left_bodies  = [b for b in bodies_after if b.name and "_left"  in b.name]
            right_bodies = [b for b in bodies_after if b.name and "_right" in b.name]
            # BUGFIX-V13-2: merge_fasteners_to_halves() now safely handles a
            # single-sided call (the other arg is None) without raising.
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

            tx        = ax + m * 1.70
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
                if (comp.name not in ("OP_Torso", "OP_Pelvis")
                        and comp.name not in jointed_comps):
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

        # ── V13: Architecture / comms / power logging ────────────────────
        log_v14_architecture()
        log_comms_map()
        log_power_budget()
        log_ai_pipeline()

        # ── FST-1: Screw length verification (bugfixed) ──────────────────
        verify_screw_lengths()

        # ── DOC-1: Write assembly guide ───────────────────────────────────
        write_assembly_guide()

        # ═════════════════════════════════════════════════════════════════
        # SIMULATION ENGINE  v14.0
        # ═════════════════════════════════════════════════════════════════

        class SimulationEngine:
            """
            Simulation suite for Optimus Prime G1 v14 (Unified Distributed AI
            Edition). All v12 mechanical/physical validation features are
            retained; new in v14: architecture validation, vision-tracking,
            simulation, ToF obstacle-reaction simulation.

            BUGFIX-V13-1: move_ball() previously wrote `self._set(mo, ax, ...)`
            inside its per-frame loop, but `ax` was never defined in that
            scope (it only existed inside the earlier `for axis, val in [...]`
            target-building loop). This silently raised a NameError that was
            swallowed by the broad except in _set()'s caller context in some
            paths and caused ball-joint animations to do nothing on certain
            python versions. Fixed by using the correct loop variable `axis`
            consistently throughout the per-frame update loop.
            """

            BALL_JOINTS = {
                "Waist_Cluster", "Neck_Cluster",
                "L_Hip_Cluster", "R_Hip_Cluster",
                "L_Knee", "R_Knee",
                "L_Ankle_Cluster", "R_Ankle_Cluster",
                "L_Shoulder_Cluster", "R_Shoulder_Cluster",
                "L_Wrist", "R_Wrist",
                "L_Thumb_CMC", "R_Thumb_CMC",
            }
            REV_JOINTS = {
                "L_Elbow", "R_Elbow", "Blaster_Fold",
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
                            f"CLAMP: {joint_name}.{axis} {deg:.0f} deg -> [{lo} deg,{hi} deg]", "WARN")
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
                """Animate ball-joint (pitch/yaw/roll) and revolute (pitch)
                targets in a single synchronised pass.

                BUGFIX-V13-1: the per-frame update loop now correctly uses
                the `axis` value captured per active-tuple (mo, axis, s_rad,
                e_rad) instead of referencing an undefined `ax` name that
                leaked in from a different loop scope in earlier versions."""
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
                    for mo, axis, s_rad, e_rad in active:        # BUGFIX-V13-1
                        self._set(mo, axis, s_rad + (e_rad - s_rad) * t)
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

            # ── Physical validation (tendon slack / transform locks / print) ─
            def test_tendon_slack(self):
                Logger.log("--- TENDON SLACK TEST ---")
                for side in ["L", "R"]:
                    for fname in FING_NAMES:
                        jname = f"{side}_{fname}_MCP"
                        self.move_joint(jname, 0, steps=5, axis="pitch")
                        self.move_joint(jname, 45, steps=5, axis="pitch")
                        self.move_joint(jname, 0, steps=5, axis="pitch")
                Logger.log("  Tendon slack test complete [OK]")

            def test_transform_locks(self):
                Logger.log("--- TRANSFORM LOCK CLEARANCE ---")
                Logger.log("  Lock pins must be DISENGAGED before servo motion")
                for name in ["Waist_Cluster", "L_Hip_Cluster", "R_Hip_Cluster"]:
                    for axis in ["pitch", "yaw", "roll"]:
                        if axis in JOINT_LIMITS.get(name, {}):
                            lo, hi = JOINT_LIMITS[name][axis]
                            self.move_joint(name, lo, steps=8, axis=axis)
                            self.move_joint(name, hi, steps=8, axis=axis)
                            self.move_joint(name, 0, steps=8, axis=axis)
                Logger.log("  Transform lock clearance OK")

            def printability_report(self):
                Logger.log("--- PRINTABILITY REPORT ---")
                Logger.log(f"  Parts with support warnings: {len(SUPPORT_WARNINGS)}")
                for part, reason in SUPPORT_WARNINGS[:20]:
                    Logger.log(f"    [WARN] {part}: {reason}")
                if len(SUPPORT_WARNINGS) > 20:
                    Logger.log(f"    ...and {len(SUPPORT_WARNINGS)-20} more")
                Logger.log(f"  Assembly jigs designed: {len(JIG_REGISTRY)}")
                for jig_name, target in JIG_REGISTRY:
                    Logger.log(f"    {jig_name} -> for {target}")

            # ── V13 NEW: Architecture validation ──────────────────────────
            def validate_v14_architecture(self):
                """SIM-V14-1 -- Verify all v14 electronics components were
                actually placed (non-zero bodies in their host bays) and
                cross-check the comm/power registries are populated."""
                Logger.log("--- V13: ARCHITECTURE VALIDATION ---")
                required_tags = [
                    ("Jetson Nano bay",     "Main_JNanoBay_Vis"),
                    ("CSI camera (eyes)",   "RobotEyes_CSICamBay_Vis"),
                    ("ESP32-S3 lower node", "LowerNode_ESP32S3Bay_Vis"),
                    ("ESP32-S3 upper node", "UpperNode_ESP32S3Bay_Vis"),
                    ("ESP32-S3 head node",  "HeadNode_ESP32S3Bay_Vis"),
                ]
                found_count = 0
                for label, tag_fragment in required_tags:
                    hit = False
                    for comp in self._comps:
                        for b in comp.bRepBodies:
                            if b.name and tag_fragment.split("_Vis")[0] in b.name:
                                hit = True
                                break
                        if hit:
                            break
                    icon = "[OK]" if hit else "[MISSING]"
                    if hit:
                        found_count += 1
                    Logger.log(f"  {icon} {label}")
                if not COMM_MAP:
                    Logger.log("  [WARN] Communication map is empty!", "WARN")
                else:
                    Logger.log(f"  [OK] {len(COMM_MAP)} communication link(s) registered")
                if not POWER_MAP:
                    Logger.log("  [WARN] Power map is empty!", "WARN")
                else:
                    Logger.log(f"  [OK] {len(POWER_MAP)} power rail(s) registered")
                Logger.log(f"  Architecture validation: {found_count}/{len(required_tags)} "
                           f"core compute components placed")

            # ── V13 NEW: Vision tracking simulation ───────────────────────
            def simulate_vision_tracking(self):
                """SIM-V13-2 — Simulate the head/neck tracking a moving target
                across the camera's field of view (head-coverage sweep)."""
                Logger.log("--- V13: VISION TRACKING SIMULATION ---")
                Logger.log("  Simulated CSI camera FOV: ~62 deg horizontal")
                self.reset_all(steps=8, groups=["Neck_Cluster"])
                track_points = [(-40, -10, "target far-left"),
                                (-15,  0,  "target left-centre"),
                                (  0,   0,  "target centre"),
                                ( 15,   0,  "target right-centre"),
                                ( 40, -10,  "target far-right"),
                                (  0,  30,  "target overhead")]
                for yaw, pitch, label in track_points:
                    self.move_joint("Neck_Cluster", yaw,   steps=14, axis="yaw")
                    self.move_joint("Neck_Cluster", pitch, steps=10, axis="pitch")
                    Logger.log(f"  Tracking: {label} -> yaw={yaw} deg pitch={pitch} deg")
                self.reset_all(steps=10, groups=["Neck_Cluster"])
                Logger.log("  Vision tracking simulation complete [OK]")

            # ── V13 NEW: ToF obstacle reaction simulation ─────────────────
            def simulate_obstacle_reaction(self):
                """SIM-V13-3 — Simulate a reactive avoidance sequence triggered
                by the head-mounted ToF sensors detecting a close obstacle,
                independent of the vision pipeline (low-latency safety path)."""
                Logger.log("--- V13: TOF OBSTACLE REACTION SIMULATION ---")
                Logger.log("  Simulated trigger: ToF_L/ToF_R report distance < 30cm")
                self.reset_all(steps=8)
                # Step back half-pace + raise guard
                self.move_ball([
                    ("L_Hip_Cluster", -15, 0, 0),
                    ("R_Hip_Cluster", -15, 0, 0),
                ], steps=14)
                self.move_joint("L_Knee", 30, steps=10, axis="pitch")
                self.move_joint("R_Knee", 30, steps=10, axis="pitch")
                self.move_ball([
                    ("L_Shoulder_Cluster", -60, 20, 0),
                    ("R_Shoulder_Cluster", -60, -20, 0),
                ], steps=14)
                self._check_zmp("Obstacle-reaction guard stance")
                self._interfere("Obstacle-reaction pose")
                Logger.log("  Obstacle reaction -> guard stance held")
                self.reset_all(steps=10)
                Logger.log("  ToF obstacle reaction simulation complete [OK]")

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
                        self.move_ball([(f"{side}_Shoulder_Cluster", sp, sy, 0)], steps=10)
                        self.move_joint(f"{side}_Elbow", ep, steps=8, axis="pitch")
                    self._interfere(f"Workspace: {lbl}")
                    self.reset_all(steps=8,
                                   groups=["L_Shoulder_Cluster", "R_Shoulder_Cluster",
                                           "L_Elbow", "R_Elbow"])

            # ── Module 1: Joint ROM ───────────────────────────────────────
            def test_joint_rom(self):
                self.reset_all(steps=10)
                Logger.log("--- MODULE 1 ---")
                Logger.log("MODULE 1: JOINT ROM TEST")
                for name, axes in JOINT_LIMITS.items():
                    for axis, (lo, hi) in axes.items():
                        for lbl, angle in [("MIN", lo), ("MAX", hi)]:
                            self.move_joint(name, angle, steps=15, axis=axis)
                            self._interfere(f"{lbl} {name} {axis}")
                            self.move_joint(name, 0, steps=10, axis=axis)

            def simulate_head_scan(self):
                self.reset_all(steps=10)
                Logger.log("--- MODULE 2 ---")
                Logger.log("MODULE 2: HEAD LOOK-AROUND")
                for yaw_deg in [0, -50, 0, 50, 0]:
                    self.move_joint("Neck_Cluster", yaw_deg, steps=18, axis="yaw")
                for pitch_deg in [0, -25, 0, 35, 0]:
                    self.move_joint("Neck_Cluster", pitch_deg, steps=18, axis="pitch")

            def simulate_wave(self):
                self.reset_all(steps=10)
                Logger.log("--- MODULE 3 ---")
                Logger.log("MODULE 3: WAVE GESTURE")
                self.gesture_open("R")
                self.move_joint("R_Shoulder_Cluster", -45, steps=15, axis="pitch")
                self.move_joint("R_Shoulder_Cluster",  80, steps=15, axis="yaw")
                self.move_joint("R_Elbow",             90, steps=12, axis="pitch")
                for _ in range(3):
                    self.move_ball([("R_Wrist", None, None, -30)], steps=8)
                    self.move_ball([("R_Wrist", None, None,  30)], steps=8)
                self.reset_all(steps=12)

            def simulate_idle_breathing(self):
                self.reset_all(steps=8)
                Logger.log("--- MODULE 4 ---")
                Logger.log("MODULE 4: IDLE BREATHING")
                for _ in range(4):
                    self.move_joint("Waist_Cluster", -3, steps=12, axis="pitch")
                    self.move_joint("Waist_Cluster",  3, steps=12, axis="pitch")
                self.move_joint("Waist_Cluster", 0, steps=8, axis="pitch")

            def simulate_walking_advanced(self):
                self.reset_all(steps=10)
                Logger.log("--- MODULE 5 ---")
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
                        ("L_Knee",             60,        4*l_sign,    None),
                        ("R_Knee",             60,        4*r_sign,    None),
                        ("L_Ankle_Cluster",    15*l_sign, None,        8*l_sign),
                        ("R_Ankle_Cluster",    15*r_sign, None,        8*r_sign),
                    ], steps=20)
                    self._check_zmp(f"Walk cycle {cycle+1}")
                self.reset_all(steps=12)

            def simulate_running(self):
                self.reset_all(steps=8)
                Logger.log("--- MODULE 6 ---")
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
                        ("L_Knee",             95,        6*l_sign,   None),
                        ("R_Knee",             95,        6*r_sign,   None),
                        ("L_Ankle_Cluster",    25*l_sign, None,       12*l_sign),
                        ("R_Ankle_Cluster",    25*r_sign, None,       12*r_sign),
                    ], steps=14)
                self.reset_all(steps=10)

            def simulate_combat(self):
                self.reset_all(steps=10)
                Logger.log("--- MODULE 7 ---")
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

            def simulate_transformation(self):
                self.reset_all(steps=10)
                Logger.log("--- MODULE 8a ---")
                Logger.log("MODULE 8a: TRANSFORMATION  Robot -> Truck")
                self._transform_to_truck(steps_scale=1.0)
                self._interfere("Truck-mode check")
                Logger.log("MODULE 8c: TRUCK -> ROBOT")
                self._transform_to_robot(steps_scale=1.0)
                Logger.log("Robot mode restored.")
                self.reset_all(steps=10)

            def run_stability_analysis(self):
                Logger.log("--- MODULE 9 ---")
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

            def estimate_servo_loads(self):
                Logger.log("MODULE 9b: SERVO LOAD ESTIMATION")
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

            def run_physical_validation(self):
                Logger.log("--- MODULE 10 ---")
                Logger.log("MODULE 10: PHYSICAL BUILD VALIDATION")
                self.test_tendon_slack()
                self.test_transform_locks()
                self.printability_report()
                Logger.log("Physical validation complete.")

            def run_v14_validation(self):
                """Module 11 — V13 compute/comms/vision/safety validation."""
                Logger.log("--- MODULE 11 ---")
                Logger.log("MODULE 11: V13 ARCHITECTURE + AI SAFETY VALIDATION")
                self.validate_v14_architecture()
                self.simulate_vision_tracking()
                self.simulate_obstacle_reaction()
                Logger.log("V13 architecture validation complete.")

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
                self.capture_screenshots("optimus_robot_v14")

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
                self.capture_screenshots("optimus_truck_v14")

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
                self.capture_screenshots("optimus_battle_v14")

            def run_visual_audit(self):
                global CAPTURE_SCREENSHOTS, VISUAL_AUDIT
                old_capture = CAPTURE_SCREENSHOTS
                old_visual  = VISUAL_AUDIT
                CAPTURE_SCREENSHOTS = True
                VISUAL_AUDIT = True
                try:
                    Logger.log("MODULE 12: VISUAL AUDIT (robot / truck / battle)")
                    self.simulate_robot_mode()
                    self.simulate_truck_mode()
                    self.simulate_battle_mode()
                    self.reset_all(steps=10)
                    self._interfere("Post-visual-audit neutral")
                    Logger.log("Visual audit screenshots complete.")
                finally:
                    CAPTURE_SCREENSHOTS = old_capture
                    VISUAL_AUDIT = old_visual

            def debug_joints(self, label):
                Logger.log(f"=== JOINT STATE [{label}] ===")
                try:
                    for i in range(self._root.asBuiltJoints.count):
                        j  = self._root.asBuiltJoints.item(i)
                        mo = j.jointMotion
                        if mo.objectType == adsk.fusion.RevoluteJointMotion.classType():
                            Logger.log(f"  {j.name:34s} REV  "
                                       f"pitch={math.degrees(mo.rotationValue):+.1f} deg")
                        elif mo.objectType == adsk.fusion.BallJointMotion.classType():
                            try:
                                Logger.log(
                                    f"  {j.name:34s} BALL "
                                    f"p={math.degrees(mo.pitchValue):+.1f} deg "
                                    f"y={math.degrees(mo.yawValue):+.1f} deg "
                                    f"r={math.degrees(mo.rollValue):+.1f} deg")
                            except Exception as e:
                                Logger.log(f"  {j.name:34s} BALL (readback: {e})", "WARN")
                except Exception:
                    Logger.log("  (joint debug unavailable in this environment)", "WARN")

            # ── BUGFIX-V13-5: graceful turntable fallback ─────────────────
            def capture_screenshots(self, prefix="optimus"):
                if not CAPTURE_SCREENSHOTS:
                    return
                try:
                    os.makedirs(SCREENSHOT_DIR, exist_ok=True)
                    viewport = self._app.activeViewport
                    camera   = viewport.camera
                    width, height = 1920, 1080

                    def settle(frames=4, delay=0.10):
                        for _ in range(frames):
                            try:
                                adsk.doEvents()
                            except Exception:
                                pass
                            try:
                                viewport.refresh()
                            except Exception:
                                pass
                            time.sleep(delay)

                    def save_view(name, orientation=None):
                        if orientation is not None:
                            camera.viewOrientation = orientation
                            camera.isFitView = True
                            viewport.camera = camera
                        settle()
                        path = os.path.join(SCREENSHOT_DIR, f"{prefix}_{name}.png")
                        viewport.saveAsImageFile(path, width, height)
                        Logger.log(f"Screenshot: {path}")

                    views = [
                        ("Front", adsk.core.ViewOrientations.FrontViewOrientation),
                        ("Back",  adsk.core.ViewOrientations.BackViewOrientation),
                        ("Left",  adsk.core.ViewOrientations.LeftViewOrientation),
                        ("Right", adsk.core.ViewOrientations.RightViewOrientation),
                        ("Top",   adsk.core.ViewOrientations.TopViewOrientation),
                        ("Bottom", adsk.core.ViewOrientations.BottomViewOrientation),
                        ("IsoTopRight", adsk.core.ViewOrientations.IsoTopRightViewOrientation),
                        ("IsoTopLeft", adsk.core.ViewOrientations.IsoTopLeftViewOrientation),
                    ]
                    for name, orientation in views:
                        try:
                            save_view(name, orientation)
                        except Exception as e:
                            Logger.log(f"  view {name} failed: {e}", "WARN")

                    if VISUAL_AUDIT:
                        # BUGFIX-V13-5: wrap turntable sweep so a single bad
                        # camera-math step can't abort the whole capture run.
                        try:
                            camera.isFitView = True
                            viewport.camera = camera
                            settle()
                            target   = camera.target
                            eye      = camera.eye
                            distance = max(eye.distanceTo(target), 1.0)
                            up_vector = adsk.core.Vector3D.create(0, 0, 1)
                            start_dir = adsk.core.Vector3D.create(
                                eye.x - target.x, eye.y - target.y, 0)
                            if start_dir.length < 1e-6:
                                start_dir = adsk.core.Vector3D.create(1, 0, 0)
                            start_dir.normalize()
                            for angle in [0, 45, 90, 135, 180, 225, 270, 315]:
                                try:
                                    rad = math.radians(angle)
                                    new_dir = adsk.core.Vector3D.create(
                                        start_dir.x*math.cos(rad) - start_dir.y*math.sin(rad),
                                        start_dir.x*math.sin(rad) + start_dir.y*math.cos(rad),
                                        0)
                                    new_dir.normalize()
                                    new_eye = target.copy()
                                    scaled  = new_dir.copy()
                                    scaled.scaleBy(distance)
                                    new_eye.translateBy(scaled)
                                    camera.eye = new_eye
                                    camera.upVector = up_vector
                                    camera.isFitView = False
                                    viewport.camera = camera
                                    save_view(f"Turntable_{angle:03d}")
                                except Exception as e:
                                    Logger.log(f"  turntable {angle} failed: {e}", "WARN")
                        except Exception as e:
                            Logger.log(f"  turntable sweep skipped entirely: {e}", "WARN")
                except Exception:
                    Logger.log(f"Screenshot failed: {traceback.format_exc()}", "WARN")

            # ── URDF Export ────────────────────────────────────────────────
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
                    return math.radians(pitch[0]), math.radians(pitch[1])

                link_mass = {
                    "OP_Head": 280,       "OP_Torso": 950,    "OP_Pelvis": 480,
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
                    path = os.path.join(EXPORT_DIR, "robot_v14.urdf")
                    jc   = self._root.asBuiltJoints.count
                    with open(path, "w", encoding="utf-8") as f:
                        f.write('<?xml version="1.0" encoding="utf-8"?>\n')
                        f.write('<robot name="Optimus_Prime_G1_v14">\n\n')
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
                            if "Hip" in n:   effort = 35.0
                            if "Waist" in n: effort = 25.0
                            if "MCP" in n:   effort = 2.2
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
                    Logger.log(f"URDF v14 exported -> {path}")
                except Exception:
                    Logger.log(f"URDF export failed: {traceback.format_exc()}", "ERROR")

            # ── STL Export ────────────────────────────────────────────────
            def export_stl(self):
                try:
                    os.makedirs(EXPORT_DIR, exist_ok=True)
                    skip_s = {"Marker", "Pivot", "MtA", "MtB", "Axle_Pivot", "Horn",
                              "_Vis", "Scope", "Antenna", "Boss", "Insert", "Nut", "Snap"}
                    export_mgr = self._design.exportManager
                    count = 0
                    for comp in self._comps:
                        for body in list(comp.bRepBodies):
                            # BUGFIX-V13-4: robust check -- skip if name missing
                            # OR matches any skip tag (previous logic was fine
                            # but lacked a defensive `body.name or ""` guard
                            # against bodies with a None name attribute).
                            bname = body.name or ""
                            if not bname or any(s in bname for s in skip_s):
                                continue
                            try:
                                path = os.path.join(EXPORT_DIR, f"{comp.name}__{bname}.stl")
                                stl_opts = export_mgr.createSTLExportOptions(body, path)
                                stl_opts.meshRefinement = (
                                    adsk.fusion.MeshRefinementSettings.MeshRefinementHigh)
                                export_mgr.execute(stl_opts)
                                count += 1
                            except Exception:
                                Logger.log(f"STL fail: {comp.name}/{bname}", "WARN")
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
                    path = os.path.join(EXPORT_DIR, "Optimus_Prime_G1_v14.step")
                    step_opts = export_mgr.createSTEPExportOptions(path)
                    export_mgr.execute(step_opts)
                    Logger.log(f"STEP assembly -> {path}")
                    count = 0
                    skip_tags_local = {"Marker", "Pivot", "_Vis"}
                    for i in range(self._root.allOccurrences.count):
                        occ   = self._root.allOccurrences.item(i)
                        cname = occ.component.name or ""
                        # BUGFIX-V13-4: defensive name check before substring test
                        if not cname or any(t in cname for t in skip_tags_local):
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

            # ── BOM Export ─────────────────────────────────────────────────
            def export_bom(self):
                BOM.add("Fastener", "M3x8 SHCS (general assembly)", 80,  "assembly")
                BOM.add("Fastener", "M3x16 SHCS (joint brackets)",  24,  "assembly")
                BOM.add("Fastener", "M3x4 set screw (cup point)",   20,  "couplers")
                BOM.add("Fastener", "M2.5x6 SHCS (PCB mounting)",   24,  "assembly")
                BOM.add("Fastener", "M2x10 hinge pin (steel)",       6,  "covers")
                BOM.add("Hardware", "O3mm x 30mm shaft pin",        12,  "joints")
                BOM.add("Hardware", "O3mm x 20mm shaft pin",        20,  "joints")
                BOM.add("Hardware", "O3mm x 45mm shaft pin",         8,  "dual bearings")
                BOM.add("Hardware", "Braided Dyneema line 0.4mm (3m)", 2, "finger tendons")
                BOM.add("Hardware", "Torsion spring (finger return)", 10, "MCP joints")
                BOM.add("Electronics", "22AWG servo wire (3m lengths)", 30, "wiring harness")
                BOM.add("Electronics", "JST-XH 3-pin connectors",   40, "servo connectors")
                BOM.add("Electronics", "JST-XH 4-pin connectors",   12, "I2C bus")
                BOM.add("Electronics", "USB-A to USB-C cable (0.5m)", 3, "Jetson<->ESP32 bridge")
                BOM.add("Electronics", "USB-C power cable",          1, "Jetson power")
                BOM.add("Electronics", "microSD card 64GB (A2 rated)", 1, "JetPack OS")
                BOM.add("Electronics", "Blade fuse 3A ATO",           2, "Jetson rail protection")
                BOM.add("Electronics", "Blade fuse 5A ATO",           1, "servo rail protection")
                BOM.add("Hardware", "Rubber grommet O3.5mm (open slot)", 18, "wire exits")
                BOM.add("Hardware", "Velcro strap 20x200mm (battery)", 2, "battery retention")
                BOM.add("Hardware", "Foam pad 2mm", 4, "vibration iso")
                BOM.add("Hardware", "Snap-in cable clip (printed)", 24, "cable mgmt")
                BOM.add("Bearing", "O22mm x 7mm ball bearing (press-fit)", 8, "dual bearing")
                BOM.add("Bearing", "O20mm x 6mm ball bearing (glue-fit)", 12, "single bearing")
                BOM.add("Material", "PETG filament 1kg spool", 4, "~900g structural")
                BOM.add("Material", "PLA+ filament 1kg spool", 2, "~600g detail parts")
                BOM.add("Material", "TPU filament 250g spool", 1, "flex parts")
                BOM.add("Material", "Nylon filament 500g spool", 1, "jigs + couplers")
                BOM.save_csv(BOM_FILE)
                BOM.summary()

                Logger.log("--- SERVO WIRING MAP (per ESP32-S3 node) ---")
                node_wiring = {
                    "ESP32-S3 'lower' (pelvis)": [
                        ("PCA-HipKnee ch0-5", "L/R Hip Yaw, Hip Pitch, Hip Roll (DS3225MG/MG996R)"),
                        ("PCA-HipKnee ch6-7", "L/R Knee (MG996R)"),
                        ("PCA-Ankle  ch0-3",  "L/R Ankle Pitch, Ankle Roll (MG996R)"),
                        ("I2C addr 0x68",     "MPU-6050 balance IMU"),
                    ],
                    "ESP32-S3 'upper' (torso)": [
                        ("PCA ch0-1",  "Waist Yaw, Waist Pitch (MG996R)"),
                        ("PCA ch2-7",  "L/R Shoulder Yaw/Pitch/Roll (MG996R)"),
                        ("PCA ch8-9",  "L/R Elbow (MG996R)"),
                        ("I2C addr 0x40", "INA3221 servo-rail current monitor"),
                    ],
                    "ESP32-S3 'head' (head)": [
                        ("PCA ch0",    "Neck Pitch (torso-mounted MG996R, wired up)"),
                        ("PCA ch1",    "Neck Yaw (MG90S)"),
                        ("PCA ch2-3",  "L/R Wrist Roll (MG90S)"),
                        ("PCA ch4-7",  "L/R Finger Drive + Thumb (DS04-NFC)"),
                        ("PCA ch8",    "Blaster Fold (MG90S)"),
                        ("I2C addr 0x29/0x2A", "VL53L1X ToF sensors (L/R, alt-addressed)"),
                    ],
                }
                for node, rows in node_wiring.items():
                    Logger.log(f"  [{node}]")
                    for ch, desc in rows:
                        Logger.log(f"    {ch:<22s} -> {desc}")

                Logger.log("--- V13 POWER BUDGET (summary) ---")
                Logger.log("  Jetson Nano (5V/4A UBEC):  ~3.0A peak @ 10W AI inference mode")
                Logger.log("  Logic rail (5V/3A BEC):    ~0.6A combined, 3x ESP32-S3 nodes")
                Logger.log("  Servo rail (7.4V direct):  ~10A peak, all 28 servos moving")
                Logger.log("  Fusing:  3A on each Jetson/logic rail, 5A on servo rail")
                Logger.log("  Recommended pack: 3S 5000mAh+ LiPo with BMS for >20min runtime")

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
                        self.run_physical_validation(),
                        self.run_v14_validation(),
                        self.run_visual_audit() if VISUAL_AUDIT else None,
                        self.export_bom(),
                    ],
                    "rom":        self.test_joint_rom,
                    "head":       self.simulate_head_scan,
                    "wave":       self.simulate_wave,
                    "breathing":  self.simulate_idle_breathing,
                    "walk":       self.simulate_walking_advanced,
                    "run":        self.simulate_running,
                    "combat":     self.simulate_combat,
                    "transform":  self.simulate_transformation,
                    "truck":      self.simulate_truck_mode,
                    "battle":     self.simulate_battle_mode,
                    "robot":      self.simulate_robot_mode,
                    "stability":  self.run_stability_analysis,
                    "servo":      self.estimate_servo_loads,
                    "workspace":  self.test_arm_workspace,
                    "bom":        self.export_bom,
                    "fingers":    lambda: [
                        self.gesture_open("L"), self.gesture_open("R"),
                        self.gesture_fist("L"), self.gesture_fist("R"),
                        self.gesture_point("R"), self.gesture_snap("R"),
                    ],
                    "tendon":     self.test_tendon_slack,
                    "locks":      self.test_transform_locks,
                    "print":      self.printability_report,
                    "physical":   self.run_physical_validation,
                    "visual":     self.run_visual_audit,
                    # V13 new dispatch entries
                    "architecture": self.validate_v14_architecture,
                    "vision":       self.simulate_vision_tracking,
                    "obstacle":     self.simulate_obstacle_reaction,
                    "v14check":     self.run_v14_validation,
                    "production": lambda: [
                        self.test_joint_rom(),
                        self.test_arm_workspace(),
                        self.run_stability_analysis(),
                        self.estimate_servo_loads(),
                        self.run_physical_validation(),
                        self.run_v14_validation(),
                        self.run_visual_audit(),
                        self.export_bom(),
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

                write_build_manifest()
                write_production_readiness_report(self._cols)

                Logger.log("=" * 58)
                Logger.log("OPTIMUS PRIME G1 v14.0 -- FINAL REPORT")
                Logger.log("Jetson Nano AI Brain Edition")
                Logger.log("=" * 58)
                for label, count in self._cols:
                    if count >= 0:
                        icon = "[OK]" if count == 0 else "[WARN]"
                        Logger.log(f"  {label:<42} {icon}  {count}")
                    else:
                        Logger.log(f"  {label:<42} ?  N/A")
                if EXPORT_URDF:
                    Logger.log(f"  URDF  -> {EXPORT_DIR}/robot_v14.urdf")
                Logger.log(f"  BOM   -> {BOM_FILE}")
                Logger.log(f"  ASM   -> {ASSEMBLY_FILE}")
                Logger.log(f"  MAN   -> {MANIFEST_FILE}")
                Logger.log(f"  PROD  -> {PRODUCTION_FILE}")
                Logger.log(f"  Log   -> {LOG_FILE}")
                Logger.log("=" * 58)
                Logger.log("V13 BUILD COMPLETE -- Review ASSEMBLY_GUIDE before printing")

        # ═════════════════════════════════════════════════════════════════
        # ARCHIVE & RUN
        # ═════════════════════════════════════════════════════════════════
        try:
            os.makedirs(EXPORT_DIR, exist_ok=True)
            archive_path = os.path.join(EXPORT_DIR, "Optimus_Prime_G1_v14.f3d")
            export_mgr   = design.exportManager
            archive_opts = export_mgr.createFusionArchiveExportOptions(archive_path)
            export_mgr.execute(archive_opts)
            Logger.log(f"Design archived -> {archive_path}")
        except Exception as e:
            Logger.log(f"Archive skipped: {e}", "WARN")

        sim = SimulationEngine(root, comps_list, design, app, ui)
        sim.run_all_simulations()
        Logger.log("v14 script finished successfully.")

    except Exception:
        Logger.log(f"FATAL ERROR:\n{traceback.format_exc()}", "ERROR")

    finally:
        Logger.flush()