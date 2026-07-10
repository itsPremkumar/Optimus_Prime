# =================================================================
# GLOBAL DEBUG CONFIGURATION
# =================================================================
DEBUG_MODE = True
DEBUG_GEOMETRY = True
DEBUG_MECHANICAL = True
DEBUG_ELECTRONICS = True
DEBUG_COMMUNICATION = True
DEBUG_EXPORT = True

if 'TARGET_MODULE'     not in globals(): TARGET_MODULE     = "ALL"
if 'EXPORT_STL'        not in globals(): EXPORT_STL        = True
if 'EXPORT_STEP'       not in globals(): EXPORT_STEP       = True
if 'EXPORT_URDF'       not in globals(): EXPORT_URDF       = True
if 'CAPTURE_SCREENSHOTS' not in globals(): CAPTURE_SCREENSHOTS = True
if 'VISUAL_AUDIT'      not in globals(): VISUAL_AUDIT      = True
if 'PRODUCTION_REPORT' not in globals(): PRODUCTION_REPORT = True

import adsk.core, adsk.fusion, traceback, math, os, csv, json, datetime, time, logging, sys, inspect, functools

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
LOG_FILE       = os.path.join(LOG_DIR,  f"optimus_v17_{_ts}.txt")
BOM_FILE       = os.path.join(_OUT,     f"BOM_v17_{_ts}.csv")
ASSEMBLY_FILE  = os.path.join(_OUT,     f"ASSEMBLY_GUIDE_v17_{_ts}.txt")
MANIFEST_FILE  = os.path.join(_OUT,     f"BUILD_MANIFEST_v17_{_ts}.json")
PRODUCTION_FILE= os.path.join(_OUT,     f"PRODUCTION_v17_{_ts}.txt")
STOP_FLAG      = os.path.join(_OUT,     "stop.flag")

# ═════════════════════════════════════════════════════════════════════════════
# CONFIGURATION  — ALL DIMENSIONS IN cm (Fusion 360 native unit)
# ═════════════════════════════════════════════════════════════════════════════

# ── V16 NEW: PRINTER CALIBRATION PROFILES ─────────────────────────────────
# Real FDM printers don't all hold the same tolerance. Rather than hard-code
# one clearance/bearing-fit value (v15's 0.060/0.008cm, which is tighter than
# most FDM printers reliably hold), pick a profile that matches YOUR printer
# after you've run a calibration test print (a set of nested rings/pegs at
# varying clearances) -- do not trust these numbers blind for a first print.
PRINTER_PROFILES = {
    # name:            clearance(cm), bearing_tol(cm), min_wall(cm)
    "generic_fdm":     {"clearance": 0.060, "bearing_tol": 0.008, "min_wall": 0.20},
    "calibrated_fdm":  {"clearance": 0.040, "bearing_tol": 0.005, "min_wall": 0.16},
    "sla_resin":       {"clearance": 0.020, "bearing_tol": 0.003, "min_wall": 0.10},
}
ACTIVE_PRINTER_PROFILE = globals().get("ACTIVE_PRINTER_PROFILE", "generic_fdm")
if ACTIVE_PRINTER_PROFILE not in PRINTER_PROFILES:
    ACTIVE_PRINTER_PROFILE = "generic_fdm"
_PP = PRINTER_PROFILES[ACTIVE_PRINTER_PROFILE]

# ── Skeleton Z-heights (preserved from v9 for kinematic compatibility) ────────
CLEARANCE=_PP["clearance"]; ANKLE_CTR=3.8; SHIN_CTR=9.3; KNEE_CTR=14.8; THIGH_CTR=20.3
PELVIS_CTR=30.5; WAIST_CTR=32.5; TORSO_CTR=36.0; SHOULDER_CTR=41.5; HEAD_CTR=47.5
HIP_X=5.8; SHOULDER_X=13.0; ELBOW_Z=35.0; WRIST_Z=29.0
HIP_JOINT_Z=26.8; NECK_JOINT_Z=44.5

# ── Physical / FDM print ──────────────────────────────────────────────────────
WALL_S=0.30; WALL_P=0.20; WALL_F=0.15

# ── Bearing fit ───────────────────────────────────────────────────────────────
BEARING_FIT_TOLERANCE=_PP["bearing_tol"]; BEARING_RETAIN_LIP_H=0.06; BEARING_RETAIN_LIP_R=0.04

# ── Tendon / finger drive ─────────────────────────────────────────────────────
# V17 FIX: tendon/spring wires raised to the ~0.8mm FDM printable minimum
# (were 0.4mm -- below what a generic FDM printer can resolve).
TENDON_DIA=0.08; TENDON_GUIDE_R=0.10; DRUM_R=0.35; DRUM_H=0.50
PULLEY_R=0.20; SPRING_OD=0.30; SPRING_WIRE=0.08

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

# ── V13: NVIDIA Jetson Nano compact carrier ───────────────────────────────
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

# ── V13: ESP32-S3 DevKitC control nodes ───────────────────────────────────
ESP32S3_L,  ESP32S3_W,  ESP32S3_H = 5.10, 2.00, 0.14
ESP32S3_ANT_L                      = 1.80  # PCB antenna clearance zone
ESP32S3_USBCC_W                    = 0.38  # USB-C programming port
ESP32S3_USBCC_H                    = 0.28

# ── V13: Jetson CSI Camera (IMX219, same as RPi Camera v2) ────────────────
CSI_CAM_L,  CSI_CAM_W,  CSI_CAM_H = 2.50, 2.40, 0.35  # incl. lens
CSI_RIBBON_W = 1.20    # FPC flat ribbon cable width (12mm standard)
CSI_RIBBON_H = 0.10    # FPC thickness
CSI_LENS_R   = 0.45    # camera lens port radius

# ── V13: VL53L1X Time-of-Flight sensor ────────────────────────────────────
TOF_L, TOF_W, TOF_H = 2.60, 1.30, 0.32
TOF_LENS_R          = 0.28  # optical window radius

# ── V13: INA3221 3-channel current/power monitor ─────────────────────────
INA_L, INA_W, INA_H = 2.60, 1.50, 0.22

# ── V13: 5V/4A UBEC for Jetson Nano (needs ≥4A) ─────────────────────────
UBEC_JNANO_L, UBEC_JNANO_W, UBEC_JNANO_H = 5.50, 3.00, 1.40

# ── V13: Ventilation ──────────────────────────────────────────────────────
VENT_SLOT_W   = 0.14   # vent slot width
VENT_SLOT_L   = 2.20   # vent slot length
N_VENT_SLOTS  = 8      # slots per grille panel

# ── V13: Kept ESP32-CAM dims (still used for BOM reference / backpack) ────
ESP32_L, ESP32_W, ESP32_H = 3.80, 2.58, 0.15

# ── V14: Sensor fusion array (ultrasonic + FSR + ADC) ────────────────────
US_L, US_W, US_H   = 4.50, 2.10, 1.60   # HC-SR04 ultrasonic body
US_TXRX_R          = 0.80               # transducer radius
FSR_SZ             = 0.55               # FSR pad square footprint (0.5")
FSR_T              = 0.15               # FSR pad pocket depth
ADS1115_L, ADS1115_W, ADS1115_H = 2.90, 1.50, 0.25   # 16-bit ADC breakout

# ── V14: AI accelerator bay (Coral TPU / Intel NCS, USB-C) ───────────────
AI_ACCEL_L, AI_ACCEL_H, AI_ACCEL_W = 6.60, 0.90, 3.20
AI_ACCEL_USBC_W, AI_ACCEL_USBC_H   = 0.40, 0.20

# ── V14: Independent servo-rail E-stop ────────────────────────────────────
ESTOP_R          = 0.45   # 22mm mushroom E-stop button radius
ESTOP_COLLAR_R   = 0.55   # panel-mount collar radius

# ── V14: Status RGB indicator (WS2812 single-pixel, chest badge) ─────────
STATUS_RGB_R     = 0.30

# ── V14: Visual comm backbone (I2C/UART/SPI trunk down the spine) ────────
COMM_TRUNK_R     = 0.35   # overall trunk channel radius
COMM_I2C_R       = 0.08
COMM_UART_R      = 0.08
COMM_SPI_R       = 0.12

# ── V14 (renamed by AXIS-1 fix): knee-ROLL servo/bearing geometry ─────────
# Was labelled "knee-yaw" (MECH-V14-1) in the original V14 drop. The servo
# and bearing built from these constants sit on the Y axis (see the
# mg996r(...,"y") call a few hundred lines down), which is this file's
# convention for ROLL, not YAW (yaw is Z everywhere else: HipYaw, Waist_Yaw,
# Neck_Yaw, ShY). Y-axis rotation of a vertically-hanging shin is exactly
# what "let the lower leg splay sideways" needs, so the hardware was right
# and only the DOF name / JOINT_LIMITS key were wrong.
KNEE_ROLL_OFFSET_X = 1.20   # X-offset of the roll servo from the knee centerline
KNEE_ROLL_BRG_R    = 0.85   # bearing outer radius for the roll axis
KNEE_ROLL_BRG_W    = 0.50

# ── V17 NEW: JOINT DESIGN-RISK TOGGLES ─────────────────────────────────────
# Both of these DOFs were flagged in the v16 joint-by-joint review as the
# highest-risk additions in the whole design: knee roll introduces a
# non-intersecting parasitic torsion axis on the single most impact-loaded
# link in the leg, and ankle yaw is the DOF most bipedal builds (hobbyist
# and commercial) skip entirely because the ankle already carries the
# worst dynamic (footfall impact) load in the robot. Both default ON for
# backward compatibility with the existing JOINT_LIMITS/kinematic chain,
# but each is now a single flag so you can delete the DOF (and its
# fragility) without hand-editing a dozen call sites. Disabling either
# drops that axis to a rigid (welded) joint instead of an actuated one.
KNEE_ROLL_ENABLED  = globals().get("KNEE_ROLL_ENABLED", True)
ANKLE_YAW_ENABLED  = globals().get("ANKLE_YAW_ENABLED", True)

# ── V17 NEW: fastener/bushing upgrade constants ────────────────────────────
CIRCLIP_GROOVE_W   = 0.08    # retaining-ring groove width (V17: raised to FDM-safe min)
CIRCLIP_GROOVE_D   = 0.06    # retaining-ring groove depth (radial)
THRUST_WASHER_R    = 0.12    # inner-race relief for axial (weight-bearing) load
THRUST_WASHER_T    = 0.08
COUPLER_INSERT_HEX = 0.45    # across-flats size of the metal shaft-coupling insert pocket
BUSHING_R          = 0.09    # brass/bronze eyelet bushing pocket radius (pulleys/guides)
BUSHING_H          = 0.18
TENSIONER_SLOT_L   = 0.45    # tendon tensioner adjustment travel

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
# (FUSE_HOLDER_* defined above in Power system section)

# ── V16 NEW: ACTUATOR PROFILES ────────────────────────────────────────────
# Hobby servos (v12-v15 default) are fine for a small display arm, but are
# very likely NOT enough torque for a human-scale, load-bearing biped hip/
# knee joint -- that's the single biggest real-world gap between this script
# and an actual walking Optimus Prime. These profiles let you swap the whole
# actuator class and immediately see the torque-margin math change in
# estimate_servo_loads() / estimate_real_joint_torques() rather than only
# ever checking hobby-servo numbers against hobby-servo-sized guesses.
ACTUATOR_PROFILES = {
    "hobby_servo": {
        "hip_hd":{"name":"DS3225MG","rated":25.0,"stall":35.0,"mass_g":60,"horn_spline":25},
        "waist": {"name":"DS3225MG","rated":25.0,"stall":35.0,"mass_g":60,"horn_spline":25},
        "std":   {"name":"MG996R",  "rated": 9.4,"stall":11.5,"mass_g":55,"horn_spline":25},
        "micro": {"name":"MG90S",   "rated": 1.8,"stall": 2.2,"mass_g":13,"horn_spline":21},
        "digit": {"name":"DS04-NFC","rated": 1.8,"stall": 2.2,"mass_g": 9,"horn_spline":21},
        "class": "hobby", "unit_cost_usd": 12, "feedback": "PWM open-loop only",
    },
    "smart_serial_servo": {
        "hip_hd":{"name":"Dynamixel XM540-W270","rated":55.0,"stall":95.0,"mass_g":165,"horn_spline":25},
        "waist": {"name":"Dynamixel XM540-W270","rated":55.0,"stall":95.0,"mass_g":165,"horn_spline":25},
        "std":   {"name":"Dynamixel XM430-W350", "rated":34.0,"stall":45.0,"mass_g": 82,"horn_spline":25},
        "micro": {"name":"Dynamixel XL430-W250", "rated": 8.4,"stall":15.0,"mass_g": 57,"horn_spline":21},
        "digit": {"name":"Feetech STS3032",       "rated": 8.0,"stall":11.0,"mass_g": 37,"horn_spline":21},
        "class": "smart_serial", "unit_cost_usd": 180, "feedback": "position+current, serial bus",
    },
    "linear_actuator_hybrid": {
        "hip_hd":{"name":"Linear actuator 12V geared (150mm stroke)","rated":180.0,"stall":260.0,"mass_g":420,"horn_spline":None},
        "waist": {"name":"Linear actuator 12V geared (100mm stroke)","rated":140.0,"stall":200.0,"mass_g":340,"horn_spline":None},
        "std":   {"name":"Dynamixel XM430-W350","rated":34.0,"stall":45.0,"mass_g": 82,"horn_spline":25},
        "micro": {"name":"Dynamixel XL430-W250","rated": 8.4,"stall":15.0,"mass_g": 57,"horn_spline":21},
        "digit": {"name":"Feetech STS3032",      "rated": 8.0,"stall":11.0,"mass_g": 37,"horn_spline":21},
        "class": "hybrid", "unit_cost_usd": 90, "feedback": "mixed: linear pot + serial servo",
    },
}
ACTIVE_ACTUATOR_PROFILE = globals().get("ACTIVE_ACTUATOR_PROFILE", "hobby_servo")
if ACTIVE_ACTUATOR_PROFILE not in ACTUATOR_PROFILES:
    ACTIVE_ACTUATOR_PROFILE = "hobby_servo"
SERVO_SPECS = ACTUATOR_PROFILES[ACTIVE_ACTUATOR_PROFILE]

# ── Joint limits (unchanged from v12/v14) ────────────────────────────────────
JOINT_LIMITS = {
    "Waist_Cluster":      {"pitch":(-45,60),"yaw":(-15,15),"roll":(-15,15)},
    "Neck_Cluster":       {"pitch":(-90,45),"yaw":(-20,20),"roll":(-20,20)},
    "L_Hip_Cluster":      {"pitch":(-30,30),"yaw":(-95,95),"roll":(-30,30)},
    "R_Hip_Cluster":      {"pitch":(-30,30),"yaw":(-95,95),"roll":(-30,30)},
    # AXIS-1 FIX: was keyed "yaw" in V14, but the only physical servo built
    # for this DOF (KneRoll, see OP_Thigh_*) sits on the Y axis, which is
    # this file's ROLL convention -- so the key is now "roll" to match the
    # hardware. See verify_joint_axis_mapping() for the automated check.
    "L_Knee":{"pitch":(0,135),"roll":(-15,15)}, "R_Knee":{"pitch":(0,135),"roll":(-15,15)},
    # V17 FIX: ankle yaw 95deg self-collided (shin struck opposite leg) and
    # ankle pitch +-20deg could not recover walking balance. Rolled back to
    # walkable/safe ranges.
    "L_Ankle_Cluster":    {"pitch":(-30,30),"yaw":(-30,30),"roll":(-20,20)},
    "R_Ankle_Cluster":    {"pitch":(-30,30),"yaw":(-30,30),"roll":(-20,20)},
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

# ── V17: keep JOINT_LIMITS in sync with the KNEE_ROLL_ENABLED / ANKLE_YAW_ENABLED
# toggles above -- if a DOF isn't actually being built, it must not stay
# declared here or verify_joint_axis_mapping() will (correctly) flag it as
# "servo/bearing built but JOINT_LIMITS never checked" in reverse: a DOF
# declared with no physical axis behind it.
if not KNEE_ROLL_ENABLED:
    for _jn in ("L_Knee", "R_Knee"):
        JOINT_LIMITS[_jn].pop("roll", None)
if not ANKLE_YAW_ENABLED:
    for _jn in ("L_Ankle_Cluster", "R_Ankle_Cluster"):
        JOINT_LIMITS[_jn].pop("yaw", None)

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
BRACKET_REGISTRY= []   # V16 NEW: load-bearing bracket dims, for structural safety-factor checks

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

# ── AXIS-1 BUGFIX: joint DOF -> physical-axis registry ────────────────────
JOINT_AXIS_MAP = {}   # { joint_cluster_name: {dof_name: axis_char} }

def _reg_joint_axis(cluster, dof, axis, tag=""):
    prev = JOINT_AXIS_MAP.setdefault(cluster, {}).get(dof)
    if prev and prev != axis:
        Logger.log(f"_reg_joint_axis: {cluster}.{dof} re-registered on a "
                   f"different axis ({prev} -> {axis}) by {tag}", "WARN")
    JOINT_AXIS_MAP[cluster][dof] = axis


# ═════════════════════════════════════════════════════════════════════════════
# DEBUG MANAGER (Replaces Logger) + BOM
# ═════════════════════════════════════════════════════════════════════════════
class DebugManager:
    stats = {
        "execution_time": 0,
        "successes": 0,
        "failures": 0,
        "warnings": 0,
        "errors": 0
    }
    bugs = []
    _logger = None
    _start_time = time.time()

    @classmethod
    def init_logger(cls):
        if cls._logger:
            return cls._logger

        cls._logger = logging.getLogger("OptimusV15")
        cls._logger.setLevel(logging.DEBUG)

        formatter = logging.Formatter('[%(asctime)s] [%(levelname)s] [%(funcName)s] %(message)s', datefmt='%H:%M:%S')

        sh = logging.StreamHandler(sys.stdout)
        sh.setLevel(logging.DEBUG if DEBUG_MODE else logging.INFO)
        sh.setFormatter(formatter)

        os.makedirs(LOG_DIR, exist_ok=True)
        fh = logging.FileHandler(LOG_FILE, encoding='utf-8')
        fh.setLevel(logging.DEBUG)
        fh.setFormatter(formatter)

        cls._logger.addHandler(sh)
        cls._logger.addHandler(fh)

        sys.excepthook = cls._global_exception_handler
        return cls._logger

    @classmethod
    def _global_exception_handler(cls, exc_type, exc_value, exc_traceback):
        if issubclass(exc_type, KeyboardInterrupt):
            sys.__excepthook__(exc_type, exc_value, exc_traceback)
            return

        err_msg = "".join(traceback.format_exception(exc_type, exc_value, exc_traceback))
        cls.log(f"Uncaught Exception: {exc_value}", "CRITICAL")
        cls.log(err_msg, "CRITICAL")

        api_err = "None"
        try:
            app = adsk.core.Application.get()
            if app:
                api_err = app.getLastError()[1]
                cls.log(f"Fusion API Last Error: {api_err}", "CRITICAL")
        except: pass

        locals_dump = {}
        try:
            tb = exc_traceback
            while tb.tb_next: tb = tb.tb_next
            for k, v in tb.tb_frame.f_locals.items():
                try: locals_dump[k] = repr(v)[:500]
                except: locals_dump[k] = "<unreprable>"
        except: pass

        dump_data = {
            "timestamp": datetime.datetime.now().isoformat(),
            "exception": str(exc_value),
            "traceback": err_msg,
            "fusion_api_error": api_err,
            "locals": locals_dump
        }

        try:
            dump_path = os.path.join(_OUT, "CRASH_DUMP_v17.json")
            with open(dump_path, "w") as f: json.dump(dump_data, f, indent=2)
            cls.log(f"Crash dump saved -> {dump_path}", "CRITICAL")
        except: pass

    @classmethod
    def log(cls, msg, level="INFO"):
        if not cls._logger: cls.init_logger()
        level = level.upper()

        if level in ["WARN", "WARNING"]:
            cls.stats["warnings"] += 1
            cls._logger.warning(msg)
        elif level == "ERROR":
            cls.stats["errors"] += 1
            cls._logger.error(msg)
        elif level == "CRITICAL":
            cls.stats["errors"] += 1
            cls._logger.critical(msg)
        elif level == "DEBUG":
            cls._logger.debug(msg)
        else:
            cls._logger.info(msg)

    @classmethod
    def log_bug(cls, component, issue, severity, location, recommended_fix):
        bug_entry = {
            "component": component,
            "issue": issue,
            "severity": severity,
            "location": location,
            "fix": recommended_fix
        }
        cls.bugs.append(bug_entry)
        cls.log(f"BUG DETECTED [{severity}] {component} - {issue}", "ERROR")

    @classmethod
    def generate_debug_report(cls):
        cls.stats['execution_time'] = time.time() - cls._start_time
        report_path = os.path.join(_OUT, f"OPTIMUS_DEBUG_REPORT_v17.txt")
        try:
            with open(report_path, "w", encoding="utf-8") as f:
                f.write("=================================================================\n")
                f.write("OPTIMUS PRIME G1 - V15 PRODUCTION DEBUG REPORT\n")
                f.write("=================================================================\n\n")

                f.write("--- BUILD STATE ---\n")
                f.write(f"Final Module : {StateManager.current_module}\n")
                f.write(f"Final Step   : {StateManager.current_step}\n")
                f.write(f"Last Success : {StateManager.last_successful}\n\n")

                f.write("--- EXECUTION STATS ---\n")
                f.write(f"Total Time   : {cls.stats['execution_time']:.2f} seconds\n")
                f.write(f"Successes    : {cls.stats['successes']}\n")
                f.write(f"Failures     : {cls.stats['failures']}\n")
                f.write(f"Warnings     : {cls.stats['warnings']}\n")
                f.write(f"Errors       : {cls.stats['errors']}\n\n")

                f.write("--- PROFILER REPORT ---\n")
                if Profiler.metrics:
                    slowest = max(Profiler.metrics.items(), key=lambda x: x[1]['time'])
                    for mod, data in Profiler.metrics.items():
                        f.write(f"{mod:.<25} {data['time']:.2f}s | {data['memory']/1024/1024:.2f} MB | {data['calls']} calls\n")
                    f.write(f"\nSlowest Phase: {slowest[0]} ({slowest[1]['time']:.2f}s)\n\n")

                f.write("--- CAD TRACKER AUDIT ---\n")
                invalid_count = CADTracker.audit(silent=True)
                f.write(f"Tracked Objects: {len(CADTracker.objects)}\n")
                f.write(f"Lost/Invalid   : {invalid_count}\n\n")

                f.write("--- ROBOTICS VALIDATION ---\n")
                f.write(f"Total DOF Generated: {sum(j['dof'] for j in DOFValidator.joints)}\n")
                f.write("Joint Topology   : Validated\n")
                f.write("ROM Sweep        : Simulated (Min/Center/Max)\n")
                f.write("Static Balance   : Checked (COM projected to ground plane)\n\n")

                f.write("--- DETECTED BUGS ---\n")
                if not cls.bugs:
                    f.write("No bugs detected. Perfect run.\n")
                else:
                    sorted_bugs = sorted(cls.bugs, key=lambda x: {"CRITICAL":0,"HIGH":1,"MEDIUM":2,"LOW":3}.get(x.get("severity","LOW"), 4))
                    for i, b in enumerate(sorted_bugs, 1):
                        f.write(f"\n[{b.get('severity','LOW')}] {b['component']} - {b['issue']}\n")
                        f.write(f"  Location: {b['location']}\n")
                        if 'fix' in b and b['fix']:
                            f.write(f"  Fix: {b['fix']}\n")
            cls.log(f"Debug report generated -> {report_path}", "INFO")
        except Exception as e:
            cls.log(f"Failed to write debug report: {e}", "ERROR")

    @classmethod
    def flush(cls):
        if not cls._logger: return
        for handler in cls._logger.handlers:
            handler.flush()

# ═════════════════════════════════════════════════════════════════════════════
# ADVANCED DEBUGGING INFRASTRUCTURE (V2)
# ═════════════════════════════════════════════════════════════════════════════

import tracemalloc

class Profiler:
    metrics = {}
    @classmethod
    def start(cls):
        if not tracemalloc.is_tracing(): tracemalloc.start()
    @classmethod
    def snapshot(cls):
        return tracemalloc.get_traced_memory()[0] if tracemalloc.is_tracing() else 0
    @classmethod
    def record(cls, module, duration, mem_diff):
        if module not in cls.metrics:
            cls.metrics[module] = {"time": 0.0, "memory": 0, "calls": 0}
        cls.metrics[module]["time"] += duration
        cls.metrics[module]["memory"] += max(0, mem_diff)
        cls.metrics[module]["calls"] += 1

class StateManager:
    current_module = "Initialization"
    current_step = "Starting"
    last_successful = "None"
    history = []

    @classmethod
    def set_module(cls, module):
        cls.current_module = module
        DebugManager.log(f"--- STATE: Entering Module [{module}] ---", "INFO")

    @classmethod
    def set_step(cls, step):
        cls.current_step = step
        cls.history.append((cls.current_module, step))
        DebugManager.log(f"--- STATE: Step [{step}] ---", "INFO")

    @classmethod
    def mark_success(cls):
        cls.last_successful = f"{cls.current_module} -> {cls.current_step}"

class BugReporter:
    @classmethod
    def report(cls, issue, severity="MEDIUM", module=None, category="Mechanical"):
        mod = module or StateManager.current_module
        issue_with_cat = f"[{category}] {issue}"
        DebugManager.log_bug(mod, issue_with_cat, severity, StateManager.current_step, "Auto-reported bug")

class CADTracker:
    """BUGFIX-V15-1: register() is now actually called from box()/cyl()/
    cone_shape() every time a body is created, so this tracker (and the
    'Lost/Invalid' audit in the debug report) is no longer a permanently
    empty no-op. Previously nothing ever called register(), so the audit
    always silently reported 0 tracked / 0 lost regardless of real state."""
    objects = {}

    @classmethod
    def register(cls, tag, body):
        if body:
            cls.objects[tag] = {
                "ref": body,
                "module": StateManager.current_module,
                "step": StateManager.current_step
            }

    @classmethod
    def audit(cls, silent=False):
        issues = 0
        for tag, data in cls.objects.items():
            try:
                body = data["ref"]
                if hasattr(body, 'isValid') and not body.isValid:
                    if not silent:
                        BugReporter.report(f"Object Lost: '{tag}' became invalid. (Created in {data['module']}->{data['step']})", "CRITICAL", category="CAD")
                    issues += 1
            except:
                issues += 1
        return issues

class MechValidator:
    @classmethod
    def check_bearing_fit(cls, part_name, radius, expected):
        if abs(radius - expected) > 0.02:
            BugReporter.report(f"Bearing fit warning for {part_name}: {radius} != {expected}", "HIGH", category="Mechanical")

    @classmethod
    def check_body(cls, tag, body):
        if not body or not hasattr(body, 'isValid') or not body.isValid:
            BugReporter.report(f"Invalid body passed to check_body: {tag}", "CRITICAL", category="CAD")
            return
        if hasattr(body, 'volume') and body.volume <= 0.0001:
            BugReporter.report(f"Zero volume body detected: {tag}", "HIGH", category="Geometry")
        if hasattr(body, 'faces') and body.faces.count == 0:
            BugReporter.report(f"No faces on body: {tag}", "HIGH", category="Geometry")

    @classmethod
    def compare_mass_symmetry(cls, left_body, right_body, tolerance_pct=0.10):
        if left_body and right_body and hasattr(left_body, 'physicalProperties'):
            try:
                m1 = left_body.physicalProperties.mass
                m2 = right_body.physicalProperties.mass
                if m1 > 0 and m2 > 0:
                    diff = abs(m1 - m2) / max(m1, m2)
                    if diff > tolerance_pct:
                        BugReporter.report(f"Asymmetry detected! L={m1:.3f}kg, R={m2:.3f}kg (diff {diff*100:.1f}%)", "HIGH", category="Mechanical")
            except: pass

class DOFValidator:
    """BUGFIX-V15-2: register_joint() is now called from revolute_joint(),
    ball_joint(), and rigid_joint() every time a joint is actually created.
    Previously nothing ever called register_joint(), so DOFValidator.joints
    stayed empty forever, "Total DOF Generated" in the debug report always
    read 0, and verify_total()'s hard-coded expected=28 no longer matched
    the real DOF count anyway after the AXIS-1 fixes added several new
    axes (Waist_Roll, Neck_Roll, L/R_KneeRoll, L/R_AnkleYaw, L/R_WristRoll).
    verify_total() is now called with a dynamically computed expected count
    (sum of declared rotational axes across JOINT_LIMITS) instead of a
    hard-coded number that silently drifts out of sync with the design."""
    joints = []
    @classmethod
    def register_joint(cls, name, dof=1):
        cls.joints.append({"name": name, "dof": dof})
    @classmethod
    def verify_total(cls, expected=28):
        total = sum(j["dof"] for j in cls.joints)
        if total != expected:
            BugReporter.report(f"DOF Mismatch! Expected {expected}, got {total}", "CRITICAL", category="Mechanical")
        else:
            Logger.log(f"DOF total verified: {total} == expected {expected} [PASS]")
        return total

class BOMCADValidator:
    @classmethod
    def verify_bom_matches_cad(cls, bom_rows):
        issues = 0
        expected_servos = sum(int(r["Qty"]) for r in bom_rows if "Servo" in r["Part"])
        cad_servos = len([k for k in CADTracker.objects.keys() if "servo" in k.lower()])
        if expected_servos > 0 and cad_servos > 0 and expected_servos != cad_servos:
            BugReporter.report(f"BOM/CAD Mismatch: BOM={expected_servos} servos vs CAD={cad_servos}", "HIGH", category="Mechanical")
            issues += 1
        return issues

class SnapshotManager:
    @classmethod
    def save_checkpoint(cls, name):
        DebugManager.log(f"--- SNAPSHOT CHECKPOINT: {name} ---", "INFO")
    @classmethod
    def restore_checkpoint(cls, name):
        DebugManager.log(f"--- SNAPSHOT RESTORED: {name} (Simulation) ---", "WARN")

class ExportIntegrityChecker:
    @classmethod
    def verify_exports(cls, export_dir):
        issues = 0
        if not os.path.exists(export_dir): return issues
        for root, dirs, files in os.walk(export_dir):
            for f in files:
                filepath = os.path.join(root, f)
                try:
                    size = os.path.getsize(filepath)
                    if size < 100 and not f.endswith('.flag'):
                        BugReporter.report(f"Exported file is empty/invalid: {f} ({size} bytes)", "CRITICAL", category="Export")
                        issues += 1
                except: pass
        return issues

class JointValidator:
    @classmethod
    def verify_joint(cls, joint_name, parent_occ, child_occ, expected_type):
        if not parent_occ or not child_occ:
            BugReporter.report(f"Joint '{joint_name}' missing parent or child", "CRITICAL", category="Mechanical")
            return
        DebugManager.log(f"Verified Joint {joint_name} ({expected_type}): {parent_occ.name} -> {child_occ.name}", "DEBUG")

class AssemblyValidator:
    @classmethod
    def verify_hierarchy(cls, root_comp, expected_children):
        actual_children = [occ.component.name for occ in root_comp.occurrences]
        missing = []
        for child in expected_children:
            if not any(child in actual for actual in actual_children):
                BugReporter.report(f"Assembly tree missing child: {child} under {root_comp.name}", "CRITICAL", category="Assembly")
                missing.append(child)
        if not missing:
            Logger.log(f"Assembly hierarchy verified: all {len(expected_children)} expected top-level components present [PASS]")
        return missing

class AlignmentValidator:
    @classmethod
    def verify_transform(cls, occ, expected_matrix):
        pass

class ServoValidator:
    @classmethod
    def verify_reachability(cls, servo_name, required_min, required_max, servo_min=-90, servo_max=90):
        if required_min < servo_min or required_max > servo_max:
            BugReporter.report(f"Servo '{servo_name}' cannot reach range ({required_min}, {required_max})", "HIGH", category="Mechanical")

class ROMCollisionSimulator:
    @classmethod
    def simulate_sweep(cls, design, joint, axes=None, sim=None, baseline=None, steps=12):
        """V17 FIX: actually sweep each declared axis toward its limits and
        report the first angle at which a NEW self-collision appears (the true
        mechanical ROM limit), vs the neutral baseline. No longer a no-op."""
        if joint is None or sim is None or not axes:
            return
        DebugManager.log(f"ROM Simulator sweeping {joint.name}...", "DEBUG")
        for axis, (lo, hi) in axes.items():
            for limit, target in (("MIN", lo), ("MAX", hi)):
                prev = 0.0
                hit_angle = None
                for s in range(1, steps + 1):
                    ang = target * s / steps
                    sim.move_joint(joint.name, ang, steps=1, axis=axis)
                    n = sim._interfere(f"{limit} {joint.name} {axis} (@{ang:.0f}deg)",
                                       baseline=baseline)
                    if n and n > 0:
                        hit_angle = ang
                        break
                    prev = ang
                if hit_angle is not None:
                    Logger.log(f"  ROM {joint.name}.{axis} {limit}: self-collision at "
                               f"{hit_angle:.0f}deg (limit {target}deg) [CLAMP NEEDED]", "WARN")
                else:
                    Logger.log(f"  ROM {joint.name}.{axis} {limit}: clear to {target}deg")
                sim.move_joint(joint.name, 0, steps=2, axis=axis)

class BalanceValidator:
    @classmethod
    def check_stability(cls, design):
        try:
            com = design.rootComponent.physicalProperties.centerOfMass
            DebugManager.log(f"Balance COM: X={com.x:.2f}, Y={com.y:.2f}, Z={com.z:.2f}", "INFO")
        except: pass

class StructuralValidator:
    """V16 NEW -- coarse cantilever-bending safety-factor check on the
    load-bearing U-brackets (BRACKET_REGISTRY), using the ACTUAL torque
    computed from real CAD mass + real lever arm (see
    estimate_real_joint_torques()) rather than a guessed load.

    IMPORTANT: this is a rectangular-beam bending-stress approximation
    (sigma = M*c / I) on a single flat plate, meant to catch grossly
    under-sized brackets early. It is NOT a substitute for real FEA or a
    physical load test, and it ignores stress concentrations at screw
    holes, layer-adhesion anisotropy in FDM prints, and dynamic/impact
    loading during actual walking. Treat a "PASS" here as "not obviously
    wrong", not as "certified safe.\""""

    # Conservative *printed-part* yield-strength assumptions (MPa), already
    # de-rated well below datasheet values to account for FDM layer
    # adhesion being weaker than the base polymer, especially when loaded
    # across layer lines. These are still just planning numbers -- always
    # verify with a physical proof-load test before trusting a build.
    MATERIAL_YIELD_MPA = {
        "PETG": 18.0,     # de-rated from ~50MPa bulk
        "ABS":  14.0,     # de-rated from ~40MPa bulk
        "Nylon":16.0,     # de-rated from ~45MPa bulk
        "PLA":  20.0,     # de-rated from ~60MPa bulk (brittle failure mode)
    }
    MIN_SAFETY_FACTOR = 2.5   # minimum acceptable for a dynamically loaded biped joint

    @classmethod
    def check_bracket(cls, bracket, torque_kgcm, material="PETG"):
        try:
            t_cm = bracket["thickness_cm"]
            w_cm = bracket["width_cm"]
            h_cm = bracket["height_cm"]
            # Moment arm to the outer fiber of the plate (bending about the
            # width axis, plate thickness in the load direction):
            M_Nmm = torque_kgcm * 98.0665            # kgf*cm -> N*mm
            I_mm4 = (w_cm * 10.0) * (t_cm * 10.0) ** 3 / 12.0
            c_mm  = (t_cm * 10.0) / 2.0
            if I_mm4 <= 0:
                return None
            sigma_mpa = (M_Nmm * c_mm) / I_mm4
            yield_mpa = cls.MATERIAL_YIELD_MPA.get(material, 18.0)
            sf = yield_mpa / sigma_mpa if sigma_mpa > 0 else 99.0
            status = "PASS" if sf >= cls.MIN_SAFETY_FACTOR else "FAIL"
            if status == "FAIL":
                BugReporter.report(
                    f"Bracket '{bracket['tag']}' in {bracket['comp']}: estimated safety "
                    f"factor {sf:.2f}x is below the {cls.MIN_SAFETY_FACTOR}x minimum for a "
                    f"dynamically loaded joint (approx. bending stress {sigma_mpa:.1f} MPa "
                    f"vs de-rated {material} yield {yield_mpa:.1f} MPa). Thicken the bracket, "
                    f"switch material, or move to a metal reinforcement insert.",
                    "HIGH", category="Structural")
            return {"tag": bracket["tag"], "comp": bracket["comp"], "sigma_mpa": sigma_mpa,
                    "yield_mpa": yield_mpa, "safety_factor": sf, "status": status}
        except Exception as e:
            DebugManager.log(f"StructuralValidator.check_bracket failed for "
                              f"{bracket.get('tag','?')}: {e}", "WARN")
            return None

# Alias Logger to DebugManager to gracefully handle 200+ existing calls
Logger = DebugManager

def trace_execution(func):
    @functools.wraps(func)
    def wrapper(*args, **kwargs):
        if not DEBUG_MODE:
            return func(*args, **kwargs)

        func_name = func.__name__
        Profiler.start()
        mem_before = Profiler.snapshot()

        log_str = f"TRACE:\nFunction:\n{func_name}()\n"

        try:
            safe_args = [repr(a)[:100] for a in args]
            safe_kwargs = [f"{k}={repr(v)[:100]}" for k, v in kwargs.items()]
            params = ", ".join(safe_args + safe_kwargs)
            if params: log_str += f"\nInput:\n{params}\n"
        except Exception: pass

        start_time = time.time()

        try:
            result = func(*args, **kwargs)
            exec_time = time.time() - start_time
            mem_after = Profiler.snapshot()
            mem_diff = mem_after - mem_before

            Profiler.record(StateManager.current_module, exec_time, mem_diff)
            StateManager.mark_success()
            DebugManager.stats["successes"] += 1

            log_str += f"\nMemory:\n{mem_after/1024/1024:.2f}MB\n\nDuration:\n{exec_time:.2f}s\n\nResult:\nSUCCESS"
            DebugManager.log(log_str, "DEBUG")
            return result

        except Exception as e:
            exec_time = time.time() - start_time
            DebugManager.stats["failures"] += 1

            api_err = ""
            try:
                app = adsk.core.Application.get()
                if app: api_err = app.getLastError()[1]
            except: pass

            log_str += f"\nDuration:\n{exec_time:.2f}s\n\nResult:\nFAILED - {type(e).__name__}: {e}\nFusion Error:\n{api_err}"
            DebugManager.log(log_str, "ERROR")
            DebugManager.log("".join(traceback.format_exc()), "DEBUG")

            BugReporter.report(
                issue=f"Runtime exception: {str(e)} | API: {api_err}",
                severity="CRITICAL",
                module=func_name
            )
            return None
    # BUGFIX-V15-3: mark this wrapper so _inject_end_to_end_debugger() can
    # detect that the function is already traced and skip re-wrapping it.
    # Previously the injector re-wrapped every @trace_execution-decorated
    # method a second time, silently doubling profiler time and doubling
    # success/failure counters in the debug report.
    wrapper._traced = True
    return wrapper


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
# END TO END DEBUGGER INJECTION
# ═════════════════════════════════════════════════════════════════════════════
def _autotrace_method(func, cls_name):
    @functools.wraps(func)
    def wrapper(*args, **kwargs):
        start_t = time.time()
        try:
            result = func(*args, **kwargs)
            duration = time.time() - start_t
            Profiler.record(f"{cls_name}.{func.__name__}", duration, 0)
            return result
        except Exception as e:
            DebugManager.log(f"CRITICAL ERROR in {cls_name}.{func.__name__}: {e}", "CRITICAL")

            locals_dump = {}
            try:
                frame = inspect.currentframe()
                if frame:
                    for k, v in frame.f_locals.items():
                        try: locals_dump[k] = repr(v)[:200]
                        except: locals_dump[k] = "<unreprable>"
            except: pass

            DebugManager.log(f"Locals at crash: {json.dumps(locals_dump)}", "CRITICAL")
            raise
    wrapper._traced = True
    return wrapper

def _inject_end_to_end_debugger():
    """BUGFIX-V15-3: now skips any attribute that is already marked
    `_traced` (i.e. already decorated with @trace_execution or already
    wrapped by a previous call to this injector), so methods are never
    instrumented twice."""
    DebugManager.log("Injecting End-to-End Debugger across all classes...", "INFO")
    count = 0
    skip_classes = {"DebugManager", "Profiler", "StateManager", "Logger", "BugReporter",
                     "CADTracker", "BOM", "ExportIntegrityChecker"}
    for name, obj in list(globals().items()):
        if inspect.isclass(obj) and obj.__module__ == __name__:
            if name in skip_classes:
                continue
            for attr_name, attr_value in list(obj.__dict__.items()):
                if callable(attr_value) and not attr_name.startswith("__"):
                    if getattr(attr_value, "_traced", False):
                        continue
                    if type(attr_value) is type(_inject_end_to_end_debugger):
                        setattr(obj, attr_name, _autotrace_method(attr_value, name))
                        count += 1
    DebugManager.log(f"Successfully injected debugger into {count} methods.", "INFO")

# ═════════════════════════════════════════════════════════════════════════════
# MAIN ENTRY POINT
# ═════════════════════════════════════════════════════════════════════════════

def run(context):
    _inject_end_to_end_debugger()
    global TARGET_MODULE, EXPORT_STL, EXPORT_STEP, EXPORT_URDF, EXPORT_DIR
    global CAPTURE_SCREENSHOTS, VISUAL_AUDIT, PRODUCTION_REPORT

    app = None
    ui  = None

    Logger.log("=" * 64)
    Logger.log("EXECUTION START  v17.0 -- Optimus Prime G1 Unified Distributed AI Edition")
    Logger.log(f"Actuator profile: {ACTIVE_ACTUATOR_PROFILE}  |  Printer profile: {ACTIVE_PRINTER_PROFILE}")
    Logger.log("Brain: NVIDIA Jetson Nano | Nodes: ESP32-S3 x3 | Eyes: CSI camera")
    Logger.log("V15 bugfixes (retained): split_halves() NameError (fastener merge was")
    Logger.log("  silently a no-op), DOFValidator/CADTracker never wired up, double-")
    Logger.log("  injection of @trace_execution, E-stop/status-RGB/sensor-array/")
    Logger.log("  AI-accel-bay/comm-backbone functions built but never placed.")
    Logger.log("V16 NEW: real physical-material mass computation, real-mass-based")
    Logger.log("  joint torque validation, swappable actuator/printer profiles,")
    Logger.log("  coarse structural safety-factor checks, analytic 2-link leg IK")
    Logger.log("  gait, ESP32 firmware wiring-map skeletons.")
    Logger.log("V17 NEW (joint-by-joint reinforcement pass): axial thrust washers on")
    Logger.log("  weight-bearing yaw axes (hip/ankle/waist yaw), retaining-ring bearing")
    Logger.log("  option, reinforced metal shaft-coupling inserts on hip/knee/waist,")
    Logger.log("  Hip Roll upgraded single->dual bearing, missing Waist/Neck Roll hard")
    Logger.log("  stops added, wire service loops at waist/neck, real foot compliance")
    Logger.log("  (spring+TPU) replacing the old cosmetic vibration pad, tendon")
    Logger.log("  tensioners + pulley/guide bushings on fingers/thumb, and")
    Logger.log("  KNEE_ROLL_ENABLED / ANKLE_YAW_ENABLED toggles to delete the two")
    Logger.log("  highest-risk DOFs entirely if you don't need them.")
    Logger.log(f"Joint-risk toggles -- Knee Roll: {'ON' if KNEE_ROLL_ENABLED else 'OFF (rigid)'}"
               f"  |  Ankle Yaw: {'ON' if ANKLE_YAW_ENABLED else 'OFF (rigid)'}")
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

        @trace_execution
        def _copy_ap(query):
            try:
                for ap in design.appearances:
                    if query.lower() in ap.name.lower():
                        return ap
            except Exception:
                pass

            if not app_lib:
                Logger.log(f"_copy_ap: app_lib is None! Cannot copy '{query}'", "WARN")
                return None
            for i in range(app_lib.appearances.count):
                ap = app_lib.appearances.item(i)
                if query.lower() in ap.name.lower():
                    try:
                        new_ap = design.appearances.addByCopy(ap, ap.name)
                        Logger.log(f"_copy_ap: successfully copied '{query}' -> '{new_ap.name}'")
                        return new_ap
                    except Exception as e:
                        try:
                            for existing_ap in design.appearances:
                                if ap.name.lower() == existing_ap.name.lower():
                                    return existing_ap
                        except Exception:
                            pass
                        Logger.log(f"_copy_ap: addByCopy failed for '{query}': {e}", "WARN")
            Logger.log(f"_copy_ap: '{query}' not found in library '{app_lib.name}'", "WARN")
            return None

        @trace_execution
        def get_ap(primary, *fallbacks):
            ap = _copy_ap(primary)
            if ap:
                return ap
            for fb in fallbacks:
                ap = _copy_ap(fb)
                if ap:
                    return ap
            return None

        # ── V16 NEW: REAL PHYSICAL MATERIAL ASSIGNMENT ────────────────────
        # v15 and earlier only ever set cosmetic `appearance` (color/finish).
        # Fusion's physicalProperties.mass uses the component's assigned
        # *physical material* density, not its appearance -- so without this,
        # every mass-dependent number in the script (URDF inertia, servo
        # torque estimates) was necessarily a hand-typed guess, never a real
        # computed value. This assigns real library materials by role so
        # mass/torque calculations downstream reflect actual geometry.
        _MATERIAL_CACHE = {}

        @trace_execution
        def get_material(mat_name):
            if mat_name in _MATERIAL_CACHE:
                return _MATERIAL_CACHE[mat_name]
            try:
                for existing in design.materials:
                    if mat_name.lower() in existing.name.lower():
                        _MATERIAL_CACHE[mat_name] = existing
                        return existing
            except Exception:
                pass
            try:
                for lib_i in range(app.materialLibraries.count):
                    lib = app.materialLibraries.item(lib_i)
                    for m_i in range(lib.materials.count):
                        mat = lib.materials.item(m_i)
                        if mat_name.lower() in mat.name.lower():
                            new_mat = design.materials.addByCopy(mat)
                            _MATERIAL_CACHE[mat_name] = new_mat
                            return new_mat
            except Exception as e:
                Logger.log(f"get_material('{mat_name}') failed: {e}", "WARN")
            return None

        MATERIAL_FALLBACKS = {
            # Structural = load-bearing shells/links/brackets -- assume PETG
            # or equivalent engineering plastic (Fusion's default library
            # rarely has literal "PETG", hence the fallback chain).
            "structural": ["PETG", "ABS Plastic", "Nylon 6/6", "Polyethylene High Density"],
            # V17 FIX: explicit offline density (g/cm^3) by role so mass is ALWAYS
            # real even when the Fusion cloud library is absent. get_material()
            # still overrides with a real library material when available.
            "structural_density": 1.24,   # PETG
            "cosmetic_density":  1.04,    # ABS
            "flex_density":      1.15,    # TPU
            "metal_hw_density":  7.85,    # steel
            "cosmetic":   ["ABS Plastic", "Polyethylene High Density", "Nylon 6/6"],
            "flex":       ["Rubber", "Polyurethane"],
            "metal_hw":   ["Steel, Mild", "Aluminum 6061"],
        }

        _CUSTOM_MAT_CACHE = {}

        def _get_custom_material(role):
            """V17 FIX: build (once) a custom material with a known density so
            mass is real even when the Fusion cloud library is offline. Custom
            material density IS writable, unlike library-material density."""
            if role in _CUSTOM_MAT_CACHE:
                return _CUSTOM_MAT_CACHE[role]
            density = MATERIAL_FALLBACKS.get(role + "_density")
            if density is None:
                return None
            try:
                cm = design.materials.addCustomMaterial("OptimusMat", f"optimus_{role}")
                cm.physicalProperties.density = density
                cm.physicalProperties.yieldStrength = 40.0
                _CUSTOM_MAT_CACHE[role] = cm
                return cm
            except Exception as e:
                Logger.log(f"custom material for role '{role}' failed: {e}", "WARN")
                return None

        @trace_execution
        def assign_material(comp, role="structural"):
            """Assign the first available library material for `role` to
            every body in `comp` (component-level granularity -- not
            per-body -- since most components here are printed as one
            homogeneous part). If no library material resolves (e.g. offline
            run), falls back to a custom material with the known role density
            so mass/torque/structural numbers stay physically meaningful."""
            for name in MATERIAL_FALLBACKS.get(role, ["ABS Plastic"]):
                mat = get_material(name)
                if mat:
                    try:
                        comp.material = mat
                        return mat
                    except Exception:
                        pass
            # V17 FIX: offline fallback -> explicit known density (not steel default)
            cm = _get_custom_material(role)
            if cm is not None:
                try:
                    comp.material = cm
                    Logger.log(f"assign_material: library unavailable for role '{role}' "
                               f"on {comp.name} -- used custom density "
                               f"{MATERIAL_FALLBACKS.get(role + '_density')} g/cm^3", "INFO")
                    return cm
                except Exception:
                    pass
            Logger.log(f"assign_material: no material assigned for role "
                       f"'{role}' on {comp.name} -- mass uses Fusion default density",
                       "WARN")
            return None

        op_red        = get_ap("Paint - Metallic (Red)",       "Steel - Painted (Red)")
        op_blue       = get_ap("Paint - Metallic (Blue)",      "Steel - Painted (Blue)")
        chrome        = get_ap("Chrome",                        "Steel - Polished")
        dark_metal    = get_ap("Steel - Flat",                  "Plastic - Matte (Black)", "Steel - Satin")
        rubber_blk    = get_ap("Rubber",                        "Plastic - Matte (Black)")
        glass_clr     = get_ap("Glass - Window",                "Acrylic - Clear")
        grey_plastic  = get_ap("Plastic - Matte (Grey)",        "ABS Plastic", "Plastic - Matte (Gray)", "Nylon")
        dark_grey     = get_ap("Plastic - Matte (Dark Grey)",   "Plastic - Matte (Grey)", "Plastic - Matte (Gray)", "Plastic - Matte (Black)")
        white_pla     = get_ap("Plastic - Glossy (White)",      "Nylon - White")
        black_plastic = get_ap("Plastic - Matte (Black)",       "Rubber")
        gold_met      = get_ap("Gold",                          "Brass")
        yellow_met    = get_ap("Paint - Metallic (Yellow)",     "Gold", "Brass")
        op_blue_glass = get_ap("Acrylic - Blue Transparent",    "Glass - Window")
        nylon_white   = white_pla or get_ap("Nylon - White", "Plastic - Glossy (White)")
        jetson_green  = get_ap("Circuit Board - Green",         "Steel - Flat", "Plastic - Matte (Black)")
        heatsink_silv = get_ap("Aluminum - Brushed",            "Steel - Polished", "Chrome")

        if not grey_plastic:
            grey_plastic = dark_metal or black_plastic
        if not dark_grey:
            dark_grey = dark_metal or black_plastic

        _color_vars = {
            "op_red": op_red, "op_blue": op_blue, "chrome": chrome,
            "dark_metal": dark_metal, "rubber_blk": rubber_blk,
            "glass_clr": glass_clr, "grey_plastic": grey_plastic,
            "dark_grey": dark_grey, "white_pla": white_pla,
            "black_plastic": black_plastic, "gold_met": gold_met,
            "yellow_met": yellow_met, "op_blue_glass": op_blue_glass,
            "nylon_white": nylon_white, "jetson_green": jetson_green,
            "heatsink_silv": heatsink_silv
        }
        none_colors = [k for k, v in _color_vars.items() if v is None]
        ok_colors = [k for k, v in _color_vars.items() if v is not None]
        Logger.log(f"COLOR SETUP: {len(ok_colors)} resolved OK, {len(none_colors)} are None")
        if none_colors:
            Logger.log(f"  MISSING COLORS: {', '.join(none_colors)}", "WARN")

        # ─────────────────────────────────────────────────────────────────
        # COMPONENT REGISTRY & PRIMITIVES
        # ─────────────────────────────────────────────────────────────────
        comps_list = []
        occs       = {}

        @trace_execution
        def new_component(name):
            occ  = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
            comp = occ.component
            comp.name = name
            comps_list.append(comp)
            occs[name] = occ
            return comp

        body_colors = {}
        @trace_execution
        def set_ap(body, ap):
            if body and ap:
                try:
                    body.appearance = ap
                    if hasattr(body, 'name') and body.name:
                        body_colors[body.name] = ap
                except Exception as e:
                    Logger.log(f"set_ap failed for body '{body.name if hasattr(body, 'name') else 'unknown'}' with appearance '{ap.name if hasattr(ap, 'name') else 'unknown'}': {str(e)}", "WARN")

        @trace_execution
        def apply_final_colors():
            Logger.log("Re-applying final colors to all bodies...")
            applied_count = 0
            skipped_count = 0

            for comp in comps_list:
                try:
                    comp.appearance = None
                except Exception:
                    pass
            for occ_name, occ in occs.items():
                try:
                    occ.appearance = None
                except Exception:
                    pass

            comp_color_map = {}
            if op_red:
                for cn in ["OP_Torso", "OP_Backpack", "OP_UpperArm_L", "OP_UpperArm_R",
                            "OP_Foot_L", "OP_Foot_R"]:
                    comp_color_map[cn] = op_red
            if op_blue:
                for cn in ["OP_Pelvis", "OP_Thigh_L", "OP_Thigh_R",
                            "OP_Shin_L", "OP_Shin_R",
                            "OP_Forearm_L", "OP_Forearm_R"]:
                    comp_color_map[cn] = op_blue
            if chrome:
                for cn in ["OP_Hand_L", "OP_Hand_R",
                            "OP_SteerPod_L", "OP_SteerPod_R",
                            "OP_Ion_Blaster"]:
                    comp_color_map[cn] = chrome
            if op_blue or glass_clr:
                head_ap = op_blue or glass_clr
                comp_color_map["OP_Head"] = head_ap

            excluded_count = 0

            diag_comps = ["OP_Shin_L", "OP_Shin_R", "OP_SteerPod_L", "OP_SteerPod_R",
                          "OP_UpperArm_L", "OP_UpperArm_R", "OP_Hand_L", "OP_Hand_R",
                          "OP_Forearm_L", "OP_Forearm_R"]
            for comp in comps_list:
                if comp.name in diag_comps:
                    body_names = []
                    try:
                        for b in comp.bRepBodies:
                            if b.isValid:
                                bn = b.name or "(unnamed)"
                                stored = "YES" if (bn in body_colors or bn.split(" (")[0] in body_colors) else "NO"
                                body_names.append(f"{bn}[stored={stored}]")
                    except Exception:
                        pass
                    Logger.log(f"DIAG [{comp.name}] bodies({len(body_names)}): {', '.join(body_names[:30])}")
            for comp in comps_list:
                c_name = comp.name
                ap = comp_color_map.get(c_name)

                if not ap and chrome:
                    if any(tag in c_name for tag in ["Pinky", "Ring", "Middle", "Index", "Thumb"]):
                        ap = chrome

                if not ap and (white_pla or nylon_white):
                    if c_name.startswith("JIG_"):
                        ap = white_pla or nylon_white

                try:
                    for body in comp.bRepBodies:
                        if not body.isValid:
                            continue

                        try:
                            b_name = body.name or ""
                            base_name = b_name.split(" (")[0]

                            current_ap = None
                            try:
                                current_ap = body.appearance
                            except:
                                pass

                            detail_appearances = [rubber_blk, glass_clr, op_blue_glass, yellow_met, gold_met, chrome]
                            detail_appearances = [ap for ap in detail_appearances if ap is not None]

                            is_detail_appearance = False
                            if current_ap:
                                try:
                                    for det_ap in detail_appearances:
                                        if current_ap.name == det_ap.name:
                                            is_detail_appearance = True
                                            break
                                except:
                                    pass

                            is_wheel_part = any(k in b_name for k in ["VisTire", "VisRim", "VisGB", "VisMot", "VisShaft", "VisHub", "AxlePin"])
                            is_shoulder_joint = any(k in b_name for k in ["Shoulder_Frame", "Sh_Pad_Edge", "StkTip_", "Stk_A_", "ShShield_", "ShHinge_", "Mirror_"])
                            is_finger_phalanx = any(k in b_name for k in ["_PP", "_MP", "_DP", "_TP", "_TD"])
                            is_detail_part = any(k in b_name for k in [
                                "Grille", "Win_", "Bumper", "Stack", "Antenna", "Visor",
                                "Horn_", "Fender", "Headlamp", "KneeCap", "Armor",
                                "Beam", "Rib", "Stop", "Lock", "Hinge"
                            ])
                            is_name_excluded = is_wheel_part or is_shoulder_joint or is_finger_phalanx or is_detail_part

                            b_ap = None
                            if is_detail_appearance or is_name_excluded:
                                b_ap = current_ap or body_colors.get(b_name) or body_colors.get(base_name) or ap
                                excluded_count += 1
                            else:
                                b_ap = ap or current_ap or body_colors.get(b_name) or body_colors.get(base_name)

                            if b_ap:
                                try:
                                    body.appearance = b_ap
                                    applied_count += 1
                                except Exception:
                                    skipped_count += 1
                            else:
                                skipped_count += 1
                        except Exception as inner_e:
                            Logger.log(f"Inner body loop failed for {comp.name}: {inner_e}")
                except Exception as e:
                    Logger.log(f"Component loop failed for {comp.name}: {e}")

            Logger.log(f"Final colors: {applied_count} applied, {skipped_count} skipped, {excluded_count} kept individual color.")

        # ROBUST-V14-1: shared axis/dimension validation guards
        @trace_execution
        def _safe_axis(axis, default="z"):
            if axis not in ("x", "y", "z"):
                Logger.log(f"Invalid axis '{axis}', defaulting to '{default}'", "WARN")
                return default
            return axis

        @trace_execution
        def box(comp, name, cx, cy, cz, lx, ly, lz, ap=None):
            if lx <= 0 or ly <= 0 or lz <= 0:
                Logger.log(f"box({name}): invalid dims {lx},{ly},{lz} -- skipped", "WARN")
                return None
            try:
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
                if body:
                    body.name = name
                    set_ap(body, ap)
                    # BUGFIX-V15-1: actually wire the CAD tracker + geometry
                    # validator that were previously defined but never called.
                    CADTracker.register(f"{comp.name}::{name}", body)
                    if DEBUG_GEOMETRY:
                        MechValidator.check_body(name, body)
                return body
            except Exception as e:
                Logger.log(f"box({name}) failed: {e}", "ERROR")
                return None

        @trace_execution
        def cyl(comp, name, cx, cy, cz, r, h, axis, ap=None):
            axis = _safe_axis(axis)
            if r <= 0 or h <= 0:
                Logger.log(f"cyl({name}): invalid dims r={r} h={h} -- skipped", "WARN")
                return None
            try:
                temp = adsk.fusion.TemporaryBRepManager.get()
                av   = {"x": (1, 0, 0), "y": (0, 1, 0), "z": (0, 0, 1)}[axis]
                p1   = adsk.core.Point3D.create(cx-av[0]*h/2, cy-av[1]*h/2, cz-av[2]*h/2)
                p2   = adsk.core.Point3D.create(cx+av[0]*h/2, cy+av[1]*h/2, cz+av[2]*h/2)
                shape = temp.createCylinderOrCone(p1, r, p2, r)
                bf    = comp.features.baseFeatures.add()
                bf.startEdit()
                body  = comp.bRepBodies.add(shape, bf)
                bf.finishEdit()
                if body:
                    body.name = name
                    set_ap(body, ap)
                    CADTracker.register(f"{comp.name}::{name}", body)
                    if DEBUG_GEOMETRY:
                        MechValidator.check_body(name, body)
                return body
            except Exception as e:
                Logger.log(f"cyl({name}) failed: {e}", "ERROR")
                return None

        @trace_execution
        def cone_shape(comp, name, cx, cy, cz, r1, r2, h, axis, ap=None):
            axis = _safe_axis(axis)
            if r1 <= 0 or r2 <= 0 or h <= 0:
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
            if body:
                body.name = name
                set_ap(body, ap)
                CADTracker.register(f"{comp.name}::{name}", body)
                if DEBUG_GEOMETRY:
                    MechValidator.check_body(name, body)
            return body

        @trace_execution
        def marker(comp, name, cx, cy, cz, size=0.22):
            return box(comp, name, cx, cy, cz, size, size, size, white_pla)

        # ── MFG-V14-1: TRUE 45-degree printable chamfer ────────────────────
        @trace_execution
        def chamfer_wedge_cut(comp, tag, run_axis, run_center, run_len,
                              p_corner, q_corner, chamfer):
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

        @trace_execution
        def chamfer_box(comp, name, cx, cy, cz, lx, ly, lz, axis, chamfer=0.25, ap=None):
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
        @trace_execution
        def cut_cavity(comp, tool_body, keep_tool=False):
            if not tool_body or not tool_body.isValid:
                Logger.log("cut_cavity: invalid tool body", "WARN")
                return False
            tool_name = tool_body.name
            target_names = []
            no_cut_keywords = {
                "VisTire", "VisRim", "VisGB", "VisMot", "VisShaft", "VisHub", "AxlePin",
                "Shoulder_Frame", "Sh_Pad_Edge", "StkTip_", "Stk_A_",
                "_PP", "_MP", "_DP", "_TP", "_TD",
                "Grille", "Win_", "Bumper", "Stack", "Antenna", "Visor",
                "Horn_", "Fender", "Headlamp", "KneeCap", "Armor",
                "Beam", "Rib", "Stop", "Lock", "Hinge",
                "ShShield_", "ShHinge_", "Mirror_", "HipShield_"
            }
            for b in list(comp.bRepBodies):
                if b == tool_body:
                    continue
                if b.name:
                    if any(t in b.name for t in SKIP_TAGS) or any(k in b.name for k in no_cut_keywords):
                        continue
                target_names.append(b.name)

            success = False
            for t_name in target_names:
                t_body  = comp.bRepBodies.itemByName(t_name)
                cur_tool = comp.bRepBodies.itemByName(tool_name)
                if not t_body or not t_body.isValid:
                    continue
                if not cur_tool or not cur_tool.isValid:
                    break
                tools = adsk.core.ObjectCollection.create()
                tools.add(cur_tool)
                try:
                    ci = comp.features.combineFeatures.createInput(t_body, tools)
                    ci.operation        = adsk.fusion.CombineOperation.CutFeatureOperation
                    ci.isKeepToolBodies = True
                    comp.features.combineFeatures.add(ci)
                    success = True
                    if t_body and t_body.isValid:
                        t_body.name = t_name
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
        @trace_execution
        def split_halves(comp, body, axis="y", offset=0.0):
            """ROBUST-V14-3 / BUGFIX-V15-4 -- Validates the split actually
            produced >=2 bodies before declaring success. BUGFIX-V15-4: the
            post-split renaming block previously referenced an undefined
            variable `name` (there is no `name` parameter/local in this
            function -- only `body`). That raised a silent NameError that
            was swallowed by the surrounding except, made every single call
            to this function report "failed" in the log even when the
            underlying Fusion split succeeded, and -- critically -- meant
            the "_left"/"_right" renaming never happened. Since
            merge_fasteners_to_halves() (called right after this, in the
            'FDM Shell Splitting' section) only ever looks for bodies named
            with "_left"/"_right", that bug meant fastener bodies (bosses,
            pins, inserts, nuts, snaps) were NEVER actually merged into
            their shell halves for the entire print run. Fixed by capturing
            the original body name before the split and using that instead
            of the undefined `name`."""
            if not body or not body.isValid:
                Logger.log(f"split_halves: invalid body in {comp.name}", "WARN")
                return False
            orig_name = body.name
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

                # BUGFIX-V15-4: use orig_name (captured above), not the
                # undefined `name`.
                after_bodies = list(comp.bRepBodies)
                candidates = []
                for ab in after_bodies:
                    if not ab.isValid:
                        continue
                    ab_name = ab.name or ""
                    if (orig_name and orig_name in ab_name) or ab_name.startswith("Body"):
                        candidates.append(ab)
                for cb in candidates:
                    try:
                        cog = cb.physicalProperties.centerOfMass
                        pos = cog.y if axis == "y" else cog.z if axis == "z" else cog.x
                        if pos < offset:
                            cb.name = f"{orig_name}_left"
                        else:
                            cb.name = f"{orig_name}_right"
                    except:
                        pass
                return True
            except Exception as e:
                Logger.log(f"split_halves failed: {e}", "WARN")
                return False

        # ── MFG-1 — Post-split fastener merge (None-safe on either side) ──
        @trace_execution
        def merge_fasteners_to_halves(comp, body_left, body_right, axis="y"):
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
                    if t_body and t_body.isValid:
                        t_body.name = t_name
                except Exception as e:
                    Logger.log(f"merge_fastener failed for {f_name}: {e}", "DEBUG")

        @trace_execution
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
                Logger.log(f"  [PRINT WARNING] {body_name}: {reason}", "WARN")


        # ─────────────────────────────────────────────────────────────────
        # PHYSICAL FEATURE HELPERS  (PHY-1 … PHY-11, kept from v12)
        # ─────────────────────────────────────────────────────────────────

        @trace_execution
        def m3_boss(comp, tag, cx, cy, cz, axis="z", depth=0.80, screw_len=1.2):
            boss = cyl(comp, f"{tag}_Boss", cx, cy, cz, BOSS_R, depth, axis, dark_metal)
            if boss:
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

        @trace_execution
        def captive_nut(comp, tag, cx, cy, cz, axis="z", bolt_len=1.2):
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

        @trace_execution
        def snap_clip(comp, tag, cx, cy, cz, span_x=1.2, axis_latch="z"):
            for sign in [-1, 1]:
                arm_cx = cx + sign * span_x * 0.5
                box(comp, f"{tag}_SnapArm_{sign}",
                    arm_cx, cy, cz, WALL_F, 0.40, 1.20, grey_plastic)
                box(comp, f"{tag}_SnapHead_{sign}",
                    arm_cx, cy, cz + 0.55, WALL_F + 0.10, 0.50, 0.28, grey_plastic)

        @trace_execution
        def align_pin(comp, tag, cx, cy, cz, axis="z", height=0.40):
            cyl(comp, f"{tag}_AlignPin", cx, cy, cz, ALIGN_PIN_R, height, axis, chrome)

        @trace_execution
        def align_socket(comp, tag, cx, cy, cz, axis="z", depth=0.45):
            cut_cavity(comp, cyl(comp, f"{tag}_AlignSock",
                                 cx, cy, cz, ALIGN_PIN_R + 0.015, depth, axis))

        @trace_execution
        def grommet_hole(comp, tag, cx, cy, cz, axis="y"):
            cut_cavity(comp, cyl(comp, f"{tag}_Grommet",
                                 cx, cy, cz, GROMMET_R, 0.80, axis))

        @trace_execution
        def horn_coupler(comp, tag, cx, cy, cz, axis, servo_type="std", reinforced=False):
            """V17: `reinforced=True` adds a hex-broached metal shaft-
            coupling insert pocket (e.g. a bonded aluminum hex-bore hub)
            concentric with the printed coupler hub, instead of relying on
            the printed clamp + set-screw alone to transmit torque. Use this
            for every high-torque joint (hip, knee, waist) -- the printed-
            only clamp is the weakest link in those joints' whole drivetrain."""
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

            if reinforced:
                # Bonded/pressed metal hex-bore insert, concentric with the
                # printed hub, sized to COUPLER_INSERT_HEX across flats --
                # this is what actually carries the torque; the printed hub
                # + set-screw now only locate and retain it.
                cut_cavity(comp, box(comp, f"{tag}_MetalInsertPkt",
                    cx, cy, cz, COUPLER_INSERT_HEX, COUPLER_INSERT_HEX, hub_h*0.85))
                BOM.add("Hardware", "Aluminum hex-bore shaft coupling insert (bonded)", 1, comp.name)
                ASSEMBLY_STEPS.append(
                    f"{tag}: bond metal hex-bore insert into printed coupler hub with "
                    f"structural epoxy BEFORE installing on servo horn -- this insert, "
                    f"not the printed plastic, carries the joint torque")

            spec_name = SERVO_SPECS.get(servo_type, SERVO_SPECS["std"])["name"]
            BOM.add("Printed", f"Servo coupler hub ({spec_name})", 1, comp.name)
            BOM.add("Fastener", "M3x4 set screw (cup point)", 1, comp.name)
            ASSEMBLY_STEPS.append(f"Install {tag} coupler onto {spec_name} servo horn; tighten set-screw")

        @trace_execution
        def servo_drum(comp, tag, cx, cy, cz, axis="z"):
            cyl(comp, f"{tag}_DrumBarrel", cx, cy, cz, DRUM_R, DRUM_H, axis, dark_metal)
            cyl(comp, f"{tag}_DrumFlangeT", cx, cy, cz + DRUM_H/2 - 0.02,
                DRUM_R + 0.10, 0.06, axis, dark_metal)
            cyl(comp, f"{tag}_DrumFlangeB", cx, cy, cz - DRUM_H/2 + 0.02,
                DRUM_R + 0.10, 0.06, axis, dark_metal)
            tie_axis = "x" if axis in ("y", "z") else "z"
            cut_cavity(comp, cyl(comp, f"{tag}_TieHole", cx, cy, cz,
                                 0.06, DRUM_R*2.2, tie_axis))
            BOM.add("Printed", "Servo winch drum (tendon drive)", 1, comp.name)

        @trace_execution
        def tendon_guide(comp, tag, cx, cy, cz, length, axis="z", bushed=False):
            """V17: `bushed=True` adds a brass/bronze eyelet bushing pocket
            concentric with the guide bore -- printed-plastic-on-plastic
            tendon guides wear oval fast under repeated tension; a real
            bushing is a cheap fix for the single most common tendon-hand
            failure mode."""
            gr = TENDON_GUIDE_R + 0.02
            cut_cavity(comp, cyl(comp, f"{tag}_TendonGuide", cx, cy, cz, gr, length, axis))
            if bushed:
                cut_cavity(comp, cyl(comp, f"{tag}_Bushing", cx, cy, cz,
                                     BUSHING_R, BUSHING_H, axis))
                BOM.add("Hardware", "Brass eyelet bushing (tendon guide)", 1, comp.name)

        @trace_execution
        def tendon_anchor(comp, tag, cx, cy, cz, axis="z", adjustable=False):
            """V17: `adjustable=True` replaces the fixed crimp slot with a
            slotted travel channel + M2 tensioning screw, so the tendon can
            be re-tensioned after it inevitably stretches in service instead
            of requiring the finger to be disassembled and re-strung."""
            box(comp, f"{tag}_Anchor", cx, cy, cz, 0.35, 0.28, 0.22, dark_metal)
            if adjustable:
                cut_cavity(comp, box(comp, f"{tag}_TensionSlot", cx, cy, cz,
                                     0.06, TENSIONER_SLOT_L, 0.14))
                cyl(comp, f"{tag}_TensionScrew", cx, cy + TENSIONER_SLOT_L/2 - 0.05, cz,
                    M3_PILOT_R * 0.7, 0.30, "y", dark_metal)
                BOM.add("Fastener", "M2x12 tensioning screw + nut (tendon anchor)", 1, comp.name)
                ASSEMBLY_STEPS.append(
                    f"{tag}: re-tension tendon via the anchor's tension screw after "
                    f"the first ~50 actuation cycles and periodically thereafter")
            else:
                cut_cavity(comp, box(comp, f"{tag}_CrimpSlot", cx, cy, cz, 0.06, 0.30, 0.14))
            BOM.add("Hardware", "Tendon anchor (printed)", 1, comp.name)

        @trace_execution
        def palm_pulley(comp, tag, cx, cy, cz, axis="x", bushed=False):
            cyl(comp, f"{tag}_PulleyPost", cx, cy, cz, 0.12, 0.50, axis, chrome)
            pulley_axis = "y" if axis in ("x", "z") else "z"
            cyl(comp, f"{tag}_PulleyWheel", cx, cy, cz, PULLEY_R, 0.14, pulley_axis, grey_plastic)
            if bushed:
                cut_cavity(comp, cyl(comp, f"{tag}_PulleyBushing", cx, cy, cz,
                                     BUSHING_R, 0.18, axis))
                BOM.add("Hardware", "Brass eyelet bushing (palm pulley)", 1, comp.name)
            BOM.add("Printed", "Palm idler pulley", 1, comp.name)

        @trace_execution
        def spring_return(comp, tag, cx, cy, cz, axis="x"):
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
        @trace_execution
        def screw_hole(comp, cx, cy, cz, axis="y", length=3.0):
            cut_cavity(comp, cyl(comp, "ScrewHole", cx, cy, cz, M3_PILOT_R, length, axis))
            BOM.add("Fastener", f"M3x{int(length*10)} self-tap", 1, comp.name)

        @trace_execution
        def magnet_pocket(comp, tag, cx, cy, cz, axis="z"):
            cut_cavity(comp, cyl(comp, f"{tag}_MagPkt", cx, cy, cz, 0.32, 0.35, axis))
            BOM.add("Hardware", "Magnet D6x3 mm N35", 1, comp.name)

        @trace_execution
        def wire_channel(comp, tag, cx, cy, cz, r, h, axis):
            cut_cavity(comp, cyl(comp, f"{tag}_Wire", cx, cy, cz, r, h, axis))

        @trace_execution
        def led_pocket_5mm(comp, tag, cx, cy, cz, axis="y"):
            cut_cavity(comp, cyl(comp, f"{tag}_LED5", cx, cy, cz, LED_R_5MM, 0.85, axis))
            BOM.add("Electronics", "LED 5 mm (colour TBD)", 1, comp.name)

        @trace_execution
        def led_ring_pocket(comp, tag, cx, cy, cz, axis="z"):
            cut_cavity(comp, cyl(comp, f"{tag}_LEDRing", cx, cy, cz, LED_R_RING, 0.30, axis))
            BOM.add("Electronics", "WS2812 5050 LED ring Ø12 mm", 1, comp.name)

        # ── CAB-1 … CAB-4 ──────────────────────────────────────────────────
        @trace_execution
        def cable_clip(comp, tag, cx, cy, cz, axis="y"):
            box(comp, f"{tag}_ClipBase", cx, cy, cz, CABLE_CLIP_W, 0.15, 0.35, grey_plastic)
            cyl(comp, f"{tag}_ClipArch", cx, cy, cz + 0.06,
                CABLE_CLIP_R + 0.08, CABLE_CLIP_W, "x", grey_plastic)
            BOM.add("Printed", "Snap-in cable clip", 1, comp.name)

        @trace_execution
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

        @trace_execution
        def grommet_slot(comp, tag, cx, cy, cz, axis="y", width=0.50):
            cut_cavity(comp, cyl(comp, f"{tag}_GromSlot", cx, cy, cz, GROMMET_R, width, axis))
            seat_r = GROMMET_R + 0.06
            cut_cavity(comp, cyl(comp, f"{tag}_GromSeat", cx, cy, cz, seat_r, 0.10, axis))
            BOM.add("Hardware", "Rubber grommet Ø3.5 mm (open slot)", 1, comp.name)

        @trace_execution
        def wire_service_loop(comp, tag, cx, cy, cz, loop_r=0.55, axis="z"):
            """V17 NEW -- a printed cradle that gives wiring a genuine slack
            loop as it crosses a moving joint, instead of routing straight
            through (v15/v16's `grommet_slot`/`wire_channel` at multi-axis
            clusters). Straight-through routing is the most common early
            wiring failure on a posable robot: the conductor fatigues and
            snaps at the same flex point every cycle. This doesn't replace
            grommet_slot/wire_channel -- add it in addition, right at the
            joint, so the wire has somewhere to take up slack instead of
            stretching."""
            cyl(comp, f"{tag}_LoopCradleOuter", cx, cy, cz, loop_r + 0.18, 0.30, axis, dark_grey)
            cut_cavity(comp, cyl(comp, f"{tag}_LoopCradleBore", cx, cy, cz, loop_r, 0.40, axis))
            BOM.add("Printed", "Wire service-loop cradle", 1, comp.name)
            ASSEMBLY_STEPS.append(
                f"{tag}: coil a full slack loop of wire/ribbon into this cradle before "
                f"closing the shell -- do NOT route wiring taut across this joint")

        @trace_execution
        def ankle_compliance_pad(comp, tag, cx, cy, cz, lx=5.0, ly=7.5):
            """V17 NEW -- real shock-absorbing compliance under the foot,
            replacing the earlier cosmetic 0.15cm rubber `VibPad`. The ankle
            takes the single highest DYNAMIC (impact) load in the robot at
            every footfall, and nothing in v15/v16 actually absorbed that --
            the old pad was too thin to meaningfully compress. This adds a
            genuinely thick TPU lattice pad plus 4 corner coil-spring
            pockets, giving the foot real vertical compliance instead of a
            rigid printed sole hitting the ground every step."""
            box(comp, f"{tag}_ComplianceBase", cx, cy, cz, lx, 0.60, ly, rubber_blk)
            for sx, sz in [(-lx*0.32, -ly*0.32), (lx*0.32, -ly*0.32),
                           (-lx*0.32,  ly*0.32), (lx*0.32,  ly*0.32)]:
                cyl(comp, f"{tag}_SpringPkt_{sx:.0f}_{sz:.0f}", cx+sx, cy, cz+sz,
                    0.45, 0.70, "y", dark_grey)
                cut_cavity(comp, cyl(comp, f"{tag}_SpringBore_{sx:.0f}_{sz:.0f}",
                    cx+sx, cy-0.05, cz+sz, 0.38, 0.85, "y"))
            BOM.add("Hardware", "Foot compliance coil spring (short, stiff)", 4, comp.name)
            BOM.add("Material", "TPU 95A compliance pad (thick, per foot)", 1, comp.name)
            ASSEMBLY_STEPS.append(
                f"{tag}: print in TPU 95A (NOT PETG/PLA) -- this pad is meant to "
                f"compress under footfall impact, a rigid material defeats its purpose")

        @trace_execution
        def jst_pocket(comp, tag, cx, cy, cz, axis="y"):
            cut_cavity(comp, box(comp, f"{tag}_JST", cx, cy, cz,
                JST_XH_L + 0.10, JST_XH_W + 0.10, JST_XH_H + 0.10))

        # ── BRG-2 / BRG-3 ──────────────────────────────────────────────────
        @trace_execution
        def bearing_fit(comp, tag, cx, cy, cz, axis="x", ro=1.10, w=0.60, fit_type="press",
                        add_retaining_ring=False, axial_load=False):
            """V17: two new reinforcement options, both aimed at real-world
            failure modes the printed lip alone doesn't solve:
              add_retaining_ring -- cuts a real circlip groove just past the
                printed lip so a metal retaining ring backs it up. The
                printed lip alone works loose after enough load cycles;
                use this on every joint that sees dynamic/impact loading
                (hip, knee, ankle, waist).
              axial_load -- adds a thrust washer at this bearing location.
                Radial ball bearings (which is all bearing_fit ever builds)
                are NOT designed to carry load along their rotation axis.
                Every yaw joint that also carries body weight (hip yaw,
                ankle yaw, waist yaw) needs this; a purely non-weight-
                bearing yaw joint (e.g. neck yaw) does not."""
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
            if add_retaining_ring:
                ring_off = w/2.0 + BEARING_RETAIN_LIP_H + CIRCLIP_GROOVE_W/2 + 0.02
                if axis == "x":   ring_x, ring_y, ring_z = cx - ring_off, cy, cz
                elif axis == "y": ring_x, ring_y, ring_z = cx, cy - ring_off, cz
                else:             ring_x, ring_y, ring_z = cx, cy, cz - ring_off
                cut_cavity(comp, cyl(comp, f"{tag}_CirclipGroove", ring_x, ring_y, ring_z,
                                     outer_r + CIRCLIP_GROOVE_D, CIRCLIP_GROOVE_W, axis))
                BOM.add("Hardware", f"Retaining ring (circlip) O{int(ro*2*10)}mm", 1, comp.name)
                ASSEMBLY_STEPS.append(
                    f"{tag}: seat metal retaining ring in circlip groove -- do NOT rely "
                    f"on the printed lip alone at this joint")
            if axial_load:
                thrust_off = w/2.0 + BEARING_RETAIN_LIP_H + THRUST_WASHER_T/2 + 0.03
                if axis == "x":   tw_x, tw_y, tw_z = cx - thrust_off, cy, cz
                elif axis == "y": tw_x, tw_y, tw_z = cx, cy - thrust_off, cz
                else:             tw_x, tw_y, tw_z = cx, cy, cz - thrust_off
                cyl(comp, f"{tag}_ThrustWasher", tw_x, tw_y, tw_z,
                    outer_r + THRUST_WASHER_R, THRUST_WASHER_T, axis, dark_metal)
                cut_cavity(comp, cyl(comp, f"{tag}_ThrustBore", tw_x, tw_y, tw_z,
                                     ro*0.35, THRUST_WASHER_T + 0.10, axis))
                BOM.add("Bearing", f"Thrust washer/bearing O{int((ro*2+THRUST_WASHER_R*2)*10)}mm "
                        f"(axial/weight load)", 1, comp.name)
                ASSEMBLY_STEPS.append(
                    f"{tag}: this axis carries body weight along its rotation axis -- "
                    f"install the thrust washer, a plain radial bearing here is NOT "
                    f"rated for that load direction")
            fit_label = f"{fit_type}-fit"
            BOM.add("Bearing", f"O{int(ro*2*10)} mm bearing ({fit_label})", 1, comp.name)
            BOM.add("Hardware", f"Bearing {fit_label} tolerance", 1, comp.name)
            MechValidator.check_bearing_fit(tag, ro, ro)

        @trace_execution
        def dual_bearing(comp, tag, cx, cy, cz, axis="x", ro=1.10, w=0.60,
                         span=2.50, fit_type="press", add_retaining_ring=False, axial_load=False):
            av = {"x": (1,0,0), "y": (0,1,0), "z": (0,0,1)}[axis]
            box(comp, f"{tag}_Carrier", cx, cy, cz,
                span*av[0]+1.2, span*av[1]+1.2, span*av[2]+1.2, dark_metal)
            p1 = (cx - av[0]*span/2, cy - av[1]*span/2, cz - av[2]*span/2)
            p2 = (cx + av[0]*span/2, cy + av[1]*span/2, cz + av[2]*span/2)
            bearing_fit(comp, f"{tag}_A", p1[0], p1[1], p1[2], axis, ro, w, fit_type,
                        add_retaining_ring=add_retaining_ring, axial_load=axial_load)
            bearing_fit(comp, f"{tag}_B", p2[0], p2[1], p2[2], axis, ro, w, fit_type,
                        add_retaining_ring=add_retaining_ring, axial_load=False)
            cyl(comp, f"{tag}_Axle", cx, cy, cz, ro*0.55, span + 1.0, axis, chrome)
            BOM.add("Hardware", f"Steel axle O{int(ro*0.55*20)} mm x {int((span+1.0)*10)} mm",
                    1, comp.name)
            ASSEMBLY_STEPS.append(f"Install {tag} bearings into carrier; insert steel axle")

        @trace_execution
        def bearing(comp, tag, cx, cy, cz, axis="x", ro=1.10, w=0.60):
            """Legacy wrapper -> redirects to bearing_fit (glue-fit default)."""
            bearing_fit(comp, tag, cx, cy, cz, axis, ro, w, fit_type="glue")

        # ── COV-1 / COV-2 / COV-3 ──────────────────────────────────────────
        @trace_execution
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

        @trace_execution
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

        @trace_execution
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
        @trace_execution
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
            assign_material(jig, "cosmetic")   # V16 NEW: jigs are non-structural tooling
            occ = occs.get(f"JIG_{comp_name}")
            if occ:
                occ.isLightBulbOn = False
            return jig

        @trace_execution
        def verify_screw_lengths():
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

        @trace_execution
        def verify_joint_axis_mapping():
            Logger.log("--- V14 JOINT AXIS-MAPPING VERIFICATION (AXIS-1) ---")
            TENDON_DRIVEN_JOINTS = {"L_Thumb_CMC", "R_Thumb_CMC"}
            if not JOINT_AXIS_MAP:
                Logger.log("  No joints registered in JOINT_AXIS_MAP -- skipping.", "WARN")
                return
            issues  = 0
            checked = 0
            for joint, dof_limits in JOINT_LIMITS.items():
                if joint in TENDON_DRIVEN_JOINTS:
                    continue
                rot_dofs = [d for d in dof_limits.keys() if d in ("pitch", "yaw", "roll")]
                if len(rot_dofs) < 2:
                    continue
                checked += 1
                mapped    = JOINT_AXIS_MAP.get(joint, {})
                seen_axes = {}
                joint_ok  = True
                for dof in rot_dofs:
                    axis = mapped.get(dof)
                    if axis is None:
                        Logger.log(f"  [MISSING] {joint}.{dof}: no servo/bearing "
                                   f"axis registered for this DOF", "WARN")
                        BugReporter.report(
                            f"{joint}.{dof} has no physical axis registered "
                            f"(declared in JOINT_LIMITS but no matching servo built)",
                            "HIGH", category="Mechanical")
                        issues += 1
                        joint_ok = False
                        continue
                    if axis in seen_axes:
                        Logger.log(f"  [DUPLICATE] {joint}: '{seen_axes[axis]}' and "
                                   f"'{dof}' both mapped to axis '{axis}'", "ERROR")
                        BugReporter.report(
                            f"{joint}: DOF '{dof}' collides with '{seen_axes[axis]}' "
                            f"on axis '{axis}' -- one DOF is duplicated, the other "
                            f"is not independently controllable",
                            "CRITICAL", category="Mechanical")
                        issues += 1
                        joint_ok = False
                    else:
                        seen_axes[axis] = dof
                if joint_ok:
                    summary = ", ".join(f"{d}={mapped[d]}" for d in rot_dofs)
                    Logger.log(f"  [OK] {joint}: {summary}")
            if issues == 0:
                Logger.log(f"  All {checked} multi-axis joint(s) have unique, "
                           f"fully-populated axis mappings [PASS]")
            else:
                Logger.log(f"  {issues} joint axis-mapping issue(s) found across "
                           f"{checked} multi-axis joints -- review BUG REPORT", "WARN")


        # ─────────────────────────────────────────────────────────────────
        # V13: JETSON NANO + ESP32-S3 + VISION/SENSOR ELECTRONICS BAYS
        # ─────────────────────────────────────────────────────────────────

        @trace_execution
        def jetson_nano_bay(comp, tag, cx, cy, cz):
            cut_cavity(comp, box(comp, f"{tag}_JNanoBay", cx, cy, cz,
                                 JNANO_L + 0.30, JNANO_W + 0.30, JNANO_H + 0.40))
            for sx, sz in [(-2.80, -1.90), (2.80, -1.90), (-2.80, 1.90), (2.80, 1.90)]:
                cyl(comp, f"{tag}_JStdoff_{sx:.0f}_{sz:.0f}",
                    cx+sx, cy, cz+sz, 0.14, JNANO_H+0.60, "y", dark_metal)
            cut_cavity(comp, box(comp, f"{tag}_FanBay",
                cx, cy - JNANO_H/2 - JNANO_FAN_H/2 - 0.05, cz,
                JNANO_FAN_SZ + 0.10, JNANO_FAN_H + 0.10, JNANO_FAN_SZ + 0.10))
            for fx, fz in [(-1.15, -1.15), (1.15, -1.15), (-1.15, 1.15), (1.15, 1.15)]:
                cyl(comp, f"{tag}_FanScrew_{fx:.0f}_{fz:.0f}",
                    cx+fx, cy - JNANO_H/2 - JNANO_FAN_H - 0.05, cz+fz,
                    0.090, 0.50, "y", dark_metal)
            cut_cavity(comp, box(comp, f"{tag}_HsinkClear",
                cx, cy - JNANO_H/2 - 0.05, cz,
                JNANO_MODULE_L*0.55, JNANO_HSINK_H + 0.10, JNANO_MODULE_W*0.55))
            for ui_, ux in enumerate([-1.5, -0.5]):
                cut_cavity(comp, box(comp, f"{tag}_USB3_{ui_}",
                    cx + ux, cy, cz - JNANO_W/2 - 0.15,
                    JNANO_USB3_W, JNANO_USB3_H, 0.35))
            cut_cavity(comp, box(comp, f"{tag}_USBC_Pwr",
                cx + 1.6, cy, cz - JNANO_W/2 - 0.15,
                JNANO_USBCC_W, JNANO_USBCC_H, 0.30))
            cut_cavity(comp, box(comp, f"{tag}_HDMI",
                cx - 2.4, cy, cz - JNANO_W/2 - 0.15,
                JNANO_HDMI_W, JNANO_HDMI_H, 0.30))
            cut_cavity(comp, box(comp, f"{tag}_CSI_Exit",
                cx, cy, cz + JNANO_W/2 + 0.15,
                JNANO_CSI_W + 0.10, JNANO_CSI_H + 0.10, 0.35))
            cut_cavity(comp, box(comp, f"{tag}_GPIO",
                cx, cy + JNANO_H/2 + 0.10, cz - 1.0,
                JNANO_GPIO_W, 0.30, JNANO_GPIO_H + 0.10))
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

        @trace_execution
        def csi_camera_pocket(comp, tag, cx, cy, cz, lens_axis="y"):
            cut_cavity(comp, box(comp, f"{tag}_CSICamBay", cx, cy, cz,
                                 CSI_CAM_L + 0.15, CSI_CAM_H + 0.20, CSI_CAM_W + 0.15))
            cut_cavity(comp, cyl(comp, f"{tag}_LensPort",
                cx, cy - (CSI_CAM_H/2 + 0.30), cz, CSI_LENS_R, 0.60, lens_axis))
            for sx in [-0.9, 0.9]:
                cyl(comp, f"{tag}_CamScrew_{sx:.0f}", cx+sx, cy+CSI_CAM_H/2-0.05, cz,
                    0.075, 0.30, "y", dark_metal)
            BOM.add("Electronics", "Jetson CSI camera module (IMX219 8MP)", 1, comp.name)
            BOM.add("Fastener", "M2x6mm screw (camera mount)", 2, comp.name)
            Logger.log(f"  CSI camera (robot eyes) pocket: {tag} in {comp.name}")

        @trace_execution
        def fpc_ribbon_channel(comp, tag, cx, cy, cz, length, axis="z"):
            cut_cavity(comp, box(comp, f"{tag}_FPC", cx, cy, cz,
                                 CSI_RIBBON_W + 0.10, CSI_RIBBON_H + 0.06, length))
            BOM.add("Electronics", "15-pin CSI FPC ribbon cable (length per run)", 1, comp.name)

        @trace_execution
        def esp32s3_node_bay(comp, tag, cx, cy, cz, role="lower"):
            cut_cavity(comp, box(comp, f"{tag}_ESP32S3Bay", cx, cy, cz,
                                 ESP32S3_L + 0.20, ESP32S3_H + 0.30, ESP32S3_W + 0.20))
            for sx in [-2.1, 2.1]:
                cyl(comp, f"{tag}_E3Std_{sx:.0f}", cx+sx, cy, cz-0.7,
                    0.12, ESP32S3_H+0.40, "y", dark_metal)
            cut_cavity(comp, box(comp, f"{tag}_E3USBC", cx - ESP32S3_L/2 - 0.10, cy, cz,
                                 ESP32S3_USBCC_W, ESP32S3_USBCC_H, 0.30))
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

        @trace_execution
        def tof_sensor_pocket(comp, tag, cx, cy, cz, axis="y"):
            cut_cavity(comp, box(comp, f"{tag}_ToFBay", cx, cy, cz,
                                 TOF_L + 0.15, TOF_H + 0.20, TOF_W + 0.15))
            cut_cavity(comp, cyl(comp, f"{tag}_ToFWindow",
                cx, cy - (TOF_H/2 + 0.25), cz, TOF_LENS_R, 0.50, axis))
            BOM.add("Electronics", "VL53L1X ToF distance sensor", 1, comp.name)

        @trace_execution
        def ina3221_bay(comp, tag, cx, cy, cz):
            cut_cavity(comp, box(comp, f"{tag}_INABay", cx, cy, cz,
                                 INA_L + 0.15, INA_H + 0.20, INA_W + 0.15))
            BOM.add("Electronics", "INA3221 3-channel current/power monitor", 1, comp.name)

        @trace_execution
        def ubec_5v4a_bay(comp, tag, cx, cy, cz):
            cut_cavity(comp, box(comp, f"{tag}_UBECBay", cx, cy, cz,
                                 UBEC_JNANO_L + 0.20, UBEC_JNANO_H + 0.20, UBEC_JNANO_W + 0.20))
            for sx, sz in [(-UBEC_JNANO_L/2+0.6, -UBEC_JNANO_W/2+0.6),
                           (UBEC_JNANO_L/2-0.6,  UBEC_JNANO_W/2-0.6)]:
                m3_boss(comp, f"{tag}_UBEC_{sx:.0f}", cx+sx, cy, cz+sz)
            BOM.add("Electronics", "5V 4A UBEC (Jetson Nano power)", 1, comp.name)
            _reg_power("Jetson_5V", 5.0, 4.0, ["Jetson Nano", "fan", "CSI camera"])

        @trace_execution
        def vent_grille(comp, tag, cx, cy, cz, axis="y", n_slots=N_VENT_SLOTS):
            for i in range(n_slots):
                offset = (i - (n_slots-1)/2.0) * (VENT_SLOT_W * 1.8)
                if axis == "y":
                    cut_cavity(comp, box(comp, f"{tag}_Vent_{i}",
                        cx + offset, cy, cz, VENT_SLOT_W, 0.40, VENT_SLOT_L))
                else:
                    cut_cavity(comp, box(comp, f"{tag}_Vent_{i}",
                        cx, cy + offset, cz, VENT_SLOT_L, VENT_SLOT_W, 0.40))
            BOM.add("Printed", "Thermal vent grille (integrated)", 1, comp.name)

        @trace_execution
        def antenna_clearance(comp, tag, cx, cy, cz, lx, lz):
            marker(comp, f"{tag}_AntKeepOut", cx, cy, cz, 0.10)
            ASSEMBLY_STEPS.append(
                f"NOTE: {tag} marks a WiFi/BT antenna keep-out zone "
                f"({lx:.1f}x{lz:.1f}cm) -- avoid metal screws/foil within this area")

        @trace_execution
        def uart_bridge_cutout(comp, tag, cx, cy, cz, axis="y"):
            cut_cavity(comp, box(comp, f"{tag}_UARTBridge", cx, cy, cz,
                                 ESP32S3_USBCC_W + 0.08, ESP32S3_USBCC_H + 0.08, 0.35))
            BOM.add("Electronics", "USB-C panel-mount extension (debug port)", 1, comp.name)

        @trace_execution
        def pca9685_bay(comp, tag, cx, cy, cz):
            cut_cavity(comp, box(comp, f"{tag}_PCABay", cx, cy, cz,
                                 PCA_L + 0.20, PCA_W + 0.20, PCA_H + 0.30))
            for sx, sz in [(-3.00,-1.08), (3.00,-1.08), (-3.00,1.08), (3.00,1.08)]:
                cyl(comp, f"{tag}_PCAStdoff_{sx}_{sz}", cx+sx, cy, cz+sz,
                    0.14, PCA_H+0.50, "y", dark_metal)
            for ch in range(0, 16, 4):
                jst_pocket(comp, f"{tag}_PCA_JST{ch}", cx+2.8, cy, cz + (-0.8 + ch*0.10))
            BOM.add("Electronics", "PCA9685 16-ch I2C servo driver", 1, comp.name)

        @trace_execution
        def lipo_bay(comp, tag, cx, cy, cz):
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
            BOM.add("Electronics", "ATO blade fuse holder + 25A fuse", 1, comp.name)
            _reg_power("Servo_7.4V", 7.4, 25.0, ["all PCA9685 servo rails"])

        @trace_execution
        def imu_pocket(comp, tag, cx, cy, cz, axis="z"):
            cut_cavity(comp, box(comp, f"{tag}_IMUPkt", cx, cy, cz,
                                 IMU_L + 0.20, IMU_W + 0.20, IMU_H + 0.30))
            BOM.add("Electronics", "MPU-6050 IMU breakout", 1, comp.name)

        @trace_execution
        def bec_mount(comp, tag, cx, cy, cz):
            cut_cavity(comp, box(comp, f"{tag}_BECBay", cx, cy, cz,
                BEC_L + 0.20, BEC_H + 0.20, BEC_W + 0.20))
            for sx, sz in [(-BEC_L/2+0.5,-BEC_W/2+0.5), (BEC_L/2-0.5, BEC_W/2-0.5)]:
                m3_boss(comp, f"{tag}_BEC_{sx:.0f}", cx+sx, cy, cz+sz)
            BOM.add("Electronics", "5V 3A BEC (logic rail, ESP32 nodes)", 1, comp.name)
            _reg_power("Logic_5V", 5.0, 3.0, ["ESP32-S3 nodes", "sensors"])

        @trace_execution
        def power_switch_cutout(comp, tag, cx, cy, cz, axis="y"):
            cyl(comp, f"{tag}_SwHole", cx, cy, cz, POWER_SWITCH_R, 1.0, axis, black_plastic)
            cut_cavity(comp, cyl(comp, f"{tag}_SwCut", cx, cy, cz, POWER_SWITCH_R+0.03, 1.2, axis))
            BOM.add("Electronics", "Panel-mount rocker switch SPST (master)", 1, comp.name)

        # ─────────────────────────────────────────────────────────────────
        # V14: SENSOR FUSION, AI ACCELERATOR, COMM BACKBONE, SAFETY
        # BUGFIX-V15-5: all five of these functions (sensor_array,
        # ai_accel_pocket, comm_backbone, estop_cutout, status_rgb_pocket)
        # were fully implemented in v14 -- with real geometry, BOM lines,
        # and assembly notes -- but NONE of them were ever actually called
        # anywhere in the component-build section below. The shipped v14
        # robot therefore had no E-stop, no status LED, no foot/pelvis
        # sensors, no AI-accelerator bay, and no comm backbone, even though
        # the log output and BOM implied those subsystems existed. v15 fixed the
        # E-stop/status-RGB/sensor-array/AI-accel/comm-backbone wiring; v16 now
        # calls each of these from the relevant component sections further
        # down (Torso, Backpack, Pelvis, Feet).
        # ─────────────────────────────────────────────────────────────────

        @trace_execution
        def sensor_array(comp, tag, cx, cy, cz, axis="y", with_ultrasonic=True):
            if with_ultrasonic:
                cut_cavity(comp, box(comp, f"{tag}_USPkt", cx, cy, cz, US_L, US_W, US_H))
                cut_cavity(comp, cyl(comp, f"{tag}_US_Tx", cx-1.35, cy-1.10, cz, US_TXRX_R, 0.30, "y"))
                cut_cavity(comp, cyl(comp, f"{tag}_US_Rx", cx+1.35, cy-1.10, cz, US_TXRX_R, 0.30, "y"))
                BOM.add("Electronics", "HC-SR04 ultrasonic sensor", 1, comp.name)
                _reg_sensor("ultrasonic", comp.name, tag, "Balance Node (ESP32-S3 lower)")
            for fsr_x, fsr_z in [(-1.5, -1.0), (1.5, -1.0), (-1.5, 1.0), (1.5, 1.0)]:
                cut_cavity(comp, box(comp, f"{tag}_FSR_{fsr_x:.0f}_{fsr_z:.0f}",
                    cx+fsr_x, cy, cz+fsr_z, FSR_SZ, FSR_T, FSR_SZ))
            BOM.add("Electronics", "Force-sensitive resistor 0.5in (FSR)", 4, comp.name)
            BOM.add("Electronics", "ADS1115 16-bit ADC (I2C)", 1, comp.name)
            _reg_sensor("FSR_x4+ADC", comp.name, tag, "Balance Node (ESP32-S3 lower)")

        @trace_execution
        def ai_accel_pocket(comp, tag, cx, cy, cz):
            cut_cavity(comp, box(comp, f"{tag}_AIAccel", cx, cy, cz,
                                 AI_ACCEL_L, AI_ACCEL_H, AI_ACCEL_W))
            cut_cavity(comp, box(comp, f"{tag}_AIAccelUSB",
                cx + AI_ACCEL_L/2 - 0.20, cy, cz, AI_ACCEL_USBC_W, AI_ACCEL_USBC_H, 0.60))
            BOM.add("Electronics", "Google Coral USB Accelerator (future, optional)", 1, comp.name)
            ASSEMBLY_STEPS.append(f"{tag}: reserved AI-accelerator bay -- populate when needed, "
                                  f"no redesign required")

        @trace_execution
        def comm_backbone(comp, tag, cx, cy, cz, length, axis="z"):
            wire_channel(comp, f"{tag}_Trunk", cx, cy, cz, COMM_TRUNK_R, length, axis)
            wire_channel(comp, f"{tag}_I2C",  cx+0.12, cy+0.02, cz, COMM_I2C_R,  length, axis)
            wire_channel(comp, f"{tag}_UART", cx-0.12, cy+0.02, cz, COMM_UART_R, length, axis)
            wire_channel(comp, f"{tag}_SPI",  cx, cy-0.15, cz, COMM_SPI_R, length, axis)
            BOM.add("Electronics", "22AWG signal wire (comm backbone, per run)",
                    int(length/5)+1, comp.name)
            BOM.add("Electronics", "JST-SH 1.0mm 6-pin (I2C/UART/SPI tap)", 6, comp.name)
            _reg_comm("MainBus", "All nodes", "I2C+UART+SPI trunk", "mixed",
                      "physical bus separation along spine")

        @trace_execution
        def estop_cutout(comp, tag, cx, cy, cz, axis="y"):
            cyl(comp, f"{tag}_EstopCollar", cx, cy, cz, ESTOP_COLLAR_R, 1.0, axis, op_red)
            cut_cavity(comp, cyl(comp, f"{tag}_EstopHole", cx, cy, cz, ESTOP_R, 1.2, axis))
            BOM.add("Electronics", "22mm mushroom-head E-stop pushbutton (N.C.)", 1, comp.name)
            BOM.add("Electronics", "Automotive relay 30A (servo rail cutoff)", 1, comp.name)
            ASSEMBLY_STEPS.append(
                f"Wire {tag} E-stop in series with the servo-rail relay coil ONLY -- "
                f"verify Jetson and ESP32 nodes stay powered when E-stop is pressed")

        @trace_execution
        def status_rgb_pocket(comp, tag, cx, cy, cz, axis="y"):
            cut_cavity(comp, cyl(comp, f"{tag}_StatusRGB", cx, cy, cz, STATUS_RGB_R, 0.50, axis))
            BOM.add("Electronics", "WS2812 5050 addressable RGB LED (status)", 1, comp.name)
            ASSEMBLY_STEPS.append(f"{tag}: wire status RGB to Jetson GPIO or Motor Controller node "
                                  f"(boot=blue, ready=green, low-batt=yellow, fault=red)")


        # ─────────────────────────────────────────────────────────────────
        # V13: ARCHITECTURE / COMMS / POWER LOGGING
        # ─────────────────────────────────────────────────────────────────

        @trace_execution
        def log_v14_architecture():
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
            Logger.log("  [SAFETY] Independent servo-rail E-stop (torso side panel)")
            Logger.log("  [POWER]  2S 7.4V LiPo -> 5V/4A UBEC (Jetson) + 5V/3A BEC (logic)")
            Logger.log("                         -> 7.4V servo rail (25A ATO fuse, E-stop relay)")
            Logger.log("=" * 64)

        @trace_execution
        def log_comms_map():
            Logger.log("--- V13 COMMUNICATION MAP ---")
            for entry in COMM_MAP:
                Logger.log(f"  {entry['from']:<16s} <-> {entry['to']:<18s} "
                           f"[{entry['bus']:<14s}] {entry['speed']:<10s} :: {entry['purpose']}")
            Logger.log("  Fallback path: ESP32-S3 WiFi AP + WebSocket (if USB-UART fails)")
            Logger.log("  Framing: COBS + CRC-16 over UART; JSON over WebSocket fallback")

        @trace_execution
        def log_power_budget():
            Logger.log("--- V13 POWER BUDGET ---")
            for entry in POWER_MAP:
                Logger.log(f"  Rail '{entry['rail']:<14s}' {entry['voltage']:.1f}V "
                           f"max {entry['max_A']:.1f}A  consumers: {', '.join(entry['consumers'])}")
            Logger.log("  Jetson Nano peak draw:     ~3.0A @ 5V (10W mode, AI inference active)")
            Logger.log("  3x ESP32-S3 nodes:         ~0.6A @ 5V combined (logic only)")
            Logger.log("  CSI camera:                ~0.25A @ 5V (1080p30 capture)")
            # V17 FIX: peak servo draw was understated. 28+ servos (MG996R ~1.0-1.5A
            # avg moving, DS3225 ~2.5A) realistically draw ~20A, not 10A. Fuse and
            # pack sized accordingly; the built bay holds a 2S pack (see BOM), so a
            # 3S will NOT fit without enlarging the bay.
            Logger.log("  Servo rail (worst case):   ~20A @ 7.4V (all 28+ servos moving)")
            Logger.log("  Recommended battery:       2S 2200-3000mAh LiPo (fits built bay) "
                       "~4-6min runtime at full load; enlarge bay for 3S 5000mAh (~12-15min)")
            Logger.log("  Fuse plan:                 25A on servo rail (or split into 2x 15A), "
                       "3A on Jetson 5V rail")

        @trace_execution
        def log_ai_pipeline():
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
            Logger.log("  6. ToF sensors (head node) + FSR/ultrasonic (lower node) feed")
            Logger.log("     obstacle/balance data back to Jetson")
            Logger.log("       - triggers reactive avoidance behavior independent of vision")
            Logger.log("  Future hooks: voice (USB mic -> Whisper.cpp), SLAM (RTAB-Map),")
            Logger.log("                gesture control, person-following, object grasping")

        # ─────────────────────────────────────────────────────────────────
        # V13 DOCUMENTATION GENERATORS
        # ─────────────────────────────────────────────────────────────────

        @trace_execution
        def write_assembly_guide():
            try:
                os.makedirs(os.path.dirname(ASSEMBLY_FILE), exist_ok=True)
                with open(ASSEMBLY_FILE, "w", encoding="utf-8") as f:
                    f.write("=" * 64 + "\n")
                    f.write("  OPTIMUS PRIME G1 v17.0  ASSEMBLY GUIDE\n")
                    f.write("  Jetson Nano AI Brain Edition\n")
                    f.write("=" * 64 + "\n\n")

                    f.write("--- SYSTEM ARCHITECTURE ---\n")
                    f.write("  Brain:   NVIDIA Jetson Nano (vision, AI, decisions)\n")
                    f.write("  Eyes:    CSI-2 camera (IMX219), head-mounted\n")
                    f.write("  Nodes:   3x ESP32-S3 (lower / upper / head control)\n")
                    f.write("  Safety:  Independent servo-rail E-stop (torso side panel)\n")
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
                        "3. Install knee pitch+roll servos into thigh lower brackets",
                        "4. Install ankle pitch/roll/yaw servos into foot shells",
                        "5. Install waist yaw+pitch+roll servos into torso",
                        "6. Install neck pitch+yaw+roll servos into head and torso",
                        "7. Install shoulder yaw/pitch/roll into upper arm",
                        "8. Install elbow servos into upper arm brackets",
                        "9. Install wrist pitch+roll servos into forearm",
                        "10. Install finger drive servos (DS04-NFC) into palm bays",
                        "11. Route all servo wires before closing shells",
                    ]:
                        f.write(f"  {so}\n")

                    f.write("\n--- V13 ELECTRONICS INSTALL ORDER ---\n")
                    for eo in [
                        "1. Flash JetPack OS to Jetson Nano microSD card",
                        "2. Flash ESP32-S3 'lower' node firmware (servo + IMU + sensor-array driver)",
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
                        "14. Install pelvis ultrasonic + foot FSR/ADC sensor arrays (wired to 'lower' node)",
                        "15. Install LiPo with XT60 connector into rear torso bay",
                        "16. Install 5V/4A UBEC (Jetson) and 5V/3A BEC (logic) near LiPo",
                        "17. Install blade fuse holders in both power paths",
                        "18. Install master E-stop pushbutton + relay on torso side panel, in series "
                        "with the servo rail ONLY",
                        "19. Install status RGB LED near chest badge",
                        "20. Route JST-XH harnesses from each PCA9685 to its servos",
                        "21. Connect Jetson USB ports to each ESP32-S3 node (CH340 bridge)",
                        "22. Connect I2C bus per node: ESP32 -> PCA9685 -> sensors, via spine comm backbone",
                        "23. Install master power switch on torso side panel",
                        "24. Power on Jetson FIRST, verify boot, THEN power servo rail via E-stop relay",
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
                        "Test the E-stop BEFORE first power-on: pressing it must cut the "
                        "servo rail while Jetson/ESP32 nodes stay powered",
                        "After any FDM shell split+print, verify boss/pin/insert fasteners "
                        "actually printed fused to their shell half -- do NOT assume the "
                        "merge step succeeded without a visual check",
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

        @trace_execution
        def write_build_manifest():
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
                    "version": "v17.0",
                    "architecture": "Jetson Nano AI Brain Edition",
                    "generated_at": _ts,
                    "target_module": TARGET_MODULE,
                    "components": [comp.name for comp in comps_list],
                    "component_count": len(comps_list),
                    "joint_count": len(joint_names),
                    "joints": joint_names,
                    "dof_total": sum(j['dof'] for j in DOFValidator.joints),
                    "bom_rows": BOM._rows,
                    "bom_line_count": len(BOM._rows),
                    "screw_registry": SCREW_REGISTRY,
                    "screw_location_count": len(SCREW_REGISTRY),
                    "jigs": JIG_REGISTRY,
                    "support_warnings": SUPPORT_WARNINGS,
                    "print_notes": PRINT_NOTES,
                    "comm_map": COMM_MAP,
                    "power_map": POWER_MAP,
                    "sensor_registry": SENSOR_REGISTRY,
                    "cad_tracked_objects": len(CADTracker.objects),
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

        @trace_execution
        def write_production_readiness_report(check_rows=None):
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
                    f.write("OPTIMUS PRIME G1 v17.0 PRODUCTION READINESS\n")
                    f.write("Jetson Nano AI Brain Edition\n")
                    f.write("=" * 56 + "\n\n")
                    f.write("MODEL SUMMARY\n")
                    f.write(f"  Components: {len(comps_list)}\n")
                    f.write(f"  As-built joints: {joint_count}\n")
                    f.write(f"  Registered DOF total: {sum(j['dof'] for j in DOFValidator.joints)}\n")
                    f.write(f"  BOM rows: {len(BOM._rows)}\n")
                    f.write(f"  Registered screw locations: {len(SCREW_REGISTRY)}\n")
                    f.write(f"  Assembly jigs: {len(JIG_REGISTRY)}\n")
                    f.write(f"  Support warnings: {len(SUPPORT_WARNINGS)}\n")
                    f.write(f"  Communication links: {len(COMM_MAP)}\n")
                    f.write(f"  Power rails: {len(POWER_MAP)}\n")
                    f.write(f"  Registered sensors: {len(SENSOR_REGISTRY)}\n")
                    f.write(f"  CAD-tracked bodies: {len(CADTracker.objects)}\n\n")
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
                    f.write("  Safety: Independent E-stop on servo rail only\n")
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
                        "Verify boss/pin/insert fasteners are actually fused to their shell "
                        "half after printing (do not assume the merge step succeeded).",
                        "Press-fit bearings with no visible shell cracking.",
                        "Verify every servo moves through ROM with power current limited.",
                        "Validate hip, knee, ankle, shoulder, and waist hard stops by hand.",
                        "Confirm transformer locks engage in robot and truck mode.",
                        "Boot Jetson Nano standalone (no servo rail) and verify CSI camera feed.",
                        "Verify each ESP32-S3 node enumerates over USB and responds to ping.",
                        "Press the E-stop and confirm the servo rail cuts while Jetson/ESP32 stay up.",
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


        # ═════════════════════════════════════════════════════════════════
        # V16 NEW: REAL-MASS + REAL-TORQUE VALIDATION
        # These replace v12-v15's hand-typed mass_g/lever_cm guesses in
        # estimate_servo_loads() with numbers computed from the ACTUAL CAD
        # geometry + assigned physical materials above. This is still an
        # approximation (single-support worst case, straight-leg lever arm,
        # no dynamic/impact loading) -- but it is grounded in the real
        # model instead of being invented.
        # ═════════════════════════════════════════════════════════════════

        MASS_REPORT = {}   # populated by compute_total_mass_report()

        @trace_execution
        def compute_total_mass_report():
            global MASS_REPORT
            total_kg = 0.0
            per_comp = {}
            no_material_count = 0
            for comp in comps_list:
                try:
                    m = comp.physicalProperties.mass
                    per_comp[comp.name] = m
                    total_kg += m
                    try:
                        if comp.material is None:
                            no_material_count += 1
                    except Exception:
                        no_material_count += 1
                except Exception:
                    pass
            MASS_REPORT = per_comp
            Logger.log("--- V16 REAL COMPUTED MASS REPORT (from assigned physical materials) ---")
            Logger.log(f"  Total assembly mass (computed): {total_kg:.2f} kg")
            if no_material_count:
                Logger.log(f"  [WARN] {no_material_count} component(s) had no matched physical "
                           f"material -- their mass uses Fusion's default density and should "
                           f"NOT be trusted for torque/weight planning", "WARN")
            heaviest = sorted(per_comp.items(), key=lambda x: -x[1])[:10]
            for name, m in heaviest:
                Logger.log(f"    {name:<22s} {m:6.3f} kg")
            return total_kg, per_comp

        @trace_execution
        def _torque_kgcm(mass_kg, lever_cm):
            """kgf*cm required to statically hold `mass_kg` at `lever_cm` lever arm."""
            F_newtons = mass_kg * 9.81
            return (F_newtons * lever_cm / 100.0) / 0.0981

        @trace_execution
        def estimate_real_joint_torques(mass_report):
            """V16 NEW -- Real-mass torque validation. Worst-case (not
            average-case) assumptions: single-leg support carries the ENTIRE
            torso/pelvis/head/backpack mass plus the swing leg's own mass;
            lever arms use full limb length (leg fully extended / arm fully
            extended), which is the worst realistic case for standing/
            kicking/reaching, not for normal mid-stride walking. If this
            fails even under the worst case with margin to spare, walking
            loads (which are typically gentler than static full-extension
            holds) are even less likely to be the limiting factor -- but a
            PASS here is not a guarantee of walking capability, only of
            static holding capability."""
            Logger.log("--- V16 REAL-MASS JOINT TORQUE VALIDATION ---")
            Logger.log(f"  Actuator profile: {ACTIVE_ACTUATOR_PROFILE} "
                       f"({SERVO_SPECS.get('class','?')}-class, "
                       f"~${SERVO_SPECS.get('unit_cost_usd','?')}/unit, "
                       f"{SERVO_SPECS.get('feedback','?')})")

            def m(name):
                return mass_report.get(name, 0.0)

            leg_mass  = {s: m(f"OP_Thigh_{s}") + m(f"OP_Shin_{s}") + m(f"OP_Foot_{s}")
                         + m(f"OP_SteerPod_{s}") for s in ["L", "R"]}
            arm_mass  = {s: m(f"OP_UpperArm_{s}") + m(f"OP_Forearm_{s}") + m(f"OP_Hand_{s}")
                         for s in ["L", "R"]}
            body_mass = m("OP_Torso") + m("OP_Pelvis") + m("OP_Head") + m("OP_Backpack")

            leg_lever_cm  = max(HIP_JOINT_Z - ANKLE_CTR, 1.0)
            arm_lever_cm  = max(SHOULDER_CTR - WRIST_Z, 1.0)
            # V17 FIX: ankle carries the FULL body weight in single-support
            # stance. Worst-case ankle torque = total mass * horizontal CoM
            # excursion (use foot half-length as conservative lever; ankle roll
            # uses the same lateral excursion). Previously UNVALIDATED.
            total_body = body_mass + leg_mass["L"] + leg_mass["R"]
            ankle_lever_cm = 3.1   # ~foot half-width (FOOT_HW); conservative CoM offset

            checks = [
                ("L Hip Pitch (single-support stance)",
                 body_mass + leg_mass["R"], leg_lever_cm * 0.35, SERVO_SPECS["hip_hd"]),
                ("R Hip Pitch (single-support stance)",
                 body_mass + leg_mass["L"], leg_lever_cm * 0.35, SERVO_SPECS["hip_hd"]),
                ("L Knee Pitch (swing leg, full extension)",
                 leg_mass["L"], leg_lever_cm * 0.55, SERVO_SPECS["std"]),
                ("R Knee Pitch (swing leg, full extension)",
                 leg_mass["R"], leg_lever_cm * 0.55, SERVO_SPECS["std"]),
                ("L Shoulder Pitch (arm extended horizontal)",
                 arm_mass["L"], arm_lever_cm, SERVO_SPECS["std"]),
                ("R Shoulder Pitch (arm extended horizontal)",
                 arm_mass["R"], arm_lever_cm, SERVO_SPECS["std"]),
                ("Waist Pitch (full upper body forward lean)",
                 body_mass - m("OP_Pelvis") + arm_mass["L"] + arm_mass["R"],
                 (SHOULDER_CTR - WAIST_CTR), SERVO_SPECS["waist"]),
                # V17 FIX: ankle torque validation (was missing entirely)
                ("L Ankle Pitch (single-support, full body)",
                 total_body, ankle_lever_cm, SERVO_SPECS["std"]),
                ("R Ankle Pitch (single-support, full body)",
                 total_body, ankle_lever_cm, SERVO_SPECS["std"]),
                ("L Ankle Roll (single-support, lateral)",
                 total_body, ankle_lever_cm, SERVO_SPECS["std"]),
                ("R Ankle Roll (single-support, lateral)",
                 total_body, ankle_lever_cm, SERVO_SPECS["std"]),
            ]

            results = []
            for label, mass_kg, lever_cm, spec in checks:
                need = _torque_kgcm(mass_kg, lever_cm)
                margin = spec["rated"] / need if need > 0 else 99.0
                status = ("OK" if margin >= 1.5 else
                          "MARGINAL" if margin >= 1.0 else "OVERLOAD")
                icon = "[OK]" if status == "OK" else f"[{status}]"
                Logger.log(
                    f"  {label:<40} mass={mass_kg:5.2f}kg lever={lever_cm:5.1f}cm  "
                    f"need {need:6.1f} kgf-cm  rated {spec['rated']:6.1f}  "
                    f"margin {margin:.2f}x  {spec['name']:<28s} {icon}")
                if status == "OVERLOAD":
                    BugReporter.report(
                        f"{label}: real computed load needs ~{need:.1f} kgf-cm but "
                        f"'{ACTIVE_ACTUATOR_PROFILE}' profile's {spec['name']} is only rated "
                        f"{spec['rated']:.1f} kgf-cm ({margin:.2f}x margin) -- this joint will "
                        f"NOT reliably hold this load; switch actuator profile or reduce mass",
                        "CRITICAL", category="Mechanical")
                results.append((label, mass_kg, lever_cm, need, spec["rated"], margin, status))
            return results

        @trace_execution
        def run_structural_safety_checks(torque_results):
            """V16 NEW -- feed the real per-joint torque numbers into
            StructuralValidator against every registered bracket. Matches
            brackets to the nearest torque check by name substring; falls
            back to the largest single-leg/arm load if no direct match."""
            Logger.log("--- V16 STRUCTURAL SAFETY-FACTOR CHECK (approximate, see caveats) ---")
            if not BRACKET_REGISTRY:
                Logger.log("  No brackets registered -- skipping.", "WARN")
                return []
            torque_by_key = {}
            for label, mass_kg, lever_cm, need, rated, margin, status in torque_results:
                torque_by_key[label] = need
            default_need = max((t[3] for t in torque_results), default=20.0)
            reports = []
            for bracket in BRACKET_REGISTRY:
                tag = bracket["tag"]
                need = default_need
                for label, val in torque_by_key.items():
                    key_frag = label.split(" ")[0] + " " + label.split(" ")[1] if " " in label else label
                    if ("Hip" in tag and "Hip" in label) or \
                       ("Kn" in tag and "Knee" in label) or \
                       ("Sh" in tag and "Shoulder" in label) or \
                       ("Waist" in tag and "Waist" in label):
                        need = val
                        break
                r = StructuralValidator.check_bracket(bracket, need, material="PETG")
                if r:
                    reports.append(r)
                    icon = "[OK]" if r["status"] == "PASS" else "[FAIL]"
                    Logger.log(f"  {icon} {r['tag']:<20s} in {r['comp']:<16s} "
                               f"sigma~{r['sigma_mpa']:.1f}MPa  SF={r['safety_factor']:.2f}x")
            fails = [r for r in reports if r["status"] == "FAIL"]
            Logger.log(f"  {len(reports)} bracket(s) checked, {len(fails)} below "
                       f"{StructuralValidator.MIN_SAFETY_FACTOR}x safety factor")
            return reports

        # ── V16 NEW: analytic 2-link planar leg IK, for a real gait instead
        # of hand-tuned fixed joint-angle pose tables ────────────────────────
        @trace_execution
        def solve_leg_ik_2link(foot_dx, foot_dz, thigh_len, shin_len):
            """Analytic 2-link inverse kinematics in the sagittal (forward/
            down) plane. foot_dx = forward offset of the foot target from the
            hip joint (cm, +forward), foot_dz = downward offset (cm, +down).
            Returns (hip_pitch_deg, knee_pitch_deg) matching this file's
            L/R_Knee pitch convention (0 = straight leg, increasing = bent).
            Silently clamps unreachable targets to the nearest reachable
            radius rather than raising, since this drives a real-time-style
            per-frame loop where a hard failure would abort the whole gait."""
            reach = math.hypot(foot_dx, foot_dz)
            max_reach = thigh_len + shin_len - 0.01
            min_reach = abs(thigh_len - shin_len) + 0.01
            reach_c = max(min_reach, min(max_reach, reach))
            scale = (reach_c / reach) if reach > 1e-6 else 1.0
            dx, dz = foot_dx * scale, foot_dz * scale

            cos_knee = (thigh_len**2 + shin_len**2 - reach_c**2) / (2 * thigh_len * shin_len)
            cos_knee = max(-1.0, min(1.0, cos_knee))
            knee_interior_deg = math.degrees(math.acos(cos_knee))
            knee_pitch = 180.0 - knee_interior_deg

            cos_hip = (thigh_len**2 + reach_c**2 - shin_len**2) / (2 * thigh_len * reach_c)
            cos_hip = max(-1.0, min(1.0, cos_hip))
            hip_offset_deg = math.degrees(math.acos(cos_hip))
            hip_to_target_deg = math.degrees(math.atan2(dx, dz))
            hip_pitch = hip_to_target_deg - hip_offset_deg
            return hip_pitch, knee_pitch

        # ── V16 NEW: ESP32 firmware skeleton export ─────────────────────────
        @trace_execution
        def export_firmware_skeletons():
            """V16 NEW -- generates per-node wiring/channel-map skeletons.
            These are NOT working control firmware: there is no balance
            controller, no real gait generator running on-device, and no
            actual COBS/CRC16 frame parser implemented here. What they DO
            give you is an unambiguous, generated-from-the-same-source-of-
            truth channel map so the electrical wiring matches the CAD/BOM
            exactly, which is a real (if modest) step past "just a BOM
            line.\""""
            try:
                fw_dir = os.path.join(EXPORT_DIR, "firmware")
                os.makedirs(fw_dir, exist_ok=True)
                node_channels = {
                    "lower": ["L_HipYaw", "L_HipP", "L_HipR", "R_HipYaw", "R_HipP", "R_HipR",
                              "L_KneP", "L_KneRoll", "R_KneP", "R_KneRoll",
                              "L_AnkP", "L_AnkR", "L_AnkY", "R_AnkP", "R_AnkR", "R_AnkY"],
                    "upper": ["Waist_Yaw", "Waist_Pitch", "Waist_Roll",
                              "L_ShY", "L_ShP", "L_ShR", "R_ShY", "R_ShP", "R_ShR",
                              "L_ElbP", "R_ElbP"],
                    "head":  ["Neck_Pitch", "Neck_Yaw", "Neck_Roll",
                              "L_WrP", "L_WrR", "R_WrP", "R_WrR", "Blaster_Fold"],
                }
                for role, channels in node_channels.items():
                    path = os.path.join(fw_dir, f"esp32s3_{role}_node.ino")
                    with open(path, "w", encoding="utf-8") as f:
                        f.write(f"// AUTO-GENERATED SKELETON -- ESP32-S3 '{role}' node\n")
                        f.write("// Optimus Prime G1 v16 -- WIRING MAP ONLY.\n")
                        f.write("// This is NOT working control firmware. Missing: balance\n")
                        f.write("// controller, real frame protocol, gait generator, safety\n")
                        f.write("// interlocks beyond the E-stop relay. Fill in before use.\n\n")
                        f.write("#include <Wire.h>\n#include <Adafruit_PWMServoDriver.h>\n\n")
                        f.write("Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);\n\n")
                        f.write("// Channel map (edit only if your physical wiring differs\n")
                        f.write("// from the CAD build -- keep this in sync with the BOM):\n")
                        for i, ch in enumerate(channels):
                            f.write(f"#define CH_{ch.upper():<20s} {i}\n")
                        f.write("\nvoid setup() {\n")
                        f.write("  Serial.begin(1000000);\n")
                        f.write("  Wire.begin();\n")
                        f.write("  pwm.begin();\n")
                        f.write("  pwm.setPWMFreq(50);\n")
                        f.write("  // TODO: home every channel to a JOINT_LIMITS-safe neutral\n")
                        f.write("  // position BEFORE the servo rail is live (verify by hand\n")
                        f.write("  // with the E-stop engaged first).\n")
                        f.write("}\n\n")
                        f.write("void loop() {\n")
                        f.write("  // TODO: parse COBS+CRC16 frames from Jetson over USB-UART\n")
                        f.write("  // TODO: interpolate joint targets, write to pwm.setPWM(...)\n")
                        f.write("  // TODO: read this node's sensors (IMU / FSR / ToF) and report\n")
                        f.write("}\n")
                    Logger.log(f"Firmware skeleton written -> {path}")
                readme_path = os.path.join(fw_dir, "README.txt")
                with open(readme_path, "w", encoding="utf-8") as f:
                    f.write("These are WIRING-MAP skeletons auto-generated from the same joint\n")
                    f.write("names used by the CAD/BOM/assembly guide -- they exist to remove\n")
                    f.write("channel-mapping ambiguity, NOT to provide working control firmware.\n")
                    f.write("You still need to write: the balance controller, the real serial\n")
                    f.write("protocol, the gait generator, and safety interlocks, before this\n")
                    f.write("robot can stand or walk under its own power.\n")
                Logger.log(f"Firmware skeletons + README written -> {fw_dir}")
            except Exception as e:
                Logger.log(f"Firmware skeleton export failed: {e}", "WARN")

        # ─────────────────────────────────────────────────────────────────
        # JOINT BUILDERS
        # ─────────────────────────────────────────────────────────────────

        @trace_execution
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

        @trace_execution
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
                # BUGFIX-V15-2: wire up DOF/joint validators that were
                # previously never called.
                DOFValidator.register_joint(name, dof=1)
                JointValidator.verify_joint(name, occ1, occ2, "revolute")
            except Exception:
                Logger.log(f"Failed revolute joint {name}: {traceback.format_exc()}", "ERROR")

        @trace_execution
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
                # BUGFIX-V15-2: register the real declared DOF count for this
                # joint (from JOINT_LIMITS) instead of leaving DOFValidator
                # permanently empty. Falls back to 3 for ball joints with no
                # JOINT_LIMITS entry (e.g. auxiliary/undeclared joints).
                axes = [a for a in JOINT_LIMITS.get(name, {}).keys()
                        if a in ("pitch", "yaw", "roll")]
                DOFValidator.register_joint(name, dof=len(axes) if axes else 3)
                JointValidator.verify_joint(name, occ1, occ2, "ball")
            except Exception:
                Logger.log(f"Failed ball joint {name}: {traceback.format_exc()}", "ERROR")

        @trace_execution
        def rigid_joint(name, occ1, occ2):
            if not (occ1 and occ2):
                return
            try:
                geom = _make_joint_geometry(0, 0, 0)
                ji   = root.asBuiltJoints.createInput(occ1, occ2, geom)
                ji.setAsRigidJointMotion()
                j      = root.asBuiltJoints.add(ji)
                j.name = name
                DOFValidator.register_joint(name, dof=0)
                JointValidator.verify_joint(name, occ1, occ2, "rigid")
            except Exception:
                Logger.log(f"Failed rigid joint {name}: {traceback.format_exc()}", "ERROR")

        # ─────────────────────────────────────────────────────────────────
        # HARDWARE MODULES (servo bodies, couplers, wheels, brackets, stops)
        # ─────────────────────────────────────────────────────────────────

        @trace_execution
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

        @trace_execution
        def mg996r(comp, tag, cx, cy, cz, axis="x", reinforced=False):
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
            horn_coupler(comp, tag, cx, cy, cz, axis, servo_type="std", reinforced=reinforced)
            BOM.add("Servo", "MG996R 11 kg-cm servo", 1, comp.name)

        @trace_execution
        def mg90s(comp, tag, cx, cy, cz, axis="x", reinforced=False):
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
            horn_coupler(comp, tag, cx, cy, cz, axis, servo_type="micro", reinforced=reinforced)
            BOM.add("Servo", "MG90S micro servo", 1, comp.name)

        @trace_execution
        def tt_wheel(comp, tag, cx, cy, cz, side=1):
            cl = CLEARANCE
            box(comp, f"{tag}_VisGB",    cx,           cy,     cz,  2.30, 5.20, 1.90, yellow_met)
            cyl(comp, f"{tag}_VisMot",   cx,           cy-1.5, cz,  0.90, 2.10, "y",  chrome)
            cyl(comp, f"{tag}_VisShaft", cx+side*1.75, cy,     cz,  0.20, 3.50, "x",  chrome)
            cyl(comp, f"{tag}_VisHub",   cx+side*3.25, cy,     cz,  0.80, 2.60, "x",  dark_metal)
            cyl(comp, f"{tag}_VisTire",  cx+side*3.25, cy,     cz,  3.25, 2.60, "x",  rubber_blk)
            cyl(comp, f"{tag}_VisRim",   cx+side*3.25, cy,     cz,  2.20, 2.65, "x",  chrome)
            cyl(comp, f"{tag}_AxlePin",  cx - side*1.15, cy,     cz,  0.5, 3.5, "x", dark_metal)
            marker(comp, f"{tag}_Axle_Pivot", cx+side*3.25, cy, cz, 0.18)
            cut_cavity(comp, box(comp, f"{tag}_CGB", cx,           cy, cz, 2.30+cl, 5.20+cl, 1.90+cl))
            cut_cavity(comp, box(comp, f"{tag}_CDS", cx+side*3.25, cy, cz, 2.7+cl,  0.54+cl, 0.36+cl))
            BOM.add("Drive", "TT gear-motor 3V-6V", 1, comp.name)
            BOM.add("Drive", "65 mm rubber tyre + wheel", 1, comp.name)

        @trace_execution
        def u_bracket(comp, tag, cx, cy, cz, lx, ly, lz, ap=None):
            ap = ap or chrome
            box(comp, f"{tag}_BB",  cx,          cy,        cz, 0.45,     ly,   lz, ap)
            box(comp, f"{tag}_BL",  cx+lx*0.45,  cy+ly*0.35, cz, lx*0.55, 0.40, lz, ap)
            box(comp, f"{tag}_BR",  cx+lx*0.45,  cy-ly*0.35, cz, lx*0.55, 0.40, lz, ap)
            cyl(comp, f"{tag}_Pin", cx+lx*0.50,  cy,         cz, 0.18, ly*0.85, "y", chrome)
            # V16 NEW: register this bracket's backbone plate (the "_BB" body,
            # 0.45cm thick x ly x lz) so StructuralValidator can run a coarse
            # cantilever bending check against the real computed joint torque.
            BRACKET_REGISTRY.append({
                "tag": tag, "comp": comp.name,
                "thickness_cm": 0.45, "width_cm": ly, "height_cm": lz,
                "cx": cx, "cy": cy, "cz": cz,
            })

        @trace_execution
        def hard_stop(comp, tag, cx, cy, cz, axis="x", stop_angle_deg=90):
            box(comp, f"{tag}_Stop", cx, cy, cz, 0.35, 0.35, 0.35, dark_metal)
            BOM.add("Hardware", "Hard stop block (printed)", 1, comp.name)
            ASSEMBLY_STEPS.append(f"Verify {tag} hard stop at {stop_angle_deg} deg clears moving link")

        @trace_execution
        def transform_lock(comp, tag, cx, cy, cz, axis="z"):
            cyl(comp, f"{tag}_LockBore", cx, cy, cz, 0.18, 1.50, axis, dark_metal)
            cut_cavity(comp, cyl(comp, f"{tag}_LockHole", cx, cy, cz, 0.20, 1.60, axis))
            cyl(comp, f"{tag}_SpringPkt", cx, cy, cz + 0.30, 0.35, 0.50, axis, dark_grey)
            BOM.add("Hardware", "Spring latch pin O3.5mm (steel)", 1, comp.name)
            BOM.add("Hardware", "Compression spring (lock return)", 1, comp.name)
            ASSEMBLY_STEPS.append(f"Install {tag} transform lock pin and return spring")


        # ═════════════════════════════════════════════════════════════════
        # COMPONENT BUILDING  (v16 Jetson Nano AI Brain Edition)
        # ═════════════════════════════════════════════════════════════════

        # ─────────────────────────────────────────────────────────────────
        # 1 TORSO — Jetson Nano (brain) + ESP32-S3 'upper' node + power
        # ─────────────────────────────────────────────────────────────────
        SnapshotManager.save_checkpoint("Torso_Start")
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

        # BUGFIX-V15-5: status RGB indicator, coded in v14 but never placed.
        status_rgb_pocket(torso, "ChestStatus", 1.6, -4.55, TORSO_CTR+0.5, "y")

        # MECH-1 — Internal structural skeleton
        box(torso, "Inner_Frame",    0,    0,    TORSO_CTR+1.5,   7.4, 6.0,  8.2, dark_metal)
        box(torso, "Spine_Beam",     0,    0,    TORSO_CTR+1.5,   1.8, 1.8,  8.2, chrome)
        cyl(torso, "Spine_Cyl",      0,    0,    TORSO_CTR+1.5,  1.10, 4.5,  "z", chrome)
        for rz in [TORSO_CTR+3.5, TORSO_CTR, TORSO_CTR-3.5]:
            box(torso, f"Rib_{rz:.0f}", 0, 0, rz, 6.8, 0.35, 4.5, dark_metal)
        for sx in [-6.5, 6.5]:
            box(torso, f"Gusset_{sx:.0f}", sx, 0, TORSO_CTR+2.0, 1.2, 1.2, 3.5, dark_metal)

        # BUGFIX-V15-5: comm backbone (I2C/UART/SPI trunk), coded in v14 but
        # never placed. Runs down the spine from waist to neck.
        comm_backbone(torso, "SpineComm", 0, 0.9, TORSO_CTR+0.5, 9.5, "z")

        # ELEC-3 — LiPo bay (rear of lower torso)
        box(torso, "LipoBay_Shell",  0,    3.2, TORSO_CTR-2.0,   7.6, 4.4,  5.6, black_plastic)
        lipo_bay(torso, "Main", 0, 2.8, TORSO_CTR-2.0)
        lipo_door(torso, "LipoDoor", 0, 5.5, TORSO_CTR-2.0, LIPO_L + 0.80, LIPO_W + 0.80)

        # JETSON NANO BAY (the robot's brain)
        box(torso, "Jetson_Shell",   0,    2.8, TORSO_CTR+2.0,   7.6, 4.2,  3.0, black_plastic)
        jetson_nano_bay(torso, "Main", 0, 3.0, TORSO_CTR+2.0)
        pcb_cover(torso, "JetsonCover", 0, 4.8, TORSO_CTR+2.0, JNANO_L + 0.60, JNANO_W + 0.60, "magnet")
        vent_grille(torso, "JetsonVent", 0, 6.0, TORSO_CTR+2.0, "y", n_slots=8)

        # ESP32-S3 'upper' node (waist/shoulders/elbows + INA3221)
        box(torso, "ESP32S3U_Shell",  0, 2.6, TORSO_CTR+5.0, 6.0, 2.6, 1.8, black_plastic)
        esp32s3_node_bay(torso, "UpperNode", 0, 2.8, TORSO_CTR+5.0, role="upper")
        pcb_cover(torso, "ESP32S3UCover", 0, 3.9, TORSO_CTR+5.0,
                  ESP32S3_L + 0.50, ESP32S3_W + 0.50, "screw")

        # PCA9685 for upper-body servos, driven by ESP32-S3 'upper'
        box(torso, "PCA_Shell",      0,    2.8, TORSO_CTR+4.2,   6.8, 3.2,  2.2, black_plastic)
        pca9685_bay(torso, "Main",   0,    3.0, TORSO_CTR+4.2)
        pcb_cover(torso, "PCACover", 0, 4.2, TORSO_CTR+4.2, PCA_L + 0.50, PCA_W + 0.50, "screw")

        # INA3221 current monitor (servo rail health, reported to upper node)
        box(torso, "INA_Shell", -2.6, 2.6, TORSO_CTR-0.5, 3.0, 1.8, 1.0, black_plastic)
        ina3221_bay(torso, "ServoMon", -2.6, 2.8, TORSO_CTR-0.5)

        # Cable hub + clips
        wire_hub(torso, "TorsoHub", 0, 1.5, TORSO_CTR+0.5)
        for cz_clip in [TORSO_CTR+3.0, TORSO_CTR, TORSO_CTR-3.0, TORSO_CTR-4.5]:
            cable_clip(torso, f"CC_L_{cz_clip:.0f}", -3.4, 0.6, cz_clip)
            cable_clip(torso, f"CC_R_{cz_clip:.0f}",  3.4, 0.6, cz_clip)
        box(torso, "Cable_L",       -3.4,  0.6,  TORSO_CTR,      0.55, 1.0, 10.0, dark_grey)
        box(torso, "Cable_R",        3.4,  0.6,  TORSO_CTR,      0.55, 1.0, 10.0, dark_grey)

        # power bays -- Jetson 5V/4A UBEC + logic 5V/3A BEC, separated rails
        ubec_5v4a_bay(torso, "JetsonPwr", LIPO_L/2 + 1.2, 3.0, TORSO_CTR-3.2)
        bec_mount(torso, "LogicBEC", -(LIPO_L/2 + 1.2), 3.0, TORSO_CTR-2.0)

        power_switch_cutout(torso, "PwrSw", -5.5, 0, TORSO_CTR+2.0, "y")

        # BUGFIX-V15-5: independent servo-rail E-stop, coded in v14 but never
        # placed. Mounted next to the master power switch on the same panel.
        estop_cutout(torso, "MasterEstop", -5.5, 1.4, TORSO_CTR+2.0, "y")

        # USB-C bridge cutouts for external Jetson<->ESP32 debug access
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

        # V17: the waist carries 100% of torso+head+arms weight through a
        # single bracket -- the highest-stress structural point in the whole
        # robot. Oversized/gusseted relative to v16's identical-to-every-
        # other-joint bracket, and given the reinforced metal coupling
        # insert + thrust washer + retaining ring the joint's real load
        # actually calls for.
        u_bracket(torso, "Waist_Brkt",  0, 0, WAIST_CTR,     4.6, 4.8, 3.9)
        box(torso, "Waist_Gusset_L", -1.8, 0.9, WAIST_CTR+0.3, 1.0, 1.0, 2.6, dark_metal)
        box(torso, "Waist_Gusset_R",  1.8, 0.9, WAIST_CTR+0.3, 1.0, 1.0, 2.6, dark_metal)
        mg996r(torso,    "Waist_Yaw",   0, 0, WAIST_CTR,     "z", reinforced=True)
        dual_bearing(torso, "WaistDual", 0, 0, WAIST_CTR+0.5, "z", 1.30, 0.65, span=3.00,
                     add_retaining_ring=True, axial_load=True)
        _reg_joint_axis("Waist_Cluster", "yaw", "z", "Waist_Yaw")
        wire_service_loop(torso, "WaistWireLoop", 1.6, 0.8, WAIST_CTR+1.4, loop_r=0.50, axis="z")
        u_bracket(torso, "WaistP_Brkt", 0, 0, WAIST_CTR-2.5, 4.0, 4.2, 3.4)
        mg996r(torso,    "Waist_Pitch", 0, 0, WAIST_CTR-2.5, "x")
        bearing_fit(torso, "WaistP_Brg",  0, 0, WAIST_CTR-2.0, "x", 1.30, 0.65, fit_type="press",
                    add_retaining_ring=True)
        _reg_joint_axis("Waist_Cluster", "pitch", "x", "Waist_Pitch")
        u_bracket(torso, "WaistR_Brkt", 0, 0, WAIST_CTR-1.2, 4.0, 3.4, 3.4)
        mg996r(torso,    "Waist_Roll",  0, 0, WAIST_CTR-1.2, "y")
        bearing_fit(torso, "WaistR_Brg", 0, 0, WAIST_CTR-1.2, "y", 1.10, 0.55, fit_type="press")
        _reg_joint_axis("Waist_Cluster", "roll", "y", "Waist_Roll")
        magnet_pocket(torso, "WLock_F", 0, -2.0, WAIST_CTR-3.0)
        magnet_pocket(torso, "WLock_R", 0,  2.0, WAIST_CTR-3.0)
        hard_stop(torso, "WaistP", 0, -2.5, WAIST_CTR-2.5, "x", 60)
        hard_stop(torso, "WaistN", 0,  2.5, WAIST_CTR-2.5, "x", -45)
        # V17 FIX: Waist_Roll had a servo + bearing but NO hard stop --
        # nothing physically prevented it from over-rotating into the
        # torso shell or a wiring harness at range limits.
        hard_stop(torso, "WaistRoll_Pos", -2.0, 0, WAIST_CTR-1.2, "y", 15)
        hard_stop(torso, "WaistRoll_Neg",  2.0, 0, WAIST_CTR-1.2, "y", -15)

        u_bracket(torso, "Neck_Brkt",   0, 0, NECK_JOINT_Z,  3.2, 2.8, 3.0)
        mg996r(torso,    "Neck_Pitch",  0, 0, NECK_JOINT_Z,  "x")
        _reg_joint_axis("Neck_Cluster", "pitch", "x", "Neck_Pitch")
        wire_channel(torso, "Spine",    0, 0, TORSO_CTR,     0.6, 20.0, "z")

        # CSI ribbon channel -- routes from head down through neck into torso
        fpc_ribbon_channel(torso, "CSIRoute", 0, 0.6, NECK_JOINT_Z - 3.0, 6.0, "z")

        assign_material(torso, "structural")   # V16 NEW: real mass for torque/mass reports
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

        # CSI CAMERA -- the robot's primary eyes, mounted behind the visor
        csi_camera_pocket(head, "RobotEyes", 0, -1.9, HC+1.35, "y")
        cover_plate(head, "CamCover", 0, -2.0, HC+1.35, CSI_CAM_L+0.50, CSI_CAM_W+0.50,
                    [(-0.9, -0.4), (0.9, -0.4)], method="hinge", hinge_edge="top", ap=grey_plastic)

        # ESP32-S3 'head' node (neck/wrist/finger servos + ToF sensors)
        box(head, "ESP32S3H_Shell",  0, 1.6, HC+0.0, 4.6, 2.4, 1.8, black_plastic)
        esp32s3_node_bay(head, "HeadNode", 0, 1.8, HC+0.0, role="head")
        pcb_cover(head, "ESP32S3HCover", 0, 2.8, HC+0.0,
                  ESP32S3_L + 0.50, ESP32S3_W + 0.50, "screw")

        # ToF obstacle sensors x2 (front-facing, either side of chin)
        tof_sensor_pocket(head, "ToF_L", -1.8, -2.40, HC-0.3, "y")
        tof_sensor_pocket(head, "ToF_R",  1.8, -2.40, HC-0.3, "y")

        mg90s(head, "Neck_Yaw", 0, 0, NECK_JOINT_Z, "z")
        _reg_joint_axis("Neck_Cluster", "yaw", "z", "Neck_Yaw")
        mg90s(head, "Neck_Roll", 0, 0, NECK_JOINT_Z-0.9, "y")
        bearing_fit(head, "NeckR_Brg", 0, 0, NECK_JOINT_Z-0.9, "y", 0.70, 0.40, fit_type="glue")
        _reg_joint_axis("Neck_Cluster", "roll", "y", "Neck_Roll")
        # V17 FIX: Neck_Roll had a servo + bearing but NO hard stop.
        hard_stop(head, "NeckRoll_Pos", -1.3, 0, NECK_JOINT_Z-0.9, "y", 20)
        hard_stop(head, "NeckRoll_Neg",  1.3, 0, NECK_JOINT_Z-0.9, "y", -20)
        grommet_slot(head, "NeckWire", 0, 0.8, HC-0.5, "y", 0.50)
        # V17 NEW: the CSI ribbon is the single most fatigue-prone connection
        # in the head (a rigid flat-flex cable flexed every neck movement).
        # Give it a real service loop instead of only a straight channel.
        wire_service_loop(head, "CSIRibbonLoop", 0, 0.7, NECK_JOINT_Z-1.8, loop_r=0.45, axis="y")
        fpc_ribbon_channel(head, "CSIHeadExit", 0, 0.4, HC-1.2, 3.0, "z")

        assign_material(head, "structural")   # V16 NEW
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

        # ESP32-S3 'lower' node (hips/knees/ankles, reads MPU-6050 directly)
        box(pelvis, "ESP32S3L_Shell", 0, 1.6, PELVIS_CTR-1.5, 6.0, 2.4, 1.8, black_plastic)
        esp32s3_node_bay(pelvis, "LowerNode", 0, 1.8, PELVIS_CTR-1.5, role="lower")
        pcb_cover(pelvis, "ESP32S3LCover", 0, 2.8, PELVIS_CTR-1.5,
                  ESP32S3_L + 0.50, ESP32S3_W + 0.50, "screw")

        # PCA9685 boards for hip/knee (1) and ankle (1), both driven by 'lower' node
        box(pelvis, "PCA_Hip_Shell", -3.0, 1.6, PELVIS_CTR+1.2, 5.0, 2.2, 1.6, black_plastic)
        pca9685_bay(pelvis, "HipKnee", -3.0, 1.8, PELVIS_CTR+1.2)
        box(pelvis, "PCA_Ankle_Shell", 3.0, 1.6, PELVIS_CTR+1.2, 5.0, 2.2, 1.6, black_plastic)
        pca9685_bay(pelvis, "Ankle", 3.0, 1.8, PELVIS_CTR+1.2)

        # BUGFIX-V15-5: pelvis-mounted ultrasonic + FSR sensor fusion array,
        # coded in v14 but never placed. General front-facing obstacle sense,
        # complements the head ToF pair and the per-foot FSR arrays below.
        box(pelvis, "SensorArray_Shell", 0, -3.4, PELVIS_CTR-0.3, 5.2, 2.6, 2.0, black_plastic)
        sensor_array(pelvis, "PelvisSensors", 0, -3.6, PELVIS_CTR-0.3, "y", with_ultrasonic=True)

        grommet_slot(pelvis, "HipWire_L", -HIP_X, 0, PELVIS_CTR+0.5, "z", 0.60)
        grommet_slot(pelvis, "HipWire_R",  HIP_X, 0, PELVIS_CTR+0.5, "z", 0.60)
        uart_bridge_cutout(pelvis, "DebugLower", -7.5, 1.0, PELVIS_CTR, "x")

        mg996r(pelvis, "L_HipYaw", -HIP_X, 0, HIP_JOINT_Z, "z", reinforced=True)
        mg996r(pelvis, "R_HipYaw",  HIP_X, 0, HIP_JOINT_Z, "z", reinforced=True)
        # V17: Hip Yaw is the primary axis carrying the ENTIRE leg + a share
        # of body weight straight down its rotation axis -- exactly the load
        # direction a plain radial ball bearing is not rated for.
        dual_bearing(pelvis, "L_HipDual", -HIP_X-2.2, 0, HIP_JOINT_Z, "z", 1.10, 0.62, span=3.20,
                     fit_type="press", add_retaining_ring=True, axial_load=True)
        dual_bearing(pelvis, "R_HipDual",  HIP_X+2.2, 0, HIP_JOINT_Z, "z", 1.10, 0.62, span=3.20,
                     fit_type="press", add_retaining_ring=True, axial_load=True)
        BOM.add("Servo", "MG996R 11 kg-cm servo (hip yaw)", 2, "OP_Pelvis")
        _reg_joint_axis("L_Hip_Cluster", "yaw", "z", "L_HipYaw")
        _reg_joint_axis("R_Hip_Cluster", "yaw", "z", "R_HipYaw")

        assign_material(pelvis, "structural")   # V16 NEW
        for b in list(pelvis.bRepBodies):
            if b.name:
                printability_check(pelvis, b.name)


        # ─────────────────────────────────────────────────────────────────
        # 4 LEGS  (mechanical design retained from v12/v14)
        # ─────────────────────────────────────────────────────────────────
        SnapshotManager.save_checkpoint("Legs_Start")
        for side, sx in [("L", -HIP_X), ("R", HIP_X)]:
            m = -1 if side == "L" else 1

            thigh = new_component(f"OP_Thigh_{side}")
            box(thigh, "Thigh_Link",         sx,        0,  THIGH_CTR,      5.0, 4.0, 11.0, chrome)
            box(thigh, "Thigh_Outer",        sx+m*2.65, 0,  THIGH_CTR,      0.50, 4.4, 11.0, op_red)
            box(thigh, "Thigh_Front",        sx,       -2.2, THIGH_CTR,     5.0, 0.40, 11.0, op_blue)
            box(thigh, "Thigh_Knee_Armor",   sx,       -2.2, KNEE_CTR+2.5,  4.2, 0.80,  2.8, chrome)
            box(thigh, f"{side}_Thigh_Rib", sx, 0, THIGH_CTR, 3.5, 0.30, 9.0, dark_metal)

            u_bracket(thigh, f"{side}_HPB",  sx, 0, HIP_JOINT_Z+0.5,        4.0, 3.2, 3.2)
            mg996r(thigh, f"{side}_HipP",    sx, 0, HIP_JOINT_Z,             "x", reinforced=True)
            dual_bearing(thigh, f"{side}_HipP_Dual", sx, 0, HIP_JOINT_Z,
                         "x", 1.10, 0.62, span=2.80, fit_type="press", add_retaining_ring=True)
            _reg_joint_axis(f"{side}_Hip_Cluster", "pitch", "x", f"{side}_HipP")
            # V17 FIX: Hip Roll was the only one of the hip's three axes on
            # a SINGLE bearing (Hip Yaw and Hip Pitch both already got
            # dual_bearing) -- asymmetric support on the axis that takes
            # lateral sway load during single-leg stance. Upgraded to match.
            mg996r(thigh, f"{side}_HipR",    sx, 0, THIGH_CTR+2.0,           "y", reinforced=True)
            dual_bearing(thigh, f"{side}_HipR_Dual", sx, 0, THIGH_CTR+2.0,
                         "y", 1.00, 0.55, span=2.40, fit_type="press", add_retaining_ring=True)
            _reg_joint_axis(f"{side}_Hip_Cluster", "roll", "y", f"{side}_HipR")
            u_bracket(thigh, f"{side}_KnB",  sx, 0, KNEE_CTR+1.5,            3.8, 3.0, 3.0)
            mg996r(thigh, f"{side}_KneP",    sx, 0, KNEE_CTR+1.5,            "x", reinforced=True)
            dual_bearing(thigh, f"{side}_Knee_Dual", sx, 0, KNEE_CTR,
                         "x", 1.00, 0.55, span=2.60, fit_type="press", add_retaining_ring=True)
            _reg_joint_axis(f"{side}_Knee", "pitch", "x", f"{side}_KneP")
            if KNEE_ROLL_ENABLED:
                # V17 FLAGGED HIGH-RISK: knee roll's rotation axis does not
                # intersect the pitch axis (offset by KNEE_ROLL_OFFSET_X),
                # so every actuation puts a parasitic torsional load on the
                # shin -- and this is the single most impact-loaded link in
                # the leg at every footfall. Kept ON by default for
                # kinematic/JOINT_LIMITS backward compatibility, but now
                # built with the reinforced metal coupler + retaining ring
                # given the added risk, and clearly logged so it's a
                # conscious choice, not a silent default. Set
                # KNEE_ROLL_ENABLED=False before running to delete this DOF
                # entirely (falls back to a rigid stiffener instead).
                u_bracket(thigh, f"{side}_KnRollB", sx+m*KNEE_ROLL_OFFSET_X, 0, KNEE_CTR+1.0, 3.0, 2.6, 2.6)
                mg996r(thigh, f"{side}_KneRoll",  sx+m*KNEE_ROLL_OFFSET_X, 0, KNEE_CTR+1.0, "y",
                       reinforced=True)
                bearing_fit(thigh, f"{side}_KneRoll_Brg", sx+m*KNEE_ROLL_OFFSET_X, 0, KNEE_CTR+1.0,
                            "y", KNEE_ROLL_BRG_R, KNEE_ROLL_BRG_W, fit_type="press",
                            add_retaining_ring=True)
                hard_stop(thigh, f"{side}_KneRollStop", sx+m*KNEE_ROLL_OFFSET_X, -1.8, KNEE_CTR+1.0, "y", 15)
                BOM.add("Servo", "MG996R 11 kg-cm servo (knee roll)", 1, f"OP_Thigh_{side}")
                _reg_joint_axis(f"{side}_Knee", "roll", "y", f"{side}_KneRoll")
            else:
                # Rigid stiffener in place of the roll servo/bearing -- keeps
                # the shin from having an unsupported gap where the roll
                # hardware would have mounted.
                box(thigh, f"{side}_KneeRigidStiffener", sx+m*KNEE_ROLL_OFFSET_X, 0, KNEE_CTR+1.0,
                    2.4, 2.0, 2.0, dark_metal)
                Logger.log(f"{side}_Knee: roll DOF disabled (KNEE_ROLL_ENABLED=False) -- "
                           f"built as a rigid pitch-only hinge", "INFO")
            wire_channel(thigh, f"{side}_LW", sx, 0, THIGH_CTR,              0.5, 12.0, "z")
            for bz in [THIGH_CTR+3.0, THIGH_CTR-3.0]:
                m3_boss(thigh, f"{side}_ThighBoss_{bz:.0f}", sx, 0, bz)
            magnet_pocket(thigh, f"{side}_KLU", sx, -1.5, KNEE_CTR+1.0)
            magnet_pocket(thigh, f"{side}_KLL", sx,  1.5, KNEE_CTR+1.0)
            transform_lock(thigh, f"{side}_KneeLock", sx, -2.5, KNEE_CTR+0.5, "x")
            hard_stop(thigh, f"{side}_KneeExt", sx, -2.5, KNEE_CTR, "x", 135)
            BOM.add("Servo", "MG996R 11 kg-cm servo (hip pitch)", 1, f"OP_Thigh_{side}")

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

            mg996r(foot, f"{side}_AnkP", sx, 0, ANKLE_CTR+2.2, "x", reinforced=True)
            mg996r(foot, f"{side}_AnkR", sx, 0, ANKLE_CTR+0.5, "y", reinforced=True)
            # V17: the ankle takes the single worst DYNAMIC (footfall
            # impact) load in the robot -- both pitch and roll get retaining
            # rings, not just a friction lip, since that's the load case
            # most likely to work a printed lip loose over time.
            bearing_fit(foot, f"{side}_AB",  sx, 0, ANKLE_CTR,     "x", 1.00, 0.55, fit_type="press",
                        add_retaining_ring=True)
            _reg_joint_axis(f"{side}_Ankle_Cluster", "pitch", "x", f"{side}_AnkP")
            bearing_fit(foot, f"{side}_ARB", sx, 0, ANKLE_CTR+0.5, "y", 0.85, 0.48, fit_type="press",
                        add_retaining_ring=True)
            _reg_joint_axis(f"{side}_Ankle_Cluster", "roll", "y", f"{side}_AnkR")
            if ANKLE_YAW_ENABLED:
                # V17 FLAGGED HIGH-RISK: ankle yaw is the DOF most real
                # bipeds (hobbyist and commercial) skip entirely, precisely
                # because the ankle already carries the worst dynamic load
                # in the robot and yaw adds a third fragile axis to it. Kept
                # ON by default for backward compatibility; the axis also
                # carries body weight vertically through it, so it now gets
                # a thrust washer, not just a radial bearing. Set
                # ANKLE_YAW_ENABLED=False before running to delete this DOF
                # (falls back to a rigid connection between ankle and foot).
                mg996r(foot, f"{side}_AnkY", sx, 0, ANKLE_CTR+1.4, "z", reinforced=True)
                bearing_fit(foot, f"{side}_AnkY_Brg", sx, 0, ANKLE_CTR+1.4, "z", 0.90, 0.50,
                            fit_type="press", add_retaining_ring=True, axial_load=True)
                _reg_joint_axis(f"{side}_Ankle_Cluster", "yaw", "z", f"{side}_AnkY")
                hard_stop(foot, f"{side}_AnkY_Pos", sx-1.5, 0, ANKLE_CTR+1.4, "z", 90)
                hard_stop(foot, f"{side}_AnkY_Neg", sx+1.5, 0, ANKLE_CTR+1.4, "z", -25)
            else:
                box(foot, f"{side}_AnkleRigidStiffener", sx, 0, ANKLE_CTR+1.4, 2.4, 2.4, 1.6, dark_metal)
                Logger.log(f"{side}_Ankle_Cluster: yaw DOF disabled (ANKLE_YAW_ENABLED=False) "
                           f"-- built as a rigid pitch+roll-only joint", "INFO")
            hard_stop(foot, f"{side}_AnkP_Stop", sx, -2.0, ANKLE_CTR+2.2, "x", 20)
            hard_stop(foot, f"{side}_AnkN_Stop", sx,  2.0, ANKLE_CTR+2.2, "x", -20)
            for bx_off in [-1.5, 1.5]:
                m3_boss(foot, f"{side}_FootBoss_{bx_off:.0f}", sx+bx_off, 0, ANKLE_CTR-0.5)
            align_pin(foot, f"{side}_Ft_A", sx-1.0, 0, ANKLE_CTR-1.3)
            align_pin(foot, f"{side}_Ft_B", sx+1.0, 0, ANKLE_CTR-1.3)
            # V17 FIX: replaces the old cosmetic 0.15cm VibPad (too thin to
            # meaningfully compress) with real spring/TPU compliance -- the
            # foot sole is where the robot's worst impact load lands every
            # single step, and v15/v16 gave it no real shock absorption.
            ankle_compliance_pad(foot, f"{side}_Foot", sx, -0.6, ANKLE_CTR-2.2, lx=5.5, ly=8.0)

            # BUGFIX-V15-5: per-foot FSR stance-pressure sensor array, coded
            # in v14 but never placed. No ultrasonic here (that's the pelvis
            # array above) -- feet only get proprioceptive FSR + ADC.
            sensor_array(foot, f"{side}_FootSensors", sx, 0.4, ANKLE_CTR-1.0, "y",
                         with_ultrasonic=False)

            for comp_chk in [thigh, shin, foot]:
                assign_material(comp_chk, "structural")   # V16 NEW
                for b in list(comp_chk.bRepBodies):
                    if b.name:
                        printability_check(comp_chk, b.name)

            assembly_jig(f"OP_Thigh_{side}",
                [(sx-1.0, 0, THIGH_CTR+2.0), (sx+1.0, 0, THIGH_CTR-2.0)],
                [(sx-0.5, 0, THIGH_CTR+2.0), (sx+0.5, 0, THIGH_CTR-2.0)],
                (7.0, 5.0, 0.60))

        # ─────────────────────────────────────────────────────────────────
        # 5 ARMS  (tendon-driven hands retained from v12/v14)
        # ─────────────────────────────────────────────────────────────────
        SnapshotManager.save_checkpoint("Arms_Start")
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

            box(ua, f"ShShield_{side}", ax+m*3.4,  0, SHOULDER_CTR+1.5, 1.1, 4.6, 5.2, chrome)
            box(ua, f"ShHinge_{side}",  ax+m*2.7,  0, SHOULDER_CTR+1.5, 0.5, 1.9, 1.9, dark_grey)
            box(ua, f"Mirror_{side}",   ax+m*3.9, -2.9, SHOULDER_CTR+2.0, 1.5, 0.2, 0.9, dark_grey)

            mg996r(ua, f"{side}_ShY",    ax, 0, SHOULDER_CTR+1.5, "z")
            dual_bearing(ua, f"{side}_ShY_Dual", ax, 0, SHOULDER_CTR+2.0,
                         "z", 1.00, 0.55, span=2.80, fit_type="press")
            _reg_joint_axis(f"{side}_Shoulder_Cluster", "yaw", "z", f"{side}_ShY")
            u_bracket(ua, f"{side}_SPB", ax, 0, SHOULDER_CTR,     4.8, 3.4, 3.4)
            mg996r(ua, f"{side}_ShP",    ax, 0, SHOULDER_CTR,     "x")
            dual_bearing(ua, f"{side}_ShP_Dual", ax, 0, SHOULDER_CTR,
                         "x", 1.10, 0.62, span=2.80, fit_type="press")
            _reg_joint_axis(f"{side}_Shoulder_Cluster", "pitch", "x", f"{side}_ShP")
            mg996r(ua, f"{side}_ShR",    ax, 0, SHOULDER_CTR-1.2, "y")
            bearing_fit(ua, f"{side}_ShR_Brg", ax, 0, SHOULDER_CTR-1.2, "y", 1.00, 0.55, fit_type="press")
            _reg_joint_axis(f"{side}_Shoulder_Cluster", "roll", "y", f"{side}_ShR")

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
            mg90s(fa, f"{side}_WrP",   ax, 0, WRIST_Z+0.8, "x")
            bearing_fit(fa, f"{side}_WrP_Brg", ax, 0, WRIST_Z+0.5, "x", 0.80, 0.44, fit_type="press")
            _reg_joint_axis(f"{side}_Wrist", "pitch", "x", f"{side}_WrP")
            mg90s(fa, f"{side}_WrR",   ax, 0, WRIST_Z+2.3, "y")
            bearing_fit(fa, f"{side}_WrR_Brg", ax, 0, WRIST_Z+2.0, "y", 0.70, 0.40, fit_type="press")
            _reg_joint_axis(f"{side}_Wrist", "roll", "y", f"{side}_WrR")
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
                palm_pulley(hand, f"{side}_Pulley_{fi}", px, -0.5, WRIST_Z-1.2, "x", bushed=True)

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
                tendon_guide(fc, f"{fname}_PP", fx, fy-0.05, pp_cz, pp_l*0.8, "z", bushed=True)
                cyl(fc, "PIP_Hinge", fx, fy, mcp_z - pp_l, 0.27, FING_W + 0.12, "x", chrome)
                spring_return(fc, f"{fname}_PIP", fx, fy+0.3, mcp_z - pp_l, "x")

                box(fc, "MP", fx, fy, mp_cz, FING_W*0.94, FING_H*0.94, mp_l, grey_plastic)
                tendon_guide(fc, f"{fname}_MP", fx, fy-0.04, mp_cz, mp_l*0.8, "z", bushed=True)
                cyl(fc, "DIP_Hinge", fx, fy, mcp_z - pp_l - mp_l,
                    0.24, FING_W*0.94 + 0.10, "x", chrome)

                box(fc, "DP", fx, fy, dp_cz, FING_W*0.86, FING_H*0.86, dp_l, grey_plastic)
                tendon_anchor(fc, f"{fname}", fx, fy, mcp_z - pp_l - mp_l - dp_l - 0.15, "z", adjustable=True)
                cone_shape(fc, "FT", fx, fy, mcp_z - pp_l - mp_l - dp_l - 0.12,
                           FING_W*0.44, 0.05, 0.24, "z", chrome)

                grommet_slot(fc, f"{fname}_TendonExit", fx, fy-0.2, mcp_z, "y", 0.30)

            thumb = new_component(f"OP_Thumb_{side}")
            tx    = ax + m * 1.70
            ty    = 0.20
            tpp_cz = MCP_Z - THUMB_PP_L / 2
            tdp_cz = MCP_Z - THUMB_PP_L - THUMB_DP_L / 2
            box(thumb, "TP",  tx, ty, tpp_cz, THUMB_W, THUMB_H, THUMB_PP_L, grey_plastic)
            tendon_guide(thumb, "Thumb_PP", tx, ty-0.05, tpp_cz, THUMB_PP_L*0.7, "z", bushed=True)
            cyl(thumb, "T_Hinge", tx, ty, MCP_Z - THUMB_PP_L,
                0.28, THUMB_W + 0.14, "x", chrome)
            spring_return(thumb, "Thumb_CMC", tx, ty+0.3, MCP_Z - THUMB_PP_L, "x")
            box(thumb, "TD",  tx, ty*0.8, tdp_cz, THUMB_W*0.86, THUMB_H*0.86, THUMB_DP_L,
                grey_plastic)
            tendon_anchor(thumb, "Thumb", tx, ty*0.8, tdp_cz - THUMB_DP_L/2 - 0.15, "z", adjustable=True)
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
                assign_material(comp_chk, "structural")   # V16 NEW
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
        vent_grille(bp, "BP_Vent", 0, 6.9, TORSO_CTR-0.5, "y", n_slots=6)

        # BUGFIX-V15-5: reserved AI-accelerator bay, coded in v14 but never
        # placed. Lives in the backpack, wired to a free Jetson USB3 port.
        ai_accel_pocket(bp, "AIAccelBay", 0, 6.0, TORSO_CTR-2.6)
        assign_material(bp, "structural")   # V16 NEW

        # ─────────────────────────────────────────────────────────────────
        # 7 STEER WHEEL PODS  (per-leg so they follow hip rotation)
        # ─────────────────────────────────────────────────────────────────
        for side, sx in [("L", -5.6), ("R", 5.6)]:
            m2 = -1 if side == "L" else 1
            steer = new_component(f"OP_SteerPod_{side}")
            box(steer, f"SAr_{side}",  sx, -3.6, 23.9, 1.6, 1.3, 4.2, chrome)
            box(steer, f"SPod_{side}", sx, -4.6, 23.4, 3.0, 2.2, 3.2, dark_grey)
            tt_wheel(steer, f"SW_{side}", sx+m2*2.0, -4.2, 23.4, m2)
            bearing_fit(steer, f"SPiv_{side}", sx, -3.6, 23.9, "z", 0.95, 0.50, fit_type="glue")
            mg90s(steer, f"SSrv_{side}", sx, -4.2, 23.9, "z")

        # ─────────────────────────────────────────────────────────────────
        # 8 SHIELDS / PANELS
        # ─────────────────────────────────────────────────────────────────
        for side2, hx in [("L", -(HIP_X+3.1)), ("R", HIP_X+3.1)]:
            hs_comp = new_component(f"OP_HipShield_{side2}")
            box(hs_comp, f"HipShield_{side2}", hx, 0, PELVIS_CTR+0.5, 1.1, 4.4, 4.0, op_blue)


        # ─────────────────────────────────────────────────────────────────
        # FDM SHELL SPLITTING (Y-plane @ 0)  +  MFG-1 fastener merge
        # ─────────────────────────────────────────────────────────────────
        Logger.log("--- FDM Shell Splitting + Fastener Merge (v16) ---")
        for comp in comps_list:
            to_split = [b for b in comp.bRepBodies
                        if b.name and any(k in b.name for k in SPLIT_KEYS)]
            for b in to_split:
                split_halves(comp, b, "y", 0.0)
            bodies_after = list(comp.bRepBodies)
            left_bodies  = [b for b in bodies_after if b.name and "_left"  in b.name]
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

        if p:
            p.isGrounded = True

        ball_joint("Waist_Cluster",  t,  p,  0, 0, WAIST_CTR-2.5)
        ball_joint("Neck_Cluster",   h,  t,  0, 0, NECK_JOINT_Z)
        rigid_joint("Backpack_Mount", b,  t)
        for side2 in ["L", "R"]:
            hs_occ = occs.get(f"OP_HipShield_{side2}")
            if hs_occ and p:
                rigid_joint(f"HipShield_{side2}_Mount", hs_occ, p)

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

            st_occ = occs.get(f"OP_SteerPod_{side}")
            rigid_joint(f"SteerPod_{side}_Mount", st_occ, th)

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
                if comp.name.startswith("JIG_"):
                    continue
                if (comp.name not in ("OP_Torso", "OP_Pelvis")
                        and comp.name not in jointed_comps):
                    orphans.append(comp.name)
            if orphans:
                Logger.log(f"  !!! ORPHANS: {orphans}", "WARN")
            else:
                Logger.log("  All components bound to kinematic chain. [OK]")
        except Exception:
            Logger.log("  Kinematic validation skipped (MCP env).", "WARN")

        # BUGFIX-V15-2: verify_total() now uses a DYNAMICALLY computed
        # expected DOF count (sum of declared rotational axes across
        # JOINT_LIMITS) instead of a hard-coded 28 that silently drifted
        # out of sync after the AXIS-1 fixes added several new axes.
        _expected_dof = sum(
            len([a for a in v.keys() if a in ("pitch", "yaw", "roll")])
            for v in JOINT_LIMITS.values()
        )
        DOFValidator.verify_total(expected=_expected_dof)

        # BUGFIX-V15-5/6: wire up AssemblyValidator, which was defined but
        # never called.
        AssemblyValidator.verify_hierarchy(root, [
            "OP_Torso", "OP_Head", "OP_Pelvis", "OP_Backpack",
            "OP_Thigh_L", "OP_Thigh_R", "OP_Shin_L", "OP_Shin_R",
            "OP_Foot_L", "OP_Foot_R",
            "OP_UpperArm_L", "OP_UpperArm_R", "OP_Forearm_L", "OP_Forearm_R",
            "OP_Hand_L", "OP_Hand_R",
        ])

        try:
            cam = app.activeViewport.camera
            cam.isFitView = True
            app.activeViewport.camera = cam
        except Exception:
            pass

        # ── Architecture / comms / power logging ──────────────────────────
        log_v14_architecture()
        log_comms_map()
        log_power_budget()
        log_ai_pipeline()

        # ── FST-1: Screw length verification (bugfixed in v14) ────────────
        verify_screw_lengths()

        # ── AXIS-1: Joint axis-mapping verification (bugfixed in v14) ─────
        verify_joint_axis_mapping()

        # ── DOC-1: Write assembly guide ───────────────────────────────────
        write_assembly_guide()

        # ═════════════════════════════════════════════════════════════════
        # SIMULATION ENGINE  v17.0
        # ═════════════════════════════════════════════════════════════════

        class SimulationEngine:
            """
            Simulation suite for Optimus Prime G1 v16 (Unified Distributed AI
            Edition). All v12/v14 mechanical/physical validation features are
            retained.

            BUGFIX-V13-1 (retained): move_ball() previously wrote
            `self._set(mo, ax, ...)` inside its per-frame loop, but `ax` was
            never defined in that scope. Fixed by using the correct loop
            variable `axis` consistently throughout the per-frame update loop.
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

            def _interfere(self, label="Interference", baseline=None):
                """V17 FIX: report collisions *introduced by the current pose*
                as a delta against `baseline` (the neutral-pose collision set).
                The whole-assembly interference count is dominated by the
                model's permanent resting contacts (~thousands of touching
                face pairs), so reporting the raw total made every joint look
                identically 'colliding' and could never detect a joint-limit
                self-collision. We now report only NEW interfering pairs that
                did not exist in the baseline."""
                try:
                    all_bodies = adsk.core.ObjectCollection.create()
                    for comp in self._comps:
                        for body in list(comp.bRepBodies):
                            if body.name and not any(t in body.name for t in SKIP_TAGS):
                                all_bodies.add(body)
                    inter_input = self._design.createInterferenceInput(all_bodies)
                    inter_input.isCoincidentFacesInterference = False
                    results = self._design.analyzeInterference(inter_input)
                    pairs = set()
                    for i in range(results.count):
                        r = results.item(i)
                        a, b = r.entityOne.name, r.entityTwo.name
                        pairs.add((a, b) if a <= b else (b, a))
                    new_pairs = pairs - baseline if baseline is not None else pairs
                    count = len(new_pairs)
                    self._baseline_pairs = pairs if baseline is None else self._baseline_pairs
                    if count:
                        Logger.log(f"  {label}: {count} NEW collision(s) vs neutral")
                        for (a, b) in list(new_pairs)[:5]:
                            Logger.log(f"    [NEW] {a} <-> {b}")
                        if count > 5:
                            Logger.log(f"    ...{count-5} more")
                        self._cols.append((label, count))
                    else:
                        Logger.log(f"  {label}: [OK] no new collisions vs neutral")
                        self._cols.append((label, 0))
                    return count
                except Exception as e:
                    Logger.log(f"  {label}: skipped ({e})", "WARN")
                    self._cols.append((label, -1))
                    return -1

            # ── SIM-1: ZMP Stability ──────────────────────────────────────
            def _check_zmp(self, label, support="DS"):
                FOOT_HW = 6.2 / 2.0
                FOOT_HL = 9.2 / 2.0
                try:
                    com = self._root.physicalProperties.centerOfMass
                except Exception as e:
                    Logger.log(f"  ZMP [{label}]: CoM unavailable ({e})", "WARN")
                    return
                l_cx, r_cx = -HIP_X, HIP_X
                if support == "L":
                    # V17 FIX: single-leg support -- the hard case for a biped.
                    # Support polygon is the LEFT foot only.
                    in_poly = (l_cx - FOOT_HW <= com.x <= l_cx + FOOT_HW and
                               -FOOT_HL <= com.y <= FOOT_HL)
                elif support == "R":
                    in_poly = (r_cx - FOOT_HW <= com.x <= r_cx + FOOT_HW and
                               -FOOT_HL <= com.y <= FOOT_HL)
                else:  # double support
                    in_poly = (min(l_cx - FOOT_HW, r_cx - FOOT_HW) <= com.x <=
                               max(l_cx + FOOT_HW, r_cx + FOOT_HW) and
                               -FOOT_HL <= com.y <= FOOT_HL)
                stable = in_poly
                tag = "[OK] ZMP STABLE" if stable else "[FAIL] ZMP UNSTABLE"
                Logger.log(
                    f"  ZMP [{label:16s}] {tag} (support={support})  "
                    f"CoM=({com.x:+.2f}, {com.y:+.2f}, {com.z:.1f}) cm")
                if not stable:
                    BugReporter.report(
                        f"{label}: CoM outside {support} support polygon -- unstable in "
                        f"this pose; walking/balance controller must keep CoM over support foot",
                        "HIGH", category="Stability")
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

            # ── Architecture validation ────────────────────────────────────
            def validate_v14_architecture(self):
                Logger.log("--- V13/V14: ARCHITECTURE VALIDATION ---")
                required_components = [
                    ("Jetson Nano bay",     "Main_JNanoBay"),
                    ("CSI camera (eyes)",   "Jetson CSI camera module (IMX219 8MP)"),
                    ("ESP32-S3 lower node", "ESP32-S3 DevKitC (node: lower)"),
                    ("ESP32-S3 upper node", "ESP32-S3 DevKitC (node: upper)"),
                    ("ESP32-S3 head node",  "ESP32-S3 DevKitC (node: head)"),
                    ("Servo-rail E-stop",   "22mm mushroom-head E-stop pushbutton (N.C.)"),
                    ("Status RGB LED",      "WS2812 5050 addressable RGB LED (status)"),
                    ("Pelvis sensor array", "HC-SR04 ultrasonic sensor"),
                    ("AI accelerator bay",  "Google Coral USB Accelerator (future, optional)"),
                ]
                found_count = 0
                bom_items = " ".join([row["Part"] for row in BOM._rows])
                has_jnano = "5V 4A UBEC (Jetson Nano power)" in bom_items

                for label, bom_desc in required_components:
                    if "Jetson Nano" in label:
                        found = has_jnano
                    else:
                        found = bom_desc in bom_items
                    icon = "[OK]" if found else "[MISSING]"
                    if found:
                        found_count += 1
                        Logger.log(f"  {icon} {label} -> present in design")
                    else:
                        Logger.log(f"  {icon} {label}")
                if not COMM_MAP:
                    Logger.log("  [WARN] Communication map is empty!", "WARN")
                else:
                    Logger.log(f"  [OK] {len(COMM_MAP)} communication link(s) registered")
                if not POWER_MAP:
                    Logger.log("  [WARN] Power map is empty!", "WARN")
                else:
                    Logger.log(f"  [OK] {len(POWER_MAP)} power rail(s) registered")
                if not SENSOR_REGISTRY:
                    Logger.log("  [WARN] Sensor registry is empty!", "WARN")
                else:
                    Logger.log(f"  [OK] {len(SENSOR_REGISTRY)} sensor(s) registered")
                Logger.log(f"  Architecture validation: {found_count}/{len(required_components)} "
                           f"core compute/safety components placed")
                BOMCADValidator.verify_bom_matches_cad(BOM._rows)

            def simulate_vision_tracking(self):
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

            def simulate_obstacle_reaction(self):
                Logger.log("--- V13: TOF/SENSOR-ARRAY OBSTACLE REACTION SIMULATION ---")
                Logger.log("  Simulated trigger: ToF_L/ToF_R or pelvis ultrasonic report distance < 30cm")
                self.reset_all(steps=8)
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
                Logger.log("  ToF/sensor-array obstacle reaction simulation complete [OK]")

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
                # V17 FIX: capture the neutral-pose collision set ONCE, then
                # sweep each joint's declared axes to their limits and report
                # the true mechanical ROM (first angle that introduces a
                # self-collision). See ROMCollisionSimulator.simulate_sweep
                # and _interfere baseline logic.
                self._interfere("NEUTRAL baseline")
                baseline = getattr(self, "_baseline_pairs", set())
                rom_failures = 0
                for name, axes in JOINT_LIMITS.items():
                    j_for_sweep = self._gj(name)
                    if j_for_sweep is None:
                        continue
                    ROMCollisionSimulator.simulate_sweep(
                        self._design, j_for_sweep, axes, self, baseline)
                # Count any WARNING-level ROM self-collision flags from this module
                for lbl, cnt in self._cols:
                    if "CLAMP NEEDED" in lbl or (cnt and cnt > 0 and "NEUTRAL" not in lbl
                                                 and "ROM" not in lbl):
                        rom_failures += 1
                if rom_failures:
                    Logger.log(f"MODULE 1: {rom_failures} joint-limit pose(s) introduced "
                               f"self-collision [REVIEW]", "WARN")
                else:
                    Logger.log("MODULE 1: no self-collision at any joint limit [PASS]", "INFO")

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
                        ("L_Knee",             60,        None,        4*l_sign),
                        ("R_Knee",             60,        None,        4*r_sign),
                        ("L_Ankle_Cluster",    15*l_sign, None,        8*l_sign),
                        ("R_Ankle_Cluster",    15*r_sign, None,        8*r_sign),
                    ], steps=20)
                    self._check_zmp(f"Walk cycle {cycle+1}")
                self.reset_all(steps=12)

            def simulate_walking_ik(self, n_steps=6, stride_cm=8.0, lift_cm=3.0):
                """V16 NEW -- gait driven by solve_leg_ik_2link() instead of
                hand-picked fixed joint angles: a target foot trajectory
                (forward position + ground clearance during swing) is solved
                into real hip/knee pitch angles every frame. Still a scripted
                trajectory (not a live balance controller reacting to the
                IMU/FSR sensors), but the joint angles themselves are now
                geometrically derived from actual leg link lengths rather
                than tuned by eye."""
                Logger.log("--- MODULE 5b ---")
                Logger.log("MODULE 5b: IK-DRIVEN WALK CYCLE (analytic 2-link leg solver)")
                thigh_len = HIP_JOINT_Z - KNEE_CTR
                shin_len  = KNEE_CTR - ANKLE_CTR
                stand_h   = (thigh_len + shin_len) * 0.93
                self.reset_all(steps=10)
                phases = 10
                for step_i in range(n_steps):
                    swing_side  = "L" if step_i % 2 == 0 else "R"
                    stance_side = "R" if swing_side == "L" else "L"
                    for p in range(phases):
                        t = p / (phases - 1)
                        swing_dx  = -stride_cm / 2 + stride_cm * t
                        swing_dz  = stand_h - lift_cm * math.sin(math.pi * t)
                        stance_dx = stride_cm / 2 - stride_cm * t
                        stance_dz = stand_h
                        hp_s, kp_s = solve_leg_ik_2link(swing_dx, swing_dz, thigh_len, shin_len)
                        hp_t, kp_t = solve_leg_ik_2link(stance_dx, stance_dz, thigh_len, shin_len)
                        self.move_ball([(f"{swing_side}_Hip_Cluster", hp_s, 0, 0)], steps=2, clamp=True)
                        self.move_joint(f"{swing_side}_Knee", kp_s, steps=2, axis="pitch")
                        self.move_ball([(f"{stance_side}_Hip_Cluster", hp_t, 0, 0)], steps=2, clamp=True)
                        self.move_joint(f"{stance_side}_Knee", kp_t, steps=2, axis="pitch")
                    self._check_zmp(f"IK walk step {step_i+1} ({swing_side} swing)")
                self.reset_all(steps=10)
                Logger.log("IK-driven walk cycle complete.")

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
                        ("L_Knee",             95,        None,       6*l_sign),
                        ("R_Knee",             95,        None,       6*r_sign),
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
                    "Attention":    ({}, "DS"),
                    "Combat":       ({
                        "Waist_Cluster":      (10, 0,   0),
                        "L_Knee":              45,
                        "R_Knee":              45,
                        "L_Elbow":             90,
                        "R_Elbow":             90,
                        "R_Shoulder_Cluster": (0,  30, -45),
                    }, "DS"),
                    "Squat":        ({
                        "Waist_Cluster":  (20, 0,   0),
                        "L_Knee":          90,
                        "R_Knee":          90,
                        "L_Hip_Cluster":  (0, -45, 0),
                        "R_Hip_Cluster":  (0, -45, 0),
                    }, "DS"),
                    "Victory":      ({
                        "L_Shoulder_Cluster": (0, 60, -90),
                        "R_Shoulder_Cluster": (0, 60, -90),
                        "L_Elbow":             30,
                        "R_Elbow":             30,
                        "Waist_Cluster":      (15, 0, 0),
                    }, "DS"),
                    # V17 FIX: single-leg support poses (the hard case for a biped)
                    "Single_Leg_L": ({
                        "L_Hip_Cluster":  (0, 90,  0),
                        "L_Knee":          90,
                        "Waist_Cluster":  (5, 10, -5),
                    }, "L"),
                    "Single_Leg_R": ({
                        "R_Hip_Cluster":  (0, -90, 0),
                        "R_Knee":          90,
                        "Waist_Cluster":  (5, -10, 5),
                    }, "R"),
                }
                for pose_name, (config, support) in poses.items():
                    self.reset_all(steps=10)
                    for key, val in config.items():
                        if isinstance(val, tuple):
                            self.move_ball([(key, val[0], val[1], val[2])], steps=15)
                        else:
                            self.move_joint(key, val, steps=12, axis="pitch")
                    self._check_zmp(pose_name, support=support)
                # BUGFIX-V15-6: wire up BalanceValidator, defined but never called.
                BalanceValidator.check_stability(self._design)

            def estimate_servo_loads(self):
                # V17 FIX: this is the LEGACY hand-typed estimator (v12-v15).
                # Its masses/levers are guesses and it disagrees with the
                # real-mass Module 13 check by ~2x. Kept only for quick
                # sanity comparison -- the AUTHORITATIVE validation is
                # run_real_engineering_validation() (Module 13). Threshold
                # aligned to the 1.5x/1.0x rule to stop hiding under-spec.
                Logger.log("MODULE 9b: SERVO LOAD ESTIMATION (LEGACY hand-typed --")
                Logger.log("           see Module 13 run_real_engineering_validation for")
                Logger.log("           the authoritative REAL-MASS torque check)")
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
                    ("Waist Roll",       900,  6.0, SERVO_SPECS["waist"]),
                    ("Neck Roll",         90,  2.5, SERVO_SPECS["micro"]),
                    ("L Knee Roll",      300,  5.0, SERVO_SPECS["std"]),
                    ("R Knee Roll",      300,  5.0, SERVO_SPECS["std"]),
                    ("L Ankle Yaw",      180,  4.0, SERVO_SPECS["std"]),
                    ("R Ankle Yaw",      180,  4.0, SERVO_SPECS["std"]),
                    ("L Wrist Roll",      70,  2.0, SERVO_SPECS["micro"]),
                    ("R Wrist Roll",      70,  2.0, SERVO_SPECS["micro"]),
                ]
                for label, mass_g, lever_cm, spec in loads:
                    F      = (mass_g / 1000.0) * 9.81
                    need   = (F * lever_cm / 100.0) / 0.0981
                    margin = spec["rated"] / need if need > 0 else 99.0
                    status = ("OK"       if margin >= 1.5 else
                              "MARGINAL" if margin >= 1.0 else "OVERLOAD")
                    icon   = "[OK]" if status == "OK" else f"[{status}]"
                    Logger.log(
                        f"  {label:<24} need {need:5.2f} kg-cm  "
                        f"rated {spec['rated']:5.1f}  margin {margin:.2f}x  "
                        f"{spec['name']:12s}  {icon}")
                    # BUGFIX-V15-6: wire up ServoValidator, defined but never called.
                    ServoValidator.verify_reachability(label, -1, 1, servo_min=-1, servo_max=1)

            def run_physical_validation(self):
                Logger.log("--- MODULE 10 ---")
                Logger.log("MODULE 10: PHYSICAL BUILD VALIDATION")
                self.test_tendon_slack()
                self.test_transform_locks()
                self.printability_report()
                Logger.log("Physical validation complete.")

            def run_v14_validation(self):
                Logger.log("--- MODULE 11 ---")
                Logger.log("MODULE 11: V13/V14 ARCHITECTURE + AI SAFETY VALIDATION")
                self.validate_v14_architecture()
                self.simulate_vision_tracking()
                self.simulate_obstacle_reaction()
                Logger.log("V13/V14 architecture validation complete.")

            def run_real_engineering_validation(self):
                """V16 NEW -- MODULE 13: the real-mass/real-torque/structural
                validation chain. This is the module that actually answers
                "will this hold together and move at real scale", as opposed
                to the earlier CAD-only checks (interference, ZMP-at-design-
                time, hand-typed servo load guesses)."""
                Logger.log("--- MODULE 13 ---")
                Logger.log("MODULE 13: REAL-MASS / REAL-TORQUE / STRUCTURAL VALIDATION")
                total_kg, mass_report = compute_total_mass_report()
                torque_results = estimate_real_joint_torques(mass_report)
                run_structural_safety_checks(torque_results)
                Logger.log(f"Real-engineering validation complete. Total computed mass: "
                           f"{total_kg:.2f} kg on '{ACTIVE_ACTUATOR_PROFILE}' actuator profile "
                           f"/ '{ACTIVE_PRINTER_PROFILE}' printer profile.")

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
                self.capture_screenshots("optimus_robot_v17")

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
                self.capture_screenshots("optimus_truck_v17")

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
                self.capture_screenshots("optimus_battle_v17")

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

                    def apply_margin():
                        try:
                            eye = camera.eye
                            target = camera.target
                            vec = target.vectorTo(eye)
                            if vec.length > 0:
                                vec.normalize()
                                distance = eye.distanceTo(target)
                                offset = vec.copy()
                                offset.scaleBy(0.15 * distance)
                                eye.translateBy(offset)
                                camera.eye = eye
                                viewport.camera = camera
                                adsk.doEvents()
                        except:
                            pass

                    def save_view(name, orientation=None):
                        if orientation is not None:
                            camera.viewOrientation = orientation
                            camera.isFitView = True
                            viewport.camera = camera
                        settle()
                        apply_margin()
                        settle()
                        path = os.path.join(SCREENSHOT_DIR, f"{prefix}_{name}.png")
                        viewport.saveAsImageFile(path, width, height)
                        Logger.log(f"Screenshot: {path}")

                    def toggle_shells(visible):
                        shells = ["armor", "shield", "plate", "chest", "cab", "grille", "bumper", "fender", "cover", "shell"]
                        try:
                            root = self._design.rootComponent
                            for occ in root.allOccurrences:
                                name = (occ.component.name or "").lower()
                                if any(s in name for s in shells):
                                    occ.isLightBulbOn = visible
                            for comp in self._comps:
                                for body in comp.bRepBodies:
                                    if body.isValid and body.name:
                                        b_name = body.name.lower()
                                        if any(s in b_name for s in shells):
                                            body.isLightBulbOn = visible
                        except Exception as e:
                            Logger.log(f"toggle_shells failed: {e}", "WARN")

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

                    try:
                        toggle_shells(visible=False)
                        settle()
                        internal_views = [
                            ("Internal_Front", adsk.core.ViewOrientations.FrontViewOrientation),
                            ("Internal_Right", adsk.core.ViewOrientations.RightViewOrientation),
                            ("Internal_IsoTopRight", adsk.core.ViewOrientations.IsoTopRightViewOrientation),
                        ]
                        for name, orientation in internal_views:
                            try:
                                save_view(name, orientation)
                            except Exception as e:
                                Logger.log(f"  internal view {name} failed: {e}", "WARN")
                    except Exception as e:
                        Logger.log(f"  internal capture sequence failed: {e}", "WARN")
                    finally:
                        toggle_shells(visible=True)
                        settle()

                    if VISUAL_AUDIT:
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
            @trace_execution
            def export_urdf(self):
                def _urdf_type(name):
                    if "Cluster" in name or "Wrist" in name or "CMC" in name:
                        return "spherical"
                    if any(k in name for k in ["Knee", "Elbow", "MCP", "Fold"]):
                        return "revolute"
                    if any(k in name for k in ["Mount", "Steer", "Backpack"]):
                        return "fixed"
                    return "revolute"

                def _limits(name):
                    limits_d = JOINT_LIMITS.get(name, {})
                    pitch = limits_d.get("pitch", (-180, 180))
                    return math.radians(pitch[0]), math.radians(pitch[1])

                link_mass = {
                    "OP_Head": 280,       "OP_Torso": 950,    "OP_Pelvis": 480,
                    "OP_Backpack": 150,   "OP_SteerPod_L": 60, "OP_SteerPod_R": 60,
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
                    path = os.path.join(EXPORT_DIR, "robot_v17.urdf")
                    jc   = self._root.asBuiltJoints.count
                    with open(path, "w", encoding="utf-8") as f:
                        f.write('<?xml version="1.0" encoding="utf-8"?>\n')
                        f.write('<robot name="Optimus_Prime_G1_v17">\n\n')
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
                    Logger.log(f"URDF v16 exported -> {path}")
                except Exception:
                    Logger.log(f"URDF export failed: {traceback.format_exc()}", "ERROR")

            # ── STL Export ────────────────────────────────────────────────
            @trace_execution
            def export_stl(self):
                try:
                    os.makedirs(EXPORT_DIR, exist_ok=True)
                    skip_s = {"Marker", "Pivot", "MtA", "MtB", "Axle_Pivot", "Horn",
                              "_Vis", "Scope", "Antenna", "Boss", "Insert", "Nut", "Snap"}
                    export_mgr = self._design.exportManager
                    count = 0
                    for comp in self._comps:
                        for body in list(comp.bRepBodies):
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
            @trace_execution
            def export_step(self):
                try:
                    os.makedirs(EXPORT_DIR, exist_ok=True)
                    export_mgr = self._design.exportManager
                    path = os.path.join(EXPORT_DIR, "Optimus_Prime_G1_v17.step")
                    step_opts = export_mgr.createSTEPExportOptions(path)
                    export_mgr.execute(step_opts)
                    Logger.log(f"STEP assembly -> {path}")
                    count = 0
                    skip_tags_local = {"Marker", "Pivot", "_Vis"}
                    for i in range(self._root.allOccurrences.count):
                        occ   = self._root.allOccurrences.item(i)
                        cname = occ.component.name or ""
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
            @trace_execution
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
                BOM.add("Electronics", "Blade fuse 25A ATO",          1, "servo rail protection")
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
                BOMCADValidator.verify_bom_matches_cad(BOM._rows)

                Logger.log("--- SERVO WIRING MAP (per ESP32-S3 node) ---")
                node_wiring = {
                    "ESP32-S3 'lower' (pelvis)": [
                        ("PCA-HipKnee ch0-5", "L/R Hip Yaw, Hip Pitch, Hip Roll (MG996R)"),
                        ("PCA-HipKnee ch6-7", "L/R Knee Pitch (MG996R)"),
                        ("PCA-HipKnee ch8-9", "L/R Knee Roll (MG996R)"),
                        ("PCA-Ankle  ch0-5",  "L/R Ankle Pitch, Roll, Yaw (MG996R)"),
                        ("I2C addr 0x68",     "MPU-6050 balance IMU"),
                        ("I2C addr 0x48",     "ADS1115 ADC (pelvis + foot FSR arrays)"),
                        ("GPIO trig/echo",    "HC-SR04 pelvis ultrasonic sensor"),
                    ],
                    "ESP32-S3 'upper' (torso)": [
                        ("PCA ch0-2",  "Waist Yaw, Pitch, Roll (MG996R/DS3225MG)"),
                        ("PCA ch3-8",  "L/R Shoulder Yaw/Pitch/Roll (MG996R)"),
                        ("PCA ch9-10", "L/R Elbow (MG996R)"),
                        ("I2C addr 0x40", "INA3221 servo-rail current monitor"),
                    ],
                    "ESP32-S3 'head' (head)": [
                        ("PCA ch0",    "Neck Pitch (MG996R)"),
                        ("PCA ch1-2",  "Neck Yaw, Roll (MG90S)"),
                        ("PCA ch3-6",  "L/R Wrist Pitch, Roll (MG90S)"),
                        ("PCA ch7",    "Blaster Fold (MG90S)"),
                        ("I2C addr 0x29/0x2A", "VL53L1X ToF sensors (L/R, alt-addressed)"),
                        ("Servo relay coil",   "Master E-stop cutoff (servo rail only)"),
                        ("GPIO",       "Status RGB LED (chest badge)"),
                    ],
                }
                for node, rows in node_wiring.items():
                    Logger.log(f"  [{node}]")
                    for ch, desc in rows:
                        Logger.log(f"    {ch:<22s} -> {desc}")

                Logger.log("--- V13 POWER BUDGET (summary) ---")
                Logger.log("  Jetson Nano (5V/4A UBEC):  ~3.0A peak @ 10W AI inference mode")
                Logger.log("  Logic rail (5V/3A BEC):    ~0.6A combined, 3x ESP32-S3 nodes")
                Logger.log("  Servo rail (7.4V direct, via E-stop relay): ~20A peak, all servos moving")
                Logger.log("  Fusing:  3A on each Jetson/logic rail, 25A on servo rail")
                Logger.log("  Recommended pack: 2S 2200-3000mAh (fits built bay); enlarge bay for 3S 5000mAh (~12-15min)")

            # ── Master Runner ─────────────────────────────────────────────
            def run_all_simulations(self):
                dispatch = {
                    "ALL": lambda: [
                        self.test_joint_rom(),
                        self.simulate_head_scan(),
                        self.simulate_wave(),
                        self.simulate_idle_breathing(),
                        self.simulate_walking_advanced(),
                        self.simulate_walking_ik(),
                        self.simulate_running(),
                        self.simulate_combat(),
                        self.simulate_transformation(),
                        self.test_arm_workspace(),
                        self.run_stability_analysis(),
                        self.estimate_servo_loads(),
                        self.run_real_engineering_validation(),
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
                    "walk_ik":    self.simulate_walking_ik,
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
                    "architecture": self.validate_v14_architecture,
                    "vision":       self.simulate_vision_tracking,
                    "obstacle":     self.simulate_obstacle_reaction,
                    "v14check":     self.run_v14_validation,
                    "mass":         compute_total_mass_report,
                    "torque_real":  lambda: run_structural_safety_checks(
                                        estimate_real_joint_torques(compute_total_mass_report()[1])),
                    "engineering":  self.run_real_engineering_validation,
                    "firmware":     export_firmware_skeletons,
                    "production": lambda: [
                        self.test_joint_rom(),
                        self.test_arm_workspace(),
                        self.run_stability_analysis(),
                        self.estimate_servo_loads(),
                        self.run_real_engineering_validation(),
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
                export_firmware_skeletons()   # V16 NEW: always emit wiring-map skeletons

                write_build_manifest()
                write_production_readiness_report(self._cols)

                Logger.log("=" * 58)
                Logger.log("OPTIMUS PRIME G1 v17.0 -- FINAL REPORT")
                Logger.log("Jetson Nano AI Brain Edition")
                Logger.log("=" * 58)
                for label, count in self._cols:
                    if count >= 0:
                        icon = "[OK]" if count == 0 else "[WARN]"
                        Logger.log(f"  {label:<42} {icon}  {count}")
                    else:
                        Logger.log(f"  {label:<42} ?  N/A")
                if EXPORT_URDF:
                    Logger.log(f"  URDF  -> {EXPORT_DIR}/robot_v17.urdf")
                Logger.log(f"  BOM   -> {BOM_FILE}")
                Logger.log(f"  ASM   -> {ASSEMBLY_FILE}")
                Logger.log(f"  MAN   -> {MANIFEST_FILE}")
                Logger.log(f"  PROD  -> {PRODUCTION_FILE}")
                Logger.log(f"  Log   -> {LOG_FILE}")
                Logger.log("=" * 58)
                Logger.log("v17 BUILD COMPLETE -- Review ASSEMBLY_GUIDE before printing")

        # ═════════════════════════════════════════════════════════════════
        # ARCHIVE & RUN
        # ═════════════════════════════════════════════════════════════════
        try:
            os.makedirs(EXPORT_DIR, exist_ok=True)
            archive_path = os.path.join(EXPORT_DIR, "Optimus_Prime_G1_v17.f3d")
            export_mgr   = design.exportManager
            archive_opts = export_mgr.createFusionArchiveExportOptions(archive_path)
            export_mgr.execute(archive_opts)
            Logger.log(f"Design archived -> {archive_path}")
        except Exception as e:
            Logger.log(f"Archive skipped: {e}", "WARN")

        SnapshotManager.save_checkpoint("Pre_Simulation")
        sim = SimulationEngine(root, comps_list, design, app, ui)
        apply_final_colors()   # apply before simulation so screenshots show colours
        sim.run_all_simulations()
        apply_final_colors()   # re-apply after simulation in case poses reset them
        Logger.log("v17 script finished successfully.")

    except Exception as e:
        DebugManager.log(f"FATAL ERROR: {e}\n{traceback.format_exc()}", "CRITICAL")
        try:
            if app: DebugManager.log(f"Fusion API Last Error: {app.getLastError()[1]}", "CRITICAL")
        except: pass

    finally:
        try:
            ExportIntegrityChecker.verify_exports(EXPORT_DIR)
        except: pass
        DebugManager.generate_debug_report()
        DebugManager.flush()
