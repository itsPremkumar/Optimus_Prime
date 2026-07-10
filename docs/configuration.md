# Configuration Guide

All configuration is defined at the top of `src/optimus_v17.py` (the CONFIGURATION section). Values can be overridden when running via MCP (set by `run_simulation.py`).

---

## Global Flags

These flags control what the script does after building the model:

| Flag | Default | Description |
|------|---------|-------------|
| `TARGET_MODULE` | `"ALL"` | Which simulation module to run (see [Usage](usage.md)) |
| `EXPORT_STL` | `False` | Batch-export all printable bodies as `.stl` files |
| `EXPORT_STEP` | `False` | Export full assembly as a single `.step` file |
| `EXPORT_URDF` | `False` | Export kinematic skeleton as `robot.urdf` |
| `CAPTURE_SCREENSHOTS` | `False` | Capture 6 viewport renders (Front, Back, Left, Right, Top, Iso) |

Override via `run_simulation.py`:

```bash
python src/run_simulation.py --module walk --capture
```

Or by editing the global variables at the top of the script.

---

## Clearance & Tolerances

```python
CLEARANCE = 0.060   # 0.60 mm — FDM 3D printing tolerance
```

- Applied to all servo cavities, bearing seats, wheel gearbox housings
- Increased from 0.45 mm (v7) for better FDM fit
- Units: model units (1 unit = 10 mm = 1 cm)

---

## Dimensional Constants (Z-axis Layout)

All positions are in **model units** (1 unit ≈ 1 cm). These define the vertical skeleton:

| Constant | Value | Description |
|----------|-------|-------------|
| `ANKLE_CTR` | 3.8 | Ankle pivot Z |
| `SHIN_CTR` | 9.3 | Shin midpoint Z |
| `KNEE_CTR` | 14.8 | Knee pivot Z |
| `THIGH_CTR` | 20.3 | Thigh midpoint Z |
| `PELVIS_CTR` | 30.5 | Pelvis centre Z |
| `WAIST_CTR` | 32.5 | Waist pivot Z |
| `TORSO_CTR` | 36.0 | Torso centre Z |
| `SHOULDER_CTR` | 41.5 | Shoulder pivot Z |
| `HEAD_CTR` | 47.5 | Head centre Z |
| `HIP_X` | 5.8 | Hip lateral offset from centreline |
| `SHOULDER_X` | 13.0 | Shoulder lateral offset |
| `ELBOW_Z` | 35.0 | Elbow pivot Z |
| `WRIST_Z` | 29.0 | Wrist pivot Z |
| `HIP_JOINT_Z` | 26.8 | Hip joint Z |
| `NECK_JOINT_Z` | 44.5 | Neck joint Z |

To resize the robot, scale these constants proportionally.

---

## Servo Specifications (`SERVO_SPECS`)

```python
SERVO_SPECS = {
    "hip":   {"name": "MG996R-HD", "rated": 20.0, "stall": 25.0},
    "waist": {"name": "MG996R-HD", "rated": 25.0, "stall": 30.0},
    "std":   {"name": "MG996R",    "rated":  9.4,  "stall": 11.5},
    "micro": {"name": "MG90S",     "rated":  1.8,  "stall":  2.2},
}
```

| Key | Model | Rated (kg·cm) | Stall (kg·cm) | Used In |
|-----|-------|:-------------:|:-------------:|---------|
| `hip` | MG996R-HD | 20.0 | 25.0 | Hip joints |
| `waist` | MG996R-HD | 25.0 | 30.0 | Waist pitch/yaw |
| `std` | MG996R | 9.4 | 11.5 | Shoulders, elbows, knees, ankles |
| `micro` | MG90S | 1.8 | 2.2 | Neck yaw, wrists, roof hinge, steer |

---

## Joint Limits (`JOINT_LIMITS`)

Defined in degrees. Used for ROM testing and transformation validation:

```python
JOINT_LIMITS = {
    "Waist_Cluster":      {"pitch": (-45,  60),  "yaw": (-15,  15),  "roll": (-15,  15)},
    "Neck_Cluster":       {"pitch": (-90,  45),  "yaw": (-20,  20),  "roll": (-20,  20)},
    "L_Hip_Cluster":      {"pitch": (-30,  30),  "yaw": (-95,  95),  "roll": (-30,  30)},
    "R_Hip_Cluster":      {"pitch": (-30,  30),  "yaw": (-95,  95),  "roll": (-30,  30)},
    "L_Knee":             {"pitch": (  0, 135)},
    "R_Knee":             {"pitch": (  0, 135)},
    "L_Ankle_Cluster":    {"pitch": (-20,  20),  "yaw": (-30,  95),  "roll": (-20,  20)},
    "R_Ankle_Cluster":    {"pitch": (-20,  20),  "yaw": (-30,  95),  "roll": (-20,  20)},
    "L_Shoulder_Cluster": {"pitch": (-175, 60),  "yaw": (-90,  90),  "roll": (-90,  90)},
    "R_Shoulder_Cluster": {"pitch": (-175, 60),  "yaw": (-90,  90),  "roll": (-90,  90)},
    "L_Elbow":            {"pitch": (  0, 150)},
    "R_Elbow":            {"pitch": (  0, 150)},
    "L_Wrist":            {"pitch": (  0,  90),  "roll": (-180, 180)},
    "R_Wrist":            {"pitch": (  0, 135),  "roll": (-180, 180)},
    "Blaster_Fold":       {"pitch": (-90,   0)},
}
```

**Axis mapping notes:**
- Waist/Neck: pitch = forward lean, yaw = twist
- Hip/Ankle: yaw = forward fold, pitch = twist
- Shoulder: yaw = side-swing, pitch = tilt, roll = twist
- Knee/Elbow: pitch = bend angle (0 = straight)

---

## Export Filters

```python
SPLIT_KEYS = {"Shell", "Link", "Main", "Armor", "Core", "Pod", "Palm", "Block", "Sole"}
SKIP_TAGS  = {"Marker", "Pivot", "MtA", "MtB", "Axle_Pivot", "Horn", "Pin", "_Vis"}
```

- **`SPLIT_KEYS`**: Body names containing these tags are split along the Y-axis midplane for FDM printing
- **`SKIP_TAGS`**: Bodies with these tags are excluded from STL export (reference geometry only)

---

## Output Directories

| Directory | Contents |
|-----------|----------|
| `output/logs/` | Timestamped execution logs (`optimus_fusion_log_*.txt`) |
| `output/exports/` | STL files, `robot_v17.urdf`, `Optimus_Prime_G1_v17.f3d`, `Optimus_Prime_G1_v17.step` |
| `output/screenshots/` | 1920×1080 PNG screenshots |
