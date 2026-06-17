# Developer Documentation

## Code Style

- Follow **PEP 8** conventions
- Use **`snake_case`** for functions and variables
- Use **`CamelCase`** for classes
- Keep functions focused and single-purpose
- Add docstrings for all public functions
- Avoid external dependencies — the project uses only the Python standard library

### Naming Conventions

| Element | Convention | Example |
|---------|-----------|---------|
| Classes | PascalCase | `SimulationEngine`, `Logger` |
| Functions | snake_case | `ball_joint()`, `cut_cavity()` |
| Constants | UPPER_CASE | `CLEARANCE`, `HIP_X` |
| Global flags | UPPER_CASE | `TARGET_MODULE`, `EXPORT_STL` |

---

## Adding a New Component

To add a new body part to the robot, follow the pattern of existing components:

### 1. Create the builder function

```python
def build_my_part(comp, ...):
    body1 = box(comp, "MyPart_Shell", cx, cy, cz, lx, ly, lz, ap=op_red)
    body2 = cyl(comp, "MyPart_Pin", cx, cy, cz, r, h, axis, ap=chrome)
    marker(comp, "MyPart_Pivot", cx, cy, cz)
    # Cut cavities, add screw holes, etc.
```

### 2. Register in `run()`

Inside the `run(context)` function, add:

```python
new_component("OP_MyPart")
build_my_part(occs["OP_MyPart"])
```

### 3. Add joints

After all components are created, join it to the kinematic tree:

```python
rigid_joint("MyPart_Mount", occs["OP_Parent"], occs["OP_MyPart"])
```

### 4. Update exports

Add any SKIP_TAGS or SPLIT_KEYS as needed in the configuration section.

---

## Adding a New Joint

### 1. Define limits

Add an entry to `JOINT_LIMITS`:

```python
JOINT_LIMITS = {
    ...
    "My_Joint": {"pitch": (-45, 45), "yaw": (-10, 10), "roll": (-10, 10)},
}
```

### 2. Add to joint type sets

If it's a ball joint (3-DOF), add to `SimulationEngine.BALL_JOINTS`:

```python
BALL_JOINTS = {"Waist_Cluster", ..., "My_Joint"}
```

If revolute (1-DOF), add to `REV_JOINTS`:

```python
REV_JOINTS = {"L_Knee", ..., "My_Joint"}
```

### 3. Create the joint

```python
ball_joint("My_Joint", occs["OP_Parent"], occs["OP_MyPart"], cx, cy, cz)
```

---

## Adding a New Simulation Module

### 1. Write the method

In the `SimulationEngine` class:

```python
def simulate_my_module(self):
    """Description of what this module does."""
    self.reset_all()
    self.move_ball([
        ("L_Knee", 45, None, None),
        ("R_Knee", 45, None, None),
    ], steps=30)
    self._interfere("My Module")
```

### 2. Register in the dispatch dictionary

In `run_all_simulations()`:

```python
def run_all_simulations(self):
    modules = {
        "rom":      self.test_joint_rom,
        ...
        "mymodule": self.simulate_my_module,
    }
```

### 3. Add to TARGET_MODULE choices

Update the CLI choices in `run_simulation.py`:

```python
parser.add_argument("--module", default="ALL",
    choices=["ALL", "rom", ..., "mymodule"])
```

---

## Architecture: Joint Motion

### How `move_ball()` Works

1. Accepts a list of `(joint_name, pitch_deg, yaw_deg, roll_deg)` tuples
2. For each ball joint, creates an interpolation array over `steps` frames
3. Applies smooth-step easing (`_ease()`) to each frame
4. For each frame:
   - Sets each joint's pitch/yaw/roll via `_set()`
   - Calls `_refresh()` to update viewport and check stop flag
5. For revolute joints passed to `move_ball()`, only pitch is used (FIX 5)

### How `move_joint()` Works

1. Animates a single DOF on any joint type
2. Uses the `axis` parameter to select pitch/yaw/roll
3. Clamps target angle to ROM limits (`_clamp()`)
4. Interpolates from current angle to target

### Joint Angle Storage

Ball joints store angles in `JointMotion.xyzValue` as a `Vector3D` where:
- `x` = pitch
- `y` = yaw  
- `z` = roll

Revolute joints use `rotationValue` (scalar, radians).

---

## The Transformation System

Transformation is divided into 5 stages, each mapping to specific joint angles:

```python
_transform_to_truck(steps_scale=1.0)
├── Stage 1: Elbows (0°), Blaster (-90°), Wrists (fold)
├── Stage 2: Neck pitch (45°) — tuck head
├── Stage 3: Shoulder yaw (90°) — fold arms back
├── Stage 4: Hip yaw (95°) — rotate legs
└── Stage 5: Ankle yaw (95°) — feet flat
```

To add a new stage, insert it in the sequence:

```python
def _transform_to_truck(self, steps_scale=1.0):
    s = steps_scale
    self.move_ball([
        # ... existing stages ...
        ("My_Joint", pitch, yaw, roll),
    ], steps=int(20 * s))
```

The reverse transformation (`_transform_to_robot`) plays stages in opposite order.

---

## Export System

### Adding a New Export Format

1. Write the export method in `SimulationEngine`:

```python
def export_my_format(self):
    path = os.path.join(EXPORT_DIR, "output.myfmt")
    # ... export logic ...
    Logger.log(f"Exported: {path}")
```

2. Add a global flag:

```python
EXPORT_MYFORMAT = False  # Set to True to enable
```

3. Call it in `run_all_simulations()`:

```python
if EXPORT_MYFORMAT:
    self.export_my_format()
```

---

## Component Registry

The global `occs` dictionary maps component names to occurrences:

```python
occs = {
    "OP_Torso":      <Occurrence>,
    "OP_Head":       <Occurrence>,
    "OP_Pelvis":     <Occurrence>,
    "OP_Thigh_L":    <Occurrence>,
    "OP_Thigh_R":    <Occurrence>,
    ...
}
```

Access any component by name:

```python
torso = occs["OP_Torso"]
left_arm = occs["OP_UpperArm_L"]
```

The `comps_list` contains all component objects (not occurrences) for iteration.

---

## Version History

See `CHANGELOG.md` for the complete version history.

### v9.0.0 Key Changes

- **9 bugs fixed** from v8 (geometry, joints, export, interference)
- **STEP export** added (full assembly + per-component)
- **Joint geometry fix** — use `ConstructionPoint` instead of `SketchPoint`
- **Documentation overhaul** — comprehensive README and docs/ folder
- **MCP reliability** — embedded document closing, session handling
- **Servo load analysis** — torque margin table for all major joints
