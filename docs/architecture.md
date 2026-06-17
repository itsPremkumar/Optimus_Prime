# Architecture Overview

## High-Level Structure

The main script (`src/optimus_prime_g1_v9.py`) follows a linear pipeline with nested building blocks:

```
run(context)                  # Entry point (called by Fusion 360)
├── Setup                     # Logger, appearances, constants
├── Component Builders        # Primitives (box, cyl, cone) + utilities
├── Joint Builders            # revolute_joint, ball_joint, rigid_joint
├── Mechanical Modules        # Servo, bearing, wheel, bracket hardware
├── Body Components           # Torso → Head → Pelvis → Legs → Arms → etc.
├── Kinematic Validation      # Joint tree check, orphan detection
├── Archive Save              # Save as .f3d
├── SimulationEngine          # 9-module animation engine
│   ├── run_all_simulations()
│   └── Export (STL/STEP/URDF) + Screenshots
└── Logging & Cleanup
```

---

## Execution Flow

### 1. Entry Point: `run(context)`

Called by Fusion 360's MCP executor with a `context` object. The function:

1. Initializes the Fusion 360 API (`adsk.core.Application.get()`)
2. Creates the output directory structure
3. Sets up material appearances
4. Registers primitive builder functions inside the root component scope
5. Calls each component builder in sequence
6. Applies all joints
7. Validates the kinematic chain
8. Saves an archive (`.f3d`)
9. Creates a `SimulationEngine` and calls `run_all_simulations()`

### 2. Component Builders

These functions operate inside a root component and use Fusion 360's `TemporaryBRepManager` to create geometry:

| Function | Purpose |
|----------|---------|
| `box(comp, name, cx, cy, cz, lx, ly, lz, ap)` | Rectangular box via `OrientedBoundingBox3D` |
| `cyl(comp, name, cx, cy, cz, r, h, axis, ap)` | Cylinder along X/Y/Z axis |
| `cone_shape(comp, name, cx, cy, cz, r1, r2, h, axis, ap)` | Tapered cylinder |
| `marker(comp, name, cx, cy, cz, size)` | Small reference cube (white) |
| `cut_cavity(comp, tool_body, keep_tool)` | Boolean cut operation |
| `split_halves(comp, body, axis, offset)` | Split body for FDM printing |
| `screw_hole(comp, cx, cy, cz, axis, length)` | M3 screw clearance hole |
| `magnet_pocket(comp, tag, cx, cy, cz, axis)` | Ø6.4×3.5mm magnet cavity |
| `wire_channel(comp, tag, cx, cy, cz, r, h, axis)` | Cable routing tunnel |

### 3. Mechanical Modules

| Function | Purpose |
|----------|---------|
| `mg996r(comp, tag, cx, cy, cz, axis)` | MG996R servo (visual + cavity + hardware) |
| `mg90s(comp, tag, cx, cy, cz, axis)` | MG90S micro servo (visual + cavity + hardware) |
| `servo_hardware(comp, tag, cx, cy, cz, axis, mg996)` | Flange/horn screw holes for servos |
| `tt_wheel(comp, tag, cx, cy, cz, side)` | TT gear motor + wheel assembly |
| `bearing(comp, tag, cx, cy, cz, axis, ro, w)` | Rolling bearing (outer ring, inner ring, balls) |
| `u_bracket(comp, tag, cx, cy, cz, lx, ly, lz, ap)` | U-shaped mounting bracket |

### 4. Body Components

19 top-level assemblies are built in sequence (see [Model Overview](../README.md#model-overview)):

```
OP_Torso (42 bodies)
├── OP_Head (15 bodies)
├── OP_Backpack (8 bodies)
├── OP_Shields (8 bodies)
├── OP_UpperArm_L/R (~13 bodies each)
│   └── OP_Forearm_L/R (~6 bodies each)
│       └── OP_Hand_L/R (~4 bodies each)
│           └── OP_Ion_Blaster (6 bodies, right only)
OP_Pelvis (7 bodies)
├── OP_Thigh_L/R (~13 bodies each)
│   └── OP_Shin_L/R (~10 bodies each)
│       └── OP_Foot_L/R (~8 bodies each)
├── OP_SteerPods (~7 bodies)
```

### 5. Joint Application

After all components are built, joints are created using:

- **`revolute_joint(name, occ1, occ2, cx, cy, cz, axis_str)`** — 1-DOF hinge
- **`ball_joint(name, occ1, occ2, cx, cy, cz)`** — 3-DOF spherical joint
- **`rigid_joint(name, occ1, occ2)`** — Fixed connection

Joint geometry points are created via `_make_joint_geometry()` which uses `constructionPoints` (preferred) or falls back to sketch points for MCP compatibility.

### 6. Kinematic Validation

After all joints are created, the script:
- Iterates `root.asBuiltJoints` to map every component in the joint chain
- Cross-references against the full component list
- Logs warnings for any orphan components (not connected by any joint)

---

## SimulationEngine Class

The `SimulationEngine` (line 822) orchestrates all animation modules.

### Key Properties

- `BALL_JOINTS` — set of 10 ball joint names (3-DOF)
- `REV_JOINTS` — set of 5 revolute joint names (1-DOF)
- `_joint_cache` — cached joint references

### Key Methods

| Method | Purpose |
|--------|---------|
| `move_joint(name, deg, steps, axis, ease, clamp)` | Animate a single joint axis |
| `move_ball(targets, steps, clamp)` | Animate multiple ball joints (pitch/yaw/roll) |
| `move_group(targets, steps, axis, ease, clamp)` | Animate multiple joints on same axis |
| `reset_all(steps, groups)` | Reset joints to 0° |
| `_interfere(label)` | Run collision detection |
| `debug_joints(label)` | Log all joint states |
| `_clamp(joint_name, axis, deg)` | Clamp angle to ROM limits |
| `_ease(t)` | Smooth-step easing function |

### Animation Pipeline

```
move_ball([(joint, pitch, yaw, roll), ...], steps=20)
  └── For each step (0 → steps):
      ├── Apply eased interpolation
      ├── Set joint angles via _set()
      ├── Call _refresh() (viewport update + stop check)
      └── Next step
```

---

## Export Pipeline

After simulation, exports are handled by:

| Function | Format | Content |
|----------|--------|---------|
| `export_stl()` | `.stl` (binary) | Each printable body as a separate file |
| `export_step()` | `.step` (AP214) | Full assembly + per-component files |
| `export_urdf()` | `.urdf` (XML) | Joint skeleton with parent/child links |
| `capture_screenshots()` | `.png` (1920×1080) | 6 orthographic views |

All exports are controlled by global flags (`EXPORT_STL`, `EXPORT_STEP`, `EXPORT_URDF`, `CAPTURE_SCREENSHOTS`).

---

## MCP Communication

```
┌─────────────────┐  HTTP POST (JSON-RPC 2.0)  ┌────────────────┐
│  run_simulation  │ ─────────────────────────▶  │  Fusion 360   │
│  (Python CLI)    │                              │  MCP Server   │
│                  │ ◀─────────────────────────  │  (127.0.0.1)  │
└─────────────────┘   Script result + logs       └───────┬────────┘
                                                         │
                                                  ┌──────▼────────┐
                                                  │  adsk.core /  │
                                                  │  adsk.fusion  │
                                                  │  (Python API) │
                                                  └───────────────┘
```

1. CLI sends `initialize` → receives session ID
2. Sends `fusion_mcp_execute` with the full script as payload
3. Fusion executes the script using its embedded Python interpreter
4. Script output (log text) is returned in the MCP response
