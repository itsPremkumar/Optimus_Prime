# Optimus Prime G1 — Full Simulation Engine v6.0

An Autodesk Fusion 360 simulation suite that programmatically builds a fully-articulated, 3D-printable **Optimus Prime G1** robot and runs a 9-module kinematic simulation, all driven remotely via the **Model Context Protocol (MCP)**.

![Optimus Prime](images/Screenshot%202026-06-15%20230823.png)

---

## Requirements

- **Autodesk Fusion 360** with an MCP server running on `http://127.0.0.1:27182/mcp`
- **Python 3.8+** (standard library only — no extra packages needed)

---

## Project Structure

```
Optimus_Prime/
├── optimus_prime_simulation_v6.py   # Main Fusion 360 script (model + simulation engine)
├── run_simulation.py                # CLI controller — sends the script to Fusion 360
├── capture_optimus.py               # Captures multi-angle viewport screenshots
├── api_test.py                      # Dev utility to query Fusion 360 API docs via MCP
└── images/                          # Saved viewport screenshots
```

---

## How It Works

`run_simulation.py` connects to the Fusion 360 MCP server and transmits `optimus_prime_simulation_v6.py` as a script payload. Fusion 360 executes it internally, which:

1. Builds the full 3D model (torso, head, pelvis, arms, legs, backpack, ion blaster)
2. Applies materials and appearances (red/blue metallic paint, chrome, rubber, glass)
3. Creates all joints (revolute, ball, rigid) and validates the kinematic chain
4. Runs the 9-module simulation suite
5. Writes a full report to `C:\opt_fusion_log.txt` and displays a summary dialog

---

## Running the Simulation

### Full simulation (all 9 modules)
```bash
python run_simulation.py
```

### Single module
```bash
python run_simulation.py --module walk
```

Available modules: `ALL`, `rom`, `head`, `wave`, `breathing`, `walk`, `run`, `combat`, `transform`, `stability`, `servo`

### Stop a running simulation
```bash
python run_simulation.py --stop
```
This creates `C:\opt_fusion_stop.flag`, which the running script detects and aborts cleanly within one animation frame.

### Capture screenshots
```bash
python capture_optimus.py
```
Saves 6 viewport renders (Front, Back, Left, Right, Top, Isometric) to `images/`.

---

## Simulation Modules

| # | Module | Description |
|---|--------|-------------|
| 1 | Joint ROM Test | Sweeps every joint min→0→max, samples collisions at each extreme |
| 2 | Head Look-Around | 5-position scan (left, right, up, down, centre) |
| 3 | Wave Gesture | Full right-arm raise and 3× wrist wave |
| 4 | Idle Breathing | 4-cycle subtle torso oscillation |
| 5 | Advanced Walking | 4 cycles with hip sway, arm counter-swing, ankle push-off |
| 6 | Running | 3 cycles, exaggerated fast gait |
| 7 | Combat Sequence | Right cross → blaster aim → forearm block → left uppercut |
| 8 | Transformation | Robot → Truck (9 stages) + driving + reverse transformation |
| 9 | Stability + Loads | CoM check for 4 poses + static servo torque table |

---

## Model Overview

### Body Components
| Component | Key Parts |
|-----------|-----------|
| Torso | Chest windows, grille, headlights, battery bay, spine beam |
| Head | Helmet, visor, faceplate, antennas, neck yaw servo |
| Pelvis | Hip armour, hip yaw servos |
| Thighs (L/R) | Hip pitch/roll servos, knee servos, bearings |
| Shins (L/R) | TT-gear drive wheels (×4), bearings |
| Feet (L/R) | Ankle pitch/roll servos, heel and toe blocks |
| Upper Arms (L/R) | Shoulder pitch/yaw/roll servos, elbow servo, exhaust stacks |
| Forearms (L/R) | Wrist servo |
| Hands (L/R) | Palm, fingers, thumb panels |
| Ion Blaster (R) | Barrel, scope, body (right hand only) |
| Backpack | Radiator, exhausts, roof hinge servo |
| Steer Pods | Front steering wheels (×2) with servos |
| Shields/Panels | Shoulder shields, hip shields, mirrors |

### Servo Hardware
- **MG996R** (9.4 kg·cm) — waist, neck pitch, hips, knees, ankles, shoulders, elbows
- **MG90S** (1.8 kg·cm) — neck yaw, wrists, backpack hinge, steering servos

### Joint Types
- **Ball joints** — waist, neck, hips, ankles, shoulders
- **Revolute joints** — knees, elbows, wrists
- **Rigid joints** — backpack, steer pods, shields, ion blaster

---

## Output Files

| File | Description |
|------|-------------|
| `C:\opt_fusion_log.txt` | Timestamped execution log with all module results and collision details |
| `C:\OptimusPrime_STL\robot.urdf` | Minimal URDF skeleton for robotics toolchain import |
| `images/*.png` | Viewport screenshots (1920×1080) from `capture_optimus.py` |

> **STL batch export** is available but commented out at the bottom of `optimus_prime_simulation_v6.py`. Uncomment to export every printable body to `C:\OptimusPrime_STL\`.

---

## 3D Printing Notes

- Clearance on all moving fits: **0.45 mm**
- All major shells are split along the Y-axis midplane for FDM printing
- Bodies tagged with `Shell`, `Link`, `Main`, `Armor`, `Core`, `Pod`, `Palm`, `Block`, or `Sole` are automatically halved
- Screw holes (M3), magnet pockets (Ø6.4 × 3.5 mm), and wire channels are pre-cut into the geometry
- Apply shrinkage compensation in your slicer before printing
