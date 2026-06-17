<!--
  ============================================================
  OPTIMUS PRIME G1 — Fusion 360 Simulation Engine
  Description: Open-source Python script that builds a 3D-printable
    Optimus Prime G1 robot in Autodesk Fusion 360 and runs 9
    kinematic simulation modules via MCP protocol.
  Keywords: Optimus Prime, Transformers, Fusion 360, MCP, 3D printing,
    robotics simulation, G1, kinematic simulation, Python, CAD,
    STL export, URDF, Model Context Protocol, FDM printing
  Author: itsPremkumar
  Repository: https://github.com/itsPremkumar/Optimus_Prime
  License: MIT
  ============================================================
-->

# Optimus Prime G1 — Full Simulation Engine v9.0

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Python](https://img.shields.io/badge/Python-3.8%2B-blue)](https://python.org)
[![Fusion 360](https://img.shields.io/badge/Fusion%20360-MCP-orange)](https://www.autodesk.com/products/fusion-360)
[![PRs Welcome](https://img.shields.io/badge/PRs-welcome-brightgreen.svg)](CONTRIBUTING.md)
[![GitHub Repo](https://img.shields.io/badge/GitHub-itsPremkumar%2FOptimus__Prime-181717?logo=github)](https://github.com/itsPremkumar/Optimus_Prime)
[![Conventional Commits](https://img.shields.io/badge/Conventional%20Commits-1.0.0-blueviolet)](https://www.conventionalcommits.org)

An open-source **Autodesk Fusion 360 simulation suite** that programmatically builds a fully-articulated, **3D-printable Optimus Prime G1** robot and runs a **9-module kinematic simulation** — all driven remotely via the **Model Context Protocol (MCP)**.

![Optimus Prime G1 3D model rendered in Fusion 360](images/Screenshot%202026-06-15%20230823.png)

---

## What Is This?

This project is a **Python script** (`src/optimus_prime_g1_v9.py`) that connects to Autodesk Fusion 360 through its **MCP server** and automatically:

1. **Builds** a complete Optimus Prime G1 3D model with 100+ components
2. **Applies** materials (red/blue metallic paint, chrome, rubber, glass)
3. **Creates** all joints (revolute, ball, rigid) and validates the kinematic chain
4. **Runs** 9 simulation modules covering full-body motion
5. **Exports** STL files and a URDF skeleton for 3D printing and robotics toolchains

Zero external Python dependencies — uses only the **standard library**.

---

## Features

- **Full 3D model** — 100+ body components (torso, head, pelvis, arms, legs, backpack, ion blaster)
- **9 simulation modules** — ROM test, head scan, wave, breathing, walking, running, combat, transformation, stability
- **MCP-driven** — remote control via Fusion 360's Model Context Protocol
- **3D-printable** — 0.60 mm clearance, Y-axis midplane shell splitting, M3 screw holes, magnet pockets
- **Zero dependencies** — Python standard library only
- **CLI control** — run single modules, stop mid-simulation, capture screenshots

---

## Requirements

- **Autodesk Fusion 360** with MCP server running on `http://127.0.0.1:27182/mcp`
- **Python 3.8+** (standard library only — no extra packages needed)

---

## Quick Start

```bash
# Clone the repo
git clone https://github.com/itsPremkumar/Optimus_Prime.git
cd Optimus_Prime

# Full simulation (all 9 modules)
python src/run_simulation.py

# Single module
python src/run_simulation.py --module walk

# Stop a running simulation
python src/run_simulation.py --stop
```

---

## Project Structure

```
Optimus_Prime/
├── src/                           # Source code
│   ├── optimus_prime_g1_v9.py     # Main Fusion 360 script (model + simulation engine)
│   ├── optimus_prime_g1_v8.py     # Previous version (v8 — reference only)
│   ├── optimus_prime_g1_v7.py     # Previous version (v7 — reference only)
│   ├── run_simulation.py          # CLI controller — sends the script to Fusion 360
│   ├── capture_optimus.py         # Multi-angle viewport screenshot capture
│   ├── analyze_bugs.py            # Post-simulation collision and bug analysis
│   └── api_test.py                # Dev utility to query Fusion 360 API
├── archive/                       # Archived legacy versions
│   └── optimus_prime_simulation_v6.py
├── images/                        # Saved viewport screenshots
├── .github/                       # GitHub issue/PR templates
├── CHANGELOG.md                   # Version history
├── CODE_OF_CONDUCT.md             # Community standards
├── CONTRIBUTING.md                # Contribution guidelines
├── LICENSE                        # MIT License
├── README.md                      # Project overview and usage
├── SECURITY.md                    # Security policy
└── .gitignore
```

### Simulation Modules

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

### Capture Screenshots

```bash
python src/capture_optimus.py
```

Saves 6 viewport renders (Front, Back, Left, Right, Top, Isometric) to `images/`.

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

- **MG996R-HD** (20.0 kg·cm) — hips, waist (high-duty)
- **MG996R** (9.4 kg·cm) — neck pitch, knees, ankles, shoulders, elbows
- **MG90S** (1.8 kg·cm) — neck yaw, wrists, backpack hinge, steering servos

### Joint Types

- **Ball joints** — waist, neck, hips, ankles, shoulders
- **Revolute joints** — knees, elbows, wrists
- **Rigid joints** — backpack, steer pods, shields, ion blaster

---

## Output Files

| File | Description |
|------|-------------|
| `output/logs/optimus_fusion_log_*.txt` | Timestamped execution log with all module results and collision details |
| `output/exports/robot.urdf` | Minimal URDF skeleton for robotics toolchain import |
| `output/exports/Optimus_Prime_G1_v9.f3d` | Fusion 360 archive of the full model |
| `output/screenshots/*.png` | Viewport screenshots (1920×1080) from `capture_optimus.py` |

> **STL batch export** is controlled via `EXPORT_STL = True/False` flag at the top of `src/optimus_prime_g1_v9.py`.

---

## 3D Printing Notes

- Clearance on all moving fits: **0.60 mm**
- All major shells are split along the Y-axis midplane for FDM printing
- Bodies tagged with `Shell`, `Link`, `Main`, `Armor`, `Core`, `Pod`, `Palm`, `Block`, or `Sole` are automatically halved
- Screw holes (M3), magnet pockets (Ø6.4 × 3.5 mm), and wire channels are pre-cut into the geometry
- Apply shrinkage compensation in your slicer before printing

---

## Frequently Asked Questions

### What is Optimus Prime G1?
Optimus Prime is the iconic leader of the Autobots from the Transformers franchise. The "G1" refers to the **Generation 1** design from the 1980s — the classic red-and-blue truck form.

### Does this work without Fusion 360?
No. This script runs **inside Autodesk Fusion 360** via its MCP server. It is not a standalone simulation.

### Can I 3D print the robot?
Yes. The model is designed for **FDM 3D printing** with 0.60 mm clearance on all moving fits, shell splitting along the midplane, M3 screw holes, and magnet pockets (Ø6.4 × 3.5 mm).

### What Python packages are required?
**None.** The project uses only Python's **standard library** (`urllib`, `json`, `os`, `argparse`). The Fusion 360 script uses the `adsk` API which is built into Fusion 360.

### How do I run only one simulation module?
Use `--module` flag: `python run_simulation.py --module walk`

### What is MCP?
**Model Context Protocol** (MCP) is a protocol that allows external applications to communicate with Fusion 360 remotely. This project uses Fusion 360's built-in MCP server at `http://127.0.0.1:27182/mcp`.

### How do I stop a simulation mid-run?
Run `python run_simulation.py --stop` from another terminal. This creates a flag file that the simulation checks every frame.

---

## Contributing

We welcome contributions! See [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines.

Please adhere to the [Code of Conduct](CODE_OF_CONDUCT.md) in all interactions.

## License

This project is licensed under the MIT License — see the [LICENSE](LICENSE) file for details.

---

<div align="center">
  <sub>
    Built with Python &middot; Powered by Autodesk Fusion 360 &middot;
    <a href="https://github.com/itsPremkumar/Optimus_Prime">GitHub</a>
  </sub>
</div>




uv run src/run_simulation.py --module truck --capture