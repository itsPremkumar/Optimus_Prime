# Optimus Prime G1 — Fusion 360 Simulation Commands

## Overview

This document lists all commands to run, test, and manage the Optimus Prime G1 robot simulation inside Autodesk Fusion 360. All commands are executed from the project root (`Optimus_Prime/`).

---

## 1. Main Simulation Commands

### 1.1 Run all 9 simulation modules (full suite)

```
python src/run_simulation.py
```

- **Function:** Builds the entire Optimus Prime G1 model in Fusion 360, runs all 9 simulation modules sequentially (ROM test, head scan, wave, breathing, walking, running, combat, transformation, stability/loads). Duration: ~2–3 minutes.
- **Scenario:** Default / full workflow — comprehensive test of the robot from build to all animations.
- **Prerequisites:** Fusion 360 installed. On first run, the script auto-launches Fusion 360 if not running. MCP server must eventually be available.

---

### 1.2 Run a single simulation module

```
python src/run_simulation.py --module <module_name>
```

- **Function:** Runs only one specific simulation module instead of all 9.
- **Scenario:** Testing or developing individual modules without running the full suite.

**Available modules (all commands):**

```
python src/run_simulation.py --module ALL         # Run all 9 modules (default)
python src/run_simulation.py --module rom          # Joint ROM (range of motion) test
python src/run_simulation.py --module head         # Head look-around scan
python src/run_simulation.py --module wave         # Right-arm wave gesture
python src/run_simulation.py --module breathing    # Idle breathing torso oscillation
python src/run_simulation.py --module walk         # 4-cycle advanced walking gait
python src/run_simulation.py --module run          # 3-cycle running gait
python src/run_simulation.py --module combat       # Combat sequence (punch, aim, block)
python src/run_simulation.py --module transform    # Full robot-to-truck-to-robot transformation
python src/run_simulation.py --module truck        # Transform to truck mode only (no reverse)
python src/run_simulation.py --module robot        # Reset to standing robot pose
python src/run_simulation.py --module stability    # Centre-of-mass stability analysis
python src/run_simulation.py --module servo        # Servo torque load estimation
python src/run_simulation.py --module visual       # Visual audit (robot/truck/battle screenshots)
```

| Module | Description | Duration |
|--------|-------------|----------|
| `ALL` | All 9 modules (default) | ~2–3 min |
| `rom` | Joint ROM (range of motion) test | ~30s |
| `head` | Head look-around scan | ~8s |
| `wave` | Right-arm wave gesture | ~10s |
| `breathing` | Idle breathing torso oscillation | ~12s |
| `walk` | 4-cycle advanced walking gait | ~20s |
| `run` | 3-cycle running gait | ~15s |
| `combat` | Combat sequence (punch, aim, block) | ~12s |
| `transform` | Full robot-to-truck-to-robot transformation | ~30s |
| `truck` | Transform to truck mode only (no reverse) | ~15s |
| `robot` | Reset to standing robot pose | ~5s |
| `stability` | Centre-of-mass stability analysis | ~5s |
| `servo` | Servo torque load estimation | ~5s |
| `visual` | Visual audit (robot/truck/battle screenshots) | ~10s |

---

### 1.3 Run with screenshot capture

```
python src/run_simulation.py --capture
```

- **Function:** After simulation completes, captures 6 multi-angle viewport screenshots (Front, Back, Left, Right, Top, Isometric) and saves to `output/screenshots/`.
- **Scenario:** Producing visual documentation or model renders after simulation.

---

### 1.4 Run a specific module with capture

```
python src/run_simulation.py --module robot --capture
python src/run_simulation.py --module truck --capture
```

- **Function:** Builds the model and captures screenshots in a specific pose.
- **Scenario:** Quick visual verification of the model in robot or truck mode.

---

### 1.5 Run with export flags (full production pipeline)

```
python src/run_simulation.py --module walk --export-stl --export-step --capture
```

- **Function:** Builds the model, runs the specified module, exports STL files for 3D printing, exports a STEP assembly file, and captures screenshots.
- **Scenario:** Full production pipeline — build, simulate, and export for 3D printing.
- **Additional flags:**

| Flag | Description |
|------|-------------|
| `--export-stl` | Export all printable bodies as STL files |
| `--export-step` | Export full assembly as STEP file |
| `--export-urdf` | Export kinematic skeleton as URDF file |
| `--visual-audit` | Run visual audit (robot/truck/battle screenshots) |
| `--no-production-report` | Skip production readiness report |

---

### 1.6 Custom output directory

```
python src/run_simulation.py --output-dir "C:\custom\path"
```

- **Function:** Sets a custom root directory for exports, logs, and screenshots. Defaults to `output/`.
- **Scenario:** Organizing multiple builds into separate folders, or redirecting output to a shared network drive.

---

### 1.7 Stop a running simulation (manual method)

```
echo $null >> output\stop.flag
```

or in PowerShell:
```powershell
New-Item -Path output\stop.flag -ItemType File -Force
```

- **Function:** The simulation engine in Fusion 360 checks for `output/stop.flag` every frame. When detected, it exits cleanly.
- **Scenario:** Aborting a long-running simulation mid-way from a second terminal window.
- **Note:** This is NOT a CLI argument — it is a file-based flag. `--stop` is not a valid argparse flag.

---

### 1.8 Custom MCP server URL

```
python src/run_simulation.py --mcp-url http://192.168.1.100:27182/mcp
```

- **Function:** Connects to a Fusion 360 MCP server on a different host/port.
- **Scenario:** Remote/networked Fusion 360 instances, or custom port configurations.
- **Alternative:** Set environment variable `$env:MCP_URL = "http://127.0.0.1:27182/mcp"`

---

### 1.9 Skip auto-launch of Fusion 360

```
python src/run_simulation.py --module robot --no-launch
```

- **Function:** Does not attempt to auto-launch Fusion 360.
- **Scenario:** When Fusion 360 is already running manually; speeds up development/debugging.

---

### 1.10 Keep existing documents open

```
python src/run_simulation.py --keep-docs
```

- **Function:** Does not close existing open documents in Fusion 360 before building.
- **Scenario:** When you have other work open and do not want it closed.

---

## 2. Pipeline Commands

### 2.1 Run build pipeline

```
python src/pipeline.py
```

- **Function:** Reads `src/config.json`, runs simulation with configured module and flags, captures screenshots, validates outputs (STL/STEP/URDF existence, log errors, screenshots, BOM), creates versioned output folders (`output/v001/`, `v002/`, etc.) with manifest + fitness report.
- **Scenario:** Production build pipeline for version-controlled output with validation.

### 2.2 Run pipeline with custom config

```
python src/pipeline.py --config my_config.json
```

- **Function:** Uses a custom JSON configuration file instead of the default `src/config.json`.
- **Scenario:** Different build configurations without modifying the default config file.

### 2.3 Config.json fields

```json
{
  "module": "ALL",
  "export_stl": true,
  "export_step": true,
  "export_urdf": false,
  "capture_screenshots": true,
  "visual_audit": false,
  "production_report": true,
  "post_capture_views": false,
  "versioned_outputs": true
}
```

---

## 3. Screenshot Capture Commands

### 3.1 Standalone screenshot capture

```
python src/capture_optimus.py
```

- **Function:** Connects to Fusion 360 MCP and captures 18+ screenshots: 6 orthographic views (Front, Back, Left, Right, Top, Bottom), 4 isometric views (IsoTopRight, IsoTopLeft, IsoBottomRight, IsoBottomLeft), 8 turntable angles (0°–315°), 4 elevated angles (0°, 90°, 180°, 270° at 30° elevation). Saved as 1920×1080 PNGs.
- **Scenario:** After a simulation has built the model, get comprehensive documentation shots.
- **Prerequisites:** Fusion 360 must be running with MCP server active, and a model must be open.

---

## 4. Ball Launcher Commands (Separate Sub-Project)

### 4.1 Build ball launcher

```
python ball_launcher/ball_launcher_ultimate.py
```

- **Function:** Builds a complete 3D-printable ball launcher in Fusion 360 with cam drive, plunger, spring, magazine, barrel, motor mount, and all 3D-printing features. Includes BOM, 4 screenshots, STEP export, print recommendations.
- **Scenario:** Generate the ball launcher CAD model and export files.
- **Prerequisites:** Fusion 360 running with MCP on port 27182.

### 4.2 Run ball launcher dynamic simulation

```
python ball_launcher/ball_launcher_dynamic.py
```

- **Function:** Advanced dynamic simulation of the ball launcher with real-time firing cycle (cam-driven plunger + spring), gravity ball feed, muzzle flash, spring compression visual, ammo counter, assembly/disassembly animation.
- **Scenario:** See the ball launcher in animated action.
- **Prerequisites:** Fusion 360 running with MCP on port 27182.

---

## 5. Fusion 360 MCP Server Setup (Prerequisite to all commands)

Before any simulation commands work, the MCP server must be running in Fusion 360.

### 5.1 Launch via UI

1. Open Fusion 360
2. Go to **Tools** → **Scripts and Add-Ins** (or press `Shift+S`)
3. Select **MCP Server** → Click **Run**

### 5.2 Launch via command line

```powershell
& "C:\Program Files\Autodesk\Fusion 360\FusionLauncher.exe" --mcp
```

- **Function:** Launches Fusion 360 with the MCP server running immediately.
- **Scenario:** Alternative to UI method; can be scripted.

### 5.3 Verify MCP is running

Open browser to: `http://127.0.0.1:27182/mcp`

You should see a JSON-RPC response.

---

## 6. Environment Setup Commands

### 6.1 Set custom Fusion 360 path

```powershell
$env:FUSION_EXE = "C:\Custom\Path\FusionLauncher.exe"
```

- **Scenario:** Fusion 360 is installed in a non-standard location and auto-detection fails.

### 6.2 Set custom MCP URL

```powershell
$env:MCP_URL = "http://127.0.0.1:27182/mcp"
```

- **Scenario:** Alternative to `--mcp-url` CLI argument.

### 6.3 Clone the repository

```
git clone https://github.com/itsPremkumar/Optimus_Prime.git
cd Optimus_Prime
```

---

## 7. Ad-hoc Script Execution via MCP

```python
from run_simulation import call_tool

with open("path/to/probe.py") as f:
    script = f.read()

result = call_tool("fusion_mcp_execute", {
    "featureType": "script",
    "object": {"script": script}
})
print(result)
```

- **Function:** Sends any Python script to Fusion 360 for execution via MCP.
- **Scenario:** Ad-hoc testing/debugging scripts in the Fusion 360 environment.

---

## Quick Reference Table

| # | Command | Purpose |
|---|---------|---------|
| 1 | `python src/run_simulation.py` | Run all 9 simulation modules |
| 2 | `python src/run_simulation.py --module <name>` | Run a single simulation module (13 choices) |
| 3 | `python src/run_simulation.py --capture` | Run with screenshot capture |
| 4 | `python src/run_simulation.py --output-dir <path>` | Set custom output directory |
| 5 | `python src/run_simulation.py --export-stl --export-step` | Export STL/STEP for 3D printing |
| 6 | `python src/run_simulation.py --mcp-url <url>` | Connect to custom MCP server |
| 7 | `python src/run_simulation.py --no-launch` | Skip auto-launch of Fusion 360 |
| 8 | `python src/run_simulation.py --keep-docs` | Keep existing documents open |
| 9 | `python src/pipeline.py` | Run build pipeline with config.json |
| 10 | `python src/pipeline.py --config <file>` | Pipeline with custom config |
| 11 | `python src/capture_optimus.py` | Standalone screenshot capture (18+ views) |
| 12 | `python ball_launcher/ball_launcher_ultimate.py` | Build ball launcher model |
| 13 | `python ball_launcher/ball_launcher_dynamic.py` | Ball launcher animated simulation |
| 14 | `New-Item -Path output\stop.flag -ItemType File -Force` | Stop running simulation mid-way |
