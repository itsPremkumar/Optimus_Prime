# Usage Guide

## CLI Controller (`run_simulation.py`)

The main entry point for running simulations.

### Basic Commands

```bash
# Run all 9 simulation modules
python src/run_simulation.py

# Run a single module
python src/run_simulation.py --module walk

# Run with screenshot capture
python src/run_simulation.py --capture

# Run a specific module with capture
python src/run_simulation.py --module truck --capture

# Stop a running simulation
python src/run_simulation.py --stop
```

### CLI Options

| Option | Default | Description |
|--------|---------|-------------|
| `--module` | `ALL` | Module to run |
| `--capture` | off | Capture 6 multi-angle screenshots |
| `--mcp-url` | `http://127.0.0.1:27182/mcp` | Custom MCP server URL |
| `--no-launch` | off | Skip auto-launching Fusion 360 |
| `--keep-docs` | off | Keep existing documents open |
| `--stop` | off | Stop a running simulation via flag file |

### Available Modules

| Module Key | Description |
|------------|-------------|
| `ALL` | Run all 9 modules sequentially |
| `rom` | Joint ROM test — sweeps every joint to limits |
| `head` | Head look-around scan |
| `wave` | Right-arm wave gesture |
| `breathing` | Idle breathing torso oscillation |
| `walk` | 4-cycle advanced walking gait |
| `run` | 3-cycle running gait |
| `combat` | Combat sequence (punch, aim, block) |
| `transform` | Full transformation cycle (Robot→Truck→Robot) |
| `truck` | Transform to truck mode (no reverse) |
| `robot` | Reset to standing robot pose |
| `battle` | Battle pose with blaster aimed |
| `stability` | Centre-of-mass stability analysis |
| `servo` | Servo torque load estimation |

---

## Example Workflows

### 1. Build Robot + Capture Screenshots

```bash
python src/run_simulation.py --module robot --capture
```

Output:
- `output/screenshots/optimus_robot_Front.png`
- `output/screenshots/optimus_robot_Back.png`
- `output/screenshots/optimus_robot_Left.png`
- `output/screenshots/optimus_robot_Right.png`
- `output/screenshots/optimus_robot_Top.png`
- `output/screenshots/optimus_robot_Iso.png`

### 2. Transform to Truck + Capture

```bash
python src/run_simulation.py --module truck --capture
```

### 3. Full Walk Cycle + Load Analysis

```bash
python src/run_simulation.py --module walk
```

### 4. Simulate All Modules + Export Files

Edit `optimus_prime_g1_v9.py` to set:
```python
EXPORT_STL = True
EXPORT_STEP = True
```

Then run:
```bash
python src/run_simulation.py
```

### 5. Custom MCP Server

```bash
python src/run_simulation.py --mcp-url http://192.168.1.100:27182/mcp
```

---

## Screenshot Capture (`capture_optimus.py`)

Standalone utility to capture viewport renders after the model is already built:

```bash
python src/capture_optimus.py
```

Captures 6 views (Front, Back, Left, Right, Top, Isometric) plus 8 turntable angles and 4 elevated angles.

---

## Post-Simulation Analysis (`analyze_bugs.py`)

Scans `optimus_prime_g1_v8.py` for common coding issues:

```bash
python src/analyze_bugs.py
```

Checks for unbalanced brackets, bare `except:` clauses, and counts function/class declarations.

---

## Stopping a Simulation Mid-Run

1. Run the stop command from another terminal:

   ```bash
   python src/run_simulation.py --stop
   ```

2. This creates `output/stop.flag`
3. The simulation checks this file every frame and exits cleanly if found

---

## Interpreting the Log File

Logs are written to `output/logs/optimus_fusion_log_YYYYMMDD_HHMMSS.txt` and contain:

- Timestamped messages from `Logger.log()`
- Collision/interference warnings with body pair names
- Joint angle changes during each simulation step
- ROM test results (min/max limits reached, collisions at extremes)
- Stability analysis (CoM position per pose)
- Servo load estimates (kg·cm required vs rated)
- Export progress (which STL/STEP files were written)

**Example log entries:**

```
[2026-06-17 14:30:01] INFO  Building component: OP_Torso
[2026-06-17 14:30:05] INFO  Joint ROM: L_Knee — min 0°, max 135°
[2026-06-17 14:30:10] WARN  Collision detected: OP_Thigh_L vs OP_Pelvis at 5.2mm overlap
[2026-06-17 14:30:15] INFO  Stability: Attention pose — STABLE (CoM x=-0.3, y=1.2)
```
