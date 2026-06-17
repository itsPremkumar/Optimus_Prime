# Installation & Setup Guide

## Prerequisites

- **Autodesk Fusion 360** (Personal/hobbyist license or subscription)
- **Python 3.8+** (standard library only — no extra packages needed)
- **Windows 10/11** (Fusion 360 runs on Windows and macOS; MCP auto-launch is Windows-only)

---

## 1. Install Fusion 360

1. Download from [autodesk.com/products/fusion-360](https://www.autodesk.com/products/fusion-360)
2. Run the installer and sign in with an Autodesk account
3. After installation, launch Fusion 360 at least once to complete setup

---

## 2. Enable the MCP Server

The **Model Context Protocol (MCP)** server allows external scripts to control Fusion 360 remotely.

### Option A: Via Fusion 360 UI

1. Open Fusion 360
2. Go to **Tools → Scripts and Add-Ins** (or press `Shift+S`)
3. In the dialog, find **MCP Server** in the list
4. Select it and click **Run**
5. Keep the dialog open — the server runs as long as Fusion 360 is running

### Option B: Via Command Line (PowerShell)

```powershell
& "C:\Program Files\Autodesk\Fusion 360\FusionLauncher.exe" --mcp
```

### Option C: Auto-Launch (Default)

The `run_simulation.py` script automatically detects and launches Fusion 360 if it's not running. It searches:

- Desktop/Start Menu shortcuts (`.lnk`)
- `C:\Program Files\Autodesk\Fusion 360\FusionLauncher.exe`
- `C:\Program Files (x86)\Autodesk\Fusion 360\FusionLauncher.exe`
- `%LOCALAPPDATA%\Autodesk\webdeploy\production\*\FusionLauncher.exe`

Set the `FUSION_EXE` environment variable to override the search path:

```powershell
$env:FUSION_EXE = "C:\Custom\Path\FusionLauncher.exe"
```

### Verify MCP Is Running

Open a browser and navigate to:

```
http://127.0.0.1:27182/mcp
```

You should see a JSON-RPC response. Alternatively, run:

```bash
python src/api_test.py
```

---

## 3. Clone the Repository

```bash
git clone https://github.com/itsPremkumar/Optimus_Prime.git
cd Optimus_Prime
```

---

## 4. Run Your First Simulation

```bash
python src/run_simulation.py --module robot --capture
```

This builds the robot model, sets the standing pose, and captures 6 screenshots.

**First run notes:**
- MCP server takes 30–60 seconds to become available after Fusion launches
- The script closes all open documents before building the model
- Logs are written to `output/logs/optimus_fusion_log_*.txt`

---

## 5. Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `MCP_URL` | `http://127.0.0.1:27182/mcp` | Custom MCP server URL |
| `FUSION_EXE` | (auto-detected) | Path to FusionLauncher.exe |

---

## 6. Directory Structure After Running

```
Optimus_Prime/
├── output/
│   ├── logs/          # Timestamped execution logs
│   ├── exports/       # STL, STEP, URDF, F3D files
│   └── screenshots/   # 1920×1080 viewport renders
├── src/               # Source code
├── docs/              # Documentation
└── videos/            # Demo videos
```
