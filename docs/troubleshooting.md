# Troubleshooting Guide

## Common Errors

### "Connection refused" / MCP Server Not Reachable

**Symptoms:**
```
Connection error: [Errno 10061] No connection could be made
MCP server not reachable – is Fusion running?
```

**Causes:**
- Fusion 360 is not running
- MCP server has not been started (Tools → Scripts and Add-Ins → MCP Server → Run)
- Fusion is still starting up (MCP takes 30–60s after launch)
- Firewall blocking `127.0.0.1:27182`

**Solutions:**
1. Ensure Fusion 360 is open and the MCP server is running
2. Wait 60 seconds after launching Fusion before running the script
3. Try `python src/api_test.py` to verify connectivity
4. Restart Fusion 360 and the MCP server
5. If using a custom port, set `--mcp-url` or `MCP_URL` environment variable

---

### "Missing MCP-Session-Id header"

**Symptoms:**
```
HTTP 400: Missing MCP-Session-Id header
```

**Cause:** The MCP session was not properly initialized. This can happen after a timeout or if Fusion was restarted.

**Solutions:**
1. Run the script again — it will initialize a new session
2. If persistent, restart Fusion 360 and the MCP server

---

### "Cannot perform action while dialog is open" / Script Hangs

**Symptoms:**
- Script runs but doesn't build anything
- Log contains "Cannot perform action while a dialog is open"
- No output for >2 minutes

**Cause:** Fusion 360 has a modal dialog open (e.g., startup tips, update notifications, error dialogs) blocking API calls.

**Solutions:**
1. **Automatic:** The script sends Escape key presses to dismiss dialogs and retries
2. **Manual:** Close any open dialogs in Fusion 360 before running
3. **Preventive:** Disable "Show Startup Dialog" in Fusion 360 preferences

---

### Joints at Z=0 / Wrong Position

**Symptoms:**
- Joint origins appear at the origin (0,0,0) instead of the intended position
- Components don't move correctly during simulation

**Cause:** The MCP environment may not support `ConstructionPoint.add()` — the script falls back to `SketchPoints` which discard the Z coordinate.

**FIX:** This was addressed in v9.0.0 by trying `constructionPoints` first and falling back to sketch points only if that fails. Update to the latest version.

**Manual workaround:** None needed — the fix is built into `_make_joint_geometry()`.

---

### STL Export Fails

**Symptoms:**
```
Error exporting STL: 'BRepBody' object has no attribute 'exportSTL'
```

**Cause:** Fusion 360 Python API does not support `BRepBody.exportSTL()` — the correct API is through `ExportManager`.

**FIX:** This was fixed in v9.0.0 (FIX 6). Ensure you're using the latest version which uses:
```python
options = design.exportManager.createSTLExportOptions(body, path)
design.exportManager.execute(options)
```

---

### Interference/Collision Check Fails

**Symptoms:**
```
Error: 'Design' object has no attribute 'measureInterference'
```

**Cause:** Fusion 360 API version differences. The correct API name varies.

**FIX:** The `_interfere()` method (v9.0.0, FIX 9) uses `design.createInterferenceInput()` / `design.analyzeInterference()`. A fallback chain handles API version differences.

---

### Simulation Returns No Output

**Symptoms:**
```
Simulation Complete!
```

But no files in `output/` directory.

**Causes and solutions:**

| Cause | Solution |
|-------|----------|
| `CAPTURE_SCREENSHOTS` is False | Add `--capture` flag or set the flag to True |
| Export flags are False | Set `EXPORT_STL`, `EXPORT_STEP`, or `EXPORT_URDF` to True |
| Wrong module name | Check `python src/run_simulation.py --module` with the correct key |
| Script error (exception) | Check the log file in `output/logs/` for traceback |

---

### MCP Server Crashes / Fusion 360 Freezes

**Symptoms:**
- Fusion 360 becomes unresponsive
- No response from MCP even after waiting

**Solutions:**
1. **Force quit Fusion 360** via Task Manager:
   ```
   taskkill /f /im Fusion360.exe
   taskkill /f /im FusionLauncher.exe
   ```
2. Restart Fusion 360
3. If the issue is reproducible, check the log file for the error that caused the crash
4. File a GitHub issue with the log contents

---

### The Stop Flag Isn't Working

**Symptoms:**
- Created `output/stop.flag` but simulation continues

**Solutions:**
1. Ensure the flag file is in the project root's `output/` directory
2. The simulation checks for `stop.flag` every frame — may take a few seconds to respond
3. If still unresponsive, force-quit Fusion 360 (see above)

---

## Reading the Log File

Logs are timestamped and written to `output/logs/optimus_fusion_log_*.txt`.

**Key sections to look for:**

| Log Pattern | Meaning |
|-------------|---------|
| `INFO` | Normal operation — component building, joint movement |
| `WARN` | Non-critical issue — collision detected, joint limit exceeded |
| `ERROR` | Critical failure — exception caught, module skipped |
| `FIX N` | Reference to a bug fix applied in this version |

**Example - troubleshooting a joint issue:**

```
[14:30:05] WARN  L_Knee: Requested 150°, clamped to 135° (limit: 135°)
```

This means the code tried to move the knee past its limit, and the `_clamp()` method corrected it.

---

## Debug Mode

To get more verbose output:

1. Open `src/optimus_v17.py`
2. Look for any `quiet=False` parameters and keep them at their defaults
3. The `Logger` class always writes to both console and log file
4. To add custom debug logging:
   ```python
   Logger.log("my debug message", level="WARN")
   ```
5. Run with `--no-launch` to avoid auto-starting Fusion if you want to control startup manually:
   ```bash
   python src/run_simulation.py --module robot --no-launch
   ```
