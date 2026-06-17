# Probe Scripts Documentation

The `src/` folder contains several diagnostic scripts that help with debugging, testing, and understanding the Fusion 360 MCP environment.

---

## `api_test.py` — MCP Connection Test

**Location:** `src/api_test.py`

**Purpose:** Tests the connection to Fusion 360's MCP server and introspects the Fusion 360 Python API.

**How to run:**

```bash
python src/api_test.py
```

**What it does:**
1. Connects to `http://127.0.0.1:27182/mcp`
2. Sends an `initialize` JSON-RPC request
3. Sends the `notifications/initialized` signal
4. Executes a Python script on the Fusion side that dumps the docstring of `JointGeometry.createByPoint` to a log file
5. Returns and prints the result

**Expected output:**
- If MCP is running: a JSON-RPC response with the script's log output
- If MCP is not running: `Connection refused` error

**Use cases:**
- Verify MCP server is operational before running full simulations
- Quick test after restarting Fusion 360

---

## `analyze_bugs.py` — Static Code Analysis

**Location:** `src/analyze_bugs.py`

**Purpose:** Scans `optimus_prime_g1_v8.py` (or any Python file) for common coding issues.

**How to run:**

```bash
python src/analyze_bugs.py
```

**What it checks:**
- Unbalanced square brackets `[` / `]` in interference log lines
- Bare `except:` clauses without exception type
- Counts total `def` and `class` declarations
- Locates the `run_all_simulations` function

**Output:**
```
=== BUG ANALYSIS REPORT ===
Line 142: Unbalanced '[' ']' in: ...entityOne...
Line 203: Bare except: at: ...except:...
Total definitions: 45
Total classes: 2
run_all_simulations found at line 875
```

**Use cases:**
- Pre-commit code review
- Checking for common pitfalls when modifying the main script

---

## Probe Scripts in `old_code/` or `testingimages/`

The repository may contain additional diagnostic scripts under `old_code/` or `testingimages/`. These are legacy utilities from earlier development versions and are provided for reference. They are not required for normal operation.

### Typical probe script patterns:

1. **truck_layout_probe.py** — Tests wheel placement and grounding by manually applying `transform2` matrices, bypassing joint constraints. Useful for debugging truck mode wheel alignment.

2. **axis_probe.py** — Visually determines pitch/yaw/roll axis mapping by driving each axis individually and observing the result. Helps verify `JOINT_LIMITS` configuration.

3. **api_probe.py** — Dumps available API methods and properties for a given Fusion 360 object to a text file. Useful for discovering undocumented API features.

**To run any probe script:**

1. Open the script and copy its contents
2. In Fusion 360, go to **Tools → Scripts and Add-Ins → Create → Python**
3. Paste the script and run

Or submit via MCP:

```python
# In a Python terminal or script:
from run_simulation import call_tool

with open("path/to/probe.py") as f:
    script = f.read()

result = call_tool("fusion_mcp_execute", {
    "featureType": "script",
    "object": {"script": script}
})
print(result)
```
