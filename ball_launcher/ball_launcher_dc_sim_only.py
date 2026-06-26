#!/usr/bin/env python3
"""
Ball Launcher DC — MCP Simulation Only
=======================================
Assumes the model is already built in Fusion 360 (from ball_launcher_dc_v1.py).
Connects via MCP and runs the continuous fire cam animation.

Usage:
    python ball_launcher_dc_sim_only.py
"""

import urllib.request
import json
import os
import sys
import time

MCP_URL = os.environ.get("MCP_URL", "http://127.0.0.1:27182/mcp")
session_id = None
_rpc_id = 0


def _next_id():
    global _rpc_id
    _rpc_id += 1
    return _rpc_id


def send_request(payload, timeout=60):
    global session_id
    headers = {'Content-Type': 'application/json'}
    if session_id:
        headers['MCP-Session-Id'] = session_id
    data = json.dumps(payload).encode('utf-8')
    req = urllib.request.Request(MCP_URL, data=data, headers=headers)
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            if 'MCP-Session-Id' in resp.headers:
                session_id = resp.headers.get('MCP-Session-Id')
            body = resp.read().decode('utf-8')
            return json.loads(body) if body else {}
    except Exception as e:
        return {"error": str(e)}


def call_tool(name, args):
    return send_request({
        "jsonrpc": "2.0", "id": _next_id(), "method": "tools/call",
        "params": {"name": name, "arguments": args}
    })


FUSION_SCRIPT = r"""
import adsk.core, adsk.fusion, traceback, math, json, time as _time

_app = adsk.core.Application.get()
_ui = _app.userInterface
design = adsk.fusion.Design.cast(_app.activeProduct)
root = design.rootComponent

# ── Find Cam_Drive Joint ────────────────────────────────────────────────
joint = None
for i in range(root.asBuiltJoints.count):
    j = root.asBuiltJoints.item(i)
    if j.name == "Cam_Drive":
        joint = j
        break

if not joint:
    # Try to find any joint with "Cam" in its name
    for i in range(root.asBuiltJoints.count):
        j = root.asBuiltJoints.item(i)
        if "Cam" in j.name or "cam" in j.name:
            joint = j
            print("Found joint:", j.name)
            break

if not joint:
    print("ERROR: No Cam_Drive joint found. Available joints:")
    for i in range(root.asBuiltJoints.count):
        print(f"  - {root.asBuiltJoints.item(i).name}")
else:
    mo = joint.jointMotion
    print(f"FOUND: {joint.name} (type: {mo.objectType})")

    def _ease(t):
        return t * t * (3.0 - 2.0 * t)

    def _refresh():
        try: _app.activeViewport.refresh()
        except: pass
        try: adsk.doEvents()
        except: pass

    TOTAL_CYCLES = 8
    STEPS_PER_CYCLE = 24
    DELAY = 0.04

    print(f"SIM_START: {TOTAL_CYCLES} firing cycles ({STEPS_PER_CYCLE} steps each)")
    total = TOTAL_CYCLES * STEPS_PER_CYCLE

    for i in range(total):
        frac = i / STEPS_PER_CYCLE
        angle_rad = frac * 2.0 * math.pi
        try:
            if mo.objectType == adsk.fusion.RevoluteJointMotion.classType():
                mo.rotationValue = angle_rad
            elif mo.objectType == adsk.fusion.BallJointMotion.classType():
                mo.yawValue = angle_rad
        except:
            pass

        if i % STEPS_PER_CYCLE == 0:
            shot = i // STEPS_PER_CYCLE + 1
            print(f"  Shot {shot}/{TOTAL_CYCLES}")

        _refresh()
        _time.sleep(DELAY)

    # Reset to home
    try: mo.rotationValue = 0.0
    except: pass
    _refresh()

    print(f"SIM_DONE: {TOTAL_CYCLES} rounds fired (simulated)")

# ── List components for verification ─────────────────────────────────────
comp_names = []
for c in root.allOccurrences:
    comp_names.append(c.component.name)

print(f"COMPONENTS:{json.dumps(comp_names)}")
"""


def main():
    print("=" * 60)
    print("Ball Launcher DC — MCP Simulation (model must be built first)")
    print("=" * 60)
    print()
    print("STEP 1: Build the model in Fusion 360")
    print("  Tools > Scripts & Add-Ins > Run ball_launcher_dc_v1.py")
    print()
    print("STEP 2: Run this simulation")
    print()

    input("Press Enter once the model is built in Fusion 360...")

    send_request({
        "jsonrpc": "2.0", "id": _next_id(), "method": "initialize",
        "params": {"protocolVersion": "2024-11-05", "capabilities": {},
                   "clientInfo": {"name": "ball_launcher_sim", "version": "1.0"}}
    })
    send_request({"jsonrpc": "2.0", "method": "notifications/initialized"})

    print("[MCP] Sending simulation script...\n")
    res = call_tool("fusion_mcp_execute", {
        "featureType": "script",
        "object": {"script": FUSION_SCRIPT}
    })

    if not res:
        print("ERROR: No response from Fusion 360.")
        sys.exit(1)
    if "error" in res:
        print(f"ERROR: {res['error']}")
        sys.exit(1)

    msg = res.get("result", {}).get("message", str(res))
    print(f"\n[MCP] Response:\n{msg}")
    print("\nDone. Watch Fusion 360 for the animation.")


if __name__ == "__main__":
    main()
