#!/usr/bin/env python3
"""
Ball Launcher DC — MCP Continuous Fire Simulation
=================================================
Connects to Autodesk Fusion 360 via MCP. Builds the DC motor-driven
ball launcher model, and runs a continuous firing animation loop.

Usage:
    uv run ball_launcher_dc_mcp_sim.py

Requires Fusion 360 running with MCP enabled (port 27182).
"""

import urllib.request
import json
import os
import sys

MCP_URL = os.environ.get("MCP_URL", "http://127.0.0.1:27182/mcp")
session_id = None
_rpc_id = 0


def _next_id():
    global _rpc_id
    _rpc_id += 1
    return _rpc_id


def send_request(payload, timeout=180):
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
_ui  = _app.userInterface

# ── New document ─────────────────────────────────────────────────────────
_new_doc = _app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
try:
    _app.preferences.generalPreferences.defaultModelingOrientation = \
        adsk.core.DefaultModelingOrientations.ZUpModelingOrientation
except:
    pass

design = adsk.fusion.Design.cast(_app.activeProduct)
root = design.rootComponent

# ── Appearances ──────────────────────────────────────────────────────────
def get_ap(*names):
    lib = None
    for i in range(_app.materialLibraries.count):
        l = _app.materialLibraries.item(i)
        if "Appearance" in l.name:
            lib = l; break
    if not lib: return None
    for n in names:
        for j in range(lib.appearances.count):
            ap = lib.appearances.item(j)
            if n.lower() in ap.name.lower():
                try: return design.appearances.addByCopy(ap)
                except: return ap
    return None

chrome     = get_ap("Chrome", "Steel - Polished")
dark_metal = get_ap("Steel - Flat", "Plastic - Matte (Black)")
dark_grey  = get_ap("Plastic - Matte (Dark Grey)", "Plastic - Matte (Grey)")

# ── Config ───────────────────────────────────────────────────────────────
HOUSING_LX, HOUSING_LY, HOUSING_LZ = 3.2, 2.8, 5.0
BARREL_LEN, BARREL_R_OUT, BARREL_R_IN = 4.5, 0.70, 0.46
CAM_H, CAM_R_HIGH, ECCENTRICITY = 0.40, 0.85, 0.35
CAM_R_LOW = 0.50
PLUNGER_R, PLUNGER_LEN = 0.36, 0.50
SPRING_OD = 0.50
N20_BODY_L, N20_BODY_W, N20_BODY_H = 2.4, 1.2, 1.4
N20_GB_L, N20_GB_W, N20_GB_H = 1.2, 1.2, 1.0

# ── Component helpers ───────────────────────────────────────────────────
comps_list = []
occs = {}
def new_comp(name):
    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    comp = occ.component
    comp.name = name
    comps_list.append(comp)
    occs[name] = occ
    return comp, occ

def set_ap(body, ap):
    if body and ap:
        try: body.appearance = ap
        except: pass

# ── Geometry helpers ─────────────────────────────────────────────────────
def _get_plane(comp, axis, offset):
    # Get construction plane at offset from origin along axis
    planes = {'z': comp.xYConstructionPlane,
              'y': comp.xZConstructionPlane,
              'x': comp.yZConstructionPlane}
    base = planes[axis]
    if abs(offset) < 1e-6:
        return base
    new_planes = comp.constructionPlanes
    pi = new_planes.createInput()
    pi.setByOffset(base, adsk.core.ValueInput.createByReal(offset))
    return new_planes.add(pi)

def extrude_box(comp, name, x0, y0, z0, dx, dy, dz, ap=None):
    # Extrude a box from (x0,y0,z0) with dimensions dx,dy,dz along Z
    if dx <= 0 or dy <= 0 or dz <= 0: return None
    try:
        plane = _get_plane(comp, 'z', z0)
        sketch = comp.sketches.add(plane)
        lines = sketch.sketchCurves.sketchLines
        p1 = adsk.core.Point3D.create(x0, y0, 0)
        p2 = adsk.core.Point3D.create(x0 + dx, y0 + dy, 0)
        lines.addTwoPointRectangle(p1, p2)
        prof = sketch.profiles.item(0)
        ext = comp.features.extrudeFeatures.createInput(
            prof, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        ext.setDistanceExtent(False, adsk.core.ValueInput.createByReal(dz))
        body = comp.features.extrudeFeatures.add(ext)
        if body:
            body.name = name
            set_ap(body, ap)
        try: sketch.isVisible = False
        except: pass
        return body
    except Exception as ex:
        print(f"  extrude_box '{name}' failed: {ex}")
        return None

def extrude_cyl(comp, name, cx, cy, cz, r, h, axis='z', ap=None):
    # Extrude a cylinder at (cx,cy,cz) along axis with height h
    if r <= 0 or h <= 0: return None
    try:
        plane = _get_plane(comp, axis, cz)
        sketch = comp.sketches.add(plane)
        c = adsk.core.Point3D.create(cx, cy, 0)
        sketch.sketchCurves.sketchCircles.addByCenterRadius(c, r)
        prof = sketch.profiles.item(0)
        ext = comp.features.extrudeFeatures.createInput(
            prof, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        ext.setDistanceExtent(False, adsk.core.ValueInput.createByReal(h))
        body = comp.features.extrudeFeatures.add(ext)
        if body:
            body.name = name
            set_ap(body, ap)
        try: sketch.isVisible = False
        except: pass
        return body
    except Exception as ex:
        print(f"  extrude_cyl '{name}' failed: {ex}")
        return None

def cut_cyl(comp, cx, cy, cz, r, depth, axis='z'):
    try:
        plane = _get_plane(comp, axis, cz)
        sketch = comp.sketches.add(plane)
        c = adsk.core.Point3D.create(cx, cy, 0)
        sketch.sketchCurves.sketchCircles.addByCenterRadius(c, r)
        prof = sketch.profiles.item(0)
        cut = comp.features.extrudeFeatures.createInput(
            prof, adsk.fusion.FeatureOperations.CutFeatureOperation)
        cut.setDistanceExtent(False, adsk.core.ValueInput.createByReal(depth))
        comp.features.extrudeFeatures.add(cut)
        try: sketch.isVisible = False
        except: pass
    except Exception as ex:
        print(f"  cut_cyl failed: {ex}")

def revolute_joint(name, occ1, occ2, cx, cy, cz, axis_str):
    if not (occ1 and occ2): return
    try:
        sketch = root.sketches.add(root.xYConstructionPlane)
        sk_pt = sketch.sketchPoints.add(adsk.core.Point3D.create(cx, cy, cz))
        geom = adsk.fusion.JointGeometry.createByPoint(sk_pt)
        ji = root.asBuiltJoints.createInput(occ1, occ2, geom)
        av = {"x": adsk.core.Vector3D.create(1,0,0),
              "y": adsk.core.Vector3D.create(0,1,0),
              "z": adsk.core.Vector3D.create(0,0,1)}[axis_str]
        ji.setAsRevoluteJointMotion(
            adsk.fusion.JointDirections.CustomJointDirection, av)
        j = root.asBuiltJoints.add(ji); j.name = name
        try: sketch.isVisible = False
        except: pass
    except Exception as ex:
        print(f"  revolute_joint '{name}' failed: {ex}")

# ═══════════════════════════════════════════════════════════════════════
# BUILD MODEL
# ═══════════════════════════════════════════════════════════════════════
print("BUILD_START")

main_comp, main_occ = new_comp("OP_Ball_Launcher_DC")
cam_comp, cam_occ = new_comp("Cam_Drop")

# Housing (box from z=0, centered XY)
hz = HOUSING_LZ  # height
extrude_box(main_comp, "Housing", -HOUSING_LX/2, -HOUSING_LY/2, 0,
            HOUSING_LX, HOUSING_LY, HOUSING_LZ, dark_metal)

# Housing interior cut
extrude_box(main_comp, "Housing_Int", -HOUSING_LX/2 + 0.18, -HOUSING_LY/2 + 0.18, 0.18,
            HOUSING_LX - 0.36, HOUSING_LY - 0.36, HOUSING_LZ - 0.36, None)

# Rear cap
extrude_box(main_comp, "Rear_Cap", -HOUSING_LX/2 + 0.2, -HOUSING_LY/2 + 0.2, 0,
            HOUSING_LX - 0.4, HOUSING_LY - 0.4, 0.30, dark_grey)

# Barrel (extends from housing front face in +Z)
bz_start = HOUSING_LZ - 0.3
extrude_cyl(main_comp, "Barrel", 0, 0, bz_start, BARREL_R_OUT, BARREL_LEN, 'z', chrome)
cut_cyl(main_comp, 0, 0, bz_start, BARREL_R_IN, BARREL_LEN + 0.2, 'z')

# Muzzle ring
extrude_cyl(main_comp, "Muzzle_Ring", 0, 0, bz_start + BARREL_LEN,
            BARREL_R_OUT + 0.08, 0.20, 'z', dark_metal)

# Plunger (inside housing, along Z from rear)
p_cz = 0.6  # starts near rear
extrude_cyl(main_comp, "Plunger_Shaft", 0, 0, p_cz, 0.15, 1.6, 'z', chrome)
extrude_cyl(main_comp, "Plunger_Head", 0, 0, p_cz + 0.3, PLUNGER_R, PLUNGER_LEN, 'z', dark_metal)

# Motor body (N20 gearmotor, mounted below housing)
m_cx = HOUSING_LX/2 - 0.1
m_cy = -HOUSING_LY/2 - N20_BODY_H/2 - 0.1
m_cz = p_cz + 0.35
extrude_box(main_comp, "Motor_Body", m_cx - N20_BODY_L/2, m_cy - N20_BODY_H/2, m_cz - N20_BODY_W/2,
            N20_BODY_L, N20_BODY_H, N20_BODY_W, dark_metal)

# Motor bracket
extrude_box(main_comp, "Motor_Bracket", m_cx - 0.9, -HOUSING_LY/2 - 0.15, m_cz - 1.0,
            1.8, 0.15, 2.0, dark_metal)

# Front sight
extrude_box(main_comp, "Front_Sight", -0.1, BARREL_R_OUT + 0.05, bz_start + BARREL_LEN - 0.5,
            0.20, 0.30, 0.12, dark_metal)

# Top rail
extrude_box(main_comp, "Top_Rail", -1.3, HOUSING_LY/2 + 0.02, -0.5,
            HOUSING_LX - 0.6, 0.12, 2.0, chrome)

# Mount hinge (at rear of housing)
extrude_box(main_comp, "Mount_Hinge", -0.55, -0.325, 0.3,
            1.1, 0.65, 0.8, dark_metal)

# ── CAM component ────────────────────────────────────────────────────────
cam_cx = HOUSING_LX/2 - 0.1
cam_cz = p_cz + 0.35  # same height as motor shaft

# Cam body (eccentric disc)
extrude_cyl(cam_comp, "Cam_Body", cam_cx + ECCENTRICITY, 0, cam_cz,
            CAM_R_HIGH, CAM_H, 'y', dark_metal)

# Cam shaft boss
extrude_cyl(cam_comp, "Cam_Shaft_Boss", cam_cx, 0, cam_cz,
            0.12, CAM_H + 0.1, 'y', chrome)

# Cam counterweight
extrude_box(cam_comp, "Cam_Weight", cam_cx - ECCENTRICITY - 0.2, -0.1, cam_cz - 0.27,
            0.25, 0.20, 0.55, dark_metal)

# ═══════════════════════════════════════════════════════════════════════
# JOINTS
# ═══════════════════════════════════════════════════════════════════════
revolute_joint("Cam_Drive", cam_occ, main_occ, cam_cx, 0, cam_cz, "y")

_app.activeViewport.refresh()
adsk.doEvents()
print("BUILD_DONE")

# ═══════════════════════════════════════════════════════════════════════
# SIMULATION — Continuous Fire
# ═══════════════════════════════════════════════════════════════════════
def _ease(t):
    return t * t * (3.0 - 2.0 * t)

joint = root.asBuiltJoints.itemByName("Cam_Drive")
if joint:
    mo = joint.jointMotion
    CYCLES = 10
    STEPS = 30
    DELAY = 0.025

    print(f"SIM_START: {CYCLES} firing cycles")
    for i in range(CYCLES * STEPS):
        frac = i / STEPS
        angle_rad = frac * 2.0 * math.pi
        try:
            if mo.objectType == adsk.fusion.RevoluteJointMotion.classType():
                mo.rotationValue = angle_rad
            else:
                mo.yawValue = angle_rad
        except: pass
        if i % STEPS == 0:
            print(f"  Shot {i//STEPS + 1}/{CYCLES}")
        try: _app.activeViewport.refresh()
        except: pass
        _time.sleep(DELAY)

    try: mo.rotationValue = 0.0
    except: pass
    _app.activeViewport.refresh()
    print("SIM_DONE")
else:
    print("ERROR: Cam_Drive not found")

# Report
body_count = sum(1 for c in comps_list for b in c.bRepBodies)
result = {
    "status": "success",
    "components": len(comps_list),
    "bodies": body_count,
    "joint": "Cam_Drive" if joint else None,
    "cycles": CYCLES,
    "drive": "N20 gearmotor + eccentric drop cam",
    "fire_mode": "full-auto (continuous)"
}
print(f"RESULT:{json.dumps(result)}")
"""


def main():
    print("=" * 60)
    print("Ball Launcher DC — MCP Continuous Fire Simulation")
    print("=" * 60)

    send_request({
        "jsonrpc": "2.0", "id": _next_id(), "method": "initialize",
        "params": {"protocolVersion": "2024-11-05", "capabilities": {},
                   "clientInfo": {"name": "ball_launcher_sim", "version": "1.0"}}
    })
    send_request({"jsonrpc": "2.0", "method": "notifications/initialized"})

    print("[MCP] Sending build + simulation script to Fusion 360...\n")
    res = call_tool("fusion_mcp_execute", {
        "featureType": "script",
        "object": {"script": FUSION_SCRIPT}
    })

    if not res:
        print("ERROR: No response from Fusion 360.")
        sys.exit(1)

    msg = res.get("result", {}).get("message", str(res))
    print(f"\n[MCP] Response:\n{msg}")

    if "RESULT:" in msg:
        try:
            data = json.loads(msg.split("RESULT:")[-1].strip())
            print("\n" + "=" * 60)
            print("SIMULATION COMPLETE")
            print("=" * 60)
            for k, v in data.items():
                print(f"  {k}: {v}")
        except:
            pass

    print("\nDone. Check Fusion 360 for the animated launcher.")


if __name__ == "__main__":
    main()
