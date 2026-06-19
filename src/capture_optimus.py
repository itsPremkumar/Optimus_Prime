#!/usr/bin/env python3
"""
Advanced Optimus Prime Screenshot Capture
- All standard orthographic views (Front, Back, Left, Right, Top, Bottom)
- Isometric views (IsoTopRight, IsoTopLeft, IsoBottomRight, IsoBottomLeft)
- Custom 45° elevated turntable views (0°, 45°, 90°, ... 315°)
- Camera margin for better composition
- Automatic AI inspection sequence (covers every angle thoroughly)
"""

import urllib.request
import json
import os
import sys
import time

# ── Configuration ────────────────────────────────────────────────────────
MCP_URL = os.environ.get("MCP_URL", "http://127.0.0.1:27182/mcp")
SCREENSHOT_WIDTH = 1920      # Change to 3840 for 4K
SCREENSHOT_HEIGHT = 1080      # Change to 2160 for 4K
CAMERA_MARGIN = 0.15          # Extra space around model (15% of view distance)
session_id = None

# ── MCP Communication ────────────────────────────────────────────────────
def send_request(payload, timeout=30):
    global session_id
    headers = {'Content-Type': 'application/json'}
    if session_id:
        headers['MCP-Session-Id'] = session_id

    req = urllib.request.Request(
        MCP_URL,
        data=json.dumps(payload).encode('utf-8'),
        headers=headers
    )
    try:
        with urllib.request.urlopen(req, timeout=timeout) as response:
            if 'MCP-Session-Id' in response.headers:
                session_id = response.headers.get('MCP-Session-Id')
            body = response.read().decode('utf-8')
            return json.loads(body) if body else {}
    except Exception as e:
        print(f"MCP error: {e}")
        return None

def _get_error(res):
    """Extract error message from MCP response."""
    if not isinstance(res, dict):
        return "Invalid response type"
    err = res.get("error")
    if err:
        return err.get("message", str(err))
    result = res.get("result", {})
    if isinstance(result, dict):
        msg = result.get("message", "")
        if "error" in msg.lower():
            return msg
    return None

# ── Main ─────────────────────────────────────────────────────────────────
def main():
    print("=" * 60)
    print("Optimus Prime Advanced Screenshot Capture")
    print("=" * 60)

    # 1. Initialize MCP
    print("Connecting to Fusion 360 MCP...")
    send_request({
        "jsonrpc": "2.0", "id": 1, "method": "initialize",
        "params": {"protocolVersion": "2024-11-05", "capabilities": {},
                   "clientInfo": {"name": "ai", "version": "1.0"}}
    })
    send_request({"jsonrpc": "2.0", "method": "notifications/initialized"})

    # 2. Prepare output directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    output_dir = os.path.join(os.path.dirname(script_dir), "output", "screenshots")
    os.makedirs(output_dir, exist_ok=True)
    print(f"Screenshots will be saved to: {output_dir}")

    # 3. Build the Fusion script (with all settings passed as variables)
    fusion_script = f'''
import adsk.core, adsk.fusion, traceback, time, os, sys, math
from datetime import datetime

OUTPUT_DIR = r"{output_dir}"
WIDTH = {SCREENSHOT_WIDTH}
HEIGHT = {SCREENSHOT_HEIGHT}
MARGIN = {CAMERA_MARGIN}

def run(context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    design = adsk.fusion.Design.cast(app.activeProduct)

    # Safety check: must have a design open
    if app.documents.count == 0:
        ui.messageBox("No document open. Please load the Optimus Prime model first.")
        raise Exception("No design open")

    viewport = app.activeViewport
    camera = viewport.camera

    # Hide construction geometry for clean screenshots
    try:
        root = design.rootComponent
        for cp in root.constructionPoints:
            cp.isLightBulbOn = False
        for sk in root.sketches:
            sk.isVisible = False
        for occ in root.allOccurrences:
            comp = occ.component
            for cp in comp.constructionPoints:
                cp.isLightBulbOn = False
            for sk in comp.sketches:
                sk.isVisible = False
    except:
        pass

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")

    # Helper to add margin after FitView
    def apply_margin():
        """Move camera back by MARGIN percentage of eye-target distance."""
        eye = camera.eye
        target = camera.target
        vec = target.vectorTo(eye)  # direction from target to eye (backward)
        if vec.length > 0:
            vec.normalize()
            distance = eye.distanceTo(target)
            offset = vec.copy()
            offset.scaleBy(MARGIN * distance)
            eye.translateBy(offset)
            camera.eye = eye
            viewport.camera = camera
            adsk.doEvents()

    # Helper to capture one view
    def capture(name, orientation, is_custom=False):
        if orientation is not None:
            camera.viewOrientation = orientation
            camera.isFitView = True
            viewport.camera = camera
            # Multiple doEvents + sleep to ensure viewport updates
            for _ in range(5):
                adsk.doEvents()
                time.sleep(0.1)
            apply_margin()
            for _ in range(3):
                adsk.doEvents()
                time.sleep(0.1)
        elif is_custom:
            apply_margin()
            adsk.doEvents()
        time.sleep(0.3)  # extra pause to ensure render

        filename = f"optimus_{{name}}_{{ts}}.png"
        path = os.path.join(OUTPUT_DIR, filename)
        viewport.saveAsImageFile(path, WIDTH, HEIGHT)
        print(f"Captured {{name}} -> {{path}}")

    # 4. Standard orthographic views
    ortho_views = [
        ("Front", adsk.core.ViewOrientations.FrontViewOrientation),
        ("Back", adsk.core.ViewOrientations.BackViewOrientation),
        ("Left", adsk.core.ViewOrientations.LeftViewOrientation),
        ("Right", adsk.core.ViewOrientations.RightViewOrientation),
        ("Top", adsk.core.ViewOrientations.TopViewOrientation),
        ("Bottom", adsk.core.ViewOrientations.BottomViewOrientation),
    ]

    # 5. Isometric views (all four combinations)
    iso_views = [
        ("IsoTopRight", adsk.core.ViewOrientations.IsoTopRightViewOrientation),
        ("IsoTopLeft", adsk.core.ViewOrientations.IsoTopLeftViewOrientation),
        ("IsoBottomRight", adsk.core.ViewOrientations.IsoBottomRightViewOrientation),
        ("IsoBottomLeft", adsk.core.ViewOrientations.IsoBottomLeftViewOrientation),
    ]

    # Capture standard orthographic views
    for name, orientation in ortho_views:
        capture(name, orientation)

    # Capture isometric views
    for name, orientation in iso_views:
        capture(name, orientation)

    # 6. Turntable sequence: rotate around vertical axis (Z-axis for Z-up)
    turntable_angles = [0, 45, 90, 135, 180, 225, 270, 315]
    # Elevated angles: orbit with camera raised above midplane
    elevated_angles = [0, 90, 180, 270]

    # First, get model extents for turntable radius
    camera.isFitView = True
    viewport.camera = camera
    adsk.doEvents()
    # Re-fit to get accurate target
    target = camera.target
    eye = camera.eye
    # Distance from eye to target
    distance = eye.distanceTo(target)
    # We'll orbit around Z (vertical) axis, maintaining same distance and upward vector.
    up_vector = adsk.core.Vector3D.create(0, 0, 1)  # Z-up
    # Starting direction from target to eye (in XY plane)
    start_dir = adsk.core.Vector3D.create(eye.x - target.x, eye.y - target.y, 0)
    if start_dir.length < 1e-6:
        start_dir = adsk.core.Vector3D.create(1, 0, 0)
    start_dir.normalize()

    for angle in turntable_angles:
        rad = math.radians(angle)
        # Rotate start_dir around Z-axis by angle
        cos_a = math.cos(rad)
        sin_a = math.sin(rad)
        new_dir = adsk.core.Vector3D.create(
            start_dir.x * cos_a - start_dir.y * sin_a,
            start_dir.x * sin_a + start_dir.y * cos_a,
            0
        )
        new_dir.normalize()
        # New eye position: target + new_dir * distance
        new_eye = target.copy()
        scaled_dir = new_dir.copy()
        scaled_dir.scaleBy(distance)
        new_eye.translateBy(scaled_dir)
        camera.eye = new_eye
        camera.upVector = up_vector
        camera.isFitView = False  # keep distance
        viewport.camera = camera
        adsk.doEvents()
        time.sleep(0.2)
        capture("Turntable_" + str(angle).zfill(3), None, is_custom=True)

    # Elevated turntable: camera raised 30 degrees above midplane
    elevation = math.radians(30)
    for angle in elevated_angles:
        rad = math.radians(angle)
        cos_a = math.cos(rad)
        sin_a = math.sin(rad)
        # Rotate in XY plane then lift Z component
        horiz_dist = distance * math.cos(elevation)
        new_dir = adsk.core.Vector3D.create(
            start_dir.x * cos_a - start_dir.y * sin_a,
            start_dir.x * sin_a + start_dir.y * cos_a,
            math.sin(elevation)
        )
        new_dir.normalize()
        new_eye = target.copy()
        scaled_dir = new_dir.copy()
        scaled_dir.scaleBy(distance)
        new_eye.translateBy(scaled_dir)
        camera.eye = new_eye
        camera.upVector = up_vector
        camera.isFitView = False
        viewport.camera = camera
        adsk.doEvents()
        time.sleep(0.2)
        capture("Elevated_" + str(angle).zfill(3), None, is_custom=True)

    total = len(ortho_views) + len(iso_views) + len(turntable_angles) + len(elevated_angles)
    print("All screenshots captured successfully. Total: " + str(total) + " images.")

'''

    # 4. Send script to Fusion
    execute_payload = {
        "jsonrpc": "2.0", "id": 2, "method": "tools/call",
        "params": {
            "name": "fusion_mcp_execute",
            "arguments": {
                "featureType": "script",
                "object": {"script": fusion_script}
            }
        }
    }

    print("Sending capture script to Fusion 360...")
    res = send_request(execute_payload, timeout=120)

    # 5. Interpret result
    if res:
        print(f"MCP Response: {json.dumps(res, indent=2)}")
        err = _get_error(res)
        if err:
            print(f"Screenshot capture failed: {err}")
        else:
            print("Screenshot capture completed successfully.")
    else:
        print("No response from Fusion MCP -- something went wrong.")

if __name__ == "__main__":
    main()
