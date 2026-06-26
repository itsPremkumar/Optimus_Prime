import urllib.request, json, os, sys

MCP_URL = os.environ.get("MCP_URL", "http://127.0.0.1:27182/mcp")
session_id = None
_rpc_id = 0

def _next_id():
    global _rpc_id; _rpc_id += 1; return _rpc_id

def send_request(payload, timeout=30):
    global session_id
    headers = {'Content-Type': 'application/json'}
    if session_id: headers['MCP-Session-Id'] = session_id
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

# Initialize session
send_request({
    "jsonrpc": "2.0", "id": _next_id(), "method": "initialize",
    "params": {"protocolVersion": "2024-11-05", "capabilities": {},
               "clientInfo": {"name": "ai_agent", "version": "1.0"}}
})
send_request({"jsonrpc": "2.0", "method": "notifications/initialized"})

script_content = """
import adsk.core, adsk.fusion, os

app = adsk.core.Application.get()
design = adsk.fusion.Design.cast(app.activeProduct)
root = design.rootComponent

# Locate components by name
ho = None
mgo = None
for occ in root.occurrences:
    if "Housing" in occ.component.name:
        ho = occ
    elif "Magazine" in occ.component.name:
        mgo = occ

# Restore all visibilities first
if ho: ho.isLightBulbOn = True
if mgo: mgo.isLightBulbOn = True
app.activeViewport.refresh()

artifact_dir = r"C:\\Users\\PREM KUMAR\\.gemini\\antigravity\\brain\\41ed2feb-a7f0-40de-807d-773ea665695e"
images = {}

def capture(filename, view_orientation, hide_housing=False, hide_mag=False):
    if ho: ho.isLightBulbOn = not hide_housing
    if mgo: mgo.isLightBulbOn = not hide_mag
    
    view = app.activeViewport
    camera = view.camera
    camera.cameraType = adsk.core.CameraTypes.OrthographicCameraType
    camera.viewOrientation = view_orientation
    camera.isFitView = True
    view.camera = camera
    view.refresh()
    
    path = os.path.join(artifact_dir, filename)
    view.saveAsImageFile(path, 1200, 900)
    images[filename] = path

# 1. External ISO View
capture("assembly_iso.png", adsk.core.ViewOrientations.IsometricViewOrientation)

# 2. Internal ISO View (Housing hidden to see Plunger, Spring, Cam, Motor, Chambered Ball)
capture("internal_iso.png", adsk.core.ViewOrientations.IsometricViewOrientation, hide_housing=True)

# 3. Side View (Housing hidden to verify Cam/Plunger roller contact and Spring seating)
capture("internal_side.png", adsk.core.ViewOrientations.RightViewOrientation, hide_housing=True)

# 4. Top View (Housing hidden to verify Ball Magazine drop center alignment)
capture("internal_top.png", adsk.core.ViewOrientations.TopViewOrientation, hide_housing=True)

# Restore all visibilities
if ho: ho.isLightBulbOn = True
if mgo: mgo.isLightBulbOn = True
app.activeViewport.refresh()

print("SCREENSHOTS_DONE: " + str(images))
"""

res = call_tool("fusion_mcp_execute", {
    "featureType": "script",
    "object": {"script": script_content}
})

print(json.dumps(res, indent=2))
