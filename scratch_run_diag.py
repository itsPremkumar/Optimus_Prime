import urllib.request
import json
import os

MCP_URL = os.environ.get("MCP_URL", "http://127.0.0.1:27182/mcp")
session_id = None
_rpc_id = 0

def _next_id():
    global _rpc_id
    _rpc_id += 1
    return _rpc_id

def send_request(payload, timeout=30):
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
        return None

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
import adsk.core, adsk.fusion, json

app = adsk.core.Application.get()
if app.documents.count == 0:
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)

design = app.activeProduct

# Find Fusion Appearance Library
app_lib = None
for i in range(app.materialLibraries.count):
    lib = app.materialLibraries.item(i)
    if "Appearance" in lib.name:
        app_lib = lib
        break

targets = ["Paint - Metallic (Red)", "Paint - Metallic (Blue)", "Chrome", "Steel - Flat", "Rubber", "Glass - Window"]
results = {}

if app_lib:
    for target in targets:
        matches = []
        for j in range(app_lib.appearances.count):
            ap = app_lib.appearances.item(j)
            if target.lower() in ap.name.lower():
                matches.append(ap.name)
        results[target] = matches

# Print the names of all design appearances currently in active document
design_apps = [ap.name for ap in design.appearances] if design else []

print(json.dumps({"matches_in_library": results, "design_appearances": design_apps}))
"""

res = call_tool("fusion_mcp_execute", {
    "featureType": "script",
    "object": {"script": script_content}
})

print(json.dumps(res, indent=2))
