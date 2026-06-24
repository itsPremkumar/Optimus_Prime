import urllib.request
import json
import os
import time

MCP_URL = os.environ.get("MCP_URL", "http://127.0.0.1:27182/mcp")
session_id = None
_rpc_id = 0

def _next_id():
    global _rpc_id
    _rpc_id += 1
    return _rpc_id

def send_request(payload):
    global session_id
    headers = {'Content-Type': 'application/json'}
    if session_id:
        headers['MCP-Session-Id'] = session_id
    data = json.dumps(payload).encode('utf-8')
    req = urllib.request.Request(MCP_URL, data=data, headers=headers)
    try:
        with urllib.request.urlopen(req, timeout=30) as resp:
            if 'MCP-Session-Id' in resp.headers:
                session_id = resp.headers.get('MCP-Session-Id')
            body = resp.read().decode('utf-8')
            return json.loads(body) if body else {}
    except urllib.error.HTTPError as e:
        print(f"HTTP {e.code}: {e.read().decode('utf-8')}")
        return None
    except Exception as e:
        print(f"Connection error: {e}")
        return None

def call_tool(name, args):
    return send_request({
        "jsonrpc": "2.0", "id": _next_id(), "method": "tools/call",
        "params": {"name": name, "arguments": args}
    })

# Initialize session
init_res = send_request({
    "jsonrpc": "2.0", "id": _next_id(), "method": "initialize",
    "params": {"protocolVersion": "2024-11-05", "capabilities": {},
               "clientInfo": {"name": "ai_agent", "version": "1.0"}}
})
send_request({"jsonrpc": "2.0", "method": "notifications/initialized"})

script_content = """
import adsk.core, adsk.fusion, json

app = adsk.core.Application.get()
design = app.activeProduct

libs_info = []
for i in range(app.materialLibraries.count):
    lib = app.materialLibraries.item(i)
    libs_info.append({
        "name": lib.name,
        "id": lib.id,
        "has_appearances": hasattr(lib, 'appearances') and lib.appearances.count > 0,
        "appearance_count": lib.appearances.count if hasattr(lib, 'appearances') else 0
    })

# Let's test finding some standard appearances
targets = ["Paint - Metallic (Red)", "Paint - Metallic (Blue)", "Chrome", "Steel - Flat", "Rubber", "Glass - Window"]
results = {}

# Try finding them in all appearance-containing libraries
for lib_info in libs_info:
    if lib_info["appearance_count"] > 0:
        lib = app.materialLibraries.itemByName(lib_info["name"])
        results[lib.name] = {}
        for target in targets:
            found = []
            for j in range(lib.appearances.count):
                ap = lib.appearances.item(j)
                if target.lower() in ap.name.lower():
                    found.append(ap.name)
            results[lib.name][target] = found

# Set document name or write to a text box, or return via print
print(json.dumps({"libraries": libs_info, "matches": results}, indent=2))
"""

res = call_tool("fusion_mcp_execute", {
    "featureType": "script",
    "object": {"script": script_content}
})

print(json.dumps(res, indent=2))
