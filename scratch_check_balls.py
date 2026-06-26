import urllib.request, json, os

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
import adsk.core, adsk.fusion, json

app = adsk.core.Application.get()
design = adsk.fusion.Design.cast(app.activeProduct)
root = design.rootComponent

balls_info = []
for occ in root.allOccurrences:
    if "Ball" in occ.name:
        t = occ.transform.translation
        balls_info.append({
            "name": occ.name,
            "comp_name": occ.component.name,
            "x": t.x, "y": t.y, "z": t.z,
            "visible": occ.isLightBulbOn
        })

print(json.dumps(balls_info, indent=2))
"""

res = call_tool("fusion_mcp_execute", {
    "featureType": "script",
    "object": {"script": script_content}
})

print(json.dumps(res, indent=2))
