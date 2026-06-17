#!/usr/bin/env python3
"""
Advanced Optimus Prime G1 Simulation Controller
- Auto-starts Fusion 360 if not running.
- Waits for MCP server to become available.
- Closes all open documents.
- Executes the payload.
"""
import urllib.request
import json
import os
import sys
import time
import subprocess
import argparse
import tempfile

# ------------------------------------------------------------------ #
#  Configuration
# ------------------------------------------------------------------ #
MCP_URL = os.environ.get("MCP_URL", "http://127.0.0.1:27182/mcp")

def find_fusion_launcher():
    """Return the best path to launch Fusion 360 (.lnk preferred for Start-Process)."""
    exe = os.environ.get("FUSION_EXE")
    if exe and os.path.exists(exe):
        return exe

    # .lnk shortcuts first — Start-Process handles them best
    known_shortcuts = [
        os.path.expandvars(r"%APPDATA%\Microsoft\Windows\Start Menu\Programs\Autodesk\Autodesk Fusion.lnk"),
        os.path.expandvars(r"%PUBLIC%\Desktop\Autodesk Fusion 360.lnk"),
        os.path.expandvars(r"%USERPROFILE%\Desktop\Autodesk Fusion 360.lnk"),
    ]
    for lnk in known_shortcuts:
        if os.path.exists(lnk):
            return lnk

    # Fallback to .exe paths
    candidates = [
        r"C:\Program Files\Autodesk\Fusion 360\FusionLauncher.exe",
        r"C:\Program Files (x86)\Autodesk\Fusion 360\FusionLauncher.exe",
    ]
    for c in candidates:
        if os.path.exists(c):
            return c

    # Search webdeploy folders
    webdeploy = os.path.expandvars(r"%LOCALAPPDATA%\Autodesk\webdeploy\production")
    if os.path.isdir(webdeploy):
        for d in os.listdir(webdeploy):
            launcher = os.path.join(webdeploy, d, "FusionLauncher.exe")
            if os.path.exists(launcher):
                return launcher

    return None

FUSION_EXE = find_fusion_launcher()

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
PAYLOAD_FILE = os.path.join(BASE_DIR, "optimus_prime_g1_v9.py")

_rpc_id = 0
session_id = None

def _next_id():
    global _rpc_id
    _rpc_id += 1
    return _rpc_id

# ------------------------------------------------------------------ #
#  MCP communication helper
# ------------------------------------------------------------------ #
def send_request(payload, timeout=30, quiet=False):
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
    except urllib.error.HTTPError as e:
        err_body = e.read().decode('utf-8')
        if not quiet:
            print(f"HTTP {e.code}: {err_body}")
        return None
    except Exception as e:
        if not quiet:
            print(f"Connection error: {e}")
        return None

def call_tool(name, args, timeout=30):
    return send_request({
        "jsonrpc": "2.0", "id": _next_id(), "method": "tools/call",
        "params": {"name": name, "arguments": args}
    }, timeout=timeout)

# ------------------------------------------------------------------ #
#  Fusion process management
# ------------------------------------------------------------------ #
def is_fusion_running():
    """Return True if a Fusion 360 process is found."""
    try:
        import psutil
        for proc in psutil.process_iter(['name']):
            if proc.info['name'] and 'fusion' in proc.info['name'].lower():
                return True
    except ImportError:
        # fallback – check via tasklist
        result = subprocess.run(['tasklist', '/fi', 'IMAGENAME eq FusionLauncher.exe'],
                                capture_output=True, text=True)
        if 'FusionLauncher.exe' in result.stdout:
            return True
    return False

def launch_fusion():
    """Start Fusion 360 via PowerShell Start-Process (handles .lnk reliably)."""
    if not FUSION_EXE:
        print("Fusion 360 executable not found.")
        print("Please start Fusion 360 manually or set FUSION_EXE env var.")
        return False
    print(f"Starting Fusion 360 from: {FUSION_EXE}")
    ps_cmd = f'Start-Process "{FUSION_EXE}"'
    try:
        subprocess.run(["powershell", "-NoProfile", "-Command", ps_cmd],
                       capture_output=True, timeout=15)
    except Exception as e:
        print(f"Launch error: {e}")
        return False
    time.sleep(5)
    return True

def wait_for_mcp(timeout=120):
    """Wait until the MCP server responds to an initialize request."""
    print("Waiting for MCP server...", end="", flush=True)
    start = time.time()
    dots = 0
    while time.time() - start < timeout:
        try:
            resp = send_request({
                "jsonrpc": "2.0", "id": _next_id(), "method": "initialize",
                "params": {"protocolVersion": "2024-11-05", "capabilities": {},
                           "clientInfo": {"name": "ai_agent", "version": "1.0"}}
            }, timeout=5, quiet=True)
            if resp is not None:
                send_request({"jsonrpc": "2.0", "method": "notifications/initialized"}, quiet=True)
                print(" ready.")
                return True
        except Exception:
            dots += 1
            if dots % 10 == 0:
                print(".", end="", flush=True)
        time.sleep(2)
    print(" timeout.")
    return False

# ------------------------------------------------------------------ #
#  Document management
# ------------------------------------------------------------------ #
CLOSE_DOCS_PROLOGUE = """
import adsk.core, adsk.fusion
try:
    app = adsk.core.Application.get()
    docs = app.documents
    for i in range(docs.count - 1, -1, -1):
        doc = docs.item(i)
        try:
            doc.close(False)
        except:
            pass
except:
    pass
"""

def send_escape():
    """Send Escape key to Fusion to dismiss modal dialogs (via PowerShell)."""
    ps_script = '''
Add-Type @"
using System;
using System.Runtime.InteropServices;
using System.Diagnostics;
public class FusionKiller {
    [DllImport("user32.dll")]
    public static extern bool PostMessage(IntPtr hWnd, uint msg, int wParam, int lParam);
    public const uint WM_KEYDOWN = 0x0100;
    public const uint WM_KEYUP   = 0x0101;
    public const int VK_ESCAPE = 0x1B;

    public static void SendEsc() {
        Process[] procs = Process.GetProcesses();
        foreach (var p in procs) {
            if (p.MainWindowTitle.ToLower().Contains("fusion 360") || p.MainWindowTitle.ToLower().Contains("autodesk")) {
                IntPtr h = p.MainWindowHandle;
                if (h != IntPtr.Zero) {
                    for (int i=0; i<5; i++) {
                        PostMessage(h, WM_KEYDOWN, VK_ESCAPE, 0);
                        PostMessage(h, WM_KEYUP,   VK_ESCAPE, 0);
                        System.Threading.Thread.Sleep(200);
                    }
                    Console.WriteLine("Esc sent to " + p.MainWindowTitle);
                    return;
                }
            }
        }
        Console.WriteLine("Fusion window not found");
    }
}
"@
[FusionKiller]::SendEsc()
'''
    try:
        subprocess.run(["powershell", "-NoProfile", "-Command", ps_script],
                       capture_output=True, timeout=10)
    except Exception as e:
        print(f"Escape error: {e}")

# ------------------------------------------------------------------ #
#  Core simulation runner
# ------------------------------------------------------------------ #
def run_simulation(script_content):
    return call_tool("fusion_mcp_execute", {
        "featureType": "script",
        "object": {"script": script_content}
    }, timeout=3600)  # up to 1 hour

def _get_log_text(res):
    if not isinstance(res, dict):
        return ""
    out = res.get("message") or (res.get("result") or {}).get("message") or ""
    if out:
        return out
    for c in (res.get("result") or {}).get("content") or []:
        if isinstance(c, dict):
            raw = c.get("text", "")
            if raw:
                try:
                    parsed = json.loads(raw)
                    msg = parsed.get("message", "")
                    if msg:
                        return msg
                except:
                    pass
                return raw
    return ""

# ------------------------------------------------------------------ #
#  Main
# ------------------------------------------------------------------ #
if __name__ == "__main__":
    session_id = None
    _rpc_id = 0

    parser = argparse.ArgumentParser()
    parser.add_argument("--module", default="ALL", choices=["ALL","rom","head","wave","breathing","walk","run","combat","transform","truck","robot","stability","servo"])
    parser.add_argument("--capture", action="store_true")
    parser.add_argument("--mcp-url", default=None)
    parser.add_argument("--no-launch", action="store_true", help="Don't try to start Fusion")
    parser.add_argument("--keep-docs", action="store_true", help="Don't close existing documents")
    args = parser.parse_args()

    if args.mcp_url:
        MCP_URL = args.mcp_url

    # Ensure output encoding
    try:
        sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    except:
        pass

    # 1. Ensure Fusion is running
    if not args.no_launch and not is_fusion_running():
        if not launch_fusion():
            sys.exit(1)
        if not wait_for_mcp(timeout=180):
            print("Fusion MCP server did not start in time.")
            sys.exit(1)
    else:
        if not wait_for_mcp(timeout=10):
            print("MCP server not reachable – is Fusion running?")
            sys.exit(1)

    # 2. Send Esc to dismiss any startup dialogs
    for _ in range(3):
        send_escape()
        time.sleep(0.5)

    # 3. Close all existing documents (embedded in payload to avoid session issues)
    output_dir = os.path.join(os.path.dirname(BASE_DIR), "output")
    print(f"Loading payload for module '{args.module}'...")
    with open(PAYLOAD_FILE, "r", encoding="utf-8") as f:
        script = ""
        if not args.keep_docs:
            script += CLOSE_DOCS_PROLOGUE + "\n"
        script += f"TARGET_MODULE = '{args.module}'\n"
        script += f"CAPTURE_SCREENSHOTS = {args.capture}\n"
        script += f"EXPORT_DIR = r'{output_dir}\\exports'\n"
        script += f"LOG_DIR = r'{output_dir}\\logs'\n"
        script += f"SCREENSHOT_DIR = r'{output_dir}\\screenshots'\n"
        script += f.read()

    # 4. Run the simulation
    print("Sending Optimus Prime G1 engine to Fusion...")
    res = run_simulation(script)

    if res:
        log = _get_log_text(res)
        if "Cannot perform" in log or "dialog" in log:
            print("Dialog blocking – trying Escape and retry...")
            for _ in range(5):
                send_escape()
                time.sleep(0.3)
            # Retry without the close-docs prologue (docs already closed)
            script_retry = ""
            script_retry += f"TARGET_MODULE = '{args.module}'\n"
            script_retry += f"CAPTURE_SCREENSHOTS = {args.capture}\n"
            script_retry += f"EXPORT_DIR = r'{output_dir}\\exports'\n"
            script_retry += f"LOG_DIR = r'{output_dir}\\logs'\n"
            script_retry += f"SCREENSHOT_DIR = r'{output_dir}\\screenshots'\n"
            with open(PAYLOAD_FILE, "r", encoding="utf-8") as f:
                script_retry += f.read()
            res = run_simulation(script_retry)
            log = _get_log_text(res) if res else ""
        print(f"\nSimulation Complete!\n{log}")
    else:
        print("No response from MCP – simulation may have failed.")

    print("Done.")