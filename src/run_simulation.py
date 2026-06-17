import urllib.request
import json
import os
import sys
import time
import subprocess

URL = "http://127.0.0.1:27182/mcp"
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_DIR = os.path.join(os.path.dirname(BASE_DIR), "output")
PAYLOAD_FILE = os.path.join(BASE_DIR, "optimus_prime_g1_v9.py")
session_id = None

def send_request(payload):
    global session_id
    headers = {'Content-Type': 'application/json'}
    if session_id:
        headers['MCP-Session-Id'] = session_id
    req = urllib.request.Request(URL, data=json.dumps(payload).encode('utf-8'), headers=headers)
    try:
        with urllib.request.urlopen(req) as response:
            if 'MCP-Session-Id' in response.headers:
                session_id = response.headers.get('MCP-Session-Id')
            resp_text = response.read().decode('utf-8')
            if resp_text:
                return json.loads(resp_text)
            return {}
    except urllib.error.HTTPError as e:
        body = e.read().decode('utf-8')
        print(f"HTTP Error {e.code}: {body}")
        return None
    except Exception as e:
        print(f"Connection refused: {e}")
        return None

def call_tool(name, args, tool_id=2):
    return send_request({
        "jsonrpc": "2.0", "id": tool_id, "method": "tools/call",
        "params": {"name": name, "arguments": args}
    })

def send_escape_to_fusion():
    """Use .NET to send Escape repeatedly to the Fusion 360 window."""
    try:
        ps = '''
        Add-Type @"
            using System;
            using System.Runtime.InteropServices;
            using System.Diagnostics;
            public class FusionKiller {
                [DllImport("user32.dll")]
                public static extern bool PostMessage(IntPtr hWnd, uint msg, int wParam, int lParam);
                [DllImport("user32.dll")]
                public static extern int SendMessage(IntPtr hWnd, uint msg, int wParam, int lParam);
                [DllImport("user32.dll")]
                public static extern IntPtr FindWindow(string lpClassName, string lpWindowName);
                [DllImport("user32.dll")]
                public static extern bool EnumWindows(EnumWindowsProc lpEnumFunc, IntPtr lParam);
                [DllImport("user32.dll")]
                public static extern uint GetWindowThreadProcessId(IntPtr hWnd, out int lpdwProcessId);
                public delegate bool EnumWindowsProc(IntPtr hWnd, IntPtr lParam);
                public const uint WM_CLOSE = 0x0010;
                public const uint WM_KEYDOWN = 0x0100;
                public const int VK_ESCAPE = 0x1B;
                
                public static void CloseAllDialogs() {
                    IntPtr f360 = IntPtr.Zero;
                    Process[] procs = Process.GetProcesses();
                    foreach (var p in procs) {
                        if (p.MainWindowTitle.ToLower().Contains("fusion 360") || p.MainWindowTitle.ToLower().Contains("autodesk")) {
                            f360 = p.MainWindowHandle;
                            break;
                        }
                    }
                    if (f360 == IntPtr.Zero) {
                        Console.WriteLine("Fusion 360 window not found");
                        return;
                    }
                    // Send Escape 5 times
                    for (int i = 0; i < 5; i++) {
                        PostMessage(f360, WM_KEYDOWN, VK_ESCAPE, 0);
                        System.Threading.Thread.Sleep(200);
                    }
                    Console.WriteLine("Sent Escape to Fusion 360");
                }
            }
"@
        [FusionKiller]::CloseAllDialogs()
        '''
        result = subprocess.run(["powershell", "-NoProfile", "-Command", ps], 
                                capture_output=True, text=True, timeout=15)
        if result.stdout.strip():
            print(f"  PS: {result.stdout.strip()}")
        if result.stderr.strip():
            print(f"  PS err: {result.stderr.strip()[:200]}")
    except Exception as e:
        print(f"  SendKeys error: {e}")

def run_script(script_content):
    execute_payload = {
        "jsonrpc": "2.0", "id": 3, "method": "tools/call",
        "params": {
            "name": "fusion_mcp_execute",
            "arguments": {"featureType": "script", "object": {"script": script_content}}
        }
    }
    print("====================================================================")
    print("Fusion 360 is now building and simulating the robot.")
    print("CRITICAL: Do NOT click inside Fusion 360 until the simulation completes!")
    print("====================================================================")
    return send_request(execute_payload)

def close_fusion_document():
    """Close the active document via MCP to dismiss dialogs."""
    return call_tool("fusion_mcp_execute", {
        "featureType": "document",
        "object": {"operation": "close"}
    }, tool_id=10)

import argparse

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Optimus Prime G1 Simulation Controller")
    parser.add_argument("--module", type=str, default="ALL", 
                        choices=["ALL", "rom", "head", "wave", "breathing", "walk", "run", "combat", "transform", "truck", "robot", "stability", "servo"])
    parser.add_argument("--capture", action="store_true")
    args = parser.parse_args()

    if not os.path.exists(PAYLOAD_FILE):
        print(f"Payload file not found: {PAYLOAD_FILE}")
        sys.exit(1)

    output_dir = os.path.join(os.path.dirname(BASE_DIR), "output")
    print(f"Loading Optimus Prime payload (> 50KB)... [Target: {args.module}]")
    with open(PAYLOAD_FILE, "r", encoding="utf-8") as f:
        script_content = f"TARGET_MODULE = '{args.module}'\n"
        script_content += f"CAPTURE_SCREENSHOTS = {str(args.capture)}\n"
        script_content += f"EXPORT_DIR = r'{output_dir}\\exports'\n"
        script_content += f"LOG_DIR = r'{output_dir}\\logs'\n"
        script_content += f"SCREENSHOT_DIR = r'{output_dir}\\screenshots'\n"
        script_content += f.read()

    print("Initializing connection to Fusion 360 MCP...")
    send_request({"jsonrpc": "2.0", "id": 1, "method": "initialize",
        "params": {"protocolVersion": "2024-11-05", "capabilities": {},
                   "clientInfo": {"name": "ai_agent", "version": "1.0"}}})
    send_request({"jsonrpc": "2.0", "method": "notifications/initialized"})

    # ── Force-dismiss any dialog ──
    print("Force-dismissing any dialogs in Fusion 360...")
    for attempt in range(3):
        send_escape_to_fusion()
        time.sleep(0.5)

    # ── Try running the script ──
    print("Transmitting Optimus Prime G1 Engine to Fusion 360...")
    res = run_script(script_content)

    if res:
        content = res.get("result", {}).get("content", [])
        text = "".join(c.get("text", "") for c in content if isinstance(c, dict))

        if "Cannot perform" in text or "dialog" in text:
            print("\n[!] Dialog still blocking. Trying harder...")
            for attempt in range(5):
                send_escape_to_fusion()
                time.sleep(0.3)
            close_fusion_document()
            time.sleep(1)
            
            print(f"\nRetry #{1}...")
            res = run_script(script_content)
            if res:
                content2 = res.get("result", {}).get("content", [])
                text2 = "".join(c.get("text", "") for c in content2 if isinstance(c, dict))
                if "Cannot perform" in text2 or "dialog" in text2:
                    print(f"\n[!] Still blocked after retry.")
                    print(f"Response: {json.dumps(res, indent=2)}")
                else:
                    print(f"\nSimulation Complete!")
                    print(json.dumps(res, indent=2))
        else:
            print(f"\nSimulation Complete!")
            print(json.dumps(res, indent=2))
