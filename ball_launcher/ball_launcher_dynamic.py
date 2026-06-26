#!/usr/bin/env python3
"""
Ball Launcher DC — Complete Dynamic Simulation (MCP)
=====================================================
Advanced ball launcher for Fusion 360 with:
  - Full 3D-printable model (all bores, holes, clearances)
  - Revolute cam joint + slider plunger joint
  - Cam-profile-driven firing cycle
  - Gravity magazine feed (balls drop into chamber)
  - Spring compression animation
  - Ammo counter window + sights + ejection port
  - Muzzle flash at firing moment
  - Assembly/disassembly animation

Usage:
    python ball_launcher_dynamic.py
Requires Fusion 360 running with MCP enabled (port 27182).
"""

import urllib.request, json, os, sys, time as _time

MCP_URL = os.environ.get("MCP_URL", "http://127.0.0.1:27182/mcp")
session_id = None
_rpc_id = 0

def _next_id():
    global _rpc_id; _rpc_id += 1; return _rpc_id

def send_request(payload, timeout=300):
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

# ═══════════════════════════════════════════════════════════════════════════
FUSION_SCRIPT = r"""
import adsk.core, adsk.fusion, traceback, math, json, time as _time

_app = adsk.core.Application.get()
_ui  = _app.userInterface

_new_doc = _app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
design = adsk.fusion.Design.cast(_app.activeProduct)
root = design.rootComponent

# ── Appearances ──────────────────────────────────────────────────────────
def get_ap(*names):
    lib = None
    for i in range(_app.materialLibraries.count):
        l = _app.materialLibraries.item(i)
        if "Appearance" in l.name: lib = l; break
    if not lib: return None
    for n in names:
        for j in range(lib.appearances.count):
            ap = lib.appearances.item(j)
            if n.lower() in ap.name.lower():
                try: return design.appearances.addByCopy(ap)
                except: return ap
    return None

chrome      = get_ap("Chrome","Steel - Polished")
dark_metal  = get_ap("Steel - Flat","Plastic - Matte (Black)")
dark_grey   = get_ap("Plastic - Matte (Dark Grey)","Plastic - Matte (Grey)")
glass_clr   = get_ap("Glass - Window","Acrylic - Clear")
black_plas  = get_ap("Plastic - Matte (Black","Rubber")
red_col     = get_ap("Paint - Metallic (Red)","Steel - Painted (Red)")

# ── Config ───────────────────────────────────────────────────────────────
HX, HY, HZ       = 3.2, 2.8, 5.0
BL, BRO, BRI     = 4.5, 0.70, 0.46    # barrel len, outer r, inner r
CH, CRH, ECC     = 0.40, 0.85, 0.35   # cam height, high r, eccentricity
PR, PL           = 0.36, 0.50         # plunger head r, len
BR               = 0.40               # ball radius 8mm
MAG_RI, MAG_RO   = 0.48, 0.66
MAG_H            = 4.5
WALL_T           = 0.18
N20_BL, N20_BW, N20_BH = 2.4, 1.2, 1.4
N20_GL, N20_GW, N20_GH = 1.2, 1.2, 1.0

comps_list = []
occs = {}
def new_comp(name):
    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    comp = occ.component; comp.name = name
    comps_list.append(comp); occs[name] = occ
    return comp, occ

def set_ap(body, ap):
    if body and ap:
        try: body.appearance = ap
        except: pass

# ── Plane helper ─────────────────────────────────────────────────────────
def _off_plane(comp, axis, offset):
    pmap = {'z':comp.xYConstructionPlane,'y':comp.xZConstructionPlane,'x':comp.yZConstructionPlane}
    base = pmap[axis]
    if abs(offset)<1e-6: return base
    pi = comp.constructionPlanes.createInput()
    pi.setByOffset(base, adsk.core.ValueInput.createByReal(offset))
    return comp.constructionPlanes.add(pi)

# ── Extrude helpers ──────────────────────────────────────────────────────
def ebox(comp, name, x0,y0,z0, dx,dy,dz, ap=None, cut=False):
    if dx<=0 or dy<=0 or dz<=0: return None
    try:
        plane = _off_plane(comp,'z',z0)
        sk = comp.sketches.add(plane)
        p1=adsk.core.Point3D.create(x0,y0,0); p2=adsk.core.Point3D.create(x0+dx,y0+dy,0)
        sk.sketchCurves.sketchLines.addTwoPointRectangle(p1,p2)
        prof=sk.profiles.item(0)
        op=adsk.fusion.FeatureOperations.CutFeatureOperation if cut else adsk.fusion.FeatureOperations.NewBodyFeatureOperation
        ex=comp.features.extrudeFeatures.createInput(prof,op)
        ex.setDistanceExtent(False,adsk.core.ValueInput.createByReal(dz))
        body=comp.features.extrudeFeatures.add(ex)
        if body and not cut: body.name=name; set_ap(body,ap)
        try: sk.isVisible=False
        except: pass
        return body
    except Exception as ex: print(f"  ebox '{name}' fail: {ex}"); return None

def ecyl(comp,name, cx,cy,cz, r,h, axis='z', ap=None, cut=False):
    if r<=0 or h<=0: return None
    try:
        plane=_off_plane(comp,axis,cz)
        sk=comp.sketches.add(plane)
        sk.sketchCurves.sketchCircles.addByCenterRadius(adsk.core.Point3D.create(cx,cy,0),r)
        prof=sk.profiles.item(0)
        op=adsk.fusion.FeatureOperations.CutFeatureOperation if cut else adsk.fusion.FeatureOperations.NewBodyFeatureOperation
        ex=comp.features.extrudeFeatures.createInput(prof,op)
        ex.setDistanceExtent(False,adsk.core.ValueInput.createByReal(h))
        body=comp.features.extrudeFeatures.add(ex)
        if body and not cut: body.name=name; set_ap(body,ap)
        try: sk.isVisible=False
        except: pass
        return body
    except Exception as ex: print(f"  ecyl '{name}' fail: {ex}"); return None

def sphere(comp,name, cx,cy,cz, r, ap=None):
    try:
        temp=adsk.fusion.TemporaryBRepManager.get()
        shape=temp.createSphere(adsk.core.Point3D.create(cx,cy,cz),r)
        bf=comp.features.baseFeatures.add(); bf.startEdit()
        body=comp.bRepBodies.add(shape,bf); bf.finishEdit()
        if body: body.name=name; set_ap(body,ap)
        return body
    except Exception as ex: print(f"  sphere '{name}' fail: {ex}"); return None

# ── Joint helpers ────────────────────────────────────────────────────────
def revolute_joint(name, occ1,occ2, cx,cy,cz, axis):
    if not(occ1 and occ2): return
    try:
        sk=root.sketches.add(root.xYConstructionPlane)
        pt=sk.sketchPoints.add(adsk.core.Point3D.create(cx,cy,cz))
        geom=adsk.fusion.JointGeometry.createByPoint(pt)
        ji=root.asBuiltJoints.createInput(occ1,occ2,geom)
        av={"x":adsk.core.Vector3D.create(1,0,0),
            "y":adsk.core.Vector3D.create(0,1,0),
            "z":adsk.core.Vector3D.create(0,0,1)}[axis]
        ji.setAsRevoluteJointMotion(adsk.fusion.JointDirections.CustomJointDirection,av)
        j=root.asBuiltJoints.add(ji); j.name=name
        try: sk.isVisible=False
        except: pass
    except: pass

def slider_joint(name, occ1,occ2, cx,cy,cz, axis):
    if not(occ1 and occ2): return
    try:
        sk=root.sketches.add(root.xYConstructionPlane)
        pt=sk.sketchPoints.add(adsk.core.Point3D.create(cx,cy,cz))
        geom=adsk.fusion.JointGeometry.createByPoint(pt)
        ji=root.asBuiltJoints.createInput(occ1,occ2,geom)
        av=adsk.core.Vector3D.create(0,0,1) if axis=='z' else adsk.core.Vector3D.create(1,0,0)
        nv=adsk.core.Vector3D.create(0,1,0)
        if axis=='y': av=adsk.core.Vector3D.create(0,1,0); nv=adsk.core.Vector3D.create(1,0,0)
        ji.setAsSliderJointMotion(av,nv)
        j=root.asBuiltJoints.add(ji); j.name=name
        try: sk.isVisible=False
        except: pass
    except Exception as ex: print(f"  slider '{name}' fail: {ex}")

# ══════════════════════════════════════════════════════════════════════════
# BUILD
# ══════════════════════════════════════════════════════════════════════════
print("BUILD_START")
bz_start = HZ - 0.3  # barrel Z start

# ── Housing ──────────────────────────────────────────────────────────────
hc, ho = new_comp("Housing")
# Outer body (centered at z=0, Z-positive)
ebox(hc,"Body", -HX/2,-HY/2,0, HX,HY,HZ, dark_metal)
# Interior cavity
ebox(hc,"Cavity", -HX/2+WALL_T,-HY/2+WALL_T, WALL_T, HX-2*WALL_T,HY-2*WALL_T,HZ-2*WALL_T, cut=True)
# Rear cap
ebox(hc,"Rear_Cap", -HX/2+0.2,-HY/2+0.2,0, HX-0.4,HY-0.4,0.30, dark_grey)
# Top rail
ebox(hc,"Top_Rail", -1.3, HY/2+0.02, -0.5, HX-0.6,0.12,2.0, chrome)
# Mounting hinge
ebox(hc,"Mount_Hinge", -0.55,-0.325, HZ-0.3, 1.1,0.65,0.8, dark_metal)
# Hinge pin hole
# For axis='y' cuts: (cx,cy,cz) -> world (cx, cz, cy)
ecyl(hc,"Hinge_Pin", 0, HZ-0.15, 0, 0.10,0.80,'y', cut=True)
# Screw holes
ecyl(hc,"Screw_L", -0.35, HZ-0.15, 0, 0.16,0.60,'y', cut=True)
ecyl(hc,"Screw_R", 0.35, HZ-0.15, 0, 0.16,0.60,'y', cut=True)
# Cooling vents
for i in range(3):
    vz=-0.6+i*0.7
    ebox(hc,f"Vent_L{i}", -HX/2-0.01,0,vz, 0.02,0.10,0.50, cut=True)
    ebox(hc,f"Vent_R{i}", HX/2+0.01,0,vz, 0.02,0.10,0.50, cut=True)

# ── Barrel ───────────────────────────────────────────────────────────────
bc_, bo = new_comp("Barrel")
ecyl(bc_,"Outer", 0,0,bz_start, BRO,BL, 'z', chrome)
ecyl(bc_,"Bore", 0,0,bz_start, BRI,BL+0.4, 'z', cut=True)
ecyl(bc_,"Muzzle_Ring", 0,0,bz_start+BL, BRO+0.08,0.20, 'z', dark_metal)
# Muzzle LED pocket
ecyl(bc_,"LED_Pocket", 0,0,bz_start+BL+0.5, 0.26,0.40, 'z', cut=True)
# Front sight
ebox(bc_,"Front_Sight", -0.1, BRO+0.05, bz_start+BL-0.5, 0.20,0.30,0.12, dark_metal)

# ── Plunger ──────────────────────────────────────────────────────────────
pc_, po = new_comp("Plunger")
pz = 0.6  # plunger Z position inside housing
ecyl(pc_,"Shaft", 0,0,pz, 0.15,1.6, 'z', chrome)
ecyl(pc_,"Head", 0,0,pz+0.3, PR,PL, 'z', dark_metal)
# Follower plate (cam pushes this)
ebox(pc_,"Follower", 0.6,-0.15,pz+0.2, 0.35,0.35,0.30, dark_metal)
ecyl(pc_,"Roller", 0.92,0,pz+0.35, 0.12,0.20, 'y', chrome)
# Bumper
ecyl(pc_,"Bumper", 0,0,pz-0.6, 0.20,0.12, 'z', black_plas)

# ── Spring (visual coils) ────────────────────────────────────────────────
spc_, spo = new_comp("Spring")
spz = pz - 0.8
for i in range(7):
    ecyl(spc_,f"Coil_{i}", 0,0, spz-i*0.3, 0.25,0.06, 'z', chrome)
ecyl(spc_,"Guide_Rod", 0,0, spz-0.9, 0.08,2.4, 'z', chrome)

# ── Drop Cam ─────────────────────────────────────────────────────────────
camc_, camo = new_comp("Cam_Drop")
cx = HX/2 - 0.1; cz = pz + 0.35
# Eccentric cam body (offset from shaft center)
ecyl(camc_,"Body", cx+ECC,0,cz, CRH,CH, 'y', dark_metal)
# Shaft boss (centered)
ecyl(camc_,"Shaft_Boss", cx,0,cz, 0.12,CH+0.1, 'y', chrome)
# Counterweight
ebox(camc_,"Weight", cx-ECC-0.2,-0.1,cz-0.27, 0.25,0.20,0.55, dark_metal)
# Release notch — cut a wedge from the cam
# A box cut that removes material on the low-radius side
notch_x = cx + CRH*0.7  # position along cam radius
ebox(camc_,"Release_Notch", notch_x,-CH/2-0.1,cz-CRH*0.4, CRH*1.2,CH+0.2,CRH*0.8, cut=True)
# Second cut for drop-edge profile
ebox(camc_,"Drop_Edge", cx+0.3,-CH/2-0.1,cz+0.1, CRH*0.4,CH+0.2,CRH*0.5, cut=True)

# ── Motor ────────────────────────────────────────────────────────────────
mtc_, mto = new_comp("Motor_N20")
my = -HY/2 - N20_BH/2 - 0.1
ebox(mtc_,"Body", cx-N20_BL/2, my-N20_BH/2, cz-N20_BW/2, N20_BL,N20_BH,N20_BW, dark_metal)
ebox(mtc_,"Gearbox", cx-N20_GL/2, my-N20_GH/2, cz+N20_BW/2, N20_GL,N20_GH,N20_GW, dark_grey)
ecyl(mtc_,"Shaft", cx,my, cz+N20_BW/2+N20_GW/2, 0.06,0.6, 'y', chrome)
# Motor bracket
ebox(hc,"Motor_Bracket", cx-0.9, -HY/2-0.15, cz-1.0, 1.8,0.15,2.0, dark_metal)
# Shaft pass-through hole
ecyl(hc,"Shaft_Hole", cx, cz, -HY/2, 0.12, HY+0.2, 'y', cut=True)
# Wire channel
ecyl(hc,"Wire_Chan", cx-0.5, cz-0.6, -HY/2, 0.10,0.40, 'y', cut=True)
# Wire visual
ecyl(mtc_,"Wire_Red", cx-0.3, my-N20_BH/2, cz-N20_BW/2-0.2, 0.04,1.8, 'y', red_col)
ecyl(mtc_,"Wire_Blk", cx+0.3, my-N20_BH/2, cz-N20_BW/2-0.2, 0.04,1.8, 'y', black_plas)

# ── Magazine ─────────────────────────────────────────────────────────────
mgc_, mgo = new_comp("Magazine")
mgx=0; mgy=HY/2; mgz=bz_start-0.2
# Outer clear tube
ecyl(mgc_,"Tube", mgx,mgy,mgz, MAG_RO,MAG_H, 'y', glass_clr)
# Inner bore
ecyl(mgc_,"Bore", mgx,mgy,mgz, MAG_RI,MAG_H+0.4, 'y', cut=True)
# Feed funnel
ecyl(mgc_,"Funnel", mgx,HY/2, bz_start+0.1, MAG_RI+0.2,0.40, 'y', dark_metal)
# Ammo window (transparent slot)
ebox(mgc_,"Ammo_Window", mgx-0.3, mgy+1.0, mgz-0.3, 0.6,0.02,1.2, glass_clr)

# ── Balls ────────────────────────────────────────────────────────────────
# Magazine balls (5)
ball_occs = []
for i in range(5):
    bcomp,bocc = new_comp(f"Ball_{i+1}")
    sphere(bcomp,"Ball", 0,0,0, BR, chrome)
    bocc.transform = adsk.core.Matrix3D.create()
    bocc.transform.translation = adsk.core.Vector3D.create(
        mgx, mgy + 0.5 + i*0.9, mgz)
    ball_occs.append(bocc)
# Chambered ball
ch_comp, ch_occ = new_comp("Ball_Chambered")
sphere(ch_comp,"Ball", 0,0,0, BR, chrome)
ch_occ.transform = adsk.core.Matrix3D.create()
ch_occ.transform.translation = adsk.core.Vector3D.create(0, 0, HZ/2 + 0.15)

# Store initial ball positions for feed animation
ball_positions = []
for i,bocc in enumerate(ball_occs):
    t = bocc.transform
    ball_positions.append((
        t.translation.x, t.translation.y, t.translation.z))

# ── Ejection port ────────────────────────────────────────────────────────
# Simulate a cut in the top-rear of the barrel area
ebox(hc,"Eject_Port", 0.25,0, HZ/2+0.3, 0.30,0.08,0.25, dark_grey)

# ── Muzzle flash (cosmetic LED, initially hidden) ────────────────────────
flc_, flo = new_comp("Muzzle_Flash")
ecyl(flc_,"Flash", 0,0, bz_start+BL+0.15, 0.25,0.60, 'z', red_col)
flo.isVisible = False

print("BUILD_DONE")

# ══════════════════════════════════════════════════════════════════════════
# JOINTS
# ══════════════════════════════════════════════════════════════════════════
# Cam revolute about Y
revolute_joint("Cam_Drive", camo, ho, cx, 0, cz, "y")
# Plunger slider along Z
slider_joint("Plunger_Slide", po, ho, 0, 0, pz, "z")

adsk.doEvents()
print("JOINTS_DONE")

# ══════════════════════════════════════════════════════════════════════════
# SIMULATION
# ══════════════════════════════════════════════════════════════════════════
cj = root.asBuiltJoints.itemByName("Cam_Drive")
sj = root.asBuiltJoints.itemByName("Plunger_Slide")

if not cj or not sj:
    print("ERROR: Joints not found")
else:
    cmo = cj.jointMotion
    smo = sj.jointMotion
    STROKE = 1.0  # plunger retraction distance (cm)
    RELEASE = 200 * math.pi / 180  # cam release angle

    def cam_lift(angle):
        a = angle % (2*math.pi)
        if a < RELEASE:
            return (a / RELEASE) * STROKE
        else:
            t = (a - RELEASE) / (2*math.pi - RELEASE)
            t = min(t, 1.0)
            # Ease-out snap forward
            return STROKE * max(0, 1.0 - 1.2*t*t)

    CYCLES = 6
    STEPS = 48
    DELAY = 0.025

    print(f"SIM_START: {CYCLES} cycles")

    for cycle in range(CYCLES):
        for step in range(STEPS):
            frac = step / STEPS
            angle = frac * 2.0 * math.pi
            cmo.rotationValue = angle
            lift = cam_lift(angle)
            smo.slideValue = -lift

            # Muzzle flash at firing moment (when lift drops fast)
            if lift < 0.05:
                flo.isVisible = True
            else:
                flo.isVisible = False

            # Chamber ball: push forward when plunger pushes
            if lift < 0.4:
                bdist = min((0.4 - lift) * 18, 12.0)
                t = ch_occ.transform
                t.translation = adsk.core.Vector3D.create(0, 0, HZ/2 + 0.15 + bdist)
                ch_occ.transform = t
            else:
                t = ch_occ.transform
                t.translation = adsk.core.Vector3D.create(0, 0, HZ/2 + 0.15)
                ch_occ.transform = t

            # Spring visual: compress coils
            # Use the spo (Spring occurrence) transform to visually compress
            if lift > 0.1:
                compress = lift * 0.25
                st = spo.transform
                st.translation = adsk.core.Vector3D.create(0, 0, -compress)
                spo.transform = st

            _app.activeViewport.refresh()
            _time.sleep(DELAY)

        # After shot: drop next ball from magazine
        if cycle < CYCLES - 1 and len(ball_occs) > 0:
            # Move lowest magazine ball to chamber position
            bocc = ball_occs[-1]  # bottom ball
            bt = bocc.transform
            # Move it into chamber
            bt.translation = adsk.core.Vector3D.create(0, 0, HZ/2 + 0.15)
            bocc.transform = bt
            ball_occs.pop()

        print(f"  Shot {cycle+1}/{CYCLES}")

    # Reset
    cmo.rotationValue = 0.0
    smo.slideValue = 0.0
    flo.isVisible = False
    st = spo.transform; st.translation = adsk.core.Vector3D.create(0,0,0); spo.transform = st
    _app.activeViewport.refresh()
    print("FIRING_DONE")

# ══════════════════════════════════════════════════════════════════════════
# ASSEMBLY / DISASSEMBLY (explode and return)
# ══════════════════════════════════════════════════════════════════════════
print("ASSEMBLY_START")
parts = [bo, po, camo, mgo, mto, ch_occ, spo]
saved = []
for p in parts:
    saved.append(p.transform.copy())

# Explode
for i, p in enumerate(parts):
    t = p.transform.copy()
    v = adsk.core.Vector3D.create(i*0.8, i*0.5, i*1.2)
    t.translation = adsk.core.Point3D.create(
        t.translation.x + v.x,
        t.translation.y + v.y,
        t.translation.z + v.z).asVector()
    p.transform = t
_app.activeViewport.refresh()
_time.sleep(2)

# Return
for i, p in enumerate(parts):
    p.transform = saved[i]
_app.activeViewport.refresh()
_time.sleep(1)
print("ASSEMBLY_DONE")

# Report
bc = sum(1 for c in comps_list for b in c.bRepBodies)
print(f"RESULT:{json.dumps({
    'status':'success','components':len(comps_list),'bodies':bc,
    'joints':'Cam_Drive + Plunger_Slide','cycles':CYCLES,
    'features':'barrel+muzzle+flash+magazine+spring+cam+notch+ammo+vent+sight+hinge'
})}")
"""

def main():
    print("=" * 60)
    print("Ball Launcher DC — Dynamic Simulation (Advanced)")
    print("=" * 60)

    send_request({
        "jsonrpc": "2.0", "id": _next_id(), "method": "initialize",
        "params": {"protocolVersion": "2024-11-05", "capabilities": {},
                   "clientInfo": {"name": "ball_launcher", "version": "1.0"}}
    })
    send_request({"jsonrpc": "2.0", "method": "notifications/initialized"})

    print("[MCP] Sending build + simulation script...\n")
    res = call_tool("fusion_mcp_execute", {
        "featureType": "script",
        "object": {"script": FUSION_SCRIPT}
    })

    if not res:
        print("ERROR: No response from Fusion 360."); sys.exit(1)

    msg = res.get("result", {}).get("message", str(res))
    print(f"\n[MCP] Response:\n{msg}")

    if "RESULT:" in msg:
        try:
            d = json.loads(msg.split("RESULT:")[-1].strip())
            print("\n"+"="*60+"\nSIMULATION COMPLETE\n"+"="*60)
            for k,v in d.items(): print(f"  {k}: {v}")
        except: pass

    print("\nCheck Fusion 360 for the animated launcher.")

if __name__ == "__main__":
    main()
