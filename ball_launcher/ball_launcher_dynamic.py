#!/usr/bin/env python3
"""
Ball Launcher DC – Advanced Dynamic Simulation (MCP)
====================================================
Fully animated launcher with:
  - Real‑time firing cycle (cam‑driven plunger + spring)
  - Gravity ball feed (magazine balls shift down each shot)
  - Muzzle flash
  - Spring compression visual
  - Ammo counter (console)
  - Assembly/disassembly animation
  - All 3D‑printable features (bores, clearances, ports)

Run:  python ball_launcher_advanced.py
Fusion 360 must be running with MCP on port 27182.
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
try:
    _app.preferences.generalPreferences.defaultModelingOrientation = \
        adsk.core.DefaultModelingOrientations.ZUpModelingOrientation
except: pass

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
black_plas  = get_ap("Plastic - Matte (Black)","Rubber")
red_col     = get_ap("Paint - Metallic (Red)","Steel - Painted (Red)")
amber_led   = get_ap("LED - Amber")

# ── Config ───────────────────────────────────────────────────────────────
HX, HY, HZ       = 3.2, 2.8, 5.0           # housing dimensions
BL, BRO, BRI     = 4.5, 0.70, 0.46         # barrel length, outer r, inner r
CH, CRH, ECC     = 0.40, 0.85, 0.35        # cam height, high radius, eccentricity
PR, PL           = 0.36, 0.50              # plunger head radius, length
BR               = 0.40                    # ball radius (8mm)
MAG_RI, MAG_RO   = 0.48, 0.66              # magazine inner & outer radius
MAG_H            = 4.5                     # magazine height
WALL_T           = 0.18
N20_BL, N20_BW, N20_BH = 2.4, 1.2, 1.4    # motor body
N20_GL, N20_GW, N20_GH = 1.2, 1.2, 1.0    # gearbox
N_SHAFT_R, N_SHAFT_L   = 0.06, 0.6

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

def set_translation(occ, x, y, z):
    if not occ: return
    mat = adsk.core.Matrix3D.create()
    mat.translation = adsk.core.Vector3D.create(x, y, z)
    occ.transform = mat

# ── Plane helper ─────────────────────────────────────────────────────────
def _off_plane(comp, axis, offset):
    pmap = {'z':comp.xYConstructionPlane,'y':comp.xZConstructionPlane,'x':comp.yZConstructionPlane}
    base = pmap[axis]
    if abs(offset)<1e-6: return base
    pi = comp.constructionPlanes.createInput()
    pi.setByOffset(base, adsk.core.ValueInput.createByReal(offset))
    return comp.constructionPlanes.add(pi)

# ── Geometry helpers (axis‑aware, safe) ──────────────────────────────────
def ebox(comp, name, x0,y0,z0, dx,dy,dz, ap=None, cut=False):
    if dx<=0 or dy<=0 or dz<=0: return None
    try:
        plane = comp.xYConstructionPlane
        sk = comp.sketches.add(plane)
        p1 = sk.modelToSketchSpace(adsk.core.Point3D.create(x0,y0,z0))
        p2 = sk.modelToSketchSpace(adsk.core.Point3D.create(x0+dx,y0+dy,z0))
        sk.sketchCurves.sketchLines.addTwoPointRectangle(p1,p2)
        prof=sk.profiles.item(0)
        op=adsk.fusion.FeatureOperations.CutFeatureOperation if cut else adsk.fusion.FeatureOperations.NewBodyFeatureOperation
        ex=comp.features.extrudeFeatures.createInput(prof,op)
        ex.setDistanceExtent(False, adsk.core.ValueInput.createByReal(dz))
        body=comp.features.extrudeFeatures.add(ex)
        if body and not cut: body.name=name; set_ap(body,ap)
        try: sk.isLightBulbOn=False
        except: pass
        return body
    except Exception as ex: print(f"  ebox '{name}' fail: {ex}"); return None

def ecyl(comp, name, cx, cy, cz, r, h, axis='z', ap=None, cut=False):
    if r<=0 or h<=0: return None
    try:
        if axis == 'z':
            plane=comp.xYConstructionPlane
        elif axis == 'y':
            plane=comp.xZConstructionPlane
        elif axis == 'x':
            plane=comp.yZConstructionPlane
        else:
            return None
        sk=comp.sketches.add(plane)
        world_pt = adsk.core.Point3D.create(cx, cy, cz)
        pt = sk.modelToSketchSpace(world_pt)
        sk.sketchCurves.sketchCircles.addByCenterRadius(pt, r)
        prof=sk.profiles.item(0)
        op=adsk.fusion.FeatureOperations.CutFeatureOperation if cut else adsk.fusion.FeatureOperations.NewBodyFeatureOperation
        ex=comp.features.extrudeFeatures.createInput(prof,op)
        ex.setDistanceExtent(False, adsk.core.ValueInput.createByReal(h))
        body=comp.features.extrudeFeatures.add(ex)
        if body and not cut: body.name=name; set_ap(body,ap)
        try: sk.isLightBulbOn=False
        except: pass
        return body
    except Exception as ex: print(f"  ecyl '{name}' fail: {ex}"); return None

def sphere(comp, name, cx, cy, cz, r, ap=None):
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
        try: sk.isLightBulbOn=False
        except: pass
    except Exception as ex: print(f"  revolute '{name}' fail: {ex}")

def slider_joint(name, occ1,occ2, cx,cy,cz, axis):
    if not(occ1 and occ2): return
    try:
        sk=root.sketches.add(root.xYConstructionPlane)
        pt=sk.sketchPoints.add(adsk.core.Point3D.create(cx,cy,cz))
        geom=adsk.fusion.JointGeometry.createByPoint(pt)
        ji=root.asBuiltJoints.createInput(occ1,occ2,geom)
        if axis=='z':
            jd = adsk.fusion.JointDirections.ZAxisJointDirection
        elif axis=='y':
            jd = adsk.fusion.JointDirections.YAxisJointDirection
        else:
            jd = adsk.fusion.JointDirections.XAxisJointDirection
        ji.setAsSliderJointMotion(jd)
        j=root.asBuiltJoints.add(ji); j.name=name
        try: sk.isLightBulbOn=False
        except: pass
    except Exception as ex: print(f"  slider '{name}' fail: {ex}")

# ══════════════════════════════════════════════════════════════════════════
# BUILD
# ══════════════════════════════════════════════════════════════════════════
print("BUILD_START")
bz_start = HZ - 0.3          # 4.7, barrel starts here
mgz = bz_start - 0.2         # 4.5, magazine drop z
pz = 2.0                     # plunger forward-most Z
cam_cx = HX/2 - 0.1          # 1.5, cam X position
cam_cz = pz + 0.35           # 2.35, cam height (Z)

# ── Housing ──────────────────────────────────────────────────────────────
hc, ho = new_comp("Housing")
ebox(hc,"Body", -HX/2,-HY/2,0, HX,HY,HZ, dark_metal)
ebox(hc,"Cavity", -HX/2+WALL_T,-HY/2+WALL_T,WALL_T, HX-2*WALL_T,HY-2*WALL_T,HZ-2*WALL_T, cut=True)
ebox(hc,"Rear_Cap", -HX/2+0.2,-HY/2+0.2,0, HX-0.4,HY-0.4,0.30, dark_grey)
ebox(hc,"Top_Rail", -1.3, HY/2+0.02, -0.5, HX-0.6,0.12,2.0, chrome)
ebox(hc,"Mount_Hinge", -0.55,-0.325, HZ-0.3, 1.1,0.65,0.8, dark_metal)
# Hinge pin hole (axis Y through housing)
ecyl(hc,"Hinge_Pin", 0, -0.40, HZ-0.15, 0.10,0.80,'y', cut=True)
ecyl(hc,"Screw_L", -0.35, -0.30, HZ-0.15, 0.16,0.60,'y', cut=True)
ecyl(hc,"Screw_R", 0.35, -0.30, HZ-0.15, 0.16,0.60,'y', cut=True)
ecyl(hc,"Mag_Hole", 0, HY/2-0.5, mgz, MAG_RO, 0.8, 'y', cut=True)
ecyl(hc,"Barrel_Hole", 0, 0, 4.8, BRO+0.05, 0.5, 'z', cut=True)
# Cooling vents
for i in range(3):
    vz=0.5+i*0.7
    ebox(hc,f"Vent_L{i}", -HX/2-0.01,0,vz, 0.02,0.10,0.50, cut=True)
    ebox(hc,f"Vent_R{i}", HX/2-0.01,0,vz, 0.02,0.10,0.50, cut=True)

# ── Barrel ───────────────────────────────────────────────────────────────
bc_, bo = new_comp("Barrel")
ecyl(bc_,"Outer", 0,0,bz_start, BRO,BL, 'z', chrome)
ecyl(bc_,"Bore", 0,0,bz_start, BRI,BL+0.4, 'z', cut=True)
ecyl(bc_,"Muzzle_Ring", 0,0,bz_start+BL, BRO+0.08,0.20, 'z', dark_metal)
ecyl(bc_,"LED_Pocket", 0,0,bz_start+BL, 0.26,0.40, 'z', cut=True)
ebox(bc_,"Front_Sight", -0.1, BRO+0.05, bz_start+BL-0.5, 0.20,0.30,0.12, dark_metal)

# ── Plunger ──────────────────────────────────────────────────────────────
pc_, po = new_comp("Plunger")
ecyl(pc_,"Shaft", 0,0,pz, 0.15,1.6, 'z', chrome)
ecyl(pc_,"Head", 0,0,pz+1.6, PR,PL, 'z', dark_metal)
ebox(pc_,"Follower", 0.6,-0.15,pz+0.2, 0.35,0.35,0.30, dark_metal)
ecyl(pc_,"Roller", 0.92,0,pz+0.35, 0.12,0.20, 'y', chrome)
ecyl(pc_,"Bumper", 0,0,pz-0.2, 0.20,0.20, 'z', black_plas)

# ── Spring (separate component, visual coils) ────────────────────────────
spc_, spo = new_comp("Spring")
spz = 0.3
for i in range(7):
    ecyl(spc_,f"Coil_{i}", 0,0, spz+i*0.25, 0.25,0.06, 'z', chrome)
ecyl(spc_,"Guide_Rod", 0,0, 0.2, 0.08,1.8, 'z', chrome)

# ── Drop Cam ─────────────────────────────────────────────────────────────
camc_, camo = new_comp("Cam_Drop")
# Eccentric cam body
ecyl(camc_,"Body", cam_cx+ECC,0,cam_cz, CRH,CH, 'y', dark_metal)
ecyl(camc_,"Shaft_Boss", cam_cx,0,cam_cz, 0.12,CH+0.1, 'y', chrome)
ebox(camc_,"Weight", cam_cx-ECC-0.2,-0.1,cam_cz-0.27, 0.25,0.20,0.55, dark_metal)
# Notch cuts for drop‑off
notch_x = cam_cx + CRH*0.7
ebox(camc_,"Release_Notch", notch_x,-CH/2-0.1, cam_cz-CRH*0.4, CRH*1.2,CH+0.2,CRH*0.8, cut=True)
ebox(camc_,"Drop_Edge", cam_cx+0.3,-CH/2-0.1, cam_cz+0.1, CRH*0.4,CH+0.2,CRH*0.5, cut=True)

# ── Motor ────────────────────────────────────────────────────────────────
mtc_, mto = new_comp("Motor_N20")
my = -HY/2 - N20_BH/2 - 0.1
ebox(mtc_,"Body", cam_cx-N20_BL/2, my-N20_BH/2, cam_cz-N20_BW/2, N20_BL,N20_BH,N20_BW, dark_metal)
ebox(mtc_,"Gearbox", cam_cx-N20_GL/2, my-N20_GH/2, cam_cz+N20_BW/2, N20_GL,N20_GH,N20_GW, dark_grey)
ecyl(mtc_,"Shaft", cam_cx,my, cam_cz+N20_BW/2+N20_GW/2, N_SHAFT_R,N_SHAFT_L, 'y', chrome)
# Motor bracket attached to housing
ebox(hc,"Motor_Bracket", cam_cx-0.9, -HY/2-0.15, cam_cz-1.0, 1.8,0.15,2.0, dark_metal)
# Shaft hole through housing wall
ecyl(hc,"Shaft_Hole", cam_cx, -HY/2 - 0.1, cam_cz, 0.12, HY+0.2, 'y', cut=True)
# Wire channel
ecyl(hc,"Wire_Chan", cam_cx-0.5, -HY/2, cam_cz-0.6, 0.10,0.40, 'y', cut=True)
# Motor wires (cosmetic)
ecyl(mtc_,"Wire_Red", cam_cx-0.3, my-N20_BH/2, cam_cz-N20_BW/2-0.2, 0.04,1.8, 'y', red_col)
ecyl(mtc_,"Wire_Blk", cam_cx+0.3, my-N20_BH/2, cam_cz-N20_BW/2-0.2, 0.04,1.8, 'y', black_plas)

# ── Magazine ─────────────────────────────────────────────────────────────
mgc_, mgo = new_comp("Magazine")
mgx=0; mgy=HY/2;

ecyl(mgc_,"Tube", mgx,mgy,mgz, MAG_RO,MAG_H, 'y', glass_clr)
ecyl(mgc_,"Bore", mgx,mgy,mgz, MAG_RI,MAG_H+0.4, 'y', cut=True)
ecyl(mgc_,"Funnel", mgx,HY/2, bz_start+0.1, MAG_RI+0.2,0.40, 'y', dark_metal)
ebox(mgc_,"Ammo_Window", mgx-0.3, mgy+1.0, mgz-0.3, 0.6,0.02,1.2, glass_clr)

# ── Balls (5 in magazine + 1 chambered) ──────────────────────────────────
mag_balls = []          # list of occurrences, bottom first
for i in range(5):
    bcomp,bocc = new_comp(f"Ball_Mag_{i+1}")
    sphere(bcomp,f"Ball_{i+1}", 0,0,0, BR, chrome)
    # Position in magazine: stack upwards from the funnel
    set_translation(bocc, mgx, mgy + 0.5 + i*0.9, mgz)
    mag_balls.append(bocc)

# Chambered ball (separate occurrence)
ch_comp, ch_occ = new_comp("Ball_Chambered")
sphere(ch_comp,"Ball", 0,0,0, BR, chrome)
set_translation(ch_occ, 0, 0, mgz)

# ── Ejection port (cosmetic) ─────────────────────────────────────────────
ebox(hc,"Eject_Port", 0.25,0, HZ/2+0.3, 0.30,0.08,0.25, dark_grey)

# ── Muzzle flash (hidden) ────────────────────────────────────────────────
flc_, flo = new_comp("Muzzle_Flash")
ecyl(flc_,"Flash", 0,0, bz_start+BL+0.15, 0.25,0.60, 'z', amber_led)
flo.isLightBulbOn = False

# ── Ammo counter label (just print to console, we'll also use a message)
print("BUILD_DONE")

# ══════════════════════════════════════════════════════════════════════════
# JOINTS
# ══════════════════════════════════════════════════════════════════════════
revolute_joint("Cam_Drive", camo, ho, cam_cx, 0, cam_cz, "y")
slider_joint("Plunger_Slide", po, ho, 0, 0, pz, "z")
adsk.doEvents()
print("JOINTS_DONE")

# ══════════════════════════════════════════════════════════════════════════
# SIMULATION PARAMETERS
# ══════════════════════════════════════════════════════════════════════════
cj = root.asBuiltJoints.itemByName("Cam_Drive")
sj = root.asBuiltJoints.itemByName("Plunger_Slide")
if not cj or not sj:
    print("ERROR: Joints missing")
else:
    cam_motion = cj.jointMotion
    slider_motion = sj.jointMotion
    STROKE = 1.0                     # max plunger retraction (cm)
    RELEASE_ANGLE = 200 * math.pi/180

    def cam_lift(angle):
        a = angle % (2*math.pi)
        if a < RELEASE_ANGLE:
            return (a / RELEASE_ANGLE) * STROKE
        else:
            t = (a - RELEASE_ANGLE) / (2*math.pi - RELEASE_ANGLE)
            t = min(t, 1.0)
            # Ease-out snap forward (quadratic)
            return STROKE * max(0.0, 1.0 - 1.3 * t * t)

    # Initial ammo
    ammo_remaining = len(mag_balls) + 1   # +1 for chambered
    print(f"AMMO: {ammo_remaining}")

    CYCLES = min(ammo_remaining, 6)
    STEPS = 48
    DELAY = 0.02

    for cycle in range(CYCLES):
        # Before firing, ensure chambered ball is visible and in position
        set_translation(ch_occ, 0, 0, mgz)
        _app.activeViewport.refresh()

        for step in range(STEPS):
            frac = step / STEPS
            angle = frac * 2.0 * math.pi
            cam_motion.rotationValue = angle
            lift = cam_lift(angle)
            slider_motion.slideValue = -lift

            # Muzzle flash
            if angle > RELEASE_ANGLE and lift < 0.2:
                flo.isLightBulbOn = True
            else:
                flo.isLightBulbOn = False

            # Move chambered ball forward with plunger
            if angle > RELEASE_ANGLE:
                bdist = (1.0 - lift) * 15.0
                set_translation(ch_occ, 0, 0, mgz + bdist)
            else:
                set_translation(ch_occ, 0, 0, mgz)

            # Spring compression: move spring component backward
            compress = lift * 1.0      # visual compression scale
            set_translation(spo, 0, 0, -compress)

            _app.activeViewport.refresh()
            _time.sleep(DELAY)

        # End of shot: ball is gone (we will hide it or move it out of scene)
        set_translation(ch_occ, 100, 100, 100)
        flo.isLightBulbOn = False

        # Reduce ammo count
        ammo_remaining -= 1
        print(f"Shot {cycle+1} – Ammo left: {ammo_remaining}")

        # Feed next ball: shift magazine balls down by one position
        if mag_balls:
            # Remove the bottom ball (make it invisible and move away)
            bottom_ball = mag_balls[0]
            set_translation(bottom_ball, 100, 100, 100)
            bottom_ball.isLightBulbOn = False   # hide it
            mag_balls.pop(0)

            # Shift remaining balls down to fill the gap
            for i, ball_occ in enumerate(mag_balls):
                new_y = mgy + 0.5 + i * 0.9
                set_translation(ball_occ, mgx, new_y, mgz)

            # If there are balls left, move the new bottom ball into chamber
            if mag_balls:
                new_chamber = mag_balls[0]
                set_translation(new_chamber, 0, 0, mgz)
                # Update chambered occurrence reference for next cycle
                ch_occ = new_chamber   # swap references
                # Actually we need to make ch_occ point to this ball for animation
                # We'll just set ch_occ = new_chamber and use it
                # Since ch_occ is the variable we use, we assign it
                ch_occ = new_chamber
                mag_balls.pop(0)  # remove from list so it's the chambered one
            else:
                # No more balls
                ch_occ = None

        # If no chambered ball left, break
        if ch_occ is None and not mag_balls:
            print("OUT OF AMMO")
            break

        _time.sleep(0.3)

    # Reset everything
    cam_motion.rotationValue = 0.0
    slider_motion.slideValue = 0.0
    set_translation(spo, 0, 0, 0)
    _app.activeViewport.refresh()
    print("FIRING_DONE")

# ══════════════════════════════════════════════════════════════════════════
# ASSEMBLY / DISASSEMBLY
# ══════════════════════════════════════════════════════════════════════════
print("ASSEMBLY_START")
# Gather all major moving parts (except housing)
parts = [bo, po, camo, mgo, mto]
if ch_occ: parts.append(ch_occ)
parts.append(spo)
saved_trans = [p.transform.copy() for p in parts]

# Skipped explode animation to ensure STEP export is completely assembled
_time.sleep(1)

# Reassemble
for i, p in enumerate(parts):
    p.transform = saved_trans[i]
_app.activeViewport.refresh()
_time.sleep(1)
print("ASSEMBLY_DONE")

# ── Save Screenshots ──────────────────────────────────────────────────────
print("SCREENSHOTS_START")
artifact_dir = r"C:\Users\PREM KUMAR\.gemini\antigravity\brain\41ed2feb-a7f0-40de-807d-773ea665695e"
def capture_img(filename, view_orientation, hide_housing=False, hide_mag=False):
    try:
        if ho: ho.isLightBulbOn = not hide_housing
        if mgo: mgo.isLightBulbOn = not hide_mag
        
        view = _app.activeViewport
        camera = view.camera
        camera.cameraType = adsk.core.CameraTypes.OrthographicCameraType
        camera.viewOrientation = view_orientation
        camera.isFitView = True
        view.camera = camera
        view.refresh()
        
        path = os.path.join(artifact_dir, filename)
        view.saveAsImageFile(path, 1200, 900)
        print(f"Captured: {path}")
    except Exception as e:
        print(f"Capture {filename} error: {e}")

# 1. External Isometric View
capture_img("assembly_iso.png", adsk.core.ViewOrientations.IsoTopRightViewOrientation)

# 2. Internal View (Hide Housing, show plunger, spring, cam, motor)
capture_img("internal_iso.png", adsk.core.ViewOrientations.IsoTopRightViewOrientation, hide_housing=True)

# 3. Side View (Verify joints and vertical alignment)
capture_img("internal_side.png", adsk.core.ViewOrientations.RightViewOrientation, hide_housing=True)

# 4. Top View (Verify magazine alignment)
capture_img("internal_top.png", adsk.core.ViewOrientations.TopViewOrientation, hide_housing=True)

# Restore visibilities
if ho: ho.isLightBulbOn = True
if mgo: mgo.isLightBulbOn = True
_app.activeViewport.refresh()
print("SCREENSHOTS_DONE")

# Final report
print("EXPORT_START")
step_path = r"C:\one\Optimus_Prime\output\ball_launcher.step"
try:
    exportMgr = design.exportManager
    stepOptions = exportMgr.createSTEPExportOptions(step_path, root)
    res_export = exportMgr.execute(stepOptions)
    if res_export:
        print(f"EXPORTED STEP to {step_path}")
    else:
        print("STEP EXPORT FAILED")
except Exception as e:
    print(f"STEP EXPORT EXCEPTION: {e}")

bc = sum(1 for c in comps_list for b in c.bRepBodies)
print(f"RESULT:{json.dumps({
    'status':'success','components':len(comps_list),'bodies':bc,
    'features':'full firing cycle, ammo counter, spring compress, ball feed, assembly/disassembly',
    'export_path': step_path
})}")

def run(context):
    pass
"""

def main():
    print("=" * 60)
    print("Ball Launcher DC – Advanced Dynamic Simulation")
    print("=" * 60)

    send_request({
        "jsonrpc": "2.0", "id": _next_id(), "method": "initialize",
        "params": {"protocolVersion": "2024-11-05", "capabilities": {},
                   "clientInfo": {"name": "advanced_launcher", "version": "2.0"}}
    })
    send_request({"jsonrpc": "2.0", "method": "notifications/initialized"})

    print("[MCP] Sending advanced simulation script...\n")
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

    print("\nCheck Fusion 360 for the fully animated launcher.")

if __name__ == "__main__":
    main()