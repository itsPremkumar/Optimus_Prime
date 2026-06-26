# =================================================================
# BALL LAUNCHER DC V1 — DC Motor Driven Spring Plunger Mechanism
# for Optimus Prime G1 Robot Arm
#
# A complete working launcher mechanism that fires small ball
# projectiles using a spring-loaded plunger, cocked and released
# by a continuous-rotation N20 micro gearmotor via a drop cam.
#
# Balls (5x, 8mm dia) are stored in an angled gravity-fed magazine
# and load into the firing chamber automatically.
#
# Mechanism cycle (1 motor rotation = 1 shot, full-auto):
#   1. Drop cam rotates — eccentric lobe pushes plunger back,
#      compressing the spring
#   2. At the cam's drop-off edge, plunger snaps forward
#      under spring force
#   3. Plunger strikes the chambered ball → launches out barrel
#   4. Next ball drops in from magazine; cam continues to next cycle
#
# No servo controller needed — just a DC motor + H-bridge driver.
# Run in Fusion 360: Tools > Scripts & Add-Ins > Run
# =================================================================

import adsk.core, adsk.fusion, traceback, math, datetime

_ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
_app = adsk.core.Application.get()
_ui  = _app.userInterface

# ── Configuration ──────────────────────────────────────────────────────
BALL_R       = 0.40    # 8mm ball radius (cm)
N_BALLS      = 5       # magazine capacity
BARREL_R_IN  = 0.46    # barrel inner radius (over ball)
BARREL_R_OUT = 0.70    # barrel outer radius
BARREL_LEN   = 4.5     # barrel length
PLUNGER_R    = 0.36    # plunger head radius
PLUNGER_LEN  = 0.50    # plunger head length
SPRING_OD    = 0.50    # spring coil outer dia
SPRING_LEN   = 2.20    # uncompressed spring length
CAM_R_HIGH   = 0.85    # cam high radius (max push)
CAM_R_LOW    = 0.50    # cam low radius (release)
CAM_H        = 0.40    # cam thickness
ECCENTRICITY = 0.35    # cam center offset from shaft axis
MAG_R        = 0.48    # magazine tube inner radius
MAG_LEN      = 5.5     # magazine tube length
MAG_ANGLE    = 25      # magazine tilt (degrees)
HOUSING_LX   = 3.2     # housing X size
HOUSING_LY   = 2.8     # housing Y size
HOUSING_LZ   = 5.0     # housing Z size
WALL_T       = 0.18    # wall thickness

# N20 micro gearmotor specs (scaled to model)
N20_BODY_L    = 2.4    # motor body length
N20_BODY_W    = 1.2    # motor body width
N20_BODY_H    = 1.4    # motor body height (incl. D-flat)
N20_GB_L      = 1.2    # gearbox length
N20_GB_W      = 1.2    # gearbox width
N20_GB_H      = 1.0    # gearbox height
N20_SHAFT_R   = 0.06   # output shaft radius
N20_SHAFT_L   = 0.6    # output shaft length

# ── Coordinates ────────────────────────────────────────────────────────
AX = 13.0
WRIST_Z = 29.0


def run(context):
    ui = None
    try:
        ui = _ui
        des = adsk.fusion.Design.cast(_app.activeProduct)
        if not des:
            root = _app.activeProduct.rootComponent
        else:
            root = des.rootComponent

        # ── Appearances ──────────────────────────────────────────────
        def get_ap(*names):
            for n in names:
                try:
                    ap = _app.materialLibraries.itemById(
                        "F3D\\Appearance").appearances.itemByName(n)
                    if ap:
                        return ap
                except:
                    pass
            return None

        chrome        = get_ap("Chrome", "Steel - Polished")
        dark_metal    = get_ap("Steel - Flat", "Plastic - Matte (Black)")
        glass_clr     = get_ap("Glass - Window", "Acrylic - Clear")
        grey_plastic  = get_ap("Plastic - Matte (Grey)", "ABS Plastic")
        dark_grey     = get_ap("Plastic - Matte (Dark Grey)",
                               "Plastic - Matte (Grey)")
        black_plastic = get_ap("Plastic - Matte (Black)", "Rubber")
        op_red        = get_ap("Paint - Metallic (Red)",
                               "Steel - Painted (Red)")

        # ── BOM ──────────────────────────────────────────────────────
        bom_rows = []

        def bom_add(cat, part, qty, note=""):
            bom_rows.append({"Category": cat, "Part": part,
                             "Qty": qty, "Note": note})

        # ── Component Registry ───────────────────────────────────────
        comps_list = []
        occs = {}

        def new_component(name):
            occ = root.occurrences.addNewComponent(
                adsk.core.Matrix3D.create())
            comp = occ.component
            comp.name = name
            comps_list.append(comp)
            occs[name] = occ
            return comp, occ

        def set_ap(body, ap):
            if body and ap:
                try:
                    body.appearance = ap
                except:
                    pass

        def _axis_vec(axis):
            return {"x": (1, 0, 0), "y": (0, 1, 0), "z": (0, 0, 1)}[axis]

        # ── Geometry Primitives ──────────────────────────────────────
        def box(comp, name, cx, cy, cz, lx, ly, lz, ap=None):
            if lx <= 0 or ly <= 0 or lz <= 0:
                return None
            try:
                temp = adsk.fusion.TemporaryBRepManager.get()
                obb = adsk.core.OrientedBoundingBox3D.create(
                    adsk.core.Point3D.create(cx, cy, cz),
                    adsk.core.Vector3D.create(1, 0, 0),
                    adsk.core.Vector3D.create(0, 1, 0),
                    lx, ly, lz)
                shape = temp.createBox(obb)
                bf = comp.features.baseFeatures.add()
                bf.startEdit()
                body = comp.bRepBodies.add(shape, bf)
                bf.finishEdit()
                if body:
                    body.name = name
                    set_ap(body, ap)
                return body
            except:
                return None

        def cyl(comp, name, cx, cy, cz, r, h, axis, ap=None):
            if r <= 0 or h <= 0:
                return None
            try:
                temp = adsk.fusion.TemporaryBRepManager.get()
                av = _axis_vec(axis)
                p1 = adsk.core.Point3D.create(
                    cx - av[0]*h/2, cy - av[1]*h/2, cz - av[2]*h/2)
                p2 = adsk.core.Point3D.create(
                    cx + av[0]*h/2, cy + av[1]*h/2, cz + av[2]*h/2)
                shape = temp.createCylinderOrCone(p1, r, p2, r)
                bf = comp.features.baseFeatures.add()
                bf.startEdit()
                body = comp.bRepBodies.add(shape, bf)
                bf.finishEdit()
                if body:
                    body.name = name
                    set_ap(body, ap)
                return body
            except:
                return None

        def sphere(comp, name, cx, cy, cz, r, ap=None):
            if r <= 0:
                return None
            try:
                temp = adsk.fusion.TemporaryBRepManager.get()
                center = adsk.core.Point3D.create(cx, cy, cz)
                shape = temp.createSphere(center, r)
                bf = comp.features.baseFeatures.add()
                bf.startEdit()
                body = comp.bRepBodies.add(shape, bf)
                bf.finishEdit()
                if body:
                    body.name = name
                    set_ap(body, ap)
                return body
            except:
                return None

        def cone_shape(comp, name, cx, cy, cz, r1, r2, h, axis, ap=None):
            if r1 < 0 or r2 < 0 or h <= 0:
                return None
            try:
                temp = adsk.fusion.TemporaryBRepManager.get()
                av = _axis_vec(axis)
                p1 = adsk.core.Point3D.create(
                    cx - av[0]*h/2, cy - av[1]*h/2, cz - av[2]*h/2)
                p2 = adsk.core.Point3D.create(
                    cx + av[0]*h/2, cy + av[1]*h/2, cz + av[2]*h/2)
                shape = temp.createCylinderOrCone(p1, r1, p2, r2)
                bf = comp.features.baseFeatures.add()
                bf.startEdit()
                body = comp.bRepBodies.add(shape, bf)
                bf.finishEdit()
                if body:
                    body.name = name
                    set_ap(body, ap)
                return body
            except:
                return None

        def cut_cavity(comp, tool_body):
            if not tool_body:
                return
            try:
                target = None
                for b in comp.bRepBodies:
                    if b.isSolid and b != tool_body:
                        target = b
                        break
                if target:
                    comp.features.removeFeatures.add(tool_body)
            except:
                pass

        # ── Joint Helpers ────────────────────────────────────────────
        def _make_joint_geometry(cx, cy, cz):
            sketch = root.sketches.add(root.xYConstructionPlane)
            s_pt = sketch.sketchPoints.add(
                adsk.core.Point3D.create(cx, cy, cz))
            geom = adsk.fusion.JointGeometry.createByPoint(s_pt)
            try:
                sketch.isVisible = False
            except:
                pass
            return geom

        def revolute_joint(name, occ1, occ2, cx, cy, cz, axis_str):
            if not (occ1 and occ2):
                return
            try:
                geom = _make_joint_geometry(cx, cy, cz)
                ji = root.asBuiltJoints.createInput(occ1, occ2, geom)
                av = {"x": adsk.core.Vector3D.create(1, 0, 0),
                      "y": adsk.core.Vector3D.create(0, 1, 0),
                      "z": adsk.core.Vector3D.create(0, 0, 1)}[axis_str]
                ji.setAsRevoluteJointMotion(
                    adsk.fusion.JointDirections.CustomJointDirection, av)
                j = root.asBuiltJoints.add(ji)
                j.name = name
            except:
                pass

        # ══════════════════════════════════════════════════════════════
        # BUILD — DC BALL LAUNCHER
        # ══════════════════════════════════════════════════════════════
        #
        # Coordinates (local):
        #   Origin   = mount point on hand
        #   +X       = right
        #   +Y       = up
        #   +Z       = forward (muzzle)
        #   Barrel   = +Z
        #   Magazine = +Y side, angled back
        #   Motor    = below housing (-Y), shaft up into housing
        #   Cam      = inside housing, motor shaft rotates it about Y
        # ══════════════════════════════════════════════════════════════

        launch_comp, launch_occ = new_component("OP_Ball_Launcher_DC")
        cam_comp, cam_occ = new_component("Cam_Drop")

        # ── 1. Barrel ────────────────────────────────────────────────
        bz_start = HOUSING_LZ / 2 - 0.3
        bz_end   = bz_start + BARREL_LEN
        bz_mid   = bz_start + BARREL_LEN / 2

        cyl(launch_comp, "Barrel_Outer", 0, 0, bz_mid,
            BARREL_R_OUT, BARREL_LEN + 0.2, "z", chrome)
        bore = cyl(launch_comp, "Barrel_Bore", 0, 0, bz_mid,
                   BARREL_R_IN, BARREL_LEN + 0.4, "z", None)
        cut_cavity(launch_comp, bore)

        cyl(launch_comp, "Muzzle_Ring", 0, 0, bz_end + 0.1,
            BARREL_R_OUT + 0.08, 0.20, "z", dark_metal)

        # Muzzle LED
        lp = cyl(launch_comp, "Muzzle_LED", 0, 0, bz_end + 0.5,
                 0.26, 0.40, "z", None)
        cut_cavity(launch_comp, lp)
        bom_add("Electronics", "LED 5mm (muzzle flash)", 1,
                "OP_Ball_Launcher_DC")

        # ── 2. Housing ───────────────────────────────────────────────
        box(launch_comp, "Housing_Main", 0, 0, 0,
            HOUSING_LX, HOUSING_LY, HOUSING_LZ, dark_metal)

        interior = box(launch_comp, "Housing_Int", 0, 0, 0,
                       HOUSING_LX - 2*WALL_T,
                       HOUSING_LY - 2*WALL_T,
                       HOUSING_LZ - 2*WALL_T, None)
        cut_cavity(launch_comp, interior)

        # Rear access panel
        box(launch_comp, "Rear_Cap", 0, 0, -HOUSING_LZ/2 + 0.15,
            HOUSING_LX - 0.4, HOUSING_LY - 0.4, 0.30, dark_grey)

        # Housing top rail
        box(launch_comp, "Top_Rail", 0, HOUSING_LY/2 + 0.02, -0.5,
            HOUSING_LX - 0.6, 0.12, 2.0, chrome)

        # Side cooling vents
        for i in range(3):
            vz = -0.6 + i*0.7
            v = box(launch_comp, f"Vent_L{i}", -HOUSING_LX/2 - 0.01,
                    0, vz, 0.02, 0.10, 0.50, None)
            cut_cavity(launch_comp, v)
            v = box(launch_comp, f"Vent_R{i}", HOUSING_LX/2 + 0.01,
                    0, vz, 0.02, 0.10, 0.50, None)
            cut_cavity(launch_comp, v)

        # Internal guide ribs for plunger
        for i in [-1, 1]:
            box(launch_comp, f"Guide_Rib_{i}", i*0.3, 0, -0.5,
                0.08, 0.6, 3.5, dark_metal)

        # ── 3. Plunger Assembly ──────────────────────────────────────
        # Plunger rides inside housing along Z
        p_cz = -HOUSING_LZ/2 + 0.6

        # Plunger head (strikes ball)
        cyl(launch_comp, "Plunger_Head", 0, 0, p_cz + 0.3,
            PLUNGER_R, PLUNGER_LEN, "z", dark_metal)

        # Plunger shaft
        cyl(launch_comp, "Plunger_Shaft", 0, 0, p_cz,
            0.15, 1.6, "z", chrome)

        # Follower plate (rides on cam surface — flat face)
        box(launch_comp, "Plunger_Follower", 0.75, 0, p_cz + 0.2,
            0.35, 0.35, 0.30, dark_metal)

        # Follower roller (rounded contact surface for cam)
        cyl(launch_comp, "Follower_Roller", 0.92, 0, p_cz + 0.35,
            0.12, 0.20, "y", chrome)

        # Plunger rear bumper
        cyl(launch_comp, "Plunger_Bumper", 0, 0, p_cz - 0.6,
            0.20, 0.12, "z", black_plastic)
        bom_add("Hardware", "Foam bumper pad (plunger)", 1,
                "OP_Ball_Launcher_DC")

        # ── 4. Spring ────────────────────────────────────────────────
        s_cz = p_cz - 0.8
        for i in range(6):
            cyl(launch_comp, f"Spring_Coil_{i}", 0, 0,
                s_cz - i*0.32, SPRING_OD/2, 0.06, "z", chrome)

        cyl(launch_comp, "Spring_Guide", 0, 0, s_cz - 0.8,
            0.08, 2.4, "z", chrome)
        bom_add("Hardware", "Compression spring O5mm x 22mm", 1,
                "OP_Ball_Launcher_DC")

        # ── 5. Drop Cam (sub-component, rotates continuously) ────────
        # Snail-profile cam: high radius pushes plunger back,
        # sharp drop-off releases it.  Motor spins continuously.
        cam_cx = HOUSING_LX/2 - 0.1
        cam_cz = p_cz + 0.35
        ecc_x = ECCENTRICITY  # cam center offset from shaft

        # Base cam cylinder (eccentric — center offset from shaft axis)
        cyl(cam_comp, "Cam_Body", cam_cx + ecc_x, 0, cam_cz,
            CAM_R_HIGH, CAM_H, "y", dark_metal)

        # Cutaway to create the low-side / release notch
        # A box cut through the cam at the low-side position
        notch = box(cam_comp, "Cam_Release_Notch",
                    cam_cx - CAM_R_HIGH, 0, cam_cz,
                    CAM_R_HIGH * 1.2, CAM_H + 0.1, CAM_R_HIGH * 0.8,
                    None)
        cut_cavity(cam_comp, notch)

        # Second notch to fine-tune the release edge profile
        notch2 = box(cam_comp, "Cam_Drop_Edge",
                     cam_cx + CAM_R_LOW, 0, cam_cz + CAM_R_LOW*0.3,
                     CAM_R_HIGH * 0.6, CAM_H + 0.1, CAM_R_HIGH * 0.4,
                     None)
        cut_cavity(cam_comp, notch2)

        # Cam shaft center marker (motor shaft connection point)
        cyl(cam_comp, "Cam_Shaft_Boss", cam_cx, 0, cam_cz,
            0.12, CAM_H + 0.1, "y", chrome)

        # Cam counterweight (balances the eccentric mass)
        box(cam_comp, "Cam_Weight", cam_cx - ecc_x - 0.2, 0,
            cam_cz + 0.15, 0.25, 0.20, 0.55, dark_metal)

        # ── 6. N20 Micro Gearmotor ──────────────────────────────────
        # Motor mounts below the housing, shaft points up into the cam
        m_cx = cam_cx
        m_cy = -HOUSING_LY/2 - N20_BODY_H/2 - 0.1
        m_cz = cam_cz

        # Motor body (the can)
        box(launch_comp, "Motor_Body", m_cx, m_cy, m_cz,
            N20_BODY_L, N20_BODY_H, N20_BODY_W, dark_metal)

        # Motor D-flat (flat side of cylindrical motor)
        box(launch_comp, "Motor_D_Flat", m_cx, m_cy,
            m_cz - N20_BODY_W/2 - 0.01, N20_BODY_L, N20_BODY_H*0.7,
            0.02, dark_grey)

        # Gearbox (front of motor, contains reduction gears)
        gb_cz = m_cz + N20_BODY_W/2 + N20_GB_W/2
        box(launch_comp, "Gearbox", m_cx, m_cy, gb_cz,
            N20_GB_L, N20_GB_H, N20_GB_W, dark_grey)

        # Output shaft (from gearbox up into cam)
        shaft_cz = gb_cz + N20_GB_W/2 + N20_SHAFT_L/2
        cyl(launch_comp, "Motor_Shaft", m_cx, m_cy, shaft_cz,
            N20_SHAFT_R, N20_SHAFT_L, "y", chrome)

        # Shaft coupling (connects shaft to cam)
        cyl(launch_comp, "Shaft_Coupler", cam_cx, 0, cam_cz,
            0.18, 0.25, "y", dark_grey)

        # Motor mounting bracket (connects motor to housing)
        box(launch_comp, "Motor_Bracket", cam_cx, -HOUSING_LY/2 - 0.1,
            cam_cz, 1.8, 0.15, 2.0, dark_metal)

        # Motor mounting flange ears
        for side in [-1, 1]:
            box(launch_comp, f"Motor_Flange_{side}",
                cam_cx + side*1.0, -HOUSING_LY/2 - 0.05, cam_cz,
                0.15, 0.15, 0.8, dark_metal)

        # Motor wires (positive/negative leads)
        wire_cz = m_cz - N20_BODY_W/2 - 0.2
        cyl(launch_comp, "Motor_Wire_Red", m_cx - 0.3,
            m_cy - N20_BODY_H/2, wire_cz, 0.04, 1.8, "y", op_red)
        cyl(launch_comp, "Motor_Wire_Black", m_cx + 0.3,
            m_cy - N20_BODY_H/2, wire_cz, 0.04, 1.8, "y",
            black_plastic)

        # Motor data label (cosmetic)
        box(launch_comp, "Motor_Label", m_cx, m_cy,
            m_cz + N20_BODY_W/4, 0.6, 0.04, 0.3, chrome)

        # Wire routing channel into housing
        wc = cyl(launch_comp, "Wire_Channel_Entry", cam_cx,
                 -HOUSING_LY/2, p_cz + 0.5, 0.10, 0.40, "y", None)
        cut_cavity(launch_comp, wc)

        bom_add("Drive", "N20 micro gearmotor 6V 30RPM", 1,
                "OP_Ball_Launcher_DC")

        # ── 7. Magazine ─────────────────────────────────────────────
        mg_rad = MAG_ANGLE * math.pi / 180
        mg_cos = math.cos(mg_rad)
        mg_sin = math.sin(mg_rad)

        me_x = 0
        me_z = HOUSING_LZ/2 - 0.2
        ml_x = -MAG_LEN * mg_sin * 0.6
        ml_z = MAG_LEN * mg_cos

        mg_cx = me_x + ml_x/2
        mg_cy = HOUSING_LY/2 + 0.2
        mg_cz = me_z + ml_z/2

        cyl(launch_comp, "Mag_Tube", mg_cx, mg_cy, mg_cz,
            MAG_R + WALL_T, MAG_LEN - 0.2, "y", glass_clr)
        mb = cyl(launch_comp, "Mag_Bore", mg_cx, mg_cy, mg_cz,
                 MAG_R, MAG_LEN, "y", None)
        cut_cavity(launch_comp, mb)

        cone_shape(launch_comp, "Mag_Funnel", me_x, HOUSING_LY/2,
                   me_z + 0.1, MAG_R + 0.2, MAG_R + WALL_T,
                   0.40, "y", dark_metal)

        cyl(launch_comp, "Feed_Chute", me_x, HOUSING_LY/2 - 0.1,
            me_z + 0.5, MAG_R + 0.05, 0.8, "y", dark_metal)
        cb = cyl(launch_comp, "Feed_Chute_Bore", me_x,
                 HOUSING_LY/2 - 0.1, me_z + 0.5,
                 MAG_R - 0.05, 0.9, "y", None)
        cut_cavity(launch_comp, cb)

        box(launch_comp, "Mag_Cap", mg_cx,
            mg_cy + MAG_LEN/2 - 0.15, me_z + ml_z,
            0.8, 0.15, 0.8, dark_metal)

        box(launch_comp, "Mag_Bracket_L", mg_cx - 0.3,
            HOUSING_LY/2, mg_cz, 0.20, 0.60, 1.6, dark_metal)
        box(launch_comp, "Mag_Bracket_R", mg_cx + 0.3,
            HOUSING_LY/2, mg_cz, 0.20, 0.60, 1.6, dark_metal)

        # ── 8. Ball Projectiles ──────────────────────────────────────
        for i in range(N_BALLS):
            frac = (i + 0.5) / N_BALLS
            bx = me_x + ml_x * frac
            bz = me_z + ml_z * frac - 0.1
            sphere(launch_comp, f"Ball_{i+1}", bx, mg_cy, bz,
                   BALL_R, chrome)

        sphere(launch_comp, "Ball_Chambered", 0, 0,
               HOUSING_LZ/2 + 0.15, BALL_R, chrome)

        # ── 9. Firing Chamber ────────────────────────────────────────
        cyl(launch_comp, "Ball_Detent", 0, -PLUNGER_R - 0.02,
            HOUSING_LZ/2 - 0.1, 0.06, 0.15, "y", dark_grey)

        cyl(launch_comp, "Chamber_Ring", 0, 0,
            HOUSING_LZ/2 + 0.05, BARREL_R_OUT, 0.15, "z", dark_metal)

        # Chamber pressure relief ports
        for i in range(4):
            pa = i * math.pi / 2
            rp = cyl(launch_comp, f"Relief_{i}",
                     0.3 * math.cos(pa), 0.3 * math.sin(pa),
                     HOUSING_LZ/2 - 0.1, 0.04, 0.08, "z", None)
            cut_cavity(launch_comp, rp)

        # ── 10. Mounting Hinge ──────────────────────────────────────
        box(launch_comp, "Mount_Hinge", 0, 0, -HOUSING_LZ/2 + 0.6,
            1.1, 0.65, 0.8, dark_metal)
        cyl(launch_comp, "Hinge_Pin", 0, 0, -HOUSING_LZ/2 + 0.6,
            0.10, 0.80, "y", chrome)

        hs = box(launch_comp, "Hinge_Slot", 0, 0,
                 -HOUSING_LZ/2 + 0.5, 0.4, 0.30, 0.5, None)
        cut_cavity(launch_comp, hs)

        for mi, mo in enumerate([-0.35, 0.35]):
            sc = cyl(launch_comp, f"Mount_Screw_{mi}", mo, 0,
                     -HOUSING_LZ/2 + 0.6, 0.16, 0.60, "y", None)
            cut_cavity(launch_comp, sc)

        # ── 11. Sights ──────────────────────────────────────────────
        box(launch_comp, "Front_Sight", 0, BARREL_R_OUT + 0.05,
            bz_end - 0.5, 0.20, 0.30, 0.12, dark_metal)
        box(launch_comp, "Rear_Sight_L", -0.30,
            HOUSING_LY/2 + 0.05, -HOUSING_LZ/4, 0.12, 0.25, 0.30,
            dark_metal)
        box(launch_comp, "Rear_Sight_R", 0.30,
            HOUSING_LY/2 + 0.05, -HOUSING_LZ/4, 0.12, 0.25, 0.30,
            dark_metal)
        sn = box(launch_comp, "Sight_Notch", 0, HOUSING_LY/2 + 0.05,
                 -HOUSING_LZ/4, 0.20, 0.02, 0.10, None)
        cut_cavity(launch_comp, sn)

        # ── 12. Ammo Window ─────────────────────────────────────────
        box(launch_comp, "Ammo_Window", 0.3, HOUSING_LY/2 + 0.01,
            mg_cz, 0.6, 0.02, 1.2, glass_clr)

        # ── 13. Ejection / Cooling Port ──────────────────────────────
        box(launch_comp, "Eject_Port", 0.25, 0, HOUSING_LZ/2 + 0.3,
            0.30, 0.08, 0.25, dark_grey)

        # ══════════════════════════════════════════════════════════════
        # JOINTS
        # ══════════════════════════════════════════════════════════════

        # Drop cam revolute — continuous rotation about Y
        revolute_joint("Cam_Drive", cam_occ, launch_occ,
                       cam_cx, 0, cam_cz, "y")

        # ══════════════════════════════════════════════════════════════
        # BOM
        # ══════════════════════════════════════════════════════════════
        bom_add("Printed", "Launcher housing (PETG)", 1,
                "OP_Ball_Launcher_DC")
        bom_add("Printed", "Drop cam + plunger (PETG)", 1,
                "OP_Ball_Launcher_DC")
        bom_add("Printed", "Magazine tube (clear resin)", 1,
                "OP_Ball_Launcher_DC")
        bom_add("Hardware", "8mm plastic/metal balls", 6,
                "OP_Ball_Launcher_DC")

        # ══════════════════════════════════════════════════════════════
        # DONE
        # ══════════════════════════════════════════════════════════════

        bcount = sum(1 for c in comps_list for b in c.bRepBodies)
        msg = (
            f"Ball Launcher DC V1 — Generated\n"
            f"{'=' * 38}\n"
            f"  Components : {len(comps_list)}\n"
            f"  Bodies     : {bcount}\n"
            f"  Balls      : {N_BALLS}+1 in magazine\n"
            f"  Ball       : {BALL_R*2:.1f}cm ({BALL_R*20:.0f}mm)\n"
            f"  Drive      : N20 gearmotor + drop cam\n"
            f"  Feed       : Gravity (full auto)\n"
            f"\n"
            f"Sub-components:\n"
            f"  OP_Ball_Launcher_DC — housing, barrel, plunger,\n"
            f"    spring, N20 motor, magazine, 6 balls, sights\n"
            f"  Cam_Drop — eccentric drop cam (revolute joint)\n"
            f"\n"
            f"Just spin the motor — it fires 1 shot per rotation.\n"
            f"Motor required: N20 micro gearmotor 6V, ~30 RPM."
        )
        ui.messageBox(msg, "Ball Launcher DC V1 — Complete", 0)

    except:
        if ui:
            ui.messageBox(
                f"Ball Launcher DC V1 failed:\n{traceback.format_exc()}",
                "Ball Launcher DC — Error", 0)
