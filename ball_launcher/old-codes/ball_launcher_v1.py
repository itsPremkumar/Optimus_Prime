# =================================================================
# BALL LAUNCHER V1 — Gravity-Fed Spring Plunger Mechanism
# for Optimus Prime G1 Robot Arm
#
# A complete working launcher mechanism that fires small ball
# projectiles using a spring-loaded plunger, cocked and released
# by a single MG90S micro servo via a cam lobe.
#
# Balls (5x, 8mm dia) are stored in an angled gravity-fed magazine
# and load into the firing chamber automatically.
#
# Mechanism cycle (1 servo rotation = 1 shot):
#   1. Cam pin pulls plunger catch lug backward, compressing spring
#   2. At max rotation, cam pin disengages from catch lug
#   3. Spring slams plunger forward, striking the chambered ball
#   4. Ball launches out barrel; next ball drops into chamber via gravity
#
# Run in Fusion 360: Tools > Scripts & Add-Ins > Run
# =================================================================

import adsk.core, adsk.fusion, traceback, math, os, datetime

_ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
_app = adsk.core.Application.get()
_ui  = _app.userInterface

# ── Configuration ──────────────────────────────────────────────────────
BALL_R       = 0.40   # 8mm dia ball projectile radius (cm)
N_BALLS      = 5      # magazine capacity
BARREL_R_IN  = 0.46   # barrel inner radius (clearance over ball)
BARREL_R_OUT = 0.70   # barrel outer radius
BARREL_LEN   = 4.5    # barrel length
PLUNGER_R    = 0.36   # plunger head radius
PLUNGER_LEN  = 0.50   # plunger head length
SPRING_OD    = 0.50   # spring outer dia visual
SPRING_LEN   = 2.20   # uncompressed spring visual length
CAM_R        = 0.90   # cam wheel radius
CAM_H        = 0.35   # cam wheel thickness
CAM_PIN_R    = 0.10   # cam pin radius
CAM_PIN_H    = 0.50   # cam pin height above cam
CAM_PIN_OFF  = 0.55   # cam pin eccentric offset from cam center
MAG_R        = 0.48   # magazine tube inner radius
MAG_LEN      = 5.5    # magazine tube length along axis
MAG_ANGLE    = 25     # magazine angle from horizontal (degrees)
HOUSING_LX   = 3.2    # housing length (X)
HOUSING_LY   = 2.8    # housing width (Y)
HOUSING_LZ   = 4.8    # housing height (Z)
WALL_T       = 0.18   # wall thickness
PLUNGER_CAM_OFFSET = 0.35  # Z offset of cam from plunger catch

# ── Position in arm coordinates ────────────────────────────────────────
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

        # ── Materials / Appearances ──────────────────────────────────
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
                    cx - av[0] * h / 2, cy - av[1] * h / 2,
                    cz - av[2] * h / 2)
                p2 = adsk.core.Point3D.create(
                    cx + av[0] * h / 2, cy + av[1] * h / 2,
                    cz + av[2] * h / 2)
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
                    cx - av[0] * h / 2, cy - av[1] * h / 2,
                    cz - av[2] * h / 2)
                p2 = adsk.core.Point3D.create(
                    cx + av[0] * h / 2, cy + av[1] * h / 2,
                    cz + av[2] * h / 2)
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

        def rigid_joint(name, occ1, occ2):
            if not (occ1 and occ2):
                return
            try:
                geom = _make_joint_geometry(0, 0, 0)
                ji = root.asBuiltJoints.createInput(occ1, occ2, geom)
                ji.setAsRigidJointMotion()
                j = root.asBuiltJoints.add(ji)
                j.name = name
            except:
                pass

        # ══════════════════════════════════════════════════════════════
        # BUILD THE BALL LAUNCHER
        # ══════════════════════════════════════════════════════════════
        #
        # Coordinate system (local to launcher):
        #   Origin  = mount point on the right hand
        #   +X      = right (away from robot center)
        #   +Y      = up
        #   +Z      = forward (muzzle direction)
        #   Barrel  = points in +Z
        #   Magazine = on +Y side, angles backward (-X, +Z)
        #   Cam     = on +X side
        # ══════════════════════════════════════════════════════════════

        # Create the main launcher component and the cam sub-component
        launcher_comp, launcher_occ = new_component("OP_Ball_Launcher")

        # Create the cam wheel as a separate sub-component for rotation
        cam_comp, cam_occ = new_component("Cam_Wheel")

        # ── 1. Barrel ────────────────────────────────────────────────
        barrel_z_start = HOUSING_LZ / 2 - 0.3
        barrel_z_end   = barrel_z_start + BARREL_LEN
        barrel_cz      = barrel_z_start + BARREL_LEN / 2

        cyl(launcher_comp, "Barrel_Outer", 0, 0, barrel_cz,
            BARREL_R_OUT, BARREL_LEN + 0.2, "z", chrome)

        bore = cyl(launcher_comp, "Barrel_Bore", 0, 0, barrel_cz,
                   BARREL_R_IN, BARREL_LEN + 0.4, "z", None)
        cut_cavity(launcher_comp, bore)

        cyl(launcher_comp, "Muzzle_Ring", 0, 0, barrel_z_end + 0.1,
            BARREL_R_OUT + 0.08, 0.20, "z", dark_metal)

        led_pocket = cyl(launcher_comp, "Muzzle_LED_Pocket", 0, 0,
                         barrel_z_end + 0.5, 0.26, 0.40, "z", None)
        cut_cavity(launcher_comp, led_pocket)
        bom_add("Electronics", "LED 5mm (muzzle flash)", 1,
                "OP_Ball_Launcher")

        # ── 2. Main Housing ──────────────────────────────────────────
        box(launcher_comp, "Housing_Main", 0, 0, 0,
            HOUSING_LX, HOUSING_LY, HOUSING_LZ, dark_metal)

        interior = box(launcher_comp, "Housing_Interior", 0, 0, 0,
                       HOUSING_LX - 2 * WALL_T,
                       HOUSING_LY - 2 * WALL_T,
                       HOUSING_LZ - 2 * WALL_T, None)
        cut_cavity(launcher_comp, interior)

        # Rear access panel
        box(launcher_comp, "Rear_Cap", 0, 0, -HOUSING_LZ / 2 + 0.15,
            HOUSING_LX - 0.4, HOUSING_LY - 0.4, 0.30, dark_grey)

        # Top rail detail
        box(launcher_comp, "Top_Rail", 0, HOUSING_LY / 2 + 0.02,
            -0.5, HOUSING_LX - 0.6, 0.12, 2.0, chrome)

        # Side vents
        for i in range(3):
            vz = -0.6 + i * 0.7
            vent = box(launcher_comp, f"Vent_L_{i}",
                       -HOUSING_LX / 2 - 0.01, 0, vz,
                       0.02, 0.10, 0.50, None)
            cut_cavity(launcher_comp, vent)
            vent2 = box(launcher_comp, f"Vent_R_{i}",
                        HOUSING_LX / 2 + 0.01, 0, vz,
                        0.02, 0.10, 0.50, None)
            cut_cavity(launcher_comp, vent2)

        # Housing ribbing (structural)
        for i in range(2):
            rz = -1.0 + i * 2.0
            box(launcher_comp, f"Rib_{i}", 0,
                -HOUSING_LY / 2 + 0.1, rz,
                HOUSING_LX - 0.4, 0.08, 0.15, dark_metal)

        # ── 3. Plunger Assembly ──────────────────────────────────────
        plunger_cz = -HOUSING_LZ / 2 + 0.6

        cyl(launcher_comp, "Plunger_Head", 0, 0, plunger_cz + 0.3,
            PLUNGER_R, PLUNGER_LEN, "z", dark_metal)

        cyl(launcher_comp, "Plunger_Shaft", 0, 0, plunger_cz,
            0.15, 1.6, "z", chrome)

        box(launcher_comp, "Plunger_Catch", 0.35, 0, plunger_cz + 0.2,
            0.20, 0.16, 0.35, dark_metal)

        # Plunger rear bumper (cushion at full retract)
        cyl(launcher_comp, "Plunger_Bumper", 0, 0, plunger_cz - 0.6,
            0.20, 0.12, "z", black_plastic)
        bom_add("Hardware", "Foam bumper pad (plunger)", 1,
                "OP_Ball_Launcher")

        # ── 4. Compression Spring (visual) ───────────────────────────
        spring_cz = plunger_cz - 0.8
        for i in range(6):
            sc = spring_cz - i * 0.32
            cyl(launcher_comp, f"Spring_Coil_{i}", 0, 0, sc,
                SPRING_OD / 2, 0.06, "z", chrome)
        # Center guide rod
        cyl(launcher_comp, "Spring_Guide", 0, 0, spring_cz - 0.8,
            0.08, 2.4, "z", chrome)
        bom_add("Hardware", "Compression spring O5mm x 22mm", 1,
                "OP_Ball_Launcher")

        # ── 5. Cam Wheel (sub-component for rotation) ────────────────
        cam_cx = HOUSING_LX / 2 + 0.1
        cam_cz = plunger_cz + PLUNGER_CAM_OFFSET

        cyl(cam_comp, "Cam_Wheel", cam_cx, 0, cam_cz,
            CAM_R, CAM_H, "y", dark_metal)

        cyl(cam_comp, "Cam_Pin", cam_cx, 0, cam_cz + CAM_PIN_OFF,
            CAM_PIN_R, CAM_PIN_H, "y", chrome)

        box(cam_comp, "Cam_Lobe_Profile", cam_cx + 0.2, 0,
            cam_cz + 0.3, 0.12, 0.20, 0.6, dark_grey)

        cyl(cam_comp, "Cam_Shaft", cam_cx, 0, cam_cz,
            0.15, 0.8, "y", chrome)

        # Cam counterweight (balance for smooth rotation)
        box(cam_comp, "Cam_Weight", cam_cx - 0.3, 0,
            cam_cz - 0.4, 0.20, 0.18, 0.25, dark_metal)

        # Cam positioning detent notch (visual)
        box(cam_comp, "Cam_Detent_Notch", cam_cx + 0.35, 0,
            cam_cz, 0.06, 0.12, 0.06, dark_grey)

        # ── 6. Servo Assembly (launcher body side) ───────────────────
        srv_cx = cam_cx
        srv_cy = -HOUSING_LY / 2 - 0.6
        srv_cz = cam_cz

        box(launcher_comp, "Servo_Body", srv_cx, srv_cy, srv_cz,
            2.30, 1.20, 2.30, op_red)
        box(launcher_comp, "Servo_Ear_L", srv_cx - 1.05, srv_cy,
            srv_cz, 0.30, 1.30, 3.20, dark_grey)
        box(launcher_comp, "Servo_Ear_R", srv_cx + 1.05, srv_cy,
            srv_cz, 0.30, 1.30, 3.20, dark_grey)

        cyl(launcher_comp, "Servo_Horn", srv_cx, srv_cy + 0.6,
            srv_cz, 0.40, 0.18, "y", chrome)

        wire_chan = cyl(launcher_comp, "Servo_Wire_Chan",
                        srv_cx - 0.5, srv_cy - 0.6, srv_cz,
                        0.10, 0.50, "y", None)
        cut_cavity(launcher_comp, wire_chan)

        box(launcher_comp, "Servo_Bracket", cam_cx,
            -HOUSING_LY / 2 - 0.2, cam_cz, 1.6, 0.30, 1.8, dark_metal)

        # Servo cable channel through housing
        cable_ch = cyl(launcher_comp, "Cable_Channel",
                       0, -HOUSING_LY / 2, -HOUSING_LZ / 4,
                       0.12, 0.6, "y", None)
        cut_cavity(launcher_comp, cable_ch)

        bom_add("Servo", "MG90S micro servo (trigger cam)", 1,
                "OP_Ball_Launcher")

        # ── 7. Gravity-Fed Magazine ──────────────────────────────────
        mag_rad = MAG_ANGLE * math.pi / 180
        mag_cos = math.cos(mag_rad)
        mag_sin = math.sin(mag_rad)

        mag_entry_x = 0
        mag_entry_z = HOUSING_LZ / 2 - 0.2

        mag_len_x = -MAG_LEN * mag_sin * 0.6
        mag_len_z = MAG_LEN * mag_cos

        mag_cx = mag_entry_x + mag_len_x / 2
        mag_cy = HOUSING_LY / 2 + 0.2
        mag_cz = mag_entry_z + mag_len_z / 2

        cyl(launcher_comp, "Magazine_Tube", mag_cx, mag_cy, mag_cz,
            MAG_R + WALL_T, MAG_LEN - 0.2, "y", glass_clr)

        mag_bore = cyl(launcher_comp, "Magazine_Bore", mag_cx,
                       mag_cy, mag_cz, MAG_R, MAG_LEN, "y", None)
        cut_cavity(launcher_comp, mag_bore)

        cone_shape(launcher_comp, "Magazine_Funnel", mag_entry_x,
                   HOUSING_LY / 2, mag_entry_z + 0.1,
                   MAG_R + 0.2, MAG_R + WALL_T, 0.40, "y", dark_metal)

        # Feed chute
        cyl(launcher_comp, "Feed_Chute", mag_entry_x,
            HOUSING_LY / 2 - 0.1, mag_entry_z + 0.5,
            MAG_R + 0.05, 0.8, "y", dark_metal)
        chute_bore = cyl(launcher_comp, "Feed_Chute_Bore", mag_entry_x,
                         HOUSING_LY / 2 - 0.1, mag_entry_z + 0.5,
                         MAG_R - 0.05, 0.9, "y", None)
        cut_cavity(launcher_comp, chute_bore)

        # Magazine top cap
        box(launcher_comp, "Magazine_Cap", mag_cx,
            mag_cy + MAG_LEN / 2 - 0.15, mag_entry_z + mag_len_z,
            0.8, 0.15, 0.8, dark_metal)

        # Magazine support brackets
        box(launcher_comp, "Magazine_Bracket_L", mag_cx - 0.3,
            HOUSING_LY / 2, mag_cz, 0.20, 0.60, 1.6, dark_metal)
        box(launcher_comp, "Magazine_Bracket_R", mag_cx + 0.3,
            HOUSING_LY / 2, mag_cz, 0.20, 0.60, 1.6, dark_metal)

        # ── 8. Ball Projectiles ──────────────────────────────────────
        ball_spacing = BALL_R * 2 + 0.05
        for i in range(N_BALLS):
            frac = (i + 0.5) / N_BALLS
            bx = mag_entry_x + mag_len_x * frac
            bz = mag_entry_z + mag_len_z * frac - 0.1
            by = mag_cy
            sphere(launcher_comp, f"Ball_{i + 1}", bx, by, bz,
                   BALL_R, chrome)

        # Chambered ball (ready position)
        sphere(launcher_comp, "Ball_Chambered", 0, 0,
               HOUSING_LZ / 2 + 0.15, BALL_R, chrome)

        # ── 9. Firing Chamber Details ────────────────────────────────
        cyl(launcher_comp, "Ball_Detent", 0, -PLUNGER_R - 0.02,
            HOUSING_LZ / 2 - 0.1, 0.06, 0.15, "y", dark_grey)

        cyl(launcher_comp, "Chamber_Ring", 0, 0,
            HOUSING_LZ / 2 + 0.05, BARREL_R_OUT, 0.15, "z", dark_metal)

        # Chamber pressure relief port (visual)
        for i in range(4):
            pa = i * math.pi / 2
            px = 0.3 * math.cos(pa)
            py = 0.3 * math.sin(pa)
            port = cyl(launcher_comp, f"Relief_Port_{i}",
                       px, py, HOUSING_LZ / 2 - 0.1,
                       0.04, 0.08, "z", None)
            cut_cavity(launcher_comp, port)

        # ── 10. Mounting Hinge ───────────────────────────────────────
        box(launcher_comp, "Mount_Hinge", 0, 0, -HOUSING_LZ / 2 + 0.6,
            1.1, 0.65, 0.8, dark_metal)
        cyl(launcher_comp, "Hinge_Pin", 0, 0, -HOUSING_LZ / 2 + 0.6,
            0.10, 0.80, "y", chrome)

        hinge_slot = box(launcher_comp, "Hinge_Slot", 0, 0,
                         -HOUSING_LZ / 2 + 0.5, 0.4, 0.30, 0.5, None)
        cut_cavity(launcher_comp, hinge_slot)

        for m_idx, m_off in enumerate([-0.35, 0.35]):
            sc = cyl(launcher_comp, f"Mount_Screw_{m_idx}", m_off, 0,
                     -HOUSING_LZ / 2 + 0.6, 0.16, 0.60, "y", None)
            cut_cavity(launcher_comp, sc)

        # ── 11. Sights ───────────────────────────────────────────────
        box(launcher_comp, "Front_Sight", 0, BARREL_R_OUT + 0.05,
            barrel_z_end - 0.5, 0.20, 0.30, 0.12, dark_metal)

        box(launcher_comp, "Rear_Sight_L", -0.30,
            HOUSING_LY / 2 + 0.05, -HOUSING_LZ / 4,
            0.12, 0.25, 0.30, dark_metal)
        box(launcher_comp, "Rear_Sight_R", 0.30,
            HOUSING_LY / 2 + 0.05, -HOUSING_LZ / 4,
            0.12, 0.25, 0.30, dark_metal)
        sight_notch = box(launcher_comp, "Sight_Notch", 0,
                          HOUSING_LY / 2 + 0.05, -HOUSING_LZ / 4,
                          0.20, 0.02, 0.10, None)
        cut_cavity(launcher_comp, sight_notch)

        # ── 12. Ammo Counter (visual indicator window) ───────────────
        # Small window showing remaining balls
        box(launcher_comp, "Ammo_Window", 0.3, HOUSING_LY / 2 + 0.01,
            mag_cz, 0.6, 0.02, 1.2, glass_clr)

        # Ejection port (where spent ball would exit - cosmetic)
        box(launcher_comp, "Eject_Port", 0.25, 0,
            HOUSING_LZ / 2 + 0.3, 0.30, 0.08, 0.25, dark_grey)

        # ══════════════════════════════════════════════════════════════
        # JOINTS
        # ══════════════════════════════════════════════════════════════

        # Cam wheel revolute joint (rotates about Y relative to launcher)
        revolute_joint("Trigger_Cam", cam_occ, launcher_occ,
                       cam_cx, 0, cam_cz, "y")

        # ══════════════════════════════════════════════════════════════
        # BOM SAVE
        # ══════════════════════════════════════════════════════════════
        bom_add("Printed", "Ball Launcher housing (PETG)", 1,
                "OP_Ball_Launcher")
        bom_add("Printed", "Cam wheel + plunger (PETG)", 1,
                "OP_Ball_Launcher")
        bom_add("Printed", "Magazine tube (clear PETG/resin)", 1,
                "OP_Ball_Launcher")
        bom_add("Hardware", "Steel pin O2mm (cam pivot)", 1,
                "OP_Ball_Launcher")
        bom_add("Hardware", "8mm plastic/metal balls (projectile)", 6,
                "OP_Ball_Launcher")

        # ══════════════════════════════════════════════════════════════
        # DONE
        # ══════════════════════════════════════════════════════════════

        body_count = sum(1 for c in comps_list for b in c.bRepBodies)
        msg = (
            f"Ball Launcher V1 — Generated Successfully\n"
            f"{'=' * 40}\n"
            f"  Components : {len(comps_list)}\n"
            f"  Bodies     : {body_count}\n"
            f"  Balls      : {N_BALLS} in magazine + 1 chambered\n"
            f"  Ball dia   : {BALL_R * 2:.1f} cm ({BALL_R * 20:.0f} mm)\n"
            f"  Barrel len : {BARREL_LEN:.1f} cm\n"
            f"  Drive      : 1x MG90S servo + compression spring\n"
            f"  Feed       : Gravity (auto after each shot)\n"
            f"\n"
            f"Components:\n"
            f"  OP_Ball_Launcher — main body, barrel, magazine,\n"
            f"    plunger, servo mount, sights\n"
            f"  Cam_Wheel — rotating cam lobe (revolute joint)\n"
            f"\n"
            f"The Trigger_Cam revolute joint lets you animate\n"
            f"the cam rotation to simulate the firing cycle."
        )
        ui.messageBox(msg, "Ball Launcher V1 — Complete", 0)

    except:
        if ui:
            ui.messageBox(
                f"Ball Launcher V1 failed:\n{traceback.format_exc()}",
                "Ball Launcher — Error", 0)
