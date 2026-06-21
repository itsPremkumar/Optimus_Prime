
        # ─────────────────────────────────────────────────────────────────
        # ELECTRONICS BAY HELPERS  (ELEC-1 … ELEC-8)
        # ─────────────────────────────────────────────────────────────────

        def rpi_zero_bay(comp, tag, cx, cy, cz):
            """ELEC-1 — RPi Zero 2W pocket with M2.5 standoffs and access cover."""
            cut_cavity(comp, box(comp, f"{tag}_RPiBay",
                                 cx, cy, cz,
                                 RPI0_L + 0.20, RPI0_W + 0.20, RPI0_H + 0.30))
            for sx, sz in [(-2.90, -1.15), (+2.90, -1.15),
                           (-2.90, +1.15), (+2.90, +1.15)]:
                cyl(comp, f"{tag}_RpiStdoff_{sx}_{sz}",
                    cx+sx, cy, cz+sz, 0.14, RPI0_H+0.50, "y", dark_metal)
            # v11: SD card access slot
            cut_cavity(comp, box(comp, f"{tag}_SDSlot",
                                 cx - RPI0_L/2 - 0.15, cy, cz,
                                 0.30, RPI0_W + 0.40, 0.25))
            # v11: USB/power access
            cut_cavity(comp, box(comp, f"{tag}_USBSLOT",
                                 cx, cy - RPI0_W/2 - 0.15, cz - 0.5,
                                 0.80, 0.30, 0.40))
            BOM.add("Electronics", "Raspberry Pi Zero 2W", 1, comp.name)
            BOM.add("Fastener",    "M2.5×11 mm brass standoff", 4, comp.name)

        def pca9685_bay(comp, tag, cx, cy, cz):
            """ELEC-2 — PCA9685 pocket with standoffs."""
            cut_cavity(comp, box(comp, f"{tag}_PCABay",
                                 cx, cy, cz,
                                 PCA_L + 0.20, PCA_W + 0.20, PCA_H + 0.30))
            for sx, sz in [(-3.00, -1.08), (+3.00, -1.08),
                           (-3.00, +1.08), (+3.00, +1.08)]:
                cyl(comp, f"{tag}_PCAStdoff_{sx}_{sz}",
                    cx+sx, cy, cz+sz, 0.14, PCA_H+0.50, "y", dark_metal)
            # v11: I2C connector access
            cut_cavity(comp, box(comp, f"{tag}_I2CPort",
                                 cx, cy - PCA_W/2 - 0.15, cz,
                                 0.60, 0.25, 0.30))
            BOM.add("Electronics", "PCA9685 16-ch servo driver", 1, comp.name)

        def lipo_bay(comp, tag, cx, cy, cz):
            """ELEC-3 — 2S LiPo pocket + XT60-F slot + v11 power system."""
            cut_cavity(comp, box(comp, f"{tag}_LipoBay",
                                 cx, cy, cz,
                                 LIPO_L + 0.30, LIPO_H + 0.30, LIPO_W + 0.30))
            cut_cavity(comp, box(comp, f"{tag}_XT60Slot",
                                 cx, cy + LIPO_H/2 + 0.15, cz,
                                 XT60_W + 0.10, 0.50, XT60_H_SLOT + 0.10))
            # v11: Power system integration
            power_system(comp, tag, cx, cy, cz)
            BOM.add("Electronics", "2S 1300 mAh 7.4V LiPo battery", 1, comp.name)
            BOM.add("Electronics", "XT60-F panel-mount connector",  1, comp.name)

        def imu_pocket(comp, tag, cx, cy, cz, axis="z"):
            """ELEC-6 — MPU-6050 pocket."""
            cut_cavity(comp, box(comp, f"{tag}_IMUPkt",
                                 cx, cy, cz,
                                 IMU_L + 0.20, IMU_W + 0.20, IMU_H + 0.30))
            BOM.add("Electronics", "MPU-6050 IMU breakout", 1, comp.name)

        def esp32_cam_pocket(comp, tag, cx, cy, cz, lens_axis="y"):
            """ELEC-5 — ESP32-CAM pocket + Ø3 mm lens channel."""
            cut_cavity(comp, box(comp, f"{tag}_ESP32Bay",
                                 cx, cy, cz,
                                 ESP32_L + 0.20, ESP32_H + 0.30, ESP32_W + 0.20))
            ax_map = {"y": (0,1,0), "x": (1,0,0), "z": (0,0,1)}
            av = ax_map[lens_axis]
            cut_cavity(comp, cyl(comp, f"{tag}_LensHole",
                                 cx, cy - (ESP32_H/2 + 0.30), cz,
                                 0.150, 0.60, lens_axis))
            # v11: USB programming access
            cut_cavity(comp, box(comp, f"{tag}_ESP32USB",
                                 cx + ESP32_L/2 + 0.10, cy, cz,
                                 0.30, 0.40, 0.20))
            BOM.add("Electronics", "ESP32-CAM module (OV2640)", 1, comp.name)
SECTION 3: JOINT BUILDERS (Unchanged from v10)
Python
        # ─────────────────────────────────────────────────────────────────
        # JOINT BUILDERS
        # ─────────────────────────────────────────────────────────────────

        def _make_joint_geometry(cx, cy, cz):
            try:
                cpi = root.constructionPoints.createInput()
                cpi.setByPoint(adsk.core.Point3D.create(cx, cy, cz))
                cp = root.constructionPoints.add(cpi)
                cp.isLightBulbOn = False
                return adsk.fusion.JointGeometry.createByPoint(cp)
            except Exception:
                pass
            try:
                sketch = root.sketches.add(root.xYConstructionPlane)
                sketch.isVisible = False
                s_pt = sketch.sketchPoints.add(adsk.core.Point3D.create(cx, cy, cz))
                return adsk.fusion.JointGeometry.createByPoint(s_pt)
            except Exception:
                return None

        def revolute_joint(name, occ1, occ2, cx, cy, cz, axis_str):
            if not (occ1 and occ2):
                return
            try:
                geom = _make_joint_geometry(cx, cy, cz)
                ji   = root.asBuiltJoints.createInput(occ1, occ2, geom)
                av   = {"x": adsk.core.Vector3D.create(1, 0, 0),
                        "y": adsk.core.Vector3D.create(0, 1, 0),
                        "z": adsk.core.Vector3D.create(0, 0, 1)}[axis_str]
                ji.setAsRevoluteJointMotion(
                    adsk.fusion.JointDirections.CustomJointDirection, av)
                j      = root.asBuiltJoints.add(ji)
                j.name = name
            except Exception:
                Logger.log(f"Failed revolute joint {name}: {traceback.format_exc()}", "ERROR")

        def ball_joint(name, occ1, occ2, cx, cy, cz):
            if not (occ1 and occ2):
                return
            try:
                geom = _make_joint_geometry(cx, cy, cz)
                ji   = root.asBuiltJoints.createInput(occ1, occ2, geom)
                ji.setAsBallJointMotion(
                    adsk.fusion.JointDirections.ZAxisJointDirection,
                    adsk.fusion.JointDirections.XAxisJointDirection)
                j      = root.asBuiltJoints.add(ji)
                j.name = name
            except Exception:
                Logger.log(f"Failed ball joint {name}: {traceback.format_exc()}", "ERROR")

        def rigid_joint(name, occ1, occ2):
            if not (occ1 and occ2):
                return
            try:
                geom = _make_joint_geometry(0, 0, 0)
                ji   = root.asBuiltJoints.createInput(occ1, occ2, geom)
                ji.setAsRigidJointMotion()
                j      = root.asBuiltJoints.add(ji)
                j.name = name
            except Exception:
                pass
SECTION 4: HARDWARE MODULES (Enhanced with v11 Couplers)
Python
        # ─────────────────────────────────────────────────────────────────
        # HARDWARE MODULES
        # ─────────────────────────────────────────────────────────────────

        def servo_hardware(comp, tag, cx, cy, cz, axis, mg996):
            if mg996:
                fd, fw, hr, pd, sd = 2.4, 0.5, 0.7, 0.125, 0.15
                if axis == "x":
                    hx,hy,hz = cx+2.40, cy,       cz+1.05
                    fx,fy,fz = cx+0.95, cy,       cz
                elif axis == "z":
                    hx,hy,hz = cx-1.10, cy,       cz+2.40
                    fx,fy,fz = cx,      cy,       cz+0.95
                else:
                    hx,hy,hz = cx,      cy+2.40,  cz+1.05
                    fx,fy,fz = cx,      cy+0.95,  cz
            else:
                fd, fw, hr, pd, sd = 1.35, 0.0, 0.4, 0.10, 0.10
                if axis == "x":
                    hx,hy,hz = cx+1.40, cy,       cz+0.50
                    fx,fy,fz = cx+0.45, cy,       cz
                elif axis == "z":
                    hx,hy,hz = cx-0.50, cy,       cz+1.40
                    fx,fy,fz = cx,      cy,       cz+0.45
                else:
                    hx,hy,hz = cx,      cy+1.40,  cz+0.50
                    fx,fy,fz = cx,      cy+0.45,  cz

            for d1 in [-fd, fd]:
                for d2 in ([-fw, fw] if fw > 0 else [0]):
                    if axis == "x":
                        c = cyl(comp, f"{tag}_FlgS_{d1}_{d2}", fx,    fy+d2, fz+d1, sd, 1.5, "x")
                    elif axis == "z":
                        c = cyl(comp, f"{tag}_FlgS_{d1}_{d2}", fx+d1, fy+d2, fz,    sd, 1.5, "z")
                    else:
                        c = cyl(comp, f"{tag}_FlgS_{d1}_{d2}", fx+d1, fy,    fz+d2, sd, 1.5, "y")
                    cut_cavity(comp, c)

            for d in [-hr, hr]:
                if axis == "x":
                    c1 = cyl(comp, f"{tag}_Hrn1_{d}", hx,   hy+d, hz,   pd, 1.5, "x")
                    c2 = cyl(comp, f"{tag}_Hrn2_{d}", hx,   hy,   hz+d, pd, 1.5, "x")
                elif axis == "z":
                    c1 = cyl(comp, f"{tag}_Hrn1_{d}", hx+d, hy,   hz,   pd, 1.5, "z")
                    c2 = cyl(comp, f"{tag}_Hrn2_{d}", hx,   hy+d, hz,   pd, 1.5, "z")
                else:
                    c1 = cyl(comp, f"{tag}_Hrn1_{d}", hx+d, hy,   hz,   pd, 1.5, "y")
                    c2 = cyl(comp, f"{tag}_Hrn2_{d}", hx,   hy,   hz+d, pd, 1.5, "y")
                cut_cavity(comp, c1)
                cut_cavity(comp, c2)
            grommet_hole(comp, tag, cx, cy, cz + (0.5 if axis != "z" else 1.0))

        def mg996r(comp, tag, cx, cy, cz, axis="x"):
            cl = CLEARANCE
            servo_type = "hip_hd" if "Hip" in tag or "Waist" in tag else "std"
            if axis == "x":
                box(comp, f"{tag}_VisBody", cx,      cy,      cz,      4.05, 2.00, 4.20, grey_plastic)
                box(comp, f"{tag}_VisEars", cx+0.95, cy,      cz,      0.30, 2.20, 5.80, dark_grey)
                cyl(comp, f"{tag}_VisHorn", cx+2.40, cy,      cz+1.05, 0.95, 0.22, "x", white_pla)
                marker(comp, f"{tag}_Pivot", cx+2.40, cy,     cz+1.05)
                cut_cavity(comp, box(comp, f"{tag}_CB",  cx,      cy, cz, 4.05+cl, 2.00+cl, 4.20+cl))
                cut_cavity(comp, box(comp, f"{tag}_CE",  cx+0.95, cy, cz, 0.30+cl, 2.20+cl, 5.80+cl))
            elif axis == "z":
                box(comp, f"{tag}_VisBody", cx,      cy,      cz,      4.05, 2.00, 4.20, grey_plastic)
                box(comp, f"{tag}_VisEars", cx,      cy,      cz+0.95, 5.80, 2.20, 0.30, dark_grey)
                cyl(comp, f"{tag}_VisHorn", cx-1.10, cy,      cz+2.40, 0.95, 0.22, "z", white_pla)
                marker(comp, f"{tag}_Pivot", cx-1.10, cy,     cz+2.40)
                cut_cavity(comp, box(comp, f"{tag}_CB", cx, cy, cz,      4.05+cl, 2.00+cl, 4.20+cl))
                cut_cavity(comp, box(comp, f"{tag}_CE", cx, cy, cz+0.95, 5.80+cl, 2.20+cl, 0.30+cl))
            else:
                box(comp, f"{tag}_VisBody", cx,      cy,      cz,      4.05, 4.20, 2.00, grey_plastic)
                box(comp, f"{tag}_VisEars", cx,      cy+0.95, cz,      4.05, 0.30, 2.20, dark_grey)
                cyl(comp, f"{tag}_VisHorn", cx,      cy+2.40, cz+1.05, 0.95, 0.22, "y", white_pla)
                marker(comp, f"{tag}_Pivot", cx,     cy+2.40, cz+1.05)
                cut_cavity(comp, box(comp, f"{tag}_CB", cx, cy,      cz, 4.05+cl, 4.20+cl, 2.00+cl))
                cut_cavity(comp, box(comp, f"{tag}_CE", cx, cy+0.95, cz, 4.05+cl, 0.30+cl, 2.20+cl))
            servo_hardware(comp, tag, cx, cy, cz, axis, True)
            # v11: Horn coupler
            horn_coupler(comp, f"{tag}_Coupler", cx, cy, cz, axis, servo_type)
            BOM.add("Servo", "MG996R 11 kg·cm servo", 1, comp.name)

        def mg90s(comp, tag, cx, cy, cz, axis="x"):
            cl = CLEARANCE
            servo_type = "micro"
            if axis == "x":
                box(comp, f"{tag}_VisBody", cx,      cy,      cz,      2.30, 1.20, 2.30, op_blue)
                box(comp, f"{tag}_VisEars", cx+0.45, cy,      cz,      0.20, 1.30, 3.20, op_blue)
                cyl(comp, f"{tag}_VisHorn", cx+1.40, cy,      cz+0.50, 0.55, 0.18, "x", white_pla)
                marker(comp, f"{tag}_Pivot", cx+1.40, cy,     cz+0.50)
                cut_cavity(comp, box(comp, f"{tag}_CB",  cx,      cy, cz, 2.30+cl, 1.20+cl, 2.30+cl))
                cut_cavity(comp, box(comp, f"{tag}_CE",  cx+0.45, cy, cz, 0.20+cl, 1.30+cl, 3.20+cl))
            elif axis == "z":
                box(comp, f"{tag}_VisBody", cx,      cy,      cz,      2.30, 1.20, 2.30, op_blue)
                box(comp, f"{tag}_VisEars", cx,      cy,      cz+0.45, 3.20, 1.30, 0.20, op_blue)
                cyl(comp, f"{tag}_VisHorn", cx-0.50, cy,      cz+1.40, 0.55, 0.18, "z", white_pla)
                marker(comp, f"{tag}_Pivot", cx-0.50, cy,     cz+1.40)
                cut_cavity(comp, box(comp, f"{tag}_CB", cx, cy, cz,      2.30+cl, 1.20+cl, 2.30+cl))
                cut_cavity(comp, box(comp, f"{tag}_CE", cx, cy, cz+0.45, 3.20+cl, 1.30+cl, 0.20+cl))
            else:
                box(comp, f"{tag}_VisBody", cx,      cy,      cz,      2.30, 2.30, 1.20, op_blue)
                box(comp, f"{tag}_VisEars", cx,      cy+0.45, cz,      3.20, 0.20, 1.30, op_blue)
                cyl(comp, f"{tag}_VisHorn", cx,      cy+1.40, cz+0.50, 0.55, 0.18, "y", white_pla)
                marker(comp, f"{tag}_Pivot", cx,     cy+1.40, cz+0.50)
                cut_cavity(comp, box(comp, f"{tag}_CB", cx, cy,      cz, 2.30+cl, 2.30+cl, 1.20+cl))
                cut_cavity(comp, box(comp, f"{tag}_CE", cx, cy+0.45, cz, 3.20+cl, 0.20+cl, 1.30+cl))
            servo_hardware(comp, tag, cx, cy, cz, axis, False)
            # v11: Horn coupler
            horn_coupler(comp, f"{tag}_Coupler", cx, cy, cz, axis, servo_type)
            BOM.add("Servo", "MG90S micro servo", 1, comp.name)

        def tt_wheel(comp, tag, cx, cy, cz, side=1):
            cl = CLEARANCE
            box(comp, f"{tag}_VisGB",    cx,           cy,     cz,  2.30, 5.20, 1.90, yellow_met)
            cyl(comp, f"{tag}_VisMot",   cx,           cy-1.5, cz,  0.90, 2.10, "y",  chrome)
            cyl(comp, f"{tag}_VisShaft", cx+side*1.75, cy,     cz,  0.20, 3.50, "x",  chrome)
            cyl(comp, f"{tag}_VisHub",   cx+side*3.25, cy,     cz,  0.80, 2.60, "x",  dark_metal)
            cyl(comp, f"{tag}_VisTire",  cx+side*3.25, cy,     cz,  3.25, 2.60, "x",  rubber_blk)
            cyl(comp, f"{tag}_VisRim",   cx+side*3.25, cy,     cz,  2.20, 2.65, "x",  chrome)
            marker(comp, f"{tag}_Axle_Pivot", cx+side*3.25, cy, cz, 0.18)
            cut_cavity(comp, box(comp, f"{tag}_CGB", cx,           cy, cz, 2.30+cl, 5.20+cl, 1.90+cl))
            cut_cavity(comp, box(comp, f"{tag}_CDS", cx+side*3.25, cy, cz, 2.7+cl,  0.54+cl, 0.36+cl))
            BOM.add("Drive",   "TT gear-motor 3V-6V",        1, comp.name)
            BOM.add("Drive",   "65 mm rubber tyre + wheel", 1, comp.name)

        def bearing(comp, tag, cx, cy, cz, axis="x", ro=1.10, w=0.60):
            """Centred bearing cavity — v11 uses bearing_retention()."""
            cyl(comp, f"{tag}_BO", cx, cy, cz, ro,       w,      axis, chrome)
            cyl(comp, f"{tag}_BI", cx, cy, cz, ro*0.58,  w*0.80, axis, dark_grey)
            cyl(comp, f"{tag}_BB", cx, cy, cz, ro*0.32,  w*1.10, axis, chrome)
            temp  = adsk.fusion.TemporaryBRepManager.get()
            ax    = {"x": (1,0,0), "y": (0,1,0), "z": (0,0,1)}[axis]
            half  = w/2.0 + 0.05
            p1    = adsk.core.Point3D.create(cx-ax[0]*half, cy-ax[1]*half, cz-ax[2]*half)
            p2    = adsk.core.Point3D.create(cx+ax[0]*half, cy+ax[1]*half, cz+ax[2]*half)
            cs    = temp.createCylinderOrCone(p1, ro+0.05, p2, ro+0.05)
            bf    = comp.features.baseFeatures.add()
            bf.startEdit()
            cb    = comp.bRepBodies.add(cs, bf)
            bf.finishEdit()
            cb.name = f"{tag}_CB"
            cut_cavity(comp, cb)
            # v11: Add retention system
            bearing_retention(comp, tag, cx, cy, cz, axis, ro, w, fit_type="lip")
            size_tag = f"Ø{int(ro*2*10)} mm bearing"
            BOM.add("Bearing", size_tag, 1, comp.name)

        def u_bracket(comp, tag, cx, cy, cz, lx, ly, lz, ap=None):
            ap = ap or chrome
            box(comp, f"{tag}_BB",  cx,          cy,        cz, 0.45,     ly,   lz, ap)
            box(comp, f"{tag}_BL",  cx+lx*0.45,  cy+ly*0.35, cz, lx*0.55, 0.40, lz, ap)
            box(comp, f"{tag}_BR",  cx+lx*0.45,  cy-ly*0.35, cz, lx*0.55, 0.40, lz, ap)
            cyl(comp, f"{tag}_Pin", cx+lx*0.50,  cy,         cz, 0.18, ly*0.85, "y", chrome)
            # v11: Gusset reinforcement
            for gy in [ly*0.25, -ly*0.25]:
                box(comp, f"{tag}_Gusset_{gy:.0f}", cx+lx*0.30, cy+gy, cz, 0.25, 0.15, lz*0.7, ap)
            # v11: Fastener verification
            fastener_verify(comp, f"{tag}_Bracket", 0.3, 1.2, 0.8, lx, 0.0)
SECTION 5: COMPONENT BUILDING (Enhanced v11)
Python
        # ═════════════════════════════════════════════════════════════════
        # COMPONENT BUILDING
        # ═════════════════════════════════════════════════════════════════

        # ─────────────────────────────────────────────────────────────────
        # ① TORSO  (GEO-4, ELEC-1, ELEC-2, ELEC-3, v11 ribs, covers, hub)
        # ─────────────────────────────────────────────────────────────────
        torso = new_component("OP_Torso")

        # Main structural shell
        box(torso, "Torso_Shell",    0,    0,   TORSO_CTR,        10.4, 8.6, 12.2, op_red)
        box(torso, "Torso_Side_L",  -5.6,  0,   TORSO_CTR,         0.5, 7.8, 11.2, op_red)
        box(torso, "Torso_Side_R",   5.6,  0,   TORSO_CTR,         0.5, 7.8, 11.2, op_red)

        # v11: Internal structural ribs
        structural_rib(torso, "Torso_V1", (-4.0, 0, TORSO_CTR-5), (4.0, 0, TORSO_CTR+5))
        structural_rib(torso, "Torso_V2", (-4.0, 0, TORSO_CTR+5), (4.0, 0, TORSO_CTR-5))
        structural_rib(torso, "Torso_H1", (0, -3.0, TORSO_CTR-4), (0, 3.0, TORSO_CTR+4))

        # GEO-4 — Chest windows (stepped frame)
        box(torso, "CWin_Frame_L",  -2.3, -4.20, TORSO_CTR+2.5,   2.8, 0.32, 3.2, op_blue)
        box(torso, "CWin_Frame_R",   2.3, -4.20, TORSO_CTR+2.5,   2.8, 0.32, 3.2, op_blue)
        box(torso, "Chest_Win_L",   -2.3, -4.35, TORSO_CTR+2.5,   2.4, 0.20, 2.8, glass_clr)
        box(torso, "Chest_Win_R",    2.3, -4.35, TORSO_CTR+2.5,   2.4, 0.20, 2.8, glass_clr)
        box(torso, "Chest_Win_Div",  0,   -4.25, TORSO_CTR+2.5,   0.40, 0.22, 3.2, chrome)

        # GEO-4 — Horizontal grille rows (4 rows)
        for gz_offset, gw in [(-0.2, 7.4), (-1.0, 7.0), (-1.8, 6.6), (-2.6, 6.2)]:
            box(torso, f"Grille_{int(gz_offset*10)}",
                0, -4.40, TORSO_CTR+gz_offset, gw, 0.22, 0.30, chrome)

        # Headlight pods
        box(torso, "Headlight_L",   -4.4, -4.50, TORSO_CTR-1.2,  1.8, 0.28, 2.0, glass_clr)
        box(torso, "Headlight_R",    4.4, -4.50, TORSO_CTR-1.2,  1.8, 0.28, 2.0, glass_clr)

        # Front bumper (GEO-6 truck-cab)
        box(torso, "Front_Bumper",   0,   -5.8,  TORSO_CTR-4.4,  10.0, 2.0,  1.8, chrome)
        box(torso, "Hood_Crease_L", -2.5, -4.60, TORSO_CTR-2.8,   0.5, 0.35, 3.0, op_red)
        box(torso, "Hood_Crease_R",  2.5, -4.60, TORSO_CTR-2.8,   0.5, 0.35, 3.0, op_red)
        box(torso, "Ind_L",         -3.8, -5.00, TORSO_CTR-3.8,   0.6, 0.25, 0.5, yellow_met)
        box(torso, "Ind_R",          3.8, -5.00, TORSO_CTR-3.8,   0.6, 0.25, 0.5, yellow_met)

        # GEO-4 — Chest plate + Autobot badge with LED ring
        box(torso, "Chest_Plate",    0,   -4.20, TORSO_CTR+0.5,   8.4, 0.32, 4.0, chrome)
        cyl(torso, "Badge_Ring",     0,   -4.55, TORSO_CTR+0.5,   0.80, 0.12, "y", op_red)
        led_ring_pocket(torso, "Badge",  0, -4.60, TORSO_CTR+0.5, "y")

        # Inner structural frame
        box(torso, "Inner_Frame",    0,    0,    TORSO_CTR+1.5,   7.4, 6.0,  8.2, dark_metal)
        box(torso, "Spine_Beam",     0,    0,    TORSO_CTR+1.5,   1.8, 1.8,  8.2, chrome)
        cyl(torso, "Spine_Cyl",      0,    0,    TORSO_CTR+1.5,  1.10, 4.5,  "z", chrome)

        # ELEC-3 — LiPo bay (rear of lower torso) + v11 power system
        box(torso, "LipoBay_Shell",  0,    3.2, TORSO_CTR-2.0,   7.6, 4.4,  5.6, black_plastic)
        lipo_bay(torso, "Main", 0, 2.8, TORSO_CTR-2.0)
        # v11: Battery access cover
        access_cover(torso, "LipoCover", 0, 3.2, TORSO_CTR-2.0 + 2.9,
                     LIPO_L + 0.5, LIPO_W + 0.5, 0.25, retention="hybrid")

        # ELEC-1 — RPi Zero 2W bay
        box(torso, "RPi_Shell",      0,    2.8, TORSO_CTR+1.8,   7.0, 3.6,  2.8, black_plastic)
        rpi_zero_bay(torso, "Main",  0,    3.2, TORSO_CTR+1.8)
        access_cover(torso, "RPiCover", 0, 2.8, TORSO_CTR+1.8 + 1.5,
                     RPI0_L + 0.4, RPI0_W + 0.4, 0.25, retention="magnet")

        # ELEC-2 — PCA9685 bay
        box(torso, "PCA_Shell",      0,    2.8, TORSO_CTR+4.2,   6.8, 3.2,  2.2, black_plastic)
        pca9685_bay(torso, "Main",   0,    3.0, TORSO_CTR+4.2)
        access_cover(torso, "PCACover", 0, 2.8, TORSO_CTR+4.2 + 1.2,
                     PCA_L + 0.4, PCA_W + 0.4, 0.25, retention="screw")

        # v11: Central wire hub
        wire_hub(torso, "TorsoHub", 0, 1.5, TORSO_CTR, n_ports=8)

        # Cable management with v11 clips
        box(torso, "Cable_L",       -3.4,  0.6,  TORSO_CTR,      0.55, 1.0, 10.0, dark_grey)
        box(torso, "Cable_R",        3.4,  0.6,  TORSO_CTR,      0.55, 1.0, 10.0, dark_grey)
        for cz_clip in [TORSO_CTR-3, TORSO_CTR, TORSO_CTR+3]:
            cable_clip(torso, f"TorsoClip_{cz_clip:.0f}", -3.4, 0.6, cz_clip, "z", 0.50, 4)
            cable_clip(torso, f"TorsoClipR_{cz_clip:.0f}", 3.4, 0.6, cz_clip, "z", 0.50, 4)

        # Shoulder collars
        box(torso, "Collar_L",      -8.0,  0,    SHOULDER_CTR-1.0, 5.5, 3.4, 3.0, chrome)
        box(torso, "Collar_R",       8.0,  0,    SHOULDER_CTR-1.0, 5.5, 3.4, 3.0, chrome)

        # Transformation flaps
        box(torso, "TF_Flap_L",     -5.40,-0.2,  TORSO_CTR+3.0,  0.40, 6.6,  6.2, op_red)
        box(torso, "TF_Flap_R",      5.40,-0.2,  TORSO_CTR+3.0,  0.40, 6.6,  6.2, op_red)
        box(torso, "TF_BackTop",     0,    5.0,  TORSO_CTR+5.2,  8.2, 0.38,  5.2, op_blue)

        # v11: Transformation locking pins
        locking_pin(torso, "TFLock_F", 0, -4.5, TORSO_CTR-2.0, "y", 1.2)
        locking_pin(torso, "TFLock_R", 0,  4.5, TORSO_CTR-2.0, "y", 1.2)

        # M3 boss fasteners
        for bx_off, bz_off in [(-3.2, 4.8), (3.2, 4.8), (-3.2, -4.8), (3.2, -4.8)]:
            m3_boss(torso, f"Torso_B{bx_off}_{bz_off}", bx_off, 0, TORSO_CTR+bz_off)
        align_pin(torso, "TorsoSplit_L", -5.0, 0, TORSO_CTR)
        align_pin(torso, "TorsoSplit_R",  5.0, 0, TORSO_CTR)

        # v11: Verify fasteners
        fastener_verify(torso, "Torso_Boss", 0.3, 0.8, 0.5, 0.5, 0.0)

        # Waist servo cluster with v11 dual bearing
        u_bracket(torso, "Waist_Brkt",  0, 0, WAIST_CTR,     4.0, 4.2, 3.4)
        mg996r(torso,    "Waist_Yaw",   0, 0, WAIST_CTR,     "z")
        bearing(torso,   "Waist_Brg",   0, 0, WAIST_CTR+0.5, "z", 1.30, 0.65)
        bearing_retention(torso, "Waist_Brg2", 0, 0, WAIST_CTR-0.5, "z", 1.30, 0.65, "lip")
        u_bracket(torso, "WaistP_Brkt", 0, 0, WAIST_CTR-2.5, 4.0, 4.2, 3.4)
        mg996r(torso,    "Waist_Pitch", 0, 0, WAIST_CTR-2.5, "x")
        bearing(torso,   "WaistP_Brg",  0, 0, WAIST_CTR-2.0, "x", 1.30, 0.65)
        magnet_pocket(torso, "WLock_F", 0, -2.0, WAIST_CTR-3.0)
        magnet_pocket(torso, "WLock_R", 0,  2.0, WAIST_CTR-3.0)

        # v11: Mechanical hard stop for waist
        box(torso, "WaistStop_F", 0, -2.3, WAIST_CTR-1.0, 1.5, 0.25, 1.0, dark_metal)
        box(torso, "WaistStop_R", 0,  2.3, WAIST_CTR-1.0, 1.5, 0.25, 1.0, dark_metal)

        # Neck servo cluster
        u_bracket(torso, "Neck_Brkt",   0, 0, NECK_JOINT_Z,  3.2, 2.8, 3.0)
        mg996r(torso,    "Neck_Pitch",  0, 0, NECK_JOINT_Z,  "x")
        wire_channel(torso, "Spine",    0, 0, TORSO_CTR,     0.6, 20.0, "z")

        # v11: Assembly jig for torso
        assembly_jig("Torso",
                     [(-5.0, 0, TORSO_CTR), (5.0, 0, TORSO_CTR),
                      (-3.2, 0, TORSO_CTR+4.8), (3.2, 0, TORSO_CTR+4.8)],
                     12.0, 14.0)

        # ─────────────────────────────────────────────────────────────────
        # ② HEAD  (GEO-1, v11 ESP32 access cover)
        # ─────────────────────────────────────────────────────────────────
        head = new_component("OP_Head")
        HC   = HEAD_CTR

        box(head, "Helm_Main",      0,     0,    HC+0.8,  5.6, 5.2, 5.0, op_blue)
        box(head, "Helm_Forehead",  0,    -2.30, HC+2.6,  5.0, 0.42, 1.8, op_blue)
        box(head, "Helm_Top",       0,     0,    HC+3.4,  4.8, 4.4, 0.90, op_blue)

        box(head, "Ear_L",         -3.30,  0,    HC+1.8,  0.70, 4.8, 3.8, op_blue)
        box(head, "Ear_R",          3.30,  0,    HC+1.8,  0.70, 4.8, 3.8, op_blue)
        box(head, "EarTop_L",      -3.05,  0,    HC+3.5,  1.40, 4.4, 0.62, op_blue)
        box(head, "EarTop_R",       3.05,  0,    HC+3.5,  1.40, 4.4, 0.62, op_blue)

        box(head, "Crest_Main",     0,    -0.30, HC+3.85, 1.05, 0.68, 3.6, chrome)
        box(head, "Crest_Stripe",   0,    -0.30, HC+3.95, 0.55, 0.36, 2.9, op_blue)

        box(head, "Face_Plate",     0,    -2.50, HC+0.5,  3.6, 0.38, 3.2, chrome)

        box(head, "Chin_Guard",     0,    -2.60, HC-0.9,  3.0, 0.38, 1.8, chrome)
        box(head, "Chin_L",        -1.35, -2.52, HC-0.4,  0.38, 0.32, 2.2, chrome)
        box(head, "Chin_R",         1.35, -2.52, HC-0.4,  0.38, 0.32, 2.2, chrome)

        # v11: Chamfered chin for printability
        chamfer_box(head, "Chin_Guard_Chamf", 0, -2.60, HC-0.9, 3.0, 0.38, 1.8, 0.08, "z", chrome)

        box(head, "Visor_Frame",    0,    -2.55, HC+1.35, 3.8, 0.30, 1.25, op_blue)
        box(head, "Visor",          0,    -2.68, HC+1.35, 3.0, 0.16, 0.92, op_blue_glass)
        for vx in [-0.85, 0.0, 0.85]:
            led_pocket_5mm(head, f"Visor_LED_{int(vx*100)}", vx, -2.80, HC+1.35, "y")

        box(head, "Nose_Bridge",    0,    -2.60, HC+1.95, 0.72, 0.22, 0.72, chrome)
        box(head, "Mouth_Plate",    0,    -2.55, HC+0.10, 2.4, 0.22, 1.10, dark_grey)
        for mz in [-0.32, 0.0, 0.32]:
            box(head, f"MouthGrill_{int(mz*100)}",
                0, -2.62, HC+0.10+mz, 1.8, 0.12, 0.18, chrome)

        box(head, "Head_Rear",      0,    1.90,  HC+1.0,  4.2, 1.5, 4.4, op_red)
        box(head, "Neck_Collar",    0,    0,     HC-1.6,  2.5, 2.5, 2.4, dark_metal)

        cyl(head, "Ant_L",         -2.80, 0,     HC+4.4, 0.13, 2.7, "z", chrome)
        cyl(head, "Ant_R",          2.80, 0,     HC+4.4, 0.13, 2.7, "z", chrome)
        cyl(head, "AntTip_L",      -2.80, 0,     HC+5.8, 0.24, 0.34, "z", gold_met)
        cyl(head, "AntTip_R",       2.80, 0,     HC+5.8, 0.24, 0.34, "z", gold_met)

        esp32_cam_pocket(head, "HeadCam", 0, -1.6, HC+2.5, "y")
        # v11: ESP32 access cover
        access_cover(head, "ESP32Cover", 0, -1.6, HC+2.5 - 0.5,
                     ESP32_L + 0.3, ESP32_W + 0.3, 0.20, retention="snap")

        mg90s(head, "Neck_Yaw", 0, 0, NECK_JOINT_Z, "z")

        # v11: Head assembly jig
        assembly_jig("Head",
                     [(-2.5, 0, HC), (2.5, 0, HC)],
                     7.0, 8.0)

        # ─────────────────────────────────────────────────────────────────
        # ③ PELVIS  (ELEC-6, v11 ribs, locking)
        # ─────────────────────────────────────────────────────────────────
        pelvis = new_component("OP_Pelvis")
        box(pelvis, "Pelvis_Shell",  0,    0,  PELVIS_CTR,  16.4, 6.2, 5.0, op_blue)
        box(pelvis, "Pelvis_Frame",  0,    0,  PELVIS_CTR,  12.2, 4.4, 3.8, dark_metal)
        box(pelvis, "Hip_Armr_L",  -7.2,  0,  PELVIS_CTR,   1.0, 5.2, 4.2, chrome)
        box(pelvis, "Hip_Armr_R",   7.2,  0,  PELVIS_CTR,   1.0, 5.2, 4.2, chrome)
        box(pelvis, "Crotch_Plate", 0,  -2.9, PELVIS_CTR-1.2, 5.2, 0.30, 2.4, op_red)

        # v11: Structural ribs
        structural_rib(pelvis, "Pelvis_V1", (-5.0, 0, PELVIS_CTR-2), (5.0, 0, PELVIS_CTR+2))
        structural_rib(pelvis, "Pelvis_H1", (0, -2.5, PELVIS_CTR-2), (0, 2.5, PELVIS_CTR+2))

        imu_pocket(pelvis, "BalanceIMU", 0, 0, PELVIS_CTR)

        mg996r(pelvis, "L_HipYaw", -HIP_X, 0, HIP_JOINT_Z, "z")
        mg996r(pelvis, "R_HipYaw",  HIP_X, 0, HIP_JOINT_Z, "z")
        bearing(pelvis, "L_HYB",  -HIP_X-2.2, 0, HIP_JOINT_Z, "z", 1.10, 0.62)
        bearing(pelvis, "R_HYB",   HIP_X+2.2, 0, HIP_JOINT_Z, "z", 1.10, 0.62)

        # v11: v11 locking pins for transformation
        locking_pin(pelvis, "HipLock_L", -HIP_X, -3.0, PELVIS_CTR, "y", 1.0)
        locking_pin(pelvis, "HipLock_R",  HIP_X, -3.0, PELVIS_CTR, "y", 1.0)

        BOM.add("Servo", "DS3225MG 25 kg·cm servo (hip yaw)", 2, "OP_Pelvis")

        # v11: Pelvis assembly jig
        assembly_jig("Pelvis",
                     [(-HIP_X, 0, HIP_JOINT_Z), (HIP_X, 0, HIP_JOINT_Z)],
                     18.0, 8.0)
SECTION 6: LEGS & ARMS (Enhanced v11 with Tendons, Couplers, Covers)
Python
        # ─────────────────────────────────────────────────────────────────
        # ④ LEGS  (GEO-5 improved feet, v11 tendon guides, covers, jigs)
        # ─────────────────────────────────────────────────────────────────
        for side, sx in [("L", -HIP_X), ("R", HIP_X)]:
            m = -1 if side == "L" else 1

            # —— THIGH ——
            thigh = new_component(f"OP_Thigh_{side}")
            box(thigh, "Thigh_Link",         sx,        0,  THIGH_CTR,      5.0, 4.0, 11.0, chrome)
            box(thigh, "Thigh_Outer",        sx+m*2.65, 0,  THIGH_CTR,      0.50, 4.4, 11.0, op_red)
            box(thigh, "Thigh_Front",        sx,       -2.2, THIGH_CTR,     5.0, 0.40, 11.0, op_blue)
            box(thigh, "Thigh_Knee_Armor",   sx,       -2.2, KNEE_CTR+2.5,  4.2, 0.80,  2.8, chrome)

            # v11: Structural ribs
            structural_rib(thigh, f"{side}_Thigh_Rib1",
                          (sx, 0, THIGH_CTR-4), (sx, 0, THIGH_CTR+4))
            structural_rib(thigh, f"{side}_Thigh_Rib2",
                          (sx-1.5, 0, THIGH_CTR), (sx+1.5, 0, THIGH_CTR))

            u_bracket(thigh, f"{side}_HPB",  sx, 0, HIP_JOINT_Z+0.5,        4.0, 3.2, 3.2)
            mg996r(thigh, f"{side}_HipP",    sx, 0, HIP_JOINT_Z,             "x")
            mg996r(thigh, f"{side}_HipR",    sx, 0, THIGH_CTR+2.0,           "y")
            bearing(thigh, f"{side}_HRB",    sx, 0, THIGH_CTR+2.0,           "y", 1.00, 0.55)
            u_bracket(thigh, f"{side}_KnB",  sx, 0, KNEE_CTR+1.5,            3.8, 3.0, 3.0)
            mg996r(thigh, f"{side}_KneP",    sx, 0, KNEE_CTR+1.5,            "x")
            bearing(thigh, f"{side}_KB",     sx, 0, KNEE_CTR,                "x", 1.00, 0.55)
            wire_channel(thigh, f"{side}_LW", sx, 0, THIGH_CTR,              0.5, 12.0, "z")

            # v11: Cable clips along thigh
            for tz in [THIGH_CTR-3, THIGH_CTR, THIGH_CTR+3]:
                cable_clip(thigh, f"{side}_ThighClip_{tz:.0f}", sx, -1.5, tz, "z", 0.40, 3)

            for bz in [THIGH_CTR+3.0, THIGH_CTR-3.0]:
                m3_boss(thigh, f"{side}_ThighBoss_{bz:.0f}", sx, 0, bz)
            magnet_pocket(thigh, f"{side}_KLU", sx, -1.5, KNEE_CTR+1.0)
            magnet_pocket(thigh, f"{side}_KLL", sx,  1.5, KNEE_CTR+1.0)

            # v11: Hard stop
            box(thigh, f"{side}_KneeStop", sx, -2.0, KNEE_CTR+0.5, 3.0, 0.30, 1.5, dark_metal)
            BOM.add("Servo", "DS3225MG 25 kg·cm servo (hip pitch)", 1, f"OP_Thigh_{side}")

            # —— SHIN ——
            shin = new_component(f"OP_Shin_{side}")
            box(shin, "Shin_Link",    sx,    0,    SHIN_CTR,  4.4, 6.0, 11.0, op_blue)
            box(shin, "Shin_Armor",   sx,   -2.7,  SHIN_CTR,  3.2, 0.34,  9.2, chrome)
            box(shin, "Shin_Rear",    sx,    2.7,  SHIN_CTR,  2.0, 0.34,  9.8, dark_grey)
            box(shin, "Shin_Beam",    sx,    0.4,  SHIN_CTR,  1.8, 2.2, 10.2, dark_metal)
            box(shin, "KneeCap",      sx,   -2.9,  KNEE_CTR-1.0, 3.0, 0.55, 2.2, chrome)
            tt_wheel(shin, f"{side}_WF", sx+m*2.0, -4.2, SHIN_CTR+4.0, m)
            tt_wheel(shin, f"{side}_WR", sx+m*2.0, -4.2, SHIN_CTR-4.0, m)
            bearing(shin, f"{side}_KLB", sx, 0, KNEE_CTR-0.5, "x", 1.00, 0.55)
            wire_channel(shin, f"{side}_SW", sx, 0, SHIN_CTR, 0.5, 11.0, "z")
            cut_cavity(shin, box(shin, "FtTuck", sx, 2.7, SHIN_CTR-3.5, 5.2, 1.3, 4.2))
            magnet_pocket(shin, f"{side}_KU", sx, -1.5, KNEE_CTR-1.0)
            magnet_pocket(shin, f"{side}_KL", sx,  1.5, KNEE_CTR-1.0)
            for bz in [SHIN_CTR+3.5, SHIN_CTR-3.5]:
                m3_boss(shin, f"{side}_ShinBoss_{bz:.0f}", sx, 0, bz)

            # v11: Cable clips
            for sz in [SHIN_CTR-3, SHIN_CTR, SHIN_CTR+3]:
                cable_clip(shin, f"{side}_ShinClip_{sz:.0f}", sx, -2.0, sz, "z", 0.40, 3)

            # —— FOOT  (GEO-5 + v11 chamfers, locking) ——
            foot = new_component(f"OP_Foot_{side}")
            box(foot, "Foot_Sole",    sx,       -0.6,  ANKLE_CTR-1.5,  6.2, 9.2, 1.10, op_red)
            for vi in [-1.0, 0.0, 1.0]:
                box(foot, f"Arch_Vent_{int(vi*10)}",
                    sx+vi*1.2, -0.6, ANKLE_CTR-1.94, 0.40, 5.5, 0.14, dark_grey)

            # v11: Chamfered heel for printability
            chamfer_box(foot, "Heel_Block_C", sx-m*0.9,  3.2,  ANKLE_CTR-0.8,
                        2.5, 3.5, 2.6, 0.10, "z", dark_grey)
            box(foot, "Heel_Plate",   sx-m*0.6,  4.4,  ANKLE_CTR-1.2,  3.2, 0.32, 2.0, chrome)
            box(foot, "Heel_Spur",    sx-m*1.0,  4.8,  ANKLE_CTR-0.2,  1.2, 0.40, 3.2, op_red)

            box(foot, "Toe_Block",    sx+m*0.8, -3.8,  ANKLE_CTR-0.8,  2.6, 3.8, 2.0, dark_grey)
            box(foot, "Toe_Plate",    sx+m*0.5, -4.6,  ANKLE_CTR-1.2,  3.8, 0.32, 1.8, chrome)
            box(foot, "Toe_Cap",      sx+m*1.0, -5.2,  ANKLE_CTR-0.8,  2.8, 0.42, 1.5, op_red)

            box(foot, "Ankle_Guard",  sx,        0,    ANKLE_CTR+1.2,  5.4, 3.2, 2.8, chrome)
            box(foot, "Ankle_Inner",  sx,       -1.0,  ANKLE_CTR+0.3,  4.0, 2.0, 1.6, dark_metal)

            # v11: Boot fin with chamfer
            chamfer_box(foot, "Boot_Fin_C", sx+m*2.0,  0,    ANKLE_CTR-0.2,
                        0.40, 6.5, 4.2, 0.06, "z", op_blue)
            box(foot, "Boot_Fin2",    sx+m*2.5,  0,    ANKLE_CTR+0.8,  0.32, 5.0, 2.8, op_red)

            mg996r(foot, f"{side}_AnkP", sx, 0, ANKLE_CTR+2.2, "x")
            mg996r(foot, f"{side}_AnkR", sx, 0, ANKLE_CTR+0.5, "y")
            bearing(foot, f"{side}_AB",  sx, 0, ANKLE_CTR,     "x", 1.00, 0.55)

            # v11: v11 cable clips in foot
            cable_clip(foot, f"{side}_FootClip_1", sx, 0, ANKLE_CTR+1.0, "z", 0.35, 3)
            cable_clip(foot, f"{side}_FootClip_2", sx, 0, ANKLE_CTR-0.5, "z", 0.35, 3)

            for bx_off in [-1.5, 1.5]:
                m3_boss(foot, f"{side}_FootBoss_{bx_off:.0f}", sx+bx_off, 0, ANKLE_CTR-0.5)
            align_pin(foot, f"{side}_Ft_A", sx-1.0, 0, ANKLE_CTR-1.3)
            align_pin(foot, f"{side}_Ft_B", sx+1.0, 0, ANKLE_CTR-1.3)

            # v11: Leg assembly jig
            assembly_jig(f"Leg_{side}",
                         [(sx, 0, KNEE_CTR), (sx, 0, ANKLE_CTR)],
                         8.0, 18.0)

        # ─────────────────────────────────────────────────────────────────
        # ⑤ ARMS  (GEO-3 shoulders + GEO-2 articulated hands + v11 tendons)
        # ─────────────────────────────────────────────────────────────────
        for side, ax in [("L", -SHOULDER_X), ("R", SHOULDER_X)]:
            m = -1 if side == "L" else 1

            # —— UPPER ARM ——
            ua = new_component(f"OP_UpperArm_{side}")

            box(ua, "Sh_Block",        ax,         0, SHOULDER_CTR,      5.6, 4.2, 5.6, op_red)
            box(ua, "Sh_Pad_Outer",    ax+m*3.20,  0, SHOULDER_CTR,      1.00, 5.2, 5.8, op_red)
            box(ua, "Sh_Pad_Edge",     ax+m*3.75,  0, SHOULDER_CTR,      0.40, 4.6, 5.2, chrome)

            # v11: Chamfered shoulder spikes for printability
            for sz, sr, sh2 in [(SHOULDER_CTR+2.8, 0.42, 0.95),
                                 (SHOULDER_CTR+1.5, 0.36, 0.75)]:
                cyl(ua, f"Stk_A_{sz:.0f}", ax+m*3.3, -1.5, sz, sr,  sh2, "z", chrome)
                cone_shape(ua, f"StkTip_{sz:.0f}", ax+m*3.3, -1.5, sz+sh2/2+0.35,
                           sr, sr*0.25, 0.70, "z", dark_grey)

            box(ua, "Sh_Guard",        ax+m*2.60,  0, SHOULDER_CTR-0.2, 0.42, 4.4, 6.6, op_blue)

            box(ua, "UA_Link",         ax,         0, ELBOW_Z+3.0,      3.2, 3.4, 5.2, op_red)
            box(ua, "UA_Skin",         ax+m*1.80,  0, ELBOW_Z+3.0,      0.52, 3.4, 5.2, chrome)

            # v11: Structural rib
            structural_rib(ua, f"{side}_UA_Rib", (ax, 0, SHOULDER_CTR), (ax, 0, ELBOW_Z))

            # Shoulder servos (3 DOF) with v11 couplers
            mg996r(ua, f"{side}_ShY",    ax, 0, SHOULDER_CTR+1.5, "z")
            bearing(ua, f"{side}_SYB",   ax, 0, SHOULDER_CTR+2.0, "z", 1.00, 0.55)
            u_bracket(ua, f"{side}_SPB", ax, 0, SHOULDER_CTR,     4.8, 3.4, 3.4)
            mg996r(ua, f"{side}_ShP",    ax, 0, SHOULDER_CTR,     "x")
            mg996r(ua, f"{side}_ShR",    ax, 0, SHOULDER_CTR-1.2, "y")
            bearing(ua, f"{side}_SB",    ax, 0, SHOULDER_CTR,     "x", 1.10, 0.62)

            # v11: Mechanical hard stop
            box(ua, f"{side}_ShStop", ax, -2.0, SHOULDER_CTR, 2.5, 0.25, 1.0, dark_metal)

            # Elbow joint
            u_bracket(ua, f"{side}_EB",  ax, 0, ELBOW_Z,          3.8, 3.0, 3.0)
            mg996r(ua, f"{side}_ElbP",   ax, 0, ELBOW_Z,          "x")
            bearing(ua, f"{side}_EBr",   ax, 0, ELBOW_Z-0.5,      "x", 0.95, 0.52)
            wire_channel(ua, f"{side}_UAW", ax, 0, ELBOW_Z+3.0,   0.4, 5.2, "z")
            m3_boss(ua, f"{side}_UAboss", ax, 0, ELBOW_Z+3.0)

            # v11: Cable clips
            for ez in [ELBOW_Z+1, ELBOW_Z+4]:
                cable_clip(ua, f"{side}_UAClip_{ez:.0f}", ax, -1.2, ez, "z", 0.35, 3)

            # —— FOREARM ——
            fa = new_component(f"OP_Forearm_{side}")
            box(fa, "FA_Link",    ax,       0,    WRIST_Z+3.5, 3.2, 3.8, 4.8, op_blue)
            box(fa, "FA_Fender",  ax+m*2.1, 0,    WRIST_Z+3.5, 0.52, 5.2, 5.8, op_red)
            box(fa, "FA_Back",    ax,       2.3,  WRIST_Z+3.5, 2.6, 0.38, 4.8, chrome)
            box(fa, "FA_Vent_L",  ax-0.6,  -1.8,  WRIST_Z+3.5, 0.30, 0.22, 3.0, dark_grey)
            box(fa, "FA_Vent_R",  ax+0.6,  -1.8,  WRIST_Z+3.5, 0.30, 0.22, 3.0, dark_grey)
            mg90s(fa, f"{side}_WR",   ax, 0, WRIST_Z+0.8, "x")
            bearing(fa, f"{side}_WB", ax, 0, WRIST_Z+0.5, "x", 0.80, 0.44)
            wire_channel(fa, f"{side}_FAW", ax, 0, WRIST_Z+3.5, 0.4, 4.8, "z")
            m3_boss(fa, f"{side}_FAboss", ax, 0, WRIST_Z+4.2)

            # v11: Cable clips
            for wz in [WRIST_Z+2, WRIST_Z+5]:
                cable_clip(fa, f"{side}_FAClip_{wz:.0f}", ax, -1.5, wz, "z", 0.35, 3)

            # —— HAND (palm) with v11 tendon system ——
            hand = new_component(f"OP_Hand_{side}")
            box(hand, "Palm_Main",   ax,         -0.4,  WRIST_Z-1.6, 3.4, 3.0, 2.2, dark_grey)
            box(hand, "Palm_Inner",  ax,          0.6,  WRIST_Z-1.6, 2.6, 2.0, 2.0, black_plastic)

            MCP_Z = WRIST_Z - PALM_BOTTOM_OFFSET
            for ki, kx_off in enumerate(FING_X_OFF):
                cyl(hand, f"Knuckle_{ki}",
                    ax + m*kx_off, -1.5, MCP_Z + 0.15, 0.28, 0.38, "y", chrome)

            cyl(hand, "Wrist_Ring",  ax, 0, WRIST_Z-0.4, 1.05, 0.42, "z", chrome)
            box(hand, "Hand_Panel",  ax+m*0.9, -0.7, WRIST_Z-1.3, 0.38, 2.8, 2.8, op_red)

            # v11: Finger servo bay with drum
            box(hand, "Finger_SrvBay", ax, 0.5, WRIST_Z-1.8, 1.4, 0.90, 2.4, black_plastic)
            servo_drum(hand, f"{side}_FingerDrum", ax, 0.5, WRIST_Z-1.2, "y")

            # v11: Tendon guides in palm
            for fi, fxo in enumerate(FING_X_OFF):
                guide_x = ax + m * fxo
                tendon_guide(hand, f"{side}_PalmGuide_{fi}", guide_x, -0.8, MCP_Z, "z")

            # v11: LED strip pocket
            for lxi in [-0.7, 0.7]:
                led_pocket_5mm(hand, f"PalmLED_{lxi:.0f}", ax+lxi, -1.65, WRIST_Z-1.6, "y")

            BOM.add("Servo", "DS04-NFC 9g digital servo (finger drive)", 2, f"OP_Hand_{side}")

            # GEO-2 + v11 Tendon-driven fingers
            for fi, (fname, fxo, pp_l, mp_l, dp_l) in enumerate(
                    zip(FING_NAMES, FING_X_OFF, FING_PP, FING_MP, FING_DP)):

                fc   = new_component(f"OP_{fname}_{side}")
                fx   = ax + m * fxo
                fy   = -1.35
                mcp_z = MCP_Z
                pp_cz = mcp_z - pp_l / 2
                mp_cz = mcp_z - pp_l - mp_l / 2
                dp_cz = mcp_z - pp_l - mp_l - dp_l / 2
System is currently busy. Please try again later.
continue
Edit
Copy
Share
I'll continue building the complete v11 script. Let me write the remaining sections directly to the output file.
SECTION 7: FINGERS, BACKPACK, STEER PODS, SHIELDS (Continued)
Python
                # Proximal phalanx
                box(fc, "PP", fx, fy, pp_cz, FING_W, FING_H, pp_l, grey_plastic)
                # PIP hinge cylinder
                cyl(fc, "PIP_Hinge", fx, fy, mcp_z - pp_l, 0.27, FING_W + 0.12, "x", chrome)
                # Middle phalanx
                box(fc, "MP", fx, fy, mp_cz, FING_W*0.94, FING_H*0.94, mp_l, grey_plastic)
                # DIP hinge cylinder
                cyl(fc, "DIP_Hinge", fx, fy, mcp_z - pp_l - mp_l,
                    0.24, FING_W*0.94 + 0.10, "x", chrome)
                # Distal phalanx
                box(fc, "DP", fx, fy, dp_cz, FING_W*0.86, FING_H*0.86, dp_l, grey_plastic)
                # Fingertip cone
                cone_shape(fc, "FT", fx, fy, mcp_z - pp_l - mp_l - dp_l - 0.12,
                           FING_W*0.44, 0.05, 0.24, "z", chrome)

                # v11: Tendon anchor on distal phalanx
                tendon_anchor(fc, f"{side}_{fname}", fx, fy, dp_cz - dp_l/2 - 0.15, "z")
                # v11: Tendon guide on middle phalanx
                tendon_guide(fc, f"{side}_{fname}_Mid", fx, fy, mp_cz, "z")
                # v11: Tendon guide on proximal phalanx
                tendon_guide(fc, f"{side}_{fname}_Prox", fx, fy, pp_cz, "z")
                # v11: Cable routing groove
                wire_channel(fc, fname, fx, fy, pp_cz, 0.08, pp_l*2.3, "z")
                # v11: Spring return pocket on proximal phalanx (back side)
                spring_return(fc, f"{side}_{fname}_Spr", fx, fy + 0.3, pp_cz + pp_l/2, "y")

            # GEO-2 + v11 Thumb (2-phalanx, angled outward)
            thumb = new_component(f"OP_Thumb_{side}")
            tx    = ax + m * 1.70
            ty    = 0.20
            tpp_cz = MCP_Z - THUMB_PP_L / 2
            tdp_cz = MCP_Z - THUMB_PP_L - THUMB_DP_L / 2
            box(thumb, "TP",  tx, ty, tpp_cz, THUMB_W, THUMB_H, THUMB_PP_L, grey_plastic)
            cyl(thumb, "T_Hinge", tx, ty, MCP_Z - THUMB_PP_L,
                0.28, THUMB_W + 0.14, "x", chrome)
            box(thumb, "TD",  tx, ty*0.8, tdp_cz, THUMB_W*0.86, THUMB_H*0.86, THUMB_DP_L,
                grey_plastic)
            cone_shape(thumb, "TT", tx, ty, tdp_cz - THUMB_DP_L/2 - 0.14,
                       THUMB_W*0.44, 0.05, 0.28, "z", chrome)

            # v11: Thumb tendon system
            tendon_anchor(thumb, f"{side}_Thumb", tx, ty, tdp_cz - THUMB_DP_L/2 - 0.20, "z")
            tendon_guide(thumb, f"{side}_ThumbGuide", tx, ty, tpp_cz, "z")
            spring_return(thumb, f"{side}_ThumbSpr", tx, ty + 0.3, tpp_cz + THUMB_PP_L/2, "y")

            # —— ION BLASTER (right arm only) ——
            if side == "R":
                blast = new_component("OP_Ion_Blaster")
                cyl(blast, "Barrel_Main",   ax,    -2.2, WRIST_Z-3.2, 0.92, 4.2, "z", dark_metal)
                cyl(blast, "Barrel_Tip",    ax,    -2.2, WRIST_Z-5.6, 0.68, 1.1, "z", chrome)
                cyl(blast, "Barrel_Inner",  ax,    -2.2, WRIST_Z-3.8, 0.44, 2.4, "z", black_plastic)
                box(blast, "Blaster_Body",  ax,    -1.1, WRIST_Z-3.2, 2.6, 2.4, 2.7, dark_metal)
                box(blast, "Blast_Guard",   ax,    -0.2, WRIST_Z-3.2, 2.8, 0.38, 1.6, chrome)
                box(blast, "Blast_Rail_T",  ax,    -0.6, WRIST_Z-2.0, 2.8, 0.22, 0.30, chrome)
                box(blast, "Blast_Rail_B",  ax,    -0.6, WRIST_Z-4.4, 2.8, 0.22, 0.30, chrome)
                box(blast, "Hinge_Block",   ax,    -0.8, WRIST_Z-1.6, 1.1, 0.65, 1.1, dark_metal)
                cyl(blast, "Scope_Body",    ax+1.5,-2.2, WRIST_Z-3.2, 0.42, 2.2, "z", chrome)
                cyl(blast, "Scope_Lens",    ax+1.5,-2.2, WRIST_Z-4.4, 0.28, 0.25, "z", glass_clr)
                led_pocket_5mm(blast, "Muzzle", ax, -2.2, WRIST_Z-6.2, "z")

                # v11: Blaster locking pin for transformation
                locking_pin(blast, "BlastLock", ax, -0.5, WRIST_Z-1.0, "y", 0.8)

            # v11: Arm assembly jig
            assembly_jig(f"Arm_{side}",
                         [(ax, 0, SHOULDER_CTR), (ax, 0, ELBOW_Z), (ax, 0, WRIST_Z)],
                         10.0, 16.0)

        # ─────────────────────────────────────────────────────────────────
        # ⑥ BACKPACK (v11 locking, access)
        # ─────────────────────────────────────────────────────────────────
        bp = new_component("OP_Backpack")
        box(bp, "BP_Core",    0, 5.6, TORSO_CTR+0.5, 7.2, 2.6,  9.2, dark_grey)
        box(bp, "BP_Hood",    0, 6.5, TORSO_CTR+1.0, 5.8, 1.1,  7.8, op_red)
        box(bp, "BP_TopFlap", 0, 5.1, TORSO_CTR+5.5, 8.4, 0.38, 5.4, op_red)
        box(bp, "BP_Rad",     0, 6.9, TORSO_CTR-0.5, 5.4, 0.44, 5.7, chrome)
        box(bp, "Exh_Blk",   0, 6.3, TORSO_CTR+2.8, 3.2, 0.65, 2.0, dark_metal)
        cyl(bp,  "Exh_L",   -1.3, 6.7, TORSO_CTR+2.8, 0.40, 1.3, "y", dark_metal)
        cyl(bp,  "Exh_R",    1.3, 6.7, TORSO_CTR+2.8, 0.40, 1.3, "y", dark_metal)
        mg90s(bp,    "Roof_Hinge", 0, 5.1, TORSO_CTR+5.1, "x")
        bearing(bp,  "Roof_Brg",   0, 5.1, TORSO_CTR+5.3, "x", 0.80, 0.44)
        magnet_pocket(bp, "RoofL", -2.6, 5.1, TORSO_CTR+5.7)
        magnet_pocket(bp, "RoofR",  2.6, 5.1, TORSO_CTR+5.7)

        # v11: Backpack locking pin
        locking_pin(bp, "BPLock", 0, 5.5, TORSO_CTR+4.5, "y", 1.0)

        # ─────────────────────────────────────────────────────────────────
        # ⑦ STEER WHEEL PODS
        # ─────────────────────────────────────────────────────────────────
        steer = new_component("OP_SteerPods")
        for side, sx in [("L", -5.6), ("R", 5.6)]:
            m2 = -1 if side == "L" else 1
            box(steer, f"SAr_{side}",  sx, -3.6, 23.9, 1.6, 1.3, 4.2, chrome)
            box(steer, f"SPod_{side}", sx, -4.6, 23.4, 3.0, 2.2, 3.2, dark_grey)
            tt_wheel(steer, f"SW_{side}", sx+m2*2.0, -4.2, 23.4, m2)
            bearing(steer, f"SPiv_{side}", sx, -3.6, 23.9, "z", 0.95, 0.50)
            mg90s(steer, f"SSrv_{side}", sx, -4.2, 23.9, "z")

        # ─────────────────────────────────────────────────────────────────
        # ⑧ SHIELDS / PANELS
        # ─────────────────────────────────────────────────────────────────
        shields = new_component("OP_Shields")
        for side, sx in [("L", -(SHOULDER_X+3.4)), ("R", SHOULDER_X+3.4)]:
            m2 = -1 if side == "L" else 1
            box(shields, f"ShShield_{side}", sx,        0, SHOULDER_CTR+1.5, 1.1, 4.6, 5.2, chrome)
            box(shields, f"ShHinge_{side}",  sx-m2*0.7, 0, SHOULDER_CTR+1.5, 0.5, 1.9, 1.9, dark_grey)
            box(shields, f"Mirror_{side}",   sx+m2*0.5,-2.9, SHOULDER_CTR+2.0, 1.5, 0.2, 0.9, dark_grey)
        for side2, hx in [("L", -(HIP_X+3.1)), ("R", HIP_X+3.1)]:
            box(shields, f"HipShield_{side2}", hx, 0, PELVIS_CTR+0.5, 1.1, 4.4, 4.0, op_blue)
SECTION 8: FDM SPLITTING WITH FASTENER MERGING (v11 Enhanced)
Python
        # ─────────────────────────────────────────────────────────────────
        # FDM SHELL SPLITTING (Y-plane @ 0) with v11 fastener merging
        # ─────────────────────────────────────────────────────────────────
        for comp in comps_list:
            to_split = [b for b in comp.bRepBodies
                        if b.name and any(k in b.name for k in SPLIT_KEYS)]
            for b in to_split:
                split_halves(comp, b, "y", 0.0)
                # v11: Merge fasteners into correct halves
                split_bodies = [sb for sb in comp.bRepBodies
                               if sb.name and b.name in sb.name and sb != b]
                if len(split_bodies) >= 2:
                    merge_fasteners_to_halves(comp, split_bodies[0], split_bodies[1])

        # ─────────────────────────────────────────────────────────────────
        # PRINTABILITY CHECKS (v11)
        # ─────────────────────────────────────────────────────────────────
        Logger.log("--- PRINTABILITY ANALYSIS ---")
        for comp in comps_list:
            for b in comp.bRepBodies:
                if not b.name or any(t in b.name for t in SKIP_TAGS):
                    continue
                # Check for common overhang-prone names
                overhang_prone = ["Spike", "Spur", "Fin", "Guard", "Crest", "Ant",
                                  "Flap", "EarTop", "Chin_Guard"]
                if any(op in b.name for op in overhang_prone):
                    printability_check(comp, b.name, overhang_faces=[(0, 0, -1)])
                else:
                    printability_check(comp, b.name, overhang_faces=[])
SECTION 9: KINEMATICS (Unchanged from v10)
Python
        # ─────────────────────────────────────────────────────────────────
        # KINEMATICS
        # ─────────────────────────────────────────────────────────────────
        t  = occs.get("OP_Torso")
        p  = occs.get("OP_Pelvis")
        h  = occs.get("OP_Head")
        b  = occs.get("OP_Backpack")
        st = occs.get("OP_SteerPods")
        sh = occs.get("OP_Shields")

        if p:
            p.isGrounded = True

        ball_joint("Waist_Cluster",  t,  p,  0, 0, WAIST_CTR-2.5)
        ball_joint("Neck_Cluster",   h,  t,  0, 0, NECK_JOINT_Z)
        rigid_joint("Backpack_Mount", b,  t)
        rigid_joint("Steer_Mount",   st,  p)
        rigid_joint("Shields_Mount", sh,  t)

        for side in ["L", "R"]:
            sx = -HIP_X      if side == "L" else  HIP_X
            ax = -SHOULDER_X if side == "L" else  SHOULDER_X
            m  = -1          if side == "L" else  1
            th = occs.get(f"OP_Thigh_{side}")
            sn = occs.get(f"OP_Shin_{side}")
            fo = occs.get(f"OP_Foot_{side}")
            ua = occs.get(f"OP_UpperArm_{side}")
            fa = occs.get(f"OP_Forearm_{side}")
            ha = occs.get(f"OP_Hand_{side}")

            ball_joint(f"{side}_Hip_Cluster",      th, p,  sx, 0, HIP_JOINT_Z)
            revolute_joint(f"{side}_Knee",         sn, th, sx, 0, KNEE_CTR+1.5, "x")
            ball_joint(f"{side}_Ankle_Cluster",    fo, sn, sx, 0, ANKLE_CTR+2.2)
            ball_joint(f"{side}_Shoulder_Cluster", ua, t,  ax, 0, SHOULDER_CTR)
            revolute_joint(f"{side}_Elbow",        fa, ua, ax, 0, ELBOW_Z,      "x")
            ball_joint(f"{side}_Wrist",            ha, fa, ax, 0, WRIST_Z+0.8)

            # GEO-2 + v11 Finger MCP joints
            MCP_Z = WRIST_Z - PALM_BOTTOM_OFFSET
            for fi, fname in enumerate(FING_NAMES):
                fx   = ax + m * FING_X_OFF[fi]
                fo_f = occs.get(f"OP_{fname}_{side}")
                revolute_joint(f"{side}_{fname}_MCP", fo_f, ha, fx, -1.35, MCP_Z, "x")

            # Thumb CMC (ball joint)
            tx     = ax + m * 1.70
            thumb_occ = occs.get(f"OP_Thumb_{side}")
            ball_joint(f"{side}_Thumb_CMC", thumb_occ, ha, tx, 0.20, MCP_Z)

            if side == "R":
                bl = occs.get("OP_Ion_Blaster")
                if bl:
                    revolute_joint("Blaster_Fold", bl, ha, ax, 0, WRIST_Z-1.0, "y")

        # ── Kinematic Validation ──────────────────────────────────────────
        Logger.log("Validating mechanical linkages...")
        try:
            orphans       = []
            jointed_comps = set()
            for i in range(root.asBuiltJoints.count):
                j = root.asBuiltJoints.item(i)
                if j.occurrenceOne:
                    jointed_comps.add(j.occurrenceOne.component.name)
                if j.occurrenceTwo:
                    jointed_comps.add(j.occurrenceTwo.component.name)
            for comp in comps_list:
                if comp.name not in ("OP_Torso", "OP_Pelvis") \
                   and comp.name not in jointed_comps:
                    orphans.append(comp.name)
            if orphans:
                Logger.log(f"  !!! ORPHANS: {orphans}", "WARN")
            else:
                Logger.log("  All components bound to kinematic chain. [OK]")
        except Exception:
            Logger.log("  Kinematic validation skipped (MCP env).", "WARN")

        try:
            cam = app.activeViewport.camera
            cam.isFitView = True
            app.activeViewport.camera = cam
        except Exception:
            pass
SECTION 10: SIMULATION ENGINE (v11 Enhanced)
Python
        # ═════════════════════════════════════════════════════════════════
        # SIMULATION ENGINE  v11.0
        # ═════════════════════════════════════════════════════════════════

        class SimulationEngine:
            """
            Simulation suite for Optimus Prime G1 v11 Physical Build.
            All transformation angles validated against JOINT_LIMITS.
            """

            BALL_JOINTS = {
                "Waist_Cluster", "Neck_Cluster",
                "L_Hip_Cluster", "R_Hip_Cluster",
                "L_Ankle_Cluster", "R_Ankle_Cluster",
                "L_Shoulder_Cluster", "R_Shoulder_Cluster",
                "L_Wrist", "R_Wrist",
                "L_Thumb_CMC", "R_Thumb_CMC",
            }
            REV_JOINTS = {
                "L_Knee", "R_Knee", "L_Elbow", "R_Elbow", "Blaster_Fold",
                "L_Pinky_MCP", "L_Ring_MCP", "L_Middle_MCP", "L_Index_MCP",
                "R_Pinky_MCP", "R_Ring_MCP", "R_Middle_MCP", "R_Index_MCP",
            }

            def __init__(self, root, comps_list, design, app, ui):
                self._root   = root
                self._comps  = comps_list
                self._design = design
                self._app    = app
                self._ui     = ui
                self._cols   = []
                self._logged_joints = False

            def _gj(self, name):
                try:
                    j = self._root.asBuiltJoints.itemByName(name)
                    if j is not None:
                        return j
                except Exception:
                    Logger.log(f"_gj exception '{name}': {traceback.format_exc()}", "ERROR")
                    return None
                try:
                    if not self._logged_joints:
                        self._logged_joints = True
                        names = [self._root.asBuiltJoints.item(i).name
                                 for i in range(self._root.asBuiltJoints.count)]
                        Logger.log(f"Joint '{name}' not found. Available: {names}", "WARN")
                except Exception:
                    pass
                return None

            @staticmethod
            def _ease(t):
                return t * t * (3.0 - 2.0 * t)

            def _get(self, mo, axis):
                try:
                    if mo.objectType == adsk.fusion.RevoluteJointMotion.classType():
                        return mo.rotationValue
                    if mo.objectType == adsk.fusion.BallJointMotion.classType():
                        return getattr(mo, axis + "Value")
                except Exception as e:
                    Logger.log(f"_get({axis}) failed: {e}", "ERROR")
                return 0.0

            def _set(self, mo, axis, val):
                try:
                    if mo.objectType == adsk.fusion.RevoluteJointMotion.classType():
                        mo.rotationValue = val
                    elif mo.objectType == adsk.fusion.BallJointMotion.classType():
                        setattr(mo, axis + "Value", val)
                except Exception as e:
                    Logger.log(f"_set({axis},{val:.2f}) failed: {e}", "ERROR")

            def _refresh(self):
                if os.path.exists(STOP_FLAG):
                    try:
                        os.remove(STOP_FLAG)
                    except Exception:
                        pass
                    raise Exception("SIMULATION_ABORTED_BY_USER")
                try:
                    self._app.activeViewport.refresh()
                except Exception:
                    pass
                try:
                    adsk.doEvents()
                except Exception:
                    pass

            def _clamp(self, joint_name, axis, deg):
                limits = JOINT_LIMITS.get(joint_name, {}).get(axis)
                if limits:
                    lo, hi = limits
                    if deg < lo or deg > hi:
                        Logger.log(
                            f"CLAMP: {joint_name}.{axis} {deg:.0f}° → [{lo}°,{hi}°]", "WARN")
                    return max(lo, min(hi, deg))
                return deg

            def move_joint(self, name, deg, steps=20, axis="pitch", ease=True, clamp=True):
                j = self._gj(name)
                if not j:
                    return
                if clamp:
                    deg = self._clamp(name, axis, deg)
                mo    = j.jointMotion
                e_rad = math.radians(deg)
                s_rad = self._get(mo, axis)
                for i in range(1, steps + 1):
                    t = self._ease(i/steps) if ease else i/steps
                    self._set(mo, axis, s_rad + (e_rad - s_rad) * t)
                    self._refresh()

            def move_group(self, targets, steps=20, axis="pitch", ease=True, clamp=True):
                active = []
                for item in targets:
                    name = item[0]; deg = item[1]
                    ax   = item[2] if len(item) > 2 else axis
                    j    = self._gj(name)
                    if not j:
                        continue
                    d  = self._clamp(name, ax, deg) if clamp else deg
                    mo = j.jointMotion
                    active.append((mo, ax, self._get(mo, ax), math.radians(d)))
                for i in range(1, steps + 1):
                    t = self._ease(i/steps) if ease else i/steps
                    for mo, ax, s_rad, e_rad in active:
                        self._set(mo, ax, s_rad + (e_rad - s_rad) * t)
                    self._refresh()

            def move_ball(self, targets, steps=20, clamp=True):
                active = []
                for name, pitch, yaw, roll in targets:
                    j = self._gj(name)
                    if not j:
                        continue
                    mo     = j.jointMotion
                    is_rev = (mo.objectType ==
                              adsk.fusion.RevoluteJointMotion.classType())
                    if is_rev:
                        if pitch is not None:
                            v = self._clamp(name, "pitch", pitch) if clamp else pitch
                            active.append((mo, "pitch",
                                           self._get(mo, "pitch"), math.radians(v)))
                    else:
                        for axis, val in [("pitch", pitch), ("yaw", yaw), ("roll", roll)]:
                            if val is None:
                                continue
                            v = self._clamp(name, axis, val) if clamp else val
                            active.append((mo, axis,
                                           self._get(mo, axis), math.radians(v)))
                for i in range(1, steps + 1):
                    t = self._ease(i / steps)
                    for mo, axis, s_rad, e_rad in active:
                        self._set(mo, axis, s_rad + (e_rad - s_rad) * t)
                    self._refresh()

            def reset_all(self, steps=10, groups=None):
                if groups is None:
                    ball_names = self.BALL_JOINTS
                    rev_names  = self.REV_JOINTS
                else:
                    ball_names = {n for n in groups if n in self.BALL_JOINTS}
                    rev_names  = {n for n in groups if n in self.REV_JOINTS}
                ball_targets = [(n, 0.0, 0.0, 0.0) for n in ball_names if self._gj(n)]
                rev_targets  = [n               for n in rev_names  if self._gj(n)]
                if ball_targets:
                    self.move_ball(ball_targets, steps=steps)
                for name in rev_targets:
                    self.move_joint(name, 0.0, steps, "pitch", ease=True)
                self._refresh()
                Logger.log("-> reset to neutral")

            def _interfere(self, label="Interference"):
                try:
                    all_bodies = adsk.core.ObjectCollection.create()
                    for comp in self._comps:
                        for body in list(comp.bRepBodies):
                            if body.name and not any(t in body.name for t in SKIP_TAGS):
                                all_bodies.add(body)
                    inter_input = self._design.createInterferenceInput(all_bodies)
                    inter_input.isCoincidentFacesInterference = False
                    results = self._design.analyzeInterference(inter_input)
                    count   = results.count
                    if count:
                        Logger.log(f"  {label}: {count} collision(s)")
                        for i in range(min(count, 5)):
                            r = results.item(i)
                            Logger.log(f"    [{r.entityOne.name}] <-> [{r.entityTwo.name}]")
                        if count > 5:
                            Logger.log(f"    ...{count-5} more")
                    else:
                        Logger.log(f"  {label}: [OK] 0 collisions")
                    self._cols.append((label, count))
                except Exception as e:
                    Logger.log(f"  {label}: skipped ({e})", "WARN")
                    self._cols.append((label, -1))

            # ── SIM-1: ZMP Stability ──────────────────────────────────────
            def _check_zmp(self, label):
                FOOT_HW = 6.2 / 2.0
                FOOT_HL = 9.2 / 2.0
                try:
                    com = self._root.physicalProperties.centerOfMass
                except Exception as e:
                    Logger.log(f"  ZMP [{label}]: CoM unavailable ({e})", "WARN")
                    return
                l_cx, r_cx = -HIP_X, HIP_X
                in_L  = (l_cx - FOOT_HW <= com.x <= l_cx + FOOT_HW and
                         -FOOT_HL <= com.y <= FOOT_HL)
                in_R  = (r_cx - FOOT_HW <= com.x <= r_cx + FOOT_HW and
                         -FOOT_HL <= com.y <= FOOT_HL)
                in_DS = (min(l_cx - FOOT_HW, r_cx - FOOT_HW) <= com.x <=
                         max(l_cx + FOOT_HW, r_cx + FOOT_HW) and
                         -FOOT_HL <= com.y <= FOOT_HL)
                stable = in_DS
                tag    = "[OK] ZMP STABLE" if stable else "[FAIL] ZMP UNSTABLE"
                Logger.log(
                    f"  ZMP [{label:16s}] {tag}  "
                    f"CoM=({com.x:+.2f}, {com.y:+.2f}, {com.z:.1f}) cm")
                return stable

            # ── SIM-2: Finger Gestures (v11 tendon-aware) ─────────────────
            def _set_fingers(self, side, degrees, thumb_deg=None, steps=12):
                targets = [(f"{side}_{n}_MCP", degrees, None, None)
                           for n in FING_NAMES]
                self.move_ball(targets, steps=steps)
                if thumb_deg is not None:
                    self.move_ball([(f"{side}_Thumb_CMC", thumb_deg, 0, 0)], steps=steps)

            def gesture_open(self, side="R", steps=10):
                self._set_fingers(side, 0, thumb_deg=0, steps=steps)
                Logger.log(f"  Gesture: {side} hand OPEN (tendons relaxed)")

            def gesture_fist(self, side="R", steps=12):
                self._set_fingers(side, 80, thumb_deg=50, steps=steps)
                Logger.log(f"  Gesture: {side} hand FIST (tendons tensioned)")

            def gesture_point(self, side="R", steps=10):
                targets = [
                    (f"{side}_Pinky_MCP",  80, None, None),
                    (f"{side}_Ring_MCP",   80, None, None),
                    (f"{side}_Middle_MCP", 80, None, None),
                    (f"{side}_Index_MCP",  -3, None, None),
                    (f"{side}_Thumb_CMC",  40, 0,    0),
                ]
                self.move_ball(targets, steps=steps)
                Logger.log(f"  Gesture: {side} hand POINT")

            def gesture_snap(self, side="R", steps=6):
                targets = [
                    (f"{side}_Pinky_MCP",  80, None, None),
                    (f"{side}_Ring_MCP",   70, None, None),
                    (f"{side}_Middle_MCP", -3, None, None),
                    (f"{side}_Index_MCP",  -3, None, None),
                    (f"{side}_Thumb_CMC",  55, 0,    0),
                ]
                self.move_ball(targets, steps=steps)
                Logger.log(f"  Gesture: {side} hand SNAP")

            # ── SIM-3: Arm Workspace Test ─────────────────────────────────
            def test_arm_workspace(self):
                Logger.log("--- SIM-3: ARM WORKSPACE TEST ---")
                poses = [
                    ( -30,   0,   0, "Forward low"),
                    ( -90,   0,  90, "Forward high"),
                    (-120,   0,  90, "Overhead"),
                    ( -80,  80,  60, "Side reach"),
                    ( -80, -80,  60, "Cross reach"),
                    ( -40,   0, 130, "Bicep curl"),
                    (-175,   0,  80, "Reach back"),
                    ( -80,   0, 150, "Elbow max"),
                ]
                for (sp, sy, ep, lbl) in poses:
                    for side in ["L", "R"]:
                        self.move_ball([
                            (f"{side}_Shoulder_Cluster", sp, sy, 0)], steps=10)
                        self.move_joint(f"{side}_Elbow", ep, steps=8, axis="pitch")
                    self._interfere(f"Workspace: {lbl}")
                    self.reset_all(steps=8,
                                   groups=["L_Shoulder_Cluster", "R_Shoulder_Cluster",
                                           "L_Elbow", "R_Elbow"])

            # ── Module 1: Joint ROM ───────────────────────────────────────
            def test_joint_rom(self):
                self.reset_all(steps=10)
                Logger.log("--- MODULE 1 / 9 ---")
                Logger.log("MODULE 1: JOINT ROM TEST")
                for name, axes in JOINT_LIMITS.items():
                    for axis, (lo, hi) in axes.items():
                        for lbl, angle in [("MIN", lo), ("MAX", hi)]:
                            self.move_joint(name, angle, steps=15, axis=axis)
                            self._interfere(f"{lbl} {name} {axis}")
                            self.move_joint(name, 0, steps=10, axis=axis)

            # ── Module 2: Head Look-Around ────────────────────────────────
            def simulate_head_scan(self):
                self.reset_all(steps=10)
                Logger.log("--- MODULE 2 / 9 ---")
                Logger.log("MODULE 2: HEAD LOOK-AROUND")
                for yaw_deg in [0, -50, 0, 50, 0]:
                    self.move_joint("Neck_Cluster", yaw_deg, steps=18, axis="yaw")
                for pitch_deg in [0, -25, 0, 35, 0]:
                    self.move_joint("Neck_Cluster", pitch_deg, steps=18, axis="pitch")

            # ── Module 3: Wave Gesture ────────────────────────────────────
            def simulate_wave(self):
                self.reset_all(steps=10)
                Logger.log("--- MODULE 3 / 9 ---")
                Logger.log("MODULE 3: WAVE GESTURE")
                self.gesture_open("R")
                self.move_joint("R_Shoulder_Cluster", -45, steps=15, axis="pitch")
                self.move_joint("R_Shoulder_Cluster",  80, steps=15, axis="yaw")
                self.move_joint("R_Elbow",             90, steps=12, axis="pitch")
                for _ in range(3):
                    self.move_ball([("R_Wrist", None, None, -30)], steps=8)
                    self.move_ball([("R_Wrist", None, None,  30)], steps=8)
                self.reset_all(steps=12)

            # ── Module 4: Idle Breathing ──────────────────────────────────
            def simulate_idle_breathing(self):
                self.reset_all(steps=8)
                Logger.log("--- MODULE 4 / 9 ---")
                Logger.log("MODULE 4: IDLE BREATHING")
                for _ in range(4):
                    self.move_joint("Waist_Cluster", -3, steps=12, axis="pitch")
                    self.move_joint("Waist_Cluster",  3, steps=12, axis="pitch")
                self.move_joint("Waist_Cluster", 0, steps=8, axis="pitch")

            # ── Module 5: Advanced Walking ────────────────────────────────
            def simulate_walking_advanced(self):
                self.reset_all(steps=10)
                Logger.log("--- MODULE 5 / 9 ---")
                Logger.log("MODULE 5: ADVANCED WALKING (ZMP checked)")
                for cycle in range(4):
                    phase  = cycle % 2
                    l_sign =  1 if phase == 0 else -1
                    r_sign = -1 if phase == 0 else  1
                    self.move_ball([
                        ("L_Hip_Cluster",      25*l_sign, 10*l_sign,  5*l_sign),
                        ("R_Hip_Cluster",      25*r_sign, 10*r_sign,  5*r_sign),
                        ("L_Shoulder_Cluster",  8*l_sign, 15*l_sign,  5*l_sign),
                        ("R_Shoulder_Cluster",  8*r_sign, 15*r_sign,  5*r_sign),
                        ("L_Knee",             60,        None,        None),
                        ("R_Knee",             60,        None,        None),
                        ("L_Ankle_Cluster",    15*l_sign, None,        8*l_sign),
                        ("R_Ankle_Cluster",    15*r_sign, None,        8*r_sign),
                    ], steps=20)
                    self._check_zmp(f"Walk cycle {cycle+1}")
                self.reset_all(steps=12)

            # ── Module 6: Running ─────────────────────────────────────────
            def simulate_running(self):
                self.reset_all(steps=8)
                Logger.log("--- MODULE 6 / 9 ---")
                Logger.log("MODULE 6: RUNNING")
                for cycle in range(3):
                    phase  = cycle % 2
                    l_sign =  1 if phase == 0 else -1
                    r_sign = -1 if phase == 0 else  1
                    self.move_ball([
                        ("L_Hip_Cluster",      30*l_sign, 20*l_sign, 10*l_sign),
                        ("R_Hip_Cluster",      30*r_sign, 20*r_sign, 10*r_sign),
                        ("L_Shoulder_Cluster", 15*l_sign, 25*l_sign, 10*l_sign),
                        ("R_Shoulder_Cluster", 15*r_sign, 25*r_sign, 10*r_sign),
                        ("L_Knee",             95,        None,       None),
                        ("R_Knee",             95,        None,       None),
                        ("L_Ankle_Cluster",    25*l_sign, None,       12*l_sign),
                        ("R_Ankle_Cluster",    25*r_sign, None,       12*r_sign),
                    ], steps=14)
                self.reset_all(steps=10)

            # ── Module 7: Combat Sequence ─────────────────────────────────
            def simulate_combat(self):
                self.reset_all(steps=10)
                Logger.log("--- MODULE 7 / 9 ---")
                Logger.log("MODULE 7: COMBAT SEQUENCE")
                self.gesture_fist("L")
                self.gesture_fist("R")
                self.move_joint("R_Shoulder_Cluster", -45, steps=10, axis="pitch")
                self.move_joint("R_Shoulder_Cluster",  45, steps=8,  axis="yaw")
                self.move_joint("R_Elbow",            120, steps=8,  axis="pitch")
                self.move_ball([("R_Wrist", None, None, 20)], steps=6)
                self.move_joint("R_Shoulder_Cluster", -10, steps=6,  axis="pitch")
                self.move_joint("R_Shoulder_Cluster", -80, steps=10, axis="pitch")
                self.move_joint("R_Shoulder_Cluster", -30, steps=6,  axis="yaw")
                self.move_joint("R_Elbow",             30, steps=8,  axis="pitch")
                self.move_joint("L_Shoulder_Cluster",  30, steps=8,  axis="pitch")
                self.move_joint("L_Elbow",             45, steps=6,  axis="pitch")
                self._check_zmp("Combat stance")
                self.reset_all(steps=12)

            # ── Transformation helpers ────────────────────────────────────
            def _transform_to_truck(self, steps_scale=1.0):
                s = steps_scale
                self.move_group([
                    ("R_Elbow",      0,   "pitch"),
                    ("L_Elbow",      0,   "pitch"),
                    ("Blaster_Fold", -90, "pitch"),
                ], steps=int(20*s))
                self.move_ball([
                    ("L_Wrist",  90, None,  90),
                    ("R_Wrist", 135, None,  90),
                ], steps=int(20*s))
                self.move_ball([("Neck_Cluster", -90, 0, 0)], steps=int(15*s))
                self.move_ball([
                    ("L_Shoulder_Cluster", -88, 0, 0),
                    ("R_Shoulder_Cluster", -88, 0, 0),
                ], steps=int(22*s))
                self.move_ball([
                    ("L_Hip_Cluster", 0, 90, 0),
                    ("R_Hip_Cluster", 0, 90, 0),
                ], steps=int(22*s))
                self.move_ball([
                    ("L_Ankle_Cluster", 0, 90, 0),
                    ("R_Ankle_Cluster", 0, 90, 0),
                ], steps=int(18*s))

            def _transform_to_robot(self, steps_scale=1.0):
                s = steps_scale
                self.move_ball([("L_Ankle_Cluster", 0, 0, 0), ("R_Ankle_Cluster", 0, 0, 0)],
                               steps=int(18*s))
                self.move_ball([("L_Hip_Cluster", 0, 0, 0), ("R_Hip_Cluster", 0, 0, 0)],
                               steps=int(22*s))
                self.move_ball([("L_Shoulder_Cluster", 0, 0, 0), ("R_Shoulder_Cluster", 0, 0, 0)],
                               steps=int(22*s))
                self.move_ball([("Neck_Cluster", 0, 0, 0)], steps=int(15*s))
                self.move_ball([("L_Wrist", 0, None, 0), ("R_Wrist", 0, None, 0)],
                               steps=int(18*s))
                self.move_group([("Blaster_Fold", 0, "pitch"), ("R_Elbow", 0, "pitch"),
                                 ("L_Elbow", 0, "pitch")], steps=int(18*s))

            # ── Module 8: Transformation ──────────────────────────────────
            def simulate_transformation(self):
                self.reset_all(steps=10)
                Logger.log("--- MODULE 8a / 9 ---")
                Logger.log("MODULE 8a: TRANSFORMATION  Robot → Truck")
                Logger.log("  [PHYSICAL NOTE: Engage all locking pins before driving servos]")
                self._transform_to_truck(steps_scale=1.0)
                self._interfere("Truck-mode check")
                Logger.log("MODULE 8c: TRUCK → ROBOT")
                Logger.log("  [PHYSICAL NOTE: Release locking pins before transformation]")
                self._transform_to_robot(steps_scale=1.0)
                Logger.log("Robot mode restored.")
                self.reset_all(steps=10)

            # ── Module 9a: Stability Analysis ──────────────────────────────
            def run_stability_analysis(self):
                Logger.log("--- MODULE 9 / 9 ---")
                Logger.log("MODULE 9a: STABILITY ANALYSIS (ZMP)")
                poses = {
                    "Attention": {"Waist_Cluster": (0, 0, 0)},
                    "Combat": {
                        "Waist_Cluster":      (10, 0,   0),
                        "L_Knee":              45,
                        "R_Knee":              45,
                        "L_Elbow":             90,
                        "R_Elbow":             90,
                        "R_Shoulder_Cluster": (0,  30, -45),
                    },
                    "Squat": {
                        "Waist_Cluster":  (20, 0,   0),
                        "L_Knee":          90,
                        "R_Knee":          90,
                        "L_Hip_Cluster":  (0, -45, 0),
                        "R_Hip_Cluster":  (0, -45, 0),
                    },
                    "Victory": {
                        "L_Shoulder_Cluster": (0, 60, -90),
                        "R_Shoulder_Cluster": (0, 60, -90),
                        "L_Elbow":             30,
                        "R_Elbow":             30,
                        "Waist_Cluster":      (15, 0, 0),
                    },
                    "Single_Leg_L": {
                        "L_Hip_Cluster":  (0, 90,  0),
                        "L_Knee":          90,
                        "Waist_Cluster":  (5, 10, -5),
                    },
                }
                for pose_name, config in poses.items():
                    self.reset_all(steps=10)
                    for key, val in config.items():
                        if isinstance(val, tuple):
                            self.move_ball([(key, val[0], val[1], val[2])], steps=15)
                        else:
                            self.move_joint(key, val, steps=12, axis="pitch")
                    self._check_zmp(pose_name)

            # ── Module 9b: Servo Load Estimation ─────────────────────────
            def estimate_servo_loads(self):
                Logger.log("MODULE 9b: SERVO LOAD ESTIMATION (v11 upgraded servos)")
                loads = [
                    ("Neck Pitch",       120,  3.0, SERVO_SPECS["micro"]),
                    ("L Shoulder Pitch", 390, 12.0, SERVO_SPECS["std"]),
                    ("R Shoulder Pitch", 390, 12.0, SERVO_SPECS["std"]),
                    ("L Elbow",          210,  7.0, SERVO_SPECS["std"]),
                    ("R Elbow",          210,  7.0, SERVO_SPECS["std"]),
                    ("L Hip Pitch",      820, 15.0, SERVO_SPECS["hip_hd"]),
                    ("R Hip Pitch",      820, 15.0, SERVO_SPECS["hip_hd"]),
                    ("L Knee Pitch",     540,  9.0, SERVO_SPECS["std"]),
                    ("R Knee Pitch",     540,  9.0, SERVO_SPECS["std"]),
                    ("Waist Pitch",     2100,  8.0, SERVO_SPECS["waist"]),
                    ("L Ankle Pitch",    280,  4.5, SERVO_SPECS["std"]),
                    ("R Ankle Pitch",    280,  4.5, SERVO_SPECS["std"]),
                    ("Finger Drive",      18,  2.0, SERVO_SPECS["digit"]),
                ]
                for label, mass_g, lever_cm, spec in loads:
                    F      = (mass_g / 1000.0) * 9.81
                    need   = (F * lever_cm / 100.0) / 0.0981
                    margin = spec["rated"] / need if need > 0 else 99.0
                    status = ("OK"       if margin >= 1.5 else
                              "MARGINAL" if margin >= 0.9 else "OVERLOAD")
                    icon   = "[OK]" if status == "OK" else f"[{status}]"
                    Logger.log(
                        f"  {label:<24} need {need:5.2f} kg·cm  "
                        f"rated {spec['rated']:5.1f}  margin {margin:.2f}×  "
                        f"{spec['name']:12s}  {icon}")

            # ── Simulation mode helpers ───────────────────────────────────
            def simulate_robot_mode(self):
                self.reset_all(steps=10)
                self.move_joint("Blaster_Fold", 0, steps=10, axis="pitch")
                self.gesture_open("L")
                self.gesture_open("R")
                Logger.log("--- ROBOT MODE --- holding neutral pose")
                Logger.log("  [PHYSICAL: All locking pins ENGAGED, covers installed]")
                try:
                    cam = self._app.activeViewport.camera
                    cam.isFitView = True
                    self._app.activeViewport.camera = cam
                    self._app.activeViewport.refresh()
                except Exception:
                    pass
                self.capture_screenshots("optimus_robot_v11")

            def simulate_truck_mode(self):
                self.reset_all(steps=10)
                Logger.log("--- TRUCK MODE ---")
                Logger.log("  [PHYSICAL: All locking pins ENGAGED for truck mode]")
                self.gesture_fist("L")
                self.gesture_fist("R")
                self._transform_to_truck(steps_scale=1.0)
                self._interfere("Truck-mode check")
                Logger.log("TRUCK MODE -- holding position")
                try:
                    cam = self._app.activeViewport.camera
                    cam.isFitView = True
                    self._app.activeViewport.camera = cam
                    self._app.activeViewport.refresh()
                except Exception:
                    pass
                self.capture_screenshots("optimus_truck_v11")

            def simulate_battle_mode(self):
                self.reset_all(steps=10)
                Logger.log("--- BATTLE MODE ---")
                Logger.log("  [PHYSICAL: Blaster locked, all covers secured]")
                self.move_joint("Blaster_Fold", 0,   steps=10, axis="pitch")
                self.gesture_fist("L")
                self.gesture_point("R")
                self.move_ball([("L_Wrist", None, None, 90),
                                ("R_Wrist", None, None, 90)], steps=15)
                self.move_joint("R_Elbow", 130, steps=18, axis="pitch", clamp=True)
                self.move_joint("L_Elbow", 130, steps=18, axis="pitch", clamp=True)
                self.move_ball([("L_Shoulder_Cluster", 0, -88, 0),
                                ("R_Shoulder_Cluster", 0, -88, 0)], steps=22)
                self._check_zmp("Battle mode")
                self._interfere("Battle-mode check")
                Logger.log("BATTLE MODE -- holding position")
                self.capture_screenshots("optimus_battle_v11")

            # ── Debug helpers ─────────────────────────────────────────────
            def debug_joints(self, label):
                Logger.log(f"=== JOINT STATE [{label}] ===")
                try:
                    for i in range(self._root.asBuiltJoints.count):
                        j  = self._root.asBuiltJoints.item(i)
                        mo = j.jointMotion
                        if mo.objectType == adsk.fusion.RevoluteJointMotion.classType():
                            Logger.log(
                                f"  {j.name:34s} REV  "
                                f"pitch={math.degrees(mo.rotationValue):+.1f}°")
                        elif mo.objectType == adsk.fusion.BallJointMotion.classType():
                            try:
                                Logger.log(
                                    f"  {j.name:34s} BALL "
                                    f"p={math.degrees(mo.pitchValue):+.1f}° "
                                    f"y={math.degrees(mo.yawValue):+.1f}° "
                                    f"r={math.degrees(mo.rollValue):+.1f}°")
                            except Exception as e:
                                Logger.log(f"  {j.name:34s} BALL (readback: {e})", "WARN")
                except Exception:
                    Logger.log("  (joint debug unavailable in this environment)", "WARN")

            # ── Screenshot Capture ────────────────────────────────────────
            def capture_screenshots(self, prefix="optimus"):
                if not CAPTURE_SCREENSHOTS:
                    return
                try:
                    os.makedirs(SCREENSHOT_DIR, exist_ok=True)
                    viewport = self._app.activeViewport
                    camera   = viewport.camera
                    views = [
                        ("Front", adsk.core.ViewOrientations.FrontViewOrientation),
                        ("Back",  adsk.core.ViewOrientations.BackViewOrientation),
                        ("Left",  adsk.core.ViewOrientations.LeftViewOrientation),
                        ("Right", adsk.core.ViewOrientations.RightViewOrientation),
                        ("Top",   adsk.core.ViewOrientations.TopViewOrientation),
                        ("Iso",   adsk.core.ViewOrientations.IsoTopRightViewOrientation),
                    ]
                    for name, orientation in views:
                        camera.viewOrientation = orientation
                        camera.isFitView       = True
                        viewport.camera        = camera
                        time.sleep(0.5)
                        path = os.path.join(SCREENSHOT_DIR, f"{prefix}_{name}.png")
                        viewport.saveAsImageFile(path, 1920, 1080)
                        Logger.log(f"Screenshot: {path}")
                except Exception:
                    Logger.log(f"Screenshot failed: {traceback.format_exc()}", "WARN")

            # ── URDF Export (v11 — proper joint types + inertia) ──────────
            def export_urdf(self):
                def _urdf_type(name):
                    if "Cluster" in name or "Wrist" in name or "CMC" in name:
                        return "spherical"
                    if any(k in name for k in ["Knee", "Elbow", "MCP", "Fold"]):
                        return "revolute"
                    if any(k in name for k in ["Mount", "Steer", "Shields", "Backpack"]):
                        return "fixed"
                    return "revolute"

                def _limits(name):
                    limits_d = JOINT_LIMITS.get(name, {})
                    pitch = limits_d.get("pitch", (-180, 180))
                    lo_r  = math.radians(pitch[0])
                    hi_r  = math.radians(pitch[1])
                    return lo_r, hi_r

                link_mass = {
                    "OP_Head": 250,       "OP_Torso": 800,    "OP_Pelvis": 400,
                    "OP_Backpack": 150,   "OP_SteerPods": 120, "OP_Shields": 80,
                    "OP_Thigh_L": 250,    "OP_Thigh_R": 250,
                    "OP_Shin_L": 220,     "OP_Shin_R": 220,
                    "OP_Foot_L": 180,     "OP_Foot_R": 180,
                    "OP_UpperArm_L": 160, "OP_UpperArm_R": 160,
                    "OP_Forearm_L": 120,  "OP_Forearm_R": 120,
                    "OP_Hand_L": 60,      "OP_Hand_R": 60,
                    "OP_Ion_Blaster": 40,
                }
                def _inertia(cname):
                    m_kg = link_mass.get(cname, 80) / 1000.0
                    lx, ly, lz = 0.10, 0.08, 0.06
                    ixx = m_kg * (ly**2 + lz**2) / 12.0
                    iyy = m_kg * (lx**2 + lz**2) / 12.0
                    izz = m_kg * (lx**2 + ly**2) / 12.0
                    return m_kg, ixx, iyy, izz

                try:
                    os.makedirs(EXPORT_DIR, exist_ok=True)
                    path = os.path.join(EXPORT_DIR, "robot_v11.urdf")
                    jc   = self._root.asBuiltJoints.count
                    with open(path, "w", encoding="utf-8") as f:
                        f.write('<?xml version="1.0" encoding="utf-8"?>\\n')
                        f.write('<robot name="Optimus_Prime_G1_v11">\\n\\n')
                        for comp in self._comps:
                            m_kg, ixx, iyy, izz = _inertia(comp.name)
                            f.write(f'  <link name="{comp.name}">\\n')
                            f.write( '    <inertial>\\n')
                            f.write( '      <origin xyz="0 0 0" rpy="0 0 0"/>\\n')
                            f.write(f'      <mass value="{m_kg:.4f}"/>\\n')
                            f.write(f'      <inertia ixx="{ixx:.6f}" ixy="0" ixz="0" '
                                    f'iyy="{iyy:.6f}" iyz="0" izz="{izz:.6f}"/>\\n')
                            f.write( '    </inertial>\\n')
                            f.write( '  </link>\\n')
                        f.write('\\n')
                        for i in range(jc):
                            j    = self._root.asBuiltJoints.item(i)
                            n    = j.name.replace(" ", "_")
                            o1   = j.occurrenceOne.component.name if j.occurrenceOne else "world"
                            o2   = j.occurrenceTwo.component.name if j.occurrenceTwo else "world"
                            jtyp = _urdf_type(n)
                            lo_r, hi_r = _limits(n)
                            effort = 25.0
                            if "Hip" in n:     effort = 35.0
                            if "Waist" in n:   effort = 25.0
                            if "MCP" in n:     effort = 2.2
                            f.write(f'  <joint name="{n}" type="{jtyp}">\\n')
                            f.write(f'    <parent link="{o1}"/>\\n')
                            f.write(f'    <child link="{o2}"/>\\n')
                            f.write( '    <origin xyz="0 0 0" rpy="0 0 0"/>\\n')
                            f.write( '    <axis xyz="1 0 0"/>\\n')
                            if jtyp == "revolute":
                                f.write(
                                    f'    <limit lower="{lo_r:.4f}" upper="{hi_r:.4f}" '
                                    f'effort="{effort:.1f}" velocity="3.14"/>\\n')
                            f.write( '  </joint>\\n')
                        f.write('</robot>\\n')
                    Logger.log(f"URDF v11 exported -> {path}")
                except Exception:
                    Logger.log(f"URDF export failed: {traceback.format_exc()}", "ERROR")

            # ── STL Export ────────────────────────────────────────────────
            def export_stl(self):
                try:
                    os.makedirs(EXPORT_DIR, exist_ok=True)
                    skip_s     = {"Marker", "Pivot", "MtA", "MtB",
                                  "Axle_Pivot", "Horn", "_Vis", "Scope",
                                  "Antenna", "Boss", "Insert", "Nut", "Snap",
                                  "Coupler", "Tendon", "Spring", "Drum", "Guide",
                                  "Clip", "Grommet", "Jig", "Rib", "Lip"}
                    export_mgr = self._design.exportManager
                    count      = 0
                    for comp in self._comps:
                        for body in list(comp.bRepBodies):
                            if not body.name or any(s in body.name for s in skip_s):
                                continue
                            try:
                                path     = os.path.join(EXPORT_DIR,
                                                        f"{comp.name}__{body.name}.stl")
                                stl_opts = export_mgr.createSTLExportOptions(body, path)
                                stl_opts.meshRefinement = (
                                    adsk.fusion.MeshRefinementSettings.MeshRefinementHigh)
                                export_mgr.execute(stl_opts)
                                count += 1
                            except Exception:
                                Logger.log(f"STL fail: {comp.name}/{body.name}", "WARN")
                    Logger.log(f"STL exported {count} bodies → {EXPORT_DIR}")
                except Exception:
                    Logger.log(f"STL export failed: {traceback.format_exc()}", "ERROR")

            # ── STEP Export ───────────────────────────────────────────────
            def export_step(self):
                try:
                    os.makedirs(EXPORT_DIR, exist_ok=True)
                    export_mgr = self._design.exportManager
                    path       = os.path.join(EXPORT_DIR, "Optimus_Prime_G1_v11.step")
                    step_opts  = export_mgr.createSTEPExportOptions(path)
                    export_mgr.execute(step_opts)
                    Logger.log(f"STEP assembly → {path}")
                    count = 0
                    for i in range(self._root.allOccurrences.count):
                        occ   = self._root.allOccurrences.item(i)
                        cname = occ.component.name
                        if any(t in cname for t in {"Marker", "Pivot", "_Vis"}):
                            continue
                        try:
                            cpath = os.path.join(EXPORT_DIR, f"{cname}.step")
                            copts = export_mgr.createSTEPExportOptions(occ, cpath)
                            export_mgr.execute(copts)
                            count += 1
                        except Exception:
                            Logger.log(f"STEP fail: {cname}", "WARN")
                    Logger.log(f"STEP {count} components → {EXPORT_DIR}")
                except Exception:
                    Logger.log(f"STEP export failed: {traceback.format_exc()}", "ERROR")

            # ── SIM-4: BOM Export (v11 enhanced) ─────────────────────────
            def export_bom(self):
                """SIM-4 — Finalise and write BOM CSV + printed parts manifest."""
                # Fixed hardware totals
                BOM.add("Fastener", "M3×8 SHCS (general assembly)", 80,  "assembly")
                BOM.add("Fastener", "M3×16 SHCS (joint brackets)",  24,  "assembly")
                BOM.add("Fastener", "M3×6 set-screw (grub screw, horn couplers)", 20, "assembly")
                BOM.add("Fastener", "M2.5×6 SHCS (PCB mounting)",    16, "assembly")
                BOM.add("Hardware", "Ø3mm × 30 mm shaft pin",        12,  "joints")
                BOM.add("Hardware", "Ø3mm × 20 mm shaft pin",        20,  "joints")
                BOM.add("Hardware", "Braided fishing line Ø0.5 mm (tendons)", 5, "meters")
                BOM.add("Hardware", "Return spring Ø4×6 mm",          10, "finger extension")
                BOM.add("Material", "PLA filament 1kg spool",          4, "~900g total")
                BOM.add("Material", "PETG filament 1kg spool",        2, "~400g couplers/drums")
                BOM.add("Material", "TPU filament 250g spool",         1, "flex parts")
                BOM.add("Electronics", "22AWG servo wire (3m lengths)", 30, "wiring harness")
                BOM.add("Electronics", "JST-XH 3-pin connectors",      40, "servo connectors")
                BOM.add("Electronics", "5V 5A BEC / power regulator",   2, "servo power")
                BOM.add("Electronics", "USB-C power cable",              1, "RPi power")
                BOM.add("Electronics", "Blade fuse 10A",                 2, "spare")
                BOM.add("Electronics", "Emergency cutoff switch",        1, "panel mount")
                BOM.add("Hardware", "Velcro strap 20×200 mm",          4, "battery retention")
                BOM.add("Hardware", "Cable tie 3×100 mm",              20, "strain relief")
                BOM.save_csv(BOM_FILE)
                BOM.summary()

                # Printed parts manifest
                manifest_path = os.path.join(_OUTPUT_DIR, f"PRINTED_PARTS_v11_{_ts}.csv")
                BOM.save_printed_manifest(manifest_path)

                # Wiring map
                Logger.log("── SERVO WIRING MAP (PCA9685 channels) ──────────────")
                wiring = [
                    ( 0, "L_Hip_Yaw",     "Pelvis → L_HipYaw"),
                    ( 1, "R_Hip_Yaw",     "Pelvis → R_HipYaw"),
                    ( 2, "L_Hip_Pitch",   "Thigh_L → HipP"),
                    ( 3, "R_Hip_Pitch",   "Thigh_R → HipP"),
                    ( 4, "L_Hip_Roll",    "Thigh_L → HipR"),
                    ( 5, "R_Hip_Roll",    "Thigh_R → HipR"),
                    ( 6, "L_Knee",        "Thigh_L → KneP"),
                    ( 7, "R_Knee",        "Thigh_R → KneP"),
                    ( 8, "L_Ankle_Pitch", "Foot_L  → AnkP"),
                    ( 9, "R_Ankle_Pitch", "Foot_R  → AnkP"),
                    (10, "L_Ankle_Roll",  "Foot_L  → AnkR"),
                    (11, "R_Ankle_Roll",  "Foot_R  → AnkR"),
                    (12, "Waist_Yaw",     "Torso   → WaistYaw"),
                    (13, "Waist_Pitch",   "Torso   → WaistPitch"),
                    (14, "Neck_Pitch",    "Torso   → NeckPitch"),
                    (15, "Neck_Yaw",      "Head    → NeckYaw"),
                ]
                wiring2 = [
                    ( 0, "L_Sh_Yaw",      "UpperArm_L → ShY"),
                    ( 1, "R_Sh_Yaw",      "UpperArm_R → ShY"),
                    ( 2, "L_Sh_Pitch",    "UpperArm_L → ShP"),
                    ( 3, "R_Sh_Pitch",    "UpperArm_R → ShP"),
                    ( 4, "L_Sh_Roll",     "UpperArm_L → ShR"),
                    ( 5, "R_Sh_Roll",     "UpperArm_R → ShR"),
                    ( 6, "L_Elbow",       "UpperArm_L → ElbP"),
                    ( 7, "R_Elbow",       "UpperArm_R → ElbP"),
                    ( 8, "L_Wrist_Roll",  "Forearm_L  → WR"),
                    ( 9, "R_Wrist_Roll",  "Forearm_R  → WR"),
                    (10, "L_Finger_All",  "Hand_L → FingerSrvBay ch0"),
                    (11, "L_Thumb",       "Hand_L → FingerSrvBay ch1"),
                    (12, "R_Finger_All",  "Hand_R → FingerSrvBay ch0"),
                    (13, "R_Thumb",       "Hand_R → FingerSrvBay ch1"),
                    (14, "Blaster_Fold",  "Hand_R → Blaster hinge"),
                    (15, "SPARE",         "–"),
                ]
                for ch, name, loc in wiring:
                    Logger.log(f"  PCA1 ch{ch:02d}  {name:<20s}  ← {loc}")
                for ch, name, loc in wiring2:
                    Logger.log(f"  PCA2 ch{ch:02d}  {name:<20s}  ← {loc}")

            # ── v11: Build Documentation Generator ────────────────────────
            def generate_build_docs(self):
                """PHY-20 — Auto-generate assembly guide with orientations."""
                try:
                    with open(BUILD_FILE, "w", encoding="utf-8") as f:
                        f.write("# Optimus Prime G1 v11.0 — Physical Build Guide\\n")
                        f.write(f"Generated: {datetime.datetime.now().isoformat()}\\n\\n")
                        
                        f.write("## CRITICAL BUILD WARNINGS\\n\\n")
                        for w in Logger.get_warnings():
                            f.write(f"- ⚠️ {w}\\n")
                        f.write("\\n")
                        
                        f.write("## ASSEMBLY ORDER\\n\\n")
                        for step in ASSEMBLY_ORDER:
                            f.write(f"{step}\\n")
                        f.write("\\n")
                        
                        f.write("## PRINT ORIENTATION RECOMMENDATIONS\\n\\n")
                        f.write("| Part | Material | Orientation | Supports | Infill | Notes |\\n")
                        f.write("|------|----------|-------------|----------|--------|-------|\\n")
                        f.write("| Torso shells | PLA | Split Y-plane, print halves flat | Minimal | 30% | 3 walls |\\n")
                        f.write("| Horn couplers | PETG | Hub up, vertical | None | 100% | Critical strength |\\n")
                        f.write("| Servo drums | PETG | Drum axis vertical | None | 100% | Tendon grooves |\\n")
                        f.write("| Finger phalanges | PLA | Flat, hinge axis up | None | 40% | 4 walls |\\n")
                        f.write("| Tendon guides | PLA | Vertical post | None | 50% | Snap-fit test |\\n")
                        f.write("| Cable clips | PLA | Flat | None | 30% | Flex test |\\n")
                        f.write("| Access covers | PLA | Flat, aesthetic face up | None | 20% | Best finish |\\n")
                        f.write("| Assembly jigs | PLA | Flat | None | 20% | Orange for visibility |\\n")
                        f.write("| Structural ribs | PLA | Long edge down | Minimal | 50% | Internal parts |\\n")
                        f.write("| Bearing retainers | PETG | Lip facing up | None | 60% | Press-fit tolerance |\\n")
                        f.write("| Heel spurs | PETG | Angled 45° | Supports | 50% | Chamfered for print |\\n")
                        f.write("| Shoulder spikes | PETG | Tip up | Supports | 40% | Chamfered base |\\n")
                        f.write("\\n")
                        
                        f.write("## MATERIAL RECOMMENDATIONS\\n\\n")
                        f.write("- **PLA**: Torso, head, cosmetic shells, jigs, covers\\n")
                        f.write("- **PETG**: Functional parts, couplers, drums, bearing housings, high-stress joints\\n")
                        f.write("- **TPU**: Flexible grommets, wire strain relief boots (optional)\\n")
                        f.write("- **Carbon fiber reinforced**: Horn couplers, high-load brackets (if available)\\n\\n")
                        
                        f.write("## SERVO INSTALLATION ORDER\\n\\n")
                        f.write("1. Install all servos into cavities BEFORE closing shells\\n")
                        f.write("2. Route all servo wires through channels and grommets\\n")
                        f.write("3. Install horn couplers on servo outputs\\n")
                        f.write("4. Test each servo individually before assembly\\n")
                        f.write("5. Set all servos to neutral (0°) before attaching links\\n\\n")
                        
                        f.write("## ELECTRONICS INSTALLATION\\n\\n")
                        f.write("1. Install heat-set inserts first (use soldering iron at 200°C)\\n")
                        f.write("2. Mount PCA9685 boards with M2.5 standoffs\\n")
                        f.write("3. Install Raspberry Pi Zero 2W with SD card pre-loaded\\n")
                        f.write("4. Route I2C wires between PCA9685 and RPi\\n")
                        f.write("5. Install IMU in pelvis (orientation matters for firmware)\\n")
                        f.write("6. Install ESP32-CAM in head, route lens channel\\n")
                        f.write("7. Install battery with Velcro straps, NOT glued\\n")
                        f.write("8. Install fuse holder and emergency cutoff switch\\n")
                        f.write("9. Connect BEC regulator for servo power (separate from logic)\\n")
                        f.write("10. Test all electronics before final shell closure\\n\\n")
                        
                        f.write("## TENDON INSTALLATION\\n\\n")
                        f.write("1. Cut braided fishing line to length (measure twice)\\n")
                        f.write("2. Tie knot in distal phalanx anchor hole\\n")
                        f.write("3. Route through guide posts on each phalanx\\n")
                        f.write("4. Wind 2-3 turns on servo drum\\n")
                        f.write("5. Tie off at drum anchor hole\\n")
                        f.write("6. Install return springs on proximal phalanges\\n")
                        f.write("7. Test range of motion before closing palm\\n\\n")
                        
                        f.write("## TRANSFORMATION PROCEDURE\\n\\n")
                        f.write("### Robot → Truck\\n")
                        f.write("1. Power OFF all servos\\n")
                        f.write("2. Release all locking pins (pull outward)\\n")
                        f.write("3. Fold arms along torso sides\\n")
                        f.write("4. Rotate legs 90° at hip yaw\\n")
                        f.write("5. Fold knees and ankles per simulation\\n")
                        f.write("6. Engage truck-mode locking pins\\n")
                        f.write("7. Verify no wire pinching\\n\\n")
                        
                        f.write("### Truck → Robot\\n")
                        f.write("1. Power OFF all servos\\n")
                        f.write("2. Release truck-mode locking pins\\n")
                        f.write("3. Extend legs to standing position\\n")
                        f.write("4. Rotate hip yaw back to 0°\\n")
                        f.write("5. Extend arms to robot position\\n")
                        f.write("6. Engage robot-mode locking pins\\n")
                        f.write("7. Power ON and run neutral pose check\\n\\n")
                        
                        f.write("## MAINTENANCE NOTES\\n\\n")
                        f.write("- Check tendon tension monthly\\n")
                        f.write("- Inspect bearing retention lips for wear\\n")
                        f.write("- Verify locking pin detents engage firmly\\n")
                        f.write("- Replace Velcro straps if frayed\\n")
                        f.write("- Keep spare M3 set-screws for horn couplers\\n\\n")
                        
                        f.write("## BUILD NOTES FROM THIS SESSION\\n\\n")
                        for note in Logger.get_build_notes():
                            f.write(f"- {note}\\n")
                    
                    Logger.log(f"Build guide generated → {BUILD_FILE}")
                except Exception as e:
                    Logger.log(f"Build docs failed: {e}", "WARN")

            # ── Master Runner ─────────────────────────────────