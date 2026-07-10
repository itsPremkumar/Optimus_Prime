# Changelog

## [17.0.0] - 2026-07-09

### Added
- **Joint design-risk toggles** — `KNEE_ROLL_ENABLED` / `ANKLE_YAW_ENABLED` let you drop the two highest-risk DOFs (parasitic knee-roll torsion; ankle yaw) to rigid joints with a single flag.
- **Fastener/bushing upgrades** — circlip retaining grooves, axial thrust washers, hex metal shaft-coupling inserts, brass/bronze bushing pockets, tendon tensioner slots.
- **CSI ribbon slack cradle** (fatigue relief for the most failure-prone connection) and real shock-absorbing foot compliance (replacing the too-thin cosmetic vibration pad).

### Fixed
- **Waist_Roll** and **Neck_Roll** had a servo + bearing but no hard stop — added mechanical limits.
- Joint-by-joint reinforcement pass: axial thrust washers on weight-bearing joints.

### Changed
- Version bump across exports: `Optimus_Prime_G1_v17.f3d/.step`, `robot_v17.urdf`, `BOM_v17_*.csv`, `ASSEMBLY_GUIDE_v17_*.txt`.

## [16.0.0] - 2026-07-07

### Added
- **Printer calibration profiles** (`generic_fdm` / `calibrated_fdm` / `sla_resin`) replacing a single hard-coded clearance.
- **Actuator profiles** (hobby servo / smart serial / linear-hybrid) with real torque-margin math per class.
- **Real physical-material mass** computation feeding real per-joint torque validation (worst-case, not average).
- **Analytic 2-link planar leg IK** for a real gait; **ESP32 firmware skeleton export**; cantilever-bending bracket safety-factor checks.

## [15.0.0] - 2026-07-06

### Changed
- Refined v14 electronics/geometry; knee-roll axis rework and joint-limit sync.

## [14.0.0] - 2026-07-05

### Added
- AI/electronics integration: NVIDIA Jetson Nano carrier, ESP32-S3 control nodes, CSI camera (IMX219), VL53L1X ToF, INA3221 power monitor, sensor-fusion array (ultrasonic + FSR + ADC).
- Independent servo-rail E-stop, WS2812 status RGB badge, I2C/UART/SPI comm backbone down the spine.

### Fixed
- **AXIS-1**: knee DOF was keyed `yaw` but the physical servo sits on the Y (roll) axis — re-keyed to `roll` to match hardware, with an automated axis-mapping check.

## [13.0.0] - 2026-06-28

### Added
- Edge-compute / AI-vision prototype line (Jetson + ESP32-CAM), assembly-guide generator, communication/power map registries.

## [10.0.0 – 12.0.0] - 2026-06

### Added
- Incremental physical-build hardening: finger tendon drives, cable management, power system (BEC/fuse), fasteners (M3 inserts/nuts/bosses), print jigs, BOM/CAD validators. (See `old_code/` for these reference versions.)

## [9.0.0] - 2026-06-17

### Fixed
9 bugs corrected from v8.0:

- **FIX 1** `_make_joint_geometry()` — SketchPoints on XY plane discard Z coordinate, placing joint origins at z=0. Replaced with `ConstructionPoint.createInput` / `setByPoint` to preserve all three axes.

- **FIX 2** `bearing()` — off-centre cavity. Cut-cylinder extended entirely to one side of bearing centre. Now symmetric: `p1 = centre - half`, `p2 = centre + half` with half = w/2 + 0.05 cm clearance.

- **FIX 3** `cut_cavity()` — mutable-collection iteration over `comp.bRepBodies` while `combineFeatures.add()` modifies the collection. Now snapshots with `list(comp.bRepBodies)` first.

- **FIX 4** `reset_all()` — `groups` parameter silently ignored; every joint was reset regardless of caller intent. Now filters `BALL_JOINTS` / `REV_JOINTS` by the supplied groups list.

- **FIX 5** `move_ball()` — revolute joints in mixed target lists. Walking/running animations passed `L_Knee`/`R_Knee` (revolute) to `move_ball()`. Now detects `RevoluteJointMotion` via `objectType` and routes pitch to `rotationValue`; yaw/roll skipped for revolute joints.

- **FIX 6** `export_stl()` — `BRepBody.exportSTL()` does not exist in Fusion 360 Python API. Replaced with `design.exportManager.createSTLExportOptions(body, path)` + `exportManager.execute()`.

- **FIX 7** `doc.saveAs(name, path_string)` — wrong argument signature. Replaced with `exportManager.createFusionArchiveExportOptions(path)` to write a local `.f3d` archive.

- **FIX 8** `run_stability_analysis()` — `measureManager.measurePhysicalProperties(product)` does not exist. Replaced with `design.physicalProperties.centerOfMass`.

- **FIX 9** `_interfere()` — `measureManager.measureInterference()` / `design.measureInterference()` do not exist in Python API. Replaced with `design.createInterferenceInput(bodies)` → `design.interferenceAnalysis()`. Fallback chain retained for API-version compatibility.

### Added
- **STEP export** — full assembly exported as `Optimus_Prime_G1_v9.step` via `EXPORT_STEP` flag
- **Demo videos** — transformation, truck mode, and robot mode videos with README integration
- **Technical specifications section** — complete hardware inventory (24 servos, 6 TT motors, 23 bearings, etc.), servo load analysis with safety margins, vertical layout dimensions, materials table
- **Inspiration & Concept section** — Transformers G1 movie origin and project vision
- **Advanced Use Cases section** — robotics education, 3D printing/fabrication, research pipeline
- **Kinematic tree diagram** — full joint hierarchy from grounded pelvis through all 18 joints
- **Joint classification table** — all 18 joints with type, DOF, limits, and servo assignments
- **Body components table** — 19 assemblies with body counts and color schemes

### Changed
- Archive filename: `Optimus_Prime_G1_v9.f3d`
- Documentation overhaul — README expanded from ~250 to ~465 lines with all new sections
- Simulation modules table — added Duration and Key Angles columns
- Output files table — added STEP export entry with full export flag documentation
- FAQs — added 5 new questions (servo count, transformation, STEP export, physical vs simulation, runtime)
- Video files renamed and organized: `Optimus_Prime_Transformation.mp4`, `Optimus_Prime_Truck_Mode.mp4`, `Optimus_Prime_Robot_Mode.mp4`
- Setup & Running guide — MCP server enablement steps (Fusion 360 UI + command line), CLI options table with all 6 flags, MCP communication architecture diagram and step-by-step flow

## [8.0.0] - 2026-06-16

### Changed
- Raised all body centres (pelvis, waist, torso, shoulder, head) by ~1.0 cm to fix torso-pelvis Z-overlap
- Clearance increased from 0.45 mm → 0.60 mm for better FDM tolerance
- Switched from hardcoded `C:\opt_fusion_log.txt` / `C:\OptimusPrime_STL\` to project-local `output/` directory structure
- Added `MG996R-HD` high-duty servo specs (`SERVO_SPECS`)
- Updated joint limits for all body clusters

### Fixed
- `CLEARANCE` parameter now correctly applied to all geometry (was inconsistent in v7)

## [7.0.0] - 2026-06-15

### Added
- Complete kinematic simulation with 9 modules:
  - Joint ROM Test — sweeps every joint min→centre→max with collision checks
  - Head Look-Around — 5-position scan (left, right, up, down, centre)
  - Wave Gesture — full right-arm raise and 3× wrist wave
  - Idle Breathing — 4-cycle subtle torso oscillation
  - Advanced Walking — 4 cycles with hip sway, arm counter-swing, ankle push-off
  - Running — 3 cycles, exaggerated fast gait
  - Combat Sequence — right cross → blaster aim → forearm block → left uppercut
  - Transformation — Robot → Truck (9 stages) + driving + reverse transformation
  - Stability + Loads — 4-pose CoM check + static servo torque table
- Full 3D-printable model with 0.45 mm clearance on all moving fits
- Shell splitting along Y-axis midplane for FDM printing
- M3 screw holes, magnet pockets (Ø6.4 mm × 3.5 mm), and wire channels
- CLI controller (`run_simulation.py`) with single-module and stop-flag support
- Screenshot capture utility (`capture_optimus.py`) — 6 viewport angles
- URDF skeleton export for robotics toolchain import
- Materials and appearances (red/blue metallic, chrome, rubber, glass)
- MG996R and MG90S servo hardware integration

### Fixed
- All `TARGET_MODULE` dispatcher method names corrected
- `split_halves` uses correct `splitBodyFeatures` API
- `_strict_mode` initialised in `__init__`
- URDF export uses indexed joint iteration
- `pose_*` methods reset before applying
- Transformation reverse sequence properly un-does each stage
- `validate_kinematics` root logic fixed (pelvis is true ground)
- `_check()` tightened exception handling
- Log writer buffered (no file-open per message)
- `CLEARANCE` applied to tt_wheel cut-cavity
- `EXPORT_STL` flag added at top for easy toggling

## [6.0.0] - 2026-06-15

### Added
- Initial release of the Fusion 360 Optimus Prime G1 simulation
- 3D-printable model with 0.45 mm clearance
- Basic simulation modules
- Full-body model with 100+ components
- Materials and appearances (red/blue metallic, chrome, rubber, glass)

### Changed
- N/A (initial release)

### Fixed
- N/A (initial release)

[9.0.0]: https://github.com/itsPremkumar/Optimus_Prime/releases/tag/v9.0.0
[8.0.0]: https://github.com/itsPremkumar/Optimus_Prime/releases/tag/v8.0.0
[7.0.0]: https://github.com/itsPremkumar/Optimus_Prime/releases/tag/v7.0.0
[6.0.0]: https://github.com/itsPremkumar/Optimus_Prime/releases/tag/v6.0.0
