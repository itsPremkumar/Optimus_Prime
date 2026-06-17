# Changelog

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

### Changed
- Archive filename: `Optimus_Prime_G1_v9.f3d`

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
