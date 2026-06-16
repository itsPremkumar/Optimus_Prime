# Changelog

## [6.0.0] - 2026-06-15

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

### Changed
- N/A (initial release)

### Fixed
- N/A (initial release)

## [5.0.0] - 2026-05-xx
- Internal pre-release (not publicly versioned)

[6.0.0]: https://github.com/itsPremkumar/Optimus_Prime/releases/tag/v6.0.0
