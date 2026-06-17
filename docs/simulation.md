# Simulation Modules Guide

The `SimulationEngine` class contains 9 modules that demonstrate the robot's kinematic capabilities. Each module runs a specific sequence of joint movements and reports results.

---

## Module 1: Joint ROM Test (`test_joint_rom`)

**Duration:** ~30s

Sweeps every joint from minimum → centre (0°) → maximum, checking for collisions/interference at each extreme.

**Purpose:** Validates that all joints can reach their declared limits without self-collision.

**Output:**
- Each joint's min/max angles reached
- Any interference detected at extreme poses
- Warnings for joints that can't reach their declared limits

**Interpretation:**
- "Collision detected: A vs B at Xmm overlap" — two bodies intersect; the joint limit may need adjustment
- If a joint logs a ROM warning, the `JOINT_LIMITS` entry may be overly optimistic

---

## Module 2: Head Look-Around (`simulate_head_scan`)

**Duration:** ~8s

Moves the neck cluster through a 5-position scan: centre, left, right, up, down, back to centre.

**Angles:**
| Position | Yaw | Pitch |
|----------|:---:|:-----:|
| Centre | 0° | 0° |
| Left | -50° | 0° |
| Right | 50° | 0° |
| Up | 0° | -25° |
| Down | 0° | 35° |

**Purpose:** Demonstrates head mobility — useful for checking neck clearance with chest and collars.

---

## Module 3: Wave Gesture (`simulate_wave`)

**Duration:** ~10s

Raises the right arm fully and waves the wrist 3 times.

**Sequence:**
1. Raise right arm: shoulder pitch to -90° (arm horizontal)
2. Wave: wrist roll ±90° × 3 cycles
3. Return arm to rest

**Purpose:** Tests right arm range of motion and wrist roll capability.

---

## Module 4: Idle Breathing (`simulate_idle_breathing`)

**Duration:** ~12s

Gentle torso oscillation — waist pitch oscillates ±3° for 4 cycles.

**Purpose:** Subtle idle animation. Useful for testing smooth joint interpolation and low-angle precision.

---

## Module 5: Advanced Walking (`simulate_walking_advanced`)

**Duration:** ~20s

4-cycle walking gait with coordinated joint movements.

**Gait Pattern (1 cycle):**

| Phase | L Hip Yaw | R Hip Yaw | L Knee | R Knee | L Ankle | R Ankle | L ShP | R ShP |
|-------|:---------:|:---------:|:------:|:------:|:-------:|:-------:|:-----:|:-----:|
| 1 | 0° | 30° | 20° | 0° | 0° | 30° | 10° | -10° |
| 2 | 20° | 20° | 60° | 0° | -10° | 30° | 10° | -10° |
| 3 | 30° | 0° | 0° | 20° | 30° | 0° | -10° | 10° |
| 4 | 20° | 20° | 0° | 60° | 30° | -10° | -10° | 10° |

- Right hip moves through yaw (0°→30°→0°)
- Left knee bends (0°→60°→0°) for swing phase
- Both ankles adjust for ground clearance
- Arms counter-swing: left arm forward when right leg forward

**Purpose:** Demonstrates coordinated multi-joint gait with arm-leg coupling.

---

## Module 6: Running (`simulate_running`)

**Duration:** ~15s

3-cycle running gait with larger joint angles and faster pace than walking.

**Key differences from walking:**
- Knee bend increased to 95° (vs 60° in walking)
- Hip yaw range increased (0°→45°→0°)
- Hip pitch used for forward lean
- Faster step transitions

**Purpose:** Tests higher-stress joint movements and rapid direction changes.

---

## Module 7: Combat Sequence (`simulate_combat`)

**Duration:** ~12s

4-move combat sequence:

1. **Right cross punch:** Right shoulder yaw 60°, pitch -90°, roll 45°
2. **Blaster aim:** Right elbow 90°, wrist -45°, blaster aimed forward
3. **Forearm block:** Right shoulder roll -45°, forearm vertical
4. **Left uppercut:** Left shoulder pitch -60°, elbow -20°

**Purpose:** Demonstrates upper-body dexterity and multi-axis arm control.

---

## Module 8: Transformation (`simulate_transformation`)

**Duration:** ~30s

Full robot-to-truck-to-robot transformation cycle.

### Robot → Truck (5 stages)

| Stage | Action |
|:-----:|--------|
| 1 | Straighten elbows (0°), fold blaster (-90°), fold wrists |
| 2 | Tuck head backward into chest (neck pitch 45°) |
| 3 | Fold shoulders backward (shoulder yaw 90°) |
| 4 | Rotate hips backward (hip yaw 95°) |
| 5 | Fold ankles flat (ankle yaw 95°) |

### Truck → Robot (5 stages, reverse)

Reverse of the above stages, restoring robot mode.

**Interference check:** After transformation, `_interfere()` checks for collisions in truck mode.

**Purpose:** Flagship module — validates the complete transformation mechanism.

---

## Module 9: Stability + Loads

Two sub-modules:

### 9a. Stability Analysis (`run_stability_analysis`)

Checks centre of mass for 4 poses:

| Pose | Key Angles | Criteria |
|------|-----------|----------|
| Attention | All joints 0° | CoM\|x\|<3.0, \|y\|<5.0 |
| Combat | Waist 10°, Knees 45°, Elbows 90°, R_ShR -45° | CoM\|x\|<3.0, \|y\|<5.0 |
| Squat | Waist 20°, Knees 90°, Hips -45° yaw | CoM\|x\|<3.0, \|y\|<5.0 |
| Victory | Sh 60° yaw, -90° roll, Elbows 30°, Waist 15° pitch | CoM\|x\|<3.0, \|y\|<5.0 |

### 9b. Servo Load Estimation (`estimate_servo_loads`)

Calculates torque requirements for 10 critical joints:

| Joint | Mass (g) | Arm (cm) | Torque (kg·cm) | Servo | Rating | Margin |
|-------|:--------:|:--------:|:--------------:|:-----:|:-----:|:------:|
| Neck Pitch | 120 | 3.0 | 0.36 | MG90S | 1.8 | 5.00× |
| Shoulder Pitch | 390 | 12.0 | 4.68 | MG996R | 9.4 | 2.01× |
| Elbow | 210 | 7.0 | 1.47 | MG996R | 9.4 | 6.39× |
| Hip Pitch | 820 | 15.0 | 12.30 | MG996R-HD | 20.0 | 1.63× |
| Knee Pitch | 540 | 9.0 | 4.86 | MG996R | 9.4 | 1.93× |
| Waist Pitch | 2100 | 8.0 | 16.80 | MG996R-HD | 25.0 | 1.49× |

**Margin thresholds:**
- ≥ 1.5× = **OK** (safe)
- 0.9× – 1.5× = **MARGINAL** (may struggle under load)
- < 0.9× = **OVERLOAD** (servo is undersized)
