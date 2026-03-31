# mmWave RIO Body-Velocity Integration into ArduCopter Velocity Control Loop

## Document Purpose

This document describes how to use a mmWave Radar Inertial Odometry (RIO) module's body-velocity estimates as the velocity feedback source for ArduCopter's velocity control loop — **without any firmware changes**.

It is intended for anyone picking up this work: it explains the project goal, the system architecture, why the chosen approach works, the exact steps to configure and test it, and what to look for when evaluating results.

---

## Project Context and Goal

### Hardware setup

```
┌────────────────────────────────────┐
│  Companion Computer (CC)           │
│  ┌───────────────────────────┐     │
│  │  mmWave RIO module        │     │
│  │  (body-frame velocity)    │     │
│  └──────────┬────────────────┘     │
│             │ internal bus         │
│  ┌──────────▼────────────────┐     │
│  │  CC application           │     │
│  │  (frame conversion +      │     │
│  │   MAVLink publisher)      │     │
│  └──────────┬────────────────┘     │
└────────────-│----──────────────────┘
              │ UART (MAVLink)
┌─────────────▼──────────────────────┐
│  ArduCopter Flight Controller      │
│  (EKF3 + velocity control loop)    │
└────────────────────────────────────┘
```

The RIO module runs on the Companion Computer and outputs **body-frame (FRD: Forward-Right-Down) linear velocity** estimates. These are better than GPS-derived velocity for short-term hover stability because they are not affected by multipath or GPS latency.

### Goal

Use the RIO velocity estimates as the **XY velocity feedback** inside ArduCopter's EKF3 state estimator, which then feeds the position/velocity control loop responsible for holding hover.

### Constraint: No firmware changes (prototyping phase)

All configuration must be achieved through:
1. MAVLink messages sent by the Companion Computer.
2. ArduCopter parameter changes via GCS (Mission Planner / QGroundControl) or MAVLink `SET_PARAM`.

No recompilation of ArduCopter firmware is needed. The required pipeline already exists in stock ArduCopter builds.

---

## ArduCopter Internal Architecture (Relevant Subset)

Understanding why this works requires knowing how ArduCopter's velocity estimate flows from sensor to control output.

### Cascaded velocity control chain

```
External velocity measurement (MAVLink)
    ↓
AP_VisualOdom bridge  (VISO_TYPE=1)
    ↓
AP_AHRS::writeExtNavVelData()  [NED m/s]
    ↓
NavEKF3 — storedExtNavVel buffer
    ↓
SelectVelPosFusion()  [when EK3_SRC1_VELXY=6]
    ↓
EKF velocity states updated (vN, vE, vD in NED)
    ↓
AP_AHRS::get_velocity_NED()
    ↓
AP_InertialNav  [converts NED→NEU, m/s→cm/s]
    ↓
AC_PosControl::update_xy_controller()
    │
    ├── Position P loop  → velocity target (cm/s)
    └── Velocity PID loop  → reads _inav.get_velocity_xy_cms()
                          → acceleration target → lean angle → rate controller
```

### Why no firmware changes are needed

ArduPilot has a built-in pipeline specifically for external velocity sensors (visual odometry, range cameras, etc.). It is enabled by default in any ArduCopter build on a flight controller with more than 1 MB of flash (i.e., any modern FC):

- `HAL_VISUALODOM_ENABLED` — compile-time flag, true when `BOARD_FLASH_SIZE > 1024`
- `EK3_FEATURE_EXTERNAL_NAV` — compile-time flag, true when `BOARD_FLASH_SIZE > 1024`

The only requirement is activating the pipeline via parameters and sending the correct MAVLink message.

### Key source files (for reference, no changes needed)

| File | Role |
|------|------|
| `libraries/GCS_MAVLink/GCS_Common.cpp:3958` | Decodes `VISION_SPEED_ESTIMATE` MAVLink message |
| `libraries/AP_VisualOdom/AP_VisualOdom_MAV.cpp:62` | Bridges VisualOdom to `AP_AHRS::writeExtNavVelData()` |
| `libraries/AP_NavEKF3/AP_NavEKF3_Measurements.cpp:1111` | Writes NED velocity into EKF3 ring buffer |
| `libraries/AP_NavEKF3/AP_NavEKF3_PosVelFusion.cpp:588` | Fuses external nav velocity when `EK3_SRC1_VELXY=6` |
| `libraries/AP_NavEKF/AP_NavEKF_Source.h` | Defines `SourceXY::EXTNAV = 6` enum value |
| `libraries/AC_AttitudeControl/AC_PosControl.cpp` | Velocity PID reads from `_inav.get_velocity_xy_cms()` |

---

## Implementation Plan

### Part 1 — Companion Computer side

#### Step 1.1: Obtain attitude from the FC

The RIO provides body-frame (FRD) velocity. The EKF expects NED. The CC must rotate the velocity using the current vehicle attitude.

Subscribe to one of these MAVLink messages from the FC to get the rotation:
- `ATTITUDE_QUATERNION` (msg ID 31) — quaternion `q1,q2,q3,q4` (w,x,y,z)
- `ATTITUDE` (msg ID 30) — Euler angles `roll, pitch, yaw` (radians)

#### Step 1.2: Convert body-frame velocity to NED

```python
# Pseudocode — adapt to your language/library

import numpy as np
from scipy.spatial.transform import Rotation

# q = [w, x, y, z] from ATTITUDE_QUATERNION
# v_body = [vx_fwd, vy_right, vz_down] from RIO (FRD convention)

R_body_to_NED = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
v_ned = R_body_to_NED @ v_body
# v_ned = [vN, vE, vD]
```

> **Frame note:** ArduPilot uses NED (North-East-Down). RIO outputs FRD (Forward-Right-Down). These are the same convention relative to the body — only the rotation to NED is needed, not any axis flip.

#### Step 1.3: Send VISION_SPEED_ESTIMATE at regular rate

MAVLink message: **`VISION_SPEED_ESTIMATE`** (message ID 103)

| Field | Value |
|-------|-------|
| `usec` | Monotonic timestamp in microseconds |
| `x` | Velocity North (m/s) |
| `y` | Velocity East (m/s) |
| `z` | Velocity Down (m/s) |
| `reset_counter` | 0 (increment only on sensor reset) |

**Recommended rate:** 50–100 Hz (match or divide-down from RIO output rate).

```python
# Pseudocode using pymavlink

msg = connection.mav.vision_speed_estimate_encode(
    usec=int(time.monotonic() * 1e6),
    x=v_ned[0],   # vN m/s
    y=v_ned[1],   # vE m/s
    z=v_ned[2],   # vD m/s
    reset_counter=0
)
connection.mav.send(msg)
```

> **Do NOT use `VISION_POSITION_DELTA`** — that message takes the body-odometry path (`writeBodyFrameOdom` → `FuseBodyVel()`), which is a different EKF fusion path with different behavior and requirements.

---

### Part 2 — ArduCopter parameter configuration (FC side)

Set these via Mission Planner / QGroundControl Full Parameter List, or via `MAV_CMD_SET_PARAM`.

#### 2.1 Enable the VisualOdom MAVLink bridge

| Parameter | Value | Notes |
|-----------|-------|-------|
| `VISO_TYPE` | **1** | Activate MAVLink visual odometry source. Default is 0 (disabled) — this is the most critical parameter. |
| `VISO_DELAY_MS` | **50** | Start here. Tune toward actual RIO pipeline latency (sensor → CC → MAVLink → FC). Typical range: 30–100 ms. |
| `VISO_VEL_M_NSE` | **0.3** | Velocity noise in m/s. Start conservative. Reduce toward 0.1 as confidence in RIO accuracy grows. |
| `VISO_QUAL_MIN` | **0** | Accept good or unknown quality (default). Set to -1 to always accept regardless of quality field. |
| `VISO_POS_X` | measure | Forward offset of radar from IMU in body frame (meters). Positive = forward. |
| `VISO_POS_Y` | measure | Right offset of radar from IMU in body frame (meters). Positive = right. |
| `VISO_POS_Z` | measure | Down offset of radar from IMU in body frame (meters). Positive = down. |

#### 2.2 Configure EKF3 velocity source

| Parameter | Value | Notes |
|-----------|-------|-------|
| `EK3_SRC1_VELXY` | **6** | Use EXTNAV (external nav) for horizontal velocity. Value 6 = `SourceXY::EXTNAV`. |
| `EK3_SRC1_VELZ` | **3** | Keep GPS for vertical velocity initially. Set to 6 for EXTNAV Z if RIO provides good vD. |
| `EK3_SRC1_POSXY` | **3** | Keep GPS for horizontal position. This is safe — position and velocity sources are independent. |
| `EK3_SRC1_POSZ` | **1** | Keep barometer for altitude. |

> **Conservative first test:** Change only `EK3_SRC1_VELXY=6`. Keep everything else at GPS/baro defaults. This means GPS still drives position; only the velocity feedback uses RIO. The risk of EKF instability is minimized.

#### Why we want velocity-only EXTNAV support

In this project, the RIO velocity estimate is currently more trustworthy than the RIO position estimate. EKF3 can ingest external position and velocity on separate paths, but once low-quality external position is fused it can still contaminate the EKF state through the shared covariance update.

The preferred architecture is therefore:

1. use EXTNAV horizontal velocity where it adds real value
2. avoid forcing EXTNAV horizontal position into EKF3 only to satisfy readiness gating
3. keep the ArduPilot change minimal by adjusting EKF3 readiness logic instead of reworking Copter mode logic

This is why the minimal patch direction is to split EKF3 EXTNAV readiness into:

1. external-position readiness
2. external-velocity readiness

so velocity-only EXTNAV operation can be supported more safely.

#### 2.3 Enable detailed logging (strongly recommended)

| Parameter | Value | Notes |
|-----------|-------|-------|
| `LOG_DISARMED` | **1** | Allow logging before arming for bench tests. |
| `LOG_BITMASK` | add bit 4 | Enables `ATTITUDE_FAST` → triggers `XKF*` EKF log messages at 400 Hz. |

---

## Verification and Testing

### Bench test (before any flight)

1. Power up copter on bench (do not arm).
2. Start CC application sending `VISION_SPEED_ESTIMATE` with zero velocity (copter stationary).
3. Open GCS and check:
   - **EKF status** — should remain healthy (green indicators).
   - **XKF4 log** — download log after 30 s. Look at the normalized innovation ratios for velocity. They should be near 0 (RIO says 0, EKF expects ~0 from GPS, should agree).
4. Move the copter by hand and observe velocity innovations. The EKF should track the RIO estimates.

### First hover (LOITER mode, GPS primary)

1. Fly in calm conditions with good GPS.
2. Hover for 60 s. Download log.
3. In log viewer (Mission Planner or MAVExplorer):
   - Check `XKF4` normalized innovation ratios for velocity (`VN`, `VE`). Values < 1.0 mean the RIO and EKF are consistent.
   - Compare `VISO` log messages — these record data received from the visual odometry bridge.
   - Compare hover position scatter vs a GPS-only baseline flight.

### Tuning `VISO_VEL_M_NSE`

This parameter controls how much the EKF trusts the RIO velocity vs its own IMU integration:
- **Too high (e.g., 1.0 m/s):** RIO data is mostly ignored. Little effect on behavior.
- **Too low (e.g., 0.01 m/s):** EKF over-trusts RIO. If RIO has any bias, position will drift.
- **Good starting point:** 0.3 m/s. Reduce toward 0.05–0.1 if hover quality improves and innovations look good.

### Innovation diagnostic

| Log message | Field | Good value | Action if bad |
|-------------|-------|------------|---------------|
| `XKF4` | `SV` (velocity NI ratio) | < 1.0 | If > 1.0: increase `VISO_VEL_M_NSE` or check frame conversion |
| `XKF4` | `SP` (position NI ratio) | < 1.0 | If growing: position source mismatch |
| `VISO` | quality | > 0 | If 0: check `VISO_QUAL_MIN`, RIO output |

---

## Recommended Test Progression

```
Phase 1 — Bench (no flight)
  → Verify pipeline active: GCS shows VISO data received
  → Verify EKF accepts data: XKF4 innovations near 0 at rest

Phase 2 — First hover (LOITER, GPS+RIO velocity)
  → EK3_SRC1_VELXY=6, everything else default
  → Evaluate hover quality vs GPS-only baseline
  → Check XKF4 innovations

Phase 3 — Tune VISO_VEL_M_NSE
  → Reduce noise figure as confidence grows

Phase 4 — (Future, after confidence) GPS-denied hover
  → Consider EK3_SRC1_POSXY=6 with an external position source
  → Or combine with optical flow for position hold
```

---

## Known Constraints and Watch-outs

### Frame conversion is the CC's responsibility

The RIO outputs FRD body-frame velocity. `VISION_SPEED_ESTIMATE` must be in NED. The CC **must** apply the body-to-NED rotation using current attitude before sending. Sending body-frame velocity directly as NED will cause the EKF to diverge whenever the vehicle is not level.

### EKF3 internal frame is always NED

The `writeExtNavVelData()` path stores NED velocity in the EKF ring buffer and fuses it as a NED measurement. This is different from the body-frame odometry path (`VISION_POSITION_DELTA` → `writeBodyFrameOdom()` → `FuseBodyVel()`), which fuses body-frame increments. Do not mix these up.

### VISO_TYPE=0 means the bridge is disabled

The default value is 0 (None). The pipeline does nothing until `VISO_TYPE=1` is set. This is the single most common mistake.

### Timestamp accuracy affects EKF alignment

The EKF uses `VISO_DELAY_MS` to align the external velocity measurement to the correct fusion time horizon in its ring buffer. If CC timestamps are inaccurate or variable, tune this parameter. Typical values: 30–80 ms.

### VISO_QUAL_MIN gates the data

If the RIO module sends a quality value of 0 in the MAVLink message and `VISO_QUAL_MIN > 0`, the measurement is dropped silently. Set `VISO_QUAL_MIN=0` (accept good/unknown) or `-1` (always accept) if unsure about the quality field.

---

## Parameter Quick-Reference

| Parameter | Recommended Start Value | Description |
|-----------|------------------------|-------------|
| `VISO_TYPE` | 1 | Enable MAVLink visual odometry bridge |
| `VISO_DELAY_MS` | 50 | Sensor pipeline latency in ms |
| `VISO_VEL_M_NSE` | 0.3 | Velocity noise 1-sigma (m/s) |
| `VISO_QUAL_MIN` | 0 | Minimum quality to accept |
| `VISO_POS_X` | measured | Radar offset forward from IMU (m) |
| `VISO_POS_Y` | measured | Radar offset right from IMU (m) |
| `VISO_POS_Z` | measured | Radar offset down from IMU (m) |
| `EK3_SRC1_VELXY` | 6 | EXTNAV source for horizontal velocity |
| `EK3_SRC1_VELZ` | 3 | GPS source for vertical velocity (start safe) |
| `EK3_SRC1_POSXY` | 3 | GPS source for horizontal position |
| `EK3_SRC1_POSZ` | 1 | Baro source for altitude |
| `LOG_DISARMED` | 1 | Log on bench without arming |

---

## Related Documents in This Folder

- `CONTROL_ARCHITECTURE.md` — Full cascaded control architecture, all flight modes, scheduler task ordering, and layer-by-layer signal flow.
- `EKF3_DATA_FLOW.md` — How EKF3 ingests sensor data, fuses measurements, and exposes outputs to the control loop.
- `EKF3_CORE_MATH.md` — EKF3 state vector, covariance prediction, strapdown kinematics, output predictor math.
- `IMU_DATA_FLOW.md` — IMU data flow from HAL through EKF to attitude control.
- `SYSTEMID_INNER_LOOP.md` — SystemID inner-loop logging for plant identification (separate concern).
