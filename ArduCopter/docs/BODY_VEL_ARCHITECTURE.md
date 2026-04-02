# Body-Frame Velocity Architecture: Findings and Decision

> **Status:** Plan B body-velocity ingress and the FlowHold body-frame controller hand-off are
> implemented.
>
> This document records the design investigation for integrating RIO body-frame radar velocity
> into ArduCopter, the options evaluated, and the conclusions reached.

---

## 1. Problem Statement

The goal is **pure body-frame velocity control** of an indoor drone using a companion-computer
radar (RIO) as the only non-IMU sensor:

- **Sensor:** mmWave radar on companion computer -> body-frame (FRD) linear velocity
- **Absent:** GPS, yaw measurement, optical flow, external position
- **Requirements:**
  1. Fuse body-frame radar velocity into EKF3 state estimator
  2. RC-driven velocity control: pilot stick deflection commands body Vx/Vy, throttle
     controls altitude - same pilot interaction model as FlowHold (optical flow)
  3. Minimal firmware changes - reuse existing pipelines where possible
  4. No dependence on world-frame coordinates or yaw

---

## 2. Key Findings

### 2.1 Why the earth-frame path fails without yaw

The obvious path - `VISION_SPEED_ESTIMATE` -> `writeExtNavVelData()` -> EKF3 NED velocity
fusion - is the wrong abstraction for this use case.

Why:
- `VISION_SPEED_ESTIMATE` semantics are earth-frame / NED velocity
- converting radar body velocity to NED requires yaw
- without compass, GPS heading, or external yaw, that conversion is underconstrained
- the older `readyToUseExtNav()` path also expects a position-oriented EXTNAV route

Conclusion:
- the EXTNAV/NED velocity path pushes the hardest problem upstream
- it is not the right primary route for IMU + radar body velocity without yaw

### 2.2 EKF3 already has the right fusion core

ArduPilot already has a body-odometry pipeline:

```text
VISION_POSITION_DELTA
  -> AP_VisualOdom_MAV::handle_vision_position_delta_msg()
  -> AP_AHRS::writeBodyFrameOdom()
  -> NavEKF3_core::writeBodyFrameOdom()
  -> storedBodyOdm
  -> SelectBodyOdomFusion()
  -> FuseBodyVel()
```

Important properties:
- `readyToUseBodyOdm()` does not require yaw alignment
- `FuseBodyVel()` compares predicted and measured velocity in **body frame**
- aiding can enter `AID_RELATIVE`, which matches the use case

Conclusion:
- the missing piece is mostly an ingress/API mismatch, not missing Kalman math

### 2.3 The input-format mismatch is real

`writeBodyFrameOdom()` accepts deltas and internally converts them back into velocity.

RIO provides velocity directly. Reconstructing synthetic deltas only to have EKF3 convert them
back into velocity is:
- semantically wrong
- unnecessary
- more fragile than a direct velocity ingress

Conclusion:
- add a direct `writeBodyFrameVel()` path instead of faking `delPos`

### 2.4 FlowHold is the right control architecture

GUIDED velocity control is still earth-frame in practice. Even body-frame guided commands are
rotated into NE before the relevant control path.

FlowHold is closer to the desired pilot interaction:
- RC roll/pitch remain pilot-facing body-oriented lean commands
- centered sticks + nonzero velocity produce braking
- stick release resumes hold/braking
- no GPS is required

However, the final route still needs one cleanup:
- FlowHold should consume canonical body velocity
- it should not reconstruct body velocity locally from earth-frame velocity output

---

## 3. Chosen Architecture

### 3.1 Expected Frame Ledger — Estimator Path

**Reuse `FuseBodyVel()` with a new direct velocity entry point, and avoid introducing a new FC
routing parameter.**

The point of this ledger is to track **which frame each quantity is in**, **what transform is
applied**, and **whether that transform is considered safe to keep**.

| Step | Function / data | Frame in | Math / transform | Frame out | Keep? | Why |
|------|------------------|----------|------------------|-----------|-------|-----|
| 1 | RIO radar / MAVLink `ODOMETRY` | Body FRD velocity, body angular rate | None | Body FRD | Yes | This is the clean physical measurement we trust |
| 2 | `GCS_MAVLINK::handle_odometry()` | Body FRD | **Plan B branch:** explicit ArduPilot body-twist contract; no body->NED rotation; reject ambiguous pose+twsit packets | Body FRD | Yes | Prevents early world-frame leakage without relying on heuristics |
| 3 | `AP_VisualOdom::handle_body_frame_velocity_estimate()` | Body FRD | No frame change; only enable/dispatch logic | Body FRD | Yes | Pure transport/front-end step |
| 4 | `AP_VisualOdom_MAV::handle_body_frame_velocity_estimate()` | Body FRD | No frame change; only quality, delay, noise, and offset handling | Body FRD | Yes | Still measurement-side bookkeeping |
| 5 | `AP_AHRS::writeBodyFrameVel()` | Body FRD | No frame change; thin bridge into EKF3 | Body FRD | Yes | Pure transport step |
| 6 | `NavEKF3::writeBodyFrameVel()` / `NavEKF3_core::writeBodyFrameVel()` | Body FRD | No frame change; write directly into `storedBodyOdm` as body velocity + angular rate | Body FRD | Yes | Avoids fake `vel -> delPos -> vel` conversion |
| 7 | `readyToUseBodyOdm()` | Body FRD measurement already buffered | No frame change; only freshness / source / alignment checks | N/A | Yes | Gate only; no math reinterpretation |
| 8 | `FuseBodyVel()` | State velocity in **local NED**, measurement in **body FRD** | `bodyVelPred = prevTnb * stateStruct.velocity`; innovation formed in **body frame** as `bodyVelPred - bodyVelMeas` | Innovation in body frame; state remains mixed-frame | **Yes, for now** | This is the EKF's own internally consistent measurement model |
| 9 | `AP_AHRS::get_velocity_body()` | Estimator output is internally world/nav-based | Canonical estimator-owned nav->body output mapping | Body FRD | **Yes, for now** | Controller should consume body velocity, not reconstruct it locally |

Frame-critical conclusion:
- the measurement enters in **body frame**
- the EKF comparison is done in **body frame**
- the EKF nominal velocity state remains in **local NED**
- the estimator-side transforms we keep are the one inside `FuseBodyVel()` and the canonical
  estimator-output mapping used to expose body velocity to the controller

Important clarification about the EKF update:
- the body-frame innovation path above is the **original existing EKF3 body-odometry mechanism**
  reached today through `writeBodyFrameOdom()`
- EKF3 does **not** perform a second explicit "rotate innovation back to NED" step after
  `innovBodyVel = bodyVelPred - bodyVelMeas`
- instead, the body-frame residual is mapped into mixed-frame state corrections by the
  observation Jacobian and Kalman gain (`H_VEL`, `Kfusion`)
- that is exactly why the plan is estimator-safe: it changes the **ingress transport**, not the
  EKF measurement model, Jacobians, or state-update logic

Critical consistency points:
- do not feed body-frame data into `writeExtNavVelData()`
- do not synthesize `delPos = vel * dt` just to reuse `writeBodyFrameOdom()`
- do not accidentally inject pose/quaternion into the RIO twist-only route
- keep the new route opt-in through explicit message contract semantics, not by changing all `ODOMETRY` behavior

Why this transport is preferred over a new `VISO_VEL_FRAME` parameter:
- `ODOMETRY` already carries frame metadata
- `ODOMETRY` already carries body angular rates
- this avoids depending on a new user-facing FC parameter
- this avoids assuming Mission Planner will expose new parameters

### 3.1.1 Plan B: Explicit ArduPilot `ODOMETRY` Contract

The original implementation used a heuristic:

- `frame_id = MAV_FRAME_LOCAL_FRD`
- `child_frame_id = MAV_FRAME_BODY_FRD`
- all three angular-rate fields finite

That heuristic is not a clean wire contract because MAVLink `ODOMETRY` legitimately allows a
sender to provide:

- valid pose
- valid twist
- valid angular rates

at the same time.

For Plan B, the body-twist route becomes an **explicit ArduPilot-local contract over existing
`ODOMETRY`**, with no new user-facing flight-controller parameter and no immediate MAVLink
standardization work.

#### Body-twist-only packet contract

A sender that wants ArduPilot to use the direct body-velocity route must send `ODOMETRY` with:

- `frame_id = MAV_FRAME_LOCAL_FRD`
- `child_frame_id = MAV_FRAME_BODY_FRD`
- `estimator_type = MAV_ESTIMATOR_TYPE_UNKNOWN`
- finite `vx, vy, vz`
- finite `rollspeed, pitchspeed, yawspeed`
- pose explicitly invalid / absent:
  - `pose_covariance[0] = NaN`
  - quaternion is ignored by ArduPilot on this route and should not be used to convey pose

Interpretation:

- `estimator_type = UNKNOWN` does **not** mean MAVLink standard has defined a body-twist-only
  semantic
- it is an **ArduPilot-documented sender convention**
- this is deliberately local to the ArduPilot + companion ecosystem for now

#### Legacy `ODOMETRY` remains supported

Any `ODOMETRY` packet that does **not** match the explicit body-twist contract continues down the
legacy route:

- pose ingestion through `handle_pose_estimate()`
- FRD velocity rotated by quaternion into NED
- velocity ingestion through `handle_vision_speed_estimate()`

#### Ambiguous packets must be rejected

ArduPilot should not silently choose one interpretation when a packet looks like both:

- explicit body-twist-only, and
- valid pose-bearing odometry

If the sender claims the body-twist contract but also provides pose-looking fields, ArduPilot
should:

- reject the body-twist route for that packet
- emit a rate-limited warning

This is safer than silently discarding pose.

#### Why this is the best practical fix now

This gives us:

- explicit receiver behavior
- no new FC parameter
- no Mission Planner dependency
- no MAVLink upstream dependency
- companion-side changes only in the packet contract

The longer-term cleaner option is still a MAVLink-standard body-twist contract, but Plan B avoids
blocking on that work.

### 3.2 Expected Frame Ledger — Control Path

**Use FlowHold (mode 22) with RC input, but finish the body-side cleanup.**

| Step | Function / data | Frame in | Math / transform | Frame out | Keep? | Why |
|------|------------------|----------|------------------|-----------|-------|-----|
| 1 | Pilot RC input | Pilot body-intent | Roll/pitch -> lean demand, throttle -> climb rate, yaw -> yaw-rate demand | Body lean + yaw-rate intent | Yes | Matches desired pilot interaction |
| 2 | `ModeFlowHold::run()` | Pilot intent + EKF health | Combines pilot lean with horizontal correction only when `flags.horiz_vel` is valid | Body-lean command path | Yes | Correct mode-level structure |
| 3 | `AP_AHRS::get_velocity_body()` | Estimator output | Canonical body-velocity output API | Body FRD velocity | Yes | Controller should consume body velocity directly |
| 4 | `ModeFlowHold::flowhold_flow_to_angle()` | Body FRD velocity | Remap to FlowHold's historical optical-flow axes: `sensor_flow = [-v_right, +v_forward]` | Body/optical-flow axes | Yes | Axis relabel only; no world-frame meaning introduced |
| 5 | `flowhold_flow_to_angle()` | Body/optical-flow axes | Clamp + low-pass filter + braking + PI in body axes | Body/optical-flow correction | Yes | Intended final control structure |
| 6 | `flowhold_flow_to_angle()` | Body/optical-flow correction | Directly sum into `bf_angles` roll/pitch channels | Body lean correction | Yes | No earth-frame detour remains in the horizontal loop |
| 7 | `attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(...)` | Body lean + yaw-rate command | Attitude/rate control and motor mixing | Motor response | Yes | Normal multirotor control path |

Pilot interaction:
- **sticks centered, drone moving:** braking lean angle applied until velocity ~= 0
- **stick deflection:** pilot lean added directly; braking suspended during stick input
- **stick released:** braking resumes from the current velocity
- **EKF velocity invalid:** FlowHold resets horizontal PI/filter/braking state, skips horizontal
  correction, and behaves like AltHold on XY until `flags.horiz_vel` is valid again
- **generic position modes:** body-velocity-only EXTNAV aiding is **not** treated as generic
  relative-position availability for other `position_ok()`-gated modes; this route remains
  intentionally scoped to FlowHold unless a true relative-position source is also configured.
  `ekf_has_relative_position()` now unlocks only for optical flow, dead reckoning, or
  EXTNAV configurations that provide XY position, not velocity-only EXTNAV.

Control-side implementation status:
- `AP_AHRS::get_velocity_body()` now provides the horizontal velocity used by FlowHold
- FlowHold clamp, filter, braking, PI, and stored `xy_I` now remain in body/optical axes through
  the full horizontal loop
- removing the earth-frame PI detour avoids stale integral corrections rotating into the wrong body
  axis after yaw maneuvers

Control-side caution:
- FlowHold is the right pilot interface, but it is not a pure body-velocity setpoint controller
- pilot sticks still primarily command lean-angle behavior
- that is acceptable for this test route, but it should be described honestly

---

## 4. Parameter Configuration

```text
# Visual odometry / MAVLink bridge
VISO_TYPE       = 1        # Enable MAVLink visual odometry plumbing
VISO_VEL_M_NSE  = 0.3      # Radar velocity noise 1-sigma - tune to actual sensor
VISO_DELAY_MS   = 50       # Radar pipeline latency - measure and tune
VISO_POS_X/Y/Z  = measured # Radar offset from IMU in body frame (m)

# EKF3 source configuration
EK3_SRC1_POSXY  = 0        # None (no position source)
EK3_SRC1_VELXY  = 6        # EXTNAV (activates body odometry path for body-frame XY fusion)
EK3_SRC1_POSZ   = 1        # Barometer
EK3_SRC1_VELZ   = 0        # None by default; set to 6 only if radar Z velocity is trustworthy
EK3_SRC1_YAW    = 0        # None

# FlowHold tuning
FHLD_FLOW_MAX   = 2.0
FHLD_BRAKE_RATE = 8
FHLD_XY_P       = tune
FHLD_XY_I       = tune
FHLD_FILT_HZ    = 5

# Optical flow - disable
FLOW_TYPE       = 0
```

Companion-computer message contract (original direct-route form):
- MAVLink `ODOMETRY`
- `frame_id = MAV_FRAME_LOCAL_FRD`
- `child_frame_id = MAV_FRAME_BODY_FRD`
- `vx, vy, vz` = radar body-frame linear velocity
- `rollspeed, pitchspeed, yawspeed` = body angular rates
- `quality` = sensor confidence if available

Companion-computer message contract (Plan B explicit form):
- MAVLink `ODOMETRY`
- `frame_id = MAV_FRAME_LOCAL_FRD`
- `child_frame_id = MAV_FRAME_BODY_FRD`
- `estimator_type = MAV_ESTIMATOR_TYPE_UNKNOWN` (**ArduPilot-local convention**)
- `vx, vy, vz` = radar body-frame linear velocity
- `rollspeed, pitchspeed, yawspeed` = body angular rates
- `pose_covariance[0] = NaN`
- quaternion invalid/unused by contract for this route
- `quality` = sensor confidence if available

Important note:
- the architecture should not assume Mission Planner must expose any new parameter
- that is why the preferred route avoids a new parameter-based frame switch
- Mission Planner does not require mandatory protocol changes for Plan B unless we later want UI,
  packet generation, or packet-inspection support for this contract
- Body-frame XY fusion is enabled by `EK3_SRC1_VELXY = EXTNAV`
- Body-frame Z fusion is independent and is enabled only when `EK3_SRC1_VELZ = EXTNAV`

### 4.1 Instructions for CC-Side

If you are implementing the companion-computer sender, treat this as a strict packet contract, not
as a best-effort hint.

Send one MAVLink `ODOMETRY` message per radar update with:

- `frame_id = MAV_FRAME_LOCAL_FRD`
- `child_frame_id = MAV_FRAME_BODY_FRD`
- `estimator_type = MAV_ESTIMATOR_TYPE_UNKNOWN`
- `vx, vy, vz` = body-frame linear velocity in **FRD**
  - `+x` = forward
  - `+y` = right
  - `+z` = down
- `rollspeed, pitchspeed, yawspeed` = body angular rates in **rad/s**
- `pose_covariance[0] = NaN`
- `quality` = sensor confidence if available, otherwise `0`

CC-side rules:

- Do **not** rotate radar velocity into NED or any world frame before sending it.
- Do **not** use quaternion fields to convey pose on this route; ArduPilot ignores them for the
  body-twist contract.
- Do **not** send a valid pose covariance with `estimator_type = UNKNOWN`; ArduPilot treats that as
  an ambiguous packet and rejects it.
- Do **not** zero missing angular-rate fields; if the CC cannot provide finite body rates, this
  route is not valid for that packet.
- Keep `time_usec` aligned to the measurement/receive time used to characterize the radar pipeline,
  because ArduPilot applies `VISO_DELAY_MS` against that timestamp.
- Measure and tune `VISO_DELAY_MS` from the real CC pipeline latency instead of guessing it.

Flight-controller settings the CC engineer should expect:

- `VISO_TYPE = 1`
- `EK3_SRC1_VELXY = 6` to enable body-frame XY fusion
- `EK3_SRC1_VELZ = 0` by default; set to `6` only if the radar Z velocity is validated
- `VISO_VEL_M_NSE`, `VISO_DELAY_MS`, and `VISO_POS_X/Y/Z` must match the actual sensor and mount
- `VISO_QUAL_MIN` can block low-confidence packets; if `quality` is populated on the CC, it should
  follow the normal ArduPilot convention (`-1` failed, `0` unknown, `1..100` usable confidence)

Minimal practical checklist for the CC implementation:

1. Publish body FRD velocity directly from radar processing.
2. Publish finite body angular rates for the same sample.
3. Mark the packet as twist-only with `estimator_type = UNKNOWN` and `pose_covariance[0] = NaN`.
4. Keep packet timing and `VISO_DELAY_MS` consistent with the actual measured latency.
5. Leave Z fusion disabled until logs show the radar Z channel is trustworthy.

---

## 5. Learned Lessons

### 5.0 Shared EKF timestamp semantics must be separated

`NavEKF3_core::writeBodyFrameOdom()` and `NavEKF3_core::writeBodyFrameVel()` currently use one
time value for two different jobs:

1. **acceptance / freshness time**
2. **delayed fusion-alignment time**

Those jobs are not mathematically identical.

Current pattern:

- gate on raw `timeStamp_ms`
- subtract `delay_ms`
- store the delay-subtracted value into both:
  - `bodyOdmMeasTime_ms`
  - `bodyOdmDataNew.time_ms`

That conflates:

- "when did I last accept a packet?"
- "what past time should this measurement be fused against?"

Plan B separates them:

- `bodyOdmMeasTime_ms = t_rx`
- `bodyOdmDataNew.time_ms = t_fuse`

where:

- `t_rx` = raw receive / measurement timestamp
- `t_fuse = max(t_rx - delay_ms, imuDataDelayed.time_ms)`

Guard rails:

- if `delay_ms >= t_rx`, reject the measurement instead of silently treating it as zero-latency
- only clamp **old but still meaningful** delayed measurements forward to the current IMU horizon

#### Why this does not break EKF math

This change does **not** modify:

- the state vector
- the body-velocity innovation
- the Jacobian
- the Kalman gain
- the frame transforms inside `FuseBodyVel()`

The actual fusion still uses the delayed measurement selected from `storedBodyOdm` at the EKF
fusion horizon.

What changes is only the bookkeeping:

- rate limiting and freshness should use **raw packet timing**
- buffer alignment should use **delayed fusion timing**

#### Why the clamp is correct

`storedBodyOdm.recall(bodyOdmDataDelayed, imuDataDelayed.time_ms)` recalls the newest measurement
older than the current delayed IMU fusion horizon.

If `t_rx - delay_ms` is older than the oldest usable IMU-backed horizon, then the estimator cannot
reconstruct that requested past exactly. In that case:

- clamp the measurement timestamp to `imuDataDelayed.time_ms`

This does not change the observation model. It only prevents storing measurements at unreachable
times.

#### Practical consequence

Separating these timestamps improves:

- rate limiting
- freshness tests using `bodyOdmMeasTime_ms`
- robustness when `delay_ms` is large

without changing the mathematical form of body-velocity fusion.

### 5.0.1 Shared body-odometry buffer is exclusive at runtime

`writeBodyFrameVel()` and `writeBodyFrameOdom()` still write into the same EKF3 body-odometry
buffer (`storedBodyOdm` plus shared freshness bookkeeping).

That means:

- body-twist `ODOMETRY` ingress and `VISION_POSITION_DELTA` ingress are not independent consumers
- they must not be streamed into the same EKF instance at the same time
- the runtime now rejects mixed fresh streams and emits a warning instead of silently interleaving
  them

The exclusivity rule is only about the active fresh source. A clean handover is still possible once
the previous source has gone stale.

### 5.1 EKF3 is not a body-frame state estimator

In EKF3, the navigation frame is still the local earth/world frame (NED), not body frame:
- `libraries/AP_NavEKF3/AP_NavEKF3_core.h:556-558`
- quaternion = rotation from local NED earth frame to body frame
- velocity = velocity of IMU in local NED earth frame
- position = position of IMU in local NED earth frame

Calling EKF3 an error-state EKF does **not** change that fact. The nominal state is mixed-frame.

For body-velocity fusion, EKF3 already does the correct mapping:
- `AP_NavEKF3_PosVelFusion.cpp:1428`
- `bodyVelPred = (prevTnb * stateStruct.velocity);`
- `AP_NavEKF3_PosVelFusion.cpp:1561`
- `innovBodyVel[0] = bodyVelPred.x - bodyOdmDataDelayed.vel.x;`

So:
- the measurement enters in body frame
- the innovation is formed in body frame
- the stored velocity state remains in local NED
- the body-frame residual is pushed into the mixed-frame state by `H_VEL` / `Kfusion`, not by a
  separate explicit residual rotation back into NED

### 5.2 Keep only the transforms we trust

The right question is not "can we remove every body<->world transform?"

The right question is:
- **which transforms stay internally consistent enough that they will not create wrong
  innovations or wrong control corrections, even when global yaw is bad?**

Transforms to remove:
- stock `ODOMETRY` body->NED conversion in `GCS_MAVLINK::handle_odometry()`
- accidental pose/quaternion leakage into the RIO twist-only route
- controller-local world->body reconstruction inside FlowHold

Transforms acceptable to keep for this project:
- EKF3's internal nav->body mapping inside `FuseBodyVel()`
- one canonical estimator-owned nav->body output mapping such as
  `AP_AHRS::get_velocity_body()`

### 5.3 Yaw drift alone is not the main failure mode

With IMU + body velocity and no absolute heading source, global yaw is weakly observable or
unobservable and can drift. That part is expected.

What does **not** automatically follow is "body-velocity hold will fail."

The key estimator relation is:

```text
v_b_pred = T_nb * v_n
innovation = v_b_pred - v_b_meas
```

If the floating nav frame is relabeled by a yaw rotation `S`, while the internal velocity and
attitude remain self-consistent:

```text
v_n'   = S * v_n
T_nb'  = T_nb * S^T
v_b_pred' = T_nb' * v_n' = T_nb * v_n
```

So the predicted body velocity is unchanged. That is why global yaw drift is mostly a
**gauge problem** for this route:
- world-frame interpretation degrades
- body-frame hold/braking does **not automatically** degrade

The real danger is **internal inconsistency**, for example:
- body velocity converted to NED using a different yaw reference
- controller reconstructing body velocity from a world-facing API
- delayed measurements compared against the wrong horizon
- sudden yaw resets
- accidental pose/yaw fusion on the twist-only route

### 5.4 A body-frame EKF state redesign is not the current route

Changing EKF3 velocity state to body frame is possible in principle, but it is not a local
change. It would touch:
- state prediction / mechanization
- position integration
- output observer
- GPS/extnav velocity fusion
- optical-flow prediction
- airspeed/wind handling
- world-frame outputs

So it is much closer to a custom EKF redesign than to a minimal RIO integration.

For the current objective, the better trade is:
- keep EKF3 internals as they are
- add clean body-frame ingress
- expose canonical body-frame velocity output
- keep the controller body-referenced

---

## 6. What Is Not Needed

| Item | Reason not needed |
|------|-------------------|
| GPS | No absolute position required; `AID_RELATIVE` is sufficient |
| Earth-frame velocity conversion on companion | Radar FRD should go directly into firmware via `writeBodyFrameVel()` |
| Changes to `FuseBodyVel()` | Existing fusion math is already the right core |
| New Copter mode | FlowHold remains the right pilot interface; only a contained body-axis refactor is needed |
| GCS/companion velocity commands | Pilot flies via RC; no MAVLink velocity commands required at runtime |
| GUIDED mode | Guided remains earth-frame oriented for this problem |
| New `VISO_VEL_FRAME` parameter | Prefer routing from explicit `ODOMETRY` frame metadata instead of introducing a new FC parameter |

---

## 7. Open Questions

1. **`angRate` policy:** `writeBodyFrameVel()` must populate angular rate explicitly. For the
   current testing route, the cleaner rule is to require finite `rollspeed/pitchspeed/yawspeed`
   and reject ambiguous messages.

2. **ODOMETRY routing split:** decide whether the body-frame branch lives directly in
   `GCS_MAVLINK::handle_odometry()` or in a helper beneath it. The design intent is the same:
   use ODOMETRY frame semantics, not a new parameter.

3. **DAL replay support:** the new ingress path should be replayable and log the real body-frame
   semantics rather than pretending it is delta-position odometry.

4. **Rate limiting:** `writeBodyFrameVel()` and `writeBodyFrameOdom()` share the same buffer, so
   they should share rate gating and should not be used simultaneously.

5. **Z velocity:** keep `EK3_SRC1_VELZ = 0` unless logs show radar Z velocity is good enough to
   help instead of hurt.

6. **Arming/filter-health interactions:** body odometry can start aiding without yaw alignment,
   but the exact target configuration still needs validation.

7. **FlowHold semantics:** confirm that the body-axis refactor preserves the intended brake/hold
   feel, since FlowHold is body-oriented velocity hold/braking rather than a literal body-velocity
   setpoint controller.

8. **Validation thresholds:** define what log evidence counts as success:
   - stable `flags.horiz_vel`
   - acceptable body-velocity innovation ratios
   - no accidental pose/yaw fusion on the twist-only route
   - stable braking after sustained yaw drift or yaw resets

---

## 8. Related Documents

| Document | Content |
|----------|---------|
| `RIO_VELOCITY_INTEGRATION.md` | Historical earth-frame EXTNAV path - superseded by this document |
| `OLD_flowhold_ekf_velocity_plan.md` | Earlier FlowHold adaptation details |
| `EKF3_DATA_FLOW.md` | EKF3 sensor data flow and fusion pipeline |
| `EKF3_CORE_MATH.md` | EKF3 state vector and Kalman filter math |
| `CONTROL_ARCHITECTURE.md` | Full cascaded control architecture |

---

## 9. Precise Implementation Plan

This section converts the architecture decision into an implementation-ready patch plan.

### 9.1 Scope of the patch

The patch is intentionally limited to:
- the RIO `ODOMETRY` ingress route
- EKF3 body-velocity ingestion
- FlowHold horizontal body-velocity control
- replay/logging needed to validate the route

It does **not** change:
- the existing `VISION_SPEED_ESTIMATE` / EXTNAV NED path
- the existing pose-based EXTNAV path
- Guided earth-frame velocity control
- EKF3 fusion core (`FuseBodyVel()`)

### 9.2 Ingress route to implement

Target route:

```text
RIO companion
  -> MAVLink ODOMETRY
  -> GCS_MAVLINK::handle_odometry()
  -> AP_VisualOdom::handle_body_frame_velocity_estimate()
  -> AP_VisualOdom_MAV::handle_body_frame_velocity_estimate()
  -> AP_AHRS::writeBodyFrameVel()
  -> NavEKF3::writeBodyFrameVel()
  -> NavEKF3_core::writeBodyFrameVel()
  -> storedBodyOdm
  -> SelectBodyOdomFusion()
  -> FuseBodyVel()
  -> FlowHold body-axis controller
```

### 9.3 File-by-file implementation

Recommended execution order for another engineer:

1. Implement the estimator ingress path first (`GCS_MAVLink` -> `AP_VisualOdom` -> `AP_AHRS` ->
   `NavEKF3`) while leaving `FuseBodyVel()` untouched.
2. Add DAL and visual-odometry logging before flight testing so the new route is replayable and
   frame semantics are visible in logs.
3. Add `AP_AHRS::get_velocity_body()` next, so the controller refactor can depend on one
   canonical output interface.
4. Refactor `mode_flowhold.cpp` only after the canonical output API exists.
5. Validate after each stage, in order: ingress logs -> EKF aiding -> FlowHold behavior ->
   yaw-drift behavior.

Implementation invariant:
- if a code change appears to require edits inside `FuseBodyVel()`, `H_VEL`, or the Kalman update
  equations, stop and re-check the design, because the intended route does **not** modify EKF3
  fusion math

#### A. `libraries/GCS_MAVLink/GCS_Common.cpp`

Modify `GCS_MAVLINK::handle_odometry()`:

Current behavior for `frame_id = LOCAL_FRD`, `child_frame_id = BODY_FRD`:
- forwards pose/quaternion into `handle_pose_estimate()`
- rotates `vx,vy,vz` from body FRD to NED using `q * vel`
- forwards NED velocity into `handle_vision_speed_estimate()`

New behavior for the RIO route:
- keep the existing frame gate
- decode:
  - `vel_bf = {m.vx, m.vy, m.vz}`
  - `ang_rate_bf = {m.rollspeed, m.pitchspeed, m.yawspeed}`
- call a new frontend method:
  - `visual_odom->handle_body_frame_velocity_estimate(...)`
- do **not** call `handle_pose_estimate()` on this route
- do **not** rotate the linear velocity into NED

Guardrails:
- require all three angular-rate fields to be finite before accepting the body-twist route
- if they are not finite, reject the message rather than silently falling back to pose fusion

#### B. `libraries/AP_VisualOdom/AP_VisualOdom.h`

Add a new public frontend method:

```cpp
void handle_body_frame_velocity_estimate(uint64_t remote_time_us,
                                         uint32_t time_ms,
                                         const Vector3f &vel,
                                         const Vector3f &ang_rate,
                                         uint8_t reset_counter,
                                         int8_t quality);
```

This should mirror the existing `handle_vision_speed_estimate()` structure and dispatch to the
active backend.

#### C. `libraries/AP_VisualOdom/AP_VisualOdom_Backend.h`

Add a new backend virtual with matching semantics.

#### D. `libraries/AP_VisualOdom/AP_VisualOdom_MAV.h/.cpp`

Implement the new method in the MAV backend.

Behavior:
- record `_quality`
- consume only when `_quality >= _frontend.get_quality_min()`
- use existing config sources for delay, noise, and offset
- call `AP::ahrs().writeBodyFrameVel(...)`
- update `_last_update_ms`

Important rule:
- reject NaN/Inf angular-rate components
- do not silently inject zeros unless a future branch explicitly chooses that fallback

#### E. `libraries/AP_AHRS/AP_AHRS.h/.cpp`

Add a new bridge method:

```cpp
void writeBodyFrameVel(const Vector3f &vel,
                       float err,
                       const Vector3f &angRate,
                       uint32_t timeStamp_ms,
                       uint16_t delay_ms,
                       const Vector3f &posOffset);
```

Behavior:
- EKF3-only bridge
- forwards to `NavEKF3::writeBodyFrameVel(...)`

#### F. `libraries/AP_NavEKF3/AP_NavEKF3.h/.cpp`

Add a matching frontend wrapper that:
- emits DAL/replay logging
- forwards to each EKF3 core

#### G. `libraries/AP_NavEKF3/AP_NavEKF3_core.h`

Declare a matching core method beside `writeBodyFrameOdom()`.

Do **not** change:
- `vel_odm_elements`
- `storedBodyOdm`
- `SelectBodyOdomFusion()`
- `FuseBodyVel()`

#### H. `libraries/AP_NavEKF3/AP_NavEKF3_Measurements.cpp`

Implement `NavEKF3_core::writeBodyFrameVel(...)`.

Behavior should mirror `writeBodyFrameOdom()` except for the delta conversion:
- reject NaN inputs
- reuse the same update-rate gate
- reject impossible `delay_ms >= timeStamp_ms` combinations
- subtract `delay_ms` from the timestamp
- write directly into `bodyOdmDataNew`
- push to `storedBodyOdm`

Mutual-exclusion requirement:
- `writeBodyFrameVel()` and `writeBodyFrameOdom()` share the same buffer
- only one body-odometry ingress mode should be active at runtime on this branch
- reject mixed fresh streams with a warning rather than silently interleaving them

#### I. `libraries/AP_DAL/AP_DAL.h/.cpp` and `libraries/AP_DAL/LogStructure.h`

Add DAL replay support for the new ingress API.

Add:
- new method `writeBodyFrameVel(...)`
- new log struct, e.g. `log_RBVH`
- replay handler that calls `ekf3.writeBodyFrameVel(...)`

Do **not** encode this as fake delta-position odometry.

#### J. `libraries/AP_VisualOdom/AP_VisualOdom_Logging.cpp` and `LogStructure.h`

Add a dedicated body-velocity visual-odometry log message for validation.

Recommended fields:
- timestamps
- body-frame `vx, vy, vz`
- body-frame `wx, wy, wz`
- configured velocity error
- reset counter
- quality

#### K. `libraries/AP_AHRS/AP_AHRS.h/.cpp`

Add a canonical estimator-output API:

```cpp
bool get_velocity_body(Vector3f &vec) const;
```

Behavior:
- obtain active estimator velocity output
- map it to body frame inside AHRS/EKF-facing code
- return body FRD velocity:
  - `.x = forward`
  - `.y = right`
  - `.z = down`

#### L. `ArduCopter/mode_flowhold.cpp`

`ModeFlowHold::flowhold_flow_to_angle()` has been refactored into a body-axis controller.

Keep:
- the current state machine
- vertical control
- pilot lean-angle summation
- braking law shape
- `sensor_flow` optical-flow axis labeling

Change:
- obtain body horizontal velocity through `AP_AHRS::get_velocity_body(...)`
- keep `sensor_flow` in body axes:
  - `sensor_flow.x = -v_right`
  - `sensor_flow.y = +v_forward`
- clamp and filter in body axes
- feed `sensor_flow` directly into `flow_pi_xy`
- interpret PI output as body-axis correction in the same axis labeling
- add the PI result directly to `bf_angles`
- remove the mode-local earth/body round trips

Semantic note:
- this changes the horizontal integrator from earth-referenced to body-referenced
- for this project, that is intentional and desirable
- it also removes the stale-integrator failure mode where accumulated earth-frame correction could
  rotate into the wrong body axis after yaw changes
- when `flags.horiz_vel` drops while FlowHold is active, the mode now discards horizontal
  controller memory and reacquires hold from a clean state once velocity validity returns

### 9.4 Why FlowHold remains the control mode

We are **not** adding a new Copter mode in this patch.

Reason:
- FlowHold already matches the desired pilot interaction model
- it already has the right no-GPS entry shape
- the horizontal part can be made body-native with a contained refactor
- this is much smaller than introducing a new mode or modifying Guided behavior

### 9.5 Configuration after the patch

Expected configuration:

```text
VISO_TYPE       = 1
VISO_DELAY_MS   = measured
VISO_VEL_M_NSE  = tune
VISO_POS_X/Y/Z  = measured

EK3_SRC1_POSXY  = 0
EK3_SRC1_VELXY  = 6
EK3_SRC1_POSZ   = 1
EK3_SRC1_VELZ   = 0 or 6
EK3_SRC1_YAW    = 0
```

Notes:
- no new Mission Planner parameter is required
- no pose source is required
- no external yaw source is required
- recommended default: `EK3_SRC1_VELZ = 0`

### 9.6 Validation plan

Validation should be done in this order:

1. **Ingress validation**
   - verify the RIO `ODOMETRY` route produces body-velocity records, not EXTNAV NED records
   - confirm the pose path is not consumed on this route
   - confirm non-finite angular-rate fields cause message rejection

2. **EKF validation**
   - confirm `readyToUseBodyOdm()` conditions are satisfied
   - confirm EKF enters `AID_RELATIVE`
   - confirm `flags.horiz_vel = 1`
   - confirm only one body-odometry ingress path is active

3. **FlowHold validation**
   - confirm mode entry succeeds with no GPS and no yaw source
   - confirm stick release produces stable body-oriented braking
   - confirm the new body-referenced PI does not introduce oscillation after sustained yaw motion
   - confirm a temporary loss of `flags.horiz_vel` clears horizontal controller memory and the mode
     degrades to manual AltHold-like XY behavior until aiding returns

4. **Yaw-drift validation**
   - intentionally allow yaw to drift / remove heading aiding
   - verify forward/right braking and hold remain body-consistent
   - verify only world-frame interpretation degrades

5. **Replay validation**
   - confirm DAL replay reproduces the new body-velocity ingress

### 9.7 Explicit non-goals of this patch

This patch does not attempt to:
- make EKF3 velocity state body-frame internally
- make world-frame navigation accurate without external yaw
- support earth-frame trajectory tracking
- preserve every existing `ODOMETRY` user for the RIO testing branch

That keeps the implementation aligned with the actual test objective.
