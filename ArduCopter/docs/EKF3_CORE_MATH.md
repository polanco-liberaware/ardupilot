# EKF3 Core Math Deep Dive (`AP_NavEKF3_core.cpp/.h`)

## Scope
This document explains the math and state-estimation mechanics specifically inside:
- `libraries/AP_NavEKF3/AP_NavEKF3_core.h`
- `libraries/AP_NavEKF3/AP_NavEKF3_core.cpp`

It focuses on the core model equations, covariance propagation, output-predictor math, and numerical conditioning strategy.

## 1) State Vector Definition (24 states)
The EKF core keeps one 24-state vector (`stateStruct` / `statesArray`) defined in `AP_NavEKF3_core.h`:

- `0..3`: quaternion `q` (NED -> body rotation)
- `4..6`: velocity `v_NED` (m/s)
- `7..9`: position `p_NED` (m)
- `10..12`: gyro delta-angle bias states
- `13..15`: accel delta-velocity bias states
- `16..18`: earth magnetic field in NED (Gauss)
- `19..21`: body magnetic field bias/disturbance (Gauss)
- `22..23`: horizontal wind states (m/s)

Implementation anchor:
- state layout in [`AP_NavEKF3_core.h:555`](/home/polanco/Liberaware/ardupilot/libraries/AP_NavEKF3/AP_NavEKF3_core.h:555)

Key nuance:
- Bias states are modeled in delta-angle / delta-velocity form and later converted to rate units when exposed (outside this file).

## 2) Core Predict/Fuse Loop Math Order
In each update cycle (`NavEKF3_core::UpdateFilter`):
1. Strapdown state propagation from delayed IMU horizon
2. Covariance prediction (`P^-`)
3. Measurement fusion modules (mag, pos/vel, etc.)
4. Output-state propagation from fusion horizon to real-time horizon

Execution anchor:
- [`AP_NavEKF3_core.cpp:627`](/home/polanco/Liberaware/ardupilot/libraries/AP_NavEKF3/AP_NavEKF3_core.cpp:627)

## 3) Strapdown Kinematics (Hand-Written Propagation)
`UpdateStrapdownEquationsNED()` is the core nonlinear process model:

### 3.1 Attitude propagation
Quaternion update uses corrected delta-angle minus Earth-rate compensation:

`q_k = q_{k-1} ⊗ δq(Δθ_corr - (T_nb * ω_earth_NED)Δt)`

Anchors:
- [`AP_NavEKF3_core.cpp:751`](/home/polanco/Liberaware/ardupilot/libraries/AP_NavEKF3/AP_NavEKF3_core.cpp:751)
- normalization: [`AP_NavEKF3_core.cpp:753`](/home/polanco/Liberaware/ardupilot/libraries/AP_NavEKF3/AP_NavEKF3_core.cpp:753)

### 3.2 Velocity propagation
Body delta-velocity is rotated to NED, gravity is added on Z:

`Δv_NED = T_bn_prev^T * Δv_body_corr + [0, 0, gΔt]`

Then:
`v_k = v_{k-1} + Δv_NED`

Anchors:
- [`AP_NavEKF3_core.cpp:760`](/home/polanco/Liberaware/ardupilot/libraries/AP_NavEKF3/AP_NavEKF3_core.cpp:760)
- [`AP_NavEKF3_core.cpp:789`](/home/polanco/Liberaware/ardupilot/libraries/AP_NavEKF3/AP_NavEKF3_core.cpp:789)

### 3.3 Position propagation
Trapezoidal integration:

`p_k = p_{k-1} + 0.5*(v_k + v_{k-1})Δt`

Anchor:
- [`AP_NavEKF3_core.cpp:792`](/home/polanco/Liberaware/ardupilot/libraries/AP_NavEKF3/AP_NavEKF3_core.cpp:792)

### 3.4 Aiding-off acceleration limiting
When no aiding (`AID_NONE`), horizontal acceleration magnitude is clipped to reduce attitude drift from aggressive inertial transients.

Anchor:
- [`AP_NavEKF3_core.cpp:779`](/home/polanco/Liberaware/ardupilot/libraries/AP_NavEKF3/AP_NavEKF3_core.cpp:779)

## 4) Covariance Initialization
`CovarianceInit()` initializes `P` diagonals from configured measurement/process assumptions:

- velocity/position from GPS/baro noises
- gyro/accel bias initial uncertainty
- mag and wind initial covariance

Anchor:
- [`AP_NavEKF3_core.cpp:575`](/home/polanco/Liberaware/ardupilot/libraries/AP_NavEKF3/AP_NavEKF3_core.cpp:575)

Important detail:
- quaternion covariance initialization is performed via `CovariancePrediction(rot_vec_var)` with an initial rotation-variance vector (0.1 rad)^2.

## 5) Covariance Prediction (`P^-`) and Process Noise
`CovariancePrediction()` combines:
- discrete-time linearized propagation from current state and IMU deltas
- additive process noise on active states
- protective logic for inactive/unobservable states

Anchors:
- function start [`AP_NavEKF3_core.cpp:1010`](/home/polanco/Liberaware/ardupilot/libraries/AP_NavEKF3/AP_NavEKF3_core.cpp:1010)
- generated-equation note [`AP_NavEKF3_core.cpp:1004`](/home/polanco/Liberaware/ardupilot/libraries/AP_NavEKF3/AP_NavEKF3_core.cpp:1004)

### 5.1 Time-step conditioning
Covariance `dt` is constrained to prevent jitter-induced conditioning issues:

`dt = constrain(0.5*(Δt_ang + Δt_vel), 0.5*dtEkfAvg, 2*dtEkfAvg)`

Anchor:
- [`AP_NavEKF3_core.cpp:1038`](/home/polanco/Liberaware/ardupilot/libraries/AP_NavEKF3/AP_NavEKF3_core.cpp:1038)

### 5.2 Process noise structure
Process noise vector is built for state groups:
- gyro bias process noise
- accel bias process noise
- mag earth/body process noise (when mag states active)
- wind process noise (with climb/descent scaling via filtered height rate)

Anchors:
- start of noise construction: [`AP_NavEKF3_core.cpp:1047`](/home/polanco/Liberaware/ardupilot/libraries/AP_NavEKF3/AP_NavEKF3_core.cpp:1047)
- wind scaling behavior: [`AP_NavEKF3_core.cpp:1113`](/home/polanco/Liberaware/ardupilot/libraries/AP_NavEKF3/AP_NavEKF3_core.cpp:1113)

### 5.3 Generated symbolic propagation
The huge `PS*` intermediate block is generated symbolic algebra (SymPy) for efficient covariance propagation.
- This is effectively an optimized unrolled form of:
  `P^- = F P F^T + Q`
- only lower-triangle is computed then mirrored due to symmetry.

Anchor:
- generated math begins after [`AP_NavEKF3_core.cpp:1188`](/home/polanco/Liberaware/ardupilot/libraries/AP_NavEKF3/AP_NavEKF3_core.cpp:1188)

### 5.4 Position-variance growth limiter
If horizontal position variance becomes very large (`P[7][7]+P[8][8] > 1e4`), growth is frozen for those states to avoid ill-conditioning.

Anchor:
- [`AP_NavEKF3_core.cpp:1764`](/home/polanco/Liberaware/ardupilot/libraries/AP_NavEKF3/AP_NavEKF3_core.cpp:1764)

## 6) Numerical Conditioning and Safety Constraints
After prediction, `ConstrainVariances()` and `ConstrainStates()` enforce boundedness.

### 6.1 Covariance constraints
`ConstrainVariances()` enforces min/max on each state class:
- quaternion variances
- velocity/position variances
- bias variances
- magnetic/wind variances depending on inhibit flags

Anchor:
- [`AP_NavEKF3_core.cpp:1874`](/home/polanco/Liberaware/ardupilot/libraries/AP_NavEKF3/AP_NavEKF3_core.cpp:1874)

Important mechanisms:
- Vertical-velocity variance collapse detector with counter and reset (`vertVelVarClipCounter`)
- Full accel-bias covariance reset if any accel-bias axis variance becomes numerically unsafe
- Forced zero rows/cols for inactive state groups

### 6.2 State constraints
`ConstrainStates()` clamps physical ranges:
- quaternion bounds
- velocity/position bounds
- gyro/accel bias bounds
- magnetic field and wind bounds
- terrain-state floor relative to current vehicle height + `rngOnGnd`

Anchor:
- [`AP_NavEKF3_core.cpp:1997`](/home/polanco/Liberaware/ardupilot/libraries/AP_NavEKF3/AP_NavEKF3_core.cpp:1997)

## 7) Output Predictor Math (Delayed-Horizon Compensation)
The EKF internal state is estimated at delayed fusion horizon. `calcOutputStates()` creates real-time outputs with smooth corrections.

Anchor:
- [`AP_NavEKF3_core.cpp:821`](/home/polanco/Liberaware/ardupilot/libraries/AP_NavEKF3/AP_NavEKF3_core.cpp:821)

### 7.1 Re-propagation at output horizon
Uses newest corrected IMU deltas to propagate `outputDataNew`.

### 7.2 Third-order vertical complementary channel
The vertical channel uses a 3rd-order complementary structure (Widnall/Sinha reference):
- integrates position error into acceleration/velocity/position states
- parameterized by `CompFiltOmega = 2π * hrt_filt_freq`

Anchors:
- filter block [`AP_NavEKF3_core.cpp:855`](/home/polanco/Liberaware/ardupilot/libraries/AP_NavEKF3/AP_NavEKF3_core.cpp:855)

### 7.3 Attitude output tracking
Quaternion error between delayed output and delayed EKF is converted via small-angle approximation:

`δθ_err ≈ 2 * sign(q_err.w) * q_err.xyz`

Then correction:
`delAngCorrection = δθ_err * (0.5/timeDelay) * dtIMUavg`

Anchors:
- quaternion error + small-angle conversion [`AP_NavEKF3_core.cpp:917`](/home/polanco/Liberaware/ardupilot/libraries/AP_NavEKF3/AP_NavEKF3_core.cpp:917)
- correction gain [`AP_NavEKF3_core.cpp:934`](/home/polanco/Liberaware/ardupilot/libraries/AP_NavEKF3/AP_NavEKF3_core.cpp:934)

### 7.4 Position/velocity output tracking
PI-style correction is applied to full output history buffer:
- `posCorrection = Kp*posErr + Ki*∫posErr`
- `velCorrection = Kp*velErr + Ki*∫velErr`

This avoids step-like output jumps and avoids extra delay in correction loop.

Anchors:
- gains and PI terms [`AP_NavEKF3_core.cpp:959`](/home/polanco/Liberaware/ardupilot/libraries/AP_NavEKF3/AP_NavEKF3_core.cpp:959)
- correction over history [`AP_NavEKF3_core.cpp:983`](/home/polanco/Liberaware/ardupilot/libraries/AP_NavEKF3/AP_NavEKF3_core.cpp:983)

## 8) Attitude Error Variance Metric (`tiltErrorVariance`)
`calcTiltErrorVariance()` analytically propagates quaternion covariance to tilt-angle variance metric.

Anchor:
- [`AP_NavEKF3_core.cpp:2118`](/home/polanco/Liberaware/ardupilot/libraries/AP_NavEKF3/AP_NavEKF3_core.cpp:2118)

Notes:
- Equations are generated by derivation tooling (`generate_2.py`), diagonal terms only.
- Result is clipped to `(30 deg)^2`.
- SITL-only `verifyTiltErrorVariance()` computes finite-difference alternative for consistency checks.

Anchors:
- verification function [`AP_NavEKF3_core.cpp:2179`](/home/polanco/Liberaware/ardupilot/libraries/AP_NavEKF3/AP_NavEKF3_core.cpp:2179)

## 9) Earth-Rate and Frame Math
Earth spin in NED:

`ω_N = earthRate*cos(lat)`
`ω_E = 0`
`ω_D = -earthRate*sin(lat)`

Anchor:
- [`AP_NavEKF3_core.cpp:2030`](/home/polanco/Liberaware/ardupilot/libraries/AP_NavEKF3/AP_NavEKF3_core.cpp:2030)

This term enters quaternion propagation to remove systematic Earth-rotation drift.

## 10) Origin Translation Math (`moveEKFOrigin`)
`moveEKFOrigin()` periodically relocates internal origin to reduce map-projection distortion.
- it offsets origin by current state NE displacement,
- then adds the inverse displacement to all stored position states/history so physical position is unchanged.

Anchor:
- [`AP_NavEKF3_core.cpp:2228`](/home/polanco/Liberaware/ardupilot/libraries/AP_NavEKF3/AP_NavEKF3_core.cpp:2228)

Mathematically:
- coordinate frame changes,
- represented world pose remains invariant.

## 11) Why This Core Is Split Between Handwritten and Generated Math
- Handwritten sections:
  - state propagation logic,
  - observer/output compensation,
  - robustness/guardrails and mode-dependent noise handling.
- Generated section:
  - large Jacobian/covariance expansion for runtime efficiency and deterministic codegen from symbolic derivation.

This keeps runtime costs low while preserving maintainability at model level via derivation scripts.

## 12) Practical Reading Order for Future Work
For modifying EKF3-core math safely, read in this order:
1. State definition and comments in `AP_NavEKF3_core.h`.
2. `UpdateFilter()` call sequence in `AP_NavEKF3_core.cpp`.
3. `UpdateStrapdownEquationsNED()`.
4. `CovariancePrediction()` pre/post generated block (noise and conditioning parts).
5. `ConstrainVariances()` + `ConstrainStates()`.
6. `calcOutputStates()` for controller-facing behavior.
7. `calcTiltErrorVariance()` for health/arming sensitivity impact.

## 13) Related Non-Core Files (for context only)
- `AP_NavEKF3_Measurements.cpp`: provides delayed sensor data that drives this core math.
- `AP_NavEKF3_PosVelFusion.cpp`, `MagFusion.cpp`, `OptFlowFusion.cpp`, `AirDataFusion.cpp`: correction/update steps that operate after this core prediction.

