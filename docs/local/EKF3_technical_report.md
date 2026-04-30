# ArduPilot EKF3 technical report

This document is a paper-to-code technical report for the current `AP_NavEKF3` implementation in this repository, with emphasis on the 24-state inertial EKF, the delayed-horizon architecture, the absolute-aiding measurement paths, and the current ExternalNav/RIO integration.

It is intentionally implementation-oriented: every mathematical statement below is tied to the code that executes it. The goal is that a reader can reconstruct the implemented filter, not just the idealized textbook EKF.

## 1. Scope

The main EKF3 backend is implemented across:

- `libraries/AP_NavEKF3/AP_NavEKF3_core.h`
- `libraries/AP_NavEKF3/AP_NavEKF3_core.cpp`
- `libraries/AP_NavEKF3/AP_NavEKF3_Measurements.cpp`
- `libraries/AP_NavEKF3/AP_NavEKF3_PosVelFusion.cpp`
- `libraries/AP_NavEKF3/AP_NavEKF3_MagFusion.cpp`
- `libraries/AP_NavEKF3/AP_NavEKF3_Control.cpp`
- `libraries/AP_NavEKF3/AP_NavEKF3.cpp`

The implementation still follows the symbolic-derivation lineage noted in the code header:

- `AP_NavEKF3_core.h`: “24 state EKF based on the derivation in PX4/ecl”
- `AP_NavEKF3_core.cpp::CovariancePrediction()`: auto-generated covariance equations
- `AP_NavEKF3_MagFusion.cpp::FuseMagnetometer()` and `fuseEulerYaw()`: auto-generated measurement equations

This report covers:

1. conventions and notation
2. architecture and data flow
3. nominal state, active-state truncation, auxiliary states, and the absence of clone states
4. propagation and covariance prediction
5. velocity, position, height, yaw, magnetometer, and RIO/ExternalNav measurement fusion
6. timing, buffering, and delay handling
7. configuration and parameter meaning
8. theory-vs-implementation differences

Auxiliary measurement families such as optical flow, wheel/body odometry, airspeed, and range beacons follow the same update pattern and are referenced where they affect the core architecture, but the deepest derivation below focuses on the parts that are in the critical path for the current project.

## 2. Conventions

### 2.1 Coordinate frames

The EKF uses a local earth frame `n` and a vehicle body frame `b`.

- `n`: local **NED** frame, with axes North, East, Down
- `b`: vehicle **body FRD** frame, with axes Forward, Right, Down

All EKF position and velocity states are expressed in the local NED frame:

- `stateStruct.velocity` in `AP_NavEKF3_core.h` indices `4..6`
- `stateStruct.position` in indices `7..9`

Magnetic states split into:

- `earth_magfield` in `n`
- `body_magfield` in `b`

Wind states are 2-D horizontal NED:

- `wind_vel = [w_N, w_E]^T`

### 2.2 Sign conventions

The implementation is NED, so:

- positive `x` is North
- positive `y` is East
- positive `z` is Down

Therefore:

- altitude above origin is `-stateStruct.position.z`
- height measurements are stored as `hgtMea > 0` upward, then mapped to the EKF position observation by `velPosObs[5] = -hgtMea`

This is visible in `AP_NavEKF3_PosVelFusion.cpp::selectHeightForFusion()`.

### 2.3 Quaternion convention

The quaternion is scalar-first Hamilton form:

$$
q = [q_0, q_1, q_2, q_3]^T
$$

stored in:

- `stateStruct.quat` at state indices `0..3`

Operationally, the EKF uses the quaternion to build the body-to-navigation DCM used in propagation:

- `outputDataNew.quat.rotation_matrix(Tbn_temp)` then `delVelNav = Tbn_temp * delVelBody` in `AP_NavEKF3_core.cpp::calcOutputStates()`
- `stateStruct.quat.inverse().rotation_matrix(prevTnb)` then `delVelNav = prevTnb.mul_transpose(delVelCorrected)` in `UpdateStrapdownEquationsNED()`

So the physically useful convention for re-derivation is:

- `T_bn(q)` maps body-frame vectors into NED:
  $$
  v^n = T_{bn}(q)\,v^b
  $$
- `T_nb = T_{bn}^T`

Some comments in the code describe the quaternion as “navigation to body”; when re-deriving the implemented math, follow the actual matrix usage at the call sites above.

### 2.4 Rotation and innovation sign

The EKF uses the innovation convention:

$$
\nu = \hat{z} - z
$$

not `z - \hat{z}`.

Examples:

- `innovVelPos[i] = stateStruct.velocity[i] - velPosObs[i]`
- `innovMag = MagPred - magDataDelayed.mag`
- `innovYaw = wrap_PI(yawAngPredicted - yawAngMeasured)`

So the state correction is applied as:

$$
x^+ = x^- - K \nu
$$

which matches the code in the scalar and grouped update paths.

### 2.5 Error-state definition

There is **no separate stored minimal error-state vector** in EKF3. The implementation stores a 24-element nominal state directly and updates those same 24 elements with Kalman corrections.

Still, the implementation is easiest to interpret as a hybrid direct-state EKF with small-angle reasoning for attitude:

- nominal quaternion is stored explicitly
- quaternion covariance is stored as a `4x4` block in `P`
- some initialization steps reason in terms of a `3x1` rotation-vector uncertainty and inject it into the quaternion block

So the exact stored correction vector is:

$$
\delta x \in \mathbb{R}^{24}
$$

with the same indexing as the nominal state vector. There is no clone state, no MSCKF-style camera clone stack, and no estimated extrinsic state in the main filter.

## 3. Architecture and ownership

### 3.1 Module ownership

| Step | Owner | Main functions |
| --- | --- | --- |
| parameter storage and source selection | `NavEKF3` frontend + `AP_NavEKF_Source` | `AP_NavEKF3.cpp`, `AP_NavEKF_Source.cpp` |
| per-lane EKF backend | `NavEKF3_core` | `AP_NavEKF3_core.*` |
| IMU ingestion and observation buffering | `AP_NavEKF3_Measurements.cpp` | `readIMUData()`, `readGpsData()`, `writeRioNavData()` |
| state propagation | `AP_NavEKF3_core.cpp` | `UpdateStrapdownEquationsNED()` |
| covariance prediction | `AP_NavEKF3_core.cpp` | `CovariancePrediction()` |
| source arbitration / mode control | `AP_NavEKF3_Control.cpp` | `controlFilterModes()`, `setAidingMode()` |
| position/velocity/height fusion | `AP_NavEKF3_PosVelFusion.cpp` | `SelectVelPosFusion()`, `FuseVelPosNED()` |
| yaw and magnetic fusion | `AP_NavEKF3_MagFusion.cpp` | `SelectMagFusion()`, `fuseEulerYaw()`, `FuseMagnetometer()` |
| delayed-to-current output propagation | `AP_NavEKF3_core.cpp` | `calcOutputStates()` |

### 3.2 Main cycle

The main backend cycle is `NavEKF3_core::UpdateFilter(bool predict)` in `AP_NavEKF3_core.cpp`.

The update order is:

1. mode and source control
2. IMU read / downsample / delayed sample extraction
3. nominal-state propagation
4. covariance prediction
5. yaw / magnetometer update
6. velocity / position / height update
7. auxiliary updates (GSF yaw correction, range beacon, optical flow, body odometry, airspeed, drag)
8. status update
9. output observer propagation from delayed horizon to current horizon

That ordering matters. In particular:

- prediction is always performed on the **delayed horizon**
- most measurements are fused against `imuDataDelayed.time_ms`
- the output used by the rest of the vehicle is then pushed forward to “now” by a separate observer

## 4. State definition

### 4.1 Main state vector

The exact 24-state nominal vector is defined in `AP_NavEKF3_core.h`:

| Indices | Dimension | Symbol | Meaning | Frame | Units |
| --- | --- | --- | --- | --- | --- |
| `0..3` | 4 | `q` | attitude quaternion | body↔NED rotation as used operationally by propagation | unitless |
| `4..6` | 3 | `v^n` | IMU velocity | NED | m/s |
| `7..9` | 3 | `p^n` | IMU position from EKF origin | NED | m |
| `10..12` | 3 | `b_g` | delta-angle IMU bias state | body | rad over one EKF step |
| `13..15` | 3 | `b_a` | delta-velocity IMU bias state | body | m/s over one EKF step |
| `16..18` | 3 | `m_e^n` | earth magnetic field | NED | Gauss |
| `19..21` | 3 | `m_b^b` | body-fixed magnetic field / bias-like term | body | Gauss |
| `22..23` | 2 | `w^n_{NE}` | horizontal wind | NED horizontal | m/s |

Important detail: the bias states are **delta-angle** and **delta-velocity** biases, not pure rate biases. Output APIs divide them by `dtEkfAvg` to report rate units:

- `AP_NavEKF3_Outputs.cpp::getGyroBias()`
- `AP_NavEKF3_Outputs.cpp::getAccelBias()`

This is a real implementation difference from many papers that define `b_g` in rad/s and `b_a` in m/s² directly.

### 4.2 Active-state truncation

The covariance and update loops do not always operate over all 24 states. `stateIndexLim` is dynamically reduced in `updateStateIndexLim()`:

- `9`: no bias, no mag, no wind
- `12`: gyro bias active
- `15`: gyro + accel bias active
- `21`: mag states active
- `23`: full 24-state filter

So the implementation is one filter with runtime-truncated active dimension, not several different filters.

### 4.3 Clone states

There are **no clone states** in the main EKF3 backend.

The code does not implement:

- camera/pose clones
- delayed cloned poses
- sliding-window MSCKF landmarks

All delayed fusion is handled by ring buffers plus a delayed-horizon observer, not by state augmentation.

### 4.4 Extrinsic states

There are **no estimated sensor extrinsic states** in the main EKF state.

Sensor offsets are handled as deterministic corrections from configuration or DAL-provided sensor geometry:

- GPS antenna correction: `CorrectGPSForAntennaOffset()`
- ExternalNav sensor offset: `CorrectExtNavForSensorOffset()`
- ExternalNav velocity offset: `CorrectExtNavVelForSensorOffset()`

So extrinsics are applied before fusion rather than estimated online.

### 4.5 Auxiliary states outside the main EKF

These are not in `statesArray`, but they materially affect behavior:

- `terrainState`: scalar terrain offset state
- `Popt`: optical-flow auxiliary covariance
- `outputDataNew`, `outputDataDelayed`, `storedOutput`: delayed/current output observer states
- GSF yaw estimator state bank

These should be treated as coupled auxiliary estimators around the main 24-state EKF.

## 5. Timing, delays, buffering, and fusion horizon

### 5.1 Why the filter is delayed

EKF3 fuses asynchronous sensors with nontrivial delays. Rather than force every sensor to “look current”, the filter estimates at a delayed fusion horizon and then propagates outputs forward.

The two key horizons are:

- **delayed / fusion horizon**: `imuDataDelayed`
- **current / output horizon**: `imuDataNew`

This is an important architectural choice. EKF3 is **not** implemented as a current-time EKF augmented with delayed clone states of the nominal state and covariance. In an augmented-state design, one would keep a live state $x_k$, append one or more delayed clones $x_{k-d}$, preserve the cross-covariances between them, and use those cross-covariances to let a delayed measurement update the current estimate directly.

EKF3 does something different:

1. the **main stochastic EKF state** itself lives at the delayed fusion horizon,
2. delayed measurements are fused **directly against that delayed state**,
3. a separate **output predictor / complementary observer** advances the fused solution to current time for control and logging outputs.

So the delayed handling is not "current state plus historical clones"; it is "historical main EKF plus a separate current-time output predictor". This keeps the main covariance dimension fixed at the 24-state EKF, regardless of sensor delay.

### 5.2 Buffer sizing

`setup_core()` computes:

1. the maximum configured sensor delay
2. an IMU buffer length large enough to cover that delay
3. observation buffers large enough to hold measurements until the fusion horizon catches up

Important inputs:

- GPS lag from the GPS driver
- `EK3_HGT_DELAY`
- magnetometer delay
- airspeed delay
- optical flow delay
- visual-odometry / ExternalNav delay

For ExternalNav buffers the code uses `extNavIntervalMin_ms = 20 ms`.

### 5.3 IMU downsampling

`readIMUData()` accumulates raw IMU deltas until roughly `EKF_TARGET_DT = 12 ms` is reached.

The accumulation is not naive:

- delta-angle accumulation is performed with quaternion composition to reduce coning error
- delta-velocity accumulation is rotated into the body frame at the start of accumulation

Once the accumulated interval reaches the target window:

1. the accumulated IMU sample is pushed into `storedIMU`
2. the oldest buffered IMU sample becomes `imuDataDelayed`
3. that delayed IMU sample is bias-corrected and used for prediction

This is the key point: the EKF prediction is **not** performed from the freshest IMU increment. The prediction uses the IMU increment that sits at the tail of the IMU FIFO, i.e. the increment corresponding to the chosen fusion horizon. In code, `storedIMU.push_youngest_element(...)` appends the newest downsampled sample, then `imuDataDelayed = storedIMU.get_oldest_element()` selects the oldest retained sample for the actual EKF propagation step.

Conceptually, if the IMU buffer spans the configured sensor delays, then the oldest retained sample is the one whose timestamp is aligned with the measurement horizon that the delayed observations can realistically support. The EKF therefore walks forward in time, but only along that delayed horizon.

The current-time IMU sample is still kept as `imuDataNew`, but it is used later by `calcOutputStates()` for the output predictor, not as the stochastic propagation point for the covariance-bearing EKF state.

### 5.4 Sensor timestamps

The general timestamp rule is:

$$
t_{\text{fuse}} = t_{\text{meas}} - t_{\text{delay}} - \frac{\Delta t_{\text{ekf}}}{2}
$$

followed by clipping to the valid IMU-buffer interval.

Examples:

- GPS: driver lag + half-step correction
- baro: fixed `EK3_HGT_DELAY` + half-step correction
- legacy ExternalNav: caller-provided `delay_ms` + half-step correction

Current RIO path difference:

- `writeRioNavData()` takes `measurement_time_ms` already in FC boot-time
- it applies only the half-step alignment:
  $$
  t_{\text{fuse,RIO}} = t_{\text{meas,fc}} - \frac{\Delta t_{\text{ekf}}}{2}
  $$

This is the important implementation difference between legacy ExternalNav and the new RIO ingress.

After this timestamp adjustment, measurements are not fused immediately. They are written into per-sensor observation ring buffers such as `storedGPS`, `storedBaro`, `storedExtNav`, `storedExtNavVel`, and `storedExtNavYawAng`. Each buffer entry carries the measurement payload plus the delayed-horizon timestamp `time_ms`.

At fusion time, the selector for that sensor calls the common buffer API

$$
\texttt{recall}(\text{measurement},\; t = \texttt{imuDataDelayed.time\_ms})
$$

which searches for the **newest sample that is older than or equal to** the current delayed-horizon time, provided it is not too old. In `ekf_ring_buffer::recall()`, this means:

1. walk forward from the oldest stored measurement,
2. keep the newest sample satisfying $0 \le t - t_i < 100 \text{ ms}$,
3. discard measurements older than the horizon that have been passed over,
4. stop once the oldest queued measurement is still newer than the current horizon.

This gives the precise delayed-fusion behavior:

1. the companion or sensor backend timestamps the measurement,
2. the write function converts that timestamp into the EKF fusion-time domain,
3. the sample waits in its observation queue,
4. when `imuDataDelayed.time_ms` catches up, the measurement is recalled and fused against the delayed EKF state.

So the horizon is driven by the IMU FIFO, while each observation queue waits until the delayed EKF time reaches the measurement-valid time. The filter is therefore synchronized by time-domain alignment of buffered streams, not by keeping a cloned covariance history of past states.

### 5.5 Output observer

After fusion, `calcOutputStates()` propagates the delayed estimate to the current horizon.

This observer does two jobs:

1. remove the control-loop lag that would come from exposing the delayed EKF state directly
2. low-pass abrupt correction steps produced by delayed measurement fusion

For height and vertical speed it uses a third-order complementary structure based on Widnall-Sinha.

The output observer is best understood as a **bias-corrected strapdown INS running at current time**, with a feedback loop that forces it to track the delayed EKF solution. It is not a second Kalman filter: it has no covariance, no stochastic update, and no measurement Jacobians. Its job is purely to produce smooth current-time states from the delayed EKF estimate.

The sequence is:

1. `UpdateFilter()` predicts and fuses the **delayed** EKF state using `imuDataDelayed`,
2. `calcOutputStates()` propagates a separate output state using the **current** IMU increment `imuDataNew`,
3. that propagated current-time state is stored in `storedOutput[storedIMU.get_youngest_index()]`,
4. the delayed-horizon counterpart `outputDataDelayed = storedOutput[storedIMU.get_oldest_index()]` is recalled,
5. the code compares `outputDataDelayed` against the EKF delayed state `stateStruct`,
6. tracking corrections are computed from that difference,
7. those corrections are fed back into the output observer so the newest output remains current-time but stays close to the delayed EKF solution.

So there are two coupled loops inside `calcOutputStates()`:

1. a **prediction loop** from current IMU data,
2. a **tracking-correction loop** from the delayed EKF/output mismatch.

#### 5.5.1 Output-state prediction

Let the output observer state be

$$
x_{\text{out}} = \left(q_{\text{out}},\; v_{\text{out}},\; p_{\text{out}}\right)
$$

plus auxiliary vertical-filter states in `vertCompFiltState`.

First, the current IMU sample is bias-corrected:

$$
\Delta \theta_{\text{new},c} = \Delta \theta_{\text{new}} - \hat b_g,
\qquad
\Delta v_{\text{new},c} = \Delta v_{\text{new}} - \hat b_a
$$

using the same `correctDeltaAngle()` and `correctDeltaVelocity()` helpers as the main EKF.

Then the observer applies the previously computed attitude-tracking correction `delAngCorrection`:

$$
\Delta \theta_{\text{out}} = \Delta \theta_{\text{new},c} + \Delta \theta_{\text{corr}}
$$

and propagates the current-time attitude:

$$
q_{\text{out}}^{+} = q_{\text{out}} \otimes \delta q(\Delta \theta_{\text{out}})
$$

which corresponds to

- `deltaQuat.from_axis_angle(delAng)`
- `outputDataNew.quat *= deltaQuat`
- `outputDataNew.quat.normalize()`

Using the resulting rotation matrix `Tbn_temp`, the observer propagates velocity as

$$
\Delta v_{\text{nav}} = T_{bn}(q_{\text{out}}^{+})\,\Delta v_{\text{new},c}
$$

$$
v_{\text{out}}^{+} = v_{\text{out}} + \Delta v_{\text{nav}} + \begin{bmatrix}0\\0\\g\,\Delta t\end{bmatrix}
$$

and position by trapezoidal integration:

$$
p_{\text{out}}^{+} = p_{\text{out}} + \frac{1}{2}\left(v_{\text{out}}^{+} + v_{\text{out}}^{-}\right)\Delta t
$$

where $v_{\text{out}}^{-}$ is the pre-update velocity stored in `lastVelocity`.

This is a standard deterministic inertial propagation step. If there were no correction loop, `outputDataNew` would simply be a current-time INS propagated from IMU data.

#### 5.5.2 Vertical complementary channel

The observer also runs a separate third-order complementary structure for the vertical channel:

$$
e_z = p_{\text{out},z} - p_{\text{vert}}
$$

and integrates internal states `vertCompFiltState.acc`, `vertCompFiltState.vel`, and `vertCompFiltState.pos` with gains determined by

$$
\omega = 2\pi f_{\text{hrt}}
$$

where `f_hrt` is `frontend->_hrt_filt_freq` after limiting.

This vertical filter is not the main EKF covariance update either; it is a deterministic shaping filter used by the output observer to smooth height and vertical-rate behavior at the current horizon.

#### 5.5.3 Delayed-horizon comparison

After the current-time propagation step, the code writes the newest output state into the output history buffer:

$$
\texttt{storedOutput[youngest]} \leftarrow x_{\text{out}}(t_{\text{now}})
$$

Then it retrieves the output state aligned with the delayed fusion horizon:

$$
x_{\text{out,delayed}} = \texttt{storedOutput[oldest]}
$$

where `oldest` is `storedIMU.get_oldest_index()`, i.e. the same delayed index that defines `imuDataDelayed`.

This gives two states at the **same delayed time**:

1. `stateStruct`: the delayed EKF state after stochastic fusion,
2. `outputDataDelayed`: the delayed output-observer state obtained by replaying current-time propagation history.

Only now does the observer form tracking errors. Because both are at the same delayed timestamp, this comparison is time-consistent.

#### 5.5.4 Attitude tracking correction

For attitude, the correction is formed from the quaternion difference

$$
q_{\text{err}} = q_{\text{EKF,delayed}} \oslash q_{\text{out,delayed}}
$$

which in code is `quatErr = stateStruct.quat / outputDataDelayed.quat`.

After normalization, the code converts this quaternion error to a small-angle vector approximation:

$$
\delta \theta_{\text{err}} \approx 2\,\mathrm{sgn}(q_{\text{err},0})
\begin{bmatrix}
q_{\text{err},1}\\
q_{\text{err},2}\\
q_{\text{err},3}
\end{bmatrix}
$$

implemented by the `scaler = \pm 2` logic and the assignments to `deltaAngErr`.

The correction gain is then chosen from the delay between current and fusion horizons:

$$
t_d = t_{\text{now}} - t_{\text{fusion}}
$$

$$
k_{\theta} = \frac{0.5}{\max(t_d,\; \Delta t_{\text{IMU}})}
$$

so that larger horizon separation produces gentler correction. The actual applied correction is

$$
\Delta \theta_{\text{corr}} = \delta \theta_{\text{err}}\, k_{\theta}\, \Delta t_{\text{IMU}}
$$

which becomes `delAngCorrection`.

This is important: the attitude mismatch is **not** applied by retrospectively rewriting the whole quaternion history. Instead, it is injected as an extra delta-angle term in the next current-time propagation step. So attitude correction is forward-fed into the observer dynamics.

#### 5.5.5 Velocity and position tracking correction

For velocity and position, the code forms delayed-horizon tracking errors

$$
e_v = v_{\text{EKF,delayed}} - v_{\text{out,delayed}}, \qquad
e_p = p_{\text{EKF,delayed}} - p_{\text{out,delayed}}
$$

and accumulates integral terms:

$$
I_v^{+} = I_v + e_v, \qquad I_p^{+} = I_p + e_p
$$

The proportional gain comes from the configured output time constant:

$$
\tau_{vp} = 0.01 \cdot \texttt{_tauVelPosOutput}
$$

after limiting, and

$$
k_{vp} = \frac{\Delta t_{\text{ekf}}}{\tau_{vp}}
$$

The applied corrections are PI-like:

$$
\Delta p_{\text{corr}} = k_{vp} e_p + 0.1\, k_{vp}^2 I_p
$$

$$
\Delta v_{\text{corr},x/y} = k_{vp} e_{v,x/y} + 0.1\, k_{vp}^2 I_{v,x/y}
$$

with a slightly different vertical integral weighting when `badIMUdata` is true.

Unlike attitude, these corrections are applied to **every stored output-history element**:

$$
\forall i,\quad
v_{\text{out},i} \leftarrow v_{\text{out},i} + \Delta v_{\text{corr}},
\qquad
p_{\text{out},i} \leftarrow p_{\text{out},i} + \Delta p_{\text{corr}}
$$

and then the youngest corrected entry is copied back into `outputDataNew`.

This is why the velocity/position correction loop can be faster and lower-lag than a pure forward-only correction: by rewriting the whole stored history, the code effectively shifts the entire output trajectory toward the delayed EKF solution in one step. The code comment explicitly notes that this approach is avoided for attitude because repeatedly applying quaternion-history rewrites would be too expensive.

This is why EKF3 can fuse an old measurement without re-running a full current-time EKF update: the measurement updates the delayed stochastic state directly, and the output predictor then makes the current-time outputs track that corrected delayed solution. Mathematically, this is an approximation to the exact delayed-measurement Bayesian solution that an augmented clone-state filter would provide, but it is computationally much cheaper and keeps the covariance dimension fixed.

## 6. Propagation model

### 6.1 Bias-corrected IMU increments

Before propagation, the delayed IMU sample is corrected:

$$
\Delta \theta_c = \Delta \theta - b_g
$$
$$
\Delta v_c = \Delta v - b_a
$$

in code via:

- `correctDeltaAngle()`
- `correctDeltaVelocity()`

The active lane bias is mirrored into `inactiveBias[active_index]` by `learnInactiveBiases()`, so the correction path still sees the current state bias even though the helper reads from `inactiveBias[...]`.

### 6.2 Quaternion propagation

The nominal attitude update in `UpdateStrapdownEquationsNED()` is:

$$
q_{k+1} = q_k \otimes \delta q\!\left(\Delta \theta_c - T_{nb}\,\omega_{ie}^n\,\Delta t\right)
$$

where:

- `\omega_{ie}^n` is earth rotation in the local NED frame
- the earth-rate correction is explicitly subtracted in code:
  `delAngCorrected - prevTnb * earthRateNED * imuDataDelayed.delAngDT`

Then the quaternion is normalized.

### 6.3 Velocity propagation

The corrected body-frame delta velocity is rotated to navigation frame:

$$
\Delta v^n = T_{bn}\,\Delta v_c
$$

and gravity is added:

$$
\Delta v^n \leftarrow \Delta v^n + g^n \Delta t
$$

then:

$$
v_{k+1}^n = v_k^n + \Delta v^n
$$

Code mapping:

- `delVelNav = prevTnb.mul_transpose(delVelCorrected)`
- `delVelNav.z += GRAVITY_MSS * imuDataDelayed.delVelDT`
- `stateStruct.velocity += delVelNav`

### 6.4 Position propagation

Position uses trapezoidal integration:

$$
p_{k+1}^n = p_k^n + \frac{1}{2}\left(v_k^n + v_{k+1}^n\right)\Delta t
$$

Code:

- `lastVelocity = stateStruct.velocity`
- update velocity
- `stateStruct.position += (stateStruct.velocity + lastVelocity) * (delVelDT * 0.5f)`

### 6.5 Physical meaning

This is a standard strapdown inertial mechanization, but implemented on delayed buffered IMU deltas rather than on the newest sample. That delayed-horizon choice is the defining architecture feature of EKF3.

## 7. Covariance prediction

### 7.1 Structure

The full covariance is:

$$
P \in \mathbb{R}^{24 \times 24}
$$

stored as `Matrix24 P`.

The prediction step conceptually is:

$$
P_{k+1}^- = F_k P_k^+ F_k^T + G_k Q_k G_k^T
$$

but the implementation does **not** build `F` and `G` explicitly at runtime. Instead, the expanded algebra is generated offline and emitted into `CovariancePrediction()`.

### 7.2 Continuous-to-discrete handling

The effective discrete timestep is:

$$
\Delta t = \text{clamp}\!\left(\frac{\Delta t_{\Delta \theta} + \Delta t_{\Delta v}}{2},\, 0.5\,dtEkfAvg,\, 2\,dtEkfAvg \right)
$$

This protects the covariance prediction from timing spikes.

### 7.3 Process noise structure

The first 10 kinematic states receive white-noise growth directly inside the auto-generated equations using:

- `daxVar = dayVar = dazVar = (dt * gyrNoise)^2`
- `dvxVar = dvyVar = dvzVar = (dt * accNoise)^2`

The remaining process-noise terms are added as a diagonal `processNoiseVariance` vector:

- gyro bias block
- accel bias block
- earth magnetic field block
- body magnetic field block
- wind block

#### Gyro-bias process noise

Because the bias state is stored in delta-angle units, the discrete process variance is:

$$
Q_{b_g} = \left(dt^2 \sigma_{bg}\right)^2
$$

implemented as:

`sq(sq(dt) * _gyroBiasProcessNoise)`

#### Accel-bias process noise

Similarly:

$$
Q_{b_a} = \left(dt^2 \sigma_{ba}\right)^2
$$

implemented as:

`sq(sq(dt) * _accelBiasProcessNoise)`

#### Magnetic-field process noise

$$
Q_{m_e} = (dt\,\sigma_{m_e})^2 I_3
$$
$$
Q_{m_b} = (dt\,\sigma_{m_b})^2 I_3
$$

#### Wind process noise

$$
Q_w = \left(dt\,\sigma_w \left(1 + k_h | \dot{h} |\right)\right)^2 I_2
$$

so wind uncertainty increases during climb/descent if `WIND_PSCALE` is nonzero.

### 7.4 Quaternion covariance initialization

At initialization, EKF3 starts from a 3-D rotation-vector variance and converts it into a quaternion covariance block by rotating that variance into body frame and running the same covariance-prediction machinery with a special `rotVarVecPtr`.

This is one of the clearest places where the implementation behaves like an error-state derivation even though the stored state is direct quaternion components.

## 8. Measurement models and updates

## 8.1 Generic update form

For any measurement:

$$
\nu = \hat{z}(x^-) - z
$$
$$
S = H P^- H^T + R
$$
$$
K = P^- H^T S^{-1}
$$
$$
x^+ = x^- - K\nu
$$
$$
P^+ = P^- - K H P^-
$$

The implementation uses this exact sign convention and usually computes the covariance update in the algebraically reduced form above instead of forming `(I-KH)`.

### 8.2 Velocity, position, and height

The main measurement update for absolute aiding is `FuseVelPosNED()`.

#### Measurement model

The observation model is direct-state:

$$
z_v = v^n + n_v
$$
$$
z_p = p^n + n_p
$$

For scalar sequential updates, each measurement row is just a selector row:

- velocity x touches state `4`
- velocity y touches state `5`
- velocity z touches state `6`
- position x touches state `7`
- position y touches state `8`
- position z touches state `9`

So for a scalar velocity-x observation:

$$
H_{vx} = [0,0,0,0,1,0,0,0,\dots,0]
$$

and similarly for the other direct-state observations.

#### Measurement variance

The implementation does not use a fixed `R`.

Instead it builds `R` from:

- sensor-reported accuracy when available
- configured minimum noise floors
- maneuver-dependent inflation terms such as `gpsNEVelVarAccScale * accNavMag`

That means the effective measurement covariance is state- and motion-dependent.

#### Gating

Scalar position and velocity gating uses normalized innovation test ratios:

$$
r = \frac{\nu^2}{(\gamma \sigma)^2}
$$

or, for 2-axis grouped horizontal tests in the legacy path:

$$
r = \frac{\nu_x^2 + \nu_y^2}{\gamma^2(\sigma_x^2 + \sigma_y^2)}
$$

with:

$$
\gamma = \max(0.01 \cdot \text{gate\_param}, 1)
$$

The gate parameters are percentages in parameters such as `EK3_VEL_I_GATE`, `EK3_POS_I_GATE`, `EK3_HGT_I_GATE`.

#### Correction step

For scalar updates, the code computes:

$$
S = P_{jj} + R_j
$$
$$
K_i = \frac{P_{ij}}{S}
$$

then updates:

$$
x_i^+ = x_i^- - K_i \nu
$$

and

$$
P^+ = P^- - K H P^-
$$

where `H` is a one-hot selector row for the directly observed state.

This is why the scalar path can be implemented very cheaply.

### 8.3 RIO / full-covariance ExternalNav position and velocity

The new RIO path adds grouped direct-state fusion when `hasCovariance = true`.

#### Measurement model

For grouped position:

$$
z_p =
\begin{bmatrix}
p_N \\ p_E \\ p_D
\end{bmatrix}
 + n_p
$$

For grouped velocity:

$$
z_v =
\begin{bmatrix}
v_N \\ v_E \\ v_D
\end{bmatrix}
 + n_v
$$

with selector Jacobians:

$$
H_p =
\begin{bmatrix}
0_{1\times7} & 1 & 0 & 0 & 0_{1\times15} \\
0_{1\times7} & 0 & 1 & 0 & 0_{1\times15} \\
0_{1\times7} & 0 & 0 & 1 & 0_{1\times15}
\end{bmatrix}
$$

acting on states `7,8,9`, and similarly:

$$
H_v
$$

acting on states `4,5,6`.

#### Innovation covariance

Because `H` selects direct-state blocks:

$$
S = H P H^T + R
$$

reduces to the selected covariance sub-block plus the measurement covariance:

$$
S_p = P_{p,p} + R_p
$$
$$
S_v = P_{v,v} + R_v
$$

which is exactly what `CalculateDirectStateGroupInnovations()` computes.

#### Grouped Kalman gain

The grouped update is:

$$
K = P H^T S^{-1}
$$

Because `H` is again a selector matrix, `H^T` just picks the relevant covariance columns, and `FuseDirectStateGroup()` computes the resulting gain directly from the selected state columns.

#### Why this matters

Legacy ExternalNav collapsed the observation uncertainty to isotropic scalars:

- `posErr`
- `err`

The RIO path instead preserves:

- anisotropy
- axis coupling
- nonzero off-diagonal covariance terms

That is the mathematically important change introduced for this project.

### 8.4 Height source fusion

Height is handled inside the same velocity/position update machinery, but the source can switch dynamically:

- barometer
- rangefinder
- GPS
- beacon
- ExternalNav / RIO
- synthetic zero-height when no height source is configured

The measurement model is always:

$$
z_h = p_D + n_h
$$

with the NED sign handled by setting:

$$
z_h = -hgtMea
$$

before fusion.

Source switching is handled in `selectHeightForFusion()`, which can also reset `p_D` when the active height source changes.

### 8.5 Yaw fusion

Yaw is fused in `fuseEulerYaw()`.

#### Measurement model

The observation is:

$$
z_\psi = \psi(q) + n_\psi
$$

where `\psi(q)` is extracted from the quaternion using either:

- 321 yaw
- 312 yaw

The implementation chooses whichever Euler order is better conditioned at the current attitude.

#### Jacobian

The Jacobian is nonzero only in the quaternion block:

$$
H_\psi = \left[\frac{\partial \psi}{\partial q_0},
\frac{\partial \psi}{\partial q_1},
\frac{\partial \psi}{\partial q_2},
\frac{\partial \psi}{\partial q_3},
0,\dots,0\right]
$$

and the code computes this explicitly in the two branch formulas inside `fuseEulerYaw()`.

This is an exact code-to-equation match: the long `SA*` / `SB*` intermediate terms are just algebraically simplified partial derivatives of yaw with respect to quaternion components.

#### Supported yaw sources

- compass-derived yaw
- GPS yaw sensor
- GSF yaw estimate
- static/predicted synthetic yaw
- ExternalNav yaw

For RIO specifically:

- full `3x3` attitude covariance is **not** fused as a full attitude measurement
- `rio_attitude_covariance_to_yaw_variance()` projects the supplied attitude covariance to a scalar yaw variance
- that scalar variance is then fused through the existing yaw path

So the current implementation is yaw-only for external attitude aiding.

### 8.6 Magnetometer fusion

The full magnetometer measurement model in `FuseMagnetometer()` is:

$$
z_m = T_{nb}(q)\,m_e^n + m_b^b + n_m
$$

where:

- `m_e^n` is the earth field state
- `m_b^b` is the body-fixed magnetic term

#### Block structure of the Jacobian

For one axis:

$$
H_m =
\left[
\frac{\partial h_m}{\partial q},
0_{1\times6},
0_{1\times6},
\frac{\partial h_m}{\partial m_e},
\frac{\partial h_m}{\partial m_b},
0_{1\times2}
\right]
$$

So the nonzero blocks touch:

- quaternion `0..3`
- earth magnetic field `16..18`
- body magnetic field `19..21`

and do **not** directly observe velocity, position, or wind.

The code exploits that sparsity explicitly when building `KH` and `KHP`.

#### Gating and health

Each magnetometer axis gets its own innovation variance and normalized innovation ratio. If any axis is unhealthy:

- `magHealth` is cleared
- fusion is skipped
- badly conditioned updates can trigger `CovarianceInit()`

This is more operationally defensive than a clean paper derivation, but it matches the code.

## 9. Source selection and configuration

### 9.1 Source routing

`AP_NavEKF_Source` decides which sources are active:

- `EK3_SRCn_POSXY`
- `EK3_SRCn_VELXY`
- `EK3_SRCn_POSZ`
- `EK3_SRCn_VELZ`
- `EK3_SRCn_YAW`

ExternalNav is considered “enabled” if any active source set references `EXTNAV` in any of those slots.

### 9.2 Parameters that map directly into the math

| Parameter | Mathematical role |
| --- | --- |
| `EK3_VELNE_M_NSE`, `EK3_VELD_M_NSE` | lower bounds on velocity observation noise |
| `EK3_POSNE_M_NSE` | lower bound on horizontal position noise |
| `EK3_ALT_M_NSE` | baro / synthetic-height noise |
| `EK3_VEL_I_GATE`, `EK3_POS_I_GATE`, `EK3_HGT_I_GATE` | innovation gate multipliers |
| `EK3_GLITCH_RAD` | reset/clipping logic for position inconsistency |
| `EK3_GYRO_P_NSE` | IMU gyro white-noise level in covariance prediction |
| `EK3_ACC_P_NSE` | IMU accel white-noise level in covariance prediction |
| `EK3_GBIAS_P_NSE` | gyro-bias random-walk level |
| `EK3_ABIAS_P_NSE` | accel-bias random-walk level |
| `EK3_MAGE_P_NSE`, `EK3_MAGB_P_NSE` | magnetic-state process noise |
| `EK3_WIND_P_NSE`, `EK3_WIND_PSCALE` | wind-state process noise |
| `EK3_TAU_OUTPUT` | output observer velocity/position tracking time constant |
| `EK3_HRT_FILT` | output vertical complementary filter bandwidth |
| `EK3_HGT_DELAY` | height-measurement lag for legacy height sources |

### 9.3 RIO-specific configuration meaning

For RIO/ExternalNav fusion to be active:

- `EK3_SRCn_POSXY = EXTNAV` for horizontal position fusion
- `EK3_SRCn_VELXY = EXTNAV` for horizontal velocity fusion
- `EK3_SRCn_POSZ = EXTNAV` for height fusion
- `EK3_SRCn_VELZ = EXTNAV` for vertical velocity fusion
- `EK3_SRCn_YAW = EXTNAV` for yaw fusion from external attitude

The measurement itself enters through `writeRioNavData()`, but whether each component is actually fused still depends on these source selections.

## 10. Theory versus implementation

### 10.1 What matches the EKF derivation

The implementation still matches the standard discrete EKF structure:

- inertial mechanization for nominal-state propagation
- covariance propagation from symbolic Jacobian algebra
- innovation-based measurement updates
- Kalman gain from predicted covariance and observation Jacobian
- covariance symmetry enforcement and variance constraints after each update

### 10.2 What is different from a textbook “clean” EKF

1. **Direct quaternion state, not minimal attitude error state**  
   The filter stores a 4-element quaternion directly and updates those four components, then renormalizes.

2. **Delayed-horizon architecture**  
   The main EKF runs on delayed buffered data, then a separate observer propagates outputs to current time.

3. **Runtime-active state dimension**  
   The filter dynamically shortens the active state dimension with `stateIndexLim`.

4. **Sequential scalar fusion for many sensors**  
   GPS-style position, velocity, and height are often fused one scalar at a time for efficiency and robustness.

5. **Grouped direct-state fusion only where justified**  
   The new RIO path upgrades only the position/velocity ExternalNav updates to grouped correlated fusion.

6. **No online extrinsic estimation**  
   Sensor offsets are corrected outside the state, not estimated inside it.

7. **Auxiliary estimators outside the main state**  
   terrain offset, output observer, and GSF yaw are coupled subsystems, not main-state augmentations.

8. **Operational reset and timeout logic is part of the estimator design**  
   source switching, timeouts, clipping, fallback-to-baro, GPS glitch resets, and sensor-health resets are first-class implementation behavior.

9. **Current external attitude fusion is yaw-only**  
   RIO carries a full attitude covariance, but EKF3 presently projects it to a scalar yaw variance.

## 11. Current project-specific implications

For the current RIO integration, the important conclusions are:

1. EKF3 internally remains a NED inertial filter with quaternion, velocity, position, IMU-bias, mag, and wind states.
2. RIO enters through the existing ExternalNav storage/fusion architecture, but with a new covariance-aware ingestion path.
3. RIO position and velocity can now be fused with full `3x3` covariances, including off-diagonal terms.
4. RIO attitude is still reduced to yaw-only fusion.
5. The timing contract for RIO is stricter than legacy ExternalNav: the packet timestamp is already expected to be in FC boot-time and is not passed through the old lag-correction path.

## 12. Re-derivation checklist

To reconstruct the implemented filter from code, follow this order:

1. state definition: `AP_NavEKF3_core.h`
2. buffer and timing setup: `setup_core()` in `AP_NavEKF3_core.cpp`
3. IMU delayed-horizon pipeline: `readIMUData()` in `AP_NavEKF3_Measurements.cpp`
4. nominal-state propagation: `UpdateStrapdownEquationsNED()`
5. covariance prediction: `CovariancePrediction()`
6. yaw/mag update: `SelectMagFusion()`, `fuseEulerYaw()`, `FuseMagnetometer()`
7. absolute aiding update: `SelectVelPosFusion()`, `FuseVelPosNED()`
8. ExternalNav/RIO grouped update: `CalculateDirectStateGroupInnovations()`, `FuseDirectStateGroup()`, `writeRioNavData()`
9. current-time output reconstruction: `calcOutputStates()`

That sequence is the implemented EKF3, not merely the conceptual one.
