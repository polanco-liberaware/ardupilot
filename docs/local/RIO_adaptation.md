# RIO adaptation

This document tracks the ArduCopter changes being made to support the custom RIO companion-computer pipeline.

## Applied changes

### 1. MAVLink message

Added a dedicated `RIO_NAV_STATE` MAVLink2 message in `modules/mavlink/message_definitions/v1.0/ardupilotmega.xml`.

Key points:
- Strict ODOMETRY-style contract:
  - `frame_id = MAV_FRAME_LOCAL_FRD`
  - `child_frame_id = MAV_FRAME_BODY_FRD`
  - position in local FRD
  - velocity in body FRD
  - quaternion is body -> local FRD, `w,x,y,z`
- Covariances are explicit and packed as `[xx, xy, xz, yy, yz, zz]`.
- Attitude uncertainty is included with `attitude_covariance[6]`.
- Compact health/debug fields remain numeric and machine-readable.
- ROS-side enums are mapped deliberately to MAVLink enums; raw internal values are not copied unless they match exactly.

### 2. Receive path

Added `handle_rio_nav_state()` in `libraries/GCS_MAVLink/GCS_Common.cpp` and wired it into the MAVLink dispatch path.

Receiver behavior:
- rejects non-finite fields
- rejects zero or non-normalized quaternions
- keeps the strict frame checks
- only forwards packets that pass the configured quality threshold

The packet is then handed to the dedicated RIO estimator-routing path:

- `AP::ahrs().writeRioNavData(...)`
- `EKF3.writeRioNavData(...)`
- `NavEKF3_core::writeRioNavData(...)`

### 3. EKF3 / receiving-side prototype

The current prototype starts separating `RIO_NAV_STATE` from the legacy scalar EXTNAV path.

Implemented:
- `RIO_NAV_STATE` now preserves the decoded 3x3 covariance blocks on ingress instead of collapsing everything immediately to one scalar per block.
- body-frame velocity covariance from the packet is rotated into the navigation frame before being handed to EKF3, matching the existing body-velocity-to-nav conversion for the velocity vector itself.
- RIO samples use a dedicated EKF3 ingress method that takes the packet timestamp directly in the FC boot-time domain, instead of reusing the fixed-delay EXTNAV write path directly.
- EKF3 now performs a RIO-specific correlated direct-state fusion for external-nav position and velocity, so the full position/velocity covariance matrices contribute to the Kalman update instead of only a single spherical `posErr` / `velErr`.
- the receive side now logs an estimated receive-side delay computed from `now_ms - packet_time_ms`, making the timing path observable during PoC testing.
- the existing EXTNAV yaw path still uses a scalar yaw uncertainty, but that scalar is now derived from the full attitude covariance matrix instead of a single diagonal entry.

Current limitation:
- the full 3x3 attitude covariance is not fused as a full 3-axis attitude observation because the current EKF3 EXTNAV attitude path is still yaw-only.

## Why both `AP_AHRS` and `AP_NavEKF3` were modified

The new RIO path crosses two layers that already exist in ArduPilot:

1. `AP_AHRS` is the estimator-facing routing layer used by higher-level code such as `GCS_MAVLink`.
2. `AP_NavEKF3` is the estimator implementation that owns the external-navigation buffers and fusion logic.

### Why not call `AP_NavEKF3` directly from `GCS_MAVLink`?

`GCS_MAVLink` already hands estimator data to the navigation stack through `AP::ahrs()`. That is the existing abstraction boundary for:
- `writeExtNavData(...)`
- `writeExtNavVelData(...)`
- `writeBodyFrameOdom(...)`

Keeping the new RIO entry point inside `AP_AHRS` preserves that layering:
- MAVLink receive code stays estimator-agnostic at the call site.
- `AP_AHRS` remains the single place where navigation measurements are handed to the active EKF implementation.
- the rest of the codebase still sees one estimator-routing interface instead of a new direct dependency from `GCS_MAVLink` into EKF3 internals.

### Why was `AP_NavEKF3` still required?

Because the existing EKF3 external-navigation APIs only accept scalar uncertainty:
- position: `posErr`
- attitude/yaw: `angErr`
- velocity: `err`

That means the old APIs fundamentally cannot carry:
- full 3x3 position covariance
- full 3x3 velocity covariance
- full 3x3 attitude covariance
- a RIO-specific measurement path that uses the FC-time packet timestamp directly

So:
- `AP_AHRS` had to grow a new forwarding method (`writeRioNavData(...)`) to preserve the normal routing architecture.
- `AP_NavEKF3` had to grow a new ingestion/fusion path because only EKF3 can actually store and fuse the covariance-aware measurement.

In short:
- `AP_AHRS` was changed for architectural routing consistency.
- `AP_NavEKF3` was changed because that is where the mathematics and state update actually live.

## Why the local matrix inverse helper exists in `AP_NavEKF3_PosVelFusion.cpp`

The new RIO covariance-aware update needs to invert innovation covariance matrices of size:
- 1x1
- 2x2
- 3x3

ArduPilot already has a general `Matrix3::inverse(...)` helper in `AP_Math`, and the new code uses it for the 3x3 case.

The local `rio_invert_matrix(...)` helper was added because the fusion code needs one small wrapper that:
- accepts a runtime observation dimension (`1`, `2`, or `3`)
- uses the cheapest correct inverse for that dimension
- rejects singular or non-invertible innovation covariance matrices in one place
- avoids creating a separate generic utility just for this RIO-specific PoC path

So the helper is not replacing existing math support. It is a thin fusion-local adapter:
- 1x1: explicit reciprocal
- 2x2: explicit closed-form inverse
- 3x3: existing `Matrix3::inverse(...)`

That keeps the EKF code simple at the call site and avoids pretending a padded 2x2 matrix is really a well-conditioned 3x3 inverse problem.

## Detailed pipeline and math

This section documents the implemented receiving-side pipeline from packet arrival to EKF3 state correction.

### 1. Packet semantics at the receive boundary

The RIO packet is treated as:
- position $p_{\text{meas}} = [x, y, z]^T$ in local FRD / navigation frame
- velocity $v_{\text{body,meas}} = [v_x, v_y, v_z]^T$ in body FRD
- attitude quaternion $q_{l\_b}$ from body to local FRD
- packed covariance blocks:
  - $P_{\text{pos,nav}}$ is navigation-frame position covariance
  - $P_{\text{vel,body}}$ is body-frame velocity covariance
  - $P_{\text{att,body}}$ is body-frame small-angle attitude covariance

The packed MAVLink order is:
- `[xx, xy, xz, yy, yz, zz]`

which is unpacked into a symmetric matrix:

$$
\begin{bmatrix}
xx & xy & xz \\
xy & yy & yz \\
xz & yz & zz
\end{bmatrix}
$$

### 2. Timestamp handling and delay interpretation

The sender transmits `time_usec` in the FC boot-time domain, not ROS wall clock.

For `RIO_NAV_STATE`, the receiver does **not** call `correct_offboard_timestamp_usec_to_ms(...)`. Unlike the generic offboard-vision paths, this RIO path assumes the sender is already propagating and publishing the measurement in the FC clock domain end-to-end.

Define:

- $t_{\text{msg,fc}}$: the timestamp carried by the MAVLink packet, after converting `time_usec` to milliseconds
- $t_{\text{meas,fc}}$: the measurement-valid time on the FC timeline used by EKF3
- $t_{\text{rx}}$: the FC local receive time taken from `AP_HAL::millis()`

Then:

$$
t_{\text{msg,fc}} = \left\lfloor \frac{m.\text{time\_usec}}{1000} \right\rfloor
$$

and for this RIO path we take:

$$
t_{\text{meas,fc}} = t_{\text{msg,fc}}
$$

This is a deliberate RIO-specific contract:

- the FC timestamps the IMU data that drives RIO
- RIO propagates its state in that same FC boot-time domain
- the sent `time_usec` therefore already represents the measurement-valid time on the FC timeline

So there is no extra receive-side lag-correction step for `RIO_NAV_STATE`.

The receive-side estimated delay that is logged is:

$$
\Delta t_{\text{rx}} = t_{\text{rx}} - t_{\text{meas,fc}}
$$

and in code that becomes the logged field:

$$
\text{delay\_est\_ms} = \max(0,\, t_{\text{rx}} - t_{\text{meas,fc}})
$$

This value is only used for **observability and debugging** in the current prototype. It lets us inspect the effective end-to-end age of the sample seen by the FC.

Important design point:
- the old EXTNAV path used `(measurement_time, fixed_delay_ms)` and then internally shifted the timestamp by `delay_ms`
- the new RIO path instead hands EKF3 the packet's FC-time measurement timestamp directly

So in the RIO path:

- the **actual EKF timing input** is $t_{\text{meas,fc}}$
- the **logged delay estimate** is $\Delta t_{\text{rx}}$

The EKF uses the packet-provided FC-time measurement timestamp directly. The logged delay estimate is not separately fed back into the EKF as another delay parameter in this prototype.

More explicitly, the timing pipeline is:

1. The packet arrives with `time_usec`.
2. ArduPilot converts it to milliseconds and sets $t_{\text{meas,fc}} = t_{\text{msg,fc}}$.
3. `handle_rio_nav_state()` passes $t_{\text{meas,fc}}$ into `writeRioNavData(...)`.
4. Inside `NavEKF3_core::writeRioNavData(...)`, EKF3 shifts that time only by half the local filter step to align the sample with the fusion horizon:

$$
t_{\text{fuse}} = t_{\text{meas,fc}} - \frac{\Delta t_{\text{ekf}}}{2}
$$

5. That $t_{\text{fuse}}$ is what gets stored in the EKF observation buffers.

So the logged delay estimate answers:

> \"How old was this sample when the FC received it?\"

while the packet timestamp answers:

> \"At what FC-local time should EKF3 treat this measurement as valid?\"

### 3. Frame conversion before EKF3 storage

The packet velocity is body-frame, but the existing EKF3 external-nav velocity fusion path works in navigation-frame velocity states.

Let:
- $R_{l\_b}$ be the body-to-local rotation matrix derived from $q_{l\_b}$

Then the receive code computes:

$$
v_{\text{nav,meas}} = R_{l\_b}\,v_{\text{body,meas}}
$$

and rotates the body-frame velocity covariance into navigation frame:

$$
P_{\text{vel,nav}} = R_{l\_b}\,P_{\text{vel,body}}\,R_{l\_b}^T
$$

That step is critical because otherwise the covariance would describe uncertainty in a different frame than the fused velocity vector.

### 4. New RIO-specific ingestion path

`handle_rio_nav_state()` no longer routes the RIO packet through:
- `AP_VisualOdom_MAV::handle_pose_estimate(...)`
- `AP_AHRS::writeExtNavData(...)`
- `AP_AHRS::writeExtNavVelData(...)`

for the actual EKF handoff.

Instead it calls:

```cpp
AP::ahrs().writeRioNavData(...)
```

which forwards to:

```cpp
EKF3.writeRioNavData(...)
```

and then to each EKF3 core:

```cpp
NavEKF3_core::writeRioNavData(...)
```

This new function stores:
- full position covariance
- full velocity covariance
- full attitude covariance
- FC-time measurement timestamp
- reset counter

without converting them to scalar errors first.

### 5. What is stored inside EKF3

For RIO-backed EXTNAV samples, EKF3 now fills:

#### Position buffer element

$$
z_{\text{pos}} = p_{\text{meas}}
$$

$$
R_{\text{pos}} = P_{\text{pos,nav}}
$$

plus flags:
- `hasCovariance = true`
- `posReset` derived from reset counter change

#### Velocity buffer element

$$
z_{\text{vel}} = v_{\text{nav,meas}}
$$

$$
R_{\text{vel}} = P_{\text{vel,nav}}
$$

plus:
- `hasCovariance = true`

#### Yaw buffer element

The current EKF3 EXTNAV attitude path is still yaw-only.

So the full attitude covariance is not fused as a 3-axis attitude observation. Instead, the code projects the full attitude covariance into an equivalent yaw variance.

### 6. Yaw variance from the full attitude covariance

Let the attitude covariance in small-angle form be:

$$
P_{\text{att}}
$$

in body-frame small-angle coordinates.

The code numerically approximates the yaw sensitivity Jacobian:

$$
J_{\text{yaw}} =
\begin{bmatrix}
\frac{\partial \,\text{yaw}}{\partial \theta_x} &
\frac{\partial \,\text{yaw}}{\partial \theta_y} &
\frac{\partial \,\text{yaw}}{\partial \theta_z}
\end{bmatrix}
$$

by perturbing the quaternion with small positive and negative rotation vectors around each axis and computing the centered finite difference.

Then the scalar yaw variance used by the existing EXTNAV yaw fusion path is:

$$
\sigma_{\text{yaw}}^2 = J_{\text{yaw}}\,P_{\text{att}}\,J_{\text{yaw}}^T
$$

and the yaw 1-sigma value passed onward is:

$$
\sigma_{\text{yaw}} = \sqrt{\sigma_{\text{yaw}}^2}
$$

with a floor to preserve the prior EXTNAV yaw safety behavior.

This means:
- all entries of the 3x3 attitude covariance influence the scalar yaw uncertainty
- but the EKF still fuses only a scalar yaw observation, not a full 3-axis attitude observation

### 7. Why non-diagonal position/velocity terms now matter

The old scalar path effectively replaced the incoming covariance matrix with something like:

$$
R_{\text{scalar}} = \sigma^2 I
$$

or, operationally, independent per-axis scalar variances.

That destroys correlation information such as:
- positive correlation between X and Y position errors
- coupling between horizontal and vertical position uncertainty
- correlated velocity uncertainty caused by estimator geometry

The new RIO path preserves the actual measurement covariance matrix:

$$
R = [r_{ij}]
$$

and uses it directly in the innovation covariance and Kalman gain.

### 8. Correlated direct-state measurement model

For the RIO external-nav position/velocity path, the measurement directly observes EKF3 state components:

- velocity states: `x_v = [v_N, v_E, v_D]^T`
- position states: `x_p = [p_N, p_E, p_D]^T`

So the measurement model is linear:

$$
z = Hx + n
$$

where `H` is just a selector matrix that picks the relevant state elements.

For example, if fusing 3D velocity:

$$
z_{\text{vel}} =
\begin{bmatrix}
v_N \\
v_E \\
v_D
\end{bmatrix}
 + n_{\text{vel}}
$$

with:

$$
H_{\text{vel}} =
\begin{bmatrix}
0 & \dots & I_3 & \dots & 0
\end{bmatrix}
$$

If fusing horizontal position plus height:

$$
z_{\text{pos}} =
\begin{bmatrix}
p_N \\
p_E \\
p_D
\end{bmatrix}
 + n_{\text{pos}}
$$

with:

$$
H_{\text{pos}} =
\begin{bmatrix}
0 & \dots & I_3 & \dots & 0
\end{bmatrix}
$$

over the position state block.

Because this is direct-state observation, the update is much simpler than a quaternion- or body-velocity-derived nonlinear observation model.

### 9. Innovation and innovation covariance

For an observation group with state indices `i_1 ... i_n`, the code builds:

$$
y = x_{\text{sel}} - z
$$

where:
- `x_sel` is the vector of current EKF state values at those indices
- `z` is the measurement vector

This is the innovation used by the current codebase sign convention.

The innovation covariance is:

$$
S = P_{\text{sel}} + R
$$

because $H$ is a direct selector and therefore:

$$
HPH^T = P_{\text{sel}}
$$

where `P_sel` is the state covariance submatrix for the selected states.

This is the first place where off-diagonal measurement terms matter directly:

$$
S_{ij} = P_{\text{sel},ij} + R_{ij}
$$

So if the measurement says errors in axes `i` and `j` are correlated, that correlation is preserved in the innovation model instead of discarded.

### 10. Innovation consistency test with full covariance

The old scalar/sequential path used sums of squared normalized residuals with diagonal variances.

The new correlated RIO group test uses the Mahalanobis form:

$$
\text{test\_ratio} = \frac{y^T S^{-1} y}{\text{gate}^2}
$$

where `gate` is the existing innovation gate parameter for the relevant measurement family.

This is the correct multivariate generalization of the scalar innovation test.

Interpretation:
- if the covariance ellipse is narrow along the innovation direction, the test ratio grows quickly
- if the covariance ellipse is broad along that direction, the same raw innovation is less surprising

So the acceptance test now respects the shape and orientation of the measurement uncertainty ellipse/ellipsoid.

### 11. Kalman update with correlated covariance

For the accepted RIO measurement group, the Kalman gain is:

$$
K = PH^T S^{-1}
$$

Because `H` selects direct state components, this becomes:

$$
K = P(:, \text{sel})\,S^{-1}
$$

where `P(:, sel)` is the full covariance matrix column block corresponding to the measured states.

Then the state correction is:

$$
x_{\text{new}} = x - Ky
$$

and the covariance correction is:

$$
P_{\text{new}} = P - K(HP)
$$

which the implementation writes as:

$$
P_{\text{new}} = P - K\,P(\text{sel}, :)
$$

again because `H` is a selector matrix.

This is the key reason non-diagonal measurement covariance terms matter:

1. `R` changes `S`
2. `S^-1` changes the Kalman gain direction and magnitude
3. the correction is therefore shaped by the full covariance ellipse, not by independent per-axis weights

### 12. What happens for 1D / 2D / 3D cases

The code supports observation groups of runtime size:
- 1: scalar
- 2: XY
- 3: XYZ

That is why the local inverse helper exists.

Examples:

#### 2D horizontal position fusion

If EXTNAV position is active for XY but not height:

$$
z =
\begin{bmatrix}
p_N \\
p_E
\end{bmatrix}
$$

$$
R =
\begin{bmatrix}
r_{xx} & r_{xy} \\
r_{xy} & r_{yy}
\end{bmatrix}
$$

Then the update uses the full 2x2 covariance, so `r_xy` directly affects the accepted innovation geometry and Kalman gain.

#### 3D position + height fusion

If EXTNAV is the source for both horizontal position and height:

$$
z =
\begin{bmatrix}
p_N \\
p_E \\
p_D
\end{bmatrix}
$$

$$
R = P_{\text{pos}}
$$

Then `r_xz` and `r_yz` are also used.

#### 2D or 3D velocity fusion

Likewise for velocity, depending on whether EXTNAV is configured as the vertical velocity source as well.

### 13. Source gating

The covariance-aware path is intentionally not global.

It is only exercised when the normal EKF3 source selection says EXTNAV is active:
- horizontal position source is `EXTNAV`
- horizontal velocity source is `EXTNAV`
- height source is `EXTNAV` if height is fused in the same correlated group
- vertical velocity source is `EXTNAV` if vertical velocity is fused in the same correlated group

So the new path does not change GPS, optical flow, or other existing fusion modes.

### 14. Summary of the mathematical difference vs. the old path

#### Old path

The old path effectively behaved like:

$$
R \rightarrow \operatorname{diag}(\sigma_x^2, \sigma_y^2, \sigma_z^2)
$$

or worse, one scalar noise value reused across axes.

That means:
- no correlation information
- axis weighting independent
- innovation gate based on independent-axis assumptions

#### New RIO path

The new path uses:

$$
R_{\text{full}}
$$

in:

$$
S = P_{\text{sel}} + R_{\text{full}}
$$

$$
K = P(:, \text{sel})\,S^{-1}
$$

$$
x_{\text{new}} = x - Ky
$$

$$
P_{\text{new}} = P - K\,P(\text{sel}, :)
$$

So the received estimated-state covariance is treated as a true multivariate measurement covariance, not as three unrelated scalar confidences.

### 4. Logging

Added RIO visual-odometry style logging in `libraries/AP_VisualOdom/LogStructure.h`.

Notes:
- `RIOA` logs the attitude quaternion exactly as received from the CC (`q_w,q_x,q_y,q_z`, body $\to$ local FRD)
- `RIOV` logs velocity exactly as received from the CC in body FRD
- `RIOP` logs position exactly as received from the CC in local FRD
- `RIOAV` logs the body-frame attitude covariance block exactly as received from the CC
- `RIOVV` logs velocity covariance exactly as received from the CC in body FRD
- `RIOPV` logs position covariance exactly as received from the CC in local FRD
- `RIOS` logs health/status/debug
- `FTimeMS` in all RIO logs is the packet `time_usec` converted directly into FC boot-time milliseconds
- only `RIOS.DlyMS` is FC-derived; the other RIO state/debug fields are packet content from the CC
- the logger label strings were kept within AP_Logger’s format-string limits
- `quality` stays separate from debug/status fields
- health/debug data remain machine-readable and compact

The AP_VisualOdom logging family is intentionally split between legacy visual-odometry conventions and the new raw-RIO packet preservation logs:

| Message | Meaning / frame |
| --- | --- |
| `VISO` | body-frame incremental odometry deltas |
| `VISP` | legacy ExternalNav-style position/attitude logging in navigation-frame XYZ convention |
| `VISV` | legacy ExternalNav-style velocity logging in NED |
| `RIOP` | raw CC position in `MAV_FRAME_LOCAL_FRD` |
| `RIOPV` | raw CC position covariance in local FRD |
| `RIOV` | raw CC velocity in `MAV_FRAME_BODY_FRD` |
| `RIOVV` | raw CC velocity covariance in body FRD |
| `RIOA` | raw CC quaternion, body -> local FRD |
| `RIOAV` | raw CC body-frame attitude covariance |
| `RIOS` | raw CC status/debug plus FC-derived receive delay |

Important precautions when reviewing logs:

- do **not** assume all `VIS*` and `RIO*` `X/Y/Z` fields are in the same frame
- `VISV` and `RIOV` are intentionally different: `VISV` is navigation-frame velocity, `RIOV` is body-frame velocity
- `VISP` and `RIOP` are also intentionally different contracts: `VISP` follows the legacy ExternalNav/navigation-frame convention, while `RIOP` preserves the raw CC packet position in local FRD
- therefore `VISP`/`VISV` should not be numerically compared against `RIOP`/`RIOV` without an explicit frame-conversion step and a check that the origins and axis definitions are actually aligned
- similarly, the main EKF `XKF*` position/velocity outputs and innovations are NED/navigation-frame quantities, so they are not directly comparable to raw `RIOP`/`RIOV` either

The intended interpretation is:

- `RIO*` logs answer **"what did the FC receive from the CC packet?"**
- `VIS*` logs answer **"what was handed to the legacy visual-odometry / ExternalNav interface?"**
- `XKF*` logs answer **"what does EKF3 estimate and fuse internally in its navigation frame?"**

## Reviewed status

Two independent end-to-end reviews and two independent FC-side reviews were run against the current implementation.

Current reviewed status:
- the end-to-end contract is consistent across the CC bridge, FC receiver, EKF handoff, DataFlash logs, and UAVLogViewer
- the DataFlash labels and order are now:
  - `RIOA`, `RIOV`, `RIOP`, `RIOAV`, `RIOVV`, `RIOPV`, `RIOS`
- the logged state/covariance records are the raw packet values as received by the FC
- only `RIOS.DlyMS` is derived on the FC
- the FC-side grouped position/velocity covariance fusion path is mathematically sound for the current prototype

One known low-severity caveat remains in the current FC implementation:

- `NavEKF3_core::writeRioNavData(...)` currently detects `posReset` by comparing the received `reset_counter` against `extNavLastResetCounter`
- on the very first received RIO packet, a non-zero `reset_counter` can therefore be interpreted as a reset event
- until that is changed in code, the safest CC behavior is:
  - start the first sent packet at `reset_counter = 0`
  - increment only on actual estimator reset / jump events

This is not a message-definition mismatch; it is a current EKF3 integration detail that the sender should know.

## Companion-computer sender review checklist

The CC side should review the following files together, because they define the effective sender contract that the FC expects.

### 1. Message definition used by the CC MAVLink library

Review:

- `RIO/humble_ws/src/fc_com_if/include/drivers/mavlink/c_library_v2/ardupilotmega/mavlink_msg_rio_nav_state.h`

This generated header must agree with ArduPilot's `ardupilotmega.xml` on:

- message id: `RIO_NAV_STATE = 11061`
- field order and offsets
- `q[4]` ordering: `w, x, y, z`
- covariance packing order: `[xx, xy, xz, yy, yz, zz]`
- frames:
  - `frame_id = MAV_FRAME_LOCAL_FRD`
  - `child_frame_id = MAV_FRAME_BODY_FRD`

### 2. RIO producer that populates `/nav/odometry_fc`

Review:

- `RIO/humble_ws/src/rio_ros/src/rio_ros.cpp`

The FC-facing odometry publisher currently defines the payload source semantics as:

- pose position in `local_frd`
- pose quaternion as body -> local FRD
- twist linear velocity in `body_frd`
- pose covariance built from:
  - navigation-frame position covariance
  - body-frame attitude covariance block
- twist covariance built from:
  - body-frame velocity covariance
- timestamp source:
  - `fc_boot_time_usec = state_time_sec * 1e6`

This file is the source of truth for what the bridge sends onward to the FC.

### 3. FC bridge that converts ROS odometry/debug into `RIO_NAV_STATE`

Review:

- `RIO/humble_ws/src/fc_com_if/src/fc_com_if.cpp`

This file is where the CC actually builds the MAVLink packet. It is responsible for:

- copying raw position from `/nav/odometry_fc` into `x,y,z`
- copying raw quaternion into `q[0..3]` in `w,x,y,z`
- copying raw body-frame velocity into `vx,vy,vz`
- packing the 3x3 covariance blocks into `[xx,xy,xz,yy,yz,zz]`
- mapping `RioDebug` / `RadarEstimatorDebug` into:
  - `health_state`
  - `health_reason`
  - `status_flags`
  - `rejection_reason`
  - `raw_point_count`
  - `valid_point_count`
  - `condition_number`
  - `sigma_radar`
  - `radar_age_sec`
- sending `time_usec = fc_boot_time_usec`

Important sender-side behavior in this file:

- if FC global origin is missing and degraded publishing is allowed, quality is capped before sending
- if local-origin zeroing is enabled after FC origin confirmation, the sent position becomes the latched local-FRD pose relative to that origin
- if pose or body velocity is non-finite, the bridge skips publishing

### 4. Sender-side invariants the FC currently assumes

Before sending `RIO_NAV_STATE`, the CC side should verify:

1. `time_usec` is already in the FC boot-time domain, not ROS wall clock
2. `q` is finite and normalized
3. `vx, vy, vz` are body-FRD values
4. `position_covariance` is local-FRD
5. `velocity_covariance` is body-FRD
6. `attitude_covariance` is the body-frame small-angle covariance
7. all covariance elements are finite
8. the first emitted `reset_counter` should be `0` unless the sender intentionally wants the FC to observe a reset condition immediately

### 5. FC files the CC team should cross-check against

If the CC side wants to verify exactly what the FC expects, these are the FC-side files to compare against:

- message definition:
  - `modules/mavlink/message_definitions/v1.0/ardupilotmega.xml`
- MAVLink receive and raw logging:
  - `libraries/GCS_MAVLink/GCS_Common.cpp`
- DataFlash log schema:
  - `libraries/AP_VisualOdom/LogStructure.h`
- EKF ingress timing / reset handling:
  - `libraries/AP_NavEKF3/AP_NavEKF3_Measurements.cpp`
- EKF grouped covariance fusion:
  - `libraries/AP_NavEKF3/AP_NavEKF3_PosVelFusion.cpp`
- local design note:
  - `docs/local/RIO_adaptation.md`

### 6. Parser/viewer files that consume the resulting `.bin` logs

For post-flight tooling agreement, the following UAVLogViewer files now expect the final RIO log contract:

- `src/tools/parsers/parser.worker.js`
- `src/tools/dataflashDataExtractor.js`
- `src/components/Home.vue`
- `src/assets/logmetadata/copter.xml`

The viewer expects:

- `RIOA` as the quaternion attitude record
- quaternion fields `QW,QX,QY,QZ`
- `RIOVV` units as velocity covariance (`m^2/s^2`)
- `FTimeMS` to mean packet FC boot-time milliseconds
- `RIOS.DlyMS` to be the only FC-derived delay/debug timing field

## Current scope

Implemented:
- MAVLink message definition
- ArduPilot receive-side handler
- packet validation and frame enforcement
- compact debug logging format
- pose/velocity/covariance/status logging
- covariance-preserving RIO EKF3 ingress prototype
- correlated EXTNAV position/velocity fusion using the full RIO covariance matrices
- receive-side delay estimation/logging from the FC-time packet timestamp
- full attitude covariance projection into yaw uncertainty for the existing EXTNAV yaw path

Pending / undecided:
- full 3-axis attitude fusion from attitude covariance
- control-path changes
- any follow-up adjustments after build-generation validation

## Contract summary

This RIO packet is intended to stay a compact nav-state + debug message, not a transport for raw radar data, point clouds, or text logs.
