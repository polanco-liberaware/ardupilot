# RIO - EKF

> Mathematical and architectural description of the EKF-RIO implementation currently used in `humble_ws/src/rio_ros/ekf_rio_lib/`.

This document describes the code that is actually running in this repository, not the original ROS1 packaging from Christopher Doer. The core EKF structure is still recognizably Doer's EKF-RIO, but the surrounding runtime, configuration, timing boundary, and radar front-end integration have been adapted for this repository.

---

## Table of Contents

- [Frame Conventions](#frame-conventions)
- [1. Architecture Summary](#1-architecture-summary)
- [2. Nominal State and Error State](#2-nominal-state-and-error-state)
  - [Nominal State](#nominal-state)
  - [Error-State Indexing](#error-state-indexing)
  - [Radar Clone State](#radar-clone-state)
- [3. Initialization Phase](#3-initialization-phase)
- [4. Predict Step (IMU)](#4-predict-step-imu)
  - [Bias Correction](#bias-correction)
  - [Nominal Strapdown Propagation](#nominal-strapdown-propagation)
  - [Error-State Covariance Propagation](#error-state-covariance-propagation)
- [5. Radar Update](#5-radar-update)
  - [Scan-Level Front End in This Repository](#scan-level-front-end-in-this-repository)
  - [Clone Augmentation](#clone-augmentation)
  - [Measurement Model](#measurement-model)
  - [Innovation and Gating](#innovation-and-gating)
  - [Nominal-State Correction](#nominal-state-correction)
- [6. Altimeter Update](#6-altimeter-update)
- [7. Adapter Boundary and Published Output](#7-adapter-boundary-and-published-output)
- [8. Configuration Summary](#8-configuration-summary)
- [9. Same vs Different From The Original Doer Implementation](#9-same-vs-different-from-the-original-doer-implementation)

---

## Frame Conventions

There are two frame layers in the current repository:

| Layer | Frame | Meaning |
|---|---|---|
| Inside `EkfRioFilter` | NED | North-East-Down navigation frame |
| Outside the filter (`rio_ros`, topics, RViz) | FLU | Forward-Left-Up body and world-facing conventions used by the ROS runtime |

The adapter is the explicit boundary:

1. IMU enters in FLU and is converted to NED before it reaches the filter.
2. Navigation state exits in NED and is converted back to FLU before publication.

The implemented FLU to NED conversion is:

$$
\vec{v}_{\text{ned}} =
\begin{bmatrix}
v_x \\
-v_y \\
-v_z
\end{bmatrix}
$$

The same sign change is applied to acceleration, angular rate, velocity, position, and biases at the adapter boundary.

Quaternion conversion is implemented as a conjugation by a $180^\circ$ rotation about the $x$ axis:

$$
q_{\text{flu}} = q_{x,\pi} \; q_{\text{ned}} \; q_{x,\pi}^{-1}
$$

with

$$
q_{x,\pi} = (0, 1, 0, 0)
$$

---

## 1. Architecture Summary

The current runtime shape is:

```text
/fc/imu (FLU) -> ekf_rio_adapter -> FLU->NED conversion -> EkfRioFilter
                                                   |
/ti_mmwave/radar_scan_pcl -> rio_ros conversion -> RadarEgoVelocityEstimator -> EKF radar update
                                                   |
                                         NED->FLU conversion -> /nav outputs
```

The filter core itself is responsible for:

1. IMU initialization
2. nominal strapdown propagation
3. error-state covariance propagation
4. optional clone augmentation for radar updates
5. Kalman updates for radar ego velocity and altimeter

The repository-local adapter is responsible for:

1. configuration loading
2. FLU to NED and NED to FLU conversion
3. stationary initialization timing
4. radar front-end invocation
5. buffered IMU aggregation for the radar update input
6. thread-safe access from callbacks and timer publication

---

## 2. Nominal State and Error State

### Nominal State

The implemented filter stores its nominal state in structured objects rather than a single flat vector.

The nominal navigation and calibration state is:

$$
\mathcal{X} = \left( \vec{p}_{n b},\; \vec{v}_{n b},\; q_{n b},\; \vec{b}_a,\; \vec{b}_g,\; b_{alt},\; \vec{l}_{b r},\; q_{b r} \right)
$$

where:

| Symbol | Dimension | Frame | Meaning |
|---|---:|---|---|
| $\vec{p}_{n b}$ | 3 | NED | Body position in navigation frame |
| $\vec{v}_{n b}$ | 3 | NED | Body velocity in navigation frame |
| $q_{n b}$ | 4 | Body to NED | Body attitude |
| $\vec{b}_a$ | 3 | Body | Accelerometer bias |
| $\vec{b}_g$ | 3 | Body | Gyroscope bias |
| $b_{alt}$ | 1 | scalar | Altimeter bias |
| $\vec{l}_{b r}$ | 3 | Body | Radar position extrinsic |
| $q_{b r}$ | 4 | Radar to Body rotation | Radar orientation extrinsic |

Important implementation note:

1. gravity is not part of the EKF state in this implementation
2. gravity is a fixed configuration parameter `g_n`
3. this is a major difference from the old UKF in `rio_ukf.md`

### Error-State Indexing

The covariance is propagated over a 22-dimensional error state:

$$
\delta \vec{x} =
\begin{bmatrix}
\delta \vec{p} \\
\delta \vec{v} \\
\delta \vec{\theta} \\
\delta \vec{b}_a \\
\delta \vec{b}_g \\
\delta b_{alt} \\
\delta \vec{l}_{b r} \\
\delta \vec{\theta}_{b r}
\end{bmatrix}
\in \mathbb{R}^{22}
$$

with index layout:

| Block | Indices | Dimension |
|---|---|---:|
| position error | 0..2 | 3 |
| velocity error | 3..5 | 3 |
| attitude error | 6..8 | 3 |
| accel bias error | 9..11 | 3 |
| gyro bias error | 12..14 | 3 |
| altimeter bias error | 15 | 1 |
| radar translation extrinsic error | 16..18 | 3 |
| radar rotation extrinsic error | 19..21 | 3 |

The attitude and radar-orientation components are small-angle error states, not quaternion components.

### Radar Clone State

During a radar update the filter augments the covariance with one 18-dimensional clone:

$$
\delta \vec{x}_{clone} =
\begin{bmatrix}
\delta \vec{p}_c \\
\delta \vec{v}_c \\
\delta \vec{\theta}_c \\
\delta \vec{b}_{g,c} \\
\delta \vec{l}_{b r,c} \\
\delta \vec{\theta}_{b r,c}
\end{bmatrix}
\in \mathbb{R}^{18}
$$

This clone captures the radar-update linearization point without permanently enlarging the state.

---

## 3. Initialization Phase

Initialization is handled in `ekf_rio_adapter`, not inside the ROS node.

The runtime phases are:

1. `ACCUMULATING`: collect IMU samples for `T_init` seconds
2. `RUNNING`: initialize the filter once, then propagate and update normally

During initialization:

1. IMU samples are converted from FLU to NED
2. samples are stored as `ImuDataStamped`
3. once the elapsed time reaches `T_init`, `filter_.init(...)` is called

The filter initialization uses the averaged IMU batch:

$$
\bar{\vec{a}} = \frac{1}{N} \sum_{i=1}^{N} \vec{a}_i,
\qquad
\bar{\vec{\omega}} = \frac{1}{N} \sum_{i=1}^{N} \vec{\omega}_i
$$

Roll and pitch are initialized from the mean acceleration:

$$
\phi_0 = -\operatorname{atan2}(\bar{a}_y, -\bar{a}_z)
$$

$$
\theta_0 = \arcsin\!\left(\operatorname{clamp}\left(\frac{\bar{a}_x}{g_n}, -1, 1\right)\right)
$$

Yaw is taken from configuration:

$$
\psi_0 = \texttt{yaw\_0\_deg} \cdot \pi / 180
$$

If gyro calibration is enabled, the initial gyro bias is:

$$
\vec{b}_{g,0} = \bar{\vec{\omega}} + \vec{b}_{g,\text{cfg}}
$$

Otherwise the configured bias is used directly.

The initial covariance is built from the configured standard deviations for position, velocity, attitude, biases, and radar extrinsics.

---

## 4. Predict Step (IMU)

### Bias Correction

At each propagate step the incoming IMU sample is bias-corrected:

$$
\vec{a}_{corr} = \vec{a}_{meas} - \vec{b}_a
$$

$$
\vec{\omega}_{corr} = \vec{\omega}_{meas} - \vec{b}_g
$$

These corrected NED-frame inputs are passed into the strapdown propagator.

### Nominal Strapdown Propagation

The nominal state is propagated by `Strapdown::propagate(...)`.

#### Attitude

Attitude uses fourth-order Runge-Kutta integration of quaternion kinematics.

Define:

$$
\vec{\omega}_q = \begin{bmatrix} 0 & \omega_x & \omega_y & \omega_z \end{bmatrix}^T
$$

and the left quaternion multiplication matrix $Q_L(q)$. The code integrates:

$$
\dot{q}_{n b} = \frac{1}{2} Q_L(q_{n b}) \; \vec{\omega}_q
$$

with RK4.

#### Velocity

Velocity is propagated using a Simpson-style integration of the rotated specific force plus gravity:

$$
\vec{v}_{n b,k} = \vec{v}_{n b,k-1} + C_{n b,k-1} \; \vec{s}_l + \vec{g}_n \; \Delta t
$$

where in code:

$$
\vec{g}_n = \begin{bmatrix}0 \\ 0 \\ g_n\end{bmatrix}
$$

because the navigation frame is NED.

The local increment $\vec{s}_l$ is computed from the current body acceleration and the relative body rotation over the interval.

#### Position

Position is propagated by chained Simpson integration:

$$
\vec{p}_{n b,k} = \vec{p}_{n b,k-1} + \vec{v}_{n b,k-1} \Delta t + C_{n b,k-1} \; \vec{y}_l + \frac{1}{2} \vec{g}_n \Delta t^2
$$

This is more accurate than the old UKF's simple timer-side integration and is one reason the EKF swap was useful diagnostically.

### Error-State Covariance Propagation

After nominal propagation, the covariance is propagated as:

$$
P_k^- = \Phi_k P_{k-1}^+ \Phi_k^T + G_k Q_k G_k^T
$$

The state-transition approximation is first-order:

$$
\Phi_k = I + F_k \Delta t
$$

with non-zero continuous-time Jacobian blocks:

$$
F_{p,v} = I_3
$$

$$
F_{v,\theta} = -\operatorname{skew}(C_{n b} \; \vec{a}_{corr})
$$

$$
F_{v,b_a} = -C_{n b}
$$

$$
F_{\theta,b_g} = -C_{n b}
$$

The 13-dimensional process noise in the actual implementation is:

$$
\vec{w} =
\begin{bmatrix}
\vec{n}_a \\
\vec{n}_\omega \\
\vec{n}_{b_a} \\
\vec{n}_{b_g} \\
n_{b_{alt}}
\end{bmatrix}
\in \mathbb{R}^{13}
$$

This is different from the old UKF document because there is no gravity-state process noise here.

The implemented noise mapping is:

$$
G =
\begin{bmatrix}
0 & 0 & 0 & 0 & 0 \\
C_{n b} & 0 & 0 & 0 & 0 \\
0 & C_{n b} & 0 & 0 & 0 \\
0 & 0 & I_3 & 0 & 0 \\
0 & 0 & 0 & I_3 & 0 \\
0 & 0 & 0 & 0 & 1 \\
0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0
\end{bmatrix}
$$

written here blockwise in the same order as the 22-state error vector.

The discrete noise covariance produced by `SystemNoisePsd::getQ(T)` is diagonal:

$$
Q_k = \operatorname{diag}
\left(
\sigma_a^2 T,
\sigma_\omega^2 T,
\sigma_{b_a}^2 / T,
\sigma_{b_g}^2 / T,
\sigma_{b_{alt}}^2 / T
\right)
$$

using 3-axis blocks where appropriate.

This is the exact implemented form, even if a continuous-time derivation might be presented differently elsewhere.

---

## 5. Radar Update

### Scan-Level Front End in This Repository

In this repository, the radar update path is not just the original Doer EKF measurement step.

Before the EKF update, the repository-local `RadarEgoVelocityEstimator` produces:

1. one scan-level velocity estimate
2. one scan-level covariance proxy expressed as `sigma_radar`
3. debug statistics such as valid point count and condition number
4. an optional zero-velocity result

The adapter then passes that estimate into `EkfRioFilter::updateRadarEgoVelocity(...)`.

So the EKF core still performs the radar measurement update, but the measurement generation pipeline is a Liberaware extension around the original filter.

### Clone Augmentation

Before a radar update, the filter augments the covariance with one radar clone. The augmentation matrix is built so the clone copies:

1. position
2. velocity
3. attitude
4. gyro bias
5. radar translation extrinsics
6. radar rotation extrinsics

The clone state stores:

1. the current navigation solution
2. the current gyro bias
3. the current radar extrinsics
4. the update timestamps

After the radar update, the clone is removed and the covariance is truncated back to the 22-state base system.

### Measurement Model

The EKF radar measurement is the radar-frame ego velocity.

Let:

$$
C_{n b} = \text{body-to-navigation rotation from the clone}
$$

$$
C_{b r} = \text{radar-to-body rotation extrinsic}
$$

$$
\vec{l}_{b r} = \text{radar position extrinsic in body frame}
$$

$$
\vec{v}_{n b} = \text{body linear velocity in navigation frame}
$$

$$
\vec{w} = \text{angular velocity input passed into the update}
$$

The rotational velocity contribution at the radar due to lever arm is:

$$
\vec{v}_w = \operatorname{skew}(\vec{w} - \vec{b}_{g,c}) \; \vec{l}_{b r}
$$

The body-frame translational velocity is:

$$
\vec{v}_b = C_{n b}^T \; \vec{v}_{n b}
$$

The predicted radar-frame velocity is then:

$$
\hat{\vec{v}}_r = C_{b r}^T (\vec{v}_w + \vec{v}_b)
$$

This is exactly the quantity called `v_r_filter` in the code.

The Jacobian blocks placed into the measurement matrix are:

$$
H_v = C_{b r}^T C_{n b}^T
$$

$$
H_q = C_{b r}^T C_{n b}^T \operatorname{skew}(\vec{v}_{n b})
$$

$$
H_{b_g} = - C_{b r}^T \operatorname{skew}(\vec{l}_{b r})
$$

$$
H_{l_{b r}} = C_{b r}^T \operatorname{skew}(\vec{w})
$$

$$
H_{q_{b r}} = C_{b r}^T \operatorname{skew}(\vec{v}_w + \vec{v}_b)
$$

These populate the clone-related portions of the full measurement Jacobian.

### Innovation and Gating

The innovation is formed as:

$$
\vec{r} = \hat{\vec{v}}_r - \vec{v}_{r,meas}
$$

The measurement covariance is diagonal:

$$
R = \operatorname{diag}(\sigma_{v_r,x}^2, \sigma_{v_r,y}^2, \sigma_{v_r,z}^2)
$$

If radar outlier rejection is enabled, the implementation computes:

$$
\gamma = \vec{r}^T (H P H^T + R)^{-1} \vec{r}
$$

and compares it against a 3-DOF chi-squared threshold.

In the current repository configuration, `radar_outlier_rejection: 0.0`, so this Mahalanobis gate is disabled by default and successful front-end estimates are fused directly.

### Nominal-State Correction

The Kalman correction uses:

$$
K = P H^T (H P H^T + R)^{-1}
$$

$$
\delta \vec{x} = K \vec{r}
$$

The implemented nominal-state correction subtracts the error-state increment:

$$
\vec{p} \leftarrow \vec{p} - \delta \vec{p}
$$

$$
\vec{v} \leftarrow \vec{v} - \delta \vec{v}
$$

$$
\vec{b}_a \leftarrow \vec{b}_a - \delta \vec{b}_a,
\qquad
\vec{b}_g \leftarrow \vec{b}_g - \delta \vec{b}_g
$$

The attitude update uses the small-angle quaternion correction:

$$
q \leftarrow \delta q(\delta \vec{\theta}) \otimes q
$$

with

$$
\delta q(\delta \vec{\theta}) =
\left(1, -\frac{1}{2}\delta\theta_x, -\frac{1}{2}\delta\theta_y, -\frac{1}{2}\delta\theta_z\right)
$$

The same pattern is used for the radar extrinsic rotation correction.

---

## 6. Altimeter Update

The filter still contains an altimeter update path:

$$
r_h = h_{filter} - h_{meas}
$$

with measurement sensitivity to vertical position and altimeter bias.

In the current Liberaware setup this path is not used, but the bias state and equations remain in the filter core.

---

## 7. Adapter Boundary and Published Output

The adapter exposes two main public runtime interfaces:

1. `update_imu(...)`
2. `update_radar_scan(...)`

Published state output comes from `get_nav_state(...)`, which returns:

1. position from the EKF nominal state
2. velocity from the EKF nominal state in the external world FLU frame
3. quaternion converted from NED to FLU
4. gravity as the fixed configured scalar `g_n`
5. biases converted back to FLU

`rio_ros.cpp` then uses that world-FLU velocity to build the published ROS messages. In particular, `callback_timer_()` rotates the world-FLU velocity into body-FLU before assigning it to `nav_msgs/Odometry.twist.twist.linear`, while the odometry pose remains in the world FLU frame. So `/nav/odometry` and `/nav/odometry_fc` carry:

1. pose in world FLU
2. twist linear velocity in body FLU
3. the same pose and twist payload copied into `OdometryFcStamped`, plus `fc_boot_time_usec`

This means the ROS outputs now reflect the EKF's own position estimate, not a separate timer-side integration layer.

---

## 8. Configuration Summary

The canonical runtime configuration is:

`humble_ws/src/rio_ros/ekf_rio_lib/config/ekf_rio_param.yaml`

Important current settings include:

1. `T_init: 3.0`
2. `g_n: 9.81`
3. `calib_gyro: true`
4. radar extrinsics enabled
5. radar front-end parameters for range filtering, zero-velocity detection, sigma offsets, and IMU buffering
6. `radar_outlier_rejection: 0.0`, which disables the EKF Mahalanobis radar gate by default in the current test setup

---

## 9. Same vs Different From The Original Doer Implementation

### Same

The following core elements remain structurally the same as the original Doer EKF-RIO implementation:

1. internal NED convention inside the EKF core
2. error-state EKF structure with a 22-state base covariance
3. strapdown-style nominal propagation
4. RK4 quaternion propagation and Simpson-style translational integration in the strapdown
5. first-order covariance propagation using $\Phi P \Phi^T + G Q G^T$
6. radar clone augmentation and truncation around the radar update
7. radar ego-velocity measurement model inside the EKF core
8. small-angle quaternion correction for nominal-state updates

### Different In This Repository

The current implementation differs from the original Doer ROS1 stack in several important ways.

#### Runtime and packaging differences

1. the filter is ported to ROS2 and made ROS-agnostic at the library layer
2. `ros::Time` was replaced with `double` timestamps
3. ROS1 message, `tf2`, and `angles` dependencies were removed from the filter library
4. configuration is loaded from a plain YAML-backed `EkfRioConfig` struct instead of dynamic reconfigure
5. startup is fail-fast if the EKF YAML cannot be found or parsed

#### Adapter boundary differences

1. an explicit adapter performs FLU to NED conversion at input and NED to FLU conversion at output
2. the adapter implements the initialization phase and thread-safety boundary for ROS callbacks
3. the adapter publishes EKF-derived position, velocity, and attitude to the rest of the ROS stack

#### Radar path differences

1. this repository adds a dedicated scan-level `RadarEgoVelocityEstimator` in front of the EKF measurement update
2. the estimator adds range filtering, zero-velocity handling, covariance reporting, and runtime debug statistics
3. the current runtime is intentionally in a simplified LSQ-first mode, so several earlier estimator-side rejection gates are disabled
4. the adapter now computes a buffered, time-weighted IMU gyro average for each radar update instead of always using only the freshest gyro sample
5. the repository publishes a dedicated radar estimator debug topic and extra runtime logs to separate producer health from EKF fusion behavior

#### Configuration and defaults differences

1. barometer updates are effectively unused in the current runtime
2. the current default radar Mahalanobis gate is disabled by configuration
3. the active radar front-end and timing behavior therefore reflects the repository's current integration strategy, not a verbatim reproduction of the original Doer runtime stack

### Practical Interpretation

So the best way to think about the current code is:

1. the EKF core math is still fundamentally Doer-style EKF-RIO
2. the adapter, configuration, ROS integration, and radar measurement generation are Liberaware-specific
3. this repository therefore preserves the original filter formulation while intentionally changing the runtime boundary conditions around it