# RIO System: Complete Data Flow & Configuration Guide

This document describes the code that is actually active in the current repository.

It is intentionally aligned with the current EKF-RIO runtime in `humble_ws/src/rio_ros/ekf_rio_lib/` and with the current ArduPilot bridge in `humble_ws/src/fc_com_if/`.

Two important scope notes:

1. The active estimator path is the EKF adapter, not the old UKF path.
2. The current hover objective uses companion-computer estimates only. It does not rely on companion-computer setpoint commands.

---

## Table of Contents

1. [Current Runtime Entry Point](#1-current-runtime-entry-point)
2. [Branch A — Flight Controller to EKF-RIO IMU Path](#2-branch-a--flight-controller-to-ekf-rio-imu-path)
3. [Branch B — Radar to EKF-RIO Update Path](#3-branch-b--radar-to-ekf-rio-update-path)
4. [Branch C — EKF-RIO Outputs and Published Frames](#4-branch-c--ekf-rio-outputs-and-published-frames)
5. [Branch D — Feedback Loop Back to the Flight Controller](#5-branch-d--feedback-loop-back-to-the-flight-controller)
6. [Offline Replay Status](#6-offline-replay-status)
7. [Configuration Quick Reference](#7-configuration-quick-reference)

---

## 1. Current Runtime Entry Point

The current live-hardware launch for the EKF-RIO plus ArduPilot bridge path is:

`humble_ws/script/test_ekf_rio.launch.py`

That launch currently starts these components:

- `fc_com_if_node`
- `rio_ros_node`
- `ConfigParameterServer`
- `mmWaveCommSrv`
- `mmWaveQuickConfig`
- `ParameterParser`
- `DataHandlerClass`
- `radar_watchdog` when enabled
- `static_transform_publisher` for `body -> ti_mmwave_0`

Important current launch values in that file:

- FC UART path: `/dev/ttyS3`
- FC baudrate: `921600`
- EKF config path: `rio_ros/config/ekf_rio_param.yaml`
- Radar config file sent to the chip: `humble_ws/script/config/param_sbir.cfg`
- Radar data port: `/dev/ttyUSB1`
- Radar command port: `/dev/ttyUSB0`
- Radar timing instrumentation: enabled
- Radar watchdog: enabled

There are older or alternate launch files in the repository, including `test_rio.launch.py`, but the current validated EKF-RIO plus EXTNAV path is represented by `test_ekf_rio.launch.py`.

---

## 2. Branch A — Flight Controller to EKF-RIO IMU Path

This is the high-rate propagation path.

```text
Flight Controller
  -> MAVLink HIGHRES_IMU on UART
  -> fc_com_if
  -> /fc/imu
  -> rio_ros
  -> ekf_rio_adapter
  -> EkfRioFilter::propagate(...)
```

### 2.1 MAVLink receive and ROS publication

Relevant files:

- `humble_ws/src/fc_com_if/src/drivers/serial_device.cpp`
- `humble_ws/src/fc_com_if/src/drivers/mavlink_control.cpp`
- `humble_ws/src/fc_com_if/src/fc_com_if.cpp`

What happens:

1. `serial_device.cpp` reads MAVLink bytes from `/dev/ttyS3`.
2. `mavlink_control.cpp` parses frames.
3. `fc_com_if::callback_mavlink_message()` handles `HIGHRES_IMU`.
4. `fc_com_if` publishes `lw_msgs/msg/Imu` on `/fc/imu`.

Important current behavior:

- The active IMU path uses `HIGHRES_IMU`.
- `fc_com_if` re-requests `HIGHRES_IMU` with `MAV_CMD_SET_MESSAGE_INTERVAL` if the stream goes stale.
- `fc_com_if` converts the incoming FC body convention to FLU by negating Y and Z before publishing `/fc/imu`.

Currently disabled in `fc_com_if`:

- `/fc/baro`
- `/fc/battery`
- `/fc/fc_status`

Those publishers are present only as commented-out code and are not part of the current runtime.

### 2.2 IMU ingestion into EKF-RIO

Relevant files:

- `humble_ws/src/rio_ros/src/rio_ros.cpp`
- `humble_ws/src/rio_ros/ekf_rio_lib/include/ekf_rio_lib/ekf_rio_adapter.h`
- `humble_ws/src/rio_ros/ekf_rio_lib/src/ekf_rio_adapter.cpp`
- `humble_ws/src/rio_ros/ekf_rio_lib/include/ekf_rio_lib/ekf_rio_filter.h`

What happens:

1. `rio_ros::callback_imu_()` receives `/fc/imu`.
2. It converts `time_usec` to seconds and checks `dt` sanity.
3. It forwards FLU acceleration and gyro to `ekf_rio_adapter::update_imu()`.
4. The adapter converts FLU to NED by negating Y and Z.
5. During the startup phase it accumulates IMU samples for stationary initialization.
6. After initialization it calls `EkfRioFilter::propagate(...)` for each IMU sample.

Important current facts:

- The active estimator object in `rio_ros.cpp` is `rio::ekf_rio_adapter`.
- The current default config loaded by `rio_ros` is `ekf_rio_param.yaml`.
- The current configured initialization time in that file is `T_init: 5.0`.

Internal frame convention:

- inside `EkfRioFilter`: NED
- outside the adapter: FLU

This matches `docs/rio_ekf.md`.

---

## 3. Branch B — Radar to EKF-RIO Update Path

This is the lower-rate measurement-update path.

```text
TI mmWave radar
  -> DataHandlerClass
  -> /ti_mmwave/radar_scan_pcl
  -> rio_ros::callback_radar_scan_()
  -> ekf_rio_adapter::update_radar_scan()
  -> RadarEgoVelocityEstimator
  -> EkfRioFilter::updateRadarEgoVelocity(...)
```

### 3.1 Radar driver side

Relevant files:

- `humble_ws/src/mmwave_ti_ros/ros2_driver/src/ti_mmwave_rospkg/src/DataHandlerClass.cpp`
- `humble_ws/src/mmwave_ti_ros/ros2_driver/src/ti_mmwave_rospkg/config/global_params.yaml`

What happens:

1. `DataHandlerClass` reads TI radar TLV frames from `/dev/ttyUSB1`.
2. It parses detections and publishes:
   - `/ti_mmwave/radar_scan_pcl`
   - radar timing and health logs
   - radar CPU load topic consumed by `rio_ros`
3. The active launch also enables timing instrumentation and watchdog-based recovery.

### 3.2 Conversion inside `rio_ros`

Relevant file:

- `humble_ws/src/rio_ros/src/rio_ros.cpp`

What happens in `callback_radar_scan_()`:

1. The incoming `sensor_msgs/PointCloud2` is converted to PCL.
2. The callback inspects the field names.
3. It normalizes either TI-format points or Doer-format points into `RadarPointCloudType`.
4. It calls `ekf_rio_adapter::update_radar_scan(time_sec, scan)`.
5. It publishes `/nav/radar_estimator_debug` with scan-level debug info.

### 3.3 What the adapter actually does

Relevant files:

- `humble_ws/src/rio_ros/ekf_rio_lib/src/ekf_rio_adapter.cpp`
- `humble_ws/src/rio_ros/ekf_rio_lib/include/ekf_rio_lib/ekf_rio_adapter.h`

This is the most important correction relative to older documentation:

The current EKF path does not directly perform a per-point UKF update from the raw point cloud.

What it does instead:

1. `RadarEgoVelocityEstimator` fits one scan-level radar ego velocity.
2. That front end returns:
   - `velocity_radar`
   - `sigma_radar`
   - `raw_point_count`
   - `valid_point_count`
   - zero-velocity detection status
   - condition number and other debug fields
3. The adapter computes a buffered gyro average over recent IMU history.
4. The EKF adds a radar clone state.
5. The EKF calls `updateRadarEgoVelocity(...)` once per accepted scan-level solve.
6. The clone is removed after the update.

This matches `docs/rio_ekf.md` and is the correct description of the current code.

---

## 4. Branch C — EKF-RIO Outputs and Published Frames

This section is the one that must stay consistent with `docs/rio_ekf.md`.

```text
EkfRioFilter internal state (NED)
  -> ekf_rio_adapter::get_nav_state() converts to world FLU
  -> rio_ros::callback_timer_()
  -> /nav/odometry
  -> /nav/odometry_fc
  -> /nav/pose
  -> /nav/path
  -> /nav/rio_debug
```

### 4.1 Timer rate

Relevant file:

- `humble_ws/src/rio_ros/src/rio_ros.cpp`

The timer is created at 10 ms, so the publication loop runs at 100 Hz.

### 4.2 Adapter output frame

Relevant files:

- `humble_ws/src/rio_ros/ekf_rio_lib/include/ekf_rio_lib/ekf_rio_adapter.h`
- `humble_ws/src/rio_ros/ekf_rio_lib/src/ekf_rio_adapter.cpp`

`ekf_rio_adapter::get_nav_state(...)` returns:

- position in world FLU
- velocity in world FLU
- quaternion body -> world FLU
- biases in FLU

That is exactly what `docs/rio_ekf.md` states.

### 4.3 `rio_ros` odometry publication

Relevant file:

- `humble_ws/src/rio_ros/src/rio_ros.cpp`

`rio_ros::callback_timer_()` does one extra step before publishing odometry:

1. It reads position and velocity in world FLU from the adapter.
2. It computes `vel_body = quat.inverse() * vel`.
3. It publishes:
   - pose in world FLU
   - twist.linear in body FLU

So the published odometry semantics are:

- `odometry.pose.pose.position`: world FLU in frame `/odm`
- `odometry.pose.pose.orientation`: body -> world FLU
- `odometry.twist.twist.linear`: body FLU in child frame `/body`

This is the key point that resolves the frame confusion:

- the filter does not publish body velocity directly
- the adapter returns world-FLU velocity
- `rio_ros` rotates that world-FLU velocity into body-FLU before placing it into `nav_msgs/Odometry.twist`

### 4.4 `/nav/odometry_fc`

`lw_msgs/msg/OdometryFcStamped` contains:

- `odometry`: exactly the same odometry payload already published on `/nav/odometry`
- `fc_boot_time_usec`: the estimator state timestamp in the FC boot-clock domain

Therefore, in the current code:

- `/nav/odometry_fc.odometry.twist.twist.linear` is also body-FLU
- it is not NED
- it is not world-frame velocity

That is consistent with both `rio_ekf.md` and `fc_com_if.cpp`.

### 4.5 Other outputs

Currently published from `rio_ros`:

- `/nav/odometry`
- `/nav/odometry_fc`
- `/nav/pose`
- `/nav/path`
- `/nav/rio_debug`
- `/nav/radar_estimator_debug`

Important correction relative to older docs:

- `pub_landed_status_` exists in the constructor, but `rio_ros.cpp` does not currently fill or publish any `LandedStatus` message in `callback_timer_()`.
- So `/nav/landed_status` should be treated as currently unused in the active EKF-RIO path.

---

## 5. Branch D — Feedback Loop Back to the Flight Controller

This is the external-navigation feedback path used by the current project objective.

```text
/nav/odometry_fc
  -> fc_com_if::callback_odometry()
  -> body-FLU velocity -> body-FRD -> NED
  -> FLU world position -> NED
  -> VISION_SPEED_ESTIMATE
  -> VISION_POSITION_ESTIMATE
  -> ArduPilot EKF3
```

Relevant files:

- `humble_ws/src/fc_com_if/src/fc_com_if.cpp`
- `humble_ws/src/lw_msgs/msg/OdometryFcStamped.msg`

### 5.1 Velocity path

`fc_com_if::callback_odometry()` explicitly treats `odometry.twist.twist.linear` as body-FLU:

1. read body-FLU velocity from `odometry.twist.twist.linear`
2. convert FLU -> FRD by negating Y and Z
3. obtain body -> NED rotation
   - prefer fresh FC `ATTITUDE_QUATERNION`
   - fall back to odometry pose quaternion if FC attitude is stale
4. rotate the FRD body velocity into NED
5. publish `VISION_SPEED_ESTIMATE`

The covariance path follows the same logic:

1. extract the linear 3x3 covariance block from odometry twist covariance
2. convert covariance FLU -> FRD
3. rotate covariance into NED
4. place it into the MAVLink VSE covariance array

### 5.2 Position and attitude path

For `VISION_POSITION_ESTIMATE`, `fc_com_if`:

1. reads position from odometry pose in world FLU
2. converts position FLU -> NED as `(x, -y, -z)`
3. converts quaternion body -> FLU world into body -> NED by conjugation with `R_x(pi)`
4. converts that quaternion to roll, pitch, yaw
5. publishes `VISION_POSITION_ESTIMATE`

### 5.3 Timestamp path

Both VSE and VPE use:

- `msg.fc_boot_time_usec`

as the outgoing MAVLink timestamp.

This is the current wrapper design used to keep the EXTNAV timestamps in the FC time domain.

### 5.4 About companion-computer setpoints

`fc_com_if` still contains a `/guide/setpoint` subscription and a `callback_setpoint()` implementation that sends `SET_POSITION_TARGET_LOCAL_NED`.

That path exists in the codebase, but it is not part of the current hover objective and it is not the path described by this document. The current project goal is estimate-only feedback from the companion computer, with the pilot flying via RC.

### 5.5 Landed-status path

Another important correction relative to older docs:

- `fc_com_if` subscribes to `/nav/landed_status`
- but `callback_landed_status()` is intentionally a no-op
- it does not send `LAND_COMPLETE`

So the landed-status path is currently inactive on both the `rio_ros` publishing side and the `fc_com_if` consumption side.

---

## 6. Offline Replay Status

The checked-in file `humble_ws/script/test_bagplay.launch.py` is not a maintained replay launch for the current EKF-RIO pipeline.

It currently launches `navigation_node` and plays GNSS/VRPN-related topics, not the active `rio_ros` plus radar pipeline.

So the correct statement for the current repository is:

- there is no maintained offline replay launch file for the current EKF-RIO plus radar EXTNAV path
- if replay is needed, use a dedicated manual `ros2 bag play` flow or create a fresh EKF-RIO replay launch

Do not treat `test_bagplay.launch.py` as the current replay entry point for the EKF-RIO integration.

---

## 7. Configuration Quick Reference

This section is restricted to the configuration that matches the active EKF-RIO codepath.

### Startup / live launch

| If you want to change... | Edit this file | Current note |
|---|---|---|
| Live EKF-RIO launch | `humble_ws/script/test_ekf_rio.launch.py` | Current validated launch |
| FC serial device | `humble_ws/script/test_ekf_rio.launch.py` | `uart_device_path: /dev/ttyS3` |
| FC serial baudrate | `humble_ws/script/test_ekf_rio.launch.py` | `baudrate: 921600` |
| EKF config path | `humble_ws/script/test_ekf_rio.launch.py` | points to `rio_ros/config/ekf_rio_param.yaml` |
| Radar watchdog enable/timeouts | `humble_ws/script/test_ekf_rio.launch.py` | `radar_watchdog_*` parameters |
| Vision publish toggles | `humble_ws/script/test_ekf_rio.launch.py` | `publish_vision_speed_estimate`, `publish_vision_position_estimate` |
| FC-attitude usage for VSE | `humble_ws/script/test_ekf_rio.launch.py` | `use_fc_attitude_for_vision_speed_estimate` |

### Radar chip behavior

| If you want to change... | Edit this file | Current note |
|---|---|---|
| Radar CLI config sent at startup | `humble_ws/script/config/param_sbir.cfg` | current launch uses this file |
| Temporary overload mitigation | `humble_ws/script/config/param_sbir.cfg` | higher CFAR threshold currently in use |
| Radar command/data ports | `humble_ws/script/test_ekf_rio.launch.py` | `/dev/ttyUSB0` and `/dev/ttyUSB1` |
| Radar timing logs | `humble_ws/script/test_ekf_rio.launch.py` | `timing_instrumentation_enabled`, `timing_log_period_frames` |

### Radar ROS driver

| If you want to change... | Edit this file | Current note |
|---|---|---|
| Shared radar ROS parameters | `humble_ws/src/mmwave_ti_ros/ros2_driver/src/ti_mmwave_rospkg/config/global_params.yaml` | loaded by `ConfigParameterServer` |
| DataHandlerClass publish toggles | `humble_ws/script/test_ekf_rio.launch.py` | `publish_scan_pcl`, `publish_scan_points`, etc. |
| Radar frame id | `humble_ws/script/test_ekf_rio.launch.py` | `frame_id: ti_mmwave_0` |

### EKF-RIO tuning

Canonical config file:

- `humble_ws/src/rio_ros/ekf_rio_lib/config/ekf_rio_param.yaml`

| If you want to change... | YAML key |
|---|---|
| Stationary init duration | `T_init` |
| Gravity magnitude | `g_n` |
| Initial yaw | `yaw_0_deg` |
| Radar lever arm | `l_b_r_x`, `l_b_r_y`, `l_b_r_z` |
| Radar orientation extrinsic | `q_b_r_w`, `q_b_r_x`, `q_b_r_y`, `q_b_r_z` |
| Per-point radar Doppler sigma | `radar_noise_sigma_doppler_v` |
| Radar min/max range | `radar_min_range`, `radar_max_range` |
| Minimum valid targets per solve | `radar_min_valid_targets` |
| Zero-velocity thresholds | `radar_zero_velocity_*` |
| Condition-number rejection | `radar_max_condition_number` |
| Sigma floors and maxima | `radar_sigma_offset_*`, `radar_max_sigma_*` |
| IMU buffering for radar update | `radar_imu_history_sec`, `radar_imu_averaging_window_sec`, `radar_imu_max_averaging_window_sec` |
| EKF radar outlier gate | `radar_outlier_rejection` |
| Initial covariance | `sigma_*` fields |
| Process-noise PSDs | `noise_psd_*` fields |

For the EKF math and frame conventions behind these fields, use `docs/rio_ekf.md`.

### Message definitions actually relevant to the current path

| Message | File |
|---|---|
| IMU from FC to `rio_ros` | `humble_ws/src/lw_msgs/msg/Imu.msg` |
| Odometry plus FC timestamp | `humble_ws/src/lw_msgs/msg/OdometryFcStamped.msg` |
| Main debug topic | `humble_ws/src/lw_msgs/msg/RioDebug.msg` |
| Radar estimator debug topic | `humble_ws/src/lw_msgs/msg/RadarEstimatorDebug.msg` |
| Landed status type | `humble_ws/src/lw_msgs/msg/LandedStatus.msg` |

### Legacy or currently inactive items

These exist in the repository but should not be treated as part of the current active EKF-RIO EXTNAV path without re-validation:

- `humble_ws/script/test_rio.launch.py`
- `humble_ws/script/test_bagplay.launch.py`
- `humble_ws/src/rio_ros/ukf_rio/`
- `fc_com_if` setpoint path via `/guide/setpoint`
- landed-status feedback to the FC
