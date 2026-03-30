# ArduPilot Copter 4.6.3 EKF3 Data Flow and File Relationships

## Scope
This document describes how EKF3 works in this workspace for ArduCopter, and how the main EKF3-related files connect from sensor input to flight-mode/failsafe decisions.

Focus areas:
- `libraries/AP_NavEKF3/*` (EKF3 implementation)
- `libraries/AP_NavEKF/*` shared helpers (`AP_NavEKF_Source`, buffers, common core scratch space)
- `libraries/AP_AHRS/*` (runtime bridge between EKF3 and vehicle code)
- `libraries/AP_InertialNav/*` (position/velocity wrapper used by Copter)
- ArduCopter consumers (`ekf_check.cpp`, `system.cpp`, `inertia.cpp`, plus parameter hookup)

## High-Level Architecture
1. Sensor producers update HAL/DAL objects (IMU, GPS, baro, mag, rangefinder, optical flow, airspeed, visual odometry, beacons).
2. `AP_AHRS::update()` calls `update_EKF3()` each loop.
3. `NavEKF3::UpdateFilter()` runs all EKF3 lanes (cores), each lane runs `NavEKF3_core::UpdateFilter()`.
4. Each core:
- reads sensor data,
- time-aligns data to a delayed fusion horizon using ring buffers,
- performs predict + fusion steps,
- pushes corrected outputs forward to output horizon.
5. `AP_AHRS` exposes EKF3 outputs through generic getters (`get_location`, `get_velocity_NED`, `get_filter_status`, `get_variances`, etc.).
6. ArduCopter uses those getters for navigation validity checks, failsafe decisions, and control inputs.

## File Relationship Map

### 1) Source selection and sensor prerequisites
- `libraries/AP_NavEKF/AP_NavEKF_Source.h/.cpp`
- Owns EKF source-set parameters (`EK3_SRC1/2/3_*`), active source set, and pre-arm source validation.
- Determines which sources are considered valid for `POSXY`, `VELXY`, `POSZ`, `VELZ`, `YAW`.
- Used by EKF3 front-end (`NavEKF3::sources`) and core logic (`setYawSource`, fusion selectors).

### 2) EKF3 front-end (multi-core manager)
- `libraries/AP_NavEKF3/AP_NavEKF3.h/.cpp`
- Owns parameters (`EK3_*`), allocates/configures cores, runs lane updates, handles lane switching and primary lane selection.
- Broadcasts external measurement writes to all cores (`writeOptFlowMeas`, `writeExtNavData`, `writeBodyFrameOdom`, etc.).
- Exposes primary-lane outputs to AHRS.

### 3) EKF3 core and common math scaffolding
- `libraries/AP_NavEKF3/AP_NavEKF3_core.h/.cpp`
- State definition, covariance, prediction, reset logic, output predictor, timing/buffer setup.
- `NavEKF3_core::UpdateFilter()` is the per-lane orchestrator.
- `libraries/AP_NavEKF/AP_NavEKF_core_common.h/.cpp`
- Shared static scratch matrices/vectors for EKF2/EKF3 (`KH`, `KHP`, `nextP`, `Kfusion`).

### 4) EKF3 measurement ingestion
- `libraries/AP_NavEKF3/AP_NavEKF3_Measurements.cpp`
- `readIMUData`, `readGpsData`, `readBaroData`, `readMagData`, `readAirSpdData`, `readRangeFinder`, `readRngBcnData`.
- Converts sensor time domains to EKF delayed horizon and pushes into EKF ring buffers.
- Handles external measurement writes: optical flow, ext-nav pose/vel, wheel/body odom, euler yaw, default airspeed.

### 5) EKF3 fusion modules
- `AP_NavEKF3_Control.cpp`: mode transitions, aiding mode, source/yaw selection, wind/mag state activation, GSF yaw hooks.
- `AP_NavEKF3_PosVelFusion.cpp`: GPS/extnav/height/body-velocity fusion, position/velocity resets, height source selection.
- `AP_NavEKF3_MagFusion.cpp`: compass/external-yaw/GSF yaw fusion and yaw reset logic.
- `AP_NavEKF3_OptFlowFusion.cpp`: optical-flow fusion + terrain offset estimator.
- `AP_NavEKF3_AirDataFusion.cpp`: TAS, sideslip, drag fusion.
- `AP_NavEKF3_RngBcnFusion.cpp`: range-beacon fusion.
- `AP_NavEKF3_VehicleStatus.cpp`: flight detection + GPS quality checks used by fusion/control logic.

### 6) EKF3 outputs and observability
- `AP_NavEKF3_Outputs.cpp`: health, error score, position/velocity/attitude getters, status/variance/innovation exports, reset event reporting, MAVLink EKF status.
- `AP_NavEKF3_Logging.cpp` + `LogStructure.h`: `XKF*` logs for internal states/innovations/selection/timing.

### 7) EKF3 feature gating
- `AP_NavEKF3_feature.h`
- Compile-time feature enablement (optflow fusion, external nav, drag fusion, beacon, etc.) based on board/build.

### 8) AHRS integration layer
- `libraries/AP_AHRS/AP_AHRS.h/.cpp`
- Owns `NavEKF3 EKF3`.
- Starts EKF3 (`InitialiseFilter`), updates each loop (`UpdateFilter`), copies active EKF3 attitude/bias outputs to canonical AHRS state.
- Exposes EKF3 outputs through estimator-agnostic methods used by vehicles.
- Provides hooks for lane switch and yaw reset (`check_lane_switch`, `request_yaw_reset`).

### 9) Inertial-nav wrapper used by Copter
- `libraries/AP_InertialNav/AP_InertialNav.cpp/.h`
- Pulls relative position/velocity from AHRS EKF interface and stores NEU cm values for vehicle code.
- Exposes `get_filter_status()` used heavily in Copter validity checks.

### 10) ArduCopter-specific consumers
- `ArduCopter/Parameters.cpp`
- Registers EKF3 parameter group as `EK3_` via `GOBJECTN(ahrs.EKF3, ...)`.
- `ArduCopter/inertia.cpp`
- Calls `inertial_nav.update()`, updates `current_loc` from AHRS/EKF outputs.
- `ArduCopter/system.cpp`
- `position_ok`, `ekf_has_absolute_position`, `ekf_has_relative_position`, `ekf_alt_ok` from EKF status flags.
- `ArduCopter/ekf_check.cpp`
- EKF variance monitoring, lane-switch/yaw-reset requests, EKF failsafe trigger/clear behavior.

## End-to-End Runtime Flow (Main Loop)

### A) Scheduler to EKF3
1. `AP_AHRS::update()` runs in the fast attitude loop.
2. `AP_AHRS::update_EKF3()`:
- starts EKF3 when startup criteria are met (`EKF3.InitialiseFilter()`),
- calls `EKF3.UpdateFilter()` every loop once started.

### B) EKF3 front-end lane handling
1. `NavEKF3::UpdateFilter()` iterates all configured cores (usually one per enabled IMU lane).
2. It passes a per-core `allow_state_prediction` budget decision.
3. Each core runs `NavEKF3_core::UpdateFilter(bool predict)`.
4. After updates, front-end may switch primary lane based on health + error score + alignment state.

### C) Per-core update internals
Inside `NavEKF3_core::UpdateFilter()`:
1. `controlFilterModes()` updates armed/flight/aiding/state-learning modes.
2. `readIMUData(predict)` downsamples IMU and drives delayed-horizon run gating.
3. If `runUpdates`:
- predict strapdown states (`UpdateStrapdownEquationsNED`),
- covariance predict (`CovariancePrediction`),
- run yaw-estimator prediction (GSF),
- run fusion selectors in sequence:
  - `SelectMagFusion()`
  - `SelectVelPosFusion()`
  - `runYawEstimatorCorrection()`
  - `SelectRngBcnFusion()` if enabled
  - `SelectFlowFusion()` if enabled
  - `SelectBodyOdomFusion()` if enabled
  - `SelectTasFusion()`
  - `SelectBetaDragFusion()`
- update filter status bits.
4. Always run `calcOutputStates()` to propagate corrected states to output horizon.

### D) Data buffering and delayed-horizon alignment
- EKF3 uses `EKF_IMU_buffer_t` and `EKF_obs_buffer_t` from `EKF_Buffer.h`.
- IMU data defines the timeline.
- Non-IMU sensors are timestamp-corrected for known delays and recalled when fusion horizon reaches them.
- This is the core mechanism that allows asynchronous/delayed sensors to be fused consistently.

## External Measurement Injection Paths
External writes enter through `AP_AHRS` and fan out to all EKF3 cores:
- `AP_AHRS::writeOptFlowMeas` -> `NavEKF3::writeOptFlowMeas` -> `NavEKF3_core::writeOptFlowMeas`
- `AP_AHRS::writeBodyFrameOdom` -> `NavEKF3::writeBodyFrameOdom` -> core buffer
- `AP_AHRS::writeExtNavData` / `writeExtNavVelData` -> `NavEKF3::*` -> per-core buffers
- `AP_AHRS::writeDefaultAirSpeed` -> `NavEKF3::writeDefaultAirSpeed`

These are then fused later by selector stages (`SelectFlowFusion`, `SelectBodyOdomFusion`, `SelectVelPosFusion`, `SelectTasFusion`).

## Outputs Path to Copter
1. EKF3 core publishes primary outputs via `NavEKF3` getter API.
2. `AP_AHRS` wraps these APIs and normalizes estimator access.
3. `AP_InertialNav` pulls relative position/velocity and filter status from AHRS.
4. ArduCopter uses:
- `inertial_nav.get_filter_status()` in `system.cpp` for `position_ok` / altitude validity.
- `ahrs.get_variances()` and `ahrs.get_innovations()` in `ekf_check.cpp` for failsafe and vibration logic.
- `ahrs.getLastYawResetAngle()` and primary-core index for reset/lane-switch handling in `check_ekf_reset()`.

## EKF3 Failure-Handling and Recovery Flow in Copter
`ArduCopter/ekf_check.cpp` (10 Hz):
1. Pulls EKF variances (`ahrs.get_variances`).
2. Applies low-pass filters to position/velocity/height variance.
3. If near-failure:
- requests yaw reset (`ahrs.request_yaw_reset()`),
- then requests lane switch (`ahrs.check_lane_switch()`).
4. If persistent bad state:
- triggers EKF failsafe action (`failsafe_ekf_event`), mode changes based on `FS_EKF_ACTION`.
5. If recovered:
- clears failsafe (`failsafe_ekf_off_event`).

This means Copter actively asks EKF3 to self-recover before hard failsafe.

## Practical Trace: Sensor -> EKF3 -> Copter Decision
1. IMU/GPS/Baro/Mag updated in DAL.
2. `AP_AHRS::update_EKF3()` calls `EKF3.UpdateFilter()`.
3. `NavEKF3_core::read*` ingests and buffers delayed measurements.
4. Fusion selectors update states and innovations.
5. `AP_AHRS::get_variances()` returns EKF3 normalized innovation metrics.
6. `Copter::ekf_check()` evaluates these metrics and can request yaw reset/lane switch/failsafe.

## Quick Reference: Primary Files by Responsibility
- EKF3 front-end manager: `libraries/AP_NavEKF3/AP_NavEKF3.cpp`
- EKF3 core predict/covariance/output horizon: `libraries/AP_NavEKF3/AP_NavEKF3_core.cpp`
- Sensor ingestion + buffering: `libraries/AP_NavEKF3/AP_NavEKF3_Measurements.cpp`
- Fusion mode control: `libraries/AP_NavEKF3/AP_NavEKF3_Control.cpp`
- Pos/vel/height fusion: `libraries/AP_NavEKF3/AP_NavEKF3_PosVelFusion.cpp`
- Mag/yaw fusion: `libraries/AP_NavEKF3/AP_NavEKF3_MagFusion.cpp`
- Optflow fusion: `libraries/AP_NavEKF3/AP_NavEKF3_OptFlowFusion.cpp`
- Air-data fusion: `libraries/AP_NavEKF3/AP_NavEKF3_AirDataFusion.cpp`
- Beacon fusion: `libraries/AP_NavEKF3/AP_NavEKF3_RngBcnFusion.cpp`
- Health/status/output getters: `libraries/AP_NavEKF3/AP_NavEKF3_Outputs.cpp`
- Logs: `libraries/AP_NavEKF3/AP_NavEKF3_Logging.cpp`
- Source-set config: `libraries/AP_NavEKF/AP_NavEKF_Source.cpp`
- AHRS bridge: `libraries/AP_AHRS/AP_AHRS.cpp`
- Copter failsafe/status consumption: `ArduCopter/ekf_check.cpp`, `ArduCopter/system.cpp`, `ArduCopter/inertia.cpp`

## Notes on “All EKF3-related files”
The strict implementation set is mostly under `libraries/AP_NavEKF3/` plus its shared dependencies (`AP_NavEKF_Source`, EKF buffers/common) and consumers (AHRS/Copter). This document focuses on those runtime-relevant paths rather than non-runtime artifacts (for example derivation scripts under `libraries/AP_NavEKF3/derivation/`).
