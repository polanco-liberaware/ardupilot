# ArduCopter IMU Data Flow

## Scope
This document traces the ArduCopter IMU path in this workspace from:

1. configuration and startup,
2. hardware/backend sample acquisition,
3. AP_InertialSensor processing,
4. DAL handoff,
5. EKF3 delayed-horizon consumption,
6. AHRS publication,
7. Copter controller use,
8. logging,
9. MAVLink output that Mission Planner can receive.

This document is based on code inspection in this repository only. It does not guess about Mission Planner internals. Where a statement is about what ArduPilot transmits rather than what Mission Planner chooses to show on a specific screen, that is stated explicitly.

## Verified Boundaries

### What is verified here
- Which files participate in the IMU path.
- Which transformations are applied to IMU data in ArduPilot.
- Which IMU-derived signals feed EKF3, AHRS, controllers, logs, and MAVLink.
- Which MAVLink stream groups and per-message senders are implemented in ArduCopter.

### What is not verified here
- Exact Mission Planner UI widget to MAVLink message mapping. That code is not in this repository.
- Which hardware IMU backend is active on a particular board at runtime. That depends on board and detected sensors.

## File Map

### Vehicle startup and runtime scheduling
- `ArduCopter/system.cpp`
  - `startup_INS_ground()` initializes AHRS, initializes INS, performs gyro warmup/calibration, then resets AHRS.
  - also sets `ins.set_log_raw_bit(MASK_LOG_IMU_RAW)`.
- `ArduCopter/Copter.cpp`
  - scheduler order is critical:
    - `AP_InertialSensor::update`
    - `run_rate_controller`
    - `motors_output`
    - `read_AHRS`
    - `read_inertia`
    - `update_flight_mode`
  - also drives IMU logging and vibration logging.
- `ArduCopter/Attitude.cpp`
  - `run_rate_controller()` runs the low-level rate controller using IMU-derived gyro feedback.
- `ArduCopter/inertia.cpp`
  - `read_inertia()` updates the inertial-nav wrapper from AHRS/EKF outputs.

### Common IMU front-end and processing
- `libraries/AP_InertialSensor/AP_InertialSensor.cpp`
  - owns INS parameters, backend startup, main sample timing, backend update fan-in, primary IMU selection, vibration/clipping tracking, notch allocation.
  - also contains the boot gyro calibration routine and the optional accel calibration/trim workflows.
- `libraries/AP_InertialSensor/AP_InertialSensor.h`
  - public accessors for accel, gyro, delta-angle, delta-velocity, health, vibration, clipping.
- `libraries/AP_InertialSensor/AP_InertialSensor_Backend.cpp`
  - common backend helper path for rotation, calibration, temperature correction, coning compensation, low-pass and harmonic notch filtering, delta-angle/delta-velocity accumulation, publish to front-end.
  - also applies saved gyro offsets and feeds accel calibration sample collection when accel calibration is active.
- `libraries/AP_InertialSensor/AP_InertialSensor_Params.cpp`
  - per-IMU parameter group for scale, offset, temperature calibration, and IMU position.
- `libraries/AP_InertialSensor/AP_InertialSensor_Logging.cpp`
  - `Write_IMU()`, `Write_Vibration()`, raw ACC/GYR logging, notch logs.
- `libraries/AP_InertialSensor/BatchSampler.cpp`
  - sensor-rate and pre/post-filter batch logging path.
- `libraries/AP_InertialSensor/AP_InertialSensor_tempcal.cpp`
  - temperature calibration support referenced by the common backend.

### Hardware-specific IMU backend branch
One or more of these backends is active depending on board and detected hardware. They all converge into `AP_InertialSensor_Backend` helpers.

- `libraries/AP_InertialSensor/AP_InertialSensor_Invensense.cpp`
- `libraries/AP_InertialSensor/AP_InertialSensor_Invensensev2.cpp`
- `libraries/AP_InertialSensor/AP_InertialSensor_Invensensev3.cpp`
- `libraries/AP_InertialSensor/AP_InertialSensor_BMI055.cpp`
- `libraries/AP_InertialSensor/AP_InertialSensor_BMI088.cpp`
- `libraries/AP_InertialSensor/AP_InertialSensor_BMI160.cpp`
- `libraries/AP_InertialSensor/AP_InertialSensor_BMI270.cpp`
- `libraries/AP_InertialSensor/AP_InertialSensor_ADIS1647x.cpp`
- `libraries/AP_InertialSensor/AP_InertialSensor_RST.cpp`
- `libraries/AP_InertialSensor/AP_InertialSensor_SCHA63T.cpp`
- `libraries/AP_InertialSensor/AP_InertialSensor_LSM9DS0.cpp`
- `libraries/AP_InertialSensor/AP_InertialSensor_LSM9DS1.cpp`
- `libraries/AP_InertialSensor/AP_InertialSensor_L3G4200D.cpp`
- `libraries/AP_InertialSensor/AP_InertialSensor_SITL.cpp`
- `libraries/AP_InertialSensor/AP_InertialSensor_NONE.cpp`
- `libraries/AP_InertialSensor/AP_InertialSensor_ExternalAHRS.cpp`

### DAL bridge used by EKF3
- `libraries/AP_DAL/AP_DAL.cpp`
  - starts per-frame DAL snapshots.
- `libraries/AP_DAL/AP_DAL_InertialSensor.cpp`
  - snapshots INS delta-angle, delta-velocity, usage flags, counts, loop timing, and IMU position offsets.
  - also creates a DAL-local 10 Hz low-pass filtered accel and gyro representation.
- `libraries/AP_DAL/AP_DAL_InertialSensor.h`
  - EKF-facing inertial sensor API.

### EKF3 consumption and output generation
- `libraries/AP_NavEKF3/AP_NavEKF3.cpp`
  - front-end multi-core manager; starts a DAL frame, timestamps it, updates all cores, chooses primary lane.
- `libraries/AP_NavEKF3/AP_NavEKF3_Measurements.cpp`
  - `readIMUData()` is the core EKF3 IMU ingestion path.
- `libraries/AP_NavEKF3/AP_NavEKF3_core.cpp`
  - per-core predict path, covariance prediction, delayed-to-output horizon propagation.
- `libraries/AP_NavEKF3/AP_NavEKF3_Outputs.cpp`
  - exports EKF3 orientation, gyro bias, accel bias, velocity, filter status, reset info.

### AHRS and control consumers
- `libraries/AP_AHRS/AP_AHRS.cpp`
  - runs EKF3, copies EKF3 outputs into canonical AHRS state.
- `libraries/AP_AHRS/AP_AHRS_Backend.cpp`
  - `get_gyro_latest()`, board orientation updates, centidegree sensor fields.
- `libraries/AP_AHRS/AP_AHRS_View.cpp`
  - rotated AHRS view used by controllers when needed.
- `libraries/AP_InertialNav/AP_InertialNav.cpp`
  - converts EKF outputs to NEU cm and fallback vertical-rate behavior.
- `libraries/AC_AttitudeControl/AC_AttitudeControl.cpp`
  - attitude error generation from AHRS quaternion and gyro.
- `libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.cpp`
  - low-level body-rate PID uses `get_gyro_latest()` and sends motor roll/pitch/yaw outputs.
- `libraries/AC_AttitudeControl/AC_PosControl.h`
  - vertical acceleration feedback uses `AHRS::get_accel_ef()`.

### Logging and telemetry outputs
- `ArduCopter/GCS_Mavlink.cpp`
  - Copter stream group definitions and `SRx_*` stream-rate parameters.
- `libraries/GCS_MAVLink/GCS_Common.cpp`
  - `send_raw_imu()`, `send_scaled_imu()`, `send_highres_imu()`, `send_attitude()`, `send_ahrs()`, `send_ahrs2()`, `send_vibration()`, per-message interval control, message dispatch.
- `libraries/GCS_MAVLink/GCS_Param.cpp`
  - legacy `REQUEST_DATA_STREAM` handling.

## Configuration and Startup Path

### Board and AHRS orientation
`libraries/AP_AHRS/AP_AHRS.cpp` defines `AHRS_ORIENTATION` as `AP_AHRS::ORIENTATION`.

`libraries/AP_AHRS/AP_AHRS_Backend.cpp::update_orientation()` pushes that orientation into:
- `AP::ins().set_board_orientation(orientation)`
- `AP::compass().set_board_orientation(orientation)`

This means board mounting rotation is not applied inside EKF3. It is pushed down into the IMU front-end before EKF3 consumes samples.

### INS parameters affecting the IMU path
The common INS object in `libraries/AP_InertialSensor/AP_InertialSensor.cpp` exposes the main front-end controls:
- `INS_GYRO_FILTER`
- `INS_ACCEL_FILTER`
- `INS_USE`, `INS_USE2`, `INS_USE3`
- `INS_STILL_THRESH`
- `INS_GYR_CAL`
- `INS_ACC_BODYFIX`
- `INS_POS1/2/3_*`
- `INS_FAST_SAMPLE`
- `INS_ENABLE_MASK`
- `INS_HNTCH_*`, `INS_HNTC2_*` when harmonic notch is enabled
- `INS_GYRO_RATE`
- temperature-calibration subgroups

Per-instance calibration and identification parameters live in `libraries/AP_InertialSensor/AP_InertialSensor_Params.cpp`:
- `INSx_ACCSCAL_*`
- `INSx_ACCOFFS_*`
- `INSx_GYROFFS_*`
- `INSx_POS_*`
- `INSx_ACC_CALTEMP`
- `INSx_GYR_CALTEMP`
- `INSx_USE`

EKF-side IMU selection is controlled separately by EKF3 parameters in `libraries/AP_NavEKF3/AP_NavEKF3.cpp`, especially `EK3_IMU_MASK`.

### Copter startup sequence
`ArduCopter/system.cpp::startup_INS_ground()` performs:
- `ahrs.init()`
- `ahrs.set_vehicle_class(AP_AHRS::VehicleClass::COPTER)`
- `ins.init(scheduler.get_loop_rate_hz())`
- `ahrs.reset()`

`AP_InertialSensor::init()` in `libraries/AP_InertialSensor/AP_InertialSensor.cpp` then:
- stores the loop rate and loop delta time,
- starts backends if needed,
- runs gyro calibration unless disabled,
- initializes batch logging,
- initializes FFT/notch ordering and notch allocations,
- enables temperature learning if configured.

### What the boot "warm-up" actually is
The startup comment in `ArduCopter/system.cpp` says "Warm up and calibrate gyro offsets", but in the normal Copter boot path this is primarily a gyro bias-calibration sequence, not a generic IMU warm-up stage applied equally to gyro and accel.

Relevant files for this startup path are:
- `ArduCopter/system.cpp`
  - enters the startup sequence and calls `ins.init(...)`.
- `libraries/AP_InertialSensor/AP_InertialSensor.cpp`
  - `init()` decides whether boot gyro calibration runs.
  - `init_gyro()` and `_init_gyro()` implement the actual gyro startup calibration.
  - `simple_accel_cal()`, `calibrate_trim()`, `acal_init()`, and `acal_update()` implement separate accel calibration workflows that are not part of the normal boot path.
- `libraries/AP_InertialSensor/AP_InertialSensor_Backend.cpp`
  - applies saved gyro offsets in the runtime correction path and supplies accel calibration samples when accel calibration is active.
- active backend files such as `AP_InertialSensor_BMI088.cpp`, `AP_InertialSensor_Invensensev3.cpp`, or other board-specific drivers
  - perform hardware reset, register setup, FIFO setup, and backend startup delays before front-end calibration begins.

### Gyro boot warm-up / calibration sequence
The boot gyro path is implemented by `AP_InertialSensor::_init_gyro()` in `libraries/AP_InertialSensor/AP_InertialSensor.cpp`.

What it does:
1. marks gyro calibration as active and sets notify flags so the vehicle is in an initializing state,
2. temporarily removes board rotation by setting orientation to `ROTATION_NONE`, so calibration is done in sensor frame,
3. clears existing gyro offsets,
4. performs a short priming phase of repeated `delay(5)` plus `update()` calls,
5. collects repeated gyro averages over short windows,
6. uses accel change during the averaging window as a motion detector and rejects samples if the vehicle moved,
7. checks whether consecutive gyro averages converge,
8. stores the new gyro offsets if convergence succeeds, or stores the best estimate found and marks calibration failed if it does not,
9. restores board orientation and saves the calibration.

So the practical meaning of gyro "warm-up" here is: allow the IMU stream to settle briefly, then estimate the zero-rate gyro bias while the vehicle is stationary.

### Accelerometer startup versus accelerometer calibration
There is not a matching generic boot accelerometer warm-up/calibration step in the normal Copter startup path.

Instead, the accel side is split into two different concerns:

1. backend hardware bring-up:
   - board-specific drivers initialise accel hardware, configure registers, set ranges and sample rates, and sometimes include small fixed delays during sensor reset/startup.
2. optional calibration workflows:
   - `simple_accel_cal()` performs a stillness-based accel offset calibration,
   - `calibrate_trim()` performs a lighter trim-calculation step and includes a short wait for INS filters to settle,
   - the `acal_*` path implements the multi-step accelerometer calibration framework.

These accel calibration flows are explicit calibration procedures. They are not part of the normal boot sequence that runs in `startup_INS_ground()`.

## Stage 1: Hardware Driver Acquisition

The hardware-specific backend is the first stage that sees register/FIFO data.

Representative verified examples:

### Invensense v3 path
In `libraries/AP_InertialSensor/AP_InertialSensor_Invensensev3.cpp`:
- FIFO frames are decoded into integer accel and gyro vectors.
- driver-specific scale factors convert counts to physical units.
- temperature is converted to degrees C.
- `_rotate_and_correct_accel()` and `_rotate_and_correct_gyro()` are called.
- then `_notify_new_accel_raw_sample()` and `_notify_new_gyro_raw_sample()` hand the samples to the common backend path.

### BMI088 path
In `libraries/AP_InertialSensor/AP_InertialSensor_BMI088.cpp`:
- FIFO/register data is converted from device counts to physical units.
- the same `_rotate_and_correct_*()` helpers are called.
- then `_notify_new_*_raw_sample()` feeds the common path.

### Important branch
Some drivers feed raw samples and call `_notify_new_accel_raw_sample()` / `_notify_new_gyro_raw_sample()`.
Some drivers can provide sensor-native delta-angle or delta-velocity and call `_notify_new_delta_angle()` / `_notify_new_delta_velocity()` instead.

That distinction matters because in the delta-angle/delta-velocity path the common helper itself performs rotation and correction before accumulation.

## Stage 2: Common AP_InertialSensor Transformations

All active backends converge in `libraries/AP_InertialSensor/AP_InertialSensor_Backend.cpp`.

### 2.1 Rotation and calibration order
`_rotate_and_correct_accel()` does:
1. rotate from sensor frame using `_accel_orientation[instance]`
2. optionally update temperature-learning model
3. if not in accel calibration:
   - apply temperature correction
   - subtract accel offset
   - apply accel scale
4. rotate into board/body frame using `_board_orientation`

`_rotate_and_correct_gyro()` does:
1. rotate from sensor frame using `_gyro_orientation[instance]`
2. optionally update temperature-learning model
3. if not in gyro calibration:
   - apply temperature correction
   - subtract gyro offset
4. rotate into board/body frame using `_board_orientation`

This is the first place where configured offsets, scales, temperature compensation, and mounting orientation are applied.

### 2.2 Gyro sample processing
`_notify_new_gyro_raw_sample()` does the following:
- updates the observed sensor rate for FIFO sensors,
- determines `dt` either from timestamps or from the inferred sample rate,
- calls `AP_Module::call_hook_gyro_sample()` if module hooks are enabled,
- forwards gyro samples to optical flow via `hal.opticalflow->push_gyro(...)` if optical flow exists,
- computes delta-angle using trapezoidal integration of current and previous gyro sample,
- computes coning correction,
- accumulates corrected delta-angle into `_delta_angle_acc` and `_delta_angle_acc_dt`,
- applies gyro filters,
- marks `_new_gyro_data[instance] = true`,
- performs raw/pre-post-filter logging depending on logging options.

### 2.3 Gyro filtering order
`apply_gyro_filters()` does:
1. optionally tap a gyro window for FFT,
2. apply enabled harmonic notch filters,
3. apply the gyro low-pass filter last,
4. reset filters if the result becomes NaN or Inf.

Important detail: harmonic notch filters are normally only run on the currently active IMU unless the notch option `EnableOnAllIMUs` is set. Inactive IMU notch filters are reset rather than kept warm.

### 2.4 Accel sample processing
`_notify_new_accel_raw_sample()` does:
- updates the observed accel sample rate,
- determines `dt`,
- calls `AP_Module::call_hook_accel_sample()` if enabled,
- updates vibration and clipping via `calc_vibration_and_clipping()`,
- accumulates delta-velocity into `_delta_velocity_acc` and `_delta_velocity_acc_dt`,
- applies accel low-pass filtering,
- updates peak-hold state for primary accel,
- marks `_new_accel_data[instance] = true`,
- logs raw or post-filter accel depending on logging mode.

### 2.5 Vibration and clipping tracking
`AP_InertialSensor::calc_vibration_and_clipping()` in `libraries/AP_InertialSensor/AP_InertialSensor.cpp`:
- increments clip count when accel exceeds backend clip limit,
- computes vibration by:
  - low-pass filtering accel at 5 Hz,
  - subtracting that floor from the current accel,
  - squaring the residual,
  - low-pass filtering the squared residual at 2 Hz.

`get_vibration_levels()` returns the square root of that filtered squared residual.

### 2.6 Filter parameter propagation
`update_gyro_filters()` and `update_accel_filters()` propagate front-end filter settings down to backend filters.

For the gyro path this includes:
- low-pass cutoff updates,
- post-filter FFT low-pass cutoff updates,
- harmonic notch parameter updates.

## Stage 3: AP_InertialSensor Front-End Publish and Primary IMU Selection

`AP_InertialSensor::update()` in `libraries/AP_InertialSensor/AP_InertialSensor.cpp` is the front-end fan-in called first in the Copter fast loop.

It does:
1. `wait_for_sample()` to pace the main loop and wait for IMU data.
2. mark all accel and gyro instances unhealthy before backend publication.
3. call every backend's `update()`.
4. each backend publishes filtered accel/gyro plus accumulated delta-angle/delta-velocity through `_publish_accel()` and `_publish_gyro()`.
5. adjust health based on backend error counts.
6. choose `_first_usable_gyro` and `_first_usable_accel` as the first healthy and enabled instances.

### 3.1 What `wait_for_sample()` actually does
`wait_for_sample()` is the function that paces the main loop. It:
- keeps loop cadence near the configured loop rate,
- sleeps or resynchronizes if the loop is early or late,
- repeatedly calls backend `accumulate()` methods while waiting,
- waits until gyro and accel data from required IMUs are available,
- sets `_have_sample` when the sample is ready.

### 3.2 What the published front-end values mean
After backend publication:
- `get_gyro(instance)` returns the filtered, corrected gyro for that instance.
- `get_accel(instance)` returns the filtered, corrected accel for that instance.
- `get_delta_angle(instance, ...)` returns accumulated delta-angle if valid, otherwise falls back to `get_gyro(i) * get_delta_time()`.
- `get_delta_velocity(instance, ...)` returns accumulated delta-velocity if valid, otherwise falls back to `get_accel(i) * get_delta_time()`.

These front-end values are already rotated and corrected. They are not raw ADC counts.

### 3.3 Instance usage logic
`use_gyro(instance)` and `use_accel(instance)` require both:
- health for that instance, and
- the instance enable/use bit.

That logic is what EKF3 and DAL see when selecting usable IMUs.

## Stage 4: DAL Snapshot Layer Used by EKF3

EKF3 does not read `AP_InertialSensor` directly. It reads the DAL wrapper.

`AP_DAL_InertialSensor::start_frame()` in `libraries/AP_DAL/AP_DAL_InertialSensor.cpp` snapshots:
- loop rate,
- loop delta time,
- first usable gyro/accel,
- accel and gyro counts,
- per-instance `use_accel` and `use_gyro`,
- per-instance delta-velocity and delta-angle plus their `dt`,
- IMU position offsets.

### DAL-local filtered accel/gyro
`AP_DAL_InertialSensor::update_filtered()` also builds a separate 10 Hz filtered accel and gyro estimate from the snapped delta-angle and delta-velocity rates.

This 10 Hz DAL filtering is specifically described in code as making EKF filtered accel/gyro independent of INS filter settings.

Important detail:
- EKF3 strapdown propagation uses delta-angle and delta-velocity from DAL.
- DAL `get_gyro()` and `get_accel()` are a separate 10 Hz filtered representation used by some auxiliary EKF logic, not the main predict integration path.

## Stage 5: EKF3 IMU Ingestion and State Propagation

### 5.1 EKF3 front-end frame handling
`NavEKF3::UpdateFilter()` in `libraries/AP_NavEKF3/AP_NavEKF3.cpp`:
- starts a DAL frame,
- timestamps the frame with `imuSampleTime_us`,
- runs every configured core,
- optionally suppresses state prediction when CPU budget is tight,
- chooses a primary lane after checking health and relative error.

Each active core is typically associated with one IMU lane chosen by `EK3_IMU_MASK`.

### 5.2 `readIMUData()` is the main IMU ingestion path
`NavEKF3_core::readIMUData()` in `libraries/AP_NavEKF3/AP_NavEKF3_Measurements.cpp` does the following:

1. reads DAL loop timing and updates `dtIMUavg` with a constrained low-pass scheme,
2. selects the active accel and gyro for this core:
   - if the core's IMU index is usable, use it,
   - otherwise fall back to DAL first usable accel/gyro,
3. if the active gyro or accel changes, swap in the learned inactive bias for that IMU,
4. updates inactive bias learners,
5. runs movement checking using IMU data,
6. reads delta-velocity and delta-angle from DAL,
7. stores IMU position offset for the active accel,
8. accumulates the IMU into a downsampled EKF packet,
9. uses quaternion accumulation for delta-angle downsampling to avoid introducing coning error,
10. rotates delta-velocity during accumulation,
11. once the target EKF step is reached, pushes the accumulated IMU packet into the IMU FIFO,
12. extracts the oldest delayed packet from the FIFO,
13. applies EKF-learned gyro and accel bias corrections with `correctDeltaAngle()` and `correctDeltaVelocity()`.

### 5.3 What EKF3 actually consumes
The IMU data that drives EKF3 predict steps is:
- delayed,
- downsampled to EKF target step size,
- bias-corrected by EKF inactive/active bias states,
- represented as delta-angle and delta-velocity,
- aligned to EKF delayed horizon.

EKF3 does not directly integrate `AP::ins().get_gyro()` or `AP::ins().get_accel()` for its main strapdown step.

### 5.4 Strapdown propagation
`NavEKF3_core::UpdateFilter()` in `libraries/AP_NavEKF3/AP_NavEKF3_core.cpp` runs:
- `readIMUData()`
- `UpdateStrapdownEquationsNED()`
- `CovariancePrediction()`
- fusion stages
- `calcOutputStates()`

`UpdateStrapdownEquationsNED()` uses the delayed corrected IMU values to:
- rotate quaternion state with corrected delta-angle and Earth-rate compensation,
- rotate body delta-velocity into nav frame,
- add gravity on Z,
- derive `velDotNED`, filtered nav acceleration magnitude, and launch-detect signals,
- optionally clip horizontal acceleration magnitude in `AID_NONE`,
- integrate velocity,
- integrate position trapezoidally,
- constrain states.

### 5.5 Output-horizon propagation
`calcOutputStates()` uses the newest IMU sample, not the delayed one, to propagate controller-facing output states forward from the delayed fusion horizon to the real-time horizon.

This stage:
- applies current gyro and accel bias corrections to the newest IMU sample,
- updates output quaternion,
- updates output velocity and position,
- runs the 3rd-order vertical complementary channel,
- compensates for IMU lever arm using IMU position offset,
- computes and applies attitude/velocity/position tracking corrections so outputs stay aligned with the EKF delayed state without step jumps.

## Stage 6: AHRS Canonical State Built from EKF3 + INS

`AP_AHRS::update()` in `libraries/AP_AHRS/AP_AHRS.cpp` runs after `AP_InertialSensor::update()` in the Copter fast loop.

`AP_AHRS::update_EKF3()` does:
- start EKF3 once startup delay and logger gating allow it,
- call `EKF3.UpdateFilter()`,
- if EKF3 is the active estimator, copy EKF3 outputs into AHRS canonical state.

### AHRS fields derived from EKF3 and INS
When EKF3 is active, AHRS sets:
- attitude matrix and Euler angles from EKF3 output quaternion,
- `state.gyro_drift` from EKF3 gyro bias estimate,
- `state.gyro_estimate` as `ins.get_gyro(primary_gyro) + gyro_drift`,
- `state.accel_bias` from EKF3 accel bias estimate,
- `state.accel_ef` as:
  - primary INS accel,
  - minus EKF accel bias,
  - transformed to earth frame by EKF body-to-NED rotation,
  - with autopilot-body to vehicle-body rotation applied.

Important consequence:
- AHRS angular rates used by controllers are not raw gyro values. They are latest filtered INS gyro plus EKF bias correction.
- AHRS earth-frame acceleration is not raw accel either. It is filtered INS accel minus EKF accel bias, then rotated to earth frame.

### `get_gyro_latest()` behavior
`AP_AHRS::get_gyro_latest()` in `libraries/AP_AHRS/AP_AHRS_Backend.cpp` returns:
- latest INS gyro from the current primary gyro instance,
- plus AHRS gyro drift correction.

This is specifically used by the rate controller to minimize latency.

### Alternative AHRS view
`AP_AHRS_View` in `libraries/AP_AHRS/AP_AHRS_View.cpp` can rotate the AHRS view and gyro into another reference frame. Copter attitude control is built on an AHRS view object, so controller inputs may be seen through this view layer rather than directly from the base AHRS object.

## Stage 7: What Feeds the Controller

### 7.1 Scheduler order matters
`ArduCopter/Copter.cpp` runs, in order:
1. `AP_InertialSensor::update`
2. `run_rate_controller`
3. `motors_output`
4. `read_AHRS`
5. `read_inertia`
6. `update_flight_mode`

The comment in the scheduler is explicit: INS is updated immediately so current gyro data is populated before the low-level rate controller runs.

### 7.2 Low-level rate controller input
`ArduCopter/Attitude.cpp::run_rate_controller()` calls `attitude_control->rate_controller_run()`.

For multicopters, `libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.cpp::rate_controller_run()` does:
- `Vector3f gyro_latest = _ahrs.get_gyro_latest();`
- `rate_controller_run_dt(gyro_latest, _dt);`

`rate_controller_run_dt()` then uses that gyro vector as the measured body-rate feedback for the roll, pitch, and yaw rate PIDs, and sends the PID outputs to motors.

So the IMU-derived signal fed directly into the low-level body-rate controller is:

`latest filtered primary INS gyro + EKF/AHRS drift correction`

### 7.3 Attitude controller state input
`libraries/AC_AttitudeControl/AC_AttitudeControl.cpp::attitude_controller_run_quat()` uses:
- `_ahrs.get_quat_body_to_ned(attitude_body)` for current attitude,
- `get_latest_gyro()` for current angular rate,
- quaternion attitude error to generate body-frame angular velocity targets.

So the attitude controller sits on top of AHRS quaternion plus AHRS gyro.

### 7.4 Position controller accel input
`libraries/AC_AttitudeControl/AC_PosControl.h::get_z_accel_cmss()` uses:

`-(_ahrs.get_accel_ef().z + GRAVITY_MSS) * 100.0f`

So the vertical position controller consumes earth-frame acceleration derived from the primary INS accel after EKF bias removal and AHRS rotation to earth frame.

### 7.5 InertialNav wrapper used by Copter logic
`ArduCopter/inertia.cpp::read_inertia()` calls `inertial_nav.update(vibration_check.high_vibes)`.

`libraries/AP_InertialNav/AP_InertialNav.cpp::update()` pulls from AHRS/EKF:
- relative position NE origin,
- relative position D origin,
- velocity NED,
- fallback vertical-rate estimate during high vibration or velocity failure.

This is not raw IMU data anymore. It is EKF state derived from IMU and other sensors.

## Stage 8: Logging Path

### 8.1 Normal IMU logging
`ArduCopter/Copter.cpp` schedules:
- `loop_rate_logging()` for `MASK_LOG_IMU_FAST`
- `twentyfive_hz_logging()` for `MASK_LOG_IMU` when not already logging fast
- `ten_hz_logging_loop()` for vibration logs

`AP::ins().Write_IMU()` in `libraries/AP_InertialSensor/AP_InertialSensor_Logging.cpp` writes, per instance:
- filtered accel,
- filtered gyro,
- error counts,
- temperature,
- health,
- reported sensor rates.

### 8.2 Vibration logging
`AP::ins().Write_Vibration()` logs:
- vibration per axis,
- accel clipping count.

### 8.3 Raw/pre-post-filter logging
Raw ACC/GYR logging is handled in `AP_InertialSensor_Backend.cpp` and `AP_InertialSensor_Logging.cpp`.

Depending on raw logging options, logs can contain:
- pre-filter raw sample,
- post-filter sample,
- both pre and post filter.

This is a separate path from the MAVLink `RAW_IMU` message. The log path can be truly pre-filter. The MAVLink message is not.

## Stage 9: MAVLink Output Path and Mission Planner-Relevant Messages

### 9.1 Legacy stream-rate groups in Copter
`ArduCopter/GCS_Mavlink.cpp` defines the stream groups.

Relevant IMU-related groups are:

#### `SRx_RAW_SENS`
Contains:
- `MSG_RAW_IMU`
- `MSG_SCALED_IMU2`
- `MSG_SCALED_IMU3`
- `MSG_SCALED_PRESSURE`
- `MSG_SCALED_PRESSURE2`
- `MSG_SCALED_PRESSURE3`
- `MSG_AIRSPEED` if enabled

Important detail: Copter's default RAW_SENSORS stream group does not include `MSG_SCALED_IMU` for primary IMU.

#### `SRx_EXTRA1`
Contains:
- `MSG_ATTITUDE`
- `MSG_AHRS2`
- `MSG_PID_TUNING`
- `MSG_SIMSTATE` in SIM builds

#### `SRx_EXTRA3`
Contains:
- `MSG_AHRS`
- `MSG_SYSTEM_TIME`
- `MSG_WIND`
- `MSG_EKF_STATUS_REPORT`
- `MSG_VIBRATION`
- plus several non-IMU-related messages

### 9.2 Per-message interval control
`libraries/GCS_MAVLink/GCS_Common.cpp` supports per-message requests with:
- `MAV_CMD_SET_MESSAGE_INTERVAL`
- `MAV_CMD_REQUEST_MESSAGE`
- legacy `REQUEST_DATA_STREAM`

This matters because some IMU-related messages are sendable even if they are not part of the legacy Copter stream groups.

### 9.2.1 Explicit HIGHRES_IMU request path
If a GCS explicitly asks for `HIGHRES_IMU` (MAVLink message 105), the ArduPilot-side path is:

1. The GCS sends either:
   - `MAV_CMD_SET_MESSAGE_INTERVAL` with `param1 = 105` and `param2 = interval_us` for periodic streaming, or
   - `MAV_CMD_REQUEST_MESSAGE` with `param1 = 105` for a one-shot sample.
2. In `libraries/GCS_MAVLink/GCS_Common.cpp`, the MAVLink-to-internal-message map includes:
   - `MAVLINK_MSG_ID_HIGHRES_IMU -> MSG_HIGHRES_IMU`
3. `MAV_CMD_SET_MESSAGE_INTERVAL` is handled by `handle_command_set_message_interval()`, which calls `set_message_interval()` and stores the message in ArduPilot's deferred message scheduler at the requested interval.
4. `MAV_CMD_REQUEST_MESSAGE` is handled by `handle_command_request_message()`, which resolves message 105 to `MSG_HIGHRES_IMU` and immediately calls `send_message(id)` for a one-shot transmission.
5. When `send_message(MSG_HIGHRES_IMU)` runs, dispatch reaches `send_highres_imu()`.

So ArduPilot supports both:
- a periodic streamed path for `HIGHRES_IMU` through `SET_MESSAGE_INTERVAL`, and
- a one-shot path through `REQUEST_MESSAGE`.

This is separate from the older grouped `REQUEST_DATA_STREAM` mechanism.

### 9.2.2 Mission Planner default behavior versus explicit request
The Mission Planner paths cited from its own repository show:

- manual per-message requests can be issued from `temp.cs` using `MAV_CMD_SET_MESSAGE_INTERVAL`
- normal connection startup still re-requests grouped streams with `REQUEST_DATA_STREAM`
- that default grouped startup path requests `RAW_SENSORS`, not `HIGHRES_IMU`

So the practical distinction is:
- ArduPilot can provide `HIGHRES_IMU` on demand,
- but Mission Planner does not normally request it during standard connection setup unless a user or a tool path explicitly asks for message 105.

### 9.3 What each IMU-related MAVLink sender actually transmits

#### `send_raw_imu()`
Source:
- `ins.get_accel(0)`
- `ins.get_gyro(0)`
- compass instance 0 if present

Important details:
- uses IMU instance 0 explicitly, not the current first usable IMU and not the EKF primary lane.
- values come from `AP_InertialSensor` front-end accel and gyro, which are already corrected and filtered.
- despite the name, this is not raw pre-calibration sensor data.

#### `send_scaled_imu(instance, ...)`
Source:
- `ins.get_accel(instance)`
- `ins.get_gyro(instance)`
- temperature for that instance
- compass field for matching instance if present

Important details:
- values are also already corrected and filtered.
- Copter stream groups include `SCALED_IMU2` and `SCALED_IMU3`, but not primary `SCALED_IMU` in `SRx_RAW_SENS`.
- `SCALED_IMU` can still be requested directly by message ID because `GCS_Common.cpp` dispatches it.

#### `send_highres_imu()`
Source:
- `ins.get_accel()`
- `ins.get_gyro()`
- default accessors use the first usable accel and first usable gyro
- compass field if a compass is present
- barometer pressure, altitude, ground-pressure-derived differential pressure, and temperature if a barometer is present

Important details:
- this is mostly a packaging step around current front-end sensor state, not a separate heavy transform pipeline.
- accel is packaged from `AP::ins().get_accel()` in m/s^2.
- gyro is packaged from `AP::ins().get_gyro()` in rad/s.
- magnetometer values are filled from the compass field path when available.
- absolute pressure, differential pressure, barometric altitude, and temperature are filled from the barometer path when available.
- `fields_updated` bits are set to indicate which groups were populated.
- `id` is set from `ins.get_first_usable_accel()`, so it identifies the currently selected accel instance for this packet.
- this is also corrected/filtered `AP_InertialSensor` output, not hardware-raw data.
- `HIGHRES_IMU` is supported by the sender and dispatch table, but it is not part of Copter's default stream group arrays in `ArduCopter/GCS_Mavlink.cpp`.

#### `send_attitude()`
Source:
- `ahrs.get_roll()`, `get_pitch()`, `get_yaw()`
- `ahrs.get_gyro()`

Important details:
- angle output is estimator output, primarily EKF3 when active.
- angular rates here are `AHRS state.gyro_estimate`, which is latest INS gyro plus EKF bias correction.
- this is the cleanest controller-facing orientation/rate signal among the standard MAVLink messages.

#### `send_ahrs2()`
Source:
- `ahrs.get_secondary_attitude()`
- `ahrs.get_secondary_position()`

This is a secondary AHRS/estimator view, not the primary controller feed.

#### `send_ahrs()`
Source:
- `ahrs.get_gyro_drift()`
- AHRS roll/pitch/yaw error metrics

This is diagnostic estimator metadata, not raw sensor output.

#### `send_vibration()`
Source:
- `ins.get_vibration_levels()`
- clipping counts for accel instances 0, 1, 2

This is derived from the vibration estimator inside `AP_InertialSensor`, not a direct hardware measurement.

### 9.4 Mission Planner receive handling for HIGHRES_IMU
The Mission Planner receive path you cited indicates that when `HIGHRES_IMU` arrives, it is handled mostly as a light unpack-and-store operation.

Mission Planner behavior:
- routes packets by `imu.id`
- uses `imu.id == 0`, `1`, or `2` to populate IMU1, IMU2, or IMU3 state slots
- checks `fields_updated` before applying each sensor group
- stores accel, gyro, and mag values essentially as received
- stores:
  - `press_abs = imu.abs_pressure`
  - `press_temp = (int)imu.temperature`
  - `altasl = imu.pressure_alt`

Important receive-side quirks:
- Mission Planner checks the XACC, XGYRO, and XMAG bits, then copies all three axes for that group, so its gating is group-style rather than axis-by-axis.
- temperature is truncated from float to int on receipt.
- the diff-pressure bit exists in the constants, but that parser block does not consume `imu.diff_pressure`.
- pressure and barometric altitude are stored as shared state, not as separate per-IMU-instance values.

So the shortest accurate summary is:
- an explicit GCS request for `HIGHRES_IMU` causes ArduPilot to schedule or emit `send_highres_imu()`,
- `send_highres_imu()` packages current INS, compass, and barometer state into MAVLink message 105,
- Mission Planner then stores most of those values with only light handling on receipt,
- but Mission Planner does not normally request `HIGHRES_IMU` during its default connection startup and still prefers the legacy grouped `RAW_SENSORS` request path unless message 105 is explicitly requested.

## End-to-End IMU Pipeline Summary

### A. Configuration and startup
1. `AHRS_ORIENTATION` is pushed into INS board orientation.
2. INS parameters define per-instance scale, offsets, filter cutoffs, use masks, IMU position offsets, and notch configuration.
3. `startup_INS_ground()` initializes AHRS and INS, performs the boot gyro warm-up/calibration path, then resets AHRS.
4. The normal boot path does not run a matching generic accelerometer warm-up/calibration stage; accel calibration is handled separately by explicit calibration workflows.

### B. Hardware sample acquisition
1. A hardware backend reads FIFO/register samples.
2. It converts counts to physical units.
3. It calls common rotation/correction helpers.
4. It hands samples into `_notify_new_*` common paths.

### C. Common INS processing
1. Apply sensor-frame rotation.
2. Apply temperature calibration if enabled.
3. Apply offsets and accel scale.
4. Apply board orientation.
5. For gyro: compute delta-angle and coning correction.
6. For accel: compute delta-velocity.
7. Apply harmonic notch filters to gyro if configured.
8. Apply low-pass filters.
9. Update clipping and vibration estimators.
10. Mark new data available for front-end publication.

### D. INS front-end publication
1. `AP_InertialSensor::update()` waits for the sample.
2. All backends publish filtered accel/gyro plus accumulated delta-angle/delta-velocity.
3. First healthy enabled gyro and accel become the first usable instances.

### E. DAL snapshot
1. DAL snapshots loop timing, use flags, counts, delta-angle, delta-velocity, and IMU position offsets.
2. DAL also builds a separate 10 Hz filtered gyro/accel representation.

### F. EKF3 consumption
1. EKF3 front-end starts a DAL frame and timestamps it.
2. Each core selects its active accel/gyro lane or falls back to the first usable IMU.
3. `readIMUData()` downsamples IMU deltas to EKF target step.
4. The delayed FIFO packet is bias-corrected.
5. `UpdateStrapdownEquationsNED()` propagates quaternion, velocity, and position.
6. `calcOutputStates()` propagates corrected states forward to output horizon.

### G. AHRS publication
1. AHRS copies EKF3 orientation into canonical roll/pitch/yaw and body-to-NED rotation.
2. AHRS gyro estimate becomes latest primary INS gyro plus EKF drift correction.
3. AHRS earth-frame accel becomes primary INS accel minus EKF accel bias, rotated to earth frame.

### H. Controller consumption
1. Low-level rate controller uses `AHRS::get_gyro_latest()`.
2. Attitude controller uses `AHRS::get_quat_body_to_ned()` and latest gyro.
3. Position controller Z-accel path uses `AHRS::get_accel_ef()`.
4. InertialNav uses EKF position/velocity, not raw IMU.

### I. Logs and MAVLink
1. Logs can store filtered IMU, vibration, and optionally pre/post-filter raw ACC/GYR samples.
2. MAVLink sends corrected/filtered INS values for `RAW_IMU`, `SCALED_IMU*`, `HIGHRES_IMU`.
3. MAVLink sends EKF/AHRS outputs for `ATTITUDE`, `AHRS`, `AHRS2`, and `VIBRATION`.

## Key Non-Obvious Findings

1. Copter low-level rate control runs immediately after `AP_InertialSensor::update()`, before `read_AHRS()`. It uses `AHRS::get_gyro_latest()`, which is specifically designed to use the latest INS sample plus AHRS drift correction.

2. EKF3 strapdown propagation is driven by delayed, downsampled, bias-corrected delta-angle and delta-velocity, not by directly integrating `get_gyro()` / `get_accel()`.

3. `RAW_IMU` in MAVLink is not hardware raw in this codebase. It comes from `ins.get_accel(0)` and `ins.get_gyro(0)`, which are already corrected and filtered front-end values.

4. `ATTITUDE` is much closer to the actual controller-facing state than `RAW_IMU`, because it is sourced from AHRS/EKF attitude and AHRS gyro estimate.

5. `HIGHRES_IMU` uses the first usable accel and gyro, while `RAW_IMU` uses instance 0 explicitly. Those can differ.

6. Copter's legacy `SRx_RAW_SENS` stream includes `RAW_IMU`, `SCALED_IMU2`, and `SCALED_IMU3`, but not primary `SCALED_IMU`. Primary `SCALED_IMU` and `HIGHRES_IMU` are still individually sendable via per-message interval requests.

7. DAL creates its own 10 Hz filtered accel/gyro view that is separate from the main AP_InertialSensor low-pass/notch outputs and separate from the delta-angle/delta-velocity path that drives EKF3 propagation.

8. The startup "warm-up" called from Copter boot is mainly a gyro bias-calibration routine with a short priming phase and a stillness/convergence test. The accelerometer participates as a motion gate during that process, but there is no equivalent generic accel warm-up stage in the normal boot path.

## If You Need to Trace a Specific Board Further

To finish the hardware-specific part for a concrete board, inspect which backend actually probes and registers on that target. The common path documented above starts after the backend converts device data into `Vector3f accel` / `Vector3f gyro` and calls the common backend helpers.

For that board-specific step, inspect the active driver among the backend files listed in the hardware-specific section.