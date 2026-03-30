# RIO Project Overview for ArduCopter Integration

## Purpose

This document is the fastest way to get up to speed on the project.


**Project goal:** Enable ArduCopter to hover and maneuver reliably in GPS-denied environments by providing external velocity (and position) estimates from a radar-inertial odometry (RIO) stack running on the companion computer. The pilot always flies using the RC. The flight controller (FC) runs all attitude, velocity, and altitude control loops as usual. The only change is that EKF3 receives its navigation state from RIO instead of GPS.

**In one sentence:** The companion computer supplies external navigation estimates (velocity and position) to EKF3, so the pilot can fly with the RC in standard modes (e.g., Loiter, AltHold), and the FC uses RIO as its navigation source for XY velocity control and altitude hold—never by sending commands from the CC.

## What Success Looks Like

The desired end state is:

- The pilot arms and flies with the RC as usual.
- ArduCopter no longer depends on GPS for horizontal state estimation.
- Horizontal motion estimates come from the RIO stack running on the companion computer.
- EKF3 accepts those external-navigation measurements cleanly.
- Hover and low-speed maneuvering remain stable even in GPS-denied spaces.

For the current project phase, the practical target is a cautious RC-assisted hover using the existing ArduCopter control stack, with RIO supplying the external navigation data that EKF3 fuses.

## Core Operating Concept


## Control Concept: RC Pilot, FC Control, External Estimates Only

The only objective is to let the pilot fly the aircraft with the RC, while the FC uses RIO-provided velocity (and optionally position) estimates for navigation. No external commands are ever sent from the companion computer. The FC’s own controllers (attitude, velocity, altitude) remain in full control. The RIO stack simply replaces GPS as the navigation source for EKF3.

The recommended initial test mode is Loiter, because it uses ArduCopter’s robust closed-loop stabilization, but any standard mode that uses EKF3’s velocity estimate for XY control and holds altitude is valid. The pilot always has direct stick authority.

## Main Inputs

The system combines four classes of inputs.

### 1. Radar detections

Source:

- TI mmWave radar through `ti_mmwave_rospkg`
- Main ROS topic: `/ti_mmwave/radar_scan_pcl`

What it provides:

- Per-frame point cloud detections
- Point positions and Doppler velocity
- Radar-side CPU load and timing instrumentation
- Raw and filtered point-count health indicators

Why it matters:

- Radar provides motion observability in GPS-denied environments.
- Doppler-rich point clouds are the basis for estimating ego velocity.

### 2. Flight-controller IMU

Source:

- MAVLink from the FC through `fc_com_if`
- Main ROS topic: `/fc/imu`

What it provides:

- Accelerometer measurements
- Gyroscope measurements
- FC-side timing context used for synchronization

Why it matters:

- IMU data drives the high-rate prediction step of the estimator.
- It keeps the estimator responsive between radar updates.

### 3. Flight-controller attitude

Source:

- MAVLink `ATTITUDE_QUATERNION`

What it provides:

- Body attitude from the FC side

Why it matters:

- The bridge uses FC attitude to rotate body-frame velocity into the NED frame expected by ArduPilot.
- This keeps the MAVLink velocity message aligned with the FC navigation convention.

### 4. RC pilot input

Source:

- Standard RC link to ArduCopter

What it provides:

- Pilot control for takeoff, hover, small translations, and recovery

Why it matters:

- The project is not fully autonomous waypoint flight.
- The near-term use case is pilot-supervised hover and manual flight with improved navigation in GPS-denied conditions.

## Main Algorithms and Components

## Radar front end

The radar driver parses TI mmWave UART frames into ROS point clouds.

Relevant package:

- `humble_ws/src/mmwave_ti_ros/ros2_driver/src/ti_mmwave_rospkg`

Key jobs:

- Read and decode radar TLVs
- Filter and crop detections
- Publish point clouds
- Expose timing logs such as sort time and publish gap

Operational lesson from this project:

- Radar overload or sparse outputs can break downstream estimation quality.
- Because of that, radar health instrumentation, point-count monitoring, and watchdog recovery were added as first-class operational tools.

## RIO estimator

The estimator lives in `rio_ros` and its EKF-RIO library.

Relevant packages:

- `humble_ws/src/rio_ros`
- `humble_ws/src/rio_ros/ekf_rio_lib`

Current role:

- Predict state forward using IMU
- Correct that state using radar information
- Publish odometry and debug outputs to ROS

Estimator outputs used by the integration:

- `/nav/odometry`
- `/nav/odometry_fc`
- `/nav/rio_debug`
- `/nav/radar_estimator_debug`

The most important published quantity for ArduPilot is velocity, along with a consistent timestamp that represents the FC time domain.

## Radar ego-velocity estimation

Inside the estimator path, radar detections are reduced to a usable motion update.

Important concepts:

- `raw_point_count`: how many radar detections entered the update path
- `valid_point_count` or kept points: how many survived filtering and were considered usable

Why this matters:

- A low kept-point count can explain poor or degraded motion updates.
- It is a direct operational clue when diagnosing drift, sparse frames, or radar overload.

## FC bridge and timestamp alignment

The ArduPilot bridge lives in `fc_com_if`.

Relevant package:

- `humble_ws/src/fc_com_if`

Main jobs:

- Receive FC MAVLink messages
- Publish ROS IMU data upstream
- Subscribe to RIO odometry downstream
- Convert ROS frame conventions to ArduPilot conventions
- Publish MAVLink external-navigation messages back to the FC

The bridge performs a critical timestamp function.

Instead of stamping outgoing vision messages with ROS wall-clock time, the project wraps odometry in `OdometryFcStamped` and carries `fc_boot_time_usec`, which is derived from the estimator state timestamp expressed in the FC boot-clock domain. That timestamp is then used for the outgoing MAVLink messages.

This was a major design decision because it prevents clock-domain mismatch between the companion computer and ArduPilot EKF3.

## Frames and conventions

Several frame conversions are necessary.

- ROS body velocity is published in FLU convention.
- ArduPilot body convention is FRD.
- ArduPilot navigation convention is NED.

So the bridge does:

1. FLU body velocity from ROS
2. Convert FLU to FRD
3. Rotate FRD body velocity into NED using FC attitude
4. Send NED velocity to ArduPilot in `VISION_SPEED_ESTIMATE`

For position, the same principle applies: the bridge converts the RIO pose into the convention ArduPilot expects for `VISION_POSITION_ESTIMATE`.


## Why We Send Both Position and Velocity

Bench testing showed that sending only velocity was not sufficient for stable no-GPS Copter operation. The reliable configuration is to send both `VISION_POSITION_ESTIMATE` and `VISION_SPEED_ESTIMATE` as a consistent pair. This ensures EKF3 enters and maintains the correct external-navigation fusion mode. Velocity is the key quantity for short-term hover, but position is needed for EKF3 to accept and use the external navigation source.

## Current Flight-Control Concept

For the current phase, think of the control stack this way:

1. The pilot commands the aircraft with the RC.
2. ArduCopter interprets those stick inputs according to the selected mode.
3. EKF3 provides the FC with horizontal state estimates based on RIO external navigation instead of GPS.
4. ArduCopter's normal controllers generate the final attitude and motor commands.

The near-term hover goal is not to bypass ArduCopter's controllers. It is to give those controllers a usable GPS-denied navigation source.


### What “velocity control for hovering with the RC” means here

This means: the pilot uses the RC sticks as usual. The FC’s own controllers use the RIO-provided velocity estimate (and optionally position) for XY stabilization and altitude hold. No commands are ever sent from the companion computer. The only change is that EKF3’s navigation state comes from RIO, not GPS. The pilot should experience predictable, drift-free hover and manual flight, even in GPS-denied environments.

## Current Key Parameters and Decisions

These are the decisions that matter most for understanding the present architecture.

- External navigation is enabled through EKF3 source selection.
- The FC bridge publishes paired `VISION_POSITION_ESTIMATE` and `VISION_SPEED_ESTIMATE`.
- The outgoing timestamps are based on `fc_boot_time_usec`, not ROS wall clock.
- The current measured starting value for `VISO_DELAY_MS` is `0` for this wrapper path.
- Radar health matters as much as estimator math; a stalled or sparse radar stream can make delay measurements and hover evaluation meaningless.

## Validation Signals We Use

When testing, these are the most important indicators.

### ROS-side indicators

- `/nav/rio_debug`
- `/nav/radar_estimator_debug`
- Radar CPU load
- Radar raw and kept point counts
- Radar timing logs such as `sort` and `publish_gap`

These tell us whether the radar and estimator pipeline is healthy before blaming ArduPilot.

### FC-side indicators

- `VISP`
- `VISV`
- `XKF3`
- `XKF4`

These tell us whether EKF3 is actually receiving and accepting the external-navigation data.

Operationally, the project learned that you must separate two failure classes:

- external-navigation timing/configuration problems on the FC side
- degraded or stalled radar input on the companion-computer side

## Major Risks Already Identified

Anyone continuing this work should know these immediately.

### Radar overload and sparse updates

This was one of the main real-world failure modes.

Symptoms:

- radar scan topic stalls or slows down
- kept-point count collapses
- estimator drifts after motion stops because it is effectively propagating on IMU only

Mitigations already added:

- watchdog restart behavior
- launch respawn behavior
- temporary radar-side CFAR tuning to reduce overload
- extra debug topics and timing logs

### Clock-domain errors

This was the other major design risk.

If outgoing vision timestamps are expressed in the wrong time base, EKF3 may receive the data but fuse it poorly or at the wrong horizon.

The FC-time wrapper path was introduced specifically to solve this.

## Recommended Mental Model

If you are new to the project, keep this model in mind:

- The companion computer estimates motion.
- The bridge translates that estimate into ArduPilot's language.
- EKF3 turns that into the FC state estimate.
- ArduCopter's existing controllers fly the vehicle.
- The pilot still uses the RC.

So this is not a custom flight controller. It is a custom external-navigation stack feeding a standard flight controller.

## Current Status Summary

As of the current project state:

- Paired VPE and VSE publication is implemented.
- FC-domain timestamp propagation is implemented.
- Bench measurements support starting with `VISO_DELAY_MS = 0`.
- Radar watchdog and recovery hooks are implemented.
- Debug visibility for radar timing, CPU load, and point counts has been improved.
- The next practical milestone is cautious hover testing with the RC using the current external-navigation path.

## Suggested Reading Order

After this document, read these in order:

1. `docs/data_flow_and_config_guide.md`
2. `docs/plans/rio_integration_ardupilot.md`
3. `docs/ekf3_ring_buffer_and_viso_delay.md`
4. `docs/ArduCopter/flight_modes_external_nav_review.md`
5. `docs/ArduCopter/plan_for_velocity_control.md`

That sequence moves from the implemented system, to the ArduPilot bridge, to timing correctness, to flight-mode selection, and finally to the future velocity-control design thread.