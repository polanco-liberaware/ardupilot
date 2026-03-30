# ArduCopter SystemID Inner-Loop Logging

## Scope
This document describes the ArduCopter System Identification path relevant to inner-loop plant identification, with emphasis on:

1. `SID_AXIS = 10` (`MIX_ROLL`),
2. how the injected chirp reaches the roll mixer input,
3. what `RATE.RDes`, `RATE.R`, and `RATE.ROut` mean,
4. how `SIDD.Targ` and `SIDD.Gx` relate to the `RATE` message,
5. timestamp alignment and expected logging rates.

This document is based on code inspection in this repository only.

## Short Conclusions

For `SID_AXIS = 10`:

- `SIDD.Targ` is the injected chirp sample.
- `RATE.ROut` includes that chirp.
- `RATE.ROut` is not just the chirp. It is the total normalized roll command channel sent toward the mixer.
- `RATE.R` and `SIDD.Gx` are both roll-rate measurements in deg/s and are usually very close, but they are not produced from the exact same internal quantity.
- The ArduPilot wiki recommends `RATE.ROut`, `RATE.POut`, `RATE.YOut` as plant inputs for multicopter identification with `SID_AXIS = 10, 11, 12`, and `SIDD.Gx`, `SIDD.Gy`, `SIDD.Gz` as measured angular-rate outputs.
- The wiki describes SystemID data as time synchronized within each loop. In the code, `RATE` and `SIDD` are emitted from the same logging call path, but their raw `time_us` fields are not populated from the exact same source.

## File Map

### SystemID mode and Copter logging
- `ArduCopter/mode_systemid.cpp`
  - chirp generation,
  - `SID_AXIS` switching,
  - `SIDD` write trigger,
  - `RATE` write trigger via `Log_Write_Attitude()`.
- `ArduCopter/mode.h`
  - `ModeSystemId` declaration and `AxisType` enum.
- `ArduCopter/Parameters.cpp`
  - registration of the `SID` parameter subgroup.
- `ArduCopter/Log.cpp`
  - `SIDD` packet write,
  - `Log_Write_Attitude()` wrapper.

### Attitude control and rate logging
- `libraries/AC_AttitudeControl/AC_AttitudeControl.h`
  - storage of SystemID rate-side and actuator-side injections.
- `libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.cpp`
  - rate-controller output generation,
  - addition of actuator-side SystemID injection.
- `libraries/AC_AttitudeControl/AC_AttitudeControl_Logging.cpp`
  - `RATE` log write.
- `libraries/AC_AttitudeControl/LogStructure.h`
  - `RATE` field definitions.

### PID and motor interface
- `libraries/AC_PID/AC_PID.cpp`
  - rate PID feedback output,
  - separate feedforward output.
- `libraries/AP_Motors/AP_Motors_Class.h`
  - normalized roll/pitch/yaw command storage.
- `libraries/AP_Motors/AP_MotorsMatrix.cpp`
  - use of normalized roll/pitch/yaw channels inside the mixer.

## SystemID Injection Path For `SID_AXIS = 10`

The public multicopter model-development documentation states that the frequency sweeps `SID_AXIS = 10, 11, 12` are used for plant identification because they superimpose the mixer inputs and are less modified by the controllers than outer-loop injection points. It also states that for the rate-controller outputs the signals `RATE.ROut`, `RATE.POut`, and `RATE.YOut` are used, and that `SIDD.Gx`, `SIDD.Gy`, and `SIDD.Gz` correspond to the measured angular rates of the copter.

### Chirp generation
`ModeSystemId::run()` computes the instantaneous chirp sample:

```cpp
waveform_sample = chirp_input.update(waveform_time - SYSTEM_ID_DELAY, waveform_magnitude);
```

That `waveform_sample` is later logged as `SIDD.Targ`.

### `SID_AXIS = 10` routing
For `MIX_ROLL`, the chirp is injected on the actuator side, not on the rate-target side:

```cpp
case AxisType::MIX_ROLL:
    attitude_control->actuator_roll_sysid(waveform_sample);
    break;
```

The setter stores the injected chirp into `_actuator_sysid.x`:

```cpp
void actuator_roll_sysid(float command) { _actuator_sysid.x = command; }
```

### Rate controller output stage
Inside the multicopter rate controller, the roll command written to the motors object is:

```cpp
_motors.set_roll(get_rate_roll_pid().update_all(ang_vel_body.x, gyro.x,  dt, _motors.limit.roll, _pd_scale.x) + _actuator_sysid.x);
_motors.set_roll_ff(get_rate_roll_pid().get_ff());
```

This separates the roll command into:

- `get_rate_roll_pid().update_all(...)`: feedback part,
- `_actuator_sysid.x`: actuator-side SystemID chirp,
- `get_rate_roll_pid().get_ff()`: separate feedforward part.

## Meaning Of `RATE.RDes`, `RATE.R`, `RATE.ROut`

The `RATE` packet is written in `AC_AttitudeControl::Write_Rate()`:

```cpp
const Vector3f rate_targets = rate_bf_targets() * RAD_TO_DEG;
const Vector3f gyro_rate = _rate_gyro * RAD_TO_DEG;

control_roll    : rate_targets.x,
roll            : gyro_rate.x,
roll_out        : _motors.get_roll()+_motors.get_roll_ff(),
```

### `RATE.RDes`
`RATE.RDes` is the desired roll rate in deg/s.

It comes from:

```cpp
Vector3f rate_bf_targets() const { return _ang_vel_body + _sysid_ang_vel_body; }
```

So `RDes` contains SystemID only when the chirp is injected as a rate target, such as `SID_AXIS = 7`.

For `SID_AXIS = 10`, the chirp is actuator-side, so it does not appear in `RDes`.

### `RATE.R`
`RATE.R` is the achieved roll rate in deg/s, derived from `_rate_gyro`.

### `RATE.ROut`
`RATE.ROut` is the normalized roll command channel sent toward the mixer.

It is dimensionless, nominally in `[-1, +1]`, because the motor interface stores roll, pitch, and yaw commands as normalized quantities:

```cpp
void set_roll(float roll_in) { _roll_in = roll_in; };        // range -1 ~ +1
float get_roll() const { return _roll_in; }
```

The mixer uses the same normalized roll and roll-FF channels as a normalized thrust-like axis input:

```cpp
const float roll_thrust = (_roll_in + _roll_in_ff) * compensation_gain;
```

So for `SID_AXIS = 10`:

```text
RATE.ROut = feedback roll control + actuator-side SystemID chirp + roll feedforward
```

This matches the wiki guidance that the plant input for the multicopter identification case is the sum of the sweep and the rate-controller output.

The PID split is explicit in `AC_PID.cpp`:

```cpp
return P_out + D_out + _integrator;
```

and

```cpp
float AC_PID::get_ff() const
{
    return  _pid_info.FF + _pid_info.DFF;
}
```

So more precisely:

```text
RATE.ROut = (P + D + I) + SIDD.Targ + (FF + DFF)
```

for `SID_AXIS = 10`, assuming `SIDD.Targ` is the chirp routed into `MIX_ROLL`.

Important practical nuance: the wiki's plant-identification workflow recommends setting `ATC_RAT_RLL_I = 0`, `ATC_RAT_PIT_I = 0`, `ATC_RAT_YAW_I = 0`, and `ATC_RATE_FF_ENAB = 0` before collecting identification data. Under that recommended setup, the effective contribution from the integral and feedforward paths is intentionally reduced or removed, which is why the public description often refers to the plant input as the sum of the sweep and the rate-controller output without dwelling on FF.

## Meaning Of `SIDD.Targ` And `SIDD.Gx`

### `SIDD.Targ`
`SIDD.Targ` is the logged `waveform_sample`, the instantaneous chirp sample.

### `SIDD.Gx`
`SIDD.Gx` is logged as:

```cpp
degrees(delta_angle.x / delta_angle_dt)
```

So `SIDD.Gx` is a roll-rate estimate in deg/s computed from delta-angle divided by delta time.

The wiki further describes the `SID` gyro and acceleration measurements as average IMU measurements since the last loop time, taken directly from IMU delta angles and delta velocities without additional filtering, using the same IMU selected for attitude prediction in the angle-control loops.

## Relationship Between `RATE.R` And `SIDD.Gx`

These are practically the same physical signal:

- `RATE.R`: roll rate from `_rate_gyro`,
- `SIDD.Gx`: roll rate from `delta_angle / delta_angle_dt`.

They are usually very close during analysis, but they are not guaranteed to be numerically identical sample-by-sample because they come from different internal representations.

## Timestamp Alignment

Although `RATE` and `SIDD` are written during the same SystemID logging call, they do not use the exact same timestamp source.

The public wiki wording says that all SystemID data from each loop is recorded at the same time and placed in the dataflash log to ensure data is time synchronized. That description is accurate at the level of the logging event and the intended analysis workflow.

At the raw packet-field level, the code uses different timestamp sources for the two messages, which matters if timestamps are compared numerically sample by sample.

### `SIDD.time_us`
`SIDD` uses wall-clock microseconds at log write time:

```cpp
time_us         : AP_HAL::micros64(),
```

### `RATE.time_us`
`RATE` uses the stored gyro-sample time:

```cpp
time_us         : _rate_gyro_time_us,
```

So the practical rule is:

- same logging event: yes,
- same intended loop alignment: yes,
- same exact raw `time_us` field: not guaranteed.

For identification, align by timestamp and interpolate if necessary. Do not assume row-by-row equality.

## Expected Sampling Frequency

### Base loop rate
For Copter and Heli builds, the scheduler default loop rate is 400 Hz:

```cpp
#define SCHEDULER_DEFAULT_LOOP_RATE 400
```

and the scheduler parameter default is:

```cpp
AP_GROUPINFO("LOOP_RATE",  1, AP_Scheduler, _loop_rate_hz, SCHEDULER_DEFAULT_LOOP_RATE),
```

So, unless changed, the base SystemID loop is expected to run at 400 Hz.

### SystemID log decimation
`ModeSystemId::run()` applies a subsampling divisor:

```cpp
if (log_subsample <= 0) {
    log_data();
    if (copter.should_log(MASK_LOG_ATTITUDE_FAST) && copter.should_log(MASK_LOG_ATTITUDE_MED)) {
        log_subsample = 1;
    } else if (copter.should_log(MASK_LOG_ATTITUDE_FAST)) {
        log_subsample = 2;
    } else if (copter.should_log(MASK_LOG_ATTITUDE_MED)) {
        log_subsample = 4;
    } else {
        log_subsample = 8;
    }
}
log_subsample -= 1;
```

This yields the following expected logging frequencies when the base loop is 400 Hz:

| Logging bits enabled | Effective divisor | Expected `RATE` / `SIDD` frequency |
| --- | ---: | ---: |
| `ATTITUDE_FAST` and `ATTITUDE_MED` | 1 | 400 Hz |
| `ATTITUDE_FAST` only | 2 | 200 Hz |
| `ATTITUDE_MED` only | 4 | 100 Hz |
| neither | 8 | 50 Hz |

### `RATE` versus `SIDD` cadence
Both are emitted from the same `log_data()` call, so their nominal cadence is the same.

This is also consistent with the public documentation, which states that the logging rate is set by `ATTITUDE_FAST` and `ATTITUDE_MED` and equals the main-loop frequency divided by the sub-sample factor.

However, `SIDD` is only written when both `delta_angle_dt` and `delta_velocity_dt` are positive:

```cpp
if (is_positive(delta_angle_dt) && is_positive(delta_velocity_dt)) {
    copter.Log_Write_SysID_Data(...);
}
```

`RATE` is still written immediately afterward via `Log_Write_Attitude()`.

So:

- expected nominal rate: same,
- occasional missing `SIDD` sample: possible,
- `RATE` is the more robust stream if a uniformly present signal is required.

## Practical Guidance For `SID_AXIS = 10`

For plant identification:

- use `SIDD.Targ` when you want the pure injected chirp,
- use `RATE.ROut` when you want the total commanded roll-axis mixer input,
- use `RATE.R` or `SIDD.Gx` as measured roll-rate output,
- align by `time_us`, not by row index.

If you specifically want the plant as seen by the mixer-axis roll command, `RATE.ROut -> RATE.R` or `RATE.ROut -> SIDD.Gx` is valid, but the timestamps should be aligned explicitly.