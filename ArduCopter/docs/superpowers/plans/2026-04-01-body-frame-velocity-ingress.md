# Body-Frame Velocity Ingress Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a direct body-frame velocity ingress path (`writeBodyFrameVel()`) from MAVLink ODOMETRY through AP_VisualOdom, AP_AHRS, and NavEKF3 into the existing `storedBodyOdm` buffer, then update FlowHold to consume body velocity via a canonical AHRS output API (`get_velocity_body()`).

**Architecture:** The new `writeBodyFrameVel()` path mirrors `writeBodyFrameOdom()` at every layer (GCS_MAVLink → AP_VisualOdom → AP_AHRS → NavEKF3 → NavEKF3_core) but skips the `delPos/delAng → vel/angRate` conversion, instead writing velocity directly into `bodyOdmDataNew`. A new `RBVH` DAL log struct enables replay. FlowHold replaces its local earth-to-body conversion with `AP_AHRS::get_velocity_body()`, which wraps the EKF's canonical nav→body mapping. `FuseBodyVel()` and all fusion math are left untouched.

**Tech Stack:** C++17, ArduPilot 4.6 codebase, MAVLink v2, EKF3 (`EK3_FEATURE_BODY_ODOM`), AP_DAL replay system.

---

## File Map

| File | Change |
|------|--------|
| `libraries/GCS_MAVLink/GCS_Common.cpp` | Modify `handle_odometry()`: detect finite `rollspeed/pitchspeed/yawspeed` → route to new `handle_body_frame_velocity_estimate()` instead of NED conversion |
| `libraries/AP_VisualOdom/AP_VisualOdom.h` | Add public `handle_body_frame_velocity_estimate()` declaration |
| `libraries/AP_VisualOdom/AP_VisualOdom.cpp` | Implement frontend dispatch |
| `libraries/AP_VisualOdom/AP_VisualOdom_Backend.h` | Add virtual `handle_body_frame_velocity_estimate()` default (no-op) + `Write_VisualBodyVelocity()` logging helper declaration |
| `libraries/AP_VisualOdom/AP_VisualOdom_MAV.h` | Add override declaration |
| `libraries/AP_VisualOdom/AP_VisualOdom_MAV.cpp` | Implement: quality gate, call `AP::ahrs().writeBodyFrameVel()`, log |
| `libraries/AP_VisualOdom/AP_VisualOdom_Logging.cpp` | Add `Write_VisualBodyVelocity()` implementation |
| `libraries/AP_VisualOdom/LogStructure.h` | Add `log_VisualBodyVelocity` struct + `LOG_VISBODYVEL_MSG` |
| `libraries/AP_AHRS/AP_AHRS.h` | Add `writeBodyFrameVel()` + `get_velocity_body()` declarations |
| `libraries/AP_AHRS/AP_AHRS.cpp` | Implement both |
| `libraries/AP_NavEKF3/AP_NavEKF3.h` | Add `writeBodyFrameVel()` declaration |
| `libraries/AP_NavEKF3/AP_NavEKF3.cpp` | Implement: DAL log + per-core dispatch |
| `libraries/AP_NavEKF3/AP_NavEKF3_core.h` | Declare `writeBodyFrameVel()` beside `writeBodyFrameOdom()` |
| `libraries/AP_NavEKF3/AP_NavEKF3_Measurements.cpp` | Implement `NavEKF3_core::writeBodyFrameVel()` |
| `libraries/AP_DAL/AP_DAL.h` | Add `writeBodyFrameVel()` + `handle_message(log_RBVH)` declarations + `log_RBVH _RBVH` member |
| `libraries/AP_DAL/AP_DAL.cpp` | Implement both |
| `libraries/AP_DAL/LogStructure.h` | Add `log_RBVH` struct + `LOG_RBVH_MSG` enum entry + log table entry |
| `ArduCopter/mode_flowhold.cpp` | Replace `inertial_nav.get_velocity_neu_cms()` + `earth_to_body2D()` with `AP_AHRS::get_velocity_body()` |

---

## Task 1: Add DAL log struct `RBVH` and enum entry

**Files:**
- Modify: `libraries/AP_DAL/LogStructure.h`

This is the foundation — every other layer depends on the `log_RBVH` type name and `LOG_RBVH_MSG` enum.

- [ ] **Step 1: Add `LOG_RBVH_MSG` to the `LOG_IDS_FROM_DAL` macro**

In `libraries/AP_DAL/LogStructure.h` at line 41, change:
```cpp
    LOG_RBOH_MSG
```
to:
```cpp
    LOG_RBOH_MSG, \
    LOG_RBVH_MSG
```

- [ ] **Step 2: Add the `log_RBVH` struct after `log_RBOH` (after line 385)**

```cpp
// @LoggerMessage: RBVH
// @Description: Replay body frame velocity data
struct log_RBVH {
    Vector3f vel;
    float velErr;
    Vector3f angRate;
    Vector3f posOffset;
    uint32_t timeStamp_ms;
    uint16_t delay_ms;
    uint8_t _end;
};
```

- [ ] **Step 3: Add the log table entry to `LOG_STRUCTURE_FROM_DAL` macro**

After the `LOG_RBOH_MSG` entry (line 453), add:
```cpp
    { LOG_RBVH_MSG, RLOG_SIZE(RBVH),                                   \
      "RBVH", "ffffffffffffffIH", "VX,VY,VZ,VErr,WX,WY,WZ,OX,OY,OZ,TS,D", "------------", "------------" },
```

- [ ] **Step 4: Build to verify no compilation errors**

```bash
cd /home/polanco/Liberaware/ardupilot
./waf configure --board sitl 2>&1 | tail -5
./waf build --target bin/arducopter 2>&1 | tail -20
```

Expected: build passes (no references to RBVH yet, just the struct definition).

- [ ] **Step 5: Commit**

```bash
git add libraries/AP_DAL/LogStructure.h
git commit -m "dal: add RBVH log struct for body-frame velocity replay"
```

---

## Task 2: Add `AP_DAL::writeBodyFrameVel()` and replay handler

**Files:**
- Modify: `libraries/AP_DAL/AP_DAL.h`
- Modify: `libraries/AP_DAL/AP_DAL.cpp`

Pattern: mirror `writeBodyFrameOdom()` / `handle_message(log_RBOH)` exactly.

- [ ] **Step 1: Add declaration in `AP_DAL.h`**

After line 224 (`writeBodyFrameOdom` declaration), add:
```cpp
    void writeBodyFrameVel(const Vector3f &vel, float velErr,
                           const Vector3f &angRate, uint32_t timeStamp_ms,
                           uint16_t delay_ms, const Vector3f &posOffset);
```

After line 329 (`handle_message(const log_RBOH&...)`), add:
```cpp
    void handle_message(const log_RBVH &msg, NavEKF2 &ekf2, NavEKF3 &ekf3);
```

After line 355 (`struct log_RBOH _RBOH;`), add:
```cpp
    struct log_RBVH _RBVH;
```

- [ ] **Step 2: Implement `writeBodyFrameVel()` in `AP_DAL.cpp`**

After the `writeBodyFrameOdom()` implementation (after line ~423), add:

```cpp
void AP_DAL::writeBodyFrameVel(const Vector3f &vel, float velErr,
                               const Vector3f &angRate, uint32_t timeStamp_ms,
                               uint16_t delay_ms, const Vector3f &posOffset)
{
    end_frame();

    const log_RBVH old = _RBVH;
    _RBVH.vel = vel;
    _RBVH.velErr = velErr;
    _RBVH.angRate = angRate;
    _RBVH.posOffset = posOffset;
    _RBVH.timeStamp_ms = timeStamp_ms;
    _RBVH.delay_ms = delay_ms;
    WRITE_REPLAY_BLOCK_IFCHANGED(RBVH, _RBVH, old);
}
```

- [ ] **Step 3: Implement `handle_message(const log_RBVH&)` in `AP_DAL.cpp`**

After the `handle_message(log_RBOH)` implementation (after line ~518), add:

```cpp
/*
  handle body frame velocity data
*/
void AP_DAL::handle_message(const log_RBVH &msg, NavEKF2 &ekf2, NavEKF3 &ekf3)
{
    _RBVH = msg;
    // note that EKF2 does not support body frame odometry
    ekf3.writeBodyFrameVel(msg.vel, msg.velErr, msg.angRate, msg.timeStamp_ms, msg.delay_ms, msg.posOffset);
}
```

- [ ] **Step 4: Build**

```bash
./waf build --target bin/arducopter 2>&1 | tail -20
```

Expected: fails with `NavEKF3::writeBodyFrameVel` undeclared — that's correct, confirms the DAL compiles and the next layer is needed.

- [ ] **Step 5: Commit**

```bash
git add libraries/AP_DAL/AP_DAL.h libraries/AP_DAL/AP_DAL.cpp
git commit -m "dal: add writeBodyFrameVel and RBVH replay handler"
```

---

## Task 3: Add `NavEKF3::writeBodyFrameVel()` (wrapper layer)

**Files:**
- Modify: `libraries/AP_NavEKF3/AP_NavEKF3.h`
- Modify: `libraries/AP_NavEKF3/AP_NavEKF3.cpp`

Pattern: mirrors `NavEKF3::writeBodyFrameOdom()` at lines 1648–1656 of `AP_NavEKF3.cpp`.

- [ ] **Step 1: Declare in `AP_NavEKF3.h`**

After line 214 (`writeBodyFrameOdom` declaration), add:
```cpp
    // write body frame velocity from direct velocity measurement (e.g. radar)
    void writeBodyFrameVel(const Vector3f &vel, float velErr,
                           const Vector3f &angRate, uint32_t timeStamp_ms,
                           uint16_t delay_ms, const Vector3f &posOffset);
```

- [ ] **Step 2: Implement in `AP_NavEKF3.cpp`**

After the `writeBodyFrameOdom()` implementation block (after line ~1656), add:

```cpp
void NavEKF3::writeBodyFrameVel(const Vector3f &vel, float velErr,
                                const Vector3f &angRate, uint32_t timeStamp_ms,
                                uint16_t delay_ms, const Vector3f &posOffset)
{
    dal.writeBodyFrameVel(vel, velErr, angRate, timeStamp_ms, delay_ms, posOffset);

    if (core) {
        for (uint8_t i=0; i<num_cores; i++) {
            core[i].writeBodyFrameVel(vel, velErr, angRate, timeStamp_ms, delay_ms, posOffset);
        }
    }
}
```

- [ ] **Step 3: Build**

```bash
./waf build --target bin/arducopter 2>&1 | tail -20
```

Expected: fails with `NavEKF3_core::writeBodyFrameVel` undeclared.

- [ ] **Step 4: Commit**

```bash
git add libraries/AP_NavEKF3/AP_NavEKF3.h libraries/AP_NavEKF3/AP_NavEKF3.cpp
git commit -m "navekf3: add writeBodyFrameVel wrapper"
```

---

## Task 4: Add `NavEKF3_core::writeBodyFrameVel()` (core measurement ingress)

**Files:**
- Modify: `libraries/AP_NavEKF3/AP_NavEKF3_core.h`
- Modify: `libraries/AP_NavEKF3/AP_NavEKF3_Measurements.cpp`

This is the heart of the implementation. Mirrors `writeBodyFrameOdom()` (lines 112–141 of `AP_NavEKF3_Measurements.cpp`) but skips the `delPos/dt` and `delAng/dt` conversion — the caller already provides velocity and angular rate directly.

- [ ] **Step 1: Declare in `AP_NavEKF3_core.h`**

After line 295 (`writeBodyFrameOdom` declaration), add:
```cpp
    void writeBodyFrameVel(const Vector3f &vel, float velErr,
                           const Vector3f &angRate, uint32_t timeStamp_ms,
                           uint16_t delay_ms, const Vector3f &posOffset);
```

- [ ] **Step 2: Implement in `AP_NavEKF3_Measurements.cpp`**

After the `writeBodyFrameOdom()` implementation block (after line ~141), add:

```cpp
/*
  write body frame velocity measurement directly (e.g. from radar).
  vel is in body FRD frame (m/s).
  velErr is 1-sigma velocity error (m/s).
  angRate is body angular rate from the odometry sensor (rad/s).
  posOffset is sensor position in body frame (m).
  timeStamp_ms is sensor timestamp.
  delay_ms is sensor pipeline latency.
*/
void NavEKF3_core::writeBodyFrameVel(const Vector3f &vel, float velErr,
                                     const Vector3f &angRate, uint32_t timeStamp_ms,
                                     uint16_t delay_ms, const Vector3f &posOffset)
{
#if EK3_FEATURE_BODY_ODOM
    // reject NaN inputs
    if (vel.is_nan() || isnan(velErr) || angRate.is_nan() || posOffset.is_nan()) {
        return;
    }

    // rate limiting — share the same gate as writeBodyFrameOdom
    if (((timeStamp_ms - bodyOdmMeasTime_ms) < frontend->sensorIntervalMin_ms) ||
        !statesInitialised) {
        return;
    }

    // subtract sensor pipeline latency
    timeStamp_ms -= delay_ms;

    bodyOdmDataNew.body_offset = posOffset.toftype();
    bodyOdmDataNew.vel         = vel.toftype();
    bodyOdmDataNew.angRate     = angRate.toftype();
    bodyOdmDataNew.velErr      = velErr;
    bodyOdmDataNew.time_ms     = timeStamp_ms;

    bodyOdmMeasTime_ms = timeStamp_ms;

    storedBodyOdm.push(bodyOdmDataNew);
#endif
}
```

Key difference from `writeBodyFrameOdom()`:
- No `delTime` parameter — no division needed.
- `velErr` is passed in directly (caller is AP_VisualOdom_MAV, which uses `get_vel_noise()`).
- No quality-dependent error scaling (the caller passes the already-configured noise directly).

- [ ] **Step 3: Build**

```bash
./waf build --target bin/arducopter 2>&1 | tail -20
```

Expected: builds clean (DAL + NavEKF3 layers now link).

- [ ] **Step 4: Commit**

```bash
git add libraries/AP_NavEKF3/AP_NavEKF3_core.h libraries/AP_NavEKF3/AP_NavEKF3_Measurements.cpp
git commit -m "navekf3: implement writeBodyFrameVel core measurement ingress"
```

---

## Task 5: Add `AP_AHRS::writeBodyFrameVel()` and `AP_AHRS::get_velocity_body()`

**Files:**
- Modify: `libraries/AP_AHRS/AP_AHRS.h`
- Modify: `libraries/AP_AHRS/AP_AHRS.cpp`

Two independent additions in one task because they're both thin AHRS bridges.

- [ ] **Step 1: Declare both in `AP_AHRS.h`**

After line 300 (`writeBodyFrameOdom` declaration), add:
```cpp
    // write body frame velocity measurement (direct velocity, no delta conversion)
    void writeBodyFrameVel(const Vector3f &vel, float err,
                           const Vector3f &angRate, uint32_t timeStamp_ms,
                           uint16_t delay_ms, const Vector3f &posOffset);
```

After line 309 (`writeExtNavVelData` declaration), add:
```cpp
    // get estimated velocity in body FRD frame: x=forward, y=right, z=down (m/s)
    // returns true if estimate is valid
    bool get_velocity_body(Vector3f &vel) const;
```

- [ ] **Step 2: Implement `writeBodyFrameVel()` in `AP_AHRS.cpp`**

After the `writeBodyFrameOdom()` implementation (after line ~2415), add:

```cpp
void AP_AHRS::writeBodyFrameVel(const Vector3f &vel, float err,
                                const Vector3f &angRate, uint32_t timeStamp_ms,
                                uint16_t delay_ms, const Vector3f &posOffset)
{
#if HAL_NAVEKF3_AVAILABLE
    EKF3.writeBodyFrameVel(vel, err, angRate, timeStamp_ms, delay_ms, posOffset);
#endif
}
```

- [ ] **Step 3: Implement `get_velocity_body()` in `AP_AHRS.cpp`**

Find the `earth_to_body2D()` implementation area (around line 2600+) and add nearby:

```cpp
// get estimated velocity in body FRD frame (m/s): x=forward, y=right, z=down
bool AP_AHRS::get_velocity_body(Vector3f &vel) const
{
    Vector3f vel_ned;
    if (!get_velocity_NED(vel_ned)) {
        return false;
    }
    // rotate NED velocity into body frame using current attitude
    const Matrix3f &rot = get_rotation_body_to_ned();
    vel = rot.mul_transpose(vel_ned);
    return true;
}
```

Note: `get_rotation_body_to_ned()` returns the DCM (body→NED). `mul_transpose()` multiplies by its transpose (NED→body), giving velocity in body frame.

- [ ] **Step 4: Build**

```bash
./waf build --target bin/arducopter 2>&1 | tail -20
```

Expected: builds clean.

- [ ] **Step 5: Commit**

```bash
git add libraries/AP_AHRS/AP_AHRS.h libraries/AP_AHRS/AP_AHRS.cpp
git commit -m "ahrs: add writeBodyFrameVel bridge and get_velocity_body() output API"
```

---

## Task 6: Add visual odometry body-velocity log message

**Files:**
- Modify: `libraries/AP_VisualOdom/LogStructure.h`
- Modify: `libraries/AP_VisualOdom/AP_VisualOdom_Backend.h`
- Modify: `libraries/AP_VisualOdom/AP_VisualOdom_Logging.cpp`

This adds the `VISBV` log message for in-flight validation of the new route.

- [ ] **Step 1: Add `LOG_VISBODYVEL_MSG` to `LOG_IDS_FROM_VISUALODOM` in `LogStructure.h`**

Change:
```cpp
#define LOG_IDS_FROM_VISUALODOM \
    LOG_VISUALODOM_MSG, \
    LOG_VISUALPOS_MSG, \
    LOG_VISUALVEL_MSG
```
to:
```cpp
#define LOG_IDS_FROM_VISUALODOM \
    LOG_VISUALODOM_MSG, \
    LOG_VISUALPOS_MSG, \
    LOG_VISUALVEL_MSG, \
    LOG_VISBODYVEL_MSG
```

- [ ] **Step 2: Add `log_VisualBodyVelocity` struct in `LogStructure.h`**

After the `log_VisualVelocity` struct (after line 93), add:

```cpp
// @LoggerMessage: VISBV
// @Description: Vision Body-Frame Velocity (radar RIO route)
// @Field: TimeUS: System time
// @Field: RTimeUS: Remote system time
// @Field: CTimeMS: Corrected system time
// @Field: VX: Body forward velocity (m/s)
// @Field: VY: Body right velocity (m/s)
// @Field: VZ: Body down velocity (m/s)
// @Field: WX: Body roll rate (rad/s)
// @Field: WY: Body pitch rate (rad/s)
// @Field: WZ: Body yaw rate (rad/s)
// @Field: VErr: Velocity estimate error (m/s)
// @Field: Rst: Reset counter
// @Field: Ign: Ignored (quality below threshold)
// @Field: Q: Quality
struct PACKED log_VisualBodyVelocity {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint64_t remote_time_us;
    uint32_t time_ms;
    float vel_x;
    float vel_y;
    float vel_z;
    float ang_x;
    float ang_y;
    float ang_z;
    float vel_err;
    uint8_t reset_counter;
    uint8_t ignored;
    int8_t quality;
};
```

- [ ] **Step 3: Add log table entry in `LOG_STRUCTURE_FROM_VISUALODOM` macro**

After the `LOG_VISUALVEL_MSG` entry (line 102), add:
```cpp
    { LOG_VISBODYVEL_MSG, sizeof(log_VisualBodyVelocity), \
      "VISBV", "QQIffffffBBb", "TimeUS,RTimeUS,CTimeMS,VX,VY,VZ,WX,WY,WZ,VErr,Rst,Ign,Q", "sssnnnnnn--%", "FFC000000--0" },
```

- [ ] **Step 4: Declare `Write_VisualBodyVelocity()` in `AP_VisualOdom_Backend.h`**

After line 72 (`Write_VisualVelocity` declaration), add:
```cpp
    void Write_VisualBodyVelocity(uint64_t remote_time_us, uint32_t time_ms,
                                  const Vector3f &vel, const Vector3f &ang_rate,
                                  float vel_err, uint8_t reset_counter,
                                  bool ignored, int8_t quality);
```

- [ ] **Step 5: Implement `Write_VisualBodyVelocity()` in `AP_VisualOdom_Logging.cpp`**

After the `Write_VisualVelocity()` function (after line 66), add:

```cpp
// Write body-frame velocity sensor data (RIO radar route), velocity in body FRD m/s
void AP_VisualOdom_Backend::Write_VisualBodyVelocity(uint64_t remote_time_us, uint32_t time_ms,
                                                     const Vector3f &vel, const Vector3f &ang_rate,
                                                     float vel_err, uint8_t reset_counter,
                                                     bool ignored, int8_t quality)
{
    const struct log_VisualBodyVelocity pkt {
        LOG_PACKET_HEADER_INIT(LOG_VISBODYVEL_MSG),
        time_us         : AP_HAL::micros64(),
        remote_time_us  : remote_time_us,
        time_ms         : time_ms,
        vel_x           : vel.x,
        vel_y           : vel.y,
        vel_z           : vel.z,
        ang_x           : ang_rate.x,
        ang_y           : ang_rate.y,
        ang_z           : ang_rate.z,
        vel_err         : vel_err,
        reset_counter   : reset_counter,
        ignored         : (uint8_t)ignored,
        quality         : quality
    };
    AP::logger().WriteBlock(&pkt, sizeof(log_VisualBodyVelocity));
}
```

- [ ] **Step 6: Build**

```bash
./waf build --target bin/arducopter 2>&1 | tail -20
```

Expected: builds clean.

- [ ] **Step 7: Commit**

```bash
git add libraries/AP_VisualOdom/LogStructure.h libraries/AP_VisualOdom/AP_VisualOdom_Backend.h libraries/AP_VisualOdom/AP_VisualOdom_Logging.cpp
git commit -m "visualodom: add VISBV log message for body-frame velocity route"
```

---

## Task 7: Add `handle_body_frame_velocity_estimate()` to AP_VisualOdom frontend and MAV backend

**Files:**
- Modify: `libraries/AP_VisualOdom/AP_VisualOdom.h`
- Modify: `libraries/AP_VisualOdom/AP_VisualOdom.cpp`
- Modify: `libraries/AP_VisualOdom/AP_VisualOdom_Backend.h`
- Modify: `libraries/AP_VisualOdom/AP_VisualOdom_MAV.h`
- Modify: `libraries/AP_VisualOdom/AP_VisualOdom_MAV.cpp`

- [ ] **Step 1: Declare frontend method in `AP_VisualOdom.h`**

After line 107 (`handle_vision_speed_estimate` declaration), add:
```cpp
    // consume body-frame velocity from radar/odometry sensor and send to EKF
    // vel is in body FRD frame (m/s), ang_rate is body angular rate (rad/s)
    // quality of -1 means failed, 0 means unknown, 1 is worst, 100 is best
    void handle_body_frame_velocity_estimate(uint64_t remote_time_us, uint32_t time_ms,
                                             const Vector3f &vel,
                                             const Vector3f &ang_rate,
                                             uint8_t reset_counter,
                                             int8_t quality);
```

- [ ] **Step 2: Implement in `AP_VisualOdom.cpp`**

After the `handle_vision_speed_estimate()` implementation (after line ~246), add:

```cpp
void AP_VisualOdom::handle_body_frame_velocity_estimate(uint64_t remote_time_us, uint32_t time_ms,
                                                        const Vector3f &vel,
                                                        const Vector3f &ang_rate,
                                                        uint8_t reset_counter,
                                                        int8_t quality)
{
    if (!enabled()) {
        return;
    }
    if (_driver != nullptr) {
        _driver->handle_body_frame_velocity_estimate(remote_time_us, time_ms, vel, ang_rate, reset_counter, quality);
    }
}
```

- [ ] **Step 3: Add default virtual to `AP_VisualOdom_Backend.h`**

After line 47 (`handle_vision_speed_estimate` virtual declaration), add:
```cpp
    // consume body-frame velocity and send to EKF; default no-op for backends that don't support it
    virtual void handle_body_frame_velocity_estimate(uint64_t remote_time_us, uint32_t time_ms,
                                                     const Vector3f &vel,
                                                     const Vector3f &ang_rate,
                                                     uint8_t reset_counter,
                                                     int8_t quality) {}
```

- [ ] **Step 4: Declare override in `AP_VisualOdom_MAV.h`**

After line 22 (`handle_vision_speed_estimate` override declaration), add:
```cpp
    // consume body-frame velocity estimate and send to EKF
    void handle_body_frame_velocity_estimate(uint64_t remote_time_us, uint32_t time_ms,
                                             const Vector3f &vel,
                                             const Vector3f &ang_rate,
                                             uint8_t reset_counter,
                                             int8_t quality) override;
```

- [ ] **Step 5: Implement in `AP_VisualOdom_MAV.cpp`**

After the `handle_vision_speed_estimate()` implementation (after line ~79), add:

```cpp
// consume body-frame velocity from radar and send to EKF
// vel is body FRD (m/s), ang_rate is body angular rate (rad/s)
void AP_VisualOdom_MAV::handle_body_frame_velocity_estimate(uint64_t remote_time_us, uint32_t time_ms,
                                                            const Vector3f &vel,
                                                            const Vector3f &ang_rate,
                                                            uint8_t reset_counter,
                                                            int8_t quality)
{
    _quality = quality;
    const bool consume = (_quality >= _frontend.get_quality_min());
    if (consume) {
        const float vel_err = _frontend.get_vel_noise();
        const Vector3f pos_offset = _frontend.get_pos_offset();
        AP::ahrs().writeBodyFrameVel(vel, vel_err, ang_rate, time_ms, _frontend.get_delay_ms(), pos_offset);
    }
    _last_update_ms = AP_HAL::millis();
#if HAL_LOGGING_ENABLED
    Write_VisualBodyVelocity(remote_time_us, time_ms, vel, ang_rate,
                             _frontend.get_vel_noise(), reset_counter, !consume, _quality);
#endif
}
```

- [ ] **Step 6: Build**

```bash
./waf build --target bin/arducopter 2>&1 | tail -20
```

Expected: builds clean.

- [ ] **Step 7: Commit**

```bash
git add libraries/AP_VisualOdom/AP_VisualOdom.h libraries/AP_VisualOdom/AP_VisualOdom.cpp \
        libraries/AP_VisualOdom/AP_VisualOdom_Backend.h \
        libraries/AP_VisualOdom/AP_VisualOdom_MAV.h libraries/AP_VisualOdom/AP_VisualOdom_MAV.cpp
git commit -m "visualodom: add handle_body_frame_velocity_estimate through MAV backend"
```

---

## Task 8: Modify `GCS_MAVLINK::handle_odometry()` to route body-twist messages

**Files:**
- Modify: `libraries/GCS_MAVLink/GCS_Common.cpp`

The routing rule: if `rollspeed`, `pitchspeed`, and `yawspeed` are all finite (not NaN), take the body-velocity route instead of the NED conversion route. This avoids a new parameter while preserving existing behaviour for ODOMETRY messages that don't include angular rates.

- [ ] **Step 1: Modify `handle_odometry()` at line 3876**

Replace the current `handle_odometry()` body (lines 3876–3908):

```cpp
void GCS_MAVLINK::handle_odometry(const mavlink_message_t &msg)
{
    AP_VisualOdom *visual_odom = AP::visualodom();
    if (visual_odom == nullptr) {
        return;
    }

    mavlink_odometry_t m;
    mavlink_msg_odometry_decode(&msg, &m);

    if (m.frame_id != MAV_FRAME_LOCAL_FRD ||
        m.child_frame_id != MAV_FRAME_BODY_FRD) {
        // only support local FRD frame data
        return;
    }

    const uint32_t timestamp_ms = correct_offboard_timestamp_usec_to_ms(m.time_usec, PAYLOAD_SIZE(chan, ODOMETRY));

    // If all three angular-rate fields are finite, treat this as a body-twist
    // message (e.g. RIO radar) and route to the body-frame velocity ingress.
    // Do NOT call handle_pose_estimate() on this route — there is no valid pose.
    if (!isnan(m.rollspeed) && !isnan(m.pitchspeed) && !isnan(m.yawspeed)) {
        const Vector3f vel_bf{m.vx, m.vy, m.vz};
        const Vector3f ang_rate{m.rollspeed, m.pitchspeed, m.yawspeed};
        visual_odom->handle_body_frame_velocity_estimate(m.time_usec, timestamp_ms,
                                                         vel_bf, ang_rate,
                                                         m.reset_counter, m.quality);
        return;
    }

    // Legacy path: pose + NED velocity (angular rates not present or NaN)
    Quaternion q{m.q[0],m.q[1],m.q[2],m.q[3]};

    float posErr = 0;
    float angErr = 0;
    if (!isnan(m.pose_covariance[0])) {
        posErr = cbrtf(sq(m.pose_covariance[0])+sq(m.pose_covariance[6])+sq(m.pose_covariance[11]));
        angErr = cbrtf(sq(m.pose_covariance[15])+sq(m.pose_covariance[18])+sq(m.pose_covariance[20]));
    }

    visual_odom->handle_pose_estimate(m.time_usec, timestamp_ms, m.x, m.y, m.z, q, posErr, angErr, m.reset_counter, m.quality);

    // convert velocity vector from FRD to NED frame
    Vector3f vel{m.vx, m.vy, m.vz};
    vel = q * vel;
    visual_odom->handle_vision_speed_estimate(m.time_usec, timestamp_ms, vel, m.reset_counter, m.quality);
}
```

- [ ] **Step 2: Build**

```bash
./waf build --target bin/arducopter 2>&1 | tail -20
```

Expected: builds clean. Full ingress path is now wired end-to-end.

- [ ] **Step 3: Commit**

```bash
git add libraries/GCS_MAVLink/GCS_Common.cpp
git commit -m "gcs: route ODOMETRY body-twist to writeBodyFrameVel when angular rates present"
```

---

## Task 9: Refactor FlowHold to use `AP_AHRS::get_velocity_body()`

**Files:**
- Modify: `ArduCopter/mode_flowhold.cpp`

Replace the `inertial_nav.get_velocity_neu_cms()` → `earth_to_body2D()` round-trip at lines 115–125 with a direct call to `ahrs.get_velocity_body()`. The sensor_flow axis mapping and everything downstream stays identical.

- [ ] **Step 1: Replace lines 114–125 in `mode_flowhold.cpp`**

Current code (lines 114–125):
```cpp
    // Get EKF horizontal velocity in earth NE frame (m/s).
    const Vector3f vel_neu_cms = copter.inertial_nav.get_velocity_neu_cms();
    Vector2f vel_ef(vel_neu_cms.x * 0.01f, vel_neu_cms.y * 0.01f);

    // Rotate to body frame: earth_to_body2D produces [v_forward, v_right].
    // The optical-flow sensor convention (AP_OpticalFlow_SITL.cpp:89-90) is:
    //   flowRate.x = -v_right   (negative right = positive roll-axis flow)
    //   flowRate.y = +v_forward  (forward = positive pitch-axis flow)
    // Apply the same axis mapping so sensor_flow is in the exact same frame
    // the original code used — clamp, filter, braking, and PI all unchanged.
    const Vector2f vel_bf = copter.ahrs.earth_to_body2D(vel_ef);
    Vector2f sensor_flow(-vel_bf.y, vel_bf.x);
```

Replace with:
```cpp
    // Get body-frame velocity from the canonical estimator output.
    // x=forward, y=right, z=down (body FRD, m/s).
    // If unavailable (EKF not healthy), fall back to zero so the PI integrator
    // drains rather than holding a stale correction.
    Vector3f vel_body;
    if (!copter.ahrs.get_velocity_body(vel_body)) {
        vel_body.zero();
    }

    // Apply the optical-flow axis mapping used throughout FlowHold:
    //   sensor_flow.x = -v_right   (negative right = positive roll-axis flow)
    //   sensor_flow.y = +v_forward  (positive forward = positive pitch-axis flow)
    Vector2f sensor_flow(-vel_body.y, vel_body.x);
```

- [ ] **Step 2: Build**

```bash
./waf build --target bin/arducopter 2>&1 | tail -20
```

Expected: builds clean.

- [ ] **Step 3: Run SITL smoke test to confirm FlowHold still compiles and links**

```bash
cd /home/polanco/Liberaware/ardupilot
sim_vehicle.py -v ArduCopter --no-rebuild -w --speedup=1 --map --console &
sleep 10
mavproxy.py --master=udp:127.0.0.1:14550 --cmd="mode flowhold; arm throttle; rc 3 1600; sleep 3; disarm" 2>&1 | tail -30
```

Expected: mode change accepted, no arming failures related to FlowHold. (Full flight test requires real EKF aiding; this just checks control-path compilation.)

- [ ] **Step 4: Commit**

```bash
git add ArduCopter/mode_flowhold.cpp
git commit -m "flowhold: replace earth/body round-trip with ahrs.get_velocity_body()"
```

---

## Task 10: Final build and verification

- [ ] **Step 1: Full clean build**

```bash
cd /home/polanco/Liberaware/ardupilot
./waf distclean
./waf configure --board sitl
./waf build --target bin/arducopter 2>&1 | tail -30
```

Expected: zero errors, zero warnings about the new symbols.

- [ ] **Step 2: Verify ingress route with SITL + MAVLink injection**

Start SITL:
```bash
sim_vehicle.py -v ArduCopter -f quad --console --map
```

In a second terminal, send a test ODOMETRY message with angular rates set:
```python
import pymavlink.mavutil as mavutil
import time, math

conn = mavutil.mavlink_connection('udp:127.0.0.1:14550')
conn.wait_heartbeat()

while True:
    conn.mav.odometry_send(
        time_usec=int(time.time() * 1e6),
        frame_id=12,       # MAV_FRAME_LOCAL_FRD
        child_frame_id=14, # MAV_FRAME_BODY_FRD
        x=0, y=0, z=0,
        q=[1.0, 0.0, 0.0, 0.0],
        vx=0.5, vy=0.0, vz=0.0,
        rollspeed=0.01, pitchspeed=0.01, yawspeed=0.01,
        pose_covariance=[float('nan')]*21,
        velocity_covariance=[float('nan')]*21,
        reset_counter=0,
        estimator_type=0,
        quality=80
    )
    time.sleep(0.05)
```

Expected observable in MAVProxy logs:
- `VISBV` log messages appear (not `VISV`)
- `VISP` (visual position) does NOT appear (pose path skipped)
- `RBVH` DAL log entries appear

- [ ] **Step 3: Verify legacy ODOMETRY path still works**

Send the same message but with `rollspeed=NaN, pitchspeed=NaN, yawspeed=NaN`:
```python
conn.mav.odometry_send(
    time_usec=int(time.time() * 1e6),
    frame_id=12, child_frame_id=14,
    x=1.0, y=0.5, z=-1.2,
    q=[1.0, 0.0, 0.0, 0.0],
    vx=0.3, vy=0.1, vz=0.0,
    rollspeed=float('nan'), pitchspeed=float('nan'), yawspeed=float('nan'),
    pose_covariance=[0.01]+[0.0]*20,
    velocity_covariance=[float('nan')]*21,
    reset_counter=0,
    estimator_type=0,
    quality=80
)
```

Expected: `VISP` and `VISV` log entries appear (legacy path preserved), no `VISBV`.

- [ ] **Step 4: Commit verification note**

```bash
git commit --allow-empty -m "verified: body-vel ingress route and legacy pose route both functional in SITL"
```

---

## Spec Coverage Check

| Spec requirement (§9.3) | Task |
|--------------------------|------|
| A. GCS handle_odometry body-twist routing | Task 8 |
| B. AP_VisualOdom frontend `handle_body_frame_velocity_estimate` | Task 7 |
| C. AP_VisualOdom_Backend virtual default | Task 7 |
| D. AP_VisualOdom_MAV backend implementation | Task 7 |
| E. AP_AHRS `writeBodyFrameVel` bridge | Task 5 |
| F. NavEKF3 wrapper `writeBodyFrameVel` + DAL log | Tasks 3, 2 |
| G. NavEKF3_core declaration | Task 4 |
| H. NavEKF3_core implementation (no fake delPos) | Task 4 |
| I. DAL replay support (`RBVH`) | Tasks 1, 2 |
| J. VisualOdom body-velocity log (`VISBV`) | Task 6 |
| K. AP_AHRS `get_velocity_body()` | Task 5 |
| L. FlowHold body-axis refactor | Task 9 |

All 12 spec items covered.
