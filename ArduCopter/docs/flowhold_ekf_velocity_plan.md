# FlowHold EKF Velocity Adaptation: Plan and Results

## Goal

### 1. Overall project goal

Enable reliable hover and maneuvering in GPS-denied indoor environments using the RIO companion-computer stack for navigation estimates while keeping all flight control loops on the flight controller. The companion provides state estimates only; it does not send control commands.

### 2. Specific FlowHold adaptation goal

Adapt ArduCopter FlowHold so its horizontal velocity-hold loop uses EKF3 velocity fused from EXTNAV instead of optical flow. The target remains the existing FlowHold mode 22 because it already provides the required architecture: braking on stick release, velocity correction on top of pilot lean commands, and RC throttle for altitude. No new flight mode or GCS mode plumbing is required.

## Expected Outcome

With `EK3_SRC1_VELXY=6` and `EK3_SRC1_POSXY=6`, switching to FlowHold should produce:

- Centered roll and pitch sticks: brake to zero horizontal velocity
- Roll and pitch stick input: pilot lean angles with velocity correction added on top
- Throttle input: normal pilot-controlled altitude response via the barometer path
- EXTNAV dropout: velocity correction disabled and behavior reduced to AltHold-like response

The mode intentionally holds velocity, not position. That is the desired behavior for this project because position hold would fight the pilot when RIO position estimates drift or reset.

```cpp
// BEFORE:
// @Param: _FLOW_MAX
// @DisplayName: FlowHold Flow Rate Max
// @Description: Controls maximum apparent flow rate in flowhold
// @Range: 0.1 2.5
// @User: Standard
AP_GROUPINFO("_FLOW_MAX", 2, ModeFlowHold, flow_max, 0.6),

// AFTER:
// @Param: _FLOW_MAX
// @DisplayName: FlowHold Velocity Max
// @Description: Maximum horizontal velocity (m/s) used as input to the velocity hold controller. NOTE: this parameter was previously optical-flow rate in rad/s with a default of 0.6. If upgrading from optical-flow FlowHold, reset this parameter to 2.0 or the new default will not apply.
// @Range: 0.1 5.0
// @Units: m/s
// @User: Standard
AP_GROUPINFO("_FLOW_MAX", 2, ModeFlowHold, flow_max, 2.0),
```
```

`AP_OPTICALFLOW_ENABLED` defaults to `1` in `AP_OpticalFlow_config.h` and the MATEK H743-SLIM-V4 (`MatekH743` hwdef) sets no override, so FlowHold **does** compile today on the target hardware.

**The change:** Set `MODE_FLOWHOLD_ENABLED 1` unconditionally.

**Risk:** On boards where `AP_OPTICALFLOW_ENABLED=0`, `copter.optflow` member does not exist (`Copter.h:312` guards it). If `mode_flowhold.cpp` compiled with `MODE_FLOWHOLD_ENABLED=1` but still referenced `copter.optflow`, it would fail to compile.

**Resolution:** This is safe **only after** all `copter.optflow` references are removed from `mode_flowhold.cpp`. The `AP_OpticalFlow.h` header itself is unconditionally included via `Copter.h:72`, so it is always available — only the `copter.optflow` **member** is conditionally declared. Removing member references eliminates the risk.

**All files gated on `MODE_FLOWHOLD_ENABLED` (must all remain consistent):**

| File | Lines | What it guards |
|------|-------|----------------|
| `ArduCopter/config.h` | 243–245 | The define itself |
| `ArduCopter/mode_flowhold.cpp` | 4, 519 | Entire file |
| `ArduCopter/mode.h` | 959, 1045 | `ModeFlowHold` class definition |
| `ArduCopter/Copter.h` | 1066–1068 | `ModeFlowHold mode_flowhold` instance |
| `ArduCopter/Parameters.h` | 575–577 | `void *mode_flowhold_ptr` in ParametersG2 |
| `ArduCopter/Parameters.cpp` | 883–886 | `AP_SUBGROUPPTR` registration |
| `ArduCopter/Parameters.cpp` | 1260–1261 | Constructor init of `mode_flowhold_ptr` |
| `ArduCopter/mode.cpp` | 146–149 | Dispatch case `Number::FLOWHOLD` |
| `ArduCopter/RC_Channel.cpp` | 528–531 | AUX switch `FLOWHOLD` aux function |

None of these files reference `copter.optflow` directly — they are pure plumbing. No changes needed in any of them.

---

### 2. All `copter.optflow` references — complete list

Every reference in `mode_flowhold.cpp` that must be removed or replaced:

| Line | Code | Disposition |
|------|------|-------------|
| 87 | `copter.optflow.enabled()` | **Replace** with EKF health check |
| 87 | `copter.optflow.healthy()` | **Replace** with EKF health check |
| 123 | `copter.optflow.flowRate()` | **Replace** with `inertial_nav.get_velocity_neu_cms()` |
| 123 | `copter.optflow.bodyRate()` | **Delete** (body rotation already removed in EKF velocity) |
| 256 | `copter.optflow.healthy()` | **Replace** with `flags.horiz_vel` check |
| 258 | `copter.optflow.quality()` | **Delete** (no quality metric for EKF velocity) |
| 380 | `copter.optflow.healthy()` | **Delete** (inside `update_height_estimate()` — entire function deleted) |
| 389 | `copter.optflow.last_update()` | **Delete** (same) |
| 395 | `copter.optflow.last_update()` | **Delete** (same) |
| 407 | `copter.optflow.flowRate()` | **Delete** (same) |
| 407 | `copter.optflow.bodyRate()` | **Delete** (same) |
| 409 | `copter.optflow.last_update()` | **Delete** (same) |
| 412 | `copter.optflow.last_update()` | **Delete** (same) |
| 428 | `copter.optflow.last_update()` | **Delete** (same) |

After changes: **zero** `copter.optflow` references remain.

---

### 3. EKF health gate — `flags.horiz_vel` is correct and sufficient

From `AP_NavEKF3_Control.cpp:768–777`:
```cpp
bool someHorizRefData = !(velTimeout && posTimeout && tasTimeout && dragTimeout)
                        || doingFlowNav || doingBodyVelNav;
bool filterHealthy = healthy() && tiltAlignComplete && (yawAlignComplete || ...);
status.flags.horiz_vel = someHorizRefData && filterHealthy;
```

`velTimeout` is reset to `false` whenever `VISION_SPEED_ESTIMATE` arrives and `EK3_SRC1_VELXY=6`. This makes `someHorizRefData=true` → `flags.horiz_vel=true` when the filter is healthy and EXTNAV data is flowing.

**Stock dependency (before the EKF3 change):** `readyToUseExtNav()` at `AP_NavEKF3_Control.cpp:609` required `EK3_SRC1_POSXY=6`. That forced EXTNAV position to be configured even when only EXTNAV velocity was wanted.

**Current project change:** EKF3 EXTNAV readiness is split into:

1. `readyToUseExtNavPos()`
2. `readyToUseExtNavVel()`

This allows the filter to enter the aided state using healthy EXTNAV horizontal velocity without forcing EXTNAV horizontal position into the filter. For the FlowHold operating concept, that removes the old hard dependency on `EK3_SRC1_POSXY=6`.

---

### 4. `update_height_estimate()` — must be deleted entirely

This 166-line function (`mode_flowhold.cpp:352–517`) solves:
```
height_m = delta_velocity_mps / delta_flowrate_rps
```
This is physically meaningful only with optical flow in rad/s. With EKF velocity (already in m/s), the equation is undefined. The function also owns all height-estimation state variables. It must be deleted completely along with its call at line 233 and its member variables in `mode.h`.

---

### 5. `flowhold_flow_to_angle()` — velocity source replacement

Implementation note: the analysis in this section reflects the final reviewed implementation.

Lines 122–140 are the only section that touches optical flow. The rest (PI controller, braking logic, integrator anti-windup, frame rotation at line 196) is **unit-agnostic** and unchanged.

**Frame conversion note:** Line 140 currently does:
```cpp
Vector2f input_ef = copter.ahrs.body_to_earth2D(sensor_flow);
```
Optical flow is body-frame, so a body→earth rotation is needed. `inertial_nav.get_velocity_neu_cms()` is already in **earth NE frame**, but the final EXTNAV adaptation does not feed that plain earth-frame velocity directly into the PI controller. Instead, it reconstructs the same body-frame optical-flow control basis the original FlowHold code used, then keeps the original `body_to_earth2D(sensor_flow)` call. That preserves the original PI, braking, and filter channel semantics.

---

### 6. `inertial_nav.get_velocity_neu_cms()` — safe to call, but freeze on failure

From `AP_InertialNav.cpp:32–45`: if `get_velocity_NED()` fails, horizontal velocity is **frozen at last good value** (not zeroed). The getter returns a `Vector3f` with no validity flag.

**Therefore:** The `flags.horiz_vel` check in `run()` is the correct health gate. When it goes false, the velocity correction block at line 324 is skipped and the mode degrades to pure AltHold behaviour — safe.

---

### 7. `LowPassFilterConstDtVector2f` — unit-agnostic, no changes needed

Template class (`Filter/LowPassFilter.h`) operates on `Vector2f` using exponential smoothing. Works identically on m/s velocity or rad/s flow. Filter frequency parameter (`FHLD_FILT_HZ`) retains its meaning.

---

### 8. Parameters — `flow_min_quality` removal

Removing `AP_GROUPINFO("_QUAL_MIN", ...)` from `var_info[]` orphans the `FHLD_QUAL_MIN` parameter on vehicles that previously had it saved. The AP_Param framework **silently ignores unknown parameters on load** — safe, no migration needed.

The `flow_min_quality` member variable in `mode.h` is also deleted.

---

### 9. Arming checks — no changes needed

Confirmed by searching `AP_Arming.cpp` (650 lines): **no mode-specific arming checks for FlowHold or optical flow health exist.** The only gate is `mode_flowhold.cpp:init()` returning `false` at mode entry — a post-arm, runtime check. Replacing it with a `flags.horiz_vel` check preserves this behaviour correctly.

---

### 10. Logging — semantics change, format unchanged

**`FHLD` log message** (`mode_flowhold.cpp:218–225`): format string `"TimeUS,SFx,SFy,Ax,Ay,Qual,Ix,Iy"` is unchanged. Field semantics change:
- `SFx/SFy`: was optical flow rad/s × height → now the adapted optical-flow control channels in m/s, numerically equivalent to `[-v_right, v_forward]`
- `Qual`: was optflow quality 0–255 → now EKF `horiz_vel` flag (0 or 255)

**`FHXY` log message** (`mode_flowhold.cpp:485–512`): deleted along with `update_height_estimate()`. This message logged height estimation internals — irrelevant after the change.

**GCS/MAVLink:** No FlowHold-specific mode advertisement. Mode name "FLOWHOLD"/"FHLD" in `mode.h:986–987` unchanged.

**Build system:** `ArduCopter/wscript` uses implicit glob — no explicit source file list. No changes needed.

---

## File Changes

### File 1: `ArduCopter/config.h` — line 244

```cpp
// BEFORE:
# define MODE_FLOWHOLD_ENABLED AP_OPTICALFLOW_ENABLED

// AFTER:
# define MODE_FLOWHOLD_ENABLED 1
```

---

### File 2: `ArduCopter/mode.h` — remove optflow-specific members from `ModeFlowHold`

**Delete** `flow_min_quality` parameter member:
```cpp
// DELETE (line ~1018):
AP_Int8 flow_min_quality;
```

**Delete** `update_height_estimate` method declaration:
```cpp
// DELETE (line ~1007):
void update_height_estimate(void);
```

**Delete** height estimation state variables:
```cpp
// DELETE (lines ~1028–1037):
Vector2f delta_velocity_ne;
Vector2f last_flow_rate_rps;
uint32_t last_flow_ms;
float last_ins_height;
float height_offset;
```

**Delete** height constants:
```cpp
// DELETE (lines ~1010–1013):
const float height_min = 0.1f;
const float height_max = 3.0f;
```

---

### File 3: `ArduCopter/mode_flowhold.cpp` — all changes

#### A. Remove `_QUAL_MIN` from `var_info[]` (lines 57–62)
```cpp
// DELETE:
AP_GROUPINFO("_QUAL_MIN", 4, ModeFlowHold, flow_min_quality, 10),
```

#### B. `init()` — replace optflow gate with EKF gate (lines 87–89)
```cpp
// BEFORE:
if (!copter.optflow.enabled() || !copter.optflow.healthy()) {
    return false;
}

// AFTER:
// Require EKF to have a valid horizontal velocity estimate
if (!ignore_checks && !inertial_nav.get_filter_status().flags.horiz_vel) {
    return false;
}
```

#### C. `init()` — delete height-estimation state init (lines 108–110)
```cpp
// DELETE:
last_ins_height = copter.inertial_nav.get_position_z_up_cm() * 0.01;
height_offset = 0;
```

#### D. `run()` — delete `update_height_estimate()` call (line 233)
```cpp
// DELETE:
update_height_estimate();
```

#### E. `run()` — replace quality filter block (lines 256–261)
```cpp
// BEFORE:
if (copter.optflow.healthy()) {
    const float filter_constant = 0.95;
    quality_filtered = filter_constant * quality_filtered + (1-filter_constant) * copter.optflow.quality();
} else {
    quality_filtered = 0;
}

// AFTER:
// Binary health signal: EKF has valid horizontal velocity or not
quality_filtered = inertial_nav.get_filter_status().flags.horiz_vel ? 255.0f : 0.0f;
```

Note: `quality_filtered >= flow_min_quality` at line 324 now becomes `255 >= 0` (always true when healthy) since `flow_min_quality` is removed. The condition at line 324 simplifies to:
```cpp
// BEFORE:
if (quality_filtered >= flow_min_quality &&
    AP_HAL::millis() - copter.arm_time_ms > 3000) {

// AFTER:
if (quality_filtered > 0 &&
    AP_HAL::millis() - copter.arm_time_ms > 3000) {
```

#### F. `flowhold_flow_to_angle()` — replace velocity source (lines 122–140)
```cpp
// BEFORE (lines 122–140):
Vector2f raw_flow = copter.optflow.flowRate() - copter.optflow.bodyRate();
raw_flow.x = constrain_float(raw_flow.x, -flow_max, flow_max);
raw_flow.y = constrain_float(raw_flow.y, -flow_max, flow_max);
Vector2f sensor_flow = flow_filter.apply(raw_flow);
float ins_height = copter.inertial_nav.get_position_z_up_cm() * 0.01;
float height_estimate = ins_height + height_offset;
sensor_flow *= constrain_float(height_estimate, height_min, height_max);

// rotate controller input to earth frame
Vector2f input_ef = copter.ahrs.body_to_earth2D(sensor_flow);

// AFTER:
// Get EKF horizontal velocity in earth NE frame (m/s)
const Vector3f vel_neu_cms = copter.inertial_nav.get_velocity_neu_cms();
Vector2f vel_ef(vel_neu_cms.x * 0.01f, vel_neu_cms.y * 0.01f);

// Convert to body forward/right, then remap to the legacy optical-flow
// control basis used by the original FlowHold path:
//   sensor_flow = [-v_right, +v_forward]
const Vector2f vel_bf = copter.ahrs.earth_to_body2D(vel_ef);
Vector2f sensor_flow(-vel_bf.y, vel_bf.x);
sensor_flow.x = constrain_float(sensor_flow.x, -flow_max, flow_max);
sensor_flow.y = constrain_float(sensor_flow.y, -flow_max, flow_max);
sensor_flow = flow_filter.apply(sensor_flow);

// Preserve the original FlowHold PI path
Vector2f input_ef = copter.ahrs.body_to_earth2D(sensor_flow);
```

#### G. Delete `update_height_estimate()` entirely (lines 352–517)

Delete the full 166-line function body including the `FHXY` log message it contains.

---

## Complete File Impact Summary

| File | Change type | Lines affected |
|------|-------------|----------------|
| `ArduCopter/config.h` | 1-line edit | 244 |
| `ArduCopter/mode.h` | Delete members/methods | ~1007, ~1010–1013, ~1018, ~1028–1037 |
| `ArduCopter/mode_flowhold.cpp` | Replace + delete | 57–62, 87–89, 108–110, 122–140, 233, 256–261, 324, 352–517 |
| `ArduCopter/Parameters.cpp` | No change | — |
| `ArduCopter/Parameters.h` | No change | — |
| `ArduCopter/Copter.h` | No change | — |
| `ArduCopter/mode.cpp` | No change | — |
| `ArduCopter/RC_Channel.cpp` | No change | — |
| `ArduCopter/GCS_Mavlink.cpp` | No change | — |
| `ArduCopter/AP_Arming.cpp` | No change | — |
| `ArduCopter/wscript` | No change | — |

**3 files change. 8 files verified as unaffected.**

---

## Required EKF3 Parameters

```
EK3_SRC1_VELXY = 6        # EXTNAV velocity source (required)
EK3_SRC1_VELZ  = 0        # Do not use EXTNAV for Z velocity — baro handles altitude
EK3_SRC1_POSXY = 0 or 3   # EXTNAV horizontal position no longer required for the velocity-only operating concept
EK3_SRC1_POSZ  = 1        # Barometer for altitude (Z is not from RIO in this phase)
VISO_TYPE      = 1        # Enable MAVLink vision odometry bridge
VISO_DELAY_MS  = <measured empirically>
FLOW_TYPE      = 0        # Disable optical flow sensor (not used)
FHLD_FLOW_MAX  = 2.0      # If upgrading from optical-flow FlowHold, set explicitly in Mission Planner/QGC
```

With the EKF3 EXTNAV readiness split, `VELXY=6` is the key requirement for the FlowHold-style velocity-only concept. `POSXY` no longer needs to be `6` just to satisfy EXTNAV readiness.

Important caveat: this change is intentionally narrow. It enables EKF3 to become ready using healthy EXTNAV horizontal velocity without forcing low-quality EXTNAV position into the filter. It does **not** automatically make all Copter GPS/position-requiring modes usable with velocity-only EXTNAV.

### Pre-arm implications

This change does **not** broadly relax Copter arming checks for GPS/position-requiring modes.

- `AP_Arming_Copter::mandatory_gps_checks()` still passes `mode_requires_gps` into `ahrs.pre_arm_check(...)`
- `AP_NavEKF_Source::pre_arm_check()` still validates horizontal position configuration when `requires_position` is true
- `Need Position Estimate` checks for GPS/position-requiring modes are unchanged

So the intended effect is:

1. velocity-only EXTNAV can support the FlowHold-style runtime gate
2. low-quality EXTNAV position no longer needs to be fused just to satisfy EXTNAV readiness
3. GPS/position-requiring Copter modes still keep their normal arming semantics unless explicitly redesigned later

## Implementation Results

This section records the final reviewed implementation, including the end-to-end frame path from the companion-computer EKF to ArduPilot FlowHold.

### End-to-End EXTNAV Path

The active companion path is:

1. `EkfRioFilter` estimates velocity internally in NED. See `ArduCopter/RIO/rio_ekf.md`.
2. `ekf_rio_adapter::get_nav_state(...)` converts that velocity to world FLU.
3. `rio_ros::callback_timer_()` rotates world-FLU velocity into body-FLU before publishing it in `nav_msgs/Odometry.twist.twist.linear`. See `ArduCopter/RIO/rio_data_pipeline.md`.
4. `fc_com_if::callback_odometry()` treats that twist as body-FLU, converts it to body-FRD, then rotates it into NED and publishes `VISION_SPEED_ESTIMATE`.
5. ArduPilot EKF3 fuses that external velocity as NED velocity. See `ArduCopter/docs/EKF3_DATA_FLOW.md` and `ArduCopter/docs/EKF3_CORE_MATH.md`.
6. `AP_InertialNav::get_velocity_neu_cms()` exposes the fused horizontal velocity to Copter as earth-frame NEU cm/s.

So the velocity that reaches `ModeFlowHold::flowhold_flow_to_angle()` is already a world or earth-frame horizontal velocity estimate, not a body-frame one.

### What The Adapted Optical-Flow Path Actually Does

The original FlowHold logic did not consume plain forward/right velocity. It consumed an optical-flow-derived control vector.

From the optical-flow SITL model, the translational part of the raw sensor signal is:

```text
flowRate.x - bodyRate.x ≈ -v_right / range
flowRate.y - bodyRate.y ≈ +v_forward / range
```

After the original FlowHold height scaling, the effective control vector became:

```text
sensor_flow_old ≈ [ -v_right, +v_forward ]
```

The final EXTNAV implementation reproduces that same vector from EKF velocity instead of from optical flow:

```text
vel_ef = [v_north, v_east]                         // earth frame, from EKF
vel_bf = earth_to_body2D(vel_ef)                  // [v_forward, v_right]
sensor_flow = [ -vel_bf.y, vel_bf.x ]             // [ -v_right, v_forward ]
sensor_flow = clamp_and_filter(sensor_flow)
input_ef = body_to_earth2D(sensor_flow)
```

This preserves the original FlowHold control basis exactly.

### Why `body_to_earth2D(sensor_flow)` Stays

The final review found that removing `body_to_earth2D(sensor_flow)` was wrong.

FlowHold's internal two-axis control signal is not a plain earth-frame velocity vector. It is the legacy optical-flow control vector carried through the existing controller path. Because the EXTNAV adaptation reconstructs the same `sensor_flow` basis as the old code, the original `body_to_earth2D(sensor_flow)` call remains correct and should stay unchanged.

That means the downstream controller structure is still:

1. `sensor_flow` in the legacy optical-flow control basis
2. `body_to_earth2D(sensor_flow)` to produce the PI input
3. PI controller output in earth frame
4. `earth_to_body2D(ef_output)` to return to the body-side command channels before adding to `bf_angles`

### Braking, PI, and Filter Semantics

Using the reconstructed `sensor_flow = [ -v_right, v_forward ]` preserves all three original mechanisms:

1. Braking: `sensor_flow[i]` still maps to the same roll/pitch braking channels the original code used.
2. PI controller: the same `body_to_earth2D(sensor_flow)` and later `earth_to_body2D(ef_output)` calls preserve the original roll/pitch correction routing.
3. Low-pass filter memory: filtering still happens on the same two control channels the original FlowHold code maintained across timesteps.

### Raw Source Frame vs Effective Control Frame

The adapted function uses both world and body information, but at different stages:

1. Raw source: world or earth-frame horizontal velocity from `inertial_nav.get_velocity_neu_cms()`.
2. Effective control signal for clamp, filter, and braking: body-derived optical-flow control channels after `earth_to_body2D()` plus the axis remap.
3. PI input: earth-frame version of that reconstructed optical-flow control vector.

So the most accurate short answer is:

- the raw estimator input is world or earth-frame velocity
- the actual adapted optical-flow control path operates on a body-derived remapped control vector

### Logging Interpretation After Final Implementation

With the final implementation:

1. `FHLD.SFx` is the filtered adapted optical-flow X control channel, equivalent to `-v_right` in m/s.
2. `FHLD.SFy` is the filtered adapted optical-flow Y control channel, equivalent to `+v_forward` in m/s.
3. `FHLD.Qual` is the EKF horizontal-velocity-valid flag represented as `0` or `255`.

---

## Verification

### 1. Build — confirm zero optflow references remain
```bash
cd /home/polanco/Liberaware/ardupilot
./waf configure --board=MatekH743
./waf copter 2>&1 | grep -iE "error|optflow|flowRate|bodyRate"
# Also verify manually:
grep -n "optflow" ArduCopter/mode_flowhold.cpp  # must return nothing
```

### 2. SITL — basic behaviour
```bash
sim_vehicle.py -v ArduCopter --console --map
# Set: EK3_SRC1_VELXY=6, EK3_SRC1_POSXY=6, VISO_TYPE=1, FLOW_TYPE=0
# Arm, takeoff in AltHold, switch to FlowHold (mode 22)
```
- Mode accepts (init returns true — SITL has horiz_vel=true)
- Sticks centered → vehicle brakes and holds velocity=0
- Roll/pitch sticks → pilot lean angles applied, velocity correction added on top
- Throttle → altitude responds normally via RC

### 3. Degradation test — confirm safe fallback
- Stop sending `VISION_SPEED_ESTIMATE`
- `flags.horiz_vel` goes false → `quality_filtered = 0`
- Velocity correction block at line 324 is skipped
- Vehicle behaves as pure AltHold — no instability

### 4. Log check
```
FHLD.SFx / FHLD.SFy  → adapted optical-flow control channels `[-v_right, v_forward]` in m/s
FHLD.Qual             → 0 or 255 (EKF horiz_vel flag)
FHXY                  → absent (deleted with update_height_estimate)
VISV                  → present (confirms EXTNAV velocity fusion active)
XKF1.VN/VE           → tracks EXTNAV input
```

### 5. Regression — confirm unmodified modes unaffected
Switch to Loiter, AltHold, Guided — confirm normal operation. FlowHold changes are fully isolated to `mode_flowhold.cpp`, `mode.h` private section, and `config.h`.
