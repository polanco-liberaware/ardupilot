#include "Copter.h"
#include <utility>

#if MODE_FLOWHOLD_ENABLED

/*
  implement FLOWHOLD mode, for position hold using optical flow
  without rangefinder
 */

const AP_Param::GroupInfo ModeFlowHold::var_info[] = {
    // @Param: _XY_P
    // @DisplayName: FlowHold P gain
    // @Description: FlowHold (horizontal) P gain.
    // @Range: 0.1 6.0
    // @Increment: 0.1
    // @User: Advanced

    // @Param: _XY_I
    // @DisplayName: FlowHold I gain
    // @Description: FlowHold (horizontal) I gain
    // @Range: 0.02 1.00
    // @Increment: 0.01
    // @User: Advanced

    // @Param: _XY_IMAX
    // @DisplayName: FlowHold Integrator Max
    // @Description: FlowHold (horizontal) integrator maximum
    // @Range: 0 4500
    // @Increment: 10
    // @Units: cdeg
    // @User: Advanced

    // @Param: _XY_FILT_HZ
    // @DisplayName: FlowHold filter on input to control
    // @Description: FlowHold (horizontal) filter on input to control
    // @Range: 0 100
    // @Units: Hz
    // @User: Advanced
    AP_SUBGROUPINFO(flow_pi_xy, "_XY_",  1, ModeFlowHold, AC_PI_2D),

    // @Param: _FLOW_MAX
    // @DisplayName: FlowHold Velocity Max
    // @Description: Maximum horizontal velocity (m/s) used as input to the velocity hold controller. NOTE: this parameter was previously optical-flow rate in rad/s with a default of 0.6. If upgrading from optical-flow FlowHold, reset this parameter to 2.0 or the new default will not apply.
    // @Range: 0.1 5.0
    // @Units: m/s
    // @User: Standard
    AP_GROUPINFO("_FLOW_MAX", 2, ModeFlowHold, flow_max, 2.0),

    // @Param: _FILT_HZ
    // @DisplayName: FlowHold Filter Frequency
    // @Description: Filter frequency for flow data
    // @Range: 1 100
    // @Units: Hz
    // @User: Standard
    AP_GROUPINFO("_FILT_HZ", 3, ModeFlowHold, flow_filter_hz, 5),

    // 4 was _QUAL_MIN (optical flow quality, removed — EKF health replaces it)
    // 5 was FLOW_SPEED

    // @Param: _BRAKE_RATE
    // @DisplayName: FlowHold Braking rate
    // @Description: Controls deceleration rate on stick release
    // @Range: 1 30
    // @User: Standard
    // @Units: deg/s
    AP_GROUPINFO("_BRAKE_RATE", 6, ModeFlowHold, brake_rate_dps, 8),

    AP_GROUPEND
};

ModeFlowHold::ModeFlowHold(void) : Mode()
{
    AP_Param::setup_object_defaults(this, var_info);
}

void ModeFlowHold::reset_flowhold_controller_state()
{
    flow_pi_xy.reset_I();
    flow_pi_xy.reset_filter();
    flow_filter.reset();
    limited = false;
    xy_I.zero();
    braking = false;
    last_stick_input_ms = 0;
}

// flowhold_init - initialise flowhold controller
bool ModeFlowHold::init(bool ignore_checks)
{
    // Require EKF to have a valid horizontal velocity estimate (EXTNAV source)
    if (!ignore_checks && !inertial_nav.get_filter_status().flags.horiz_vel) {
        return false;
    }

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control->set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // initialise the vertical position controller
    if (!copter.pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    flow_filter.set_cutoff_frequency(copter.scheduler.get_loop_rate_hz(), flow_filter_hz.get());

    quality_filtered = 0;
    reset_flowhold_controller_state();
    last_stick_input_ms = 0;

    flow_pi_xy.set_dt(1.0/copter.scheduler.get_loop_rate_hz());

    return true;
}

/*
  calculate desired attitude from flow sensor. Called when flow sensor is healthy
 */
void ModeFlowHold::flowhold_flow_to_angle(Vector2f &bf_angles, bool stick_input)
{
    uint32_t now = AP_HAL::millis();

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

    // clamp and filter in the optical-flow axis frame (preserves filter memory
    // consistency across yaw rotations, matching the original path)
    sensor_flow.x = constrain_float(sensor_flow.x, -flow_max, flow_max);
    sensor_flow.y = constrain_float(sensor_flow.y, -flow_max, flow_max);
    sensor_flow = flow_filter.apply(sensor_flow);

    // Keep the controller entirely in the optical-flow/body-axis basis so
    // filtered error and integrator state remain aligned through yaw changes.
    flow_pi_xy.set_input(sensor_flow);

    // Controller correction in the same optical-flow/body-axis basis. This is
    // later added directly to body lean commands.
    Vector2f flow_output = flow_pi_xy.get_p();

    if (stick_input) {
        last_stick_input_ms = now;
        braking = true;
    }
    if (!stick_input && braking) {
        // stop braking if either 3s has passed, or we have slowed below 0.3m/s
        if (now - last_stick_input_ms > 3000 || sensor_flow.length() < 0.3) {
            braking = false;
#if 0
            printf("braking done at %u vel=%f\n", now - last_stick_input_ms,
                   (double)sensor_flow.length());
#endif
        }
    }

    if (!stick_input && !braking) {
        // Update only the integrator state on top of the already-computed P
        // term. Using get_pi() here would double-count P.
        if (limited) {
            // only allow I term to shrink in length
            xy_I = flow_pi_xy.get_i_shrink();
        } else {
            // normal I term operation
            xy_I = flow_pi_xy.get_i();
        }
    }

    if (!stick_input && braking) {
        // calculate brake angle for each axis separately
        for (uint8_t i=0; i<2; i++) {
            float &velocity = sensor_flow[i];
            float abs_vel_cms = fabsf(velocity)*100;
            const float brake_gain = (15.0f * brake_rate_dps.get() + 95.0f) * 0.01f;
            float lean_angle_cd = brake_gain * abs_vel_cms * (1.0f+500.0f/(abs_vel_cms+60.0f));
            if (velocity < 0) {
                lean_angle_cd = -lean_angle_cd;
            }
            bf_angles[i] = lean_angle_cd;
        }
        flow_output.zero();
    }

    flow_output += xy_I;
    flow_output *= copter.aparm.angle_max;
    bf_angles += flow_output;

    // set limited flag to prevent integrator windup
    limited = fabsf(bf_angles.x) > copter.aparm.angle_max || fabsf(bf_angles.y) > copter.aparm.angle_max;

    // constrain to angle limit
    bf_angles.x = constrain_float(bf_angles.x, -copter.aparm.angle_max, copter.aparm.angle_max);
    bf_angles.y = constrain_float(bf_angles.y, -copter.aparm.angle_max, copter.aparm.angle_max);

#if HAL_LOGGING_ENABLED
// @LoggerMessage: FHLD
// @Description: FlowHold mode messages
// @URL: https://ardupilot.org/copter/docs/flowhold-mode.html
// @Field: TimeUS: Time since system startup
// @Field: SFx: Filtered adapted optical-flow control channel X, equivalent to negative body-right velocity in m/s
// @Field: SFy: Filtered adapted optical-flow control channel Y, equivalent to positive body-forward velocity in m/s
// @Field: Ax: Target body lean angle correction, X-Axis
// @Field: Ay: Target body lean angle correction, Y-Axis
// @Field: Qual: EKF horizontal velocity validity flag, reported as 0 when invalid and 255 when valid
// @Field: Ix: Body/optical-axis PI integrator component, X-Axis
// @Field: Iy: Body/optical-axis PI integrator component, Y-Axis

    if (log_counter++ % 20 == 0) {
        AP::logger().WriteStreaming("FHLD", "TimeUS,SFx,SFy,Ax,Ay,Qual,Ix,Iy", "Qfffffff",
                                               AP_HAL::micros64(),
                                               (double)sensor_flow.x, (double)sensor_flow.y,
                                               (double)bf_angles.x, (double)bf_angles.y,
                                               (double)quality_filtered,
                                               (double)xy_I.x, (double)xy_I.y);
    }
#endif  // HAL_LOGGING_ENABLED
}

// flowhold_run - runs the flowhold controller
// should be called at 100hz or more
void ModeFlowHold::run()
{
    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // check for filter change
    if (!is_equal(flow_filter.get_cutoff_freq(), flow_filter_hz.get())) {
        flow_filter.set_cutoff_frequency(copter.scheduler.get_loop_rate_hz(), flow_filter_hz.get());
    }

    // get pilot desired climb rate
    float target_climb_rate = copter.get_pilot_desired_climb_rate(copter.channel_throttle->get_control_in());
    target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), copter.g.pilot_speed_up);

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate();

    // Flow Hold State Machine Determination
    AltHoldModeState flowhold_state = get_alt_hold_state(target_climb_rate);

    // Binary EKF health signal: valid horizontal velocity estimate or not
    const float previous_quality_filtered = quality_filtered;
    quality_filtered = inertial_nav.get_filter_status().flags.horiz_vel ? 255.0f : 0.0f;

    // FlowHold owns its own horizontal validity gate. If velocity aiding drops
    // out while the mode is active, discard horizontal controller memory and
    // degrade to manual AltHold-like lean control until aiding returns.
    if ((previous_quality_filtered > 0.0f) && (quality_filtered <= 0.0f)) {
        reset_flowhold_controller_state();
    }

    // Flow Hold State Machine
    switch (flowhold_state) {

    case AltHoldModeState::MotorStopped:
        copter.motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        copter.attitude_control->reset_rate_controller_I_terms();
        copter.attitude_control->reset_yaw_target_and_rate();
        copter.pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
        reset_flowhold_controller_state();
        break;

    case AltHoldModeState::Takeoff:
        // set motors to full range
        copter.motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // initiate take-off
        if (!takeoff.running()) {
            takeoff.start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
        }

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // set position controller targets adjusted for pilot input
        takeoff.do_pilot_takeoff(target_climb_rate);
        break;

    case AltHoldModeState::Landed_Ground_Idle:
        attitude_control->reset_yaw_target_and_rate();
        FALLTHROUGH;

    case AltHoldModeState::Landed_Pre_Takeoff:
        attitude_control->reset_rate_controller_I_terms_smoothly();
        pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
        reset_flowhold_controller_state();
        break;

    case AltHoldModeState::Flying:
        copter.motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

#if AP_RANGEFINDER_ENABLED
        // update the vertical offset based on the surface measurement
        copter.surface_tracking.update_surface_offset();
#endif

        // Send the commanded climb rate to the position controller
        pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate);
        break;
    }

    // flowhold attitude target calculations
    Vector2f bf_angles;

    // calculate alt-hold angles
    int16_t roll_in = copter.channel_roll->get_control_in();
    int16_t pitch_in = copter.channel_pitch->get_control_in();
    float angle_max = copter.aparm.angle_max;
    get_pilot_desired_lean_angles(bf_angles.x, bf_angles.y, angle_max, attitude_control->get_althold_lean_angle_max_cd());

    if (quality_filtered > 0 &&
        AP_HAL::millis() - copter.arm_time_ms > 3000) {
        // don't use for first 3s when we are just taking off
        Vector2f flow_angles;

        flowhold_flow_to_angle(flow_angles, (roll_in != 0) || (pitch_in != 0));
        flow_angles.x = constrain_float(flow_angles.x, -angle_max/2, angle_max/2);
        flow_angles.y = constrain_float(flow_angles.y, -angle_max/2, angle_max/2);
        bf_angles += flow_angles;
    }
    bf_angles.x = constrain_float(bf_angles.x, -angle_max, angle_max);
    bf_angles.y = constrain_float(bf_angles.y, -angle_max, angle_max);

#if AP_AVOIDANCE_ENABLED
    // apply avoidance
    copter.avoid.adjust_roll_pitch(bf_angles.x, bf_angles.y, copter.aparm.angle_max);
#endif

    // call attitude controller
    copter.attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(bf_angles.x, bf_angles.y, target_yaw_rate);

    // run the vertical position controller and set output throttle
    pos_control->update_z_controller();
}

#endif // MODE_FLOWHOLD_ENABLED
