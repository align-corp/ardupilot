#include "Copter.h"

#if MODE_LOITER_ENABLED == ENABLED

/*
 * Init and run calls for loiter flight mode
 */

// loiter_init - initialise loiter controller
bool ModeLoiter::init(bool ignore_checks)
{
    if (!copter.failsafe.radio) {
        float target_roll, target_pitch;
        // apply SIMPLE mode transform to pilot inputs
        update_simple_mode();

        // convert pilot input to lean angles
        get_pilot_desired_lean_angles(target_roll, target_pitch, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max_cd());

        // process pilot's roll and pitch input
        loiter_nav->set_pilot_desired_acceleration(target_roll, target_pitch);
    } else {
        // clear out pilot desired acceleration in case radio failsafe event occurs and we do not switch to RTL for some reason
        loiter_nav->clear_pilot_desired_acceleration();
    }
    loiter_nav->init_target();

    // initialise the vertical position controller
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control->set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // set default landing state
    landing_state = LandingState::ALTITUDE_HIGH;

#if AC_PRECLAND_ENABLED
    _precision_loiter_active = false;
#endif

    return true;
}

#if AC_PRECLAND_ENABLED
bool ModeLoiter::do_precision_loiter()
{
    if (!_precision_loiter_enabled) {
        return false;
    }
    if (copter.ap.land_complete_maybe) {
        return false;        // don't move on the ground
    }
    // if the pilot *really* wants to move the vehicle, let them....
    if (loiter_nav->get_pilot_desired_acceleration().length() > 50.0f) {
        return false;
    }
    if (!copter.precland.target_acquired()) {
        return false; // we don't have a good vector
    }
    return true;
}

void ModeLoiter::precision_loiter_xy()
{
    loiter_nav->clear_pilot_desired_acceleration();
    Vector2f target_pos, target_vel;
    if (!copter.precland.get_target_position_cm(target_pos)) {
        target_pos = inertial_nav.get_position_xy_cm();
    }
    // get the velocity of the target
    copter.precland.get_target_velocity_cms(inertial_nav.get_velocity_xy_cms(), target_vel);

    Vector2f zero;
    Vector2p landing_pos = target_pos.topostype();
    // target vel will remain zero if landing target is stationary
    pos_control->input_pos_vel_accel_xy(landing_pos, target_vel, zero);
    // run pos controller
    pos_control->update_xy_controller();
}
#endif

void ModeLoiter::update_landing_state(AltHoldModeState alt_hold_state)
{

    // keep landing even if rangefinder is not healthy
    if (landing_state == LandingState::LANDING) {
        // after landing and disarming go back to default state and init loiter controller again
        if (!motors->armed()) {
            init(true);
            return;
        }
        // do not change state if we have landed: traditional helicopters wait
        // spooling down before disarming
        if (copter.ap.land_complete) {
            return;
        }
        // abort landing if throttle is increased by user
        if (channel_throttle->norm_input_ignore_trim() > 0.1f || !motors->armed()) {
            // when landing controller is landing loiter mode should be initialized again
            init(true);
            landing_state = LandingState::ALTITUDE_LOW;
            LOGGER_WRITE_EVENT(LogEvent::LOITER_LAND_ABORT);
        }
        return;
    }

    // prevent rangefinder OutOfRangeHigh state from switching to ALTITUDE_HIGH and cause hard landing
    if (landing_state == LandingState::ALTITUDE_LOW &&
        copter.rangefinder.status_orient(ROTATION_PITCH_270) == RangeFinder::Status::OutOfRangeHigh &&
        channel_throttle->norm_input_ignore_trim() < 0.1f) {
        return;
    }

    // check if rangefinder is healthy
    if (!copter.rangefinder_alt_ok()) {
        landing_state = LandingState::ALTITUDE_HIGH;
        return;
    }

    // enable landing controller only when copter is flying
    if (alt_hold_state != AltHoldModeState::AltHold_Flying) {
        landing_state = LandingState::ALTITUDE_HIGH;
        landing_request_start_ms = 0;
        return;
    }

    uint32_t now_ms = AP_HAL::millis();
    // we should control landing gear altitute, to avoid ground touch with long landing gear vehicles
    int16_t lgr_land_alt = g.pilot_land_alt + copter.rangefinder.ground_clearance_cm_orient(ROTATION_PITCH_270);
    int16_t lgr_land_low_alt = g.pilot_land_low_alt + copter.rangefinder.ground_clearance_cm_orient(ROTATION_PITCH_270);

    // check landing state based on rangefinder altitude
    if (get_alt_above_ground_cm() < lgr_land_low_alt+30) {
        landing_state = LandingState::ALTITUDE_LOW;
        // full negative throttle and 2 s delay for landing routine when altitude < 80 cm
        if (channel_throttle->norm_input_ignore_trim() < -0.9f) {
            if (landing_request_start_ms == 0) {
                landing_request_start_ms = now_ms;
            } else if (now_ms - landing_request_start_ms > 2000) {
                // initialise yaw
                auto_yaw.set_mode(AutoYaw::Mode::HOLD);

                // reset flag indicating if pilot has applied roll or pitch inputs during landing
                copter.ap.land_repo_active = false;

                // this will be set true if prec land is later active
                copter.ap.prec_land_active = false;

                // set landing state
                landing_state = LandingState::LANDING;
                LOGGER_WRITE_EVENT(LogEvent::LOITER_LAND_START);
            }
        } else {
            landing_request_start_ms = 0;
        }
    } else if (get_alt_above_ground_cm() < lgr_land_alt) {
        landing_request_start_ms = 0;
        landing_state = LandingState::ALTITUDE_LOW;
    } else {
        landing_state = LandingState::ALTITUDE_HIGH;
    }
}

// loiter_run - runs the loiter controller
// should be called at 100hz or more
void ModeLoiter::run()
{
    float target_roll, target_pitch;
    float target_yaw_rate = 0.0f;
    float target_climb_rate = 0.0f;
    int16_t max_speed_down = 0;
    uint16_t land_speed = abs(g.land_speed) > 0 ? abs(g.land_speed) : get_pilot_speed_dn();
    float input_angle_max_cd = loiter_nav->get_angle_max_cd();

    // Landing state controller
    update_landing_state(loiter_state);

    // calculate landing gear altitude
    int16_t lgr_land_alt = g.pilot_land_alt + copter.rangefinder.ground_clearance_cm_orient(ROTATION_PITCH_270);
    int16_t lgr_land_low_alt = g.pilot_land_low_alt + copter.rangefinder.ground_clearance_cm_orient(ROTATION_PITCH_270);

    switch (landing_state) {
        case LandingState::ALTITUDE_HIGH:
            // if rangefinder state is out of range high always use max speed down
            if (copter.rangefinder.status_orient(ROTATION_PITCH_270) == RangeFinder::Status::OutOfRangeHigh) {
                max_speed_down = -get_pilot_speed_dn();
                break;
            }
            // Compute a vertical velocity demand such that the vehicle approaches g2.land_alt_low. 
            max_speed_down = sqrt_controller(g2.land_alt_low-get_alt_above_ground_cm(), pos_control->get_pos_z_p().kP(), 
                    pos_control->get_max_accel_z_cmss(), G_Dt);
            // Constrain the demanded vertical velocity
            max_speed_down = constrain_float(max_speed_down, -get_pilot_speed_dn(), -land_speed);
            break;

        // altitude is below PILOT_LAND_ALT
        case LandingState::ALTITUDE_LOW:
            // Compute a vertical velocity demand such that the vehicle approaches land altitude.
            max_speed_down = sqrt_controller(lgr_land_low_alt-get_alt_above_ground_cm(), pos_control->get_pos_z_p().kP(), 
                    pos_control->get_max_accel_z_cmss(), G_Dt);

            // Constrain the demanded vertical velocity
            max_speed_down = constrain_float(max_speed_down, -land_speed, 0);

            // limit roll and pitch pilot input
            if (g.land_repositioning > 1 && lgr_land_alt > lgr_land_low_alt) {
                input_angle_max_cd = linear_interpolate(0, input_angle_max_cd,
                        get_alt_above_ground_cm(), lgr_land_low_alt, lgr_land_alt);
            }
            break;

        // altitude is below PILOT_LAND_LOW
        case LandingState::LANDING:
            // disarm when the landing detector says we've landed
            if (copter.ap.land_complete && motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE) {
                copter.arming.disarm(AP_Arming::Method::LANDED);
            }

            // Land State Machine Determination
            if (is_disarmed_or_landed()) {
                make_safe_ground_handling();
            } else {
                // set motors to full range
                motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

                // run normal landing or precision landing (if enabled)
                land_run_normal_or_precland(false);
            }
            return;
    }

    // process pilot inputs unless we are in radio failsafe
    if (!copter.failsafe.radio) {
        // apply SIMPLE mode transform to pilot inputs
        update_simple_mode();

        // convert pilot input to lean angles
        get_pilot_desired_lean_angles(target_roll, target_pitch, input_angle_max_cd, attitude_control->get_althold_lean_angle_max_cd());

        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());

        // get pilot desired climb rate
        target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
        target_climb_rate = constrain_float(target_climb_rate, max_speed_down, g.pilot_speed_up);
    } else {
        // clear out pilot desired acceleration in case radio failsafe event occurs and we do not switch to RTL for some reason
        loiter_nav->clear_pilot_desired_acceleration();
    }

    // relax loiter target if we might be landed
    if (copter.ap.land_complete_maybe) {
        loiter_nav->soften_for_landing();
    }

    // Loiter State Machine Determination
    loiter_state = get_alt_hold_state(target_climb_rate);

    // Loiter State Machine
    switch (loiter_state) {

    case AltHold_MotorStopped:
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->reset_yaw_target_and_rate();
        pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
        loiter_nav->init_target();
        attitude_control->input_thrust_vector_rate_heading(loiter_nav->get_thrust_vector(), target_yaw_rate, false);
        break;

    case AltHold_Landed_Ground_Idle:
        attitude_control->reset_yaw_target_and_rate();
        FALLTHROUGH;

    case AltHold_Landed_Pre_Takeoff:
        if (motors->get_spool_state() == AP_Motors::SpoolState::SPOOLING_UP) {
            attitude_control->reset_yaw_target_and_rate();
        }
        attitude_control->reset_rate_controller_I_terms_smoothly();
        loiter_nav->init_target();
        attitude_control->input_thrust_vector_rate_heading(loiter_nav->get_thrust_vector(), target_yaw_rate, false);
        // set vertical speed and acceleration limits
        pos_control->set_max_speed_accel_z(max_speed_down, g.pilot_speed_up, g.pilot_accel_z*0.25f);
        pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero

        // speed should be near 0
        if (inertial_nav.get_velocity_xy_cms().length() > 30.0f) {
            // disarm and raise error
            copter.arming.disarm(AP_Arming::Method::FAILSAFE_ACTION_TERMINATE);
            GCS_SEND_TEXT(MAV_SEVERITY_EMERGENCY, "Loiter: Speed too high, Disarming");
        }
        break;

    case AltHold_Takeoff:
        // initiate take-off
        if (!takeoff.running()) {
            takeoff.start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
        }

        // limit acceleration to achieve a smooth takeoff
        pos_control->set_max_speed_accel_z(max_speed_down, g.pilot_speed_up, g.pilot_accel_z*0.25f);

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // set position controller targets adjusted for pilot input
        takeoff.do_pilot_takeoff(target_climb_rate);

#if FRAME_CONFIG == HELI_FRAME
        // use altitude hold controller to take off
        // loiter controller sometimes cause dangerous roll/pitch errors when taking off
        loiter_nav->init_target();
        // update pilot input, use the land_repositioning param for consistency
        if (g.land_repositioning > 1) {
            get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max,
                    attitude_control->get_althold_lean_angle_max_cd());
        } else {
            target_roll = target_pitch = 0.0f;
        }
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
#else
        // update pilot input, use the land_repositioning param for consistency
        if (g.land_repositioning > 1) {
            loiter_nav->clear_pilot_desired_acceleration();
        } else {
            loiter_nav->set_pilot_desired_acceleration(target_roll, target_pitch);
        }
        // run loiter controller
        loiter_nav->update();
        // call attitude controller
        attitude_control->input_thrust_vector_rate_heading(loiter_nav->get_thrust_vector(), target_yaw_rate, false);
#endif

        break;

    case AltHold_Flying:
        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // process pilot's roll and pitch input
        loiter_nav->set_pilot_desired_acceleration(target_roll, target_pitch);

        // set vertical speed and acceleration limits
        pos_control->set_max_speed_accel_z(max_speed_down, g.pilot_speed_up, g.pilot_accel_z);

#if AC_PRECLAND_ENABLED
        bool precision_loiter_old_state = _precision_loiter_active;
        if (do_precision_loiter()) {
            precision_loiter_xy();
            _precision_loiter_active = true;
        } else {
            _precision_loiter_active = false;
        }
        if (precision_loiter_old_state && !_precision_loiter_active) {
            // prec loiter was active, not any more, let's init again as user takes control
            loiter_nav->init_target();
        }
        // run loiter controller if we are not doing prec loiter
        if (!_precision_loiter_active) {
            loiter_nav->update();
        }
#else
        loiter_nav->update();
#endif

        // call attitude controller
        attitude_control->input_thrust_vector_rate_heading(loiter_nav->get_thrust_vector(), target_yaw_rate, false);

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // update the vertical offset based on the surface measurement
        copter.surface_tracking.update_surface_offset();

        // Send the commanded climb rate to the position controller
        pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate);
        break;
    }

    // run the vertical position controller and set output throttle
    pos_control->update_z_controller();
}

uint32_t ModeLoiter::wp_distance() const
{
    return loiter_nav->get_distance_to_target();
}

int32_t ModeLoiter::wp_bearing() const
{
    return loiter_nav->get_bearing_to_target();
}

#endif
