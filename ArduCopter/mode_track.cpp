#include "Copter.h"

#if MODE_TRACK_ENABLED == ENABLED

/*
 * mode_track.cpp - follow a target provided by a supported camera
 */

// initialise follow mode
bool ModeTrack::init(const bool ignore_checks)
{
    if (!g2.follow_mount.enabled()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Set TRACK_ENABLE = 1");
        return false;
    }

    // re-use guided mode
    return ModeGuided::init(ignore_checks);
}

// perform cleanup required when leaving follow mode
void ModeTrack::exit()
{
}

void ModeTrack::run()
{
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // re-use guided mode's velocity controller
    // Note: this is safe from interference from GCSs and companion computer's whose guided mode
    //       position and velocity requests will be ignored while the vehicle is not in guided mode

    // variables to be sent to velocity controller
    Vector3f desired_velocity_body_frame_cms;
    float yaw_rate = 0.0f;

    AP_Follow_Mount::MountTracking tracking;
    
    if(g2.follow_mount.have_target() && g2.follow_mount.get_mount_tracking(tracking)) {
        // calculate yaw rate
        const float yaw_err_cd = degrees(tracking.yaw_error_rad) * 100.0f;
        const float yaw_kp = g2.follow_mount.get_yaw_p().kP();
        yaw_rate = yaw_err_cd * yaw_kp;

        // keep distance from target
        if (distance_cm == 0) {
            // desired distance is not set yet, wait target to be centered
            if(fabsf(yaw_err_cd) < 200) {
                distance_cm = tracking.distance_cm;
            }
        } else {
            // set x (body frame) velocity to keep target distance
            const uint16_t x_pos_error_cm = tracking.distance_cm - distance_cm;
            const float x_kp = g2.follow_mount.get_forward_p().kP();
            desired_velocity_body_frame_cms.x = x_pos_error_cm * x_kp;
            // constrain forward speed to 2 m/s
            desired_velocity_body_frame_cms.x = constrain_float(desired_velocity_body_frame_cms.x, -200.0f, 200.0f);
        }
    } else {
        // reset desired target distance
        distance_cm = 0;
    }

    //TODO: pilots should be able to manual control roll and altitude

    // log output at 10hz
    uint32_t now = AP_HAL::millis();
    bool log_request = false;
    if ((now - last_log_ms >= 100) || (last_log_ms == 0)) {
        log_request = true;
        last_log_ms = now;
    }

    // convert body frame velocity to NEU
    Vector3f desired_velocity_neu_cms;
    const float yaw_rad = AP::ahrs().get_yaw();
    desired_velocity_neu_cms.x = desired_velocity_body_frame_cms.x * cosf(yaw_rad) - desired_velocity_body_frame_cms.y * sinf(yaw_rad);
    desired_velocity_neu_cms.y = desired_velocity_body_frame_cms.x * sinf(yaw_rad) + desired_velocity_body_frame_cms.y * cosf(yaw_rad);

    // re-use guided mode's velocity controller (takes NEU)
    ModeGuided::set_velocity(desired_velocity_neu_cms, false, 0.0f, true, yaw_rate, false, log_request);
    ModeGuided::run();

}

#endif // MODE_TRACK_ENABLED
