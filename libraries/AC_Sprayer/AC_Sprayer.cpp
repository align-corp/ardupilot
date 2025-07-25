#include "AC_Sprayer.h"

#if HAL_SPRAYER_ENABLED

#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <SRV_Channel/SRV_Channel.h>

extern const AP_HAL::HAL& hal;

// ------------------------------

const AP_Param::GroupInfo AC_Sprayer::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Sprayer enable/disable
    // @Description: Allows you to enable (1) or disable (0) the sprayer
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO_FLAGS("ENABLE", 0, AC_Sprayer, _enabled, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: PUMP_RATE
    // @DisplayName: Pump speed
    // @Description: Desired pump speed when travelling 1m/s expressed as a percentage
    // @Units: %
    // @Range: 0 100
    // @User: Standard
    AP_GROUPINFO("PUMP_RATE",   1, AC_Sprayer, _pump_pct_1ms, AC_SPRAYER_DEFAULT_PUMP_RATE),

    // @Param: SPINNER
    // @DisplayName: Spinner rotation speed
    // @Description: Spinner's rotation speed in PWM (a higher rate will disperse the spray over a wider area horizontally)
    // @Units: ms
    // @Range: 1000 2000
    // @User: Standard
    AP_GROUPINFO("SPINNER",     2, AC_Sprayer, _spinner_pwm, AC_SPRAYER_DEFAULT_SPINNER_PWM),

    // @Param: SPEED_MIN
    // @DisplayName: Speed minimum
    // @Description: Speed minimum at which we will begin spraying
    // @Units: cm/s
    // @Range: 0 1000
    // @User: Standard
    AP_GROUPINFO("SPEED_MIN",   3, AC_Sprayer, _speed_min, AC_SPRAYER_DEFAULT_SPEED_MIN),

    // @Param: PUMP_MIN
    // @DisplayName: Pump speed minimum
    // @Description: Minimum pump speed expressed as a percentage
    // @Units: %
    // @Range: 0 100
    // @User: Standard
    AP_GROUPINFO("PUMP_MIN",   4, AC_Sprayer, _pump_min_pct, AC_SPRAYER_DEFAULT_PUMP_MIN),

    // @Param: SPIN_DEL
    // @DisplayName: Spinner delay
    // @Description: Spinner's delay in PWM (a higher rate will disperse the spray over a wider area horizontally)
    // @Units: ms
    // @Range: 1000 2000
    // @User: Standard
    AP_GROUPINFO("SPIN_DEL",   5, AC_Sprayer, _spinner_delay_pwm, AC_SPRAYER_DEFAULT_SPINNER_DELAY_PWM),

    // @Param: ALT_MIN
    // @DisplayName: Sprayer minimum altitude
    // @Description: Minimum altitude in cm at which we will begin spraying
    // @Units: %
    // @Range: 0 500
    // @User: Standard
    AP_GROUPINFO("ALT_MIN",   6, AC_Sprayer, _min_alt, AC_SPRAYER_DEFAULT_MIN_ALT),

    AP_GROUPEND
};

AC_Sprayer::AC_Sprayer()
{
    if (_singleton) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("Too many sprayers");
#endif
        return;
    }
    _singleton = this;

    AP_Param::setup_object_defaults(this, var_info);

    // check for silly parameter values
    if (_pump_pct_1ms < 0.0f || _pump_pct_1ms > 100.0f) {
        _pump_pct_1ms.set_and_save(AC_SPRAYER_DEFAULT_PUMP_RATE);
    }
    if (_spinner_pwm < 0) {
        _spinner_pwm.set_and_save(AC_SPRAYER_DEFAULT_SPINNER_PWM);
    }

    // To-Do: ensure that the pump and spinner servo channels are enabled
}

/*
 * Get the AP_Sprayer singleton
 */
AC_Sprayer *AC_Sprayer::_singleton;
AC_Sprayer *AC_Sprayer::get_singleton()
{
    return _singleton;
}

void AC_Sprayer::run(const bool activate)
{
    // turn off the pump and spinner servos if necessary
    if (!_flags.running) {
        stop_spraying();
    }

    // return immediately if no change
    if (_flags.running == activate) {
        return;
    }

    // set flag indicate whether spraying is permitted:
    // do not allow running to be set to true if we are currently not enabled
    _flags.running = _enabled && activate;

}

void AC_Sprayer::stop_spraying()
{
    // set flag and turn off pump
    SRV_Channels::set_output_limit(SRV_Channel::k_sprayer_pump, SRV_Channel::Limit::MIN);
    _flags.spraying = false;

    // if no delay is set then turn off spinner immediately
    if (_spinner_delay_pwm == 0) {
        SRV_Channels::set_output_limit(SRV_Channel::k_sprayer_spinner, SRV_Channel::Limit::MIN);
        return;
    }

    // some delay is set, so reduce spinner speed gradually
    SRV_Channel* chan = SRV_Channels::get_channel_for(SRV_Channel::k_sprayer_spinner);

    // prevent segmentation fault if channel is not set
    if (chan == nullptr) {
        return;
    }
    uint16_t spinner_pwm_now = chan->get_output_pwm();
    uint16_t spinner_pwm_min = chan->get_output_min();
    uint16_t spinner_pwm_set = MAX(spinner_pwm_now - _spinner_delay_pwm, spinner_pwm_min);
    chan->set_output_pwm(spinner_pwm_set);
}

/// update - adjust pwm of servo controlling pump speed according to the desired quantity and our horizontal speed
void AC_Sprayer::update()
{
    // exit immediately if we are disabled or shouldn't be running
    if (!_enabled || !running()) {
        run(false);
        return;
    }

    // exit immediately if the pump function has not been set-up for any servo
    if (!SRV_Channels::function_assigned(SRV_Channel::k_sprayer_pump)) {
        return;
    }

    // get horizontal velocity
    Vector3f velocity;
    if (!AP::ahrs().get_velocity_NED(velocity)) {
        // treat unknown velocity as zero which should lead to pump stopping
        // velocity will already be zero but this avoids a coverity warning
        velocity.zero();
    }

    float ground_speed = velocity.xy().length() * 100.0;

    // get the current time
    const uint32_t now = AP_HAL::millis();

    // Check speed condition
    if (ground_speed >= _speed_min) {
        if (_speed_over_min_time == 0) {
            _speed_over_min_time = now;
        } else if ((now - _speed_over_min_time) > AC_SPRAYER_DEFAULT_TURN_ON_DELAY) {
            _speed_ok = true;
        }
        _speed_under_min_time = 0;
    } else {
        if (_speed_under_min_time == 0) {
            _speed_under_min_time = now;
        } else if ((now - _speed_under_min_time) > AC_SPRAYER_DEFAULT_SHUT_OFF_DELAY) {
            _speed_ok = false;
        }
        _speed_over_min_time = 0;
    }

    // Check altitude condition
    if (_current_altitude >= _min_alt) {
        if (_alt_over_min_time == 0) {
            _alt_over_min_time = now;
        } else if ((now - _alt_over_min_time) > AC_SPRAYER_DEFAULT_ALT_TURN_ON_DELAY) {
            _altitude_ok = true;
        }
        _alt_under_min_time = 0;
    } else {
        if (_alt_under_min_time == 0) {
            _alt_under_min_time = now;
        } else if ((now - _alt_under_min_time) > AC_SPRAYER_DEFAULT_ALT_SHUT_OFF_DELAY) {
            _altitude_ok = false;
        }
        _alt_over_min_time = 0;
    }

    // Combine conditions
    bool should_be_spraying = _speed_ok && _altitude_ok;

    // if testing pump output speed as if travelling at 1m/s
    if (_flags.testing) {
        ground_speed = 100.0f;
        should_be_spraying = true;
    }

    // if spraying or testing update the pump servo position
    if (should_be_spraying) {
        float pos = ground_speed * _pump_pct_1ms;
        pos = MAX(pos, 100 *_pump_min_pct); // ensure min pump speed
        pos = MIN(pos,10000); // clamp to range
        SRV_Channels::move_servo(SRV_Channel::k_sprayer_pump, pos, 0, 10000);
        SRV_Channels::set_output_pwm(SRV_Channel::k_sprayer_spinner, _spinner_pwm);
        _flags.spraying = true;
    } else {
        stop_spraying();
    }
}

void AC_Sprayer::update_copter(int32_t terrain_altitude_cm)
{
    // update altitude
    _current_altitude = terrain_altitude_cm;

    // run default update function without alititude enable
    update();
}

namespace AP {

AC_Sprayer *sprayer()
{
    return AC_Sprayer::get_singleton();
}

};
#endif // HAL_SPRAYER_ENABLED
