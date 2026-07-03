#include "AC_Sprayer.h"

#if HAL_SPRAYER_ENABLED

#include <AP_AHRS/AP_AHRS.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <SRV_Channel/SRV_Channel.h>

extern const AP_HAL::HAL& hal;

// ------------------------------

const AP_Param::GroupInfo AC_Sprayer::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Sprayer enable/disable
    // @Description: Allows you to enable (1) or disable (0) the sprayer. When set to 1 the sprayer can be controlled from both the RC aux function and MAV_CMD_DO_SPRAYER (GCS or mission), when set to 2 only the RC aux function is accepted. Changing between 1 and 2 takes effect immediately, changing from 0 requires a reboot to see the other parameters
    // @Values: 0:Disabled,1:Enabled,2:Enabled RC control only
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
    // @Description: Spinner's rotation speed (a higher rate will disperse the spray over a wider area horizontally). Values above 100 are used directly as PWM, values of 100 or below are a percentage of the servo output range (SERVOx_MIN to SERVOx_MAX)
    // @Units: ms
    // @Range: 0 2000
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

    // @Param: FLOW_BATT
    // @DisplayName: Flow meter battery monitor instance
    // @Description: Battery monitor instance used as flow meter. Should point at a monitor of type FuelFlow (11) which reports flow in litres/minute as current. Set to 0 to disable flow control
    // @Values: 0:Disabled,1:BATT,2:BATT2,3:BATT3,4:BATT4
    // @User: Standard
    AP_GROUPINFO("FLOW_BATT", 7, AC_Sprayer, _flow_batt, 0),

    // @Param: FLOW_RATE
    // @DisplayName: Desired flow rate
    // @Description: Desired flow rate. When this and FLOW_BATT are set the pump output is controlled to hold this flow rate using the flow meter reading instead of scaling the output with ground speed. When testing the pump or when SPEED_MIN is zero this is a fixed rate in litres/minute. When SPEED_MIN is greater than zero this is litres/km and the litres/minute target is scaled with ground speed. Set to 0 to disable flow control
    // @Range: 0 100
    // @User: Standard
    AP_GROUPINFO("FLOW_RATE", 8, AC_Sprayer, _flow_rate, 0),

    // @Param: FLOW_P
    // @DisplayName: Flow controller P gain
    // @Description: Pump output percentage added per litre/minute of flow error
    // @Range: 0 100
    // @User: Advanced
    AP_GROUPINFO("FLOW_P",    9, AC_Sprayer, _flow_p, AC_SPRAYER_DEFAULT_FLOW_P),

    // @Param: FLOW_I
    // @DisplayName: Flow controller I gain
    // @Description: Pump output percentage added per second per litre/minute of flow error
    // @Range: 0 100
    // @User: Advanced
    AP_GROUPINFO("FLOW_I",   10, AC_Sprayer, _flow_i, AC_SPRAYER_DEFAULT_FLOW_I),

    // @Param: START_MS
    // @DisplayName: Sprayer start delay
    // @Description: Delay in milliseconds between starting the pump and the spinner. If positive the spinner starts this many milliseconds after the pump. If negative the pump starts this many milliseconds after the spinner. Set to 0 to start both together
    // @Units: ms
    // @Range: -5000 5000
    // @User: Standard
    AP_GROUPINFO("START_MS", 11, AC_Sprayer, _start_delay_ms, 0),

    // @Param: STOP_MS
    // @DisplayName: Sprayer stop delay
    // @Description: Delay in milliseconds between stopping the pump and the spinner. If positive the spinner stops this many milliseconds after the pump. If negative the pump stops this many milliseconds after the spinner. Set to 0 to stop both together
    // @Units: ms
    // @Range: -5000 5000
    // @User: Standard
    AP_GROUPINFO("STOP_MS",  12, AC_Sprayer, _stop_delay_ms, 0),

    AP_GROUPEND
};

AC_Sprayer::AC_Sprayer(uint8_t instance)
{
    if (instance >= AC_SPRAYER_MAX_INSTANCES || _singletons[instance] != nullptr) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("Too many sprayers");
#endif
        return;
    }
    _singletons[instance] = this;

#if HAL_SPRAYER2_ENABLED
    if (instance == 1) {
        _pump_function = SRV_Channel::k_sprayer2_pump;
        _spinner_function = SRV_Channel::k_sprayer2_spinner;
    }
#endif

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
 * Get an AC_Sprayer instance
 */
AC_Sprayer *AC_Sprayer::_singletons[AC_SPRAYER_MAX_INSTANCES];
AC_Sprayer *AC_Sprayer::get_singleton(uint8_t instance)
{
    if (instance >= AC_SPRAYER_MAX_INSTANCES) {
        return nullptr;
    }
    return _singletons[instance];
}

void AC_Sprayer::run(const bool activate)
{
    // set flag indicating whether spraying is permitted:
    // do not allow running to be set to true if we are currently not enabled.
    // the pump and spinner are started/stopped by update() so the
    // start/stop delays also apply when spraying is toggled from here
    _flags.running = _enabled && activate;
}

void AC_Sprayer::stop_spraying()
{
    // turn off pump and spinner immediately, cancelling any start/stop delay sequencing
    SRV_Channels::set_output_limit(_pump_function, SRV_Channel::Limit::MIN);
    _flags.spraying = false;
    _flags.spray_req = false;
    _flags.spinner_on = false;

    // flow controller must restart from scratch on the next spray
    _flow_last_ms = 0;

    stop_spinner();
}

void AC_Sprayer::stop_spinner()
{
    // if no delay is set then turn off spinner immediately
    if (_spinner_delay_pwm == 0) {
        SRV_Channels::set_output_limit(_spinner_function, SRV_Channel::Limit::MIN);
        return;
    }

    // some delay is set, so reduce spinner speed gradually
    SRV_Channel* chan = SRV_Channels::get_channel_for(_spinner_function);

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
    // exit immediately if we are disabled, turning everything off
    if (!_enabled) {
        _flags.running = false;
        stop_spraying();
        return;
    }

    // exit immediately if the pump function has not been set-up for any servo
    if (!SRV_Channels::function_assigned(_pump_function)) {
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

    // spraying also requires the pilot/GCS to have requested it via run(),
    // so that stopping from the aux switch goes through the stop delay too
    should_be_spraying &= _flags.running;

    // detect start/stop transitions so we can sequence the pump and spinner with the configured delays
    if (should_be_spraying && !_flags.spray_req) {
        _spray_start_ms = now;
        _flags.spray_req = true;
    } else if (!should_be_spraying && _flags.spray_req) {
        _spray_stop_ms = now;
        _flags.spray_req = false;
    }

    // work out whether the pump and spinner should currently be on, applying the start/stop
    // delays between them. A positive delay means the spinner leads/lags the pump, a negative
    // delay means the pump leads/lags the spinner. The previous state is latched in so that a
    // device that is still on from an unfinished stop stays on across a restart, and a device
    // that never started does not turn on during the stop tail
    bool pump_on;
    bool spinner_on;
    if (should_be_spraying) {
        const uint32_t elapsed = now - _spray_start_ms;
        if (_start_delay_ms >= 0) {
            // pump starts first, spinner follows START_MS later
            pump_on = true;
            spinner_on = _flags.spinner_on || elapsed >= (uint32_t)_start_delay_ms;
        } else {
            // spinner starts first, pump follows -START_MS later
            spinner_on = true;
            pump_on = _flags.spraying || elapsed >= (uint32_t)(-_start_delay_ms);
        }
    } else {
        const uint32_t elapsed = now - _spray_stop_ms;
        if (_stop_delay_ms >= 0) {
            // pump stops first, spinner keeps running STOP_MS longer
            pump_on = false;
            spinner_on = _flags.spinner_on && elapsed < (uint32_t)_stop_delay_ms;
        } else {
            // spinner stops first, pump keeps running -STOP_MS longer
            spinner_on = false;
            pump_on = _flags.spraying && elapsed < (uint32_t)(-_stop_delay_ms);
        }
    }

    // drive the pump
    if (pump_on) {
        float pos;
        if (!should_be_spraying) {
            // stop tail: hold the last commanded output. Ground speed (and the testing
            // flag) may already have dropped away, which would otherwise command the
            // pump to zero and hide the stop delay
            pos = _last_pump_pos;
        } else if (flow_control_active()) {
            // closed loop control from the flow meter, percentage to servo range
            pos = flow_control_update(ground_speed) * 100.0f;
        } else {
            pos = ground_speed * _pump_pct_1ms;
        }
        pos = MAX(pos, 100 *_pump_min_pct); // ensure min pump speed
        pos = MIN(pos,10000); // clamp to range
        _last_pump_pos = pos;
        SRV_Channels::move_servo(_pump_function, pos, 0, 10000);
    } else {
        SRV_Channels::set_output_limit(_pump_function, SRV_Channel::Limit::MIN);
        // flow controller must restart from scratch on the next spray
        _flow_last_ms = 0;
    }

    // drive the spinner
    if (spinner_on) {
        // spinner values of 100 or below are a percentage of the servo range (SERVOx_MIN to SERVOx_MAX), above that a raw pwm
        if (_spinner_pwm <= 100) {
            SRV_Channels::move_servo(_spinner_function, _spinner_pwm, 0, 100);
        } else {
            SRV_Channels::set_output_pwm(_spinner_function, _spinner_pwm);
        }
    } else {
        stop_spinner();
    }

    _flags.spraying = pump_on;
    _flags.spinner_on = spinner_on;
}

// flow control is used instead of ground speed scaling when a flow meter and target rate are configured
bool AC_Sprayer::flow_control_active() const
{
    return _flow_batt > 0 && is_positive(_flow_rate);
}

// run the closed loop flow controller, returns desired pump output as a percentage (0 ~ 100)
float AC_Sprayer::flow_control_update(float ground_speed_cms)
{
    const uint32_t now = AP_HAL::millis();

    // reset the controller if it has not run recently
    if (_flow_last_ms == 0 || (now - _flow_last_ms) > 1000) {
        _flow_last_ms = now;
        _flow_integrator = _pump_min_pct;
        return _flow_integrator;
    }
    const float dt = (now - _flow_last_ms) * 0.001f;
    _flow_last_ms = now;

    // the fuel flow battery monitor reports flow in litres/minute as current
    float measured_lpm;
    if (!AP::battery().current_amps(measured_lpm, _flow_batt - 1)) {
        // no flow reading available, hold the last output
        return _flow_integrator;
    }

    // FLOW_RATE is litres/minute when testing or when SPEED_MIN is zero. Otherwise it is
    // litres/km and the litres/minute target scales with ground speed (cm/s to km/min)
    float target_lpm = _flow_rate;
    if (!_flags.testing && is_positive(_speed_min)) {
        target_lpm = _flow_rate * ground_speed_cms * (60.0f / 100000.0f);
    }

    const float error = target_lpm - measured_lpm;
    _flow_integrator = constrain_float(_flow_integrator + error * _flow_i * dt, 0.0f, 100.0f);
    return constrain_float(_flow_integrator + error * _flow_p, 0.0f, 100.0f);
}

void AC_Sprayer::update_copter(int32_t terrain_altitude_cm)
{
    // update altitude
    _current_altitude = terrain_altitude_cm;

    // run default update function without alititude enable
    update();
}

namespace AP {

AC_Sprayer *sprayer(uint8_t instance)
{
    return AC_Sprayer::get_singleton(instance);
}

};
#endif // HAL_SPRAYER_ENABLED
