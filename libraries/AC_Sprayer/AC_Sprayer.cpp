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
    // @DisplayName: Spinner spin down time
    // @Description: Time the spinner is held at SPIN_LOW after the pump stops, before it is turned off. Set to 0 to stop the spinner together with the pump
    // @Units: s
    // @Range: 0 60
    // @User: Standard
    AP_GROUPINFO("SPIN_DEL",   5, AC_Sprayer, _spin_down_sec, AC_SPRAYER_DEFAULT_SPIN_DOWN_SEC),

    // @Param: ALT_MIN
    // @DisplayName: Sprayer minimum altitude
    // @Description: Minimum altitude in cm at which we will begin spraying
    // @Units: cm
    // @Range: 0 500
    // @User: Standard
    AP_GROUPINFO("ALT_MIN",   6, AC_Sprayer, _min_alt, AC_SPRAYER_DEFAULT_MIN_ALT),

    // @Param: TIMEOUT
    // @DisplayName: Sprayer test timeout
    // @Description: When testing the sprayer (vehicle disarmed) the pump and spinner will be stopped after this time.  The test switch must be toggled off and on again to restart the test.  Set to 0 to disable the timeout
    // @Units: s
    // @Range: 0 127
    // @User: Standard
    AP_GROUPINFO("TIMEOUT",   7, AC_Sprayer, _test_timeout_s, AC_SPRAYER_DEFAULT_TEST_TIMEOUT),
    // @Param: FLOW_BATT
    // @DisplayName: Flow meter battery monitor instance
    // @Description: Battery monitor instance used as flow meter. Should point at a monitor of type FuelFlow (11) which reports flow in litres/minute as current. Set to 0 to disable flow control
    // @Values: 0:Disabled,1:BATT,2:BATT2,3:BATT3,4:BATT4
    // @User: Standard
    AP_GROUPINFO("FLOW_BATT", 8, AC_Sprayer, _flow_batt, 0),

    // @Param: FLOW_RATE
    // @DisplayName: Desired flow rate
    // @Description: Desired flow per distance travelled. When FLOW_BATT is set the pump output is controlled from the flow meter to hold a litres/minute target of this rate scaled by ground speed, never less than FLOW_MIN. When testing the pump the target is computed as if travelling at 1m/s, the same as PUMP_RATE. Flow control is used when this or FLOW_MIN is above zero
    // @Units: l/km
    // @Range: 0 100
    // @User: Standard
    AP_GROUPINFO("FLOW_RATE", 9, AC_Sprayer, _flow_rate, 0),

    // @Param: FLOW_P
    // @DisplayName: Flow controller P gain
    // @Description: Pump output percentage added per litre/minute of flow error
    // @Range: 0 100
    // @User: Advanced
    AP_GROUPINFO("FLOW_P",    10, AC_Sprayer, _flow_p, AC_SPRAYER_DEFAULT_FLOW_P),

    // @Param: FLOW_I
    // @DisplayName: Flow controller I gain
    // @Description: Pump output percentage added per second per litre/minute of flow error
    // @Range: 0 100
    // @User: Advanced
    AP_GROUPINFO("FLOW_I",   11, AC_Sprayer, _flow_i, AC_SPRAYER_DEFAULT_FLOW_I),

    // @Param: START_MS
    // @DisplayName: Sprayer start delay
    // @Description: Delay in milliseconds between starting the pump and the spinner. If positive the spinner starts this many milliseconds after the pump. If negative the pump starts this many milliseconds after the spinner. Set to 0 to start both together
    // @Units: ms
    // @Range: -5000 5000
    // @User: Standard
    AP_GROUPINFO("START_MS", 12, AC_Sprayer, _start_delay_ms, 0),

    // @Param: STOP_MS
    // @DisplayName: Sprayer stop delay
    // @Description: Delay in milliseconds between stopping the pump and the spinner. If positive the spinner stops this many milliseconds after the pump. If negative the pump stops this many milliseconds after the spinner. Set to 0 to stop both together
    // @Units: ms
    // @Range: -5000 5000
    // @User: Standard
    AP_GROUPINFO("STOP_MS",  13, AC_Sprayer, _stop_delay_ms, 0),

    // @Param: SPIN_LOW
    // @DisplayName: Spinner low speed
    // @Description: Low spinner speed held for SPIN_DEL seconds after the pump stops, before the spinner is turned off. Values above 100 are used directly as PWM, values of 100 or below are a percentage of the servo output range (SERVOx_MIN to SERVOx_MAX), the same rule as SPINNER. Set to 0 to stop the spinner together with the pump
    // @Range: 0 2000
    // @User: Standard
    AP_GROUPINFO("SPIN_LOW", 14, AC_Sprayer, _spin_low, AC_SPRAYER_DEFAULT_SPIN_LOW),

    // @Param: FLOW_MIN
    // @DisplayName: Minimum flow rate
    // @Description: Minimum flow held by the flow controller while spraying, whatever the ground speed. Set FLOW_RATE to 0 for a fixed flow of this rate. Flow control is used when this or FLOW_RATE is above zero and FLOW_BATT is set
    // @Units: l/min
    // @Range: 0 100
    // @User: Standard
    AP_GROUPINFO("FLOW_MIN", 15, AC_Sprayer, _flow_min, 0),

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
    SRV_Channels::set_output_limit(_spinner_function, SRV_Channel::Limit::MIN);
    _spinner_off_ms = 0;
}

/// drive_spinner - command the spinner to the given speed.  Values at or below
/// AC_SPRAYER_SPINNER_PCT_MAX are a percentage of the servo range (SERVOx_MIN to
/// SERVOx_MAX), above that the value is used directly as a pwm
void AC_Sprayer::drive_spinner(int16_t speed)
{
    if (speed <= AC_SPRAYER_SPINNER_PCT_MAX) {
        SRV_Channels::move_servo(_spinner_function, speed, 0, AC_SPRAYER_SPINNER_PCT_MAX);
    } else {
        SRV_Channels::set_output_pwm(_spinner_function, speed);
    }
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

        // stop the test after the timeout. Disabled when set to zero or when spinner is not driven
        const uint32_t test_timeout_ms = _test_timeout_s > 0 ? uint32_t(_test_timeout_s) * 1000 : 0;
        if (test_timeout_ms == 0 || spinner_speed_off(_spinner_pwm)) {
            _test_start_ms = 0;
        } else if (_test_start_ms == 0) {
            _test_start_ms = now;
        } else if ((now - _test_start_ms) > test_timeout_ms) {
            // timed out, stop the test
            _flags.testing = false;
            _flags.test_timed_out = true;
            should_be_spraying = false;
        }
    } else {
        _test_start_ms = 0;
    }

    // a ground test that timed out must stay stopped until the sprayer switch is turned off and
    // on again.  Clearing the testing flag alone is not enough: on the ground the normal spray
    // conditions can already both be true (SPEED_MIN of zero makes the speed check pass, and an
    // unhealthy rangefinder reports the altitude as INT32_MAX), which would otherwise restart
    // the pump on the very next iteration.  Only the switch going low releases the latch, so
    // that arming can never start the pump on its own
    if (_flags.test_timed_out) {
        if (!_flags.running) {
            _flags.test_timed_out = false;
        } else {
            should_be_spraying = false;
        }
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
        drive_spinner(_spinner_pwm);
        _spinner_off_ms = 0;
    } else {
        // the spinner has just been commanded off, start the spin down tail
        if (_flags.spinner_on) {
            _spinner_off_ms = now;
        }
        // hold the spinner at SPIN_LOW for SPIN_DEL seconds, then turn it off.  There is nothing to
        // spin down if the spinner is not driven in the first place
        const uint32_t spin_down_ms = _spin_down_sec > 0 ? uint32_t(_spin_down_sec) * 1000 : 0;
        if (!spinner_speed_off(_spinner_pwm) && !spinner_speed_off(_spin_low) &&
            _spinner_off_ms != 0 && (now - _spinner_off_ms) < spin_down_ms) {
            drive_spinner(_spin_low);
        } else {
            stop_spinner();
        }
    }

    _flags.spraying = pump_on;
    _flags.spinner_on = spinner_on;
}

// flow control is used instead of ground speed scaling when a flow meter and target rate are configured
bool AC_Sprayer::flow_control_active() const
{
    return _flow_batt > 0 && (is_positive(_flow_rate) || is_positive(_flow_min));
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

    // FLOW_RATE litres/km scaled with ground speed (cm/s to km/min), floored at FLOW_MIN
    // litres/minute. Ground speed is already 1m/s while testing
    const float target_lpm = MAX(_flow_rate * ground_speed_cms * (60.0f / 100000.0f), _flow_min.get());

    const float error = target_lpm - measured_lpm;
    _flow_integrator = constrain_float(_flow_integrator + error * _flow_i * dt, 0.0f, 100.0f);
    return constrain_float(_flow_integrator + error * _flow_p, 0.0f, 100.0f);
}

void AC_Sprayer::update_copter(int32_t terrain_altitude_cm)
{
    // update altitude
    if (terrain_altitude_cm <= 0) {
        // Protect against negative terrain altitude (= crash lol)
        _current_altitude = INT32_MAX;
    } else {
        _current_altitude = terrain_altitude_cm;
    }

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
