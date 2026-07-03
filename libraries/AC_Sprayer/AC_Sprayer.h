/// @file   AC_Sprayer.h
/// @brief  Crop sprayer library

/**
    The crop spraying functionality can be enabled in ArduCopter by doing the following:
        - set RCx_OPTION to 15 to allow turning the sprayer on/off from one of these channels (use 178 for the second sprayer/spreader)
        - set SERVO10_FUNCTION to 22 to enable the servo output controlling the pump speed on servo-out 10
        - set SERVO11_FUNCTION to 23 to enable the servo output controlling the spinner on servo-out 11
        - for the second sprayer (e.g. fertilizer spreader) use SERVOx_FUNCTION 111 (pump) and 112 (spinner) and the SPRAY2_ parameters
        - ensure the RC10_MIN, RC10_MAX, RC11_MIN, RC11_MAX accurately hold the min and maximum servo values you could possibly output to the pump and spinner
        - set the SPRAY_SPINNER to the pwm value the spinner should spin at when on
        - set the SPRAY_PUMP_RATE to the value the pump servo should move to when the vehicle is travelling at 1m/s. This is expressed as a percentage (i.e. 0 ~ 100) of the full servo range.  I.e. 0 = the pump will not operate, 100 = maximum speed at 1m/s.  50 = 1/2 speed at 1m/s, full speed at 2m/s
        - set the SPRAY_PUMP_MIN to the minimum value that the pump servo should move to while engaged expressed as a percentage (i.e. 0 ~ 100) of the full servo range
        - set the SPRAY_SPEED_MIN to the minimum speed (in cm/s) the vehicle should be moving at before the pump and sprayer are turned on.  0 will mean the pump and spinner will always be on when the system is enabled with ch7/ch8 switch
        - optionally set SPRAY_FLOW_BATT to a fuel-flow battery monitor instance and SPRAY_FLOW_RATE to a desired flow to have the pump speed controlled from the flow meter. The rate is litres/minute when testing the pump or when SPRAY_SPEED_MIN is 0, otherwise litres/km with the litres/minute target scaled by ground speed
**/
#pragma once

#include <inttypes.h>
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <SRV_Channel/SRV_Channel.h>

#define AC_SPRAYER_DEFAULT_PUMP_RATE        10.0f   ///< default quantity of spray per meter travelled
#define AC_SPRAYER_DEFAULT_PUMP_MIN         0       ///< default minimum pump speed expressed as a percentage from 0 to 100
#define AC_SPRAYER_DEFAULT_SPINNER_PWM      1300    ///< default speed of spinner (higher means spray is throw further horizontally
#define AC_SPRAYER_DEFAULT_SPINNER_DELAY_PWM 100
#define AC_SPRAYER_DEFAULT_MIN_ALT          200
#define AC_SPRAYER_DEFAULT_SPEED_MIN        100     ///< we must be travelling at least 1m/s to begin spraying
#define AC_SPRAYER_DEFAULT_TURN_ON_DELAY    100     ///< delay between when we reach the minimum speed and we begin spraying.  This reduces the likelihood of constantly turning on/off the pump
#define AC_SPRAYER_DEFAULT_SHUT_OFF_DELAY   1000    ///< shut-off delay in milli seconds.  This reduces the likelihood of constantly turning on/off the pump
#define AC_SPRAYER_DEFAULT_ALT_TURN_ON_DELAY 100    ///< delay before spraying starts after reaching minimum altitude (ms)
#define AC_SPRAYER_DEFAULT_ALT_SHUT_OFF_DELAY 2500  ///< delay before spraying stops after going below minimum altitude (ms)
#define AC_SPRAYER_DEFAULT_FLOW_P           10.0f   ///< default flow controller P gain (pump % per l/min of error)
#define AC_SPRAYER_DEFAULT_FLOW_I           5.0f    ///< default flow controller I gain (pump %/sec per l/min of error)

#ifndef HAL_SPRAYER_ENABLED
#define HAL_SPRAYER_ENABLED 1
#endif

// second sprayer instance (e.g. fertilizer spreader), enable per-board with "define HAL_SPRAYER2_ENABLED 1" in hwdef
#ifndef HAL_SPRAYER2_ENABLED
#define HAL_SPRAYER2_ENABLED 0
#endif

#if HAL_SPRAYER2_ENABLED
#define AC_SPRAYER_MAX_INSTANCES            2       ///< instance 0 is the sprayer, instance 1 the spreader
#else
#define AC_SPRAYER_MAX_INSTANCES            1
#endif

#if HAL_SPRAYER_ENABLED

/// @class  AC_Sprayer
/// @brief  Object managing a crop sprayer or spreader comprised of a spinner and a pump both controlled by pwm
class AC_Sprayer {
public:
    AC_Sprayer(uint8_t instance = 0);

    /* Do not allow copies */
    CLASS_NO_COPY(AC_Sprayer);

    static AC_Sprayer *get_singleton(uint8_t instance = 0);

    /// run - allow or disallow spraying to occur
    void run(bool true_false);

    /// running - returns true if spraying is currently permitted
    bool running() const { return _flags.running; }

    /// mavlink_control_enabled - true if control from MAV_CMD_DO_SPRAYER (GCS or mission) is allowed.
    /// ENABLE = 2 restricts control to the RC aux function. Read live so 1 <-> 2 works without a reboot
    bool mavlink_control_enabled() const { return _enabled == 1; }

    /// spraying - returns true if spraying is actually happening
    bool spraying() const { return _flags.spraying; }

    /// test_pump - set to true to turn on pump as if travelling at 1m/s as a test
    void test_pump(bool true_false) { _flags.testing = true_false; }

    /// To-Do: add function to decode pilot input from channel 6 tuning knob

    /// set_pump_rate - sets desired quantity of spray when travelling at 1m/s as a percentage of the pumps maximum rate
    void set_pump_rate(float pct_at_1ms) { _pump_pct_1ms.set(pct_at_1ms); }

    /// update - adjusts servo positions based on speed and requested quantity
    void update();
    void update_copter(int32_t terrain_altitude_cm = 0);

    static const struct AP_Param::GroupInfo var_info[];

private:

    static AC_Sprayer *_singletons[AC_SPRAYER_MAX_INSTANCES];

    // servo output functions assigned to this instance
    SRV_Channel::Aux_servo_function_t _pump_function = SRV_Channel::k_sprayer_pump;
    SRV_Channel::Aux_servo_function_t _spinner_function = SRV_Channel::k_sprayer_spinner;

    // parameters
    AP_Int8         _enabled;               ///< top level enable/disable control
    AP_Float        _pump_pct_1ms;          ///< desired pump rate (expressed as a percentage of top rate) when travelling at 1m/s
    AP_Int8         _pump_min_pct;          ///< minimum pump rate (expressed as a percentage from 0 to 100)
    AP_Int16        _spinner_pwm;           ///< pwm rate of spinner
    AP_Int16        _spinner_delay_pwm;     ///< delay to prevent spinner malfunction
    AP_Float        _speed_min;             ///< minimum speed in cm/s above which the sprayer will be started
    AP_Int16        _min_alt;               ///< minimum altitude in cm at which we will begin spraying
    AP_Int8         _flow_batt;             ///< battery monitor instance used as flow meter (0:disabled, 1:BATT, 2:BATT2 ...)
    AP_Float        _flow_rate;             ///< desired flow rate: litres/minute when testing or SPEED_MIN is 0, otherwise litres/km scaled with ground speed (0:disabled)
    AP_Float        _flow_p;                ///< flow controller P gain
    AP_Float        _flow_i;                ///< flow controller I gain
    AP_Int16        _start_delay_ms;        ///< start delay between pump and spinner (ms). +ve: spinner starts this long after pump; -ve: pump starts this long after spinner
    AP_Int16        _stop_delay_ms;         ///< stop delay between pump and spinner (ms). +ve: spinner stops this long after pump; -ve: pump stops this long after spinner

    /// flag bitmask
    struct sprayer_flags_type {
        uint8_t spraying    : 1;            ///< 1 if we are currently spraying
        uint8_t testing     : 1;            ///< 1 if we are testing the sprayer and should output a minimum value
        uint8_t running     : 1;            ///< 1 if we are permitted to run sprayer
        uint8_t spray_req   : 1;            ///< 1 if spraying is currently requested (used to detect start/stop transitions for the pump/spinner delays)
        uint8_t spinner_on  : 1;            ///< 1 if the spinner is currently commanded on (latched so start/stop delays behave across quick start/stop toggles)
    } _flags;

    // internal variables
    uint32_t _speed_over_min_time = 0;      ///< time at which we reached speed minimum
    uint32_t _speed_under_min_time = 0;     ///< time at which we fell below speed minimum
    uint32_t _alt_under_min_time = 0;       ///< time at which we were last under minimum altitude
    uint32_t _alt_over_min_time = 0;        ///< time at which we were last over minimum altitude
    int32_t _current_altitude = 0;          ///< current altitude in cm, no use if == 0
    bool _altitude_ok = false;              ///< true if we are above minimum altitude
    bool _speed_ok = false;                 ///< true if we are above minimum speed
    float _flow_integrator = 0;             ///< flow controller integrator (pump output percentage)
    uint32_t _flow_last_ms = 0;             ///< last time the flow controller ran, 0 if controller needs reset
    uint32_t _spray_start_ms = 0;           ///< time at which spraying was last requested (used for the start delay between pump and spinner)
    uint32_t _spray_stop_ms = 0;            ///< time at which spraying was last stopped (used for the stop delay between pump and spinner)
    float _last_pump_pos = 0;               ///< last commanded pump servo position, held during the stop delay tail

    void stop_spraying();
    void stop_spinner();                    ///< turn off (or gradually slow) the spinner

    // flow control - closed loop control of pump output from a flow meter
    bool flow_control_active() const;
    float flow_control_update(float ground_speed_cms);
};

namespace AP {
    AC_Sprayer *sprayer(uint8_t instance = 0);
};
#endif // HAL_SPRAYER_ENABLED
