#pragma once

#include <RC_Channel/RC_Channel.h>
#include <AP_Motors/AP_Motors.h>
#include "mode.h"

class RC_Channel_Copter : public RC_Channel
{

public:

protected:

    void init_aux_function(aux_func_t ch_option, AuxSwitchPos) override;
    bool do_aux_function(aux_func_t ch_option, AuxSwitchPos) override;

private:

    void do_aux_function_change_mode(const Mode::Number mode,
                                     const AuxSwitchPos ch_flag);
    void do_aux_function_change_air_mode(const AuxSwitchPos ch_flag);
    void do_aux_function_change_force_flying(const AuxSwitchPos ch_flag);

    // RTL channel last state
    AuxSwitchPos rtl_ch_flag_last = AuxSwitchPos::LOW;

    // called when the mode switch changes position:
    void mode_switch_changed(modeswitch_pos_t new_pos) override;

#ifdef FLTMODE_SWITCH_COMPASS_CALIBRATION
    // compass cal with fast movement of mode switch
    uint8_t fltmode_switch_count = 0;
    uint8_t fltmode_switch_last_pos = 0;
    uint32_t fltmode_switch_time_ms = 0;
#endif
};

class RC_Channels_Copter : public RC_Channels
{
public:

    bool has_valid_input() const override;
    bool in_rc_failsafe() const override;

    RC_Channel *get_arming_channel(void) const override;

    RC_Channel_Copter obj_channels[NUM_RC_CHANNELS];
    RC_Channel_Copter *channel(const uint8_t chan) override {
        if (chan >= NUM_RC_CHANNELS) {
            return nullptr;
        }
        return &obj_channels[chan];
    }

    // returns true if throttle arming checks should be run
    bool arming_check_throttle() const override;

protected:

    int8_t flight_mode_channel_number() const override;

};
