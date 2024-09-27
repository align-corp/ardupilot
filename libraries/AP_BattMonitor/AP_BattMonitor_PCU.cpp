#include "AP_BattMonitor_config.h"

#if AP_BATTERY_PCU_ENABLED

#include "AP_BattMonitor_PCU.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_SerialManager/AP_SerialManager.h>
#if !defined(HAL_BUILD_AP_PERIPH)
#include <AP_Arming/AP_Arming.h>
#endif

#define PCU_NOT_HEALTY_MICROS 5000000

#ifdef AP_BATTERY_PCU_DEBUG
#define debug(fmt, args ...)  do {hal.console->printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#else
#define debug(fmt, args ...)
#endif

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_BattMonitor_PCU::var_info[] = {

    // @Param: VOLT_MULT
    // @DisplayName: Voltage Multiplier
    // @Description: Adjust voltage reading: (PCU_volt) - _volt_offset) * VOLT_MULT;
    // @User: Advanced
    AP_GROUPINFO("VOLT_MULT", 1, AP_BattMonitor_PCU, _volt_multiplier, AP_BATT_PCU_VOLT_MULT),

    // @Param: AMP_MULT
    // @DisplayName: Amps Multiplier
    // @Description: Adjust voltage reading: (PCU_volt) - _volt_offset) * VOLT_MULT;
    // @User: Standard
    AP_GROUPINFO("AMP_MULT", 2, AP_BattMonitor_PCU, _amp_multiplier, AP_BATT_PCU_AMP_MULT),

    // @Param: AMP_OFFSET
    // @DisplayName: AMP offset
    // @Description: Voltage offset at zero current on current sensor
    // @Units: V
    // @User: Standard
    AP_GROUPINFO("AMP_OFFSET", 3, AP_BattMonitor_PCU, _amp_offset, AP_BATT_PCU_AMP_OFFS),

    // @Param: VLT_OFFSET
    // @DisplayName: Volage offset
    // @Description: Voltage offset on voltage pin. This allows for an offset due to a diode. This voltage is subtracted before the scaling is applied
    // @Units: V
    // @User: Advanced
    AP_GROUPINFO("VLT_OFFSET", 4, AP_BattMonitor_PCU, _volt_offset, AP_BATT_PCU_VOLT_OFFS),
    
    // Param indexes must be less than 10 to avoid conflict with other battery monitor param tables loaded by pointer

    AP_GROUPEND
};

/// Constructor
AP_BattMonitor_PCU::AP_BattMonitor_PCU(AP_BattMonitor &mon,
                                             AP_BattMonitor::BattMonitor_State &mon_state,
                                             AP_BattMonitor_Params &params) :
    AP_BattMonitor_Backend(mon, mon_state, params)
{
    AP_Param::setup_object_defaults(this, var_info);
    _state.var_info = var_info;
}

// read - read the voltage and current
void
AP_BattMonitor_PCU::read()
{
    const uint32_t tnow = AP_HAL::micros();

    if (!_init)
    {
        _uart = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_PCU, 0);
        if (_uart != nullptr) {
            _uart->begin(115200);
            _init = true;
            debug("PCU: started");
        }
        return;
    }

    // send message to PCU for landing gear and LED control
    send_message();

    // Receive battery informations from PCU
    if (parse_message()){
        // Healthy if messages are received
        _state.healthy = true;

        // get voltage
        _state.voltage = (_volt - _volt_offset) * _volt_multiplier;

        // calculate time since last current read
        const uint32_t dt_us = tnow - _state.last_time_micros;
        // read current
        _state.current_amps = (_amp - _amp_offset) * _amp_multiplier;
        update_consumed(_state, dt_us);
        // record time
        _state.last_time_micros = tnow;
    }
    // Not healthy if no messages for 5 seconds
    else if (((tnow - _state.last_time_micros) > PCU_NOT_HEALTY_MICROS)){
        _state.healthy = false;
    }
}

// Return landing gear PWM value
uint16_t AP_BattMonitor_PCU::get_landing_gear()
{
    uint16_t lgr_pwm;
    if (SRV_Channels::get_output_pwm(SRV_Channel::k_landing_gear_control, lgr_pwm)) {
        return lgr_pwm;
    } else {
        return 0;
    }
}

bool AP_BattMonitor_PCU::send_message()
{
    if (_uart == nullptr) {
        // that _uart doesn't exist on this platform
        return false;
    }

    // Get int to send
    uint16_t pwm_LGR = get_landing_gear();

    // Create the serialized message
    bool pre_arm_checks = true;
#if !defined(HAL_BUILD_AP_PERIPH)
    pre_arm_checks = AP::arming().pre_arm_checks(false);
#endif
    _message_to_send[6] = pre_arm_checks ? 1 : 0;    // LED green
    _message_to_send[7] = pre_arm_checks ? 0 : 1;    // LED red
    _message_to_send[8] = static_cast<uint8_t>(pwm_LGR & 0xFF);
    _message_to_send[9] = static_cast<uint8_t>((pwm_LGR >> 8) & 0xFF);

    // Calculate and add checksum
    _message_to_send[1] = crc_xor(_message_to_send, ARRAY_SIZE(_message_to_send), 2) ^ AP_BATT_PCU_CHECKSUM;

    // Write the message to the Serial port
    _uart->write((const uint8_t*)_message_to_send, ARRAY_SIZE(_message_to_send));
    return true;
}

bool AP_BattMonitor_PCU::parse_message()
{
    if (_uart == nullptr) {
        return false;
    }

    // check for bytes on the serial port
    int16_t nbytes = MIN(_uart->available(), 1024U);

    if (nbytes <= 0 ) {
        return false;
    }

    // true if a packet was successfully parsed
    bool parsed = false;

    // process bytes received
    for (int16_t i = 0; i < nbytes; i++) {
        const int16_t b = _uart->read();

        // sanity check byte
        if ((b < 0) || (b > 0xFF)) {
            continue;
        }

        _msg_buff[_msg_buff_len++] = b;

        // flag to allow cases below to reset parser state
        bool reset_parser = false;

        // protect against overly long messages
        if (_msg_buff_len >= AP_BATT_PCU_PACKET_LENGTH) {
            reset_parser = true;
        }

        // process byte depending upon current state
        switch (_parsed_msg.state) {

        case ParseState::WAITING_FOR_HEADER:
            if (b == AP_BATT_PCU_HEADER) {
                _parsed_msg.state = ParseState::WAITING_FOR_CHECKSUM;
            } else {
                reset_parser = true;
            }
            break;

        case ParseState::WAITING_FOR_CHECKSUM:
            _parsed_msg.checksum = b;
            _parsed_msg.state = ParseState::WAITING_FOR_DEVICE_ID;
            break;

        case ParseState::WAITING_FOR_DEVICE_ID:
            _parsed_msg.device_id = b;
            _parsed_msg.state = ParseState::WAITING_FOR_COMMAND;
            break;

        case ParseState::WAITING_FOR_COMMAND:
            _parsed_msg.command_id = b;
            _parsed_msg.state = ParseState::WAITING_FOR_LEN_1;
            break;

        case ParseState::WAITING_FOR_LEN_1:
            _parsed_msg.data_len_1 = b;
            _parsed_msg.state = ParseState::WAITING_FOR_LEN_2;
            break;

        case ParseState::WAITING_FOR_LEN_2:
            _parsed_msg.data_len = UINT16_VALUE(b, _parsed_msg.data_len_1);
            if (_parsed_msg.data_len == AP_BATT_PCU_MESSAGE_MIN_SIZE) {
                _parsed_msg.state = ParseState::WAITING_FOR_PAYLOAD;
            } else {
                reset_parser = true;
                debug("data len should be %u, received: %u", (unsigned)AP_BATT_PCU_MESSAGE_MIN_SIZE, (unsigned)_parsed_msg.data_len);
            }
            break;

        case ParseState::WAITING_FOR_PAYLOAD:
            _parsed_msg.data_bytes_received++;
            if (_parsed_msg.data_bytes_received >= _parsed_msg.data_len) {
                uint8_t crc = crc_xor(_msg_buff, _msg_buff_len, 2) ^ AP_BATT_PCU_CHECKSUM;
                if (crc == _parsed_msg.checksum) {
                    // deserialize packet, right now just one command
                    _volt = UINT16_VALUE(_msg_buff[AP_BATT_PCU_MESSAGE_MIN_SIZE+1], _msg_buff[AP_BATT_PCU_MESSAGE_MIN_SIZE]) * 0.1f;
                    _amp = UINT16_VALUE(_msg_buff[AP_BATT_PCU_MESSAGE_MIN_SIZE+3], _msg_buff[AP_BATT_PCU_MESSAGE_MIN_SIZE+2]) * 0.1f;
                    _temp = UINT16_VALUE(_msg_buff[AP_BATT_PCU_MESSAGE_MIN_SIZE+5], _msg_buff[AP_BATT_PCU_MESSAGE_MIN_SIZE+4]) * 0.1f;
                    parsed = true;
                } else { 
                    debug("checksum expected:%x, got:%x", crc, _parsed_msg.checksum);
                }
                reset_parser = true;
            }
            break;
        }

        // handle reset of parser
        if (reset_parser) {
            _parsed_msg.state = ParseState::WAITING_FOR_HEADER;
            _msg_buff_len = 0;
            _parsed_msg.data_bytes_received = 0;
        }
    }
    return parsed;
}

#endif
