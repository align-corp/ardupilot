#pragma once

#include "AP_BattMonitor_Backend.h"

#if AP_BATTERY_PCU_ENABLED

#include "AP_BattMonitor.h"

#define AP_BATT_PCU_VOLT_MULT  1
#define AP_BATT_PCU_VOLT_OFFS  0
#define AP_BATT_PCU_AMP_MULT  1
#define AP_BATT_PCU_AMP_OFFS  0

// Protocol 
#define AP_BATT_PCU_HEADER 0x3F
#define AP_BATT_PCU_DEVICE_ID 0xAA
#define AP_BATT_PCU_MESSAGE_MIN_SIZE 6
#define AP_BATT_PCU_COMMAND 0x3E
#define AP_BATT_PCU_PACKET_LENGTH 12
#define AP_BATT_PCU_PACKET_SEND_LENGTH 11
#define AP_BATT_PCU_SEND_LENGTH 5
#define AP_BATT_PCU_CHECKSUM 0x21
#define AP_BATT_PCU_LGR_STARTUP 1000

class AP_BattMonitor_PCU : public AP_BattMonitor_Backend
{
public:

    /// Constructor
    AP_BattMonitor_PCU(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, AP_BattMonitor_Params &params);

    /// Read the battery voltage and current.  Should be called at 10hz
    virtual void read() override;

    /// returns true if battery monitor instance provides current info
    bool has_current() const override { return true; };

    /// returns true if battery monitor provides consumed energy info
    virtual bool has_consumed_energy() const override { return true; }

    virtual void init(void) override {}

    static const struct AP_Param::GroupInfo var_info[];

    

protected:
    // Parameters
    AP_Float _volt_multiplier;          /// voltage on volt pin multiplied by this to calculate battery voltage
    AP_Float _amp_multiplier;        /// voltage on current pin multiplied by this to calculate current in amps
    AP_Float _amp_offset;          /// offset voltage that is subtracted from current pin before conversion to amps
    AP_Float _volt_offset;              /// offset voltage that is subtracted from voltage pin before conversion

private:
    // Serialize the message, call at 10 Hz
    bool send_message();

    // Receive and parse the message, call at 10 Hz
    bool parse_message();

    // Check landing gear PWM
    uint16_t get_landing_gear();

    enum class ParseState : uint8_t {
        WAITING_FOR_HEADER,
        WAITING_FOR_CHECKSUM,
        WAITING_FOR_DEVICE_ID,
        WAITING_FOR_COMMAND,
        WAITING_FOR_LEN_1,
        WAITING_FOR_LEN_2,
        WAITING_FOR_PAYLOAD,
    };

    // parser state and unpacked fields
    struct PACKED {
        uint8_t data_len_1;
        uint16_t data_len;                          // expected number of data bytes
        uint8_t device_id;
        uint8_t command_id;                         // command id
        uint16_t data_bytes_received;               // number of data bytes received so far
        uint8_t checksum;                           // latest message's crc
        ParseState state;                     // state of incoming message processing
    } _parsed_msg;

    AP_HAL::UARTDriver *_uart = nullptr;
    bool _init = false;
    float _volt = 0;
    float _amp = 0;
    float _temp = 0;
    uint8_t _message_to_send[AP_BATT_PCU_PACKET_SEND_LENGTH] = {
        AP_BATT_PCU_HEADER,
        0, // checksum
        AP_BATT_PCU_DEVICE_ID,
        AP_BATT_PCU_COMMAND,
        LOWBYTE(AP_BATT_PCU_SEND_LENGTH),
        HIGHBYTE(AP_BATT_PCU_SEND_LENGTH),
        1, // LED Green
        1, // LED Red
        LOWBYTE(AP_BATT_PCU_LGR_STARTUP),
        HIGHBYTE(AP_BATT_PCU_LGR_STARTUP),
        0 // Turn OFF
        };
    uint8_t _msg_buff[AP_BATT_PCU_PACKET_LENGTH];
    uint8_t _msg_buff_len = 0;
};

#endif
