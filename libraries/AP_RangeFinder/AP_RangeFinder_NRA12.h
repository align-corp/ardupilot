#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_ALIGN_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend_Serial.h"

#define AP_RANGEFINDER_NRA12_MAX_PACKET_LENGTH 14

class AP_RangeFinder_NRA12 : public AP_RangeFinder_Backend_Serial
{

public:
    static AP_RangeFinder_Backend_Serial *create(
            RangeFinder::RangeFinder_State &_state,
            AP_RangeFinder_Params &_params) {
            return new AP_RangeFinder_NRA12(_state, _params);
     }

protected:

    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_RADAR;
    }

private:
    using AP_RangeFinder_Backend_Serial::AP_RangeFinder_Backend_Serial;


    // get a reading
    // distance returned in reading_m
    bool get_reading(float &reading_m) override;

    enum class ParseState : uint8_t {
        WAITING_FOR_HEADER1,
        WAITING_FOR_HEADER2,
        WAITING_FOR_MESSAGE_ID1,
        WAITING_FOR_MESSAGE_ID2,
        WAITING_FOR_PAYLOAD,
        WAITING_FOR_END1,
        WAITING_FOR_END2,
    };

    // parser state and unpacked fields
    struct PACKED {
        uint16_t message_id;
        uint16_t data_bytes_received;               // number of data bytes received so far
        ParseState state;                     // state of incoming message processing
    } _parsed_msg;

    uint8_t _msg_buff[AP_RANGEFINDER_NRA12_MAX_PACKET_LENGTH];
    uint8_t _msg_buff_len;
};

#endif  // AP_RANGEFINDER_ALIGN_ENABLED
