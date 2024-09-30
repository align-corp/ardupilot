#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_ALIGN_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend_Serial.h"

#define AP_RANGEFINDER_ALIGN_RDR01_MAX_PACKET_LENGTH 6

class AP_RangeFinder_Align_RDR01 : public AP_RangeFinder_Backend_Serial
{

public:
    static AP_RangeFinder_Backend_Serial *create(
            RangeFinder::RangeFinder_State &_state,
            AP_RangeFinder_Params &_params) {
            return new AP_RangeFinder_Align_RDR01(_state, _params);
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
        WAITING_FOR_HEADER,
        WAITING_FOR_DEV_NUMBER,
        WAITING_FOR_FIRMWARE_VERSION,
        WAITING_FOR_LOW_BYTE,
        WAITING_FOR_HIGH_BYTE,
        WAITING_FOR_CHECKSUM,
    };

    ParseState _state;
    uint8_t _dist_high_byte;
    uint8_t _dist_low_byte;
    uint8_t _msg_buff[AP_RANGEFINDER_ALIGN_RDR01_MAX_PACKET_LENGTH];
    uint8_t _msg_buff_len;
};

#endif  // AP_RANGEFINDER_ALIGN_ENABLED
