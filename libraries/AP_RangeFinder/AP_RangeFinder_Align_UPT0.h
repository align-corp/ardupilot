#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_ALIGN_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend_Serial.h"

#define AP_RANGEFINDER_ALIGN_UPT0_MAX_PACKET_LENGTH 19

class AP_RangeFinder_Align_UPT0 : public AP_RangeFinder_Backend_Serial
{

public:
    static AP_RangeFinder_Backend_Serial *create(
            RangeFinder::RangeFinder_State &_state,
            AP_RangeFinder_Params &_params) {
            return new AP_RangeFinder_Align_UPT0(_state, _params);
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
    bool parse(uint16_t &dist_mm);
    uint8_t parse_hex2(const char *s);
    void parse_hex_buff(const char *s, uint8_t *buff, size_t buff_len);

    enum class ParseState : uint8_t {
        WAITING_FOR_HEADER,
        WAITING_FOR_CR,
        WAITING_FOR_LF,
    };

    ParseState _state;
    char _msg_buff[AP_RANGEFINDER_ALIGN_UPT0_MAX_PACKET_LENGTH ];
    uint8_t _msg_buff_len;
};

#endif  // AP_RANGEFINDER_ALIGN_ENABLED
