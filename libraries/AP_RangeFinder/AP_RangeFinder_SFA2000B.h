#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_SFA2000B_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend_Serial.h"

// Sunflaser SFA2000B micro laser (TOF) rangefinder module, UART-TTL interface.
// The module is commanded into continuous-ranging mode and then streams fixed
// 8-byte frames containing the measured distance.

class AP_RangeFinder_SFA2000B : public AP_RangeFinder_Backend_Serial
{

public:
    static AP_RangeFinder_Backend_Serial *create(
            RangeFinder::RangeFinder_State &_state,
            AP_RangeFinder_Params &_params) {
        return new AP_RangeFinder_SFA2000B(_state, _params);
    }

protected:

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

private:
    using AP_RangeFinder_Backend_Serial::AP_RangeFinder_Backend_Serial;

    // the SFA2000B defaults to 115200 baud
    uint32_t initial_baudrate(uint8_t serial_instance) const override {
        return 115200;
    }

    // get a reading; distance returned in reading_m
    bool get_reading(float &reading_m) override;

    // allow plenty of margin above the slowest (1Hz) continuous rate
    uint16_t read_timeout_ms() const override { return 1000; }

    // send the continuous-ranging command to the module
    void send_command();

    // try to decode a single received byte; returns true and fills reading_m
    // when a complete, valid measurement frame has been parsed
    bool parse_byte(uint8_t b, float &reading_m);

    uint8_t linebuf[8];
    uint8_t linebuf_len;
    uint32_t last_command_ms;       // system time the start command was last sent
    uint32_t last_distance_ms;      // system time a valid distance was last decoded
};

#endif  // AP_RANGEFINDER_SFA2000B_ENABLED
