#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_ALIGN_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend_Serial.h"

/*
  ARadTek radar (e.g. on the MT40), UART output. Each 18 byte frame is:
    0xFF 0xFF
    5 x uint16 LE distance, nearest obstacle first, 0 = no obstacle
    5 x uint8 SNR, 0 = no obstacle
    checksum = sum of the first 17 bytes & 0xFF
  Only the nearest obstacle is used.
 */
class AP_RangeFinder_ARadTek_Serial : public AP_RangeFinder_Backend_Serial
{

public:
    static AP_RangeFinder_Backend_Serial *create(
            RangeFinder::RangeFinder_State &_state,
            AP_RangeFinder_Params &_params) {
        return new AP_RangeFinder_ARadTek_Serial(_state, _params);
    }

protected:

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_RADAR;
    }

private:
    using AP_RangeFinder_Backend_Serial::AP_RangeFinder_Backend_Serial;

    // distance returned in reading_m
    bool get_reading(float &reading_m) override;

    // drop bytes from the front of the buffer up to the next candidate header
    void resync();

    static constexpr uint8_t FRAME_LEN = 18;
    static constexpr uint8_t HEADER = 0xFF;
    static constexpr uint8_t NUM_OBSTACLES = 5;

    uint8_t buf[FRAME_LEN];
    uint8_t buf_len;
};

#endif  // AP_RANGEFINDER_ALIGN_ENABLED
