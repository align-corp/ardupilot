#pragma once

#include "AP_Proximity_config.h"

#if AP_PROXIMITY_N10P_ENABLED

#include "AP_Proximity_Backend_Serial.h"

#define PROXIMITY_N10P_TIMEOUT_MS       500
#define PROXIMITY_N10P_PACKET_SIZE      108
#define	PROXIMITY_N10P_PACKAGE_POINTS   32

// packet definition
#define PROXIMITY_N10P_HEADER1     0xA5
#define PROXIMITY_N10P_HEADER2     0x5A
#define PROXIMITY_N10P_LENGTH      0x6C

class AP_Proximity_N10P : public AP_Proximity_Backend_Serial {

public:

    using AP_Proximity_Backend_Serial::AP_Proximity_Backend_Serial;

    // update state
    void update(void) override;

    // get maximum and minimum distances (in meters) of sensor
    float distance_max() const override { return 12.0f; }
    float distance_min() const override { return 0.1f; }

private:

    // check and process replies from sensor
    bool read_sensor_data();
    void update_sector_data(float angle_deg, uint16_t distance_cm);
    uint8_t crc8_N10(const uint8_t *p, uint8_t len);

    enum N10P_state {
        WAIT_HEADER1 = 0,
        WAIT_HEADER2,
        WAIT_LENGTH,
        WAIT_CHECKSUM
    };

    uint8_t _parse_state = WAIT_HEADER1;
    uint8_t _buffer[PROXIMITY_N10P_PACKET_SIZE]; // buffer where to store data from serial
    uint8_t _buffer_count = 0;
    uint32_t _last_distance_received_ms; // system time of last distance measurement received from sensor
};

#endif // AP_PROXIMITY_N10P_ENABLED
