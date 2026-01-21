/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "AP_Proximity_config.h"

#if AP_PROXIMITY_TOPAZB_ENABLED

#include "AP_Proximity_Backend_Serial.h"

// timeout if no data received within 500ms
#define PROXIMITY_TOPAZB_TIMEOUT_MS         500

// packet structure: Header(2) + StartAngle(2) + ScanRate(2) + Data(96) + CRC(2) = 104 bytes
#define PROXIMITY_TOPAZB_PACKET_SIZE        104
#define PROXIMITY_TOPAZB_PACKAGE_POINTS     32

// packet header bytes (little-endian 0x05FA)
#define PROXIMITY_TOPAZB_HEADER1            0xFA
#define PROXIMITY_TOPAZB_HEADER2            0x05

// error packet header (little-endian 0xEEFA)
#define PROXIMITY_TOPAZB_ERROR_HEADER1      0xFA
#define PROXIMITY_TOPAZB_ERROR_HEADER2      0xEE

// special distance values
#define PROXIMITY_TOPAZB_DIST_LEVEL_LOW     65535   // signal level too low
#define PROXIMITY_TOPAZB_DIST_SENSOR_ERROR  65534   // sensor abnormal

class AP_Proximity_TopazB : public AP_Proximity_Backend_Serial
{

public:
    using AP_Proximity_Backend_Serial::AP_Proximity_Backend_Serial;

    // update state
    void update(void) override;

    // get maximum and minimum distances (in meters) of sensor
    float distance_max() const override { return 12.0f; }
    float distance_min() const override { return 0.15f; }

private:
    // check and process replies from sensor
    bool read_sensor_data();

    // process a complete valid packet
    void process_packet();

    // update sector data with a single measurement
    void update_sector_data(float angle_deg, uint16_t distance_mm);

    // parser state machine
    enum ParseState {
        WAIT_HEADER1 = 0,
        WAIT_HEADER2,
        WAIT_PAYLOAD,
    };

    ParseState _parse_state = WAIT_HEADER1;
    uint8_t _buffer[PROXIMITY_TOPAZB_PACKET_SIZE];  // buffer for incoming packet
    uint8_t _buffer_count = 0;
    bool _is_error_packet = false;                   // true if parsing error packet

    uint32_t _last_distance_received_ms;            // system time of last valid distance

    // sector tracking for finding shortest distance per sector
    uint8_t _last_sector = UINT8_MAX;
    uint16_t _shortest_distance_mm;
    bool _invalidate_sector;
};

#endif // AP_PROXIMITY_TOPAZB_ENABLED
