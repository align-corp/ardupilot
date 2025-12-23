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

#include "AP_RangeFinder_Align_SDM.h"
#if AP_RANGEFINDER_ALIGN_ENABLED
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <stdlib.h>

#ifdef AP_RANGEFINDER_ALIGN_SDM_DEBUG
#define debug(fmt, args ...)  do {hal.console->printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#else
#define debug(fmt, args ...)
#endif

extern const AP_HAL::HAL& hal;

#define HEADER 0x5C

// distance returned in reading_m
bool AP_RangeFinder_Align_SDM::get_reading(float &reading_m)
{
    if (uart == nullptr) {
        return false;
    }

    // store distances for averaging
    uint8_t count_read = 0;
    uint32_t sum_reading_mm = 0;

    // check for bytes on the serial port
    int16_t nbytes = MIN(uart->available(), 1024U);
    if (nbytes <= 0) {
        return false;
    }

    // process bytes received
    for (int16_t i = 0; i < nbytes; i++) {
        const char b = uart->read();

        // protect against overly long messages
        if (_msg_buff_len >= AP_RANGEFINDER_ALIGN_SDM_MAX_PACKET_LENGTH) {
            _state = ParseState::WAITING_FOR_HEADER;
            _msg_buff_len = 0;
        }

        // process byte depending upon current state
        switch (_state) {
        case ParseState::WAITING_FOR_HEADER:
            _msg_buff_len = 0;
            if (b == HEADER) {
                _msg_buff[_msg_buff_len++] = b;
                _state = ParseState::WAITING_FOR_DIST_L;
            } else {
                debug("Wrong header: %d\n", b);
            }
            break;

        case ParseState::WAITING_FOR_DIST_L:
            _msg_buff[_msg_buff_len++] = b;
            _state = ParseState::WAITING_FOR_DIST_H;
            break;

        case ParseState::WAITING_FOR_DIST_H:
            _msg_buff[_msg_buff_len++] = b;
            _state = ParseState::WAITING_FOR_CHECKSUM;
            break;

        case ParseState::WAITING_FOR_CHECKSUM:
            uint8_t checksum_rec = b;
            uint8_t checksum_calc = ~(_msg_buff[1] + _msg_buff[2]) & 0xFF;

            if (checksum_calc == checksum_rec) {
                uint16_t dist_mm = UINT16_VALUE(_msg_buff[2], _msg_buff[1]);
                sum_reading_mm += dist_mm;
                count_read++;
            } else {
                debug("Checksum mismatch: recv=%02X calc=%02X", checksum_rec , checksum_calc);
            }
            _state = ParseState::WAITING_FOR_HEADER;
            _msg_buff_len = 0;
            break;
        }
    }

    // average reads
    if (count_read > 0) {
        reading_m = sum_reading_mm * 0.001f / count_read;
        debug("count = %d, sum = %lu, dist = %f\n", count_read, sum_reading_mm, reading_m);
        return true;
    }
    return false;
}

#endif  // AP_RANGEFINDER_ALIGN_ENABLED
