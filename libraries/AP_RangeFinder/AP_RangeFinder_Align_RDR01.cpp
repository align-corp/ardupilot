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

#include "AP_RangeFinder_Align_RDR01.h"

#if AP_RANGEFINDER_ALIGN_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

#include <ctype.h>

#ifdef AP_RANGEFINDER_ALIGN_RDR01_DEBUG
#define debug(fmt, args ...)  do {hal.console->printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#else
#define debug(fmt, args ...)
#endif

extern const AP_HAL::HAL& hal;

#define ALIGN_RDR01_FRAME_HEADER 0x55

// distance returned in reading_m
bool AP_RangeFinder_Align_RDR01::get_reading(float &reading_m)
{
    if (uart == nullptr) {
        return false;
    }

    // store distances for averaging
    uint8_t count_read = 0;
    uint32_t sum_reading_cm = 0;

    // check for bytes on the serial port
    int16_t nbytes = MIN(uart->available(), 1024U);

    if (nbytes <= 0 ) {
        return false;
    }

    // process bytes received
    for (int16_t i = 0; i < nbytes; i++) {
        const int16_t b = uart->read();

        // sanity check byte
        if ((b < 0) || (b > 0xFF)) {
            continue;
        }

        _msg_buff[_msg_buff_len++] = b;

        // flag to allow cases below to reset parser state
        bool reset_parser = false;

        // protect against overly long messages
        if (_msg_buff_len >= AP_RANGEFINDER_ALIGN_RDR01_MAX_PACKET_LENGTH) {
            reset_parser = true;
        }

        // process byte depending upon current state
        switch (_state) {

        case ParseState::WAITING_FOR_HEADER:
            if (b == ALIGN_RDR01_FRAME_HEADER) {
                _state = ParseState::WAITING_FOR_DEV_NUMBER;
            } else {
                reset_parser = true;
            }
            break;

        case ParseState::WAITING_FOR_DEV_NUMBER:
            _state = ParseState::WAITING_FOR_FIRMWARE_VERSION;
            break;

        case ParseState::WAITING_FOR_FIRMWARE_VERSION:
            _state = ParseState::WAITING_FOR_LOW_BYTE;
            break;

        case ParseState::WAITING_FOR_LOW_BYTE:
            _dist_low_byte = b;
            _state = ParseState::WAITING_FOR_HIGH_BYTE;
            break;

        case ParseState::WAITING_FOR_HIGH_BYTE:
            _dist_high_byte = b;
            _state = ParseState::WAITING_FOR_CHECKSUM;
            break;

        case ParseState::WAITING_FOR_CHECKSUM:
            uint8_t checksum = crc_sum_of_bytes(_msg_buff+1, 4);
            if (b == checksum) {
                uint16_t dist_cm = UINT16_VALUE(_dist_high_byte, _dist_low_byte);
                sum_reading_cm += dist_cm;
                count_read++;
            } else {
                debug("checksum expected:%x, got:%x", checksum, b);
            }
            reset_parser = true;
            break;
        }

        // handle reset of parser
        if (reset_parser) {
            _state = ParseState::WAITING_FOR_HEADER;
            _msg_buff_len = 0;
        }
    }

    // average reads
    if (count_read > 0) {
        reading_m = sum_reading_cm * 0.01f / count_read;
        return true;
    }

    return false;
}

#endif  // AP_RANGEFINDER_ALIGN_ENABLED
