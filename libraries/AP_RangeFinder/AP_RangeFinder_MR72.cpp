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

#include "AP_RangeFinder_MR72.h"

#if AP_RANGEFINDER_ALIGN_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

#include <ctype.h>

#ifdef AP_RANGEFINDER_MR72_DEBUG
#define debug(fmt, args ...)  do {hal.console->printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#else
#define debug(fmt, args ...)
#endif

extern const AP_HAL::HAL& hal;

#define MR72_HEADER 0xAA
#define MR72_TARGET_STATUS 0x0C
#define MR72_TARGET_INFO 0x0B
#define MR72_TARGET_COMMON2 0x70
#define MR72_END 0x55

// distance returned in reading_m
bool AP_RangeFinder_MR72::get_reading(float &reading_m)
{
    if (uart == nullptr) {
        return false;
    }

    // store distances for averaging
    uint16_t sum_reading_cm = 0;
    uint8_t count_read = 0;

    // check for bytes on the serial port
    int16_t nbytes = MIN(uart->available(), 1024U);
    if (nbytes <= 0 ) {
        return false;
    }

    // flag to allow cases below to reset parser state
    bool reset_parser = false;

    // process bytes received
    for (int16_t i = 0; i < nbytes; i++) {
        const int16_t b = uart->read();

        // sanity check byte
        if ((b < 0) || (b > 0xFF)) {
            continue;
        }


        // process byte depending upon current state
        switch (_msg.state) {

            case ParseState::WAITING_FOR_HEADER1:
            if (b == MR72_HEADER) {
                _msg.state = ParseState::WAITING_FOR_HEADER2;
            } else {
                reset_parser = true;
            }
            break;

        case ParseState::WAITING_FOR_HEADER2:
            if (b == MR72_HEADER) {
                _msg.state = ParseState::WAITING_FOR_MESSAGE_ID1;
            } else {
                reset_parser = true;
            }
            break;

        case ParseState::WAITING_FOR_MESSAGE_ID1:
            _msg.message_id = b;
            _msg.state = ParseState::WAITING_FOR_MESSAGE_ID2;
            break;

        case ParseState::WAITING_FOR_MESSAGE_ID2:
            if (b == MR72_TARGET_COMMON2) {
                _msg.state = ParseState::WAITING_FOR_PAYLOAD;
            } else {
                reset_parser = true;
            }
            break;

        case ParseState::WAITING_FOR_PAYLOAD:
            _msg.payload[_msg.payload_index++] = b;
            if (_msg.payload_index == MR72_PAYLOAD_LENGTH) {
                _msg.state = ParseState::WAITING_FOR_END1;
            }
            break;

        case ParseState::WAITING_FOR_END1:
            _msg.state = ParseState::WAITING_FOR_END2; 
            break;

        case ParseState::WAITING_FOR_END2:
            // use TARGET_INFO to know how many targets were detected
            if (_msg.message_id == MR72_TARGET_INFO) {
                // update targets found
                _msg.targets_found = _msg.payload[0];
                _msg.target_index = 0;
            }

            // use TARGET_STATUS to get the nearest target detected
            else if (_msg.message_id == MR72_TARGET_STATUS) {
                uint16_t dist = UINT16_VALUE(_msg.payload[2], _msg.payload[3]);
                if (_msg.target_index == 0) {
                    _msg.min_dist = dist;
                } else {
                    _msg.min_dist = MIN(_msg.min_dist, dist);
                }

                // last target, use the minimum distance for averaging
                if (_msg.target_index++ == _msg.targets_found) {
                    sum_reading_cm += _msg.min_dist;
                    count_read++;
                }
            }
            break;
        }


        // handle reset of parser
        if (reset_parser) {
            _msg.state = ParseState::WAITING_FOR_HEADER1;
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
