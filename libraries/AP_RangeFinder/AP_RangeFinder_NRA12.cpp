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

#include "AP_RangeFinder_NRA12.h"

#if AP_RANGEFINDER_ALIGN_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

#include <ctype.h>

#ifdef AP_RANGEFINDER_NRA12_DEBUG
#define debug(fmt, args ...)  do {hal.console->printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#else
#define debug(fmt, args ...)
#endif

extern const AP_HAL::HAL& hal;

#define AP_RANGEFINDER_NRA12_DIST_MAX_CM 2000
#define NRA12_FRAME_HEADER 0xAA
#define NRA12_FRAME_HEADER 0xAA
#define NRA12_FRAME_SENSOR_STATUS 0x060A
#define NRA12_FRAME_TARGET_INFO 0x070C
#define NRA12_FRAME_PAYLOAD_LENGTH 8
#define NRA12_FRAME_END 0x55

// distance returned in reading_m
bool AP_RangeFinder_NRA12::get_reading(float &reading_m)
{
    if (uart == nullptr) {
        return false;
    }

    // store distances for averaging
    bool radar_healthy = false;
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
        if (_msg_buff_len >= AP_RANGEFINDER_NRA12_MAX_PACKET_LENGTH) {
            reset_parser = true;
        }

        // process byte depending upon current state
        switch (_parsed_msg.state) {

        case ParseState::WAITING_FOR_HEADER1:
            if (b == NRA12_FRAME_HEADER) {
                _parsed_msg.state = ParseState::WAITING_FOR_HEADER2;
            } else {
                reset_parser = true;
            }
            break;

        case ParseState::WAITING_FOR_HEADER2:
            if (b == NRA12_FRAME_HEADER) {
                _parsed_msg.state = ParseState::WAITING_FOR_MESSAGE_ID1;
            } else {
                reset_parser = true;
            }
            break;

        case ParseState::WAITING_FOR_MESSAGE_ID1:
            _parsed_msg.message_id = b;
            _parsed_msg.state = ParseState::WAITING_FOR_MESSAGE_ID2;
            break;

        case ParseState::WAITING_FOR_MESSAGE_ID2:
            _parsed_msg.message_id = (b << 8) | _parsed_msg.message_id;
            _parsed_msg.state = ParseState::WAITING_FOR_PAYLOAD;
            _parsed_msg.data_bytes_received = 0;
            break;

        case ParseState::WAITING_FOR_PAYLOAD:
            _parsed_msg.data_bytes_received++;
            if (_parsed_msg.data_bytes_received == NRA12_FRAME_PAYLOAD_LENGTH) {
                _parsed_msg.state = ParseState::WAITING_FOR_END1;
            }
            break;

        case ParseState::WAITING_FOR_END1:
            if (b == NRA12_FRAME_END) {
                _parsed_msg.state = ParseState::WAITING_FOR_END2;
            } else {
                reset_parser = true;
            }
            break;

        case ParseState::WAITING_FOR_END2:
            if (b == NRA12_FRAME_END) {
                switch (_parsed_msg.message_id)
                {
                case NRA12_FRAME_TARGET_INFO:
                    if (_msg_buff[4] == 1) {
                        // check only target 0
                        uint16_t dist_cm = UINT16_VALUE(_msg_buff[6], _msg_buff[7]);
                        if (dist_cm > AP_RANGEFINDER_NRA12_DIST_MAX_CM) {
                            dist_cm = AP_RANGEFINDER_NRA12_DIST_MAX_CM;
                        }
                        sum_reading_cm += dist_cm;
                        count_read++;
                    }
                    break;

                case NRA12_FRAME_SENSOR_STATUS:
                    radar_healthy = true;
                    break;
                
                default:
                    break;
                }
            }
            reset_parser = true;
            break;
        }

        // handle reset of parser
        if (reset_parser) {
            _parsed_msg.state = ParseState::WAITING_FOR_HEADER1;
            _msg_buff_len = 0;
        }
    }

    // average reads
    if (count_read > 0) {
        reading_m = sum_reading_cm * 0.01f / count_read;
        return true;
    }

    if (radar_healthy) {
        reading_m = (AP_RANGEFINDER_NRA12_DIST_MAX_CM+500) * 0.01f;
        return true;
    }

    return false;
}

#endif  // AP_RANGEFINDER_ALIGN_ENABLED
