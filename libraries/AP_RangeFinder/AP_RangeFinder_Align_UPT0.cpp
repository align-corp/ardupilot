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

#include "AP_RangeFinder_Align_UPT0.h"
#if AP_RANGEFINDER_ALIGN_ENABLED
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <stdlib.h>

#ifdef AP_RANGEFINDER_ALIGN_UPT0_DEBUG
#define debug(fmt, args ...)  do {hal.console->printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#else
#define debug(fmt, args ...)
#endif

extern const AP_HAL::HAL& hal;

#define HEADER '~'
#define FRAME_LENGTH 19  // ~(1) + data(16) + \r\n(2)

// distance returned in reading_m
bool AP_RangeFinder_Align_UPT0::get_reading(float &reading_m)
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
        if (_msg_buff_len >= AP_RANGEFINDER_ALIGN_UPT0_MAX_PACKET_LENGTH) {
            _state = ParseState::WAITING_FOR_HEADER;
            _msg_buff_len = 0;
        }

        // process byte depending upon current state
        switch (_state) {
        case ParseState::WAITING_FOR_HEADER:
            _msg_buff_len = 0;
            if (b == HEADER) {
                _msg_buff[_msg_buff_len++] = b;
                _state = ParseState::WAITING_FOR_CR;
            }
            break;

        case ParseState::WAITING_FOR_CR:
            _msg_buff[_msg_buff_len++] = b;
            if (b == '\r') {
                _state = ParseState::WAITING_FOR_LF;
            }
            break;

        case ParseState::WAITING_FOR_LF:
            _msg_buff[_msg_buff_len++] = b;
            if (b == '\n') {
                uint16_t dist_mm;
                if (parse(dist_mm)) {
                    sum_reading_mm += dist_mm;
                    count_read++;
                }
            }
            _state = ParseState::WAITING_FOR_HEADER;
            _msg_buff_len = 0;
            break;
        }
    }

    // average reads
    if (count_read > 0) {
        reading_m = sum_reading_mm * 0.001f / count_read;
        return true;
    }
    return false;
}

// parse 2 hex ASCII characters into uint8_t
uint8_t AP_RangeFinder_Align_UPT0::parse_hex2(const char *s)
{
    char tmp[3] = {s[0], s[1], '\0'};
    return (uint8_t)strtol(tmp, nullptr, 16);
}

void AP_RangeFinder_Align_UPT0::parse_hex_buff(const char *s, uint8_t *buff, size_t buff_len) {
    for (size_t i=0; i<buff_len; i++) {
        buff[i] = parse_hex2(s+i*2);
    }
}

bool AP_RangeFinder_Align_UPT0::parse(uint16_t &dist_mm)
{
    // frame: ~01030100XXXXCCCC\r\n (19 chars)
    // positions: 0=~, 1-2=addr, 3-4=cmd, 5-8=reg, 9-12=dist, 13-14=crc_l, 15-16=crc_h, 17=\r, 18=\n

    // check length
    if (_msg_buff_len != FRAME_LENGTH) {
        debug("Wrong length: %d", _msg_buff_len);
        return false;
    }

    // check header
    if (strncmp(_msg_buff, "~01030100", 9) != 0) {
        debug("Wrong header");
        return false;
    }

    // from ASCII to byte buffer
    uint8_t buff[8];
    parse_hex_buff(_msg_buff+1, buff, sizeof(buff));

    // parse CRC from message (low byte first, unbelivable)
    uint16_t crc_received = UINT16_VALUE(buff[7], buff[6]);

    // calculate CRC over bytes 1-6 (addr, cmd, reg, data)
    uint16_t crc_calculated = calc_crc_modbus(buff, 6);

    if (crc_received != crc_calculated) {
        debug("CRC mismatch: recv=0x%04X calc=0x%04X", crc_received, crc_calculated);
        return false;
    }

    // parse distance (high byte first)
    dist_mm = UINT16_VALUE(buff[4], buff[5]);

    return true;
}

#endif  // AP_RANGEFINDER_ALIGN_ENABLED
