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

    while (nbytes-- > 0) {
        int16_t c = _uart->read();
        if (c==-1) {
            return false;
        }
        if (char(c) == 'T' ) {
            _buffer_count = 0;
        }

        _buffer[buffer_count++] = c;

        // we should always read 19 bytes THxxxxxxxxxxxxxxxxC
        if (_buffer_count >= 19){
            _buffer_count = 0;

            // check if message has right CRC
            if (crc_crc8(_buffer, 18) == buffer[18]){
                uint16_t dist_0 = UINT16_VALUE(_buffer[2],  buffer[3]);
                uint16_t dist_1 = UINT16_VALUE(_buffer[16], buffer[17]);
                uint16_t dist_2 = UINT16_VALUE(_buffer[4],  buffer[5]);
                uint16_t dist_min = MIN(dist_0, MIN(dist_1, dist_2);
                sum_reading_cm += dist_min;
                count_read++;
            }
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
