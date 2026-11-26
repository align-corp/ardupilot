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

#include "AP_RangeFinder_ARadTek_Serial.h"

#if AP_RANGEFINDER_ALIGN_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

// distances in the frame are in cm
#define ARADTEK_OUT_OF_RANGE_ADD_CM 100

// shift the buffer to the next position where a 0xFF 0xFF header could start.
// Distance bytes can legitimately be 0xFF, so a bad checksum only drops one byte.
void AP_RangeFinder_ARadTek_Serial::resync()
{
    uint8_t i = 1;
    for (; i < buf_len; i++) {
        if (buf[i] == HEADER && (i + 1 >= buf_len || buf[i+1] == HEADER)) {
            break;
        }
    }
    buf_len -= i;
    memmove(buf, &buf[i], buf_len);
}

bool AP_RangeFinder_ARadTek_Serial::get_reading(float &reading_m)
{
    if (uart == nullptr) {
        return false;
    }

    uint32_t sum_cm = 0;
    uint16_t valid_count = 0;
    uint16_t no_target_count = 0;

    for (uint16_t n = uart->available(); n > 0; n--) {
        uint8_t b;
        if (!uart->read(b)) {
            break;
        }
        if (buf_len < 2 && b != HEADER) {
            buf_len = 0;
            continue;
        }
        buf[buf_len++] = b;
        if (buf_len < FRAME_LEN) {
            continue;
        }

        if (crc_sum_of_bytes(buf, FRAME_LEN-1) != buf[FRAME_LEN-1]) {
            resync();
            continue;
        }

        // nearest obstacle is first
        const uint16_t dist_cm = UINT16_VALUE(buf[3], buf[2]);
        const uint8_t snr = buf[2 + NUM_OBSTACLES*2];
        if (dist_cm == 0 || snr == 0) {
            no_target_count++;
        } else {
            sum_cm += dist_cm;
            valid_count++;
        }
        buf_len = 0;
    }

    if (valid_count > 0) {
        reading_m = sum_cm * 0.01f / valid_count;
        return true;
    }

    if (no_target_count > 0) {
        // nothing in view: report out of range high
        reading_m = (max_distance_cm() + ARADTEK_OUT_OF_RANGE_ADD_CM) * 0.01f;
        return true;
    }

    return false;
}

#endif  // AP_RANGEFINDER_ALIGN_ENABLED
