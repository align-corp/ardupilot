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

#include "AP_RangeFinder_SFA2000B.h"

#if AP_RANGEFINDER_SFA2000B_ENABLED

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

// Sunflaser SFA2000B serial protocol (see product manual, appendix 1).
//
// All frames (command and reply) are 8 bytes long:
//   byte 0    0x55                  frame header
//   byte 1    0xAA                  frame header
//   byte 2    command / frequency   0x88 single, 0x8E stop, or a Freq code
//   byte 3    status                in replies: 1=valid, 0=measurement failed
//   byte 4    0xFF
//   byte 5    DATA_H                distance high byte
//   byte 6    DATA_L                distance low byte
//   byte 7    checksum              low byte of the sum of bytes 0..6
//
// The distance value (DATA_H<<8 | DATA_L) is the real distance in metres
// multiplied by 10, i.e. 0.1m resolution (e.g. 20003 -> 2000.3m).
//
// Continuous-ranging frequency codes:
//   0x89 = 1Hz, 0xB9 = 5Hz, 0xC9 = 10Hz, 0xF9 = boresight/alignment mode.

#define SFA2000B_FRAME_HEADER_0     0x55
#define SFA2000B_FRAME_HEADER_1     0xAA
#define SFA2000B_FRAME_LEN          8

// continuous ranging at 10Hz for a responsive update rate
#define SFA2000B_FREQ_10HZ          0xC9

// distance values are reported in units of 0.1m
#define SFA2000B_DIST_SCALE         0.1f

// send the continuous-ranging command to the module
void AP_RangeFinder_SFA2000B::send_command()
{
    if (uart == nullptr) {
        return;
    }

    // continuous ranging command: 0x55 0xAA Freq 0xFF 0xFF 0xFF 0xFF checksum
    // the send checksum is the low byte of the sum of bytes 2..6
    uint8_t cmd[SFA2000B_FRAME_LEN] = {
        SFA2000B_FRAME_HEADER_0,
        SFA2000B_FRAME_HEADER_1,
        SFA2000B_FREQ_10HZ,
        0xFF, 0xFF, 0xFF, 0xFF,
        0x00,
    };
    uint8_t checksum = 0;
    for (uint8_t i = 2; i < SFA2000B_FRAME_LEN - 1; i++) {
        checksum += cmd[i];
    }
    cmd[SFA2000B_FRAME_LEN - 1] = checksum;

    uart->write(cmd, sizeof(cmd));
}

// try to decode a single received byte; returns true and fills reading_m
// when a complete, valid measurement frame has been parsed
bool AP_RangeFinder_SFA2000B::parse_byte(uint8_t b, float &reading_m)
{
    // resynchronise on the two-byte header
    if (linebuf_len == 0) {
        if (b != SFA2000B_FRAME_HEADER_0) {
            return false;
        }
    } else if (linebuf_len == 1) {
        if (b != SFA2000B_FRAME_HEADER_1) {
            // restart, allowing this byte to be a fresh header
            linebuf_len = 0;
            if (b == SFA2000B_FRAME_HEADER_0) {
                linebuf[linebuf_len++] = b;
            }
            return false;
        }
    }

    linebuf[linebuf_len++] = b;

    if (linebuf_len < SFA2000B_FRAME_LEN) {
        return false;
    }

    // we have a full frame; reset the buffer for the next one
    linebuf_len = 0;

    // verify checksum (low byte of the sum of the first 7 bytes)
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < SFA2000B_FRAME_LEN - 1; i++) {
        checksum += linebuf[i];
    }
    if (checksum != linebuf[SFA2000B_FRAME_LEN - 1]) {
        return false;
    }

    // byte 3 is the status: 1 = valid measurement, 0 = measurement failed
    if (linebuf[3] != 1) {
        return false;
    }

    const uint16_t raw = (uint16_t(linebuf[5]) << 8) | linebuf[6];
    reading_m = raw * SFA2000B_DIST_SCALE;
    return true;
}

// distance returned in reading_m
bool AP_RangeFinder_SFA2000B::get_reading(float &reading_m)
{
    if (uart == nullptr) {
        return false;
    }

    const uint32_t now_ms = AP_HAL::millis();

    // (re)issue the continuous-ranging command if the module has gone quiet,
    // e.g. on first run or after a power cycle
    if (now_ms - last_distance_ms > read_timeout_ms() &&
        now_ms - last_command_ms > read_timeout_ms()) {
        send_command();
        last_command_ms = now_ms;
    }

    bool got_reading = false;
    uint32_t nbytes = uart->available();
    while (nbytes-- > 0) {
        uint8_t c;
        if (!uart->read(c)) {
            break;
        }
        if (parse_byte(c, reading_m)) {
            got_reading = true;
            last_distance_ms = now_ms;
        }
    }

    return got_reading;
}

#endif  // AP_RANGEFINDER_SFA2000B_ENABLED
