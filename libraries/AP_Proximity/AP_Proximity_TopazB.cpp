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

#include "AP_Proximity_config.h"

#if AP_PROXIMITY_TOPAZB_ENABLED

#include "AP_Proximity_TopazB.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL& hal;

// update the state of the sensor
void AP_Proximity_TopazB::update(void)
{
    if (_uart == nullptr) {
        return;
    }

    // process incoming messages
    if (read_sensor_data()) {
        _last_distance_received_ms = AP_HAL::millis();
    }

    // check for timeout and set health status
    if ((_last_distance_received_ms == 0) ||
        (AP_HAL::millis() - _last_distance_received_ms > PROXIMITY_TOPAZB_TIMEOUT_MS)) {
        set_status(AP_Proximity::Status::NoData);
    } else {
        set_status(AP_Proximity::Status::Good);
    }
}

// read and parse incoming serial data, returns true when a message is successfully parsed
bool AP_Proximity_TopazB::read_sensor_data()
{
    if (_uart == nullptr) {
        return false;
    }

    uint16_t nbytes = _uart->available();
    bool message_parsed = false;

    while (nbytes-- > 0) {
        const uint8_t byte = _uart->read();

        switch (_parse_state) {
        case WAIT_HEADER1:
            // look for first header byte (0xFA for both data and error packets)
            if (byte == PROXIMITY_TOPAZB_HEADER1) {
                _buffer[0] = byte;
                _buffer_count = 1;
                _parse_state = WAIT_HEADER2;
            }
            break;

        case WAIT_HEADER2:
            // check second header byte to determine packet type
            if (byte == PROXIMITY_TOPAZB_HEADER2) {
                // data packet header 0x05FA
                _buffer[1] = byte;
                _buffer_count = 2;
                _is_error_packet = false;
                _parse_state = WAIT_PAYLOAD;
            } else if (byte == PROXIMITY_TOPAZB_ERROR_HEADER2) {
                // error packet header 0xEEFA
                _buffer[1] = byte;
                _buffer_count = 2;
                _is_error_packet = true;
                _parse_state = WAIT_PAYLOAD;
            } else {
                // invalid header, reset
                _parse_state = WAIT_HEADER1;
            }
            break;

        case WAIT_PAYLOAD:
            // accumulate payload bytes
            _buffer[_buffer_count++] = byte;

            // check if we have a complete packet
            if (_buffer_count >= PROXIMITY_TOPAZB_PACKET_SIZE) {
                // verify CRC (calculated over first 102 bytes, CRC is last 2 bytes)
                const uint16_t calc_crc = crc16_ccitt(_buffer, PROXIMITY_TOPAZB_PACKET_SIZE - 2, 0xFFFF);
                const uint16_t recv_crc = UINT16_VALUE(_buffer[PROXIMITY_TOPAZB_PACKET_SIZE - 1],
                                                    _buffer[PROXIMITY_TOPAZB_PACKET_SIZE - 2]);

                if (calc_crc == recv_crc) {
                    if (!_is_error_packet) {
                        // valid data packet - process measurements
                        process_packet();
                        message_parsed = true;
                    }
                    // error packets are acknowledged but not processed further
                }

                // reset parser for next packet
                _buffer_count = 0;
                _parse_state = WAIT_HEADER1;
            }
            break;
        }
    }

    return message_parsed;
}

// process a complete valid data packet
void AP_Proximity_TopazB::process_packet()
{
    // extract start angle (bytes 2-3, little-endian, value/100 = degrees)
    const uint16_t start_angle_raw = UINT16_VALUE(_buffer[3], _buffer[2]);
    const float start_angle_deg = start_angle_raw * 0.01f;

    // extract scan rate (bytes 4-5, little-endian)
    // RPS (Rotations Per Second) = scan_rate / 480
    const uint16_t scan_rate = UINT16_VALUE(_buffer[5], _buffer[4]);

    float angle_step_deg;
    if (scan_rate > 0) {
        // angular resolution:
        // 0.167° @ 10 Hz
        // 0.25° @ 15 Hz
        // 0.33° @ 20 Hz
        // seems like RPS/60 gives as angular resolution
        // 1/(480*60) = 0.0000347222
        angle_step_deg = scan_rate * 0.0000347222f;
    } else {
        // fallback to reasonable default (~10Hz = 4800 scan_rate)
        angle_step_deg = 0.167f;  // 6° / 32 points
    }

    float angle_deg = start_angle_deg;

    // data starts at byte 6, each measurement is 3 bytes (2 distance + 1 energy)
    // 32 measurements total
    for (uint8_t i = 0; i < PROXIMITY_TOPAZB_PACKAGE_POINTS; i++) {
        const uint8_t offset = 6 + (i * 3);
        const uint16_t distance_mm = UINT16_VALUE(_buffer[offset + 1], _buffer[offset]);
        // energy byte at _buffer[offset + 2] is available but not used for proximity

        // skip invalid readings
        if (distance_mm < PROXIMITY_TOPAZB_DIST_SENSOR_ERROR) {
            update_sector_data(angle_deg, distance_mm);
        }
        angle_deg += angle_step_deg;
    }
}

// update sector data with a single measurement, angle in [0, 360]
void AP_Proximity_TopazB::update_sector_data(float angle_deg, uint16_t distance_mm)
{
    const uint16_t min_dist_mm = distance_min() * 1000;
    const uint16_t max_dist_mm = distance_max() * 1000;

    // convert angles from [0, 360] to [-22.5, 337.5] for sector calculation
    if (angle_deg > 337.5f) {
        angle_deg -= 360.0f;
    }

    // determine which of 8 sectors (30° each) this angle falls into
    uint8_t sector = UINT8_MAX;
    for (uint8_t i = 0; i < 8; i++) {
        const float sector_min = -15.0f + (i * 45.0f);
        const float sector_max = 15.0f + (i * 45.0f);
        if ((angle_deg >= sector_min) && (angle_deg < sector_max)) {
            sector = i;
            break;
        }
    }

    // skip if angle falls between sectors
    if (sector == UINT8_MAX) {
        return;
    }

    // initialize tracking on first call
    if (_last_sector == UINT8_MAX) {
        _last_sector = sector;
        _shortest_distance_mm = max_dist_mm;
        _invalidate_sector = false;
    }

    if (sector == _last_sector) {
        // same sector - track shortest valid distance
        if (_invalidate_sector) {
            return;
        } else if (ignore_reading(angle_deg, distance_mm * 0.001f, false)) {
            // invalidate all reading from this sector when a ground reading is received
            _invalidate_sector = true;
        } if (distance_mm > min_dist_mm && distance_mm < _shortest_distance_mm) {
            _shortest_distance_mm = distance_mm;
        }
    } else {
        // sector changed - push the accumulated shortest distance for previous sector
        const float sector_angle_deg = _last_sector * 45.0f;
        const AP_Proximity_Boundary_3D::Face face = frontend.boundary.get_face(sector_angle_deg);

        if (_invalidate_sector) {
            _invalidate_sector = false;
            frontend.boundary.reset_face(face, state.instance);
        } else if (_shortest_distance_mm > min_dist_mm && _shortest_distance_mm < max_dist_mm &&
            !ignore_reading(sector_angle_deg, distance_mm * 0.001f, false))
        {
            // convert mm to meters and update boundary
            frontend.boundary.set_face_attributes(face, sector_angle_deg,
                                                  _shortest_distance_mm * 0.001f,
                                                  state.instance);
            // update obstacle avoidance database
            database_push(sector_angle_deg, _shortest_distance_mm * 0.001f);
        } else {
            frontend.boundary.reset_face(face, state.instance);
        }

        // reset for new sector
        _last_sector = sector;
        _shortest_distance_mm = max_dist_mm;
    }
}

#endif // AP_PROXIMITY_TOPAZB_ENABLED
