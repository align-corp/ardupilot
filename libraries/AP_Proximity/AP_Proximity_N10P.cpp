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

#if AP_PROXIMITY_N10P_ENABLED

#include "AP_Proximity_N10P.h"
#include <AP_HAL/AP_HAL.h>
#include <ctype.h>
#include <stdio.h>

#ifdef N10P_DEBUGGING
#define Debug(fmt, args ...)  do { printf(fmt, ## args); } while (0)
#else
#define Debug(fmt, args ...)
#endif

extern const AP_HAL::HAL& hal;

// update the state of the sensor
void AP_Proximity_N10P::update(void)
{
    if (_uart == nullptr) {
        return;
    }

    // process incoming messages
    read_sensor_data();

    // check for timeout and set health status
    if ((_last_distance_received_ms == 0) || (AP_HAL::millis() - _last_distance_received_ms > PROXIMITY_N10P_TIMEOUT_MS)) {
        set_status(AP_Proximity::Status::NoData);
    } else {
        set_status(AP_Proximity::Status::Good);
    }
}

// check for replies from sensor, return true when a message is successfully parsed
bool AP_Proximity_N10P::read_sensor_data()
{
    if (_uart == nullptr) {
        return false;
    }

    uint16_t nbytes = _uart->available();
    bool message_parsed = false;

    while (nbytes-- > 0) {
        // read byte
        uint8_t byte = _uart->read();

        // prevent array out of range
        if (_buffer_count == PROXIMITY_N10P_PACKET_SIZE) {
            _buffer_count = 0;
        }

        // append to buffer
        _buffer[_buffer_count++] = byte;

        switch (_parse_state)
        {
        case WAIT_HEADER1:
            if (byte == PROXIMITY_N10P_HEADER1 ) {
                _buffer[0] = byte;
                _buffer_count = 1;
                _parse_state = WAIT_HEADER2;
            }
            break;
        case WAIT_HEADER2:
            if (byte == PROXIMITY_N10P_HEADER2 ) {
                _parse_state = WAIT_LENGTH;
            } else {
                _parse_state = WAIT_HEADER1;
                Debug("Wrong header 2: %d\n", byte);
            }
            break;
        case WAIT_LENGTH:
            if (byte == PROXIMITY_N10P_LENGTH ) {
                _parse_state = WAIT_CHECKSUM;
            } else {
                _parse_state = WAIT_HEADER1;
                Debug("Wrong length: %d\n", byte);
            }
            break;
        case WAIT_CHECKSUM:
            if (_buffer_count == PROXIMITY_N10P_PACKET_SIZE){
                _buffer_count = 0;
                //check CRC and update measure
                if (crc8_N10(_buffer, PROXIMITY_N10P_PACKET_SIZE-1) == _buffer[PROXIMITY_N10P_PACKET_SIZE-1]){
                    // decode message
                    float angle_start = UINT16_VALUE(_buffer[5],  _buffer[6]) * 0.01f;
                    float angle_stop = UINT16_VALUE(_buffer[105],  _buffer[106]) * 0.01f;
                    float angle_step = (angle_stop - angle_start > 0) ?  (angle_stop - angle_start) / PROXIMITY_N10P_PACKAGE_POINTS : (angle_stop+360 - angle_start) / PROXIMITY_N10P_PACKAGE_POINTS;
                    float angle = angle_start;

                    // Calculate and update distances
                    for (uint8_t i = 7; i < 102; i=i+3) {
                        uint16_t distance = UINT16_VALUE(_buffer[i],  _buffer[i+1]);
                        angle = (angle > 360) ? (angle - 360) : angle;
                        update_sector_data(angle, distance);
                        angle = angle + angle_step;
                    }
                    message_parsed = true;
                } else {
                    // wrong crc
                    _parse_state = WAIT_HEADER1;
                    Debug("Wrong checksum: %d\n", _buffer[PROXIMITY_N10P_PACKET_SIZE-1]);
                }
            }
            break;

        default:
            break;
        }
    }
    return message_parsed; 
}

// process reply
void AP_Proximity_N10P::update_sector_data(float angle_deg, uint16_t distance_mm)
{
    // convert angles from [0 360] to [-22.5 337.5]
    if (angle_deg > 337.5) {
        angle_deg -= 360;
    }
    uint8_t sector = UINT8_MAX;

    // understand in which sector we are
    for (uint8_t i=0; i<8; i++) {
        if ((angle_deg > -22 + i*45) && (angle_deg < 22 + i*45)) {
            sector = i;
        }
    }

    // abort if the angle is in the middle of two sectors
    if (sector == UINT8_MAX) {
        return;
    }

    // static variable for last sector sent
    static uint8_t last_sector = sector;
    static uint16_t shortest_distance_mm = distance_max()*1000;
    static uint16_t count = 0;

    if (sector == last_sector ) {
        // find shortest distance
        count++;
        if (distance_mm < shortest_distance_mm && distance_mm > distance_min()*1000) {
            shortest_distance_mm = distance_mm;
        }
    } else {
        //Just consider the shortest distance (skip the 3 shortest distance for filtering)
        float angle_deg_sector = last_sector * 45.0f;

        // Get location on 3-D boundary based on angle to the object
        const AP_Proximity_Boundary_3D::Face face = frontend.boundary.get_face(angle_deg_sector);
        //check for target too far, target too close and sensor not connected
        const bool valid = (shortest_distance_mm < distance_max()*1000) && (shortest_distance_mm > distance_min()*1000);
        Debug("Shortest distance: %d, angle: %f, count: %d\n", shortest_distance_mm, angle_deg_sector, count);
        count = 0;
        if (valid && !ignore_reading(angle_deg_sector, shortest_distance_mm * 0.001f, false)) {
            frontend.boundary.set_face_attributes(face, angle_deg_sector, shortest_distance_mm * 0.001f, state.instance);
            // update OA database
            database_push(angle_deg_sector, shortest_distance_mm * 0.001f);
        } else {
            frontend.boundary.reset_face(face, state.instance);
        }
        _last_distance_received_ms = AP_HAL::millis();

        // reset shortest_distance_mm, consider also the first new sector distance
        if (distance_mm > distance_min()*1000) {
            shortest_distance_mm = distance_mm;
        } else {
            shortest_distance_mm = distance_max()*1000;
        }

        // reset last_sector
        last_sector = sector;
    }
}

uint8_t AP_Proximity_N10P::crc8_N10(const uint8_t *p, uint8_t len)
	{
		uint16_t sum = 0;
		for (int i = 0; i < len; i++) {
			sum += p[i];
        }
		uint8_t crc = sum & 0xff;
		return crc;
	}

#endif // AP_PROXIMITY_N10P_ENABLED
