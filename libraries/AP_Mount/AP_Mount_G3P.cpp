#include "AP_Mount_G3P.h"

#if HAL_MOUNT_G3P_ENABLED
#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_GPS/AP_GPS.h>
 
#ifdef AP_MOUNT_G3P_DEBUG
#include <stdio.h>
#endif

// update rate
#define AP_MOUNT_G3P_REQUEST_ATTITUDE_MS 50
#define AP_MOUNT_G3P_SEND_LOCATION_MS 100


#define BYTE1(i) ((uint8_t)(i))
#define BYTE2(i) ((uint8_t)(i>>8))
#define BYTE3(i) ((uint8_t)(i>>16))
#define BYTE4(i) ((uint8_t)(i>>24))

// Align location
#define ALIGN_LAT 242554040     // degE7
#define ALIGN_LON 1207353010    // degE7
#define ALIGN_ALT 12345         // cm

extern const AP_HAL::HAL& hal;
const AP_GPS &gps = AP::gps();

#ifdef AP_MOUNT_G3P_DEBUG
#define debug(fmt, args ...)  do {hal.console->printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#else
#define debug(fmt, args ...)
#endif


// init - performs any required initialisation for this instance
void AP_Mount_G3P::init()
{
    const AP_SerialManager& serial_manager = AP::serialmanager();

    _uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Gimbal, 0);
    _uart_dv = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Gimbal, 1);
    if (_uart != nullptr) {
        _uart->begin((uint32_t)120000);
        _initialised = true;
        set_mode((enum MAV_MOUNT_MODE)_params.default_mode.get());
    }

    if (_uart_dv != nullptr) {
        _uart_dv->begin((uint32_t)115200);
    }

    AP_Mount_Backend::init();
}

// update mount position - should be called periodically
void AP_Mount_G3P::update()
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    // reading incoming packets from gimbal
    read_incoming_packets();

    uint32_t now_ms = AP_HAL::millis();
    
    // request attitude at 20 Hz
    if ((now_ms - _last_req_current_angle_rad_ms) >= AP_MOUNT_G3P_REQUEST_ATTITUDE_MS) {
        request_gimbal_attitude();
        _last_req_current_angle_rad_ms = now_ms;
    }

    // send GNSS coordinates to DV at 10 Hz (also move the phase to avoid UART noise)
    if (_uart_dv != nullptr && ((now_ms + (AP_MOUNT_G3P_REQUEST_ATTITUDE_MS / 2) - _last_coordinates_send_ms)) >= AP_MOUNT_G3P_SEND_LOCATION_MS) {
        send_gps_position();
        _last_coordinates_send_ms = now_ms;
    }

    // update based on mount mode
    switch (get_mode()) {

        // move mount to a "retracted" position.
        case MAV_MOUNT_MODE_RETRACT: {
            const Vector3f &angle_bf_target = _params.retract_angles.get();
            mnt_target.target_type = MountTargetType::ANGLE;
            mnt_target.angle_rad.set(angle_bf_target*DEG_TO_RAD, false);
            break;
        }

        // move mount to a neutral position, typically pointing forward
        case MAV_MOUNT_MODE_NEUTRAL: {
            const Vector3f &angle_bf_target = _params.neutral_angles.get();
            mnt_target.target_type = MountTargetType::ANGLE;
            mnt_target.angle_rad.set(angle_bf_target*DEG_TO_RAD, false);
            break;
        }

        case MAV_MOUNT_MODE_MAVLINK_TARGETING: {
            // mavlink targets are stored while handling the incoming message set_angle_target() or set_rate_target()
            break;
        }

        // RC radio manual angle control, but with stabilization from the AHRS
        case MAV_MOUNT_MODE_RC_TARGETING: {
            // update targets using pilot's RC inputs
            MountTarget rc_target;
            get_rc_target(mnt_target.target_type, rc_target);
            switch (mnt_target.target_type) {
            case MountTargetType::ANGLE:
                mnt_target.angle_rad = rc_target;
                break;
            case MountTargetType::RATE:
                mnt_target.rate_rads = rc_target;
                break;
            }
            break;
        }

        // point mount to a GPS point given by the mission planner
        case MAV_MOUNT_MODE_GPS_POINT:
            if (get_angle_target_to_roi(mnt_target.angle_rad)) {
                mnt_target.target_type = MountTargetType::ANGLE;
            }
            break;

        // point mount to Home location
        case MAV_MOUNT_MODE_HOME_LOCATION:
            if (get_angle_target_to_home(mnt_target.angle_rad)) {
                mnt_target.target_type = MountTargetType::ANGLE;
            }
            break;

        // point mount to another vehicle
        case MAV_MOUNT_MODE_SYSID_TARGET:
            if (get_angle_target_to_sysid(mnt_target.angle_rad)) {
                mnt_target.target_type = MountTargetType::ANGLE;
            }
            break;

        default:
            // unknown mode so do nothing
            break;
    }

    // send target angles or rates depending on the target type
    switch (mnt_target.target_type) {
        case MountTargetType::ANGLE:
            send_target_angles(mnt_target.angle_rad.pitch, mnt_target.angle_rad.yaw, mnt_target.angle_rad.yaw_is_ef);
            break;
        case MountTargetType::RATE:
            send_target_rates(mnt_target.rate_rads.pitch, mnt_target.rate_rads.roll, mnt_target.rate_rads.yaw);
            break;
    }

}

// return true if healthy
bool AP_Mount_G3P::healthy() const
{
    // unhealthy until gimbal has been found and replied with firmware version info
    if (!_initialised) {
        return false;
    }

    // unhealthy if attitude information NOT received recently
    if (AP_HAL::millis() - _last_current_angle_rad_ms > 1000) {
        return false;
    }

    // if we get this far return healthy
    return true;
}

// get attitude as a quaternion.  returns true on success
bool AP_Mount_G3P::get_attitude_quaternion(Quaternion& att_quat)
{
    att_quat.from_euler(_current_angle_rad.x, _current_angle_rad.y, _current_angle_rad.z);
    return true;
}

// reading incoming packets from gimbal and confirm they are of the correct format
// results are held in the _parsed_msg structure
void AP_Mount_G3P::read_incoming_packets()
{
    // check for bytes on the serial port
    int16_t nbytes = MIN(_uart->available(), 1024U);
    if (nbytes <= 0 ) {
        return;
    }

    // flag to allow cases below to reset parser state
    bool reset_parser = false;

    // process bytes received
    for (int16_t i = 0; i < nbytes; i++) {
        const int16_t b = _uart->read();

        // sanity check byte
        if ((b < 0) || (b > 0xFF)) {
            continue;
        }

        _msg_buff[_msg_buff_len++] = b;

        // protect against overly long messages
        if (_msg_buff_len >= AP_MOUNT_G3P_PACKETLEN_MAX) {
            reset_parser = true;
        }

        // process byte depending upon current state
        switch (_parsed_msg.state) {

        case ParseStateGimbal::WAITING_FOR_HEADER:
            if (b == AP_MOUNT_G3P_HEADER_RECEIVE) {
                _parsed_msg.state = ParseStateGimbal::WAITING_FOR_COMMAND;
            } else {
                reset_parser = true;
            }
            break;

        case ParseStateGimbal::WAITING_FOR_COMMAND:
            _parsed_msg.command_id = b;
            _parsed_msg.state = ParseStateGimbal::WAITING_FOR_LENGTH;
            break;

        case ParseStateGimbal::WAITING_FOR_LENGTH:
            _parsed_msg.data_len = b;
            if (_parsed_msg.data_len == 0) {
                _parsed_msg.state = ParseStateGimbal::WAITING_FOR_CRC_A;
            } else if (_parsed_msg.data_len <= AP_MOUNT_G3P_DATALEN_MAX) {
                _parsed_msg.state = ParseStateGimbal::WAITING_FOR_DATA;
            } else {
                reset_parser = true;
                debug("data len too large:%u (>%u)", (unsigned)_parsed_msg.data_len, (unsigned)AP_MOUNT_G3P_DATALEN_MAX);
            }
            break;

        case ParseStateGimbal::WAITING_FOR_DATA:
            _parsed_msg.data_bytes_received++;
            if (_parsed_msg.data_bytes_received >= _parsed_msg.data_len) {
                _parsed_msg.state = ParseStateGimbal::WAITING_FOR_CRC_A;
            }
            break;

        case ParseStateGimbal::WAITING_FOR_CRC_A:
            _parsed_msg.ck_a = b;
            _parsed_msg.state = ParseStateGimbal::WAITING_FOR_CRC_B;
            break;

        case ParseStateGimbal::WAITING_FOR_CRC_B:
            _parsed_msg.ck_b = b;

            // check crc
            uint8_t expected_ck_a;
            uint8_t expected_ck_b;
            crc16_tcp(_msg_buff, _msg_buff_len-2, 1, expected_ck_a, expected_ck_b);
            if (expected_ck_a == _parsed_msg.ck_a && expected_ck_b == _parsed_msg.ck_b) {
                // successfully received a message, do something with it
                process_packet();
            } else {
#ifdef AP_MOUNT_G3P_DEBUG
                char hexString[_msg_buff_len*3+1];
                for (uint8_t j = 0; j < _msg_buff_len; ++j) {
                    snprintf(hexString + (j * 3), 4, "%02X ", _msg_buff[j]);
                }
                debug("ck_a and ck_b expected:%x %x got:%x %x \narray: %s", expected_ck_a, expected_ck_b, _parsed_msg.ck_a, _parsed_msg.ck_b, hexString);
#endif
            }
            reset_parser = true;
            break;
        }

        // handle reset of parser
        if (reset_parser) {
            _parsed_msg.state = ParseStateGimbal::WAITING_FOR_HEADER;
            _msg_buff_len = 0;
            _parsed_msg.data_bytes_received = 0;
        }
    }
}

// process successfully decoded packets held in the _parsed_msg structure
void AP_Mount_G3P::process_packet()
{
#ifdef AP_MOUNT_G3P_DEBUG
    // flag to warn of unexpected data buffer length
    bool unexpected_len = false;
#endif

    // process packet depending upon command id
    switch (_parsed_msg.command_id) {

    case AP_MOUNT_G3P_CMD_ANGLE_REQUEST: {
        if (_parsed_msg.data_bytes_received != AP_MOUNT_G3P_LENGTH_ANGLE_REQUEST) {
#ifdef AP_MOUNT_G3P_DEBUG
            unexpected_len = true;
#endif
            break;
        }
        _last_current_angle_rad_ms = AP_HAL::millis();
        _current_angle_rad.z = -radians((int16_t)UINT16_VALUE(_msg_buff[AP_MOUNT_G3P_MSG_BUF_DATA_START], _msg_buff[AP_MOUNT_G3P_MSG_BUF_DATA_START+1])/182.0444);   // yaw angle
        _current_angle_rad.x = radians((int16_t)UINT16_VALUE(_msg_buff[AP_MOUNT_G3P_MSG_BUF_DATA_START+2], _msg_buff[AP_MOUNT_G3P_MSG_BUF_DATA_START+3])/182.0444);  // roll angle
        _current_angle_rad.y = radians((int16_t)UINT16_VALUE(_msg_buff[AP_MOUNT_G3P_MSG_BUF_DATA_START+4], _msg_buff[AP_MOUNT_G3P_MSG_BUF_DATA_START+5])/182.0444);  // pitch angle
        break;
    }

    default:
#ifdef AP_MOUNT_G3P_DEBUG
        debug("Unhandled CmdId:%u", (unsigned)_parsed_msg.command_id);
#endif
        break;
    }

#ifdef AP_MOUNT_G3P_DEBUG
    // handle unexpected data buffer length
    if (unexpected_len) {
        debug("CmdId:%u unexpected len:%u", (unsigned)_parsed_msg.command_id, (unsigned)_parsed_msg.data_bytes_received);
    }
#endif
}

// methods to send commands to gimbal
// returns true on success, false if outgoing serial buffer is full
bool AP_Mount_G3P::send_packet_gimbal(uint8_t cmd_id, const uint8_t* databuff, uint8_t databuff_len)
{
    if (!_initialised) {
        return false;
    }
    // calculate and sanity check packet size
    const uint16_t packet_size = AP_MOUNT_G3P_PACKETLEN_MIN + databuff_len;

#ifdef AP_MOUNT_G3P_DEBUG
    if (packet_size > AP_MOUNT_G3P_PACKETLEN_MAX) {
        debug("send_packet data buff too large");
        return false;
    }
#endif

    // check for sufficient space in outgoing buffer
    if (_uart->txspace() < packet_size) {
        return false;
    }

    // buffer for holding outgoing packet
    uint8_t send_buff[packet_size];
    uint8_t send_buff_ofs = 0;

    // packet header
    send_buff[send_buff_ofs++] = AP_MOUNT_G3P_HEADER_SEND;

    // CMD_ID
    send_buff[send_buff_ofs++] = cmd_id;

    // Data_len.
    send_buff[send_buff_ofs++] = databuff_len;

    // DATA
    if (databuff_len != 0) {
        memcpy(&send_buff[send_buff_ofs], databuff, databuff_len);
        send_buff_ofs += databuff_len;
    }

    // CRC16 TCP
    uint8_t ck_a;
    uint8_t ck_b;
    crc16_tcp(send_buff, send_buff_ofs, 1, ck_a, ck_b);
    send_buff[send_buff_ofs++] = ck_a;
    send_buff[send_buff_ofs++] = ck_b;

    // send packet
    _uart->write(send_buff, send_buff_ofs);

    return true;

}

bool AP_Mount_G3P::send_packet_dv(uint8_t cmd_id1, uint8_t cmd_id2, uint8_t data1, uint8_t data2)
{
    if (!_initialised) {
        return false;
    }

    // check for sufficient space in outgoing buffer
    if (_uart->txspace() < AP_MOUNT_DV_PACKETLEN) {
        return false;
    }

    // buffer for holding outgoing packet
    uint8_t send_buff[AP_MOUNT_DV_PACKETLEN];
    uint8_t send_buff_ofs = 0;

    // packet header
    send_buff[send_buff_ofs++] = AP_MOUNT_DV_HEADER;
    send_buff[send_buff_ofs++] = AP_MOUNT_DV_HEADER2_SEND;

    // CMD_ID
    send_buff[send_buff_ofs++] = cmd_id1;
    send_buff[send_buff_ofs++] = cmd_id2;

    // DATA
    send_buff[send_buff_ofs++] = data1;
    send_buff[send_buff_ofs++] = data2;

    // CRC16
    const uint16_t crc = crc_sum16(send_buff, send_buff_ofs, 1);
    send_buff[send_buff_ofs++] = HIGHBYTE(crc);
    send_buff[send_buff_ofs++] = LOWBYTE(crc);

    // END
    send_buff[send_buff_ofs++] = AP_MOUNT_DV_END;

    // send packet
    _uart_dv->write(send_buff, send_buff_ofs);

    return true;
}

// send target pitch and yaw rates to gimbal
void AP_Mount_G3P::send_target_rates(float pitch_rads, float roll_rads, float yaw_rads)
{
    //TODO: constrain rate
    int16_t yaw_strange = -floorf(yaw_rads*10430.376f+0.5f);
    int16_t roll_strange = floorf(roll_rads*10430.376f+0.5f);
    int16_t pitch_strange = floorf(pitch_rads*10430.376f+0.5f);
    const uint8_t yaw_roll_pitch_rate[] = { HIGHBYTE(yaw_strange),
                                            LOWBYTE(yaw_strange),
                                            HIGHBYTE(roll_strange),
                                            LOWBYTE(roll_strange),
                                            HIGHBYTE(pitch_strange),
                                            LOWBYTE(pitch_strange) };
    send_packet_gimbal(AP_MOUNT_G3P_CMD_ANGLE_SET, yaw_roll_pitch_rate, ARRAY_SIZE(yaw_roll_pitch_rate));

}

// send target pitch and yaw angles to gimbal
// yaw_is_ef should be true if yaw_rad target is an earth frame angle, false if body_frame
void AP_Mount_G3P::send_target_angles(float pitch_rad, float yaw_rad, bool yaw_is_ef)
{
    // stop gimbal if no recent actual angles
    uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _last_current_angle_rad_ms >= 200) {
        send_target_rates(0, 0, 0);
        return;
    }

    // gimbal centering function
    if (fabsf(pitch_rad) < 0.01f && fabsf(yaw_rad) < 0.01f) {
        center_gimbal();
        set_mode(MAV_MOUNT_MODE_RC_TARGETING);
        return;
    }

    // use simple P controller to convert pitch angle error (in radians) to a target rate scalar (-100 to +100)
    const float pitch_err_rad = (pitch_rad - _current_angle_rad.y);
    float pitch_rate_rads = constrain_float(pitch_err_rad * AP_MOUNT_G3P_PITCH_P, -2.0f, 2.0f);

    // convert yaw angle to body-frame the use simple P controller to convert yaw angle error to a target rate scalar (-100 to +100)
    const float yaw_bf_rad = yaw_is_ef ? wrap_PI(yaw_rad - AP::ahrs().get_yaw()) : yaw_rad;
    const float yaw_err_rad = (yaw_bf_rad - _current_angle_rad.z);
    float yaw_rate_rads = constrain_float(yaw_err_rad * AP_MOUNT_G3P_YAW_P, -2.0f, 2.0f);

    // send calculated rates
    send_target_rates(pitch_rate_rads, 0, yaw_rate_rads);

    // switch back to rate mode when desired angle is reached to prevent 2 position controller interference
    if (fabsf(pitch_err_rad) < 0.007f && fabsf(yaw_err_rad) < 0.007f) {
        set_mode(MAV_MOUNT_MODE_RC_TARGETING);
    }
}

void AP_Mount_G3P::center_gimbal()
{
    uint8_t buffer[1] = {AP_MOUNT_G3P_DATA_CENTER};
    send_packet_gimbal(AP_MOUNT_G3P_CMD_CALIBRATE, buffer, 1);
}

void AP_Mount_G3P::send_gps_position()
{
    // set Align location as default
    Location here = Location(ALIGN_LAT, ALIGN_LON, ALIGN_ALT, Location::AltFrame::ABSOLUTE);
    uint8_t instance = gps.primary_sensor();
    int32_t alt_cm = ALIGN_ALT;
    if (gps.status(instance) >= AP_GPS::GPS_Status::GPS_OK_FIX_3D) {
        // gps location is good, use real coordinates
        here = gps.location(instance);
        if (!here.get_alt_cm(Location::AltFrame::ABSOLUTE, alt_cm)) {
            // alt not available
            alt_cm = 0;
        }
    }
    uint16_t alt_dm = constrain_uint16(alt_cm * 0.1, 0, 65535);
    send_packet_dv(AP_MOUNT_DV_CMD1, AP_MOUNT_DV_CMD2_LATH, BYTE4(here.lat), BYTE3(here.lat));
    send_packet_dv(AP_MOUNT_DV_CMD1, AP_MOUNT_DV_CMD2_LATL, BYTE2(here.lat), BYTE1(here.lat));
    send_packet_dv(AP_MOUNT_DV_CMD1, AP_MOUNT_DV_CMD2_LONH, BYTE4(here.lng), BYTE3(here.lng));
    send_packet_dv(AP_MOUNT_DV_CMD1, AP_MOUNT_DV_CMD2_LONL, BYTE2(here.lng), BYTE1(here.lng));
    send_packet_dv(AP_MOUNT_DV_CMD1, AP_MOUNT_DV_CMD2_ALT, HIGHBYTE(alt_dm), LOWBYTE(alt_dm));
}

#endif // HAL_MOUNT_G3P_ENABLED
