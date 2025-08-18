#include "AP_Mount_Skydroid_C13.h"

#if HAL_MOUNT_SKYDROID_C13_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Follow/AP_Follow_Mount.h>

extern const AP_HAL::HAL& hal;
const AP_Follow_Mount &follow = AP::follow_mount();

#ifdef AP_MOUNT_C13_DEBUG
#define debug(fmt, args ...)  do {hal.console->printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#else
#define debug(fmt, args ...)
#endif

#define SKYDROID_SERIAL_SPEED 115200

// init - performs any required initialisation for this instance
void AP_Mount_Skydroid_C13::init()
{
    const AP_SerialManager& serial_manager = AP::serialmanager();

    _uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Gimbal, 0);
    if (_uart != nullptr) {
        _uart->begin(SKYDROID_SERIAL_SPEED);
        _initialised = true;
        set_mode((enum MAV_MOUNT_MODE)_params.default_mode.get());
    }

    // Initialize parser state
    _parsed_msg.state = ParseState::WAITING_FOR_HEADER1;
    _msg_buff_len = 0;
    _parsed_msg.data_bytes_received = 0;
    _tracking_active = false;
    _tracking_confidence = 0;

    AP_Mount_Backend::init();
    debug("Hello C13");
}

// update mount position - should be called periodically
void AP_Mount_Skydroid_C13::update_fast()
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    // read incoming packets from gimbal
    read_incoming_packets();
}

// return true if healthy - don't use timeout since no data when not tracking is normal
bool AP_Mount_Skydroid_C13::healthy() const
{
    return _initialised;  // Always healthy if initialized, regardless of data flow
}

// get attitude as a quaternion
bool AP_Mount_Skydroid_C13::get_attitude_quaternion(Quaternion& att_quat)
{
    att_quat.from_euler(_current_angle_rad.x, _current_angle_rad.y, _current_angle_rad.z);
    return true;
}

// reading incoming packets from gimbal - similar to G3P driver style
void AP_Mount_Skydroid_C13::read_incoming_packets()
{
    // check for bytes on the serial port
    int16_t nbytes = MIN(_uart->available(), 1024U);
    if (nbytes <= 0) {
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
        if (_msg_buff_len >= AP_MOUNT_SKYDROID_PACKETLEN_MAX) {
            reset_parser = true;
        }

        // process byte depending upon current state
        switch (_parsed_msg.state) {

        case ParseState::WAITING_FOR_HEADER1:
            if (b == AP_MOUNT_SKYDROID_HEADER1) {
                _parsed_msg.state = ParseState::WAITING_FOR_HEADER2;
            } else {
                debug("Skydroid: Header1 mismatch");
                reset_parser = true;
            }
            break;

        case ParseState::WAITING_FOR_HEADER2:
            if (b == AP_MOUNT_SKYDROID_HEADER2) {
                _parsed_msg.state = ParseState::WAITING_FOR_LENGTH_LOW;
            } else {
                debug("Skydroid: Header2 mismatch");
                reset_parser = true;
            }
            break;

        case ParseState::WAITING_FOR_LENGTH_LOW:
            _parsed_msg.data_len = b;  // Low byte of length
            _parsed_msg.state = ParseState::WAITING_FOR_LENGTH_HIGH;
            break;

        case ParseState::WAITING_FOR_LENGTH_HIGH:
            _parsed_msg.data_len |= (b << 8);  // High byte of length
            if (_parsed_msg.data_len > AP_MOUNT_SKYDROID_DATALEN_MAX) {
                reset_parser = true;
            } else {
                _parsed_msg.state = ParseState::WAITING_FOR_SEQUENCE_LOW;
            }
            break;

        case ParseState::WAITING_FOR_SEQUENCE_LOW:
            _parsed_msg.sequence = b;  // Low byte of sequence
            _parsed_msg.state = ParseState::WAITING_FOR_SEQUENCE_HIGH;
            break;

        case ParseState::WAITING_FOR_SEQUENCE_HIGH:
            _parsed_msg.sequence |= (b << 8);  // High byte of sequence
            _parsed_msg.state = ParseState::WAITING_FOR_CONTROL;
            break;

        case ParseState::WAITING_FOR_CONTROL:
            _parsed_msg.command_id = b;
            if (_parsed_msg.data_len == 0) {
                _parsed_msg.state = ParseState::WAITING_FOR_CRC_LOW;
            } else {
                _parsed_msg.state = ParseState::WAITING_FOR_DATA;
            }
            break;

        case ParseState::WAITING_FOR_DATA:
            _parsed_msg.data_bytes_received++;
            if (_parsed_msg.data_bytes_received >= _parsed_msg.data_len) {
                _parsed_msg.state = ParseState::WAITING_FOR_CRC_LOW;
            }
            break;

        case ParseState::WAITING_FOR_CRC_LOW:
            _parsed_msg.crc_low = b;
            _parsed_msg.state = ParseState::WAITING_FOR_CRC_HIGH;
            break;

        case ParseState::WAITING_FOR_CRC_HIGH:
            _parsed_msg.crc_high = b;
            _parsed_msg.state = ParseState::WAITING_FOR_TAIL;
            break;

        case ParseState::WAITING_FOR_TAIL:
            if (b == AP_MOUNT_SKYDROID_TAIL) {
                // check crc
                uint16_t expected_crc = crc16_ccitt(_msg_buff, _msg_buff_len - 3, 0);
                uint16_t received_crc = _parsed_msg.crc_low | (_parsed_msg.crc_high << 8);
                
                if (expected_crc == received_crc) {
                    // successfully received a message, process it
                    process_packet();
                } else {
                    debug("Skydroid: CRC error expected:%x got:%x", (unsigned)expected_crc, (unsigned)received_crc);
                }
            }
            reset_parser = true;
            break;
        }

        // handle reset of parser
        if (reset_parser) {
            _parsed_msg.state = ParseState::WAITING_FOR_HEADER1;
            _msg_buff_len = 0;
            _parsed_msg.data_bytes_received = 0;
            reset_parser = false;
        }
    }
}

// process successfully decoded packets - switch case style like G3P driver
void AP_Mount_Skydroid_C13::process_packet()
{
    // process packet depending upon command id
    switch (_parsed_msg.command_id) {

    case AP_MOUNT_SKYDROID_CMD_START_CANCEL_TRACKING: {
        if (_parsed_msg.data_len >= sizeof(TrackingDataToFC)) {
            // This is tracking data from gimbal to flight controller
            const TrackingDataToFC* tracking_data = (const TrackingDataToFC*)&_msg_buff[AP_MOUNT_SKYDROID_MSG_BUF_DATA_START];
            
            // Update gimbal angles
            _current_angle_rad.x = radians(tracking_data->gimbal_roll * 0.01f);   // roll
            _current_angle_rad.y = radians(tracking_data->gimbal_pitch * 0.01f);  // pitch  
            _current_angle_rad.z = radians(tracking_data->gimbal_yaw * 0.01f);    // yaw
            _last_current_angle_rad_ms = AP_HAL::millis();
            
            // Update tracking status
            _tracking_active = (tracking_data->status == 1);  // 1 = normal tracking
            _tracking_confidence = tracking_data->confidence;
            _last_tracking_data_ms = AP_HAL::millis();
            
            if (_tracking_active) {
                // Calculate target position in NED coordinates
                float h_offset_rad = radians(tracking_data->offset_h * 0.01f);
                float v_offset_rad = radians(tracking_data->offset_v * 0.01f);
                float distance_m = tracking_data->laser_distance * 0.1f;
                
                // Target bearing relative to gimbal
                float target_bearing = -_current_angle_rad.z - h_offset_rad;
                float target_elevation = _current_angle_rad.y + v_offset_rad;
                
                // Convert to NED offset from drone
                _target_ned_offset_m.x = distance_m * cos(target_elevation) * cos(target_bearing);  // North
                _target_ned_offset_m.y = distance_m * cos(target_elevation) * sin(target_bearing);  // East
                _target_ned_offset_m.z = -distance_m * sin(target_elevation);                       // Down
                
                // Update follow struct
                AP_Follow_Mount::MountTracking tracking;
                tracking.last_update_ms = _last_tracking_data_ms;
                tracking.yaw_error_rad = target_bearing;
                tracking.pitch_error_rad = target_elevation;
                tracking.crop_width = tracking_data->target_width;
                tracking.crop_height = tracking_data->target_height;
                tracking.distance_cm = tracking_data->laser_distance * 10;
                if (!follow.set_mount_tracking(tracking)) {
                    debug("Error set_mount_tracking");
                }
                
#ifdef AP_MOUNT_C13_DEBUG
                // Send tracking info to GCS every 5 seconds
                static uint32_t last_tracking_report_ms = 0;
                if (AP_HAL::millis() - last_tracking_report_ms >= 5000) {
                    debug("Tracking: dist %.1fm, bearing %.0f°, elevation%.0f°, width %u, height %u, conf %u%%",
                              distance_m, degrees(target_bearing), degrees(target_elevation),
                              tracking_data->target_width, tracking_data->target_height, tracking_data->confidence);
                    last_tracking_report_ms = AP_HAL::millis();
                }
#endif
                
            } else if (tracking_data->status == 3) {
                // Tracking error/lost
                if (!follow.set_is_tracking(false)) {
                    debug("Error set_mount_tracking");
                }
                debug("Skydroid: Target lost");
                _tracking_active = false;
            }
        } else {
            debug("Skydroid: Tracking data too short: %u bytes", (unsigned)_parsed_msg.data_len);
        }
        break;
    }

    case AP_MOUNT_SKYDROID_CMD_GIMBAL_ANGLE_OUTPUT: {
        if (_parsed_msg.data_len >= 6) {
            // Simple gimbal angle data (Roll, Pitch, Yaw as int16_t * 100)
            int16_t roll = _msg_buff[AP_MOUNT_SKYDROID_MSG_BUF_DATA_START] | 
                          (_msg_buff[AP_MOUNT_SKYDROID_MSG_BUF_DATA_START+1] << 8);
            int16_t pitch = _msg_buff[AP_MOUNT_SKYDROID_MSG_BUF_DATA_START+2] | 
                           (_msg_buff[AP_MOUNT_SKYDROID_MSG_BUF_DATA_START+3] << 8);
            int16_t yaw = _msg_buff[AP_MOUNT_SKYDROID_MSG_BUF_DATA_START+4] | 
                         (_msg_buff[AP_MOUNT_SKYDROID_MSG_BUF_DATA_START+5] << 8);
            
            _current_angle_rad.x = radians(roll * 0.01f);
            _current_angle_rad.y = radians(pitch * 0.01f);
            _current_angle_rad.z = radians(yaw * 0.01f);
            _last_current_angle_rad_ms = AP_HAL::millis();
        }
        break;
    }

    default:
        // Unknown packet type - ignore
        break;
    }
}

#endif // HAL_MOUNT_SKYDROID_ENABLED
