#pragma once

#include "AP_Mount_Backend.h"

#if HAL_MOUNT_SKYDROID_C13_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>

// Skydroid protocol constants
#define AP_MOUNT_SKYDROID_HEADER1 0xAA
#define AP_MOUNT_SKYDROID_HEADER2 0xA5
#define AP_MOUNT_SKYDROID_TAIL 0xCD
#define AP_MOUNT_SKYDROID_PACKETLEN_MAX 52  // max packet size
#define AP_MOUNT_SKYDROID_PACKETLEN_MIN 10  // minimum packet size
#define AP_MOUNT_SKYDROID_DATALEN_MAX 42  // max data payload
#define AP_MOUNT_SKYDROID_MSG_BUF_DATA_START 7 // data starts at this byte in _msg_buff

// Function codes
#define AP_MOUNT_SKYDROID_CMD_SET_RECOGNITION_AREA 1
#define AP_MOUNT_SKYDROID_CMD_START_CANCEL_TRACKING 2
#define AP_MOUNT_SKYDROID_CMD_EXTERNAL_COMPUTE_REQUEST 3
#define AP_MOUNT_SKYDROID_CMD_ISP_PARAMETERS 4
#define AP_MOUNT_SKYDROID_CMD_AI_RESET_FRAME_INTERVAL 5
#define AP_MOUNT_SKYDROID_CMD_ACKNOWLEDGMENT 23
#define AP_MOUNT_SKYDROID_CMD_GIMBAL_ANGLE_OUTPUT 33

class AP_Mount_Skydroid_C13 : public AP_Mount_Backend
{
public:
    // Constructor
    using AP_Mount_Backend::AP_Mount_Backend;

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Mount_Skydroid_C13);

    // init - performs any required initialisation for this instance
    void init() override;

    // update mount position - should be called periodically
    void update() override {}
    void update_fast() override;

    // return true if healthy
    bool healthy() const override;

    // has_pan_control - returns true if this mount can control its pan
    bool has_pan_control() const override { return yaw_range_valid(); }

    // get attitude as a quaternion. returns true on success
    bool get_attitude_quaternion(Quaternion& att_quat) override;

private:
    // parsing state
    enum class ParseState : uint8_t {
        WAITING_FOR_HEADER1,
        WAITING_FOR_HEADER2,
        WAITING_FOR_LENGTH_LOW,
        WAITING_FOR_LENGTH_HIGH,
        WAITING_FOR_SEQUENCE_LOW,
        WAITING_FOR_SEQUENCE_HIGH,
        WAITING_FOR_CONTROL,
        WAITING_FOR_DATA,
        WAITING_FOR_CRC_LOW,
        WAITING_FOR_CRC_HIGH,
        WAITING_FOR_TAIL,
    };

    // TrackingDataToFC structure (39 bytes)
    struct PACKED TrackingDataToFC {
        uint8_t status;                 // 1=normal tracking, 3=error
        int16_t target_center_x;        // Target center X (720P coords)
        int16_t target_center_y;        // Target center Y (720P coords)
        int16_t gimbal_roll;            // Roll angle * 100
        int16_t gimbal_pitch;           // Pitch angle * 100
        int16_t gimbal_yaw;             // Yaw angle * 100
        uint8_t video_type;             // 0=visible, 1=IR
        uint16_t crop_width;            // Crop region width (720P)
        uint16_t crop_height;           // Crop region height (720P)
        int16_t offset_h;               // Horizontal offset angle * 100
        int16_t offset_v;               // Vertical offset angle * 100
        uint16_t target_width;          // Target width (actual pixels)
        uint16_t target_height;         // Target height (actual pixels)
        uint8_t confidence;             // Confidence percentage
        uint8_t focal_length;           // Focal length * 10
        uint16_t sensor_w;              // Sensor width
        uint16_t sensor_h;              // Sensor height
        uint16_t h_fov;                 // Horizontal FOV * 100
        uint16_t v_fov;                 // Vertical FOV * 100
        uint16_t laser_distance;        // Laser distance in decimeters
        uint8_t reserved[6];            // Reserved bytes
    };

    // reading incoming packets from gimbal
    void read_incoming_packets();

    // process successfully decoded packets
    void process_packet();

    // internal variables
    AP_HAL::UARTDriver *_uart;                      // uart connected to gimbal
    bool _initialised;                              // true once the driver has been initialised

    // buffer holding bytes from latest packet
    uint8_t _msg_buff[AP_MOUNT_SKYDROID_PACKETLEN_MAX];
    uint8_t _msg_buff_len;

    // parser state and unpacked fields
    struct PACKED {
        uint16_t data_len;                          // expected number of data bytes
        uint16_t sequence;                          // sequence number
        uint8_t command_id;                         // command id
        uint16_t data_bytes_received;               // number of data bytes received so far
        uint8_t crc_low, crc_high;                  // received CRC bytes
        ParseState state;                           // state of incoming message processing
    } _parsed_msg;

    // actual attitude received from gimbal
    Vector3f _current_angle_rad;                    // current angles in radians (x=roll, y=pitch, z=yaw)
    uint32_t _last_current_angle_rad_ms;            // system time _current_angle_rad was updated

    // tracking state
    bool _tracking_active;                          // true if actively tracking
    uint8_t _tracking_confidence;                   // tracking confidence 0-100%
    Vector3f _target_ned_offset_m;                  // target offset in NED coordinates (meters)
    uint32_t _last_tracking_data_ms;                // system time of last tracking data
};

#endif // HAL_MOUNT_SKYDROID_ENABLED
