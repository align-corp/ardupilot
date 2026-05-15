/***** Align G3P protocol *****
 * Parameter to set:
 * CAM1_TYPE 4
 * MNT1_TYPE 30
 * MNT1_RC_RATE 60
 * SERIALn_PROTOCOL 8 (G3P gimbal)
 * SERIALm_PROTOCOL 8 (G3P DV, m > n)
 * SERIALn_PROTOCOL 26 (zoom focus controller)
 * RCn_OPTION 167 (zoom control)
 * RCn_OPTION 213 (Pitch)
 * RCn_OPTION 214 (Yaw)
 * RCn_OPTION 27 (Camera trigger)
 */

#pragma once

#include "AP_Mount_Backend.h"

#if HAL_MOUNT_G3P_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>

// Gimbal bytes
#define AP_MOUNT_G3P_HEADER_SEND 0x18
#define AP_MOUNT_G3P_HEADER_RECEIVE 0x19
#define AP_MOUNT_G3P_CMD_PARAM_SET 0x01
#define AP_MOUNT_G3P_CMD_PARAM_GET 0x02
#define AP_MOUNT_G3P_CMD_ANGLE_SET 0x03
#define AP_MOUNT_G3P_CMD_CALIBRATE 0x04
#define AP_MOUNT_G3P_DATA_CENTER   0x5E
#define AP_MOUNT_G3P_CMD_ANGLE_REQUEST 0x05
#define AP_MOUNT_G3P_LENGTH_ANGLE_REQUEST 0x06

// Gimbal packet info
#define AP_MOUNT_G3P_PACKETLEN_MAX     87  // maximum number of bytes in a packet sent to or received from the gimbal
#define AP_MOUNT_G3P_PACKETLEN_MIN  5      // minimum number of bytes in a packet.  this is a packet with no data bytes
#define AP_MOUNT_G3P_DATALEN_MAX   (AP_MOUNT_G3P_PACKETLEN_MAX-AP_MOUNT_G3P_PACKETLEN_MIN) // max bytes for data portion of packet
#define AP_MOUNT_G3P_MSG_BUF_DATA_START 3  // data starts at this byte in _msg_buf
#define AP_MOUNT_G3P_PITCH_P       20    // pitch controller P gain (converts pitch angle error to target rate)
#define AP_MOUNT_G3P_YAW_P         20    // yaw controller P gain (converts yaw angle error to target rate)

// DV bytes
#define AP_MOUNT_DV_HEADER              0xAE
#define AP_MOUNT_DV_HEADER2_SEND        0xA1
#define AP_MOUNT_DV_HEADER2_RECEIVE     0xA2
#define AP_MOUNT_DV_CMD1                0x00
#define AP_MOUNT_DV_CMD2_LATH           0xA8
#define AP_MOUNT_DV_CMD2_LATL           0xA9
#define AP_MOUNT_DV_CMD2_LONH           0xAA
#define AP_MOUNT_DV_CMD2_LONL           0xAB
#define AP_MOUNT_DV_CMD2_ALT            0xAC
#define AP_MOUNT_DV_CMD2_CAPTURE        0xCE
#define AP_MOUNT_DV_DATA1_CAPTURE       0x06
#define AP_MOUNT_DV_DATA2_CAPTURE       0x20
#define AP_MOUNT_DV_CRC1_CAPTURE        0x01
#define AP_MOUNT_DV_CRC2_CAPTURE        0x95
#define AP_MOUNT_DV_END                 0xEA

#define AP_MOUNT_DV_PACKETLEN       9       // Only 9 bytes packet are allowed (strange)

// Zoom motor controller bytes
#define AP_MOUNT_G3P_ZOOM_HEADER_REQ    0xA0
#define AP_MOUNT_G3P_ZOOM_HEADER_RESP   0xA1
#define AP_MOUNT_G3P_ZOOM_CMD_MOVE      0x00
#define AP_MOUNT_G3P_ZOOM_CMD_GET_POS   0x04
#define AP_MOUNT_G3P_ZOOM_STATUS_OK     0x00
#define AP_MOUNT_G3P_ZOOM_PAYLOAD_MAX   4       // max payload bytes (request or response)
#define AP_MOUNT_G3P_ZOOM_MAX           17100   // maximum zoom value
#define AP_MOUNT_G3P_ZOOM_STEP          1900    // size of each relative step sent to the controller
#define AP_MOUNT_G3P_ZOOM_TIMEOUT_MS    5000   // timeout for both the startup GET_POS handshake and each move ack


class AP_Mount_G3P : public AP_Mount_Backend
{

public:
    // Constructor
    using AP_Mount_Backend::AP_Mount_Backend;

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Mount_G3P);

    // init - performs any required initialisation for this instance
    void init() override;

    // update mount position - should be called periodically
    void update() override;

    // return true if healthy
    bool healthy() const override;

    // has_pan_control - returns true if this mount can control its pan (required for multicopters)
    bool has_pan_control() const override { return yaw_range_valid(); };

    //
    // camera controls
    //

    // take a picture.  returns true on success
    bool take_picture() override {return send_packet_dv(AP_MOUNT_DV_CMD1, AP_MOUNT_DV_CMD2_CAPTURE, AP_MOUNT_DV_DATA1_CAPTURE, AP_MOUNT_DV_DATA2_CAPTURE); }

protected:

    // get attitude as a quaternion.  returns true on success
    bool get_attitude_quaternion(Quaternion& att_quat) override;

private:

    // parsing state
    enum class ParseStateGimbal : uint8_t {
        WAITING_FOR_HEADER,
        WAITING_FOR_COMMAND,
        WAITING_FOR_LENGTH,
        WAITING_FOR_DATA,
        WAITING_FOR_CRC_A,
        WAITING_FOR_CRC_B,
    };

    // zoom controller response parser state
    enum class ParseStateZoom : uint8_t {
        WAITING_FOR_HEADER,
        WAITING_FOR_CMD,
        WAITING_FOR_STATUS,
        WAITING_FOR_LEN,
        WAITING_FOR_PAYLOAD,
        WAITING_FOR_CRC,
    };

    // reading incoming packets from gimbal and confirm they are of the correct format
    // results are held in the _parsed_msg structure
    void read_incoming_packets();

    // process successfully decoded packets held in the _parsed_msg structure
    void process_packet();

    // send packet to gimbal
    // returns true on success, false if outgoing serial buffer is full
    bool send_packet_gimbal(uint8_t cmd_id, const uint8_t* databuff, uint8_t databuff_len);

    // send packet to DV
    // returns true on success, false if outgoing serial buffer is full
    bool send_packet_dv(uint8_t cmd_id1, uint8_t cmd_id2, uint8_t data1, uint8_t data2);

    // send a framed packet to the zoom motor controller.
    // payload is at most AP_MOUNT_G3P_ZOOM_PAYLOAD_MAX bytes; pass nullptr / 0 for no payload.
    // returns true on success, false if outgoing serial buffer is full or payload too big
    bool send_packet_zoom(uint8_t cmd, const uint8_t *payload, uint8_t payload_len);

    // read the camera-zoom RC channel and forward the target to the zoom motor controller
    void update_zoom();

    // consume any bytes from the zoom controller (any byte = move complete) and report timeout
    void check_zoom_response();

    // request info from gimbal
    void request_gimbal_attitude() { send_packet_gimbal(AP_MOUNT_G3P_CMD_ANGLE_REQUEST, nullptr, 0); }

    // center gimbal
    void center_gimbal();

    // send target pitch and yaw rates to gimbal
    // yaw_is_ef should be true if yaw_rads target is an earth frame rate, false if body_frame
    void send_target_rates(float pitch_rads, float roll_rads, float yaw_rads);

    // send target pitch and yaw angles to gimbal
    // yaw_is_ef should be true if yaw_rad target is an earth frame angle, false if body_frame
    void send_target_angles(float pitch_rad, float yaw_rad, bool yaw_is_ef, bool pitch_send_rate = false);

    // send position to DV
    void send_gps_position();

    // internal variables
    AP_HAL::UARTDriver *_uart;                      // uart connected to gimbal
    AP_HAL::UARTDriver *_uart_dv;                   // uart connected to DV
    AP_HAL::UARTDriver *_uart_zoom;                 // uart connected to zoom/focus motor controller
    bool _initialised;                              // true once the driver has been initialised
    int16_t _current_zoom_position = 0;             // motor position in controller counts (updated from GET_POS reply and after each OK MOVE ack)
    int16_t _last_zoom_step = 0;                    // step value sent in last MOVE; applied to _current_zoom_position when OK ack arrives
    bool _zoom_pos_known = false;                   // true once the startup GET_POS handshake has succeeded
    bool _zoom_pos_requested = false;               // true once the startup GET_POS request has been sent
    bool _zoom_is_moving = false;                   // true while waiting for a MOVE ack
    bool _zoom_failed = false;                      // true if init handshake failed; zoom controller disabled
    uint32_t _last_zoom_send_ms = 0;                // system time of the most recent zoom request

    // zoom response parser state and unpacked fields
    struct {
        uint8_t cmd;
        uint8_t status;
        uint8_t len;
        uint8_t payload[AP_MOUNT_G3P_ZOOM_PAYLOAD_MAX];
        uint8_t payload_received;
        ParseStateZoom state;
    } _zoom_parsed_msg;
    // buffer holding bytes from latest packet
    uint8_t _msg_buff[AP_MOUNT_G3P_PACKETLEN_MAX];
    uint8_t _msg_buff_len;

    // parser state and unpacked fields
    struct PACKED {
        uint8_t data_len;                           // expected number of data bytes
        uint8_t command_id;                         // command id
        uint16_t data_bytes_received;               // number of data bytes received so far
        uint8_t ck_a;                               // latest message's crc
        uint8_t ck_b;                               // latest message's crc
        ParseStateGimbal state;                     // state of incoming message processing
    } _parsed_msg;

    // variables for sending packets to gimbal
    uint32_t _last_send_ms;                         // system time (in milliseconds) of last packet sent to gimbal

    // actual attitude received from gimbal
    Vector3f _current_angle_rad;                    // current angles in radians received from gimbal (x=roll, y=pitch, z=yaw)
    uint32_t _last_current_angle_rad_ms;            // system time _current_angle_rad was updated
    uint32_t _last_req_current_angle_rad_ms;        // system time that this driver last requested current angle
    uint32_t _last_coordinates_send_ms;             // system time that this driver last sent the GNSS coordinates to DV

};

#endif // HAL_MOUNT_G3P_ENABLED
