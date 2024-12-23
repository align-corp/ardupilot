/*
  Siyi gimbal driver using custom serial protocol

  Packet format (courtesy of Siyi's SDK document)

  -------------------------------------------------------------------------------------------
  Field     Index   Bytes       Description
  -------------------------------------------------------------------------------------------
  STX       0       2           0x5566: starting mark
  CTRL      2       1           bit 0: need_ack.  set if the current data packet needs ack
                                bit 1: ack_pack.  set if the current data packate IS an ack
                                bit 2-7: reserved
  Data_len  3       2           Data field byte length.  Low byte in the front
  SEQ       5       2           Frame sequence (0 ~ 65535).  Low byte in the front.  May be used to detect packet loss
  CMD_ID    7       1           Command ID
  DATA      8       Data_len    Data
  CRC16             2           CRC16 check the complete data package.  Low byte in the front
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
    bool _initialised;                              // true once the driver has been initialised
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
