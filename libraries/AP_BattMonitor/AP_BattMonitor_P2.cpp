#include "AP_BattMonitor_config.h"

#if AP_BATTERY_P2_ENABLED

//#define AP_BATTERY_P2_DEBUG

#include "AP_BattMonitor_P2.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_GPS/AP_GPS.h>
#include <RC_Channel/RC_Channel.h>

/*
  battery and fluid monitor for Align E1.

  The first driver AP_BattMonitor_P2 actually parse the protocol,
  and return the main 12s battery data.
  Second driver AP_BattMonitor_P2_3s return the 3s battery data.
  Third driver AP_BattMonitor_P2_BEC return the BEC voltages
  Fourth driver AP_BattMonitor_P2_FuelFlow return the flow infos:
    - current in Amps maps to in litres/minute
    - consumed mAh is in consumed millilitres
    - 1.0v voltage if liquid is full
 */

// not healthy in 5 s
#define P2_NOT_HEALTY_MICROS 5000000
#define P2_SEND_MESSAGE_MICROS 500000

// State LED
#define AP_BATTERY_P2_LED_1_LONG_BLINK_RED      0
#define AP_BATTERY_P2_LED_1_FAST_BLINK_GREEN    2
#define AP_BATTERY_P2_LED_2_FAST_BLINK_GREEN    3
#define AP_BATTERY_P2_LED_3_FAST_BLINK_GREEN    4
#define AP_BATTERY_P2_LED_4_FAST_BLINK_GREEN    5
#define AP_BATTERY_P2_LED_2_FAST_BLINK_RED      7
#define AP_BATTERY_P2_LED_1_FAST_BLINK_RED      9
#define AP_BATTERY_P2_LED_3_FAST_BLINK_ORANGE   10

// Flights mode
#define ALT_HOLD 2
#define AUTO 3
#define GUIDED 4
#define LOITER 5
#define RTL 6
#define LAND 9
#define POSHOLD 16
#define BRAKE 17
#define SMART_RTL 21

#ifdef AP_BATTERY_P2_DEBUG
#define debug(fmt, args ...)  do {hal.console->printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#else
#define debug(fmt, args ...)
#endif

extern const AP_HAL::HAL& hal;

// read - read the voltage and current
void AP_BattMonitor_P2::read()
{
    if (_uart == nullptr)
    {
        _uart = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_PCU, 0);
        if (_uart != nullptr) {
            _uart->begin(115200);
            debug("P2: started");
        }
        return;
    }

    const uint32_t now_micros = AP_HAL::micros();

    // Receive battery informations from P2
    if (parse_message()) {
        // Healthy if messages are received
        _state.healthy = true;

        // get voltage
        _state.voltage = _mon.pcu_shared_data.vbatt_12s * 0.1f;

        // get temp
        _state.temperature = _mon.pcu_shared_data.motor_temp;

        // calculate time since last current read
        const uint32_t dt_us = _mon.pcu_shared_data.micros_batt - _state.last_time_micros;
        // read current
        _state.current_amps = _mon.pcu_shared_data.ibatt_12s * 0.1f;
        update_consumed(_state, dt_us);
        // record time
        _state.last_time_micros = _mon.pcu_shared_data.micros_batt;
    }
    // Not healthy if no messages for 5 seconds
    else if (((now_micros - _state.last_time_micros) > P2_NOT_HEALTY_MICROS)){
        _state.healthy = false;
    }

    if (now_micros - _last_send_micros > P2_SEND_MESSAGE_MICROS) {
        send_message();
        _last_send_micros = now_micros;
    }
}

bool AP_BattMonitor_P2::parse_message()
{
    // check for bytes on the serial port
    int16_t nbytes = MIN(_uart->available(), 1024U);

    if (nbytes <= 0 ) {
        return false;
    }

    // true if a packet was successfully parsed
    bool parsed = false;

    // process bytes received
    for (int16_t i = 0; i < nbytes; i++) {
        const int16_t b = _uart->read();

        // sanity check byte
        if ((b < 0) || (b > 0xFF)) {
            continue;
        }

        _msg_buff[_msg_buff_len++] = b;

        // flag to allow cases below to reset parser state
        bool reset_parser = false;

        // protect against overly long messages
        if (_msg_buff_len >= AP_BATT_P2_PACKET_MAX_LENGTH) {
            reset_parser = true;
        }

        // process byte depending upon current state
        switch (_parsed_msg.state) {

        case ParseState::WAITING_FOR_HEADER:
            if (b == AP_BATT_P2_HEADER) {
                _parsed_msg.state = ParseState::WAITING_FOR_OPERATION;
            } else {
                debug("Wrong header");
                reset_parser = true;
            }
            break;

        case ParseState::WAITING_FOR_OPERATION:
            if (b == AP_BATT_P2_OPERATION_BROADCAST) {
                _parsed_msg.state = ParseState::WAITING_FOR_LENGTH;
            } else {
                reset_parser = true;
            }
            break;

        case ParseState::WAITING_FOR_LENGTH:
            _parsed_msg.data_len = b;
            if (_parsed_msg.data_len >= AP_BATT_P2_LENGTH_FUEL) {
                _parsed_msg.state = ParseState::WAITING_FOR_SENDER_ID;
            } else {
                reset_parser = true;
                debug("error: data len received: %u", (unsigned)_parsed_msg.data_len);
            }
            break;

        case ParseState::WAITING_FOR_SENDER_ID:
            if (b == AP_BATT_P2_SENDER_P2) {
                _parsed_msg.sender_id = b;
                _parsed_msg.state = ParseState::WAITING_FOR_RECEIVER_ID;
            } else {
                debug("error: wrong sender");
                reset_parser = true;
            }
            break;

        case ParseState::WAITING_FOR_RECEIVER_ID:
                _parsed_msg.receiver_id = b;
                _parsed_msg.state = ParseState::WAITING_FOR_PACKET_ID;
            break;

        case ParseState::WAITING_FOR_PACKET_ID:
                _parsed_msg.packet_id = b;
                _parsed_msg.state = ParseState::WAITING_FOR_PAYLOAD;
            break;

        case ParseState::WAITING_FOR_PAYLOAD:
            _parsed_msg.data_bytes_received++;
            if (_parsed_msg.data_bytes_received == _parsed_msg.data_len) {
                _parsed_msg.checksum = 0;
                _parsed_msg.checksum_bit = 0;
                _parsed_msg.state = ParseState::WAITING_FOR_CRC;
            }
            break;

        case ParseState::WAITING_FOR_CRC:
            _parsed_msg.checksum = _parsed_msg.checksum | (b << (_parsed_msg.checksum_bit*8));
            _parsed_msg.checksum_bit++;
            if (_parsed_msg.checksum_bit == 4) {
                uint32_t checksum = strange_crc32(_msg_buff+1, _parsed_msg.data_len+5);
                if (checksum == _parsed_msg.checksum) {
                    switch (_parsed_msg.packet_id) {
                        case AP_BATT_P2_PACKET_ID_BATT:
                            parse_batt();
                            parsed = true;
                            break;
                        case AP_BATT_P2_PACKET_ID_FUEL:
                            parse_fuel();
                            break;
                        default:
                            debug("unknown packet id:%x", _parsed_msg.packet_id);
                            break;
                    }
                } else { 
                    debug("checksum expected: %x, got: %x", (unsigned int)checksum, (unsigned int)_parsed_msg.checksum);
                }
                reset_parser = true;
            }
            break;
        }

        // handle reset of parser
        if (reset_parser) {
            _parsed_msg.state = ParseState::WAITING_FOR_HEADER;
            _msg_buff_len = 0;
            _parsed_msg.data_bytes_received = 0;
        }
    }
    return parsed;
}

uint32_t AP_BattMonitor_P2::strange_crc32(const uint8_t *buf, uint8_t size)
{
    // buffer must have an exact multiple of 4 bytes before pass it for checksum calculation
    // add 0x00 bytes to fill the array until his length is multiple of 4
    uint8_t msg_buffer_checksum_size = size + ((4 - (size % 4)) % 4);
    uint8_t msg_buffer_checksum[msg_buffer_checksum_size] = { 0 };
    memcpy(msg_buffer_checksum, buf, size);

    // calculate checksum
    uint32_t checksum = 0xFFFFFFFF;
    return crc_crc32_mpeg2(checksum, msg_buffer_checksum, msg_buffer_checksum_size);
}

void AP_BattMonitor_P2::parse_batt()
{
    // populate shared data
    _mon.pcu_shared_data.vbatt_3s = UINT16_VALUE(_msg_buff[7], _msg_buff[6]);
    _mon.pcu_shared_data.vbatt_12s = UINT16_VALUE(_msg_buff[9], _msg_buff[8]);
    _mon.pcu_shared_data.vbec1 = UINT16_VALUE(_msg_buff[11], _msg_buff[10]);
    _mon.pcu_shared_data.vbec2 = UINT16_VALUE(_msg_buff[13], _msg_buff[12]);
    _mon.pcu_shared_data.vbec_out = UINT16_VALUE(_msg_buff[15], _msg_buff[14]);
    _mon.pcu_shared_data.vfcu = UINT16_VALUE(_msg_buff[17], _msg_buff[16]);
    _mon.pcu_shared_data.ibatt_3s = UINT16_VALUE(_msg_buff[19], _msg_buff[18]);
    _mon.pcu_shared_data.ibatt_12s = UINT16_VALUE(_msg_buff[21], _msg_buff[20]);
    _mon.pcu_shared_data.ibec = UINT16_VALUE(_msg_buff[23], _msg_buff[22]);
    _mon.pcu_shared_data.motor_temp = static_cast<int16_t>(((_msg_buff[25]) << 8) | (_msg_buff[24]));
    _mon.pcu_shared_data.fan_on = UINT16_VALUE(_msg_buff[27], _msg_buff[26]) > 0;
    _mon.pcu_shared_data.micros_batt = AP_HAL::micros();
}

void AP_BattMonitor_P2::parse_fuel()
{
    // populate shared data
    _mon.pcu_shared_data.fuel_level_full = _msg_buff[6] > 0;
    _mon.pcu_shared_data.flow_sensor_raw = UINT32_VALUE(_msg_buff[17], _msg_buff[16], _msg_buff[15], _msg_buff[14]);
    _mon.pcu_shared_data.micros_fuel = AP_HAL::micros();
}

void AP_BattMonitor_P2::send_message()
{
    // static local count variable for LED test
    static uint8_t led_count = 0;

    // Pointer to notify to check system status
    const AP_Notify &notify = AP::notify();

    uint8_t heartbeat[18] = {AP_BATT_P2_HEADER, AP_BATT_P2_OPERATION_BROADCAST, AP_BATT_P2_LENGTH_HEARTBEAT, 0x01, 0x00, AP_BATT_P2_PACKET_ID_HEARTBEAT};
    
    if (notify.flags.gps_status < AP_GPS::GPS_OK_FIX_3D || notify.flags.gps_num_sats < 8) {
        heartbeat[10] = 4; // GPS error
    }

    // Tail LEDs controller
    if (notify.flags.flight_mode == POSHOLD) {
        // Test LED routine
        if (led_count < 5) {
            heartbeat[12] = AP_BATTERY_P2_LED_2_FAST_BLINK_RED;
        } else if (led_count < 10) {
            heartbeat[12] = AP_BATTERY_P2_LED_3_FAST_BLINK_ORANGE;
        } else if (led_count < 15) {
            heartbeat[12] = AP_BATTERY_P2_LED_3_FAST_BLINK_GREEN;
        } else {
            led_count = 0;
        }
        led_count++;
    } else if (!notify.flags.pre_arm_check) {
        heartbeat[12] = AP_BATTERY_P2_LED_2_FAST_BLINK_RED;
    } else {
        switch (notify.flags.flight_mode)
        {
        case ALT_HOLD:
            heartbeat[12] = AP_BATTERY_P2_LED_3_FAST_BLINK_ORANGE;
            break;

        case LOITER:
            heartbeat[12] = AP_BATTERY_P2_LED_2_FAST_BLINK_GREEN;
            break;

        case AUTO:
        case GUIDED:
        case BRAKE:
            heartbeat[12] = AP_BATTERY_P2_LED_3_FAST_BLINK_GREEN;
            break;

        case RTL:
        case SMART_RTL:
            heartbeat[12] = AP_BATTERY_P2_LED_4_FAST_BLINK_GREEN;
            break;
        
        default:
            heartbeat[12] = AP_BATTERY_P2_LED_1_FAST_BLINK_RED;
            break;
        }
    }

    //Calculate and add checksum
    uint32_t checksum = strange_crc32(heartbeat+1, AP_BATT_P2_LENGTH_HEARTBEAT+5);
    heartbeat[14] = checksum & 0xFF;
    heartbeat[15] = (checksum >> 8) & 0xFF;
    heartbeat[16] = (checksum >> 16) & 0xFF;
    heartbeat[17] = (checksum >> 24) & 0xFF;

    // Write the message to the Serial port
    _uart->write((const uint8_t*)heartbeat, ARRAY_SIZE(heartbeat));

    if (!notify.flags.failsafe_radio && rc().has_had_rc_receiver()) {
        // send gcs heartbeat packet
        uint8_t heartbeat_gcs[18] = {   AP_BATT_P2_HEADER,
                                        AP_BATT_P2_OPERATION_BROADCAST,
                                        AP_BATT_P2_LENGTH_HEARTBEAT,
                                        0xFF, // sender ID: GCS
                                        0x00,
                                        AP_BATT_P2_PACKET_ID_HEARTBEAT,
                                        0x00, // payload: all 0x00
                                        0x00,
                                        0x00,
                                        0x00,
                                        0x00,
                                        0x00,
                                        0x00,
                                        0x00,
                                        0xC1, // checksum: hard coded for efficiency
                                        0x14,
                                        0x2C,
                                        0x85 };
        _uart->write((const uint8_t*)heartbeat_gcs, ARRAY_SIZE(heartbeat_gcs));
    }
}

// read - read the voltage and current
void AP_BattMonitor_P2_3s::read()
{
    _state.healthy = (AP_HAL::micros() - _mon.pcu_shared_data.micros_batt) < P2_NOT_HEALTY_MICROS;

    if (_mon.pcu_shared_data.micros_batt != _state.last_time_micros) {

        // update time
        _state.last_time_micros = _mon.pcu_shared_data.micros_batt;
    
        // update voltage
        _state.voltage = _mon.pcu_shared_data.vbatt_3s * 0.1f;
        
        // update current
        _state.current_amps = _mon.pcu_shared_data.ibatt_3s * 0.1f;

        // update fan state as temperature
        _state.temperature = _mon.pcu_shared_data.fan_on ? 1.0f : 0.0f;
    }
}

// read - read the voltage and current
void AP_BattMonitor_P2_BEC::read()
{
    if ((AP_HAL::micros() - _mon.pcu_shared_data.micros_batt) > P2_NOT_HEALTY_MICROS) {
        _state.healthy = false;
        return;
    }

    if (_mon.pcu_shared_data.micros_batt != _state.last_time_micros) {
        // set healthy flag
        _state.healthy = true;

        // update time
        _state.last_time_micros = _mon.pcu_shared_data.micros_batt;
    
        // update voltage
        _state.cell_voltages.cells[0] = _mon.pcu_shared_data.vbec_out * 100;
        _state.cell_voltages.cells[1] = _mon.pcu_shared_data.vbec1 * 100;
        _state.cell_voltages.cells[2] = _mon.pcu_shared_data.vbec2 * 100;
        
        // update current
        _state.current_amps = _mon.pcu_shared_data.ibec * 0.1f;
    }
}

/// Constructor
AP_BattMonitor_P2_FuelFlow::AP_BattMonitor_P2_FuelFlow( AP_BattMonitor &mon,
                                                        AP_BattMonitor::BattMonitor_State &mon_state,
                                                        AP_BattMonitor_Params &params) :
    AP_BattMonitor_Analog(mon, mon_state, params)
{ 
    // Just to be sure it is init to 0
    _state.last_time_micros = 0;
}

void AP_BattMonitor_P2_FuelFlow::read()
{
    // check healthy state
    if ((AP_HAL::micros() - _mon.pcu_shared_data.micros_fuel) > P2_NOT_HEALTY_MICROS) {
        _state.healthy = false;
        return;
    }

    if (_mon.pcu_shared_data.micros_fuel != _state.last_time_micros) {
        if (_state.last_time_micros == 0) {
        // need initial time and liters
        _state.last_time_micros = _mon.pcu_shared_data.micros_fuel;
        _zero_milliliters = _mon.pcu_shared_data.flow_sensor_raw;
        return;
        }

        // set healthy flag
        _state.healthy = true;

        float dt = (_mon.pcu_shared_data.micros_fuel - _state.last_time_micros) * 1.0e-6f;

        /*
        this driver assumes that BATTx_AMP_PERVLT is set to give the
        number of millilitres per pulse.
        */
        float litres = (_mon.pcu_shared_data.flow_sensor_raw - _zero_milliliters) * _curr_amp_per_volt * 0.001f;
        float litres_pec_sec = (litres - _last_litres) / dt;

        // update time and last litres
        _state.last_time_micros = _mon.pcu_shared_data.micros_fuel;
        _last_litres = litres;

        // map amps to litres/minute
        _state.current_amps = litres_pec_sec * 60;

        // map consumed_mah to consumed millilitres
        _state.consumed_mah = litres * 1000;

        // map consumed_wh using fixed voltage of 1
        _state.consumed_wh = _state.consumed_mah;

        // use voltage as digital signal for fuel level sensor
        _state.voltage = _mon.pcu_shared_data.fuel_level_full ? 1.0f : 0.0f;
    }
}

bool AP_BattMonitor_P2_FuelFlow::reset_remaining(float litres)
{
    _params._pack_capacity.set(litres * 1000);
    _zero_milliliters = _mon.pcu_shared_data.flow_sensor_raw;
    return true;
}

#endif
