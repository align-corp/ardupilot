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

#include "AP_RangeFinder_Align_R50.h"

#if AP_RANGEFINDER_ALIGN_ENABLED

#define R50_REQUEST_DEVICE_ID 0x00
#define R50_REQUEST_DIST 0x01
#define R50_REGISTER_DEVICE_ID 0x32

#ifdef AP_RANGEFINDER_ALIGN_R50_DEBUG
#define debug(fmt, args ...)  do {hal.console->printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#else
#define debug(fmt, args ...)
#endif

#include <utility>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>

extern const AP_HAL::HAL& hal;

AP_RangeFinder_Align_R50::AP_RangeFinder_Align_R50(RangeFinder::RangeFinder_State &_state,
                                                           AP_RangeFinder_Params &_params,
                                                           AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
    : AP_RangeFinder_Backend(_state, _params)
    , _dev(std::move(dev))
{
}

// Check device ID for detection
AP_RangeFinder_Backend *AP_RangeFinder_Align_R50::detect(RangeFinder::RangeFinder_State &_state,
																AP_RangeFinder_Params &_params,
                                                             AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
{
    if (!dev) {
        return nullptr;
    }

    AP_RangeFinder_Align_R50 *sensor
        = new AP_RangeFinder_Align_R50(_state, _params, std::move(dev));
    if (!sensor) {
        return nullptr;
    }

    if (!sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

// initialise sensor
bool AP_RangeFinder_Align_R50::init(void)
{
    _dev->get_semaphore()->take_blocking();
    uint8_t id;
    if (!_dev->read_registers(R50_REQUEST_DEVICE_ID, &id, 1)) {
        _dev->get_semaphore()->give();
        debug("Can't read device id register\n"); 
        return false;
    }

    if (id != R50_REGISTER_DEVICE_ID) {
        _dev->get_semaphore()->give();
        debug("Wrong device ID\n"); 
        return false;
    }

    _dev->get_semaphore()->give();
    debug("Init OK\n"); 

    // init accum
    accum.sum = 0;
    accum.count = 0;

    // callback at 100 Hz
    _dev->register_periodic_callback(10000,
                                     FUNCTOR_BIND_MEMBER(&AP_RangeFinder_Align_R50::timer, void));

    return true;
}

// start_reading() - ask sensor to make a range reading
bool AP_RangeFinder_Align_R50::start_reading()
{
    // send command to take reading
    return _dev->read_registers(R50_REQUEST_DIST, nullptr, 0);
}

// read - return last value measured by sensor
bool AP_RangeFinder_Align_R50::get_reading(uint32_t &reading_mm, uint16_t &signal_strength)
{

    struct PACKED {
        uint8_t distance_mm_low;
        uint8_t distance_mm_high;
        uint8_t signal_strength_low;
        uint8_t signal_strength_high;
    } packet;

    // take range reading and read back results
    const bool ret = _dev->transfer(nullptr, 0, (uint8_t *) &packet, sizeof(packet));

    if (ret) {
        // combine results into distance
        reading_mm = UINT16_VALUE(packet.distance_mm_high, packet.distance_mm_low);
        signal_strength = UINT16_VALUE(packet.signal_strength_high, packet.signal_strength_low);
    }

    // trigger a new reading
    start_reading();

    return ret;
}

void AP_RangeFinder_Align_R50::timer(void)
{
    uint32_t dist_mm;
    uint16_t signal_strength;

    if (get_reading(dist_mm, signal_strength)) {
        WITH_SEMAPHORE(_sem);
        if (signal_strength > 0 && dist_mm > 0) {
            // healthy data
            accum.sum += dist_mm;
            accum.count++;
        } else if (accum.count == 0){
            accum.sum = params.max_distance_cm * 10 + 5000;
            accum.count = 1;
        }
    }
}

// update the state of the sensor
void AP_RangeFinder_Align_R50::update(void)
{
    WITH_SEMAPHORE(_sem);
    if (accum.count > 0) {
        state.last_reading_ms = AP_HAL::millis();
        state.distance_m = (accum.sum * 0.001f) / accum.count;
        accum.sum = 0;
        accum.count = 0;
        update_status();
    } else if (AP_HAL::millis() - state.last_reading_ms > 300) {
        // if no updates for 0.3 seconds set no-data
        set_status(RangeFinder::Status::NoData);
    }
}

#endif  // AP_RANGEFINDER_TOFSENSEF_I2C_ENABLED
