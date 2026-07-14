/*
 * Copyright (C) 2017 Emlid Ltd. All rights reserved.
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/system.h>

#if HAL_ENABLE_DRONECAN_DRIVERS
#include "DroneCAN_RGB_LED.h"

#include <AP_DroneCAN/AP_DroneCAN.h>

#include <AP_CANManager/AP_CANManager.h>

#ifdef ALIGN_PCU_CAN
#include "AP_BattMonitor/AP_BattMonitor.h"
const AP_BattMonitor &batt = AP::battery();
#endif

#define LED_OFF 0
#define LED_FULL_BRIGHT 255
#define LED_MEDIUM ((LED_FULL_BRIGHT / 5) * 4)
#define LED_DIM ((LED_FULL_BRIGHT / 5) * 2)

DroneCAN_RGB_LED::DroneCAN_RGB_LED()
    : DroneCAN_RGB_LED(LED_OFF,
                     LED_FULL_BRIGHT, LED_MEDIUM, LED_DIM)
{
}

DroneCAN_RGB_LED::DroneCAN_RGB_LED(uint8_t led_off,
                               uint8_t led_full, uint8_t led_medium,
                               uint8_t led_dim)
    : RGBLed(led_off, led_full, led_medium, led_dim)
{
#ifdef ALIGN_PCU_CAN
    _last_led_bitmask = 0;
    _bitmask_sent = false;
#endif
}

bool DroneCAN_RGB_LED::init()
{
    // LEDs can turn up later
    return true;
}


bool DroneCAN_RGB_LED::hw_set_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    uavcan_equipment_indication_LightsCommand msg {};
    msg.commands.len = 1;
    msg.commands.data[0].light_id =0;
    msg.commands.data[0].color.red = red >> 3;
    msg.commands.data[0].color.green = green >> 2;
    msg.commands.data[0].color.blue = blue >> 3;

    return broadcast(msg);
}

// broadcast the message on all CAN ifaces
bool DroneCAN_RGB_LED::broadcast(uavcan_equipment_indication_LightsCommand &msg)
{
    uint8_t can_num_drivers = AP::can().get_num_drivers();
    bool ok = false;
    for (uint8_t i = 0; i < can_num_drivers; i++) {
        auto *dronecan = AP_DroneCAN::get_dronecan(i);
        if (dronecan != nullptr) {
            ok |= dronecan->rgb_led.broadcast(msg);
        }
    }
    return ok;
}

#ifdef ALIGN_PCU_CAN
// update - drives the PCU battery panel. Runs at 4 Hz and only broadcasts
// when the computed bitmask changes (the <20% blink toggles the bitmask, so
// it is still sent on each transition).
void DroneCAN_RGB_LED::update()
{
    RGBLed::update();

    // light id 10 is the PCU battery panel bitmask, carried on the green
    // channel. Each bit represents one of the 4 LEDs in the panel.
    uint8_t percentage = 0;
    if (!batt.capacity_remaining_pct(percentage, 0)) {
        // battery driver is not ready
        return;
    }
    if (percentage >= 99) {
        // Wait rtl_dist.lua to override the percentage, to prevent glitch. 
        // 100% is already handled corrected by PCU for all battery types.
        return;
    }

    const uint32_t now_ms = AP_HAL::millis();
    uint8_t led_bitmask;
    if (percentage > 80) {
        led_bitmask = 0b1111;   // all 4 LEDs on
    } else if (percentage > 60) {
        led_bitmask = 0b0111;   // 3 LEDs on
    } else if (percentage > 40) {
        led_bitmask = 0b0011;   // 2 LEDs on
    } else if (percentage > 20) {
        led_bitmask = 0b0001;   // 1 LED on
    } else {
        // blink LED 1 at 2 Hz (250ms on, 250ms off)
        led_bitmask = (now_ms % 500) < 250 ? 0b0001 : 0b0000;
    }

    // only broadcast on a change
    if (_bitmask_sent && led_bitmask == _last_led_bitmask) {
        return;
    }

    uavcan_equipment_indication_LightsCommand msg {};
    msg.commands.len = 1;
    msg.commands.data[0].light_id = 10;
    msg.commands.data[0].color.red = 0;
    msg.commands.data[0].color.green = led_bitmask;
    msg.commands.data[0].color.blue = 0;

    if (broadcast(msg)) {
        _last_led_bitmask = led_bitmask;
        _bitmask_sent = true;
    }
}
#endif // ALIGN_PCU_CAN

#endif // HAL_ENABLE_DRONECAN_DRIVERS

