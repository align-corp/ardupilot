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
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL_Boards.h>

#include "NotifyDevice.h"

class AP_BoardLED_Align: public NotifyDevice
{
public:
    // initialise the LED driver
    bool init(void) override;

    // should be called at 50Hz
    void update(void) override;

private:

#if (defined(HAL_GPIO_A_LED_PIN) && defined(HAL_GPIO_B_LED_PIN) && \
     defined(HAL_GPIO_C_LED_PIN))

    void set_led_from_voltage();

    enum class State : uint8_t {
        START,
        LED1_ON,
        LED2_ON,
        LED3_ON,
        LED4_ON,
        TURNING_ON,
        WAIT_BUTTON_RELEASED_ON,
        ON,
        TURNING_OFF,
        WAIT_BUTTON_RELEASED_OFF,
        LED4_OFF,
        LED3_OFF,
        LED2_OFF,
        LED1_OFF,
        DIE_HERE,
        CHECK_VOLTAGE,
    } _state;
    uint32_t _last_update_ms;
    uint32_t _check_voltage_ms;

#endif
};
