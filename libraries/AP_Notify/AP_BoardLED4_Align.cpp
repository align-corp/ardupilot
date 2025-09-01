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

#include "AP_BoardLED4_Align.h"

#if (defined(HAL_GPIO_A_LED_PIN) && defined(HAL_GPIO_B_LED_PIN) && \
     defined(HAL_GPIO_C_LED_PIN) && defined(HAL_GPIO_D_LED_PIN) && \
     defined(HAL_GPIO_BUTTON_PIN)&& defined(ALIGN_BATTERY_PANEL) && \
     defined(HAL_GPIO_MAIN_POWER_PIN) && defined(HAL_GPIO_POWER_OFF))

#include "AP_Notify.h"
#include "AP_BattMonitor/AP_BattMonitor.h"

static_assert((HAL_GPIO_A_LED_PIN != HAL_GPIO_B_LED_PIN) &&
              (HAL_GPIO_A_LED_PIN != HAL_GPIO_C_LED_PIN) &&
              (HAL_GPIO_A_LED_PIN != HAL_GPIO_D_LED_PIN) &&
              (HAL_GPIO_B_LED_PIN != HAL_GPIO_D_LED_PIN) &&
              (HAL_GPIO_B_LED_PIN != HAL_GPIO_C_LED_PIN) &&
              (HAL_GPIO_D_LED_PIN != HAL_GPIO_C_LED_PIN), "Duplicate LED assignments detected");

#ifndef HAL_GPIO_BUTTON_PRESSED
#define HAL_GPIO_BUTTON_PRESSED 1
#endif

#ifndef ALIGN_LED_BATT_80
#define ALIGN_LED_BATT_80 23.4f
#endif

#ifndef ALIGN_LED_BATT_60
#define ALIGN_LED_BATT_60 22.6f
#endif

#ifndef ALIGN_LED_BATT_40
#define ALIGN_LED_BATT_40 22.2f
#endif

#ifndef ALIGN_LED_BATT_20
#define ALIGN_LED_BATT_20 21.6f
#endif

extern const AP_HAL::HAL& hal;
const AP_BattMonitor &batt = AP::battery();

bool AP_BoardLED_Align::init(void)
{
    // setup the main LEDs as outputs
    // define pin mode in hwdef, to be sure all I/O are
    // set before init() is called
    /* hal.gpio->pinMode(HAL_GPIO_A_LED_PIN, HAL_GPIO_OUTPUT); */
    /* hal.gpio->pinMode(HAL_GPIO_B_LED_PIN, HAL_GPIO_OUTPUT); */
    /* hal.gpio->pinMode(HAL_GPIO_C_LED_PIN, HAL_GPIO_OUTPUT); */
    /* hal.gpio->pinMode(HAL_GPIO_D_LED_PIN, HAL_GPIO_OUTPUT); */
    _check_voltage_ms = 0;
    AP_Notify::flags.align_led_priority = true;
    _state = State::START;
    _last_update_ms = 0;
    return true;
}

/*
  main update function called at 50Hz
 */
void AP_BoardLED_Align::update(void)
{
    // slower update time
    uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _last_update_ms < 150) {
        return;
    }
    _last_update_ms = now_ms;

    // battery light state machine
    switch(_state) {
        case State::START:
            // Set turning off flag
            AP_Notify::flags.powering_off = false;

            // Check if this is a watchdog reset or a software reset
            if (hal.util->was_watchdog_reset() || hal.util->was_software_reset()) {
                // Skip manual sequence for reboot only near reboot
                if (now_ms < 5000) {
                    _state = State::ON;
                    AP_Notify::flags.align_led_priority = false;
                    break;
                }
            }
            // Turn all LED off
            hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_OFF);
            hal.gpio->write(HAL_GPIO_B_LED_PIN, HAL_GPIO_LED_OFF);
            hal.gpio->write(HAL_GPIO_C_LED_PIN, HAL_GPIO_LED_OFF);
            hal.gpio->write(HAL_GPIO_D_LED_PIN, HAL_GPIO_LED_OFF);
            
            // Check button
            if (hal.gpio->read(HAL_GPIO_BUTTON_PIN) == HAL_GPIO_BUTTON_PRESSED) {
                _state = State::LED1_ON;
            } else {
                _state = State::CHECK_VOLTAGE;
            }
            break;

        case State::LED1_ON:
            // Turn on LED1
            hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_ON);
            
            // Check button
            if (hal.gpio->read(HAL_GPIO_BUTTON_PIN) == HAL_GPIO_BUTTON_PRESSED) {
                _state = State::LED2_ON;
            } else {
                _state = State::CHECK_VOLTAGE;
            }
            break;

        case State::LED2_ON:
            // Turn on LED2
            hal.gpio->write(HAL_GPIO_B_LED_PIN, HAL_GPIO_LED_ON);
            
            // Check button
            if (hal.gpio->read(HAL_GPIO_BUTTON_PIN) == HAL_GPIO_BUTTON_PRESSED) {
                _state = State::LED3_ON;
            } else {
                _state = State::CHECK_VOLTAGE;
            }
            break;

        case State::LED3_ON:
            // Turn on LED3
            hal.gpio->write(HAL_GPIO_C_LED_PIN, HAL_GPIO_LED_ON);
            
            // Check button
            if (hal.gpio->read(HAL_GPIO_BUTTON_PIN) == HAL_GPIO_BUTTON_PRESSED) {
                _state = State::LED4_ON;
            } else {
                _state = State::CHECK_VOLTAGE;
            }
            break;

        case State::LED4_ON:
            // Turn on LED4
            hal.gpio->write(HAL_GPIO_D_LED_PIN, HAL_GPIO_LED_ON);

            // Check button
            if (hal.gpio->read(HAL_GPIO_BUTTON_PIN) == HAL_GPIO_BUTTON_PRESSED) {
                _state = State::TURNING_ON;
                // we don't need anymore LED priority
                AP_Notify::flags.align_led_priority = false;
            } else {
                _state = State::CHECK_VOLTAGE;
            }
            break;

        case State::TURNING_ON:
            // TODO: turn on 5 V, 12 V and ESC

            // Wait button to be released
            _state = State::WAIT_BUTTON_RELEASED_ON;

            break;

        case State::WAIT_BUTTON_RELEASED_ON:
            // Wait for button to be released
            if (hal.gpio->read(HAL_GPIO_BUTTON_PIN) == HAL_GPIO_BUTTON_PRESSED) {
                break;
            }

            _state = State::ON;
            break;

        case State::ON:
            // Update LED based on battery voltage
            set_led_from_voltage();

            // Do nothing if drone is armed
            if (AP_Notify::flags.armed) {
                break;
            }

            // Turn off if button is pressed
            if (hal.gpio->read(HAL_GPIO_BUTTON_PIN) == HAL_GPIO_BUTTON_PRESSED) {
                // Turn all LED on
                hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_ON);
                hal.gpio->write(HAL_GPIO_B_LED_PIN, HAL_GPIO_LED_ON);
                hal.gpio->write(HAL_GPIO_C_LED_PIN, HAL_GPIO_LED_ON);
                hal.gpio->write(HAL_GPIO_D_LED_PIN, HAL_GPIO_LED_ON);
                // Reset flash variable
                led_flash_first = true;
                _state = State::LED4_OFF;
            }
            break;

        case State::LED4_OFF:
            // Turn on LED4
            hal.gpio->write(HAL_GPIO_D_LED_PIN, HAL_GPIO_LED_OFF);

            // Check button
            if (hal.gpio->read(HAL_GPIO_BUTTON_PIN) == HAL_GPIO_BUTTON_PRESSED) {
                _state = State::LED3_OFF;
            } else {
                _state = State::ON;
            }
            break;

        case State::LED3_OFF:
            // Turn on LED4
            hal.gpio->write(HAL_GPIO_C_LED_PIN, HAL_GPIO_LED_OFF);

            // Check button
            if (hal.gpio->read(HAL_GPIO_BUTTON_PIN) == HAL_GPIO_BUTTON_PRESSED) {
                _state = State::LED2_OFF;
            } else {
                _state = State::ON;
            }
            break;

        case State::LED2_OFF:
            // Turn on LED4
            hal.gpio->write(HAL_GPIO_B_LED_PIN, HAL_GPIO_LED_OFF);

            // Check button
            if (hal.gpio->read(HAL_GPIO_BUTTON_PIN) == HAL_GPIO_BUTTON_PRESSED) {
                _state = State::LED1_OFF;
            } else {
                _state = State::ON;
            }
            break;

        case State::LED1_OFF:
            // Turn on LED4
            hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_OFF);

            // Check button
            if (hal.gpio->read(HAL_GPIO_BUTTON_PIN) == HAL_GPIO_BUTTON_PRESSED) {
                _state = State::TURNING_OFF;
            } else {
                _state = State::ON;
            }
            break;

        case State::TURNING_OFF:
            // turn off all LEDs
            hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_OFF);
            hal.gpio->write(HAL_GPIO_B_LED_PIN, HAL_GPIO_LED_OFF);
            hal.gpio->write(HAL_GPIO_C_LED_PIN, HAL_GPIO_LED_OFF);
            hal.gpio->write(HAL_GPIO_D_LED_PIN, HAL_GPIO_LED_OFF);
            
            _state = State::WAIT_BUTTON_RELEASED_OFF;
            break;

        case State::WAIT_BUTTON_RELEASED_OFF:
            // should be impossible to get here armed, but still better safe than sorry
            if (AP_Notify::flags.armed) {
                _state = State::ON;
                AP_Notify::flags.powering_off = false;
                break;
            }

            // Wait for button to be released
            if (hal.gpio->read(HAL_GPIO_BUTTON_PIN) == HAL_GPIO_BUTTON_PRESSED) {
                break;
            }

            // Turn main systems OFF
            AP_Notify::flags.powering_off = true;
            hal.gpio->write(HAL_GPIO_MAIN_POWER_PIN, HAL_GPIO_POWER_OFF); 

            _state = State::DIE_HERE;
            break;

        case State::DIE_HERE:
            // turn ON if button is pressed again (useful only if USB is connected)
            if (hal.gpio->read(HAL_GPIO_BUTTON_PIN) == HAL_GPIO_BUTTON_PRESSED) {
                // Turn main systems ON
                AP_Notify::flags.powering_off = false;
                hal.gpio->write(HAL_GPIO_MAIN_POWER_PIN, !HAL_GPIO_POWER_OFF); 
                _state = State::START;
            }
            break;

        case State::CHECK_VOLTAGE:
            // turn ON if button is pressed 
            if (hal.gpio->read(HAL_GPIO_BUTTON_PIN) == HAL_GPIO_BUTTON_PRESSED) {
                _state = State::START;
            }

            // Update LED based on battery voltage
            if (_check_voltage_ms == 0) {
                _check_voltage_ms = now_ms;
            } else if (now_ms - _check_voltage_ms > 3000) {
                // turn off after 3 seconds
                _check_voltage_ms = 0;
                _state = State::TURNING_OFF;
                // we don't need anymore LED priority
                AP_Notify::flags.align_led_priority = false;
            }
            set_led_from_voltage();
            break;

        default:
            break;
    }
}

void AP_BoardLED_Align::set_led_from_voltage()
{
    float voltage = batt.voltage(0);

    // control LED based on battery voltage
    /* if (voltage < 1.0f) { */
    /*     // battery monitor is not ready */
    /*     return; */
    /* } */
    if (voltage < ALIGN_LED_BATT_20) {
        // toggle works only if first all LEDs are ON
        if (led_flash_first) {
            led_flash_first = false;
            hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_ON);
            hal.gpio->write(HAL_GPIO_B_LED_PIN, HAL_GPIO_LED_ON);
            hal.gpio->write(HAL_GPIO_C_LED_PIN, HAL_GPIO_LED_ON);
            hal.gpio->write(HAL_GPIO_D_LED_PIN, HAL_GPIO_LED_ON);
        } else {
            // flash all LEDs
            hal.gpio->toggle(HAL_GPIO_A_LED_PIN);
            hal.gpio->toggle(HAL_GPIO_B_LED_PIN);
            hal.gpio->toggle(HAL_GPIO_C_LED_PIN);
            hal.gpio->toggle(HAL_GPIO_D_LED_PIN);
        }
    } else if (voltage < ALIGN_LED_BATT_40) {
        led_flash_first = true;
        hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_ON);
        hal.gpio->write(HAL_GPIO_B_LED_PIN, HAL_GPIO_LED_OFF);
        hal.gpio->write(HAL_GPIO_C_LED_PIN, HAL_GPIO_LED_OFF);
        hal.gpio->write(HAL_GPIO_D_LED_PIN, HAL_GPIO_LED_OFF);
    } else if (voltage < ALIGN_LED_BATT_60) {
        led_flash_first = true;
        hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_ON);
        hal.gpio->write(HAL_GPIO_B_LED_PIN, HAL_GPIO_LED_ON);
        hal.gpio->write(HAL_GPIO_C_LED_PIN, HAL_GPIO_LED_OFF);
        hal.gpio->write(HAL_GPIO_D_LED_PIN, HAL_GPIO_LED_OFF);
    } else if (voltage < ALIGN_LED_BATT_80) {
        led_flash_first = true;
        hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_ON);
        hal.gpio->write(HAL_GPIO_B_LED_PIN, HAL_GPIO_LED_ON);
        hal.gpio->write(HAL_GPIO_C_LED_PIN, HAL_GPIO_LED_ON);
        hal.gpio->write(HAL_GPIO_D_LED_PIN, HAL_GPIO_LED_OFF);
    } else {
        led_flash_first = true;
        hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_ON);
        hal.gpio->write(HAL_GPIO_B_LED_PIN, HAL_GPIO_LED_ON);
        hal.gpio->write(HAL_GPIO_C_LED_PIN, HAL_GPIO_LED_ON);
        hal.gpio->write(HAL_GPIO_D_LED_PIN, HAL_GPIO_LED_ON);
    }

}
#else
bool AP_BoardLED_Align::init(void) {return true;}
void AP_BoardLED_Align::update(void) {return;}
#endif
