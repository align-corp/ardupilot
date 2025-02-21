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

#include "AP_Notify.h"
#include "AP_BattMonitor/AP_BattMonitor.h"


#if (defined(HAL_GPIO_A_LED_PIN) && defined(HAL_GPIO_B_LED_PIN) && \
     defined(HAL_GPIO_C_LED_PIN) && defined(HAL_GPIO_D_LED_PIN) && \
     defined(HAL_GPIO_BUTTON_PIN))

static_assert((HAL_GPIO_A_LED_PIN != HAL_GPIO_B_LED_PIN) &&
              (HAL_GPIO_A_LED_PIN != HAL_GPIO_C_LED_PIN) &&
              (HAL_GPIO_A_LED_PIN != HAL_GPIO_D_LED_PIN) &&
              (HAL_GPIO_B_LED_PIN != HAL_GPIO_D_LED_PIN) &&
              (HAL_GPIO_B_LED_PIN != HAL_GPIO_C_LED_PIN) &&
              (HAL_GPIO_D_LED_PIN != HAL_GPIO_C_LED_PIN), "Duplicate LED assignments detected");

#ifndef HAL_GPIO_BUTTON_PRESSED
#define HAL_GPIO_BUTTON_PRESSED 1
#endif

#ifndef LED_BATT_VOLT_MAX
#define LED_BATT_VOLT_MAX 12.6f
#endif

#ifndef LED_BATT_VOLT_MIN
#define LED_BATT_VOLT_MIN 10.5f
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
    return true;
}

/*
  main update function called at 50Hz
 */
void AP_BoardLED_Align::update(void)
{
    // update at 10 Hz
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

            // Turn all LED off
            hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_OFF);
            hal.gpio->write(HAL_GPIO_B_LED_PIN, HAL_GPIO_LED_OFF);
            hal.gpio->write(HAL_GPIO_C_LED_PIN, HAL_GPIO_LED_OFF);
            hal.gpio->write(HAL_GPIO_D_LED_PIN, HAL_GPIO_LED_OFF);
            
            // Check button
            if (hal.gpio->read(HAL_GPIO_BUTTON_PIN) == HAL_GPIO_BUTTON_PRESSED) {
                _state = State::LED1_ON;
            } else {
                _state = State::TURNING_OFF;
            }
            break;

        case State::LED1_ON:
            // Turn on LED1
            hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_ON);
            
            // Check button
            if (hal.gpio->read(HAL_GPIO_BUTTON_PIN) == HAL_GPIO_BUTTON_PRESSED) {
                _state = State::LED2_ON;
            } else {
                _state = State::TURNING_OFF;
            }
            break;

        case State::LED2_ON:
            // Turn on LED2
            hal.gpio->write(HAL_GPIO_B_LED_PIN, HAL_GPIO_LED_ON);
            
            // Check button
            if (hal.gpio->read(HAL_GPIO_BUTTON_PIN) == HAL_GPIO_BUTTON_PRESSED) {
                _state = State::LED3_ON;
            } else {
                _state = State::TURNING_OFF;
            }
            break;

        case State::LED3_ON:
            // Turn on LED3
            hal.gpio->write(HAL_GPIO_C_LED_PIN, HAL_GPIO_LED_ON);
            
            // Check button
            if (hal.gpio->read(HAL_GPIO_BUTTON_PIN) == HAL_GPIO_BUTTON_PRESSED) {
                _state = State::LED4_ON;
            } else {
                _state = State::TURNING_OFF;
            }
            break;

        case State::LED4_ON:
            // Turn on LED4
            hal.gpio->write(HAL_GPIO_D_LED_PIN, HAL_GPIO_LED_ON);

            // Check button
            if (hal.gpio->read(HAL_GPIO_BUTTON_PIN) == HAL_GPIO_BUTTON_PRESSED) {
                _state = State::TURNING_ON;
            } else {
                _state = State::TURNING_OFF;
            }
            break;

        case State::TURNING_ON:
            // TODO: turn on 5 V, 12 V and ESC

            // Wait button to be released
            _state = State::WAIT_BUTTON_RELEASED;

            break;

        case State::WAIT_BUTTON_RELEASED:
            // Wait for button to be released
            if (hal.gpio->read(HAL_GPIO_BUTTON_PIN) == HAL_GPIO_BUTTON_PRESSED) {
                break;
            }

            _state = State::ON;
            break;

        case State::ON:
            // Do nothing if drone is armed
            if (AP_Notify::flags.armed) {
                break;
            }
            
            // Update LED based on battery voltage
            set_led_from_voltage();
            /* hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_ON); */
            /* hal.gpio->write(HAL_GPIO_B_LED_PIN, HAL_GPIO_LED_ON); */
            /* hal.gpio->write(HAL_GPIO_C_LED_PIN, HAL_GPIO_LED_ON); */

            // Turn off if button is pressed
            if (hal.gpio->read(HAL_GPIO_BUTTON_PIN) == HAL_GPIO_BUTTON_PRESSED) {
                // Turn all LED on
                hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_ON);
                hal.gpio->write(HAL_GPIO_B_LED_PIN, HAL_GPIO_LED_ON);
                hal.gpio->write(HAL_GPIO_C_LED_PIN, HAL_GPIO_LED_ON);
                hal.gpio->write(HAL_GPIO_D_LED_PIN, HAL_GPIO_LED_ON);
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
            // should be impossible to get here armed, but still better safe than sorry
            if (AP_Notify::flags.armed) {
                _state = State::ON;
                AP_Notify::flags.powering_off = false;
                break;
            }

            // turn off all LEDs
            hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_OFF);
            hal.gpio->write(HAL_GPIO_B_LED_PIN, HAL_GPIO_LED_OFF);
            hal.gpio->write(HAL_GPIO_C_LED_PIN, HAL_GPIO_LED_OFF);
            hal.gpio->write(HAL_GPIO_D_LED_PIN, HAL_GPIO_LED_OFF);
            
            // Turn main systems OFF
            AP_Notify::flags.powering_off = true;

            break;

        default:
            break;
    }
}

void AP_BoardLED_Align::set_led_from_voltage()
{
    const float volt_step = (LED_BATT_VOLT_MAX - LED_BATT_VOLT_MIN) / 4.0f;
    float voltage = batt.voltage(0);
    static bool  first_low = true;

    // control LED based on battery voltage
    /* if (voltage < 1.0f) { */
    /*     // battery monitor is not ready */
    /*     return; */
    /* } */
    if (voltage < LED_BATT_VOLT_MIN) {
        if (first_low) {
            first_low = false;
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
    } else if (voltage < LED_BATT_VOLT_MIN + volt_step) {
        first_low = true;
        hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_ON);
        hal.gpio->write(HAL_GPIO_B_LED_PIN, HAL_GPIO_LED_OFF);
        hal.gpio->write(HAL_GPIO_C_LED_PIN, HAL_GPIO_LED_OFF);
        hal.gpio->write(HAL_GPIO_D_LED_PIN, HAL_GPIO_LED_OFF);
    } else if (voltage < LED_BATT_VOLT_MIN + 2 * volt_step) {
        first_low = true;
        hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_ON);
        hal.gpio->write(HAL_GPIO_B_LED_PIN, HAL_GPIO_LED_ON);
        hal.gpio->write(HAL_GPIO_C_LED_PIN, HAL_GPIO_LED_OFF);
        hal.gpio->write(HAL_GPIO_D_LED_PIN, HAL_GPIO_LED_OFF);
    } else if (voltage < LED_BATT_VOLT_MIN + 3 * volt_step) {
        first_low = true;
        hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_ON);
        hal.gpio->write(HAL_GPIO_B_LED_PIN, HAL_GPIO_LED_ON);
        hal.gpio->write(HAL_GPIO_C_LED_PIN, HAL_GPIO_LED_ON);
        hal.gpio->write(HAL_GPIO_D_LED_PIN, HAL_GPIO_LED_OFF);
    } else {
        first_low = true;
        hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_ON);
        hal.gpio->write(HAL_GPIO_B_LED_PIN, HAL_GPIO_LED_ON);
        hal.gpio->write(HAL_GPIO_C_LED_PIN, HAL_GPIO_LED_ON);
        hal.gpio->write(HAL_GPIO_D_LED_PIN, HAL_GPIO_LED_ON);
    }

}
#else
bool AP_BoardLED::init(void) {return true;}
void AP_BoardLED::update(void) {return;}
#endif
