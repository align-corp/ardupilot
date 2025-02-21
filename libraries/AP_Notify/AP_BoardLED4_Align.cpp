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

extern const AP_HAL::HAL& hal;

bool AP_BoardLED::init(void)
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
void AP_BoardLED::update(void)
{
    // update at 10 Hz
    uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _last_update_ms < 100) {
        return;
    }
    _last_update_ms = now_ms;

    if (_state < State::TURNING_ON) {
        if (hal.gpio    }
    // battery light state machine
    switch(_state) {
        case State::START:
            // Turn all LED off
            hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_OFF);
            hal.gpio->write(HAL_GPIO_B_LED_PIN, HAL_GPIO_LED_OFF);
            hal.gpio->write(HAL_GPIO_C_LED_PIN, HAL_GPIO_LED_OFF);
            hal.gpio->write(HAL_GPIO_D_LED_PIN, HAL_GPIO_LED_OFF);
            
            // Check button
            if (hal.gpio->read(HAL_GPIO_BUTTON_PIN) != HAL_GPIO_BUTTON_PRESSED) {
                _state = State::TURNING_OFF;
            } else {
                _state = State::LED1_ON;
            }
            break;

        case State::LED1_ON:
            // Turn on LED1
            hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_ON);
            
            // Check button
            if (hal.gpio->read(HAL_GPIO_BUTTON_PIN) != HAL_GPIO_BUTTON_PRESSED) {
                _state = State::TURNING_OFF;
            } else {
                _state = State::LED2_ON;
            }
            break;

        case State::LED2_ON:
            // Turn on LED2
            hal.gpio->write(HAL_GPIO_B_LED_PIN, HAL_GPIO_LED_ON);
            
            // Check button
            if (hal.gpio->read(HAL_GPIO_BUTTON_PIN) != HAL_GPIO_BUTTON_PRESSED) {
                _state = State::TURNING_OFF;
            } else {
                _state = State::LED3_ON;
            }
            break;

        case State::LED3_ON:
            // Turn on LED3
            hal.gpio->write(HAL_GPIO_C_LED_PIN, HAL_GPIO_LED_ON);
            
            // Check button
            if (hal.gpio->read(HAL_GPIO_BUTTON_PIN) != HAL_GPIO_BUTTON_PRESSED) {
                _state = State::TURNING_OFF;
            } else {
                _state = State::LED4_ON;
            }
            break;

        case State::LED4_ON:
            // Turn on LED4
            hal.gpio->write(HAL_GPIO_D_LED_PIN, HAL_GPIO_LED_ON);

            // Check button
            if (hal.gpio->read(HAL_GPIO_BUTTON_PIN) != HAL_GPIO_BUTTON_PRESSED) {
                _state = State::TURNING_OFF;
            } else {
                _state = State::TURNING_ON;
            }
            break;

        case State::TURNING_ON:

            break;

        case State::ON:
            // Do nothing if drone is armed
            if (AP_Notify::flags.armed) {
                break;
            }

            if (hal.gpio->read(HAL_GPIO_BUTTON_PIN) == HAL_GPIO_BUTTON_PRESSED) {
                _state = State::LED4_OFF;
            }

        default:
            break;
    }

    // arming light
    static uint8_t arm_counter = 0;
	if (AP_Notify::flags.armed) {
        // red led solid
        hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_ON);
    }else{
        if ((counter2 & 0x2) == 0) {
            arm_counter++;
        }
        if (AP_Notify::flags.pre_arm_check) {
            // passed pre-arm checks so slower single flash
            switch(arm_counter) {
                case 0:
                case 1:
                case 2:
                    hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_ON);
                    break;
                case 3:
                case 4:
                case 5:
                    hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_OFF);
                    break;
                default:
                    // reset counter to restart the sequence
                    arm_counter = -1;
                    break;
            }
        }else{
            // failed pre-arm checks so double flash
            switch(arm_counter) {
                case 0:
                case 1:
                case 3:
                case 4:
                    hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_ON);
                    break;

                case 2:
                case 5:
                case 6:
                    hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_OFF);
                    break;

                default:
                    arm_counter = -1;
                    break;
            }
        }
    }

    // gps light
    switch (AP_Notify::flags.gps_status) {
        case 0:
            // no GPS attached
            hal.gpio->write(HAL_GPIO_C_LED_PIN, HAL_GPIO_LED_OFF);
            break;

        case 1:
            // GPS attached but no lock, blink at 4Hz
            if ((counter2 & 0x3) == 0) {
                hal.gpio->toggle(HAL_GPIO_C_LED_PIN);
            }
            break;

        case 2:
            // GPS attached but 2D lock, blink more slowly (around 2Hz)
            if ((counter2 & 0x7) == 0) {
                hal.gpio->toggle(HAL_GPIO_C_LED_PIN);
            }
            break;

        default:
            // solid blue on gps lock
            hal.gpio->write(HAL_GPIO_C_LED_PIN, HAL_GPIO_LED_ON);
            break;        
    }
}
#else
bool AP_BoardLED::init(void) {return true;}
void AP_BoardLED::update(void) {return;}
#endif
