#include "Rover.h"

// declare GPIO pins
const uint8_t ALIGN_BUCKET_GPIO_UP[ALIGN_BUCKET_AXIS_NUM] = {1, 3};
const uint8_t ALIGN_BUCKET_GPIO_DOWN[ALIGN_BUCKET_AXIS_NUM] = {2, 4};
const uint8_t ALIGN_BUCKET_PWM_CHAN[ALIGN_BUCKET_AXIS_NUM] = {2, 4};

void Rover::update_align_bucket()
{
#if ALIGN_BUCKET_ENABLED
    uint32_t delay_ms = (uint32_t)(g.align_bucket_delay);
    const uint32_t now_ms = AP_HAL::millis();
    for (uint8_t axis_id = 0; axis_id < ALIGN_BUCKET_AXIS_NUM; axis_id++) {
        // check rc channel input
        RC_Channel *c = rc().channel(ALIGN_BUCKET_PWM_CHAN[axis_id]-1);
        if (c == nullptr) {
            continue;
        }
        uint16_t ch_value = c->get_radio_in();
        if (ch_value < 900 || rc().in_rc_failsafe()) {
            if (align_bucket.state[axis_id] == AlignBucket::State::STOPPED ||
                align_bucket.state[axis_id] == AlignBucket::State::STOP_WAIT) {
                continue;
            } else {
                align_bucket.state[axis_id] = AlignBucket::State::STOP;
            }
        }

        switch (align_bucket.state[axis_id]) {
            case AlignBucket::State::STOP:
                hal.gpio->write(ALIGN_BUCKET_GPIO_UP[axis_id], 0);
                hal.gpio->write(ALIGN_BUCKET_GPIO_DOWN[axis_id], 0);
                align_bucket.state[axis_id] = AlignBucket::State::STOP_WAIT;
                align_bucket.last_ms[axis_id] = now_ms;
                break;

            case AlignBucket::State::STOP_WAIT:
                if (now_ms - align_bucket.last_ms[axis_id] > delay_ms) {
                    align_bucket.state[axis_id] = AlignBucket::State::STOPPED;
                }
                break;

            case AlignBucket::State::STOPPED:
                if (ch_value > 1850) {
                    align_bucket.state[axis_id] = AlignBucket::State::UP;
                } else if (ch_value < 1150) {
                    align_bucket.state[axis_id] = AlignBucket::State::DOWN;
                }
                break;

            case AlignBucket::State::UP:
                hal.gpio->write(ALIGN_BUCKET_GPIO_DOWN[axis_id], 0);
                hal.gpio->write(ALIGN_BUCKET_GPIO_UP[axis_id], 1);
                align_bucket.state[axis_id] = AlignBucket::State::UP_WAIT;
                align_bucket.last_ms[axis_id] = now_ms;
                break;

            case AlignBucket::State::UP_WAIT:
                if (now_ms - align_bucket.last_ms[axis_id] > delay_ms) {
                    align_bucket.state[axis_id] = AlignBucket::State::UPPED;
                }
                break;

            case AlignBucket::State::UPPED:
                if (ch_value < 1150) {
                    align_bucket.state[axis_id] = AlignBucket::State::DOWN;
                } else if (ch_value < 1850) {
                    align_bucket.state[axis_id] = AlignBucket::State::STOP;
                }
                break;

            case AlignBucket::State::DOWN:
                hal.gpio->write(ALIGN_BUCKET_GPIO_UP[axis_id], 0);
                hal.gpio->write(ALIGN_BUCKET_GPIO_DOWN[axis_id], 1);
                align_bucket.state[axis_id] = AlignBucket::State::DOWN_WAIT;
                align_bucket.last_ms[axis_id] = now_ms;
                break;

            case AlignBucket::State::DOWN_WAIT:
                if (now_ms - align_bucket.last_ms[axis_id] > delay_ms) {
                    align_bucket.state[axis_id] = AlignBucket::State::DOWNED;
                }
                break;

            case AlignBucket::State::DOWNED:
                if (ch_value > 1850) {
                    align_bucket.state[axis_id] = AlignBucket::State::UP;
                } else if (ch_value > 1150) {
                    align_bucket.state[axis_id] = AlignBucket::State::STOP;
                }
                break;
        }
    }
#endif
}
