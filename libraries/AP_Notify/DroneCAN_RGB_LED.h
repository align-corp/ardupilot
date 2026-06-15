#pragma once

#include "RGBLed.h"
#if HAL_ENABLE_DRONECAN_DRIVERS

#include <AP_DroneCAN/AP_DroneCAN.h>

class DroneCAN_RGB_LED: public RGBLed {
public:
    DroneCAN_RGB_LED(uint8_t led_off, uint8_t led_full,
                   uint8_t led_medium, uint8_t led_dim);
    DroneCAN_RGB_LED();
    bool init() override;
#ifdef ALIGN_PCU_CAN
    void update() override;
#endif
protected:
    virtual bool hw_set_rgb(uint8_t red, uint8_t green, uint8_t blue) override;
private:
    bool broadcast(uavcan_equipment_indication_LightsCommand &msg);
#ifdef ALIGN_PCU_CAN
    uint8_t _last_led_bitmask;
    bool _bitmask_sent;        // false until the first bitmask has been broadcast
    // (members initialised in the constructor)
#endif // ALIGN_PCU_CAN
};
#endif
