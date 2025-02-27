#include "Copter.h"

#if HAL_SPRAYER_ENABLED

// Run sprayer controller
void Copter::sprayer_update()
{
    // pass terrain altitude to sprayer update function
    int32_t height_cm = flightmode->get_alt_above_ground_cm();
    sprayer.update_copter(height_cm);
}

#endif
