#include "Copter.h"

#if HAL_SPRAYER_ENABLED

// Run sprayer controller
void Copter::sprayer_update()
{
    // pass rangefinder altitude to sprayer update function
    int32_t height_cm;
    if (!get_rangefinder_height_interpolated_cm(height_cm)) {
        height_cm = INT32_MAX;
    }
    sprayer.update_copter(height_cm);
#if HAL_SPRAYER2_ENABLED
    sprayer2.update_copter(height_cm);
#endif
}

#endif
