#include "Copter.h"

#if AP_LANDINGGEAR_ENABLED

// Run landing gear controller at 10Hz
void Copter::landinggear_update()
{
#ifndef AP_LANDINGGEAR_GPIO
    // exit immediately if no landing gear output has been enabled
    if (!SRV_Channels::function_assigned(SRV_Channel::k_landing_gear_control)) {
        return;
    }
#endif

    // don't automatic deploy/retract landing gear if in RTL or LAND
    if (flightmode->is_landing()) {
        return;
    }

    // support height based triggering using rangefinder or altitude above ground
    int32_t height_cm = flightmode->get_alt_above_ground_cm();

    // use rangefinder if available
    switch (rangefinder.status_orient(ROTATION_PITCH_270)) {
    case RangeFinder::Status::NotConnected:
    case RangeFinder::Status::NoData:
        // use altitude above home for non-functioning rangefinder
        break;

    case RangeFinder::Status::OutOfRangeLow:
        // altitude is close to zero (gear should deploy)
        height_cm = 0;
        break;

    case RangeFinder::Status::OutOfRangeHigh:
        // altitude is at least max rangefinder distance
        height_cm = rangefinder.max_distance_cm_orient(ROTATION_PITCH_270);
        break;
        
    case RangeFinder::Status::Good:
        // use last good reading
        height_cm = rangefinder_state.alt_cm_filt.get();
        break;
    }

    landinggear.update(height_cm * 0.01f); // convert cm->m for update call
}

#endif // AP_LANDINGGEAR_ENABLED
