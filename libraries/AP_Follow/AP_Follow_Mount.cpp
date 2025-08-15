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

#include "AP_Follow_config.h"

#if AP_FOLLOW_ENABLED

#include <AP_HAL/AP_HAL.h>
#include "AP_Follow_Mount.h"
#include <ctype.h>
#include <stdio.h>

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

extern const AP_HAL::HAL& hal;

#define AP_FOLLOW_TIMEOUT_MS    3000    // position estimate timeout after 1 second
#define AP_FOLLOW_MOUNT_YAW_DEFAULT 0.8f
#define AP_FOLLOW_MOUNT_X_DEFAULT 0.2f

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
#define AP_FOLLOW_ALT_TYPE_DEFAULT 0
#else
#define AP_FOLLOW_ALT_TYPE_DEFAULT AP_FOLLOW_ALTITUDE_TYPE_RELATIVE
#endif

AP_Follow_Mount *AP_Follow_Mount::_singleton;

// table of user settable parameters
const AP_Param::GroupInfo AP_Follow_Mount::var_info[] = {

    // @Param: _ENABLE
    // @DisplayName: Follow enable/disable
    // @Description: Enabled/disable following a target
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO_FLAGS("_ENABLE", 1, AP_Follow_Mount, _enabled, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: _X_P
    // @DisplayName: Follow position error P gain
    // @Description: Follow position error P gain.  Converts the difference between desired vertical speed and actual speed into a desired acceleration that is passed to the throttle acceleration controller
    // @Range: 0.01 1.00
    // @Increment: 0.01
    // @User: Standard
    AP_SUBGROUPINFO(_p_x, "_X_", 2, AP_Follow_Mount, AC_P),

    // @Param: _YAW_P
    // @DisplayName: Follow position error P gain
    // @Description: Follow position error P gain.  Converts the difference between desired vertical speed and actual speed into a desired acceleration that is passed to the throttle acceleration controller
    // @Range: 0.01 1.00
    // @Increment: 0.01
    // @User: Standard
    AP_SUBGROUPINFO(_p_yaw, "_YAW_", 3, AP_Follow_Mount, AC_P),

    AP_GROUPEND
};

/* 
   The constructor also initialises the proximity sensor. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the proximity sensor
*/
AP_Follow_Mount::AP_Follow_Mount() :
        _p_x(AP_FOLLOW_MOUNT_X_DEFAULT ), _p_yaw(AP_FOLLOW_MOUNT_YAW_DEFAULT )
{
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
}

// get target's estimated location
bool AP_Follow_Mount::get_target_yaw_error(float &yaw_error_rad) 
{
    // exit immediately if not enabled
    if (!_enabled) {
        return false;
    }

    // are we tracking?
    if (!have_target()) {
        return false;
    }


    // get mutex and copy tracking data we need
    if (!_tracking_mutex.take(1)) return false;
    uint32_t last_update_ms = _mount_tracking.last_update_ms;
    yaw_error_rad = _mount_tracking.yaw_error_rad;
    _tracking_mutex.give();

    // check for timeout
    if ((last_update_ms == 0) || (AP_HAL::millis() - last_update_ms > AP_FOLLOW_TIMEOUT_MS)) {
        return false;
    }

    return true;
}

// return true if we have a target
bool AP_Follow_Mount::have_target(void)
{
    if (!_enabled) {
        return false;
    }

    // get mutex and copy last update time and if we're tracking
    if (!_tracking_mutex.take(1)) return false;
    uint32_t last_update_ms = _mount_tracking.last_update_ms;
    bool tracking = _is_tracking;
    _tracking_mutex.give();

    if (!tracking) {
        return false;
    }

    // check for timeout
    if ((last_update_ms == 0) || (AP_HAL::millis() - last_update_ms > AP_FOLLOW_TIMEOUT_MS)) {
        return false;
    }
    return true;
}

// get mount tracking
bool AP_Follow_Mount::get_mount_tracking(MountTracking &tracking) const
{
    // acquire mutex
    if (!_tracking_mutex.take(1)) return false;
    tracking = _mount_tracking;
    _tracking_mutex.give();
    return true;
}

// set mount tracking
bool AP_Follow_Mount::set_mount_tracking(const MountTracking &tracking) const
{
    // acquire mutex
    if (!_tracking_mutex.take(1)) return false;
    _mount_tracking = tracking;
    _is_tracking = true;
    _tracking_mutex.give();
    return true;
}

bool AP_Follow_Mount::set_is_tracking(bool tracking) const
{
    // acquire mutex
    if (!_tracking_mutex.take(1)) return false;
    _is_tracking = tracking;
    _tracking_mutex.give();
    return true;
}

namespace AP {

AP_Follow_Mount &follow_mount()
{
    return *AP_Follow_Mount::get_singleton();
}

}

#endif  // AP_FOLLOW_ENABLED
