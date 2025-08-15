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
#include "AP_Follow_config.h"

#if AP_FOLLOW_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AC_PID/AC_P.h>
#include <AP_RTC/JitterCorrection.h>

class AP_Follow_Mount
{

public:

    // constructor
    AP_Follow_Mount();

    // enable as singleton
    static AP_Follow_Mount *get_singleton(void) {
        return _singleton;
    }

    // returns true if library is enabled
    bool enabled() const { return _enabled; }

    // restore offsets to zero if necessary, should be called when vehicle exits follow mode
    void clear_offsets_if_required();

    //
    // position tracking related methods
    //

    // true if we have a valid target location estimate
    bool have_target();

    // get target's estimated location and velocity (in NED)
    bool get_target_yaw_error(float &yaw_error_rad);

    // get position controller.  this controller is not used within this library but it is convenient to hold it here
    const AC_P& get_forward_p() const { return _p_x; }
    const AC_P& get_yaw_p() const { return _p_yaw; }

    // mount based follow me struct
    struct PACKED MountTracking {
        uint32_t last_update_ms;
        float yaw_error_rad;
        float pitch_error_rad;
        uint16_t crop_width;
        uint16_t crop_height;
        uint16_t distance_cm;
    };

    bool get_mount_tracking(MountTracking &tracking) const;
    bool set_mount_tracking(const MountTracking &tracking) const;
    bool set_is_tracking(bool tracking) const;

    // parameter list
    static const struct AP_Param::GroupInfo var_info[];


private:
    static AP_Follow_Mount *_singleton;

    // parameters
    AP_Int8     _enabled;           // 1 if this subsystem is enabled
    AC_P        _p_x;             // position error P controller
    AC_P        _p_yaw;             // position error P controller

    // local variables
    mutable MountTracking _mount_tracking;
    mutable bool _is_tracking;
    mutable HAL_Semaphore _tracking_mutex; // semaphore for mount tracking
};

namespace AP {
    AP_Follow_Mount &follow_mount();
};

#endif
