-- switches between AHRS/EKF sources based on the pilot's source selection switch or using an automatic source selection algorithm
-- this script is intended to help vehicles automatically switch between GPS and optical flow
--
-- configure a forward or downward facing lidar with a range of at least 5m
-- setup RCx_OPTION = 90 (EKF Pos Source) to select the source (low=GPS, middle=opticalflow, high=Not Used)
-- setup RCx_OPTION = 300 (Scripting1).  When this switch is pulled high, the source will be automatically selected
-- SRC_ENABLE = 1 (enable scripting)
-- setup EK3_SRCn_ parameters so that GPS is the primary source, opticalflow is secondary.
--     EK3_SRC1_POSXY = 3 (GPS)
--     EK3_SRC1_VELXY = 3 (GPS)
--     EK3_SRC1_VELZ  = 3 (GPS)
--     EK3_SRC1_POSZ  = 1 (Baro)
--     EK3_SRC1_YAW   = 1 (Compass)
--     EK3_SRC2_POSXY = 0 (None)
--     EK3_SRC2_VELXY = 5 (OpticalFlow)
--     EK3_SRC2_VELZ  = 0 (None)
--     EK3_SRC2_POSZ  = 1 (Baro)
--     EK3_SRC2_YAW   = 1 (Compass)
--     EK3_SRC_OPTIONS    = 0 (Do not fuse all velocities)
--
-- SCR_USER1 holds the threshold (in meters) for rangefinder altitude (around 15 is a good choice)
--     if rangefinder distance >= SCR_USER1, source1 (GPS) will be used
--     if rangefinder distance < SCR_USER1, source2 (optical flow) will be used if innovations are below SRC_USER3 value
-- SCR_USER2 holds the threshold for GPS speed accuracy (around 0.3 is a good choice)
-- SCR_USER3 holds the threshold for optical flow quality (about 50 is a good choice)
-- SCR_USER4 holds the threshold for optical flow innovations (about 0.15 is a good choice)
--
-- When the 2nd auxiliary switch (300/Scripting1) is pulled high automatic source selection uses these thresholds:
-- luacheck: only 0

-- constants
local GPS_HDOP_GOOD = 95      -- HDOP below this value is considered good [cm]
local GPS_MINSATS_GOOD = 17   -- number of satellites above this value is considered good
local GPS_HDOP_USABLE = 120   -- HDOP below this value is considered usable [cm]
local GPS_MINSATS_USABLE = 12 -- number of satellites above this value is considered usable
local EKF_SRC_GPS = 0
local EKF_SRC_OPTICALFLOW = 1
local EKF_SRC_UNDECIDED = -1
local RNG_ROTATION_DOWN = 25
local VOTE_COUNT_MAX = 20 -- when a vote counter reaches this number (i.e. 2sec) source may be switched

-- variables
local source_prev = EKF_SRC_OPTICALFLOW -- opticalflow for takeoff
local gps_vs_opticalflow_vote = 0       -- vote counter for GPS vs optical (-20 = GPS, +20 = optical flow)
local optical_flow_dangerous_count = 0  -- count of dangerous optical flow quality
local opticalflow_state_dangerous = false
local led_on_count = 0
local last_send_ekf_source = uint32_t(0)

local PARAM_TABLE_KEY = 81
PARAM_TABLE_PREFIX = "FLGP_"
local MAV_SEVERITY = { EMERGENCY = 0, ALERT = 1, CRITICAL = 2, ERROR = 3, WARNING = 4, NOTICE = 5, INFO = 6, DEBUG = 7 }

-- bind a parameter to a variable
function bind_param(name)
    local p = Parameter()
    assert(p:init(name), string.format('could not find %s parameter', name))
    return p
end

-- add a parameter and bind it to a variable
local function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return bind_param(PARAM_TABLE_PREFIX .. name)
end

-- add param table
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 5), 'ahrs-source-gps-optflow: could not add param table')

-- Enable 0: Disabled (always use SRC1=GPS), 1: Automatic, 2: Disabled (Always use SRC2=OpticalFlow)
local FLGP_ENABLE = bind_add_param('ENABLE', 1, 1)

-- EKF Source OpticalFlow Innovation Threshold
local FLGP_FLOW_THRESH = bind_add_param('FLOW_THRESH', 2, 0.5)

-- OpticalFlow may be used if quality is above this threshold
local FLGP_FLOW_QUAL = bind_add_param('FLOW_QUAL', 3, 60)

-- Down LED control 1:Automatic, 2:Always ON, 0:Always OFF
local FLGP_LED = bind_add_param('LED', 5, 1)

-- use SCR_USER6 for A10 control via RC switch
local SCR_USER6 = bind_param('SCR_USER6')

assert(optical_flow, 'could not access optical flow')

-- get roll and pitch channels
local chan_roll = rc:get_channel(1)
local chan_pitch = rc:get_channel(2)
local chan_yaw = rc:get_channel(4)

-- the main update function
function update()
    -- enable check: -1 disable, 0 always use GPS, 1 automatic, 2 always use OpticalFlow
    if FLGP_ENABLE:get() < 0 then
        return update, 100
    elseif FLGP_ENABLE:get() < 1 then
        if source_prev ~= EKF_SRC_GPS then
            source_prev = EKF_SRC_GPS
            ahrs:set_posvelyaw_source_set(source_prev) -- switch to GPS
            gcs:send_text(0, "FLGP disabled: switched to Source " .. string.format("%d", source_prev + 1))
        end
        return update, 100
    elseif FLGP_ENABLE:get() > 1 then
        if source_prev ~= EKF_SRC_OPTICALFLOW then
            source_prev = EKF_SRC_OPTICALFLOW
            ahrs:set_posvelyaw_source_set(source_prev) -- switch to optical flow
            gcs:send_text(0, "FLGP disabled: switched to Source " .. string.format("%d", source_prev + 1))
        end
        return update, 100
    end

    -- use SCR_USER6 for A10 control via RC switch (pwm CH8)
    if SCR_USER6:get() == 1 then
        if rc:get_pwm(8) < 1300 then
            -- always use GPS
            if source_prev ~= EKF_SRC_GPS then
                source_prev = EKF_SRC_GPS
                ahrs:set_posvelyaw_source_set(source_prev) -- switch to GPS
                gcs:send_text(0, "FLGP disabled: switched to Source " .. string.format("%d", source_prev + 1))
            end
            return update, 100
        end
    end

    -- check optical flow quality threshold has been set
    local opticalflow_quality_thresh = FLGP_FLOW_QUAL:get() -- SCR_USER3 holds opticalflow quality
    if (opticalflow_quality_thresh <= 0) then
        gcs:send_text(0, "ahrs-source-gps-optflow.lua: set SCR_USER3 to OpticalFlow quality threshold")
        return update, 1000
    end

    -- check optical flow innovation threshold has been set
    local opticalflow_innov_thresh = FLGP_FLOW_THRESH:get() -- SCR_USER4 holds opticalflow innovation
    if (opticalflow_innov_thresh <= 0) then
        gcs:send_text(0, "ahrs-source-gps-optflow.lua: set SCR_USER4 to OpticalFlow innovation threshold")
        return update, 1000
    end

    -- check if GPS is usable or if it's good
    local gps_hdop = gps:get_hdop(gps:primary_sensor())
    local gps_nsats = gps:num_sats(gps:primary_sensor())
    local gps_fix = gps:status(gps:primary_sensor())
    local gps_usable = false
    local gps_good = false
    if gps_hdop ~= nil and gps_nsats ~= nil and gps_fix ~= nil then
        gps_usable = (gps_hdop <= GPS_HDOP_USABLE) and
            (gps_nsats >= GPS_MINSATS_USABLE) and
            (gps_fix >= 3)
        gps_good = (gps_hdop <= GPS_HDOP_GOOD) and
            (gps_nsats >= GPS_MINSATS_GOOD) and
            (gps_fix >= 3)
    end

    -- check optical flow quality
    local opticalflow_quality_good = false
    local opticalflow_quality_dangerous = true
    if (optical_flow) then
        opticalflow_quality_good = (optical_flow:enabled() and optical_flow:healthy() and optical_flow:quality() >= opticalflow_quality_thresh)
        opticalflow_quality_dangerous = optical_flow:quality() < opticalflow_quality_thresh - 30
    end

    -- get opticalflow innovations from ahrs (only x and y values are valid, no variances available)
    local opticalflow_innovation_good = false
    local opticalflow_innovation_dangerous = false
    local opticalflow_xy_innov = 0
    local opticalflow_innov = Vector3f()
    local opticalflow_var = Vector3f()
    opticalflow_innov, opticalflow_var = ahrs:get_vel_innovations_and_variances_for_source(5)
    if (opticalflow_innov) then
        opticalflow_xy_innov = math.sqrt(opticalflow_innov:x() * opticalflow_innov:x() +
        opticalflow_innov:y() * opticalflow_innov:y())
        opticalflow_innovation_good = opticalflow_xy_innov < opticalflow_innov_thresh
        opticalflow_innovation_dangerous = opticalflow_xy_innov > opticalflow_innov_thresh * 2
    end

    -- get rangefinder distance
    local rangefinder_thresh_dist_m = rangefinder:max_distance_cm_orient(RNG_ROTATION_DOWN) * 0.0075
    local rangefinder_thresh_dist_fast_climb_m = rangefinder:max_distance_cm_orient(RNG_ROTATION_DOWN) * 0.006
    local rngfnd_distance_m = 0
    local rngfnd_out_of_range = rangefinder:status_orient(RNG_ROTATION_DOWN) == 3
    if rangefinder:has_data_orient(RNG_ROTATION_DOWN) then
        rngfnd_distance_m = rangefinder:distance_cm_orient(RNG_ROTATION_DOWN) * 0.01
    end
    local rngfnd_over_threshold = (rngfnd_distance_m == 0) or (rngfnd_distance_m > rangefinder_thresh_dist_m)

    -- update led
    led(opticalflow_quality_good, rngfnd_over_threshold, rngfnd_out_of_range, gps_good)

    -- opticalflow is usable if quality and innovations are good and rangefinder is in range
    local opticalflow_usable = opticalflow_quality_good and opticalflow_innovation_good and
        (not rngfnd_over_threshold)
    local opticalflow_dangerous = opticalflow_quality_dangerous or opticalflow_innovation_dangerous

    -- automatic selection logic --

    local auto_source = EKF_SRC_UNDECIDED
    local switch_to_loiter = false

    -- if we switched to altitude_hold due to dangerous optical flow quality, switch back to loiter when quality is good
    if opticalflow_state_dangerous then
        if opticalflow_usable then
            auto_source = EKF_SRC_OPTICALFLOW
            switch_to_loiter = true
        elseif gps_usable then
            auto_source = EKF_SRC_GPS
            switch_to_loiter = true
        end

    -- altitude hold and stabilize: don't use opticalflow if rangefinder is out of range
    -- this is needed, otherwise EKF set to optical flow prevent vehicle to climb higher than 0.8*RNGFND_MAX_DIST
    elseif vehicle:get_mode() <= 2 then
        if rngfnd_distance_m > rangefinder_thresh_dist_fast_climb_m then
            -- immediately switch to GPS if we're over rangefinder_thresh_dist_fast_climb_m
            auto_source = EKF_SRC_GPS
            gps_vs_opticalflow_vote = -VOTE_COUNT_MAX
        elseif opticalflow_usable then
            -- vote for opticalflow, to prevent switching up and down between GPS and opticalflow
            gps_vs_opticalflow_vote = gps_vs_opticalflow_vote + 1
            if gps_vs_opticalflow_vote >= VOTE_COUNT_MAX then
                auto_source = EKF_SRC_OPTICALFLOW
                gps_vs_opticalflow_vote = VOTE_COUNT_MAX
            end
        end

    -- modes that requires position
    else
        -- here we have a loooot of conditions:
        -- 1 -- prevent EKF failsafe immediately switching to GPS if we're getting out of rangefinder range
        local velocity_ned = ahrs:get_velocity_NED()
        local climb_rate = 0
        if velocity_ned ~= nil then
            climb_rate = -velocity_ned:z() -- m/s
        end
        if (gps_usable) and
            (rngfnd_distance_m > rangefinder_thresh_dist_fast_climb_m) and
            (climb_rate > 0.3) and
            (source_prev ~= EKF_SRC_GPS) then
            gps_vs_opticalflow_vote = -VOTE_COUNT_MAX
            source_prev = EKF_SRC_GPS
            ahrs:set_posvelyaw_source_set(source_prev)
            gcs:send_text(MAV_SEVERITY.INFO, "Fast climb detected, switch to GPS")
            -- wait at least two second before switching back to auto mode, to prevent switching back
            -- to optical flow while still climbing
            return update, 2000

        -- 2 -- if pilot is using roll, pitch or yaw, immediately switch to GPS
        elseif gps_good and arming:is_armed() and source_prev == EKF_SRC_OPTICALFLOW and
            (math.abs(chan_roll:norm_input()) > 0.3 or math.abs(chan_pitch:norm_input()) > 0.3 or math.abs(chan_yaw:norm_input()) > 0.3) then
            gps_vs_opticalflow_vote = -VOTE_COUNT_MAX
            optical_flow_dangerous_count = 0

        -- 3 -- if gps isn't good and opticalflow is usable, immediately switch to opticalflow (maybe we're moving inside a building)
        elseif not gps_good and arming:is_armed() and source_prev == EKF_SRC_GPS and opticalflow_usable then
            gps_vs_opticalflow_vote = VOTE_COUNT_MAX
            optical_flow_dangerous_count = 0

        -- 4 -- vote for GPS if it's good. Prioritize GPS over OF
        elseif gps_good then
            gps_vs_opticalflow_vote = gps_vs_opticalflow_vote - 1
            optical_flow_dangerous_count = 0

        -- 5 -- vote for opticalflow if usable
        elseif opticalflow_usable then
            gps_vs_opticalflow_vote = gps_vs_opticalflow_vote + 1
            optical_flow_dangerous_count = 0

        -- 6 -- if we're in loiter and opticalflow quality is dangerously low switch to alt_hold and alert user
        elseif opticalflow_dangerous and arming:is_armed() then
            optical_flow_dangerous_count = optical_flow_dangerous_count + 1
            if (optical_flow_dangerous_count >= 10) then
                optical_flow_dangerous_count = 0
                gcs:send_text(MAV_SEVERITY.ALERT, "AltHold: OpticalFlow quality too low")
                vehicle:set_mode(2)
                opticalflow_state_dangerous = true
            end
        end

        -- auto source vote collation
        if gps_vs_opticalflow_vote <= -VOTE_COUNT_MAX then
            auto_source = EKF_SRC_GPS
            gps_vs_opticalflow_vote = -VOTE_COUNT_MAX
        elseif gps_vs_opticalflow_vote >= VOTE_COUNT_MAX then
            auto_source = EKF_SRC_OPTICALFLOW
            gps_vs_opticalflow_vote = VOTE_COUNT_MAX
        end
    end

    local send_to_gcs_now = false

    -- auto switching
    if (auto_source >= 0) and (auto_source ~= source_prev) then
        source_prev = auto_source
        send_to_gcs_now = true
        ahrs:set_posvelyaw_source_set(source_prev)
        gcs:send_text(MAV_SEVERITY.INFO, "Auto switched to Source " .. string.format("%d", source_prev + 1))
    end

    -- send ekf source to GCS
    send_ekf_source_to_gcs(send_to_gcs_now)

    -- switch back to loiter if opticalflow quality or gps is good
    if switch_to_loiter then
        opticalflow_state_dangerous = false
        gcs:send_text(MAV_SEVERITY.WARNING, "Loiter: OpticalFlow quality recovered")
        if vehicle:get_mode() == 2 then
            vehicle:set_mode(5)
        end
    end
    return update, 100
end

-- LED control
function led(of_quality_acceptable, rng_over_threshold, rng_out_of_range, gps_good)
    if FLGP_LED:get() == 0 then
        -- always OFF
        relay:off(5)
    elseif FLGP_LED:get() == 1 then
        -- automatic mode
        if not arming:is_armed() then
            relay:off(5)
            led_on_count = 0
        elseif rng_out_of_range and source_prev == EKF_SRC_GPS then
            relay:off(5)
            led_on_count = 0
        elseif gps_good and source_prev == EKF_SRC_GPS then
            relay:off(5)
            led_on_count = 0
        elseif not of_quality_acceptable and not rng_over_threshold then
            led_on_count = led_on_count + 1
            if led_on_count > 8 then
                relay:on(5)
            end
        end
    elseif FLGP_LED:get() == 2 then
        -- always ON
        relay:on(5)
    end
end

-- send ekf source to GCS at 1 Hz or when changed 
function send_ekf_source_to_gcs(send_now)
    if send_now or (last_send_ekf_source + 1000 < millis()) then
        last_send_ekf_source = millis()
        gcs:send_named_float("EKF_SOURCE", source_prev)
    end
end

if FLGP_ENABLE:get() < 1 then
    source_prev = EKF_SRC_GPS
    ahrs:set_posvelyaw_source_set(source_prev)
    gcs:send_text(MAV_SEVERITY.INFO, "FLGP disabled, switched to Source " .. string.format("%d", source_prev + 1))
else
    -- use optical flow for takeoff
    source_prev = EKF_SRC_OPTICALFLOW
    ahrs:set_posvelyaw_source_set(source_prev)
    gcs:send_text(MAV_SEVERITY.INFO, "Takeoff, switch to Source " .. string.format("%d", source_prev + 1))
end
return update, 100
