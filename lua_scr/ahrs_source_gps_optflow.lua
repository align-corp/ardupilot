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
local EKF_SRC_GPS = 0
local EKF_SRC_OPTICALFLOW = 1
local EKF_SRC_UNDECIDED = -1
local RNG_ROTATION_DOWN = 25
local VOTE_COUNT_MAX = 20 -- when a vote counter reaches this number (i.e. 2sec) source may be switched

-- variables
local source_prev = EKF_SRC_OPTICALFLOW -- opticalflow for takeoff
local gps_vs_opticalflow_vote = 0 -- vote counter for GPS vs optical (-20 = GPS, +20 = optical flow)
local gps_hdop_threshold = param:get("GPS_HDOP_GOOD")
local gpa_nsats_threshold = param:get("AHRS_GPS_MINSATS")

local PARAM_TABLE_KEY = 81
PARAM_TABLE_PREFIX = "FLGP_"
local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

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

-- enable
local FLGP_ENABLE = bind_add_param('ENABLE', 1, 1)

-- EKF Source OpticalFlow Innovation Threshold
local FLGP_FLOW_THRESH = bind_add_param('FLOW_THRESH', 2, 0.3)

-- OpticalFlow may be used if quality is above this threshold
local FLGP_FLOW_QUAL = bind_add_param('FLOW_QUAL', 3, 40)

-- OpticalFlow may be used if rangefinder distance is below this threshold
local FLGP_RNGFND_MAX = bind_add_param('RNGFND_MAX', 4, 3.5)

assert(optical_flow, 'could not access optical flow')

-- the main update function
function update()

  if FLGP_ENABLE:get() < 1 then
    if source_prev ~= EKF_SRC_GPS then
        source_prev = EKF_SRC_GPS
        ahrs:set_posvelyaw_source_set(source_prev) -- switch to GPS
        gcs:send_text(0, "FLGP disabled: switched to Source " .. string.format("%d", source_prev+1))
    end
    return update, 100
  end

  -- check rangefinder distance threshold has been set
  local rangefinder_thresh_dist = FLGP_RNGFND_MAX:get()     -- SCR_USER1 holds rangefinder threshold
  if (rangefinder_thresh_dist <= 0) then
    gcs:send_text(0, "ahrs-source-gps-optflow.lua: set SCR_USER1 to rangefinder threshold")
    return update, 1000
  end

  -- check optical flow quality threshold has been set
  local opticalflow_quality_thresh = FLGP_FLOW_QUAL:get()  -- SCR_USER3 holds opticalflow quality
  if (opticalflow_quality_thresh <= 0) then
    gcs:send_text(0, "ahrs-source-gps-optflow.lua: set SCR_USER3 to OpticalFlow quality threshold")
    return update, 1000
  end

  -- check optical flow innovation threshold has been set
  local opticalflow_innov_thresh = FLGP_FLOW_THRESH:get()    -- SCR_USER4 holds opticalflow innovation
  if (opticalflow_innov_thresh <= 0) then
    gcs:send_text(0, "ahrs-source-gps-optflow.lua: set SCR_USER4 to OpticalFlow innovation threshold")
    return update, 1000
  end

  -- check if GPS is usable
  local gps_hdop = gps:get_hdop(gps:primary_sensor())
  local gps_nsats = gps:num_sats(gps:primary_sensor())
  local gps_fix = gps:status(gps:primary_sensor())
  local gps_usable = false
  if gps_hdop ~= nil and gps_nsats ~= nil and gps_fix ~= nil then
    gps_usable = (gps_hdop <= gps_hdop_threshold) and
      (gps_nsats >= gpa_nsats_threshold) and
      (gps_fix >= 3)
  end

  -- check optical flow quality
  local opticalflow_quality_good = false
  if (optical_flow) then
    opticalflow_quality_good = (optical_flow:enabled() and optical_flow:healthy() and optical_flow:quality() >= opticalflow_quality_thresh)
  end

  -- get opticalflow innovations from ahrs (only x and y values are valid)
  local opticalflow_over_threshold = true
  local opticalflow_xy_innov = 0
  local opticalflow_innov = Vector3f()
  local opticalflow_var = Vector3f()
  opticalflow_innov, opticalflow_var = ahrs:get_vel_innovations_and_variances_for_source(5)
  if (opticalflow_innov) then
    opticalflow_xy_innov = math.sqrt(opticalflow_innov:x() * opticalflow_innov:x() + opticalflow_innov:y() * opticalflow_innov:y())
    opticalflow_over_threshold = (opticalflow_xy_innov == 0.0) or (opticalflow_xy_innov > opticalflow_innov_thresh)
  end

  -- get rangefinder distance
  local rngfnd_distance_m = 0
  if rangefinder:has_data_orient(RNG_ROTATION_DOWN) then
      rngfnd_distance_m = rangefinder:distance_cm_orient(RNG_ROTATION_DOWN) * 0.01
  end
  local rngfnd_over_threshold = (rngfnd_distance_m == 0) or (rngfnd_distance_m > rangefinder_thresh_dist)

  -- altitude hold: don't use opticalflow if rangefinder is out of range
  -- this is needed, otherwise EKF set to optical flow prevent vehicle to climb higher than 0.7*RNGFND_MAX_DIST
  if vehicle:get_mode() <= 2 then
      local althold_source_requested = EKF_SRC_UNDECIDED
      if rngfnd_over_threshold then
          althold_source_requested = EKF_SRC_GPS
      else
          althold_source_requested = EKF_SRC_OPTICALFLOW
      end
      if source_prev ~= althold_source_requested then
          source_prev = althold_source_requested
          ahrs:set_posvelyaw_source_set(source_prev) -- switch to GPS
          gcs:send_text(0, "alt_hold: switched to Source " .. string.format("%d", source_prev+1))
      end
      return update, 100
  end

  -- opticalflow is usable if quality and innovations are good and rangefinder is in range
  local opticalflow_usable = opticalflow_quality_good and (not opticalflow_over_threshold) and (not rngfnd_over_threshold)

  -- automatic selection logic --

  -- GPS vs opticalflow vote. "-1" to move towards GPS, "+1" to move to Non-GPS
  if (gps_usable and not opticalflow_usable) or (gps_fix == 6) then
    -- vote for GPS GPS is usable and opticalflow is unusable OR we're in RTK fix
    gps_vs_opticalflow_vote = gps_vs_opticalflow_vote - 1
  elseif opticalflow_usable then
    -- vote for opticalflow if usable
    gps_vs_opticalflow_vote = gps_vs_opticalflow_vote + 1
  end

  -- prevent EKF failsafe immediately switching to GPS if we're getting out of rangefinder range
  -- yes, this is cheating
  local velocity_ned = ahrs:get_velocity_NED()
  local climb_rate = 0
  if velocity_ned ~= nil then
    climb_rate = -velocity_ned:z() -- m/s
  end
  if (gps_usable) and
      (rngfnd_distance_m > rangefinder:max_distance_cm_orient(RNG_ROTATION_DOWN) * 0.006) and
      (climb_rate > 0.3) and
      (source_prev ~= EKF_SRC_GPS) then
    gps_vs_opticalflow_vote = -VOTE_COUNT_MAX
    source_prev = EKF_SRC_GPS
    ahrs:set_posvelyaw_source_set(source_prev)
    gcs:send_text(4, "Fast climb detected, switch to GPS")
    return update, 100
  end

  -- auto source vote collation
  local auto_source = EKF_SRC_UNDECIDED
  if gps_vs_opticalflow_vote <= -VOTE_COUNT_MAX then
    auto_source = EKF_SRC_GPS
    gps_vs_opticalflow_vote = -VOTE_COUNT_MAX
  elseif gps_vs_opticalflow_vote >= VOTE_COUNT_MAX then
    auto_source = EKF_SRC_OPTICALFLOW
    gps_vs_opticalflow_vote = VOTE_COUNT_MAX
  end

  -- auto switching
  if (auto_source >= 0) and (auto_source ~= source_prev) then
    source_prev = auto_source
    ahrs:set_posvelyaw_source_set(source_prev)
    gcs:send_text(4, "Auto switched to Source " .. string.format("%d", source_prev+1))
  end

  return update, 100
end

if FLGP_ENABLE:get() < 1 then
    source_prev = EKF_SRC_GPS
    ahrs:set_posvelyaw_source_set(source_prev)
    gcs:send_text(4, "FLGP disabled, switched to Source " .. string.format("%d", source_prev+1))
else
    -- use optical flow for takeoff
    source_prev = EKF_SRC_OPTICALFLOW
    ahrs:set_posvelyaw_source_set(source_prev)
    gcs:send_text(4, "Takeoff, switch to Source " .. string.format("%d", source_prev+1))
end
return update, 100
