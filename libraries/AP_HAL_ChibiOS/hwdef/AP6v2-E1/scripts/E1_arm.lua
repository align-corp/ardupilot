-- custom arming routine for Align helicopters - version 1.1
local AUX_FUNCTION_MOTOR_INTERLOCK = 32
local AUX_HIGH = 2
local AUX_LOW = 0
local STATES = { WAIT_INTERLOCK_LOW = 0,
WAIT_INTERLOCK_HIGH = 1,
WAIT_THROTTLE_LOW = 2,
WAIT_THROTTLE_LOW_TIMEOUT = 3,
ARM = 4,
ARMED_ON = 5,
ARMED_OFF = 6,
ARMED_ON_SPOOLUP = 7,
ERROR_VIBRATIONS = 8
}
local state = STATES.WAIT_INTERLOCK_LOW
local time_ms = uint32_t(0)
local PARAM_TABLE_KEY = 46
assert(param:add_table(PARAM_TABLE_KEY, "VIBE_", 1), "could not add param table")
assert(param:add_param(PARAM_TABLE_KEY, 1, "TKOFF_MAX", 50), "could not add G3P_DEBUG param")
local VIBE_TKOFF_MAX = Parameter("VIBE_TKOFF_MAX")
function update()
if state == STATES.WAIT_INTERLOCK_LOW then
if rc:get_pwm(3) < 950 then
return update, 100
end
if rc:get_aux_cached(AUX_FUNCTION_MOTOR_INTERLOCK) == AUX_LOW then
state = STATES.WAIT_INTERLOCK_HIGH
end
elseif state == STATES.WAIT_INTERLOCK_HIGH then
if rc:get_aux_cached(AUX_FUNCTION_MOTOR_INTERLOCK) == AUX_HIGH then
time_ms = millis()
state = STATES.WAIT_THROTTLE_LOW
end
elseif state == STATES.WAIT_THROTTLE_LOW then
if rc:get_aux_cached(AUX_FUNCTION_MOTOR_INTERLOCK) == AUX_LOW then
state = STATES.WAIT_INTERLOCK_HIGH
end
if rc:get_pwm(3) > 950 and rc:get_pwm(3) < 1150 then
state = STATES.WAIT_THROTTLE_LOW_TIMEOUT
time_ms = millis()
end
if millis() - time_ms > 10000 then
state = STATES.WAIT_INTERLOCK_LOW
end
elseif state == STATES.WAIT_THROTTLE_LOW_TIMEOUT then
if rc:get_aux_cached(AUX_FUNCTION_MOTOR_INTERLOCK) == AUX_LOW then
state = STATES.WAIT_INTERLOCK_HIGH
end
if rc:get_pwm(3) > 1150 then
state = STATES.WAIT_THROTTLE_LOW
end
if millis() - time_ms > 2000 then
rc:run_aux_function(AUX_FUNCTION_MOTOR_INTERLOCK, AUX_LOW)
state = STATES.ARM
end
elseif state == STATES.ARM then
if arming:arm() then
rc:run_aux_function(AUX_FUNCTION_MOTOR_INTERLOCK, AUX_HIGH)
state = STATES.ARMED_ON_SPOOLUP
else
state = STATES.WAIT_INTERLOCK_LOW
end
elseif state == STATES.ARMED_ON_SPOOLUP then
if not arming:is_armed() then
state = STATES.WAIT_INTERLOCK_LOW
end
if rc:get_aux_cached(AUX_FUNCTION_MOTOR_INTERLOCK) == AUX_LOW then
time_ms = millis()
state = STATES.ARMED_OFF
end
if motors:get_spool_state() == 3 then
state = STATES.ARMED_ON
end
if ahrs:get_vibration():x() > VIBE_TKOFF_MAX:get() or ahrs:get_vibration():y() > VIBE_TKOFF_MAX:get() or ahrs:get_vibration():z() > VIBE_TKOFF_MAX:get() then
rc:run_aux_function(AUX_FUNCTION_MOTOR_INTERLOCK, AUX_LOW)
state = STATES.ERROR_VIBRATIONS
end
elseif state == STATES.ARMED_ON then
if not arming:is_armed() then
state = STATES.WAIT_INTERLOCK_LOW
end
if rc:get_aux_cached(AUX_FUNCTION_MOTOR_INTERLOCK) == AUX_LOW then
time_ms = millis()
state = STATES.ARMED_OFF
end
elseif state == STATES.ARMED_OFF then
if not arming:is_armed() then
state = STATES.WAIT_INTERLOCK_LOW
end
if rc:get_aux_cached(AUX_FUNCTION_MOTOR_INTERLOCK) == AUX_HIGH then
state = STATES.ARMED_ON
end
if motors:get_spool_state() < 2 then
if arming:disarm() then
state = STATES.WAIT_INTERLOCK_LOW
end
end
elseif state == STATES.ERROR_VIBRATIONS then
if arming:is_armed() then
rc:run_aux_function(AUX_FUNCTION_MOTOR_INTERLOCK, AUX_LOW)
if motors:get_spool_state() < 2 then
arming:disarm()
end
end
end
gcs:send_named_float("arm", state)
return update, 100
end
gcs:send_text('6', "E1_arm.lua is running")
return update, 100
