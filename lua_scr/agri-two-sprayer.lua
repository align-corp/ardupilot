-- pick the sprayer aux function to match the hardware that is actually fitted
--
-- The airframe can carry either a pump-only sprayer or a spreader driven by a
-- CAN ESC on SERVO11 (ESC telemetry instance 10). Both are wired to the same
-- RC switch, so this script watches that ESC's telemetry and retargets the
-- switch: SPRAYER2 while the ESC answers, plain SPRAYER once it goes quiet.
-- MAV_CMD_DO_SPRAYER (GCS or mission) drives every sprayer with SPRAYx_ENABLE
-- = 1, so the same choice is mirrored there: the active sprayer gets 1, the
-- other 2 (RC control only). A sprayer left at 0 is disabled and not touched.
--
-- It also ties the sprayers to return to home:
-- - entering RTL, SMART_RTL or AUTO_RTL turns both sprayers off
-- - when the drone was armed with more than RTLAG_ARM_KG on the load cells, an
--   empty tank engages RTL. SPRAYER: pump above RTLAG_PUMP_PCT with flow below
--   RTLAG_FLOW_LPM for RTLAG_PUMP_SEC. SPRAYER2: load below RTLAG_SPRD_KG for
--   RTLAG_SPRD_SEC
-- version 0.3

-- user constants
local MILLIS_UPDATE = 500
local ESC_INSTANCE = 10         -- 0 based ESC telemetry index, SERVO11
local CONFIRM_COUNT = 4         -- consecutive agreeing samples before switching
local STARTUP_GRACE_MS = 10000  -- wait this long for a CAN ESC to come up at boot
local NUM_RC_CHANNELS = 16
local CHANNEL_RETRY_MS = 5000   -- how often to look again for a sprayer switch

-- RC_Channel aux function numbers, see RC_Channel.h
local AUX_FUNC_SPRAYER = 15
local AUX_FUNC_SPRAYER2 = 178

-- RC_Channel::AuxSwitchPos
local AUX_POS_LOW = 0

-- SPRAYx_ENABLE values, see AC_Sprayer.cpp
local SPRAY_ENABLE_DISABLED = 0
local SPRAY_ENABLE_ALL = 1
local SPRAY_ENABLE_RC_ONLY = 2

-- Copter flight modes
local MODE_RTL = 6
local MODE_LAND = 9
local MODE_SMART_RTL = 21
local MODE_AUTO_RTL = 27

-- SRV_Channel function of the SPRAYER pump
local SERVO_FUNC_SPRAYER_PUMP = 22

-- parameters
local PARAM_TABLE_KEY = 52
local PARAM_TABLE_PREFIX = "RTLAG_"

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 7), 'agri-two-sprayer: could not add param table')

--[[
  // @Param: RTLAG_ENABLE
  // @DisplayName: Sprayer empty RTL enable
  // @Description: Engage RTL when the active sprayer runs empty. Turning the sprayers off on RTL is always active
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local RTLAG_ENABLE = bind_add_param('ENABLE', 1, 1)
--[[
  // @Param: RTLAG_ARM_KG
  // @DisplayName: Minimum load at arming
  // @Description: The empty tank checks only run when the load cells read more than this at arming
  // @Units: kg
  // @User: Standard
--]]
local RTLAG_ARM_KG = bind_add_param('ARM_KG', 2, 5)
--[[
  // @Param: RTLAG_PUMP_PCT
  // @DisplayName: Sprayer empty pump output
  // @Description: SPRAYER pump output above which a low flow means the tank is empty
  // @Units: %
  // @Range: 0 100
  // @User: Standard
--]]
local RTLAG_PUMP_PCT = bind_add_param('PUMP_PCT', 3, 80)
--[[
  // @Param: RTLAG_FLOW_LPM
  // @DisplayName: Sprayer empty flow
  // @Description: Flow below which, with the pump above RTLAG_PUMP_PCT, the tank is empty. Read from the SPRAY_FLOW_BATT monitor
  // @Units: litres/minute
  // @User: Standard
--]]
local RTLAG_FLOW_LPM = bind_add_param('FLOW_LPM', 4, 4)
--[[
  // @Param: RTLAG_PUMP_SEC
  // @DisplayName: Sprayer empty time
  // @Description: How long the pump must run high with a low flow before RTL, covers the flow meter lag at pump start
  // @Units: s
  // @User: Standard
--]]
local RTLAG_PUMP_SEC = bind_add_param('PUMP_SEC', 5, 3)
--[[
  // @Param: RTLAG_SPRD_KG
  // @DisplayName: Spreader empty load
  // @Description: Load below which the SPRAYER2 hopper is empty
  // @Units: kg
  // @User: Standard
--]]
local RTLAG_SPRD_KG = bind_add_param('SPRD_KG', 6, 2)
--[[
  // @Param: RTLAG_SPRD_SEC
  // @DisplayName: Spreader empty time
  // @Description: How long the load must stay below RTLAG_SPRD_KG before RTL
  // @Units: s
  // @User: Standard
--]]
local RTLAG_SPRD_SEC = bind_add_param('SPRD_SEC', 7, 20)

-- state
local rc_num = nil              -- RC channel carrying the sprayer switch
local option_param = nil        -- its RCn_OPTION parameter
local current_option = nil      -- aux function currently assigned to it
local pending_option = nil      -- function we are waiting to confirm
local pending_count = 0
local decided = false           -- false until the first swap has been applied
local spray_enable = {}         -- aux function -> its SPRAYx_ENABLE parameter
local last_channel_search_ms = nil

-- RTH state
local was_armed = false
local was_rth = false
local arm_loaded = false        -- load at arming was above RTLAG_ARM_KG
local empty_since_ms = nil      -- time the empty condition was first seen
local empty_rtl_done = false    -- empty RTL already engaged since arming
local load_batt_param = nil     -- LOAD_BATT_IDX from agri-load.lua
local flow_batt_param = nil     -- SPRAY_FLOW_BATT

-- find the RC channel already assigned to a sprayer aux function and latch on
-- to it. Returns false while no channel is configured.
function find_sprayer_channel()
    for i = 1, NUM_RC_CHANNELS do
        -- Parameter(name) raises on an unknown name, so bind in two steps and
        -- let init() report the channels this build does not have
        local p = Parameter()
        if p:init(string.format('RC%u_OPTION', i)) then
            local opt = p:get()
            if opt == AUX_FUNC_SPRAYER or opt == AUX_FUNC_SPRAYER2 then
                if rc_num then
                    -- a second switch is configured, the swap would only ever
                    -- follow the first one so say so rather than fight it
                    gcs:send_text(4, string.format("agri-two-sprayer: RC%u also set to sprayer, ignoring", i))
                else
                    rc_num = i
                    option_param = p
                    current_option = math.floor(opt)
                end
            end
        end
    end
    return rc_num ~= nil
end

-- bind SPRAY_ENABLE and SPRAY2_ENABLE, either may be missing from the build
function bind_spray_enable()
    local names = { [AUX_FUNC_SPRAYER] = 'SPRAY_ENABLE', [AUX_FUNC_SPRAYER2] = 'SPRAY2_ENABLE' }
    for option, name in pairs(names) do
        local p = Parameter()
        if p:init(name) then
            spray_enable[option] = p
        else
            gcs:send_text(4, string.format("agri-two-sprayer: no %s, MAV_CMD_DO_SPRAYER target not managed", name))
        end
    end
end

-- let MAV_CMD_DO_SPRAYER reach only the sprayer behind active_option
function apply_spray_enable(active_option)
    for option, p in pairs(spray_enable) do
        local wanted = (option == active_option) and SPRAY_ENABLE_ALL or SPRAY_ENABLE_RC_ONLY
        local now = math.floor(p:get())
        -- 0 -> 1 needs a reboot and means the user disabled it, leave it alone
        if now ~= SPRAY_ENABLE_DISABLED and now ~= wanted then
            -- set() and not set_and_save(), redone on every boot like RCn_OPTION
            if not p:set(wanted) then
                gcs:send_text(3, "agri-two-sprayer: could not set SPRAYx_ENABLE")
            end
        end
    end
end

-- true while ESC telemetry for ESC_INSTANCE is fresh. The ESC may not publish
-- every field, so any live field counts as present. RPM times out after 1s,
-- the remaining fields after 5s.
function esc_telem_present()
    return (esc_telem:get_rpm(ESC_INSTANCE) ~= nil)
end

function report(present, verb)
    gcs:send_text(6, string.format("agri-two-sprayer: RC%u %s %s (ESC%u %s)",
                                   rc_num, verb,
                                   present and "SPRAYER2" or "SPRAYER",
                                   ESC_INSTANCE,
                                   present and "present" or "absent"))
end

-- point the switch at wanted_option, carrying the switch position across so
-- the outgoing sprayer cannot stay latched on
function apply_option(wanted_option)
    local channel = rc:get_channel(rc_num)
    local pos = AUX_POS_LOW
    if channel then
        pos = channel:get_aux_switch_pos()
    end

    -- stop the sprayer we are about to orphan
    rc:run_aux_function(current_option, AUX_POS_LOW)

    -- set() and not set_and_save(): the choice follows the hardware and is
    -- made again on every boot, no need to wear out the flash
    if not option_param:set(wanted_option) then
        gcs:send_text(3, string.format("agri-two-sprayer: could not set RC%u_OPTION", rc_num))
        return false
    end
    current_option = wanted_option
    apply_spray_enable(wanted_option)

    -- read_aux() only acts on a change of switch position, so hand the current
    -- position to the incoming sprayer ourselves
    if not rc:run_aux_function(wanted_option, pos) then
        gcs:send_text(3, string.format("agri-two-sprayer: aux function %u not available", wanted_option))
    end
    return true
end

function update_switch()
    local present = esc_telem_present()

    -- at boot give a powered CAN ESC time to be heard before calling it absent
    if not decided and not present and millis():toint() < STARTUP_GRACE_MS then
        return
    end

    local wanted_option = present and AUX_FUNC_SPRAYER2 or AUX_FUNC_SPRAYER

    if wanted_option == current_option then
        -- already right, and on the first pass that means the saved option
        -- matches the hardware so there is nothing to toggle
        if not decided then
            decided = true
            apply_spray_enable(wanted_option)
            report(present, "already set to")
        end
        pending_option = nil
        pending_count = 0
        return
    end

    -- debounce, a single dropped telemetry frame must not flip the sprayer
    if wanted_option ~= pending_option then
        pending_option = wanted_option
        pending_count = 1
    else
        pending_count = pending_count + 1
    end

    if pending_count >= CONFIRM_COUNT or not decided then
        if apply_option(wanted_option) then
            decided = true
            report(present, "->")
        end
        pending_option = nil
        pending_count = 0
    end
end

-- load on the cells in kg, nil if agri-load.lua is not feeding it
function load_kg()
    if not load_batt_param then
        local p = Parameter()
        if not p:init('LOAD_BATT_IDX') then
            return nil
        end
        load_batt_param = p
    end
    local idx = math.floor(load_batt_param:get())
    if not battery:healthy(idx) then
        return nil
    end
    -- agri-load.lua reports the total in kg as the voltage
    return battery:voltage(idx)
end

-- SPRAYER flow in litres/minute from the SPRAY_FLOW_BATT monitor, nil if none
function flow_lpm()
    if not flow_batt_param then
        local p = Parameter()
        if not p:init('SPRAY_FLOW_BATT') then
            return nil
        end
        flow_batt_param = p
    end
    local instance = math.floor(flow_batt_param:get())
    if instance <= 0 then
        return nil
    end
    -- the fuel flow monitor reports litres/minute as current
    return battery:current_amps(instance - 1)
end

-- SPRAYER pump output as a percentage of its SERVOn_MIN..SERVOn_MAX range,
-- nil if no output is assigned to the pump
function pump_pct()
    local chan = SRV_Channels:find_channel(SERVO_FUNC_SPRAYER_PUMP)
    local pwm = SRV_Channels:get_output_pwm(SERVO_FUNC_SPRAYER_PUMP)
    if not chan or not pwm then
        return nil
    end
    local n = chan + 1
    local pwm_min = param:get(string.format('SERVO%u_MIN', n))
    local pwm_max = param:get(string.format('SERVO%u_MAX', n))
    local reversed = param:get(string.format('SERVO%u_REVERSED', n))
    if not pwm_min or not pwm_max or pwm_max <= pwm_min then
        return nil
    end
    local pct = (pwm - pwm_min) * 100 / (pwm_max - pwm_min)
    if reversed and reversed ~= 0 then
        pct = 100 - pct
    end
    return pct
end

function is_rth_mode(mode)
    return mode == MODE_RTL or mode == MODE_SMART_RTL or mode == MODE_AUTO_RTL
end

-- stop both sprayers, whichever one the switch or MAV_CMD_DO_SPRAYER started.
-- The switch stays where it is: read_aux() only acts on a change of position,
-- so the pilot has to cycle it to spray again
function stop_sprayers()
    rc:run_aux_function(AUX_FUNC_SPRAYER, AUX_POS_LOW)
    rc:run_aux_function(AUX_FUNC_SPRAYER2, AUX_POS_LOW)
end

-- true once the active sprayer has been empty long enough
function tank_empty(now)
    local empty = false
    local hold_ms = 0
    if current_option == AUX_FUNC_SPRAYER then
        local pct = pump_pct()
        local lpm = flow_lpm()
        empty = pct ~= nil and lpm ~= nil and pct > RTLAG_PUMP_PCT:get() and lpm < RTLAG_FLOW_LPM:get()
        hold_ms = RTLAG_PUMP_SEC:get() * 1000
    elseif current_option == AUX_FUNC_SPRAYER2 then
        local kg = load_kg()
        empty = kg ~= nil and kg < RTLAG_SPRD_KG:get()
        hold_ms = RTLAG_SPRD_SEC:get() * 1000
    end

    if not empty then
        empty_since_ms = nil
        return false
    end
    if not empty_since_ms then
        empty_since_ms = now
    end
    return (now - empty_since_ms) >= hold_ms
end

function update_rth()
    local now = millis():toint()
    local armed = arming:is_armed()
    local mode = vehicle:get_mode()
    local rth = is_rth_mode(mode)

    -- turn the sprayers off on entering RTH, from any source
    if rth and not was_rth then
        stop_sprayers()
        gcs:send_text(5, "agri-two-sprayer: RTL, sprayer off")
    end
    was_rth = rth

    -- latch the load at arming, the empty checks only make sense if we took
    -- off with something in the tank
    if armed and not was_armed then
        local kg = load_kg()
        arm_loaded = kg ~= nil and kg > RTLAG_ARM_KG:get()
        empty_since_ms = nil
        empty_rtl_done = false
        if kg then
            gcs:send_text(6, string.format("agri-two-sprayer: armed with %.1f kg, empty RTL %s",
                                           kg, arm_loaded and "active" or "inactive"))
        end
    end
    was_armed = armed

    if not armed or not arm_loaded or empty_rtl_done or RTLAG_ENABLE:get() < 1 then
        empty_since_ms = nil
        return
    end
    -- nothing to do on the ground or once already heading home or landing
    if rth or mode == MODE_LAND or not vehicle:get_likely_flying() then
        empty_since_ms = nil
        return
    end

    if tank_empty(now) then
        -- only once per flight, the pilot may take over and finish by hand
        empty_rtl_done = true
        empty_since_ms = nil
        if vehicle:set_mode(MODE_RTL) then
            gcs:send_text(2, "agri-two-sprayer: tank empty, RTL")
        else
            gcs:send_text(2, "agri-two-sprayer: tank empty, RTL failed")
        end
    end
end

function update()
    update_rth()

    -- keep looking for a sprayer switch until one is configured
    if not rc_num then
        local now = millis():toint()
        if last_channel_search_ms and (now - last_channel_search_ms) < CHANNEL_RETRY_MS then
            return update, MILLIS_UPDATE
        end
        last_channel_search_ms = now
        if not find_sprayer_channel() then
            return update, MILLIS_UPDATE
        end
        gcs:send_text(6, string.format("agri-two-sprayer: watching RC%u", rc_num))
    end

    update_switch()
    return update, MILLIS_UPDATE
end

if not esc_telem then
    gcs:send_text(3, "agri-two-sprayer: no ESC telemetry support, stopping")
    return
end

bind_spray_enable()
gcs:send_text(6, "agri-two-sprayer is running")
return update, 1000
