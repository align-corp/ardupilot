-- pick the sprayer aux function to match the hardware that is actually fitted
--
-- The airframe can carry either a pump-only sprayer or a spreader driven by a
-- CAN ESC on SERVO11 (ESC telemetry instance 10). Both are wired to the same
-- RC switch, so this script watches that ESC's telemetry and retargets the
-- switch: SPRAYER2 while the ESC answers, plain SPRAYER once it goes quiet.
-- MAV_CMD_DO_SPRAYER (GCS or mission) drives every sprayer with SPRAYx_ENABLE
-- = 1, so the same choice is mirrored there: the active sprayer gets 1, the
-- other 2 (RC control only). A sprayer left at 0 is disabled and not touched.
-- version 0.2

-- user constants
local MILLIS_UPDATE = 500
local ESC_INSTANCE = 10         -- 0 based ESC telemetry index, SERVO11
local CONFIRM_COUNT = 4         -- consecutive agreeing samples before switching
local STARTUP_GRACE_MS = 10000  -- wait this long for a CAN ESC to come up at boot
local NUM_RC_CHANNELS = 16

-- RC_Channel aux function numbers, see RC_Channel.h
local AUX_FUNC_SPRAYER = 15
local AUX_FUNC_SPRAYER2 = 178

-- RC_Channel::AuxSwitchPos
local AUX_POS_LOW = 0

-- SPRAYx_ENABLE values, see AC_Sprayer.cpp
local SPRAY_ENABLE_DISABLED = 0
local SPRAY_ENABLE_ALL = 1
local SPRAY_ENABLE_RC_ONLY = 2

-- state
local rc_num = nil              -- RC channel carrying the sprayer switch
local option_param = nil        -- its RCn_OPTION parameter
local current_option = nil      -- aux function currently assigned to it
local pending_option = nil      -- function we are waiting to confirm
local pending_count = 0
local decided = false           -- false until the first swap has been applied
local spray_enable = {}         -- aux function -> its SPRAYx_ENABLE parameter

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
                    gcs:send_text(4, string.format("sprayer-switch: RC%u also set to sprayer, ignoring", i))
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
            gcs:send_text(4, string.format("sprayer-switch: no %s, MAV_CMD_DO_SPRAYER target not managed", name))
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
                gcs:send_text(3, "sprayer-switch: could not set SPRAYx_ENABLE")
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
    gcs:send_text(6, string.format("sprayer-switch: RC%u %s %s (ESC%u %s)",
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
        gcs:send_text(3, string.format("sprayer-switch: could not set RC%u_OPTION", rc_num))
        return false
    end
    current_option = wanted_option
    apply_spray_enable(wanted_option)

    -- read_aux() only acts on a change of switch position, so hand the current
    -- position to the incoming sprayer ourselves
    if not rc:run_aux_function(wanted_option, pos) then
        gcs:send_text(3, string.format("sprayer-switch: aux function %u not available", wanted_option))
    end
    return true
end

function update()
    local present = esc_telem_present()

    -- at boot give a powered CAN ESC time to be heard before calling it absent
    if not decided and not present and millis():toint() < STARTUP_GRACE_MS then
        return update, MILLIS_UPDATE
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
        return update, MILLIS_UPDATE
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

    return update, MILLIS_UPDATE
end

-- hold here until a sprayer switch is configured, there is nothing to retarget
-- without one
function wait_for_channel()
    if not find_sprayer_channel() then
        return wait_for_channel, 5000
    end
    gcs:send_text(6, string.format("sprayer-switch: watching RC%u", rc_num))
    return update, MILLIS_UPDATE
end

if not esc_telem then
    gcs:send_text(3, "sprayer-switch: no ESC telemetry support, stopping")
    return
end

bind_spray_enable()
gcs:send_text(6, "Sprayer Switch is running")
return wait_for_channel, 1000
