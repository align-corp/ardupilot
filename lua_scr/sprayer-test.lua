-- cycle a sprayer on and off forever, for bench and endurance testing
--
-- Turns the selected sprayer on for ON_SECONDS, then off for OFF_SECONDS, and
-- repeats. It drives the same aux function an RC switch would, so the usual
-- sprayer rules apply: disarmed it runs in test mode (pump as if at 1m/s, and
-- stopped early by SPRAYx_TIMEOUT if ON_SECONDS is longer than that), armed it
-- also needs SPRAYx_SPEED_MIN and SPRAYx_ALT_MIN to be met.
-- version 0.1

-- user constants
local SPRAYER = 2               -- 1 = SPRAYER, 2 = SPRAYER2
local ON_SECONDS = 15
local OFF_SECONDS = 5

-- RC_Channel aux function numbers, see RC_Channel.h
local AUX_FUNC_SPRAYER = 15
local AUX_FUNC_SPRAYER2 = 178

-- RC_Channel::AuxSwitchPos
local AUX_POS_LOW = 0
local AUX_POS_HIGH = 2

local aux_func = (SPRAYER == 2) and AUX_FUNC_SPRAYER2 or AUX_FUNC_SPRAYER
local cycle = 0

function set_sprayer(on)
    if not rc:run_aux_function(aux_func, on and AUX_POS_HIGH or AUX_POS_LOW) then
        gcs:send_text(3, string.format("sprayer_test: aux function %u not available", aux_func))
        return false
    end
    return true
end

function turn_on()
    cycle = cycle + 1
    if not set_sprayer(true) then
        return
    end
    gcs:send_text(6, string.format("sprayer_test: sprayer%u ON (cycle %u)", SPRAYER, cycle))
    return turn_off, ON_SECONDS * 1000
end

function turn_off()
    if not set_sprayer(false) then
        return
    end
    gcs:send_text(6, string.format("sprayer_test: sprayer%u OFF", SPRAYER))
    return turn_on, OFF_SECONDS * 1000
end

if SPRAYER ~= 1 and SPRAYER ~= 2 then
    gcs:send_text(3, string.format("sprayer_test: SPRAYER must be 1 or 2, got %s", tostring(SPRAYER)))
    return
end

gcs:send_text(6, string.format("sprayer_test: cycling sprayer%u %us on / %us off", SPRAYER, ON_SECONDS, OFF_SECONDS))
return turn_on, 10000
