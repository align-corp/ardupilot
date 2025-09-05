-- This script runs a custom arming check of the battery temperature

local auth_id = arming:get_aux_auth_id()

-- parameters
local PARAM_TABLE_KEY = 47
assert(param:add_table(PARAM_TABLE_KEY, "ARM_", 2), "could not add param table")
assert(param:add_param(PARAM_TABLE_KEY, 1, "ENABLE", 1), "could not add ARM_ENABLE param")
assert(param:add_param(PARAM_TABLE_KEY, 2, "AUTH", 0), "could not add ARM_AUTH param")
local ARM_ENABLE = Parameter("ARM_ENABLE")
local ARM_AUTH = Parameter("ARM_AUTH")
local SERIAL_NUM = Parameter("BRD_SERIAL_NUM")

function update() -- this is the loop which periodically runs
    if auth_id then
        -- enable check
        if ARM_ENABLE:get() == 0 then
            arming:set_aux_auth_passed(auth_id)
            -- die here
            return
        end
        -- serial num check
        if SERIAL_NUM:get() == 0 then
            arming:set_aux_auth_passed(auth_id)
            -- die here
            return
        end
        if ARM_AUTH:get() < 1 then
            arming:set_aux_auth_failed(auth_id, "Serial number not authorised")
        else
            arming:set_aux_auth_passed(auth_id)
            -- die here
            return
        end
    end
    return update, 5000 -- reschedules the loop in 5 seconds
end

-- set ARM_AUTH to 0
ARM_AUTH:set_and_save(0)
return update()
