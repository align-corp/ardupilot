-- This script checks additional battery data and report them through console

local battery_instance = 0
local PARAM_TABLE_KEY = 51
assert(param:add_table(PARAM_TABLE_KEY, "BAT_", 1), "could not add param table")
assert(param:add_param(PARAM_TABLE_KEY, 1, "CYCLES", -1), "could not add param")
BATTC_CYCLES = Parameter("BAT_CYCLES")
local count = 0

function update()
    local cycle_count = battery:get_cycle_count(battery_instance)
    if cycle_count then
        BATTC_CYCLES:set(cycle_count)
        return
    end
    count = count + 1
    if count > 20 then
        return
    end
    return update, 1000
end

BATTC_CYCLES:set(-1)
return update, 5000
