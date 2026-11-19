--[[
   autotest helper: simulate a downward rangefinder stuck at a constant distance.
   Requires RNGFND1_TYPE=36 (Lua) and RNGFND1_ORIENT=25 (down).
--]]

local STUCK_DIST_M = 6.0

---@type AP_RangeFinder_Backend_ud
local backend

local function update()
    if not backend then
        backend = rangefinder:get_backend(0)
        if not backend then
            return update, 1000
        end
    end
    backend:handle_script_msg(STUCK_DIST_M)
    return update, 20
end

return update, 1000
