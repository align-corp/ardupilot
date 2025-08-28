-- Monitor battery voltage, applies median filter, and triggers RTL based on distance from home

----------------------------------------------
--- Configuration parameters
----------------------------------------------
local BATTERY_CELLS = 6
local VOLTAGE_TO_PERCENT_TABLE = {
    {3.4, 0},
    {3.5, 4},
    {3.6, 11},
    {3.7, 42},
    {3.8, 65},
    {3.9, 79},
    {4.0, 90},
    {4.1, 100}
}

-- %/m = 1/(SPEED[m/s]*36*BATT_CAPACITY[Ah]/CRUISE_CURRENT[A])
local PERCENT_PER_METER = 0.012 -- Battery percentage consumed per meter of travel
local PERCENT_PER_METER_DOWN = 0.03  -- Battery percentage consumed per meter of travel
local MIN_SAFE_PERCENT = 7    -- Minimum battery percentage needed to return home
local MINIMUM_RTL_DIST = 30  -- Minimum distance from home to trigger RTL

----------------------------------------------
--- script start, don't change below this line
----------------------------------------------
local UPDATE_VOLTAGE_MS = 200
local VOLTAGES_SAMPLES_TO_MEDIAN = 51
local RTL_MODE = 6  -- Mode number for RTL
local PARAM_TABLE_KEY = 82

-- Variables
local voltage_samples = {}
local sample_count = 0

-- add a parameter and bind it to a variable
local function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    local p = Parameter()
    assert(p:init('RTLS_' .. name), string.format('could not find %s parameter', name))
    return p
end

-- add param table
assert(param:add_table(PARAM_TABLE_KEY, "RTLS_", 2), 'rtl_dist: could not add param table')

local RTLS_ENABLE = bind_add_param('ENABLE', 1, 1)
local RTLS_DIST = bind_add_param('DIST', 2, MINIMUM_RTL_DIST)

function update()
    -- return if not enabled
    if RTLS_ENABLE:get() < 1 then
        return update, 1000
    end

    -- return if not armed
    if not arming:is_armed() then
        return update, 1000
    end

    -- return if already in RTL
    if vehicle:get_mode() == RTL_MODE then
        return update, 1000
    end

    -- Collect voltage samples for specified duration
    if sample_count < VOLTAGES_SAMPLES_TO_MEDIAN then
        local voltage = battery:voltage(0)
        if voltage then
            table.insert(voltage_samples, voltage)
            sample_count = sample_count + 1
        end
        return update, UPDATE_VOLTAGE_MS
    end

    -- Apply median filter to voltage samples
    table.sort(voltage_samples)
    local median_voltage = voltage_samples[math.floor(#voltage_samples / 2) + 1]

    -- Reset for next check cycle
    voltage_samples = {}
    sample_count = 0

    -- Convert voltage to percentage using lookup table
    local percent = voltage_to_percent(median_voltage)

    -- Get distance from home and altitude
    local home = ahrs:get_home()
    local current_loc = ahrs:get_position()
    local alt = -ahrs:get_relative_position_D_home()

    if home and current_loc and alt then
        local distance_m = home:get_distance(current_loc)

        -- return if distance from home is < MINIMUM_RTL_DIST
        local minimum_dist = RTLS_DIST:get()
        if minimum_dist < 0 or minimum_dist > 1000 then
            minimum_dist = MINIMUM_RTL_DIST
        end
        if distance_m < minimum_dist and alt < minimum_dist then
            return update, UPDATE_VOLTAGE_MS
        end

        -- Calculate required battery percentage for return trip
        local required_percent_horizontal = distance_m * PERCENT_PER_METER
        local required_percent_down = alt * PERCENT_PER_METER_DOWN
        local required_percent = required_percent_horizontal + required_percent_down

        -- trigger RTL if battery is low
        if percent < (required_percent + MIN_SAFE_PERCENT) then
            gcs:send_text(3, string.format("Low battery! %.1f%% remaining, start RTL", percent))
            vehicle:set_mode(RTL_MODE)
        end
    end

    return update, UPDATE_VOLTAGE_MS
end

-- Function to convert voltage to percentage using lookup table
function voltage_to_percent(voltage)
    -- divide for number of cells
    voltage = voltage / BATTERY_CELLS

    -- Handle edge cases
    if voltage <= VOLTAGE_TO_PERCENT_TABLE[1][1] then
        return VOLTAGE_TO_PERCENT_TABLE[1][2]
    elseif voltage >= VOLTAGE_TO_PERCENT_TABLE[#VOLTAGE_TO_PERCENT_TABLE][1] then
        return VOLTAGE_TO_PERCENT_TABLE[#VOLTAGE_TO_PERCENT_TABLE][2]
    end

    -- Find the two closest voltage entries in the table
    for i = 1, #VOLTAGE_TO_PERCENT_TABLE - 1 do
        local v1 = VOLTAGE_TO_PERCENT_TABLE[i][1]
        local p1 = VOLTAGE_TO_PERCENT_TABLE[i][2]
        local v2 = VOLTAGE_TO_PERCENT_TABLE[i+1][1]
        local p2 = VOLTAGE_TO_PERCENT_TABLE[i+1][2]

        if voltage >= v1 and voltage <= v2 then
            -- Linear interpolation
            return p1 + (voltage - v1) * (p2 - p1) / (v2 - v1)
        end
    end

    return 0  -- Fallback
end

gcs:send_text(6, "rtl_dist.lua is running")
return update, 1000
