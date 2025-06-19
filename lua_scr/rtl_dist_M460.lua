-- Battery voltage to RTL script
-- Monitors battery voltage, applies median filter, and triggers RTL based on distance from home

-- Configuration parameters
local BATTERY_CELLS = 6
local VOLTAGE_TO_PERCENT_TABLE = {
    {3.3, 0},
    {3.4, 9},
    {3.5, 26},
    {3.6, 44},
    {3.7, 59},
    {3.8, 70},
    {3.9, 79},
    {4.0, 90},
    {4.1, 100}
}

-- %/m = 1/(SPEED[m/s]*36*BATT_CAPACITY[Ah]/CRUISE_CURRENT[A])
local PERCENT_PER_METER = 0.015  -- Battery percentage consumed per meter of travel
local PERCENT_PER_METER_DOWN = 0.03  -- Battery percentage consumed per meter of travel
local MIN_SAFE_PERCENT = 5    -- Minimum battery percentage needed to return home
local MINIMUM_RTL_DIST = 30  -- Minimum distance from home to trigger RTL
local UPDATE_VOLTAGE_MS = 200
local VOLTAGES_SAMPLES_TO_MEDIAN = 51
local RTL_MODE = 6  -- Mode number for RTL

-- Variables
local voltage_samples = {}
local sample_count = 0

function update()
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
        if distance_m < MINIMUM_RTL_DIST and alt < MINIMUM_RTL_DIST then
            gcs:send_text(6, string.format("Distance from home = %.0f m, altitude = %.0f m, no RTH", distance_m, alt))
            return update, UPDATE_VOLTAGE_MS
        end

        -- Calculate required battery percentage for return trip
        local required_percent_horizontal = distance_m * PERCENT_PER_METER
        local required_percent_down = alt * PERCENT_PER_METER_DOWN
        local required_percent = required_percent_horizontal + required_percent_down

        -- Decide if RTL should be triggered
        if percent < (required_percent + MIN_SAFE_PERCENT) then
            gcs:send_text(3, string.format("Low battery! %.1f%% remaining, %.1f%% needed for return",
                                          percent, required_percent))
            vehicle:set_mode(RTL_MODE)
        else
            gcs:send_text(6, string.format("Battery OK: %.1f%% remaining, %.1f%% needed for return",
                                          percent, required_percent))
        end
    else
        gcs:send_text(4, "Could not get home or current location")
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
