-- Monitor battery voltage, applies median filter, and triggers RTL based on distance from home

----------------------------------------------
--- Configuration parameters
----------------------------------------------
local VOLTAGE_TO_PERCENT_TABLE = {
    {  0, 1300, 3500, 4200, 4800, 5650, 6000,10000,16000,22000,30000}, -- capacity (mAh)
    {  0,    3,    6,    6,    6,    6,    6,    6,   12,   12,   14}, -- Num cells
    {  0,  0.1,0.025,0.020,0.019,0.018,0.020,0.015,0.030,0.026,0.025}, -- percent per meter                   
    {  0,  0.2,0.075,0.060,0.057,0.055,0.060,0.045,0.090,0.080,0.070}, -- percent per meter down
    {  0, 0.10, 0.10, 0.07, 0.06, 0.20, 0.07, 0.07, 0.07, 0.07, 0.07}, -- voltage drop when flying (per cell)
    {2.7,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0},
    {2.8,    0,    0,    0,    0,    2,    0,    0,    0,    0,    0},
    {2.9,    0,    0,    0,    0,    6,    0,    0,    0,    0,    0},
    {3.0,    0,    0,    0,    0,   10,    0,    0,    0,    0,    0},
    {3.1,    0,    0,    0,    0,   15,    0,    0,    0,    0,    0},
    {3.2,    0,    0,    0,    0,   24,    0,    0,    0,    0,    0},
    {3.3,    0,    0,    0,    0,   34,    0,    0,    0,    0,    2},
    {3.4,    0,    0,    0,    0,   42,    1,    4,    2,    2,    7},
    {3.5,   10,    4,    2,    2,   50,    2,   26,    6,    4,   21},
    {3.6,   46,    9,   11,   12,   59,    6,   44,   11,   12,   40},
    {3.7,   72,   39,   42,   38,   71,   28,   59,   38,   46,   52},
    {3.8,   85,   62,   65,   56,   94,   58,   70,   62,   66,   61},
    {3.9,   97,   77,   79,   69,   99,   75,   79,   77,   79,   70},
    {4.0,   99,   88,   90,   79,  100,   86,   90,   88,   90,   79},
    {4.1,  100,  100,  100,   89,  100,  100,  100,  100,  100,   89},
    {4.2,  100,  100,  100,   98,  100,  100,  100,  100,  100,  100},
    {4.3,  100,  100,  100,  100,  100,  100,  100,  100,  100,  100},
}

-- %/m = 1/(SPEED[m/s]*36*BATT_CAPACITY[Ah]/CRUISE_CURRENT[A])
local MIN_SAFE_PERCENT = 8  -- Minimum battery percentage needed to return home
local LAND_PERCENTAGE = 5
local MINIMUM_RTL_DIST = 30  -- Minimum distance from home to trigger RTL
local MINIMUM_ARM_PERCENT = 25

----------------------------------------------
--- script start, don't change below this line
----------------------------------------------
local UPDATE_VOLTAGE_MS = 200
local VOLTAGES_SAMPLES_TO_MEDIAN = 51
local RTL_MODE = 6
local LAND_MODE = 9
local PARAM_TABLE_KEY = 82
local BATT_CAPACITY = Parameter("BATT_CAPACITY")

-- Variables
local voltage_samples = {}
local sample_count = 0
local batt_id = -1
local rtl_engaged = false
local land_engaged = false
local cells_num = 6
local voltage_drop = 0
local last_sent_percent = nil

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

    -- Battery is discharging: never let the reported percentage increase
    if percent >= 0 then
        if last_sent_percent and percent > last_sent_percent then
            percent = last_sent_percent
        end
        last_sent_percent = percent
    end

    -- return if not armed
    if not arming:is_armed() then
        rtl_engaged = false
        land_engaged = false
        sample_count = 0
        -- Override battery percentage for GCS
        battery:override_percentage(0, percent)
        return update, 1000
    end

    -- Override battery percentage for GCS
    battery:override_percentage(0, percent)

    -- return if battery is not correctly set
    if percent < 0 or batt_id < 0 then
        return update, 1000
    end

    -- return if not enabled
    if RTLS_ENABLE:get() < 1 then
        return update, 1000
    end

    -- return if already in LAND
    local mode = vehicle:get_mode()
    if mode == LAND_MODE or land_engaged then
        return update, 1000
    end

    if percent < LAND_PERCENTAGE then
        gcs:send_text(3, string.format("Low battery! %.1f%% remaining, start LAND", percent))
        vehicle:set_mode(LAND_MODE)
        land_engaged = true
        return update, UPDATE_VOLTAGE_MS
    end

    -- return if already in RTL
    if mode == RTL_MODE or rtl_engaged then
        return update, 1000
    end

    -- Get distance from home and altitude
    local home = ahrs:get_home()
    local current_loc = ahrs:get_position()
    local negative_alt = ahrs:get_relative_position_D_home()

    if home and current_loc and negative_alt then
        local alt = -negative_alt
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
        local required_percent_horizontal = distance_m * VOLTAGE_TO_PERCENT_TABLE[3][batt_id]
        local required_percent_down = alt * VOLTAGE_TO_PERCENT_TABLE[4][batt_id]
        local required_percent = required_percent_horizontal + required_percent_down

        -- trigger RTL if battery is low
        if percent < (required_percent + MIN_SAFE_PERCENT) then
            gcs:send_text(3, string.format("Low battery! %.1f%% remaining, start RTL", percent))
            vehicle:set_mode(RTL_MODE)
            rtl_engaged = true
        end
    end

    return update, UPDATE_VOLTAGE_MS
end

-- Function to convert voltage to percentage using lookup table
function voltage_to_percent(voltage)
    local batt_capacity = BATT_CAPACITY:get()
    if batt_capacity <= 0 then
        gcs:send_text(4, "rtl_dist: invalid BATT_CAPACITY")
        return -1
    end
    if batt_id < 0 or batt_capacity ~= VOLTAGE_TO_PERCENT_TABLE[1][batt_id] then
        -- batt id is not set or user change BATT_CAPACITY parameter
        -- look for battery capacity in LOT, update batt_id and
        -- calculate minimum arming voltage
        batt_id = -1
        -- new battery: reset monotonic percentage tracker
        last_sent_percent = nil
        for i = 2, #VOLTAGE_TO_PERCENT_TABLE[1] do
            if batt_capacity == VOLTAGE_TO_PERCENT_TABLE[1][i] then
                batt_id = i
                break
            end
        end
        if batt_id == -1 then
            gcs:send_text(4, "rtl_dist: BATT_CAPACITY not in battery list")
            return -1
        end

        -- update cells number and voltage drop
        cells_num = VOLTAGE_TO_PERCENT_TABLE[2][batt_id]
        voltage_drop = VOLTAGE_TO_PERCENT_TABLE[5][batt_id]

        -- set minimum arming voltage
        for i = 6, #VOLTAGE_TO_PERCENT_TABLE - 1 do
            local v1 = VOLTAGE_TO_PERCENT_TABLE[i][1]
            local p1 = VOLTAGE_TO_PERCENT_TABLE[i][batt_id]
            local v2 = VOLTAGE_TO_PERCENT_TABLE[i+1][1]
            local p2 = VOLTAGE_TO_PERCENT_TABLE[i+1][batt_id]
            if MINIMUM_ARM_PERCENT >= p1 and MINIMUM_ARM_PERCENT <= p2 then
                -- Linear interpolation
                local arm_volt = v1 + (MINIMUM_ARM_PERCENT - p1) * (v2 - v1) / (p2 - p1)
                -- remove voltage drop and multiply for cells number
                arm_volt = (arm_volt + voltage_drop) * cells_num
                param:set("BATT_ARM_VOLT", arm_volt)
                break
            end
        end
    end

    -- divide for number of cells
    voltage = voltage / cells_num

    -- remove voltage drop if we're not armed:
    -- the lookup table is calculated from a flight log
    if not vehicle:get_likely_flying() then
        voltage = voltage - voltage_drop
    end

    -- Handle edge cases
    if voltage <= VOLTAGE_TO_PERCENT_TABLE[6][1] then
        return VOLTAGE_TO_PERCENT_TABLE[6][batt_id]
    elseif voltage >= VOLTAGE_TO_PERCENT_TABLE[#VOLTAGE_TO_PERCENT_TABLE][1] then
        return VOLTAGE_TO_PERCENT_TABLE[#VOLTAGE_TO_PERCENT_TABLE][batt_id]
    end

    -- Find the two closest voltage entries in the table
    for i = 6, #VOLTAGE_TO_PERCENT_TABLE - 1 do
        local v1 = VOLTAGE_TO_PERCENT_TABLE[i][1]
        local p1 = VOLTAGE_TO_PERCENT_TABLE[i][batt_id]
        local v2 = VOLTAGE_TO_PERCENT_TABLE[i+1][1]
        local p2 = VOLTAGE_TO_PERCENT_TABLE[i+1][batt_id]

        if voltage >= v1 and voltage <= v2 then
            -- Linear interpolation
            return p1 + (voltage - v1) * (p2 - p1) / (v2 - v1)
        end
    end

    -- should be impossible to get here
    gcs:send_text(4, "rtl_dist: voltage out of range")
    return -1
end

gcs:send_text(6, "rtl_dist.lua is running")
return update, 1000
