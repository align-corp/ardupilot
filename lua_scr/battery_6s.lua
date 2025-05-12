-- constants to set
local BATTERY_INSTANCE = 0
local BATTERY_CELLS = 6
local VOLT_TO_PERC = {-2964.1, 1369.6, -152.37} -- voltage to percentage, quadratic equation {1, x, x^2}

-- global variables
local count = 1
local volts = {0,0,0,0,0,0,0}
local volt
local BATT_FULL_CAPACITY = battery:pack_capacity_mah(BATTERY_INSTANCE)

function read_voltage()
    v = battery:voltage(BATTERY_INSTANCE)/BATTERY_CELLS
    -- single cell voltage must be > 3 V, otherwise voltage read is not ready
    if v < 3 then
        count = 1
        --gcs:send_text(5, "battery voltage too low")
        return read_voltage, 500
    end

    -- add voltage read to array
    volts[count] = v

    -- repeat until the voltage array is filled
    if count < 7 then
        count = count+1
        return read_voltage, 200
    end

    -- voltage array us filled, time to sort
    table.sort(volts)

    -- delete outliers and compute average
    volt = (volts[3]+volts[4]+volts[5])/3

    -- set percentage to ArduPilot
    return set_perc, 10
end

function set_perc()
    -- compute percentage
    percentage = VOLT_TO_PERC[1] + volt*VOLT_TO_PERC[2] + volt*volt*VOLT_TO_PERC[3]

    -- check limits
    if percentage > 100 then
        percentage = 100
    elseif percentage < 0 then
        percentage = 0
    end

    -- set battery percentage
    battery:reset_remaining(BATTERY_INSTANCE, percentage)
end

return read_voltage, 100