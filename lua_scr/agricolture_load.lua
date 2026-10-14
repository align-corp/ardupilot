-- read amplified load cells on analog pins and feed their weights
-- into a scripting battery driver via the cell_voltages array - version 0.2

-- user constants
local MILLIS_UPDATE = 1000
local LOAD_PINS = {10, 11, 12, 13} -- analog pins num
local NUM_CELLS = #LOAD_PINS
-- at boot, a total weight above this (g) with the saved offsets means the
-- drone is loaded, so the saved offsets are kept instead of recalibrating
local BOOT_LOADED_GRAMS = 3000

-- parameters
local PARAM_TABLE_KEY = 50
local PARAM_TABLE_PREFIX = "LOAD"

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

-- 1 (battery index) + 2 params per cell
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 1 + 2 * NUM_CELLS), 'could not add param table')

--[[
  // @Param: LOAD_BATT_IDX
  // @DisplayName: Load cell battery index
  // @Description: Scripting battery monitor index to feed with the load cell weights
  // @Range: 0 9
  // @User: Standard
--]]
local LOAD_BATT_IDX = bind_add_param('_BATT_IDX', 1, 3)

-- per cell offset and scale: weight = (voltage - LOADn_OFFSET) * LOADn_SCALE
local LOAD_OFFSET = {}
local LOAD_SCALE = {}
for i = 1, NUM_CELLS do
    --[[
      // @Param: LOADn_OFFSET
      // @DisplayName: Load cell n voltage offset
      // @Description: Zero-weight voltage of load cell n, subtracted before scaling. Set LOAD1_OFFSET to 0 with no payload to recalibrate the zero of all cells
      // @Units: V
      // @User: Standard
    --]]
    LOAD_OFFSET[i] = bind_add_param(string.format('%u_OFFSET', i), 2 * i, 0)
    --[[
      // @Param: LOADn_SCALE
      // @DisplayName: Load cell n scale
      // @Description: Weight per volt used to convert load cell n voltage to weight
      // @User: Standard
    --]]
    LOAD_SCALE[i] = bind_add_param(string.format('%u_SCALE', i), 2 * i + 1, 10000) --TODO: add default scale
end

-- setup analog channels, one per load cell
local channels = {}
for i = 1, NUM_CELLS do
    channels[i] = analog:channel()
    if not channels[i]:set_pin(LOAD_PINS[i]) then
        gcs:send_text(0, string.format("agricolture_load: invalid analog pin %d", LOAD_PINS[i]))
    end
end

-- zero calibration runs at boot unless the saved offsets show a payload
-- (see BOOT_LOADED_GRAMS), and whenever the user sets LOAD1_OFFSET to 0 with
-- no payload. It saves the offsets of all cells.
local cal_error_sent = false
local boot_check_done = false

function calibrate_zero()
    local voltages = {}
    for i = 1, NUM_CELLS do
        voltages[i] = channels[i]:voltage_average()
    end
    -- a 0 V reading means no ADC samples or a dead input; saving it would
    -- keep LOAD1_OFFSET at 0 and retrigger, so refuse and retry next update
    if voltages[1] < 0.01 then
        if not cal_error_sent then
            gcs:send_text(3, "agricolture_load: zero cal failed, cell 1 reads 0 V")
            cal_error_sent = true
        end
        return
    end
    for i = 1, NUM_CELLS do
        LOAD_OFFSET[i]:set_and_save(voltages[i])
    end
    cal_error_sent = false
    gcs:send_text(6, "agricolture_load: zero cal done")
end

-- total weight in grams using the current offsets
function total_grams()
    local total = 0
    for i = 1, NUM_CELLS do
        total = total + (channels[i]:voltage_average() - LOAD_OFFSET[i]:get()) * LOAD_SCALE[i]:get()
    end
    return total
end

function update()
    if LOAD_OFFSET[1]:get() == 0 then
        calibrate_zero()
    elseif not boot_check_done then
        -- a negative or small weight with the saved offsets means the drone
        -- is empty and the zero has drifted, so recalibrate
        local g_tot_old = total_grams()
        if g_tot_old > BOOT_LOADED_GRAMS then
            gcs:send_text(6, string.format("agricolture_load: %.1f kg at boot, keeping saved offsets", g_tot_old * 0.001))
        else
            calibrate_zero()
        end
    end
    boot_check_done = true

    local state = BattMonitorScript_State()
    state:healthy(true)
    state:cell_count(NUM_CELLS)

    local total = 0
    for i = 1, NUM_CELLS do
        local voltage = channels[i]:voltage_average()
        -- weight in grams: LOADn_SCALE is the grams-per-volt gain
        local grams = (voltage - LOAD_OFFSET[i]:get()) * LOAD_SCALE[i]:get()
        total = total + grams
        -- cell_voltages is a uint16 array with no range check, clamp to [0, 65535]
        -- to avoid wraparound (max 65535 g = 65.5 kg per cell)
        grams = math.floor(grams + 0.5)
        if grams < 0 then
            grams = 0
        elseif grams > 65535 then
            grams = 65535
        end
        state:cell_voltages(i - 1, grams)
    end

    -- report total for logging
    state:voltage(total * 0.001)

    battery:handle_scripting(LOAD_BATT_IDX:get(), state)

    return update, MILLIS_UPDATE
end

gcs:send_text('6', "Agricolture Load is running")
-- delay the first update so voltage_average() has real samples before the
-- boot weight check
return update, 2000
