-- read amplified load cells on analog pins and feed their weights
-- into a scripting battery driver via the cell_voltages array - version 0.1

-- user constants
local MILLIS_UPDATE = 1000
local LOAD_PINS = {5, 9, 15, 10} -- analog pins num
local NUM_CELLS = #LOAD_PINS

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
local LOAD_BATT_IDX = bind_add_param('_BATT_IDX', 1, 0)

-- per cell offset and scale: weight = (voltage - LOADn_OFFSET) * LOADn_SCALE
local LOAD_OFFSET = {}
local LOAD_SCALE = {}
for i = 1, NUM_CELLS do
    --[[
      // @Param: LOADn_OFFSET
      // @DisplayName: Load cell n voltage offset
      // @Description: Zero-weight voltage of load cell n, subtracted before scaling
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
    LOAD_SCALE[i] = bind_add_param(string.format('%u_SCALE', i), 2 * i + 1, 1)
end

-- setup analog channels, one per load cell
local channels = {}
for i = 1, NUM_CELLS do
    channels[i] = analog:channel()
    if not channels[i]:set_pin(LOAD_PINS[i]) then
        gcs:send_text(0, string.format("agricolture_load: invalid analog pin %d", LOAD_PINS[i]))
    end
end

-- capture the zero-weight offsets on the first update(), once the ADC has had
-- time to accumulate samples. Doing this at script load reads 0 V because no
-- conversions have happened yet, which leaves the raw voltage in the output.
local offsets_captured = false

function capture_offsets()
    for i = 1, NUM_CELLS do
        local voltage = channels[i]:voltage_average()
        LOAD_OFFSET[i]:set(voltage)
    end
    offsets_captured = true
    gcs:send_text(6, "agricolture_load: offsets captured")
end

function update()
    if not offsets_captured then
        capture_offsets()
    end

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

    battery:handle_scripting(LOAD_BATT_IDX:get(), state)

    return update, MILLIS_UPDATE
end

gcs:send_text('6', "Agricolture Load is running")
-- delay the first update so voltage_average() has real samples before we
-- capture the offsets
return update, 2000
