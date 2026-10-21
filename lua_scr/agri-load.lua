-- read amplified load cells on analog pins and feed their weights
-- into a scripting battery driver via the cell_voltages array
--
-- It also scales the EKF3 drag model with the payload, so the wind estimate
-- (and the baro wind compensation that uses it) stays right as the tank
-- empties. LOAD_BCOEF_X/Y and LOAD_MCOEF hold the coefficients fitted with an
-- empty tank, and for a total mass m = LOAD_EMPTY_KG + payload:
--   EK3_DRAG_BCOEF_x = LOAD_BCOEF_x * m / LOAD_EMPTY_KG  (mass / drag area)
--   EK3_DRAG_MCOEF   = LOAD_MCOEF * sqrt(LOAD_EMPTY_KG / m)
-- rotor momentum drag force grows with sqrt(thrust), so its acceleration
-- falls with 1/sqrt(m). The EK3 values are set but never saved, so after a
-- reboot they still hold whatever was saved before
-- version 0.3

-- user constants
local MILLIS_UPDATE = 1000
local LOAD_PINS = {10, 11, 12, 13} -- analog pins num
local NUM_CELLS = #LOAD_PINS
-- at boot, a total weight above this (g) with the saved offsets means the
-- drone is loaded, so the saved offsets are kept instead of recalibrating
local BOOT_LOADED_GRAMS = 3000
-- in flight the cells read m * (g + vertical accel), filter that out. The
-- payload only changes over minutes so a slow filter costs nothing
local DRAG_FILTER_TC_S = 10
-- rewrite the EK3 coefficients only when the mass moves more than this
local DRAG_UPDATE_RATIO = 0.01
-- bound a bad load reading, max payload as a multiple of the empty mass
local DRAG_MAX_MASS_RATIO = 2.5

-- parameters
local PARAM_TABLE_KEY = 50
local PARAM_TABLE_PREFIX = "LOAD"

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

-- 1 (battery index) + 2 params per cell + 4 drag scaling
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 1 + 2 * NUM_CELLS + 4), 'could not add param table')

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

local DRAG_PARAM_IDX = 2 * NUM_CELLS + 2
--[[
  // @Param: LOAD_EMPTY_KG
  // @DisplayName: Empty drone mass
  // @Description: Take-off mass with an empty tank, the load cell payload is added to it to scale the EKF3 drag coefficients. Set to 0 to leave EK3_DRAG_BCOEF_X, EK3_DRAG_BCOEF_Y and EK3_DRAG_MCOEF alone
  // @Units: kg
  // @User: Standard
--]]
local LOAD_EMPTY_KG = bind_add_param('_EMPTY_KG', DRAG_PARAM_IDX, 34)
--[[
  // @Param: LOAD_BCOEF_X
  // @DisplayName: Empty ballistic coefficient X
  // @Description: EK3_DRAG_BCOEF_X fitted with an empty tank, scaled up with the payload. 0 leaves EK3_DRAG_BCOEF_X alone
  // @Units: kg/m/m
  // @User: Advanced
--]]
local LOAD_BCOEF_X = bind_add_param('_BCOEF_X', DRAG_PARAM_IDX + 1, 101)
--[[
  // @Param: LOAD_BCOEF_Y
  // @DisplayName: Empty ballistic coefficient Y
  // @Description: EK3_DRAG_BCOEF_Y fitted with an empty tank, scaled up with the payload. 0 leaves EK3_DRAG_BCOEF_Y alone
  // @Units: kg/m/m
  // @User: Advanced
--]]
local LOAD_BCOEF_Y = bind_add_param('_BCOEF_Y', DRAG_PARAM_IDX + 2, 108)
--[[
  // @Param: LOAD_MCOEF
  // @DisplayName: Empty momentum drag coefficient
  // @Description: EK3_DRAG_MCOEF fitted with an empty tank, scaled down with the payload. 0 leaves EK3_DRAG_MCOEF alone
  // @Units: 1/s
  // @User: Advanced
--]]
local LOAD_MCOEF = bind_add_param('_MCOEF', DRAG_PARAM_IDX + 3, 0.14)

-- EK3 drag parameters, bound in two steps as a build may lack EKF3 drag fusion
local ek3_bcoef_x = Parameter()
local ek3_bcoef_y = Parameter()
local ek3_mcoef = Parameter()
local ek3_drag_ok = ek3_bcoef_x:init('EK3_DRAG_BCOEF_X') and
                    ek3_bcoef_y:init('EK3_DRAG_BCOEF_Y') and
                    ek3_mcoef:init('EK3_DRAG_MCOEF')
if not ek3_drag_ok then
    gcs:send_text(4, "agri-load: no EK3 drag params, drag scaling off")
end

-- setup analog channels, one per load cell
local channels = {}
for i = 1, NUM_CELLS do
    channels[i] = analog:channel()
    if not channels[i]:set_pin(LOAD_PINS[i]) then
        gcs:send_text(0, string.format("agri-load: invalid analog pin %d", LOAD_PINS[i]))
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
            gcs:send_text(3, "agri-load: zero cal failed, cell 1 reads 0 V")
            cal_error_sent = true
        end
        return
    end
    for i = 1, NUM_CELLS do
        LOAD_OFFSET[i]:set_and_save(voltages[i])
    end
    cal_error_sent = false
    gcs:send_text(6, "agri-load: zero cal done")
end

-- total weight in grams using the current offsets
function total_grams()
    local total = 0
    for i = 1, NUM_CELLS do
        total = total + (channels[i]:voltage_average() - LOAD_OFFSET[i]:get()) * LOAD_SCALE[i]:get()
    end
    return total
end

local payload_kg_filt = nil     -- filtered payload, nil until the first sample
local drag_mass_ratio = nil     -- mass ratio the EK3 coefficients were last set for

-- set an EK3 drag coefficient from its empty tank reference, a reference of 0
-- means the user manages it by hand
function set_drag_coef(ek3_param, ref_param, factor)
    local ref = ref_param:get()
    if ref <= 0 then
        return
    end
    ek3_param:set(ref * factor)
end

-- scale the EKF3 drag model with the measured payload
function update_drag(payload_kg)
    local empty_kg = LOAD_EMPTY_KG:get()
    if not ek3_drag_ok or empty_kg <= 0 then
        return
    end

    payload_kg = math.max(payload_kg, 0)
    if not payload_kg_filt or not arming:is_armed() then
        -- on the ground follow the reading, so a tank filled before take-off
        -- is right at arming
        payload_kg_filt = payload_kg
    else
        local dt = MILLIS_UPDATE * 0.001
        payload_kg_filt = payload_kg_filt + (dt / (dt + DRAG_FILTER_TC_S)) * (payload_kg - payload_kg_filt)
    end

    local ratio = math.min((empty_kg + payload_kg_filt) / empty_kg, DRAG_MAX_MASS_RATIO)
    if drag_mass_ratio and math.abs(ratio - drag_mass_ratio) < DRAG_UPDATE_RATIO * drag_mass_ratio then
        return
    end
    drag_mass_ratio = ratio

    -- set() and not set_and_save(), redone on every boot
    set_drag_coef(ek3_bcoef_x, LOAD_BCOEF_X, ratio)
    set_drag_coef(ek3_bcoef_y, LOAD_BCOEF_Y, ratio)
    set_drag_coef(ek3_mcoef, LOAD_MCOEF, 1 / math.sqrt(ratio))
end

function update()
    if LOAD_OFFSET[1]:get() == 0 then
        calibrate_zero()
    elseif not boot_check_done then
        -- a negative or small weight with the saved offsets means the drone
        -- is empty and the zero has drifted, so recalibrate
        local g_tot_old = total_grams()
        if g_tot_old > BOOT_LOADED_GRAMS then
            gcs:send_text(6, string.format("agri-load: %.1f kg at boot, keeping saved offsets", g_tot_old * 0.001))
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

    -- the zero is not trusted until a calibration has succeeded
    if LOAD_OFFSET[1]:get() ~= 0 then
        update_drag(total * 0.001)
    end

    return update, MILLIS_UPDATE
end

gcs:send_text('6', "agri-load is running")
-- delay the first update so voltage_average() has real samples before the
-- boot weight check
return update, 2000
