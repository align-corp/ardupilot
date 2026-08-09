--[[
   Device driver for the Vicor BCM6135CD1E5165yzz isolated fixed-ratio bus
   converter (384V HI side -> 48V LO side, K = 1/8), talking PMBus over I2C.

   The converter is presented to ArduPilot as a scripting battery monitor
   (BATTn_MONITOR = 29) so that voltage, current and temperature are visible
   on the GCS and are integrated into consumed mAh/Wh as usual. Optionally a
   second battery instance reports the HI-side (384V) bus voltage.

   Telemetry ends up in the normal BAT log messages for those instances; the
   driver does not write a log message of its own. PMBus faults are reported
   to the GCS as they change.

   Wiring: DATA -> SDA, CLK -> SCL, SGND -> autopilot GND. The PMBus interface
   is referenced to the LO side. Pull-ups to 3V3 or 5V are required (they are
   not internal to the BCM). The ADDR terminal must be strapped to SGND with
   the resistor that selects the address set in BCM_ADDR (see table below).

   Reference: BCM6135CD1E5165yzz datasheet Rev 1.2, 04/2024.
--]]

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local PARAM_TABLE_KEY = 83
local PARAM_TABLE_PREFIX = "BCM_"

local function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value),
          string.format('BCM: could not add param %s', name))
   return Parameter(PARAM_TABLE_PREFIX .. name)
end

assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 6), 'BCM: could not add param table')

--[[
  // @Param: BCM_ENABLE
  // @DisplayName: Enable Vicor BCM bus converter driver
  // @Description: Enable Vicor BCM6135 PMBus telemetry driver
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local BCM_ENABLE  = bind_add_param('ENABLE',  1, 1)

--[[
  // @Param: BCM_BUS
  // @DisplayName: BCM I2C bus
  // @Description: I2C bus the BCM PMBus interface is wired to
  // @Range: 0 3
  // @User: Standard
--]]
local BCM_BUS     = bind_add_param('BUS',     2, 0)

--[[
  // @Param: BCM_ADDR
  // @DisplayName: BCM I2C address
  // @Description: 7-bit PMBus address of the BCM, selected by the ADDR resistor. 80..95 (0x50..0x5F). Datasheet resistor values: 0x50=487R 0x51=1050R 0x52=1870R 0x53=2800R 0x54=3920R 0x55=5230R 0x56=6810R 0x57=8870R 0x58=11300R 0x59=14700R 0x5A=19100R 0x5B=25500R 0x5C=35700R 0x5D=53600R 0x5E=97600R 0x5F=316000R
  // @Range: 80 95
  // @User: Standard
--]]
local BCM_ADDR    = bind_add_param('ADDR',    3, 0x57)

--[[
  // @Param: BCM_LO_IDX
  // @DisplayName: BCM low side battery index
  // @Description: Battery instance (1 = BATT, 2 = BATT2, ...) reporting the 48V LO side. That instance must have BATTn_MONITOR = 29. Set to 0 to disable.
  // @Range: 0 9
  // @User: Standard
--]]
local BCM_LO_IDX  = bind_add_param('LO_IDX',  4, 3)

--[[
  // @Param: BCM_HI_IDX
  // @DisplayName: BCM high side battery index
  // @Description: Battery instance (1 = BATT, 2 = BATT2, ...) reporting the 384V HI side bus voltage. That instance must have BATTn_MONITOR = 29. Set to 0 to disable. The BCM cannot measure HI side current, so no current is reported for this instance.
  // @Range: 0 9
  // @User: Standard
--]]
local BCM_HI_IDX  = bind_add_param('HI_IDX',  5, 4)

--[[
  // @Param: BCM_OPTIONS
  // @DisplayName: BCM driver options
  // @Description: Vicor BCM driver options.
  // @User: Advanced
--]]
local BCM_OPTIONS = bind_add_param('OPTIONS', 6, 0)
local _ = BCM_OPTIONS   -- reserved, not read by the driver yet

if BCM_ENABLE:get() == 0 then
   return
end

-- PMBus command codes (datasheet "Supported Command List", p.22)
local CMD_CLEAR_FAULTS       = 0x03
local CMD_STATUS_WORD        = 0x79
local CMD_READ_VIN           = 0x88
local CMD_READ_VOUT          = 0x8B
local CMD_READ_IOUT          = 0x8C
local CMD_READ_TEMP_1        = 0x8D
local CMD_PMBUS_REVISION     = 0x98

--[[
   Direct data format scaling, X = (1/m) * (Y * 10^-R - b), with the m/R/b
   coefficients from the "Reported DATA Formats" table on datasheet p.21.
   Note that the summary table on p.8 claims 10^-1 for READ_VOUT; that
   contradicts both the coefficient table on p.21 and the READ_VOUT command
   description on p.31, which both give 10^-2. We follow the latter two.
   The startup banner prints VIN, VOUT and VOUT/VIN so a wrong scaling is
   immediately obvious on the bench (the ratio must come out at K = 1/8).
--]]
local SCALE_VIN  = 0.1    -- READ_VIN            m=1 R=1
local SCALE_VOUT = 0.01   -- READ_VOUT           m=1 R=2
local SCALE_IOUT = 0.01   -- READ_IOUT           m=1 R=2
local SCALE_TEMP = 1.0    -- READ_TEMPERATURE_1  m=1 R=0

-- 100kHz rather than the 400kHz the BCM supports: the chassis-mount module is
-- reached over a signal harness, so keep some margin on bus capacitance.
local BUS_CLOCK = 100000
local UPDATE_MS = 500          -- 2Hz, well inside the 5s scripting battery timeout
local FAIL_LIMIT = 5           -- consecutive failed reads before reporting lost comms

local dev = i2c:get_device(BCM_BUS:get(), BCM_ADDR:get(), BUS_CLOCK, false)
if not dev then
   gcs:send_text(MAV_SEVERITY.ERROR, "BCM: failed to open I2C device")
   return
end
dev:set_retries(2)

--[[
   PMBus read byte: write the command code, repeated start, read one byte.
--]]
local function read_byte(cmd)
   return dev:read_registers(cmd)
end

--[[
   PMBus read word: low byte first (datasheet "Read Word protocol", p.24).
   Returns nil on a bus error.
--]]
local function read_word_raw(cmd)
   local t = dev:read_registers(cmd, 2)
   if not t then
      return nil
   end
   return t[1] | (t[2] << 8)
end

--[[
   As above, but for the telemetry registers, which read back the 0xFFFF
   "no data yet" default listed on p.22 before the first conversion.
   Not used for the status registers, where 0xFFFF is a legal value.
--]]
local function read_word(cmd)
   local raw = read_word_raw(cmd)
   if raw == 0xFFFF then
      return nil
   end
   return raw
end

-- Y is a two's complement integer in the direct format
local function to_signed16(v)
   if v >= 0x8000 then
      return v - 0x10000
   end
   return v
end

--[[
   Startup probe. PMBUS_REVISION reads back 0x33 (PMBus 1.3 part I and II)
   on this part, which is a cheap confirmation that we are talking to a BCM
   and not to some other device that happens to ACK the address.
--]]
local function probe()
   local rev = read_byte(CMD_PMBUS_REVISION)
   if not rev then
      return false, "no response"
   end
   if rev ~= 0x33 then
      return false, string.format("unexpected PMBUS_REVISION 0x%02x", rev)
   end
   return true, nil
end

-- Names of the STATUS_WORD bits that this part actually supports (p.28).
-- Unsupported bits from the PMBus spec are deliberately absent, so they stay
-- unnamed and are ignored if they ever come back set.
local STATUS_WORD_BITS = {
   [7]  = "BUSY",
   [6]  = "OFF",
   [4]  = "IOUT_OC_FAULT",
   [3]  = "VIN_UV_FAULT",
   [2]  = "TEMP_FAULT",
   [1]  = "CML",
   [10] = "INPUT_FAULT",
   [9]  = "IOUT_POUT_FAULT",
   [11] = "MFR_FAULT",
   [12] = "POWER_GOOD_NEGATED",
}

-- STATUS_WORD bits that mean the powertrain is in trouble, as opposed to
-- merely being off or busy. Sets the severity of the GCS status message.
local FAULT_MASK = (1<<4) | (1<<3) | (1<<2) | (1<<9) | (1<<10) | (1<<11)

local function status_word_text(sw)
   local parts = {}
   for bit = 0, 15 do
      if (sw & (1 << bit)) ~= 0 then
         local name = STATUS_WORD_BITS[bit]
         if name then
            parts[#parts+1] = name
         end
      end
   end
   if #parts == 0 then
      return "none"
   end
   return table.concat(parts, ",")
end

local fail_count = 0
local comms_ok = false
local reported_comms_fail = false
local last_status_word = 0
local warned_lo_idx = false
local warned_hi_idx = false

--[[
   Push one set of readings into a scripting battery instance. Returns whether
   the instance took them (false means BATTn_MONITOR is not 29)
   The instance is always reported healthy
--]]
local function publish(idx, volts, amps, temp)
   if idx <= 0 then
      return true
   end
   local state = BattMonitorScript_State()
   state:healthy(true)
   state:voltage(volts or 0)
   if amps then
      state:current_amps(amps)
   end
   if temp then
      state:temperature(temp)
   end
   return battery:handle_scripting(idx-1, state)
end

local function update()
   local lo_idx = math.floor(BCM_LO_IDX:get())
   local hi_idx = math.floor(BCM_HI_IDX:get())

   local vout_raw = read_word(CMD_READ_VOUT)
   local iout_raw = read_word(CMD_READ_IOUT)
   local temp_raw = read_word(CMD_READ_TEMP_1)
   local status   = read_word_raw(CMD_STATUS_WORD)
   local vin_raw  = (hi_idx > 0) and read_word(CMD_READ_VIN) or nil

   local vout, iout, temp, vin

   -- Voltage is the one reading we cannot do without
   if vout_raw == nil or status == nil then
      fail_count = fail_count + 1
      if fail_count < FAIL_LIMIT then
         return update, UPDATE_MS
      end
      comms_ok = false
      if not reported_comms_fail then
         gcs:send_text(MAV_SEVERITY.ERROR, "BCM: lost PMBus communication")
         reported_comms_fail = true
      end
      vout, vin = 0, 0
   else
      if not comms_ok then
         comms_ok = true
         if reported_comms_fail then
            gcs:send_text(MAV_SEVERITY.INFO, "BCM: PMBus communication restored")
            reported_comms_fail = false
         end
      end
      fail_count = 0

      vout = vout_raw * SCALE_VOUT
      iout = iout_raw and (iout_raw * SCALE_IOUT)
      temp = temp_raw and (to_signed16(temp_raw) * SCALE_TEMP)
      vin  = vin_raw  and (vin_raw  * SCALE_VIN) or 0

      -- Report status changes to the GCS as they happen
      if status ~= last_status_word then
         local severity = ((status & FAULT_MASK) ~= 0) and MAV_SEVERITY.ERROR or MAV_SEVERITY.INFO
         gcs:send_text(severity, string.format("BCM: status 0x%04x %s", status, status_word_text(status)))
         last_status_word = status
      end
   end

   if not publish(lo_idx, vout, iout, temp) and not warned_lo_idx then
      gcs:send_text(MAV_SEVERITY.ERROR,
                    string.format("BCM: BATT%d_MONITOR must be 29", lo_idx))
      warned_lo_idx = true
   end

   -- The BCM has no READ_IIN, so the HI side gets voltage and temperature only
   if not publish(hi_idx, vin, nil, temp) and not warned_hi_idx then
      gcs:send_text(MAV_SEVERITY.ERROR,
                    string.format("BCM: BATT%d_MONITOR must be 29", hi_idx))
      warned_hi_idx = true
   end

   return update, UPDATE_MS
end

--[[
   Startup: probe the device, clear any latched faults from the previous power
   cycle, and print a banner with enough numbers to sanity check the scaling.
--]]
local function start()
   local ok, err = probe()
   if not ok then
      gcs:send_text(MAV_SEVERITY.ERROR,
                    string.format("BCM: not found on I2C%d addr 0x%02x (%s)",
                                  math.floor(BCM_BUS:get()), math.floor(BCM_ADDR:get()), err))
      -- keep retrying, the BCM is unpowered until the HI side bus comes up
      return start, 5000
   end

   dev:write_register(CMD_CLEAR_FAULTS, 0)

   local vin_raw = read_word(CMD_READ_VIN)
   local vout_raw = read_word(CMD_READ_VOUT)
   local vin = vin_raw and (vin_raw * SCALE_VIN) or 0
   local vout = vout_raw and (vout_raw * SCALE_VOUT) or 0
   gcs:send_text(MAV_SEVERITY.INFO,
                 string.format("BCM: found on I2C%d 0x%02x Vin %.1fV Vout %.2fV ratio 1/%.1f",
                               math.floor(BCM_BUS:get()), math.floor(BCM_ADDR:get()),
                               vin, vout, (vout > 0) and (vin/vout) or 0))

   return update, UPDATE_MS
end

return start, 1000
