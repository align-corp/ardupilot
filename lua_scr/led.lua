-- control serial LEDs for navigation lights - unified version for M3 and M460
-- set parameter SERVOx_FUNCTION to 94 (Script1) to control LEDs
-- the SERVO output must be a GPIO MCU pin (9-14 on PixHawk)

-- enable parameters
local PARAM_TABLE_KEY = 46
local PARAM_TABLE_PREFIX = "LED_"
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 2), 'could not add param table')
assert(param:add_param(PARAM_TABLE_KEY, 1, 'ENABLE', 1), 'could not add ENABLE param')
local led_enable = Parameter("LED_ENABLE")

-- align_vehicles
local M3 = 0
local M4 = 1

-- flight mode constants
local LOITER_MODE = 5
local ALTHOLD_MODE = 2
local RTL_MODE = 6
local AUTO_MODE = 3
local GUIDED_MODE = 4
local LAND = 9
local FOLLOW_MODE = 23

-- color constants
local RED = 0
local GREEN = 1
local BLUE = 2
local WHITE_LOW = 3
local WHITE_HIGH = 4
local ORANGE = 5

-- global variables
local count = 0
local channels = {}
local num_leds_per_channel
local align_vehicle

-- configuration based on vehicle type
local function setup_channels()
    local align_frame = FWVersion:os_sw_version():toint()
    if align_frame > 0x03010000 and align_frame < 0x03070000 then
        -- AP6-M450 AP6-M460 AP6-M490
        align_vehicle = M4
    elseif align_frame > 0x04070000 and align_frame < 0x04080000 then
        -- AP6m-M3
        align_vehicle = M3
    end

    if align_vehicle == M3 then
        local chan = SRV_Channels:find_channel(94)
        if not chan then
            gcs:send_text(6, "LEDs: M3 channel not set")
            return false
        end
        channels = {
            front_right = {chan = chan + 1, led = 0},
            front_left = {chan = chan + 1, led = 1},
            rear_left = {chan = chan + 1, led = 2},
            rear_right = {chan = chan + 1, led = 3}
        }
        num_leds_per_channel = 4
        serialLED:set_num_neopixel(chan + 1, num_leds_per_channel)
        gcs:send_text(6, "LED: M3")
        return true

    elseif align_vehicle == M4 then
        local chan1 = SRV_Channels:find_channel(94)
        local chan2 = SRV_Channels:find_channel(95)
        local chan3 = SRV_Channels:find_channel(96)
        local chan4 = SRV_Channels:find_channel(97)

        if not chan1 or not chan2 or not chan3 or not chan4 then
            gcs:send_text(6, "LEDs: M460 channels not set")
            return false
        end
        channels = {
            front_right = {chan = chan2 + 1, led = nil},
            front_left = {chan = chan4 + 1, led = nil},
            rear_left = {chan = chan3 + 1, led = nil},
            rear_right = {chan = chan1 + 1, led = nil}
        }
        num_leds_per_channel = 3
        serialLED:set_num_neopixel(chan1 + 1, num_leds_per_channel)
        serialLED:set_num_neopixel(chan2 + 1, num_leds_per_channel)
        serialLED:set_num_neopixel(chan3 + 1, num_leds_per_channel)
        serialLED:set_num_neopixel(chan4 + 1, num_leds_per_channel)
        gcs:send_text(6, "LED: M460")
        return true
    end
    gcs:send_text(5, "LED: unknown vehicle, stopping")
    return false
end

-- color setting functions
local color_values = {
    [RED] = {255, 0, 0},
    [GREEN] = {0, 255, 0},
    [BLUE] = {0, 0, 255},
    [WHITE_LOW] = {85, 85, 85},
    [WHITE_HIGH] = {255, 255, 255},
    [ORANGE] = {175, 80, 0}
}

local function set_channel_color(chan, led_index, color)
    local rgb = color_values[color]
    if not rgb then return end

    if align_vehicle == M3 then
        serialLED:set_RGB(chan, led_index, rgb[1], rgb[2], rgb[3])
    elseif align_vehicle == M4 then
        for i = 0, num_leds_per_channel - 1 do
            serialLED:set_RGB(chan, i, rgb[1], rgb[2], rgb[3])
        end
    end
end

local function clear_channel(chan, led_index)
    if align_vehicle == M3 then
        serialLED:set_RGB(chan, led_index, 0, 0, 0)
    elseif align_vehicle == M4 then
        for i = 0, num_leds_per_channel - 1 do
            serialLED:set_RGB(chan, i, 0, 0, 0)
        end
    end
end

-- set LED color by position
local function set_led_color(position, color)
    local ch_info = channels[position]
    if ch_info then
        set_channel_color(ch_info.chan, ch_info.led, color)
    end
end

-- set all LEDs to specified colors
local function set_all_leds(front_color, rear_color)
    set_led_color("front_left", front_color)
    set_led_color("front_right", front_color)
    set_led_color("rear_left", rear_color)
    set_led_color("rear_right", rear_color)
end

-- turn all LEDs off
local function leds_off()
    for _, ch_info in pairs(channels) do
        clear_channel(ch_info.chan, ch_info.led)
    end
end

-- send updates to all channels
local function send_led_updates()
    if align_vehicle == M3 then
        local ch_info = channels.front_right -- any channel will work since they're all the same
        serialLED:send(ch_info.chan)
    elseif align_vehicle == M4 then
        for _, ch_info in pairs(channels) do
            serialLED:send(ch_info.chan)
        end
    end
end

-- main LED control function
function led()
    -- check if LEDs are enabled
    if led_enable:get() < 1 or notify:is_powering_off() then
        leds_off()
        send_led_updates()
        return led, 150
    end

    local mode = vehicle:get_mode()

    -- determine LED pattern based on flight mode
    if not arming:is_armed() and not arming:pre_arm_checks() then
        set_all_leds(RED, RED)
    elseif mode == ALTHOLD_MODE then
        -- 1 white blink on green background
        if count == 0 then
            set_all_leds(WHITE_HIGH, WHITE_HIGH)
        else
            set_all_leds(WHITE_LOW, GREEN)
        end
    elseif mode == LOITER_MODE then
        -- 2 white blinks on green background
        if count == 0 or count == 2 then
            set_all_leds(WHITE_HIGH, WHITE_HIGH)
        else
            set_all_leds(WHITE_LOW, GREEN)
        end
    elseif mode == AUTO_MODE or mode == GUIDED_MODE or mode == FOLLOW_MODE then
        -- 2 white blinks on blue background
        if count == 0 or count == 2 then
            set_all_leds(WHITE_HIGH, WHITE_HIGH)
        else
            set_all_leds(WHITE_LOW, BLUE)
        end
    elseif mode == RTL_MODE then
        -- 2 white blinks on orange background
        if count == 0 or count == 2 then
            set_all_leds(WHITE_HIGH, WHITE_HIGH)
        else
            set_all_leds(WHITE_LOW, ORANGE)
        end
    elseif mode == LAND then
        -- 3 white blinks on orange background
        if count == 0 or count == 2 or count == 4 then
            set_all_leds(WHITE_HIGH, WHITE_HIGH)
        else
            set_all_leds(WHITE_LOW, ORANGE)
        end
    else
        -- unknown mode: orange lights
        set_all_leds(ORANGE, ORANGE)
    end

    -- update count for blinking pattern
    count = count + 1
    if count > 9 then
        count = 0
    end

    send_led_updates()
    return led, 150
end

-- initialize and start
if setup_channels() then
    return led, 100
else
    return
end
