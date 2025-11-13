-- control 4 serial LEDs for navigation lights - simplified version
-- set parameter SERVOx_FUNCTION to 94 (Script1) to control LEDs
-- the SERVO output must be a GPIO MCU pin (9-14 on PixHawk)

local num_leds = 3
local chan1 = SRV_Channels:find_channel(94)
local chan2 = SRV_Channels:find_channel(95)
local chan3 = SRV_Channels:find_channel(96)
local chan4 = SRV_Channels:find_channel(97)

-- enable parameter
local PARAM_TABLE_KEY = 46
local PARAM_TABLE_PREFIX = "LED_"
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 1), 'could not add param table')
assert(param:add_param(PARAM_TABLE_KEY, 1, 'ENABLE', 1), 'could not add ENABLE param')
local led_enable = Parameter("LED_ENABLE")

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

if not chan1 or not chan2 or not chan3 or not chan4 then
    gcs:send_text(6, "LEDs: channel not set")
    return
end

-- find_channel returns 0 to 15, convert to 1 to 16
chan1 = chan1 + 1
chan2 = chan2 + 1
chan3 = chan3 + 1
chan4 = chan4 + 1

-- setup neopixel LEDs
serialLED:set_num_neopixel(chan1, num_leds)
serialLED:set_num_neopixel(chan2, num_leds)
serialLED:set_num_neopixel(chan3, num_leds)
serialLED:set_num_neopixel(chan4, num_leds)

-- LED position mapping (change these indices to match your physical setup)
local led_positions = {
    front_right = chan2,
    front_left = chan4,
    rear_left = chan3,
    rear_right = chan1
}

-- color functions
function set_green(chan)
    for i = 0, num_leds-1 do
        serialLED:set_RGB(chan, i, 0, 255, 0)
    end
end

function set_orange(chan)
    for i = 0, num_leds-1 do
        serialLED:set_RGB(chan, i, 175, 80, 0)
    end
end

function set_red(chan)
    for i = 0, num_leds-1 do
        serialLED:set_RGB(chan, i, 255, 0, 0)
    end
end

function set_white_low(chan)
    for i = 0, num_leds-1 do
        serialLED:set_RGB(chan, i, 85, 85, 85)
    end
end

function set_white_high(chan)
    for i = 0, num_leds-1 do
        serialLED:set_RGB(chan, i, 255, 255, 255)
    end
end

function set_blue(chan)
    for i = 0, num_leds-1 do
        serialLED:set_RGB(chan, i, 0, 0, 255)
    end
end

function clear_led(chan)
    for i = 0, num_leds-1 do
        serialLED:set_RGB(chan, i, 0, 0, 0)
    end
end

-- set LED color by position and color constant
function set_led_color(position, color)
    local led_index = led_positions[position]
    if color == RED then
        set_red(led_index)
    elseif color == GREEN then
        set_green(led_index)
    elseif color == BLUE then
        set_blue(led_index)
    elseif color == WHITE_LOW then
        set_white_low(led_index)
    elseif color == WHITE_HIGH then
        set_white_high(led_index)
    elseif color == ORANGE then
        set_orange(led_index)
    end
end

-- set all LEDs to specified colors
function set_all_leds(front_color, rear_color)
    set_led_color("front_left", front_color)
    set_led_color("front_right", front_color)
    set_led_color("rear_left", rear_color)
    set_led_color("rear_right", rear_color)
end

-- turn all LEDs off
function leds_off()
    for i = 0, num_leds - 1 do
        clear_led(i)
    end
end

-- main LED control function
function led()
    -- check if LEDs are enabled
    if led_enable:get() < 1 then
        leds_off()
        serialLED:send(chan)
        return led, 150
    end

    -- not armed and prearm checks are failing: red lights
    if not arming:is_armed() and not arming:pre_arm_checks() then
        set_all_leds(RED, RED)
    -- ALTITUDE HOLD: 1 white blink on green background
    elseif vehicle:get_mode() == ALTHOLD_MODE then
        if count == 0 then
            set_all_leds(WHITE_HIGH, WHITE_HIGH)
        else
            set_all_leds(WHITE_LOW, GREEN)
        end
    -- LOITER: 2 white blinks on green background
    elseif vehicle:get_mode() == LOITER_MODE then
        if count == 0 or count == 2 then
            set_all_leds(WHITE_HIGH, WHITE_HIGH)
        else
            set_all_leds(WHITE_LOW, GREEN)
        end
    -- AUTO or GUIDED: 2 white blinks on blue background
    elseif vehicle:get_mode() == AUTO_MODE or vehicle:get_mode() == GUIDED_MODE or vehicle:get_mode() == FOLLOW_MODE then
        if count == 0 or count == 2 then
            set_all_leds(WHITE_HIGH, WHITE_HIGH)
        else
            set_all_leds(WHITE_LOW, BLUE)
        end
    -- RTL: 2 white blinks on blue background
    elseif vehicle:get_mode() == RTL_MODE then
        if count == 0 or count == 2 then
            set_all_leds(WHITE_HIGH, WHITE_HIGH)
        else
            set_all_leds(WHITE_LOW, ORANGE)
        end
    -- LAND: 3 white blinks on blue background
    elseif vehicle:get_mode() == LAND then
        if count == 0 or count == 2 or count == 4 then
            set_all_leds(WHITE_HIGH, WHITE_HIGH)
        else
            set_all_leds(WHITE_LOW, ORANGE)
        end
    -- unknown mode: orange lights
    else
        set_all_leds(ORANGE, ORANGE)
    end

    -- update count
    count = count + 1
    if count > 9 then
        count = 0
    end

    -- send LED update
    serialLED:send(chan1)
    serialLED:send(chan2)
    serialLED:send(chan3)
    serialLED:send(chan4)
    return led, 150
end


gcs:send_text(5, "LED navigation script is running")
return led, 100

