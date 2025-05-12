local count = 0
local flip = true
local RELAY_NUM1 = 0
local RELAY_NUM2 = 1
local RELAY_NUM3 = 2
local RELAY_NUM4 = 3
local ALTHOLD_MODE = 2
local RTL_MODE = 6
local AUTO_MODE = 3
local LAND_MODE = 9

function light_on()
    relay:off(RELAY_NUM1)
    relay:off(RELAY_NUM2)
    relay:off(RELAY_NUM3)
    relay:off(RELAY_NUM4)
end
function light_off()
    relay:on(RELAY_NUM1)
    relay:on(RELAY_NUM2)
    relay:on(RELAY_NUM3)
    relay:on(RELAY_NUM4)
end

function led()
    if arming:is_armed() then
        -- if armed different flash pattern with different flight modes
        if fence:get_breaches() ~= 0 then
            if flip then
                light_on()
                flip = false
            else
                light_off()
                flip = true
            end
            return led, 50
        elseif vehicle:get_mode() == ALTHOLD_MODE then
            if count == 0 then
                light_off()
            else
                light_on()
            end
        elseif vehicle:get_mode() == AUTO_MODE then
            if count == 0 or count == 2 or count == 4 then
                light_off()
            else
                light_on()
            end
        elseif vehicle:get_mode() == LAND_MODE or vehicle:get_mode() == RTL_MODE then
            if count == 0 or count == 2 then
                light_off()
            else
                light_on()
            end
        else
            light_on()
        end

        -- arm count
        count = count + 1
        if count > 9 then
            count = 0
    end
    else
        -- circle animation when disarmed
        if count < 3 then
            relay:off(RELAY_NUM1)
            relay:on(RELAY_NUM2)
            relay:on(RELAY_NUM3)
            relay:on(RELAY_NUM4)
        elseif count < 6 then
            relay:off(RELAY_NUM3)
            relay:on(RELAY_NUM1)
            relay:on(RELAY_NUM2)
            relay:on(RELAY_NUM4)
        elseif count < 9 then
            relay:off(RELAY_NUM2)
            relay:on(RELAY_NUM1)
            relay:on(RELAY_NUM3)
            relay:on(RELAY_NUM4)
        elseif count < 12 then
            relay:off(RELAY_NUM4)
            relay:on(RELAY_NUM2)
            relay:on(RELAY_NUM3)
            relay:on(RELAY_NUM1)
        else
            count = -1
        end
        -- disarm count
        count = count + 1
    end

    return led, 250
end

return led, 10
