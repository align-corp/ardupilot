-- check sprayer level - version 1.0
local MILLIS_UPDATE = 1000
local LEVEL_PIN = 58
gpio:pinMode(LEVEL_PIN,0)
function check_level()
if gpio:read(LEVEL_PIN) then
gcs:send_named_float("level", 1)
else
gcs:send_named_float("level", 0)
end
return check_level, MILLIS_UPDATE
end
gcs:send_text('6', "Agricolture Level is running")
return check_level, 1000
