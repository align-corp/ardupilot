local AUTO_MODE = 3
local MANUAL_MODE = 0
function update()
if vehicle:get_mode() ~= AUTO_MODE then
return update, 1000
end
if gps:status(0) < gps.GPS_OK_FIX_3D_RTK_FLOAT or gps:status(1) < gps.GPS_OK_FIX_3D_RTK_FLOAT then
vehicle:set_mode(MANUAL_MODE)
arming:disarm()
gcs:send_text('0', "Auto mode need RTK")
end
return update, 1000
end
gcs:send_text('6', "stop_auto.lua is running")
return update, 1000
