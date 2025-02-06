local BATTERY_INSTANCE = 0
local BATTERY_CELLS = 12
local VOLT_TO_PERC = {-2964.1, 1369.6, -152.37}
local count = 1
local volts = {0,0,0,0,0,0,0}
local volt
function read_voltage()
v = battery:voltage(BATTERY_INSTANCE)/BATTERY_CELLS
if v < 3 then
count = 1
return read_voltage, 500
end
volts[count] = v
if count < 7 then
count = count+1
return read_voltage, 200
end
table.sort(volts)
volt = (volts[3]+volts[4]+volts[5])/3
return set_perc, 10
end
function set_perc()
percentage = VOLT_TO_PERC[1] + volt*VOLT_TO_PERC[2] + volt*volt*VOLT_TO_PERC[3]
if percentage > 100 then
percentage = 100
elseif percentage < 0 then
percentage = 0
end
battery:reset_remaining(BATTERY_INSTANCE, percentage)
end
return read_voltage, 100