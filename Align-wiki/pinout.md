# Copter pinout
## M450/M460/M490 4in1 ESC
### Firmwares
- AP6-M450-ds
- AP6-M450-A10-ds
- AP6-M450-nogps-ds
- AP6-M460-ds
- AP6-M460-A10-ds
- AP6-M490-ds
- AP6-M490-A10-ds

### Servo
1. Motor 1
2. Motor 2
3. Motor 3
4. LED 4
5. 
6. Gripper
7. Motor 4
8. LED 1
9. LED 2
10. LED 3

### UART/I2C (G3P)
1. GCS Telemetry
2. Rangefinder MT15 (forward)
3. ESC telemetry
4. PCU
5. G3P gimbal
6. GPS
7. G3P gimbal
8. (I2C) Rangefinder MT15 (downward, configure as I2C)

### UART/I2C (A10)
1. GCS Telemetry
2. 
3. 
4. PCU
5. ESC Telemetry
6. GPS
7. 
8. 

# Heli pinout
## E1
### Servo
1. SERVO 1
2. SERVO 2
3. SERVO 3
4. SERVO 4
5. Fan IO output (Spinner)
6. Servo Gimbal
7. Front (and Down) LED GPIO56
8. ESC
9. Gripper Hook (Pump)
10. RPM fan cooler
11. RPM HW feedback

### UART
1. GCS Telemetry
2. Rangefinder NoopLoop (forward)
3. Rangefinder NoopLoop (downward)
4. PCU P2
5. 2nd GCS Telemetry or G3P gimbal
6. GPS
7. G3P gimbal

# Rover pinout
## GA22
### Servo
1. SERVO - motor left
2. GPIO OUTPUT - blade control
3. SERVO - motor right
4. GPIO OUTPUT - LED searchlight
5. GPIO INPUT - liquid level gauge
6. 
7. SERVO - engine throttle
8. GPIO OUTPUT - engine
9. GPIO OUTPUT - blade
10. GPIO OUTPUT - engine

## GA45
### Servo
1. SERVO - motor left
2. GPIO OUTPUT - blade control
3. SERVO - motor right
4. GPIO OUTPUT - LED searchlight
5. ANALOG INPUT - liquid level gauge
6. 
7. SERVO - engine throttle
8. GPIO OUTPUT - engine
9. GPIO OUTPUT - blade
10. GPIO OUTPUT - engine

## GM22/45
### UART
- UART4 TX (GPIO 1) axis 0 UP
- UART4 RX (GPIO 2) axis 0 DOWN
- UART5 TX (GPIO 3) axis 1 UP
- UART5 RX (GPIO 4) axis 1 DOWN
