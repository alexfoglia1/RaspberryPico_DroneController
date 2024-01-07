# RaspberryPico_DroneController

A self-built quadcopter control software.

## Frame
The quadcopter is built on a [DJI F450 Frame](https://www.researchgate.net/figure/F450-Frame-dimensions_fig2_355023499).

## Motors
Four [Kyrio 1000 KV RC Brushless Motors with 30A ESC](https://amzn.eu/d/81mC5Vf).

## Battery
[Ovonic Air LiPo 3s battery 2200 mAh](https://amzn.eu/d/aYqwFZ2)

## Hardware
1) Controller: [Raspberry Pi Pico](https://www.raspberrypi.com/products/raspberry-pi-pico/)
2) IMU: [Adafruit BNO055](https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/overview)
3) Radio: [FlySky FS-I6 Transmitter](https://www.flysky-cn.com/fsi6) and [FS-IA6 Receiver](https://www.flysky-cn.com/ia6-canshu)
4) Bluetooth UART: [AZ Delivery HC-05 Bluetooth Module](https://amzn.eu/d/cSJ2lOk)

Hardware is placed on a matrix board tied to the central part of the quadcopter frame.
Current setup is using the Adafruit BNO055 IMU but software is compatible with the following units:
1) LSM9DS1
2) MPU6050
3) BNO055

## Software

### Controller
Controller is an interrupt-based multi-core RP2040 application.
Software execution is divided in timed operations and asynchronous operations.
Once executed, software initializes board gpio and device drivers, then it reads runtime parameters from flash memory and it starts one timer foreach CPU on the pico to handle timed operations:

| CPU | Operations | Task frequency |
| --- | ---------- | -------------- |
| 0 | Read user input, apply an alpha/beta filter on those, perform builtin test | 100 Hz |
| 1 | Read raw IMU, filter data with a pt1 filter, estimate body attitude (*), perform a PID loop to control motors | 100 Hz |

(*) If BNO55 is used, attitude estimation is replaced with the absolute orientation given by the sensor. First roll/pitch measured are considered IMU offset to obtain a known zero position.


User input are PWM signals the FS-IA6 module outputs on its channels:

| Radio Channel | Meaning | Signal range (millis) | Command range |
| ------------- | ------- | --------------------- | ------------- |
Channel1 | Roll command | (1000, 2000) | (-5.0, 5.0)° |
Channel2 | Pitch command | (1000, 2000) | (-5.0, 5.0)° |
Channel4 | Throttle command | (1000, 2000) | enum |
Channel5 | Armed flag | (1000, 2000) | (false, true) |

Throttle command is an enumerative value based on radio channel 4 pulse width duration:
| Duration | Value |
| -------- | ----- |
| 1000ms -| 1332 ms | DESCEND |
| 1333ms -| 1665 ms | HOVERING |
| 1666ms -| 2000 ms | CLIMB |

DESCEND, HOVERING and CLIMB are values in the range [1000-2000] stored in flash.

When Channel5 signal pulse width is greater than 1500 ms the armed flag is true and motors starts spinning at a fixed speed which does not generate enough lift for quadcopter to take-off.
If motors are armed, Channel1, Channel2 signals are converted into desired setpoint of roll and pitch. The PID loop starts to control quadcopter flight.
Channel4 signal is translated in one of the three possible values (DESCEND, HOVERING or CLIMB).
After timers have been launched, software eventually waits for messages from USB serial interface or UART.

Asynchronous operations are executed on CPU0 by interrupt handlers:

#### Interrupts
| Source   | Interrupt | Type | Frequency | CPU | Action |
| -------- | --------- | ---- | --------- | --- | ------ |
| FS-IA6    | Channel1 rising edge | Asynchronous | N/A | 0 | Store current milliseconds |
| FS-IA6    | Channel1 falling edge | Asynchronous | N/A | 0 | Compute radio channel 1 pulse width to obtain the Roll set point |
| FS-IA6    | Channel2 rising edge | Asynchronous | N/A | 0 | Store current milliseconds |
| FS-IA6    | Channel2 falling edge | Asynchronous | N/A | 0 | Compute radio channel 2 pulse width to obtain the Pitch set point |
| FS-IA6    | Channel4 rising edge | Asynchronous | N/A | 0 | Store current milliseconds |
| FS-IA6    | Channel4 falling edge | Asynchronous | N/A | 0 | Compute radio channel 3 pulse width to obtain the Throttle command |
| FS-IA6    | Channel5 rising edge | Asynchronous | N/A | 0 | Store current milliseconds |
| FS-IA6    | Channel5 falling edge | Asynchronous | N/A | 0 | Compute radio channel 5 pulse width to decide if motors shall be armed or not |
| Pico Oscillator    | Timer0 | Synchronous | 100 Hz | 0 | Read user command, perform builtin test  |
| Pico Oscillator    | Timer1 | Synchronous | 100 Hz | 1 | Read IMU through I2C interface, estimates body attitude and perform PID loop to control motors |
| HC-05 | UART RX | Asynchronous | N/A | 0 | Read one byte from pico UART0 interface for maintenance purposes |

#### Maintenance Protocol
In order to facilitate debugging, a maintenance protocol is defined to communicate with the pico through USB or UART/Bluetooth serial interfaces.
Through the maintenance protocol it is possible to:
1) See current software-processed data
2) Independently control motors PWM
3) Independently communicate with IMU through I2C
4) Retrieve and update flash-stored control loop parameters
5) Override radio signals and flight the quadcopter

##### Serial parameters

| Interface| Baud Rate | Data Bits | Stop Bits | Parity | Flow Control |
| -------- | --------- | --------- | --------- | ------ | ------------ |
| USB      | Any       | 8         | 1         | No     | No           |
| UART     | 115200    | 8         | 1         | No     | No           |

If you are using the HC-05 UART-Bluetooth adapter module, your client will interface with the [Bluetooth Serial Port Profile](https://www.bluetooth.com/specifications/specs/serial-port-profile-1-1/) of the HC-05 module. The HC-05 module will do the bridge and it interfaces with pico UART.
To change HC-05 serial parameters you can use Arduino IDE to upload the [AT interface sketch](https://forum.arduino.cc/t/how-do-you-change-the-baud-rate-of-hc-05/993572/4) to the pico, power on the module in AT mode by wiring KEY pin to VCC, open the serial monitor and send with CR+LF:

    AT+UART=115200,0,0

[Here](https://s3-sa-east-1.amazonaws.com/robocore-lojavirtual/709/HC-05_ATCommandSet.pdf) it is a list of supported AT commands.

##### Software protocol

Maintenance protocol is a client/server protocol in which pico is the server.

Server expects the following packet:
| Byte\Bit                | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
| ----------------------- | - | - | - | - | - | - | - | - |
| 0 (0xFF) - Synchronism  | 1 | 1 | 1 | 1 | 1 | 1 | 1 | 1 | 
| 1 Header byte 0         | raw_accel_x | raw_accel_y | raw_accel_z | raw_gyro_x | raw_gyro_y | raw_gyro_z | raw_magn_x | raw_magn_y |
| 2 Header byte 1         | raw_magn_z | filtered_accel_x | filtered_accel_y | filtered_accel_z | filtered_gyro_x | filtered_gyro_y | filtered_gyro_z | filtered_magn_x |
| 3 Header byte 2         | filtered_magn_y | filtered_magn_z | throttle_signal | roll_signal | pitch_signal | throttle_set_point | roll_set_point | pitch_set_point |
| 4 Header byte 3         | body_roll | body_pitch | body_yaw | roll_PID_error | roll_PID_p | roll_PID_i | roll_PID_d | roll_PID_u |
| 5 Header byte 4         | pitch_PID_error | pitch_PID_p | pitch_PID_i | pitch_PID_d | pitch_PID_u | yaw_PID_error | yaw_PID_p | yaw_PID_i |
| 6 Header byte 5         | yaw_PID_d | yaw_PID_u | motor1_signal | motor2_signal | motor3_signal | motor4_signal | motors_armed | builtin_test_status |
| 7 Header byte 6         | motor_params | js_params | PID_params | ptf1_params | imu_type | i2c_read | sw_ver | imu_offset |
| 8 Header byte 7         | throttle_params | cmd_id_bit_6 | cmd_id_bit_5 | cmd_id_bit_4 | cmd_id_bit_3 | cmd_id_bit_2 | cmd_id_bit_1 | cmd_id_bit_0 |
| 9..N Payload            | payload_data_n_bit_7 | payload_data_n_bit_6 | payload_data_n_bit_5 | payload_data_n_bit_4 | payload_data_n_bit_3 | payload_data_n_bit_2 | payload_data_n_bit_1      | payload_data_n_bit_0 |
| N+1 Checksum            | cks_bit_7 | cks_bit_6 | cks_bit_5 | cks_bit_4 | cks_bit_3 | cks_bit_2 | cks_bit_1 | cks_bit_0 |

Checksum algorithm:

 ```c++
uint8_t checksum(uint8_t* buf, uint32_t size)
{
    // Buf shall not contain synchronism byte
    uint8_t cks = 0;
    for (uint32_t i = 0; i < size; i++)
    {
        cks ^= buf[i];
    }
    return cks;
}
```
The minimal payload servers expects is composed by the REMOTE_CONTROL_TAG data structure + checksum.
REMOTE_CONTROL_TAG is a 9-bytes length data structure containing informations that the server will use to control the quadcopter flight from maintenance protocol:

| Field | Type | Values | Meaning | Size |
| ----- | ---- | ------ | ------- | ---- |
Override radio | uint8_t | 0/1 | Enable quadcpoter flight from maintenance by overriding radio PWM signals | 1 |
Armed signal | uint16_t | (1000, 2000) | Override armed radio signal | 2 |
Roll  signal | uint16_t | (1000, 2000) | Override armed radio signal | 2 |
Pitch signal | uint16_t | (1000, 2000) | Override armed radio signal | 2 |
Throttle signal | uint16_t | (1000, 2000) | Override armed radio signal | 2 |

REMOTE_CONTROL_TAG is always placed at payload end, immediatly before checksum.

Values of the header fields determine the expected payload the client will send:

| Header field             | Value | Meaning | Expected payload (type) | Payload size (bytes) |
| ------------------------ | ----- | ------- | ----------------------- | ------------ |
| All except cmd_id_bit_n  | 0     | Corresponding data is not requested to the server | REMOTE_CONTROL_TAG + checksum (uint8_t) | 10 |
| All except cmd_id_bit_n  | 1     | Corresponding data is requested to the server | REMOTE_CONTROL_TAG + checksum (uint8_t) | 10 |
| set_cmd_id               | 0     | No command requested | REMOTE_CONTROL_TAG + checksum (uint8_t) | 10 |
| set_cmd_id               | 1     | Set motor 1 speed | Pulse width to motor1 in microseconds (uint16_t) + REMOTE_CONTROL_TAG + checksum (uint8_t) | 12 |
| set_cmd_id               | 2     | Set motor 2 speed | Pulse width to motor2 in microseconds (uint16_t) + REMOTE_CONTROL_TAG + checksum (uint8_t) | 12 |
| set_cmd_id               | 3     | Set motor 3 speed | Pulse width to motor3 in microseconds (uint16_t) + REMOTE_CONTROL_TAG + checksum (uint8_t) | 12 |
| set_cmd_id               | 4     | Set motor 4 speed | Pulse width to motor4 in microseconds (uint16_t) + REMOTE_CONTROL_TAG + checksum (uint8_t) | 12 |
| set_cmd_id               | 5     | Set all motors speed | Pulse width to all motors in microseconds (uint16_t) + REMOTE_CONTROL_TAG +  checksum (uint8_t) | 12 |
| set_cmd_id               | 6     | Control motors | Maintenance motor control flag (uint8_t) + REMOTE_CONTROL_TAG + checksum (uint8_t) | 11 |
| set_cmd_id               | 7     | Flash write parameters | REMOTE_CONTROL_TAG + checksum (uint8_t) | 10 |
| set_cmd_id               | 8     | Update motor 1 parameters | Enabled/Disabled flag (uint8_t) + min pulse width (uint16_t) + max pulse width(uint16_t) + REMOTE_CONTROL_TAG + checksum (uint8_t) | 15 |
| set_cmd_id               | 9     | Update motor 2 parameters | Enabled/Disabled flag (uint8_t) + min pulse width (uint16_t) + max pulse width(uint16_t) + REMOTE_CONTROL_TAG + checksum (uint8_t) | 15 |
| set_cmd_id               | 10    | Update motor 3 parameters | Enabled/Disabled flag (uint8_t) + min pulse width (uint16_t) + max pulse width(uint16_t) + REMOTE_CONTROL_TAG + checksum (uint8_t) | 15 |
| set_cmd_id               | 11    | Update motor 4 parameters | Enabled/Disabled flag (uint8_t) + min pulse width (uint16_t) + max pulse width(uint16_t) + REMOTE_CONTROL_TAG + checksum (uint8_t) | 15 |
| set_cmd_id               | 12    | Update joystick throttle alpha/beta filter parameters | alpha (float) + beta (float) + REMOTE_CONTROL_TAG + checksum (uint8_t) | 18 |
| set_cmd_id               | 13    | Update joystick roll alpha/beta filter parameters | alpha (float) + beta (float) + REMOTE_CONTROL_TAG + checksum (uint8_t) | 18 |
| set_cmd_id               | 14    | Update joystick pitch alpha/beta filter parameters | alpha (float) + beta (float) + REMOTE_CONTROL_TAG + checksum (uint8_t) | 18 |
| set_cmd_id               | 15    | Update roll PID parameters | KP (float) + KI (float) + SAT (float) + AD (float) + BD (float) + REMOTE_CONTROL_TAG + checksum (uint8_t) | 34 | 
| set_cmd_id               | 16    | Update pitch PID parameters | KP (float) + KI (float) + SAT (float) + AD (float) + BD (float) + REMOTE_CONTROL_TAG + checksum (uint8_t) | 34 | 
| set_cmd_id               | 17    | Update yaw PID parameters | KP (float) + KI (float) + SAT (float) + AD (float) + BD (float) + REMOTE_CONTROL_TAG + checksum (uint8_t) | 34 | 
| set_cmd_id               | 18    | Update pt1 filter parameters for accelerometer | pt1f time constant x axis (float) + pt1f time constant y axis (float) + pt1f time constant z axis (float) + REMOTE_CONTROL_TAG + checksum (uint8_t) | 22 |
| set_cmd_id               | 19    | Update pt1 filter parameters for gyroscope | pt1f time constant x axis (float) + pt1f time constant y axis (float) + pt1f time constant z axis (float) + REMOTE_CONTROL_TAG + checksum (uint8_t) | 22 |
| set_cmd_id               | 20    | Update pt1 filter parameters for magnetometer | pt1f time constant x axis (float) + pt1f time constant y axis (float) + pt1f time constant z axis (float) + REMOTE_CONTROL_TAG + checksum (uint8_t) | 22 |
| set_cmd_id               | 21    | Update parameter IMU type | imu_type (enum : uint8_t) + REMOTE_CONTROL_TAG + checksum (uint8_t) | 11 |
| set_cmd_id               | 22    | I2C read | i2c channel (uint8_t) + i2c address (uint8_t) + i2c register (uint8_t) + REMOTE_CONTROL_TAG + checksum (uint8_t) | 13 |
| set_cmd_id               | 23    | I2C write | i2c channel (uint8_t) + i2c address (uint8_t) + i2c register (uint8_t) + value (uint8_t) + REMOTE_CONTROL_TAG + checksum (uint8_t) | 14 |
| set_cmd_id               | 24    | Reset IMU offset | REMOTE_CONTROL_TAG + checksum (uint8_t) | 10 |
| set_cmd_id               | 25    | Set throttle params | Descend value (uint16_t) + Hovering value (uint16_t) + Climb value (uint16_t) + REMOTE_CONTROL_TAG + checksum (uint8_t) | 16 |

IMU Type:

| Enum value | Integer value |
| ---------- | ------------- |
| LSM9DS1    | 0             |
| MPU6050    | 1             |
| BNO055     | 2             |


Server will answer to the client with the same packet structure.
Values of the header fields determine the expected payload the server will send. Payload values are packaged in the same order they appear in header fields.
A certain value is present in payload if the corresponding header field value is 1.

| Header field             | Meaning | Data type | Size (bytes) |
|--------------------------| ------- | --------- | ------------ |
raw_accel_x | Accelerometer x axis | float | 4 |
raw_accel_y | Accelerometer y axis | float | 4 |
raw_accel_z | Accelerometer z axis | float | 4 |
raw_gyro_x  | Gyroscope x axis | float | 4 |
raw_gyro_y  | Gyroscope y axis | float | 4 |
raw_gyro_z  | Gyroscope z axis | float | 4 |
raw_magn_x  | Magnetometer x axis | float | 4 |
raw_magn_y  | Magnetometer y axis | float | 4 |
raw_magn_z  | Magnetometer z axis | float | 4 |
filtered_accel_x | pt1 filtered accelerometer x axis | float | 4 |
filtered_accel_y | pt1 filtered accelerometer y axis | float | 4 |
filtered_accel_z | pt1 filtered accelerometer z axis | float | 4 |
filtered_gyro_x  | pt1 filtered gyroscope x axis | float | 4 |
filtered_gyro_y  | pt1 filtered gyroscope y axis | float | 4 |
filtered_gyro_z  | pt1 filtered gyroscope z axis | float | 4 |
filtered_magn_x  | pt1 filtered magnetometer x axis | float | 4 |
filtered_magn_y | pt1 filtered magnetometer y axis | float | 4 |
filtered_magn_z | pt1 filtered magnetometer z axis | float | 4 |
throttle_signal | radio channel 3 (throttle) pulse width | uint16_t | 2 |
roll_signal | radio channel 1 (roll) pulse width | uint16_t | 2 |
pitch_signal | radio channel 2 (pitch) pulse width | uint16_t | 2 |
throttle_set_point | desired motors signal pulse width | uint16_t | 2 |
roll_set_point | roll angle set point | float | 4 |
pitch_set_point | pitch angle set point | float | 4 |
body_roll | quadcopter roll angle | float | 4 |
body_pitch | quadcopter pitch angle | float | 4 |
body_yaw | quadcopter yaw angle | float | 4 |
roll_PID_error | difference between roll set point and estimated roll | float | 4
roll_PID_p | roll PID proportional term | float | 4 |
roll_PID_i | roll PID integral term | float | 4 |
roll_PID_d | roll PID derivative term | float | 4 |
roll_PID_u | roll PID output | float | 4 |
pitch_PID_error | difference between pitch set point and estimated pitch | float | 4
pitch_PID_p | pitch PID proportional term | float | 4 |
pitch_PID_i | pitch PID integral term | float | 4 |
pitch_PID_d | pitch PID derivative term | float | 4 |
pitch_PID_u | pitch PID output | float | 4 |
yaw_PID_error | difference between yaw set point (always 0) and estimated yaw | float | 4
yaw_PID_p | yaw PID proportional term | float | 4 |
yaw_PID_i | yaw PID integral term | float | 4 |
yaw_PID_d | yaw PID derivative term | float | 4 |
yaw_PID_u | yaw PID output | float | 4 |
motor1_signal | current motor 1 pulse width | uint16_t | 2 |
motor2_signal | current motor 2 pulse width | uint16_t | 2 |
motor3_signal | current motor 3 pulse width | uint16_t | 2 |
motor4_signal | current motor 4 pulse width | uint16_t | 2 |
motors_armed | motors armed flag | uint8_t | 1 |
builtin_test_status | continous builtin test status | enum : uint32_t | 4 |
motor_params | (Foreach motor) Enabled/Disabled flag + min pulse width + max pulse width | uint32_t[4][3] | 48 |
js_params | (Foreach joystick channel but armed) joystick alpha + joystick beta | float[3][2] | 24 |
PID_params | (Foreach euler angle) KP KI KT SAT AD BD | float[3][6] | 72 |
pt1f_params | (Foreach IMU sensor) pt1 filter time constant x axis + pt1 filter time constant y axis + pt1 filter time constant z axis  | float[3][3] | 36 | 
imu_type | IMU type | enum : uint8_t | 1 |
i2c_read | value read after the i2c read command | uint8_t | 1 |
sw_ver | software version | struct : uint8_t[4] | 4 |
imu_offset | current IMU offset | Offset roll + Offset pitch | float[2] | 12 |
throttle_params | current descend, hovering and climb signal values | Descend + Hovering + Climb | uint16_t[3] | 6 |

Software version data structure:

| Field         | Type           |
| ------------- | -------------- |
| MAJOR Version | uint8_t        |
| MINOR Version | uint8_t        |
| STAGE Version | uint8_t        |
| Release Type  | enum : uint8_t |


Release Type:

| Enum value | uint8_t value |
| ---------- | ------------- |
| Beta       | 0             |
| Release    | 1             |

### Maintenance Client

Under extras/maintenance a cross-platform Qt application is provided as an example of a maintenance protocol client.

[SDL2](https://www.libsdl.org/) libraries are required to control flight with a joystick.

![Screenshot](/extras/joystick.png)

## Credits

The LSM9DS1 device driver is a porting of the [Arduino LSM9DS1 library](https://github.com/sparkfun/LSM9DS1_Breakout) by Jim Lindblom of [SparkFun Electronics](https://www.sparkfun.com/).

The BNO055 device driver is a porting of the [BNO055 device driver for Raspberry Pi](https://github.com/AngelPerezM/rpi-bno055-drv) developed by [Ángel Pérez Muñoz](https://github.com/AngelPerezM).

The [QJoysticks](https://github.com/alex-spataru/QJoysticks) library by [Alex Spataru](https://github.com/alex-spataru) is used in maintenance client to interface with the joystick.