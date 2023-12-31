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
Once executed, software initializes board gpio and device drivers, then it reads runtime parameters from flash memory and it starts one timer foreach CPU on the pico:

1) Attitude Producer: Reads raw IMU, filters data with a pt1 filter and estimates body attitude, on CPU1. If BNO55 is used, attitude estimation is replaced with the absolute orientation given by the sensor.
2) Attitude Consumer: Reads body attitude and user commands. User commands are alpha/beta filtered and the output is used to perform a PID control loop, on CPU0. PID output is the PWM pulse width actuated to the motors.

After timers have been launched, software eventually waits for flash updates.

#### Interrupts
| Source   | Interrupt | Type | Frequency | CPU | Action |
| -------- | --------- | ---- | --------- | --- | ------ |
| FS-IA6    | Channel1 rising edge | Asynchronous | N/A | 0 | Store current milliseconds |
| FS-IA6    | Channel1 falling edge | Asynchronous | N/A | 0 | Compute radio channel 1 pulse width to obtain the Roll set point |
| FS-IA6    | Channel2 rising edge | Asynchronous | N/A | 0 | Store current milliseconds |
| FS-IA6    | Channel2 falling edge | Asynchronous | N/A | 0 | Compute radio channel 2 pulse width to obtain the Pitch set point |
| FS-IA6    | Channel3 rising edge | Asynchronous | N/A | 0 | Store current milliseconds |
| FS-IA6    | Channel3 falling edge | Asynchronous | N/A | 0 | Compute radio channel 3 pulse width to obtain the Throttle set point |
| FS-IA6    | Channel5 rising edge | Asynchronous | N/A | 0 | Store current milliseconds |
| FS-IA6    | Channel5 falling edge | Asynchronous | N/A | 0 | Compute radio channel 5 pulse width to decide if motors shall be armed or not |
| Pico Oscillator    | Timer0 | Synchronous | 500 Hz | 0 | Read attitude and user commands, perform PID control loop and actuates PID output to the motors |
| Pico Oscillator    | Timer1 | Synchronous | 500 Hz | 1 | Read IMU through I2C interface and estimates body attitude |

#### Maintenance Protocol
In order to facilitate debugging, a maintenance protocol is defined to communicate with the pico through USB or UART/Bluetooth serial interfaces.
Through the maintenance protocol it is possible to:
1) See current software-processed data
2) Independently control motors PWM
3) Independently communicate with IMU through I2C
2) Retrieve and update flash-stored control loop parameters

##### Serial parameters

| Interface| Baud Rate | Data Bits | Stop Bits | Parity | Flow Control |
| -------- | --------- | --------- | --------- | ------ | ------------ |
| USB      | Any       | 8         | 1         | No     | No           |
| UART     | 115200 (Pico Default)   | 8         | 1         | No     | No           |

UART or USB interface is managed by pico bsp, software initializes stdio from both UART and USB.

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
| 4 Header byte 3         | body_roll | body_pitch | body_yaw | roll_pid_error | roll_pid_p | roll_pid_i | roll_pid_d | roll_pid_u |
| 5 Header byte 4         | pitch_pid_error | pitch_pid_p | pitch_pid_i | pitch_pid_d | pitch_pid_u | yaw_pid_error | yaw_pid_p | yaw_pid_i |
| 6 Header byte 5         | yaw_pid_d | yaw_pid_u | motor1_signal | motor2_signal | motor3_signal | motor4_signal | motors_armed | builtin_test_status |
| 7 Header byte 6         | motor_params | js_params | pid_params | ptf1_params | imu_type | i2c_read | sw_ver | cmd_id_bit_8 |
| 8 Header byte 7         | cmd_id_bit_7 | cmd_id_bit_6 | cmd_id_bit_5 | cmd_id_bit_4 | cmd_id_bit_3 | cmd_id_bit_2 | cmd_id_bit_1 | cmd_id_bit_0 |
| 9..N Payload            | payload_data_n_bit_7 | payload_data_n_bit_6 | payload_data_n_bit_5 | payload_data_n_bit_4 | payload_data_n_bit_3 | payload_data_n_bit_2 | payload_data_n_bit_1 | payload_data_n_bit_0 |
| N+1 Checksum            | cks_bit_7 | cks_bit_6 | cks_bit_5 | cks_bit_4 | cks_bit_3 | cks_bit_2 | cks_bit_1 | cks_bit_0 |

Checksum algorithm:

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


Values of the header fields determine the expected payload the client will send:

| Header field             | Value | Meaning | Expected payload (type) | Payload size (bytes) |
| ------------------------ | ----- | ------- | ----------------------- | ------------ |
| All except cmd_id_bit_n  | 0     | Corresponding data is not requested to the server | checksum (uint8_t) | 1 |
| All except cmd_id_bit_n  | 1     | Corresponding data is requested to the server | checksum (uint8_t) | 1 |
| set_cmd_id               | 0     | No command requested | checksum (uint8_t) | 1 |
| set_cmd_id               | 1     | Set motor 1 speed | Pulse width to motor1 in microseconds (uint32_t) + checksum (uint8_t) | 5 |
| set_cmd_id               | 2     | Set motor 2 speed | Pulse width to motor2 in microseconds (uint32_t) + checksum (uint8_t) | 5 |
| set_cmd_id               | 3     | Set motor 3 speed | Pulse width to motor3 in microseconds (uint32_t) + checksum (uint8_t) | 5 |
| set_cmd_id               | 4     | Set motor 4 speed | Pulse width to motor4 in microseconds (uint32_t) + checksum (uint8_t) | 5 |
| set_cmd_id               | 5     | Set all motors speed | Pulse width to all motors in microseconds (uint32_t) + checksum (uint8_t) | 5 |
| set_cmd_id               | 6     | Control motors | Maintenance motor control flag (uint32_t) + checksum (uint8_t) | 5 |
| set_cmd_id               | 7     | Flash write parameters | checksum (uint8_t) | 1 |
| set_cmd_id               | 8     | Update motor 1 parameters | Enabled/Disabled flag (uint32_t) + min pulse width (uint32_t) + max pulse width(uint32_t) + checksum (uint8_t) | 13 |
| set_cmd_id               | 9     | Update motor 2 parameters | Enabled/Disabled flag (uint32_t) + min pulse width (uint32_t) + max pulse width(uint32_t) + checksum (uint8_t) | 13 |
| set_cmd_id               | 10    | Update motor 3 parameters | Enabled/Disabled flag (uint32_t) + min pulse width (uint32_t) + max pulse width(uint32_t) + checksum (uint8_t) | 13 |
| set_cmd_id               | 11    | Update motor 4 parameters | Enabled/Disabled flag (uint32_t) + min pulse width (uint32_t) + max pulse width(uint32_t) + checksum (uint8_t) | 13 |
| set_cmd_id               | 12    | Update joystick throttle alpha/beta filter parameters | alpha (float) + beta (float) + checksum (uint8_t) | 9 |
| set_cmd_id               | 13    | Update joystick roll alpha/beta filter parameters | alpha (float) + beta (float) + checksum (uint8_t) | 9 |
| set_cmd_id               | 14    | Update joystick pitch alpha/beta filter parameters | alpha (float) + beta (float) + checksum (uint8_t) | 9 |
| set_cmd_id               | 15    | Update roll PID parameters | KP (float) + KI (float) + SAT (float) + AD (float) + BD (float) + checksum (uint8_t) | 25 | 
| set_cmd_id               | 16    | Update pitch PID parameters | KP (float) + KI (float) + SAT (float) + AD (float) + BD (float) + checksum (uint8_t) | 25 | 
| set_cmd_id               | 17    | Update yaw PID parameters | KP (float) + KI (float) + SAT (float) + AD (float) + BD (float) + checksum (uint8_t) | 25 | 
| set_cmd_id               | 18    | Update pt1 filter parameters for accelerometer | pt1f time constant x axis (float) + pt1f time constant y axis (float) + pt1f time constant z axis (float) + checksum (uint8_t) | 13 |
| set_cmd_id               | 19    | Update pt1 filter parameters for gyroscope | pt1f time constant x axis (float) + pt1f time constant y axis (float) + pt1f time constant z axis (float) + checksum (uint8_t) | 13 |
| set_cmd_id               | 20    | Update pt1 filter parameters for magnetometer | pt1f time constant x axis (float) + pt1f time constant y axis (float) + pt1f time constant z axis (float) + checksum (uint8_t) | 13 |
| set_cmd_id               | 21    | Update parameter IMU type | imu_type (enum : uint32_t) + checksum (uint8_t) | 5 |
| set_cmd_id               | 22    | I2C read | i2c channel (uint32_t) + i2c address (uint32_t) + i2c register (uint32_t) + checksum (uint8_t) | 13 |
| set_cmd_id               | 23    | I2C write | i2c channel (uint32_t) + i2c address (uint32_t) + i2c register (uint32_t) + value (uint32_t) + checksum (uint8_t) | 17 |

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
throttle_signal | radio channel 3 (throttle) pulse width | uint32_t | 4 |
roll_signal | radio channel 1 (roll) pulse width | uint32_t | 4 |
pitch_signal | radio channel 2 (pitch) pulse width | uint32_t | 4 |
throttle_set_point | desired motors signal pulse width | uint32_t | 4 |
roll_set_point | roll angle set point | float | 4 |
pitch_set_point | pitch angle set point | float | 4 |
body_roll | quadcopter roll angle | float | 4 |
body_pitch | quadcopter pitch angle | float | 4 |
body_yaw | quadcopter yaw angle | float | 4 |
roll_pid_error | difference between roll set point and estimated roll | float | 4
roll_pid_p | roll pid proportional term | float | 4 |
roll_pid_i | roll pid integral term | float | 4 |
roll_pid_d | roll pid derivative term | float | 4 |
roll_pid_u | roll pid output | float | 4 |
pitch_pid_error | difference between pitch set point and estimated pitch | float | 4
pitch_pid_p | pitch pid proportional term | float | 4 |
pitch_pid_i | pitch pid integral term | float | 4 |
pitch_pid_d | pitch pid derivative term | float | 4 |
pitch_pid_u | pitch pid output | float | 4 |
yaw_pid_error | difference between yaw set point (always 0) and estimated yaw | float | 4
yaw_pid_p | yaw pid proportional term | float | 4 |
yaw_pid_i | yaw pid integral term | float | 4 |
yaw_pid_d | yaw pid derivative term | float | 4 |
yaw_pid_u | yaw pid output | float | 4 |
motor1_signal | current motor 1 pulse width | uint32_t | 4 |
motor2_signal | current motor 2 pulse width | uint32_t | 4 |
motor3_signal | current motor 3 pulse width | uint32_t | 4 |
motor4_signal | current motor 4 pulse width | uint32_t | 4 |
motors_armed | motors armed flag | uint32_t | 4 |
builtin_test_status | continous builtin test status | enum : uint32_t | 4 |
motor_params | (Foreach motor) Enabled/Disabled flag + min pulse width + max pulse width | uint32_t[4][3] | 48 |
js_params | (Foreach joystick channel but armed) joystick alpha + joystick beta | float[3][2] | 24 |
pid_params | (Foreach euler angle) KP KI KT SAT AD BD | float[3][6] | 72 |
pt1f_params | (Foreach IMU sensor) pt1 filter time constant x axis + pt1 filter time constant y axis + pt1 filter time constant z axis  | float[3][3] | 36 | 
imu_type | IMU type | enum : uint32_t | 4 |
i2c_read | value read after the i2c read command | uint32_t | 4 |
sw_ver | software version | struct : uint8_t[4] | 4 |


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


## Credits

The LSM9DS1 device driver is a porting of the [Arduino LSM9DS1 library](https://github.com/sparkfun/LSM9DS1_Breakout) by Jim Lindblom of [SparkFun Electronics](https://www.sparkfun.com/).

The BNO055 device driver is a porting of the [BNO055 device driver for Raspberry Pi](https://github.com/AngelPerezM/rpi-bno055-drv) developed by [Ángel Pérez Muñoz](https://github.com/AngelPerezM).