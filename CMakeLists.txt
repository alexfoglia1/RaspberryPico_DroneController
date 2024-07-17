cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(DroneController C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()
 
add_executable(DroneController
  src/drivers/i2c_utils/i2c_utils.h
  src/drivers/lsm9ds1/LSM9DS1.h
  src/drivers/lsm9ds1/LSM9DS1_Registers.h
  src/drivers/lsm9ds1/LSM9DS1_Types.h
  src/drivers/mpu6050/mpu6050.h
  src/drivers/bno055/bno055.h
  src/drivers/servo/servo.h 
  src/drivers/servo/pulse.h
  src/imu_interface.h
  src/mpu6050_interface.h
  src/lsm9ds1_interface.h
  src/bno055_interface.h
  src/user.h
  src/timer.h
  src/attitude.h
  src/pid.h
  src/joystick.h
  src/motors.h
  src/maint.h
  src/uart.h
  src/cbit.h

  src/drivers/i2c_utils/i2c_utils.cpp
  src/drivers/lsm9ds1/LSM9DS1.cpp
  src/drivers/mpu6050/mpu6050.cpp
  src/drivers/bno055/bno055.cpp
  src/drivers/servo/servo.cpp
  src/drivers/servo/pulse.cpp
  src/imu_interface.cpp
  src/lsm9ds1_interface.cpp
  src/mpu6050_interface.cpp
  src/bno055_interface.cpp
  src/user.cpp
  src/timer.cpp
  src/attitude.cpp
  src/pid.cpp
  src/joystick.cpp
  src/motors.cpp
  src/maint.cpp
  src/uart.cpp
  src/cbit.cpp
  src/main.cpp
)

message(${CMAKE_CURRENT_LIST_DIR})

target_include_directories(DroneController PUBLIC src/drivers/lsm9ds1 src/drivers/servo src/drivers/mpu6050 src/drivers/bno055 src/drivers/i2c_utils)

pico_enable_stdio_usb(DroneController 1)

pico_add_extra_outputs(DroneController)
 
target_link_libraries(DroneController pico_stdlib hardware_i2c hardware_pwm hardware_flash hardware_sync pico_multicore) 