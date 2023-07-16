cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(DroneController C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()
 
add_executable(DroneController
  src/drivers/lsm9ds1/LSM9DS1.h
  src/drivers/lsm9ds1/LSM9DS1_Registers.h
  src/drivers/lsm9ds1/LSM9DS1_Types.h
  src/drivers/servo/servo.h 
  src/user.h
  src/timer.h
  src/attitude.h

  src/drivers/lsm9ds1/LSM9DS1.cpp
  src/drivers/servo/servo.cpp
  src/user.cpp
  src/timer.cpp
  src/attitude.cpp
  src/main.cpp
)

target_include_directories(DroneController PUBLIC src/drivers/lsm9ds1 src/drivers/servo)

pico_enable_stdio_usb(DroneController 1)
pico_enable_stdio_uart(DroneController 1)

pico_add_extra_outputs(DroneController)
 
target_link_libraries(DroneController pico_stdlib hardware_i2c hardware_pwm) 