cmake_minimum_required(VERSION 3.6)

project(pid-tuner)

set(Qt5_DIR C:\\Qt\\5.15.2\\msvc2019_64\\lib\\cmake\\Qt5)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

add_executable(pid-tuner
${CMAKE_SOURCE_DIR}/src/tunergui.ui

${CMAKE_SOURCE_DIR}/src/tunerwindow.h
${CMAKE_SOURCE_DIR}/src/tunerwindow.cpp

${CMAKE_SOURCE_DIR}/src/menuwidget.h
${CMAKE_SOURCE_DIR}/src/menuwidget.cpp

${CMAKE_SOURCE_DIR}/src/rollpitchdisplay.h
${CMAKE_SOURCE_DIR}/src/rollpitchdisplay.cpp

${CMAKE_SOURCE_DIR}/src/headingdisplay.h
${CMAKE_SOURCE_DIR}/src/headingdisplay.cpp

${CMAKE_SOURCE_DIR}/src/proto.h

${CMAKE_SOURCE_DIR}/src/joystick.h
${CMAKE_SOURCE_DIR}/src/joystick.cpp

${CMAKE_SOURCE_DIR}/src/comthread.h
${CMAKE_SOURCE_DIR}/src/comthread.cpp


${CMAKE_SOURCE_DIR}/src/main.cpp
)

find_package(Threads REQUIRED)
find_package(Qt5 COMPONENTS Core Gui Widgets SerialPort REQUIRED)
find_package(SDL2 REQUIRED)

target_include_directories(pid-tuner PUBLIC ${CMAKE_SOURCE_DIR}/src/ ${SDL2_INCLUDE_DIRS})
target_link_libraries(pid-tuner Qt5::Core Qt5::Gui Qt5::Widgets Qt5::SerialPort ${SDL2_LIBRARIES})
