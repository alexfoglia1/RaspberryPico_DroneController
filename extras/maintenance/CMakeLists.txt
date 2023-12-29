cmake_minimum_required(VERSION 3.6)

project(DroneMaintenance)

set(Qt5_DIR C:\\Qt\\5.15.2\\msvc2019_64\\lib\\cmake\\Qt5)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

add_executable(DroneMaintenance
${CMAKE_SOURCE_DIR}/src/MaintenanceGui.ui
${CMAKE_SOURCE_DIR}/src/AutoscanComPortsGui.ui

${CMAKE_SOURCE_DIR}/src/MaintenanceWindow.h
${CMAKE_SOURCE_DIR}/src/MaintenanceWindow.cpp

${CMAKE_SOURCE_DIR}/src/maint.h
${CMAKE_SOURCE_DIR}/src/maint.cpp

${CMAKE_SOURCE_DIR}/src/PlotWidget.h
${CMAKE_SOURCE_DIR}/src/PlotWidget.cpp
 
${CMAKE_SOURCE_DIR}/src/main.cpp
)

find_package(Qt5 COMPONENTS Core Gui Widgets SerialPort REQUIRED)

target_include_directories(DroneMaintenance PUBLIC ${CMAKE_SOURCE_DIR}/src/)

target_link_libraries(DroneMaintenance Qt5::Core Qt5::Gui Qt5::Widgets Qt5::SerialPort)
