cmake_minimum_required(VERSION 3.0.2)
project(hightorque_motor)

if(NOT CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR}/install CACHE PATH "Install path prefix" FORCE)
endif()

set(VERSION_MAJOR 4)
set(VERSION_MINOR 4)
set(VERSION_PATCH 7)
set(PROJECT_VERSION ${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH})

add_subdirectory(third_part/lcm)
add_subdirectory(third_part/serial_cmake)

# 查找依赖库
find_package(yaml-cpp REQUIRED)

find_library(SERIALPORT_LIBRARY serialport)
if(NOT SERIALPORT_LIBRARY)
  message(FATAL_ERROR "libserialport not found! Please install libserialport-dev.")
else()
  message(STATUS "Found libserialport library: ${SERIALPORT_LIBRARY}")
endif()

set(rt_LIBRARIES rt)
set(pthread_LIBRARIES pthread)

# ==================== hightorque_motor 库 ====================
# 电机底层驱动库

set(motor_SRCS
    src/serial_driver.cpp
    src/parse_robot_params.cpp
    src/hardware/motor.cpp
    src/hardware/canport.cpp
    src/hardware/canboard.cpp
    src/hardware/robot.cpp
    src/crc/crc8.cpp
    src/crc/crc16.cpp
)

add_library(hightorque_motor SHARED ${motor_SRCS})

target_include_directories(hightorque_motor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/crc>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/hardware>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/msg>
  $<INSTALL_INTERFACE:include>
  $<INSTALL_INTERFACE:include/crc>
  $<INSTALL_INTERFACE:include/hardware>
  $<INSTALL_INTERFACE:include/motor_msg>
)

target_link_libraries(hightorque_motor
    PUBLIC
        lcm
        yaml-cpp
        serial_cmake
        ${SERIALPORT_LIBRARY}
    PRIVATE
        rt
        pthread
)

# 设置库属性
set_target_properties(hightorque_motor PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${VERSION_MAJOR}
    PUBLIC_HEADER "include/hardware/robot.hpp;include/hardware/motor.hpp;include/hardware/canport.hpp;include/hardware/canboard.hpp;include/parse_robot_params.hpp;include/serial_struct.hpp;include/serial_driver.hpp;include/crc/crc8.hpp;include/crc/crc16.hpp;msg/motor_msg/motor_msg.hpp"
)

# ==================== 安装配置 ====================

install(TARGETS hightorque_motor
    EXPORT hightorque_motorTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    PUBLIC_HEADER DESTINATION include
)

install(DIRECTORY include/ DESTINATION include
    FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h")
install(DIRECTORY msg/motor_msg/ DESTINATION include/motor_msg
    FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h")
install(DIRECTORY robot_param/ DESTINATION share/hightorque_motor/robot_param
    FILES_MATCHING PATTERN "*.yaml")

install(EXPORT hightorque_motorTargets
    FILE hightorque_motorTargets.cmake
    NAMESPACE hightorque_motor::
    DESTINATION lib/cmake/hightorque_motor
)

# CMake 配置文件
include(CMakePackageConfigHelpers)
configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/hightorque_robotConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/hightorque_motorConfig.cmake
    INSTALL_DESTINATION lib/cmake/hightorque_motor
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/hightorque_motorConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/hightorque_motorConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/hightorque_motorConfigVersion.cmake
    DESTINATION lib/cmake/hightorque_motor
)

# ==================== 电机控制示例程序 ====================

add_executable(canboard_update example/canboard_update.cpp)
target_link_libraries(canboard_update hightorque_motor)

add_executable(motor_feedack example/motor_feedack.cpp)
target_link_libraries(motor_feedack hightorque_motor)

add_executable(motor_move_zero example/motor_move_zero.cpp)
target_link_libraries(motor_move_zero hightorque_motor)

add_executable(motor_run example/motor_run.cpp)
target_link_libraries(motor_run hightorque_motor)

add_executable(motor_msg_subscriber example/motor_msg_subscriber.cpp)
target_link_libraries(motor_msg_subscriber hightorque_motor)

add_executable(motor_set_zero example/motor_set_zero.cpp)
target_link_libraries(motor_set_zero hightorque_motor)

add_executable(parse_demo example/parse_demo.cpp)
target_link_libraries(parse_demo hightorque_motor)
