cmake_minimum_required(VERSION 3.12)
project(hightorque_robot_python VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找Python
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(hightorque_robot REQUIRED)

# 查找pybind11 - 优先使用 Python 环境中的版本
execute_process(
    COMMAND ${Python3_EXECUTABLE} -c "import pybind11; print(pybind11.get_cmake_dir())"
    OUTPUT_VARIABLE PYBIND11_CMAKE_DIR
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE PYBIND11_FOUND
)

if(PYBIND11_FOUND EQUAL 0)
    message(STATUS "使用 Python 环境中的 pybind11: ${PYBIND11_CMAKE_DIR}")
    set(pybind11_DIR ${PYBIND11_CMAKE_DIR})
else()
    message(STATUS "使用系统的 pybind11")
endif()

find_package(pybind11 CONFIG REQUIRED)

# 添加C++项目的头文件路径
include_directories(
    ${HIGHTORQUE_ROBOT_DIR}/include
    ${HIGHTORQUE_ROBOT_DIR}/include/hardware
    ${HIGHTORQUE_ROBOT_DIR}/include/crc
    ${HIGHTORQUE_ROBOT_DIR}/msg/motor_msg
)

# 查找依赖库
find_package(PkgConfig REQUIRED)
pkg_check_modules(LCM REQUIRED lcm)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

# 创建pybind11模块
pybind11_add_module(_hightorque_robot src/bindings.cpp)

# 链接库
target_link_libraries(_hightorque_robot PRIVATE
    ${HIGHTORQUE_ROBOT_LIB}
    ${LCM_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    pthread
    rt
    hightorque_robot
)

# 设置输出目录
set_target_properties(_hightorque_robot PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/hightorque_robot
)

# 安装规则
install(TARGETS _hightorque_robot
    LIBRARY DESTINATION hightorque_robot
)
