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

# 创建pybind11模块
pybind11_add_module(_hightorque_robot src/bindings.cpp)

# hightorque_robot::hightorque_robot 是 motor_cpp 导出的 CMake 目标，
# 同时携带 SDK 的头文件路径和传递依赖。
target_link_libraries(_hightorque_robot PRIVATE
    hightorque_robot::hightorque_robot
)

# 设置输出目录
set_target_properties(_hightorque_robot PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/hightorque_robot
)

# Keep a Conda-local C++ SDK discoverable at runtime when the package is
# built with CMAKE_PREFIX_PATH pointing at the active environment.
if(CMAKE_BUILD_RPATH)
    set_property(TARGET _hightorque_robot APPEND PROPERTY BUILD_RPATH "${CMAKE_BUILD_RPATH}")
endif()
if(CMAKE_INSTALL_RPATH)
    set_property(TARGET _hightorque_robot APPEND PROPERTY INSTALL_RPATH "${CMAKE_INSTALL_RPATH}")
endif()

# 安装规则
install(TARGETS _hightorque_robot
    LIBRARY DESTINATION hightorque_robot
)
