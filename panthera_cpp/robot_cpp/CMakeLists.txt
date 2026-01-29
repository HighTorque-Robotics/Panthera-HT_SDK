cmake_minimum_required(VERSION 3.5)
project(panthera_robot)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ==================== 查找依赖 ====================

# 查找 motor_cpp 库
# 如果 motor_cpp 已安装，使用 find_package
# 否则，直接添加子目录
set(MOTOR_CPP_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../motor_cpp")

if(EXISTS "${MOTOR_CPP_DIR}/CMakeLists.txt")
    message(STATUS "Using motor_cpp from: ${MOTOR_CPP_DIR}")
    add_subdirectory(${MOTOR_CPP_DIR} ${CMAKE_CURRENT_BINARY_DIR}/motor_cpp)
else()
    message(FATAL_ERROR "motor_cpp not found at ${MOTOR_CPP_DIR}")
endif()

# 查找其他依赖库
find_package(yaml-cpp REQUIRED)

# 查找 Pinocchio 库（用于重力补偿等动力学计算）- 必需依赖
# 添加 Pinocchio 的安装路径（robotpkg 安装在 /opt/openrobots）
set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH};/opt/openrobots/lib/cmake;/opt/openrobots")
find_package(pinocchio REQUIRED)

# ==================== panthera_robot 库 ====================
# 机械臂高级控制库

set(panthera_SRCS
    src/panthera/Panthera.cpp
)

add_library(panthera_robot SHARED ${panthera_SRCS})

# 头文件路径
target_include_directories(panthera_robot PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${MOTOR_CPP_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# 链接库
target_link_libraries(panthera_robot
    PUBLIC
        hightorque_motor
        yaml-cpp
        pinocchio::pinocchio
)

message(STATUS "Pinocchio found - gravity compensation enabled")

# 设置库属性
set_target_properties(panthera_robot PROPERTIES
    VERSION 1.0.0
    SOVERSION 1
    PUBLIC_HEADER "include/panthera/Panthera.hpp;include/panthera/SignalHandler.hpp"
)

# ==================== 安装配置 ====================

install(TARGETS panthera_robot
    EXPORT panthera_robotTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    PUBLIC_HEADER DESTINATION include
)

install(DIRECTORY include/ DESTINATION include
    FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h")

install(EXPORT panthera_robotTargets
    FILE panthera_robotTargets.cmake
    NAMESPACE panthera_robot::
    DESTINATION lib/cmake/panthera_robot
)

# CMake 配置文件
include(CMakePackageConfigHelpers)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/panthera_robotConfigVersion.cmake
    VERSION 1.0.0
    COMPATIBILITY AnyNewerVersion
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/panthera_robotConfigVersion.cmake
    DESTINATION lib/cmake/panthera_robot
)

# 安装配置文件和URDF
install(DIRECTORY robot_param/ DESTINATION share/panthera_robot/robot_param
    FILES_MATCHING PATTERN "*.yaml")
install(DIRECTORY urdf/ DESTINATION share/panthera_robot/urdf
    FILES_MATCHING PATTERN "*.urdf")

# ==================== 机械臂控制示例程序 ====================

# 定义所有示例程序
set(EXAMPLES
    0_robot_get_state
    0_robot_set_zero
    1_PosVel_control
    1_Vel_control
    1_Joints_Sync_Arrival_control
    1_forward_kinematics_test
    1_inverse_kinematics_test
    1_PD_control
    2_gravity_compensation_control
    2_gravity_friction_compensation_control
    2_inv_PosVel_control
    2_Jointimpendence_control_with_gra_pd
    2_Jointimpendence_control_with_gra_fri_pd
    3_gravity_compensation_with_fk
    3_interpolation_control_zeroVel
    3_interpolation_control_nozeroVel
    3_sin_trajectory_control
    4_impedance_trajectory_control_with_gra_pd
    5_record_trajectory
    5_replay_trajectory
    5_teleop_control
)

# 为每个示例创建可执行文件
foreach(EXAMPLE ${EXAMPLES})
    add_executable(${EXAMPLE} example/${EXAMPLE}.cpp)

    # 链接库
    target_link_libraries(${EXAMPLE}
        panthera_robot
    )
endforeach()

# 打印信息
message(STATUS "Building Panthera Robot SDK")
message(STATUS "Motor SDK path: ${MOTOR_CPP_DIR}")
message(STATUS "Pinocchio found - building all examples with gravity compensation")
message(STATUS "Total examples: ${EXAMPLES}")
