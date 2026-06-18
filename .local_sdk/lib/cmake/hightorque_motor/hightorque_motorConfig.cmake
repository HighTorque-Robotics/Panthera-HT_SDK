
####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was hightorque_robotConfig.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

macro(check_required_components _NAME)
  foreach(comp ${${_NAME}_FIND_COMPONENTS})
    if(NOT ${_NAME}_${comp}_FOUND)
      if(${_NAME}_FIND_REQUIRED_${comp})
        set(${_NAME}_FOUND FALSE)
      endif()
    endif()
  endforeach()
endmacro()

####################################################################################

# 包含CMakeFindDependencyMacro以使用find_dependency
include(CMakeFindDependencyMacro)

# 查找必需的依赖项
find_dependency(lcm REQUIRED)
find_dependency(serial_cmake REQUIRED)  
find_dependency(yaml-cpp REQUIRED)

# 查找libserialport库
find_library(SERIALPORT_LIBRARY serialport)
if(NOT SERIALPORT_LIBRARY)
    set(hightorque_robot_FOUND FALSE)
    if(NOT hightorque_robot_FIND_QUIETLY)
        message(FATAL_ERROR "libserialport not found. Please install with: sudo apt-get install libserialport-dev")
    endif()
    return()
endif()

# 包含导出的目标
include("${CMAKE_CURRENT_LIST_DIR}/hightorque_motorTargets.cmake")

# 设置配置文件路径变量（供用户使用）
get_filename_component(HIGHTORQUE_ROBOT_CMAKE_DIR "${CMAKE_CURRENT_LIST_DIR}" DIRECTORY)
get_filename_component(HIGHTORQUE_ROBOT_CMAKE_DIR "${HIGHTORQUE_ROBOT_CMAKE_DIR}" DIRECTORY)
get_filename_component(HIGHTORQUE_ROBOT_CMAKE_DIR "${HIGHTORQUE_ROBOT_CMAKE_DIR}" DIRECTORY)
set(HIGHTORQUE_ROBOT_CONFIG_DIR "${HIGHTORQUE_ROBOT_CMAKE_DIR}/share/hightorque_robot/robot_param")

check_required_components(hightorque_robot)
