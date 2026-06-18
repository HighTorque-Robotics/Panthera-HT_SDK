# Install script for directory: /Users/ky/code/Panthera-HT_SDK/panthera_cpp/motor_cpp/third_part/lcm/lcm

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/Users/ky/code/Panthera-HT_SDK/.local_sdk")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set path to fallback-tool for dependency-resolution.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/Users/ky/code/Panthera-HT_SDK/.build/panthera_motor/third_part/lcm/lcm/liblcm.a")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/liblcm.a" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/liblcm.a")
    execute_process(COMMAND "/usr/bin/ranlib" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/liblcm.a")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/lcm" TYPE FILE FILES
    "/Users/ky/code/Panthera-HT_SDK/panthera_cpp/motor_cpp/third_part/lcm/lcm/eventlog.h"
    "/Users/ky/code/Panthera-HT_SDK/panthera_cpp/motor_cpp/third_part/lcm/lcm/lcm.h"
    "/Users/ky/code/Panthera-HT_SDK/panthera_cpp/motor_cpp/third_part/lcm/lcm/lcm_coretypes.h"
    "/Users/ky/code/Panthera-HT_SDK/panthera_cpp/motor_cpp/third_part/lcm/lcm/lcm_version.h"
    "/Users/ky/code/Panthera-HT_SDK/panthera_cpp/motor_cpp/third_part/lcm/lcm/lcm-cpp.hpp"
    "/Users/ky/code/Panthera-HT_SDK/panthera_cpp/motor_cpp/third_part/lcm/lcm/lcm-cpp-impl.hpp"
    "/Users/ky/code/Panthera-HT_SDK/.build/panthera_motor/third_part/lcm/lcm/lcm_c_namespace.h"
    "/Users/ky/code/Panthera-HT_SDK/.build/panthera_motor/third_part/lcm/lcm/lcm_export.h"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES
    "/Users/ky/code/Panthera-HT_SDK/.build/panthera_motor/third_part/lcm/lcm/liblcm.1.5.1.dylib"
    "/Users/ky/code/Panthera-HT_SDK/.build/panthera_motor/third_part/lcm/lcm/liblcm.1.dylib"
    )
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/liblcm.1.5.1.dylib"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/liblcm.1.dylib"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      execute_process(COMMAND "/usr/bin/install_name_tool"
        -id "liblcm.1.dylib"
        "${file}")
      if(CMAKE_INSTALL_DO_STRIP)
        execute_process(COMMAND "/usr/bin/strip" -x "${file}")
      endif()
    endif()
  endforeach()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/Users/ky/code/Panthera-HT_SDK/.build/panthera_motor/third_part/lcm/lcm/liblcm.dylib")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/lcm" TYPE FILE FILES
    "/Users/ky/code/Panthera-HT_SDK/panthera_cpp/motor_cpp/third_part/lcm/lcm/eventlog.h"
    "/Users/ky/code/Panthera-HT_SDK/panthera_cpp/motor_cpp/third_part/lcm/lcm/lcm.h"
    "/Users/ky/code/Panthera-HT_SDK/panthera_cpp/motor_cpp/third_part/lcm/lcm/lcm_coretypes.h"
    "/Users/ky/code/Panthera-HT_SDK/panthera_cpp/motor_cpp/third_part/lcm/lcm/lcm_version.h"
    "/Users/ky/code/Panthera-HT_SDK/panthera_cpp/motor_cpp/third_part/lcm/lcm/lcm-cpp.hpp"
    "/Users/ky/code/Panthera-HT_SDK/panthera_cpp/motor_cpp/third_part/lcm/lcm/lcm-cpp-impl.hpp"
    "/Users/ky/code/Panthera-HT_SDK/.build/panthera_motor/third_part/lcm/lcm/lcm_c_namespace.h"
    "/Users/ky/code/Panthera-HT_SDK/.build/panthera_motor/third_part/lcm/lcm/lcm_export.h"
    )
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
if(CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "/Users/ky/code/Panthera-HT_SDK/.build/panthera_motor/third_part/lcm/lcm/install_local_manifest.txt"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()
