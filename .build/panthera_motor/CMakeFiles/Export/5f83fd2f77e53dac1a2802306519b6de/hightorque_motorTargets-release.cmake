#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "hightorque_motor::hightorque_motor" for configuration "Release"
set_property(TARGET hightorque_motor::hightorque_motor APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(hightorque_motor::hightorque_motor PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libhightorque_motor.4.4.7.dylib"
  IMPORTED_SONAME_RELEASE "@rpath/libhightorque_motor.4.dylib"
  )

list(APPEND _cmake_import_check_targets hightorque_motor::hightorque_motor )
list(APPEND _cmake_import_check_files_for_hightorque_motor::hightorque_motor "${_IMPORT_PREFIX}/lib/libhightorque_motor.4.4.7.dylib" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
