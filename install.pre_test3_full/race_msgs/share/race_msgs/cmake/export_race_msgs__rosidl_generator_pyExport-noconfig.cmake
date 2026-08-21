#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "race_msgs::race_msgs__rosidl_generator_py" for configuration ""
set_property(TARGET race_msgs::race_msgs__rosidl_generator_py APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(race_msgs::race_msgs__rosidl_generator_py PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/librace_msgs__rosidl_generator_py.so"
  IMPORTED_SONAME_NOCONFIG "librace_msgs__rosidl_generator_py.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS race_msgs::race_msgs__rosidl_generator_py )
list(APPEND _IMPORT_CHECK_FILES_FOR_race_msgs::race_msgs__rosidl_generator_py "${_IMPORT_PREFIX}/lib/librace_msgs__rosidl_generator_py.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
