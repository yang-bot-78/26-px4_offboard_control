# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_fr_lio_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED fr_lio_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(fr_lio_FOUND FALSE)
  elseif(NOT fr_lio_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(fr_lio_FOUND FALSE)
  endif()
  return()
endif()
set(_fr_lio_CONFIG_INCLUDED TRUE)

# output package information
if(NOT fr_lio_FIND_QUIETLY)
  message(STATUS "Found fr_lio: 0.1.0 (${fr_lio_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'fr_lio' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${fr_lio_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(fr_lio_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${fr_lio_DIR}/${_extra}")
endforeach()
