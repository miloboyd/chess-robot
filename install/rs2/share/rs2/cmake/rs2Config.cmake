# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_rs2_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED rs2_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(rs2_FOUND FALSE)
  elseif(NOT rs2_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(rs2_FOUND FALSE)
  endif()
  return()
endif()
set(_rs2_CONFIG_INCLUDED TRUE)

# output package information
if(NOT rs2_FIND_QUIETLY)
  message(STATUS "Found rs2: 0.0.1 (${rs2_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'rs2' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${rs2_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(rs2_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${rs2_DIR}/${_extra}")
endforeach()
