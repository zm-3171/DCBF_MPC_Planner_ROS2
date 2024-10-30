# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_obs_param_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED obs_param_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(obs_param_FOUND FALSE)
  elseif(NOT obs_param_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(obs_param_FOUND FALSE)
  endif()
  return()
endif()
set(_obs_param_CONFIG_INCLUDED TRUE)

# output package information
if(NOT obs_param_FIND_QUIETLY)
  message(STATUS "Found obs_param: 0.0.0 (${obs_param_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'obs_param' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${obs_param_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(obs_param_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${obs_param_DIR}/${_extra}")
endforeach()
