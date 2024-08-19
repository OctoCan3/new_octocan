# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_octocan_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED octocan_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(octocan_FOUND FALSE)
  elseif(NOT octocan_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(octocan_FOUND FALSE)
  endif()
  return()
endif()
set(_octocan_CONFIG_INCLUDED TRUE)

# output package information
if(NOT octocan_FIND_QUIETLY)
  message(STATUS "Found octocan: 0.0.1 (${octocan_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'octocan' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${octocan_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(octocan_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_libraries-extras.cmake;ament_cmake_export_dependencies-extras.cmake")
foreach(_extra ${_extras})
  include("${octocan_DIR}/${_extra}")
endforeach()
