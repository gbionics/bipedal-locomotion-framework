# SPDX-FileCopyrightText: 2023 Giulio Romualdi
# SPDX-License-Identifier: BSD-3-Clause

#[=======================================================================[.rst:
Findonnxruntime
-----------

The following imported targets are created:

onnxruntime::onnxruntime

#]=======================================================================]

include(FindPackageHandleStandardArgs)

# Prefer the config package when available, as it defines the include paths
# and imported target for the installed version of ONNX Runtime.
find_package(onnxruntime CONFIG QUIET)
if(onnxruntime_FOUND)
  get_target_property(_onnxruntime_include_dirs onnxruntime::onnxruntime INTERFACE_INCLUDE_DIRECTORIES)
endif()

find_path(onnxruntime_INCLUDE_DIR
  NAMES onnxruntime_cxx_api.h
  HINTS ${_onnxruntime_include_dirs}
  PATH_SUFFIXES onnxruntime onnxruntime/core/session core/session)
mark_as_advanced(onnxruntime_INCLUDE_DIR)

if(onnxruntime_FOUND)
  find_package_handle_standard_args(onnxruntime CONFIG_MODE REQUIRED_VARS onnxruntime_INCLUDE_DIR)
  # Older config packages export include/onnxruntime, but install the headers
  # under core/session. Preserve their target and add the directory for bare includes.
  if(onnxruntime_FOUND AND NOT onnxruntime_INCLUDE_DIR IN_LIST _onnxruntime_include_dirs)
    set_property(TARGET onnxruntime::onnxruntime APPEND PROPERTY
      INTERFACE_INCLUDE_DIRECTORIES "${onnxruntime_INCLUDE_DIR}")
  endif()
  unset(_onnxruntime_include_dirs)
  return()
endif()
unset(_onnxruntime_include_dirs)
find_library(onnxruntime_LIBRARY
  NAMES onnxruntime onnxruntime_conda)
mark_as_advanced(onnxruntime_LIBRARY)

find_package_handle_standard_args(onnxruntime DEFAULT_MSG onnxruntime_INCLUDE_DIR onnxruntime_LIBRARY)

if(onnxruntime_FOUND)
  if(NOT TARGET onnxruntime::onnxruntime)
    add_library(onnxruntime::onnxruntime UNKNOWN IMPORTED)
    set_target_properties(onnxruntime::onnxruntime PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${onnxruntime_INCLUDE_DIR}")
    set_property(TARGET onnxruntime::onnxruntime PROPERTY
      IMPORTED_LOCATION "${onnxruntime_LIBRARY}")
  endif()
endif()
