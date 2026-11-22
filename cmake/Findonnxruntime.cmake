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
  find_package_handle_standard_args(onnxruntime CONFIG_MODE)
  return()
endif()

find_path(onnxruntime_INCLUDE_DIR
  NAMES onnxruntime_cxx_api.h
  PATH_SUFFIXES onnxruntime onnxruntime/core/session)
mark_as_advanced(onnxruntime_INCLUDE_DIR)
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
