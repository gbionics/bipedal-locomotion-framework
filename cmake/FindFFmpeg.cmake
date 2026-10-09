# SPDX-FileCopyrightText: 2026 Generative Bionics S.R.L.
# SPDX-License-Identifier: BSD-3-Clause

#[=======================================================================[.rst:
FindFFmpeg
----------

Find the FFmpeg libraries (libavformat, libavcodec, libavutil and libswscale) with pkg-config.

The following imported targets are created:

FFmpeg::FFmpeg

#]=======================================================================]

include(FindPackageHandleStandardArgs)

find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
  pkg_check_modules(PC_FFmpeg QUIET IMPORTED_TARGET libavformat libavcodec libavutil libswscale)
endif()

find_package_handle_standard_args(FFmpeg DEFAULT_MSG PC_FFmpeg_FOUND)

if(FFmpeg_FOUND AND NOT TARGET FFmpeg::FFmpeg)
  add_library(FFmpeg::FFmpeg INTERFACE IMPORTED)
  set_property(TARGET FFmpeg::FFmpeg PROPERTY INTERFACE_LINK_LIBRARIES PkgConfig::PC_FFmpeg)
endif()
