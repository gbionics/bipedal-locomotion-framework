# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

# Detect if we are doing a standalone build of the bindings, using an external bipedal-locomotion-framework
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  set(FRAMEWORK_PYTHON_BINDINGS_BUILD_STANDALONE ON)
  set(FRAMEWORK_COMPILE_PYTHON_BINDINGS ON)
else()
  set(FRAMEWORK_PYTHON_BINDINGS_BUILD_STANDALONE FALSE)
endif()

if(FRAMEWORK_PYTHON_BINDINGS_BUILD_STANDALONE)
  cmake_minimum_required(VERSION 3.16)
  project(BipedalLocomotionFrameworkBindings)
  include(GNUInstallDirs)
  find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
  find_package(pybind11 REQUIRED)
  find_package(BipedalLocomotionFramework REQUIRED)
  get_filename_component(CMAKE_CURRENT_PARENT_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR} DIRECTORY)
  list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_PARENT_SOURCE_DIR}/cmake)
  include(AddBipedalLocomotionLibrary)
  include(AddBipedalLocomotionPythonModule)
endif()

if(FRAMEWORK_COMPILE_PYTHON_BINDINGS)

    set(NEW_LINE "\n")

    option(FRAMEWORK_DETECT_ACTIVE_PYTHON_SITEPACKAGES
        "Do you want BLF to detect and use the active site-package directory? (it could be a system dir)"
        FALSE)

    if(NOT DEFINED FRAMEWORK_PYTHON_INSTALL_DIR)
      if(FRAMEWORK_DETECT_ACTIVE_PYTHON_SITEPACKAGES)
        set(FRAMEWORK_PYTHON_INSTALL_DIR ${Python3_SITELIB})
      else()
        execute_process(COMMAND ${Python3_EXECUTABLE} -c "import os;import sysconfig;relative_site_packages = sysconfig.get_path('purelib').replace(sysconfig.get_path('data'), '').lstrip(os.path.sep);print(relative_site_packages)"
                        OUTPUT_VARIABLE _PYTHON_INSTDIR)
        string(STRIP ${_PYTHON_INSTDIR} _PYTHON_INSTDIR_CLEAN)
        set(FRAMEWORK_PYTHON_INSTALL_DIR ${_PYTHON_INSTDIR_CLEAN})
      endif()
    endif()
    set(PYTHON_INSTDIR ${FRAMEWORK_PYTHON_INSTALL_DIR}/bipedal_locomotion_framework)


    # Folder of the Python package within the build tree.
    # It is used for the Python tests.
    set(BLF_PYTHON_PACKAGE "${CMAKE_BINARY_DIR}/bipedal_locomotion_framework")

    # Create the __init__.py file
    file(WRITE ${BLF_PYTHON_PACKAGE}/__init__.py "")

    # If we are on Windows and BUILD_SHARED_LIBS is ON, handle the fact that
    # the Python interpreter does not look into PATH to find dll (see https://docs.python.org/3.8/library/os.html#os.add_dll_directory)
    if(WIN32 AND BUILD_SHARED_LIBS)
        if(IS_ABSOLUTE PYTHON_INSTDIR)
            set(PYTHON_FULL_INSTDIR "${PYTHON_INSTDIR}")
        else()
            set(PYTHON_FULL_INSTDIR "${CMAKE_INSTALL_PREFIX}/${PYTHON_INSTDIR}")
        endif()
        file(RELATIVE_PATH RELATIVE_PATH_BETWEEN_INIT_PY_AND_DLL_DIRECTORY ${PYTHON_FULL_INSTDIR} ${CMAKE_INSTALL_FULL_BINDIR})
        file(APPEND "${BLF_PYTHON_PACKAGE}/__init__.py" "import os${NEW_LINE}")
        file(APPEND "${BLF_PYTHON_PACKAGE}/__init__.py" "library_dll_path = os.path.join(os.path.dirname(__file__),'${RELATIVE_PATH_BETWEEN_INIT_PY_AND_DLL_DIRECTORY}')${NEW_LINE}")
        file(APPEND "${BLF_PYTHON_PACKAGE}/__init__.py" "# Avoid to call add_dll_directory if not necessary,${NEW_LINE}")
        file(APPEND "${BLF_PYTHON_PACKAGE}/__init__.py" "# for example if the library to find are already found in the proper location in a conda environment${NEW_LINE}")
        file(APPEND "${BLF_PYTHON_PACKAGE}/__init__.py" "if(library_dll_path != os.path.join(os.environ.get('CONDA_PREFIX', ''),'Library','bin') and library_dll_path != os.path.join(os.environ.get('CONDA_PREFIX', ''),'bin')):${NEW_LINE}")
        file(APPEND "${BLF_PYTHON_PACKAGE}/__init__.py" "    if(os.path.exists(library_dll_path)):${NEW_LINE}")
        file(APPEND "${BLF_PYTHON_PACKAGE}/__init__.py" "        os.add_dll_directory(library_dll_path)${NEW_LINE}")

        # For running tests, we need to add also the path where the dependecies are installed.
        # We add idyntree to update the dll path for the tests.
        file(APPEND "${BLF_PYTHON_PACKAGE}/__init__.py" "import idyntree${NEW_LINE}")


    endif()

    file(APPEND "${BLF_PYTHON_PACKAGE}/__init__.py" "from .bindings import *${NEW_LINE}from . import utils${NEW_LINE}")

    # Install the __init__.py file
    install(FILES "${BLF_PYTHON_PACKAGE}/__init__.py"
            DESTINATION ${PYTHON_INSTDIR})

    # Install pip metadata files to ensure that blf installed via CMake is listed by pip list
    # See https://packaging.python.org/specifications/recording-installed-packages/
    # and https://packaging.python.org/en/latest/specifications/core-metadata/#core-metadata
    option(BLF_PYTHON_PIP_METADATA_INSTALL "Use CMake to install Python pip metadata. Set to off if some other tool already installs it." ON)
    mark_as_advanced(BLF_PYTHON_PIP_METADATA_INSTALL)
    set(BLF_PYTHON_PIP_METADATA_INSTALLER "cmake" CACHE STRING "Specify the string to identify the pip Installer. Default: cmake, change this if you are using another tool.")
    mark_as_advanced(BLF_PYTHON_PIP_METADATA_INSTALLER)
    if(BLF_PYTHON_PIP_METADATA_INSTALL)
      file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/METADATA "")
      file(APPEND ${CMAKE_CURRENT_BINARY_DIR}/METADATA "Metadata-Version: 2.1${NEW_LINE}")
      file(APPEND ${CMAKE_CURRENT_BINARY_DIR}/METADATA "Name: bipedal_locomotion_framework${NEW_LINE}")
      file(APPEND ${CMAKE_CURRENT_BINARY_DIR}/METADATA "Version: ${BipedalLocomotionFramework_VERSION}${NEW_LINE}")
      file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/INSTALLER "${BLF_PYTHON_PIP_METADATA_INSTALLER}${NEW_LINE}")
      install(
        FILES "${CMAKE_CURRENT_BINARY_DIR}/METADATA" "${CMAKE_CURRENT_BINARY_DIR}/INSTALLER"
        DESTINATION ${FRAMEWORK_PYTHON_INSTALL_DIR}/bipedal_locomotion_framework-${BipedalLocomotionFramework_VERSION}.dist-info)
    endif()

endif()

# Add the bindings directory, this is done even if FRAMEWORK_COMPILE_PYTHON_BINDINGS is OFF
# to ensure that the header-only bindings C++ library as installed nevertheless
add_subdirectory(python)
