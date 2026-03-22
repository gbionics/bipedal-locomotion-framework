# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

function(add_bipedal_locomotion_python_module)
  # If bindings support is not enabled, add_bipedal_locomotion_python_module
  # is a no operation
  if(NOT FRAMEWORK_COMPILE_PYTHON_BINDINGS)
    return()
  endif()

  set(options )
  set(oneValueArgs NAME)
  set(multiValueArgs
    SOURCES
    HEADERS
    LINK_LIBRARIES
    TESTS
    TESTS_RUNTIME_CONDITIONS
    ADDITIONAL_COMPILE_DEFINITIONS)

  set(prefix "bipedal_component")

  cmake_parse_arguments(${prefix}
    "${options}"
    "${oneValueArgs}"
    "${multiValueArgs}"
    ${ARGN})

  set(name ${${prefix}_NAME})
  set(is_interface ${${prefix}_IS_INTERFACE})
  set(sources ${${prefix}_SOURCES})
  set(headers ${${prefix}_HEADERS})
  set(link_libraries ${${prefix}_LINK_LIBRARIES})
  set(subdirectories ${${prefix}_SUBDIRECTORIES})
  set(tests ${${prefix}_TESTS})
  set(tests_runtime_conditions ${${prefix}_TESTS_RUNTIME_CONDITIONS})
  set(additional_compile_definitions ${${prefix}_ADDITIONAL_COMPILE_DEFINITIONS})

  foreach(file ${headers})
    set_property(GLOBAL APPEND PROPERTY pybind_headers ${CMAKE_CURRENT_SOURCE_DIR}/${file})
  endforeach()

  foreach(file ${sources})
    set_property(GLOBAL APPEND PROPERTY pybind_sources ${CMAKE_CURRENT_SOURCE_DIR}/${file})
  endforeach()

  set_property(GLOBAL APPEND PROPERTY pybind_include_dirs ${CMAKE_CURRENT_SOURCE_DIR}/include)

  set_property(GLOBAL APPEND PROPERTY pybind_link_libraries ${link_libraries})

  set(run_python_test ON)
  foreach(condition ${tests_runtime_conditions})
    if(NOT ${condition})
      set(run_python_test OFF)
      break()
    endif()
  endforeach()

  if(${run_python_test})
    foreach(test ${tests})
      set_property(GLOBAL APPEND PROPERTY BipedalLocomotionFrameworkBindings_TESTS
        ${CMAKE_CURRENT_SOURCE_DIR}/${test})
    endforeach()
  endif()

  foreach(definition ${additional_compile_definitions})
    set_property(GLOBAL APPEND PROPERTY pybind_compile_definitions ${definition})
  endforeach()

  message(STATUS "Added files for bindings named ${name} in ${PROJECT_NAME}.")

endfunction()
