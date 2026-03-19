# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

if(TARGET BipedalLocomotion::System)

  set(H_PREFIX include/BipedalLocomotion/bindings/System)

  add_bipedal_locomotion_library(
    NAME BindingsSystem
    IS_INTERFACE
    PUBLIC_HEADERS ${H_PREFIX}/Advanceable.h
    PUBLIC_LINK_LIBRARIES BipedalLocomotion::System
    INSTALLATION_FOLDER bindings/System)

  add_bipedal_locomotion_python_module(
    NAME SystemBindings
    SOURCES src/Advanceable.cpp src/VariablesHandler.cpp src/LinearTask.cpp src/Module.cpp src/ITaskControllerManager.cpp src/IClock.cpp src/Clock.cpp src/WeightProvider.cpp
    HEADERS ${H_PREFIX}/VariablesHandler.h ${H_PREFIX}/LinearTask.h ${H_PREFIX}/ITaskControllerManager.h ${H_PREFIX}/ILinearTaskSolver.h ${H_PREFIX}/IClock.h ${H_PREFIX}/Clock.h ${H_PREFIX}/WeightProvider.h
    LINK_LIBRARIES BipedalLocomotion::System
    TESTS tests/test_variables_handler.py
    )

  if(TARGET BipedalLocomotion::SystemYarpImplementation)

    add_bipedal_locomotion_python_module(
      NAME  SystemYarpImplementationBindings
      SOURCES src/YarpClock.cpp src/YarpModule.cpp
      HEADERS ${H_PREFIX}/YarpClock.h ${H_PREFIX}/YarpModule.h
      LINK_LIBRARIES BipedalLocomotion::SystemYarpImplementation
      )

  endif()

  if(TARGET BipedalLocomotion::SystemRosImplementation)

    add_bipedal_locomotion_python_module(
      NAME  SystemRosImplementationBindings
      SOURCES src/RosClock.cpp src/RosModule.cpp
      HEADERS ${H_PREFIX}/RosClock.h ${H_PREFIX}/RosModule.h
      LINK_LIBRARIES BipedalLocomotion::SystemRosImplementation
      )

  endif()


endif()
