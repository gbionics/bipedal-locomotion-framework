# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

if(TARGET BipedalLocomotion::RobotInterface)
  set(H_PREFIX include/BipedalLocomotion/bindings/RobotInterface)

  add_bipedal_locomotion_python_module(
    NAME RobotInterfaceBindings
    SOURCES src/IRobotControl.cpp src/ISensorBridge.cpp src/Module.cpp
    HEADERS ${H_PREFIX}/IRobotControl.h ${H_PREFIX}/ISensorBridge.h ${H_PREFIX}/Module.h
    LINK_LIBRARIES BipedalLocomotion::RobotInterface
    )
endif()

if(TARGET BipedalLocomotion::RobotInterface AND TARGET BipedalLocomotion::RobotInterfaceYarpImplementation)

  set(H_PREFIX include/BipedalLocomotion/bindings/RobotInterface)

  add_bipedal_locomotion_python_module(
    NAME RobotInterfaceYarpImplementationBindings
    SOURCES src/Polydriver.cpp src/YarpRobotControl.cpp src/YarpSensorBridge.cpp src/YarpModule.cpp
    HEADERS ${H_PREFIX}/Polydriver.h ${H_PREFIX}/YarpRobotControl.h ${H_PREFIX}/YarpSensorBridge.h ${H_PREFIX}/YarpModule.h
    LINK_LIBRARIES BipedalLocomotion::RobotInterface BipedalLocomotion::RobotInterfaceYarpImplementation
    )

endif()

if(TARGET BipedalLocomotion::RobotInterface
    AND TARGET BipedalLocomotion::PerceptionInterface
    AND TARGET BipedalLocomotion::RobotInterfaceYarpImplementation
    AND TARGET BipedalLocomotion::PerceptionInterfaceYarpImplementation)

  if(FRAMEWORK_COMPILE_PYTHON_BINDINGS)
    set(cvnp_target_link )

    # This compiles only if pybind11 is at least v2.7.0
    # Indeed we need a feature in pybind that has been introduced by this commit
    # https://github.com/pybind/pybind11/commit/74a767d42921001fc4569ecee3b8726383c42ad4
    # https://github.com/pybind/pybind11/pull/2864
    if (${pybind11_VERSION} VERSION_GREATER_EQUAL "2.7.0")
      include(FetchContent)
      FetchContent_Declare(
        cvnp
        # Temporary use a fork until https://github.com/pthom/cvnp/pull/20 is merged
        GIT_REPOSITORY https://github.com/traversaro/cvnp
        GIT_TAG        08af05fe536daa692c75c15a437be900d3b3bc0f
        )
      FetchContent_MakeAvailable(cvnp)
      set(cvnp_target_link cvnp)
    endif()
  endif()

  set(H_PREFIX include/BipedalLocomotion/bindings/RobotInterface)

  add_bipedal_locomotion_python_module(
    NAME PerceptionInterfaceBindings
    SOURCES src/PerceptionModule.cpp src/CameraBridge.cpp
    HEADERS ${H_PREFIX}/PerceptionModule.h ${H_PREFIX}/CameraBridge.h
    LINK_LIBRARIES BipedalLocomotion::PerceptionInterface BipedalLocomotion::PerceptionInterfaceYarpImplementation ${cvnp_target_link}
    ADDITIONAL_COMPILE_DEFINITIONS pybind11_VERSION_MAJOR=${pybind11_VERSION_MAJOR}
                                   pybind11_VERSION_MINOR=${pybind11_VERSION_MINOR}
                                   pybind11_VERSION_PATCH=${pybind11_VERSION_PATCH}
    )

endif()
