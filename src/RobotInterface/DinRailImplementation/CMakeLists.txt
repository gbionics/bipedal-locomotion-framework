# Copyright (C) Generative Bionics S.R.L. All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

if(FRAMEWORK_COMPILE_DinRailImplementation)

  add_bipedal_locomotion_library(
    NAME                   RobotInterfaceDinRailImplementation
    SOURCES                src/DinRailRobotControl.cpp
    PUBLIC_HEADERS         include/BipedalLocomotion/RobotInterface/DinRailRobotControl.h
    PUBLIC_LINK_LIBRARIES  BipedalLocomotion::RobotInterfaceYarpImplementation
                           dinrail::dinrail
                           YARP::YARP_dev
    PRIVATE_LINK_LIBRARIES BipedalLocomotion::TextLogging
    INSTALLATION_FOLDER    RobotInterface)

  if(FRAMEWORK_COMPILE_Camera)
    add_bipedal_locomotion_library(
      NAME                   CameraInterfaceDinrailImplementation
      SOURCES                src/DinrailCameraBridge.cpp src/DinrailHelper.cpp
      PUBLIC_HEADERS         include/BipedalLocomotion/RobotInterface/DinrailCameraBridge.h include/BipedalLocomotion/RobotInterface/DinrailHelper.h
      PUBLIC_LINK_LIBRARIES  BipedalLocomotion::CameraInterface BipedalLocomotion::ParametersHandler dinrail::dinrail
      PRIVATE_LINK_LIBRARIES BipedalLocomotion::TextLogging BipedalLocomotion::System
      SUBDIRECTORIES         tests
      INSTALLATION_FOLDER    RobotInterface)
  endif()

endif()
