# Copyright (C) 2024 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

add_bipedal_test(
  NAME YarpRobotLoggerDevice
  SOURCES YarpRobotLoggerDeviceTest.cpp
  LINKS YARP::YARP_robotinterface matioCpp::matioCpp BipedalLocomotion::TextLogging BipedalLocomotion::YarpRobotLoggerDeviceCommands)

if(TARGET YarpRobotLoggerDeviceUnitTests)
  target_compile_definitions(YarpRobotLoggerDeviceUnitTests PRIVATE CMAKE_CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  target_compile_definitions(YarpRobotLoggerDeviceUnitTests PRIVATE CMAKE_BINARY_DIR="${CMAKE_BINARY_DIR}")
  target_compile_definitions(YarpRobotLoggerDeviceUnitTests PRIVATE YARP_DATA_INSTALL_DIR_FULL="${YARP_DATA_INSTALL_DIR_FULL}")
endif()
