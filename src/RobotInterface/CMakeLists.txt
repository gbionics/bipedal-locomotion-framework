# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

set(H_PREFIX include/BipedalLocomotion/RobotInterface)

if (FRAMEWORK_COMPILE_RobotInterface)

add_bipedal_locomotion_library(
    NAME                   RobotInterface
    SOURCES                src/IRobotControl.cpp
    PUBLIC_HEADERS         ${H_PREFIX}/ISensorBridge.h ${H_PREFIX}/IRobotControl.h ${H_PREFIX}/JointType.h
    PUBLIC_LINK_LIBRARIES  BipedalLocomotion::ParametersHandler Eigen3::Eigen
    PRIVATE_LINK_LIBRARIES BipedalLocomotion::TextLogging
    SUBDIRECTORIES         YarpImplementation tests)
endif()

if (FRAMEWORK_COMPILE_Camera)

add_bipedal_locomotion_library(
    NAME                   CameraInterface
    IS_INTERFACE
    PUBLIC_HEADERS         ${H_PREFIX}/ICameraBridge.h
    PUBLIC_LINK_LIBRARIES  BipedalLocomotion::ParametersHandler BipedalLocomotion::System ${OpenCV_LIBS}
    INSTALLATION_FOLDER    RobotInterface)
endif()

if (FRAMEWORK_COMPILE_Pointcloud)

add_bipedal_locomotion_library(
    NAME                   PointCloudInterface
    IS_INTERFACE
    PUBLIC_HEADERS         ${H_PREFIX}/IPointCloudBridge.h
    PUBLIC_LINK_LIBRARIES  BipedalLocomotion::ParametersHandler BipedalLocomotion::System pcl_common
    INSTALLATION_FOLDER    RobotInterface)
endif()
