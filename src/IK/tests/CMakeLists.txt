# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

add_bipedal_test(
  NAME R3TaskIK
  SOURCES R3TaskTest.cpp
  LINKS BipedalLocomotion::IK BipedalLocomotion::ManifConversions)

add_bipedal_test(
  NAME SE3TaskIK
  SOURCES SE3TaskTest.cpp
  LINKS BipedalLocomotion::IK BipedalLocomotion::ManifConversions)

add_bipedal_test(
  NAME SO3TaskIK
  SOURCES SO3TaskTest.cpp
  LINKS BipedalLocomotion::IK BipedalLocomotion::ManifConversions)

add_bipedal_test(
  NAME JointsTrackingTaskIK
  SOURCES JointTrackingTaskTest.cpp
  LINKS BipedalLocomotion::IK)

add_bipedal_test(
  NAME JointsLimitsTaskIK
  SOURCES JointLimitsTaskTest.cpp
  LINKS BipedalLocomotion::IK OsqpEigen::OsqpEigen)

add_bipedal_test(
  NAME CoMTaskIK
  SOURCES CoMTaskTest.cpp
  LINKS BipedalLocomotion::IK)

add_bipedal_test(
  NAME AngularMomentumTaskIK
  SOURCES AngularMomentumTaskTest.cpp
  LINKS BipedalLocomotion::IK)

add_bipedal_test(
  NAME DistanceTaskIK
  SOURCES DistanceTaskTest.cpp
  LINKS BipedalLocomotion::IK)

add_bipedal_test(
  NAME GravityTaskIK
  SOURCES GravityTaskTest.cpp
  LINKS BipedalLocomotion::IK)

  add_bipedal_test(
    NAME JointVelocityLimitsTaskIK
    SOURCES JointVelocityLimitsTaskTest.cpp
    LINKS BipedalLocomotion::IK)

if (FRAMEWORK_COMPILE_ContinuousDynamicalSystem)

  add_bipedal_test(
    NAME QPInverseKinematics
    SOURCES QPInverseKinematicsTest.cpp
    LINKS BipedalLocomotion::IK BipedalLocomotion::ManifConversions
    BipedalLocomotion::ContinuousDynamicalSystem)

  add_bipedal_test(
    NAME QPFixedBaseInverseKinematics
    SOURCES QPFixedBaseInverseKinematicsTest.cpp
    LINKS BipedalLocomotion::IK BipedalLocomotion::ManifConversions
    BipedalLocomotion::ContinuousDynamicalSystem)

endif()
