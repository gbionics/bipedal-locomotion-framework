# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

include(BipedalLocomotionFrameworkFindDependencies)
include(BipedalLocomotionDependencyClassifier)

# Workaround for issue that occurs with CMake 3.26.1 and pybind11 2.4.3
# see https://github.com/ami-iit/bipedal-locomotion-framework/issues/636
# This is done here as it needs to be done before any call (even transitive)
# to find_package(pybind11)
# It can be removed once pybind11 2.4.3 is not supported anymore
if(NOT DEFINED CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

################################################################################
########################## Mandatory dependencies ##############################

find_package(iDynTree 12.2.1 REQUIRED)
dependency_classifier(iDynTree MINIMUM_VERSION 12.2.1 IS_USED TRUE PUBLIC)

find_package(Eigen3 REQUIRED)
dependency_classifier(Eigen3 IS_USED TRUE PUBLIC)

find_package(spdlog 1.5.0 REQUIRED)
dependency_classifier(spdlog MINIMUM_VERSION 1.5.0 IS_USED TRUE PUBLIC)

########################## Optional dependencies ##############################

blf_optional_find_package(rclcpp 16.0.0 QUIET)
checkandset_dependency(rclcpp MINIMUM_VERSION 16.0.0)
dependency_classifier(rclcpp MINIMUM_VERSION 16.0.0 IS_USED ${FRAMEWORK_USE_rclcpp} PUBLIC)

blf_optional_find_package(YARP 3.7.0 COMPONENTS companion profiler dev os idl_tools robotinterface QUIET)
checkandset_dependency(YARP MINIMUM_VERSION 3.7.0)
dependency_classifier(YARP MINIMUM_VERSION 3.7.0 IS_USED ${FRAMEWORK_USE_YARP}
                      COMPONENTS companion profiler dev os idl_tools robotinterface PUBLIC)

blf_optional_find_package(Qhull QUIET)
checkandset_dependency(Qhull MINIMUM_VERSION)
dependency_classifier(Qhull MINIMUM_VERSION IS_USED ${FRAMEWORK_USE_Qhull} PRIVATE)

blf_optional_find_package(casadi QUIET)
checkandset_dependency(casadi)
dependency_classifier(casadi IS_USED ${FRAMEWORK_USE_casadi} PUBLIC)
add_compile_definitions(casadi_VERSION=${casadi_VERSION})
# Support for both casadi < and >= 3.7.0
if(casadi_FOUND AND TARGET casadi AND NOT casadi::casadi)
  # This requires at least CMake 3.18 as minimum supported CMake version https://gitlab.kitware.com/cmake/cmake/-/merge_requests/4837
  add_library(casadi::casadi ALIAS casadi)
endif()

blf_optional_find_package(cppad QUIET)
checkandset_dependency(cppad)
dependency_classifier(cppad PUBLIC IS_USED ${FRAMEWORK_USE_cppad})

blf_optional_find_package(manif 0.0.4 QUIET)
checkandset_dependency(manif MINIMUM_VERSION 0.0.4)
dependency_classifier(manif MINIMUM_VERSION 0.0.4 IS_USED ${FRAMEWORK_USE_manif} PUBLIC)

blf_optional_find_package(OsqpEigen 0.7.0 QUIET)
checkandset_dependency(OsqpEigen MINIMUM_VERSION 0.7.0)
dependency_classifier(OsqpEigen MINIMUM_VERSION 0.7.0 IS_USED ${FRAMEWORK_USE_OsqpEigen})

blf_optional_find_package(Python3 3.6 COMPONENTS Interpreter Development QUIET)
checkandset_dependency(Python3 MINIMUM_VERSION 3.6 COMPONENTS Interpreter Development)

blf_optional_find_package(pybind11 2.4.3 CONFIG QUIET)
checkandset_dependency(pybind11)

blf_optional_find_package(matioCpp QUIET)
checkandset_dependency(matioCpp)
dependency_classifier(matioCpp IS_USED ${FRAMEWORK_USE_matioCpp} PUBLIC)

blf_optional_find_package(LieGroupControllers 0.2.0 QUIET)
checkandset_dependency(LieGroupControllers MINIMUM_VERSION 0.2.0)
dependency_classifier(LieGroupControllers MINIMUM_VERSION 0.2.0 IS_USED ${FRAMEWORK_USE_LieGroupControllers} PUBLIC)

blf_optional_find_package(OpenCV QUIET)
checkandset_dependency(OpenCV)
dependency_classifier(OpenCV IS_USED ${FRAMEWORK_USE_OpenCV} PUBLIC)

blf_optional_find_package(PCL QUIET)
checkandset_dependency(PCL)
dependency_classifier(PCL IS_USED ${FRAMEWORK_USE_PCL} PUBLIC)

blf_optional_find_package(realsense2 QUIET)
checkandset_dependency(realsense2)
dependency_classifier(realsense2 IS_USED ${FRAMEWORK_USE_realsense2} PUBLIC)

blf_optional_find_package(nlohmann_json 3.7.3 QUIET)
checkandset_dependency(nlohmann_json MINIMUM_VERSION 3.7.3)
dependency_classifier(nlohmann_json MINIMUM_VERSION 3.7.3 IS_USED ${FRAMEWORK_USE_nlohmann_json})

blf_optional_find_package(tomlplusplus 3.0.1 QUIET)
checkandset_dependency(tomlplusplus MINIMUM_VERSION 3.0.1)
dependency_classifier(tomlplusplus MINIMUM_VERSION 3.0.1 IS_USED ${FRAMEWORK_USE_tomlplusplus} PUBLIC)

blf_optional_find_package(robometry 1.1.0 QUIET)
checkandset_dependency(robometry MINIMUM_VERSION 1.1.0)
dependency_classifier(robometry MINIMUM_VERSION 1.1.0 IS_USED ${FRAMEWORK_USE_robometry})

blf_optional_find_package(BayesFilters QUIET)
checkandset_dependency(BayesFilters)
dependency_classifier(BayesFilters IS_USED ${FRAMEWORK_USE_BayesFilters} PUBLIC)

# required only for some tests
blf_optional_find_package(icub-models 1.23.3 QUIET)
checkandset_dependency(icub-models)

blf_optional_find_package(VALGRIND QUIET)
checkandset_dependency(VALGRIND)

blf_optional_find_package(UnicyclePlanner QUIET)
checkandset_dependency(UnicyclePlanner)

blf_optional_find_package(onnxruntime QUIET)
checkandset_dependency(onnxruntime)

blf_optional_find_package(trintrin QUIET)
checkandset_dependency(trintrin)

##########################      Test-related options       ##############################

# MemoryAllocationMonitor require glibc >= 2.35
set(FRAMEWORK_GLIBC_GEQ_2_35 OFF)
if(BUILD_TESTING AND UNIX AND NOT APPLE)
  execute_process(COMMAND ldd --version
                  OUTPUT_VARIABLE FRAMEWORK_LDD_VERSION_OUTPUT)
  string(REGEX MATCH "GLIBC ([0-9]+.[0-9]+)" FRAMEWORK_LDD_REGEX_OUTPUT ${FRAMEWORK_LDD_VERSION_OUTPUT})
  set(FRAMEWORK_LDD_GLIBC_VERSION ${CMAKE_MATCH_1})
  if(FRAMEWORK_LDD_GLIBC_VERSION VERSION_GREATER_EQUAL "2.35")
    set(FRAMEWORK_GLIBC_GEQ_2_35 ON)
  endif()
endif()
framework_dependent_option(FRAMEWORK_RUN_MemoryAllocationMonitor_tests
  "Run MemoryAllocationMonitor tests?" ON
  "BUILD_TESTING;UNIX;NOT APPLE;FRAMEWORK_GLIBC_GEQ_2_35" OFF)

framework_dependent_option(FRAMEWORK_RUN_Valgrind_tests
  "Run Valgrind tests?" OFF
  "BUILD_TESTING;VALGRIND_FOUND" OFF)

##########################      Components       ##############################

framework_dependent_option(FRAMEWORK_COMPILE_YarpUtilities
  "Compile YarpHelper library?" ON
  "FRAMEWORK_USE_YARP" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_RosImplementation
  "Compile All the ROS implementations?" ON
  "FRAMEWORK_USE_rclcpp" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_YarpImplementation
  "Compile All the YARP implementations?" ON
  "FRAMEWORK_COMPILE_YarpUtilities" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_TomlImplementation
  "Compile All the TOML implementations?" ON
  "FRAMEWORK_USE_tomlplusplus" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_Estimators
  "Compile Estimators library?" ON
  "" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_Contact
  "Compile Contact libraries?" ON
  "FRAMEWORK_USE_manif;FRAMEWORK_USE_nlohmann_json" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_System
  "Compile System library?" ON
  "" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_Math
  "Compile Math library?" ON
  "FRAMEWORK_USE_manif;FRAMEWORK_COMPILE_System" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_Planners
  "Compile Planners libraries?" ON
  "FRAMEWORK_USE_Qhull;FRAMEWORK_USE_casadi;FRAMEWORK_USE_manif;FRAMEWORK_COMPILE_Math;FRAMEWORK_COMPILE_Contact" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_ContactModels
  "Compile ContactModels library?" ON
  "" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_Unicycle
    "Compile the Unicycle Planner library?" ON
    "FRAMEWORK_USE_UnicyclePlanner;FRAMEWORK_COMPILE_System;FRAMEWORK_COMPILE_Planners;FRAMEWORK_COMPILE_Contact" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_ContinuousDynamicalSystem
  "Compile System ContinuousDynamicalSystem?" ON
  "FRAMEWORK_COMPILE_ContactModels;FRAMEWORK_COMPILE_Math;FRAMEWORK_COMPILE_Contact" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_AutoDiffCppAD
  "Compile CppAD-Eigen wrapper?" ON
  "FRAMEWORK_USE_cppad" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_RobotInterface
  "Compile RobotInterface libraries?" ON
  "" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_FloatingBaseEstimators
  "Compile FloatingBaseEstimators libraries?" ON
  "FRAMEWORK_USE_manif;FRAMEWORK_COMPILE_Contact" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_ManifConversions
  "Compile manif Conversions libraries?" ON
  "FRAMEWORK_USE_manif" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_RobotDynamicsEstimator
  "Compile RobotDynamicsEstimator libraries?" ON
  "FRAMEWORK_COMPILE_System;FRAMEWORK_COMPILE_ManifConversions;FRAMEWORK_USE_manif;FRAMEWORK_USE_BayesFilters" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_matioCppConversions
  "Compile matioCpp Conversions libraries?" ON
  "FRAMEWORK_USE_matioCpp" OFF)

  framework_dependent_option(FRAMEWORK_COMPILE_CasadiConversions
  "Compile casadi Conversions libraries?" ON
  "FRAMEWORK_USE_casadi" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_TSID
  "Compile TSID library?" ON
  "FRAMEWORK_COMPILE_System;FRAMEWORK_USE_LieGroupControllers;FRAMEWORK_COMPILE_ManifConversions;FRAMEWORK_USE_manif;FRAMEWORK_COMPILE_Contact;FRAMEWORK_USE_OsqpEigen" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_IK
  "Compile IK library?" ON
  "FRAMEWORK_COMPILE_System;FRAMEWORK_USE_LieGroupControllers;FRAMEWORK_COMPILE_ManifConversions;FRAMEWORK_USE_manif;FRAMEWORK_USE_OsqpEigen" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_ML
  "Compile machine learning libraries?" ON
  "FRAMEWORK_USE_onnxruntime;FRAMEWORK_USE_manif" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_SimplifiedModelControllers
  "Compile SimplifiedModelControllers library?" ON
  "FRAMEWORK_USE_manif" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_JointPositionTrackingApplication
  "Compile joint-position-tracking application?" ON
  "FRAMEWORK_COMPILE_YarpImplementation;FRAMEWORK_COMPILE_Planners;FRAMEWORK_COMPILE_RobotInterface;FRAMEWORK_COMPILE_Contact" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_PYTHON_BINDINGS
  "Do you want to generate and compile the Python bindings?" ON
  "FRAMEWORK_USE_Python3;FRAMEWORK_USE_pybind11;FRAMEWORK_COMPILE_Math" OFF)

framework_dependent_option(FRAMEWORK_TEST_PYTHON_BINDINGS
  "Do you want to test the Python bindings?" ON
  "BUILD_TESTING;FRAMEWORK_COMPILE_PYTHON_BINDINGS" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_MasImuTest
  "Compile test on the MAS IMU?" ON
  "FRAMEWORK_COMPILE_YarpImplementation;FRAMEWORK_COMPILE_matioCppConversions;FRAMEWORK_COMPILE_Math" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_JointTrajectoryPlayer
  "Compile joint-trajectory-player application?" ON
  "FRAMEWORK_COMPILE_YarpImplementation;FRAMEWORK_COMPILE_RobotInterface;FRAMEWORK_COMPILE_matioCppConversions;FRAMEWORK_USE_matioCpp;FRAMEWORK_USE_YARP;FRAMEWORK_COMPILE_Contact" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_Perception
  "Compile Perception libraries?" ON
  "FRAMEWORK_USE_OpenCV;FRAMEWORK_USE_PCL" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_PerceptionInterface
  "Compile PerceptionInterface libraries?" ON
  "FRAMEWORK_COMPILE_Perception" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_RealsenseCapture
  "Compile Realsense related software?" ON
  "FRAMEWORK_COMPILE_PerceptionInterface;FRAMEWORK_USE_realsense2" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_RealSenseTestApplication
  "Compile realsense-test application?" ON
  "FRAMEWORK_COMPILE_YarpImplementation;FRAMEWORK_COMPILE_Perception;FRAMEWORK_COMPILE_RealsenseCapture;FRAMEWORK_COMPILE_PerceptionInterface" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_CalibrationDeltaUpdaterApplication
  "Compile calibration-delta-updater application?" ON
  "FRAMEWORK_COMPILE_YarpImplementation;FRAMEWORK_COMPILE_PYTHON_BINDINGS;FRAMEWORK_COMPILE_RobotInterface" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_BalancingPositionControlApplication
  "Compile balancing-position-control application?" ON
  "FRAMEWORK_COMPILE_YarpImplementation;FRAMEWORK_COMPILE_PYTHON_BINDINGS;FRAMEWORK_COMPILE_RobotInterface;FRAMEWORK_COMPILE_IK" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_YarpRobotLoggerDevice
  "Do you want to generate and compile the YarpRobotLoggerDevice?" ON
  "FRAMEWORK_COMPILE_RobotInterface;FRAMEWORK_COMPILE_YarpImplementation;FRAMEWORK_COMPILE_Perception;FRAMEWORK_COMPILE_YarpUtilities;FRAMEWORK_USE_robometry;FRAMEWORK_USE_trintrin" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_JointTorqueControlDevice
  "Do you want to generate and compile the YarpRobotLoggerDevice?" ON
  "FRAMEWORK_COMPILE_YarpImplementation;FRAMEWORK_COMPILE_YarpUtilities;FRAMEWORK_USE_onnxruntime" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_VectorsCollectionWrapper
  "Do you want to generate and compile the VectorsCollectionWrapper?" ON
  "FRAMEWORK_COMPILE_YarpImplementation;FRAMEWORK_COMPILE_YarpUtilities" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_ReducedModelControllers
  "Do you want to generate and compile the ReducedModelControllers?" ON
  "FRAMEWORK_USE_casadi;FRAMEWORK_COMPILE_System;FRAMEWORK_COMPILE_Contact;FRAMEWORK_COMPILE_Math;FRAMEWORK_COMPILE_CasadiConversions" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_JointLevelControllers
  "Do you want to generate and compile the JointLevelControllers?" ON
  "FRAMEWORK_COMPILE_System" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_JointsGridPositionTrackingApplication
  "Compile joints-grid-position-tracking application?" ON
  "FRAMEWORK_COMPILE_YarpImplementation;FRAMEWORK_COMPILE_PYTHON_BINDINGS;FRAMEWORK_COMPILE_RobotInterface;FRAMEWORK_COMPILE_Math" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_MotorCurrentTrackingApplication
  "Compile motor-current-tracking application?" ON
  "FRAMEWORK_COMPILE_YarpImplementation;FRAMEWORK_COMPILE_PYTHON_BINDINGS;FRAMEWORK_COMPILE_RobotInterface" OFF)
