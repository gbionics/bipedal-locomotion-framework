# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

if(FRAMEWORK_COMPILE_YarpRobotLoggerDevice)
  option(USE_SYSTEM_tiny-process-library "Use system tiny-process-library" OFF)

  if(USE_SYSTEM_tiny-process-library)
    find_package(tiny-process-library REQUIRED)
  else()
    include(FetchContent)
    FetchContent_Declare(tiny_process_library
      GIT_REPOSITORY https://gitlab.com/eidheim/tiny-process-library.git
      GIT_TAG 8bbb5a211c5c9df8ee69301da9d22fb977b27dc1)

    if(NOT tiny_process_library_POPULATED)
      FetchContent_Populate(tiny_process_library)

      set(BUILD_SHARED_LIBS_OLD ${BUILD_SHARED_LIBS})
      set(BUILD_SHARED_LIBS OFF CACHE INTERNAL "Build libraries as shared as opposed to static")

      set(BUILD_TESTING_OLD ${BUILD_TESTING})
      set(BUILD_TESTING OFF CACHE INTERNAL "Create tests using CMake")

      # Bring the populated content into the build
      add_subdirectory(${tiny_process_library_SOURCE_DIR} ${tiny_process_library_BINARY_DIR})

      # Restore the old value of the parameter
      set(BUILD_TESTING ${BUILD_TESTING_OLD} CACHE BOOL
        "Create tests using CMake" FORCE)

      set(BUILD_SHARED_LIBS ${BUILD_SHARED_LIBS_OLD} CACHE BOOL
        "Build libraries as shared as opposed to static" FORCE)
    endif()
  endif()

  add_bipedal_locomotion_yarp_thrift(
      NAME YarpRobotLoggerDeviceCommands
      THRIFT thrifts/YarpRobotLoggerDeviceCommands.thrift
      INSTALLATION_FOLDER YarpRobotLoggerDevice)

  # Warning: the <package> option of yarp_configure_plugins_installation should be different from the plugin name
  add_bipedal_yarp_device(
    NAME YarpRobotLoggerDevice
    TYPE BipedalLocomotion::YarpRobotLoggerDevice
    SOURCES src/YarpRobotLoggerDevice.cpp src/YarpTextLoggingUtilities.cpp src/MessageConversionUtilities.cpp
    PUBLIC_HEADERS include/BipedalLocomotion/YarpRobotLoggerDevice.h include/BipedalLocomotion/YarpTextLoggingUtilities.h include/BipedalLocomotion/MessageConversionUtilities.h
    PRIVATE_LINK_LIBRARIES
      Eigen3::Eigen
      YARP::YARP_os
      YARP::YARP_dev
      YARP::YARP_profiler
      robometry::robometry
      trintrin::msgs
      BipedalLocomotion::TextLogging
      BipedalLocomotion::System
      BipedalLocomotion::YarpUtilities
      BipedalLocomotion::VectorsCollection
      BipedalLocomotion::ParametersHandlerYarpImplementation
      BipedalLocomotion::RobotInterfaceYarpImplementation
      BipedalLocomotion::PerceptionInterfaceYarpImplementation
      BipedalLocomotion::TextLoggingYarpImplementation
      BipedalLocomotion::SystemYarpImplementation
      BipedalLocomotion::YarpRobotLoggerDeviceCommands
      tiny-process-library::tiny-process-library
    CONFIGURE_PACKAGE_NAME yarp_robot_logger_device)

  install(FILES scripts/blf-logger-with-audio.sh
    PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
    DESTINATION "${CMAKE_INSTALL_BINDIR}")

  # This test only works on Linux, see https://github.com/ami-iit/bipedal-locomotion-framework/issues/873
  if(NOT (WIN32 OR APPLE))
    add_subdirectory(tests)
  endif()
endif()

