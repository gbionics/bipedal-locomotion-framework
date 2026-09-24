# Copyright (C) 2023 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

if(FRAMEWORK_COMPILE_ReducedModelControllers)

  set(H_PREFIX include/BipedalLocomotion/ReducedModelControllers)

  add_bipedal_locomotion_library(
    NAME                   ReducedModelControllers
    PUBLIC_HEADERS         ${H_PREFIX}/CentroidalMPC.h
    SOURCES                src/CentroidalMPC.cpp
    PUBLIC_LINK_LIBRARIES  Eigen3::Eigen BipedalLocomotion::ParametersHandler BipedalLocomotion::System BipedalLocomotion::Contacts
    PRIVATE_LINK_LIBRARIES casadi::casadi BipedalLocomotion::Math BipedalLocomotion::TextLogging BipedalLocomotion::CasadiConversions
    SUBDIRECTORIES         tests)

  # Used to run the solver single-threaded, which is faster for the size of the MPC problem.
  find_package(OpenMP QUIET COMPONENTS CXX)
  if(OpenMP_CXX_FOUND)
    target_link_libraries(ReducedModelControllers PRIVATE $<BUILD_INTERFACE:OpenMP::OpenMP_CXX>)
    target_compile_definitions(ReducedModelControllers PRIVATE BLF_CENTROIDAL_MPC_USE_OPENMP)
  endif()

endif()
