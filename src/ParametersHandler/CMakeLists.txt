# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

set(H_PREFIX include/BipedalLocomotion/ParametersHandler)

add_bipedal_locomotion_library(
  NAME                  ParametersHandler
  PUBLIC_HEADERS        ${H_PREFIX}/IParametersHandler.h ${H_PREFIX}/StdImplementation.h ${H_PREFIX}/StdImplementation.tpp
  SOURCES                src/StdImplementation.cpp
  PUBLIC_LINK_LIBRARIES BipedalLocomotion::GenericContainer BipedalLocomotion::TextLogging
  SUBDIRECTORIES        tests YarpImplementation TomlImplementation DinRailImplementation)
