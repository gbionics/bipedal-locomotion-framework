# Copyright (C) Generative Bionics S.R.L. All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

if(FRAMEWORK_COMPILE_DinRailImplementation)

  add_bipedal_locomotion_library(
    NAME                  ParametersHandlerDinRailImplementation
    SOURCES               src/DinRailImplementation.cpp
    PUBLIC_HEADERS        include/BipedalLocomotion/ParametersHandler/DinRailImplementation.h
    PUBLIC_LINK_LIBRARIES BipedalLocomotion::ParametersHandler dinrail::dinrail
    INSTALLATION_FOLDER   ParametersHandler)

endif()
