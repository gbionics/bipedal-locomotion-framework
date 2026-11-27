# Copyright (C) Generative Bionics S.R.L. All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

# The camera is the dr_rgbd_fake device of dinrail, found next to the dinrail library.
add_bipedal_test(
  NAME DinrailCameraBridge
  SOURCES DinrailCameraBridgeTest.cpp
  LINKS BipedalLocomotion::CameraInterfaceDinrailImplementation BipedalLocomotion::ParametersHandler)
