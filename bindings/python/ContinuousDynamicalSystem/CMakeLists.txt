# Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.
if(TARGET BipedalLocomotion::ContinuousDynamicalSystem)
  set(H_PREFIX include/BipedalLocomotion/bindings/ContinuousDynamicalSystem)

  add_bipedal_locomotion_python_module(
    NAME ContinuousDynamicalSystemBindings
    SOURCES src/MultiStateWeightProvider.cpp src/LinearTimeInvariantSystem.cpp src/FloatingBaseSystemVelocityKinematics.cpp src/FloatingBaseSystemAccelerationKinematics.cpp src/CentroidalDynamics.cpp src/ButterworthLowPassFilter.cpp src/FirstOrderSmoother.cpp src/Module.cpp
    HEADERS ${H_PREFIX}/DynamicalSystem.h ${H_PREFIX}/LinearTimeInvariantSystem.h ${H_PREFIX}/FloatingBaseSystemVelocityKinematics.h ${H_PREFIX}/FloatingBaseSystemAccelerationKinematics.h ${H_PREFIX}/Integrator.h
            ${H_PREFIX}/MultiStateWeightProvider.h ${H_PREFIX}/CentroidalDynamics.h ${H_PREFIX}/ButterworthLowPassFilter.h ${H_PREFIX}/FirstOrderSmoother.h
            ${H_PREFIX}/Module.h
    LINK_LIBRARIES BipedalLocomotion::ContinuousDynamicalSystem
    TESTS tests/test_linear_system.py tests/test_floating_base_system_kinematics.py tests/test_centroidal_dynamics.py tests/test_first_order_smoother.py
    )
endif()
