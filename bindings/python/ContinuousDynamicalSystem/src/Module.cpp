/**
 * @file Module.cpp
 * @authors Giulio Romualdi
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/pybind11.h>

#include <BipedalLocomotion/bindings/ContinuousDynamicalSystem/ButterworthLowPassFilter.h>
#include <BipedalLocomotion/bindings/ContinuousDynamicalSystem/CentroidalDynamics.h>
#include <BipedalLocomotion/bindings/ContinuousDynamicalSystem/FirstOrderSmoother.h>
#include <BipedalLocomotion/bindings/ContinuousDynamicalSystem/FloatingBaseSystemAccelerationKinematics.h>
#include <BipedalLocomotion/bindings/ContinuousDynamicalSystem/FloatingBaseSystemVelocityKinematics.h>
#include <BipedalLocomotion/bindings/ContinuousDynamicalSystem/LinearTimeInvariantSystem.h>
#include <BipedalLocomotion/bindings/ContinuousDynamicalSystem/Module.h>
#include <BipedalLocomotion/bindings/ContinuousDynamicalSystem/MultiStateWeightProvider.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace ContinuousDynamicalSystem
{
void CreateModule(pybind11::module& module)
{
    module.doc() = "continuous_dynamical_system module contains the bindings for "
                   "BipedalLocomotion::ContinuousDynamicalSystem";

    CreateLinearTimeInvariantSystem(module);
    CreateFloatingBaseSystemVelocityKinematics(module);
    CreateFloatingBaseSystemAccelerationKinematics(module);
    CreateFirstOrderSmoother(module);
    CreateMultiStateWeightProvider(module);
    CreateCentroidalDynamics(module);
    CreateButterworthLowPassFilter(module);
}
} // namespace ContinuousDynamicalSystem
} // namespace bindings
} // namespace BipedalLocomotion
