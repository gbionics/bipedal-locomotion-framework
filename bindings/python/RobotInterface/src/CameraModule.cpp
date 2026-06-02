/**
 * @file CameraModule.cpp
 * @authors Giulio Romualdi
 * @copyright Generative Bionics S.R.L. This software may be modified and distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/pybind11.h>

#include <BipedalLocomotion/bindings/RobotInterface/CameraModule.h>

#include <BipedalLocomotion/bindings/RobotInterface/CameraBridge.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace RobotInterface
{
void CreateCameraModule(pybind11::module& module)
{
    CreateICameraBridge(module);
    CreateYarpCameraBridge(module);
}
} // namespace RobotInterface
} // namespace bindings
} // namespace BipedalLocomotion
