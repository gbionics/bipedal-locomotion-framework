/**
 * @file CameraModule.h
 * @authors Giulio Romualdi
 * @copyright Generative Bionics S.R.L. This software may be modified and distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_BINDINGS_ROBOT_INTERFACE_CAMERA_MODULE_H
#define BIPEDAL_LOCOMOTION_BINDINGS_ROBOT_INTERFACE_CAMERA_MODULE_H

#include <pybind11/pybind11.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace RobotInterface
{

void CreateCameraModule(pybind11::module& module);

} // namespace RobotInterface
} // namespace bindings
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_BINDINGS_ROBOT_INTERFACE_CAMERA_MODULE_H
