/**
 * @file DinRailModule.cpp
 * @authors Giulio Romualdi
 * @copyright Generative Bionics S.R.L. This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/pybind11.h>

#include <BipedalLocomotion/bindings/RobotInterface/DinRailModule.h>
#include <BipedalLocomotion/bindings/RobotInterface/DinRailRobotControl.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace RobotInterface
{

void CreateDinRailModule(pybind11::module& module)
{
    CreateDinRailRobotControl(module);
}

} // namespace RobotInterface
} // namespace bindings
} // namespace BipedalLocomotion
