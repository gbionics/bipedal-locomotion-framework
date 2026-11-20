/**
 * @file IRobotControl.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <optional>

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/RobotInterface/IRobotControl.h>
#include <BipedalLocomotion/System/Advanceable.h>

#include <BipedalLocomotion/bindings/RobotInterface/IRobotControl.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace RobotInterface
{

void CreateIRobotControl(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::RobotInterface;

    py::enum_<ControlMode>(module, "ControlMode")
        .value("Position", ControlMode::Position)
        .value("PositionDirect", ControlMode::PositionDirect)
        .value("Velocity", ControlMode::Velocity)
        .value("Torque", ControlMode::Torque)
        .value("PWM", ControlMode::PWM)
        .value("Current", ControlMode::Current)
        .value("Idle", ControlMode::Idle)
        .value("ForceIdle", ControlMode::ForceIdle)
        .value("HardwareFault", ControlMode::HardwareFault)
        .value("Unknown", ControlMode::Unknown);

    py::class_<IRobotControl> iRobotControl(module, "IRobotControl");
    // Kept for backward compatibility: ControlMode used to be defined inside IRobotControl, with
    // its values exported in the IRobotControl scope (e.g. IRobotControl.Position).
    py::object controlMode = module.attr("ControlMode");
    iRobotControl.attr("ControlMode") = controlMode;
    for (const auto& [name, value] : controlMode.attr("__members__").cast<py::dict>())
    {
        iRobotControl.attr(name) = value;
    }
}

} // namespace RobotInterface
} // namespace bindings
} // namespace BipedalLocomotion
