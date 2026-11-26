/**
 * @file IRobotControl.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <optional>
#include <string>

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

    py::enum_<InteractionMode>(module, "InteractionMode")
        .value("Stiff", InteractionMode::Stiff)
        .value("Compliant", InteractionMode::Compliant)
        .value("Unknown", InteractionMode::Unknown);

    py::class_<IRobotControl> iRobotControl(module, "IRobotControl");
    // Deprecated, kept for backward compatibility and it will be removed in a future release:
    // ControlMode used to be defined inside IRobotControl, with its values exported in the
    // IRobotControl scope (e.g. IRobotControl.Position). Accessing them raises a
    // DeprecationWarning.
    auto addDeprecatedAlias = [&iRobotControl](const std::string& name,
                                               py::object value,
                                               const std::string& replacement) {
        const std::string message = "IRobotControl." + name
                                    + " is deprecated and it will be removed in a future "
                                      "release. Use "
                                    + replacement + " instead.";
        iRobotControl.def_property_readonly_static(name.c_str(),
                                                   [value, message](py::object) {
                                                       if (PyErr_WarnEx(PyExc_DeprecationWarning,
                                                                        message.c_str(),
                                                                        1)
                                                           != 0)
                                                       {
                                                           throw py::error_already_set();
                                                       }
                                                       return value;
                                                   });
    };

    py::object controlMode = module.attr("ControlMode");
    addDeprecatedAlias("ControlMode", controlMode, "ControlMode");
    for (const auto& [name, value] : controlMode.attr("__members__").cast<py::dict>())
    {
        const auto valueName = name.cast<std::string>();
        addDeprecatedAlias(valueName,
                           py::reinterpret_borrow<py::object>(value),
                           "ControlMode." + valueName);
    }
}

} // namespace RobotInterface
} // namespace bindings
} // namespace BipedalLocomotion
