/**
 * @file DinRailRobotControl.cpp
 * @authors Giulio Romualdi
 * @copyright Generative Bionics S.R.L. This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/RobotInterface/DinRailRobotControl.h>
#include <BipedalLocomotion/RobotInterface/YarpRobotControl.h>
#include <BipedalLocomotion/bindings/RobotInterface/DinRailRobotControl.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace RobotInterface
{

void CreateDinRailRobotControl(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::ParametersHandler;
    using namespace BipedalLocomotion::RobotInterface;

    py::class_<DinRailRobotControl, YarpRobotControl>(module, "DinRailRobotControl")
        .def(py::init())
        .def(
            "initialize",
            [](DinRailRobotControl& impl, std::shared_ptr<IParametersHandler> handler) -> bool {
                return impl.initialize(handler);
            },
            py::arg("handler"))
        .def("set_driver", &DinRailRobotControl::setDriver, py::arg("driver"))
        .def("set_impedance_set_points", //
             &DinRailRobotControl::setImpedanceSetPoints,
             py::arg("position"),
             py::arg("velocity"),
             py::arg("torque"),
             py::arg("stiffness"),
             py::arg("damping"));
}

} // namespace RobotInterface
} // namespace bindings
} // namespace BipedalLocomotion
