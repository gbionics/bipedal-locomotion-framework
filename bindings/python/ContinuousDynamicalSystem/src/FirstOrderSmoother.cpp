/**
 * @file FirstOrderSmoother.cpp
 * @authors Giulio Romualdi
 * @copyright 2024 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/ContinuousDynamicalSystem/FirstOrderSmoother.h>
#include <BipedalLocomotion/System/Advanceable.h>

#include <BipedalLocomotion/bindings/ContinuousDynamicalSystem/FirstOrderSmoother.h>
#include <BipedalLocomotion/bindings/System/Advanceable.h>

#include <pybind11/eigen.h>
#include <pybind11/stl.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace ContinuousDynamicalSystem
{

void CreateFirstOrderSmoother(pybind11::module& module)
{
    using namespace BipedalLocomotion::ContinuousDynamicalSystem;
    namespace py = ::pybind11;

    BipedalLocomotion::bindings::System::CreateAdvanceable<Eigen::VectorXd, //
                                                           Eigen::VectorXd>(module,
                                                                            "FirstOrderSmoother");

    py::class_<FirstOrderSmoother, //
               ::BipedalLocomotion::System::Advanceable<Eigen::VectorXd, //
                                                        Eigen::VectorXd>>(module,
                                                                          "FirstOrderSmoother")
        .def(py::init())
        .def("reset", &FirstOrderSmoother::reset, py::arg("initial_state"))
        .def("set_settling_time",
             &FirstOrderSmoother::setSettlingTime,
             py::arg("settling_time"))
        .def("get_settling_time", &FirstOrderSmoother::getSettlingTime);
}

} // namespace ContinuousDynamicalSystem
} // namespace bindings
} // namespace BipedalLocomotion
