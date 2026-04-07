/**
 * @file SensorBridge.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/RobotInterface/ISensorBridge.h>
#include <BipedalLocomotion/System/Source.h>

#include <BipedalLocomotion/bindings/RobotInterface/ISensorBridge.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace RobotInterface
{

void CreateISensorBridge(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::RobotInterface;

    py::class_<SensorBridgeOptions>(module, "SensorBridgeOptions")
        .def(py::init())
        .def_readwrite("is_kinematics_enabled", &SensorBridgeOptions::isJointSensorsEnabled)
        .def_readwrite("is_linear_accelerometer_enabled",
                       &SensorBridgeOptions::isLinearAccelerometerEnabled)
        .def_readwrite("is_gyroscope_enabled", &SensorBridgeOptions::isGyroscopeEnabled)
        .def_readwrite("is_orientation_sensor_enabled",
                       &SensorBridgeOptions::isOrientationSensorEnabled)
        .def_readwrite("is_magnetometer_enabled", &SensorBridgeOptions::isMagnetometerEnabled)
        .def_readwrite("is_six_axis_force_torque_sensor_enabled",
                       &SensorBridgeOptions::isSixAxisForceTorqueSensorEnabled)
        .def_readwrite("is_three_axis_force_enabled",
                       &SensorBridgeOptions::isThreeAxisForceTorqueSensorEnabled)
        .def_readwrite("is_cartesian_wrench_enabled",
                       &SensorBridgeOptions::isCartesianWrenchEnabled)
        .def_readwrite("nr_joints", &SensorBridgeOptions::nrJoints);

    py::class_<SensorLists>(module, "SensorLists")
        .def(py::init())
        .def_readwrite("joints_list", &SensorLists::jointsList)
        .def_readwrite("linear_accelerometers_list", &SensorLists::linearAccelerometersList)
        .def_readwrite("gyroscopes_list", &SensorLists::gyroscopesList)
        .def_readwrite("orientation_sensors_list", &SensorLists::orientationSensorsList)
        .def_readwrite("magnetometers_list", &SensorLists::magnetometersList)
        .def_readwrite("six_axis_force_torque_sensors_list",
                       &SensorLists::sixAxisForceTorqueSensorsList)
        .def_readwrite("three_axis_force_torque_sensors_list",
                       &SensorLists::threeAxisForceTorqueSensorsList)
        .def_readwrite("cartesian_wrenches_list", &SensorLists::cartesianWrenchesList);

    py::class_<SensorBridgeMetaData>(module, "SensorBridgeMetaData")
        .def(py::init())
        .def_readwrite("sensors_list", &SensorBridgeMetaData::sensorsList)
        .def_readwrite("bridge_options", &SensorBridgeMetaData::bridgeOptions);

    py::class_<ISensorBridge>(module, "ISensorBridge");
}

} // namespace RobotInterface
} // namespace bindings
} // namespace BipedalLocomotion
