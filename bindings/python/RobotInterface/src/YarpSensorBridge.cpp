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
#include <BipedalLocomotion/RobotInterface/YarpHelper.h>
#include <BipedalLocomotion/RobotInterface/YarpSensorBridge.h>
#include <BipedalLocomotion/System/Source.h>

#include <BipedalLocomotion/bindings/RobotInterface/YarpSensorBridge.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace RobotInterface
{

void CreateYarpSensorBridge(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::RobotInterface;
    using namespace BipedalLocomotion::System;
    using namespace BipedalLocomotion::ParametersHandler;

    py::class_<Source<SensorBridgeMetaData>>(module, "SensorBridgeMetaDataSource");

    py::class_<YarpSensorBridge, ISensorBridge, Source<SensorBridgeMetaData>>(module,
                                                                              "YarpSensorBridge")
        .def(py::init())
        .def(
            "initialize",
            [](YarpSensorBridge& impl, std::shared_ptr<const IParametersHandler> handler) -> bool {
                return impl.initialize(handler);
            },
            py::arg("handler"))
        .def(
            "set_drivers_list",
            [](YarpSensorBridge& impl, std::vector<PolyDriverDescriptor> polydrivers) -> bool {
                yarp::dev::PolyDriverList list;
                for (const auto& polydriver : polydrivers)
                    list.push(polydriver.poly.get(), polydriver.key.c_str());

                return impl.setDriversList(list);
            },
            py::arg("polydrivers"))
        .def("advance", &YarpSensorBridge::advance)
        .def("is_output_valid", &YarpSensorBridge::isOutputValid)
        .def("get_failed_sensor_reads", &YarpSensorBridge::getFailedSensorReads)
        .def("get_joints_list",
             [](YarpSensorBridge& impl) {
                 std::vector<std::string> list;
                 bool ok = impl.getJointsList(list);
                 return std::make_tuple(ok, list);
             })
        .def("get_gyroscopes_list",
             [](YarpSensorBridge& impl) {
                 std::vector<std::string> list;
                 bool ok = impl.getGyroscopesList(list);
                 return std::make_tuple(ok, list);
             })
        .def("get_orientation_sensors_list",
             [](YarpSensorBridge& impl) {
                 std::vector<std::string> list;
                 bool ok = impl.getOrientationSensorsList(list);
                 return std::make_tuple(ok, list);
             })
        .def("get_six_axis_force_torque_sensors_list",
             [](YarpSensorBridge& impl) {
                 std::vector<std::string> list;
                 bool ok = impl.getSixAxisForceTorqueSensorsList(list);
                 return std::make_tuple(ok, list);
             })
        .def("get_cartesian_wrenches_list",
             [](YarpSensorBridge& impl) {
                 std::vector<std::string> list;
                 bool ok = impl.getCartesianWrenchesList(list);
                 return std::make_tuple(ok, list);
             })
        .def(
            "get_joint_position",
            [](YarpSensorBridge& impl, const std::string& jointName) {
                double joint, receiveTimeInSeconds;
                bool ok = impl.getJointPosition(jointName, joint, receiveTimeInSeconds);
                return std::make_tuple(ok, joint, receiveTimeInSeconds);
            },
            py::arg("joint_name"))
        .def("get_joint_positions",
             [](YarpSensorBridge& impl) {
                 Eigen::VectorXd joints(impl.getOutput().bridgeOptions.nrJoints);
                 double receiveTimeInSeconds;
                 bool ok = impl.getJointPositions(joints, receiveTimeInSeconds);
                 return std::make_tuple(ok, joints, receiveTimeInSeconds);
             })
        .def(
            "get_joint_velocity",
            [](YarpSensorBridge& impl, const std::string& name) {
                double joint, receiveTimeInSeconds;
                bool ok = impl.getJointVelocity(name, joint, receiveTimeInSeconds);
                return std::make_tuple(ok, joint, receiveTimeInSeconds);
            },
            py::arg("joint_name"))
        .def("get_joint_velocities",
             [](YarpSensorBridge& impl) {
                 Eigen::VectorXd joints(impl.getOutput().bridgeOptions.nrJoints);
                 double receiveTimeInSeconds;
                 bool ok = impl.getJointVelocities(joints, receiveTimeInSeconds);
                 return std::make_tuple(ok, joints, receiveTimeInSeconds);
             })
        .def(
            "get_joint_acceleration",
            [](YarpSensorBridge& impl, const std::string& name) {
                double joint, receiveTimeInSeconds;
                bool ok = impl.getJointAcceleration(name, joint, receiveTimeInSeconds);
                return std::make_tuple(ok, joint, receiveTimeInSeconds);
            },
            py::arg("joint_name"))
        .def("get_joint_accelerations",
             [](YarpSensorBridge& impl) {
                 Eigen::VectorXd joints(impl.getOutput().bridgeOptions.nrJoints);
                 double receiveTimeInSeconds;
                 bool ok = impl.getJointAccelerations(joints, receiveTimeInSeconds);
                 return std::make_tuple(ok, joints, receiveTimeInSeconds);
             })
        .def(
            "get_linear_accelerometer_measurement",
            [](YarpSensorBridge& impl, const std::string& name) {
                Eigen::Vector3d measurement;
                double receiveTimeInSeconds;
                bool ok = impl.getLinearAccelerometerMeasurement(name,
                                                                 measurement,
                                                                 receiveTimeInSeconds);
                return std::make_tuple(ok, measurement, receiveTimeInSeconds);
            },
            py::arg("sensor_name"))
        .def(
            "get_gyroscope_measurement",
            [](YarpSensorBridge& impl, const std::string& name) {
                Eigen::Vector3d measurement;
                double receiveTimeInSeconds;
                bool ok = impl.getGyroscopeMeasure(name, measurement, receiveTimeInSeconds);
                return std::make_tuple(ok, measurement, receiveTimeInSeconds);
            },
            py::arg("sensor_name"))
        .def(
            "get_orientation_sensor_measurement",
            [](YarpSensorBridge& impl, const std::string& name) {
                Eigen::Vector3d measurement;
                double receiveTimeInSeconds;
                bool ok
                    = impl.getOrientationSensorMeasurement(name, measurement, receiveTimeInSeconds);
                return std::make_tuple(ok, measurement, receiveTimeInSeconds);
            },
            py::arg("sensor_name"))
        .def(
            "get_magnetometer_measurement",
            [](YarpSensorBridge& impl, const std::string& name) {
                Eigen::Vector3d measurement;
                double receiveTimeInSeconds;
                bool ok = impl.getMagnetometerMeasurement(name, measurement, receiveTimeInSeconds);
                return std::make_tuple(ok, measurement, receiveTimeInSeconds);
            },
            py::arg("sensor_name"))
        .def(
            "get_six_axis_force_torque_measurement",
            [](YarpSensorBridge& impl, const std::string& name) {
                Eigen::Matrix<double, 6, 1> measurement;
                double receiveTimeInSeconds;
                bool ok = impl.getSixAxisForceTorqueMeasurement(name,
                                                                measurement,
                                                                receiveTimeInSeconds);
                return std::make_tuple(ok, measurement, receiveTimeInSeconds);
            },
            py::arg("sensor_name"))
        .def(
            "get_three_axis_force_torque_measurement",
            [](YarpSensorBridge& impl, const std::string& name) {
                Eigen::Vector3d measurement;
                double receiveTimeInSeconds;
                bool ok = impl.getThreeAxisForceTorqueMeasurement(name,
                                                                  measurement,
                                                                  receiveTimeInSeconds);
                return std::make_tuple(ok, measurement, receiveTimeInSeconds);
            },
            py::arg("sensor_name"))
        .def(
            "get_cartesian_wrench",
            [](YarpSensorBridge& impl, const std::string& name) {
                Eigen::Matrix<double, 6, 1> measurement;
                double receiveTimeInSeconds;
                bool ok = impl.getCartesianWrench(name, measurement, receiveTimeInSeconds);
                return std::make_tuple(ok, measurement, receiveTimeInSeconds);
            },
            py::arg("wrench_name"))
        .def(
            "get_motor_current",
            [](YarpSensorBridge& impl, const std::string& jointName) {
                double joint, receiveTimeInSeconds;
                bool ok = impl.getMotorCurrent(jointName, joint, receiveTimeInSeconds);
                return std::make_tuple(ok, joint, receiveTimeInSeconds);
            },
            py::arg("motor_name"))
        .def("get_motor_currents",
             [](YarpSensorBridge& impl) {
                 Eigen::VectorXd joints(impl.getOutput().bridgeOptions.nrJoints);
                 double receiveTimeInSeconds;
                 bool ok = impl.getMotorCurrents(joints, receiveTimeInSeconds);
                 return std::make_tuple(ok, joints, receiveTimeInSeconds);
             })
        .def(
            "get_joint_torque",
            [](YarpSensorBridge& impl, const std::string& jointName) {
                double joint, receiveTimeInSeconds;
                bool ok = impl.getJointTorque(jointName, joint, receiveTimeInSeconds);
                return std::make_tuple(ok, joint, receiveTimeInSeconds);
            },
            py::arg("motor_name"))
        .def("get_joint_torques", [](YarpSensorBridge& impl) {
            Eigen::VectorXd joints(impl.getOutput().bridgeOptions.nrJoints);
            double receiveTimeInSeconds;
            bool ok = impl.getJointTorques(joints, receiveTimeInSeconds);
            return std::make_tuple(ok, joints, receiveTimeInSeconds);
        });
}

} // namespace RobotInterface
} // namespace bindings
} // namespace BipedalLocomotion
