/**
 * @file DummySensorBridge.h
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_ROBOT_INTERFACE_DUMMY_SENSOR_BRIDGE_H
#define BIPEDAL_LOCOMOTION_ROBOT_INTERFACE_DUMMY_SENSOR_BRIDGE_H

#include <BipedalLocomotion/RobotInterface/ISensorBridge.h>
#include <vector>
#include <string>
#include <iostream>

namespace BipedalLocomotion
{
namespace RobotInterface
{
class DummySensorBridge : public ISensorBridge
{
public:
    bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler)
    {
        if (!populateSensorBridgeOptionsFromConfig(handler, m_options))
        {
            std::cerr << "[DummySensorBridge::initialize] could not configure sensor bridge options" << std::endl;
            return false;
        }

        if (!populateSensorListsFromConfig(handler, m_options, m_lists))
        {
            std::cerr << "[DummySensorBridge::initialize] could not configure sensor lists" << std::endl;
            return false;
        }

        m_initialized = true;
        return true;
    };

    virtual bool getJointsList(std::vector<std::string>& jointsList) override
    {
        jointsList = std::vector<std::string>{""};
        return true;
    };

    virtual bool getLinearAccelerometersList(std::vector<std::string>& linearAccelerometersList) override
    {
        linearAccelerometersList = std::vector<std::string>{""};
        return true;
    };

    virtual bool getGyroscopesList(std::vector<std::string>& gyroscopesList) override
    {
        gyroscopesList = std::vector<std::string>{""};
        return true;
    };

    virtual bool getOrientationSensorsList(std::vector<std::string>& orientationSensorsList) override
    {
        orientationSensorsList = std::vector<std::string>{""};
        return true;
    };

    virtual bool getMagnetometersList(std::vector<std::string>& magnetometersList) override
    {
        magnetometersList = std::vector<std::string>{""};
        return true;
    };

    virtual bool getSixAxisForceTorqueSensorsList(std::vector<std::string>& sixAxisForceTorqueSensorsList) override
    {
        sixAxisForceTorqueSensorsList = std::vector<std::string>{""};
        return true;
    };

    virtual bool getThreeAxisForceTorqueSensorsList(std::vector<std::string>& threeAxisForceTorqueSensorsList) override
    {
        threeAxisForceTorqueSensorsList = std::vector<std::string>{""};
        return true;
    };

    virtual bool getCartesianWrenchesList(std::vector<std::string>& cartesianWrenchesList) override
    {
        cartesianWrenchesList = std::vector<std::string>{""};
        return true;
    };


    virtual bool getJointPosition(const std::string& jointName,
                                  double& jointPosition,
                                  OptionalDoubleRef receiveTimeInSeconds = {}) override { return true; };
    virtual bool getJointPositions(Eigen::Ref<Eigen::VectorXd> jointPositions,
                                   OptionalDoubleRef receiveTimeInSeconds = {}) override { return true; };
    virtual bool getJointVelocity(const std::string& jointName,
                                  double& jointVelocity,
                                  OptionalDoubleRef receiveTimeInSeconds = {}) override { return true; };
    virtual bool getJointVelocities(Eigen::Ref<Eigen::VectorXd> jointVelocties,
                                    OptionalDoubleRef receiveTimeInSeconds = {}) override { return true; };
    virtual bool getLinearAccelerometerMeasurement(const std::string& accName,
                                                   Eigen::Ref<Eigen::Vector3d> accMeasurement,
                                                   OptionalDoubleRef receiveTimeInSeconds = {}) override
    {
        if (!m_initialized)
        {
            std::cerr << "[DummySensorBridge::getLinearAccelerometerMeasurement] Please intialize te SensorBridge before calling this method."
                << std::endl;
            return false;
        }

        if (!m_options.isLinearAccelerometerEnabled)
        {
            std::cerr << "[DummySensorBridge::getLinearAccelerometerMeasurement] Stream not enabled."
                << std::endl;
            return false;
        }

        // get the sensor from the attached streams and pass the measurement

        // pass dummy measurements now
        accMeasurement << 0, 0, -9.8;
        if (receiveTimeInSeconds)
            receiveTimeInSeconds.value().get() = 10.0;

        return true;
    };

    virtual bool getGyroscopeMeasure(const std::string& gyroName,
                                     Eigen::Ref<Eigen::Vector3d> gyroMeasurement,
                                     OptionalDoubleRef receiveTimeInSeconds = {}) override { return true; };
    virtual bool getOrientationSensorMeasurement(const std::string& rpyName,
                                                 Eigen::Ref<Eigen::Vector3d> rpyMeasurement,
                                                 OptionalDoubleRef receiveTimeInSeconds = {}) override { return true; };
    virtual bool getMagnetometerMeasurement(const std::string& magName,
                                            Eigen::Ref<Eigen::Vector3d> magMeasurement,
                                            OptionalDoubleRef receiveTimeInSeconds = {}) override { return true; };
    virtual bool getSixAxisForceTorqueMeasurement(const std::string& ftName,
                                                  Eigen::Ref<Vector6d> ftMeasurement,
                                                  OptionalDoubleRef receiveTimeInSeconds = {}) override { return true; };
    virtual bool getThreeAxisForceTorqueMeasurement(const std::string& ftName,
                                                    Eigen::Ref<Eigen::Vector3d> ftMeasurement,
                                                    OptionalDoubleRef receiveTimeInSeconds = {}) override { return true; };
    virtual bool getCartesianWrench(const std::string& cartesianWrenchName,
                                    Eigen::Ref<Vector6d> cartesianWrenchMeasurement,
                                    OptionalDoubleRef receiveTimeInSeconds = {}) override { return true; };

protected:
    bool populateSensorBridgeOptionsFromConfig(std::weak_ptr<const ParametersHandler::IParametersHandler> handler,
                                                      SensorBridgeOptions& sensorBridgeOptions) override
    {
        auto handle = handler.lock();
        if (handle == nullptr)
        {
            std::cerr << "[DummySensorBridge::initialize] The parameter handler has expired. Please check its scope."
            << std::endl;
            return false;
        }

        auto optionsHandler = handle->getGroup("Options");
        auto options = optionsHandler.lock();
        if (options == nullptr)
        {
            std::cerr << "[DummySensorBridge::initialize] Could not load required group \"Options\"."
            << std::endl;
            return false;
        }

        if (!options->getParameter("streamLinearAccelerometerMeasurements", sensorBridgeOptions.isLinearAccelerometerEnabled))
        {
            std::cerr << "[DummySensorBridge::initialize] The parameter handler could not find \" streamLinearAccelerometerMeasurements \" in the configuration file."
            << std::endl;
            return false;
        }
        return true;
    };

    bool populateSensorListsFromConfig(std::weak_ptr<const ParametersHandler::IParametersHandler> handler,
                                               const SensorBridgeOptions& sensorBridgeOptions,
                                               SensorLists& sensorLists) override
    {
        auto handle = handler.lock();
        if (handle == nullptr)
        {
            std::cerr << "[DummySensorBridge::initialize] The parameter handler has expired. Please check its scope."
            << std::endl;
            return false;
        }

        auto sourceHandler = handle->getGroup("Sources");
        auto source = sourceHandler.lock();
        if (source == nullptr)
        {
            std::cerr << "[DummySensorBridge::initialize] Could not load required group \"Sources\"."
            << std::endl;
            return false;
        }

        if (sensorBridgeOptions.isLinearAccelerometerEnabled)
        {
            if (!source->getParameter("LinearAccelerometers", sensorLists.linearAccelerometersList))
            {
                std::cerr << "[DummySensorBridge::initialize] The parameter handler could not find \" LinearAccelerometers \" in the configuration file."
                << std::endl;
                return false;
            }
        }

        return true;
    };

private:
    bool m_initialized{false};
    SensorBridgeOptions m_options;
    SensorLists m_lists;
};

} // namespace BipedalLocomotion
} // namespace RobotInterface

#endif // BIPEDAL_LOCOMOTION_ROBOT_INTERFACE_DUMMY_SENSOR_BRIDGE_H
