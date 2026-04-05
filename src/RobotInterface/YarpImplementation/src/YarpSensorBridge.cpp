/**
 * @file YarpSensorBridge.cpp
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/RobotInterface/YarpSensorBridgeImpl.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <yarp/eigen/Eigen.h>

using namespace BipedalLocomotion::RobotInterface;
using namespace BipedalLocomotion::GenericContainer;
using namespace BipedalLocomotion::ParametersHandler;

YarpSensorBridge::YarpSensorBridge()
    : m_pimpl(std::make_unique<Impl>())
{
}

YarpSensorBridge::~YarpSensorBridge() = default;

bool YarpSensorBridge::initialize(std::weak_ptr<const IParametersHandler> handler)
{
    constexpr auto logPrefix = "[YarpSensorBridge::initialize]";

    auto ptr = handler.lock();
    if (ptr == nullptr)
    {
        log()->error("{} The handler is not pointing to an already initialized memory.", logPrefix);
        return false;
    }

    if (!ptr->getParameter("check_for_nan", m_pimpl->checkForNAN))
    {
        log()->error("{} Unable to get check_for_nan.", logPrefix);
        return false;
    }

    if (!ptr->getParameter("stream_joint_accelerations", m_pimpl->streamJointAccelerations))
    {
        log()->info("{} Unable to get stream_joint_accelerations. Set to true by default",
                    logPrefix);
    }
    if (!m_pimpl->streamJointAccelerations)
    {
        log()->info("{} Joint accelerations will not be streamed.", logPrefix);
    }

    if (!ptr->getParameter("stream_motor_temperature",
                           m_pimpl->metaData.bridgeOptions.isMotorTemperatureSensorEnabled))
    {
        log()->info("{} Unable to get stream_motor_temperature. Set to false by default",
                    logPrefix);
    }
    if (!m_pimpl->metaData.bridgeOptions.isMotorTemperatureSensorEnabled)
    {
        log()->info("{} Motor temperature sensors will not be streamed.", logPrefix);
    }

    bool ret{true};
    ret = m_pimpl->subConfigLoader("stream_joint_states",
                                   "RemoteControlBoardRemapper",
                                   &YarpSensorBridge::Impl::configureRemoteControlBoardRemapper,
                                   handler,
                                   m_pimpl->metaData,
                                   m_pimpl->metaData.bridgeOptions.isJointSensorsEnabled);
    if (!ret)
    {
        log()->info("{} Skipping the configuration of RemoteControlBoardRemapper. YarpSensorBridge "
                    "will not stream relevant measures.",
                    logPrefix);
    }

    ret = m_pimpl->subConfigLoader("stream_pids",
                                   "PIDs",
                                   &YarpSensorBridge::Impl::configureRemoteControlBoardRemapper,
                                   handler,
                                   m_pimpl->metaData,
                                   m_pimpl->metaData.bridgeOptions.isPIDsEnabled);
    if (!ret)
    {
        log()->info("{} Skipping the configuration of configureRemoteControlBoardRemapper. "
                    "YarpSensorBridge "
                    "will not stream relevant measures.",
                    logPrefix);
    }

    ret = m_pimpl->subConfigLoader("stream_motor_states",
                                   "Motors",
                                   &YarpSensorBridge::Impl::configureRemoteControlBoardRemapper,
                                   handler,
                                   m_pimpl->metaData,
                                   m_pimpl->metaData.bridgeOptions.isMotorSensorsEnabled);
    if (!ret)
    {
        log()->info("{} Skipping the configuration of configureRemoteControlBoardRemapper. "
                    "YarpSensorBridge "
                    "will not stream relevant measures.",
                    logPrefix);
    }

    ret = m_pimpl->subConfigLoader("stream_motor_PWM",
                                   "MotorPWM",
                                   &YarpSensorBridge::Impl::configureRemoteControlBoardRemapper,
                                   handler,
                                   m_pimpl->metaData,
                                   m_pimpl->metaData.bridgeOptions.isPWMControlEnabled);
    if (!ret)
    {
        log()->info("{} Skipping the configuration of configureRemoteControlBoardRemapper. "
                    "YarpSensorBridge "
                    "will not stream relevant measures.",
                    logPrefix);
    }

    bool useInertialSensors{false};
    ret = m_pimpl->subConfigLoader("stream_inertials",
                                   "InertialSensors",
                                   &YarpSensorBridge::Impl::configureInertialSensors,
                                   handler,
                                   m_pimpl->metaData,
                                   useInertialSensors);
    if (!ret)
    {
        log()->info("{} Skipping the configuration of InertialSensors. YarpSensorBridge will not "
                    "stream relevant measures.",
                    logPrefix);
    }

    ret = m_pimpl
              ->subConfigLoader("stream_forcetorque_sensors", //
                                "SixAxisForceTorqueSensors",
                                &YarpSensorBridge::Impl::configureSixAxisForceTorqueSensors,
                                handler,
                                m_pimpl->metaData,
                                m_pimpl->metaData.bridgeOptions.isSixAxisForceTorqueSensorEnabled);
    if (!ret)
    {
        log()->info("{} Skipping the configuration of SixAxisForceTorqueSensors. YarpSensorBridge "
                    "will not stream relevant measures.",
                    logPrefix);
    }

    ret = m_pimpl->subConfigLoader("stream_cartesian_wrenches",
                                   "CartesianWrenches",
                                   &YarpSensorBridge::Impl::configureCartesianWrenches,
                                   handler,
                                   m_pimpl->metaData,
                                   m_pimpl->metaData.bridgeOptions.isCartesianWrenchEnabled);
    if (!ret)
    {
        log()->info("{} Skipping the configuration of CartesianWrenches. YarpSensorBridge "
                    "will not stream relevant measures.",
                    logPrefix);
    }

    ret = m_pimpl->subConfigLoader("stream_temperatures",
                                   "TemperatureSensors",
                                   &YarpSensorBridge::Impl::configureTemperatureSensors,
                                   handler,
                                   m_pimpl->metaData,
                                   m_pimpl->metaData.bridgeOptions.isTemperatureSensorEnabled);
    if (!ret)
    {
        log()->info("{} Skipping the configuration of TemperatureSensors. YarpSensorBridge "
                    "will not stream relevant measures.",
                    logPrefix);
    }

    m_pimpl->bridgeInitialized = true;
    return true;
}

bool YarpSensorBridge::setDriversList(const yarp::dev::PolyDriverList& deviceDriversList)
{
    constexpr auto logPrefix = "[YarpSensorBridge::setDriversList]";

    if (!m_pimpl->bridgeInitialized)
    {
        log()->error("{} Please initialize YarpSensorBridge before calling setDriversList(...).",
                     logPrefix);
        return false;
    }

    bool ret{true};
    ret = ret && m_pimpl->attachRemappedRemoteControlBoard(deviceDriversList);
    ret = ret && m_pimpl->attachAllInertials(deviceDriversList);
    ret = ret && m_pimpl->attachAllSixAxisForceTorqueSensors(deviceDriversList);
    ret = ret && m_pimpl->attachCartesianWrenchInterface(deviceDriversList);
    ret = ret && m_pimpl->attachAllTemperatureSensors(deviceDriversList);

    if (!ret)
    {
        log()->error("{} Failed to attach to one or more device drivers.", logPrefix);
        return false;
    }
    m_pimpl->driversAttached = true;
    return true;
}

bool YarpSensorBridge::advance()
{
    constexpr auto logPrefix = "[YarpSensorBridge::advance]";
    if (!m_pimpl->checkValid(logPrefix))
    {
        log()->error("{} Please initialize and set drivers list before running advance().",
                     logPrefix);
        return false;
    }

    return m_pimpl->readAllSensors(m_pimpl->failedSensorReads);
}

bool YarpSensorBridge::isOutputValid() const
{
    return m_pimpl->checkValid("[YarpSensorBridge::isValid]");
}

std::vector<std::string> YarpSensorBridge::getFailedSensorReads() const
{
    return m_pimpl->failedSensorReads;
}

const SensorBridgeMetaData& YarpSensorBridge::getOutput() const
{
    return m_pimpl->metaData;
}

bool YarpSensorBridge::getJointsList(std::vector<std::string>& jointsList)
{
    if (!m_pimpl->checkValid("[YarpSensorBridge::getJointsList]"))
    {
        return false;
    }
    jointsList = m_pimpl->metaData.sensorsList.jointsList;
    return true;
}

bool YarpSensorBridge::getLinearAccelerometersList(
    std::vector<std::string>& linearAccelerometersList)
{
    if (!m_pimpl->checkValid("[YarpSensorBridge::getLinearAccelerometersList]"))
    {
        return false;
    }
    linearAccelerometersList = m_pimpl->metaData.sensorsList.linearAccelerometersList;
    return true;
}

bool YarpSensorBridge::getGyroscopesList(std::vector<std::string>& gyroscopesList)
{
    if (!m_pimpl->checkValid("[YarpSensorBridge::getGyroscopesList]"))
    {
        return false;
    }
    gyroscopesList = m_pimpl->metaData.sensorsList.gyroscopesList;
    return true;
}

bool YarpSensorBridge::getOrientationSensorsList(std::vector<std::string>& orientationSensorsList)
{
    if (!m_pimpl->checkValid("[YarpSensorBridge::getOrientationSensorsList]"))
    {
        return false;
    }
    orientationSensorsList = m_pimpl->metaData.sensorsList.orientationSensorsList;
    return true;
}

bool YarpSensorBridge::getMagnetometersList(std::vector<std::string>& magnetometersList)
{
    if (!m_pimpl->checkValid("[YarpSensorBridge::getMagnetometersList]"))
    {
        return false;
    }
    magnetometersList = m_pimpl->metaData.sensorsList.magnetometersList;
    return true;
}

bool YarpSensorBridge::getSixAxisForceTorqueSensorsList(
    std::vector<std::string>& sixAxisForceTorqueSensorsList)
{
    if (!m_pimpl->checkValid("[YarpSensorBridge::getSixAxisForceTorqueSensorsList]"))
    {
        return false;
    }
    sixAxisForceTorqueSensorsList = m_pimpl->metaData.sensorsList.sixAxisForceTorqueSensorsList;
    return true;
}

bool YarpSensorBridge::getCartesianWrenchesList(std::vector<std::string>& cartesianWrenchesList)
{
    if (!m_pimpl->checkValid("[YarpSensorBridge::getCartesianWrenchesList]"))
    {
        return false;
    }
    cartesianWrenchesList = m_pimpl->metaData.sensorsList.cartesianWrenchesList;
    return true;
}

bool YarpSensorBridge::getTemperatureSensorsList(std::vector<std::string>& temperatureSensorsList)
{
    if (!m_pimpl->checkValid("[YarpSensorBridge::getTemperatureSensorsList]"))
    {
        return false;
    }
    temperatureSensorsList = m_pimpl->metaData.sensorsList.temperatureSensorsList;
    return true;
}

const std::vector<std::string>& YarpSensorBridge::getJointsList() const
{
    return m_pimpl->metaData.sensorsList.jointsList;
}

const std::vector<std::string>& YarpSensorBridge::getLinearAccelerometersList() const
{
    return m_pimpl->metaData.sensorsList.linearAccelerometersList;
}

const std::vector<std::string>& YarpSensorBridge::getGyroscopesList() const
{
    return m_pimpl->metaData.sensorsList.gyroscopesList;
}

const std::vector<std::string>& YarpSensorBridge::getOrientationSensorsList() const
{
    return m_pimpl->metaData.sensorsList.orientationSensorsList;
}

const std::vector<std::string>& YarpSensorBridge::getMagnetometersList() const
{
    return m_pimpl->metaData.sensorsList.magnetometersList;
}

const std::vector<std::string>& YarpSensorBridge::getSixAxisForceTorqueSensorsList() const
{
    return m_pimpl->metaData.sensorsList.sixAxisForceTorqueSensorsList;
}

const std::vector<std::string>& YarpSensorBridge::getTemperatureSensorsList() const
{
    return m_pimpl->metaData.sensorsList.temperatureSensorsList;
}

const std::vector<std::string>& YarpSensorBridge::getCartesianWrenchesList() const
{
    return m_pimpl->metaData.sensorsList.cartesianWrenchesList;
}

bool YarpSensorBridge::getJointPosition(const std::string& jointName,
                                        double& jointPosition,
                                        OptionalDoubleRef receiveTimeInSeconds)
{
    constexpr auto logPrefix = "[YarpSensorBridge::getJointPosition]";
    int idx;
    if (!m_pimpl->getIndexFromVector(m_pimpl->metaData.sensorsList.jointsList, jointName, idx))
    {
        log()->error("{} {} could not be found in the configured list of joints.",
                     logPrefix,
                     jointName);
        return false;
    }

    jointPosition = m_pimpl->controlBoardRemapperMeasures.jointPositions[idx];

    if (receiveTimeInSeconds)
    {
        receiveTimeInSeconds.value().get()
            = m_pimpl->controlBoardRemapperMeasures.receivedTimeInSeconds;
    }
    return true;
}

bool YarpSensorBridge::getJointPositions(Eigen::Ref<Eigen::VectorXd> jointPositions,
                                         OptionalDoubleRef receiveTimeInSeconds)
{
    if (!m_pimpl->checkControlBoardSensor("[YarpSensorBridge::getJointPositions]",
                                          m_pimpl->controlBoardRemapperInterfaces.encoders,
                                          m_pimpl->metaData.bridgeOptions.isJointSensorsEnabled,
                                          m_pimpl->controlBoardRemapperMeasures.jointPositions))
    {
        return false;
    }
    jointPositions = m_pimpl->controlBoardRemapperMeasures.jointPositions;
    if (receiveTimeInSeconds)
    {
        receiveTimeInSeconds.value().get()
            = m_pimpl->controlBoardRemapperMeasures.receivedTimeInSeconds;
    }
    return true;
}

bool YarpSensorBridge::getJointVelocity(const std::string& jointName,
                                        double& jointVelocity,
                                        OptionalDoubleRef receiveTimeInSeconds)
{
    constexpr auto logPrefix = "[YarpSensorBridge::getJointVelocity]";
    if (!m_pimpl->checkControlBoardSensor(logPrefix,
                                          m_pimpl->controlBoardRemapperInterfaces.encoders,
                                          m_pimpl->metaData.bridgeOptions.isJointSensorsEnabled,
                                          m_pimpl->controlBoardRemapperMeasures.jointVelocities))
    {
        return false;
    }
    int idx;
    if (!m_pimpl->getIndexFromVector(m_pimpl->metaData.sensorsList.jointsList, jointName, idx))
    {
        log()->error("{} {} could not be found in the configured list of joints.",
                     logPrefix,
                     jointName);
        return false;
    }

    jointVelocity = m_pimpl->controlBoardRemapperMeasures.jointVelocities[idx];
    if (receiveTimeInSeconds)
    {
        receiveTimeInSeconds.value().get()
            = m_pimpl->controlBoardRemapperMeasures.receivedTimeInSeconds;
    }
    return true;
}

bool YarpSensorBridge::getJointVelocities(Eigen::Ref<Eigen::VectorXd> jointVelocties,
                                          OptionalDoubleRef receiveTimeInSeconds)
{
    if (!m_pimpl->checkControlBoardSensor("[YarpSensorBridge::getJointVelocities]",
                                          m_pimpl->controlBoardRemapperInterfaces.encoders,
                                          m_pimpl->metaData.bridgeOptions.isJointSensorsEnabled,
                                          m_pimpl->controlBoardRemapperMeasures.jointVelocities))
    {
        return false;
    }

    if (jointVelocties.size() != m_pimpl->controlBoardRemapperMeasures.jointVelocities.size())
    {
        log()->error("[YarpSensorBridge::getJointVelocities] The size of the input vector does not "
                     "match the number of joints.");
        return false;
    }

    jointVelocties = m_pimpl->controlBoardRemapperMeasures.jointVelocities;

    if (receiveTimeInSeconds)
    {
        receiveTimeInSeconds.value().get()
            = m_pimpl->controlBoardRemapperMeasures.receivedTimeInSeconds;
    }

    return true;
}

bool YarpSensorBridge::getJointAcceleration(const std::string& jointName,
                                            double& jointAcceleration,
                                            OptionalDoubleRef receiveTimeInSeconds)
{
    constexpr auto logPrefix = "[YarpSensorBridge::getJointAcceleration]";

    if (!m_pimpl->streamJointAccelerations)
    {
        log()->error("{} Joint acceleration is not streamed.", logPrefix);
        return false;
    }

    int idx;
    if (!m_pimpl->getIndexFromVector(m_pimpl->metaData.sensorsList.jointsList, jointName, idx))
    {
        log()->error("{} {} could not be found in the configured list of joints.",
                     logPrefix,
                     jointName);
        return false;
    }

    jointAcceleration = m_pimpl->controlBoardRemapperMeasures.jointAccelerations[idx];

    if (receiveTimeInSeconds)
    {
        receiveTimeInSeconds.value().get()
            = m_pimpl->controlBoardRemapperMeasures.receivedTimeInSeconds;
    }

    return true;
}

bool YarpSensorBridge::getJointAccelerations(Eigen::Ref<Eigen::VectorXd> jointAccelerations,
                                             OptionalDoubleRef receiveTimeInSeconds)
{
    constexpr auto logPrefix = "[YarpSensorBridge::getJointAcceleration]";

    if (!m_pimpl->streamJointAccelerations)
    {
        log()->error("{} Joint acceleration is not streamed.", logPrefix);
        return false;
    }

    if (!m_pimpl->checkControlBoardSensor(logPrefix,
                                          m_pimpl->controlBoardRemapperInterfaces.encoders,
                                          m_pimpl->metaData.bridgeOptions.isJointSensorsEnabled,
                                          m_pimpl->controlBoardRemapperMeasures.jointAccelerations))
    {
        return false;
    }

    if (jointAccelerations.size()
        != m_pimpl->controlBoardRemapperMeasures.jointAccelerations.size())
    {
        log()->error("{} The size of the input vector does not match the number of joints.",
                     logPrefix);
        return false;
    }

    jointAccelerations = m_pimpl->controlBoardRemapperMeasures.jointAccelerations;
    if (receiveTimeInSeconds)
    {
        receiveTimeInSeconds.value().get()
            = m_pimpl->controlBoardRemapperMeasures.receivedTimeInSeconds;
    }
    return true;
}

bool YarpSensorBridge::getLinearAccelerometerMeasurement(const std::string& accName,
                                                         Eigen::Ref<Eigen::Vector3d> accMeasurement,
                                                         OptionalDoubleRef receiveTimeInSeconds)
{
    if (!m_pimpl->checkValidSensorMeasure("YarpSensorBridge::getLinearAccelerometerMeasurement ",
                                          m_pimpl->accelerometerMeasures,
                                          accName))
    {
        return false;
    }

    auto iter = m_pimpl->accelerometerMeasures.find(accName);
    accMeasurement = yarp::eigen::toEigen(iter->second.first);
    if (receiveTimeInSeconds)
    {
        receiveTimeInSeconds.value().get() = iter->second.second;
    }
    return true;
}

bool YarpSensorBridge::getGyroscopeMeasure(const std::string& gyroName,
                                           Eigen::Ref<Eigen::Vector3d> gyroMeasurement,
                                           OptionalDoubleRef receiveTimeInSeconds)
{
    if (!m_pimpl->checkValidSensorMeasure("YarpSensorBridge::getGyroscopeMeasure ",
                                          m_pimpl->gyroMeasures,
                                          gyroName))
    {
        return false;
    }

    auto iter = m_pimpl->gyroMeasures.find(gyroName);
    gyroMeasurement = yarp::eigen::toEigen(iter->second.first);
    if (receiveTimeInSeconds)
        receiveTimeInSeconds.value().get() = iter->second.second;
    return true;
}

bool YarpSensorBridge::getOrientationSensorMeasurement(const std::string& rpyName,
                                                       Eigen::Ref<Eigen::Vector3d> rpyMeasurement,
                                                       OptionalDoubleRef receiveTimeInSeconds)
{
    if (!m_pimpl->checkValidSensorMeasure("YarpSensorBridge::getOrientationSensorMeasurement ",
                                          m_pimpl->orientationMeasures,
                                          rpyName))
    {
        return false;
    }

    auto iter = m_pimpl->orientationMeasures.find(rpyName);
    rpyMeasurement = yarp::eigen::toEigen(iter->second.first);
    if (receiveTimeInSeconds)
    {
        receiveTimeInSeconds.value().get() = iter->second.second;
    }
    return true;
}

bool YarpSensorBridge::getMagnetometerMeasurement(const std::string& magName,
                                                  Eigen::Ref<Eigen::Vector3d> magMeasurement,
                                                  OptionalDoubleRef receiveTimeInSeconds)
{
    if (!m_pimpl->checkValidSensorMeasure("YarpSensorBridge::getMagnetometerMeasurement ",
                                          m_pimpl->magnetometerMeasures,
                                          magName))
    {
        return false;
    }

    auto iter = m_pimpl->magnetometerMeasures.find(magName);
    magMeasurement = yarp::eigen::toEigen(iter->second.first);
    if (receiveTimeInSeconds)
        receiveTimeInSeconds.value().get() = iter->second.second;
    return true;
}

bool YarpSensorBridge::getSixAxisForceTorqueMeasurement(const std::string& ftName,
                                                        Eigen::Ref<Vector6d> ftMeasurement,
                                                        OptionalDoubleRef receiveTimeInSeconds)
{
    if (!m_pimpl->checkValidSensorMeasure("YarpSensorBridge::getSixAxisForceTorqueMeasurement ",
                                          m_pimpl->FTMeasures,
                                          ftName))
    {
        return false;
    }

    auto iter = m_pimpl->FTMeasures.find(ftName);
    ftMeasurement = yarp::eigen::toEigen(iter->second.first);
    if (receiveTimeInSeconds)
        receiveTimeInSeconds.value().get() = iter->second.second;
    return true;
}

bool YarpSensorBridge::getCartesianWrench(const std::string& cartesianWrenchName,
                                          Eigen::Ref<Vector6d> cartesianWrenchMeasurement,
                                          OptionalDoubleRef receiveTimeInSeconds)
{
    if (!m_pimpl->checkValidSensorMeasure("YarpSensorBridge::getCartesianWrench ",
                                          m_pimpl->cartesianWrenchMeasures,
                                          cartesianWrenchName))
    {
        return false;
    }

    auto iter = m_pimpl->cartesianWrenchMeasures.find(cartesianWrenchName);
    cartesianWrenchMeasurement = yarp::eigen::toEigen(iter->second.first);
    if (receiveTimeInSeconds)
        receiveTimeInSeconds.value().get() = iter->second.second;
    return true;
}

bool YarpSensorBridge::getTemperature(const std::string& temperatureSensorName,
                                      double& temperature,
                                      OptionalDoubleRef receiveTimeInSeconds)
{
    if (!m_pimpl->checkValidSensorMeasure("YarpSensorBridge::getTemperature ",
                                          m_pimpl->temperatureMeasures,
                                          temperatureSensorName))
    {
        return false;
    }

    auto iter = m_pimpl->temperatureMeasures.find(temperatureSensorName);
    // assuming the vector has only one value
    temperature = iter->second.first(0);
    if (receiveTimeInSeconds)
    {
        receiveTimeInSeconds.value().get() = iter->second.second;
    }
    return true;
}

bool YarpSensorBridge::getThreeAxisForceTorqueMeasurement(const std::string& ftName,
                                                          Eigen::Ref<Eigen::Vector3d> ftMeasurement,
                                                          OptionalDoubleRef receiveTimeInSeconds)
{
    log()->error("[YarpSensorBridge::getThreeAxisForceTorqueMeasurement] Currently unimplemented");
    return false;
}

bool YarpSensorBridge::getThreeAxisForceTorqueSensorsList(
    std::vector<std::string>& threeAxisForceTorqueSensorsList)
{
    log()->error("[YarpSensorBridge::getThreeAxisForceTorqueSensorsList] Currently unimplemented");
    return false;
}

bool YarpSensorBridge::getMotorCurrent(const std::string& jointName,
                                       double& motorCurrent,
                                       OptionalDoubleRef receiveTimeInSeconds)
{
    if (!m_pimpl->checkControlBoardSensor("[YarpSensorBridge::getMotorCurrent]",
                                          m_pimpl->controlBoardRemapperInterfaces.currsensors,
                                          m_pimpl->metaData.bridgeOptions.isMotorSensorsEnabled,
                                          m_pimpl->controlBoardRemapperMeasures.motorCurrents))
    {
        return false;
    }

    int idx;
    if (!m_pimpl->getIndexFromVector(m_pimpl->metaData.sensorsList.jointsList, jointName, idx))
    {
        log()->error("[YarpSensorBridge::getJointCurrent] {} could not be found in the configured "
                     "list of joints.",
                     jointName);
        return false;
    }

    motorCurrent = m_pimpl->controlBoardRemapperMeasures.motorCurrents[idx];

    if (receiveTimeInSeconds)
        receiveTimeInSeconds.value().get()
            = m_pimpl->controlBoardRemapperMeasures.receivedTimeInSeconds;

    return true;
}

bool YarpSensorBridge::getMotorCurrents(Eigen::Ref<Eigen::VectorXd> motorCurrents,
                                        OptionalDoubleRef receiveTimeInSeconds)
{
    if (!m_pimpl->checkControlBoardSensor("[YarpSensorBridge::getMotorCurrents]",
                                          m_pimpl->controlBoardRemapperInterfaces.currsensors,
                                          m_pimpl->metaData.bridgeOptions.isMotorSensorsEnabled,
                                          m_pimpl->controlBoardRemapperMeasures.motorCurrents))
    {
        return false;
    }

    if (motorCurrents.size() != m_pimpl->controlBoardRemapperMeasures.motorCurrents.size())
    {
        log()->error("[YarpSensorBridge::getMotorCurrents] The size of the input vector does not "
                     "match the number of motor currents.");
        return false;
    }

    motorCurrents = m_pimpl->controlBoardRemapperMeasures.motorCurrents;

    if (receiveTimeInSeconds)
        receiveTimeInSeconds.value().get()
            = m_pimpl->controlBoardRemapperMeasures.receivedTimeInSeconds;

    return true;
}

bool YarpSensorBridge::getMotorPWM(const std::string& jointName,
                                   double& motorPWM,
                                   OptionalDoubleRef receiveTimeInSeconds)
{
    if (!m_pimpl->checkControlBoardSensor("[YarpSensorBridge::getMotorPWM]",
                                          m_pimpl->controlBoardRemapperInterfaces.amp,
                                          m_pimpl->metaData.bridgeOptions.isPWMControlEnabled,
                                          m_pimpl->controlBoardRemapperMeasures.motorPWMs))
    {
        return false;
    }

    int idx;
    if (!m_pimpl->getIndexFromVector(m_pimpl->metaData.sensorsList.jointsList, jointName, idx))
    {
        log()->error("[YarpSensorBridge::getMotorPWM] {} could not be found in the configured "
                     "list of joints.",
                     jointName);
        return false;
    }

    motorPWM = m_pimpl->controlBoardRemapperMeasures.motorPWMs[idx];

    if (receiveTimeInSeconds)
        receiveTimeInSeconds.value().get()
            = m_pimpl->controlBoardRemapperMeasures.receivedTimeInSeconds;

    return true;
}

bool YarpSensorBridge::getMotorPWMs(Eigen::Ref<Eigen::VectorXd> motorPWMs,
                                    OptionalDoubleRef receiveTimeInSeconds)
{
    if (!m_pimpl->checkControlBoardSensor("[YarpSensorBridge::getMotorPWMs]",
                                          m_pimpl->controlBoardRemapperInterfaces.amp,
                                          m_pimpl->metaData.bridgeOptions.isPWMControlEnabled,
                                          m_pimpl->controlBoardRemapperMeasures.motorPWMs))
    {
        return false;
    }

    if (motorPWMs.size() != m_pimpl->controlBoardRemapperMeasures.motorPWMs.size())
    {
        log()->error("[YarpSensorBridge::getMotorPWMs] The size of the input vector does not match "
                     "the number of motor PWMs.");
        return false;
    }

    motorPWMs = m_pimpl->controlBoardRemapperMeasures.motorPWMs;

    if (receiveTimeInSeconds)
        receiveTimeInSeconds.value().get()
            = m_pimpl->controlBoardRemapperMeasures.receivedTimeInSeconds;

    return true;
}

bool YarpSensorBridge::getJointTorque(const std::string& jointName,
                                      double& jointTorque,
                                      OptionalDoubleRef receiveTimeInSeconds)
{
    if (!m_pimpl->checkControlBoardSensor("[YarpSensorBridge::getJointTorque]",
                                          m_pimpl->controlBoardRemapperInterfaces.torques,
                                          m_pimpl->metaData.bridgeOptions.isJointSensorsEnabled,
                                          m_pimpl->controlBoardRemapperMeasures.jointTorques))
    {
        return false;
    }

    int idx;
    if (!m_pimpl->getIndexFromVector(m_pimpl->metaData.sensorsList.jointsList, jointName, idx))
    {
        log()->error("[YarpSensorBridge::getJointTorque] {} could not be found in the configured "
                     "list of joints.",
                     jointName);
        return false;
    }

    jointTorque = m_pimpl->controlBoardRemapperMeasures.jointTorques[idx];

    if (receiveTimeInSeconds)
        receiveTimeInSeconds.value().get()
            = m_pimpl->controlBoardRemapperMeasures.receivedTimeInSeconds;

    return true;
}

bool YarpSensorBridge::getJointTorques(Eigen::Ref<Eigen::VectorXd> jointTorques,
                                       OptionalDoubleRef receiveTimeInSeconds)
{
    if (!m_pimpl->checkControlBoardSensor("[YarpSensorBridge::getJointTorques]",
                                          m_pimpl->controlBoardRemapperInterfaces.torques,
                                          m_pimpl->metaData.bridgeOptions.isJointSensorsEnabled,
                                          m_pimpl->controlBoardRemapperMeasures.jointTorques))
    {
        return false;
    }

    if (jointTorques.size() != m_pimpl->controlBoardRemapperMeasures.jointTorques.size())
    {
        log()->error("[YarpSensorBridge::getJointTorques] The size of the input vector does not "
                     "match the number of joint torques.");
        return false;
    }

    jointTorques = m_pimpl->controlBoardRemapperMeasures.jointTorques;

    if (receiveTimeInSeconds)
        receiveTimeInSeconds.value().get()
            = m_pimpl->controlBoardRemapperMeasures.receivedTimeInSeconds;

    return true;
}

bool YarpSensorBridge::getPidPosition(const std::string& jointName,
                                      double& pidPosition,
                                      OptionalDoubleRef receiveTimeInSeconds)
{
    if (!m_pimpl->checkControlBoardSensor("[YarpSensorBridge::getPidPosition]",
                                          m_pimpl->controlBoardRemapperInterfaces.pids,
                                          m_pimpl->metaData.bridgeOptions.isPIDsEnabled,
                                          m_pimpl->controlBoardRemapperMeasures.pidPositions))
    {
        return false;
    }

    int idx;
    if (!m_pimpl->getIndexFromVector(m_pimpl->metaData.sensorsList.jointsList, jointName, idx))
    {
        log()->error("[YarpSensorBridge::getPidPosition] {} could not be found in the configured "
                     "list of motors.",
                     jointName);
        return false;
    }

    pidPosition = m_pimpl->controlBoardRemapperMeasures.pidPositions[idx];

    if (receiveTimeInSeconds)
        receiveTimeInSeconds.value().get()
            = m_pimpl->controlBoardRemapperMeasures.receivedTimeInSeconds;

    return true;
}

bool YarpSensorBridge::getPidPositions(Eigen::Ref<Eigen::VectorXd> pidPositions,
                                       OptionalDoubleRef receiveTimeInSeconds)
{
    if (!m_pimpl->checkControlBoardSensor("[YarpSensorBridge::getPidPositions]",
                                          m_pimpl->controlBoardRemapperInterfaces.pids,
                                          m_pimpl->metaData.bridgeOptions.isPIDsEnabled,
                                          m_pimpl->controlBoardRemapperMeasures.pidPositions))
    {
        return false;
    }

    if (pidPositions.size() != m_pimpl->controlBoardRemapperMeasures.pidPositions.size())
    {
        log()->error("[YarpSensorBridge::getPidPositions] The size of the input vector does not "
                     "match the number of pid positions.");
        return false;
    }

    pidPositions = m_pimpl->controlBoardRemapperMeasures.pidPositions;

    if (receiveTimeInSeconds)
        receiveTimeInSeconds.value().get()
            = m_pimpl->controlBoardRemapperMeasures.receivedTimeInSeconds;

    return true;
}

bool YarpSensorBridge::getPidPositionError(const std::string& jointName,
                                           double& pidPositionError,
                                           OptionalDoubleRef receiveTimeInSeconds)
{
    if (!m_pimpl->checkControlBoardSensor("[YarpSensorBridge::getPidPositionError]",
                                          m_pimpl->controlBoardRemapperInterfaces.pids,
                                          m_pimpl->metaData.bridgeOptions.isPIDsEnabled,
                                          m_pimpl->controlBoardRemapperMeasures.pidPositionErrors))
    {
        return false;
    }

    int idx;
    if (!m_pimpl->getIndexFromVector(m_pimpl->metaData.sensorsList.jointsList, jointName, idx))
    {
        log()->error("[YarpSensorBridge::getPidPositionError] {} could not be found in the "
                     "configured "
                     "list of motors.",
                     jointName);
        return false;
    }

    pidPositionError = m_pimpl->controlBoardRemapperMeasures.pidPositionErrors[idx];

    if (receiveTimeInSeconds)
        receiveTimeInSeconds.value().get()
            = m_pimpl->controlBoardRemapperMeasures.receivedTimeInSeconds;

    return true;
}

bool YarpSensorBridge::getPidPositionErrors(Eigen::Ref<Eigen::VectorXd> pidPositionErrors,
                                            OptionalDoubleRef receiveTimeInSeconds)
{
    if (!m_pimpl->checkControlBoardSensor("[YarpSensorBridge::getPidPositionErrors]",
                                          m_pimpl->controlBoardRemapperInterfaces.pids,
                                          m_pimpl->metaData.bridgeOptions.isPIDsEnabled,
                                          m_pimpl->controlBoardRemapperMeasures.pidPositionErrors))
    {
        return false;
    }

    if (pidPositionErrors.size() != m_pimpl->controlBoardRemapperMeasures.pidPositionErrors.size())
    {
        log()->error("[YarpSensorBridge::getPidPositionErrors] The size of the input vector does "
                     "not match the number of pid position errors.");
        return false;
    }

    pidPositionErrors = m_pimpl->controlBoardRemapperMeasures.pidPositionErrors;

    if (receiveTimeInSeconds)
        receiveTimeInSeconds.value().get()
            = m_pimpl->controlBoardRemapperMeasures.receivedTimeInSeconds;

    return true;
}

bool YarpSensorBridge::getMotorPosition(const std::string& jointName,
                                        double& motorPosition,
                                        OptionalDoubleRef receiveTimeInSeconds)
{
    if (!m_pimpl->checkControlBoardSensor("[YarpSensorBridge::getMotorPosition]",
                                          m_pimpl->controlBoardRemapperInterfaces.motorEncoders,
                                          m_pimpl->metaData.bridgeOptions.isMotorSensorsEnabled,
                                          m_pimpl->controlBoardRemapperMeasures.motorPositions))
    {
        return false;
    }

    int idx;
    if (!m_pimpl->getIndexFromVector(m_pimpl->metaData.sensorsList.jointsList, jointName, idx))
    {
        log()->error("[YarpSensorBridge::getMotorPosition] {} could not be found in the configured "
                     "list of motors.",
                     jointName);
        return false;
    }

    motorPosition = m_pimpl->controlBoardRemapperMeasures.motorPositions[idx];

    if (receiveTimeInSeconds)
        receiveTimeInSeconds.value().get()
            = m_pimpl->controlBoardRemapperMeasures.receivedTimeInSeconds;

    return true;
}

bool YarpSensorBridge::getMotorPositions(Eigen::Ref<Eigen::VectorXd> motorPositions,
                                         OptionalDoubleRef receiveTimeInSeconds)
{
    if (!m_pimpl->checkControlBoardSensor("[YarpSensorBridge::getMotorPositions]",
                                          m_pimpl->controlBoardRemapperInterfaces.motorEncoders,
                                          m_pimpl->metaData.bridgeOptions.isMotorSensorsEnabled,
                                          m_pimpl->controlBoardRemapperMeasures.motorPositions))
    {
        return false;
    }

    if (motorPositions.size() != m_pimpl->controlBoardRemapperMeasures.motorPositions.size())
    {
        log()->error("[YarpSensorBridge::getMotorPositions] The size of the input vector does not "
                     "match the number of motor positions.");
        return false;
    }

    motorPositions = m_pimpl->controlBoardRemapperMeasures.motorPositions;

    if (receiveTimeInSeconds)
        receiveTimeInSeconds.value().get()
            = m_pimpl->controlBoardRemapperMeasures.receivedTimeInSeconds;

    return true;
}

bool YarpSensorBridge::getMotorVelocity(const std::string& jointName,
                                        double& motorVelocity,
                                        OptionalDoubleRef receiveTimeInSeconds)
{
    if (!m_pimpl->checkControlBoardSensor("[YarpSensorBridge::getMotorVelocity]",
                                          m_pimpl->controlBoardRemapperInterfaces.motorEncoders,
                                          m_pimpl->metaData.bridgeOptions.isMotorSensorsEnabled,
                                          m_pimpl->controlBoardRemapperMeasures.motorVelocities))
    {
        return false;
    }

    int idx;
    if (!m_pimpl->getIndexFromVector(m_pimpl->metaData.sensorsList.jointsList, jointName, idx))
    {
        log()->error("[YarpSensorBridge::getMotorVelocity] {} could not be found in the configured "
                     "list of motors.",
                     jointName);
        return false;
    }

    motorVelocity = m_pimpl->controlBoardRemapperMeasures.motorVelocities[idx];

    if (receiveTimeInSeconds)
        receiveTimeInSeconds.value().get()
            = m_pimpl->controlBoardRemapperMeasures.receivedTimeInSeconds;

    return true;
}

bool YarpSensorBridge::getMotorVelocities(Eigen::Ref<Eigen::VectorXd> motorVelocties,
                                          OptionalDoubleRef receiveTimeInSeconds)
{
    if (!m_pimpl->checkControlBoardSensor("[YarpSensorBridge::getMotorVelocities]",
                                          m_pimpl->controlBoardRemapperInterfaces.motorEncoders,
                                          m_pimpl->metaData.bridgeOptions.isMotorSensorsEnabled,
                                          m_pimpl->controlBoardRemapperMeasures.motorVelocities))
    {
        return false;
    }

    if (motorVelocties.size() != m_pimpl->controlBoardRemapperMeasures.motorVelocities.size())
    {
        log()->error("[YarpSensorBridge::getMotorVelocities] The size of the input vector does not "
                     "match the number of motor velocities.");
        return false;
    }

    motorVelocties = m_pimpl->controlBoardRemapperMeasures.motorVelocities;

    if (receiveTimeInSeconds)
        receiveTimeInSeconds.value().get()
            = m_pimpl->controlBoardRemapperMeasures.receivedTimeInSeconds;

    return true;
}

bool YarpSensorBridge::getMotorAcceleration(const std::string& jointName,
                                            double& motorAcceleration,
                                            OptionalDoubleRef receiveTimeInSeconds)
{
    if (!m_pimpl->checkControlBoardSensor("[YarpSensorBridge::getMotorAcceleration]",
                                          m_pimpl->controlBoardRemapperInterfaces.motorEncoders,
                                          m_pimpl->metaData.bridgeOptions.isMotorSensorsEnabled,
                                          m_pimpl->controlBoardRemapperMeasures.motorAccelerations))
    {
        return false;
    }

    int idx;
    if (!m_pimpl->getIndexFromVector(m_pimpl->metaData.sensorsList.jointsList, jointName, idx))
    {
        log()->error("[YarpSensorBridge::getMotorPosition] {} could not be found in the configured "
                     "list of motors.",
                     jointName);
        return false;
    }

    motorAcceleration = m_pimpl->controlBoardRemapperMeasures.motorAccelerations[idx];

    if (receiveTimeInSeconds)
        receiveTimeInSeconds.value().get()
            = m_pimpl->controlBoardRemapperMeasures.receivedTimeInSeconds;

    return true;
}

bool YarpSensorBridge::getMotorAccelerations(Eigen::Ref<Eigen::VectorXd> motorAccelerations,
                                             OptionalDoubleRef receiveTimeInSeconds)
{
    if (!m_pimpl->checkControlBoardSensor("[YarpSensorBridge::getMotorAccelerations]",
                                          m_pimpl->controlBoardRemapperInterfaces.motorEncoders,
                                          m_pimpl->metaData.bridgeOptions.isMotorSensorsEnabled,
                                          m_pimpl->controlBoardRemapperMeasures.motorAccelerations))
    {
        return false;
    }

    if (motorAccelerations.size()
        != m_pimpl->controlBoardRemapperMeasures.motorAccelerations.size())
    {
        log()->error("[YarpSensorBridge::getMotorAccelerations] The size of the input vector does "
                     "not match the number of motor accelerations.");
        return false;
    }

    motorAccelerations = m_pimpl->controlBoardRemapperMeasures.motorAccelerations;

    if (receiveTimeInSeconds)
        receiveTimeInSeconds.value().get()
            = m_pimpl->controlBoardRemapperMeasures.receivedTimeInSeconds;

    return true;
}

bool YarpSensorBridge::getMotorTemperature(const std::string& jointName,
                                           double& motorTorque,
                                           OptionalDoubleRef receiveTimeInSeconds)
{
    if (!m_pimpl->checkControlBoardSensor("[YarpSensorBridge::getMotorTorque]",
                                          m_pimpl->controlBoardRemapperInterfaces.motor,
                                          m_pimpl->metaData.bridgeOptions.isMotorSensorsEnabled,
                                          m_pimpl->controlBoardRemapperMeasures.motorTemperatures))
    {
        return false;
    }

    int idx;
    if (!m_pimpl->getIndexFromVector(m_pimpl->metaData.sensorsList.jointsList, jointName, idx))
    {
        log()->error("[YarpSensorBridge::getMotorTorque] {} could not be found in the configured "
                     "list of motors.",
                     jointName);
        return false;
    }

    motorTorque = m_pimpl->controlBoardRemapperMeasures.motorTemperatures[idx];
    if (receiveTimeInSeconds)
        receiveTimeInSeconds.value().get()
            = m_pimpl->controlBoardRemapperMeasures.receivedTimeInSeconds;

    return true;
}

bool YarpSensorBridge::getMotorTemperatures(Eigen::Ref<Eigen::VectorXd> motorTemperatures,
                                            OptionalDoubleRef receiveTimeInSeconds)
{
    if (!m_pimpl->checkControlBoardSensor("[YarpSensorBridge::getMotorTemperatures]",
                                          m_pimpl->controlBoardRemapperInterfaces.motor,
                                          m_pimpl->metaData.bridgeOptions.isMotorSensorsEnabled,
                                          m_pimpl->controlBoardRemapperMeasures.motorTemperatures))
    {
        return false;
    }

    if (motorTemperatures.size() != m_pimpl->controlBoardRemapperMeasures.motorTemperatures.size())
    {
        log()->error("[YarpSensorBridge::getMotorTemperatures] The size of the input vector does "
                     "not match the number of motor temperatures.");
        return false;
    }

    motorTemperatures = m_pimpl->controlBoardRemapperMeasures.motorTemperatures;
    if (receiveTimeInSeconds)
        receiveTimeInSeconds.value().get()
            = m_pimpl->controlBoardRemapperMeasures.receivedTimeInSeconds;

    return true;
}
