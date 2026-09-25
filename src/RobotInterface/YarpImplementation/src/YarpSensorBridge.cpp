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

    ret = m_pimpl->subConfigLoader("stream_battery",
                                   "Batteries",
                                   &YarpSensorBridge::Impl::configureBatteries,
                                   handler,
                                   m_pimpl->metaData,
                                   m_pimpl->metaData.bridgeOptions.isBatteryEnabled);
    if (!ret)
    {
        log()->info("{} Skipping the configuration of Batteries. YarpSensorBridge "
                    "will not stream battery measures.",
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
    ret = ret && m_pimpl->attachAllBatteries(deviceDriversList);

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

bool YarpSensorBridge::getBatteriesList(std::vector<std::string>& batteriesList)
{
    if (!m_pimpl->checkValid("[YarpSensorBridge::getBatteriesList]"))
    {
        return false;
    }
    batteriesList = m_pimpl->metaData.sensorsList.batteriesList;
    return true;
}

const std::vector<std::string>& YarpSensorBridge::getBatteriesList() const
{
    return m_pimpl->metaData.sensorsList.batteriesList;
}

bool YarpSensorBridge::getBatteryStatus(const std::string& batteryName,
                                        BatteryStatus& batteryStatus,
                                        OptionalDoubleRef receiveTimeInSeconds)
{
    constexpr auto logPrefix = "[YarpSensorBridge::getBatteryStatus]";

    if (!m_pimpl->checkValidSensorMeasure(logPrefix,
                                          m_pimpl->batteryMeasures,
                                          batteryName))
    {
        return false;
    }

    const auto& measure = m_pimpl->batteryMeasures.at(batteryName);
    batteryStatus.voltage     = measure.first[0];
    batteryStatus.current     = measure.first[1];
    batteryStatus.charge      = measure.first[2];
    batteryStatus.temperature = measure.first[3];

    if (receiveTimeInSeconds)
    {
        receiveTimeInSeconds.value().get() = measure.second;
    }
    return true;
}

bool YarpSensorBridge::getJointPosition(const std::string& jointName,
                                        double& jointPosition,
                                        OptionalDoubleRef receiveTimeInSeconds)
{
    return m_pimpl->getControlBoardMeasure("[YarpSensorBridge::getJointPosition]",
                                           m_pimpl->controlBoardRemapperInterfaces.encoders,
                                           m_pimpl->metaData.bridgeOptions.isJointSensorsEnabled,
                                           m_pimpl->controlBoardRemapperMeasures.jointPositions,
                                           jointName,
                                           jointPosition,
                                           receiveTimeInSeconds);
}

bool YarpSensorBridge::getJointPositions(Eigen::Ref<Eigen::VectorXd> jointPositions,
                                         OptionalDoubleRef receiveTimeInSeconds)
{
    return m_pimpl->getControlBoardMeasures("[YarpSensorBridge::getJointPositions]",
                                            m_pimpl->controlBoardRemapperInterfaces.encoders,
                                            m_pimpl->metaData.bridgeOptions.isJointSensorsEnabled,
                                            m_pimpl->controlBoardRemapperMeasures.jointPositions,
                                            jointPositions,
                                            receiveTimeInSeconds);
}

bool YarpSensorBridge::getJointVelocity(const std::string& jointName,
                                        double& jointVelocity,
                                        OptionalDoubleRef receiveTimeInSeconds)
{
    return m_pimpl->getControlBoardMeasure("[YarpSensorBridge::getJointVelocity]",
                                           m_pimpl->controlBoardRemapperInterfaces.encoders,
                                           m_pimpl->metaData.bridgeOptions.isJointSensorsEnabled,
                                           m_pimpl->controlBoardRemapperMeasures.jointVelocities,
                                           jointName,
                                           jointVelocity,
                                           receiveTimeInSeconds);
}

bool YarpSensorBridge::getJointVelocities(Eigen::Ref<Eigen::VectorXd> jointVelocties,
                                          OptionalDoubleRef receiveTimeInSeconds)
{
    return m_pimpl->getControlBoardMeasures("[YarpSensorBridge::getJointVelocities]",
                                            m_pimpl->controlBoardRemapperInterfaces.encoders,
                                            m_pimpl->metaData.bridgeOptions.isJointSensorsEnabled,
                                            m_pimpl->controlBoardRemapperMeasures.jointVelocities,
                                            jointVelocties,
                                            receiveTimeInSeconds);
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

    return m_pimpl->getControlBoardMeasure(logPrefix,
                                           m_pimpl->controlBoardRemapperInterfaces.encoders,
                                           m_pimpl->metaData.bridgeOptions.isJointSensorsEnabled,
                                           m_pimpl->controlBoardRemapperMeasures.jointAccelerations,
                                           jointName,
                                           jointAcceleration,
                                           receiveTimeInSeconds);
}

bool YarpSensorBridge::getJointAccelerations(Eigen::Ref<Eigen::VectorXd> jointAccelerations,
                                             OptionalDoubleRef receiveTimeInSeconds)
{
    constexpr auto logPrefix = "[YarpSensorBridge::getJointAccelerations]";

    if (!m_pimpl->streamJointAccelerations)
    {
        log()->error("{} Joint acceleration is not streamed.", logPrefix);
        return false;
    }

    return m_pimpl->getControlBoardMeasures(logPrefix,
                                            m_pimpl->controlBoardRemapperInterfaces.encoders,
                                            m_pimpl->metaData.bridgeOptions.isJointSensorsEnabled,
                                            m_pimpl->controlBoardRemapperMeasures.jointAccelerations,
                                            jointAccelerations,
                                            receiveTimeInSeconds);
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
    return m_pimpl->getControlBoardMeasure("[YarpSensorBridge::getMotorCurrent]",
                                           m_pimpl->controlBoardRemapperInterfaces.currsensors,
                                           m_pimpl->metaData.bridgeOptions.isMotorSensorsEnabled,
                                           m_pimpl->controlBoardRemapperMeasures.motorCurrents,
                                           jointName,
                                           motorCurrent,
                                           receiveTimeInSeconds);
}

bool YarpSensorBridge::getMotorCurrents(Eigen::Ref<Eigen::VectorXd> motorCurrents,
                                        OptionalDoubleRef receiveTimeInSeconds)
{
    return m_pimpl->getControlBoardMeasures("[YarpSensorBridge::getMotorCurrents]",
                                            m_pimpl->controlBoardRemapperInterfaces.currsensors,
                                            m_pimpl->metaData.bridgeOptions.isMotorSensorsEnabled,
                                            m_pimpl->controlBoardRemapperMeasures.motorCurrents,
                                            motorCurrents,
                                            receiveTimeInSeconds);
}

bool YarpSensorBridge::getMotorPWM(const std::string& jointName,
                                   double& motorPWM,
                                   OptionalDoubleRef receiveTimeInSeconds)
{
    return m_pimpl->getControlBoardMeasure("[YarpSensorBridge::getMotorPWM]",
                                           m_pimpl->controlBoardRemapperInterfaces.amp,
                                           m_pimpl->metaData.bridgeOptions.isPWMControlEnabled,
                                           m_pimpl->controlBoardRemapperMeasures.motorPWMs,
                                           jointName,
                                           motorPWM,
                                           receiveTimeInSeconds);
}

bool YarpSensorBridge::getMotorPWMs(Eigen::Ref<Eigen::VectorXd> motorPWMs,
                                    OptionalDoubleRef receiveTimeInSeconds)
{
    return m_pimpl->getControlBoardMeasures("[YarpSensorBridge::getMotorPWMs]",
                                            m_pimpl->controlBoardRemapperInterfaces.amp,
                                            m_pimpl->metaData.bridgeOptions.isPWMControlEnabled,
                                            m_pimpl->controlBoardRemapperMeasures.motorPWMs,
                                            motorPWMs,
                                            receiveTimeInSeconds);
}

bool YarpSensorBridge::getJointTorque(const std::string& jointName,
                                      double& jointTorque,
                                      OptionalDoubleRef receiveTimeInSeconds)
{
    return m_pimpl->getControlBoardMeasure("[YarpSensorBridge::getJointTorque]",
                                           m_pimpl->controlBoardRemapperInterfaces.torques,
                                           m_pimpl->metaData.bridgeOptions.isJointSensorsEnabled,
                                           m_pimpl->controlBoardRemapperMeasures.jointTorques,
                                           jointName,
                                           jointTorque,
                                           receiveTimeInSeconds);
}

bool YarpSensorBridge::getJointTorques(Eigen::Ref<Eigen::VectorXd> jointTorques,
                                       OptionalDoubleRef receiveTimeInSeconds)
{
    return m_pimpl->getControlBoardMeasures("[YarpSensorBridge::getJointTorques]",
                                            m_pimpl->controlBoardRemapperInterfaces.torques,
                                            m_pimpl->metaData.bridgeOptions.isJointSensorsEnabled,
                                            m_pimpl->controlBoardRemapperMeasures.jointTorques,
                                            jointTorques,
                                            receiveTimeInSeconds);
}

bool YarpSensorBridge::getPidPosition(const std::string& jointName,
                                      double& pidPosition,
                                      OptionalDoubleRef receiveTimeInSeconds)
{
    return m_pimpl->getControlBoardMeasure("[YarpSensorBridge::getPidPosition]",
                                           m_pimpl->controlBoardRemapperInterfaces.pids,
                                           m_pimpl->metaData.bridgeOptions.isPIDsEnabled,
                                           m_pimpl->controlBoardRemapperMeasures.pidPositions,
                                           jointName,
                                           pidPosition,
                                           receiveTimeInSeconds);
}

bool YarpSensorBridge::getPidPositions(Eigen::Ref<Eigen::VectorXd> pidPositions,
                                       OptionalDoubleRef receiveTimeInSeconds)
{
    return m_pimpl->getControlBoardMeasures("[YarpSensorBridge::getPidPositions]",
                                            m_pimpl->controlBoardRemapperInterfaces.pids,
                                            m_pimpl->metaData.bridgeOptions.isPIDsEnabled,
                                            m_pimpl->controlBoardRemapperMeasures.pidPositions,
                                            pidPositions,
                                            receiveTimeInSeconds);
}

bool YarpSensorBridge::getPidPositionError(const std::string& jointName,
                                           double& pidPositionError,
                                           OptionalDoubleRef receiveTimeInSeconds)
{
    return m_pimpl->getControlBoardMeasure("[YarpSensorBridge::getPidPositionError]",
                                           m_pimpl->controlBoardRemapperInterfaces.pids,
                                           m_pimpl->metaData.bridgeOptions.isPIDsEnabled,
                                           m_pimpl->controlBoardRemapperMeasures.pidPositionErrors,
                                           jointName,
                                           pidPositionError,
                                           receiveTimeInSeconds);
}

bool YarpSensorBridge::getPidPositionErrors(Eigen::Ref<Eigen::VectorXd> pidPositionErrors,
                                            OptionalDoubleRef receiveTimeInSeconds)
{
    return m_pimpl->getControlBoardMeasures("[YarpSensorBridge::getPidPositionErrors]",
                                            m_pimpl->controlBoardRemapperInterfaces.pids,
                                            m_pimpl->metaData.bridgeOptions.isPIDsEnabled,
                                            m_pimpl->controlBoardRemapperMeasures.pidPositionErrors,
                                            pidPositionErrors,
                                            receiveTimeInSeconds);
}

bool YarpSensorBridge::getMotorPosition(const std::string& jointName,
                                        double& motorPosition,
                                        OptionalDoubleRef receiveTimeInSeconds)
{
    return m_pimpl->getControlBoardMeasure("[YarpSensorBridge::getMotorPosition]",
                                           m_pimpl->controlBoardRemapperInterfaces.motorEncoders,
                                           m_pimpl->metaData.bridgeOptions.isMotorSensorsEnabled,
                                           m_pimpl->controlBoardRemapperMeasures.motorPositions,
                                           jointName,
                                           motorPosition,
                                           receiveTimeInSeconds);
}

bool YarpSensorBridge::getMotorPositions(Eigen::Ref<Eigen::VectorXd> motorPositions,
                                         OptionalDoubleRef receiveTimeInSeconds)
{
    return m_pimpl->getControlBoardMeasures("[YarpSensorBridge::getMotorPositions]",
                                            m_pimpl->controlBoardRemapperInterfaces.motorEncoders,
                                            m_pimpl->metaData.bridgeOptions.isMotorSensorsEnabled,
                                            m_pimpl->controlBoardRemapperMeasures.motorPositions,
                                            motorPositions,
                                            receiveTimeInSeconds);
}

bool YarpSensorBridge::getMotorVelocity(const std::string& jointName,
                                        double& motorVelocity,
                                        OptionalDoubleRef receiveTimeInSeconds)
{
    return m_pimpl->getControlBoardMeasure("[YarpSensorBridge::getMotorVelocity]",
                                           m_pimpl->controlBoardRemapperInterfaces.motorEncoders,
                                           m_pimpl->metaData.bridgeOptions.isMotorSensorsEnabled,
                                           m_pimpl->controlBoardRemapperMeasures.motorVelocities,
                                           jointName,
                                           motorVelocity,
                                           receiveTimeInSeconds);
}

bool YarpSensorBridge::getMotorVelocities(Eigen::Ref<Eigen::VectorXd> motorVelocties,
                                          OptionalDoubleRef receiveTimeInSeconds)
{
    return m_pimpl->getControlBoardMeasures("[YarpSensorBridge::getMotorVelocities]",
                                            m_pimpl->controlBoardRemapperInterfaces.motorEncoders,
                                            m_pimpl->metaData.bridgeOptions.isMotorSensorsEnabled,
                                            m_pimpl->controlBoardRemapperMeasures.motorVelocities,
                                            motorVelocties,
                                            receiveTimeInSeconds);
}

bool YarpSensorBridge::getMotorAcceleration(const std::string& jointName,
                                            double& motorAcceleration,
                                            OptionalDoubleRef receiveTimeInSeconds)
{
    return m_pimpl->getControlBoardMeasure("[YarpSensorBridge::getMotorAcceleration]",
                                           m_pimpl->controlBoardRemapperInterfaces.motorEncoders,
                                           m_pimpl->metaData.bridgeOptions.isMotorSensorsEnabled,
                                           m_pimpl->controlBoardRemapperMeasures.motorAccelerations,
                                           jointName,
                                           motorAcceleration,
                                           receiveTimeInSeconds);
}

bool YarpSensorBridge::getMotorAccelerations(Eigen::Ref<Eigen::VectorXd> motorAccelerations,
                                             OptionalDoubleRef receiveTimeInSeconds)
{
    return m_pimpl->getControlBoardMeasures("[YarpSensorBridge::getMotorAccelerations]",
                                            m_pimpl->controlBoardRemapperInterfaces.motorEncoders,
                                            m_pimpl->metaData.bridgeOptions.isMotorSensorsEnabled,
                                            m_pimpl->controlBoardRemapperMeasures.motorAccelerations,
                                            motorAccelerations,
                                            receiveTimeInSeconds);
}

bool YarpSensorBridge::getMotorTemperature(const std::string& jointName,
                                           double& motorTemperature,
                                           OptionalDoubleRef receiveTimeInSeconds)
{
    return m_pimpl->getControlBoardMeasure(
        "[YarpSensorBridge::getMotorTemperature]",
        m_pimpl->controlBoardRemapperInterfaces.motor,
        m_pimpl->metaData.bridgeOptions.isMotorTemperatureSensorEnabled,
        m_pimpl->controlBoardRemapperMeasures.motorTemperatures,
        jointName,
        motorTemperature,
        receiveTimeInSeconds);
}

bool YarpSensorBridge::getMotorTemperatures(Eigen::Ref<Eigen::VectorXd> motorTemperatures,
                                            OptionalDoubleRef receiveTimeInSeconds)
{
    return m_pimpl->getControlBoardMeasures(
        "[YarpSensorBridge::getMotorTemperatures]",
        m_pimpl->controlBoardRemapperInterfaces.motor,
        m_pimpl->metaData.bridgeOptions.isMotorTemperatureSensorEnabled,
        m_pimpl->controlBoardRemapperMeasures.motorTemperatures,
        motorTemperatures,
        receiveTimeInSeconds);
}
