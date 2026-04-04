/**
 * @file ISensorBridge.h
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_ROBOT_INTERFACE_ISENSOR_BRIDGE_H
#define BIPEDAL_LOCOMOTION_ROBOT_INTERFACE_ISENSOR_BRIDGE_H

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/RobotInterface/JointType.h>

namespace BipedalLocomotion
{
namespace RobotInterface
{

/**
 * Sensor bridge options
 */
struct SensorBridgeOptions
{
    bool isJointSensorsEnabled{false}; /**< flag to connect joint measurement sources */
    bool isLinearAccelerometerEnabled{false}; /**< flag to connect linear accelerometer measurement
                                                 sources */
    bool isGyroscopeEnabled{false}; /**< flag to connect gyroscope measurement sources */
    bool isOrientationSensorEnabled{false}; /**< flag to connect gyroscope measurement sources */
    bool isMagnetometerEnabled{false}; /**< flag to connect magnetometer measurement sources */
    bool isSixAxisForceTorqueSensorEnabled{false}; /**< flag to connect six axis force torque
                                                      measurement sources */
    bool isThreeAxisForceTorqueSensorEnabled{false}; /**< flag to connect six axis force torque
                                                        measurement sources */
    bool isCartesianWrenchEnabled{false}; /**< flag to connect cartesian wrench measurement sources
                                           */
    bool isPIDsEnabled{false}; /** flag to connect pid position measurement sources */
    bool isMotorSensorsEnabled{false}; /** flag to connect motor measurement sources */
    bool isPWMControlEnabled{false}; /** flag to connect PWM measurement sources */
    bool isMotorTemperatureSensorEnabled{false}; /** flag to connect motor temperature measurement sources */

    bool isTemperatureSensorEnabled{false}; /** flag to connect temperature measurement sources */

    size_t nrJoints{0}; /**< number of joints available through Kinematics stream, to be configured
                           at initialization */
};

/**
 *  Sensor lists
 */
struct SensorLists
{
    std::vector<std::string> jointsList; /**< list of joints attached to the bridge */
    std::vector<JointType> jointsTypeList; /**< list of joint types */
    std::vector<std::string> linearAccelerometersList; /**< list of linear accelerometers attached
                                                          to the bridge */
    std::vector<std::string> gyroscopesList; /**< list of gyroscopes attached to the bridge */
    std::vector<std::string> orientationSensorsList; /**< list of orientation sensors attached to
                                                        the bridge */
    std::vector<std::string> magnetometersList; /**< list of magnetometers attached to the bridge */
    std::vector<std::string> sixAxisForceTorqueSensorsList; /**< list of six axis force torque
                                                               sensors attached to the bridge */
    std::vector<std::string> threeAxisForceTorqueSensorsList; /**< list of three axis force torque
                                                                 sensors attached to the bridge */
    std::vector<std::string> cartesianWrenchesList; /**< list of cartesian wrench streams attached
                                                       to the bridge */
    std::vector<std::string> temperatureSensorsList; /**< list of temperature sensors attached to the bridge */
};

/**
 * Meta data struct to hold list of sensors and configured options
 * available from the Sensor bridge interface
 */
struct SensorBridgeMetaData
{
    SensorLists sensorsList;
    SensorBridgeOptions bridgeOptions;
};

/**
 * Sensor bridge interface.
 */
class ISensorBridge
{
public:
    using unique_ptr = std::unique_ptr<ISensorBridge>;

    using shared_ptr = std::shared_ptr<ISensorBridge>;

    using weak_ptr = std::weak_ptr<ISensorBridge>;

    using OptionalDoubleRef = std::optional<std::reference_wrapper<double>>;

    using Vector12d = Eigen::Matrix<double, 12, 1>;
    using Vector6d = Eigen::Matrix<double, 6, 1>;

    /**
     * Get joints list
     * @param[out] jointsList list of joints attached to the bridge
     * @return  true/false in case of success/failure
     */
    virtual bool getJointsList(std::vector<std::string>& jointsList)
    {
        return false;
    };

    /**
     * Get linear accelerometers
     * @param[out] linearAccelerometersList list of linear accelerometers attached to the bridge
     * @return  true/false in case of success/failure
     */
    virtual bool getLinearAccelerometersList(std::vector<std::string>& linearAccelerometersList)
    {
        return false;
    };

    /**
     * Get gyroscopes
     * @param[out] gyroscopesList list of gyroscopes attached to the bridge
     * @return  true/false in case of success/failure
     */
    virtual bool getGyroscopesList(std::vector<std::string>& gyroscopesList)
    {
        return false;
    };

    /**
     * Get orientation sensors
     * @param[out] orientationSensorsList list of orientation sensors attached to the bridge
     * @return  true/false in case of success/failure
     */
    virtual bool getOrientationSensorsList(std::vector<std::string>& orientationSensorsList)
    {
        return false;
    };

    /**
     * Get magnetometers sensors
     * @param[out] magnetometersList list of magnetometers attached to the bridge
     * @return  true/false in case of success/failure
     */
    virtual bool getMagnetometersList(std::vector<std::string>& magnetometersList)
    {
        return false;
    };

    /**
     * Get 6 axis FT sensors
     * @param[out] sixAxisForceTorqueSensorsList list of 6 axis force torque sensors attached to the
     * bridge
     * @return  true/false in case of success/failure
     */
    virtual bool
    getSixAxisForceTorqueSensorsList(std::vector<std::string>& sixAxisForceTorqueSensorsList)
    {
        return false;
    };

    /**
     * Get 6 axis FT sensors
     * @param[out] threeAxisForceTorqueSensorsList list of 3 axis force torque sensors attached to
     * the bridge
     * @return  true/false in case of success/failure
     */
    virtual bool
    getThreeAxisForceTorqueSensorsList(std::vector<std::string>& threeAxisForceTorqueSensorsList)
    {
        return false;
    };

    /**
     * Get cartesian wrenches
     * @param[out] cartesianWrenchesList list of cartesian wrenches attached to the bridge
     * @return  true/false in case of success/failure
     */
    virtual bool getCartesianWrenchesList(std::vector<std::string>& cartesianWrenchesList)
    {
        return false;
    };

    /**
     * Get temperature sensors
     * @param[out] cartesianWrenchesList list of cartesian wrenches attached to the bridge
     * @return  true/false in case of success/failure
     */
    virtual bool getTemperatureSensorsList(std::vector<std::string>& temperatureSensorsList)
    {
        return false;
    };

    /**
     * Get joint position  in radians
     * @param[in] jointName name of the joint
     * @param[out] jointPosition joint position in radians
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @return true/false in case of success/failure
     */
    virtual bool getJointPosition(const std::string& jointName,
                                  double& jointPosition,
                                  OptionalDoubleRef receiveTimeInSeconds = {})
    {
        return false;
    };

    /**
     * Get all joints' positions in radians
     * @param[out] jointPositions all joints' position in radians
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @warning the size is decided at the configuration and remains fixed,
     * and internal checks must be done at the implementation level by the Derived class.
     * This means that the user must pass a resized argument "jointPositions" to this method
     * @return true/false in case of success/failure
     */
    virtual bool getJointPositions(Eigen::Ref<Eigen::VectorXd> jointPositions,
                                   OptionalDoubleRef receiveTimeInSeconds = {})
    {
        return false;
    };

    /**
     * Get joint velocity in rad/s
     * @param[in] jointName name of the joint
     * @param[out] jointVelocity joint velocity in radians per second
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @return true/false in case of success/failure
     */
    virtual bool getJointVelocity(const std::string& jointName,
                                  double& jointVelocity,
                                  OptionalDoubleRef receiveTimeInSeconds = {})
    {
        return false;
    };

    /**
     * Get all joints' velocities in rad/s
     * @param[out] jointVelocties all joints' velocities in radians per second
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @warning the size is decided at the configuration and remains fixed,
     * and internal checks must be done at the implementation level by the Derived class.
     * This means that the user must pass a resized argument "jointVelocties" to this method
     * @return true/false in case of success/failure
     */
    virtual bool getJointVelocities(Eigen::Ref<Eigen::VectorXd> jointVelocties,
                                    OptionalDoubleRef receiveTimeInSeconds = {})
    {
        return false;
    };

    /**
     * Get joint acceleration in rad/s^2
     * @param[in] jointName name of the joint
     * @param[out] jointAcceleration joint acceleration in radians per second squared
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @return true/false in case of success/failure
     */
    virtual bool getJointAcceleration(const std::string& jointName,
                                      double& jointAcceleration,
                                      OptionalDoubleRef receiveTimeInSeconds = {})
    {
        return false;
    };

    /**
     * Get all joints' accelerations in rad/s^2
     * @param[out] jointAccelerations all joints' accelerations in radians per second squared
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @warning the size is decided at the configuration and remains fixed,
     * and internal checks must be done at the implementation level by the Derived class.
     * This means that the user must pass a resized argument "jointAccelerations" to this method
     * @return true/false in case of success/failure
     */
    virtual bool getJointAccelerations(Eigen::Ref<Eigen::VectorXd> jointAccelerations,
                                       OptionalDoubleRef receiveTimeInSeconds = {})
    {
        return false;
    };

    /**
     * Get linear accelerometer measurement in m/s^2
     * @param[in] accName name of the linear accelerometer
     * @param[out] accMeasurement linear accelerometer measurements of size 3
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @return true/false in case of success/failure
     */
    virtual bool getLinearAccelerometerMeasurement(const std::string& accName,
                                                   Eigen::Ref<Eigen::Vector3d> accMeasurement,
                                                   OptionalDoubleRef receiveTimeInSeconds = {})
    {
        return false;
    };

    /**
     * Get gyroscope measurement in rad/s
     * @param[in] gyroName name of the gyroscope
     * @param[out] gyroMeasurement gyroscope measurements of size 3
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @return true/false in case of success/failure
     */
    virtual bool getGyroscopeMeasure(const std::string& gyroName,
                                     Eigen::Ref<Eigen::Vector3d> gyroMeasurement,
                                     OptionalDoubleRef receiveTimeInSeconds = {})
    {
        return false;
    };

    /**
     * Get orientation sensor measurement in radians as roll pitch yaw Euler angles
     * @param[in] rpyName name of the orientation sensor
     * @param[out] rpyMeasurement rpy measurements of size 3
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @return true/false in case of success/failure
     */
    virtual bool getOrientationSensorMeasurement(const std::string& rpyName,
                                                 Eigen::Ref<Eigen::Vector3d> rpyMeasurement,
                                                 OptionalDoubleRef receiveTimeInSeconds = {})
    {
        return false;
    };

    /**
     * Get magentometer measurement in tesla
     * @param[in] magName name of the magnetometer
     * @param[out] magMeasurement magnetometer measurements of size 3
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @return true/false in case of success/failure
     */
    virtual bool getMagnetometerMeasurement(const std::string& magName,
                                            Eigen::Ref<Eigen::Vector3d> magMeasurement,
                                            OptionalDoubleRef receiveTimeInSeconds = {})
    {
        return false;
    };

    /**
     * Get six axis force torque measurement
     * @param[in] ftName name of the FT sensor
     * @param[out] ftMeasurement FT measurements of size 6 containing 3d forces and 3d torques
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @return true/false in case of success/failure
     */
    virtual bool getSixAxisForceTorqueMeasurement(const std::string& ftName,
                                                  Eigen::Ref<Vector6d> ftMeasurement,
                                                  OptionalDoubleRef receiveTimeInSeconds = {})
    {
        return false;
    };

    /**
     * Get three axis force-torque measurement containing normal force (N) and tangential moments
     * (Nm)
     * @param[in] ftName name of the FT sensor
     * @param[out] ftMeasurement FT measurements of size 3 containing tau_x tau_y and fz
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @return true/false in case of success/failure
     */
    virtual bool getThreeAxisForceTorqueMeasurement(const std::string& ftName,
                                                    Eigen::Ref<Eigen::Vector3d> ftMeasurement,
                                                    OptionalDoubleRef receiveTimeInSeconds = {})
    {
        return false;
    };

    /**
     * Get 6D end effector wrenches in N and Nm for forces and torques respectively
     * @param[in] cartesianWrenchName name of the end effector wrench
     * @param[out] cartesianWrenchMeasurement end effector wrench measurement of size 6
     * @param[out] rreceiveTimeInSeconds time at which the measurement was received
     * @return true/false in case of success/failure
     */
    virtual bool getCartesianWrench(const std::string& cartesianWrenchName,
                                    Eigen::Ref<Vector6d> cartesianWrenchMeasurement,
                                    OptionalDoubleRef receiveTimeInSeconds = {})
    {
        return false;
    };

    /**
     * Get temperature measurement
     * @param[in] temperatureSensorName name of the temperature sensor
     * @param[out] temperature temperature measurement
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @return true/false in case of success/failure
     */
    virtual bool getTemperature(const std::string& temperatureSensorName,
                                double& temperature,
                                OptionalDoubleRef receiveTimeInSeconds = {})
    {
        return false;
    };

    /**
     * Destructor
     */
    virtual ~ISensorBridge() = default;

protected:
    /**
     * Helper method to maintain SensorBridgeOptions struct by populating it from the configuration
     * parameters
     * @note the user may choose to use/not use this method depending on their requirements for the
     * implementation if the user chooses to not use the method, the implementation must simply
     * contain "return true;"
     * @param[in] handler  Parameters handler
     * @param[in] sensorBridgeOptions SensorBridgeOptions to hold the bridge options for streaming
     * sensor measurements
     */
    virtual bool populateSensorBridgeOptionsFromConfig(
        std::weak_ptr<const ParametersHandler::IParametersHandler> handler,
        SensorBridgeOptions& sensorBridgeOptions)
    {
        return true;
    };

    /**
     * Helper method to maintain SensorLists struct by populating it from the configuration
     * parameters
     * @note the user may choose to use/not use this method depending on their requirements for the
     * implementation if the user chooses to not use the method, the implementation must simply
     * contain "return true;"
     * @param[in] handler  Parameters handler
     * @param[in] sensorBridgeOptions configured object of SensorBridgeOptions
     * @param[in] sensorLists SensorLists object holding list of connected sensor devices
     */
    virtual bool
    populateSensorListsFromConfig(std::weak_ptr<const ParametersHandler::IParametersHandler> handler,
                                  const SensorBridgeOptions& sensorBridgeOptions,
                                  SensorLists& sensorLists)
    {
        return true;
    };

    /**
     * Helper method to maintain SensorBridgeMetaData struct by populating it from the configuration
     * parameters
     * @note the user may choose to use/not use this method depending on their requirements for the
     * implementation if the user chooses to not use the method, the implementation must simply
     * contain "return true;"
     * @param[in] handler  Parameters handler
     * @param[in] sensorBridgeMetaData configured object of SensorBridgeMetadata
     * @param[in] sensorLists SensorLists object holding list of connected sensor devices
     */
    virtual bool populateSensorBridgeMetaDataFromConfig(
        std::weak_ptr<const ParametersHandler::IParametersHandler> handler,
        SensorBridgeMetaData& sensorBridgeMetaData)
    {
        return true;
    };

public:
    /**
     * Get motor currents in ampere
     * @param[in] jointName name of the joint
     * @param[out] motorCurrent motor current in ampere
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @return true/false in case of success/failure
     */
    virtual bool getMotorCurrent(const std::string& jointName,
                                 double& motorCurrent,
                                 OptionalDoubleRef receiveTimeInSeconds = {})
    {
        return false;
    };

    /**
     * Get all motors' current in ampere
     * @param[out] motorCurrents all motors' current in ampere
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @warning the size is decided at the configuration and remains fixed,
     * and internal checks must be done at the implementation level by the Derived class.
     * This means that the user must pass a resized argument "motorCurrents" to this method
     * @return true/false in case of success/failure
     */
    virtual bool getMotorCurrents(Eigen::Ref<Eigen::VectorXd> motorCurrents,
                                  OptionalDoubleRef receiveTimeInSeconds = {})
    {
        return false;
    };

    /**
     * Get motor PWM
     * @param[in] jointName name of the joint
     * @param[out] motorPWM motor PWM
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @return true/false in case of success/failure
     */
    virtual bool getMotorPWM(const std::string& jointName,
                             double& motorPWM,
                             OptionalDoubleRef receiveTimeInSeconds = {})
    {
        return false;
    };

    /**
     * Get all motors' PWM
     * @param[out] motorPWMs all motors' PWM
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @warning the size is decided at the configuration and remains fixed,
     * and internal checks must be done at the implementation level by the Derived class.
     * This means that the user must pass a resized argument "motorPWMs" to this method
     * @return true/false in case of success/failure
     */
    virtual bool
    getMotorPWMs(Eigen::Ref<Eigen::VectorXd> motorPWMs, OptionalDoubleRef receiveTimeInSeconds = {})
    {
        return false;
    };

    /**
     * Get joint torques in Nm
     * @param[in] jointName name of the joint
     * @param[out] jointTorque motor torque in Nm
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @return true/false in case of success/failure
     */
    virtual bool getJointTorque(const std::string& jointName,
                                double& jointTorque,
                                OptionalDoubleRef receiveTimeInSeconds = {})
    {
        return false;
    };

    /**
     * Get all joints' torque in Nm
     * @param[out] jointTorques all motors' torque in Nm
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @warning the size is decided at the configuration and remains fixed,
     * and internal checks must be done at the implementation level by the Derived class.
     * This means that the user must pass a resized argument "jointTorques" to this method
     * @return true/false in case of success/failure
     */
    virtual bool getJointTorques(Eigen::Ref<Eigen::VectorXd> jointTorques,
                                 OptionalDoubleRef receiveTimeInSeconds = {})
    {
        return false;
    };

    /**
     * Get pid position in rad
     * @param[in] jointName name of the joint
     * @param[out] pidPosition pid position in radians
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @return true/false in case of success/failure
     */
    virtual bool getPidPosition(const std::string& jointName,
                                double& pidPosition,
                                OptionalDoubleRef receiveTimeInSeconds = {})
    {
        return false;
    };

    /**
     * Get all pid positions in rad
     * @param[out] pidPositions all pid positions in radians
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @warning the size is decided at the configuration and remains fixed,
     * and internal checks must be done at the implementation level by the Derived class.
     * This means that the user must pass a resized argument "pidPositions" to this method
     * @return true/false in case of success/failure
     */
    virtual bool getPidPositions(Eigen::Ref<Eigen::VectorXd> pidPositions,
                                 OptionalDoubleRef receiveTimeInSeconds = {})
    {
        return false;
    };

    /**
     * Get pid position error in rad
     * @param[in] jointName name of the joint
     * @param[out] pidPositionError pid position error in radians
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @return true/false in case of success/failure
     */
    virtual bool getPidPositionError(const std::string& jointName,
                                     double& pidPositionError,
                                     OptionalDoubleRef receiveTimeInSeconds = {})
    {
        return false;
    };

    /**
     * Get all pid position errors in rad
     * @param[out] pidPositionErrors all pid position errors in radians
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @warning the size is decided at the configuration and remains fixed,
     * and internal checks must be done at the implementation level by the Derived class.
     * This means that the user must pass a resized argument "pidPositionErrors" to this method
     * @return true/false in case of success/failure
     */
    virtual bool getPidPositionErrors(Eigen::Ref<Eigen::VectorXd> pidPositionErrors,
                                      OptionalDoubleRef receiveTimeInSeconds = {})
    {
        return false;
    };

    /**
     * Get motor position in rad
     * @param[in] jointName name of the joint
     * @param[out] motorPosition motor position in radians
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @return true/false in case of success/failure
     */
    virtual bool getMotorPosition(const std::string& jointName,
                                  double& motorPosition,
                                  OptionalDoubleRef receiveTimeInSeconds = {})
    {
        return false;
    };

    /**
     * Get all motors' positions in rad
     * @param[out] motorPositions all motors' position in radians
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @warning the size is decided at the configuration and remains fixed,
     * and internal checks must be done at the implementation level by the Derived class.
     * This means that the user must pass a resized argument "motorPositions" to this method
     * @return true/false in case of success/failure
     */
    virtual bool getMotorPositions(Eigen::Ref<Eigen::VectorXd> motorPositions,
                                   OptionalDoubleRef receiveTimeInSeconds = {})
    {
        return false;
    };

    /**
     * Get motor velocity in rad/s
     * @param[in] jointName name of the joint
     * @param[out] motorVelocity motor velocity in radians per second
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @return true/false in case of success/failure
     */
    virtual bool getMotorVelocity(const std::string& jointName,
                                  double& motorVelocity,
                                  OptionalDoubleRef receiveTimeInSeconds = {})
    {
        return false;
    };

    /**
     * Get all motors' velocities in rad/s
     * @param[out] motorVelocties all motors' velocities in radians per second
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @warning the size is decided at the configuration and remains fixed,
     * and internal checks must be done at the implementation level by the Derived class.
     * This means that the user must pass a resized argument "motorVelocties" to this method
     * @return true/false in case of success/failure
     */
    virtual bool getMotorVelocities(Eigen::Ref<Eigen::VectorXd> motorVelocties,
                                    OptionalDoubleRef receiveTimeInSeconds = {})
    {
        return false;
    };

    /**
     * Get motor acceleration in rad/s^2
     * @param[in] jointName name of the joint
     * @param[out] motorAcceleration motor acceleration in radians per second squared
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @return true/false in case of success/failure
     */
    virtual bool getMotorAcceleration(const std::string& jointName,
                                      double& motorAcceleration,
                                      OptionalDoubleRef receiveTimeInSeconds = {})
    {
        return false;
    };

    /**
     * Get all motors' accelerations in rad/s^2
     * @param[out] motorAccelerations all motors' accelerations in radians per second squared
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @warning the size is decided at the configuration and remains fixed,
     * and internal checks must be done at the implementation level by the Derived class.
     * This means that the user must pass a resized argument "motorAccelerations" to this method
     * @return true/false in case of success/failure
     */
    virtual bool getMotorAccelerations(Eigen::Ref<Eigen::VectorXd> motorAccelerations,
                                       OptionalDoubleRef receiveTimeInSeconds = {})
    {
        return false;
    };

    /**
     * Get motor temperature in degrees celsius
     * @param[out] motorTemperature motor temperature in degrees celsius
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @warning the size is decided at the configuration and remains fixed,
     * and internal checks must be done at the implementation level by the Derived class.
     * This means that the user must pass a resized argument "motorTemperatures" to this method
     * @return true/false in case of success/failure
     */
    virtual bool getMotorTemperatures(Eigen::Ref<Eigen::VectorXd> motorTemperatures,
                                       OptionalDoubleRef receiveTimeInSeconds = {})
    {
        return false;
    };

    /**
     * Get motor temperature in degrees celsius
     * @param[in] jointName name of the joint
     * @param[out] motorTemperature motor temperature in degrees celsius
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @return true/false in case of success/failure
     */
    virtual bool getMotorTemperature(const std::string& jointName,
                                     double& motorTemperature,
                                     OptionalDoubleRef receiveTimeInSeconds = {})
    {
        return false;
    };
};
} // namespace RobotInterface
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_ROBOT_INTERFACE_ISENSOR_BRIDGE_H
