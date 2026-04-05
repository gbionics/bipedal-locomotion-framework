/**
 * @file YarpSensorBridge.h
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_ROBOT_INTERFACE_YARP_SENSOR_BRIDGE_H
#define BIPEDAL_LOCOMOTION_ROBOT_INTERFACE_YARP_SENSOR_BRIDGE_H

// std
#include <functional>
#include <memory>
#include <optional>

// YARP
#include <yarp/dev/PolyDriverList.h>

#include <BipedalLocomotion/RobotInterface/ISensorBridge.h>
#include <BipedalLocomotion/System/Source.h>

namespace BipedalLocomotion
{
namespace RobotInterface
{

// clang-format off
/**
 * YarpSensorBridge Yarp implementation of the ISensorBridge interface
 * Currently available interfaces
 * - Remapped Remote Control Board for joint states
 * - MAS-based inertials (gyroscopes, accelerometers, orientation sensors, magnetometers)
 * - Whole Body Dynamics Estimated end effector wrenches through a generic sensor interface
 * - Force Torque Sensors through remapped multiple analog sensor interface
 * - Depth Cameras through RGBD sensor interface
 * - Camera images through OpenCV Grabber interface
 *
 * The YarpSensorBridge expects a list of device drivers through the yarp::dev::PolyDriverList object.
 * Each PolyDriver object in the list is compared with the configured sensor names and the assumptions listed below
 * to infer the sensor types and relevant interfaces in order to to read the relevant data.
 *
 * MAJOR ASSUMPTIONS
 * - Every sensor unit(device driver) attached to this Bridge is identified by a unique name
 * - A single instance of a remote control board remapper and a multiple analog sensor remapper is expected if the suer wants to use the control board interfaces and multiple analog sensor interfaces
 * - Any generic sensor interface with channel dimensions of 6 is considered to be a cartesian wrench interface
 * - Any generic sensor interface with channel dimensions of 12 is considered as a IMU interface (server inertial)
 * - Any analog sensor interface with channel dimensions of 6 is considered as a force torque sensor interface
 * - The current internal design (read all sensors in a serial fashion) may not be suitable for a heavy measurement set
 *
 * The parameters for writing the configuration file for this class is given as,
 * |     Group                  |         Parameter               | Type              |                   Description                   |
 * |:--------------------------:|:-------------------------------:|:-----------------:|:---------------------------------------------- :|
 * |                            |check_for_nan                    | boolean           |flag to activate checking for NANs in the incoming measurement buffers, not applicable for images|
 * |                            |stream_joint_states              | boolean           |Flag to activate the attachment to remapped control boards for joint states reading     |
 * |                            |stream_inertials                 | boolean           |Flag to activate the attachment to IMU sensor devices       |
 * |                            |stream_cartesian_wrenches        | boolean           |Flag to activate the attachment to Cartesian wrench related devices       |
 * |                            |stream_forcetorque_sensors       | boolean           |Flag to activate the attachment to six axis FT sensor devices       |
 * |                            |stream_cameras                   | boolean           |Flag to activate the attachment to Cameras devices       |
 * |                            |stream_pids                      | boolean           |Flag to activate the attachment to remapped control boards for pids reading      |
 * |                            |stream_motor_states              | boolean           |Flag to activate the attachment to remapped control boards for motor states reading      |
 * |                            |stream_motor_PWM                 | boolean           |Flag to activate the attachment to remapped control boards for PWM reading      |
 * |                            |stream_temperatures              | boolean           |Flag to activate the attachment to MAS temperature sensors      |
 * |                            |stream_joint_accelerations       | boolean           |Flag to activate the attachment to joint accelerations from encoders      |
 * |                            |stream_motor_temperature         | boolean           |Flag to activate the attachment to motor temperature sensors      |
 * |RemoteControlBoardRemapper  |                                 |                   |Expects only one remapped remotecontrolboard device attached to it, if there multiple remote control boards, then  use a remapper to create a single remotecontrolboard |
 * |                            |joints_list                      | vector of strings |This parameter is **optional**. The joints list used to open the remote control board remapper. If the list is not passed, the order of the joint stored in the PolyDriver is used       |
 * |InertialSensors             |                                 |                   |Expects inertials to be opened as a part of the multiple analog sensors remapper ("multipleanalogsensorsremapper") |
 * |                            |gyroscopes_list                  | vector of strings |list of the names of devices opened with ThreeAxisGyroscope interface remapped through the "multipleanalogsensorsremapper" interfaces and having a channel dimension of 3  |
 * |                            |accelerometers_list              | vector of strings |list of the names of devices opened with ThreeAxisLinearAccelerometers interface remapped through the "multipleanalogsensorsremapper" interfaces and having a channel dimension of 3 |
 * |                            |orientation_sensors_list         | vector of strings |list of the names of devices opened with OrientationSensors interface remapped through the "multipleanalogsensorsremapper" interfaces and having a channel dimension of 3 |
 * |                            |magnetometers_list               | vector of strings |list of the names of devices opened with ThreeAxisMagnetometer interface remapped through the "multipleanalogsensorsremapper" interfaces and having a channel dimension of 3 |
 * |CartesianWrenches           |                                 |                   |Expects the devices wrapping the cartesian wrenches ports to be opened as "genericSensorClient" device and have a channel dimension of 6                      |
 * |                            |cartesian_wrenches_list          | vector of strings |list of the names of devices opened as genericSensorClient device and having a channel dimension of 6      |
 * |SixAxisForceTorqueSensors   |                                 |                   |Expects the Six axis FT sensors to be opened with SixAxisForceTorqueSensors interface remapped through multiple analog sensors remapper ("multipleanalogsensorsremapper")|
 * |                            |sixaxis_forcetorque_sensors_list | vector of strings |list of six axis FT sensors |
 * |TemperatureSensors          |                                 |                   |Expects the temperature sensors to be opened with TemperatureSensors interface remapped through multiple analog sensors remapper|
 * |                            |temperature_sensors_list         | vector of strings |list containing the devices opened with TemperatureSensors interface      |
 */
// clang-format on

class YarpSensorBridge : public ISensorBridge,
                         public BipedalLocomotion::System::Source<SensorBridgeMetaData>
{
public:
    /**
     * Constructor
     */
    YarpSensorBridge();

    /**
     * Destructor
     */
    ~YarpSensorBridge();

    /**
     * Initialize estimator
     * @param[in] handler Parameters handler
     */
    bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler) final;

    /**
     * Set the list of device drivers from which the sensor measurements need to be streamed
     * @param deviceDriversList device drivers holding the pointer to sensor interfaces
     * @return True/False in case of success/failure.
     */
    bool setDriversList(const yarp::dev::PolyDriverList& deviceDriversList);

    /**
     * @brief Advance the internal state. This may change the value retrievable from get().
     * @return True if the advance is successful.
     */
    bool advance() final;

    /**
     * @brief Determines the validity of the object retrieved with get()
     * @return True if the object is valid, false otherwise.
     */
    bool isOutputValid() const final;

    /**
     * @brief list of sensors that was failed to be read in the current advance() step
     * @return list of sensors as a vector of strings
     */
    std::vector<std::string> getFailedSensorReads() const;

    /**
     * @brief Get the object.
     * @return a const reference of the requested object.
     */
    const SensorBridgeMetaData& getOutput() const final;

    /**
     * Get joints list
     * @param[out] jointsList list of joints attached to the bridge
     * @return  true/false in case of success/failure
     */
    bool getJointsList(std::vector<std::string>& jointsList) final;

    bool getLinearAccelerometersList(std::vector<std::string>& linearAccelerometersList) final;

    /**
     * Get gyroscopes
     * @param[out] gyroscopesList list of gyroscopes attached to the bridge
     * @return  true/false in case of success/failure
     */
    bool getGyroscopesList(std::vector<std::string>& gyroscopesList) final;

    /**
     * Get orientation sensors
     * @param[out] orientationSensorsList list of orientation sensors attached to the bridge
     * @return  true/false in case of success/failure
     */
    bool getOrientationSensorsList(std::vector<std::string>& orientationSensorsList) final;

    /**
     * Get magnetometers sensors
     * @param[out] magnetometersList list of magnetometers attached to the bridge
     * @return  true/false in case of success/failure
     */
    bool getMagnetometersList(std::vector<std::string>& magnetometersList) final;

    /**
     * Get 6 axis FT sensors
     * @param[out] sixAxisForceTorqueSensorsList list of 6 axis force torque sensors attached to the
     * bridge
     * @return  true/false in case of success/failure
     */
    bool
    getSixAxisForceTorqueSensorsList(std::vector<std::string>& sixAxisForceTorqueSensorsList) final;

    /**
     * Get 6 axis FT sensors
     * @param[out] threeAxisForceTorqueSensorsList list of 3 axis force torque sensors attached to
     * the bridge
     * @return  true/false in case of success/failure
     */
    bool getThreeAxisForceTorqueSensorsList(
        std::vector<std::string>& threeAxisForceTorqueSensorsList) final;

    /**
     * Get cartesian wrenches
     * @param[out] cartesianWrenchesList list of cartesian wrenches attached to the bridge
     * @return  true/false in case of success/failure
     */
    bool getCartesianWrenchesList(std::vector<std::string>& cartesianWrenchesList) final;

    /**
     * Get temperature sensors
     * @param[out] cartesianWrenchesList list of cartesian wrenches attached to the bridge
     * @return  true/false in case of success/failure
     */
    bool getTemperatureSensorsList(std::vector<std::string>& temperatureSensorsList) final;

    const std::vector<std::string>& getJointsList() const;

    const std::vector<std::string>& getLinearAccelerometersList() const;

    const std::vector<std::string>& getGyroscopesList() const;

    const std::vector<std::string>& getOrientationSensorsList() const;

    const std::vector<std::string>& getMagnetometersList() const;

    const std::vector<std::string>& getSixAxisForceTorqueSensorsList() const;

    const std::vector<std::string>& getTemperatureSensorsList() const;

    const std::vector<std::string>& getCartesianWrenchesList() const;

    /**
     * Get joint position  in radians
     * @param[in] jointName name of the joint
     * @param[out] jointPosition joint position in radians
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @return true/false in case of success/failure
     */
    bool getJointPosition(const std::string& jointName,
                          double& jointPosition,
                          OptionalDoubleRef receiveTimeInSeconds = {}) final;

    /**
     * Get all joints' positions in radians
     * @param[out] jointPositions all joints' position in radians
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @warning the size is decided at the configuration and remains fixed,
     * and internal checks must be done at the implementation level by the Derived class.
     * This means that the user must pass a resized argument "jointPositions" to this method
     * @return true/false in case of success/failure
     */
    bool getJointPositions(Eigen::Ref<Eigen::VectorXd> jointPositions,
                           OptionalDoubleRef receiveTimeInSeconds = {}) final;

    /**
     * Get joint velocity in rad/s
     * @param[in] jointName name of the joint
     * @param[out] jointVelocity joint velocity in radians per second
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @return true/false in case of success/failure
     */
    bool getJointVelocity(const std::string& jointName,
                          double& jointVelocity,
                          OptionalDoubleRef receiveTimeInSeconds = {}) final;

    /**
     * Get all joints' velocities in rad/s
     * @param[out] jointVelocties all joints' velocities in radians per second
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @warning the size is decided at the configuration and remains fixed,
     * and internal checks must be done at the implementation level by the Derived class.
     * This means that the user must pass a resized argument "jointVelocties" to this method
     * @return true/false in case of success/failure
     */
    bool getJointVelocities(Eigen::Ref<Eigen::VectorXd> jointVelocties,
                            OptionalDoubleRef receiveTimeInSeconds = {}) final;

    /**
     * Get joint acceleration in rad/s^2
     * @param[in] jointName name of the joint
     * @param[out] jointAcceleration joint acceleration in radians per second squared
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @return true/false in case of success/failure
     */
    bool getJointAcceleration(const std::string& jointName,
                              double& jointAcceleration,
                              OptionalDoubleRef receiveTimeInSeconds = {}) final;

    /**
     * Get all joints' accelerations in rad/s^2
     * @param[out] jointAccelerations all joints' accelerations in radians per second squared
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @warning the size is decided at the configuration and remains fixed,
     * and internal checks must be done at the implementation level by the Derived class.
     * This means that the user must pass a resized argument "jointAccelerations" to this method
     * @return true/false in case of success/failure
     */
    bool getJointAccelerations(Eigen::Ref<Eigen::VectorXd> jointAccelerations,
                               OptionalDoubleRef receiveTimeInSeconds = {}) final;

    bool getLinearAccelerometerMeasurement(const std::string& accName,
                                           Eigen::Ref<Eigen::Vector3d> accMeasurement,
                                           OptionalDoubleRef receiveTimeInSeconds = {}) final;

    /**
     * Get gyroscope measurement in rad/s
     * @param[in] gyroName name of the gyroscope
     * @param[out] gyroMeasurement gyroscope measurements of size 3
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @return true/false in case of success/failure
     */
    bool getGyroscopeMeasure(const std::string& gyroName,
                             Eigen::Ref<Eigen::Vector3d> gyroMeasurement,
                             OptionalDoubleRef receiveTimeInSeconds = {}) final;

    /**
     * Get orientation sensor measurement in radians as roll pitch yaw Euler angles
     * @param[in] rpyName name of the orientation sensor
     * @param[out] rpyMeasurement rpy measurements of size 3
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @return true/false in case of success/failure
     */
    bool getOrientationSensorMeasurement(const std::string& rpyName,
                                         Eigen::Ref<Eigen::Vector3d> rpyMeasurement,
                                         OptionalDoubleRef receiveTimeInSeconds = {}) final;

    /**
     * Get magentometer measurement in tesla
     * @param[in] magName name of the magnetometer
     * @param[out] magMeasurement magnetometer measurements of size 3
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @return true/false in case of success/failure
     */
    bool getMagnetometerMeasurement(const std::string& magName,
                                    Eigen::Ref<Eigen::Vector3d> magMeasurement,
                                    OptionalDoubleRef receiveTimeInSeconds = {}) final;

    /**
     * Get six axis force torque measurement
     * @param[in] ftName name of the FT sensor
     * @param[out] ftMeasurement FT measurements of size 6 containing 3d forces and 3d torques
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @return true/false in case of success/failure
     */
    bool getSixAxisForceTorqueMeasurement(const std::string& ftName,
                                          Eigen::Ref<Vector6d> ftMeasurement,
                                          OptionalDoubleRef receiveTimeInSeconds = {}) final;

    /**
     * Get three axis force-torque measurement containing normal force (N) and tangential moments
     * (Nm)
     * @param[in] ftName name of the FT sensor
     * @param[out] ftMeasurement FT measurements of size 3 containing tau_x tau_y and fz
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @return true/false in case of success/failure
     */
    bool getThreeAxisForceTorqueMeasurement(const std::string& ftName,
                                            Eigen::Ref<Eigen::Vector3d> ftMeasurement,
                                            OptionalDoubleRef receiveTimeInSeconds = {}) final;

    /**
     * Get 6D end effector wrenches in N and Nm for forces and torques respectively
     * @param[in] cartesianWrenchName name of the end effector wrench
     * @param[out] cartesianWrenchMeasurement end effector wrench measurement of size 6
     * @param[out] rreceiveTimeInSeconds time at which the measurement was received
     * @return true/false in case of success/failure
     */
    bool getCartesianWrench(const std::string& cartesianWrenchName,
                            Eigen::Ref<Vector6d> cartesianWrenchMeasurement,
                            OptionalDoubleRef receiveTimeInSeconds = {}) final;

    /**
     * Get temperature measurement
     * @param[in] temperatureSensorName name of the temperature sensor
     * @param[out] temperature temperature measurement
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @return true/false in case of success/failure
     */
    bool getTemperature(const std::string& temperatureSensorName,
                        double& temperature,
                        OptionalDoubleRef receiveTimeInSeconds = {}) final;

    /**
     * Get motor current in ampere
     * @param[in] jointName name of the joint
     * @param[out] motorCurrent motor current in ampere
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @return true/false in case of success/failure
     */
    bool getMotorCurrent(const std::string& jointName,
                         double& motorCurrent,
                         OptionalDoubleRef receiveTimeInSeconds = {}) final;

    /**
     * Get all motors' currents in ampere
     * @param[out] motorCurrents all motors' current in ampere
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @warning the size is decided at the configuration and remains fixed,
     * and internal checks must be done at the implementation level by the Derived class.
     * This means that the user must pass a resized argument "motorCurrents" to this method
     * @return true/false in case of success/failure
     */
    bool getMotorCurrents(Eigen::Ref<Eigen::VectorXd> motorCurrents,
                          OptionalDoubleRef receiveTimeInSeconds = {}) final;


    /**
     * Get motor temperature in degrees celsius
      * @param[out] motorTemperature motor temperature in degrees celsius
      * @param[out] receiveTimeInSeconds time at which the measurement was received
      * @warning the size is decided at the configuration and remains fixed,
      * and internal checks must be done at the implementation level by the Derived class.
      * This means that the user must pass a resized argument "motorTemperatures" to this method
      * @return true/false in case of success/failure
      */
    bool getMotorTemperatures(Eigen::Ref<Eigen::VectorXd> motorTemperatures,
                              OptionalDoubleRef receiveTimeInSeconds = {}) final;


    /**
     * Get motor temperature in degrees celsius
     * @param[in] jointName name of the joint
     * @param[out] motorTemperature motor temperature in degrees celsius
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @return true/false in case of success/failure
     * @return true/false in case of success/failure
     */
    bool getMotorTemperature(const std::string& jointName,
                             double& motorTemperature,
                             OptionalDoubleRef receiveTimeInSeconds = {}) final;

    /**
     * Get motor PWM
     * @param[in] jointName name of the joint
     * @param[out] motorPWM motor PWM
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @return true/false in case of success/failure
     */
    bool getMotorPWM(const std::string& jointName,
                     double& motorPWM,
                     OptionalDoubleRef receiveTimeInSeconds = {}) final;

    /**
     * Get all motors' PWMs
     * @param[out] motorPWMs all motors' PWM in ampere
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @warning the size is decided at the configuration and remains fixed,
     * and internal checks must be done at the implementation level by the Derived class.
     * This means that the user must pass a resized argument "motorPWMs" to this method
     * @return true/false in case of success/failure
     */
    bool getMotorPWMs(Eigen::Ref<Eigen::VectorXd> motorPWMs,
                      OptionalDoubleRef receiveTimeInSeconds = {}) final;

    /**
     * Get motor torque in Nm
     * @param[in] jointName name of the joint
     * @param[out] jointTorque motor torque in Nm
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @return true/false in case of success/failure
     */
    bool getJointTorque(const std::string& jointName,
                        double& jointTorque,
                        OptionalDoubleRef receiveTimeInSeconds = {}) final;

    /**
     * Get all motors' torques in Nm
     * @param[out] jointTorques all motors' torque in Nm
     * @param[out] receiveTimeInSeconds time at which the measurement was receivedW
     * @warning the size is decided at the configuration and remains fixed,
     * and internal checks must be done at the implementation level by the Derived class.
     * This means that the user must pass a resized argument "jointTorques" to this method
     * @return true/false in case of success/failure
     */
    bool getJointTorques(Eigen::Ref<Eigen::VectorXd> jointTorques,
                         OptionalDoubleRef receiveTimeInSeconds = {}) final;

    /**
     * Get pid position in rad
     * @param[in] jointName name of the joint
     * @param[out] pidPosition pid position in radians
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @return true/false in case of success/failure
     */
    virtual bool getPidPosition(const std::string& jointName,
                                double& pidPosition,
                                OptionalDoubleRef receiveTimeInSeconds = {}) final;

    /**
     * Get all pid positions in rad
     * @param[out] pidPositions all pid positions in radians
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @warning the size is decided at the configuration and remains fixed,
     * and internal checks must be done at the implementation level by the Derived class.
     * This means that the user must pass a resized argument "pidPositions" to this methodW
     * @return true/false in case of success/failure
     */
    virtual bool getPidPositions(Eigen::Ref<Eigen::VectorXd> pidPositions,
                                 OptionalDoubleRef receiveTimeInSeconds = {}) final;

    /**
     * Get pid position error in rad
     * @param[in] jointName name of the joint
     * @param[out] pidPositionError pid position error in radians
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @return true/false in case of success/failure
     */
    virtual bool getPidPositionError(const std::string& jointName,
                                     double& pidPositionError,
                                     OptionalDoubleRef receiveTimeInSeconds = {}) final;

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
                                      OptionalDoubleRef receiveTimeInSeconds = {}) final;

    /**
     * Get motor position in rad
     * @param[in] jointName name of the joint
     * @param[out] motorPosition motor position in radians
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @return true/false in case of success/failure
     */
    virtual bool getMotorPosition(const std::string& jointName,
                                  double& motorPosition,
                                  OptionalDoubleRef receiveTimeInSeconds = {}) final;

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
                                   OptionalDoubleRef receiveTimeInSeconds = {}) final;

    /**
     * Get motor velocity in rad/s
     * @param[in] jointName name of the joint
     * @param[out] motorVelocity motor velocity in radians per second
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @return true/false in case of success/failure
     */
    virtual bool getMotorVelocity(const std::string& jointName,
                                  double& motorVelocity,
                                  OptionalDoubleRef receiveTimeInSeconds = {}) final;

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
                                    OptionalDoubleRef receiveTimeInSeconds = {}) final;

    /**
     * Get motor acceleration in rad/s^2
     * @param[in] jointName name of the joint
     * @param[out] motorAcceleration motor acceleration in radians per second squared
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @return true/false in case of success/failure
     */
    virtual bool getMotorAcceleration(const std::string& jointName,
                                      double& motorAcceleration,
                                      OptionalDoubleRef receiveTimeInSeconds = {}) final;

    /**
     * Get all motors' accelerations in rad/s
     * @param[out] motorAccelerations all motors' accelerations in radians per second squared
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @warning the size is decided at the configuration and remains fixed,
     * and internal checks must be done at the implementation level by the Derived class.
     * This means that the user must pass a resized argument "motorAccelerations" to this method
     * @return true/false in case of success/failure
     */
    virtual bool getMotorAccelerations(Eigen::Ref<Eigen::VectorXd> motorAccelerations,
                                       OptionalDoubleRef receiveTimeInSeconds = {}) final;

private:
    /** Private implementation */
    struct Impl;
    std::unique_ptr<Impl> m_pimpl;
};

} // namespace RobotInterface
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_ROBOT_INTERFACE_YARP_SENSOR_BRIDGE_H
