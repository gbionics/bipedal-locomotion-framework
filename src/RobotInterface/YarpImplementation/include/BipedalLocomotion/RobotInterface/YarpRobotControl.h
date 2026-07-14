/**
 * @file YarpRobotControl.h
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_ROBOT_INTERFACE_YARP_ROBOT_CONTROL_H
#define BIPEDAL_LOCOMOTION_ROBOT_INTERFACE_YARP_ROBOT_CONTROL_H

// std
#include <future>
#include <memory>
#include <optional>

// Eigen
#include <Eigen/Dense>

// YARP
#include <yarp/dev/PolyDriver.h>

#include <BipedalLocomotion/RobotInterface/IRobotControl.h>

namespace BipedalLocomotion
{
namespace RobotInterface
{

/**
 * YarpRobotControl Yarp implementation of the IRobotControl interface
 * @warning At the current stage only revolute joints are supported.
 */
class YarpRobotControl : public IRobotControl
{
    /** Private implementation */
    struct Impl;
    std::unique_ptr<Impl> m_pimpl;

public:
    /**
     * Constructor
     */
    YarpRobotControl();

    // clang-format off
    /**
     * Initialize the Interface
     * @param handler pointer to a parameter handler interface
     * @note the following parameters are required by the class
     * |             Parameter name             |   Type   |                                          Description                                         | Mandatory |
     * |:--------------------------------------:|:--------:|:--------------------------------------------------------------------------------------------:|:---------:|
     * |            `reading_timeout`           |   `int`  |    Timeout used while reading from the YARP interfaces in microseconds. (Positive Number)    |     No    |
     * |         `max_reading_attempts`         |   `int`  |      Max number of attempts used for reading from the YARP interfaces. (Positive Number)     |     No    |
     * |         `positioning_duration`         | `double` | Duration of the trajectory generated when the joint is controlled in position mode [seconds] |    Yes    |
     * |         `positioning_tolerance`        | `double` |                    Max Admissible error for position control joint [rad]                     |    Yes    |
     * | `position_direct_max_admissible_error` | `double` |                 Max admissible error for position direct control joint [rad]                 |    Yes    |
     * @return True/False in case of success/failure.
     // clang-format on
     */
    bool initialize(std::weak_ptr<ParametersHandler::IParametersHandler> handler) override;

    /**
     * Set the driver required to control the robot.
     * @param robotDevice device used to control the robot.
     * @return True/False in case of success/failure.
     */
    virtual bool setDriver(std::shared_ptr<yarp::dev::PolyDriver> robotDevice);

    /**
     * Check if the motion set through the position control mode ended.
     * @param[out] motionDone true if the motion ended.
     * @param[out] isTimerExpired true if the internal timer is expired or not.
     * @param[out] info vector containing the list of the joint whose motion did not finish yet.
     * @return True/False in case of success/failure.
     * @note If the timer is expired and the motion did not finish yet, there may be a problem
     * with the robot.
     */
    bool checkMotionDone(bool& motionDone,
                         bool& isTimeExpired,
                         std::vector<std::pair<std::string, double>>& info) final;

    /**
     * Set the control mode.
     * @param controlModes vector containing the control mode for each joint.
     * @return True/False in case of success/failure.
     * @warning At the current stage only revolute joints are supported.
     */
    bool setControlMode(const std::vector<IRobotControl::ControlMode>& controlModes) final;

    /**
     * Set the desired control mode.
     * @param controlMode a control mode for all the joints.
     * @return True/False in case of success/failure.
     * @warning Call this function if you want to control all the joint with the same control mode.
     */
    bool setControlMode(const IRobotControl::ControlMode& mode) final;

    /**
     * Set the control mode in an asynchronous thread.
     * @param controlModes vector containing the control mode for each joint.
     * @return An std::future object to a boolean True/False in case of success/failure.
     * @warning At the current stage only revolute joints are supported.
     * Since this function spawns a new thread, the invoking thread is not blocked.
     * Note that this function is not thread safe. You should check the future object status before
     * calling other functions like setReferences().
     */
    std::future<bool>
    setControlModeAsync(const std::vector<IRobotControl::ControlMode>& controlModes) final;

    /**
     * Set the desired control mode in an asynchronous thread.
     * @param controlMode a control mode for all the joints.
     * @return An std::future object to a boolean True/False in case of success/failure.
     * @warning Call this function if you want to control all the joint with the same control mode.
     * Since this function spawns a new thread, the invoking thread is not blocked.
     * Note that this function is not thread safe. You should check the future object status before
     * calling other functions like setReferences().
     */
    std::future<bool> setControlModeAsync(const IRobotControl::ControlMode& mode) final;

    /**
     * Set the desired reference.
     * @param desiredJointValues desired joint values.
     * @param controlModes vector containing the control mode for each joint.
     * @return True/False in case of success/failure.
     * @note In case of position control the values has to be expressed in rad, in case of velocity
     * control in rad/s. If the robot is controlled in torques, the desired joint values are
     * expressed in Nm. If the robot is controlled in PWM, the desired joint values are
     * between -100 and 100.
     * @warning At the current stage only revolute joints are supported.
     */
    bool
    setReferences(Eigen::Ref<const Eigen::VectorXd> desiredJointValues,
                  const std::vector<IRobotControl::ControlMode>& controlModes,
                  std::optional<Eigen::Ref<const Eigen::VectorXd>> currentJointValues = {}) final;

    /**
     * Set the desired reference.
     * @param desiredJointValues desired joint values.
     * @param controlMode a control mode for all the joints.
     * @return True/False in case of success/failure.
     * @note In case of position control the values has to be expressed in rad, in case of velocity
     * control in rad/s. If the robot is controlled in torques, the desired joint values are
     * expressed in Nm. If the robot is controlled in PWM, the desired joint values are
     * between -100 and 100.
     * @warning Call this function if you want to control all the joint with the same control mode.
     * Otherwise call setReferences(Eigen::Ref<const Eigen::VectorXd>, const
     * std::vector<IRobotControl::ControlMode>&).
     */
    bool
    setReferences(Eigen::Ref<const Eigen::VectorXd> desiredJointValues,
                  const IRobotControl::ControlMode& mode,
                  std::optional<Eigen::Ref<const Eigen::VectorXd>> currentJointValues = {}) final;

    /**
     * Get the list of the controlled joints
     * @return A vector containing the name of the controlled joints.
     */
    std::vector<std::string> getJointList() const final;

    /**
     * Get the actuated joints limits.
     * @param lowerLimits vector to be filled with the lower limits of the actuated joints.
     * @param upperLimits vector to be filled with the upper limits of the actuated joints.
     * @return True/False in case of success/failure.
     */
    bool getJointLimits(Eigen::Ref<Eigen::VectorXd> lowerLimits, Eigen::Ref<Eigen::VectorXd> upperLimits) const final;

    /**
     * Check if the class is valid.
     * @note If it is valid you can directly control the robot
     * @return True if it is valid, false otherwise.
     */
    bool isValid() const final;

    /**
     * Destructor
     */
    ~YarpRobotControl();
};
} // namespace RobotInterface
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_ROBOT_INTERFACE_YARP_ROBOT_CONTROL_H
