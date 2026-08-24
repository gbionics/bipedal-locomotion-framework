/**
 * @file IRobotControl.h
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_ROBOT_INTERFACE_IROBOT_CONTROL_H
#define BIPEDAL_LOCOMOTION_ROBOT_INTERFACE_IROBOT_CONTROL_H

#include <future>
#include <memory>
#include <optional>
#include <vector>

#include <Eigen/Dense>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>

namespace BipedalLocomotion
{
namespace RobotInterface
{

/**
 * Robot control interface
 */
class IRobotControl
{
public:
    /**
     * ControlMode contains the supported control mode.
     */
    enum class ControlMode
    {
        Position,
        PositionDirect,
        Velocity,
        Torque,
        PWM,
        Current,
        Idle,
        Unknown
    };

    using unique_ptr = std::unique_ptr<IRobotControl>;

    using shared_ptr = std::shared_ptr<IRobotControl>;

    using weak_ptr = std::weak_ptr<IRobotControl>;

    /**
     * Initialize the Interface
     * @param handler pointer to a parameter handler interface
     * @return True/False in case of success/failure.
     */
    virtual bool initialize(std::weak_ptr<ParametersHandler::IParametersHandler> handler);

    /**
     * Check if the motion set through the position control mode ended.
     * @param[out] motionDone true if the motion ended.
     * @param[out] isTimerExpired true if the internal timer is expired or not.
     * @param[out] info vector containing the list of the joint whose motion did not finish yet.
     * @return True/False in case of success/failure.
     * @note If the timer is expired and the motion did not finish yet, there may be a problem
     * with the robot.
     */
    virtual bool checkMotionDone(bool& motionDone,
                                 bool& isTimerExpired,
                                 std::vector<std::pair<std::string, double>>& info)
        = 0;

    /**
     * Set the desired reference.
     * @param desiredJointValues desired joint values.
     * @param controlModes vector containing the control mode for each joint.
     * @param currentJointValues current joint values.
     * @return True/False in case of success/failure.
     * @note In case of position control the values has to be expressed in rad, in case of velocity
     * control in rad/s. If the robot is controlled in torques, the desired joint values are
     * expressed in Nm.
     */
    virtual bool
    setReferences(Eigen::Ref<const Eigen::VectorXd> desiredJointValues,
                  const std::vector<IRobotControl::ControlMode>& controlModes,
                  std::optional<Eigen::Ref<const Eigen::VectorXd>> currentJointValues = {})
        = 0;

    /**
     * Set the desired reference.
     * @param desiredJointValues desired joint values.
     * @param controlMode a control mode for all the joints.
     * @param currentJointValues current joint values.
     * @return True/False in case of success/failure.
     * @note In case of position control the values has to be expressed in rad, in case of velocity
     * control in rad/s. If the robot is controlled in torques, the desired joint values are
     * expressed in Nm.
     * @warining Call this function if you want to control all the joint with the same control mode.
     * Otherwise call setReferences(Eigen::Ref<const Eigen::VectorXd>, const
     * std::vector<IRobotControl::ControlMode>&).
     */
    virtual bool
    setReferences(Eigen::Ref<const Eigen::VectorXd> desiredJointValues,
                  const IRobotControl::ControlMode& controlMode,
                  std::optional<Eigen::Ref<const Eigen::VectorXd>> currentJointValues = {})
        = 0;

    /**
     * Set the desired reference for a subset of joints identified by their indices.
     * @param desiredJointValues desired joint values for the subset (size must match jointIndices).
     * @param jointIndices indices of the joints to control (into the full joint list).
     * @param controlMode control mode for all the subset joints.
     * @param currentJointValues current joint values for the subset (same size as desiredJointValues).
     *        If not provided, the values are read from the hardware.
     * @return True/False in case of success/failure.
     * @note In case of position control the values have to be expressed in rad, in case of velocity
     * control in rad/s. If the robot is controlled in torques, the desired joint values are
     * expressed in Nm.
     */
    virtual bool
    setReferences(Eigen::Ref<const Eigen::VectorXd> desiredJointValues,
                  const std::vector<int>& jointIndices,
                  const IRobotControl::ControlMode& controlMode,
                  std::optional<Eigen::Ref<const Eigen::VectorXd>> currentJointValues = {})
        = 0;

    /**
     * Set the desired reference for a subset of joints identified by their names.
     * @param desiredJointValues desired joint values for the subset (size must match jointNames).
     * @param jointNames names of the joints to control.
     * @param controlMode control mode for all the subset joints.
     * @param currentJointValues current joint values for the subset (same size as desiredJointValues).
     *        If not provided, the values are read from the hardware.
     * @return True/False in case of success/failure.
     * @note In case of position control the values have to be expressed in rad, in case of velocity
     * control in rad/s. If the robot is controlled in torques, the desired joint values are
     * expressed in Nm.
     */
    virtual bool
    setReferences(Eigen::Ref<const Eigen::VectorXd> desiredJointValues,
                  const std::vector<std::string>& jointNames,
                  const IRobotControl::ControlMode& controlMode,
                  std::optional<Eigen::Ref<const Eigen::VectorXd>> currentJointValues = {})
        = 0;

    /**
     * Set the control mode.
     * @param controlModes vector containing the control mode for each joint.
     * @return True/False in case of success/failure.
     * @warning At the current stage only revolute joints are supported.
     */
    virtual bool setControlMode(const std::vector<IRobotControl::ControlMode>& controlModes) = 0;

    /**
     * Set the desired control mode.
     * @param controlMode a control mode for all the joints.
     * @return True/False in case of success/failure.
     * @warning Call this function if you want to control all the joint with the same control mode.
     */
    virtual bool setControlMode(const IRobotControl::ControlMode& mode) = 0;

    /**
     * Set the control mode in an asynchronous thread.
     * @param controlModes vector containing the control mode for each joint.
     * @return An std::future object to a boolean True/False in case of success/failure.
     * @warning At the current stage only revolute joints are supported.
     * Since this function spawns a new thread, the invoking thread is not blocked.
     */
    virtual std::future<bool>
    setControlModeAsync(const std::vector<IRobotControl::ControlMode>& controlModes) = 0;

    /**
     * Set the desired control mode in an asynchronous thread.
     * @param controlMode a control mode for all the joints.
     * @return An std::future object to a boolean True/False in case of success/failure.
     * @warning Call this function if you want to control all the joint with the same control mode.
     * Since this function spawns a new thread, the invoking thread is not blocked.
     */
    virtual std::future<bool> setControlModeAsync(const IRobotControl::ControlMode& mode) = 0;

    /**
     * Get the list of the controlled joints
     * @return A vector containing the name of the controlled joints.
     */
    virtual std::vector<std::string> getJointList() const = 0;

    /**
     * Get the actuated joints limits.
     * @param lowerLimits vector to be filled with the lower limits of the actuated joints.
     * @param upperLimits vector to be filled with the upper limits of the actuated joints.
     * @return True/False in case of success/failure.
     */
    virtual bool getJointLimits(Eigen::Ref<Eigen::VectorXd> lowerLimits,
                                Eigen::Ref<Eigen::VectorXd> upperLimits) const
        = 0;

    /**
     * Check if the class is valid.
     * @note If it is valid you can directly control the robot
     * @return True if it is valid, false otherwise.
     */
    virtual bool isValid() const = 0;

    /**
     * Destructor.
     */
    virtual ~IRobotControl() = default;
};
} // namespace RobotInterface
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_ROBOT_INTERFACE_IROBOT_CONTROL_H
