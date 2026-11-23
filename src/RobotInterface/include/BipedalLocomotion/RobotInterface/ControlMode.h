/**
 * @file ControlMode.h
 * @authors Giulio Romualdi
 * @copyright Generative Bionics S.R.L. This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_ROBOT_INTERFACE_CONTROL_MODE_H
#define BIPEDAL_LOCOMOTION_ROBOT_INTERFACE_CONTROL_MODE_H

namespace BipedalLocomotion::RobotInterface
{
/**
 * ControlMode contains the supported control modes of a joint.
 * @note The units refer to revolute joints. The references are set with
 * IRobotControl::setReferences().
 * @note The values are explicit since they are stored in the logs (e.g., the
 * `joints_state::control_modes` channel of the YarpRobotLoggerDevice): do not change them.
 */
enum class ControlMode
{
    Position = 0, /**< The joint reaches the desired position [rad] following a trajectory generated
                 by the low-level controller. Use it for point-to-point motions. */
    PositionDirect = 1, /**< The desired position [rad] is tracked directly, without any trajectory.
                       The reference has to be streamed at every control cycle. */
    Velocity = 2, /**< The joint tracks the desired velocity [rad/s]. */
    Torque = 3, /**< The joint tracks the desired torque [Nm]. */
    PWM = 4, /**< The desired PWM duty cycle is applied to the motor. */
    Current = 5, /**< The motor tracks the desired current [A]. */
    Idle = 6, /**< The joint is not controlled and the motor is not powered. No reference is
             accepted. */
    ForceIdle = 7, /**< Like Idle, but it also acknowledges the hardware fault of the joint, if any.
                  It is the only mode accepted by a joint in HardwareFault. Once set, the joint is
                  reported as Idle, or as HardwareFault while the fault persists. */
    HardwareFault = 8, /**< The joint is in hardware fault and it is not controlled. This mode can
                      only be read: the joint leaves it after ForceIdle, when the fault is cleared.
                    */
    Unknown = 9 /**< The control mode is not available or not supported. */
};
} // namespace BipedalLocomotion::RobotInterface

#endif // BIPEDAL_LOCOMOTION_ROBOT_INTERFACE_CONTROL_MODE_H
