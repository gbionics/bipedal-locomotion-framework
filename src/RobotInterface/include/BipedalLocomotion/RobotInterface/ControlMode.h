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
    ForceIdle, /**< Idle the joint and acknowledge its hardware fault, if any. Once set, the joint
                  is reported as Idle (or HardwareFault if the fault persists). */
    HardwareFault, /**< The joint is in hardware fault. It can only be read: the only mode that
                      can be set to a joint in hardware fault is ForceIdle. */
    Unknown
};
} // namespace BipedalLocomotion::RobotInterface

#endif // BIPEDAL_LOCOMOTION_ROBOT_INTERFACE_CONTROL_MODE_H
