/**
 * @file InteractionMode.h
 * @copyright Generative Bionics S.R.L. This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_ROBOT_INTERFACE_INTERACTION_MODE_H
#define BIPEDAL_LOCOMOTION_ROBOT_INTERFACE_INTERACTION_MODE_H

namespace BipedalLocomotion::RobotInterface
{
/**
 * InteractionMode contains the supported interaction modes of a joint.
 */
enum class InteractionMode
{
    Stiff = 0, /**< The joint behaves stiffly. */
    Compliant = 1, /**< The joint behaves compliantly. */
    Unknown = 2 /**< The interaction mode is not available or not supported. */
};
} // namespace BipedalLocomotion::RobotInterface

#endif // BIPEDAL_LOCOMOTION_ROBOT_INTERFACE_INTERACTION_MODE_H
