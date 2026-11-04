/**
 * @file SO3Task.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_IK_SO3_TASK_H
#define BIPEDAL_LOCOMOTION_IK_SO3_TASK_H

#include <memory>

#include <manif/manif.h>

#include <BipedalLocomotion/IK/IKLinearTask.h>

#include <iDynTree/KinDynComputations.h>

#include <LieGroupControllers/ProportionalController.h>

namespace BipedalLocomotion
{
namespace IK
{

/**
 * SO3Task is a concrete implementation of the IK::Task. Please use this element if you
 * want to control the position and the orientation of a given frame rigidly attached to the robot.
 * The task assumes perfect control of the robot velocity \f$\dot{\nu}\f$ that contains the base
 * linear and angular velocity expressed in mixed representation and the joint velocities.
 * The task represents the following equation
 * \f[
 * J_\omega \nu = \omega ^ *
 * \f]
 * where \f$J_\omega\f$ is the angular part of the robot Jacobian and \f$\omega ^ *\f$ is the
 * desired velocity. The desired velocity is chosen such that the orientation of the frame will
 * asymptotically converge to the desired trajectory. \f$\omega ^ *\f$ is computed with a
 * Proportional controller in \f$SO(3)\f$.
 * @note Please refer to https://github.com/ami-iit/lie-group-controllers if you are interested in
 * the implementation of the PD controllers.
 */
class SO3Task : public IKLinearTask
{
    LieGroupControllers::ProportionalControllerSO3d m_SO3Controller; /**< P Controller in SO(3) */

    System::VariablesHandler::VariableDescription m_robotVelocityVariable; /**< Variable
                                                                                  describing the
                                                                                  robot velocity
                                                                                  (base + joint) */

    iDynTree::FrameIndex m_frameIndex; /**< Frame controlled by the OptimalControlElement */

    static constexpr std::size_t m_angularVelocitySize{3}; /**< Size of the spatial velocity vector. */
    static constexpr std::size_t m_spatialVelocitySize{6}; /**< Size of the spatial velocity vector. */

    bool m_isInitialized{false}; /**< True if the task has been initialized. */
    bool m_isValid{false}; /**< True if the task is valid. */

    std::shared_ptr<iDynTree::KinDynComputations> m_kinDyn; /**< Pointer to a KinDynComputations
                                                               object */

    Eigen::MatrixXd m_jacobian; /**< Internal buffer to store the jacobian. */

    bool m_isSetPointSetAtLeastOnce{false}; /**< True if the set point has been set at least once. */
public:

    /**
     * Initialize the task.
     * @param paramHandler pointer to the parameters handler.
     * @note the following parameters are required by the class
     * |           Parameter Name           |   Type   |                                       Description                                      | Mandatory |
     * |:----------------------------------:|:--------:|:--------------------------------------------------------------------------------------:|:---------:|
     * |    `robot_velocity_variable_name`  | `string` |   Name of the variable contained in `VariablesHandler` describing the robot velocity   |    Yes    |
     * |            `frame_name`            | `string` |                       Name of the frame controlled by the SO3Task                      |    Yes    |
     * |            `kp_angular`            | `double` or `vector<double>` |                           Gain of the orientation controller                           |    Yes    |
     * Where the generalized robot velocity is a vector containing the base spatial-velocity
     * (expressed in mixed representation) and the joint velocities.
     * @return True in case of success, false otherwise.
     */
    bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> paramHandler) override;

    /**
     * Set the kinDynComputations object.
     * @param kinDyn pointer to a kinDynComputations object.
     * @return True in case of success, false otherwise.
     */
    bool setKinDyn(std::shared_ptr<iDynTree::KinDynComputations> kinDyn) override;

    /**
     * Set the set of variables required by the task. The variables are stored in the
     * System::VariablesHandler.
     * @param variablesHandler reference to a variables handler.
     * @note The handler must contain a variable named as the parameter
     * `robot_velocity_variable_name` stored in the parameter handler. The variable represents the
     * generalized velocity of the robot. Where the generalized robot velocity is a vector
     * containing the base spatial-velocity (expressed in mixed representation) and the joints
     * velocity.
     * @return True in case of success, false otherwise.
     */
    bool setVariablesHandler(const System::VariablesHandler& variablesHandler) override;

    /**
     * Update the content of the element.
     * @return True in case of success, false otherwise.
     */
    bool update() override;

    /**
     * Set the desired reference trajectory.
     * @param I_R_F Rotation between the link and the inertial frame.
     * @param angularVelocity angular velocity written in mixed inertial frame. The default value is
     * zero.
     * @return True in case of success, false otherwise.
     */
    bool setSetPoint(const manif::SO3d& I_R_F,
                     const manif::SO3d::Tangent& angularVelocity = manif::SO3d::Tangent::Zero());

    /**
     * Get the size of the task. (I.e the number of rows of the vector b)
     * @return the size of the task.
     */
    std::size_t size() const override;

    /**
     * The SO3Task is an equality task.
     * @return the size of the task.
     */
    Type type() const override;

    /**
     * Determines the validity of the objects retrieved with getA() and getB()
     * @return True if the objects are valid, false otherwise.
     */
    bool isValid() const override;

    /**
     * Get frame name.
     * @return the name of the frame.
     */
    iDynTree::FrameIndex getFrameIndex() const;
};

BLF_REGISTER_IK_TASK(SO3Task);

} // namespace IK
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_IK_SO3_TASK_H
