/**
 * @file SE3Task.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_IK_SE3_TASK_H
#define BIPEDAL_LOCOMOTION_IK_SE3_TASK_H

#include <memory>

#include <manif/manif.h>

#include <BipedalLocomotion/IK/IKLinearTask.h>
#include <BipedalLocomotion/System/ITaskControllerManager.h>

#include <iDynTree/KinDynComputations.h>

#include <LieGroupControllers/ProportionalController.h>

namespace BipedalLocomotion
{
namespace IK
{

/**
 * SE3Task is a concrete implementation of the Task. Please use this element if you
 * want to control the position and the orientation of a given frame rigidly attached to the robot.
 * The task assumes perfect control of the robot velocity \f$\nu\f$ that contains the base
 * linear and angular velocity expressed in mixed representation and the joint velocities.
 * The task represents the following equation
 * \f[
 * J \nu = \mathrm{v} ^ *
 * \f]
 * where \f$J\f$ is the robot jacobian and \f$\mathrm{v} ^ *\f$ is the desired velocity.
 * The desired velocity is chosen such that the frame will asymptotically converge to the
 * desired trajectory. The linear component of \f$\mathrm{v} ^ *\f$ is computed with a
 * standard Proportional controller in \f$R^3\f$ while the angular velocity is computed
 * by a Proportional controller in \f$SO(3)\f$.
 * @note Please refer to https://github.com/ami-iit/lie-group-controllers if you are interested in
 * the implementation of the PD controllers.
 * @note The SE3Task is technically not a \f$SE(3)\f$ space defined task, instead is a \f$SO(3)
 * \times \mathbb{R}^3\f$ task. Theoretically, there are differences between the two due to the
 * different definitions of exponential maps and logarithm maps. Please consider that here the MIXED
 * representation is used to define the 6d-velocity. You can find further details in Section 2.3.4
 * of https://traversaro.github.io/phd-thesis/traversaro-phd-thesis.pdf.
 * @note SE3Task can be used to control also a subset of element of the linear part of the SE3.
 * Please refer to `mask` parameter in IK::SE3Task::initialize method.
 */
class SE3Task : public IKLinearTask, public BipedalLocomotion::System::ITaskControllerManager
{
public:
    using Mode = System::ITaskControllerManager::Mode;

private:
    LieGroupControllers::ProportionalControllerSO3d m_SO3Controller; /**< P Controller in SO(3) */
    LieGroupControllers::ProportionalControllerR3d m_R3Controller; /**< P Controller in R3 */

    System::VariablesHandler::VariableDescription m_robotVelocityVariable; /**< Variable
                                                                                  describing the
                                                                                  robot velocity
                                                                                  (base + joint) */

    iDynTree::FrameIndex m_frameIndex; /**< Frame controlled by the OptimalControlElement */

    static constexpr std::size_t m_spatialVelocitySize{6}; /**< Size of the spatial velocity vector. */
    static constexpr std::size_t m_linearVelocitySize{3}; /**< Size of the linear velocity vector. */
    static constexpr std::size_t m_angularVelocitySize{3}; /**< Size of the angular velocity vector. */

    bool m_isInitialized{false}; /**< True if the task has been initialized. */
    bool m_isValid{false}; /**< True if the task is valid. */
    bool m_usePositionExogenousFeedback{false}; /**< True if the feedback of the position task must
                                                   be provided by the user. */
    bool m_useOrientationExogenousFeedback{false}; /**< True if the feedback of the orientation task
                                                   must be provided by the user. */
    bool m_angularTrackingEnabled{true}; /**< True if angular tracking is active. */

    std::shared_ptr<iDynTree::KinDynComputations> m_kinDyn; /**< Pointer to a KinDynComputations
                                                               object */

    /** Mask used to select the DoFs controlled by the task */
    std::array<bool, m_linearVelocitySize> m_mask{true, true, true};
    std::size_t m_linearDoFs{m_linearVelocitySize}; /**< DoFs associated to the linear task */
    std::size_t m_DoFs{m_spatialVelocitySize}; /**< DoFs associated to the entire task */

    Eigen::MatrixXd m_jacobian; /**< Jacobian matrix in MIXED representation */

    /** State of the proportional controller implemented in the task */
    Mode m_controllerMode{Mode::Enable};

    bool m_isSetPointSetAtLeastOnce{false}; /**< True if the set-point has been set at least once */
public:
    /**
     * Initialize the task.
     * @param paramHandler pointer to the parameters handler.
     * @note the following parameters are required by the class
     * |           Parameter Name           |   Type   |                                       Description                                      | Mandatory |
     * |:----------------------------------:|:--------:|:--------------------------------------------------------------------------------------:|:---------:|
     * |   `robot_velocity_variable_name`   | `string` |   Name of the variable contained in `VariablesHandler` describing the robot velocity   |    Yes    |
     * |            `frame_name`            | `string` |                       Name of the frame controlled by the SE3Task                      |    Yes    |
     * |             `kp_linear`            | `double` or `vector<double>` |                             Gain of the position controller                            |    Yes    |
     * |            `kp_angular`            | `double` or `vector<double>` |                           Gain of the orientation controller                           |    Yes    |
     * |               `mask`               | `vector<bool>` |  Mask representing the linear DoFs controlled. E.g. [1,0,1] will enable the control on the x and z coordinates only and the angular part. (Default value, [1,1,1])   |    No    |
     * |  `use_position_exogenous_feedback` |  `bool`  |    If true the task will consider the frame position provided by the user as feedback. The feedback must be set using `setFeedback()`. (Default value `false`) |   No   |
     * |  `use_orientation_exogenous_feedback` |  `bool`  |    If true the task will consider the frame orientation provided by the user as feedback. The feedback must be set using `setFeedback()`. (Default value `false`) |   No   |
     * @return True in case of success, false otherwise.
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
     * Set the desired set-point of the trajectory.
     * @param I_H_F Homogeneous transform between the link and the inertial frame.
     * @param mixedVelocity 6D-velocity written in mixed representation. The default value is zero.
     * @return True in case of success, false otherwise.
     */
    bool setSetPoint(const manif::SE3d& I_H_F,
                     const manif::SE3d::Tangent& mixedVelocity = manif::SE3d::Tangent::Zero());

    /**
     * Set the feedback for the Proportional controller.
     * @param I_H_F Homogeneous transform between the link and the inertial frame.
     * @note the feedback will be considered only if the `use_orientation_exogenous_feedback` or
     * `use_position_exogenous_feedback` is set to true.
     * @return True in case of success, false otherwise.
     */
    bool setFeedback(const manif::SE3d& I_H_F);

    /**
     * Set the feedback for the Proportional controller.
     * @param I_p_F position of the link respect to the inertial frame.
     * @note the feedback will be considered only if the `use_position_exogenous_feedback` is set to
     * true.
     * @return True in case of success, false otherwise.
     */
    bool setFeedback(const manif::SE3d::Translation& I_p_F);

    /**
     * Set the feedback for the Proportional controller.
     * @param I_R_F orientation of the link respect to the inertial frame.
     * @note the feedback will be considered only if the `use_orientation_exogenous_feedback` is set
     * to true.
     * @return True in case of success, false otherwise.
     */
    bool setFeedback(const manif::SO3d& I_R_F);

    /**
     * Get the size of the task. (I.e the number of rows of the vector b)
     * @return the size of the task.
     */
    std::size_t size() const override;

    /**
     * The SE3Task is an equality task.
     * @return the size of the task.
     */
    Type type() const override;

    /**
     * Determines the validity of the objects retrieved with getA() and getB()
     * @return True if the objects are valid, false otherwise.
     */
    bool isValid() const override;

    /**
     * Set the task controller mode. Please use this method to disable/enable the Proportional
     * controller implemented in this task.
     * @param state state of the controller
     */
    void setTaskControllerMode(Mode mode) override;

    /**
     * Get the task controller mode.
     * @return the state of the controller
     */
    Mode getTaskControllerMode() const override;

    /**
     * Enable or disable angular tracking. When disabled, the angular task rows are set to zero
     * while the task size remains unchanged.
     * @param enabled True to enable angular tracking, false to disable it.
     */
    void setAngularTrackingEnabled(bool enabled);

    /**
     * Get frame name.
     * @return the name of the frame.
     */
    iDynTree::FrameIndex getFrameIndex() const;
};

BLF_REGISTER_IK_TASK(SE3Task);

} // namespace IK
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_IK_SE3_TASK_H
