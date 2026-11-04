/**
 * @file R3Task.h
 * @authors Giulio Romualdi
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_IK_R3_TASK_H
#define BIPEDAL_LOCOMOTION_IK_R3_TASK_H

#include <memory>

#include <Eigen/Dense>

#include <BipedalLocomotion/IK/IKLinearTask.h>
#include <BipedalLocomotion/System/ITaskControllerManager.h>

#include <iDynTree/KinDynComputations.h>

#include <LieGroupControllers/ProportionalController.h>

namespace BipedalLocomotion
{
namespace IK
{

/**
 * R3Task is a concrete implementation of the Task. Please use this element if you
 * want to control the position of a given frame rigidly attached to the robot.
 * The task assumes perfect control of the robot velocity \f$\nu\f$ that contains the base
 * linear and angular velocity expressed in mixed representation and the joint velocities.
 * The task represents the following equation
 * \f[
 * J_l \nu = v ^ *
 * \f]
 * where \f$J_l\f$ is the linear part robot jacobian and \f$v ^ *\f$ is the desired velocity.
 * The desired velocity is chosen such that the frame position will asymptotically converge to the
 * desired trajectory and is computed with a standard Proportional controller in \f$R^3\f$.
 * @note Please refer to https://github.com/ami-iit/lie-group-controllers if you are interested in
 * the implementation of the PD controllers.
 * @note R3Task can be used to control also a subset of coordinates.
 * Please refer to `mask` parameter in IK::R3Task::initialize method.
 */
class R3Task : public IKLinearTask, public BipedalLocomotion::System::ITaskControllerManager
{
public:
    using Mode = System::ITaskControllerManager::Mode;

private:
    LieGroupControllers::ProportionalControllerR3d m_R3Controller; /**< P Controller in R3 */

    System::VariablesHandler::VariableDescription m_robotVelocityVariable; /**< Variable
                                                                                  describing the
                                                                                  robot velocity
                                                                                  (base + joint) */

    iDynTree::FrameIndex m_frameIndex; /**< Frame controlled by the OptimalControlElement */

    static constexpr std::size_t m_linearVelocitySize{3}; /**< Size of the linear velocity vector. */
    static constexpr std::size_t m_spatialVelocitySize{6}; /**< Size of the spatial velocity vector. */

    bool m_isInitialized{false}; /**< True if the task has been initialized. */
    bool m_isValid{false}; /**< True if the task is valid. */

    std::shared_ptr<iDynTree::KinDynComputations> m_kinDyn; /**< Pointer to a KinDynComputations
                                                               object */

    /** Mask used to select the DoFs controlled by the task */
    std::array<bool, m_linearVelocitySize> m_mask{true, true, true};
    std::size_t m_DoFs{m_linearVelocitySize}; /**< DoFs associated to the entire task */

    Eigen::MatrixXd m_jacobian; /**< Jacobian matrix in MIXED representation */

    /** State of the proportional controller implemented in the task */
    Mode m_controllerMode{Mode::Enable};

    bool m_isSetPointSetAtLeastOnce{false}; /**< True if the set point has been set at least once */
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
     * |               `mask`               | `vector<bool>` |  Mask representing the linear DoFs controlled. E.g. [1,0,1] will enable the control on the x and z coordinates only. (Default value, [1,1,1])   |    No    |
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
     * @param I_p_F position of the origin of the frame with respect to the inertial frame.
     * @param linear velocity of the frame. The default value is set to zero.
     * @return True in case of success, false otherwise.
     */
    bool setSetPoint(Eigen::Ref<const Eigen::Vector3d> I_p_F,
                     Eigen::Ref<const Eigen::Vector3d> velocity = Eigen::Vector3d::Zero());

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
     * Get frame name.
     * @return the name of the frame.
     */
    iDynTree::FrameIndex getFrameIndex() const;
};

BLF_REGISTER_IK_TASK(R3Task);

} // namespace IK
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_IK_R3_TASK_H
