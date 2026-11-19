/**
 * @file BaseEstimatorFromFootIMU.h
 * @authors Guglielmo Cervettini
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be
 * modified and distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_ESTIMATORS_BASE_ESTIMATION_FROM_FOOT_IMU_H
#define BIPEDAL_LOCOMOTION_ESTIMATORS_BASE_ESTIMATION_FROM_FOOT_IMU_H

#include <BipedalLocomotion/Math/Constants.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/System/Advanceable.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Model/Model.h>
#include <manif/SE3.h>
#include <manif/SO3.h>

#include <memory>

namespace BipedalLocomotion
{
namespace Estimators
{

/**
 * BaseEstimatorFromFootIMUState contains the internal state of the estimator.
 */
struct BaseEstimatorFromFootIMUState
{
    manif::SE3d basePose; /**< final output of the estimator - pose of the robot
                             root link. */
    manif::SE3d::Tangent baseVelocity; /**< velocity of the robot  root link. */
    manif::SE3d footPose_L; /**< pose of the left foot */
    manif::SE3d footPose_R; /**< pose of the right foot */
    Eigen::Vector3d centerOfMassPosition;
    std::vector<Eigen::Vector3d> stanceFootShadowCorners;
    std::vector<Eigen::Vector3d> stanceFootCorners;
    int supportCornerIndex;
};

/**
 * BaseEstimatorFromFootIMUInput contains the input of the BaseEstimatorFromFootIMU class.
 */
struct BaseEstimatorFromFootIMUInput
{
    bool isLeftStance; /**< true if the left foot is in contact with the ground */
    bool isRightStance; /**< true if the right foot is in contact with the ground */
    Eigen::VectorXd jointPositions; /**< vector of the robot joint positions */
    Eigen::VectorXd jointVelocities; /**< vector of the robot joint velocities */
    manif::SE3d offsetStanceFootPose; /**< Optional offset orientation and position of the foot.
                                      E.g. as per footstep planner output */
    manif::SO3d measuredRotation_L; /**< actual orientation of the left foot measured by
                                    on-board IMU */
    manif::SO3d measuredRotation_R; /**< actual orientation of the right foot measured by
                                    on-board IMU */
    manif::SO3Tangentd measuredAngularVelocity_L; /**< actual angular velocity of the left foot
                                                  measured by on-board IMU */
    manif::SO3Tangentd measuredAngularVelocity_R; /**< actual angular velocity of the right foot
                                                  measured by on-board IMU */
};

/**
 * BaseEstimatorFromFootIMU implements the propagation of the foot pose to the root link through the
 * kinematic chain given by the leg joints positions.
 *
 * This class assumes that the foot has a rectangular shape as shown in the following schematics
 *
 *       FRONT
 *          +X
 *    p1 __|__  p0   __
 *      |  |  |        |
 * +Y___|__|__|___     | FOOT
 *      |  |  |        | LENGTH
 *      |__|__|      __|
 *    p2   |    p3
 *
 *       HIND
 *
 *      |_____|
 *       FOOT
 *       WIDTH
 */
class BaseEstimatorFromFootIMU
    : public BipedalLocomotion::System::Advanceable<BaseEstimatorFromFootIMUInput,
                                                    BaseEstimatorFromFootIMUState>
{
public:
    /**
     * Set the iDynTree::Model of the robot to be used by the estimator.
     * @param model the model considered in the estimator
     * @return true in case of success, false otherwise.
     */
    bool setModel(const iDynTree::Model& model);

    // clang-format off
    /**
     * Initialize the BaseEstimatorFromFootIMU block.
     * The setModel method must be called before initialization.
     * @param handler pointer to the parameter handler.
     * @note The following parameters are required
     * |   Parameter Name    |   Type   |                Description               | Mandatory |
     * |:-------------------:|:--------:|:----------------------------------------:|:---------:|
     * |  `foot_width_in_m`  | `double` | Transversal dimension of the robot foot  |    Yes    |
     * |  `foot_length_in_m` | `double` | Longitudinal dimension of the robot foot |    Yes    |
     * Moreover the user needs to add a group named `MODEL_INFO` that contains the following parameters.
     * | Parameter Name |     Type      |                                  Description                                          | Mandatory |
     * |:--------------:|:-------------:|:-------------------------------------------------------------------------------------:|:---------:|
     * |  `base_frame`  | `std::string` |                 Name of the frame whose pose must be estimated                        |    Yes    |
     * |  `foot_frame`  | `std::string` |  Name of the frame of one of the possible stance feet (whose orientation is measured) |    Yes    |
     * @return true in case of success/false otherwise.
     */
    bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler) override;
    // clang-format on

    /**
     * Set the input of the estimator.
     * @param input the input of the system.
     * @return true in case of success, false otherwise.
     */
    bool setInput(const BaseEstimatorFromFootIMUInput& input) override;

    /**
     * Perform one step of the estimator.
     * The estimator must have been initialized first.
     * @return true in case of success, false otherwise.
     */
    bool advance() override;

    /**
     * Check if the output of the estimator is valid.
     * @return true in case of success, false otherwise.
     */
    bool isOutputValid() const override;

    /**
     * Get the internal state of the BaseEstimatorFromFootIMU.
     * @return a const reference to the state of the estimator.
     */
    const BaseEstimatorFromFootIMUState& getOutput() const override;

    void resetBaseEstimatorFromFootIMU();

    /**
     * Set the state of the BaseEstimatorFromFootIMU.
     * @param state of the BaseEstimatorFromFootIMU
     * @note This method should be called only if the user want to force the state
     * of the system.
     */
    void setState(const BaseEstimatorFromFootIMUState& state);

private:
    BaseEstimatorFromFootIMUInput m_input; /**< Last input stored in the estimator */
    BaseEstimatorFromFootIMUState m_state; /**< Current state stored in the estimator */

    // Geometric quantities of the foot
    double m_footWidth; /**< Lateral dimension of the robot foot */
    double m_footLength; /**< Frontal dimension of the robot foot */

    /**
     * Define the 4 foot vertices in World reference frame:
     *
     *         +x              FRONT
     *   m_p1 __|__ m_p0   __
     *       |  |  |         |
     *  +y___|__|__|___      | FOOT
     *       |  |  |         | LENGTH
     *       |__|__|       __|
     *   m_p2   |   m_p3
     *                         HIND
     *       |_____|
     *        FOOT
     *        WIDTH
     *
     */
    std::vector<Eigen::Vector3d> m_cornersInInertialFrame; /**< this implementation is considering
                                                           rectangular feet (4 corners) */
    std::vector<Eigen::Vector3d> m_tiltedFootCorners; /**< vector of the foot corners
                                                              transformed into the inertial frame */
    Eigen::Vector3d m_offsetTranslation; /**< the position vector extracted from the
                                             `offsetFootPose` */
    Eigen::Matrix3d m_offsetRotation; /**< the rotation matrix extracted from the `offsetFootPose`
                                       */
    Eigen::Vector3d m_offsetRPY; /**< the rotation matrix extracted from the `offsetFootPose`
                                     converted into Euler angles */
    Eigen::Matrix3d m_measuredRotation; /**< the measured foot orientation casted manif::SO3d -->
                                           Eigen::Matrix3d */
    manif::SO3Tangentd m_measuredAngularVelocity; /**< the measured stance foot angular velocity */
    Eigen::Vector3d m_measuredRPY; /**< the measured foot orientation converted into Euler angles */
    manif::SO3d m_measuredRotationCorrected; /**< rotation matrix that employs: measured Roll,
                                                measured Pitch, offset Yaw */
    manif::SO3d m_offsetRotationCasted;
    manif::SO3d m_measuredTilt;
    manif::SE3d m_measuredFootPose; /**< the final foot pose matrix obtained through measured and
                                       offset quantities */
    manif::SE3d m_T_walk;
    manif::SE3d m_T_yawDrift;
    double m_yawOld;
    manif::SE3d m_footFrame_H_link_L; /**< coordinate change matrix from left foot link frame to
                                       * left foot sole frame
                                       */
    manif::SE3d m_footFrame_H_link_R; /**< coordinate change matrix from right foot link frame to
                                       * right foot sole frame
                                       */
    Eigen::Vector3d m_gravity;
    const Eigen::Vector3d m_noTras{0.0, 0.0, 0.0};
    iDynTree::KinDynComputations m_kinDyn;
    iDynTree::Model m_model;
    iDynTree::LinkIndex m_stanceLinkIndex;

    std::string m_baseFrameName; /**< Base link of the robot (whose pose must be estimated) */
    int m_baseFrameIndex; /**< Index of the frame whose pose needs to be estimated */

    std::string m_footFrameName_L; /**< reference frame of the left stance foot (whose
                                    orientation is measured)*/
    int m_footFrameIndex_L; /**< Index of the left foot frame */

    std::string m_footFrameName_R; /**< reference frame of the right stance foot (whose
                                    orientation is measured)*/
    int m_footFrameIndex_R; /**< Index of the right foot frame */

    bool m_isLastStanceFoot_L{false}; /**< true if the last stance foot was the left one */
    bool m_isLastStanceFoot_R{false}; /**< true if the last stance foot was the right one */

    bool m_isInitialized{false};
    bool m_isInputSet{false};
    bool m_isOutputValid{false};
};

} // namespace Estimators
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_ESTIMATORS_BASE_ESTIMATION_FROM_FOOT_IMU_H
