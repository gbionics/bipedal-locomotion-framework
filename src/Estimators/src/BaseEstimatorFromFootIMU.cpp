/*
 * @file BaseEstimatorFromFootIMU.cpp
 * @authors Guglielmo Cervettini
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be
 * modified and distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/Conversions/ManifConversions.h>
#include <BipedalLocomotion/FloatingBaseEstimators/BaseEstimatorFromFootIMU.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

using namespace BipedalLocomotion;

Eigen::Vector3d toXYZ(const Eigen::Matrix3d& r)
{
    Eigen::Vector3d output;
    double& thetaX = output[0]; // Roll
    double& thetaY = output[1]; // Pitch
    double& thetaZ = output[2]; // Yaw

    if (r(2, 0) < +1)
    {
        if (r(2, 0) > -1)
        {
            thetaY = asin(-r(2, 0));
            thetaZ = atan2(r(1, 0), r(0, 0));
            thetaX = atan2(r(2, 1), r(2, 2));
        } else // r20 = -1
        {
            // Not a unique solution: thetaX - thetaZ = atan2(-r12 , r11)
            thetaY = +M_PI / 2;
            thetaZ = -atan2(-r(1, 2), r(1, 1));
            thetaX = 0;
        }
    } else // r20 = +1
    {
        // Not a unique solution: thetaX + thetaZ = atan2(-r12, r11)
        thetaY = -M_PI / 2;
        thetaZ = atan2(-r(1, 2), r(1, 1));
        thetaX = 0;
    }

    return output;
}

using namespace BipedalLocomotion::Estimators;

bool BaseEstimatorFromFootIMU::setModel(const iDynTree::Model& model)
{
    constexpr auto logPrefix = "[BaseEstimatorFromFootIMU::setModel]";

    if (!m_kinDyn.loadRobotModel(model))
    {
        log()->error("{} Unable to load the model.", logPrefix);
        return false;
    }

    m_model = model;

    return true;
}

bool BaseEstimatorFromFootIMU::initialize(
    std::weak_ptr<const ParametersHandler::IParametersHandler> handler)
{
    constexpr auto logPrefix = "[BaseEstimatorFromFootIMU::initialize]";

    m_isInitialized = false;

    if (!m_model.isValid())
    {
        log()->error("{} No iDynTree::Model has been set.", logPrefix);
        return false;
    }

    if (!m_kinDyn.isValid())
    {
        log()->error("{} iDynTree::KinDynComputations m_kinDyn is not valid.", logPrefix);
        return false;
    }

    auto ptr = handler.lock();
    if (ptr == nullptr)
    {
        log()->error("{} Invalid parameter handler.", logPrefix);
        return false;
    }

    auto populateParameter
        = [logPrefix](std::weak_ptr<const ParametersHandler::IParametersHandler> handler,
                      const std::string& paramName,
                      auto& parameter) -> bool {
        auto ptr = handler.lock();
        if (ptr == nullptr)
        {
            log()->error("{} Invalid parameter handler for the parameter '{}'",
                         logPrefix,
                         paramName);
            return false;
        }

        if (!ptr->getParameter(paramName, parameter))
        {
            log()->error("{} Unable to find the parameter '{}'", logPrefix, paramName);
            return false;
        }
        return true;
    };

    auto baseEstimatorPtr = ptr->getGroup("BASE_ESTIMATOR").lock();
    if (baseEstimatorPtr == nullptr)
    {
        log()->error("{} Invalid parameter handler for the group 'BASE_ESTIMATOR'", logPrefix);
        return false;
    }

    // Frame associated to the base of the robot (whose pose is estimated)
    bool ok = populateParameter(baseEstimatorPtr->getGroup("MODEL_INFO"),
                                "base_frame",
                                m_baseFrameName);

    // Frame associated to the left foot of the robot (whose orientation is measured)
    ok = ok
         && populateParameter(baseEstimatorPtr->getGroup("MODEL_INFO"),
                              "foot_frame_L",
                              m_footFrameName_L);
    // Frame associated to the right foot of the robot (whose orientation is measured)
    ok = ok
         && populateParameter(baseEstimatorPtr->getGroup("MODEL_INFO"),
                              "foot_frame_R",
                              m_footFrameName_R);
    // Foot dimensions. Same for both feet.
    ok = ok && populateParameter(baseEstimatorPtr, "foot_width_in_m", m_footWidth);
    ok = ok && populateParameter(baseEstimatorPtr, "foot_length_in_m", m_footLength);

    if (!ok)
    {
        log()->error("{} Unable to retrieve all the parameters from the parameter handler.", logPrefix);
        return false;
    }

    m_gravity << 0, 0, -BipedalLocomotion::Math::StandardAccelerationOfGravitation;

    m_baseFrameIndex = m_kinDyn.getFrameIndex(m_baseFrameName);
    if (m_baseFrameIndex == iDynTree::FRAME_INVALID_INDEX)
    {
        log()->error("{} Invalid frame named: {}", logPrefix, m_baseFrameName);
        return false;
    }

    m_footFrameIndex_L = m_kinDyn.getFrameIndex(m_footFrameName_L);
    if (m_footFrameIndex_L == iDynTree::FRAME_INVALID_INDEX)
    {
        log()->error("{} Invalid frame named: {}", logPrefix, m_footFrameName_L);
        return false;
    }
    iDynTree::Transform frame_H_link_L = m_model.getFrameTransform(m_footFrameIndex_L).inverse();
    m_footFrame_H_link_L = Conversions::toManifPose(frame_H_link_L);

    m_footFrameIndex_R = m_kinDyn.getFrameIndex(m_footFrameName_R);
    if (m_footFrameIndex_R == iDynTree::FRAME_INVALID_INDEX)
    {
        log()->error("{} Invalid frame named: {}", logPrefix, m_footFrameName_R);
        return false;
    }
    iDynTree::Transform frame_H_link_R = m_model.getFrameTransform(m_footFrameIndex_R).inverse();
    m_footFrame_H_link_R = Conversions::toManifPose(frame_H_link_R);

    // resetting the vector of foot corners to be sure it is correctly initialized.
    m_cornersInInertialFrame.clear();

    // Set the 4 foot vertices in World reference frame [dimensions in meters]
    m_cornersInInertialFrame.emplace_back(+m_footLength / 2, -m_footWidth / 2, 0);
    m_cornersInInertialFrame.emplace_back(+m_footLength / 2, +m_footWidth / 2, 0);
    m_cornersInInertialFrame.emplace_back(-m_footLength / 2, +m_footWidth / 2, 0);
    m_cornersInInertialFrame.emplace_back(-m_footLength / 2, -m_footWidth / 2, 0);

    m_yawOld = 0.0;
    m_T_yawDrift.setIdentity();
    m_measuredFootPose.setIdentity();
    m_T_walk.setIdentity();
    m_state.stanceFootShadowCorners.resize(m_cornersInInertialFrame.size());
    m_state.stanceFootCorners.resize(m_cornersInInertialFrame.size());

    m_isInitialized = true;

    return ok;
}

bool BaseEstimatorFromFootIMU::setInput(const BaseEstimatorFromFootIMUInput& input)
{
    constexpr auto logPrefix = "[BaseEstimatorFromFootIMU::setInput]";
    m_isInputSet = false;

    if (!m_isInitialized)
    {
        log()->error("{} The estimator must be initialized before setting the input.", logPrefix);
        return false;
    }

    m_input = input;
    m_isInputSet = true;
    return true;
}

bool BaseEstimatorFromFootIMU::advance()
{
    constexpr auto logPrefix = "[BaseEstimatorFromFootIMU::advance]";

    m_isOutputValid = false;

    if (!m_isInputSet)
    {
        log()->error("{} The estimator input has not been set.", logPrefix);
        return false;
    }

    // checking the stance foot
    if (m_input.isLeftStance && m_input.isRightStance)
    {
        log()->error("{} Both feet are stance feet. The estimator accept only one stance foot.",
                     logPrefix);
        return false;
    }
    if (!m_input.isLeftStance && !m_input.isRightStance)
    {
        log()->error("{} No stance foot set. The estimator needs one stance foot in input.",
                     logPrefix);
        return false;
    }
    manif::SE3d stanceFootFrame_H_link = manif::SE3d::Identity();
    int stanceFootFrameIndex = iDynTree::FRAME_INVALID_INDEX;

    if (m_input.isLeftStance)
    {
        stanceFootFrame_H_link = m_footFrame_H_link_L;
        stanceFootFrameIndex = m_footFrameIndex_L;
        // casting the measured foot orientation manif::SO3d --> Eigen::Matrix3d.
        m_measuredRotation = m_input.measuredRotation_L.rotation();
        m_measuredAngularVelocity = m_input.measuredAngularVelocity_L;
        // has the stance foot just changed?
        if (m_isLastStanceFoot_R)
        {
            // updating the yawOld value.
            Eigen::Vector3d lastRPY_R = toXYZ(m_state.footPose_R.rotation());
            // Eigen::Vector3d lastRPY_L = toXYZ(m_input.measuredRotation_L.rotation());
            m_yawOld = lastRPY_R(2);

            // updating the m_T_walk matrix.
            Eigen::Vector3d stepTras = (m_state.footPose_L.translation() - m_state.footPose_R.translation());
            stepTras(2) = 0.0;
            manif::SE3d T_walkTras(stepTras, manif::SO3d::Identity());

            // Eigen::Matrix3d stepRot = m_state.footPose_R.rotation().inverse() * m_state.footPose_L.rotation();
            // Eigen::Vector3d stepRotRPY = toXYZ(stepRot);
            // manif::SO3d stepRotManif = manif::SO3d(stepRotRPY(0), stepRotRPY(1), stepRotRPY(2));
            // manif::SE3d T_walkRot(m_noTras, stepRotManif);
            // manif::SE3d temp = T_walkTras * T_walkRot * m_T_walk * m_T_yawDrift;

            manif::SE3d temp = T_walkTras * m_T_walk;
            m_T_walk = temp;

            // m_T_yawDrift.setIdentity();
        }
    }

    if (m_input.isRightStance)
    {
        stanceFootFrame_H_link = m_footFrame_H_link_R;
        stanceFootFrameIndex = m_footFrameIndex_R;
        // casting the measured foot orientation manif::SO3d --> Eigen::Matrix3d.
        m_measuredRotation = m_input.measuredRotation_R.rotation();
        m_measuredAngularVelocity = m_input.measuredAngularVelocity_R;
        // has the stance foot just changed?
        if (m_isLastStanceFoot_L)
        {
            // updating the yawOld value.
            Eigen::Vector3d lastRPY_L = toXYZ(m_state.footPose_L.rotation());
            // Eigen::Vector3d lastRPY_R = toXYZ(m_input.measuredRotation_R.rotation());
            m_yawOld = lastRPY_L(2);

            // updating the m_T_walk matrix.
            Eigen::Vector3d stepTras = (m_state.footPose_R.translation() - m_state.footPose_L.translation());
            stepTras(2) = 0.0;
            manif::SE3d T_walkTras(stepTras, manif::SO3d::Identity());

            // Eigen::Matrix3d stepRot = m_state.footPose_L.rotation().inverse() * m_state.footPose_R.rotation();
            // Eigen::Vector3d stepRotRPY = toXYZ(stepRot);
            // manif::SO3d stepRotManif = manif::SO3d(stepRotRPY(0), stepRotRPY(1), stepRotRPY(2));
            // manif::SE3d T_walkRot(m_noTras, stepRotManif);
            // manif::SE3d temp = T_walkTras * T_walkRot * m_T_walk * m_T_yawDrift;

            manif::SE3d temp = T_walkTras * m_T_walk;
            m_T_walk = temp;

            // m_T_yawDrift.setIdentity();
        }
    }

    m_stanceLinkIndex = m_model.getFrameLink(stanceFootFrameIndex);
    if (!m_kinDyn.setFloatingBase(m_model.getLinkName(m_stanceLinkIndex)))
    {
        log()->error("{} Unable to set the stance foot as floating base.", logPrefix);
        return false;
    }

    // `offsetStanceFootPose` is intended as the output of the footstep planner.

    // extracting the position part of the `offsetStanceFootPose`.
    m_offsetTranslation = m_input.offsetStanceFootPose.translation();

    // extracting the orientation part of the `offsetStanceFootPose` and expressing it
    // through RPY Euler angles.
    m_offsetRotation = m_input.offsetStanceFootPose.rotation();
    m_offsetRPY = toXYZ(m_offsetRotation);
    m_offsetRotationCasted = manif::SO3d(m_offsetRPY(0), m_offsetRPY(1), m_offsetRPY(2));

    // expressing measured orientation through RPY Euler angles.
    m_measuredRPY = toXYZ(m_measuredRotation);

    // offset Yaw is used instead of measured Yaw.
    // m_measuredRPY(2) = m_offsetRPY(2);

    // MANUAL CORRECTION: measured Roll, Pitch and Yaw.
    // double temp_roll = -m_measuredRPY(1);
    // double temp_pitch = -m_measuredRPY(0);
    // double temp_yaw = -m_measuredRPY(2);
    // m_measuredRPY(0) = temp_roll;
    // m_measuredRPY(1) = temp_pitch;
    // m_measuredRPY(2) = temp_yaw;

    // manif::SO3d rotation matrix that employs: measured Roll, measured Pitch, offset Yaw.
    m_measuredRotationCorrected = manif::SO3d(m_measuredRPY(0), m_measuredRPY(1), m_measuredRPY(2));
    m_measuredTilt = manif::SO3d(m_measuredRPY(0), m_measuredRPY(1), 0.0);
    double measuredYaw = m_measuredRPY(2);

    // manif::SE3d pose matrix that employs: offset Position, measured Roll, measured Pitch, offset
    // Yaw.
    manif::SE3d T_foot_imu(m_noTras, m_measuredRotationCorrected);
    manif::SE3d T_foot_tilt(m_noTras, m_measuredTilt);
    manif::SE3d T_foot_offset(m_offsetTranslation, m_offsetRotationCasted);

    // finding the positions of the foot corners in world frame given `T_foot_imu`
    // pose matrix.

    // resetting the vector of transformed foot corners from previous iteration.
    m_tiltedFootCorners.clear();

    // for each corner we compute the position in the inertial frame
    for (const auto& corner : m_cornersInInertialFrame)
    {
        m_tiltedFootCorners.emplace_back(T_foot_tilt.act(corner));
    }

    // The center of the foot sole is at ground level --> some corners may be
    // under ground level. The foot may need to be translated by a 3D offset value.

    // extracting the vertical quotes of the foot corners and finding the lowest
    // corner.
    Eigen::VectorXd cornersZ;
    cornersZ.resize(m_cornersInInertialFrame.size());
    cornersZ.setZero();
    int index = 0;
    for (const auto& corner : m_tiltedFootCorners)
    {
        cornersZ[index] = corner[2];
        index++;
    }

    // finding the index of the lowest corner.
    double minZ = cornersZ[0];
    int indexMinZ = 0;
    for (int i = 1; i < cornersZ.size(); i++)
    {
        if (cornersZ[i] < minZ)
        {
            minZ = cornersZ[i];
            indexMinZ = i;
        }
    }
    m_state.supportCornerIndex = indexMinZ;

    // checking that the index of the lowest corner is within the range [0, 3].
    if (!(0 <= m_state.supportCornerIndex <= 3))
    {
        log()->error("{} Foot vertex index out of bounds (0, 3): {}.",
                     logPrefix,
                     m_state.supportCornerIndex);
        return false;
    }

    // finding the index of the highest corner.
    // double maxZ = cornersZ[0];
    // int indexMaxZ = 0;
    // for (int i = 1; i < cornersZ.size(); i++)
    // {
    //     if (cornersZ[i] > maxZ)
    //     {
    //         maxZ = cornersZ[i];
    //         indexMaxZ = i;
    //     }
    // }

    // double deltaZ = cornersZ[indexMaxZ] - cornersZ[m_state.supportCornerIndex];
    // std::cerr << "Foot deltaZ: " << deltaZ << std::endl;

    // finding the translation vector needed to bring the lowest corner back to its
    // untilted position.
    Eigen::Vector3d p_untilted(0, 0, 0);
    Eigen::Vector3d supportCornerTranslation(0, 0, 0);
    p_untilted = (m_cornersInInertialFrame[m_state.supportCornerIndex]);
    supportCornerTranslation
        = p_untilted - (m_tiltedFootCorners[m_state.supportCornerIndex]); // TODO: change if
                                                                          // flat ground
                                                                          // assumption is
                                                                          // removed

    // transforming the offset vector into a translation matrix.
    manif::SE3d T_supportCornerTranslation(supportCornerTranslation, manif::SO3d::Identity());

    // calculating the yaw drift - VALID FOR BOTH FEET ONLY IF FRAMES ARE ORIENTED IN THE SAME WAY.
    double deltaYaw = 0.0;
    deltaYaw = measuredYaw - m_yawOld;
    m_yawOld = measuredYaw;
    // manif::SO3d rotation matrix that employs: zero Roll, zero Pitch, measured Yaw variation.
    auto R_deltaYaw = manif::SO3d(0.0, 0.0, deltaYaw);
    manif::SE3d T_deltaYaw(m_noTras, R_deltaYaw);

    std::vector<Eigen::Vector3d> tempCorners;
    tempCorners.resize(m_cornersInInertialFrame.size());
    int j = 0;
    for (const auto& corner : m_cornersInInertialFrame)
    {
        tempCorners[j] = T_deltaYaw.act(corner);
        j++;
    }
    Eigen::Vector3d yawDrift(0, 0, 0);
    yawDrift = m_cornersInInertialFrame[m_state.supportCornerIndex]
               - tempCorners[m_state.supportCornerIndex];
    // manif::SE3d T_yawDrift(yawDrift, R_deltaYaw);
    manif::SE3d T_yawDrift(m_noTras, R_deltaYaw);

    manif::SE3d temp = T_yawDrift * m_T_yawDrift;
    m_T_yawDrift = temp;

    // obtaining the final foot pose using both measured and offset quantities.
    // cordinate change is performed from foot sole frame to foot link frame.
    m_measuredFootPose = T_foot_offset * m_T_walk * m_T_yawDrift * T_supportCornerTranslation
                         * T_foot_tilt * stanceFootFrame_H_link;

    Eigen::VectorXd baseVelocity(6);
    // baseVelocity.setZero();
    Eigen::Vector3d angularVelocityInLinkFrame = stanceFootFrame_H_link.asSO3().inverse().act(m_measuredAngularVelocity.coeffs());
    Eigen::Vector3d cornerInLinkFrame = stanceFootFrame_H_link.inverse().act(m_cornersInInertialFrame[m_state.supportCornerIndex]);

    baseVelocity.head<3>() = m_measuredFootPose.asSO3().act(cornerInLinkFrame.cross(angularVelocityInLinkFrame));

    manif::SO3d measuredRotationManif = Conversions::toManifRot(m_measuredRotation);
    baseVelocity.tail<3>() = measuredRotationManif.act(m_measuredAngularVelocity.coeffs());

    // setting the robot state in terms of stance foot pose and joint positions.
    if (!m_kinDyn.setRobotState(m_measuredFootPose.transform(),
                                m_input.jointPositions,
                                baseVelocity,
                                m_input.jointVelocities,
                                m_gravity))
    {
        log()->error("{} Unable to set the robot state from the stance foot pose.", logPrefix);
        return false;
    }

    // calculating the output of the estimator given the robot state.
    m_state.basePose = Conversions::toManifPose(m_kinDyn.getWorldTransform(m_baseFrameIndex));
    m_state.footPose_L = Conversions::toManifPose(m_kinDyn.getWorldTransform(m_footFrameIndex_L));
    m_state.footPose_R = Conversions::toManifPose(m_kinDyn.getWorldTransform(m_footFrameIndex_R));
    m_state.baseVelocity = Conversions::toManifTwist(m_kinDyn.getFrameVel(m_baseFrameIndex));
    m_kinDyn.getCenterOfMassPosition(m_state.centerOfMassPosition);

    for (int i = 0; i < m_cornersInInertialFrame.size(); i++)
    {
        m_state.stanceFootShadowCorners[i]
            = m_T_walk.act(m_T_yawDrift.act(m_cornersInInertialFrame[i]));
        m_state.stanceFootCorners[i] = m_T_walk.act(
            m_T_yawDrift.act(T_supportCornerTranslation.act(m_tiltedFootCorners[i])));
    }

    double orientationError_L = (toXYZ(m_state.footPose_L.rotation()) - toXYZ(m_input.measuredRotation_L.rotation())).norm();
    orientationError_L = std::fmod(orientationError_L, 2.0 * M_PI) * 180.0 / M_PI;
    double orientationError_R = (toXYZ(m_state.footPose_R.rotation()) - toXYZ(m_input.measuredRotation_R.rotation())).norm();
    orientationError_R = std::fmod(orientationError_R, 2.0 * M_PI) * 180.0 / M_PI;
    double orientationErrorThreshold = 1.5; // [deg]. TODO: get parameter from config file. 1,5 deg = 0,0261799 rad.

    if ( ((orientationError_L > orientationErrorThreshold) && (orientationError_L < (360.0 - orientationErrorThreshold))) ||
         ((orientationError_R > orientationErrorThreshold) && (orientationError_R < (360.0 - orientationErrorThreshold))))
    {
        log()->warn("{} Foot orientation error above {} [deg] threshold: {} Left, {} Right.",
                     logPrefix,
                     orientationErrorThreshold,
                     orientationError_L,
                     orientationError_R);
    }
    // std::cerr << "L FOOT ROTATION IN: " << toXYZ(m_input.measuredRotation_L.rotation()).transpose() << std::endl;
    // std::cerr << "L FOOT ROTATION OUT: " << toXYZ(m_state.footPose_L.rotation()).transpose() << std::endl;
    // std::cerr << "R FOOT ROTATION IN: " << toXYZ(m_input.measuredRotation_R.rotation()).transpose() << std::endl;
    // std::cerr << "R FOOT ROTATION OUT: " << toXYZ(m_state.footPose_R.rotation()).transpose() << std::endl;

    // updating the stance foot flags.
    if (m_input.isLeftStance)
    {
        m_isLastStanceFoot_L = true;
        m_isLastStanceFoot_R = false;
    }
    if (m_input.isRightStance)
    {
        m_isLastStanceFoot_L = false;
        m_isLastStanceFoot_R = true;
    }

    m_isOutputValid = true;

    return true;
}

bool BaseEstimatorFromFootIMU::isOutputValid() const
{
    return m_isOutputValid;
}

const BaseEstimatorFromFootIMUState& BaseEstimatorFromFootIMU::getOutput() const
{
    constexpr auto logPrefix = "[BaseEstimatorFromFootIMU::getOutput]";

    if (!m_isOutputValid)
    {
        log()->error("{} The output is not valid.", logPrefix);
    }

    return m_state; // m_state.basePose is the actual output
}

void BaseEstimatorFromFootIMU::resetBaseEstimatorFromFootIMU()
{
    m_isOutputValid = false;
    m_yawOld = 0.0;
    m_T_yawDrift.setIdentity();
    m_T_walk.setIdentity();
    m_isLastStanceFoot_L = false;
    m_isLastStanceFoot_R = false;
}

// OPTIONAL METHOD - TO FORCE THE STATE

void BaseEstimatorFromFootIMU::setState(const BaseEstimatorFromFootIMUState& state)
{
    m_state = state;
}
