/**
 * @file SE3Task.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/Conversions/ManifConversions.h>
#include <BipedalLocomotion/IK/SE3Task.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <Eigen/src/Geometry/Quaternion.h>

#include <iDynTree/EigenHelpers.h>
#include <iDynTree/Model.h>

using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::System;
using namespace BipedalLocomotion::IK;

bool SE3Task::setKinDyn(std::shared_ptr<iDynTree::KinDynComputations> kinDyn)
{
    if ((kinDyn == nullptr) || (!kinDyn->isValid()))
    {
        log()->error("[SE3Task::setKinDyn] Invalid kinDyn object.");
        return false;
    }

    m_kinDyn = kinDyn;
    return true;
}

bool SE3Task::setVariablesHandler(const System::VariablesHandler& variablesHandler)
{
    if (!m_isInitialized)
    {
        log()->error("[SE3Task::setVariablesHandler] The task is not initialized. Please call "
                     "initialize method.");
        return false;
    }

    // get the variable
    if (!variablesHandler.getVariable(m_robotVelocityVariable.name, m_robotVelocityVariable))
    {
        log()->error("[SE3Task::setVariablesHandler] Unable to get the variable named {}.",
                     m_robotVelocityVariable.name);
        return false;
    }

    // get the variable
    if (m_robotVelocityVariable.size != m_kinDyn->getNrOfDegreesOfFreedom() + m_spatialVelocitySize)
    {
        log()->error("[SE3Task::setVariablesHandler] The size of the robot velocity variable is "
                     "different from the one expected. Expected size: {}. Given size: {}.",
                     m_kinDyn->getNrOfDegreesOfFreedom() + m_spatialVelocitySize,
                     m_robotVelocityVariable.size);
        return false;
    }

    // resize the matrices
    m_A.resize(m_DoFs, variablesHandler.getNumberOfVariables());
    m_A.setZero();
    m_b.resize(m_DoFs);
    m_jacobian.resize(m_spatialVelocitySize, m_robotVelocityVariable.size);

    return true;
}

bool SE3Task::initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> paramHandler)
{
    constexpr auto errorPrefix = "[SE3Task::initialize] ";

    std::string frameName = "Unknown";
    constexpr auto descriptionPrefix = "IK-SE3Task - Frame name: ";

    std::string maskDescription = "";
    auto boolToString = [](bool b) { return b ? " true" : " false"; };
    for (const auto flag : m_mask)
    {
        maskDescription += boolToString(flag);
    }

    if (m_kinDyn == nullptr || !m_kinDyn->isValid())
    {
        log()->error("{} [{} {}] KinDynComputations object is not valid.",
                     errorPrefix,
                     descriptionPrefix,
                     frameName);
        return false;
    }

    if (m_kinDyn->getFrameVelocityRepresentation()
        != iDynTree::FrameVelocityRepresentation::MIXED_REPRESENTATION)
    {
        log()->error("{} [{} {}] The task supports only quantities expressed in MIXED "
                     "representation. Please provide a KinDynComputations with Frame velocity "
                     "representation set to MIXED_REPRESENTATION.",
                     errorPrefix,
                     descriptionPrefix,
                     frameName);
        return false;
    }

    auto ptr = paramHandler.lock();
    if (ptr == nullptr)
    {
        log()->error("{} [{} {}] The parameter handler is not valid.",
                     errorPrefix,
                     descriptionPrefix,
                     frameName);
        return false;
    }

    if (!ptr->getParameter("robot_velocity_variable_name", m_robotVelocityVariable.name))
    {
        log()->error("{} [{} {}] Error while retrieving the robot velocity variable.",
                     errorPrefix,
                     descriptionPrefix,
                     frameName);
        return false;
    }

    if (!ptr->getParameter("frame_name", frameName)
        || (m_frameIndex = m_kinDyn->model().getFrameIndex(frameName))
               == iDynTree::FRAME_INVALID_INDEX)
    {
        log()->error("{} [{} {}] Error while retrieving the frame that should be controlled.",
                     errorPrefix,
                     descriptionPrefix,
                     frameName);
        return false;
    }

    // set the gains for the R3 controller
    double kpLinearScalar;
    Eigen::Vector3d kpLinearVector;
    if (ptr->getParameter("kp_linear", kpLinearScalar))
    {
        m_R3Controller.setGains(kpLinearScalar);
    } else if (ptr->getParameter("kp_linear", kpLinearVector))
    {
        m_R3Controller.setGains(kpLinearVector);
    } else
    {
        log()->error("{} [{} {}] Unable to get the proportional linear gain.",
                     errorPrefix,
                     descriptionPrefix,
                     frameName);
        return false;
    }

    // set gains for the SO3 controller
    double kpAngularScalar;
    Eigen::Vector3d kpAngularVector;
    if (ptr->getParameter("kp_angular", kpAngularScalar))
    {
        m_SO3Controller.setGains(kpAngularScalar);
    } else if (ptr->getParameter("kp_angular", kpAngularVector))
    {
        m_SO3Controller.setGains(kpAngularVector);
    } else
    {
        log()->error("{} [{} {}] Unable to get the proportional angular gain.",
                     errorPrefix,
                     descriptionPrefix,
                     frameName);
        return false;
    }

    std::vector<bool> mask;
    if (!ptr->getParameter("mask", mask) || (mask.size() != m_linearVelocitySize))
    {
        log()->info("{} [{} {}] Unable to find the mask parameter. The default value is used:{}.",
                    errorPrefix,
                    descriptionPrefix,
                    frameName,
                    maskDescription);
    } else
    {
        // covert an std::vector in a std::array
        std::copy(mask.begin(), mask.end(), m_mask.begin());
        // compute the DoFs associated to the task
        m_linearDoFs = std::count(m_mask.begin(), m_mask.end(), true);

        m_DoFs = m_linearDoFs + m_angularVelocitySize;

        // Update the mask description
        maskDescription.clear();
        for (const auto flag : m_mask)
        {
            maskDescription += boolToString(flag);
        }
    }

    // the following parameters are optional
    if (!ptr->getParameter("use_orientation_exogenous_feedback", m_useOrientationExogenousFeedback))
    {
        log()->info("{} [{} {}] Unable to find the use_orientation_exogenous_feedback parameter. "
                    "The default value is used: {}.",
                    errorPrefix,
                    descriptionPrefix,
                    frameName,
                    m_useOrientationExogenousFeedback);
    }

    if (!ptr->getParameter("use_position_exogenous_feedback", m_usePositionExogenousFeedback))
    {
        log()->info("{} [{} {}] Unable to find the use_position_exogenous_feedback parameter. "
                    "The default value is used: {}.",
                    errorPrefix,
                    descriptionPrefix,
                    frameName,
                    m_usePositionExogenousFeedback);
    }

    m_description = descriptionPrefix + frameName + " Mask:" + maskDescription
                    + ". Use exogenous feedback position:"
                    + boolToString(m_usePositionExogenousFeedback)
                    + ". Use exogenous feedback orientation:"
                    + boolToString(m_useOrientationExogenousFeedback) + ".";

    // initialize the feedback of the controller
    m_R3Controller.setState(manif::SE3d::Translation::Zero());
    m_SO3Controller.setState(Eigen::Quaterniond::Identity());

    m_isInitialized = true;

    return true;
}

bool SE3Task::update()
{
    using namespace BipedalLocomotion::Conversions;
    using namespace iDynTree;

    m_isValid = false;

    if (!m_isInitialized)
    {
        log()->error("[SE3Task::update] The task is not initialized. Please call initialize "
                     "method.");
        return m_isValid;
    }

    if (!m_isSetPointSetAtLeastOnce)
    {
        log()->error("[SE3Task::update] The set point has not been set at least once.");
        return m_isValid;
    }

    auto getControllerState = [&](const auto& controller) {
        if (m_controllerMode == Mode::Enable)
        {
            return controller.getControl().coeffs();
        }

        return controller.getFeedForward().coeffs();
    };

    // set the state
    if (!m_useOrientationExogenousFeedback)
    {
        m_SO3Controller.setState(
            toManifRot(m_kinDyn->getWorldTransform(m_frameIndex).getRotation()));
    }
    if (!m_usePositionExogenousFeedback)
    {
        m_R3Controller.setState(toEigen(m_kinDyn->getWorldTransform(m_frameIndex).getPosition()));
    }

    // update the controller
    if (m_angularTrackingEnabled)
    {
        m_SO3Controller.computeControlLaw();
    }
    m_R3Controller.computeControlLaw();

    // Set the angular target velocity, or a no-op row if tracking is disabled.
    if (m_angularTrackingEnabled)
    {
        m_b.tail<3>() = getControllerState(m_SO3Controller);
    } else
    {
        m_b.tail<3>().setZero();
    }

    // if we want to control all 6 DoF we avoid to lose performances
    if (m_linearDoFs == m_linearVelocitySize)
    {
        m_b.head<3>() = getControllerState(m_R3Controller);

        if (!m_kinDyn->getFrameFreeFloatingJacobian(m_frameIndex,
                                                    this->subA(m_robotVelocityVariable)))
        {
            log()->error("[SE3Task::update] Unable to get the jacobian.");
            return m_isValid;
        }
        if (!m_angularTrackingEnabled)
        {
            iDynTree::toEigen(this->subA(m_robotVelocityVariable)).bottomRows<3>().setZero();
        }
    } else
    {
        // store the jacobian associated to the given frame
        if (!m_kinDyn->getFrameFreeFloatingJacobian(m_frameIndex, m_jacobian))
        {
            log()->error("[SE3Task::update] Unable to get the jacobian.");
            return m_isValid;
        }

        // take only the required components
        std::size_t index = 0;

        // linear components
        for (std::size_t i = 0; i < 3; i++)
        {
            if (m_mask[i])
            {
                m_b(index) = getControllerState(m_R3Controller)(i);
                iDynTree::toEigen(this->subA(m_robotVelocityVariable)).row(index)
                    = m_jacobian.row(i);
                index++;
            }
        }

        // Keep the angular rows unconstrained when angular tracking is disabled.
        if (m_angularTrackingEnabled)
        {
            iDynTree::toEigen(this->subA(m_robotVelocityVariable)).bottomRows<3>()
                = m_jacobian.bottomRows<3>();
        } else
        {
            iDynTree::toEigen(this->subA(m_robotVelocityVariable)).bottomRows<3>().setZero();
        }
    }

    m_isValid = true;

    return m_isValid;
}

bool SE3Task::setSetPoint(const manif::SE3d& I_H_F, const manif::SE3d::Tangent& mixedVelocity)
{

    bool ok = true;
    ok = ok && m_R3Controller.setDesiredState(I_H_F.translation());
    ok = ok && m_R3Controller.setFeedForward(mixedVelocity.lin());

    ok = ok && m_SO3Controller.setDesiredState(I_H_F.quat());
    ok = ok && m_SO3Controller.setFeedForward(mixedVelocity.ang());

    m_isSetPointSetAtLeastOnce = ok;

    return ok;
}

bool SE3Task::setFeedback(const manif::SE3d& I_H_F)
{
    bool ok = this->setFeedback(I_H_F.translation());
    ok = ok && this->setFeedback(I_H_F.quat());
    return ok;
}

bool SE3Task::setFeedback(const manif::SE3d::Translation& I_p_F)
{
    bool ok = true;
    if (m_usePositionExogenousFeedback)
    {
        ok = ok && m_R3Controller.setState(I_p_F);
    }

    return ok;
}

bool SE3Task::setFeedback(const manif::SO3d& I_R_F)
{
    bool ok = true;
    if (m_useOrientationExogenousFeedback)
    {
        ok = ok && m_SO3Controller.setState(I_R_F);
    }

    return ok;
}

std::size_t SE3Task::size() const
{
    return m_DoFs;
}

SE3Task::Type SE3Task::type() const
{
    return Type::equality;
}

bool SE3Task::isValid() const
{
    return m_isValid;
}

void SE3Task::setTaskControllerMode(Mode mode)
{
    m_controllerMode = mode;
}

SE3Task::Mode SE3Task::getTaskControllerMode() const
{
    return m_controllerMode;
}

void SE3Task::setAngularTrackingEnabled(bool enabled)
{
    m_angularTrackingEnabled = enabled;
}

iDynTree::FrameIndex SE3Task::getFrameIndex() const
{
    return m_frameIndex;
}
