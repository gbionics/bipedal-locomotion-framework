/**
 * @file SO3Task.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/Conversions/ManifConversions.h>
#include <BipedalLocomotion/IK/SO3Task.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <iDynTree/EigenHelpers.h>
#include <iDynTree/Model.h>

using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::System;
using namespace BipedalLocomotion::IK;

bool SO3Task::setKinDyn(std::shared_ptr<iDynTree::KinDynComputations> kinDyn)
{
    if ((kinDyn == nullptr) || (!kinDyn->isValid()))
    {
        log()->error("[SO3Task::setKinDyn] Invalid kinDyn object.");
        return false;
    }

    m_kinDyn = kinDyn;
    return true;
}

bool SO3Task::setVariablesHandler(const System::VariablesHandler& variablesHandler)
{
    if (!m_isInitialized)
    {
        log()->error("[SO3Task::setVariablesHandler] The task is not initialized. Please call "
                     "initialize method.");
        return false;
    }

    // get the variable
    if (!variablesHandler.getVariable(m_robotVelocityVariable.name, m_robotVelocityVariable))
    {
        log()->error("[SO3Task::setVariablesHandler] Unable to get the variable named {}.",
                     m_robotVelocityVariable.name);
        return false;
    }

    if (m_robotVelocityVariable.size != m_kinDyn->getNrOfDegreesOfFreedom() + m_spatialVelocitySize)
    {
        log()->error("[SO3Task::setVariablesHandler] The size of the robot velocity variable is "
                     "different from the one expected. Expected size: {}. Given size: {}.",
                     m_kinDyn->getNrOfDegreesOfFreedom() + m_spatialVelocitySize,
                     m_robotVelocityVariable.size);
        return false;
    }

    // resize the matrices
    m_A.resize(m_angularVelocitySize, variablesHandler.getNumberOfVariables());
    m_A.setZero();
    m_b.resize(m_angularVelocitySize);
    m_jacobian.resize(m_spatialVelocitySize,
                      m_kinDyn->getNrOfDegreesOfFreedom() + m_spatialVelocitySize);

    return true;
}

bool SO3Task::initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> paramHandler)
{
    constexpr std::string_view errorPrefix = "[SO3Task::initialize] ";

    std::string frameName = "Unknown";
    constexpr std::string_view descriptionPrefix = "SO3Task Optimal Control Element - Frame name: ";

    if (m_kinDyn == nullptr || !m_kinDyn->isValid())
    {
        log()->error("{}, [{} {}] KinDynComputations object is not valid.",
                     errorPrefix,
                     descriptionPrefix,
                     frameName);

        return false;
    }

    if (m_kinDyn->getFrameVelocityRepresentation()
        != iDynTree::FrameVelocityRepresentation::MIXED_REPRESENTATION)
    {
        log()->error("{}, [{} {}] The task supports only quantities expressed in MIXED "
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
        log()->error("{}, [{} {}] The parameter handler is not valid.",
                     errorPrefix,
                     descriptionPrefix,
                     frameName);
        return false;
    }

    std::string robotVelocityVariableName;
    if (!ptr->getParameter("robot_velocity_variable_name", m_robotVelocityVariable.name))
    {
        log()->error("{}, [{} {}] Error while retrieving the robot velocity variable.",
                     errorPrefix,
                     descriptionPrefix,
                     frameName);
        return false;
    }

    if (!ptr->getParameter("frame_name", frameName)
        || (m_frameIndex = m_kinDyn->model().getFrameIndex(frameName))
               == iDynTree::FRAME_INVALID_INDEX)
    {
        log()->error("{}, [{} {}] Error while retrieving the frame that should be controlled.",
                     errorPrefix,
                     descriptionPrefix,
                     frameName);
        return false;
    }

    // set the gains for the controllers
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
        log()->error("{}, [{} {}] Unable to get the proportional angular gain.",
                     errorPrefix,
                     descriptionPrefix,
                     frameName);
        return false;
    }

    // set the description
    m_description = std::string(descriptionPrefix) + frameName + ".";

    m_isInitialized = true;
    return true;
}

bool SO3Task::update()
{
    using namespace BipedalLocomotion::Conversions;
    using namespace iDynTree;

    m_isValid = false;

    if (!m_isInitialized)
    {
        log()->error("[SO3Task::update] The task is not initialized. Please call initialize "
                     "method.");
        return m_isValid;
    }

    if (!m_isSetPointSetAtLeastOnce)
    {
        log()->error("[SO3Task::update] The set point has not been set at least once.");
        return m_isValid;
    }

    // set the state
    m_SO3Controller.setState(toManifRot(m_kinDyn->getWorldTransform(m_frameIndex).getRotation()));

    // update the controller
    m_SO3Controller.computeControlLaw();

    m_b = m_SO3Controller.getControl().coeffs();

    if (!m_kinDyn->getFrameFreeFloatingJacobian(m_frameIndex, m_jacobian))
    {
        log()->error("[SO3Task::update] Unable to get the jacobian.");
        return m_isValid;
    }

    iDynTree::toEigen(this->subA(m_robotVelocityVariable)) = m_jacobian.bottomRows<3>();

    m_isValid = true;

    return m_isValid;
}

bool SO3Task::setSetPoint(const manif::SO3d& I_R_F, const manif::SO3d::Tangent& angularVelocity)
{

    bool ok = true;
    ok = ok && m_SO3Controller.setDesiredState(I_R_F);
    ok = ok && m_SO3Controller.setFeedForward(angularVelocity);

    m_isSetPointSetAtLeastOnce = ok;

    return ok;
}

std::size_t SO3Task::size() const
{
    return m_angularVelocitySize;
}

SO3Task::Type SO3Task::type() const
{
    return Type::equality;
}

bool SO3Task::isValid() const
{
    return m_isValid;
}

iDynTree::FrameIndex SO3Task::getFrameIndex() const
{
    return m_frameIndex;
}
