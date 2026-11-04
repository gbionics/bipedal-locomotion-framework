/**
 * @file R3Task.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/Conversions/ManifConversions.h>
#include <BipedalLocomotion/IK/R3Task.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <iDynTree/EigenHelpers.h>
#include <iDynTree/Model.h>

using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::System;
using namespace BipedalLocomotion::IK;

bool R3Task::setKinDyn(std::shared_ptr<iDynTree::KinDynComputations> kinDyn)
{
    if ((kinDyn == nullptr) || (!kinDyn->isValid()))
    {
        log()->error("[R3Task::setKinDyn] Invalid kinDyn object.");
        return false;
    }

    m_kinDyn = kinDyn;
    return true;
}

bool R3Task::setVariablesHandler(const System::VariablesHandler& variablesHandler)
{
    if (!m_isInitialized)
    {
        log()->error("[R3Task::setVariablesHandler] The task is not initialized. Please call "
                     "initialize method.");
        return false;
    }

    // get the variable
    if (!variablesHandler.getVariable(m_robotVelocityVariable.name, m_robotVelocityVariable))
    {
        log()->error("[R3Task::setVariablesHandler] Unable to get the variable named {}.",
                     m_robotVelocityVariable.name);
        return false;
    }

    // get the variable
    if (m_robotVelocityVariable.size != m_kinDyn->getNrOfDegreesOfFreedom() + m_spatialVelocitySize)
    {
        log()->error("[R3Task::setVariablesHandler] The size of the robot velocity variable is "
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

bool R3Task::initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> paramHandler)
{
    constexpr auto errorPrefix = "[R3Task::initialize] ";

    std::string frameName = "Unknown";
    constexpr auto descriptionPrefix = "IK-R3Task - Frame name: ";

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

    // set the gains for the controllers
    Eigen::Vector3d kpLinear;
    double scalarBuffer;
    if (ptr->getParameter("kp_linear", scalarBuffer))
    {
        kpLinear.setConstant(scalarBuffer);
    } else if (!ptr->getParameter("kp_linear", kpLinear))
    {
        log()->error("{} [{} {}] Unable to get the proportional linear gain.",
                     errorPrefix,
                     descriptionPrefix,
                     frameName);
        return false;
    }
    m_R3Controller.setGains(kpLinear);

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
        m_DoFs = std::count(m_mask.begin(), m_mask.end(), true);

        // Update the mask description
        maskDescription.clear();
        for (const auto flag : m_mask)
        {
            maskDescription += boolToString(flag);
        }
    }

    m_description = descriptionPrefix + frameName + " Mask:" + maskDescription + ".";

    m_isInitialized = true;

    return true;
}

bool R3Task::update()
{
    using namespace BipedalLocomotion::Conversions;
    using namespace iDynTree;

    m_isValid = false;

    if (!m_isInitialized)
    {
        log()->error("[R3Task::update] The task is not initialized. Please call initialize "
                     "method.");
        return m_isValid;
    }

    if (!m_isSetPointSetAtLeastOnce)
    {
        log()->error("[R3Task::update] The set-point has not been set at least once.");
        return m_isValid;
    }

    auto getControllerState = [&](const auto& controller) {
        if (m_controllerMode == Mode::Enable)
            return controller.getControl().coeffs();
        else
            return controller.getFeedForward().coeffs();
    };

    // set the state
    m_R3Controller.setState(toEigen(m_kinDyn->getWorldTransform(m_frameIndex).getPosition()));

    // update the controller
    m_R3Controller.computeControlLaw();

    // store the jacobian associated to the given frame
    if (!m_kinDyn->getFrameFreeFloatingJacobian(m_frameIndex, m_jacobian))
    {
        log()->error("[R3Task::update] Unable to get the jacobian.");
        return m_isValid;
    }

    if (m_DoFs == m_linearVelocitySize)
    {
        m_b = getControllerState(m_R3Controller);
        iDynTree::toEigen(this->subA(m_robotVelocityVariable)) = m_jacobian.topRows<3>();
    } else
    {
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
    }

    m_isValid = true;

    return m_isValid;
}

bool R3Task::setSetPoint(Eigen::Ref<const Eigen::Vector3d> I_p_F,
                         Eigen::Ref<const Eigen::Vector3d> velocity)
{

    bool ok = true;
    ok = ok && m_R3Controller.setDesiredState(I_p_F);
    ok = ok && m_R3Controller.setFeedForward(velocity);

    m_isSetPointSetAtLeastOnce = ok;

    return ok;
}

std::size_t R3Task::size() const
{
    return m_DoFs;
}

R3Task::Type R3Task::type() const
{
    return Type::equality;
}

bool R3Task::isValid() const
{
    return m_isValid;
}

void R3Task::setTaskControllerMode(Mode mode)
{
    m_controllerMode = mode;
}

R3Task::Mode R3Task::getTaskControllerMode() const
{
    return m_controllerMode;
}

iDynTree::FrameIndex R3Task::getFrameIndex() const
{
    return m_frameIndex;
}
