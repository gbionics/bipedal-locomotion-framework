/**
 * @file JointLimitsTask.cpp
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <chrono>
#include <vector>

#include <OsqpEigen/Constants.hpp>

#include <BipedalLocomotion/IK/JointLimitsTask.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <iDynTree/EigenHelpers.h>
#include <iDynTree/Model.h>

using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::IK;

bool JointLimitsTask::setKinDyn(std::shared_ptr<iDynTree::KinDynComputations> kinDyn)
{
    if ((kinDyn == nullptr) || (!kinDyn->isValid()))
    {
        log()->error("[JointLimitsTask::setKinDyn] Invalid kinDyn object.");
        return false;
    }

    m_kinDyn = kinDyn;

    // populate the limits with the one retrieved from the model
    m_upperLimits = Eigen::VectorXd::Constant(m_kinDyn->getNrOfDegreesOfFreedom(), OsqpEigen::INFTY);
    m_lowerLimits = Eigen::VectorXd::Constant(m_kinDyn->getNrOfDegreesOfFreedom(), -OsqpEigen::INFTY);

    unsigned int jointsWithLimits{0};
    for (std::size_t jointIdx = 0; jointIdx < m_kinDyn->model().getNrOfJoints(); jointIdx++)
    {
        iDynTree::IJointConstPtr joint = m_kinDyn->model().getJoint(jointIdx);

        // if the joint does not have limits skip it
        if (!joint->hasPosLimits())
        {
            continue;
        }

        // for each DoF modelled by the joint get the limits
        for (unsigned int dof = 0; dof < joint->getNrOfDOFs(); dof++)
        {
            if (!joint->getPosLimits(dof,
                                     m_lowerLimits[joint->getDOFsOffset() + dof],
                                     m_upperLimits[joint->getDOFsOffset() + dof]))
            {
                continue;
            } else
            {
                // the number of joints with lower and upper limits in the model increases
                jointsWithLimits++;
            }
        }
    }

    // check if all the joints has the associated limits
    m_isLimitConsideredForAllJoints = (m_kinDyn->getNrOfDegreesOfFreedom() == jointsWithLimits);

    return true;
}

bool JointLimitsTask::setVariablesHandler(const System::VariablesHandler& variablesHandler)
{
    constexpr auto errorPrefix = "[JointLimitsTask::setVariablesHandler]";

    System::VariablesHandler::VariableDescription robotVelocityVariable;

    if (!m_isInitialized)
    {
        log()->error("{} The task is not initialized. Please call initialize method.", errorPrefix);
        return false;
    }

    if (!variablesHandler.getVariable(m_robotVelocityVariableName, robotVelocityVariable))
    {
        log()->error("{} Error while retrieving the robot velocity variable.", errorPrefix);
        return false;
    }

    if (robotVelocityVariable.size != m_kinDyn->getNrOfDegreesOfFreedom() + 6)
    {
        log()->error("{} The size of the robot velocity variable does not match with the one "
                     "stored in kinDynComputations object. Expected: {}. Given: {}",
                     errorPrefix,
                     m_kinDyn->getNrOfDegreesOfFreedom() + 6,
                     robotVelocityVariable.size);
        return false;
    }


    // resize the matrices
    m_A.resize(2 * m_kinDyn->getNrOfDegreesOfFreedom(), variablesHandler.getNumberOfVariables());
    m_A.setZero();
    m_b.resize(2 * m_kinDyn->getNrOfDegreesOfFreedom());
    m_b.head(m_kinDyn->getNrOfDegreesOfFreedom()).setConstant(OsqpEigen::INFTY);
    m_b.tail(m_kinDyn->getNrOfDegreesOfFreedom()).setConstant(-OsqpEigen::INFTY);

    // the submatrix associated to the robot velocity
    //      _                _
    // A = | 0_{6x6} +I_{nxn} |
    //     | 0_{6x6} -I_{nxn} |
    //     |_                _|
    iDynTree::toEigen(this->subA(robotVelocityVariable))
        .topRightCorner(m_kinDyn->getNrOfDegreesOfFreedom(), m_kinDyn->getNrOfDegreesOfFreedom()).diagonal().setConstant(1);
    iDynTree::toEigen(this->subA(robotVelocityVariable))
        .bottomRightCorner(m_kinDyn->getNrOfDegreesOfFreedom(), m_kinDyn->getNrOfDegreesOfFreedom()).diagonal().setConstant(-1);

    return true;
}

bool JointLimitsTask::initialize(
    std::weak_ptr<const ParametersHandler::IParametersHandler> paramHandler)
{
    constexpr auto errorPrefix = "[JointLimitsTask::initialize]";

    if (m_kinDyn == nullptr || !m_kinDyn->isValid())
    {
        log()->error("{} KinDynComputations object is not valid.", errorPrefix);
        return false;
    }

    auto ptr = paramHandler.lock();
    if (ptr == nullptr)
    {
        log()->error("{} The parameter handler is not valid.", errorPrefix);
        return false;
    }

    if (!ptr->getParameter("robot_velocity_variable_name", m_robotVelocityVariableName))
    {
        log()->error("{} Error while retrieving the robot velocity variable.", errorPrefix);
        return false;
    }

    // set the gains for the controllers
    m_klim.resize(m_kinDyn->getNrOfDegreesOfFreedom());
    if (!ptr->getParameter("klim", m_klim))
    {
        log()->error("{} Error while retrieving the parameter named 'klim'.", errorPrefix);
        return false;
    }

    if (m_klim.size() != m_kinDyn->getNrOfDegreesOfFreedom())
    {
        log()->error("{} The size of the 'klim' parameter does not match with the number of "
                     "degrees of freedom of the robot. Expected: {}. Given: {}",
                     errorPrefix,
                     m_kinDyn->getNrOfDegreesOfFreedom(),
                     m_klim.size());
        return false;
    }

    if (m_klim.minCoeff() < 0 || m_klim.maxCoeff() > 1)
    {
        log()->error("{} 'klim' parameter must contains only positive numbers lower than 1.",
                     errorPrefix);
        return false;
    }

    if (!ptr->getParameter("sampling_time", m_samplingTime)
        || m_samplingTime <= std::chrono::nanoseconds::zero())
    {
        log()->error("{} Error while retrieving the 'sampling_time'. Please remember that must be "
                     "a "
                     "positive number",
                     errorPrefix);
        return false;
    }

    // check if the
    bool useModelLimits = true;
    if (!ptr->getParameter("use_model_limits", useModelLimits))
    {
        log()->error("{} Error while retrieving the 'use_model_limits' parameter", errorPrefix);
        return false;
    }

    if (!useModelLimits)
    {
        if (!ptr->getParameter("upper_limits", m_upperLimits)
            || !ptr->getParameter("lower_limits", m_lowerLimits))
        {
            log()->error("{} Error while retrieving the 'upper_limits' and/or the 'lower_limit' "
                         "parameters. They are required since 'use_model_limits' is set to false.",
                         errorPrefix);
            return false;
        }
        // if the upper and lower limits are passed as parameters the user has to specify all of
        // them
        m_isLimitConsideredForAllJoints = true;
    }

    // set the description
    m_description = "Joint limits task";

    // initialize the joint position
    m_jointPosition = Eigen::VectorXd::Zero(m_kinDyn->getNrOfDegreesOfFreedom());

    m_isInitialized = true;

    return true;
}

bool JointLimitsTask::update()
{
    constexpr auto errorPrefix = "[JointLimitsTask::update]";

    m_isValid = false;

    const double dT = std::chrono::duration<double>(m_samplingTime).count();

    if (!m_kinDyn->getJointPos(m_jointPosition))
    {
        log()->error("{} Unable to get the joint position.", errorPrefix);
        return m_isValid;
    }

    if (m_isLimitConsideredForAllJoints)
    {
        m_b.head(m_kinDyn->getNrOfDegreesOfFreedom()).noalias()
            = m_klim.asDiagonal() * (m_upperLimits - m_jointPosition) / dT;

        m_b.tail(m_kinDyn->getNrOfDegreesOfFreedom()).noalias()
            = m_klim.asDiagonal() * (-m_lowerLimits + m_jointPosition) / dT;
    } else
    {
        Eigen::DenseBase<Eigen::Matrix<double, -1, 1, 0, -1, 1>>::SegmentReturnType upperLimitPart
                = m_b.head(m_kinDyn->getNrOfDegreesOfFreedom());
        Eigen::DenseBase<Eigen::Matrix<double, -1, 1, 0, -1, 1>>::SegmentReturnType lowerLimitPart
                = m_b.tail(m_kinDyn->getNrOfDegreesOfFreedom());

        for (int i = 0; i < m_upperLimits.size(); i++)
        {
            if (m_upperLimits[i] != OsqpEigen::INFTY)
            {
                upperLimitPart(i) = m_klim(i) * (m_upperLimits(i) - m_jointPosition(i)) //
                                    / dT;
            } else
            {
                upperLimitPart(i) = OsqpEigen::INFTY;
            }
        }
        for (int i = 0; i < m_lowerLimits.size(); i++)
        {
            if (m_lowerLimits[i] != -OsqpEigen::INFTY)
            {
                lowerLimitPart(i) = m_klim(i) * (m_jointPosition(i) - m_lowerLimits(i)) //
                                    / dT;
            } else
            {
                lowerLimitPart(i) = OsqpEigen::INFTY;
            }
        }
    }

    m_isValid = true;
    return m_isValid;
}

std::size_t JointLimitsTask::size() const
{
    constexpr auto errorMessage = "[JointLimitsTask::size] Please call setKinDyn method before. "
                                  "A size equal to zero will be returned.";

    assert((m_kinDyn != nullptr) && errorMessage);

    if (m_kinDyn == nullptr)
    {
        log()->warn(errorMessage);
        return 0;
    }
    return 2 * m_kinDyn->getNrOfDegreesOfFreedom();
}

JointLimitsTask::Type JointLimitsTask::type() const
{
    return Type::inequality;
}

bool JointLimitsTask::isValid() const
{
    return m_isValid;
}
