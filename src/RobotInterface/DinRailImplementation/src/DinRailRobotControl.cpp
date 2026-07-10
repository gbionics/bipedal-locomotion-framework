/**
 * @file DinRailRobotControl.cpp
 * @authors Giulio Romualdi
 * @copyright Generative Bionics S.R.L. This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <cmath>
#include <memory>
#include <vector>

#include <yarp/dev/IAxisInfo.h>
#include <yarp/dev/PolyDriver.h>

#include <dinrail/IImpedanceAllSetPointsControl.h>
#include <dinrail/VectorProxy.h>

#include <BipedalLocomotion/RobotInterface/DinRailRobotControl.h>
#include <BipedalLocomotion/RobotInterface/JointType.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

using namespace BipedalLocomotion::RobotInterface;

struct DinRailRobotControl::Impl
{
    /** Pointer to the DinRail-specific impedance set-point interface. */
    dinrail::IImpedanceAllSetPointsControl* impedanceInterface{nullptr};

    /** Joint type list (revolute vs prismatic), populated in setDriver(). */
    std::vector<JointType> jointTypes;

    /** Number of actuated degrees of freedom. */
    std::size_t actuatedDOFs{0};

    /**
     * Scratch buffers used in setImpedanceSetPoints() to avoid per-call heap
     * allocations in the control loop.
     */
    std::vector<double> posBuffer;
    std::vector<double> velBuffer;
    std::vector<double> stiffnessBuffer;
    std::vector<double> dampingBuffer;
};

DinRailRobotControl::DinRailRobotControl()
    : m_pimpl(std::make_unique<Impl>())
{
}

DinRailRobotControl::~DinRailRobotControl() = default;

bool DinRailRobotControl::initialize(std::weak_ptr<ParametersHandler::IParametersHandler> handler)
{
    // Delegate entirely to the parent; no additional parameters are needed.
    return YarpRobotControl::initialize(handler);
}

bool DinRailRobotControl::setDriver(std::shared_ptr<yarp::dev::PolyDriver> robotDevice)
{
    constexpr auto errorPrefix = "[DinRailRobotControl::setDriver]";

    // First set up all standard YARP interfaces via the parent implementation.
    if (!YarpRobotControl::setDriver(robotDevice))
    {
        log()->error("{} Failed to initialize the base YarpRobotControl.", errorPrefix);
        return false;
    }

    // Additionally acquire the DinRail-specific impedance interface.
    if (!robotDevice->view(m_pimpl->impedanceInterface) || m_pimpl->impedanceInterface == nullptr)
    {
        log()->error("{} Cannot load the dinrail::IImpedanceAllSetPointsControl interface. "
                     "Make sure the device is a DinRailControlBoardNWCYarp.",
                     errorPrefix);
        return false;
    }

    // Cache the joint types so that setImpedanceSetPoints() can perform the
    // correct radian-to-degree conversion without querying the device each call.
    yarp::dev::IAxisInfo* axisInfo{nullptr};
    if (!robotDevice->view(axisInfo) || axisInfo == nullptr)
    {
        log()->error("{} Cannot load the yarp::dev::IAxisInfo interface.", errorPrefix);
        return false;
    }

    int dofs{0};
    if (!axisInfo->getAxes(&dofs) || dofs <= 0)
    {
        log()->error("{} Cannot retrieve the number of actuated DoFs.", errorPrefix);
        return false;
    }
    m_pimpl->actuatedDOFs = static_cast<std::size_t>(dofs);

    m_pimpl->jointTypes.resize(m_pimpl->actuatedDOFs);
    for (int i = 0; i < dofs; ++i)
    {
        yarp::dev::JointTypeEnum jType;
        axisInfo->getJointType(i, jType);
        m_pimpl->jointTypes[i] = (jType == yarp::dev::VOCAB_JOINTTYPE_REVOLUTE)
                                     ? JointType::REVOLUTE
                                     : JointType::PRISMATIC;
    }

    // Pre-allocate conversion buffers.
    m_pimpl->posBuffer.resize(m_pimpl->actuatedDOFs);
    m_pimpl->velBuffer.resize(m_pimpl->actuatedDOFs);
    m_pimpl->stiffnessBuffer.resize(m_pimpl->actuatedDOFs);
    m_pimpl->dampingBuffer.resize(m_pimpl->actuatedDOFs);

    return true;
}

bool DinRailRobotControl::setImpedanceSetPoints(Eigen::Ref<const Eigen::VectorXd> position,
                                                Eigen::Ref<const Eigen::VectorXd> velocity,
                                                Eigen::Ref<const Eigen::VectorXd> torque,
                                                Eigen::Ref<const Eigen::VectorXd> stiffness,
                                                Eigen::Ref<const Eigen::VectorXd> damping)
{
    constexpr auto errorPrefix = "[DinRailRobotControl::setImpedanceSetPoints]";

    if (m_pimpl->impedanceInterface == nullptr)
    {
        log()->error("{} The impedance interface is not ready. Did you call setDriver()?",
                     errorPrefix);
        return false;
    }

    const auto n = static_cast<Eigen::Index>(m_pimpl->actuatedDOFs);

    if (position.size() != n || velocity.size() != n || torque.size() != n || stiffness.size() != n
        || damping.size() != n)
    {
        log()->error("{} Input vector size mismatch. Expected {} for all inputs. "
                     "Got position={}, velocity={}, torque={}, stiffness={}, damping={}.",
                     errorPrefix,
                     n,
                     position.size(),
                     velocity.size(),
                     torque.size(),
                     stiffness.size(),
                     damping.size());
        return false;
    }

    // Convert position (rad→deg) and velocity (rad/s→deg/s) for revolute joints.
    // Stiffness (Nm/rad→Nm/deg) and damping (Nms/rad→Nms/deg) are also scaled by
    // the same factor for revolute joints: 1 Nm/rad = (π/180) Nm/deg.
    // Prismatic joints are forwarded in SI units without conversion.
    constexpr double radToDeg = 180.0 / M_PI;
    constexpr double degToRad = M_PI / 180.0; // scaling factor for stiffness/damping
    for (std::size_t i = 0; i < m_pimpl->actuatedDOFs; ++i)
    {
        if (m_pimpl->jointTypes[i] == JointType::REVOLUTE)
        {
            m_pimpl->posBuffer[i] = radToDeg * position[static_cast<Eigen::Index>(i)];
            m_pimpl->velBuffer[i] = radToDeg * velocity[static_cast<Eigen::Index>(i)];
            m_pimpl->stiffnessBuffer[i] = degToRad * stiffness[static_cast<Eigen::Index>(i)];
            m_pimpl->dampingBuffer[i] = degToRad * damping[static_cast<Eigen::Index>(i)];
        } else
        {
            m_pimpl->posBuffer[i] = position[static_cast<Eigen::Index>(i)];
            m_pimpl->velBuffer[i] = velocity[static_cast<Eigen::Index>(i)];
            m_pimpl->stiffnessBuffer[i] = stiffness[static_cast<Eigen::Index>(i)];
            m_pimpl->dampingBuffer[i] = damping[static_cast<Eigen::Index>(i)];
        }
    }

    // Build non-owning VectorProxy views over the converted buffers.
    // VectorProxy<const double>::Ref can be constructed from any contiguous
    // container with .data() and .size(), which includes std::vector and
    // Eigen::Ref<const Eigen::VectorXd>.
    if (!m_pimpl->impedanceInterface->setSetPoints(m_pimpl->posBuffer,
                                                   m_pimpl->velBuffer,
                                                   torque,
                                                   m_pimpl->stiffnessBuffer,
                                                   m_pimpl->dampingBuffer))
    {
        log()->error("{} Failed to set impedance setpoints.", errorPrefix);
        return false;
    }

    return true;
}
