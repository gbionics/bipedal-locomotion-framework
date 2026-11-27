/**
 * @file IRobotControl.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/RobotInterface/IRobotControl.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

using namespace BipedalLocomotion::RobotInterface;

bool IRobotControl::initialize(std::weak_ptr<ParametersHandler::IParametersHandler> handler)
{
    return true;
}

bool IRobotControl::setControlMode(const std::vector<RobotInterface::ControlMode>& controlModes,
                                   const std::vector<int>& jointIndices)
{
    log()->error("[IRobotControl::setControlMode] Setting the control mode of a subset of joints "
                 "is not supported by this implementation.");
    return false;
}

bool IRobotControl::setControlMode(const std::vector<RobotInterface::ControlMode>& controlModes,
                                   const std::vector<std::string>& jointNames)
{
    log()->error("[IRobotControl::setControlMode] Setting the control mode of a subset of joints "
                 "is not supported by this implementation.");
    return false;
}

bool IRobotControl::setInteractionMode(
    const std::vector<RobotInterface::InteractionMode>& interactionModes)
{
    log()->error("[IRobotControl::setInteractionMode] Setting the interaction mode is not "
                 "supported by this implementation.");
    return false;
}

bool IRobotControl::setInteractionMode(const RobotInterface::InteractionMode& interactionMode)
{
    log()->error("[IRobotControl::setInteractionMode] Setting the interaction mode is not "
                 "supported by this implementation.");
    return false;
}

bool IRobotControl::setInteractionMode(
    const std::vector<RobotInterface::InteractionMode>& interactionModes,
    const std::vector<int>& jointIndices)
{
    log()->error("[IRobotControl::setInteractionMode] Setting the interaction mode of a subset of "
                 "joints is not supported by this implementation.");
    return false;
}

bool IRobotControl::setInteractionMode(
    const std::vector<RobotInterface::InteractionMode>& interactionModes,
    const std::vector<std::string>& jointNames)
{
    log()->error("[IRobotControl::setInteractionMode] Setting the interaction mode of a subset of "
                 "joints is not supported by this implementation.");
    return false;
}
