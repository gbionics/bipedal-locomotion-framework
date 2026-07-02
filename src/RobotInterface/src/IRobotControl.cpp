/**
 * @file IRobotControl.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/RobotInterface/IRobotControl.h>

using namespace BipedalLocomotion::RobotInterface;

bool IRobotControl::initialize(std::weak_ptr<ParametersHandler::IParametersHandler> handler)
{
    return true;
}


