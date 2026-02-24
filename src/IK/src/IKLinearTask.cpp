/**
 * @file IKLinearTask.cpp
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */
#include <BipedalLocomotion/IK/IKLinearTask.h>

using namespace BipedalLocomotion::IK;

bool IKLinearTask::setKinDyn(std::shared_ptr<iDynTree::KinDynComputations> kinDyn)
{
    return true;
}

// Explicit instantiation of the factory template to ensure proper singleton behavior
// across shared library boundaries. This ensures that all libraries using IKLinearTaskFactory
// share the same registry of task builders.
template class BipedalLocomotion::System::Factory<IKLinearTask>;
template class BipedalLocomotion::System::ILinearTaskFactory<IKLinearTask>;
