/**
 * @file TSIDLinearTask.cpp
 * @authors Giulio Romualdi
 * @copyright 2024 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/TSID/TSIDLinearTask.h>

using namespace BipedalLocomotion::TSID;

bool TSIDLinearTask::setKinDyn(std::shared_ptr<iDynTree::KinDynComputations> kinDyn)
{
    return true;
}

// Explicit instantiation of the factory template to ensure proper singleton behavior
// across shared library boundaries. This ensures that all libraries using TSIDLinearTaskFactory
// share the same registry of task builders.
template class BipedalLocomotion::System::Factory<TSIDLinearTask>;
template class BipedalLocomotion::System::ILinearTaskFactory<TSIDLinearTask>;
