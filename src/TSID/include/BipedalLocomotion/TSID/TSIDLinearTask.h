/**
 * @file TSIDLinearTask.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_SYSTEM_TSID_LINEAR_TASK_H
#define BIPEDAL_LOCOMOTION_SYSTEM_TSID_LINEAR_TASK_H

#include <BipedalLocomotion/System/LinearTask.h>
#include <BipedalLocomotion/System/ILinearTaskFactory.h>

#include <iDynTree/KinDynComputations.h>

/**
 * BLF_REGISTER_TSID_TASK is a macro that can be used to register an TSIDLinearTask. The key of the
 * task will be the stringified version of the Task C++ Type
 * @param _type the type of the task
 * 
 * For external libraries defining custom TSID tasks, you can also use:
 * @code
 * // Method 1: Use the macro (automatic registration)
 * BLF_REGISTER_TSID_TASK(MyCustomTask);
 * 
 * // Method 2: Manual registration (in your library's initialization)
 * TSIDLinearTaskFactory::registerBuilder<MyCustomTask>("MyCustomTask");
 * @endcode
 */
#define BLF_REGISTER_TSID_TASK(_type) BLF_REGISTER_TASK(_type, ::BipedalLocomotion::TSID::TSIDLinearTask)

namespace BipedalLocomotion
{
namespace TSID
{

/**
 * TSIDLinearTask specializes a LinearTask in the case of Task based Inverse Dynamics.
 */
struct TSIDLinearTask : public BipedalLocomotion::System::LinearTask
{
    virtual ~TSIDLinearTask() = default;

    virtual bool setKinDyn(std::shared_ptr<iDynTree::KinDynComputations> kinDyn);
};

using TSIDLinearTaskFactory = ::BipedalLocomotion::System::ILinearTaskFactory<TSIDLinearTask>;

} // namespace TSID
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_SYSTEM_TSID_LINEAR_TASK_H
