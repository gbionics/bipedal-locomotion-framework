/**
 * @file ILinearTaskFactory.h
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_SYSTEM_ILINEAR_TASK_FACTORY
#define BIPEDAL_LOCOMOTION_SYSTEM_ILINEAR_TASK_FACTORY

#include <memory>
#include <string>
#include <type_traits>
#include <utility>

#include <BipedalLocomotion/System/Factory.h>
#include <BipedalLocomotion/System/LinearTask.h>

/**
 * BLF_REGISTER_TASK is a macro that can be used to register a task given a baseType. The key of the
 * task will be the stringified version of the Task C++ Type.
 * 
 * This macro uses static initialization to register the task automatically when the library/executable
 * is loaded. For tasks defined in the same compilation unit as the IK/TSID problem, this works
 * automatically. For tasks defined in external libraries, you may need to ensure the library is
 * properly linked and loaded.
 * 
 * @param _type the type of the task
 * @param _baseType the base type from which the _task inherits.
 * 
 * @note For external libraries that define custom tasks, consider using BLF_REGISTER_TASK or
 * manually calling Factory<BaseType>::registerBuilder() in your library's initialization code
 * to ensure proper registration.
 */
#define BLF_REGISTER_TASK(_type, _baseType)                                                        \
    static std::shared_ptr<_baseType> _type##FactoryBuilder()                                      \
    {                                                                                              \
        return std::make_shared<_type>();                                                          \
    }                                                                                              \
                                                                                                   \
    namespace /* anonymous namespace for unique hook variable */                                   \
    {                                                                                              \
        struct _type##RegistrationHelper                                                           \
        {                                                                                          \
            _type##RegistrationHelper()                                                            \
            {                                                                                      \
                ::BipedalLocomotion::System::ILinearTaskFactory<_baseType>::registerBuilder(       \
                    #_type, _type##FactoryBuilder);                                                \
            }                                                                                      \
        };                                                                                         \
        static _type##RegistrationHelper _type##BuilderAutoRegHook;                                \
    }

namespace BipedalLocomotion
{
namespace System
{

/**
 * ILinearTaskFactory implements the factory design patter for constructing a linear task given its
 * type.
 * 
 * The factory is extensible and allows external libraries to register custom tasks at runtime.
 * This enables plugin-like architectures where custom tasks can be defined in separate libraries
 * and registered dynamically.
 * 
 * There are three ways to register a custom task:
 * 
 * 1. Using the BLF_REGISTER_TASK macro (for automatic registration during static initialization):
 * @code
 * class MyCustomTask : public IKLinearTask { ... };
 * BLF_REGISTER_IK_TASK(MyCustomTask);
 * @endcode
 * 
 * 2. Using the template helper method (recommended for external libraries):
 * @code
 * // In your library initialization code
 * IKLinearTaskFactory::registerBuilder<MyCustomTask>("MyCustomTask");
 * @endcode
 * 
 * 3. Using a custom builder function:
 * @code
 * std::shared_ptr<IKLinearTask> myBuilder() { return std::make_shared<MyCustomTask>(); }
 * IKLinearTaskFactory::registerBuilder("MyCustomTask", myBuilder);
 * @endcode
 */
template <typename _Task> class ILinearTaskFactory : public Factory<_Task>
{
    static_assert(std::is_base_of_v<LinearTask, _Task>,
                  "The _Task template argument of ILinearTaskFactory must inherits from "
                  "LinearTask");

public:
    using Task = typename Factory<_Task>::Type;
    using TaskBuilder = typename Factory<_Task>::Builder;

    virtual ~ILinearTaskFactory() = default;
};
} // namespace System
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_SYSTEM_ILINEAR_TASK_FACTORY
