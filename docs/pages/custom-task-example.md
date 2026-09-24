# Custom Task Example

This example demonstrates how to create and register a custom IK task in an external library.

## Complete Example: Custom Distance Task

Let's create a custom task that maintains a minimum distance between two frames.

### Step 1: Define Your Custom Task

```cpp
// CustomDistanceTask.h
#ifndef MY_LIBRARY_CUSTOM_DISTANCE_TASK_H
#define MY_LIBRARY_CUSTOM_DISTANCE_TASK_H

#include <BipedalLocomotion/IK/IKLinearTask.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <iDynTree/KinDynComputations.h>

namespace MyLibrary {

/**
 * CustomDistanceTask ensures a minimum distance between two frames.
 * This is a custom task that extends BLF's IK capabilities.
 */
class CustomDistanceTask : public BipedalLocomotion::IK::IKLinearTask
{
private:
    std::string m_frame1Name;
    std::string m_frame2Name;
    double m_minDistance;
    bool m_isInitialized{false};
    bool m_isValid{false};
    
    std::shared_ptr<iDynTree::KinDynComputations> m_kinDyn;
    BipedalLocomotion::System::VariablesHandler::VariableDescription m_robotVelocityVariable;
    
public:
    /**
     * Initialize the task from parameters.
     * Expected parameters:
     * - robot_velocity_variable_name: name of the robot velocity variable
     * - frame_1: name of the first frame
     * - frame_2: name of the second frame
     * - min_distance: minimum distance to maintain (meters)
     */
    bool initialize(std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> paramHandler) override
    {
        auto ptr = paramHandler.lock();
        if (ptr == nullptr)
        {
            return false;
        }
        
        if (!ptr->getParameter("robot_velocity_variable_name", m_robotVelocityVariable.name))
        {
            return false;
        }
        
        if (!ptr->getParameter("frame_1", m_frame1Name))
        {
            return false;
        }
        
        if (!ptr->getParameter("frame_2", m_frame2Name))
        {
            return false;
        }
        
        if (!ptr->getParameter("min_distance", m_minDistance))
        {
            return false;
        }
        
        m_isInitialized = true;
        return true;
    }
    
    bool setKinDyn(std::shared_ptr<iDynTree::KinDynComputations> kinDyn) override
    {
        if (kinDyn == nullptr || !kinDyn->isValid())
        {
            return false;
        }
        
        m_kinDyn = kinDyn;
        return true;
    }
    
    bool setVariablesHandler(const BipedalLocomotion::System::VariablesHandler& variablesHandler) override
    {
        if (!m_isInitialized)
        {
            return false;
        }
        
        if (!variablesHandler.getVariable(m_robotVelocityVariable.name, m_robotVelocityVariable))
        {
            return false;
        }
        
        // Initialize matrices
        m_A.resize(1, variablesHandler.getNumberOfVariables());
        m_A.setZero();
        m_b.resize(1);
        
        return true;
    }
    
    bool update() override
    {
        // Implement your task logic here:
        // 1. Get current positions of both frames
        // 2. Calculate distance and its derivative
        // 3. Update m_A (Jacobian) and m_b (desired velocity)
        
        // For this example, we just set valid
        m_isValid = true;
        return true;
    }
    
    std::size_t size() const override
    {
        return 1; // One constraint for distance
    }
    
    BipedalLocomotion::System::LinearTask::Type type() const override
    {
        return BipedalLocomotion::System::LinearTask::Type::inequality;
    }
    
    bool isValid() const override
    {
        return m_isValid;
    }
};

} // namespace MyLibrary

#endif // MY_LIBRARY_CUSTOM_DISTANCE_TASK_H
```

### Step 2: Register Your Task

Create a registration function in your library:

```cpp
// MyLibraryInit.h
#ifndef MY_LIBRARY_INIT_H
#define MY_LIBRARY_INIT_H

namespace MyLibrary {

/**
 * Register all custom tasks provided by MyLibrary.
 * Call this function once before creating any IK problems that need these tasks.
 * @return true if all tasks were registered successfully
 */
bool registerCustomIKTasks();

} // namespace MyLibrary

#endif // MY_LIBRARY_INIT_H
```

```cpp
// MyLibraryInit.cpp
#include "MyLibraryInit.h"
#include "CustomDistanceTask.h"
#include <BipedalLocomotion/IK/IKLinearTask.h>
#include <iostream>

namespace MyLibrary {

bool registerCustomIKTasks()
{
    bool allRegistered = true;
    
    // Register CustomDistanceTask
    bool result = BipedalLocomotion::IK::IKLinearTaskFactory::registerBuilder<CustomDistanceTask>(
        "CustomDistanceTask");
    
    if (result)
    {
        std::cout << "[MyLibrary] CustomDistanceTask registered successfully" << std::endl;
    }
    else
    {
        std::cerr << "[MyLibrary] Failed to register CustomDistanceTask (already registered?)" << std::endl;
        allRegistered = false;
    }
    
    // Register additional custom tasks here...
    
    return allRegistered;
}

} // namespace MyLibrary
```

### Step 3: Use Your Custom Task

In your application:

```cpp
// main.cpp
#include <MyLibrary/MyLibraryInit.h>
#include <BipedalLocomotion/IK/QPInverseKinematics.h>
#include <BipedalLocomotion/IK/IntegrationBasedIK.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>

int main()
{
    // 1. Register custom tasks BEFORE building IK problems
    if (!MyLibrary::registerCustomIKTasks())
    {
        std::cerr << "Failed to register custom tasks" << std::endl;
        return 1;
    }
    
    // 2. Set up parameters as usual
    auto parameterHandler = std::make_shared<BipedalLocomotion::ParametersHandler::StdImplementation>();
    
    // ... configure your IK problem ...
    
    // 3. Build the IK problem (custom tasks are now available)
    auto ikProblem = BipedalLocomotion::IK::IntegrationBasedIKProblem::build(
        parameterHandler, kinDyn);
    
    if (!ikProblem.isValid())
    {
        std::cerr << "Failed to build IK problem" << std::endl;
        return 1;
    }
    
    // 4. Use the IK problem normally
    // ...
    
    return 0;
}
```

### Step 4: Configuration File

Now you can use your custom task in configuration files:

```toml
# ik_config.toml
tasks = ["EE_TASK", "DISTANCE_TASK"]

[EE_TASK]
type = "SE3Task"
priority = 0
frame_name = "end_effector"
kp_linear = 10.0
kp_angular = 10.0

[DISTANCE_TASK]
type = "CustomDistanceTask"  # Your custom task!
priority = 0
frame_1 = "left_hand"
frame_2 = "right_hand"
min_distance = 0.1  # Maintain at least 10cm distance
```

## Building Your Library

### CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.16)
project(MyCustomIKLibrary)

find_package(BipedalLocomotionFramework REQUIRED)

add_library(MyCustomIKLibrary SHARED
    src/CustomDistanceTask.cpp
    src/MyLibraryInit.cpp
)

target_link_libraries(MyCustomIKLibrary PUBLIC
    BipedalLocomotion::IK
)

target_include_directories(MyCustomIKLibrary PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Install your library
install(TARGETS MyCustomIKLibrary
    EXPORT MyCustomIKLibraryTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)
```

## Best Practices

1. **Always call registration functions early**: Before creating any IK/TSID problems
2. **Check registration success**: Handle the case where registration fails
3. **Use descriptive names**: Choose unique names for your custom tasks
4. **Document parameters**: Clearly document what parameters your custom tasks expect
5. **Test thoroughly**: Verify your custom tasks work in isolation before integration

## Debugging Tips

### Check what tasks are registered

```cpp
#include <BipedalLocomotion/IK/IKLinearTask.h>

void printRegisteredTasks()
{
    auto tasks = BipedalLocomotion::IK::IKLinearTaskFactory::getRegisteredKeys();
    std::cout << "Registered IK tasks:" << std::endl;
    for (const auto& task : tasks)
    {
        std::cout << "  - " << task << std::endl;
    }
}
```

### Check if a specific task is registered

```cpp
if (!BipedalLocomotion::IK::IKLinearTaskFactory::isBuilderRegistered("CustomDistanceTask"))
{
    std::cerr << "ERROR: CustomDistanceTask is not registered!" << std::endl;
    std::cerr << "Did you call MyLibrary::registerCustomIKTasks()?" << std::endl;
}
```

### Common Issues

1. **Task not found**: Ensure you called the registration function before building the IK problem
2. **Linker errors**: Make sure your library links against BipedalLocomotion::IK
3. **Duplicate registration**: Check that you're not calling registration multiple times (though this is safe, it returns false)
4. **Different factory instances**: If using across shared libraries, ensure all libraries link to the same BLF installation

## TSID Custom Tasks

The same approach works for TSID tasks - just use `TSIDLinearTask` instead:

```cpp
#include <BipedalLocomotion/TSID/TSIDLinearTask.h>

class MyCustomTSIDTask : public BipedalLocomotion::TSID::TSIDLinearTask
{
    // Implement your TSID task
};

// Register it
bool registerCustomTSIDTasks()
{
    return BipedalLocomotion::TSID::TSIDLinearTaskFactory::registerBuilder<MyCustomTSIDTask>(
        "MyCustomTSIDTask");
}
```
