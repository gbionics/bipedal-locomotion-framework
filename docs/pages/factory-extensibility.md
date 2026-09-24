# Factory Extensibility Guide

This guide explains how to extend the BipedalLocomotionFramework factory pattern with custom tasks in external libraries.

## Problem Background

The IK and TSID components use a factory design pattern to automatically build problems via configuration files. Tasks are identified by string names (e.g., "SE3Task", "CoMTask") in the configuration.

Previously, if you created a library that depends on BLF, you could not extend the task list with custom tasks because the factory used static initialization that didn't work across compilation unit boundaries.

## Solution

The factory has been made extensible to allow external libraries to register custom tasks at runtime. The singleton pattern has been improved to work correctly across shared library boundaries through explicit template instantiation.

## How to Register Custom Tasks

There are three methods to register custom tasks from external libraries:

### Method 1: Using the Registration Macro (Recommended for Simple Cases)

If your custom task is defined in a header file that will be included by the code that uses it:

```cpp
// MyCustomTask.h
#include <BipedalLocomotion/IK/IKLinearTask.h>

namespace MyLibrary {

class MyCustomTask : public BipedalLocomotion::IK::IKLinearTask
{
    // ... implement required methods ...
};

// Register the task with automatic static initialization
BLF_REGISTER_IK_TASK(MyLibrary::MyCustomTask);

} // namespace MyLibrary
```

**Note**: With this method, you must ensure the translation unit containing this code is linked into the final executable. The linker may optimize away the static initializer if the task class is never directly referenced.

### Method 2: Manual Registration (Recommended for External Libraries)

This is the preferred method for plugin-like architectures where tasks are defined in separate shared libraries:

```cpp
// MyCustomTask.h
#include <BipedalLocomotion/IK/IKLinearTask.h>

namespace MyLibrary {

class MyCustomTask : public BipedalLocomotion::IK::IKLinearTask
{
    // ... implement required methods ...
};

} // namespace MyLibrary

// MyLibraryInit.cpp
#include "MyCustomTask.h"
#include <BipedalLocomotion/IK/IKLinearTask.h>

namespace MyLibrary {

// Call this function before creating IK problems that need your custom tasks
void registerCustomTasks()
{
    // Register using the template helper (simplest)
    BipedalLocomotion::IK::IKLinearTaskFactory::registerBuilder<MyCustomTask>("MyCustomTask");
    
    // You can also check if registration was successful
    if (!BipedalLocomotion::IK::IKLinearTaskFactory::isBuilderRegistered("MyCustomTask"))
    {
        // Handle error
    }
}

} // namespace MyLibrary
```

Then in your main application:

```cpp
#include <MyLibrary/MyLibraryInit.h>
#include <BipedalLocomotion/IK/QPInverseKinematics.h>

int main()
{
    // Register custom tasks before building IK problems
    MyLibrary::registerCustomTasks();
    
    // Now you can use "MyCustomTask" in your configuration files
    auto ikProblem = BipedalLocomotion::IK::QPInverseKinematics::build(...);
    
    return 0;
}
```

### Method 3: Using a Custom Builder Function

For more complex scenarios where you need custom initialization logic:

```cpp
#include <BipedalLocomotion/IK/IKLinearTask.h>
#include "MyCustomTask.h"

std::shared_ptr<BipedalLocomotion::IK::IKLinearTask> myCustomBuilder()
{
    auto task = std::make_shared<MyLibrary::MyCustomTask>();
    // Custom initialization if needed
    return task;
}

void registerMyTask()
{
    BipedalLocomotion::IK::IKLinearTaskFactory::registerBuilder(
        "MyCustomTask", myCustomBuilder);
}
```

## Configuration File Usage

Once registered, custom tasks can be used in configuration files just like built-in tasks:

```toml
tasks = ["EE_TASK", "MY_CUSTOM_TASK"]

[MY_CUSTOM_TASK]
type = "MyCustomTask"  # This matches the name you registered
priority = 0
# ... other task-specific parameters ...
```

## TSID Tasks

The same approach works for TSID tasks:

```cpp
// For TSID tasks, use TSIDLinearTaskFactory
#include <BipedalLocomotion/TSID/TSIDLinearTask.h>

namespace MyLibrary {

class MyCustomTSIDTask : public BipedalLocomotion::TSID::TSIDLinearTask
{
    // ... implement required methods ...
};

void registerCustomTSIDTasks()
{
    BipedalLocomotion::TSID::TSIDLinearTaskFactory::registerBuilder<MyCustomTSIDTask>(
        "MyCustomTSIDTask");
}

} // namespace MyLibrary
```

Or use the macro:

```cpp
BLF_REGISTER_TSID_TASK(MyLibrary::MyCustomTSIDTask);
```

## Best Practices

1. **Explicit Registration**: For external libraries, prefer manual registration (Method 2) over static initialization (Method 1) to avoid linker optimization issues.

2. **Initialization Functions**: Provide a clear initialization function (e.g., `registerCustomTasks()`) that users can call before creating IK/TSID problems.

3. **Error Handling**: Check the return value of `registerBuilder()` to detect naming conflicts.

4. **Documentation**: Document the string names of your custom tasks so users know what to use in configuration files.

5. **Naming Conventions**: Use descriptive, unique names for your tasks to avoid conflicts with other libraries.

## Debugging

To see what tasks are currently registered:

```cpp
#include <BipedalLocomotion/IK/IKLinearTask.h>

// Get all registered task names
auto registeredTasks = BipedalLocomotion::IK::IKLinearTaskFactory::getRegisteredKeys();
for (const auto& taskName : registeredTasks)
{
    std::cout << "Registered: " << taskName << std::endl;
}

// Check if a specific task is registered
if (BipedalLocomotion::IK::IKLinearTaskFactory::isBuilderRegistered("MyCustomTask"))
{
    std::cout << "MyCustomTask is registered" << std::endl;
}
```

## Technical Details

### Singleton Pattern Across Shared Libraries

The factory uses Meyer's Singleton pattern (function-local static) with explicit template instantiation to ensure the same registry is shared across all shared libraries. The factories for `IKLinearTask` and `TSIDLinearTask` are explicitly instantiated in the BipedalLocomotionFramework library, ensuring a single registry instance.

### Thread Safety

The factory's `getMapFactory()` method uses function-local static initialization, which is thread-safe in C++11 and later.

### Registration Return Values

- `registerBuilder()` returns `true` if the builder was successfully registered
- `registerBuilder()` returns `false` if a builder with the same name already exists (the new builder is NOT registered in this case)
