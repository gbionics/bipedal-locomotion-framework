/**
 * @file Factory.h
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_SYSTEM_FACTORY
#define BIPEDAL_LOCOMOTION_SYSTEM_FACTORY

#include <memory>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

namespace BipedalLocomotion
{
namespace System
{

/**
 * Factory implements the factory design pattern for constructing an object of a given Type given
 * its id.
 * 
 * The Factory class is designed to be extensible, allowing external libraries to register custom
 * types at runtime. This is particularly useful for creating plugin-like architectures.
 * 
 * @note This class uses a template-based singleton pattern. To avoid issues with multiple singleton
 * instances across shared library boundaries, the factory map is properly exported and shared.
 * Each template instantiation (e.g., Factory<IKLinearTask>, Factory<TSIDLinearTask>) maintains
 * its own separate registry.
 * 
 * Example usage for registering a custom type:
 * @code
 * // Define a custom type
 * class MyCustomTask : public BaseTask { ... };
 * 
 * // Create a builder function
 * std::shared_ptr<BaseTask> myCustomTaskBuilder() {
 *     return std::make_shared<MyCustomTask>();
 * }
 * 
 * // Register the builder
 * Factory<BaseTask>::registerBuilder("MyCustomTask", myCustomTaskBuilder);
 * 
 * // Or use the template helper method
 * Factory<BaseTask>::registerBuilder<MyCustomTask>("MyCustomTask");
 * @endcode
 */
template <typename _Type> class Factory
{
public:
    using Type = _Type;
    using Builder = typename std::add_pointer<std::shared_ptr<Type>()>::type; /**< Pointer to the
                                                                                 Type builder */

protected:
    /**
     * Get the map containing a map of idKey and the associated Builder function.
     * This method uses the Meyer's Singleton pattern (function-local static) which ensures
     * proper initialization order and thread-safety in C++11 and later.
     * @return an unordered_map of idKey and pointer to create the Type
     */
    static std::unordered_map<std::string, Builder>& getMapFactory()
    {
        static std::unordered_map<std::string, Builder> m_mapFactory;
        return m_mapFactory;
    }

public:
    /**
     * Add a Builder for a given Type.
     * @param idKey the string representing the type of the Type. i.e., the stringify version of the
     * class type
     * @param classBuilder pointer to the function that creates a given Type.
     * @return true if the builder was successfully registered, false if a builder with the same key
     * already exists
     * @note This method can be used by external libraries to register custom types at runtime,
     * making the factory extensible beyond the types defined in the same compilation unit.
     * @warning When using across shared library boundaries, ensure all libraries link against
     * the same Factory instantiation. This is typically handled automatically when using shared
     * libraries correctly.
     */
    static bool registerBuilder(const std::string& idKey, Builder classBuilder)
    {
        auto result = getMapFactory().insert(std::pair<std::string, Builder>(idKey, classBuilder));
        return result.second; // returns true if insertion was successful, false if key already existed
    }

    /**
     * Template helper to register a Builder for a given Type.
     * This is a convenience method that creates the builder function automatically.
     * @tparam ConcreteType the concrete type to be built, must inherit from Type
     * @param idKey the string representing the type. i.e., the stringify version of the class type
     * @return true if the builder was successfully registered, false if a builder with the same key
     * already exists
     * @note This method is particularly useful for external libraries registering custom types.
     * 
     * Example usage:
     * @code
     * Factory<BaseTask>::registerBuilder<MyCustomTask>("MyCustomTask");
     * @endcode
     */
    template <typename ConcreteType>
    static bool registerBuilder(const std::string& idKey)
    {
        static_assert(std::is_base_of_v<Type, ConcreteType>,
                      "ConcreteType must inherit from the Factory's Type");
        
        auto builder = []() -> std::shared_ptr<Type> {
            return std::make_shared<ConcreteType>();
        };
        
        return registerBuilder(idKey, builder);
    }

    /**
     * Create an instance of a Type given its id.
     * @param idKey the string representing the type of the Type. i.e., the stringify version of the
     * class type
     * @return a pointer to the Type. In case of issues, the pointer will be a nullptr.
     */
    static std::shared_ptr<Type> createInstance(const std::string& idKey)
    {
        auto it = getMapFactory().find(idKey);
        if (it == getMapFactory().end())
        {
            return std::shared_ptr<Type>();
        }

        if (it->second == nullptr)
        {
            return std::shared_ptr<Type>();
        }

        return it->second();
    }

    /**
     * Check if a builder for the given type is registered.
     * @param idKey the string representing the type of the Type.
     * @return true if a builder is registered for the given key, false otherwise.
     */
    static bool isBuilderRegistered(const std::string& idKey)
    {
        return getMapFactory().find(idKey) != getMapFactory().end();
    }

    /**
     * Get all registered builder keys.
     * @return a vector containing all registered type keys.
     * @note This is useful for debugging and for external libraries to check what types are available.
     */
    static std::vector<std::string> getRegisteredKeys()
    {
        std::vector<std::string> keys;
        keys.reserve(getMapFactory().size());
        for (const auto& pair : getMapFactory())
        {
            keys.push_back(pair.first);
        }
        return keys;
    }

    virtual ~Factory() = default;
};
} // namespace System
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_SYSTEM_FACTORY
