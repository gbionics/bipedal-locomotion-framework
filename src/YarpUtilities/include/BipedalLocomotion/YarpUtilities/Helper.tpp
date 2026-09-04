/**
 * @file Helper.tpp
 * @authors Giulio Romualdi
 * @copyright 2019 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

// std
#include <string>
#include <type_traits>

// GenericContainer
#include <BipedalLocomotion/GenericContainer/TemplateHelpers.h>
#include <BipedalLocomotion/GenericContainer/Vector.h>

#include <BipedalLocomotion/TextLogging/Logger.h>

namespace BipedalLocomotion
{

/**
 * Helper for YARP library.
 */
namespace YarpUtilities
{

template <typename T>
struct is_element_supported_by_yarp
    : public std::disjunction<std::is_same<double, typename std::decay<T>::type>,
                              std::is_same<int, typename std::decay<T>::type>,
                              std::is_same<std::string, typename std::decay<T>::type>,
                              std::is_same<bool, typename std::decay<T>::type>>
{
};

template <typename T> bool isValueValid(const yarp::os::Value& value)
{
    if constexpr (std::is_same<T, int>::value)
    {
        return value.isInt32();
    }
    if constexpr (std::is_same<T, double>::value)
    {
        // YARP stores integral and floating-point tokens using different types. An integer is
        // nevertheless a valid value for a double parameter and asFloat64() performs the
        // corresponding numeric conversion.
        return value.isFloat64() || value.isInt32();
    }
    if constexpr (std::is_same<T, float>::value)
    {
        return value.isFloat32();
    }
    if constexpr (std::is_same<T, bool>::value)
    {
        return value.isBool() || value.isInt32();
    }
    if constexpr (std::is_same<T, std::string>::value)
    {
        return value.isString();
    }

    return false;
}

template <typename T> T convertValue(const yarp::os::Value& value)
{
    static_assert(dependent_false<T>::value,
                  "[BipedalLocomotion::YarpUtilities::convertValue] The non specialized "
                  "version has not been implemented");

    return T();
}

template <typename T>
bool getElementFromSearchable(const yarp::os::Searchable& config,
                              const std::string& key,
                              T& element)
{
    constexpr auto logPrefix = "[BipedalLocomotion::YarpUtilities::getElementFromSearchable]";

    static_assert(is_element_supported_by_yarp<T>(),
                  "[BipedalLocomotion::YarpUtilities::getElementFromSearchable] The "
                  "function getElementFromSearchable() cannot be called with the desired "
                  "element type");

    yarp::os::Value* value;
    if (!config.check(key, value))
    {
        log()->debug("{} Missing field named: {}.", logPrefix, key);
        return false;
    }

    if (!isValueValid<T>(*value))
    {
        log()->debug("{} The value named: {} is not a {}.",
                     logPrefix,
                     key,
                     BipedalLocomotion::type_name<T>());

        return false;
    }

    element = convertValue<T>(*value);
    return true;
}

template <typename T>
bool getVectorFromSearchable(const yarp::os::Searchable& config, const std::string& key, T& vector)
{
    constexpr auto logPrefix = "[BipedalLocomotion::YarpUtilities::getVectorFromSearchable]";
    using elementType = typename std::pointer_traits<decltype(vector.data())>::element_type;

    static_assert(is_element_supported_by_yarp<elementType>(),
                  "[BipedalLocomotion::YarpUtilities::getVectorFromSearchable] The "
                  "function getElementFromSearchable() cannot be called with the desired "
                  "element type");

    yarp::os::Value* value;
    if (!config.check(key, value))
    {
        log()->debug("{} Missing field named: {}.", logPrefix, key);
        return false;
    }

    if (value->isNull())
    {
        log()->debug("{} Empty input named: {}.", logPrefix, key);
        return false;
    }

    if (!value->isList())
    {
        log()->debug("{} The value named: {} is not associated to a list.", logPrefix, key);
        return false;
    }

    yarp::os::Bottle* inputPtr = value->asList();
    if (inputPtr == nullptr)
    {
        log()->debug("{} The list associated to the value named: {} is empty.", logPrefix, key);
        return false;
    }

    if (vector.size() != inputPtr->size())
    {
        // If the vector can be resize, let resize it. Otherwise it is a fix-size vector and the
        // dimensions has to be the same of list
        if constexpr (GenericContainer::is_vector<T>::value)
        {
            if (!vector.resizeVector(inputPtr->size()))
            {
                log()->debug("{} Unable to resize {}, List size: {}. Vector size: {}.",
                             logPrefix,
                             type_name<T>(),
                             inputPtr->size(),
                             vector.size());
                return false;
            }
        } else if constexpr (is_resizable<T>::value)
            vector.resize(inputPtr->size());
        else
        {
            log()->debug("{} The size of the vector does not match with the size of the list. List "
                         "size {}. Vector size {}.",
                         logPrefix,
                         inputPtr->size(),
                         vector.size());
            return false;
        }
    }

    for (int i = 0; i < inputPtr->size(); i++)
    {
        if (!isValueValid<elementType>(inputPtr->get(i)))
        {
            log()->debug("{} The element of the list associated to the value named {} is not a {}.",
                         logPrefix,
                         key,
                         BipedalLocomotion::type_name<elementType>());
            return false;
        }

        vector[i] = convertValue<elementType>(inputPtr->get(i));
    }
    return true;
}

template <typename T> void mergeSigVector(yarp::sig::Vector& vector, const T& t)
{
    if constexpr (std::is_arithmetic<T>::value)
        vector.push_back(t);

    else if constexpr (!std::is_arithmetic<T>::value)
    {
        using elementType = typename std::pointer_traits<decltype(t.data())>::element_type;
        static_assert(std::is_convertible<elementType, double>::value,
                      "[BipedalLocomotion::YarpUtilities::mergeSigVector] The element "
                      "contained in the vector cannot be converted in a double");

        if constexpr (is_iterable<T>::value)
        {
            for (auto it = t.begin(); it != t.end(); it++)
                vector.push_back(*it);

        } else if constexpr (has_square_bracket_operator<T>::value)
        {
            for (int i = 0; i < t.size(); i++)
                vector.push_back(t[i]);

        } else
            static_assert(dependent_false<T>::value,
                          "[BipedalLocomotion::YarpUtilities::mergeSigVector] The "
                          "Vector type does not have square bracket operator nor begin()/end() "
                          "methods");
        return;
    }

    else
    {
        static_assert(dependent_false<T>::value,
                      "[BipedalLocomotion::YarpUtilities::mergeSigVector] The type of "
                      "the input element cannot be handled by the function");
    }

    return;
}

template <typename T, typename... Args>
void mergeSigVector(yarp::sig::Vector& vector, const T& t, const Args&... args)
{
    mergeSigVector<T>(vector, t);
    mergeSigVector<Args...>(vector, args...);

    return;
}

template <typename... Args>
void sendVariadicVector(yarp::os::BufferedPort<yarp::sig::Vector>& port, const Args&... args)
{
    yarp::sig::Vector& vector = port.prepare();
    vector.clear();

    mergeSigVector(vector, args...);

    port.write();
}
} // namespace YarpUtilities
} // namespace BipedalLocomotion
