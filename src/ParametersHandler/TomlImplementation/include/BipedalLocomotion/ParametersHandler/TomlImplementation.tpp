/**
 * @file TomlImplementation.tpp
 * @authors Giulio Romualdi
 * @copyright 2020,2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_PARAMETERS_HANDLER_TOML_IMPLEMENTATION_TPP
#define BIPEDAL_LOCOMOTION_PARAMETERS_HANDLER_TOML_IMPLEMENTATION_TPP

#include <chrono>
#include <type_traits>

#include <BipedalLocomotion/GenericContainer/TemplateHelpers.h>
#include <BipedalLocomotion/ParametersHandler/TomlImplementation.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

namespace BipedalLocomotion
{
namespace ParametersHandler
{

template <typename T>
bool TomlImplementation::getParameterPrivate(const std::string& parameterName, T& parameter) const
{
    constexpr auto logPrefix = "[TomlImplementation::getParameterPrivate]";
    auto paramToml = m_container[parameterName];

    if constexpr (std::is_same_v<T, std::chrono::nanoseconds>)
    {
        using namespace std::chrono_literals;
        if (paramToml.is_time())
        {

            const toml::time time = paramToml.value<toml::time>().value();
            parameter = time.hour * 1h //
                        + time.minute * 1min //
                        + time.second * 1s //
                        + time.nanosecond * 1ns;
            return true;
        }

        if (paramToml.is<double>())
        {
            // Rounded, not truncated: most decimal literals are not representable in binary, so
            // duration_cast would turn 0.3 into 299999999ns instead of 300ms.
            parameter = std::chrono::round<std::chrono::nanoseconds>(
                std::chrono::duration<double>(paramToml.value<double>().value()));
            return true;
        }

        BipedalLocomotion::log()->debug("{} The type of the parameter named {} is "
                                        "different from expected one.",
                                        logPrefix,
                                        parameterName);
        return false;
    } else if constexpr (std::is_scalar<T>::value || is_string<T>::value)
    {
        if constexpr (std::is_integral<T>() && !std::is_same_v<T, bool>)
        {
            if (!paramToml.is_integer())
            {
                BipedalLocomotion::log()->debug("{} The type of the parameter named {} is "
                                                "different from expected one.",
                                                logPrefix,
                                                parameterName);
                return false;
            }
        } else
        {
            if (!paramToml.is<T>())
            {
                BipedalLocomotion::log()->debug("{} The type of the parameter named {} is "
                                                "different from expected one.",
                                                logPrefix,
                                                parameterName);

                return false;
            }
        }
        parameter = paramToml.value<T>().value();
        return true;


    } else // otherwise it is considered as a vector
    {
        using elementType = typename T::value_type;

        if (!paramToml.is_array())
        {
            BipedalLocomotion::log()->debug("{} The parameter named {} is not an array.",
                                            logPrefix,
                                            parameterName);
            return false;
        }

        const toml::array& array = *paramToml.as_array();

        // check the size
        if (array.size() != parameter.size())
        {
            // If the vector can be resize, let resize it. Otherwise it is a fix-size vector and
            // the dimensions has to be the same of list
            if constexpr (GenericContainer::is_vector<T>::value)
            {
                if (!parameter.resizeVector(array.size()))
                {
                    BipedalLocomotion::log()->debug("{} Unable to resize {} List size: {}. Vector "
                                                    "size: {}.",
                                                    logPrefix,
                                                    type_name<T>(),
                                                    array.size(),
                                                    parameter.size());

                    return false;
                }
            } else if constexpr (is_resizable<T>::value)
                parameter.resize(array.size());
            else
            {
                BipedalLocomotion::log()->debug("{} The size of the vector does not match with the "
                                                "size of the list. "
                                                "List size: {}. Vector size: {}.",
                                                logPrefix,
                                                array.size(),
                                                parameter.size());
                return false;
            }
        }

        // check if the array is homogeneous
        if constexpr (std::is_same_v<elementType, int>)
        {
            if (!array.is_homogeneous(toml::node_type::integer))
            {
                BipedalLocomotion::log()->debug("{} The type of the parameter named {} is "
                                                "different from expected one.",
                                                logPrefix,
                                                parameterName);

                return false;
            }
        } else if constexpr (std::is_same_v<elementType, std::chrono::nanoseconds>)
        {
            if (!array.is_homogeneous(toml::node_type::time) || !array.is_homogeneous<double>())
            {
                BipedalLocomotion::log()->debug("{} The type of the parameter named {} is "
                                                "different from expected one.",
                                                logPrefix,
                                                parameterName);

                return false;
            }
        } else
        {
            if (!array.is_homogeneous<elementType>())
            {
                BipedalLocomotion::log()->debug("{} The type of the parameter named {} is "
                                                "different from expected one.",
                                                logPrefix,
                                                parameterName);

                return false;
            }
        }

        for (int i = 0; i < array.size(); i++)
        {
            if constexpr (std::is_same_v<elementType, std::chrono::nanoseconds>)
            {
                if (array[i].is_time())
                {
                    using namespace std::chrono_literals;
                    const toml::time time = array[i].value<toml::time>().value();
                    parameter[i] = time.hour * 1h //
                                   + time.minute * 1min //
                                   + time.second * 1s //
                                   + time.nanosecond * 1ns;
                } else // in this case is a double
                {
                    parameter[i] = std::chrono::round<std::chrono::nanoseconds>(
                        std::chrono::duration<double>(array[i].value<double>().value()));
                }
            } else
            {
                parameter[i] = array[i].value<elementType>().value();
            }
        }

            return true;
    }
}

template <typename T>
void TomlImplementation::setParameterPrivate(const std::string& parameterName, const T& parameter)
{
    if constexpr (std::is_same<T, std::chrono::nanoseconds>::value)
    {
        toml::time time;
        time.hour = std::chrono::duration_cast<std::chrono::hours>(parameter).count();
        time.minute
            = std::chrono::duration_cast<std::chrono::minutes>(parameter % std::chrono::hours(1))
                  .count();
        time.second
            = std::chrono::duration_cast<std::chrono::seconds>(parameter % std::chrono::minutes(1))
                  .count();
        time.nanosecond = std::chrono::duration_cast<std::chrono::nanoseconds>(
                              parameter % std::chrono::seconds(1))
                              .count();

        m_container.insert_or_assign(parameterName, time);

    } else if constexpr (std::is_scalar<T>::value || is_string<T>::value)
    {
        m_container.insert_or_assign(parameterName, parameter);
    } else // if it is a vector
    {
        m_container.insert_or_assign(parameterName, toml::array());
        using elementType = typename T::value_type;

        for (const auto& element : parameter)
        {
            if constexpr (std::is_same_v<elementType, std::chrono::nanoseconds>)
            {
                toml::time time;
                time.hour = std::chrono::duration_cast<std::chrono::hours>(element).count();
                time.minute = std::chrono::duration_cast<std::chrono::minutes>(
                                  element % std::chrono::hours(1))
                                  .count();
                time.second = std::chrono::duration_cast<std::chrono::seconds>(
                                  element % std::chrono::minutes(1))
                                  .count();
                time.nanosecond = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                      element % std::chrono::seconds(1)).count();

                m_container[parameterName].as_array()->push_back(time);
            } else
            {
                m_container[parameterName].as_array()->push_back(element);
            }
        }
    }
}

} // namespace ParametersHandler
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_PARAMETERS_HANDLER_TOML_IMPLEMENTATION_TPP
