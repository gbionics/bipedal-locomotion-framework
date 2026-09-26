/**
 * @file DinRailImplementation.cpp
 * @copyright 2026 Generative Bionics S.R.L. This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/ParametersHandler/DinRailImplementation.h>

using namespace BipedalLocomotion;
using namespace BipedalLocomotion::ParametersHandler;

namespace
{
template <typename T>
bool getVectorParameter(const dinrail::Parameters& parameters,
                        const std::string& parameterName,
                        typename GenericContainer::Vector<T>::Ref parameter)
{
    std::vector<T> values;
    if (!parameters.getParameter(parameterName, values))
    {
        return false;
    }

    if (parameter.size() != values.size() && !parameter.resizeVector(values.size()))
    {
        return false;
    }

    for (std::size_t i = 0; i < values.size(); ++i)
    {
        parameter[i] = values[i];
    }

    return true;
}

template <typename T>
void setVectorParameter(dinrail::Parameters& parameters,
                        const std::string& parameterName,
                        const typename GenericContainer::Vector<const T>::Ref parameter)
{
    std::vector<T> values(parameter.size());
    for (std::size_t i = 0; i < values.size(); ++i)
    {
        values[i] = parameter[i];
    }
    parameters.put(parameterName, values);
}
} // namespace

DinRailImplementation::DinRailImplementation(const dinrail::Parameters& parameters)
    : m_parameters(parameters)
{
}

bool DinRailImplementation::getParameter(const std::string& parameterName, int& parameter) const
{
    return m_parameters.getParameter(parameterName, parameter);
}

bool DinRailImplementation::getParameter(const std::string& parameterName, double& parameter) const
{
    return m_parameters.getParameter(parameterName, parameter);
}

bool DinRailImplementation::getParameter(const std::string& parameterName,
                                         std::string& parameter) const
{
    return m_parameters.getParameter(parameterName, parameter);
}

bool DinRailImplementation::getParameter(const std::string& parameterName, bool& parameter) const
{
    return m_parameters.getParameter(parameterName, parameter);
}

bool DinRailImplementation::getParameter(const std::string& parameterName,
                                         std::chrono::nanoseconds& parameter) const
{
    return m_parameters.getParameter(parameterName, parameter);
}

bool DinRailImplementation::getParameter(const std::string& parameterName,
                                         std::vector<bool>& parameter) const
{
    return m_parameters.getParameter(parameterName, parameter);
}

bool DinRailImplementation::getParameter(const std::string& parameterName,
                                         GenericContainer::Vector<int>::Ref parameter) const
{
    return getVectorParameter<int>(m_parameters, parameterName, parameter);
}

bool DinRailImplementation::getParameter(const std::string& parameterName,
                                         GenericContainer::Vector<double>::Ref parameter) const
{
    return getVectorParameter<double>(m_parameters, parameterName, parameter);
}

bool DinRailImplementation::getParameter(const std::string& parameterName,
                                         GenericContainer::Vector<std::string>::Ref parameter) const
{
    return getVectorParameter<std::string>(m_parameters, parameterName, parameter);
}

bool DinRailImplementation::getParameter(
    const std::string& parameterName,
    GenericContainer::Vector<std::chrono::nanoseconds>::Ref parameter) const
{
    return getVectorParameter<std::chrono::nanoseconds>(m_parameters, parameterName, parameter);
}

void DinRailImplementation::setParameter(const std::string& parameterName, const int& parameter)
{
    m_parameters.setParameter(parameterName, parameter);
}

void DinRailImplementation::setParameter(const std::string& parameterName, const double& parameter)
{
    m_parameters.setParameter(parameterName, parameter);
}

void DinRailImplementation::setParameter(const std::string& parameterName,
                                         const std::string& parameter)
{
    m_parameters.setParameter(parameterName, parameter);
}

void DinRailImplementation::setParameter(const std::string& parameterName, const char* parameter)
{
    m_parameters.setParameter(parameterName, parameter);
}

void DinRailImplementation::setParameter(const std::string& parameterName, const bool& parameter)
{
    m_parameters.setParameter(parameterName, parameter);
}

void DinRailImplementation::setParameter(const std::string& parameterName,
                                         const std::chrono::nanoseconds& parameter)
{
    m_parameters.setParameter(parameterName, parameter);
}

void DinRailImplementation::setParameter(const std::string& parameterName,
                                         const std::vector<bool>& parameter)
{
    m_parameters.setParameter(parameterName, parameter);
}

void DinRailImplementation::setParameter(const std::string& parameterName,
                                         const GenericContainer::Vector<const int>::Ref parameter)
{
    setVectorParameter<int>(m_parameters, parameterName, parameter);
}

void DinRailImplementation::setParameter(
    const std::string& parameterName, const GenericContainer::Vector<const double>::Ref parameter)
{
    setVectorParameter<double>(m_parameters, parameterName, parameter);
}

void DinRailImplementation::setParameter(
    const std::string& parameterName,
    const GenericContainer::Vector<const std::string>::Ref parameter)
{
    setVectorParameter<std::string>(m_parameters, parameterName, parameter);
}

void DinRailImplementation::setParameter(
    const std::string& parameterName,
    const GenericContainer::Vector<const std::chrono::nanoseconds>::Ref parameter)
{
    setVectorParameter<std::chrono::nanoseconds>(m_parameters, parameterName, parameter);
}

IParametersHandler::weak_ptr DinRailImplementation::getGroup(const std::string& name) const
{
    const auto& group = m_parameters.findGroup(name);
    if (group.isNull())
    {
        return weak_ptr{};
    }

    auto it = m_groupsCache.find(name);
    if (it == m_groupsCache.end())
    {
        it = m_groupsCache.emplace(name, std::make_shared<DinRailImplementation>(group)).first;
    }

    return it->second;
}

bool DinRailImplementation::setGroup(const std::string& name, shared_ptr newGroup)
{
    auto casted = std::dynamic_pointer_cast<DinRailImplementation>(newGroup);
    if (casted == nullptr)
    {
        return false;
    }

    m_parameters.addGroup(name) = casted->m_parameters;
    m_groupsCache.erase(name);
    return true;
}

std::string DinRailImplementation::toString() const
{
    return "DinRailImplementation";
}

bool DinRailImplementation::isEmpty() const
{
    return m_parameters.getValueKeys().empty() && m_parameters.getGroupKeys().empty();
}

void DinRailImplementation::clear()
{
    m_parameters.clear();
    m_groupsCache.clear();
}

IParametersHandler::shared_ptr DinRailImplementation::clone() const
{
    return std::make_shared<DinRailImplementation>(m_parameters);
}
