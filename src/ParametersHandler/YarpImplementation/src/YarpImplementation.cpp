/**
 * @file YarpImplementation.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <chrono>
#include <regex>
#include <yarp/os/Bottle.h>

#include <cassert>
#include <string>

#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

using namespace BipedalLocomotion::ParametersHandler;

bool YarpImplementation::stringToDuration(const std::string& string, std::chrono::nanoseconds& time)
{
    using namespace std::chrono_literals;

    constexpr auto logPrefix = "[YarpImplementation::stringToDuration]";

    // It matches a string of the type HH:MM:SS.xxxxxx with H, M, S, x a number from 0 to 9.
    // In detail:
    // [0-9]{2}:[0-9]{2}:[0-9]{2}\.[0-9]{6}\b means:
    // - Match a single character present in the list below [0-9]
    //   - {2} matches the previous token exactly 2 times
    //   - 0-9 matches a single character in the range between 0 and 9
    // - : matches the character :
    // - Match a single character present in the list below [0-9]
    //   - {2} matches the previous token exactly 2 times
    //   - 0-9 matches a single character in the range between 0 and 9
    // - : matches the character :
    // - Match a single character present in the list below [0-9]
    //   - {2} matches the previous token exactly 2 times
    //   - 0-9 matches a single character in the range between 0 and 9
    // - \. matches the character .
    // - Match a single character present in the list below [0-9]
    //   - {6} matches the previous token exactly 6 times
    //   - 0-9 matches a single character in the range between 0 and 9
    // - \b assert position at a word boundary: (^\w|\w$|\W\w|\w\W)
    //
    // Please note that instead of having \. and \b we have \\. and \\b. Indeed the first \ is used
    // as escape character for the string literal.
    const std::regex expression("[0-9]{2}:[0-9]{2}:[0-9]{2}\\.[0-9]{6}\\b");
    if (!std::regex_match(string, expression))
    {
        log()->debug("{} The parameter do not match the excepted expression. Expected: "
                     "'HH:MM:SS.xxxxxx'. Get: {}.",
                     logPrefix,
                     string);
        return false;
    }

    std::stringstream stringstream{string};
    std::vector<std::string> strs;
    std::string temp;
    while (std::getline(stringstream, temp, ':'))
    {
        strs.push_back(temp);
    }

    // split the seconds and microseconds
    std::stringstream stringstreamSeconds{strs[2]};
    std::vector<std::string> strsSeconds;
    while (std::getline(stringstreamSeconds, temp, '.'))
    {
        strsSeconds.push_back(temp);
    }

    time = stoi(strs[0]) * 1h + stoi(strs[1]) * 1min + stoi(strsSeconds[0]) * 1s
           + stoi(strsSeconds[1]) * 1us;

    return true;
}

std::string YarpImplementation::durationToString(const std::chrono::nanoseconds& parameter)
{
    using namespace std::chrono_literals;

    constexpr std::size_t zeroPadding = 2;
    constexpr std::size_t zeroPaddingMicroseconds = 6;

    std::string hours = std::to_string(std::chrono::duration_cast<std::chrono::hours>(parameter).count());
    std::string minutes = std::to_string(
        std::chrono::duration_cast<std::chrono::minutes>(parameter % 1h).count());
    std::string seconds = std::to_string(
        std::chrono::duration_cast<std::chrono::seconds>(parameter % 1min).count());
    std::string microseconds = std::to_string(
        std::chrono::duration_cast<std::chrono::microseconds>(parameter % 1s).count());
    // apply the zero padding
    std::string timeString = std::string(zeroPadding - std::min(zeroPadding, hours.length()), '0') + hours + ":";
    // apply the zero padding
    timeString += std::string(zeroPadding - std::min(zeroPadding, minutes.length()), '0') + minutes + ":";
    // apply the zero padding
    timeString += std::string(zeroPadding - std::min(zeroPadding, seconds.length()), '0') + seconds + ".";
    // apply the zero padding
    timeString += std::string(zeroPaddingMicroseconds
                                  - std::min(zeroPaddingMicroseconds, microseconds.length()),
                              '0') + microseconds;

    return timeString;
}

template <>
void YarpImplementation::setParameterPrivate<std::vector<bool>>(const std::string& parameterName,
                                                                const std::vector<bool>& parameter)
{
    yarp::os::Value yarpValue;
    auto property = yarpValue.asList();
    property->add(yarp::os::Value(parameterName));

    yarp::os::Value yarpNewList;
    auto newList = yarpNewList.asList();

    for (bool element : parameter)
    {
        // if the parameter is a boolean we cannot use the usual way to add the new parameter
        // Please check https://github.com/robotology/yarp/issues/2584#issuecomment-847778679
        if (element)
        {
            newList->add(yarp::os::Value::makeValue("true"));
        } else
        {
            newList->add(yarp::os::Value::makeValue("false"));
        }
    }

    property->add(yarpNewList);

    m_lists[parameterName] = std::make_shared<YarpImplementation>(yarpValue);
}

bool YarpImplementation::getParameter(const std::string& parameterName, int& parameter) const
{
    return getParameterPrivate(parameterName, parameter);
}

bool YarpImplementation::getParameter(const std::string& parameterName, double& parameter) const
{
    return getParameterPrivate(parameterName, parameter);
}

bool YarpImplementation::getParameter(const std::string& parameterName,
                                      std::string& parameter) const
{
    return getParameterPrivate(parameterName, parameter);
}

bool YarpImplementation::getParameter(const std::string& parameterName, bool& parameter) const
{
    return getParameterPrivate(parameterName, parameter);
}

bool YarpImplementation::getParameter(const std::string& parameterName,
                                      std::chrono::nanoseconds& parameter) const
{
    constexpr auto logPrefix = "[YarpImplementation::getParameter]";

    // we support two types of formatting.
    // - The RFC 3339  specification, i.e., HH:MM:SS:xxxxxx
    // - A double. In this case we assume that the time is stored as seconds
    double doubleParam;
    if (getParameterPrivate(parameterName, doubleParam))
    {
        // Rounded, not truncated: most decimal literals are not representable in binary, so
        // duration_cast would turn 0.3 into 299999999ns instead of 300ms.
        parameter = std::chrono::round<std::chrono::nanoseconds>(
            std::chrono::duration<double>(doubleParam));
        return true;
    }


    std::string stringParam;
    if (getParameterPrivate(parameterName, stringParam))
    {
        return this->stringToDuration(stringParam, parameter);
    }

    return false;
}

bool YarpImplementation::getParameter(const std::string& parameterName,
                                      GenericContainer::Vector<int>::Ref parameter) const
{
    return getParameterPrivate(parameterName, parameter);
}

bool YarpImplementation::getParameter(const std::string& parameterName,
                                      GenericContainer::Vector<double>::Ref parameter) const
{
    return getParameterPrivate(parameterName, parameter);
}

bool YarpImplementation::getParameter(const std::string& parameterName,
                                      GenericContainer::Vector<std::string>::Ref parameter) const
{
    return getParameterPrivate(parameterName, parameter);
}

bool YarpImplementation::getParameter(
    const std::string& parameterName,
    GenericContainer::Vector<std::chrono::nanoseconds>::Ref parameter) const
{
    constexpr auto logPrefix = "[YarpImplementation::getParameter]";

    auto convertVectorParamToVectorChrono
        = [logPrefix, &parameter, &parameterName](const auto& tempVector,
                                                  const auto& conversionFunction) -> bool {
        if (parameter.size() != tempVector.size())
        {
            if (!parameter.resizeVector(tempVector.size()))
            {
                log()->debug("{} Unable to resize the vector associated to the parameter named "
                             "'{}'.",
                             logPrefix,
                             parameterName);
                return false;
            }
        }

        for (std::size_t i = 0; i < tempVector.size(); i++)
        {
            if (!conversionFunction(tempVector[i], parameter[i]))
            {
                log()->debug("{} Convert the element number '{}' of the parameter named {}.",
                             logPrefix,
                             i,
                             parameterName);
                return false;
            }
        }

        return true;
    };

    // try to convert the vector of double in a vector of chrono
    std::vector<double> doubleParam;
    if (this->getParameter(parameterName, doubleParam))
    {
        // Rounded, not truncated: most decimal literals are not representable in binary, so
        // duration_cast would turn 0.3 into 299999999ns instead of 300ms.
        return convertVectorParamToVectorChrono(doubleParam,
                                                [](const double& tempParam,
                                                   std::chrono::nanoseconds& element) -> bool {
                                                    element = std::chrono::round<
                                                        std::chrono::nanoseconds>(
                                                        std::chrono::duration<double>(tempParam));
                                                    return true;
                                                });
    }

    // try to convert the vector of string in a vector of chrono
    std::vector<std::string> stringParam;
    if (this->getParameter(parameterName, stringParam))
    {
        return convertVectorParamToVectorChrono(stringParam, &this->stringToDuration);
    }

    return false;
}

bool YarpImplementation::getParameter(const std::string& parameterName,
                                      std::vector<bool>& parameter) const
{
    return getParameterPrivate(parameterName, parameter);
}

void YarpImplementation::setParameter(const std::string& parameterName, const int& parameter)
{
    return setParameterPrivate(parameterName, parameter);
}

void YarpImplementation::setParameter(const std::string& parameterName, const double& parameter)
{
    return setParameterPrivate(parameterName, parameter);
}

void YarpImplementation::setParameter(const std::string& parameterName, const char* parameter)
{
    return setParameterPrivate(parameterName, std::string(parameter));
}

void YarpImplementation::setParameter(const std::string& parameterName,
                                      const std::string& parameter)
{
    return setParameterPrivate(parameterName, parameter);
}

void YarpImplementation::setParameter(const std::string& parameterName, const bool& parameter)
{
    return setParameterPrivate(parameterName, parameter);
}

void YarpImplementation::setParameter(const std::string& parameterName,
                                      const std::chrono::nanoseconds& parameter)
{
    return setParameterPrivate(parameterName, this->durationToString(parameter));
}

void YarpImplementation::setParameter(const std::string& parameterName,
                                      const GenericContainer::Vector<const int>::Ref parameter)
{
    return setParameterPrivate(parameterName, parameter);
}
void YarpImplementation::setParameter(const std::string& parameterName,
                                      const GenericContainer::Vector<const double>::Ref parameter)
{
    return setParameterPrivate(parameterName, parameter);
}
void YarpImplementation::setParameter(const std::string& parameterName,
                                      const GenericContainer::Vector<const std::string>::Ref parameter)
{
    return setParameterPrivate(parameterName, parameter);
}

void YarpImplementation::setParameter(
    const std::string& parameterName,
    const GenericContainer::Vector<const std::chrono::nanoseconds>::Ref parameter)
{
    std::vector<std::string> temp;
    for (const auto& time : parameter)
    {
        temp.push_back(this->durationToString(time));
    }
    return this->setParameter(parameterName, temp);
}

void YarpImplementation::setParameter(const std::string& parameterName,
                                      const std::vector<bool>& parameter)
{
    return setParameterPrivate(parameterName, parameter);
}

YarpImplementation::YarpImplementation(const yarp::os::Searchable& searchable)
{
    set(searchable);
}

void YarpImplementation::set(const yarp::os::Searchable& searchable)
{
    clear();

    yarp::os::Bottle bot;
    bot.fromString(searchable.toString());

    for (size_t i = 0; i < bot.size(); i++) // all sublists are included in a new object
    {
        yarp::os::Value& bb = bot.get(i);

        yarp::os::Bottle* sub = bb.asList();
        if ((sub) && (sub->size() > 1))
        {
            std::string name = sub->get(0).toString();
            yarp::os::Bottle* subSub = sub->get(1).asList();
            if ((subSub) && (subSub->size() > 1))
            {
                m_lists.emplace(name, std::make_shared<YarpImplementation>(*sub));
            } else
            {
                m_container.add(bb);
            }

        } else
        {
            m_container.add(bb);
        }
    }
}

bool YarpImplementation::setFromFile(const std::string& filename)
{
    // load the configuration file
    yarp::os::Property prop;
    if (!prop.fromConfigFile(filename))
    {
        log()->error("[YarpImplementation::setFromFile] Unable to interpret the file named '{}' as "
                     "a configuration file.",
                     filename);

        return false;
    }

    // set the parameters handler
    this->set(prop);

    return true;
}

YarpImplementation::weak_ptr YarpImplementation::getGroup(const std::string& name) const
{
    if (m_lists.find(name) != m_lists.end())
    {
        return m_lists.at(name);
    }

    return std::make_shared<YarpImplementation>();
}

bool YarpImplementation::setGroup(const std::string& name, IParametersHandler::shared_ptr newGroup)
{
    auto downcastedPtr = std::dynamic_pointer_cast<YarpImplementation>(newGroup); // to access
                                                                                  // m_container
    if (downcastedPtr == nullptr)
    {
        log()->debug("[YarpImplementation::setGroup] Unable to downcast the pointer to "
                     "YarpImplementation.");

        return false;
    }

    yarp::os::Bottle backup = downcastedPtr->m_container;
    yarp::os::Bottle nameAdded;
    nameAdded.add(yarp::os::Value(name));
    nameAdded.append(backup);
    downcastedPtr->m_container = nameAdded; // This is all to add the name at the beginning of the
                                            // bottle
    m_lists[name] = downcastedPtr;

    return true;
}

std::string YarpImplementation::toString() const
{
    std::string output = m_container.toString();
    for (auto& group : m_lists)
    {
        output += " (" + group.second->toString() + ")";
    }
    return output;
}

bool YarpImplementation::isEmpty() const
{
    // We check if the container is null and there are no lists.
    // A special case is when the container has a single string element.
    // This is the case for newly created groups, where the container has only the name of the group
    // itself.
    return ((m_container.size() == 0 || (m_container.size() == 1 && m_container.get(0).isString()))
            && (m_lists.size() == 0));
}

void YarpImplementation::clear()
{
    m_container.clear();
    m_lists.clear();
}

std::shared_ptr<YarpImplementation> YarpImplementation::clonePrivate() const
{
    auto handler = std::make_shared<YarpImplementation>();

    // copy the content of the parameters stored in the handler.
    handler->m_container = this->m_container;

    for (const auto& [key, value] : m_lists)
        handler->m_lists[key] = value->clonePrivate();

    return handler;
}

IParametersHandler::shared_ptr YarpImplementation::clone() const
{
    return this->clonePrivate();
}
