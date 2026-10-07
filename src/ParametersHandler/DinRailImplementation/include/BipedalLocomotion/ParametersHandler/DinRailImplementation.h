/**
 * @file DinRailImplementation.h
 * @copyright 2026 Generative Bionics S.R.L. This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_PARAMETERS_HANDLER_DINRAIL_IMPLEMENTATION_H
#define BIPEDAL_LOCOMOTION_PARAMETERS_HANDLER_DINRAIL_IMPLEMENTATION_H

#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <dinrail/Parameters.h>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>

namespace BipedalLocomotion
{
namespace ParametersHandler
{

/**
 * Adapter exposing a dinrail::Parameters container through the IParametersHandler interface.
 */
class DinRailImplementation final : public IParametersHandler
{
public:
    /** Construct an empty handler. */
    DinRailImplementation() = default;

    /**
     * Construct a handler by copying a dinrail parameter container.
     * @param parameters parameter container to copy
     */
    explicit DinRailImplementation(const dinrail::Parameters& parameters);

    bool getParameter(const std::string& parameterName, int& parameter) const final;
    bool getParameter(const std::string& parameterName, double& parameter) const final;
    bool getParameter(const std::string& parameterName, std::string& parameter) const final;
    bool getParameter(const std::string& parameterName, bool& parameter) const final;
    bool
    getParameter(const std::string& parameterName, std::chrono::nanoseconds& parameter) const final;
    bool getParameter(const std::string& parameterName, std::vector<bool>& parameter) const final;
    bool getParameter(const std::string& parameterName,
                      GenericContainer::Vector<int>::Ref parameter) const final;
    bool getParameter(const std::string& parameterName,
                      GenericContainer::Vector<double>::Ref parameter) const final;
    bool getParameter(const std::string& parameterName,
                      GenericContainer::Vector<std::string>::Ref parameter) const final;
    bool
    getParameter(const std::string& parameterName,
                 GenericContainer::Vector<std::chrono::nanoseconds>::Ref parameter) const final;

    void setParameter(const std::string& parameterName, const int& parameter) final;
    void setParameter(const std::string& parameterName, const double& parameter) final;
    void setParameter(const std::string& parameterName, const std::string& parameter) final;
    void setParameter(const std::string& parameterName, const char* parameter) final;
    void setParameter(const std::string& parameterName, const bool& parameter) final;
    void
    setParameter(const std::string& parameterName, const std::chrono::nanoseconds& parameter) final;
    void setParameter(const std::string& parameterName, const std::vector<bool>& parameter) final;
    void setParameter(const std::string& parameterName,
                      const GenericContainer::Vector<const int>::Ref parameter) final;
    void setParameter(const std::string& parameterName,
                      const GenericContainer::Vector<const double>::Ref parameter) final;
    void setParameter(const std::string& parameterName,
                      const GenericContainer::Vector<const std::string>::Ref parameter) final;
    void setParameter(
        const std::string& parameterName,
        const GenericContainer::Vector<const std::chrono::nanoseconds>::Ref parameter) final;

    weak_ptr getGroup(const std::string& name) const final;
    bool setGroup(const std::string& name, shared_ptr newGroup) final;

    std::string toString() const final;
    bool isEmpty() const final;
    void clear() final;
    shared_ptr clone() const final;

    ~DinRailImplementation() final = default;

private:
    dinrail::Parameters m_parameters;
    mutable std::unordered_map<std::string, std::shared_ptr<DinRailImplementation>> m_groupsCache;
};

} // namespace ParametersHandler
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_PARAMETERS_HANDLER_DINRAIL_IMPLEMENTATION_H
