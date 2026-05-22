/**
 * @file PositionToCurrentController.cpp
 * @authors Giulio Romualdi
 * @copyright 2025 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/JointLevelControllers/EMAWithLimits.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <stdexcept>

using namespace BipedalLocomotion::JointLevelControllers;

struct EMAWithLimits::Impl
{
    double alpha{0.9}; /**< Exponential weighting factor for the EMA */
    Eigen::VectorXd scale; /**< Scaling factor for the input actions */    
    Eigen::VectorXd lowerLimit;
    Eigen::VectorXd upperLimit;
    Eigen::VectorXd softLowerLimit;
    Eigen::VectorXd softUpperLimit;
    Eigen::VectorXd previousAppliedActions;
    Eigen::VectorXd processedActions;

    Eigen::VectorXd input;

    enum class State
    {
        NotReset,
        WaitingForAdvance,
        Running,
    };
    State state{State::NotReset}; /**< Current state of the object */

    void computeSoftLimits(double softLimitFactor)
    {
        const Eigen::VectorXd offset = (this->lowerLimit + this->upperLimit) * 0.5;
        const Eigen::VectorXd range = (this->upperLimit - this->lowerLimit);
        this->softLowerLimit = offset - 0.5 * softLimitFactor * range;
        this->softUpperLimit = offset + 0.5 * softLimitFactor * range;
    }

    Eigen::VectorXd unscaleTransform(Eigen::Ref<const Eigen::VectorXd> x,
                                     Eigen::Ref<const Eigen::VectorXd> lower,
                                     Eigen::Ref<const Eigen::VectorXd> upper) const
    {
        const Eigen::VectorXd offset = (lower + upper) * 0.5;
        return x.cwiseProduct((upper - lower) * 0.5).array() + offset.array();
    }

    Eigen::VectorXd clip(Eigen::Ref<const Eigen::VectorXd> x,
                         Eigen::Ref<const Eigen::VectorXd> lower,
                         Eigen::Ref<const Eigen::VectorXd> upper) const
    {
        return x.cwiseMax(lower).cwiseMin(upper);
    }
};

EMAWithLimits::EMAWithLimits()
    : m_pimpl(std::make_unique<Impl>())
{
}

EMAWithLimits::~EMAWithLimits() = default;

bool EMAWithLimits::initialize(
    std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler)
{
    constexpr auto logPrefix = "[EMAWithLimits::initialize]";
    auto ptr = handler.lock();

    if (ptr == nullptr)
    {
        log()->error("{} Invalid parameter handler.", logPrefix);
        return false;
    }

    auto loadParam
        = [ptr, logPrefix](const std::string& paramName, auto& param, bool isOptional) -> bool {
        if (!ptr->getParameter(paramName, param))
        {
            if (isOptional)
            {
                log()->warn("{} Parameter '{}' not found, using default value {}.",
                            logPrefix,
                            paramName,
                            param);
            } else
            {
                log()->error("{} Required parameter '{}' not found.", logPrefix, paramName);
            }
            return isOptional; // Return true if optional, false if required and not found
        }
        return true;
    };

    // Load parameters
    constexpr bool optional = true;
    constexpr auto required = false;
    double softLimitFactor{1.0};
    if (!loadParam("scale", m_pimpl->scale, required)
        || !loadParam("alpha", m_pimpl->alpha, required)
        || !loadParam("lower_limit", m_pimpl->lowerLimit, required)
        || !loadParam("upper_limit", m_pimpl->upperLimit, required)
        || !loadParam("soft_limit_factor", softLimitFactor, optional))
    {
        return false;
    }

    if (m_pimpl->lowerLimit.size() != m_pimpl->upperLimit.size())
    {
        log()->error("{} Size mismatch between lower and upper limits.", logPrefix);
        return false;
    }

    if (m_pimpl->scale.size() != m_pimpl->lowerLimit.size())
    {
        log()->error("{} Size mismatch between scale and limits.", logPrefix);
        return false;
    }

    if ((m_pimpl->scale.array() <= 0.0).any())
    {
        log()->error("{} The scale must be greater than zero for all joints.", logPrefix);
        return false;
    }

    if (m_pimpl->alpha < 0.0 || m_pimpl->alpha > 1.0)
    {
        log()->error("{} The alpha must be in the range [0, 1].", logPrefix);
        return false;
    }

    if (softLimitFactor <= 0.0)
    {
        log()->error("{} The soft limit factor must be greater than zero.", logPrefix);
        return false;
    }

    m_pimpl->computeSoftLimits(softLimitFactor);
    m_pimpl->state = Impl::State::NotReset;

    // reset the system state
    this->reset(Eigen::VectorXd::Zero(m_pimpl->lowerLimit.size()));

    log()->info("{} EMAWithLimits successfully initialized.", logPrefix);
    return true;
}

bool EMAWithLimits::advance()
{
    if (m_pimpl->state == Impl::State::NotReset)
    {
        log()->error("[EMAWithLimits::advance] The system must be reset before "
                     "use.");
        return false;
    }

    // 1. the policy is scaled
    m_pimpl->processedActions = m_pimpl->scale.cwiseProduct(m_pimpl->input);

    // 2. clip the policy between [-1, 1]
    m_pimpl->processedActions = m_pimpl->clip(m_pimpl->processedActions,
                                              Eigen::VectorXd::Constant(m_pimpl->input.size(), -1),
                                              Eigen::VectorXd::Constant(m_pimpl->input.size(), 1));

    // 3. unscale the policy to the joint limits
    m_pimpl->processedActions = m_pimpl->unscaleTransform(m_pimpl->processedActions,
                                                          m_pimpl->softLowerLimit,
                                                          m_pimpl->softUpperLimit);

    // 4. apply the EMA
    m_pimpl->processedActions = m_pimpl->alpha * m_pimpl->processedActions
                                + (1.0 - m_pimpl->alpha) * m_pimpl->previousAppliedActions;

    // 5. clip the policy to the joint limits
    m_pimpl->processedActions = m_pimpl->clip(m_pimpl->processedActions,
                                              m_pimpl->softLowerLimit,
                                              m_pimpl->softUpperLimit);

    // store the processed actions
    m_pimpl->previousAppliedActions = m_pimpl->processedActions;

    m_pimpl->state = Impl::State::Running;

    return true;
}

const EMAWithLimits::Output& EMAWithLimits::getOutput() const
{
    return m_pimpl->processedActions;
}

bool EMAWithLimits::reset(Eigen::Ref<const Eigen::VectorXd> initialCondition)
{
    if (initialCondition.size() != m_pimpl->softLowerLimit.size())
    {
        log()->error("[EMAWithLimits::reset] Initial condition size mismatch. Provided "
                     "initial condition has size {} while the expected size is {}.",
                     initialCondition.size(),
                     m_pimpl->softLowerLimit.size());
        return false;
    }

    // check if the initial condition is within the limits. If not warn and clip it
    if ((initialCondition.array() < m_pimpl->lowerLimit.array()).any()
        || (initialCondition.array() > m_pimpl->upperLimit.array()).any())
    {
        log()->warn("[EMAWithLimits::reset] The provided initial condition is not within the "
                    "joint limits. Clipping it to be within the joint limits. Provided initial {}, "
                    "Min {}, Max {}",
                    initialCondition.transpose(),
                    m_pimpl->lowerLimit.transpose(),
                    m_pimpl->upperLimit.transpose());
    }

    m_pimpl->previousAppliedActions
        = m_pimpl->clip(initialCondition, m_pimpl->lowerLimit, m_pimpl->upperLimit);

    m_pimpl->input = Eigen::VectorXd::Zero(m_pimpl->softLowerLimit.size());
    m_pimpl->processedActions = m_pimpl->previousAppliedActions;
    m_pimpl->state = Impl::State::WaitingForAdvance;
    return true;
}

bool EMAWithLimits::setInput(const EMAWithLimits::Input& input)
{
    if (input.size() != m_pimpl->softLowerLimit.size())
    {
        log()->error("[EMAWithLimits::setInput] Input size mismatch. Provided "
                     "input has size {} while the expected size is {}.",
                     input.size(),
                     m_pimpl->softLowerLimit.size());
        return false;
    }

    m_pimpl->input = input;
    return true;
}

bool EMAWithLimits::isOutputValid() const
{
    return m_pimpl->state == Impl::State::Running;
}
