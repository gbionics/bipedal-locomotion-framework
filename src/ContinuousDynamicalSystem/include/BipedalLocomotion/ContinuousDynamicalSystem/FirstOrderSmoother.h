/**
 * @file FirstOrderSmoother.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTINUOUS_DYNAMICAL_SYSTEM_FIRST_ORDER_SMOOTHER_H
#define BIPEDAL_LOCOMOTION_CONTINUOUS_DYNAMICAL_SYSTEM_FIRST_ORDER_SMOOTHER_H

#include <memory>

#include <BipedalLocomotion/ContinuousDynamicalSystem/LinearTimeInvariantSystem.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/ForwardEuler.h>
#include <BipedalLocomotion/System/Advanceable.h>

#include <Eigen/Dense>

namespace BipedalLocomotion
{
namespace ContinuousDynamicalSystem
{

/**
 * FirstOrderSmoother smoother implements a simple smoother based on a first order linear
 * system. The system is described by the following ode
 * \f[
 * \dot{x} = a (-x + u)
 * \f]
 * where the \f$a\f$ is given by \f$a = 3.0/T_{s_5}\f$. \f$T_{s_5}\f$ is the settling time at 5%.
 * The linear system is propagated with a Forward Euler integrator.
 * @note The settling time can be time-varying. Indeed, it can be updated at run time by calling
 * FirstOrderSmoother::setSettlingTime(). This is useful, for instance, when the desired smoothness
 * of the output needs to change while the smoother is running.
 */
class FirstOrderSmoother
    : public BipedalLocomotion::System::Advanceable<Eigen::VectorXd, Eigen::VectorXd>
{
public:
    /**
     * Initialize the Dynamical system.
     * @param handler pointer to the parameter handler.
     * @note the following parameters are required
     * |   Parameter Name  |   Type   |                   Description                    | Mandatory |
     * |:-----------------:|:--------:|:------------------------------------------------:|:---------:|
     * |  `sampling_time`  | `double` | Sampling time used to discrete the linear system |     Yes    |
     * |  `settling_time`  | `double` | 5% settling time (The settling time, \f$T_s\f$, is the time required for the system output to fall within a certain percentage (i.e. 5%) of the steady-state value for a step input.) |    Yes    |
     * @note the `settling_time` set here is only the initial value. It can be changed at run time,
     * i.e. it can be time-varying, by calling FirstOrderSmoother::setSettlingTime().
     * @return true in case of success/false otherwise.
     */
    bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler) override;

    /**
     * Set the state of the smoother.
     * @param initialState initial state of the smoother
     * @return true in case of success, false otherwise.
     */
    bool reset(Eigen::Ref<const Eigen::VectorXd> initialState);

    /**
     * Set (or update) the settling time of the smoother. This method can be called at any time
     * after FirstOrderSmoother::reset() to make the settling time time-varying.
     * @param settlingTime the new 5% settling time. It must be strictly positive.
     * @return true in case of success, false otherwise.
     */
    bool setSettlingTime(const double settlingTime);

    /**
     * Get the settling time currently used by the smoother.
     * @return the 5% settling time.
     */
    double getSettlingTime() const;

    /**
     * @brief Perform one integration step using the input set by the FirstOrderSmoother::setInput
     * method.
     * @return True in case of success and false otherwise
     */
    bool advance() override;

    /**
     * Get the output of the smoother.
     * @return a vector containing the outpur of the smoother
     */
    const Eigen::VectorXd& getOutput() const override;

    /**
     * @brief Set the input of the smoother
     * @param input the vector representing the input of the smoother
     * @return True in case of success and false otherwise
     */
    bool setInput(const Eigen::VectorXd& input) override;

    /**
     * Determines the validity of the object retrieved with getOutput()
     * @return True if the object is valid, false otherwise.
     */
    bool isOutputValid() const override;

private:
    bool m_isInitialized{false};
    bool m_isInitialStateSet{false};
    bool m_isOutputValid{false};
    double m_settlingTime{-1};
    Eigen::VectorXd m_output;

    std::shared_ptr<LinearTimeInvariantSystem> m_linearSystem;
    ForwardEuler<LinearTimeInvariantSystem> m_integrator;
};

} // namespace ContinuousDynamicalSystem
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_CONTINUOUS_DYNAMICAL_SYSTEM_FIRST_ORDER_SMOOTHER_H
