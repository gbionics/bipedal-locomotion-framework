/**
 * @file EMAWithLimits.h
 * @authors Giulio Romualdi
 * @copyright 2024 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_JOINT_LEVEL_CONTROLLERS_EMA_WITH_LIMITS_H
#define BIPEDAL_LOCOMOTION_JOINT_LEVEL_CONTROLLERS_EMA_WITH_LIMITS_H

#include <Eigen/Dense>

#include <memory>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/System/Advanceable.h>
#include <BipedalLocomotion/System/VariablesHandler.h>

namespace BipedalLocomotion
{

namespace JointLevelControllers
{

/**
 * @brief Exponential Moving Average (EMA) with Joint Limits
 *
 * This class implements an Exponential Moving Average (EMA) controller that applies limits
 * to the joint values. The EMA is computed based on the input reference and feedback signals,
 * and the output is adjusted to respect the specified joint limits. The controller
 * applies a scaling factor to the input signals and has options for softening the limits.
 */
class EMAWithLimits
    : public BipedalLocomotion::System::Advanceable<Eigen::VectorXd, Eigen::VectorXd>
{
public:
    /**
     * Constructor
     */
    EMAWithLimits();

    /**
     * Destructor. It is required by the pimpl implementation.
     */
    ~EMAWithLimits();

    // clang-format off
    /**
     * Initialize the EMAWithLimits.
     * @param paramHandler pointer to the parameters handler.
     * @note the following parameters are required by the class
     * |        Parameter Name       |       Type       |                                       Description                                               | Mandatory |
     * |:---------------------------:|:----------------:|:-----------------------------------------------------------------------------------------------:|:---------:|
     * |          `scale`            |      `vector`    |            Array specifying the scaling factors for each joint                                  |    Yes    |
     * |          `alpha`            |      `double`    |                    Weighting factor or coefficient for adjustments                              |    Yes    |
     * |      `lower_limit`          |      `vector`    |            Array specifying the lower limits for the joints                                     |    Yes    |
     * |      `upper_limit`          |      `vector`    |            Array specifying the upper limits for the joints                                     |    Yes    |
     * |   `soft_limit_factor`       |      `vector`    |       Factor to soften or scale the limits (default is 1.0)                                     |     No    |
     * @return True in case of success, false otherwise.
     */
    bool initialize(std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> paramHandler) override;
    // clang-format on

    /**
     * Set the input of the EMAWithLimits
     * @param input the struct containing all the inputs of the EMAWithLimits.
     * @return true if case of success, false otherwise.
     */
    bool setInput(const EMAWithLimits::Input& input) override;

    /**
     * Perform one step of the inference given the input set.
     * @return true if case of success, false otherwise.
     */
    bool advance() override;

    /**
     * Get the output of the EMAWithLimits once advance has been called.
     * @return the output of the EMAWithLimits.
     */
    const EMAWithLimits::Output& getOutput() const override;

    /**
     * Check if the output of the EMAWithLimits is valid.
     * @return true if it is valid, false otherwise.
     */
    bool isOutputValid() const override;

    /**
     * Reset the EMAWithLimits with a new input.
     * @param input the new input to reset the EMAWithLimits.
     * @return true if the reset was successful, false otherwise.
     */
    bool reset(Eigen::Ref<const Eigen::VectorXd> input);

private:
    /** Private implementation */
    struct Impl;
    std::unique_ptr<Impl> m_pimpl;
};

} // namespace JointLevelControllers

} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_JOINT_LEVEL_CONTROLLERS_EMA_WITH_LIMITS_H
