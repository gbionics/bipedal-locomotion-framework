/**
 * @file EMAWithLimitsTest.cpp
 * @authors Giulio Romualdi
 * @copyright 2025 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

// Catch2
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <BipedalLocomotion/JointLevelControllers/EMAWithLimits.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>

using namespace BipedalLocomotion::JointLevelControllers;
using namespace BipedalLocomotion::ParametersHandler;
using Catch::Approx;

// Helper function to create parameter handler with basic configuration
std::shared_ptr<StdImplementation>
createBasicParameterHandler(const Eigen::Ref<const Eigen::VectorXd>& scale,
                            double alpha = 0.8,
                            const Eigen::VectorXd& lowerLimit = Eigen::Vector3d(-1.0, -2.0, -3.0),
                            const Eigen::VectorXd& upperLimit = Eigen::Vector3d(1.0, 2.0, 3.0),
                            double softLimitFactor = 0.9)
{
    auto handler = std::make_shared<StdImplementation>();
    handler->setParameter("scale", scale);
    handler->setParameter("alpha", alpha);
    handler->setParameter("lower_limit", lowerLimit);
    handler->setParameter("upper_limit", upperLimit);
    handler->setParameter("soft_limit_factor", softLimitFactor);
    return handler;
}

std::shared_ptr<StdImplementation>
createBasicParameterHandler(double scale = 0.5,
                            double alpha = 0.8,
                            const Eigen::VectorXd& lowerLimit = Eigen::Vector3d(-1.0, -2.0, -3.0),
                            const Eigen::VectorXd& upperLimit = Eigen::Vector3d(1.0, 2.0, 3.0),
                            double softLimitFactor = 0.9)
{
    return createBasicParameterHandler(
        Eigen::VectorXd::Constant(lowerLimit.size(), scale), alpha, lowerLimit, upperLimit, softLimitFactor);
}

TEST_CASE("EMAWithLimits - Constructor and Destructor")
{
    // Test constructor - should start in invalid state
    EMAWithLimits controller;
    REQUIRE_FALSE(controller.isOutputValid());

    // Test destructor through scope
    {
        EMAWithLimits localController;
        // Should be destroyed without issues
    }
}

TEST_CASE("EMAWithLimits - Initialization Parameter Validation")
{
    EMAWithLimits controller;

    SECTION("Valid initialization with all parameters")
    {
        auto handler = createBasicParameterHandler();
        REQUIRE(controller.initialize(handler));
        REQUIRE_FALSE(controller.isOutputValid()); // Should be WaitingForAdvance after init+reset
    }

    SECTION("Invalid parameter handler (nullptr)")
    {
        std::weak_ptr<const IParametersHandler> nullHandler;
        REQUIRE_FALSE(controller.initialize(nullHandler));
    }

    SECTION("Missing all required parameters")
    {
        auto handler = std::make_shared<StdImplementation>();
        REQUIRE_FALSE(controller.initialize(handler));
    }

    SECTION("Missing scale parameter")
    {
        auto handler = std::make_shared<StdImplementation>();
        handler->setParameter("alpha", 0.8);
        handler->setParameter("lower_limit", Eigen::Vector2d(-1.0, -2.0));
        handler->setParameter("upper_limit", Eigen::Vector2d(1.0, 2.0));
        REQUIRE_FALSE(controller.initialize(handler));
    }

    SECTION("Missing alpha parameter")
    {
        auto handler = std::make_shared<StdImplementation>();
        handler->setParameter("scale", Eigen::Vector2d(0.5, 0.5));
        handler->setParameter("lower_limit", Eigen::Vector2d(-1.0, -2.0));
        handler->setParameter("upper_limit", Eigen::Vector2d(1.0, 2.0));
        REQUIRE_FALSE(controller.initialize(handler));
    }

    SECTION("Missing lower_limit parameter")
    {
        auto handler = std::make_shared<StdImplementation>();
        handler->setParameter("scale", Eigen::Vector2d(0.5, 0.5));
        handler->setParameter("alpha", 0.8);
        handler->setParameter("upper_limit", Eigen::Vector2d(1.0, 2.0));
        REQUIRE_FALSE(controller.initialize(handler));
    }

    SECTION("Missing upper_limit parameter")
    {
        auto handler = std::make_shared<StdImplementation>();
        handler->setParameter("scale", Eigen::Vector2d(0.5, 0.5));
        handler->setParameter("alpha", 0.8);
        handler->setParameter("lower_limit", Eigen::Vector2d(-1.0, -2.0));
        REQUIRE_FALSE(controller.initialize(handler));
    }

    SECTION("Size mismatch between lower and upper limits")
    {
        auto handler = std::make_shared<StdImplementation>();
        handler->setParameter("scale", Eigen::Vector2d(0.5, 0.5));
        handler->setParameter("alpha", 0.8);
        handler->setParameter("lower_limit", Eigen::Vector2d(-1.0, -2.0)); // Size 2
        handler->setParameter("upper_limit", Eigen::Vector3d(1.0, 2.0, 3.0)); // Size 3
        REQUIRE_FALSE(controller.initialize(handler));
    }

    SECTION("Invalid scale (zero)")
    {
        auto handler = createBasicParameterHandler(0.0); // scale = 0
        REQUIRE_FALSE(controller.initialize(handler));
    }

    SECTION("Invalid scale (negative)")
    {
        auto handler = createBasicParameterHandler(-0.5); // scale < 0
        REQUIRE_FALSE(controller.initialize(handler));
    }

    SECTION("Valid alpha boundary values")
    {
        // Test alpha = 0 (valid boundary)
        auto handler1 = createBasicParameterHandler(0.5, 0.0);
        REQUIRE(controller.initialize(handler1));

        // Test alpha = 1 (valid boundary)
        auto handler2 = createBasicParameterHandler(0.5, 1.0);
        REQUIRE(controller.initialize(handler2));
    }

    SECTION("Invalid alpha values")
    {
        // Test alpha > 1
        auto handler1 = createBasicParameterHandler(0.5, 1.5);
        REQUIRE_FALSE(controller.initialize(handler1));

        // Test alpha < 0
        auto handler2 = createBasicParameterHandler(0.5, -0.1);
        REQUIRE_FALSE(controller.initialize(handler2));
    }

    SECTION("Valid initialization without soft limit factor (uses default)")
    {
        auto handler = std::make_shared<StdImplementation>();
        handler->setParameter("scale", Eigen::Vector2d(0.5, 0.5));
        handler->setParameter("alpha", 0.8);
        handler->setParameter("lower_limit", Eigen::Vector2d(-1.0, -2.0));
        handler->setParameter("upper_limit", Eigen::Vector2d(1.0, 2.0));
        // soft_limit_factor will use default value
        REQUIRE(controller.initialize(handler));
    }

    SECTION("Invalid soft limit factor values")
    {
        // Test soft_limit_factor = 0
        auto handler1 = createBasicParameterHandler(0.5,
                                                    0.8,
                                                    Eigen::Vector2d(-1.0, -2.0),
                                                    Eigen::Vector2d(1.0, 2.0),
                                                    0.0);
        REQUIRE_FALSE(controller.initialize(handler1));

        // Test soft_limit_factor < 0
        auto handler2 = createBasicParameterHandler(0.5,
                                                    0.8,
                                                    Eigen::Vector2d(-1.0, -2.0),
                                                    Eigen::Vector2d(1.0, 2.0),
                                                    -0.5);
        REQUIRE_FALSE(controller.initialize(handler2));
    }

    SECTION("Edge cases: Single joint and many joints")
    {
        // Single joint
        Eigen::VectorXd singleLower(1), singleUpper(1);
        singleLower << -1.0;
        singleUpper << 1.0;
        auto handler1 = createBasicParameterHandler(1.0, 0.5, singleLower, singleUpper, 1.0);
        REQUIRE(controller.initialize(handler1));

        // Many joints
        const int numJoints = 50;
        Eigen::VectorXd manyLower = Eigen::VectorXd::Constant(numJoints, -2.0);
        Eigen::VectorXd manyUpper = Eigen::VectorXd::Constant(numJoints, 2.0);
        auto handler2 = createBasicParameterHandler(1.0, 0.5, manyLower, manyUpper, 1.0);
        REQUIRE(controller.initialize(handler2));
    }
}

TEST_CASE("EMAWithLimits - State Management and Transitions")
{
    EMAWithLimits controller;
    auto handler = createBasicParameterHandler();
    REQUIRE(controller.initialize(handler));

    SECTION("State progression: NotReset -> WaitingForAdvance -> Running")
    {
        // After initialization, automatic reset puts us in WaitingForAdvance
        REQUIRE_FALSE(controller.isOutputValid()); // WaitingForAdvance state

        // Explicit reset should maintain WaitingForAdvance
        REQUIRE(controller.reset(Eigen::Vector3d::Zero()));
        REQUIRE_FALSE(controller.isOutputValid()); // Still WaitingForAdvance

        // Set input and advance to reach Running state
        Eigen::Vector3d input(0.1, 0.2, 0.3);
        REQUIRE(controller.setInput(input));
        REQUIRE(controller.advance());
        REQUIRE(controller.isOutputValid()); // Now in Running state
    }

    SECTION("Reset clears state and sets to WaitingForAdvance")
    {
        // Get to Running state first
        REQUIRE(controller.reset(Eigen::Vector3d::Zero()));
        Eigen::Vector3d input(0.1, 0.2, 0.3);
        REQUIRE(controller.setInput(input));
        REQUIRE(controller.advance());
        REQUIRE(controller.isOutputValid()); // Running state

        // Reset should clear state
        REQUIRE(controller.reset(Eigen::Vector3d::Zero()));
        REQUIRE_FALSE(controller.isOutputValid()); // WaitingForAdvance state
    }

    SECTION("Advance fails when not properly initialized/reset")
    {
        EMAWithLimits freshController;
        // Don't initialize - should be in NotReset state

        Eigen::Vector3d input(0.1, 0.2, 0.3);
        REQUIRE_FALSE(freshController.setInput(input)); // Should fail without initialization
    }

    SECTION("Multiple state transitions")
    {
        for (int i = 0; i < 5; ++i)
        {
            REQUIRE(controller.reset(Eigen::Vector3d::Zero()));
            REQUIRE_FALSE(controller.isOutputValid()); // WaitingForAdvance

            Eigen::Vector3d input(0.1 * i, 0.2 * i, 0.3 * i);
            REQUIRE(controller.setInput(input));
            REQUIRE(controller.advance());
            REQUIRE(controller.isOutputValid()); // Running
        }
    }

    SECTION("State persists through multiple advances")
    {
        REQUIRE(controller.reset(Eigen::Vector3d::Zero()));
        Eigen::Vector3d input(0.1, 0.2, 0.3);
        REQUIRE(controller.setInput(input));
        REQUIRE(controller.advance());
        REQUIRE(controller.isOutputValid()); // Running

        // Continue advancing - should stay in Running state
        for (int i = 0; i < 5; ++i)
        {
            input *= 1.1;
            REQUIRE(controller.setInput(input));
            REQUIRE(controller.advance());
            REQUIRE(controller.isOutputValid()); // Still Running
        }
    }
}

TEST_CASE("EMAWithLimits - Input Validation")
{
    EMAWithLimits controller;
    auto handler = createBasicParameterHandler(); // 3-joint configuration
    REQUIRE(controller.initialize(handler));
    REQUIRE(controller.reset(Eigen::Vector3d::Zero()));

    SECTION("Valid input sizes")
    {
        Eigen::Vector3d input(0.1, 0.2, 0.3);
        REQUIRE(controller.setInput(input));

        Eigen::Vector3d zeroInput = Eigen::Vector3d::Zero();
        REQUIRE(controller.setInput(zeroInput));
    }

    SECTION("Invalid input sizes")
    {
        // Too small
        Eigen::Vector2d smallInput(0.1, 0.2);
        REQUIRE_FALSE(controller.setInput(smallInput));

        // Too large
        Eigen::Vector4d largeInput(0.1, 0.2, 0.3, 0.4);
        REQUIRE_FALSE(controller.setInput(largeInput));

        // Empty
        Eigen::VectorXd emptyInput(0);
        REQUIRE_FALSE(controller.setInput(emptyInput));
    }

    SECTION("Extreme input values")
    {
        // Very large values
        Eigen::Vector3d largeInput(1000.0, -1000.0, 500.0);
        REQUIRE(controller.setInput(largeInput));

        // Very small values
        Eigen::Vector3d smallInput(1e-10, -1e-10, 1e-15);
        REQUIRE(controller.setInput(smallInput));

        // Special values (should be accepted by setInput, handled in advance)
        Eigen::Vector3d specialInput(std::numeric_limits<double>::infinity(),
                                     std::numeric_limits<double>::quiet_NaN(),
                                     std::numeric_limits<double>::max());
        REQUIRE(controller.setInput(specialInput));
    }

    SECTION("Input validation without initialization")
    {
        EMAWithLimits uninitializedController;
        Eigen::Vector3d input(0.1, 0.2, 0.3);
        REQUIRE_FALSE(uninitializedController.setInput(input));
    }
}

TEST_CASE("EMAWithLimits - Algorithm Correctness")
{
    SECTION("Basic algorithm flow: Scale -> Clip -> Unscale -> EMA -> Clip")
    {
        // Use simple parameters for predictable results
        const double scale = 2.0;
        const double alpha = 1.0; // No EMA smoothing
        Eigen::VectorXd lowerLimit(1), upperLimit(1);
        lowerLimit << -2.0;
        upperLimit << 2.0;

        auto handler = createBasicParameterHandler(scale, alpha, lowerLimit, upperLimit, 1.0);
        EMAWithLimits controller;
        REQUIRE(controller.initialize(handler));
        REQUIRE(controller.reset(Eigen::VectorXd::Zero(1)));

        // Test with input 0.25
        Eigen::VectorXd input(1);
        input << 0.25;
        REQUIRE(controller.setInput(input));
        REQUIRE(controller.advance());

        const auto& output = controller.getOutput();
        // Expected: 0.25 * 2.0 = 0.5 (scale)
        // Clipped to [-1, 1]: 0.5 (no clipping needed)
        // Unscaled: 0.5 * (2.0 - (-2.0))/2 + (2.0 + (-2.0))/2 = 0.5 * 2 + 0 = 1.0
        // EMA with alpha=1.0 and prev=0: 1.0 * 1.0 + 0.0 * 0 = 1.0
        // Final clip: within [-2, 2], so 1.0
        REQUIRE(output(0) == Approx(1.0).epsilon(1e-6));
    }

    SECTION("Scaling with clipping test")
    {
        const double scale = 3.0;
        const double alpha = 1.0;
        Eigen::VectorXd lowerLimit(1), upperLimit(1);
        lowerLimit << -1.0;
        upperLimit << 1.0;

        auto handler = createBasicParameterHandler(scale, alpha, lowerLimit, upperLimit, 1.0);
        EMAWithLimits controller;
        REQUIRE(controller.initialize(handler));
        REQUIRE(controller.reset(Eigen::VectorXd::Zero(1)));

        // Input that will be clipped after scaling
        Eigen::VectorXd input(1);
        input << 0.5; // 0.5 * 3.0 = 1.5, clipped to 1.0
        REQUIRE(controller.setInput(input));
        REQUIRE(controller.advance());

        const auto& output = controller.getOutput();
        // Should be at upper limit
        REQUIRE(output(0) == Approx(1.0).epsilon(1e-6));
    }

    SECTION("Different scale per joint")
    {
        const double alpha = 1.0;
        Eigen::Vector3d lowerLimit(-2.0, -2.0, -2.0);
        Eigen::Vector3d upperLimit(2.0, 2.0, 2.0);
        Eigen::Vector3d scale(0.5, 1.0, 3.0);

        auto handler = createBasicParameterHandler(scale, alpha, lowerLimit, upperLimit, 1.0);
        EMAWithLimits controller;
        REQUIRE(controller.initialize(handler));
        REQUIRE(controller.reset(Eigen::Vector3d::Zero()));

        Eigen::Vector3d input(0.5, 0.5, 0.5);
        REQUIRE(controller.setInput(input));
        REQUIRE(controller.advance());

        const auto& output = controller.getOutput();

        // scaled input: [0.25, 0.5, 1.5] -> clipped: [0.25, 0.5, 1.0]
        // unscaled to [-2, 2]: [0.5, 1.0, 2.0]
        REQUIRE(output(0) == Approx(0.5).epsilon(1e-6));
        REQUIRE(output(1) == Approx(1.0).epsilon(1e-6));
        REQUIRE(output(2) == Approx(2.0).epsilon(1e-6));
    }

    SECTION("EMA behavior with different alpha values")
    {
        // Test alpha = 0 (no current input influence)
        {
            auto handler = createBasicParameterHandler(1.0, 0.0);
            EMAWithLimits controller;
            REQUIRE(controller.initialize(handler));
            REQUIRE(controller.reset(Eigen::Vector3d::Zero()));

            Eigen::Vector3d input(1.0, 1.0, 1.0);
            REQUIRE(controller.setInput(input));
            REQUIRE(controller.advance());
            auto output = controller.getOutput();

            // With alpha=0, output should be all zeros (previous values dominate)
            REQUIRE(output(0) == Approx(0.0).margin(1e-10));
            REQUIRE(output(1) == Approx(0.0).margin(1e-10));
            REQUIRE(output(2) == Approx(0.0).margin(1e-10));
        }

        // Test alpha = 1 (no previous input influence)
        {
            auto handler = createBasicParameterHandler(1.0, 1.0);
            EMAWithLimits controller;
            REQUIRE(controller.initialize(handler));
            REQUIRE(controller.reset(Eigen::Vector3d::Zero()));

            Eigen::Vector3d input(0.5, -0.5, 0.0);
            REQUIRE(controller.setInput(input));
            REQUIRE(controller.advance());
            auto output = controller.getOutput();

            // With alpha=1.0, previous values don't influence result
            // Output should not be zero for non-zero inputs
            REQUIRE(std::abs(output(0)) > 1e-6);
            REQUIRE(std::abs(output(1)) > 1e-6);
        }
    }

    SECTION("EMA convergence over multiple steps")
    {
        auto handler = createBasicParameterHandler(1.0, 0.8);
        EMAWithLimits controller;
        REQUIRE(controller.initialize(handler));
        REQUIRE(controller.reset(Eigen::Vector3d::Zero()));

        const Eigen::Vector3d constantInput(0.6, -0.4, 0.2);
        std::vector<Eigen::VectorXd> outputs;

        // Apply same input multiple times
        for (int i = 0; i < 10; ++i)
        {
            REQUIRE(controller.setInput(constantInput));
            REQUIRE(controller.advance());
            outputs.push_back(controller.getOutput());
        }

        // Check convergence - changes should decrease
        for (size_t i = 2; i < outputs.size(); ++i)
        {
            double currentChange = (outputs[i] - outputs[i - 1]).norm();
            double previousChange = (outputs[i - 1] - outputs[i - 2]).norm();
            // Generally, change should decrease or remain small
            REQUIRE(currentChange <= previousChange + 1e-10);
        }
    }
}

TEST_CASE("EMAWithLimits - Soft Limits")
{
    SECTION("Soft limits reduce effective range")
    {
        const double scale = 1.0;
        const double alpha = 1.0;
        Eigen::VectorXd lowerLimit(1), upperLimit(1);
        lowerLimit << -4.0;
        upperLimit << 4.0;
        const double softLimitFactor = 0.5; // 50% of range

        auto handler
            = createBasicParameterHandler(scale, alpha, lowerLimit, upperLimit, softLimitFactor);
        EMAWithLimits controller;
        REQUIRE(controller.initialize(handler));
        REQUIRE(controller.reset(Eigen::VectorXd::Zero(1)));

        // Soft limits should be [-2, 2] (50% of [-4, 4])
        Eigen::VectorXd input(1);
        input << 1.0;
        REQUIRE(controller.setInput(input));
        REQUIRE(controller.advance());

        const auto& output = controller.getOutput();
        // Output should be within soft limits
        REQUIRE(output(0) >= -2.0 - 1e-6);
        REQUIRE(output(0) <= 2.0 + 1e-6);
    }

    SECTION("Very restrictive soft limits")
    {
        const double softLimitFactor = 0.1; // 10% of range
        Eigen::VectorXd lowerLimit(1), upperLimit(1);
        lowerLimit << -10.0;
        upperLimit << 10.0;

        auto handler
            = createBasicParameterHandler(1.0, 1.0, lowerLimit, upperLimit, softLimitFactor);
        EMAWithLimits controller;
        REQUIRE(controller.initialize(handler));
        REQUIRE(controller.reset(Eigen::VectorXd::Zero(1)));

        // Soft limits should be [-1, 1] (10% of [-10, 10])
        Eigen::VectorXd input(1);
        input << 1.0;
        REQUIRE(controller.setInput(input));
        REQUIRE(controller.advance());

        const auto& output = controller.getOutput();
        REQUIRE(std::abs(output(0)) <= 1.0 + 1e-6);
    }

    SECTION("Asymmetric joint limits with soft limits")
    {
        Eigen::VectorXd lowerLimit(1), upperLimit(1);
        lowerLimit << -1.0;
        upperLimit << 3.0; // Asymmetric
        const double softLimitFactor = 0.5;

        auto handler
            = createBasicParameterHandler(1.0, 1.0, lowerLimit, upperLimit, softLimitFactor);
        EMAWithLimits controller;
        REQUIRE(controller.initialize(handler));
        REQUIRE(controller.reset(Eigen::VectorXd::Zero(1)));

        // Center = (3.0 + (-1.0))/2 = 1.0
        // Range = 3.0 - (-1.0) = 4.0
        // Soft range = 4.0 * 0.5 = 2.0
        // Soft limits = [1.0 - 1.0, 1.0 + 1.0] = [0.0, 2.0]

        Eigen::VectorXd input(1);
        input << 0.0; // Should map to center
        REQUIRE(controller.setInput(input));
        REQUIRE(controller.advance());

        const auto& output = controller.getOutput();
        REQUIRE(output(0) == Approx(1.0).epsilon(1e-6)); // Center of soft limits
    }
}

TEST_CASE("EMAWithLimits - Reset Functionality")
{
    auto handler = createBasicParameterHandler(1.0, 0.5);
    EMAWithLimits controller;
    REQUIRE(controller.initialize(handler));

    SECTION("Reset clears EMA history")
    {
        // Build up some EMA history
        REQUIRE(controller.reset(Eigen::Vector3d::Zero()));
        Eigen::Vector3d input(0.8, -0.6, 0.4);
        for (int i = 0; i < 5; ++i)
        {
            REQUIRE(controller.setInput(input));
            REQUIRE(controller.advance());
        }
        auto outputWithHistory = controller.getOutput();

        // Reset and apply same input
        REQUIRE(controller.reset(Eigen::Vector3d::Zero()));
        REQUIRE(controller.setInput(input));
        REQUIRE(controller.advance());
        auto outputAfterReset = controller.getOutput();

        // Should be different due to cleared history
        REQUIRE((outputAfterReset - outputWithHistory).norm() > 1e-6);
    }

    SECTION("Reset initializes all internal vectors")
    {
        REQUIRE(controller.reset(Eigen::Vector3d::Zero()));

        // After reset, should be in WaitingForAdvance state
        REQUIRE_FALSE(controller.isOutputValid());

        // Should be able to advance after reset
        Eigen::Vector3d input(0.1, 0.2, 0.3);
        REQUIRE(controller.setInput(input));
        REQUIRE(controller.advance());
        REQUIRE(controller.isOutputValid());

        // Output should be valid and have correct size
        const auto& output = controller.getOutput();
        REQUIRE(output.size() == 3);
    }

    SECTION("Multiple consecutive resets")
    {
        for (int i = 0; i < 10; ++i)
        {
            REQUIRE(controller.reset(Eigen::Vector3d::Zero()));
            REQUIRE_FALSE(controller.isOutputValid());

            Eigen::Vector3d input(0.1 * i, 0.2 * i, 0.3 * i);
            REQUIRE(controller.setInput(input));
            REQUIRE(controller.advance());
            REQUIRE(controller.isOutputValid());
        }
    }

    SECTION("Reset with custom initial condition")
    {
        Eigen::Vector3d initialCondition(0.5, -0.3, 0.2);
        REQUIRE(controller.reset(initialCondition));

        // After reset, output should match initial condition
        const auto& output = controller.getOutput();
        REQUIRE((output - initialCondition).norm() < 1e-10);
        REQUIRE_FALSE(controller.isOutputValid()); // WaitingForAdvance state
    }

    SECTION("Reset with invalid initial condition size")
    {
        Eigen::Vector2d invalidInitialCondition(0.5, -0.3);
        REQUIRE_FALSE(controller.reset(invalidInitialCondition)); // Should fail
    }
}

TEST_CASE("EMAWithLimits - Edge Cases and Robustness")
{
    SECTION("Zero input produces center output")
    {
        auto handler = createBasicParameterHandler(1.0, 1.0);
        EMAWithLimits controller;
        REQUIRE(controller.initialize(handler));
        REQUIRE(controller.reset(Eigen::Vector3d::Zero()));

        Eigen::Vector3d input = Eigen::Vector3d::Zero();
        REQUIRE(controller.setInput(input));
        REQUIRE(controller.advance());

        const auto& output = controller.getOutput();
        // Zero input should produce output at center of joint ranges
        REQUIRE(output(0) == Approx(0.0).margin(1e-6));
        REQUIRE(output(1) == Approx(0.0).margin(1e-6));
        REQUIRE(output(2) == Approx(0.0).margin(1e-6));
    }

    SECTION("Extreme input values are handled gracefully")
    {
        auto handler = createBasicParameterHandler(1.0, 1.0);
        EMAWithLimits controller;
        REQUIRE(controller.initialize(handler));
        REQUIRE(controller.reset(Eigen::Vector3d::Zero()));

        Eigen::Vector3d extremeInput(1000.0, -1000.0, 500.0);
        REQUIRE(controller.setInput(extremeInput));
        REQUIRE(controller.advance());

        const auto& output = controller.getOutput();
        // Output should be within joint limits
        const Eigen::Vector3d lowerLimits(-1.0, -2.0, -3.0);
        const Eigen::Vector3d upperLimits(1.0, 2.0, 3.0);

        for (int i = 0; i < 3; ++i)
        {
            REQUIRE(output(i) >= lowerLimits(i) - 1e-6);
            REQUIRE(output(i) <= upperLimits(i) + 1e-6);
        }
    }

    SECTION("Sequential different inputs")
    {
        auto handler = createBasicParameterHandler(1.0, 0.5);
        EMAWithLimits controller;
        REQUIRE(controller.initialize(handler));
        REQUIRE(controller.reset(Eigen::Vector3d::Zero()));

        std::vector<Eigen::Vector3d> inputs = {Eigen::Vector3d(0.5, 0.0, -0.5),
                                               Eigen::Vector3d(-0.5, 0.5, 0.0),
                                               Eigen::Vector3d(0.0, -0.5, 0.5),
                                               Eigen::Vector3d(0.2, 0.3, -0.1)};

        std::vector<Eigen::VectorXd> outputs;
        for (const auto& input : inputs)
        {
            REQUIRE(controller.setInput(input));
            REQUIRE(controller.advance());
            outputs.push_back(controller.getOutput());
        }

        // All outputs should be different
        for (size_t i = 1; i < outputs.size(); ++i)
        {
            REQUIRE((outputs[i] - outputs[i - 1]).norm() > 1e-6);
        }
    }

    SECTION("Very small alpha value (slow convergence)")
    {
        auto handler = createBasicParameterHandler(1.0, 0.001);
        EMAWithLimits controller;
        REQUIRE(controller.initialize(handler));
        REQUIRE(controller.reset(Eigen::Vector3d::Zero()));

        Eigen::Vector3d input(1.0, 1.0, 1.0);
        REQUIRE(controller.setInput(input));
        REQUIRE(controller.advance());

        const auto& output = controller.getOutput();
        // With very small alpha, output should be very close to previous (0)
        REQUIRE(std::abs(output(0)) < 0.1);
        REQUIRE(std::abs(output(1)) < 0.1);
        REQUIRE(std::abs(output(2)) < 0.1);
    }
}

TEST_CASE("EMAWithLimits - Multi-Joint Scenarios")
{
    SECTION("Different joint limits per joint")
    {
        Eigen::VectorXd lowerLimit(4), upperLimit(4);
        lowerLimit << -1.0, -2.0, -5.0, -0.5;
        upperLimit << 1.0, 3.0, 5.0, 2.0;

        auto handler = createBasicParameterHandler(1.0, 0.8, lowerLimit, upperLimit, 1.0);
        EMAWithLimits controller;
        REQUIRE(controller.initialize(handler));
        REQUIRE(controller.reset(Eigen::Vector4d::Zero()));

        Eigen::Vector4d input(0.5, -0.5, 0.8, -0.3);
        REQUIRE(controller.setInput(input));
        REQUIRE(controller.advance());

        const auto& output = controller.getOutput();

        // Check each joint is within its limits
        for (int i = 0; i < 4; ++i)
        {
            REQUIRE(output(i) >= lowerLimit(i) - 1e-6);
            REQUIRE(output(i) <= upperLimit(i) + 1e-6);
        }
    }

    SECTION("Large number of joints performance")
    {
        const int numJoints = 100;
        Eigen::VectorXd lowerLimit = Eigen::VectorXd::Constant(numJoints, -1.0);
        Eigen::VectorXd upperLimit = Eigen::VectorXd::Constant(numJoints, 1.0);

        auto handler = createBasicParameterHandler(1.0, 0.7, lowerLimit, upperLimit, 0.8);
        EMAWithLimits controller;
        REQUIRE(controller.initialize(handler));
        REQUIRE(controller.reset(Eigen::VectorXd::Zero(numJoints)));

        Eigen::VectorXd input = Eigen::VectorXd::Random(numJoints);
        REQUIRE(controller.setInput(input));
        REQUIRE(controller.advance());

        const auto& output = controller.getOutput();
        REQUIRE(output.size() == numJoints);

        // All outputs should be within limits
        for (int i = 0; i < numJoints; ++i)
        {
            REQUIRE(output(i) >= lowerLimit(i) - 1e-6);
            REQUIRE(output(i) <= upperLimit(i) + 1e-6);
        }
    }
}

TEST_CASE("EMAWithLimits - Error Handling")
{
    SECTION("Operations without initialization")
    {
        EMAWithLimits controller;

        // setInput should fail
        Eigen::Vector3d input(0.1, 0.2, 0.3);
        REQUIRE_FALSE(controller.setInput(input));

        // isOutputValid should be false
        REQUIRE_FALSE(controller.isOutputValid());
    }

    SECTION("Multiple initialization calls")
    {
        auto handler1 = createBasicParameterHandler(1.0, 0.5);
        auto handler2 = createBasicParameterHandler(2.0, 0.8);

        EMAWithLimits controller;
        REQUIRE(controller.initialize(handler1));
        REQUIRE(controller.initialize(handler2)); // Re-initialization should work

        // Should work with new parameters
        REQUIRE(controller.reset(Eigen::Vector3d::Zero()));
        Eigen::Vector3d input(0.1, 0.2, 0.3);
        REQUIRE(controller.setInput(input));
        REQUIRE(controller.advance());
        REQUIRE(controller.isOutputValid());
    }

    SECTION("Output access before and after operations")
    {
        auto handler = createBasicParameterHandler();
        EMAWithLimits controller;
        REQUIRE(controller.initialize(handler));

        // Output should exist after initialization (from reset)
        const auto& outputAfterInit = controller.getOutput();
        REQUIRE(outputAfterInit.size() == 3);

        // But should not be valid until advance
        REQUIRE_FALSE(controller.isOutputValid());

        // After advance, should be valid
        Eigen::Vector3d input(0.1, 0.2, 0.3);
        REQUIRE(controller.setInput(input));
        REQUIRE(controller.advance());
        REQUIRE(controller.isOutputValid());

        const auto& outputAfterAdvance = controller.getOutput();
        REQUIRE(outputAfterAdvance.size() == 3);
    }
}

TEST_CASE("EMAWithLimits - Deterministic Behavior")
{
    SECTION("Same inputs produce same outputs")
    {
        auto handler = createBasicParameterHandler(1.0, 0.6);

        // Two identical controllers
        EMAWithLimits controller1, controller2;
        REQUIRE(controller1.initialize(handler));
        REQUIRE(controller2.initialize(handler));

        std::vector<Eigen::Vector3d> inputs = {Eigen::Vector3d(0.1, 0.2, 0.3),
                                               Eigen::Vector3d(-0.2, 0.4, -0.1),
                                               Eigen::Vector3d(0.5, -0.3, 0.2),
                                               Eigen::Vector3d(0.0, 0.0, 0.0)};

        for (const auto& input : inputs)
        {
            REQUIRE(controller1.setInput(input));
            REQUIRE(controller2.setInput(input));
            REQUIRE(controller1.advance());
            REQUIRE(controller2.advance());

            auto output1 = controller1.getOutput();
            auto output2 = controller2.getOutput();

            // Outputs should be identical
            REQUIRE((output1 - output2).norm() < 1e-15);
        }
    }

    SECTION("Repeated operations with constant input converge")
    {
        auto handler = createBasicParameterHandler(1.0, 0.9);
        EMAWithLimits controller;
        REQUIRE(controller.initialize(handler));
        REQUIRE(controller.reset(Eigen::Vector3d::Zero()));

        const Eigen::Vector3d constantInput(0.6, -0.4, 0.2);

        // Apply same input many times
        for (int i = 0; i < 50; ++i)
        {
            REQUIRE(controller.setInput(constantInput));
            REQUIRE(controller.advance());
        }

        auto finalOutput = controller.getOutput();

        // One more step should produce very similar result
        REQUIRE(controller.setInput(constantInput));
        REQUIRE(controller.advance());
        auto nextOutput = controller.getOutput();

        // Should be very close (converged)
        REQUIRE((nextOutput - finalOutput).norm() < 1e-10);
    }
}

TEST_CASE("EMAWithLimits - Non-Zero Reset Values")
{
    // Create controller with known limits
    // Lower limits: [-1.0, -2.0, -3.0]
    // Upper limits: [1.0, 2.0, 3.0]
    auto handler = createBasicParameterHandler(1.0, 0.8);
    EMAWithLimits controller;
    REQUIRE(controller.initialize(handler));

    SECTION("Reset with valid non-zero initial condition")
    {
        Eigen::Vector3d initialCondition(0.5, -1.0, 2.5);
        REQUIRE(controller.reset(initialCondition));

        // Output should match the initial condition after reset
        const auto& output = controller.getOutput();
        REQUIRE(output(0) == Approx(0.5).epsilon(1e-10));
        REQUIRE(output(1) == Approx(-1.0).epsilon(1e-10));
        REQUIRE(output(2) == Approx(2.5).epsilon(1e-10));

        // Should be in WaitingForAdvance state
        REQUIRE_FALSE(controller.isOutputValid());
    }

    SECTION("Reset with random values within limits")
    {
        // Test multiple random reset values
        for (int trial = 0; trial < 10; ++trial)
        {
            // Generate random values within limits
            Eigen::Vector3d randomInitial;
            randomInitial << -1.0 + 2.0 * static_cast<double>(rand()) / RAND_MAX, // [-1.0, 1.0]
                -2.0 + 4.0 * static_cast<double>(rand()) / RAND_MAX, // [-2.0, 2.0]
                -3.0 + 6.0 * static_cast<double>(rand()) / RAND_MAX; // [-3.0, 3.0]

            REQUIRE(controller.reset(randomInitial));

            const auto& output = controller.getOutput();
            REQUIRE(output(0) == Approx(randomInitial(0)).epsilon(1e-10));
            REQUIRE(output(1) == Approx(randomInitial(1)).epsilon(1e-10));
            REQUIRE(output(2) == Approx(randomInitial(2)).epsilon(1e-10));

            // Verify output is within limits
            REQUIRE(output(0) >= -1.0);
            REQUIRE(output(0) <= 1.0);
            REQUIRE(output(1) >= -2.0);
            REQUIRE(output(1) <= 2.0);
            REQUIRE(output(2) >= -3.0);
            REQUIRE(output(2) <= 3.0);
        }
    }

    SECTION("Reset with values outside limits - clipping lower bound")
    {
        // Values below lower limits should be clipped
        Eigen::Vector3d belowLimits(-5.0, -10.0, -15.0);
        REQUIRE(controller.reset(belowLimits));

        const auto& output = controller.getOutput();
        // Should be clipped to lower limits
        REQUIRE(output(0) == Approx(-1.0).epsilon(1e-10));
        REQUIRE(output(1) == Approx(-2.0).epsilon(1e-10));
        REQUIRE(output(2) == Approx(-3.0).epsilon(1e-10));
    }

    SECTION("Reset with values outside limits - clipping upper bound")
    {
        // Values above upper limits should be clipped
        Eigen::Vector3d aboveLimits(5.0, 10.0, 15.0);
        REQUIRE(controller.reset(aboveLimits));

        const auto& output = controller.getOutput();
        // Should be clipped to upper limits
        REQUIRE(output(0) == Approx(1.0).epsilon(1e-10));
        REQUIRE(output(1) == Approx(2.0).epsilon(1e-10));
        REQUIRE(output(2) == Approx(3.0).epsilon(1e-10));
    }

    SECTION("Reset with mixed in/out of bounds values")
    {
        // Some values within limits, some outside
        Eigen::Vector3d mixedValues(-5.0, 0.5, 10.0);
        REQUIRE(controller.reset(mixedValues));

        const auto& output = controller.getOutput();
        // First value clipped to lower limit
        REQUIRE(output(0) == Approx(-1.0).epsilon(1e-10));
        // Second value unchanged (within limits)
        REQUIRE(output(1) == Approx(0.5).epsilon(1e-10));
        // Third value clipped to upper limit
        REQUIRE(output(2) == Approx(3.0).epsilon(1e-10));
    }

    SECTION("Reset at exact limit boundaries")
    {
        // Test at lower boundaries
        Eigen::Vector3d atLowerLimits(-1.0, -2.0, -3.0);
        REQUIRE(controller.reset(atLowerLimits));
        const auto& outputLower = controller.getOutput();
        REQUIRE(outputLower(0) == Approx(-1.0).epsilon(1e-10));
        REQUIRE(outputLower(1) == Approx(-2.0).epsilon(1e-10));
        REQUIRE(outputLower(2) == Approx(-3.0).epsilon(1e-10));

        // Test at upper boundaries
        Eigen::Vector3d atUpperLimits(1.0, 2.0, 3.0);
        REQUIRE(controller.reset(atUpperLimits));
        const auto& outputUpper = controller.getOutput();
        REQUIRE(outputUpper(0) == Approx(1.0).epsilon(1e-10));
        REQUIRE(outputUpper(1) == Approx(2.0).epsilon(1e-10));
        REQUIRE(outputUpper(2) == Approx(3.0).epsilon(1e-10));
    }

    SECTION("EMA behavior starts from non-zero reset value")
    {
        // Reset to a specific non-zero value
        Eigen::Vector3d initialCondition(0.5, 1.5, -2.5);
        REQUIRE(controller.reset(initialCondition));

        // Apply zero input - EMA should move towards center
        Eigen::Vector3d zeroInput = Eigen::Vector3d::Zero();
        REQUIRE(controller.setInput(zeroInput));
        REQUIRE(controller.advance());

        const auto& output = controller.getOutput();

        // With alpha=0.8 and zero input, output should be between initial and center
        // The exact value depends on scaling and soft limits, but should be within hard limits
        REQUIRE(output(0) >= -1.0);
        REQUIRE(output(0) <= 1.0);
        REQUIRE(output(1) >= -2.0);
        REQUIRE(output(1) <= 2.0);
        REQUIRE(output(2) >= -3.0);
        REQUIRE(output(2) <= 3.0);
    }

    SECTION("Multiple resets with random values")
    {
        for (int trial = 0; trial < 20; ++trial)
        {
            // Generate random values (potentially outside limits)
            Eigen::Vector3d randomInitial;
            randomInitial << -5.0 + 10.0 * static_cast<double>(rand()) / RAND_MAX, // [-5.0, 5.0]
                -10.0 + 20.0 * static_cast<double>(rand()) / RAND_MAX, // [-10.0, 10.0]
                -15.0 + 30.0 * static_cast<double>(rand()) / RAND_MAX; // [-15.0, 15.0]

            REQUIRE(controller.reset(randomInitial));

            const auto& output = controller.getOutput();

            // Verify output is always within hard limits (clipped)
            REQUIRE(output(0) >= -1.0);
            REQUIRE(output(0) <= 1.0);
            REQUIRE(output(1) >= -2.0);
            REQUIRE(output(1) <= 2.0);
            REQUIRE(output(2) >= -3.0);
            REQUIRE(output(2) <= 3.0);

            // Controller should be in WaitingForAdvance state after reset
            REQUIRE_FALSE(controller.isOutputValid());

            // Should be able to advance after reset
            Eigen::Vector3d input = Eigen::Vector3d::Random() * 0.5;
            REQUIRE(controller.setInput(input));
            REQUIRE(controller.advance());
            REQUIRE(controller.isOutputValid());
        }
    }

    SECTION("Reset with extreme values")
    {
        // Test with very large values
        Eigen::Vector3d extremeValues(1e6, -1e6, 1e10);
        REQUIRE(controller.reset(extremeValues));

        const auto& output = controller.getOutput();
        // Should be clipped to limits
        REQUIRE(output(0) == Approx(1.0).epsilon(1e-10));
        REQUIRE(output(1) == Approx(-2.0).epsilon(1e-10));
        REQUIRE(output(2) == Approx(3.0).epsilon(1e-10));
    }
}
