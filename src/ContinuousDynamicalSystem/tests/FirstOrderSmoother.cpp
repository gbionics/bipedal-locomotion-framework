/**
 * @file FirstOrderSmoother.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <chrono>
#include <limits>
#include <memory>

// Catch2
#include <catch2/catch_test_macros.hpp>

#include <Eigen/Dense>

#include <BipedalLocomotion/ContinuousDynamicalSystem/FirstOrderSmoother.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>
#include <BipedalLocomotion/TestUtils/MemoryAllocationMonitor.h>

using namespace BipedalLocomotion::ContinuousDynamicalSystem;
using namespace BipedalLocomotion::TestUtils;

TEST_CASE("First order smoother")
{
    using namespace std::chrono_literals;

    constexpr std::chrono::nanoseconds dT = 100us;
    constexpr double settlingTime = 0.1;
    constexpr double tolerance = 1e-2;

    auto params = std::make_shared<BipedalLocomotion::ParametersHandler::StdImplementation>();
    params->setParameter("settling_time", settlingTime);
    params->setParameter("sampling_time", dT);

    FirstOrderSmoother smoother;
    REQUIRE(smoother.initialize(params));

    Eigen::Vector2d initialState = Eigen::Vector2d::Zero();
    Eigen::Vector2d input = Eigen::Vector2d::Ones();

    REQUIRE(smoother.reset(initialState));
    REQUIRE(smoother.setInput(input));

    double settilingTimeSubSystem1 = std::numeric_limits<double>::infinity();
    double settilingTimeSubSystem2 = std::numeric_limits<double>::infinity();
    bool settilingSubSystem1 = false;
    bool settilingSubSystem2 = false;

    // the presented dynamical has the following close form solution in case of step response
    auto closeFormSolution = [settlingTime](const double& t) -> Eigen::Vector2d {
        const double a = 3.0 / settlingTime;
        Eigen::Vector2d sol;
        sol(0) = 1 - std::exp(-a * t);
        sol(1) = 1 - std::exp(-a * t);
        return sol;
    };

    for (unsigned int i = 0; i < 1000; i++)
    {
        const Eigen::Vector2d output = smoother.getOutput();

        // check if the solution is similar to the expected one
        REQUIRE(output.isApprox(closeFormSolution(std::chrono::duration<double>(dT * i).count()),
                                tolerance));

        if (!settilingSubSystem1)
        {
            if ((output[0] > 0.95) && (output[0] < 1.05))
            {
                settilingTimeSubSystem1 = std::chrono::duration<double>(dT * i).count();
                settilingSubSystem1 = true;
            }
        } else if ((output[0] < 0.95) || (output[0] > 1.05))
        {
            settilingTimeSubSystem1 = std::numeric_limits<double>::infinity();
            settilingSubSystem1 = false;
        }

        if (!settilingSubSystem2)
        {
            if ((output[1] > 0.95) && (output[1] < 1.05))
            {
                settilingTimeSubSystem2 = std::chrono::duration<double>(dT * i).count();
                settilingSubSystem2 = true;
            }
        } else if ((output[1] < 0.95) || (output[1] > 1.05))
        {
            settilingTimeSubSystem2 = std::numeric_limits<double>::infinity();
            settilingSubSystem2 = false;
        }

        // advance the smoother

        // We only test no memory allocation from the second step,
        // as FirstOrderSmoother allocates memory at the first step
        if (i >= 1)
        {
            MemoryAllocationMonitor::startMonitor();
        }
        REQUIRE(smoother.advance());
        if (i >= 1)
        {
            REQUIRE(MemoryAllocationMonitor::endMonitorAndCheckNoMemoryAllocationInLastMonitor());
        }
    }

    // check the settling time
    REQUIRE(settilingTimeSubSystem1 <= settlingTime);
    REQUIRE(settilingTimeSubSystem2 <= settlingTime);
}

TEST_CASE("First order smoother - time-varying settling time")
{
    using namespace std::chrono_literals;

    constexpr std::chrono::nanoseconds dT = 100us;
    constexpr double initialSettlingTime = 0.1;
    constexpr double updatedSettlingTime = 0.05;
    constexpr double tolerance = 1e-2;

    auto params = std::make_shared<BipedalLocomotion::ParametersHandler::StdImplementation>();
    params->setParameter("settling_time", initialSettlingTime);
    params->setParameter("sampling_time", dT);

    FirstOrderSmoother smoother;

    // the settling time cannot be changed before the class is initialized
    REQUIRE_FALSE(smoother.setSettlingTime(updatedSettlingTime));

    REQUIRE(smoother.initialize(params));

    // the settling time cannot be changed before reset() since the size of the system is unknown
    REQUIRE_FALSE(smoother.setSettlingTime(updatedSettlingTime));

    Eigen::Vector2d initialState = Eigen::Vector2d::Zero();
    Eigen::Vector2d input = Eigen::Vector2d::Ones();

    REQUIRE(smoother.reset(initialState));
    REQUIRE(smoother.setInput(input));
    REQUIRE(smoother.getSettlingTime() == initialSettlingTime);

    // a non positive settling time is not valid
    REQUIRE_FALSE(smoother.setSettlingTime(0.0));
    REQUIRE_FALSE(smoother.setSettlingTime(-1.0));

    // let the system evolve for half of the initial settling time
    for (unsigned int i = 0; i < 500; i++)
    {
        REQUIRE(smoother.advance());
    }

    // speed up the response by reducing the settling time. The smoother should keep the current
    // state as initial condition of the new dynamics
    REQUIRE(smoother.setSettlingTime(updatedSettlingTime));
    REQUIRE(smoother.getSettlingTime() == updatedSettlingTime);

    const Eigen::Vector2d stateAtSwitch = smoother.getOutput();
    const double a = 3.0 / updatedSettlingTime;
    auto closeFormSolution = [&stateAtSwitch, a](const double& t) -> Eigen::Vector2d {
        return Eigen::Vector2d::Ones() - (Eigen::Vector2d::Ones() - stateAtSwitch) * std::exp(-a * t);
    };

    for (unsigned int i = 0; i < 1000; i++)
    {
        const Eigen::Vector2d output = smoother.getOutput();
        REQUIRE(output.isApprox(closeFormSolution(std::chrono::duration<double>(dT * i).count()),
                                tolerance));
        REQUIRE(smoother.advance());
    }

    // since the settling time has been reduced the system should have already converged to the
    // input
    REQUIRE(smoother.getOutput().isApprox(input, tolerance));
}
