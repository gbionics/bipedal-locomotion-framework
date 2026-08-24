/**
 * @file JointLimitsTaskTest.cpp
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <chrono>
#include <cmath>

// Catch2
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

// BipedalLocomotion
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>
#include <BipedalLocomotion/System/VariablesHandler.h>
#include <BipedalLocomotion/IK/JointLimitsTask.h>

#include <OsqpEigen/Constants.hpp>

#include <iDynTree/EigenHelpers.h>
#include <iDynTree/ModelTestUtils.h>

#include <Eigen/Dense>

using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::System;
using namespace BipedalLocomotion::IK;

TEST_CASE("Joint Regularization task")
{
    using namespace std::chrono_literals;
    const std::string robotVelocity = "robotVelocity";

    Eigen::VectorXd klim;
    constexpr std::chrono::nanoseconds dt = 10ms;

    auto kinDyn = std::make_shared<iDynTree::KinDynComputations>();
    auto parameterHandler = std::make_shared<StdImplementation>();

    parameterHandler->setParameter("robot_velocity_variable_name",
                                   robotVelocity);
    parameterHandler->setParameter("sampling_time", dt);
    parameterHandler->setParameter("use_model_limits", false);

    // set the velocity representation
    REQUIRE(kinDyn->setFrameVelocityRepresentation(iDynTree::FrameVelocityRepresentation::MIXED_REPRESENTATION));

    for (std::size_t numberOfJoints = 6; numberOfJoints < 40; numberOfJoints += 15)
    {
        DYNAMIC_SECTION("Model with " << numberOfJoints << " joints")
        {
            // create the model
            const iDynTree::Model model = iDynTree::getRandomModel(numberOfJoints);
            REQUIRE(kinDyn->loadRobotModel(model));

            const auto worldBasePos = iDynTree::getRandomTransform();
            const auto baseVel = iDynTree::getRandomTwist();
            iDynTree::VectorDynSize jointsPos(model.getNrOfDOFs());
            iDynTree::VectorDynSize jointsVel(model.getNrOfDOFs());
            iDynTree::Vector3 gravity;

            for (auto& joint : jointsPos)
            {
                joint = iDynTree::getRandomDouble();
            }

            for (auto& joint : jointsVel)
            {
                joint = iDynTree::getRandomDouble();
            }

            for (auto& element : gravity)
            {
                element = iDynTree::getRandomDouble();
            }

            REQUIRE(kinDyn->setRobotState(worldBasePos, jointsPos, baseVel, jointsVel, gravity));

            // Instantiate the handler
            VariablesHandler variablesHandler;
            variablesHandler.addVariable("dummy1", 10);
            variablesHandler.addVariable(robotVelocity, model.getNrOfDOFs() + 6);
            variablesHandler.addVariable("dummy2", 15);


            klim.resize(model.getNrOfDOFs());
            klim.setConstant(0.7);
            parameterHandler->setParameter("klim", klim);

            Eigen::VectorXd deltaLimit(jointsPos.size());
            deltaLimit.setConstant(0.1);
            const Eigen::VectorXd upperLimits = iDynTree::toEigen(jointsPos) + deltaLimit;
            const Eigen::VectorXd lowerLimits = iDynTree::toEigen(jointsPos) - deltaLimit;
            parameterHandler->setParameter("upper_limits", upperLimits);
            parameterHandler->setParameter("lower_limits", lowerLimits);

            JointLimitsTask task;
            REQUIRE(task.setKinDyn(kinDyn));
            REQUIRE(task.initialize(parameterHandler));
            REQUIRE(task.setVariablesHandler(variablesHandler));

            REQUIRE(task.update());
            REQUIRE(task.isValid());

            // get A and b
            Eigen::Ref<const Eigen::MatrixXd> A = task.getA();
            Eigen::Ref<const Eigen::VectorXd> b = task.getB();

            // check the matrix A
            REQUIRE(A.middleCols(variablesHandler.getVariable("dummy1").offset,
                                 variablesHandler.getVariable("dummy1").size)
                        .isZero());

            REQUIRE(A.middleCols(variablesHandler.getVariable("dummy2").offset,
                                 variablesHandler.getVariable("dummy2").size)
                        .isZero());
            REQUIRE(A.block(0,
                            variablesHandler.getVariable(robotVelocity).offset + 6,
                            model.getNrOfDOFs(),
                            model.getNrOfDOFs())
                        .isIdentity());
            REQUIRE(A.block(model.getNrOfDOFs(),
                            variablesHandler.getVariable(robotVelocity).offset + 6,
                            model.getNrOfDOFs(),
                            model.getNrOfDOFs())
                        .isDiagonal());
            REQUIRE(A.block(model.getNrOfDOFs(),
                            variablesHandler.getVariable(robotVelocity).offset + 6,
                            model.getNrOfDOFs(),
                            model.getNrOfDOFs()).diagonal().isConstant(-1));

            // check the vector b
            Eigen::VectorXd expectedB;
            expectedB = klim.asDiagonal() * deltaLimit / std::chrono::duration<double>(dt).count();
            REQUIRE(b.head(model.getNrOfDOFs()).isApprox(expectedB));

            expectedB = klim.asDiagonal() * (deltaLimit) / std::chrono::duration<double>(dt).count();
            REQUIRE(b.tail(model.getNrOfDOFs()).isApprox(expectedB));

        }
    }
}

TEST_CASE("Joint Limits task (some joints without limits)")
{
    using namespace std::chrono_literals;
    const std::string robotVelocity = "robotVelocity";

    constexpr std::chrono::nanoseconds dt = 10ms;
    const double dtSeconds = std::chrono::duration<double>(dt).count();
    constexpr double lowerLimit = -1.0;
    constexpr double upperLimit = 1.0;

    auto kinDyn = std::make_shared<iDynTree::KinDynComputations>();
    auto parameterHandler = std::make_shared<StdImplementation>();

    parameterHandler->setParameter("robot_velocity_variable_name", robotVelocity);
    parameterHandler->setParameter("sampling_time", dt);
    parameterHandler->setParameter("use_model_limits", true);

    REQUIRE(kinDyn->setFrameVelocityRepresentation(iDynTree::FrameVelocityRepresentation::MIXED_REPRESENTATION));

    iDynTree::Model model = iDynTree::getRandomModel(15);

    // enable position limits only for the joints with an even DoF index, so that the model has
    // both limited and unlimited joints and the else-branch of update() is exercised
    std::vector<bool> hasLimits(model.getNrOfDOFs(), false);
    for (std::size_t jointIdx = 0; jointIdx < model.getNrOfJoints(); jointIdx++)
    {
        iDynTree::IJointPtr joint = model.getJoint(jointIdx);
        if (joint->getNrOfDOFs() != 1)
        {
            continue;
        }

        const std::size_t dofIndex = joint->getDOFsOffset();
        if (dofIndex % 2 == 0)
        {
            joint->enablePosLimits(true);
            joint->setPosLimits(0, lowerLimit, upperLimit);
            hasLimits[dofIndex] = true;
        } else
        {
            joint->enablePosLimits(false);
        }
    }

    REQUIRE(kinDyn->loadRobotModel(model));

    const auto worldBasePos = iDynTree::getRandomTransform();
    const auto baseVel = iDynTree::getRandomTwist();
    iDynTree::VectorDynSize jointsPos(model.getNrOfDOFs());
    iDynTree::VectorDynSize jointsVel(model.getNrOfDOFs());
    iDynTree::Vector3 gravity;

    // keep the joint position at the middle of the limited joints range
    for (auto& joint : jointsPos)
    {
        joint = 0.0;
    }

    for (auto& joint : jointsVel)
    {
        joint = iDynTree::getRandomDouble();
    }

    for (auto& element : gravity)
    {
        element = iDynTree::getRandomDouble();
    }

    REQUIRE(kinDyn->setRobotState(worldBasePos, jointsPos, baseVel, jointsVel, gravity));

    VariablesHandler variablesHandler;
    variablesHandler.addVariable(robotVelocity, model.getNrOfDOFs() + 6);

    Eigen::VectorXd klim(model.getNrOfDOFs());
    klim.setConstant(0.7);
    // set to zero the gain of one of the unlimited joints to check that this does not produce NaN
    for (std::size_t i = 0; i < hasLimits.size(); i++)
    {
        if (!hasLimits[i])
        {
            klim(i) = 0.0;
            break;
        }
    }
    parameterHandler->setParameter("klim", klim);

    JointLimitsTask task;
    REQUIRE(task.setKinDyn(kinDyn));
    REQUIRE(task.initialize(parameterHandler));
    REQUIRE(task.setVariablesHandler(variablesHandler));

    REQUIRE(task.update());
    REQUIRE(task.isValid());

    Eigen::Ref<const Eigen::VectorXd> b = task.getB();

    for (std::size_t i = 0; i < model.getNrOfDOFs(); i++)
    {
        REQUIRE_FALSE(std::isnan(b(i)));
        REQUIRE_FALSE(std::isnan(b(model.getNrOfDOFs() + i)));

        if (hasLimits[i])
        {
            const double expectedUpper = klim(i) * (upperLimit - jointsPos(i)) / dtSeconds;
            const double expectedLower = klim(i) * (jointsPos(i) - lowerLimit) / dtSeconds;
            REQUIRE(b(i) == Catch::Approx(expectedUpper));
            REQUIRE(b(model.getNrOfDOFs() + i) == Catch::Approx(expectedLower));
        } else
        {
            REQUIRE(b(i) == OsqpEigen::INFTY);
            REQUIRE(b(model.getNrOfDOFs() + i) == OsqpEigen::INFTY);
        }
    }
}
