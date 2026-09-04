/**
 * @file ParametersHandlerDinRailTest.cpp
 * @copyright 2026 Generative Bionics S.R.L. This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <catch2/catch_test_macros.hpp>

#include <BipedalLocomotion/ParametersHandler/DinRailImplementation.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>

#include <dinrail/Parameters.h>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

using namespace std::chrono_literals;
using namespace BipedalLocomotion::ParametersHandler;

TEST_CASE("DinRail parameters can be exposed through IParametersHandler")
{
    dinrail::Parameters parameters;
    parameters.put("answer", 42);
    parameters.put("pi", 3.14);
    parameters.put("name", "dinrail");
    parameters.put("enabled", true);
    parameters.put("period", 10ms);
    parameters.put("flags", std::vector<bool>{true, false, true});
    parameters.put("indices", std::vector<int>{1, 2, 3});
    parameters.put("gains", std::vector<double>{1.0, 2.5, 4.0});
    parameters.put("joints", std::vector<std::string>{"hip", "knee"});
    parameters.put("timeouts", std::vector<std::chrono::nanoseconds>{1ms, 2ms});
    parameters.addGroup("nested").put("value", 7);

    IParametersHandler::shared_ptr handler = std::make_shared<DinRailImplementation>(parameters);

    int answer{};
    double pi{};
    std::string name;
    bool enabled{};
    std::chrono::nanoseconds period{};
    std::vector<bool> flags;
    std::vector<int> indices;
    std::vector<double> gains;
    std::vector<std::string> joints;
    std::vector<std::chrono::nanoseconds> timeouts;

    REQUIRE(handler->getParameter("answer", answer));
    REQUIRE(answer == 42);
    REQUIRE(handler->getParameter("pi", pi));
    REQUIRE(pi == 3.14);
    REQUIRE(handler->getParameter("name", name));
    REQUIRE(name == "dinrail");
    REQUIRE(handler->getParameter("enabled", enabled));
    REQUIRE(enabled);
    REQUIRE(handler->getParameter("period", period));
    REQUIRE(period == 10ms);
    REQUIRE(handler->getParameter("flags", flags));
    REQUIRE(flags == (std::vector<bool>{true, false, true}));
    REQUIRE(handler->getParameter("indices", indices));
    REQUIRE(indices == (std::vector<int>{1, 2, 3}));
    REQUIRE(handler->getParameter("gains", gains));
    REQUIRE(gains == (std::vector<double>{1.0, 2.5, 4.0}));
    REQUIRE(handler->getParameter("joints", joints));
    REQUIRE(joints == (std::vector<std::string>{"hip", "knee"}));
    REQUIRE(handler->getParameter("timeouts", timeouts));
    REQUIRE(timeouts == (std::vector<std::chrono::nanoseconds>{1ms, 2ms}));

    auto nested = handler->getGroup("nested").lock();
    REQUIRE(nested);
    int nestedValue{};
    REQUIRE(nested->getParameter("value", nestedValue));
    REQUIRE(nestedValue == 7);
    REQUIRE(handler->getGroup("nested").lock() == nested);
    REQUIRE(handler->getGroup("missing").expired());
}

TEST_CASE("DinRail implementation supports the IParametersHandler mutations")
{
    IParametersHandler::shared_ptr handler = std::make_shared<DinRailImplementation>();
    REQUIRE(handler->isEmpty());

    handler->setParameter("integer", 5);
    handler->setParameter("double", 2.0);
    handler->setParameter("string", std::string{"value"});
    handler->setParameter("c-string", "value");
    handler->setParameter("bool", true);
    handler->setParameter("duration", 3ms);
    handler->setParameter("bool-vector", std::vector<bool>{false, true});
    handler->setParameter("int-vector", std::vector<int>{4, 5});
    handler->setParameter("double-vector", std::vector<double>{1.5, 2.5});
    handler->setParameter("string-vector", std::vector<std::string>{"a", "b"});
    handler->setParameter("duration-vector", std::vector<std::chrono::nanoseconds>{4ms, 5ms});
    REQUIRE_FALSE(handler->isEmpty());

    auto group = std::make_shared<DinRailImplementation>();
    group->setParameter("group-value", 11);
    REQUIRE(handler->setGroup("group", group));
    REQUIRE_FALSE(handler->setGroup("wrong-type", std::make_shared<StdImplementation>()));

    auto storedGroup = handler->getGroup("group").lock();
    REQUIRE(storedGroup);
    int groupValue{};
    REQUIRE(storedGroup->getParameter("group-value", groupValue));
    REQUIRE(groupValue == 11);

    auto clone = handler->clone();
    REQUIRE(clone);
    int integer{};
    REQUIRE(clone->getParameter("integer", integer));
    REQUIRE(integer == 5);
    REQUIRE(clone->getGroup("group").lock());

    handler->clear();
    REQUIRE(handler->isEmpty());
    REQUIRE(handler->getGroup("group").expired());
    REQUIRE_FALSE(clone->isEmpty());
    REQUIRE(handler->toString() == "DinRailImplementation");
}
