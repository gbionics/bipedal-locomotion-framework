/**
 * @file ParametersHandlerYarpTest.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

// std
#include <chrono>
#include <memory>

// Catch2
#include <catch2/catch_test_macros.hpp>

// YARP
#include <yarp/os/Property.h>
#include <yarp/os/Searchable.h>
#include <yarp/os/Value.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Bottle.h>

#include <BipedalLocomotion/GenericContainer/Vector.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h>

#include <ConfigFolderPath.h>

using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::GenericContainer;

TEST_CASE("Get parameters")
{
    using namespace std::chrono_literals;
    std::vector<int> fibonacciNumbers{1, 1, 2, 3, 5, 8, 13, 21};
    std::vector<std::string> donaldsNephews{"Huey", "Dewey", "Louie"};
    bool flag = true;
    std::vector<bool> flags{true, false, false, true, true, true};

    std::shared_ptr<YarpImplementation> originalHandler = std::make_shared<YarpImplementation>();
    IParametersHandler::shared_ptr parameterHandler = originalHandler;
    std::chrono::nanoseconds time = 13h + 39min + 51s + 523ms;
    double timeAsDouble = 0.01;

    parameterHandler->setParameter("answer_to_the_ultimate_question_of_life", 42);
    parameterHandler->setParameter("pi", 3.14);
    parameterHandler->setParameter("John", "Smith");
    parameterHandler->setParameter("Fibonacci Numbers", fibonacciNumbers);
    parameterHandler->setParameter("flag", flag);
    parameterHandler->setParameter("flags", flags);
    parameterHandler->setParameter("time", time);
    parameterHandler->setParameter("time_as_double", timeAsDouble);

    SECTION("Get integer")
    {
        int element;
        REQUIRE(parameterHandler->getParameter("answer_to_the_ultimate_question_of_life", element));
        REQUIRE(element == 42);
    }

    SECTION("Get Double")
    {
        double element;
        REQUIRE(parameterHandler->getParameter("pi", element));
        REQUIRE(element == 3.14);
    }

    SECTION("Get Bool")
    {
        bool element ;
        REQUIRE(parameterHandler->getParameter("flag", element));
        REQUIRE(element == flag);
    }

    SECTION("Get String")
    {
        std::string element;
        REQUIRE(parameterHandler->getParameter("John", element));
        REQUIRE(element == "Smith");
    }

    SECTION("Get std::chrono::nanoseconds")
    {
        std::chrono::nanoseconds element;
        REQUIRE(parameterHandler->getParameter("time", element));
        REQUIRE(element == time);
    }

    SECTION("Get std::chrono::nanoseconds from double")
    {
        std::chrono::nanoseconds element;
        REQUIRE(parameterHandler->getParameter("time_as_double", element));
        REQUIRE(element == std::chrono::duration<double>(timeAsDouble));
    }

    SECTION("Change String")
    {
        parameterHandler->setParameter("John", "Doe");
        std::string element;
        REQUIRE(parameterHandler->getParameter("John", element));
        REQUIRE(element == "Doe");
    }

    SECTION("Get Vector")
    {
        std::vector<int> element;
        REQUIRE(parameterHandler->getParameter("Fibonacci Numbers", element));
        REQUIRE(element == fibonacciNumbers);
    }

    SECTION("Get bool Vector")
    {
        std::vector<bool> element;
        REQUIRE(parameterHandler->getParameter("flags", element));
        REQUIRE(element == flags);
    }

    SECTION("Change Vector")
    {
        fibonacciNumbers.push_back(34);
        parameterHandler->setParameter("Fibonacci Numbers", fibonacciNumbers);
        std::vector<int> element;
        REQUIRE(parameterHandler->getParameter("Fibonacci Numbers", element));
        REQUIRE(element == fibonacciNumbers);
    }

    SECTION("Set/Get Group")
    {
        IParametersHandler::shared_ptr setGroup = std::make_shared<YarpImplementation>();
        setGroup->setParameter("Donald's nephews", donaldsNephews);
        REQUIRE(parameterHandler->setGroup("CARTOONS", setGroup));
        IParametersHandler::shared_ptr cartoonsGroup = parameterHandler->getGroup("CARTOONS").lock();
        REQUIRE(cartoonsGroup);

        std::vector<std::string> element;
        REQUIRE(cartoonsGroup->getParameter("Donald's nephews", element));
        REQUIRE(element == donaldsNephews);
    }

    SECTION("Print content")
    {
        std::cout << "Parameters: " << parameterHandler->toString() << std::endl;
    }

    SECTION("is Empty")
    {
        IParametersHandler::shared_ptr groupHandler = parameterHandler->getGroup("CARTOONS").lock();
        REQUIRE_FALSE(groupHandler);
        IParametersHandler::shared_ptr setGroup = std::make_shared<YarpImplementation>();
        REQUIRE(parameterHandler->setGroup("CARTOONS", setGroup));

        groupHandler = parameterHandler->getGroup("CARTOONS").lock(); //now the pointer should be lockable
        REQUIRE(groupHandler);
        REQUIRE(groupHandler->isEmpty());

        groupHandler->setParameter("Donald's nephews", donaldsNephews);
        REQUIRE_FALSE(groupHandler->isEmpty());
        std::cout << "Parameters: " << parameterHandler->toString() << std::endl;
    }

    SECTION("Clear")
    {
        REQUIRE_FALSE(parameterHandler->isEmpty());
        parameterHandler->clear();
        REQUIRE(parameterHandler->isEmpty());
    }

    SECTION("Set from object")
    {
        yarp::os::ResourceFinder rf;
        originalHandler->set(rf);

        yarp::os::Property property;
        property.put("value", 10);
        originalHandler->set(property);
        int expected;
        REQUIRE(parameterHandler->getParameter("value", expected));
        REQUIRE(expected == 10);
    }

    SECTION("Set from file")
    {
        parameterHandler->clear();

        REQUIRE(originalHandler->setFromFile(getConfigPath()));

        {
            int element;
            REQUIRE(parameterHandler->getParameter("answer_to_the_ultimate_question_of_life", //
                                                   element));
            REQUIRE(element == 42);
        }

        {
            double element;
            REQUIRE(parameterHandler->getParameter("pi", element));
            REQUIRE(element == 3.14);
        }

        {
            std::string element;
            REQUIRE(parameterHandler->getParameter("John", element));
            REQUIRE(element == "Smith");
        }

        {
            std::vector<int> element;
            REQUIRE(parameterHandler->getParameter("Fibonacci Numbers", element));
            REQUIRE(element == fibonacciNumbers);
        }

        auto cartoonsGroup = parameterHandler->getGroup("CARTOONS").lock();
        REQUIRE(cartoonsGroup);

        {
            std::vector<std::string> element;
            REQUIRE(cartoonsGroup->getParameter("Donald's nephews", element));
            REQUIRE(element == donaldsNephews);
        }

        {
            std::vector<int> element(fibonacciNumbers.size());
            REQUIRE(cartoonsGroup->getParameter("Fibonacci_Numbers", element));
            REQUIRE(element == fibonacciNumbers);
        }

        {
            std::vector<double> element;
            REQUIRE(cartoonsGroup->getParameter("fingersScaling", element));
            REQUIRE(element == std::vector<double>{1.0, 2.0, 3.5, 3.5, 3.5});
        }

        {
            std::string element;
            REQUIRE(cartoonsGroup->getParameter("John", element));
            REQUIRE(element == "Doe");
        }
    }

    SECTION("Set from RF")
    {
        yarp::os::ResourceFinder &rf = yarp::os::ResourceFinder::getResourceFinderSingleton();
        rf.setDefaultConfigFile("config.ini");

        std::vector<std::string> arguments = {" ", "--from ", getConfigPath()};

        std::vector<char*> argv;
        for (const auto& arg : arguments)
            argv.push_back((char*)arg.data());
        argv.push_back(nullptr);

        rf.configure(argv.size() - 1, argv.data());

        REQUIRE_FALSE(rf.isNull());
        parameterHandler->clear();
        REQUIRE(parameterHandler->isEmpty());
        originalHandler->set(rf);

        {
            int element;
            REQUIRE(parameterHandler->getParameter("answer_to_the_ultimate_question_of_life", element));
            REQUIRE(element == 42);
        }

        {
            double element;
            REQUIRE(parameterHandler->getParameter("pi", element));
            REQUIRE(element == 3.14);
        }

        {
            std::string element;
            REQUIRE(parameterHandler->getParameter("John", element));
            REQUIRE(element == "Smith");
        }

        {
            std::vector<int> element;
            REQUIRE(parameterHandler->getParameter("Fibonacci Numbers", element));
            REQUIRE(element == fibonacciNumbers);
        }

        {
            std::chrono::nanoseconds element;
            REQUIRE(parameterHandler->getParameter("time", element));
            REQUIRE(3 == std::chrono::duration_cast<std::chrono::hours>(element).count());
            REQUIRE(
                13
                == std::chrono::duration_cast<std::chrono::minutes>(element % std::chrono::hours(1))
                       .count());
            REQUIRE(38
                    == std::chrono::duration_cast<std::chrono::seconds>(element
                                                                        % std::chrono::minutes(1))
                           .count());
            REQUIRE(21451
                    == std::chrono::duration_cast<std::chrono::microseconds>(
                           element % std::chrono::seconds(1))
                           .count());
        }

        {
            std::chrono::nanoseconds element;
            REQUIRE(parameterHandler->getParameter("time_as_double", element));
            REQUIRE(element == std::chrono::duration<double>(0.05));
        }

        IParametersHandler::shared_ptr cartoonsGroup = parameterHandler->getGroup("CARTOONS").lock();
        REQUIRE(cartoonsGroup);

        {
            std::vector<std::string> element;
            REQUIRE(cartoonsGroup->getParameter("Donald's nephews", element));
            REQUIRE(element == donaldsNephews);

        }

        {
            std::vector<int> element(fibonacciNumbers.size());
            REQUIRE(cartoonsGroup->getParameter("Fibonacci_Numbers", element));
            REQUIRE(element == fibonacciNumbers);
        }

        {
            std::string element;
            REQUIRE(cartoonsGroup->getParameter("John", element));
            REQUIRE(element == "Doe");
        }

        IParametersHandler::shared_ptr timingsGroup = parameterHandler->getGroup("TIMINGS").lock();
        REQUIRE(timingsGroup);

        {
            std::vector<std::chrono::nanoseconds> element;
            REQUIRE(timingsGroup->getParameter("time", element));

            using namespace std::chrono_literals;

            // these are copied from the ini file
            REQUIRE(element
                    == std::vector<std::chrono::nanoseconds>{3h + 13min + 38s + 21451us,
                                                             5h + 31min + 48s + 189405us});
        }

        {
            std::vector<std::chrono::nanoseconds> element;
            REQUIRE(timingsGroup->getParameter("time_as_double", element));

            using namespace std::chrono_literals;

            // these are copied from the ini file
            REQUIRE(
                element
                == std::vector<std::chrono::nanoseconds>{std::chrono::duration_cast<std::chrono::nanoseconds>(0.03s),
                                                         std::chrono::duration_cast<std::chrono::nanoseconds>(1.8s)});
        }
    }

    SECTION("Clone")
    {
        IParametersHandler::shared_ptr setGroup = std::make_shared<YarpImplementation>();
        REQUIRE(parameterHandler->setGroup("CARTOONS", setGroup));
        auto groupHandler = parameterHandler->getGroup("CARTOONS").lock(); //now the pointer should be lockable
        REQUIRE(groupHandler);
        REQUIRE(groupHandler->isEmpty());

        groupHandler->setParameter("Donald's nephews", donaldsNephews);

        auto newHandler = parameterHandler->clone();

        SECTION("Get integer")
        {
            int element;
            REQUIRE(newHandler->getParameter("answer_to_the_ultimate_question_of_life", element));
            REQUIRE(element == 42);
        }

        SECTION("Get Double")
        {
            double element;
            REQUIRE(newHandler->getParameter("pi", element));
            REQUIRE(element == 3.14);
        }

        SECTION("Get String")
        {
            std::string element;
            REQUIRE(newHandler->getParameter("John", element));
            REQUIRE(element == "Smith");
        }

        SECTION("Get Vector")
        {
            std::vector<int> element;
            REQUIRE(newHandler->getParameter("Fibonacci Numbers", element));
            REQUIRE(element == fibonacciNumbers);
        }

        SECTION("Get group")
        {
            std::vector<std::string> element;
            REQUIRE(newHandler->getGroup("CARTOONS").lock()->getParameter("Donald's nephews",//
                                                                          element));
            REQUIRE(element == donaldsNephews);
        }
    }
}
