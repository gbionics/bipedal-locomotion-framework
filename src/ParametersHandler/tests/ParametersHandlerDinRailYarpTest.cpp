// SPDX-FileCopyrightText: Generative Bionics S.R.L.
// SPDX-License-Identifier: BSD-3-Clause

#include <catch2/catch_test_macros.hpp>

#include <BipedalLocomotion/ParametersHandler/DinRailImplementation.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h>

#include <dinrail/Parameters.h>
#include <dinrail/YarpPropertyConverter.h>

#include <ConfigFolderPath.h>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <yarp/os/Bottle.h>
#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/os/Time.h>
#include <yarp/os/Value.h>

#include <yarp/robotinterface/Types.h>
#include <yarp/robotinterface/XMLReader.h>

namespace
{
yarp::os::Property toProperty(const yarp::robotinterface::Device& dev)
{
    using namespace yarp::robotinterface;

    ParamList params = mergeDuplicateGroups(dev.params());

    yarp::os::Property prop;
    prop.put("device", dev.type());
    prop.put("id", dev.name());

    for (const auto& param : params)
    {
        prop.fromString("(" + param.name() + " " + param.value() + ")", false);
    }

    return prop;
}

yarp::os::Property loadPropertyFromFile(const std::filesystem::path& file)
{
    yarp::os::Property yarpProperty;

    if (file.extension() == ".xml")
    {
        // Avoid Clock is not initialized error by initializing the YARP network
        yarp::os::Time::useSystemClock();

        yarp::robotinterface::XMLReader reader;
        reader.setEnableDeprecated(true);
        auto result = reader.getRobotFromFile(file.string());

        // Some legacy fixtures are a raw <device> XML block instead of a
        // robotinterface <robot> root. In that case, wrap the device block.
        if (!result.parsingIsSuccessful)
        {
            std::ifstream input(file);
            REQUIRE(input.is_open());

            std::stringstream buffer;
            buffer << input.rdbuf();
            const std::string xml = buffer.str();

            const std::size_t start = xml.find("<device");
            const std::size_t endTag = xml.rfind("</device>");
            REQUIRE(start != std::string::npos);
            REQUIRE(endTag != std::string::npos);

            const std::size_t end = endTag + std::string("</device>").size();
            const std::string deviceBlock = xml.substr(start, end - start);

            const std::string wrapped = "<robot name=\"wrapped\" build=\"0\" portprefix=\"/\">\n"
                                        + deviceBlock + "\n</robot>\n";
            result = reader.getRobotFromString(wrapped);
            REQUIRE(result.parsingIsSuccessful);
        }

        const auto& devices = result.robot.devices();
        REQUIRE_FALSE(devices.empty());

        const auto& dev = devices.front();
        yarpProperty = toProperty(dev);
        return yarpProperty;
    }

    REQUIRE(yarpProperty.fromConfigFile(file.string()));
    return yarpProperty;
}

bool isGroupEntry(const yarp::os::Bottle& entry)
{
    if (entry.size() < 2)
    {
        return false;
    }

    for (int i = 1; i < entry.size(); ++i)
    {
        if (!entry.get(i).isList())
        {
            return false;
        }
        yarp::os::Bottle* child = entry.get(i).asList();
        if (child == nullptr || child->size() < 2 || !child->get(0).isString())
        {
            return false;
        }
    }

    return true;
}

template <typename T>
void requireSameParameter(const BipedalLocomotion::ParametersHandler::IParametersHandler& lhs,
                          const BipedalLocomotion::ParametersHandler::IParametersHandler& rhs,
                          const std::string& key,
                          const std::string& path)
{
    T lhsValue{};
    T rhsValue{};

    INFO("Parameter path: " << path);
    const bool lhsOk = lhs.getParameter(key, lhsValue);
    const bool rhsOk = rhs.getParameter(key, rhsValue);

    REQUIRE(lhsOk);
    REQUIRE(rhsOk);
    REQUIRE(lhsValue == rhsValue);
}

void compareEntry(const yarp::os::Bottle& entry,
                  const BipedalLocomotion::ParametersHandler::IParametersHandler& yarpHandler,
                  const BipedalLocomotion::ParametersHandler::IParametersHandler& dinRailHandler,
                  const std::string& parentPath = "")
{
    REQUIRE(entry.size() >= 2);
    REQUIRE(entry.get(0).isString());

    const std::string key = entry.get(0).asString();
    const std::string keyPath = parentPath.empty() ? key : parentPath + "/" + key;
    INFO("Parameter path: " << keyPath);

    if (isGroupEntry(entry))
    {
        const auto yarpGroup = yarpHandler.getGroup(key).lock();
        const auto dinRailGroup = dinRailHandler.getGroup(key).lock();

        REQUIRE(static_cast<bool>(yarpGroup));
        REQUIRE(static_cast<bool>(dinRailGroup));

        for (int i = 1; i < entry.size(); ++i)
        {
            yarp::os::Bottle* child = entry.get(i).asList();
            if (child == nullptr)
            {
                FAIL("Unexpected null sub-bottle in group entry at: " + keyPath);
            }
            compareEntry(*child, *yarpGroup, *dinRailGroup, keyPath);
        }
    } else
    {
        const yarp::os::Value& value = entry.get(1);
        if (value.isBool())
        {
            requireSameParameter<bool>(yarpHandler, dinRailHandler, key, keyPath);
        } else if (value.isInt32() || value.isInt64())
        {
            requireSameParameter<int>(yarpHandler, dinRailHandler, key, keyPath);
        } else if (value.isFloat64())
        {
            requireSameParameter<double>(yarpHandler, dinRailHandler, key, keyPath);
        } else if (value.isString())
        {
            requireSameParameter<std::string>(yarpHandler, dinRailHandler, key, keyPath);
        } else if (value.isList())
        {
            yarp::os::Bottle* list = value.asList();
            REQUIRE(list != nullptr);
            REQUIRE(list->size() > 0);

            bool allInt = true;
            bool allFloat = true;
            bool allString = true;
            bool allBool = true;
            for (int i = 0; i < list->size(); ++i)
            {
                const yarp::os::Value& element = list->get(i);
                const bool isInt = element.isInt32() || element.isInt64();
                allInt = allInt && isInt;
                allFloat = allFloat && element.isFloat64();
                allString = allString && element.isString();
                allBool = allBool && element.isBool();
            }

            if (allBool)
            {
                requireSameParameter<std::vector<bool>>(yarpHandler, dinRailHandler, key, keyPath);
            } else if (allInt)
            {
                requireSameParameter<std::vector<int>>(yarpHandler, dinRailHandler, key, keyPath);
            } else if (allFloat)
            {
                requireSameParameter<std::vector<double>>(yarpHandler, dinRailHandler, key, keyPath);
            } else if (allString)
            {
                requireSameParameter<std::vector<std::string>>(yarpHandler,
                                                               dinRailHandler,
                                                               key,
                                                               keyPath);
            } else
            {
                FAIL("Unsupported list element type mix at key: " + keyPath);
            }
        } else
        {
            FAIL("Unsupported YARP value type at key: " + keyPath);
        }
    }
}

void comparePropertyHandlers(
    const yarp::os::Property& yarpProp,
    const BipedalLocomotion::ParametersHandler::IParametersHandler& yarpHandler,
    const BipedalLocomotion::ParametersHandler::IParametersHandler& dinRailHandler)
{
    yarp::os::Bottle root;
    root.fromString(yarpProp.toString());

    for (int i = 0; i < root.size(); ++i)
    {
        if (!root.get(i).isList())
        {
            FAIL("Unexpected non-list element at top-level index " + std::to_string(i));
        }

        yarp::os::Bottle* entry = root.get(i).asList();
        REQUIRE(entry != nullptr);
        compareEntry(*entry, yarpHandler, dinRailHandler);
    }
}

} // namespace

TEST_CASE("BLF YarpImplementation and DinRailImplementation are compatible on sample files",
          "[YarpPropertyConverter][BLFCompatibility]")
{
    yarp::os::Network yarpNetwork;
    yarp::os::Time::useSystemClock();

    const std::vector<std::filesystem::path> files = {
        getConfigPath(),
        getOpenXRJoypadConfigPath(),
        getRobotDynamicsEstimatorConfigPath(),
        getQpIKBLFConfigPath(),
    };

    for (const auto& file : files)
    {
        INFO("Checking file: " << file.string());
        yarp::os::Property yarpProperty = loadPropertyFromFile(file);

        BipedalLocomotion::ParametersHandler::YarpImplementation yarpHandler(yarpProperty);

        dinrail::Parameters params
            = dinrail::YarpPropertyConverter::toDinrailParameters(yarpProperty);
        BipedalLocomotion::ParametersHandler::DinRailImplementation dinRailHandler(params);

        comparePropertyHandlers(yarpProperty, yarpHandler, dinRailHandler);
    }
}
