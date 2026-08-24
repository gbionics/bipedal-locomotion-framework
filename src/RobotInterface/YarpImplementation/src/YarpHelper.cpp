/**
 * @file YarpHelper.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/RobotInterface/YarpHelper.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <yarp/dev/IMultipleWrapper.h>
#include <yarp/dev/PolyDriverList.h>

using namespace BipedalLocomotion::RobotInterface;

PolyDriverDescriptor::PolyDriverDescriptor(const std::string& key,
                                           std::shared_ptr<yarp::dev::PolyDriver> poly)
    : key(key)
    , poly(poly)
{
}

PolyDriverDescriptor::PolyDriverDescriptor() = default;

bool PolyDriverDescriptor::isValid() const
{
    return ((!key.empty()) && (poly != nullptr));
}

PolyDriverDescriptor BipedalLocomotion::RobotInterface::constructRemoteControlBoardRemapper(
    std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler)
{
    constexpr auto errorPrefix = "[RobotInterface::constructRemoteControlBoardRemapper]";

    auto ptr = handler.lock();

    if (ptr == nullptr)
    {
        log()->error("{} Invalid parameter handler.", errorPrefix);
        return PolyDriverDescriptor();
    }

    bool ok = true;

    std::vector<std::string> jointsList;
    ok = ok && ptr->getParameter("joints_list", jointsList);

    std::vector<std::string> controlBoards;
    ok = ok && ptr->getParameter("remote_control_boards", controlBoards);

    std::string robotName;
    ok = ok && ptr->getParameter("robot_name", robotName);

    std::string localPrefix;
    ok = ok && ptr->getParameter("local_prefix", localPrefix);

    if (!ok)
    {
        log()->error("{} Unable to get all the parameters from configuration file.", errorPrefix);
        return PolyDriverDescriptor();
    }

    // Get optional device parameter, default to "remotecontrolboardremapper"
    std::string deviceName = "remotecontrolboardremapper";
    std::string deviceNameFromParam;
    ok = ptr->getParameter("device", deviceNameFromParam);
    if (ok)
    {
        deviceName = deviceNameFromParam;
    }

    // open the remotecontrolboardremepper YARP device
    yarp::os::Property options;
    options.put("device", deviceName);

    options.addGroup("axesNames");
    yarp::os::Bottle& bottle = options.findGroup("axesNames").addList();
    for (const auto& joint : jointsList)
    {
        bottle.addString(joint);
    }

    yarp::os::Bottle remoteControlBoards;

    yarp::os::Bottle& remoteControlBoardsList = remoteControlBoards.addList();
    for (const auto& controlBoard : controlBoards)
    {
        remoteControlBoardsList.addString("/" + robotName + "/" + controlBoard);
    }

    options.put("remoteControlBoards", remoteControlBoards.get(0));
    options.put("localPortPrefix", "/" + localPrefix + "/remoteControlBoard");
    yarp::os::Property& remoteControlBoardsOpts = options.addGroup("REMOTE_CONTROLBOARD_OPTIONS");
    remoteControlBoardsOpts.put("writeStrict", "on");

    std::string carrier;
    if (ptr->getParameter("carrier", carrier))
    {
        remoteControlBoardsOpts.put("carrier", carrier);
    }

    PolyDriverDescriptor device("remoteControlBoards", std::make_shared<yarp::dev::PolyDriver>());

    if (!device.poly->open(options) && !device.poly->isValid())
    {
        log()->error("{} Could not open polydriver object.", errorPrefix);
        return PolyDriverDescriptor();
    }

    return device;
}

PolyDriverDescriptor BipedalLocomotion::RobotInterface::constructGenericSensorClient(
    std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler)
{
    constexpr auto errorPrefix = "[RobotInterface::constructGenericSensorClient]";

    auto ptr = handler.lock();

    if (ptr == nullptr)
    {
        log()->error("{} Invalid parameter handler.", errorPrefix);
        return PolyDriverDescriptor();
    }

    bool ok = true;

    std::string description;
    ok = ok && ptr->getParameter("description", description);

    std::string remotePortName;
    ok = ok && ptr->getParameter("remote_port_name", remotePortName);

    std::string localPrefix;
    ok = ok && ptr->getParameter("local_prefix", localPrefix);

    std::string localPortNamePostfix;
    ok = ok && ptr->getParameter("local_port_name_postfix", localPortNamePostfix);

    if (!ok)
    {
        log()->error("{} Unable to get all the parameters from configuration file.", errorPrefix);
        return PolyDriverDescriptor();
    }

    // open the YARP device
    yarp::os::Property options;
    options.put("device", "genericSensorClient");
    options.put("remote", remotePortName);
    options.put("local", "/" + localPrefix + localPortNamePostfix);

    PolyDriverDescriptor device(description, std::make_shared<yarp::dev::PolyDriver>());

    if (!device.poly->open(options) && !device.poly->isValid())
    {
        log()->error("{} Could not open polydriver object.", errorPrefix);
        return PolyDriverDescriptor();
    }

    return device;
}

PolyDriverDescriptor BipedalLocomotion::RobotInterface::constructMultipleAnalogSensorsClient(
    std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler)
{
    constexpr auto errorPrefix = "[RobotInterface::constructMultipleAnalogSensorsClient]";

    auto ptr = handler.lock();

    if (ptr == nullptr)
    {
        log()->error("{} Invalid parameter handler.", errorPrefix);
        return PolyDriverDescriptor();
    }

    bool ok = true;

    std::string description;
    ok = ok && ptr->getParameter("description", description);

    std::string remotePortName;
    ok = ok && ptr->getParameter("remote_port_name", remotePortName);

    std::string localPrefix;
    ok = ok && ptr->getParameter("local_prefix", localPrefix);

    std::string localPortNamePostfix;
    ok = ok && ptr->getParameter("local_port_name_postfix", localPortNamePostfix);

    if (!ok)
    {
        log()->error("{} Unable to get all the parameters from configuration file.", errorPrefix);
        return PolyDriverDescriptor();
    }

    // open the YARP device
    yarp::os::Property options;
    options.put("device", "multipleanalogsensorsclient");
    options.put("remote", remotePortName);
    options.put("local", "/" + localPrefix + localPortNamePostfix);

    double timeout;
    if (ptr->getParameter("timeout", timeout))
    {
        options.put("timeout", timeout);
    }

    bool externalConnection;
    if (ptr->getParameter("external_connection", externalConnection))
    {
        options.put("externalConnection", externalConnection);
    }

    std::string carrier;
    if (ptr->getParameter("carrier", carrier))
    {
        options.put("carrier", carrier);
    }

    PolyDriverDescriptor device(description, std::make_shared<yarp::dev::PolyDriver>());

    if (!device.poly->open(options) && !device.poly->isValid())
    {
        log()->error("{} Could not open polydriver object.", errorPrefix);
        return PolyDriverDescriptor();
    }

    return device;
}

PolyDriverDescriptor BipedalLocomotion::RobotInterface::constructMultipleAnalogSensorsRemapper(
    std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler)
{
    constexpr auto errorPrefix = "[RobotInterface::constructMultipleAnalogsensorsRemapper]";

    auto ptr = handler.lock();

    if (ptr == nullptr)
    {
        log()->error("{} Invalid parameter handler.", errorPrefix);
        return PolyDriverDescriptor();
    }

    std::string description;
    if (!ptr->getParameter("description", description))
    {
        log()->error("{} Unable to find the parameter 'description'.", errorPrefix);
        return PolyDriverDescriptor();
    }

    // open the multipleanalogsensorsremapper YARP device
    yarp::os::Property options;
    options.put("device", "multipleanalogsensorsremapper");

    auto addOption = [&](const std::string& parameterName, const std::string& optionName) -> void {
        std::vector<std::string> temp;
        if (ptr->getParameter(parameterName, temp))
        {
            yarp::os::Bottle sensorsNames;
            yarp::os::Bottle& sensorsList = sensorsNames.addList();
            for (const auto& name : temp)
            {
                sensorsList.addString(name);
            }
            options.put(optionName, sensorsNames.get(0));
        }
    };

    addOption("three_axis_gyroscopes_names", "ThreeAxisGyroscopesNames");
    addOption("three_axis_linear_accelerometers_names", "ThreeAxisLinearAccelerometersNames");
    addOption("three_axis_magnetometers_names", "ThreeAxisMagnetometersNames");
    addOption("orientation_sensors_names", "OrientationSensorsNames");
    addOption("six_axis_force_torque_sensors_names", "SixAxisForceTorqueSensorsNames");
    addOption("temperature_sensors_names", "TemperatureSensorsNames");

    PolyDriverDescriptor device(description, std::make_shared<yarp::dev::PolyDriver>());

    if (!device.poly->open(options) && !device.poly->isValid())
    {
        log()->error("{} Could not open polydriver object.", errorPrefix);
        return PolyDriverDescriptor();
    }

    return device;
}

PolyDriverDescriptor BipedalLocomotion::RobotInterface::constructMultipleAnalogSensorsRemapper(
    std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler,
    const std::vector<PolyDriverDescriptor>& polydriverList)
{
    // create the yarp::dev::PolyDriverList
    yarp::dev::PolyDriverList list;
    for (const auto& driver : polydriverList)
    {
        list.push(driver.poly.get(), driver.key.c_str());
    }

    return constructMultipleAnalogSensorsRemapper(handler, list);
}

PolyDriverDescriptor BipedalLocomotion::RobotInterface::constructMultipleAnalogSensorsRemapper(
    std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler,
    const yarp::dev::PolyDriverList& polydriverList)
{
    constexpr auto errorPrefix = "[RobotInterface::constructMultipleAnalogsensorsRemapper]";

    auto device = constructMultipleAnalogSensorsRemapper(handler);

    if (!device.isValid())
    {
        return device;
    }

    // attach the the interface
    yarp::dev::IMultipleWrapper* multipleWrapper = nullptr;
    if (!device.poly->view(multipleWrapper) || multipleWrapper == nullptr)
    {
        log()->error("{} Could not view the IMultipleWrapper interface.", errorPrefix);
        return device;
    }

    if (!multipleWrapper->attachAll(polydriverList))
    {
        log()->error("{} Could not attach the polydriver list.", errorPrefix);
        return PolyDriverDescriptor();
    }

    return device;
}

PolyDriverDescriptor BipedalLocomotion::RobotInterface::constructRDGBSensorClient(
    std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler)
{
    constexpr auto errorPrefix = "[RobotInterface::constructRDGBSensorClient]";

    auto ptr = handler.lock();

    if (ptr == nullptr)
    {
        log()->error("{} Invalid parameter handler.", errorPrefix);
        return PolyDriverDescriptor();
    }

    bool ok = true;

    std::string name;
    ok = ok && ptr->getParameter("name", name);

    std::string localPrefix;
    ok = ok && ptr->getParameter("local_prefix", localPrefix);

    std::string localImagePortNamePostfix;
    ok = ok && ptr->getParameter("local_image_port_postfix", localImagePortNamePostfix);

    std::string localDepthPortNamePostfix;
    ok = ok && ptr->getParameter("local_depth_port_postfix", localDepthPortNamePostfix);

    std::string localRpcPortNamePostfix;
    ok = ok && ptr->getParameter("local_rpc_port_postfix", localRpcPortNamePostfix);

    std::string remoteImagePortName;
    ok = ok && ptr->getParameter("remote_image_port", remoteImagePortName);

    std::string remoteDepthPortName;
    ok = ok && ptr->getParameter("remote_depth_port", remoteDepthPortName);

    std::string remoteRpcPortName;
    ok = ok && ptr->getParameter("remote_rpc_port", remoteRpcPortName);

    std::string imageCarrier;
    ok = ok && ptr->getParameter("image_carrier", imageCarrier);

    std::string depthCarrier;
    ok = ok && ptr->getParameter("depth_carrier", depthCarrier);

    if (!ok)
    {
        log()->error("{} Unable to get all the parameters from configuration file.", errorPrefix);
        return PolyDriverDescriptor();
    }

    yarp::os::Property options;
    options.put("device", "RGBDSensorClient");
    options.put("localImagePort", "/" + localPrefix + localImagePortNamePostfix);
    options.put("localDepthPort", "/" + localPrefix + localDepthPortNamePostfix);
    options.put("localRpcPort", "/" + localPrefix + localRpcPortNamePostfix);
    options.put("ImageCarrier", imageCarrier);
    options.put("DepthCarrier", depthCarrier);
    options.put("remoteImagePort", remoteImagePortName);
    options.put("remoteDepthPort", remoteDepthPortName);
    options.put("remoteRpcPort", remoteRpcPortName);

    PolyDriverDescriptor device(name, std::make_shared<yarp::dev::PolyDriver>());

    if (!device.poly->open(options) && !device.poly->isValid())
    {
        log()->error("{} Could not open polydriver object.", errorPrefix);
        return PolyDriverDescriptor();
    }

    return device;
}

PolyDriverDescriptor BipedalLocomotion::RobotInterface::constructBatteryClient(
    std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler)
{
    constexpr auto errorPrefix = "[RobotInterface::constructBatteryClient]";

    auto ptr = handler.lock();

    if (ptr == nullptr)
    {
        log()->error("{} Invalid parameter handler.", errorPrefix);
        return PolyDriverDescriptor();
    }

    bool ok = true;

    std::string description;
    ok = ok && ptr->getParameter("description", description);

    std::string remotePortName;
    ok = ok && ptr->getParameter("remote_port_name", remotePortName);

    std::string localPrefix;
    ok = ok && ptr->getParameter("local_prefix", localPrefix);

    std::string localPortNamePostfix;
    ok = ok && ptr->getParameter("local_port_name_postfix", localPortNamePostfix);

    if (!ok)
    {
        log()->error("{} Unable to get all the parameters from configuration file.", errorPrefix);
        return PolyDriverDescriptor();
    }

    yarp::os::Property options;
    options.put("device", "battery_nwc_yarp");
    options.put("remote", remotePortName);
    options.put("local", "/" + localPrefix + localPortNamePostfix);

    std::string carrier;
    if (ptr->getParameter("carrier", carrier))
    {
        options.put("carrier", carrier);
    }

    PolyDriverDescriptor device(description, std::make_shared<yarp::dev::PolyDriver>());

    if (!device.poly->open(options) || !device.poly->isValid())
    {
        log()->error("{} Could not open polydriver object.", errorPrefix);
        return PolyDriverDescriptor();
    }

    return device;
}
