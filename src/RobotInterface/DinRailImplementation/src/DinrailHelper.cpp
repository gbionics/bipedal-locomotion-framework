/**
 * @file DinrailHelper.cpp
 * @copyright Generative Bionics S.R.L. This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/RobotInterface/DinrailHelper.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <dinrail/Parameters.h>

#include <utility>

using namespace BipedalLocomotion::RobotInterface;

DinrailDeviceDescriptor::DinrailDeviceDescriptor(const std::string& key,
                                                 std::shared_ptr<dinrail::Device> device)
    : key(key)
    , device(std::move(device))
{
}

bool DinrailDeviceDescriptor::isValid() const
{
    return device != nullptr && device->isValid();
}

DinrailDeviceDescriptor BipedalLocomotion::RobotInterface::constructDinrailRGBDSensorClient(
    std::weak_ptr<const ParametersHandler::IParametersHandler> handler)
{
    constexpr auto logPrefix = "[RobotInterface::constructDinrailRGBDSensorClient]";

    auto ptr = handler.lock();
    if (ptr == nullptr)
    {
        log()->error("{} Invalid parameter handler.", logPrefix);
        return DinrailDeviceDescriptor();
    }

    std::string name;
    std::string localPrefix;
    std::string localDepthPortPostfix;
    std::string localRpcPortPostfix;
    std::string remoteDepthPort;
    std::string remoteRpcPort;
    if (!ptr->getParameter("name", name) || !ptr->getParameter("local_prefix", localPrefix)
        || !ptr->getParameter("local_depth_port_postfix", localDepthPortPostfix)
        || !ptr->getParameter("local_rpc_port_postfix", localRpcPortPostfix)
        || !ptr->getParameter("remote_depth_port", remoteDepthPort)
        || !ptr->getParameter("remote_rpc_port", remoteRpcPort))
    {
        log()->error("{} Unable to get all the mandatory parameters from the configuration.",
                     logPrefix);
        return DinrailDeviceDescriptor();
    }

    dinrail::Parameters options;
    options.put("device", "dr_rgbd_nwc_yarp");
    // dr_rgbd_nwc_yarp is a YARP plugin that implements the dinrail interfaces directly.
    options.put("dinrail_device_type", "yarp");
    options.put("local_depth_port", "/" + localPrefix + localDepthPortPostfix);
    options.put("local_rpc_port", "/" + localPrefix + localRpcPortPostfix);
    options.put("remote_depth_port", remoteDepthPort);
    options.put("remote_rpc_port", remoteRpcPort);
    if (std::string depthCarrier; ptr->getParameter("depth_carrier", depthCarrier))
    {
        options.put("depth_carrier", depthCarrier);
    }

    std::string localImagePortPostfix;
    std::string remoteImagePort;
    const bool hasLocalImagePort
        = ptr->getParameter("local_image_port_postfix", localImagePortPostfix);
    const bool hasRemoteImagePort = ptr->getParameter("remote_image_port", remoteImagePort);
    if (hasLocalImagePort != hasRemoteImagePort)
    {
        log()->error("{} 'local_image_port_postfix' and 'remote_image_port' must be both set, to "
                     "receive the color images, or both unset.",
                     logPrefix);
        return DinrailDeviceDescriptor();
    }
    if (hasLocalImagePort)
    {
        options.put("local_image_port", "/" + localPrefix + localImagePortPostfix);
        options.put("remote_image_port", remoteImagePort);
        if (std::string imageCarrier; ptr->getParameter("image_carrier", imageCarrier))
        {
            options.put("image_carrier", imageCarrier);
        }
    }

    DinrailDeviceDescriptor descriptor(name, std::make_shared<dinrail::Device>());
    if (!descriptor.device->open(options))
    {
        log()->error("{} Unable to open the RGBD sensor client named {}.", logPrefix, name);
        return DinrailDeviceDescriptor();
    }

    return descriptor;
}
