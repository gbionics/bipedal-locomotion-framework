/**
 * @file DinrailHelper.h
 * @copyright Generative Bionics S.R.L. This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_ROBOT_INTERFACE_DINRAIL_HELPER_H
#define BIPEDAL_LOCOMOTION_ROBOT_INTERFACE_DINRAIL_HELPER_H

#include <memory>
#include <string>

#include <dinrail/Device.h>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>

namespace BipedalLocomotion
{
namespace RobotInterface
{

/**
 * DinrailDeviceDescriptor describes an open dinrail device, identified by a key. It is the dinrail
 * counterpart of PolyDriverDescriptor.
 */
struct DinrailDeviceDescriptor
{
    std::string key; /**< Name of the device. */
    std::shared_ptr<dinrail::Device> device; /**< The device. */

    DinrailDeviceDescriptor() = default;
    DinrailDeviceDescriptor(const std::string& key, std::shared_ptr<dinrail::Device> device);

    /**
     * Check if the descriptor holds an open device.
     * @return True if the device is not null and it is open.
     */
    bool isValid() const;
};

/**
 * Helper function that can be used to build a dinrail RGBD sensor client (dr_rgbd_nwc_yarp of
 * dinrail), the dinrail counterpart of constructRDGBSensorClient(). The client receives the images
 * with port callbacks and its getters never block. It can be connected to the dr_rgbd_nws_yarp
 * server of dinrail and to the YARP rgbdSensor_nws_yarp.
 * @param handler pointer to a parameter handler interface.
 * @note The following parameters are required by the function
 * |       Parameter Name       |   Type   |                         Description                              | Mandatory |
 * |:--------------------------:|:--------:|:----------------------------------------------------------------:|:---------:|
 * |          `name`            | `string` | Name of the device, used as key of the descriptor.               |    Yes    |
 * |       `local_prefix`       | `string` | Prefix of the local ports, without the leading `/`.              |    Yes    |
 * | `local_depth_port_postfix` | `string` | Postfix of the local depth port, it must start with `/`.         |    Yes    |
 * |  `local_rpc_port_postfix`  | `string` | Postfix of the local RPC port, it must start with `/`.           |    Yes    |
 * |    `remote_depth_port`     | `string` | Depth port opened by the server.                                 |    Yes    |
 * |     `remote_rpc_port`      | `string` | RPC port opened by the server.                                   |    Yes    |
 * |      `depth_carrier`       | `string` | Carrier of the depth connection (default `tcp`).                 |    No     |
 * | `local_image_port_postfix` | `string` | Postfix of the local color port, it must start with `/`.         |    No     |
 * |    `remote_image_port`     | `string` | Color port opened by the server. The color images are received only if this parameter and `local_image_port_postfix` are set. | No |
 * |      `image_carrier`       | `string` | Carrier of the color connection (default `tcp`).                 |    No     |
 * @return A DinrailDeviceDescriptor. In case of error an invalid DinrailDeviceDescriptor is
 * returned.
 */
DinrailDeviceDescriptor
constructDinrailRGBDSensorClient(std::weak_ptr<const ParametersHandler::IParametersHandler> handler);

} // namespace RobotInterface
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_ROBOT_INTERFACE_DINRAIL_HELPER_H
