/**
 * @file YarpHelper.h
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_ROBOT_INTERFACE_YARP_HELPER_H
#define BIPEDAL_LOCOMOTION_ROBOT_INTERFACE_YARP_HELPER_H

// std
#include <memory>
#include <string>

// YARP
#include <yarp/dev/PolyDriver.h>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <yarp/dev/PolyDriverList.h>

namespace BipedalLocomotion
{
namespace RobotInterface
{

/**
 * PolyDriverDescriptor describes a PolyDriver.
 * @note Please be careful with the namespaces then to avoid clashes with YARP
 * [PolyDriverDescriptor](https://www.yarp.it/git-master/classyarp_1_1dev_1_1PolyDriverDescriptor.html).
 */
struct PolyDriverDescriptor
{
    std::string key{""}; /**< key associated to the polydriver */
    std::shared_ptr<yarp::dev::PolyDriver> poly{nullptr}; /**< Pointer associated to the polydriver. */

    /**
     * Constructor.
     */
    PolyDriverDescriptor(const std::string& key, std::shared_ptr<yarp::dev::PolyDriver> poly);

    /**
     * Constructor.
     */
    PolyDriverDescriptor();

    /**
     * Check if the poly driver descriptor is valid.
     * @return True if the polydriver is valid, false otherwise.
     */
    bool isValid() const;
};

/**
 * Helper function that can be used to build a RemoteControlBoardRemapper device.
 * @param handler pointer to a parameter handler interface.
 * @note the following parameters are required by the function
 * |      Parameter Name     |       Type       |                         Description                        | Mandatory |
 * |:-----------------------:|:----------------:|:----------------------------------------------------------:|:---------:|
 * |      `joints_list`      | `vector<string>` |                List of the controlled joints               |    Yes    |
 * | `remote_control_boards` | `vector<string>` | List of the remote control boards associated to the joints |    Yes    |
 * |       `robot_name`      |     `string`     |                      Name of the robot                     |    Yes    |
 * |      `local_prefix`     |     `string`     |    Prefix of the local port (e.g. the application name)    |    Yes    |
 * |       `device`         |     `string`     | Name of the device to open. (Default value `remotecontrolboardremapper`) |     No    |
 * @return A PolyDriverDescriptor. If one of the parameters is missing an invalid PolyDriverDescriptor is returned.
 */
PolyDriverDescriptor constructRemoteControlBoardRemapper(
    std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler);

/**
 * Helper function that can be used to build a GenericSensorClient device.
 * @param handler pointer to a parameter handler interface.
 * @note the following parameters are required by the function
 * |       Parameter Name      |   Type   |                                          Description                                         | Mandatory |
 * |:-------------------------:|:--------:|:--------------------------------------------------------------------------------------------:|:---------:|
 * |       `description`       | `string` |          Description of the genericSensorClient. It is the device name                       |    Yes    |
 * |     `remote_port_name`    | `string` |                           Name of the port associate to the remote                           |    Yes    |
 * |       `local_prefix`      | `string` |                     Prefix of the local port (e.g. the application name)                     |    Yes    |
 * | `local_port_name_postfix` | `string` | Postfix of the local port. The local port name is `/<local_prefix><local_port_name_postfix>` |    Yes    |
 * @note The genericSensorClient device is implement in [whole-body-estimators](https://github.com/robotology/whole-body-estimators/tree/cc8ffb83375a2d410b225f4fa67aee4a29074b42/devices/genericSensorClient).
 * @return A PolyDriverDescriptor. If one of the parameters is missing an invalid PolyDriverDescriptor is returned.
 */
PolyDriverDescriptor constructGenericSensorClient(
    std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler);

/**
 * Helper function that can be used to build a `MultipleAnalogSensorsClient` device.
 * @param handler pointer to a parameter handler interface.
 * @note The following parameters are taken into consideration
 * |       Parameter Name      |   Type   |                                                                                              Description                                                                             | Mandatory |
 * |:-------------------------:|:--------:|:------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------:|:---------:|
 * |       `description`       | `string` |                                                               Description of the multiple analog sensor client. It is the device name                                                |    Yes    |
 * |     `remote_port_name`    | `string` |                                                                          Name of the port associate to the remote                                                                    |    Yes    |
 * |       `local_prefix`      | `string` |                                                                  Prefix of the local port (e.g. the application name)                                                                |    Yes    |
 * | `local_port_name_postfix` | `string` |                                               Postfix of the local port. The local port name is `/<local_prefix><local_port_name_postfix>`                                           |    Yes    |
 * |           `timeout`       | `double` |                                      Timeout in seconds after which the device reports an error if no measurement was received. (Default value 0.01)                                 |     No    |
 * |   `external_connection`   |  `bool`  | If set to true, the connection to the rpc port of the MAS server is skipped and it is possible to connect to the data source externally after being opened. (Default value `false`)  |     No    |
 * |          `carrier`        | `string` |                                                   The carrier used for the connection with the server. (Default value `tcp`)                                                         |     No    |
 * @note The `MultipleAnalogSensorsClient` device is implement in [yarp](https://www.yarp.it/git-master/classMultipleAnalogSensorsClient.html).
 * @return A `PolyDriverDescriptor`. In case of error an invalid `PolyDriverDescriptor` is returned.
 */
PolyDriverDescriptor constructMultipleAnalogSensorsClient(
    std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler);

/**
 * Helper function that can be used to build a `MultipleAnalogSensorsRemapper` device.
 * @param handler pointer to a parameter handler interface.
 * @note The following parameters are taken into consideration
 * |                Parameter Name           |       Type       |                                          Description                                    | Mandatory |
 * |:---------------------------------------:|:----------------:|:---------------------------------------------------------------------------------------:|:---------:|
 * |       `three_axis_gyroscopes_names`     | `vector<string>` |     Vector containing the names of the gyroscopes (Default empty vector)                |     No    |
 * | `three_axis_linear_accelerometers_names`| `vector<string>` |  Vector containing the names of the accelerometers (Default empty vector)               |     No    |
 * |    `three_axis_magnetometers_names`     | `vector<string>` |    Vector containing the names of the magnetometers (Default empty vector)              |     No    |
 * |       `orientation_sensors_names`       | `vector<string>` |    Vector containing the names of the orientation sensors (Default empty vector)        |     No    |
 * | `six_axis_force_torque_sensors_names`   | `vector<string>` |    Vector containing the names of the FT sensors (Default empty vector)                 |     No    |
 * |      `temperature_sensors_names`        | `vector<string>` |    Vector containing the names of the temperature sensors (Default empty vector)        |     No    |
 * @note The `MultipleAnalogSensorsRemapper` device is implement in [yarp](https://www.yarp.it/git-master/classMultipleAnalogSensorsRemapper.html).
 * @return A PolyDriverDescriptor. In case of error an invalid `PolyDriverDescriptor` is returned.
 */
PolyDriverDescriptor constructMultipleAnalogSensorsRemapper(
    std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler);

/**
 * Helper function that can be used to build a `MultipleAnalogSensorsRemapper` device.
 * @param handler pointer to a parameter handler interface.
 * @param polydriverList a list of the polydriver that will be attached to the multiple analog sensor remapper
 * @note The following parameters are taken into consideration
 * |                Parameter Name           |       Type       |                                          Description                                    | Mandatory |
 * |:---------------------------------------:|:----------------:|:---------------------------------------------------------------------------------------:|:---------:|
 * |       `three_axis_gyroscopes_names`     | `vector<string>` |     Vector containing the names of the gyroscopes (Default empty vector)                |     No    |
 * | `three_axis_linear_accelerometers_names`| `vector<string>` |  Vector containing the names of the accelerometers (Default empty vector)               |     No    |
 * |    `three_axis_magnetometers_names`     | `vector<string>` |    Vector containing the names of the magnetometers (Default empty vector)              |     No    |
 * |       `orientation_sensors_names`       | `vector<string>` |    Vector containing the names of the orientation sensors (Default empty vector)        |     No    |
 * | `six_axis_force_torque_sensors_names`   | `vector<string>` |    Vector containing the names of the FT sensors (Default empty vector)                 |     No    |
 * |      `temperature_sensors_names`        | `vector<string>` |    Vector containing the names of the temperature sensors (Default empty vector)        |     No    |
 * @note The `MultipleAnalogSensorsRemapper` device is implement in [yarp](https://www.yarp.it/git-master/classMultipleAnalogSensorsRemapper.html).
 * @return A PolyDriverDescriptor. In case of error an invalid `PolyDriverDescriptor` is returned.
 */
PolyDriverDescriptor constructMultipleAnalogSensorsRemapper(
    std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler,
    const std::vector<PolyDriverDescriptor>& polydriverList);

/**
 * Helper function that can be used to build a `MultipleAnalogSensorsRemapper` device.
 * @param handler pointer to a parameter handler interface.
 * @param polydriverList a list of the polydriver that will be attached to the multiple analog sensor remapper.
 * @note The following parameters are taken into consideration
 * |                Parameter Name           |       Type       |                                          Description                                    | Mandatory |
 * |:---------------------------------------:|:----------------:|:---------------------------------------------------------------------------------------:|:---------:|
 * |       `three_axis_gyroscopes_names`     | `vector<string>` |     Vector containing the names of the gyroscopes (Default empty vector)                |     No    |
 * | `three_axis_linear_accelerometers_names`| `vector<string>` |  Vector containing the names of the accelerometers (Default empty vector)               |     No    |
 * |    `three_axis_magnetometers_names`     | `vector<string>` |    Vector containing the names of the magnetometers (Default empty vector)              |     No    |
 * |       `orientation_sensors_names`       | `vector<string>` |    Vector containing the names of the orientation sensors (Default empty vector)        |     No    |
 * | `six_axis_force_torque_sensors_names`   | `vector<string>` |    Vector containing the names of the FT sensors (Default empty vector)                 |     No    |
 * |      `temperature_sensors_names`        | `vector<string>` |    Vector containing the names of the temperature sensors (Default empty vector)        |     No    |
 * @note The `MultipleAnalogSensorsRemapper` device is implement in [yarp](https://www.yarp.it/git-master/classMultipleAnalogSensorsRemapper.html).
 * @return A PolyDriverDescriptor. In case of error an invalid `PolyDriverDescriptor` is returned.
 */
PolyDriverDescriptor constructMultipleAnalogSensorsRemapper(
    std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler,
    const yarp::dev::PolyDriverList& polydriverList);

/**
 * Helper function that can be used to build a `RGBDSensorClient` device.
 * @param handler pointer to a parameter handler interface.
 * @note The following parameters are taken into consideration
 * |         Parameter Name       |   Type   |                                                  Description                                         | Mandatory |
 * |:----------------------------:|:--------:|:----------------------------------------------------------------------------------------------------:|:---------:|
 * |            `name`            | `string` |                                           Name of the camera client                                  |    Yes    |
 * |       `local_prefix`         | `string` |                         Prefix of the local port (e.g. the application name)                         |    Yes    |
 * |  `local_image_port_postfix`  | `string` | Postfix of the local image port. The local port name is `/<local_prefix><local_image_port_postfix>`  |    Yes    |
 * |  `local_depth_port_postfix`  | `string` | Postfix of the local depth port. The local port name is `/<local_prefix><local_depth_port_postfix>`  |    Yes    |
 * |   `local_rpc_port_postfix`   | `string` |    Postfix of the local rpc port. The local port name is `/<local_prefix><local_rpc_port_postfix>`   |    Yes    |
 * |      `remote_image_port`     | `string` |                                  Name of the port associate to the RGB image.                        |    Yes    |
 * |      `remote_depth_port`     | `string` |                                 Name of the port associate to the depth image.                       |    Yes    |
 * |      `remote_rpc_port`       | `string` |                                  Name of the port associate to the rpc port.                         |    Yes    |
 * |       `image_carrier`        | `string` |                                          Carrier for the RGB image.                                  |    Yes    |
 * |       `depth_carrier`        | `string` |                                         Carrier for the depth image.                                 |    Yes    |
 * @note The `RGBDSensorClient` device is implement in [yarp](https://yarp.it/git-master/classRGBDSensorClient.html).
 * @return A PolyDriverDescriptor. In case of error an invalid `PolyDriverDescriptor` is returned.
 */
PolyDriverDescriptor constructRDGBSensorClient(
    std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler);

/**
 * Helper function that can be used to build a `battery_nwc_yarp` device.
 * @param handler pointer to a parameter handler interface.
 * @note The following parameters are taken into consideration
 * |       Parameter Name      |   Type   |                                          Description                                         | Mandatory |
 * |:-------------------------:|:--------:|:--------------------------------------------------------------------------------------------:|:---------:|
 * |       `description`       | `string` |          Description of the battery client. It is the device key name                        |    Yes    |
 * |       `local_prefix`      | `string` |                     Prefix of the local port (e.g. the application name)                     |    Yes    |
 * | `local_port_name_postfix` | `string` | Postfix of the local port. The local port name is `/<local_prefix><local_port_name_postfix>` |    Yes    |
 * |     `remote_port_name`    | `string` |                Full name of the port opened on the battery server side                       |    Yes    |
 * |          `carrier`        | `string` |                         Carrier for the connection (default "tcp")                           |     No    |
 * @return A PolyDriverDescriptor. In case of error an invalid `PolyDriverDescriptor` is returned.
 */
PolyDriverDescriptor constructBatteryClient(
    std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler);

} // namespace RobotInterface
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_ROBOT_INTERFACE_YARP_HELPER_H
