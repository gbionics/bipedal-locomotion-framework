/**
 * @file YarpRobotLoggerDeviceCommands.thrift
 * @authors Lorenzo Moretti <lorenzo.moretti@iit.it>
 * @copyright 2024 Fondazione Istituto Italiano di Tecnologia
 *            Released under the terms of the BSD-3-Clause license.
 * @date 2024
 */

service YarpRobotLoggerDeviceCommands
{
    bool record();
    bool isRecording();
    bool saveData(1: string tag = "");
    bool stopRecording();
}
