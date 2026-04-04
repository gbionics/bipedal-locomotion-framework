/**
 * @copyright 2024 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <cstdlib>
#include <chrono>
#include <filesystem>
#include <random>
#include <thread>

// BLF
#include <BipedalLocomotion/TextLogging/Logger.h>

// Catch2
#include <catch2/catch_test_macros.hpp>

// YARP
#include <yarp/sig/Vector.h>
#include <yarp/eigen/Eigen.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>
#include <yarp/os/RpcClient.h>
#include <yarp/robotinterface/XMLReader.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>
#include <yarp/dev/PolyDriver.h>

// matioCpp
#include <matioCpp/matioCpp.h>

// Thrift-generated RPC client
#include <YarpRobotLoggerDeviceCommands.h>

// This function populate the YARP_DATA_DIRS environment variable to
// ensure that YARP devices from YARP (from YARP install prefix) and BLF (from the build directory of BLF)
// can be correctly found and launched by libYARP_robotinterface
// Once this is used in multiple tests, we can move it to a common place to share it among tests
bool blf_setenv(const std::string &_name, const std::string &_value)
{
#ifdef _WIN32
    if (0 != _putenv_s(_name.c_str(), _value.c_str()))
    {
        return false;
    }
#else
    if (0 != ::setenv(_name.c_str(), _value.c_str(), true))
    {
        return false;
    }
#endif
    return true;
}


bool ensureYARPAndBLFYARPDevicesCanBeFound()
{
    // To make sure that YarpRobotLoggerDevice is found from the build directory, we add CMAKE_BINARY_DIR
    // to YARP_DATA_DIRS
#ifdef _WIN32
    std::string envVarListSeparator = ";";
#else
    std::string envVarListSeparator = ":";
#endif

    // Make sure that YARP devices can be found
    std::string new_yarp_data_dirs_value = YARP_DATA_INSTALL_DIR_FULL;

    // Make sure that BLF devices are available
    new_yarp_data_dirs_value = new_yarp_data_dirs_value + envVarListSeparator + CMAKE_BINARY_DIR + "/share/yarp";

    return blf_setenv("YARP_DATA_DIRS", new_yarp_data_dirs_value);
}

// Helper: remove all .mat and .md files in the current directory, renaming them to .bak
void backupExistingFiles()
{
    std::filesystem::path currentPath = std::filesystem::current_path();
    for (const auto& entry : std::filesystem::directory_iterator(currentPath))
    {
        if (entry.is_regular_file())
        {
            std::filesystem::path filePath = entry.path();
            std::string extension = filePath.extension().string();
            if (extension == ".mat" || extension == ".md")
            {
                std::filesystem::path newFilePath = filePath;
                newFilePath += ".bak";
                std::filesystem::rename(filePath, newFilePath);
            }
        }
    }
}

// Helper: find the first .mat file in the current working directory
std::string findMatFile()
{
    std::filesystem::path currentPath = std::filesystem::current_path();
    for (const auto& entry : std::filesystem::directory_iterator(currentPath))
    {
        if (entry.is_regular_file() && entry.path().extension().string() == ".mat")
        {
            return entry.path().string();
        }
    }
    return "";
}

// Helper: count .mat files in the current working directory
int countMatFiles()
{
    int count = 0;
    std::filesystem::path currentPath = std::filesystem::current_path();
    for (const auto& entry : std::filesystem::directory_iterator(currentPath))
    {
        if (entry.is_regular_file() && entry.path().extension().string() == ".mat")
        {
            count++;
        }
    }
    return count;
}

// Helper: start yarprobotinterface from a given XML file
yarp::robotinterface::XMLReaderResult launchRobotInterface(const std::string& xmlFileName)
{
    std::filesystem::path pathToXmlConfigurationFile
        = std::filesystem::path(CMAKE_CURRENT_SOURCE_DIR) / std::filesystem::path(xmlFileName);

    BipedalLocomotion::log()->info("Loading yarprobotinterface file from {}",
                                   pathToXmlConfigurationFile.string());

    yarp::os::Property yarprobotinterfaceConfig;
    yarp::os::Bottle enableTags;
    yarp::os::Bottle& enableTagsList = enableTags.addList();
    yarp::os::Bottle disableTags;
    yarp::os::Bottle& disableTagsList = disableTags.addList();

    if (enableTagsList.size() > 0)
    {
        yarprobotinterfaceConfig.put("enable_tags", enableTags.get(0));
    }

    if (disableTagsList.size() > 0)
    {
        yarprobotinterfaceConfig.put("disable_tags", disableTags.get(0));
    }

    yarp::robotinterface::XMLReader yarprobotinterfaceReader;
    return yarprobotinterfaceReader.getRobotFromFile(
        pathToXmlConfigurationFile.string(), yarprobotinterfaceConfig);
}

// Helper: shut down yarprobotinterface
void shutdownRobotInterface(yarp::robotinterface::XMLReaderResult& instance)
{
    BipedalLocomotion::log()->info("Halting yarprobotinterface.");
    REQUIRE(instance.robot.enterPhase(yarp::robotinterface::ActionPhaseInterrupt1));
    REQUIRE(instance.robot.enterPhase(yarp::robotinterface::ActionPhaseShutdown));
}

// Helper: validate joint positions in the .mat file match the expected values
void validateJointPositions(const std::string& matFile,
                            const Eigen::VectorXd& expected,
                            size_t nrOfJoints)
{
    matioCpp::File savedLog(matFile);
    REQUIRE(savedLog.isOpen());

    matioCpp::Struct robotLoggerDeviceStruct = savedLog.read("robot_logger_device").asStruct();

    matioCpp::MultiDimensionalArray<double> jointPosLoggedData
        = robotLoggerDeviceStruct["joints_state"]
              .asStruct()["positions"]
              .asStruct()["data"]
              .asMultiDimensionalArray<double>();

    Eigen::VectorXd jointPosLogged;
    jointPosLogged.resize(nrOfJoints);
    for (size_t i = 0; i < jointPosLogged.size(); i++)
    {
        jointPosLogged(i) = jointPosLoggedData({i, 0, 0});
    }

    REQUIRE(jointPosLogged.isApprox(expected, 1e-14));
}

// Helper: validate accelerometer data in the .mat file
void validateAccelerometer(const std::string& matFile,
                           const Eigen::VectorXd& expected,
                           size_t nrOfDirections)
{
    matioCpp::File savedLog(matFile);
    REQUIRE(savedLog.isOpen());

    matioCpp::Struct robotLoggerDeviceStruct = savedLog.read("robot_logger_device").asStruct();

    matioCpp::MultiDimensionalArray<double> accelerometerLoggedData
        = robotLoggerDeviceStruct["accelerometers"]
              .asStruct()["sim_imu_sensor"]
              .asStruct()["data"]
              .asMultiDimensionalArray<double>();

    Eigen::VectorXd accelerometerLogged;
    accelerometerLogged.resize(nrOfDirections);
    for (size_t i = 0; i < accelerometerLogged.size(); i++)
    {
        accelerometerLogged(i) = accelerometerLoggedData({i, 0, 0});
    }

    REQUIRE(accelerometerLogged.isApprox(expected, 1e-14));
    // The z component should be near to -9.8
    CHECK(std::abs(accelerometerLogged(2) - (-9.8)) <= 1.0);
}

// Helper: read current encoder/accelerometer values from the robotinterface devices
struct SensorReadings
{
    Eigen::VectorXd jointPositions;
    Eigen::VectorXd accelerometer;
    size_t nrOfJoints{4};
    size_t nrOfAccelDirs{3};
};

SensorReadings readSensorValues(yarp::robotinterface::XMLReaderResult& instance)
{
    SensorReadings readings;

    CHECK(instance.robot.hasDevice("sim_controlboard"));
    CHECK(instance.robot.hasDevice("sim_imu"));

    // Read encoders
    yarp::dev::IEncoders* iencs = nullptr;
    CHECK(instance.robot.device("sim_controlboard").driver()->view(iencs));
    REQUIRE(iencs != nullptr);
    readings.jointPositions.resize(readings.nrOfJoints);
    CHECK(iencs->getEncoders(readings.jointPositions.data()));

    // Read accelerometer
    yarp::dev::IThreeAxisLinearAccelerometers* ilinacc = nullptr;
    CHECK(instance.robot.device("sim_imu").driver()->view(ilinacc));
    REQUIRE(ilinacc != nullptr);
    yarp::sig::Vector accelerometerReadYARP;
    readings.accelerometer.resize(readings.nrOfAccelDirs);
    size_t sensIndex = 0;
    double timestamp;
    CHECK(ilinacc->getThreeAxisLinearAccelerometerMeasure(sensIndex, accelerometerReadYARP, timestamp));
    readings.accelerometer = yarp::eigen::toEigen(accelerometerReadYARP);

    return readings;
}

TEST_CASE("Auto-start logger saves on close (Ctrl+C)")
{
    // This test verifies the original behavior: with auto_start_logging=true the logger
    // records automatically, and when the device is closed (simulating Ctrl+C / shutdown),
    // the data is auto-saved to a .mat file.
    backupExistingFiles();

    yarp::os::Network network;
    yarp::os::NetworkBase::setLocalMode(true);

    REQUIRE(ensureYARPAndBLFYARPDevicesCanBeFound());

    auto instance = launchRobotInterface("launch-yarp-robot-logger.xml");
    REQUIRE(instance.parsingIsSuccessful);
    REQUIRE(instance.robot.enterPhase(yarp::robotinterface::ActionPhaseStartup));

    BipedalLocomotion::log()->info("yarprobotinterface successfully loaded and started.");

    // Collect some data
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Read sensor values for later comparison
    auto readings = readSensorValues(instance);

    // Shut down (triggers auto-save in close())
    shutdownRobotInterface(instance);

    // Verify .mat file was created
    std::string matFile = findMatFile();
    REQUIRE(!matFile.empty());

    validateJointPositions(matFile, readings.jointPositions, readings.nrOfJoints);
    validateAccelerometer(matFile, readings.accelerometer, readings.nrOfAccelDirs);
}

TEST_CASE("RPC record and saveData")
{
    // This test verifies the RPC-driven workflow:
    // 1. Start with auto_start_logging=false → device is idle
    // 2. Call record() via RPC → recording starts
    // 3. Collect some data
    // 4. Call saveData("test_tag") → data saved and recording stops
    // 5. Verify the .mat file is created with correct data
    backupExistingFiles();

    yarp::os::Network network;
    yarp::os::NetworkBase::setLocalMode(true);

    REQUIRE(ensureYARPAndBLFYARPDevicesCanBeFound());

    auto instance = launchRobotInterface("launch-yarp-robot-logger-no-autostart.xml");
    REQUIRE(instance.parsingIsSuccessful);
    REQUIRE(instance.robot.enterPhase(yarp::robotinterface::ActionPhaseStartup));

    BipedalLocomotion::log()->info("yarprobotinterface loaded (no autostart).");

    // No .mat file should exist yet (no recording has started)
    CHECK(countMatFiles() == 0);

    // Connect RPC client to the logger's RPC port
    yarp::os::RpcClient rpcClient;
    REQUIRE(rpcClient.open("/test/rpc:o"));
    REQUIRE(yarp::os::Network::connect("/test/rpc:o", "/yarp-robot-logger/commands/rpc:i"));

    YarpRobotLoggerDeviceCommands rpcInterface;
    rpcInterface.yarp().attachAsClient(rpcClient);

    // Start recording via RPC
    bool recordOk = rpcInterface.record();
    CHECK(recordOk);

    // Collect some data
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Read sensor values for comparison
    auto readings = readSensorValues(instance);

    // Save episode with a tag via RPC (this saves and stops recording)
    bool saveOk = rpcInterface.saveData("test_tag");
    CHECK(saveOk);

    // Verify the .mat file was created
    std::string matFile = findMatFile();
    REQUIRE(!matFile.empty());

    // The filename should contain the tag
    CHECK(matFile.find("test_tag") != std::string::npos);

    validateJointPositions(matFile, readings.jointPositions, readings.nrOfJoints);
    validateAccelerometer(matFile, readings.accelerometer, readings.nrOfAccelDirs);

    rpcClient.close();
    shutdownRobotInterface(instance);
}

TEST_CASE("RPC record and stopRecording discards data")
{
    // This test verifies that stopRecording discards data without saving:
    // 1. Start with auto_start_logging=false → device is idle
    // 2. Call record() via RPC → recording starts
    // 3. Collect some data
    // 4. Call stopRecording() → recording stops, no .mat saved
    // 5. Verify no .mat file was created
    backupExistingFiles();

    yarp::os::Network network;
    yarp::os::NetworkBase::setLocalMode(true);

    REQUIRE(ensureYARPAndBLFYARPDevicesCanBeFound());

    auto instance = launchRobotInterface("launch-yarp-robot-logger-no-autostart.xml");
    REQUIRE(instance.parsingIsSuccessful);
    REQUIRE(instance.robot.enterPhase(yarp::robotinterface::ActionPhaseStartup));

    BipedalLocomotion::log()->info("yarprobotinterface loaded (no autostart).");

    // Connect RPC client
    yarp::os::RpcClient rpcClient;
    REQUIRE(rpcClient.open("/test/rpc:o"));
    REQUIRE(yarp::os::Network::connect("/test/rpc:o", "/yarp-robot-logger/commands/rpc:i"));

    YarpRobotLoggerDeviceCommands rpcInterface;
    rpcInterface.yarp().attachAsClient(rpcClient);

    // Start and then stop recording without saving
    CHECK(rpcInterface.record());
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    CHECK(rpcInterface.stopRecording());

    // No .mat file should be produced
    CHECK(countMatFiles() == 0);

    rpcClient.close();
    shutdownRobotInterface(instance);
}

TEST_CASE("RPC record, saveData, then record again")
{
    // This test verifies that after saveData the device can record() again:
    // 1. record() → collect data → saveData() → .mat created
    // 2. record() again → collect data → saveData("second") → second .mat created
    backupExistingFiles();

    yarp::os::Network network;
    yarp::os::NetworkBase::setLocalMode(true);

    REQUIRE(ensureYARPAndBLFYARPDevicesCanBeFound());

    auto instance = launchRobotInterface("launch-yarp-robot-logger-no-autostart.xml");
    REQUIRE(instance.parsingIsSuccessful);
    REQUIRE(instance.robot.enterPhase(yarp::robotinterface::ActionPhaseStartup));

    // Connect RPC client
    yarp::os::RpcClient rpcClient;
    REQUIRE(rpcClient.open("/test/rpc:o"));
    REQUIRE(yarp::os::Network::connect("/test/rpc:o", "/yarp-robot-logger/commands/rpc:i"));

    YarpRobotLoggerDeviceCommands rpcInterface;
    rpcInterface.yarp().attachAsClient(rpcClient);

    // First recording session
    CHECK(rpcInterface.record());
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    CHECK(rpcInterface.saveData("first"));
    CHECK(countMatFiles() == 1);

    // Second recording session
    CHECK(rpcInterface.record());
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    CHECK(rpcInterface.saveData("second"));
    CHECK(countMatFiles() == 2);

    rpcClient.close();
    shutdownRobotInterface(instance);
}
