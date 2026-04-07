/**
 * @copyright 2020, 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <tuple>

#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h>
#include <BipedalLocomotion/System/Clock.h>
#include <BipedalLocomotion/System/YarpClock.h>
#include <BipedalLocomotion/TextLogging/Logger.h>
#include <BipedalLocomotion/TextLogging/LoggerBuilder.h>
#include <BipedalLocomotion/TextLogging/YarpLogger.h>
#include <BipedalLocomotion/YarpRobotLoggerDevice.h>
#include <BipedalLocomotion/YarpTextLoggingUtilities.h>
#include <BipedalLocomotion/YarpUtilities/Helper.h>
#include <BipedalLocomotion/YarpUtilities/VectorsCollection.h>
#include <BipedalLocomotion/MessageConversionUtilities.h>

#include <Eigen/Core>

#include <yarp/conf/version.h>
#include <yarp/cv/Cv.h>
#include <yarp/eigen/Eigen.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/profiler/NetworkProfiler.h>

#include <robometry/BufferConfig.h>
#include <robometry/BufferManager.h>

#include <process.hpp>

using namespace BipedalLocomotion::YarpUtilities;
using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::RobotInterface;
using namespace BipedalLocomotion;

VISITABLE_STRUCT(TextLoggingEntry,
                 level,
                 text,
                 filename,
                 line,
                 function,
                 hostname,
                 cmd,
                 args,
                 pid,
                 thread_id,
                 component,
                 id,
                 systemtime,
                 networktime,
                 externaltime,
                 backtrace,
                 yarprun_timestamp,
                 local_timestamp);

void findAndReplaceAll(std::string& data,
                       const std::string& toSearch,
                       const std::string& replaceStr)
{
    // Get the first occurrence
    size_t pos = data.find(toSearch);
    // Repeat till end is reached
    while (pos != std::string::npos)
    {
        // Replace this occurrence of Sub String
        data.replace(pos, toSearch.size(), replaceStr);
        // Get the next occurrence from the current position
        pos = data.find(toSearch, pos + replaceStr.size());
    }
}

bool YarpRobotLoggerDevice::VectorsCollectionSignal::connect()
{
    return client.connect();
}

void YarpRobotLoggerDevice::VectorsCollectionSignal::disconnect()
{
    if (connected)
    {
        client.disconnect();
    }
}

YarpRobotLoggerDevice::YarpRobotLoggerDevice(double period,
                                             yarp::os::ShouldUseSystemClock useSystemClock)
    : yarp::os::PeriodicThread(period, useSystemClock, yarp::os::PeriodicThreadClock::Absolute)
{
    // Use the yarp clock in blf
    BipedalLocomotion::System::ClockBuilder::setFactory(
        std::make_shared<BipedalLocomotion::System::YarpClockFactory>());

    // the logging message are streamed using yarp
    BipedalLocomotion::TextLogging::LoggerBuilder::setFactory(
        std::make_shared<BipedalLocomotion::TextLogging::YarpLoggerFactory>());

    m_sendDataRT = false;
}

YarpRobotLoggerDevice::YarpRobotLoggerDevice()
    : yarp::os::PeriodicThread(0.01,
                               yarp::os::ShouldUseSystemClock::No,
                               yarp::os::PeriodicThreadClock::Absolute)
{
    // Use the yarp clock in blf
    BipedalLocomotion::System::ClockBuilder::setFactory(
        std::make_shared<BipedalLocomotion::System::YarpClockFactory>());

    // the logging message are streamed using yarp
    BipedalLocomotion::TextLogging::LoggerBuilder::setFactory(
        std::make_shared<BipedalLocomotion::TextLogging::YarpLoggerFactory>());

    m_sendDataRT = false;
}

YarpRobotLoggerDevice::~YarpRobotLoggerDevice() = default;

bool YarpRobotLoggerDevice::open(yarp::os::Searchable& config)
{

    bool needsAttach = false;

    constexpr auto logPrefix = "[YarpRobotLoggerDevice::open]";
    auto params = std::make_shared<ParametersHandler::YarpImplementation>(config);

    if (!params->getParameter("enable_real_time_logging", m_sendDataRT))
    {
        log()->error("{} Unable to get the 'enable_real_time_logging' parameter. The device will "
                     "not "
                     "be opened.",
                     logPrefix);
        return false;
    }

    if (m_sendDataRT)
    {
        auto rtParameters = params->getGroup("REAL_TIME_STREAMING").lock();

        if (!m_vectorCollectionRTDataServer.initialize(rtParameters))
        {
            log()->error("Failed to initalize the vectorsCollectionServer", logPrefix);
            return false;
        }
        std::string remote;
        rtParameters->getParameter("remote", remote);
        log()->info("{} Activating Real Time Logging on yarp port: {}", logPrefix, remote);
    } else
    {
        log()->info("{} Real time logging not activated", logPrefix);
    }

    double devicePeriod{0.01};
    if (params->getParameter("sampling_period_in_s", devicePeriod))
    {
        this->setPeriod(devicePeriod);
    }

    if (!params->getParameter("log_text", m_logText))
    {
        log()->info("{} Unable to get the 'log_text' parameter for the telemetry. Default value: "
                    "{}.",
                    logPrefix,
                    m_logText);
    }

    if (m_logText)
    {
        if (!params->getParameter("text_logging_subnames", m_textLoggingSubnames))
        {
            log()->info("{} Unable to get the 'text_logging_subnames' parameter for the telemetry. "
                        "All the ports related to the text logging will be considered.",
                        logPrefix);
        }
    }

    if (!params->getParameter("log_code_status", m_logCodeStatus))
    {
        log()->info("{} Unable to get the 'log_code_status' parameter for the telemetry. Default "
                    "value: {}.",
                    logPrefix,
                    m_logCodeStatus);
    }

    if (m_logCodeStatus)
    {
        if (!params->getParameter("code_status_cmds", m_codeStatusCmds))
        {
            log()->info("{} Unable to get the 'code_status_cmds' parameter. No custom "
                        "commands will be executed.",
                        logPrefix);
        }
    }

    if (!params->getParameter("maximum_admissible_time_step", m_acceptableStep))
    {
        log()->info("{} Unable to get the 'maximum_admissible_time_step' parameter. The time "
                    "step will be set to the default value. Default value: {}",
                    logPrefix,
                    m_acceptableStep);
    }

    if (!params->getParameter("auto_start_logging", m_autoStartLogging))
    {
        log()->info("{} Unable to get the 'auto_start_logging' parameter. Default value: {}.",
                    logPrefix,
                    m_autoStartLogging);
    }

    if (!params->getParameter("log_robot_data", m_logRobot))
    {
        log()->info("{} Unable to get the 'log_robot_data' parameter for the telemetry. Default "
                    "value: {}.",
                    logPrefix,
                    m_logRobot);
    }

    if (m_logRobot)
    {
        needsAttach = true;
    }

    if (!this->setupRobotSensorBridge(params->getGroup("RobotSensorBridge")))
    {
        return false;
    }

    if (!params->getParameter("log_cameras", m_logCameras))
    {
        log()->info("{} Unable to get the 'log_cameras' parameter for the telemetry. Default "
                    "value: {}.",
                    logPrefix,
                    m_logCameras);
    }

    if (m_logCameras && this->setupRobotCameraBridge(params->getGroup("RobotCameraBridge")))
    {
        // get the metadata for rgb camera
        if (m_cameraBridge->getMetaData().bridgeOptions.isRGBCameraEnabled)
        {
            if (!populateCamerasData(logPrefix,
                                     params,
                                     "rgb_cameras_fps",
                                     m_cameraBridge->getMetaData().sensorsList.rgbCamerasList))
            {
                log()->error("{} Unable to populate the camera fps for RGB cameras.", logPrefix);
                return false;
            }
        }

        // Currently the logger supports only rgb cameras
        if (m_cameraBridge->getMetaData().bridgeOptions.isRGBDCameraEnabled)
        {
            if (!populateCamerasData(logPrefix,
                                     params,
                                     "rgbd_cameras_fps",
                                     m_cameraBridge->getMetaData().sensorsList.rgbdCamerasList))
            {
                log()->error("{} Unable to populate the camera fps for RGBD cameras.", logPrefix);
                return false;
            }
        }

        // get the video codec in case rgb or rgbd camera are enabled
        if (m_cameraBridge->getMetaData().bridgeOptions.isRGBDCameraEnabled
            || m_cameraBridge->getMetaData().bridgeOptions.isRGBCameraEnabled)
        {
            if (!params->getParameter("video_codec_code", m_videoCodecCode))
            {
                constexpr auto fourccCodecUrl = "https://abcavi.kibi.ru/fourcc.php";
                log()->info("{} The parameter 'video_codec_code' is not provided. The default one "
                            "will be used {}. You can find the list of supported parameters at: "
                            "{}.",
                            logPrefix,
                            m_videoCodecCode,
                            fourccCodecUrl);
            } else if (m_videoCodecCode.size() != 4)
            {
                constexpr auto fourccCodecUrl = "https://fourcc.org/codecs.php";
                log()->error("{} The parameter 'video_codec_code' must be a string with 4 "
                             "characters. You can find the list of supported parameters at: {}.",
                             logPrefix,
                             fourccCodecUrl);
                return false;
            }
            needsAttach = true;
        }
    } else
    {
        if (m_logCameras)
        {
            m_logCameras = false;
            log()->error("{} Failed to setup the camera bridge. The cameras will not "
                         "be logged.",
                         logPrefix);
        } else
        {
            log()->info("{} The video will not be recorded", logPrefix);
        }
    }

    if (!this->setupTelemetry(params->getGroup("Telemetry"), devicePeriod))
    {
        return false;
    }

    if (!this->setupExogenousInputs(params->getGroup("ExogenousSignals")))
    {
        return false;
    }

    if (!params->getParameter("log_frames", m_logFrames))
    {
        log()->info("{} Unable to get the 'log_frames' parameter for the telemetry. Default "
                    "value: {}.",
                    logPrefix,
                    m_logFrames);
    }

    auto tf_options = config.findGroup("Transforms");
    if (m_logFrames && setupTransformInputs(tf_options))
    {
        log()->info("{} The transforms will be logged.", logPrefix);
    } else if (m_logFrames)
    {
        log()->error("{} Failed to configure the frames logging. No frame will be logged.",
                     logPrefix);
        m_logFrames = false;
    } else
    {
        log()->info("{} The frames will not be logged.", logPrefix);
    }

    // open rpc port for YarpRobotLoggerDevice commands
    std::string portPrefix{"/yarp-robot-logger"};

    if (!params->getParameter("port_prefix", portPrefix))
    {

        log()->info("{} The 'port_prefix' is not provided. The default prefix {} will be used.",
                    logPrefix,
                    portPrefix);
    }

    std::string rpcPortFullName = portPrefix + m_rpcPortName;

    this->yarp().attachAsServer(this->m_rpcPort);
    if (!m_rpcPort.open(rpcPortFullName))
    {
        log()->error("{} Could not open", logPrefix);
        return false;
    }

    std::string statusPortFullName = portPrefix + m_statusPortName;
    if (!m_statusPort.open(statusPortFullName))
    {
        log()->error("{} Unable to open the status port named: {}.", logPrefix, statusPortFullName);
        return false;
    }

    log()->info("{} Logger configuration completed.", logPrefix);
    if (needsAttach)
    {
        log()->info("{} Waiting for the attach phases before starting the logging.", logPrefix);
    } else
    {
        if (m_autoStartLogging)
        {
            return record();
        }
        log()->info("{} auto_start_logging is disabled. Logging will not start automatically. "
                    "Use the RPC command 'record' to start.",
                    logPrefix);
    }

    return true;
}

bool YarpRobotLoggerDevice::setupExogenousInputs(
    std::weak_ptr<const ParametersHandler::IParametersHandler> params)
{
    constexpr auto logPrefix = "[YarpRobotLoggerDevice::setupExogenousInputs]";

    auto ptr = params.lock();
    if (ptr == nullptr)
    {
        log()->info("{} No exogenous input will be logged.", logPrefix);
        return true;
    }

    std::vector<std::string> inputs;
    if (!ptr->getParameter("vectors_collection_exogenous_inputs", inputs))
    {
        log()->error("{} Unable to get the exogenous inputs.", logPrefix);
        return false;
    }

    for (const auto& input : inputs)
    {
        auto group = ptr->getGroup(input).lock();
        std::string signalFullName, remote;
        if (group == nullptr)
        {
            log()->error("{} Unable to get the group named {}.", logPrefix, input);
            return false;
        }

        if (!group->getParameter("remote", remote))
        {
            log()->error("{} Unable to get the remote parameter for the group named {}.",
                         logPrefix,
                         input);
            return false;
        }

        if (!group->getParameter("signal_name", signalFullName))
        {
            log()->error("{} Unable to get the signal_name parameter for the group named {}.",
                         logPrefix,
                         input);
            return false;
        }

        m_vectorsCollectionSignals[remote].signalName = signalFullName;
        if (!m_vectorsCollectionSignals[remote].client.initialize(group))
        {
            log()->error("{} Unable to initialize the vectors collection signal for the group "
                         "named {}.",
                         logPrefix,
                         signalFullName);
            return false;
        }
    }

    auto openExogenousSignals = [logPrefix](auto ptr,
                                            const std::vector<std::string>& inputs,
                                            auto& signals_vector) -> bool {
        for (const auto& input : inputs)
        {
            auto group = ptr->getGroup(input).lock();
            std::string local, signalFullName, remote, carrier;
            if (group == nullptr || !group->getParameter("local", local)
                || !group->getParameter("remote", remote)
                || !group->getParameter("carrier", carrier)
                || !group->getParameter("signal_name", signalFullName))
            {
                log()->error("{} Unable to get the parameters related to the input: {}.",
                             logPrefix,
                             input);
                return false;
            }
            signals_vector[remote].signalName = signalFullName;
            signals_vector[remote].remote = remote;
            signals_vector[remote].local = local;
            signals_vector[remote].carrier = carrier;
            if (!signals_vector[remote].port.open(signals_vector[remote].local))
            {
                log()->error("{} Unable to open the port named: {}.",
                             logPrefix,
                             signals_vector[remote].local);
                return false;
            }
        }
        return true;
    };

    // Helper to open a group of exogenous signal ports.
    // If 'required' is true, failure to find the parameter returns false.
    // Otherwise, the missing parameter is just a warning and we continue.
    auto openExogenousGroup = [&](const std::string& paramName,
                                  auto& signalsMap,
                                  bool required) -> bool {
        inputs.clear();
        if (!ptr->getParameter(paramName, inputs))
        {
            if (required)
            {
                log()->error("{} Unable to get the '{}'.", logPrefix, paramName);
                return false;
            }
            log()->warn("{} Unable to get the '{}'. Assuming none.", logPrefix, paramName);
        }
        return openExogenousSignals(ptr, inputs, signalsMap);
    };

    if (!openExogenousGroup("vectors_exogenous_inputs", m_vectorSignals, true))
    {
        return false;
    }

    if (!openExogenousGroup("string_exogenous_inputs", m_stringSignals, false))
    {
        return false;
    }

    if (!openExogenousGroup("image_exogenous_inputs", m_imageSignals, false))
    {
        return false;
    }

    if (!prepareExogenousImageLogging())
    {
        log()->error("{} Unable to prepare the exogenous image logging.", logPrefix);
        return false;
    }

    if (!openExogenousGroup("human_state_exogenous_inputs", m_humanStateSignals, false))
    {
        return false;
    }

    if (!openExogenousGroup("wearable_targets_exogenous_inputs", m_wearableTargetsSignals, false))
    {
        return false;
    }

    if (!openExogenousGroup("wearable_data_exogenous_inputs", m_wearableDataSignals, false))
    {
        return false;
    }

    return true;
}

bool BipedalLocomotion::YarpRobotLoggerDevice::setupTransformInputs(const yarp::os::Bottle& config)
{
    // For this method we use directly the bottle and not the ParametersHandler since
    // we need to get the subgroup "TransformClientDevice" as a yarp bottle to be
    // passed directly to the PolyDriver

    constexpr auto logPrefix = "[YarpRobotLoggerDevice::setupTransformInputs]";
    if (config.isNull())
    {
        log()->error("{} The input config is not valid.", logPrefix);
        return false;
    }

    auto params = std::make_shared<ParametersHandler::YarpImplementation>(config);

    std::vector<std::string> parents;

    if (!params->getParameter("parent_frames", parents))
    {
        log()->error("{} Unable to get the 'parent_frames' parameter.", logPrefix);
        return false;
    }

    for (const auto& parent : parents)
    {
        m_tfRootFrames.insert(parent);
    }

    if (m_tfRootFrames.empty())
    {
        log()->info("{} The 'parent_frames' parameter is empty. No frame will be logged.",
                    logPrefix);
        return false;
    }

    std::string rotationParametrization;

    yarp::os::Bottle& device_group = config.findGroup("TransformClientDevice");

    if (device_group.isNull())
    {
        log()->error("{} The 'TransformClientDevice' group is not provided.", logPrefix);
        return false;
    }

    if (!m_tfDevice.open(device_group))
    {
        log()->error("{} Unable to open the TransformClientDevice.", logPrefix);
        return false;
    }

    if (!m_tfDevice.view(m_tf) || !m_tf)
    {
        log()->error("{} Unable to view the TransformClient interface.", logPrefix);
        return false;
    }

    return true;
}

bool BipedalLocomotion::YarpRobotLoggerDevice::updateChildTransformList()
{
    constexpr auto logPrefix = "[YarpRobotLoggerDevice::updateChildTransforms]";

    if (!m_tf)
    {
        log()->error("{} The TransformClient interface is not available.", logPrefix);
        return false;
    }

    for (auto& childFrame : m_tfChildFrames)
    {
        childFrame.second.active = false;
    }

    m_allFrames.clear(); // When getting the ids, it seems that the input vector is not
                         // cleared. Passing a clean one every time.
    if (m_tf->getAllFrameIds(m_allFrames))
    {
        for (std::string& id : m_allFrames)
        {
            if (m_tfChildFrames.find(id) == m_tfChildFrames.end()
                && m_tfRootFrames.find(id) == m_tfRootFrames.end())
            {
                bool canTransform = false;
                for (const auto& rootFrame : m_tfRootFrames)
                {
#if YARP_VERSION_COMPARE(<, 3, 11, 0)
                    canTransform = m_tf->canTransform(id, rootFrame);
#else
                    bool canTranformOk = false;
                    bool canTransformRetValue = m_tf->canTransform(id, rootFrame, canTranformOk);
                    canTransform = canTranformOk && canTransformRetValue;
#endif
                    if (canTransform)
                    {
                        FrameDescriptor frameDesc;
                        frameDesc.parent = rootFrame;
                        frameDesc.positionChannelName
                            = "frames::" + rootFrame + "::" + id + "::position";
                        frameDesc.orientationChannelName
                            = "frames::" + rootFrame + "::" + id + "::orientation";
                        m_tfChildFrames[id] = frameDesc;
                        bool ok = addChannel(frameDesc.positionChannelName, 3, {"x", "y", "z"});
                        ok = ok
                             && addChannel(frameDesc.orientationChannelName,
                                           4,
                                           {"qx", "qy", "qz", "qw"});
                        if (!ok)
                        {
                            log()->error("{} Unable to add the channels for the frame: {}.",
                                         logPrefix,
                                         id);
                            return false;
                        }
                        break;
                    }
                }
            } else if (m_tfChildFrames.find(id) != m_tfChildFrames.end())
            {
                m_tfChildFrames[id].active = true;
            }
        }
    }
    return true;
}

bool YarpRobotLoggerDevice::setupTelemetry(
    std::weak_ptr<const ParametersHandler::IParametersHandler> params, const double& devicePeriod)
{
    constexpr auto logPrefix = "[YarpRobotLoggerDevice::setupTelemetry]";

    auto ptr = params.lock();
    if (ptr == nullptr)
    {
        log()->error("{} The parameters handler is not valid.", logPrefix);
        return false;
    }

    robometry::BufferConfig config;
    char* tmp = std::getenv("YARP_ROBOT_NAME");
    // if the variable does not exist it points to NULL
    if (tmp != NULL)
    {
        config.yarp_robot_name = tmp;
    }
    config.filename = defaultFilePrefix;
    // We always want to save on shutdown since we are also logging videos that are saving
    // frames in a temporary folder. Thus we need to ensure that on shutdown the folder
    // is renamed, and this is done in the save callback.
    config.auto_save = true;
    config.file_indexing = "%Y_%m_%d_%H_%M_%S";
    config.mat_file_version = matioCpp::FileVersion::MAT7_3;

    double savePeriod{1800.0}; // default 30 minutes

    if (!ptr->getParameter("save_period", savePeriod))
    {
        log()->info("{} Unable to get the 'save_period' parameter for the telemetry. "
                    "Default value: {}.",
                    logPrefix,
                    savePeriod);
    }
    config.save_period = savePeriod;

    bool savePeriodically{true};
    if (!ptr->getParameter("save_periodically", savePeriodically))
    {
        log()->info("{} Unable to get the 'save_periodically' parameter for the telemetry. "
                    "Default value: {}.",
                    logPrefix,
                    savePeriodically);
    }
    config.save_periodically = savePeriodically;

    // the telemetry will flush the content of its storage every config.save_period
    // and this device runs every devicePeriod
    // so the size of the telemetry buffer must be at least config.save_period / devicePeriod
    // to be sure we are not going to lose data the buffer will be 10% longer
    constexpr double percentage = 0.1;
    config.n_samples = static_cast<int>(std::ceil((1 + percentage) //
                                                  * (savePeriod / devicePeriod)));

    return m_bufferManager.configure(config);
}

bool YarpRobotLoggerDevice::setupRobotSensorBridge(
    std::weak_ptr<const ParametersHandler::IParametersHandler> params)
{
    if (!m_logRobot)
    {
        log()->info("[YarpRobotLoggerDevice::setupRobotSensorBridge] The robot data will not be "
                    "logged.");
        m_robotSensorBridge = nullptr;
        m_streamJointStates = false;
        m_streamJointAccelerations = false;
        m_streamMotorTemperature = false;
        m_streamMotorStates = false;
        m_streamMotorPWM = false;
        m_streamPIDs = false;
        m_streamInertials = false;
        m_streamCartesianWrenches = false;
        m_streamFTSensors = false;
        m_streamTemperatureSensors = false;

        return true;
    }

    constexpr auto logPrefix = "[YarpRobotLoggerDevice::setupRobotSensorBridge]";

    auto ptr = params.lock();
    if (ptr == nullptr)
    {
        log()->error("{} The parameters handler is not valid.", logPrefix);
        return false;
    }

    m_robotSensorBridge = std::make_unique<YarpSensorBridge>();
    if (!m_robotSensorBridge->initialize(ptr))
    {
        log()->error("{} Unable to configure the 'SensorBridge'", logPrefix);
        return false;
    }

    // Get additional flags required by the device
    auto getStreamFlag = [&](const std::string& paramName, bool& flag) {
        if (!ptr->getParameter(paramName, flag))
        {
            log()->info("{} The '{}' parameter is not found. Not logged.", logPrefix, paramName);
        }
    };

    getStreamFlag("stream_joint_states", m_streamJointStates);
    getStreamFlag("stream_joint_accelerations", m_streamJointAccelerations);
    getStreamFlag("stream_motor_temperature", m_streamMotorTemperature);
    getStreamFlag("stream_motor_states", m_streamMotorStates);
    getStreamFlag("stream_motor_PWM", m_streamMotorPWM);
    getStreamFlag("stream_pids", m_streamPIDs);
    getStreamFlag("stream_inertials", m_streamInertials);
    getStreamFlag("stream_cartesian_wrenches", m_streamCartesianWrenches);
    getStreamFlag("stream_forcetorque_sensors", m_streamFTSensors);
    getStreamFlag("stream_temperatures", m_streamTemperatureSensors);

    return true;
}

bool YarpRobotLoggerDevice::setupRobotCameraBridge(
    std::weak_ptr<const ParametersHandler::IParametersHandler> params)
{
    constexpr auto logPrefix = "[YarpRobotLoggerDevice::setupRobotCameraBridge]";

    auto ptr = params.lock();

    if (ptr == nullptr)
    {
        log()->error("{} The 'RobotCameraBridge' group is not provided.", logPrefix);
        return false;
    }

    m_cameraBridge = std::make_unique<YarpCameraBridge>();
    if (!m_cameraBridge->initialize(ptr))
    {
        log()->error("{} Unable to configure the 'Camera bridge'", logPrefix);
        return false;
    }

    return true;
}

bool YarpRobotLoggerDevice::addChannel(const std::string& nameKey,
                                       std::size_t vectorSize,
                                       const std::vector<std::string>& metadataNames)
{
    // Skip adding channels if they were already registered in a previous recording session
    if (m_recordingPrepared)
    {
        return true;
    }

    if (metadataNames.empty() || vectorSize != metadataNames.size())
    {
        log()->warn("The metadata names for channel {} are empty or the size of the metadata names "
                    "is different from the vector size. The default metadata will be used."
                    "Vector size: {}. Metadata names size: {}",
                    nameKey,
                    vectorSize,
                    metadataNames.size());
        if (!m_bufferManager.addChannel({nameKey, {vectorSize, 1}}))
        {
            log()->error("Failed to add the channel in buffer manager named: {}", nameKey);
            return false;
        }
    } else
    {
        if (!m_bufferManager.addChannel({nameKey, {metadataNames.size(), 1}, metadataNames}))
        {
            log()->error("Failed to add the channel in buffer manager named: {}", nameKey);
            return false;
        }
    }

    // RT mode: populate metadata for all signals (robot and exogenous)
    if (m_sendDataRT)
    {
        std::string rtNameKey = robotRtRootName + treeDelim + nameKey;

        // use provided metadata if available, otherwise generate defaults
        const bool hasMetadata = !metadataNames.empty();
        if (hasMetadata)
        {
            if (!m_vectorCollectionRTDataServer.populateMetadata(rtNameKey, metadataNames))
            {
                log()->error("[Realtime logging] Failed to populate the metadata for the signal {}",
                             rtNameKey);
                return false;
            }
        } else
        {
            std::vector<std::string> defaultMetadata;
            defaultMetadata.reserve(vectorSize);
            for (std::size_t i = 0; i < vectorSize; i++)
            {
                defaultMetadata.push_back("element_" + std::to_string(i));
            }

            if (!m_vectorCollectionRTDataServer.populateMetadata(rtNameKey, defaultMetadata))
            {
                log()->error("[Realtime logging] Failed to populate default metadata for the "
                             "signal {}",
                             rtNameKey);
                return false;
            }
        }

        // Make the new metadata available to clients
        if (!m_vectorCollectionRTDataServer.finalizeMetadata())
        {
            log()->warn("[Realtime logging] Failed to finalize RT metadata after adding '{}'. "
                        "Clients may not see the update until the next successful finalization.",
                        rtNameKey);
            return false;
        }
    }
    return true;
}

bool BipedalLocomotion::YarpRobotLoggerDevice::populateCamerasData(
    const std::string& logPrefix,
    std::shared_ptr<const ParametersHandler::IParametersHandler> params,
    const std::string& fpsParamName,
    const std::vector<std::string>& cameraNames)
{
    std::vector<int> fps, depthScale;
    std::vector<std::string> rgbSaveMode, depthSaveMode;
    if (!params->getParameter(fpsParamName, fps))
    {
        log()->error("{} Unable to find the parameter named: {}.", logPrefix, fpsParamName);
        return false;
    }

    // if the camera is an rgbd camera then user should provide the depth scale
    if ((&cameraNames) == (&m_cameraBridge->getMetaData().sensorsList.rgbdCamerasList))
    {
        if (!params->getParameter("rgbd_cameras_depth_scale", depthScale))
        {
            log()->error("{} Unable to find the parameter named: "
                         "'rgbd_cameras_depth_scale'.",
                         logPrefix);
            return false;
        }

        if (!params->getParameter("rgbd_cameras_rgb_save_mode", rgbSaveMode))
        {
            log()->error("{} Unable to find the parameter named: "
                         "'rgb_cameras_rgb_save_mode.",
                         logPrefix);
            return false;
        }

        if (!params->getParameter("rgbd_cameras_depth_save_mode", depthSaveMode))
        {
            log()->error("{} Unable to find the parameter named: "
                         "'rgbd_cameras_depth_save_mode.",
                         logPrefix);
            return false;
        }

        if (fps.size() != depthScale.size() || (fps.size() != rgbSaveMode.size())
            || (fps.size() != depthSaveMode.size()))
        {
            log()->error("{} Mismatch between the vector containing the size of the vector "
                         "provided from configuration"
                         "Number of cameras: {}. Size of the FPS vector {}. Size of the "
                         "depth scale vector {}."
                         "Size of 'rgb_cameras_rgb_save_mode' {}. Size of "
                         "'rgb_cameras_depth_save_mode': {}",
                         logPrefix,
                         cameraNames.size(),
                         fps.size(),
                         depthScale.size(),
                         rgbSaveMode.size(),
                         depthSaveMode.size());
            return false;
        }
    } else
    {
        if (!params->getParameter("rgb_cameras_rgb_save_mode", rgbSaveMode))
        {
            log()->error("{} Unable to find the parameter named: "
                         "'rgb_cameras_rgb_save_mode.",
                         logPrefix);
            return false;
        }
    }

    if ((fps.size() != rgbSaveMode.size()))
    {
        log()->error("{} Mismatch between the vector containing the size of the vector "
                     "provided from configuration"
                     "Number of cameras: {}. Size of the FPS vector {}."
                     "Size of 'rgb_cameras_rgb_save_mode' {}.",
                     logPrefix,
                     cameraNames.size(),
                     fps.size(),
                     rgbSaveMode.size());
        return false;
    }

    if (fps.size() != cameraNames.size())
    {
        log()->error("{} Mismatch between the number of cameras and the vector containing "
                     "the FPS. Number of cameras: {}. Size of the FPS vector {}.",
                     logPrefix,
                     cameraNames.size(),
                     fps.size());
        return false;
    }

    auto createImageSaver
        = [logPrefix](const std::string& saveMode) -> std::shared_ptr<VideoWriter::ImageSaver> {
        auto saver = std::make_shared<VideoWriter::ImageSaver>();
        if (saveMode == "frame")
        {
            saver->saveMode = VideoWriter::SaveMode::Frame;
        } else if (saveMode == "video")
        {
            saver->saveMode = VideoWriter::SaveMode::Video;
        } else
        {
            log()->error("{} The save mode associated to the one of the camera is neither "
                         "'frame' nor 'video'. Provided: {}",
                         logPrefix,
                         saveMode);
            return nullptr;
        }
        return saver;
    };

    bool ok = true;
    for (unsigned int i = 0; i < fps.size(); i++)
    {
        if (fps[i] <= 0)
        {
            log()->error("{} The FPS associated to the camera {} is negative or equal to "
                         "zero.",
                         logPrefix,
                         i);
            return false;
        }

        // get the desired fps for each camera
        m_videoWriters[cameraNames[i]].fps = fps[i];

        // this means that the list of cameras are rgb camera
        if ((&cameraNames) == (&m_cameraBridge->getMetaData().sensorsList.rgbCamerasList))
        {
            m_videoWriters[cameraNames[i]].rgb = createImageSaver(rgbSaveMode[i]);
            ok = ok && m_videoWriters[cameraNames[i]].rgb != nullptr;
        }
        // this means that the list of cameras are rgbd camera
        else
        {
            if (depthSaveMode[i] == "video")
            {
                log()->warn("{} The depth stream of the rgbd camera {} will be saved as a "
                            "grayscale 8bit video. We suggest to save it as a set of "
                            "frames.",
                            logPrefix,
                            i);
            }
            m_videoWriters[cameraNames[i]].rgb = createImageSaver(rgbSaveMode[i]);
            ok = ok && m_videoWriters[cameraNames[i]].rgb != nullptr;
            m_videoWriters[cameraNames[i]].depth = createImageSaver(depthSaveMode[i]);
            ok = ok && m_videoWriters[cameraNames[i]].depth != nullptr;
            m_videoWriters[cameraNames[i]].depthScale = depthScale[i];
        }
    }

    return ok;
}

bool YarpRobotLoggerDevice::attachAll(const yarp::dev::PolyDriverList& poly)
{
    constexpr auto logPrefix = "[YarpRobotLoggerDevice::attachAll]";

    if (m_robotSensorBridge != nullptr)
    {
        if (!m_robotSensorBridge->setDriversList(poly))
        {
            log()->error("{} Could not attach drivers list to sensor bridge.", logPrefix);
            return false;
        }
    }

    // The user can avoid to record the camera
    if (m_cameraBridge != nullptr)
    {
        if (!m_cameraBridge->setDriversList(poly))
        {
            log()->error("{} Could not attach drivers list to camera bridge.", logPrefix);
            return false;
        }
    }

    log()->info("{} Attach completed. Starting logger.", logPrefix);
    if (!this->isRunning())
    {
        if (m_autoStartLogging)
        {
            return record();
        }
        log()->info("{} auto_start_logging is disabled. Logging will not start automatically. "
                    "Use the RPC command 'record' to start.",
                    logPrefix);
    }

    return true;
}

bool BipedalLocomotion::YarpRobotLoggerDevice::record()
{
    constexpr auto logPrefix = "[YarpRobotLoggerDevice::record]";

    if (this->isRunning())
    {
        log()->warn("{} Recording is already in progress.", logPrefix);
        return false;
    }

    if (m_logText)
    {
        // open the TextLogging port
        bool ok = m_textLoggingPort.open(m_textLoggingPortName);
        if (!ok)
        {
            log()->error("{} Unable to open the text logging port named {}.",
                         logPrefix,
                         m_textLoggingPortName);
            return false;
        }
        // run the thread
        m_lookForNewLogsThread = std::thread([this] { this->lookForNewLogs(); });
    }

    // Refresh all exogenous signal connections so they are re-established cleanly
    auto refreshExogenousConnections = [](auto& signals) {
        for (auto& [name, signal] : signals)
        {
            signal.disconnect();
            signal.connected = false;
            signal.dataArrived = false;
        }
    };
    refreshExogenousConnections(m_vectorsCollectionSignals);
    refreshExogenousConnections(m_vectorSignals);
    refreshExogenousConnections(m_stringSignals);
    refreshExogenousConnections(m_humanStateSignals);
    refreshExogenousConnections(m_wearableTargetsSignals);
    refreshExogenousConnections(m_wearableDataSignals);
    refreshExogenousConnections(m_imageSignals);

    // Also clear cached metadata for VectorsCollection signals
    for (auto& [name, signal] : m_vectorsCollectionSignals)
    {
        signal.metadata.vectors.clear();
    }

    // run the thread for reading the exogenous signals
    m_lookForNewExogenousSignalThread = std::thread([this] { this->lookForExogenousSignals(); });

    bool ok = m_bufferManager.setSaveCallback(
        [this](const std::string& filePrefix,
               const robometry::SaveCallbackSaveMethod& method) -> bool {
            return this->saveCallback(filePrefix, method);
        });

    if (!ok)
    {
        log()->error("{} Unable to set the save callback for the telemetry.", logPrefix);
        return false;
    }

    if (!this->prepareRobotLogging())
    {
        log()->error("{} Unable to prepare the robot logging.", logPrefix);
        return false;
    }

    // prepare the camera logging
    if (!this->prepareCameraLogging())
    {
        log()->error("{} Unable to prepare the camera logging.", logPrefix);
        return false;
    }

    // prepare real time streaming
    if (!this->prepareRTStreaming())
    {
        log()->error("{} Unable to prepare the real time streaming.", logPrefix);
        return false;
    }

    // start the thread
    if (!this->start())
    {
        log()->error("{} Unable to start the device.", logPrefix);
        return false;
    }

    m_recordingPrepared = true;

    log()->info("{} Recording started.", logPrefix);
    return true;
}

bool BipedalLocomotion::YarpRobotLoggerDevice::prepareRobotLogging()
{
    constexpr auto logPrefix = "[YarpRobotLoggerDevice::prepareRobotLogging]";
    if (m_robotSensorBridge == nullptr)
    {
        return true;
    }

    // this sleep is required since the sensor bridge could be not ready
    using namespace std::chrono_literals;
    BipedalLocomotion::clock().sleepFor(2000ms);

    if (!m_robotSensorBridge->getJointsList(m_jointList))
    {
        log()->error("{} Could not get the joints list.", logPrefix);
        return false;
    }

    // resize the temporary vectors
    m_jointSensorBuffer.resize(m_jointList.size());

    m_bufferManager.setDescriptionList(m_jointList);

    bool ok = true;

    // prepare the telemetry
    if (m_streamJointStates)
    {
        ok = ok && addChannel(jointStatePositionsName, m_jointList.size(), m_jointList);
        ok = ok && addChannel(jointStateVelocitiesName, m_jointList.size(), m_jointList);
        if (m_streamJointAccelerations)
        {
            ok = ok && addChannel(jointStateAccelerationsName, m_jointList.size(), m_jointList);
        }
        ok = ok && addChannel(jointStateTorquesName, m_jointList.size(), m_jointList);
    }
    if (m_streamMotorStates)
    {
        ok = ok && addChannel(motorStatePositionsName, m_jointList.size(), m_jointList);
        ok = ok && addChannel(motorStateVelocitiesName, m_jointList.size(), m_jointList);
        ok = ok && addChannel(motorStateAccelerationsName, m_jointList.size(), m_jointList);
        ok = ok && addChannel(motorStateCurrentsName, m_jointList.size(), m_jointList);
        if (m_streamMotorTemperature)
        {
            ok = ok && addChannel(motorStateTemperaturesName, m_jointList.size(), m_jointList);
        }
    }

    if (m_streamMotorPWM)
    {
        ok = ok && addChannel(motorStatePwmName, m_jointList.size(), m_jointList);
    }

    if (m_streamPIDs)
    {
        ok = ok && addChannel(motorStatePidsName, m_jointList.size(), m_jointList);
    }

    if (m_streamFTSensors)
    {
        for (const auto& sensorName : m_robotSensorBridge->getSixAxisForceTorqueSensorsList())
        {
            std::string fullFTSensorName = ftsName + treeDelim + sensorName;
            ok = ok && addChannel(fullFTSensorName, ftElementNames.size(), ftElementNames);
        }
    }

    if (m_streamInertials)
    {
        for (const auto& sensorName : m_robotSensorBridge->getGyroscopesList())
        {
            std::string fullGyroSensorName = gyrosName + treeDelim + sensorName;
            ok = ok && addChannel(fullGyroSensorName, gyroElementNames.size(), gyroElementNames);
        }

        for (const auto& sensorName : m_robotSensorBridge->getLinearAccelerometersList())
        {
            std::string fullAccelerometerSensorName = accelerometersName + treeDelim + sensorName;
            ok = ok
                 && addChannel(fullAccelerometerSensorName,
                               accelerometerElementNames.size(), //
                               accelerometerElementNames);
        }

        for (const auto& sensorName : m_robotSensorBridge->getOrientationSensorsList())
        {
            std::string fullOrientationsSensorName = orientationsName + treeDelim + sensorName;
            ok = ok
                 && addChannel(fullOrientationsSensorName,
                               orientationElementNames.size(), //
                               orientationElementNames);
        }

        for (const auto& sensorName : m_robotSensorBridge->getMagnetometersList())
        {
            std::string fullMagnetometersSensorName = magnetometersName + treeDelim + sensorName;
            ok = ok
                 && addChannel(fullMagnetometersSensorName,
                               magnetometerElementNames.size(), //
                               magnetometerElementNames);
        }

    }

    if (m_streamCartesianWrenches)
    {
        for (const auto& cartesianWrenchName : m_robotSensorBridge->getCartesianWrenchesList())
        {
            std::string fullCartesianWrenchName
                = cartesianWrenchesName + treeDelim + cartesianWrenchName;
            ok = ok
                 && addChannel(fullCartesianWrenchName,
                               cartesianWrenchNames.size(), //
                               cartesianWrenchNames);
        }
    }

    if (m_streamTemperatureSensors)
    {
        for (const auto& sensorName : m_robotSensorBridge->getTemperatureSensorsList())
        {
            std::string fullTemperatureSensorName = temperatureName + treeDelim + sensorName;
            ok = ok
                 && addChannel(fullTemperatureSensorName,
                               temperatureNames.size(), //
                               temperatureNames);
        }
    }

    if (!ok)
    {
        log()->error("{} Unable to add the channels for the robot sensors.", logPrefix);
        return false;
    }

    return true;
}

bool BipedalLocomotion::YarpRobotLoggerDevice::prepareCameraLogging()
{
    constexpr auto logPrefix = "[YarpRobotLoggerDevice::prepareCameraLogging]";
    // The user can avoid to record the camera
    if (m_cameraBridge == nullptr)
    {
        return true;
    }

    bool ok = m_cameraBridge->getRGBCamerasList(m_rgbCamerasList);
    if (!ok)
    {
        log()->error("{} Unable to get the list of RGB cameras.", logPrefix);
        return false;
    }
    for (const auto& camera : m_rgbCamerasList)
    {
        if (m_videoWriters[camera].rgb->saveMode == VideoWriter::SaveMode::Video)
        {
            if (!this->openVideoWriter(m_videoWriters[camera].rgb,
                                       camera,
                                       "rgb",
                                       m_cameraBridge->getMetaData().bridgeOptions.rgbImgDimensions))
            {
                log()->error("{} Unable open the video writer for the camera named {}.",
                             logPrefix,
                             camera);
                return false;
            }
        } else
        {
            if (!this->createFramesFolder(m_videoWriters[camera].rgb, camera, "rgb"))
            {
                log()->error("{} Unable to create the folder to store the frames for the "
                             "camera named {}.",
                             logPrefix,
                             camera);
                return false;
            }
        }
        ok = !m_recordingPrepared
             ? m_bufferManager.addChannel({"camera::" + camera + "::rgb",
                                           {1, 1}, //
                                           {"frame_index"}})
             : true;
        if (!ok)
        {
            log()->error("{} Unable to add the channel for the camera named {}.",
                         logPrefix,
                         camera);
            return false;
        }
    }

    ok = m_cameraBridge->getRGBDCamerasList(m_rgbdCamerasList);
    if (!ok)
    {
        log()->error("{} Unable to get the list of RGBD cameras.", logPrefix);
        return false;
    }
    for (const auto& camera : m_rgbdCamerasList)
    {
        if (m_videoWriters[camera].rgb->saveMode == VideoWriter::SaveMode::Video)
        {
            if (!this->openVideoWriter(m_videoWriters[camera].rgb,
                                       camera,
                                       "rgb",
                                       m_cameraBridge->getMetaData()
                                           .bridgeOptions.rgbdImgDimensions))
            {
                log()->error("{} Unable open the video writer for the rgbd camera named {}.",
                             logPrefix,
                             camera);
                return false;
            }
        } else
        {
            if (!this->createFramesFolder(m_videoWriters[camera].rgb, camera, "rgb"))
            {
                log()->error("{} Unable to create the folder to store the frames for the "
                             "camera named {}.",
                             logPrefix,
                             camera);
                return false;
            }
        }
        if (m_videoWriters[camera].depth->saveMode == VideoWriter::SaveMode::Video)
        {
            if (!this->openVideoWriter(m_videoWriters[camera].depth,
                                       camera,
                                       "depth",
                                       m_cameraBridge->getMetaData()
                                           .bridgeOptions.rgbdImgDimensions))
            {
                log()->error("{} Unable open the video writer for the rgbd camera named {}.",
                             logPrefix,
                             camera);
                return false;
            }
        } else
        {
            if (!this->createFramesFolder(m_videoWriters[camera].depth, camera, "depth"))
            {
                log()->error("{} Unable to create the folder to store the frames for the "
                             "camera named {}.",
                             logPrefix,
                             camera);
                return false;
            }
        }

        if (!m_recordingPrepared)
        {
            ok = m_bufferManager.addChannel({"camera::" + camera + "::rgb",
                                             {1, 1}, //
                                             {"frame_index"}});

            ok = ok
                 && m_bufferManager.addChannel({"camera::" + camera + "::depth",
                                                {1, 1}, //
                                                {"frame_index"}});
        }

        if (!ok)
        {
            log()->error("{} Unable to add the channels for the rgbd camera named {}.",
                         logPrefix,
                         camera);
            return false;
        }
    }

    // using C++17 it is not possible to use a structured binding in the for loop, i.e. for
    // (auto& [key, val] : m_videoWriters) since Lambda implicit capture fails with variable
    // declared from structured binding.
    // As explained in http://www.open-std.org/jtc1/sc22/wg21/docs/papers/2017/p0588r1.html
    // If a lambda-expression [...] captures a structured binding (explicitly or
    // implicitly), the program is ill-formed.
    // you can find further information here:
    // https://stackoverflow.com/questions/46114214/lambda-implicit-capture-fails-with-variable-declared-from-structured-binding
    // Note if one day we will support c++20 we can use structured binding see
    // https://en.cppreference.com/w/cpp/language/structured_binding
    for (auto iter = m_videoWriters.begin(); iter != m_videoWriters.end(); ++iter)
    {
        // start a separate the thread for each camera
        iter->second.videoThread
            = std::thread([this, iter] { this->recordVideo(iter->first, iter->second); });
    }

    return true;
}

bool BipedalLocomotion::YarpRobotLoggerDevice::prepareExogenousImageLogging()
{
    constexpr auto logPrefix = "[YarpRobotLoggerDevice::prepareExogenousImageLogging]";
    bool ok = true;
    for (auto& [name, signal] : m_imageSignals)
    {
        // Print some info
        log()->info("{} Preparing the logging for the exogenous image named {}, signal: {}.",
                    logPrefix,
                    name,
                    signal.signalName);
        // Equivalent to populateCamerasData for exogenous images
        auto saver = std::make_shared<VideoWriter::ImageSaver>();
        saver->saveMode = VideoWriter::SaveMode::Frame; // We assume the exogenous images are
                                                        // spurious frames
        m_exogenousImageWriters[signal.signalName].rgb = saver;

        // Equivalent to prepareCameraLogging for exogenous images
        if (!this->createFramesFolder(saver, signal.signalName, "rgb"))
        {
            log()->error("{} Unable to create the folder to store the frames for the exogenous "
                         "image named {}.",
                         logPrefix,
                         signal.signalName);
            ok = false;
        }
        if (!m_recordingPrepared)
        {
            ok = m_bufferManager.addChannel({"exogenous_images::" + signal.signalName + "::rgb",
                                             {1, 1}, //
                                             {"frame_index"}});
        }

        if (!ok)
        {
            log()->error("{} Unable to add the channels for the exogenous images signal {}.",
                         logPrefix,
                         signal.signalName);
            return false;
        }
    }

    return true;
}

bool BipedalLocomotion::YarpRobotLoggerDevice::prepareRTStreaming()
{
    constexpr auto logPrefix = "[YarpRobotLoggerDevice::prepareRTStreaming]";
    if (!m_sendDataRT)
    {
        return true;
    }

    char* tmp = std::getenv("YARP_ROBOT_NAME");
    std::string metadataName = robotRtRootName + treeDelim + robotName;
    m_vectorCollectionRTDataServer.populateMetadata(metadataName, {std::string(tmp)});

    if (m_jointList.size() > 0)
    {
        std::string rtMetadataName = robotRtRootName + treeDelim + robotDescriptionList;
        m_vectorCollectionRTDataServer.populateMetadata(rtMetadataName, m_jointList);
    }

    metadataName = robotRtRootName + treeDelim + timestampsName;
    m_vectorCollectionRTDataServer.populateMetadata(metadataName, {timestampsName});

    return true;
}

bool YarpRobotLoggerDevice::openVideoWriter(
    std::shared_ptr<VideoWriter::ImageSaver> imageSaver,
    const std::string& camera,
    const std::string& imageType,
    const std::unordered_map<std::string, std::pair<std::size_t, std::size_t>>& imgDimensions)
{
    constexpr auto logPrefix = "[YarpRobotLoggerDevice::openVideoWriter]";
    if (imageSaver == nullptr)
    {
        log()->error("{} It seems that the camera named {} do not support {}. This shouldn't be "
                     "possible.",
                     logPrefix,
                     camera,
                     imageType);
        return false;
    }

    const auto imgDimension = imgDimensions.find(camera);
    const auto videoWriter = m_videoWriters.find(camera);

    if (imgDimension == imgDimensions.cend() || videoWriter == m_videoWriters.cend())
    {
        log()->error("{} Unable to find the dimension of the image or the video writers for the "
                     "camera named {}.",
                     logPrefix,
                     camera);
        return false;
    }

    std::lock_guard guard(imageSaver->mutex);
    imageSaver->writer
        = std::make_shared<cv::VideoWriter>("output_" + camera + "_" + imageType + ".mp4",
                                            cv::VideoWriter::fourcc(m_videoCodecCode.at(0),
                                                                    m_videoCodecCode.at(1),
                                                                    m_videoCodecCode.at(2),
                                                                    m_videoCodecCode.at(3)),
                                            videoWriter->second.fps,
                                            cv::Size(imgDimension->second.first,
                                                     imgDimension->second.second),
                                            "rgb" == imageType);
    return true;
}

bool YarpRobotLoggerDevice::createFramesFolder(std::shared_ptr<VideoWriter::ImageSaver> imageSaver,
                                               const std::string& camera,
                                               const std::string& imageType)
{
    namespace fs = std::filesystem;

    constexpr auto logPrefix = "[YarpRobotLoggerDevice::createFramesFolder]";
    if (imageSaver == nullptr)
    {
        log()->error("{} It seems that the camera named {} do not support {}. This shouldn't be "
                     "possible.",
                     logPrefix,
                     camera,
                     imageType);
        return false;
    }

    imageSaver->framesPath = "output_" + camera + "_" + imageType;
    std::lock_guard guard(imageSaver->mutex);
    std::filesystem::create_directory(imageSaver->framesPath);
    return true;
}

void YarpRobotLoggerDevice::lookForExogenousSignals()
{
    yarp::profiler::NetworkProfiler::ports_name_set yarpPorts;

    using namespace std::chrono_literals;

    auto time = BipedalLocomotion::clock().now();
    auto oldTime = time;
    auto wakeUpTime = time;
    const std::chrono::nanoseconds lookForExogenousSignalPeriod = 1s;
    m_lookForNewExogenousSignalIsRunning = true;

    auto connectToExogeneous = [this](auto& signals) -> void {
        for (auto& [name, signal] : signals)
        {
            if (signal.connected)
            {
                continue;
            }

            bool connectionDone = false;

            {
                std::lock_guard<std::mutex> lock(signal.mutex);

                connectionDone = signal.connect();
            }

            signal.connected = connectionDone;
        }
    };

    auto checkExogeneousConnections
        = [this](
              std::unordered_map<std::string, VectorsCollectionSignal>& signals) -> void {
        for (auto& [name, signal] : signals)
        {
            if (!signal.connected)
            {
                continue;
            }

            std::lock_guard<std::mutex> lock(signal.mutex);
            if (!signal.client.checkConnection())
            {
                log()->warn("[YarpRobotLoggerDevice::lookForExogenousSignals] Connection lost "
                            "for exogenous signal '{}'. Will attempt to reconnect.",
                            name);
                signal.connected = false;
                signal.dataArrived = false;
                signal.metadata.vectors.clear();
            }
        }
    };

    while (m_lookForNewExogenousSignalIsRunning)
    {
        // detect if a clock has been reset
        oldTime = time;
        time = BipedalLocomotion::clock().now();

        // if the current time is lower than old time, the timer has been reset.
        if ((time - oldTime).count() < 1e-12)
        {
            wakeUpTime = time;
        }
        wakeUpTime += lookForExogenousSignalPeriod;

        // try to connect to the exogenous signals
        connectToExogeneous(m_vectorsCollectionSignals);
        connectToExogeneous(m_vectorSignals);
        connectToExogeneous(m_stringSignals);
        connectToExogeneous(m_humanStateSignals);
        connectToExogeneous(m_wearableTargetsSignals);
        connectToExogeneous(m_wearableDataSignals);
        connectToExogeneous(m_imageSignals);

        // check if already connected VectorsCollection signals are still alive
        checkExogeneousConnections(m_vectorsCollectionSignals);

        // Start the logging for exogenous images
        for (auto& [name, signal] : m_imageSignals)
        {
            if (!signal.connected
                || m_exogenousImageWriters[signal.signalName].videoThread.joinable())
            {
                continue;
            }

            // start a separate thread for each exogenous image signal
            auto& writer = m_exogenousImageWriters[signal.signalName];

            writer.videoThread = std::thread([this, name, &writer, &signal] {
                this->saveExogenousImages(name, writer, signal);
            });
        }

        // release the CPU
        BipedalLocomotion::clock().yield();

        // sleep
        BipedalLocomotion::clock().sleepUntil(wakeUpTime);
    }
}

bool YarpRobotLoggerDevice::hasSubstring(const std::string& str,
                                         const std::vector<std::string>& substrings) const
{
    for (const auto& substring : substrings)
    {
        if (str.find(substring) != std::string::npos)
        {
            return true;
        }
    }
    return false;
}

void YarpRobotLoggerDevice::lookForNewLogs()
{
    using namespace std::chrono_literals;
    yarp::profiler::NetworkProfiler::ports_name_set yarpPorts;
    constexpr auto textLoggingPortPrefix = "/log/";

    auto time = BipedalLocomotion::clock().now();
    auto oldTime = time;
    auto wakeUpTime = time;
    const auto lookForNewLogsPeriod = 2s;
    m_lookForNewLogsIsRunning = true;

    while (m_lookForNewLogsIsRunning)
    {
        // detect if a clock has been reset
        oldTime = time;
        time = BipedalLocomotion::clock().now();
        // if the current time is lower than old time, the timer has been reset.
        if ((time - oldTime).count() < 1e-12)
        {
            wakeUpTime = time;
        }
        wakeUpTime += lookForNewLogsPeriod;

        // check for new messages
        yarp::profiler::NetworkProfiler::getPortsList(yarpPorts);

        // make a new scope for lock guarding text logging port
        {
            for (const auto& port : yarpPorts)
            {
                // check if the port has not be already connected if exits, its resposive
                // it is a text logging port and it should be logged
                // This operation does not require a lock since it is not touching the port
                // object as the connection operation is done through yarpserver and not through
                // the port directly. YARP inside will take care of the connection.
                if ((port.name.rfind(textLoggingPortPrefix, 0) == 0)
                    && (m_textLoggingPortNames.find(port.name) == m_textLoggingPortNames.end())
                    && (m_textLoggingSubnames.empty()
                        || this->hasSubstring(port.name, m_textLoggingSubnames))
                    && yarp::os::Network::exists(port.name))
                {
                    m_textLoggingPortNames.insert(port.name);
                    yarp::os::Network::connect(port.name, m_textLoggingPortName, "udp");
                }
            }
        } // end of scope for lock guarding text logging port

        // release the CPU
        BipedalLocomotion::clock().yield();

        // sleep
        BipedalLocomotion::clock().sleepUntil(wakeUpTime);
    }
}

void YarpRobotLoggerDevice::recordVideo(const std::string& cameraName, VideoWriter& writer)
{
    constexpr auto logPrefix = "[YarpRobotLoggerDevice::recordVideo]";

    auto time = BipedalLocomotion::clock().now();
    auto oldTime = time;
    auto wakeUpTime = time;
    writer.recordVideoIsRunning = true;
    const auto recordVideoPeriod = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / double(writer.fps)));

    while (writer.recordVideoIsRunning)
    {
        // detect if a clock has been reset
        oldTime = time;
        time = BipedalLocomotion::clock().now();
        // if the current time is lower than old time, the timer has been reset.
        if ((time - oldTime).count() < 1e-12)
        {
            wakeUpTime = time;
        }
        wakeUpTime += recordVideoPeriod;
        if (wakeUpTime < time)
        {
            // Before acquiring the image we already spent more time than expected.
            // At this point the next frame should be acquired considering the nominal period
            // starting from the current time, as if we reset the clock.
            wakeUpTime = time + recordVideoPeriod;
        }

        if (writer.requestPause)
        {
            writer.paused = true;
        }

        if (writer.paused)
        {
            // if the recording is paused we just wait for the next iteration
            BipedalLocomotion::clock().sleepUntil(wakeUpTime);
            continue;
        }

        if (writer.resetIndex)
        {
            writer.frameIndex = 0;
            writer.resetIndex = false;
        }

        if (!writer.recordVideoIsRunning)
        {
            break;
        }

        // get the frame from the camera
        if (writer.rgb != nullptr)
        {
            if (!m_cameraBridge->getColorImage(cameraName, writer.rgb->frame))
            {
                log()->info("{} Unable to get the frame of the camera named: {}. The "
                            "previous "
                            "frame "
                            "will be used.",
                            logPrefix,
                            cameraName);
            }

            std::lock_guard<std::mutex> lock(writer.rgb->mutex);

            // save the frame in the video writer
            if (writer.rgb->saveMode == VideoWriter::SaveMode::Video)
            {
                writer.rgb->writer->write(writer.rgb->frame);
            } else
            {
                assert(writer.rgb->saveMode == VideoWriter::SaveMode::Frame);

                unsigned int frameIndex = writer.frameIndex.load();
                const std::filesystem::path imgPath
                    = writer.rgb->framesPath / ("img_" + std::to_string(frameIndex) + ".png");

                cv::imwrite(imgPath.string(), writer.rgb->frame);

                // lock the the buffered manager mutex
                std::lock_guard lock(m_bufferManagerMutex);

                // TODO here we may save the frame itself
                m_bufferManager.push_back(frameIndex,
                                            std::chrono::duration<double>(time).count(),
                                            "camera::" + cameraName + "::rgb");
            }
        }

        if (writer.depth != nullptr)
        {
            if (!m_cameraBridge->getDepthImage(cameraName, writer.depth->frame))
            {
                log()->info("{} Unable to get the frame of the camera named: {}. The "
                            "previous "
                            "frame "
                            "will be used.",
                            logPrefix,
                            cameraName);

            } else
            {
                // If a new frame arrived the we should scale it
                writer.depth->frame = writer.depth->frame * writer.depthScale;
            }

            std::lock_guard<std::mutex> lock(writer.depth->mutex);

            if (writer.depth->saveMode == VideoWriter::SaveMode::Video)
            {
                // we need to convert the image to 8bit this is required by the video writer
                cv::Mat image8Bit;
                writer.depth->frame.convertTo(image8Bit, CV_8UC1);

                // save the frame in the video writer
                writer.depth->writer->write(image8Bit);
            } else
            {
                assert(writer.depth->saveMode == VideoWriter::SaveMode::Frame);

                unsigned int frameIndex = writer.frameIndex.load();
                const std::filesystem::path imgPath
                    = writer.depth->framesPath / ("img_" + std::to_string(frameIndex) + ".png");

                // convert the image into 16bit grayscale image
                cv::Mat image16Bit;
                writer.depth->frame.convertTo(image16Bit, CV_16UC1);
                cv::imwrite(imgPath.string(), image16Bit);

                // lock the the buffered manager mutex
                std::lock_guard lock(m_bufferManagerMutex);

                // TODO here we may save the frame itself
                m_bufferManager.push_back(frameIndex,
                                            std::chrono::duration<double>(time).count(),
                                            "camera::" + cameraName + "::depth");
            }
        }

        // increase the index
        writer.frameIndex++;

        // release the CPU
        BipedalLocomotion::clock().yield();
        auto endTime = BipedalLocomotion::clock().now();
        if (wakeUpTime < endTime)
        {
            log()->info("{} The video thread spent more time than expected to save the camera "
                        "named: {}. Expected: {}. Measured: {}. Nominal: {}.",
                        logPrefix,
                        cameraName,
                        std::chrono::duration<double>(wakeUpTime - time),
                        std::chrono::duration<double>(endTime - time),
                        std::chrono::duration<double>(recordVideoPeriod));
        }

        // sleep
        BipedalLocomotion::clock().sleepUntil(wakeUpTime);
    }
}

void YarpRobotLoggerDevice::saveExogenousImages(
    const std::string& signalName,
    VideoWriter& writer,
    ExogenousSignal<yarp::sig::ImageOf<yarp::sig::PixelRgb>>& signal)
{
    constexpr auto logPrefix = "[YarpRobotLoggerDevice::saveExogenousImages]";

    writer.recordVideoIsRunning = true;

    while (writer.recordVideoIsRunning)
    {
        //Notify the other threads that we are somehow blocked
        writer.paused = true;

        std::lock_guard<std::mutex> lock(signal.mutex);
        // Blocking read, so we save only when a new frame arrives
        yarp::sig::ImageOf<yarp::sig::PixelRgb>* yarpImage = signal.port.read(true);

        // Once we got the frame we are not blocked anymore
        // unless requested
        writer.paused = writer.requestPause.load();

        if (writer.paused)
        {
            continue;
        }

        if (writer.resetIndex)
        {
            writer.frameIndex = 0;
            writer.resetIndex = false;
        }

        auto time = BipedalLocomotion::clock().now();

        if (yarpImage != nullptr)
        {
            // Convert the frame from yarp to cv
            auto colorImg = cv::Mat(yarpImage->height(),
                                    yarpImage->width(),
                                    yarp::cv::type_code<yarp::sig::PixelRgb>::value,
                                    yarpImage->getRawImage(),
                                    yarpImage->getRowSize());

            // Convert from RGB to BGR for OpenCV compatibility
            cv::cvtColor(colorImg, colorImg, cv::COLOR_RGB2BGR);

            // Lock the image saver mutex
            std::lock_guard<std::mutex> imageLock(writer.rgb->mutex);

            unsigned int frameIndex = writer.frameIndex.load();
            // Save the frame
            const std::filesystem::path imgPath
                = writer.rgb->framesPath
                  / ("img_" + std::to_string(frameIndex)
                     + ".png");
            cv::imwrite(imgPath.string(), colorImg);

            // lock the the buffered manager mutex
            std::lock_guard bufferLock(m_bufferManagerMutex);

            // TODO here we may save the frame itself
            m_bufferManager.push_back(frameIndex,
                                      std::chrono::duration<double>(time).count(),
                                      "exogenous_images::" + signal.signalName + "::rgb");
            writer.frameIndex++;
        }
    }
}

void BipedalLocomotion::YarpRobotLoggerDevice::saveCodeStatus(const std::string& logPrefix,
                                                              const std::string& fileName) const
{
    if (!m_logCodeStatus)
    {
        // Do nothing
        return;
    }

    log()->info("{} Saving the status of the code...", logPrefix);
    auto start = BipedalLocomotion::clock().now();

    for (const auto& cmdTemplate : m_codeStatusCmds)
    {
        std::string cmd = cmdTemplate;
        // Replace {filename} placeholder with the actual file prefix
        findAndReplaceAll(cmd, "{filename}", fileName);

        log()->info("{} Running code status command: {}", logPrefix, cmd);

        std::stringstream processStream;
        TinyProcessLib::Process process(cmd, "", [&](const char* bytes, size_t n) -> void {
            processStream << std::string(bytes, n);
        });
        auto exitStatus = process.get_exit_status();
        if (exitStatus != 0)
        {
            log()->warn("{} Code status command '{}' exited with status {}. Output: {}",
                        logPrefix,
                        cmd,
                        exitStatus,
                        processStream.str());
        }
    }

    log()->info("{} Status of the code saved in {}.",
                logPrefix,
                std::chrono::duration<double>(BipedalLocomotion::clock().now() - start));
}

void YarpRobotLoggerDevice::run()
{
    auto logData = [this](const std::string& name, const auto& data, const double time) {
        m_bufferManager.push_back(data, time, name);
        std::string rtName = robotRtRootName + treeDelim + name;
        if (m_sendDataRT)
        {
            m_vectorCollectionRTDataServer.populateData(rtName, data);
        }
    };

    constexpr auto logPrefix = "[YarpRobotLoggerDevice::run]";
    const std::chrono::nanoseconds t = BipedalLocomotion::clock().now();

    if (!m_firstRun)
    {
        // This is to check if something happened with the clock.
        // When the clock is reset, especially in simulation, the time difference
        // between two consecutive run could be very big.
        // This effectively stops the logging until the next valid timestamp
        if (t - m_previousTimestamp > m_acceptableStep)
        {
            log()->warn("{} The time step is too big. The previous timestamp is {} and the "
                        "current "
                        "timestamp is {}. The time step is {}.",
                        logPrefix,
                        std::chrono::duration<double>(m_previousTimestamp),
                        std::chrono::duration<double>(t),
                        std::chrono::duration<double>(t - m_previousTimestamp));
            return;
        }
    }

    if (m_requestPause)
    {
        m_paused = true;
    }
    if (m_paused)
    {
        m_previousTimestamp = t;
        m_firstRun = false;
        return;
    }

    const double time = std::chrono::duration<double>(t).count();
    std::string signalFullName = "";
    std::string rtSignalFullName = "";

    std::lock_guard lock(m_bufferManagerMutex);
    if (m_sendDataRT)
    {
        m_vectorCollectionRTDataServer.prepareData();
        m_vectorCollectionRTDataServer.clearData();
        Eigen::Matrix<double, 1, 1> timeData;
        timeData << time;
        rtSignalFullName = robotRtRootName + treeDelim + timestampsName;
        m_vectorCollectionRTDataServer.populateData(rtSignalFullName, timeData);
    }

    if (m_robotSensorBridge != nullptr)
    {
        // get the data
        if (!m_robotSensorBridge->advance())
        {
            log()->error("{} Could not advance sensor bridge.", logPrefix);
        }
    }

    if (m_streamJointStates)
    {
        if (m_robotSensorBridge->getJointPositions(m_jointSensorBuffer))
        {
            logData(jointStatePositionsName, m_jointSensorBuffer, time);
        }
        if (m_robotSensorBridge->getJointVelocities(m_jointSensorBuffer))
        {
            logData(jointStateVelocitiesName, m_jointSensorBuffer, time);
        }
        if (m_streamJointAccelerations)
        {
            if (m_robotSensorBridge->getJointAccelerations(m_jointSensorBuffer))
            {
                logData(jointStateAccelerationsName, m_jointSensorBuffer, time);
            }
        }
        if (m_robotSensorBridge->getJointTorques(m_jointSensorBuffer))
        {
            logData(jointStateTorquesName, m_jointSensorBuffer, time);
        }
    }

    if (m_streamMotorStates)
    {
        if (m_robotSensorBridge->getMotorPositions(m_jointSensorBuffer))
        {
            logData(motorStatePositionsName, m_jointSensorBuffer, time);
        }
        if (m_robotSensorBridge->getMotorVelocities(m_jointSensorBuffer))
        {
            logData(motorStateVelocitiesName, m_jointSensorBuffer, time);
        }
        if (m_robotSensorBridge->getMotorAccelerations(m_jointSensorBuffer))
        {
            logData(motorStateAccelerationsName, m_jointSensorBuffer, time);
        }
        if (m_robotSensorBridge->getMotorCurrents(m_jointSensorBuffer))
        {
            logData(motorStateCurrentsName, m_jointSensorBuffer, time);
        }

        if (m_streamMotorTemperature)
        {
            if (m_robotSensorBridge->getMotorTemperatures(m_jointSensorBuffer))
            {
                logData(motorStateTemperaturesName, m_jointSensorBuffer, time);
            }
        }
    }

    if (m_streamMotorPWM)
    {
        if (m_robotSensorBridge->getMotorPWMs(m_jointSensorBuffer))
        {
            logData(motorStatePwmName, m_jointSensorBuffer, time);
        }
    }

    if (m_streamPIDs)
    {
        if (m_robotSensorBridge->getPidPositions(m_jointSensorBuffer))
        {
            logData(motorStatePidsName, m_jointSensorBuffer, time);
        }
    }

    if (m_streamFTSensors)
    {
        for (const auto& sensorName : m_robotSensorBridge->getSixAxisForceTorqueSensorsList())
        {
            if (m_robotSensorBridge->getSixAxisForceTorqueMeasurement(sensorName, m_ftBuffer))
            {
                signalFullName = ftsName + treeDelim + sensorName;
                logData(signalFullName, m_ftBuffer, time);
            }
        }
    }

    if (m_streamTemperatureSensors)
    {
        for (const auto& sensorName : m_robotSensorBridge->getTemperatureSensorsList())
        {
            if (m_robotSensorBridge->getTemperature(sensorName, m_ftTemperatureBuffer))
            {
                signalFullName = temperatureName + treeDelim + sensorName;

                Eigen::Matrix<double, 1, 1> temperatureData;
                temperatureData << m_ftTemperatureBuffer;
                logData(signalFullName, temperatureData, time);
            }
        }
    }

    if (m_streamInertials)
    {
        for (const auto& sensorName : m_robotSensorBridge->getGyroscopesList())
        {
            if (m_robotSensorBridge->getGyroscopeMeasure(sensorName, m_gyroBuffer))
            {
                signalFullName = gyrosName + treeDelim + sensorName;
                logData(signalFullName, m_gyroBuffer, time);
            }
        }

        // pack the data for the accelerometer
        for (const auto& sensorName : m_robotSensorBridge->getLinearAccelerometersList())
        {
            if (m_robotSensorBridge->getLinearAccelerometerMeasurement(sensorName,
                                                                       m_acceloremeterBuffer))
            {
                signalFullName = accelerometersName + treeDelim + sensorName;
                logData(signalFullName, m_acceloremeterBuffer, time);
            }
        }

        // pack the data for the orientations
        for (const auto& sensorName : m_robotSensorBridge->getOrientationSensorsList())
        {
            if (m_robotSensorBridge->getOrientationSensorMeasurement(sensorName,
                                                                     m_orientationBuffer))
            {
                signalFullName = orientationsName + treeDelim + sensorName;
                logData(signalFullName, m_orientationBuffer, time);
            }
        }

        for (const auto& sensorName : m_robotSensorBridge->getMagnetometersList())
        {
            if (m_robotSensorBridge->getMagnetometerMeasurement(sensorName, m_magnemetometerBuffer))
            {
                signalFullName = magnetometersName + treeDelim + sensorName;
                logData(signalFullName, m_magnemetometerBuffer, time);
            }
        }

    }

    if (m_streamCartesianWrenches)
    {
        for (const auto& cartesianWrenchName : m_robotSensorBridge->getCartesianWrenchesList())
        {
            if (m_robotSensorBridge->getCartesianWrench(cartesianWrenchName, m_ftBuffer))
            {
                signalFullName = cartesianWrenchesName + treeDelim + cartesianWrenchName;
                logData(signalFullName, m_ftBuffer, time);
            }
        }
    }

    for (auto& [name, signal] : m_vectorsCollectionSignals)
    {
        if (!signal.connected)
        {
            continue;
        }

        std::lock_guard<std::mutex> lock(signal.mutex);
        const BipedalLocomotion::YarpUtilities::VectorsCollection* collection
            = signal.client.readData(false);

        if (collection != nullptr)
        {
            if (!signal.dataArrived)
            {
                bool channelAdded = true;
                // Fetch metadata on-demand if not yet available
                if (signal.metadata.vectors.empty())
                {
                    if (signal.client.getMetadata(signal.metadata))
                    {
                        log()->info("{} Fetched metadata on-demand for exogenous signal group: {}",
                                    logPrefix,
                                    signal.signalName);
                    }
                }
                for (const auto& [key, vector] : collection->vectors)
                {
                    signalFullName = signal.signalName + treeDelim + key;
                    const auto& metadata = signal.metadata.vectors.find(key);
                    if (metadata == signal.metadata.vectors.cend())
                    {
                        // Metadata not available yet, skip adding channel for now
                        log()->warn("{} Metadata not available yet for signal {}. Skipping channel "
                                    "addition.",
                                    logPrefix,
                                    signalFullName);
                        channelAdded = false;
                    } else
                    {
                        log()->info("{} Found metadata for the exogenous signal: {}.",
                                    logPrefix,
                                    signalFullName);
                        channelAdded
                            = channelAdded
                              && addChannel(signalFullName, vector.size(), {metadata->second});
                    }
                }
                signal.dataArrived = channelAdded;
            }
            if (signal.dataArrived)
            {
                for (const auto& [key, vector] : collection->vectors)
                {
                    signalFullName = signal.signalName + treeDelim + key;
                    logData(signalFullName, vector, time);
                }
            }
        }
    }

    for (auto& [name, signal] : m_vectorSignals)
    {
        if (!signal.connected)
        {
            continue;
        }

        std::lock_guard<std::mutex> lock(signal.mutex);
        yarp::sig::Vector* vector = signal.port.read(false);

        signalFullName = signal.signalName;

        if (vector != nullptr)
        {
            if (!signal.dataArrived)
            {
                signal.dataArrived = addChannel(signalFullName, vector->size());
            }
            logData(signalFullName, *vector, time);
        }
    }

    // String signals are not streamed in RT
    for (auto& [name, signal] : m_stringSignals)
    {
        if (!signal.connected)
        {
            continue;
        }

        std::lock_guard<std::mutex> lock(signal.mutex);
        yarp::os::Bottle* bottle = signal.port.read(false);

        signalFullName = signal.signalName;

        if (bottle != nullptr)
        {
            if (!signal.dataArrived)
            {
                if (!m_bufferManager.addChannel({signalFullName, {1}}))
                {
                    log()->error("Failed to add the channel in buffer manager named: {}",
                                 signalFullName);
                    signal.dataArrived = false;
                    continue;
                }
                signal.dataArrived = true;
            }
            m_bufferManager.push_back(bottle->toString(), time, signalFullName);
        }
    }

    auto handleExogenousWithMetadata = [this, logData](auto& list, double time) {
        for (auto& [name, signal] : list)
        {
            if (!signal.connected)
            {
                continue;
            }

            std::lock_guard<std::mutex> lock(signal.mutex);
            auto* message = signal.port.read(false);

            if (message != nullptr)
            {
                if (!signal.dataArrived)
                {
                    extractMetadata(*message, signal.signalName, signal.metadata);
                    bool channelAdded = true;
                    for (const auto& [key, vector] : signal.metadata.vectors)
                    {
                        channelAdded &= addChannel(key, vector.size(), vector);
                    }
                    signal.dataArrived = channelAdded;
                }

                if (signal.dataArrived)
                {
                    convertToVectorsCollection(*message, signal.signalName, signal.convertedSignal);
                    for (const auto& [key, vector] : signal.convertedSignal.vectors)
                    {
                        logData(key, vector, time);
                    }
                }
            }
        }
    };

    handleExogenousWithMetadata(m_humanStateSignals, time);
    handleExogenousWithMetadata(m_wearableTargetsSignals, time);
    handleExogenousWithMetadata(m_wearableDataSignals, time);

    if (m_logText)
    {
        int bufferportSize = m_textLoggingPort.getPendingReads();
        BipedalLocomotion::TextLoggingEntry msg;

        while (bufferportSize > 0)
        {
            yarp::os::Bottle* b = m_textLoggingPort.read(false);
            if (b != nullptr)
            {
                msg = BipedalLocomotion::TextLoggingEntry::deserializeMessage(*b,
                                                                              std::to_string(time));
                if (msg.isValid)
                {
                    signalFullName = msg.portSystem + "::" + msg.portPrefix + "::" + msg.processName
                                     + "::p" + msg.processPID;

                    // matlab does not support the character - as a key of a struct
                    findAndReplaceAll(signalFullName, "-", "_");

                    // if it is the first time this signal is seen by the device the channel
                    // is added
                    if (m_textLogsStoredInManager.find(signalFullName)
                        == m_textLogsStoredInManager.end())
                    {
                        m_bufferManager.addChannel({signalFullName, {1, 1}});
                        m_textLogsStoredInManager.insert(signalFullName);
                    }
                    // Not using logData here because we don't want to stream the data to RT
                    m_bufferManager.push_back(msg, time, signalFullName);
                }
                bufferportSize = m_textLoggingPort.getPendingReads();
            } else
            {
                break;
            }
        }
    }

    if (m_logFrames)
    {
        if (updateChildTransformList())
        {
            for (const auto& frame : m_tfChildFrames)
            {
                if (!frame.second.active)
                {
                    continue;
                }
                if (m_tf->getTransform(frame.first, frame.second.parent, m_tfMatrix))
                {
                    Eigen::Matrix4d eigenMatrix = yarp::eigen::toEigen(m_tfMatrix);
                    Eigen::Vector3d position = eigenMatrix.block<3, 1>(0, 3);
                    Eigen::Quaterniond quat(eigenMatrix.block<3, 3>(0, 0));
                    Eigen::Vector4d orientationVec;
                    orientationVec << quat.x(), quat.y(), quat.z(), quat.w();
                    logData(frame.second.positionChannelName, position, time);
                    logData(frame.second.orientationChannelName, orientationVec, time);
                }
            }
        }
    }

    if (m_sendDataRT)
    {
        m_vectorCollectionRTDataServer.sendData();
    }

    // We send the current timestamp in the status port
    yarp::os::Bottle& status = m_statusPort.prepare();
    status.clear();
    status.addFloat64(time);
    m_statusPort.write();

    m_previousTimestamp = t;
    m_firstRun = false;

    // Check the duration of the run
    double runDuration
        = std::chrono::duration<double>(BipedalLocomotion::clock().now() - t).count();
    if (runDuration > getPeriod())
    {
        log()->warn("{} The run method took {} seconds which is more than the "
                    "configured period of {} seconds.",
                    logPrefix,
                    runDuration,
                    getPeriod());
    }

    yInfoThrottle(5) << "Logging data..";
}

bool YarpRobotLoggerDevice::saveCallback(const std::string& fileName,
                                         const robometry::SaveCallbackSaveMethod& method)
{
    constexpr auto logPrefix = "[YarpRobotLoggerDevice::saveCallback]";

    auto saveVideo = [&fileName, logPrefix](std::shared_ptr<VideoWriter::ImageSaver> imageSaver,
                                            const std::string& camera,
                                            const std::string& videoTypePostfix) -> bool {
        if (imageSaver == nullptr)
        {
            log()->error("{} The camera named {} do not expose the rgb image. This "
                         "should't be "
                         "possible.",
                         logPrefix,
                         camera);
            return false;
        }

        std::string temp = fileName + "_" + camera + "_" + videoTypePostfix;
        std::string oldName = "output_" + camera + "_" + videoTypePostfix;

        // release the writer
        std::lock_guard<std::mutex> lock(imageSaver->mutex);
        if (imageSaver->saveMode == VideoWriter::SaveMode::Video)
        {
            // the name of the files contains mp4
            temp += ".mp4";
            oldName += ".mp4";

            imageSaver->writer->release();
        }

        // check if temp folder already exists
        if (std::filesystem::exists(temp))
        {
            log()->error("{} Attempted to rename {} to {}, but it already exists. "
                             "Please choose a different name.",
                         logPrefix,
                         oldName,
                         temp);
            return false;
        }

        // rename the file associated to the camera
        std::filesystem::rename(oldName, temp);

        return true;
    };

    waitForAcquisitionThreadsToPause();

    // save the video if there is any
    for (const auto& camera : m_rgbCamerasList)
    {
        log()->info("{} Saving the rgb camera named {}.", logPrefix, camera);

        auto start = BipedalLocomotion::clock().now();

        if (!saveVideo(m_videoWriters[camera].rgb, camera, "rgb"))
        {
            log()->error("{} Unable to save the rgb for the camera named {}", logPrefix, camera);
            return false;
        }

        log()->info("{} Saved video {}_{}_{} in {}.",
                    logPrefix,
                    fileName,
                    camera,
                    "rgb",
                    std::chrono::duration<double>(BipedalLocomotion::clock().now() - start));

        if (method != robometry::SaveCallbackSaveMethod::periodic)
        {
            continue;
        }

        if (m_videoWriters[camera].rgb->saveMode == VideoWriter::SaveMode::Video)
        {
            if (!this->openVideoWriter(m_videoWriters[camera].rgb,
                                       camera,
                                       "rgb",
                                       m_cameraBridge->getMetaData().bridgeOptions.rgbImgDimensions))
            {
                log()->error("{} Unable to open a video writer fro the camera named {}.",
                             logPrefix,
                             camera);
                return false;
            }
        } else
        {
            if (!this->createFramesFolder(m_videoWriters[camera].rgb, camera, "rgb"))
            {
                log()->error("{} Unable to create the folder associated to the frames of "
                             "the "
                             "camera named {}.",
                             logPrefix,
                             camera);
                return false;
            }
        }
        m_videoWriters[camera].resetIndex = true;
    }

    for (const auto& camera : m_rgbdCamerasList)
    {
        log()->info("{} Saving the rgb camera named {}.", logPrefix, camera);

        auto start = BipedalLocomotion::clock().now();

        if (!saveVideo(m_videoWriters[camera].rgb, camera, "rgb"))
        {
            log()->error("{} Unable to save the rgb for the camera named {}", logPrefix, camera);
            return false;
        }

        log()->info("{} Saved video {}_{}_{} in {}.",
                    logPrefix,
                    fileName,
                    camera,
                    "rgb",
                    std::chrono::duration<double>(BipedalLocomotion::clock().now() - start));

        log()->info("{} Saving the depth camera named {}.", logPrefix, camera);

        start = BipedalLocomotion::clock().now();

        if (!saveVideo(m_videoWriters[camera].depth, camera, "depth"))
        {
            log()->error("{} Unable to save the depth for the camera named {}", logPrefix, camera);
            return false;
        }

        log()->info("{} Saved video {}_{}_{} in {}.",
                    logPrefix,
                    fileName,
                    camera,
                    "depth",
                    std::chrono::duration<double>(BipedalLocomotion::clock().now() - start));

        if (method != robometry::SaveCallbackSaveMethod::periodic)
        {
            continue;
        }

        if (m_videoWriters[camera].rgb->saveMode == VideoWriter::SaveMode::Video)
        {

            if (!this->openVideoWriter(m_videoWriters[camera].rgb,
                                       camera,
                                       "rgb",
                                       m_cameraBridge->getMetaData()
                                           .bridgeOptions.rgbdImgDimensions))
            {
                log()->error("{} Unable to open a video writer fro the camera named {}.",
                             logPrefix,
                             camera);
                return false;
            }
        } else
        {
            if (!this->createFramesFolder(m_videoWriters[camera].rgb, camera, "rgb"))
            {
                log()->error("{} Unable to create the folder associated to the frames of "
                             "the "
                             "camera named {}.",
                             logPrefix,
                             camera);
                return false;
            }
        }
        if (m_videoWriters[camera].depth->saveMode == VideoWriter::SaveMode::Video)
        {
            if (!this->openVideoWriter(m_videoWriters[camera].depth,
                                       camera,
                                       "depth",
                                       m_cameraBridge->getMetaData()
                                           .bridgeOptions.rgbdImgDimensions))
            {
                log()->error("{} Unable to open a video writer for the depth camera named "
                             "{}.",
                             logPrefix,
                             camera);
                return false;
            }
        } else
        {
            if (!this->createFramesFolder(m_videoWriters[camera].depth, camera, "depth"))
            {
                log()->error("{} Unable to create the folder associated to the depth "
                             "frames of "
                             "the "
                             "camera named {}.",
                             logPrefix,
                             camera);
                return false;
            }
        }
        m_videoWriters[camera].resetIndex = true;
    }

    // rename the exogenous images folder if any
    for (auto& [signalName, writer] : m_exogenousImageWriters)
    {
        log()->info("{} Saving the exogenous images for the signal named {}.",
                    logPrefix,
                    signalName);

        auto start = BipedalLocomotion::clock().now();

        if (!saveVideo(writer.rgb, signalName, "rgb"))
        {
            log()->error("{} Unable to save the exogenous images for the signal named {}.",
                         logPrefix,
                         signalName);
            return false;
        }

        log()->info("{} Saved exogenous images {}_{}_{} in {}.",
                    logPrefix,
                    fileName,
                    signalName,
                    "rgb",
                    std::chrono::duration<double>(BipedalLocomotion::clock().now() - start));

        if (method != robometry::SaveCallbackSaveMethod::periodic)
        {
            continue;
        }

        if (writer.rgb->saveMode == VideoWriter::SaveMode::Frame)
        {
            if (!this->createFramesFolder(writer.rgb, signalName, "rgb"))
            {
                log()->error("{} Unable to create the folder associated to the frames of "
                             "the "
                             "exogenous image signal named {}.",
                             logPrefix,
                             signalName);
                return false;
            }
        }
        writer.resetIndex = true;
    }

    if (method != robometry::SaveCallbackSaveMethod::last_call)
    {
        resumeAcquisitionThreads();
    }

    // save the status of the code
    this->saveCodeStatus(logPrefix, fileName);

    return true;
}

bool YarpRobotLoggerDevice::detachAll()
{
    if (isRunning())
    {
        stop();
    }

    return true;
}

void YarpRobotLoggerDevice::stopRecordingThreads()
{
    constexpr auto logPrefix = "[YarpRobotLoggerDevice::stopRecordingThreads]";

    // Stop the periodic thread
    if (this->isRunning())
    {
        this->stop();
    }

    // stop all the video threads
    for (auto& [cameraName, writer] : m_videoWriters)
    {
        writer.recordVideoIsRunning = false;
    }

    for (auto& [cameraName, writer] : m_videoWriters)
    {
        if (writer.videoThread.joinable())
        {
            writer.videoThread.join();
            writer.videoThread = std::thread();
        }
    }

    // stop all the exogenous image logging threads
    for (auto& [name, signal] : m_imageSignals)
    {
        m_exogenousImageWriters[signal.signalName].recordVideoIsRunning = false;
        signal.port.interrupt();
        if (m_exogenousImageWriters[signal.signalName].videoThread.joinable())
        {
            m_exogenousImageWriters[signal.signalName].videoThread.join();
            m_exogenousImageWriters[signal.signalName].videoThread = std::thread();
        }
    }

    // stop the text logging polling thread
    m_lookForNewLogsIsRunning = false;
    if (m_lookForNewLogsThread.joinable())
    {
        m_lookForNewLogsThread.join();
        m_lookForNewLogsThread = std::thread();
    }

    // close the text logging port so it can be reopened on next record() call
    m_textLoggingPort.close();

    // stop the exogenous signal polling thread
    m_lookForNewExogenousSignalIsRunning = false;
    if (m_lookForNewExogenousSignalThread.joinable())
    {
        m_lookForNewExogenousSignalThread.join();
        m_lookForNewExogenousSignalThread = std::thread();
    }

    // Reset pause state so next record() starts cleanly
    m_requestPause = false;
    m_paused = false;
    m_firstRun = true;

    log()->info("{} All recording threads stopped.", logPrefix);
}

bool BipedalLocomotion::YarpRobotLoggerDevice::stopRecording()
{
    constexpr auto logPrefix = "[YarpRobotLoggerDevice::stopRecording]";

    if (!this->isRunning())
    {
        log()->warn("{} Recording is not running.", logPrefix);
        return false;
    }

    log()->info("{} Stopping recording without saving...", logPrefix);

    stopRecordingThreads();

    log()->info("{} Recording stopped. Data was not saved. "
                "Use the RPC command 'record' to start a new recording.",
                logPrefix);

    return true;
}

bool YarpRobotLoggerDevice::close()
{
    constexpr auto logPrefix = "[YarpRobotLoggerDevice::close]";

    // If recording is active, auto-save the data before shutting down
    if (this->isRunning())
    {
        log()->info("{} Recording is active. Auto-saving episode before closing...", logPrefix);
        this->saveData("");
    } else
    {
        stopRecordingThreads();
    }

    m_rpcPort.close();
    m_statusPort.close();

    return true;
}

void BipedalLocomotion::YarpRobotLoggerDevice::waitForAcquisitionThreadsToPause()
{
    // First we request all acquisition threads to pause and we wait for them to be paused
    log()->info("[YarpRobotLoggerDevice::waitForAcquisitionThreadsToPause] Pausing acquisition threads...");
    for (auto& [cameraName, writer] : m_videoWriters)
    {
        writer.requestPause = true;
    }
    for (auto& [cameraName, writer] : m_exogenousImageWriters)
    {
        writer.requestPause = true;
    }
    m_requestPause = true;

    // Wait for all the acquisition threads to be paused
    bool allPaused = false;
    while (!allPaused)
    {
        allPaused = true;
        for (auto& [cameraName, writer] : m_videoWriters)
        {
            if (writer.recordVideoIsRunning && !writer.paused)
            {
                allPaused = false;
                break;
            }
        }
        if (allPaused)
        {
            for (auto& [cameraName, writer] : m_exogenousImageWriters)
            {
                if (writer.recordVideoIsRunning && !writer.paused)
                {
                    allPaused = false;
                    break;
                }
            }
        }
        allPaused = allPaused && (!isRunning() || m_paused.load());
        using namespace std::chrono_literals;
        BipedalLocomotion::clock().sleepFor(1ms);
    }
    log()->info("[YarpRobotLoggerDevice::waitForAcquisitionThreadsToPause] All acquisition threads paused.");
}

void BipedalLocomotion::YarpRobotLoggerDevice::resumeAcquisitionThreads()
{
    for (auto& [cameraName, writer] : m_videoWriters)
    {
        if (writer.recordVideoIsRunning)
        {
            writer.requestPause = false;
            writer.paused = false;
        }
    }
    for (auto& [cameraName, writer] : m_exogenousImageWriters)
    {
        if (writer.recordVideoIsRunning)
        {
            writer.requestPause = false;
            writer.paused = false;
        }
    }
    m_requestPause = false;
    m_paused = false;
    log()->info("[YarpRobotLoggerDevice::resumeAcquisitionThreads] Resumed acquisition threads.");
}

bool BipedalLocomotion::YarpRobotLoggerDevice::saveData(const std::string& tag)
{
    constexpr auto logPrefix = "[YarpRobotLoggerDevice::saveData]";

    if (!this->isRunning())
    {
        log()->warn("{} Recording is not running. Nothing to save.", logPrefix);
        return false;
    }

    std::string actualFileName;
    log()->info("{} Saving episode to .mat file...", logPrefix);
    auto start = BipedalLocomotion::clock().now();
    std::string inputFileName = defaultFilePrefix;
    bool output = false;
    std::chrono::nanoseconds duration;
    auto startTime = BipedalLocomotion::clock().now();

    if (!tag.empty())
    {
        std::string edited_tag = tag;

        // Check if the tag is valid
        for (size_t i = 1; i < edited_tag.size(); ++i)
        {
            if (!isalnum(edited_tag[i]) && (edited_tag[i] != '_') && (edited_tag[i] != ' '))
            {
                log()->error("{} The tag can contain only alphanumeric characters or "
                             "underscores (tag = \"{}\").",
                             logPrefix,
                             inputFileName);
                return false;
            }
        }

        // Check if the tag contains spaces. Trigger a warning if this is the case
        // and replace the spaces with underscores
        if (edited_tag.find(' ') != std::string::npos)
        {
            log()->warn("{} The tag \"{}\" contains spaces."
                        "They will be replaced with underscores.",
                        logPrefix,
                        edited_tag);
            size_t start_pos = 0;
            while ((start_pos = edited_tag.find(" ", start_pos)) != std::string::npos)
            {
                edited_tag.replace(start_pos, std::string(" ").length(), "_");
                start_pos += std::string("_").length();
            }
        }

        inputFileName = defaultFilePrefix + "_" + edited_tag;
    }

    waitForAcquisitionThreadsToPause();

    {
        std::lock_guard<std::mutex> lockBuffer(m_bufferManagerMutex);

        m_bufferManager.setFileName(inputFileName);

        // save current time
        m_bufferManager.saveToFile(actualFileName);
        // Restore the file name
        m_bufferManager.setFileName(defaultFilePrefix);

        log()->info("{} Data saved to file {}.mat in {}.",
                    logPrefix,
                    actualFileName,
                    std::chrono::duration<double>(BipedalLocomotion::clock().now() - start));
        // Use last_call since we are stopping recording after saving
        output = this->saveCallback(actualFileName, robometry::SaveCallbackSaveMethod::last_call);
    }

    // If it lasted less than 1 second we wait a bit to avoid that
    // we save files with the same name
    duration = BipedalLocomotion::clock().now() - startTime;
    using namespace std::chrono_literals;
    if (duration < 1s)
    {
        BipedalLocomotion::clock().sleepFor(1s - duration);
    }

    log()->info("{} Episode saved in {}.", logPrefix, std::chrono::duration<double>(duration));

    // Stop recording and bring device to idle mode
    stopRecordingThreads();

    log()->info("{} Recording stopped. Device is now idle. "
                "Use the RPC command 'record' to start a new recording.",
                logPrefix);

    return output;
}
