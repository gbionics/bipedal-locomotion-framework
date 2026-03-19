/**
 * @copyright 2020, 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <algorithm>
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
        if (!params->getParameter("code_status_cmd_prefixes", m_codeStatusCmdPrefixes))
        {
            log()->info("{} Unable to get the 'code_status_cmd_prefixes' parameter. No prefix will "
                        "be "
                        "added to commands.",
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
                constexpr auto fourccCodecUrl = "https://abcavi.kibi.ru/fourcc.php";
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
        return startLogging();
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

    inputs.clear();
    if (!ptr->getParameter("vectors_exogenous_inputs", inputs))
    {
        log()->error("{} Unable to get the exogenous inputs.", logPrefix);
        return false;
    }

    if (!openExogenousSignals(ptr, inputs, m_vectorSignals))
    {
        return false;
    }

    inputs.clear();
    if (!ptr->getParameter("string_exogenous_inputs", inputs))
    {
        log()->warn("{} Unable to get the string exogenous inputs. Assuming none.", logPrefix);
    }

    if (!openExogenousSignals(ptr, inputs, m_stringSignals))
    {
        return false;
    }

    inputs.clear();
    if (!ptr->getParameter("image_exogenous_inputs", inputs))
    {
        log()->warn("{} Unable to get the image exogenous inputs. Assuming none.", logPrefix);
    }

    if (!openExogenousSignals(ptr, inputs, m_imageSignals))
    {
        return false;
    }

    if (!prepareExogenousImageLogging())
    {
        log()->error("{} Unable to prepare the exogenous image logging.", logPrefix);
        return false;
    }

    inputs.clear();
    if (!ptr->getParameter("human_state_exogenous_inputs", inputs))
    {
        log()->warn("{} Unable to get the human state exogenous inputs. Assuming none.", logPrefix);
    }
    if (!openExogenousSignals(ptr, inputs, m_humanStateSignals))
    {
        return false;
    }

    inputs.clear();
    if (!ptr->getParameter("wearable_targets_exogenous_inputs", inputs))
    {
        log()->warn("{} Unable to get the wearable targets exogenous inputs. Assuming none.",
                    logPrefix);
    }
    if (!openExogenousSignals(ptr, inputs, m_wearableTargetsSignals))
    {
        return false;
    }

    inputs.clear();
    if (!ptr->getParameter("wearable_data_exogenous_inputs", inputs))
    {
        log()->warn("{} Unable to get the wearable data exogenous inputs. Assuming none.",
                    logPrefix);
    }
    if (!openExogenousSignals(ptr, inputs, m_wearableDataSignals))
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
    if (!ptr->getParameter("stream_joint_states", m_streamJointStates))
    {
        log()->info("{} The 'stream_joint_states' parameter is not found. The joint states is not "
                    "logged",
                    logPrefix);
    }

    if (!ptr->getParameter("stream_joint_accelerations", m_streamJointAccelerations))
    {
        log()->info("{} The 'stream_joint_accelerations' parameter is not found. Set to true by "
                    "default",
                    logPrefix);
    }
    if (!m_streamJointAccelerations)
    {
        log()->info("{} The joint accelerations is not logged", logPrefix);
    }

    if (!ptr->getParameter("stream_motor_temperature", m_streamMotorTemperature))
    {
        log()->info("{} The 'stream_motor_temperature' parameter is not found. The motor "
                    "temperature is not logged",
                    logPrefix);
    }
    if (!m_streamMotorTemperature)
    {
        log()->info("{} The motor temperature is not logged", logPrefix);
    }

    if (!ptr->getParameter("stream_motor_states", m_streamMotorStates))
    {
        log()->info("{} The 'stream_motor_states' parameter is not found. The motor states is not "
                    "logged",
                    logPrefix);
    }

    if (!ptr->getParameter("stream_motor_PWM", m_streamMotorPWM))
    {
        log()->info("{} The 'stream_motor_PWM' parameter is not found. The motor PWM is not logged",
                    logPrefix);
    }

    if (!ptr->getParameter("stream_pids", m_streamPIDs))
    {
        log()->info("{} The 'stream_pids' parameter is not found. The motor pid values are not "
                    "logged",
                    logPrefix);
    }

    if (!ptr->getParameter("stream_inertials", m_streamInertials))
    {
        log()->info("{} The 'stream_inertials' parameter is not found. The IMU values are not "
                    "logged",
                    logPrefix);
    }

    if (!ptr->getParameter("stream_cartesian_wrenches", m_streamCartesianWrenches))
    {
        log()->info("{} The 'stream_cartesian_wrenches' parameter is not found. The cartesian "
                    "wrench values are not "
                    "logged",
                    logPrefix);
    }

    if (!ptr->getParameter("stream_forcetorque_sensors", m_streamFTSensors))
    {
        log()->info("{} The 'stream_forcetorque_sensors' parameter is not found. The FT values are "
                    "not "
                    "logged",
                    logPrefix);
    }

    if (!ptr->getParameter("stream_temperatures", m_streamTemperatureSensors))
    {
        log()->info("{} The 'stream_temperatures' parameter is not found. The temperature sensor "
                    "values are not "
                    "logged",
                    logPrefix);
    }

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
        return startLogging();
    }

    return true;
}

bool BipedalLocomotion::YarpRobotLoggerDevice::startLogging()
{
    constexpr auto logPrefix = "[YarpRobotLoggerDevice::startLogging]";

    if (this->isRunning())
    {
        log()->error("{} The logger has already started. This should not have happened.",
                     logPrefix);
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

    log()->info("{} The logger has started.", logPrefix);
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

        // an IMU contains a gyro accelerometer and an orientation sensor
        for (const auto& sensorName : m_robotSensorBridge->getIMUsList())
        {
            std::string fullAccelerometerSensorName = accelerometersName + treeDelim + sensorName;
            std::string fullGyroSensorName = gyrosName + treeDelim + sensorName;
            std::string fullOrientationsSensorName = orientationsName + treeDelim + sensorName;
            ok = ok
                 && addChannel(fullAccelerometerSensorName,
                               accelerometerElementNames.size(), //
                               accelerometerElementNames);
            ok = ok && addChannel(fullGyroSensorName, gyroElementNames.size(), gyroElementNames);
            ok = ok
                 && addChannel(fullOrientationsSensorName,
                               orientationElementNames.size(), //
                               orientationElementNames);
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
        ok = m_bufferManager.addChannel({"camera::" + camera + "::rgb",
                                         {1, 1}, //
                                         {"frame_index"}});
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

        ok = m_bufferManager.addChannel({"camera::" + camera + "::rgb",
                                         {1, 1}, //
                                         {"frame_index"}});

        ok = ok
             && m_bufferManager.addChannel({"camera::" + camera + "::depth",
                                            {1, 1}, //
                                            {"frame_index"}});

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
    constexpr std::size_t queueBufferSeconds = 3;
    constexpr std::size_t minQueueDepth = 10;
    constexpr std::size_t fallbackFps = 30;

    for (auto iter = m_videoWriters.begin(); iter != m_videoWriters.end(); ++iter)
    {
        auto& writer = iter->second;
        const std::string cameraName = iter->first;
        const std::size_t fpsValue = writer.fps > 0 ? static_cast<std::size_t>(writer.fps)
                                                    : fallbackFps;

        writer.maxBufferedBundles
            = std::max<std::size_t>(minQueueDepth, fpsValue * queueBufferSeconds);
        writer.flushThreadShouldRun = true;
        writer.flushPaused = false;
        writer.queueOverflowNotified = false;

        writer.flushThread = std::thread(
            [this, cameraName, &writer] { this->flushVideoQueue(cameraName, writer); });

        // start a separate thread for each camera that captures frames
        writer.videoThread
            = std::thread([this, cameraName, &writer] { this->recordVideo(cameraName, writer); });
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
        ok = m_bufferManager.addChannel({"exogenous_images::" + signal.signalName + "::rgb",
                                         {1, 1}, //
                                         {"frame_index"}});

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

void YarpRobotLoggerDevice::unpackIMU(Eigen::Ref<const analog_sensor_t> signal,
                                      Eigen::Ref<accelerometer_t> accelerometer,
                                      Eigen::Ref<gyro_t> gyro,
                                      Eigen::Ref<orientation_t> orientation)
{
    // the output consists 12 double, organized as follows:
    //  euler angles [3]
    // linear acceleration [3]
    // angular speed [3]
    // magnetic field [3]
    // http://wiki.icub.org/wiki/Inertial_Sensor
    orientation = signal.segment<3>(0);
    accelerometer = signal.segment<3>(3);
    gyro = signal.segment<3>(6);
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

        // TODO check for updated metadata from already connected signals

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

        const double timeSeconds = std::chrono::duration<double>(time).count();
        VideoWriter::FrameBundle bundle;
        bundle.frameIndex = writer.frameIndex.load();
        bundle.timestamp = timeSeconds;
        bool hasFrame = false;

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

            if (!writer.rgb->frame.empty())
            {
                bundle.rgbFrame = writer.rgb->frame.clone();
                bundle.hasRgb = true;
                hasFrame = true;
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
                // If a new frame arrived then we should scale it
                writer.depth->frame = writer.depth->frame * writer.depthScale;
            }

            if (!writer.depth->frame.empty())
            {
                bundle.depthFrame = writer.depth->frame.clone();
                bundle.hasDepth = true;
                hasFrame = true;
            }
        }

        if (hasFrame)
        {
            std::unique_lock<std::mutex> queueLock(writer.frameQueueMutex);
            if (writer.maxBufferedBundles == 0)
            {
                writer.maxBufferedBundles = 1;
            }
            if (writer.frameQueue.size() >= writer.maxBufferedBundles)
            {
                writer.frameQueue.pop_front();
                if (!writer.queueOverflowNotified.exchange(true))
                {
                    log()->warn("{} Dropping camera frame for {} because the flush queue reached "
                                "{} elements.",
                                logPrefix,
                                cameraName,
                                writer.maxBufferedBundles);
                }
            }

            writer.frameQueue.push_back(std::move(bundle));
            queueLock.unlock();
            writer.frameQueueCv.notify_one();
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

void YarpRobotLoggerDevice::flushVideoQueue(const std::string& cameraName, VideoWriter& writer)
{
    const std::string rgbChannel = "camera::" + cameraName + "::rgb";
    const std::string depthChannel = "camera::" + cameraName + "::depth";

    auto queueIsOverflowing = [&writer]() -> bool {
        if (writer.maxBufferedBundles == 0)
        {
            return false;
        }

        const std::size_t threshold = std::max<std::size_t>(1, writer.maxBufferedBundles / 2);
        return writer.frameQueue.size() >= threshold;
    };

    while (true)
    {
        VideoWriter::FrameBundle bundle;

        {
            std::unique_lock<std::mutex> lock(writer.frameQueueMutex);
            writer.frameQueueCv.wait(lock, [&writer]() {
                return !writer.frameQueue.empty() || !writer.flushThreadShouldRun.load();
            });

            if (writer.frameQueue.empty())
            {
                if (!writer.flushThreadShouldRun.load())
                {
                    writer.flushPaused = true;
                    break;
                }

                if (writer.requestPause)
                {
                    writer.flushPaused = true;
                }

                continue;
            }

            bundle = std::move(writer.frameQueue.front());
            writer.frameQueue.pop_front();

            if (writer.queueOverflowNotified.load() && !queueIsOverflowing())
            {
                writer.queueOverflowNotified = false;
            }

            writer.flushPaused = false;
        }

        if (bundle.hasRgb && writer.rgb != nullptr)
        {
            if (writer.rgb->saveMode == VideoWriter::SaveMode::Video)
            {
                std::lock_guard<std::mutex> lock(writer.rgb->mutex);
                if (writer.rgb->writer != nullptr)
                {
                    writer.rgb->writer->write(bundle.rgbFrame);
                }
            } else
            {
                const std::filesystem::path imgPath
                    = writer.rgb->framesPath
                      / ("img_" + std::to_string(bundle.frameIndex) + ".png");
                cv::imwrite(imgPath.string(), bundle.rgbFrame);

                std::lock_guard<std::mutex> bufferLock(m_bufferManagerMutex);
                m_bufferManager.push_back(bundle.frameIndex, bundle.timestamp, rgbChannel);
            }
        }

        if (bundle.hasDepth && writer.depth != nullptr)
        {
            if (writer.depth->saveMode == VideoWriter::SaveMode::Video)
            {
                cv::Mat image8Bit;
                bundle.depthFrame.convertTo(image8Bit, CV_8UC1);
                std::lock_guard<std::mutex> lock(writer.depth->mutex);
                if (writer.depth->writer != nullptr)
                {
                    writer.depth->writer->write(image8Bit);
                }
            } else
            {
                const std::filesystem::path imgPath
                    = writer.depth->framesPath
                      / ("img_" + std::to_string(bundle.frameIndex) + ".png");
                cv::Mat image16Bit;
                bundle.depthFrame.convertTo(image16Bit, CV_16UC1);
                cv::imwrite(imgPath.string(), image16Bit);

                std::lock_guard<std::mutex> bufferLock(m_bufferManagerMutex);
                m_bufferManager.push_back(bundle.frameIndex, bundle.timestamp, depthChannel);
            }
        }

        {
            std::lock_guard<std::mutex> lock(writer.frameQueueMutex);
            if (writer.requestPause && writer.frameQueue.empty())
            {
                writer.flushPaused = true;
            }
        }
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
        // Notify the other threads that we are somehow blocked
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
                = writer.rgb->framesPath / ("img_" + std::to_string(frameIndex) + ".png");
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

    auto codeStatus = [](const std::string& cmd, const std::string& head) -> std::string {
        std::stringstream processStream, stream;

        // run the process
        TinyProcessLib::Process process(cmd, "", [&](const char* bytes, size_t n) -> void {
            processStream << std::string(bytes, n);
        });

        // if the process status is ok we can save the output
        auto exitStatus = process.get_exit_status();
        if (exitStatus == 0)
        {
            stream << "### " << head << std::endl;
            stream << "```" << std::endl;
            stream << processStream.str() << std::endl;
            stream << "```" << std::endl;
        }
        return stream.str();
    };

    log()->info("{} Saving the status of the code...", logPrefix);
    auto start = BipedalLocomotion::clock().now();

    std::ofstream file(fileName + ".md");
    file << "# " << fileName << std::endl;
    file << "File containing all the installed software required to replicate the "
            "experiment.  "
         << std::endl;

    if (m_codeStatusCmdPrefixes.empty())
    {
        file << codeStatus("bash "
                           "${ROBOTOLOGY_SUPERBUILD_SOURCE_DIR}/scripts/"
                           "robotologyGitStatus.sh",
                           "ROBOTOLOGY");
        file << codeStatus("apt list --installed", "APT");
    } else
    {
        for (const auto& prefix : m_codeStatusCmdPrefixes)
        {
            file << "## `" << prefix << "`" << std::endl;
            file << codeStatus(prefix
                                   + " \"bash "
                                     "${ROBOTOLOGY_SUPERBUILD_SOURCE_DIR}/scripts/"
                                     "robotologyGitStatus.sh\"",
                               "ROBOTOLOGY");
            file << codeStatus(prefix + " \"apt list --installed\"", "APT");
        }
    }

    file.close();
    log()->info("{} Status of the code saved to file {} in {}.",
                logPrefix,
                fileName + ".md",
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

        // an IMU contains a gyro accelerometer and an orientation sensor
        for (const auto& sensorName : m_robotSensorBridge->getIMUsList())
        {
            if (m_robotSensorBridge->getIMUMeasurement(sensorName, m_analogSensorBuffer))
            {
                // it will return a tuple containing the Accelerometer, the gyro and the
                // orientation
                this->unpackIMU(m_analogSensorBuffer,
                                m_acceloremeterBuffer,
                                m_gyroBuffer,
                                m_orientationBuffer);

                signalFullName = accelerometersName + treeDelim + sensorName;
                logData(signalFullName, m_acceloremeterBuffer, time);

                signalFullName = gyrosName + treeDelim + sensorName;
                logData(signalFullName, m_gyroBuffer, time);

                signalFullName = orientationsName + treeDelim + sensorName;
                logData(signalFullName, m_orientationBuffer, time);
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

bool YarpRobotLoggerDevice::close()
{
    // Wait for the run method to finish
    stop();

    m_rpcPort.close();
    m_statusPort.close();

    // stop all the video thread
    for (auto& [cameraName, writer] : m_videoWriters)
    {
        writer.recordVideoIsRunning = false;
    }

    // close all the thread associated to the video logging
    for (auto& [cameraName, writer] : m_videoWriters)
    {
        if (writer.videoThread.joinable())
        {
            writer.videoThread.join();
            writer.videoThread = std::thread();
        }
    }

    for (auto& [cameraName, writer] : m_videoWriters)
    {
        writer.flushThreadShouldRun = false;
        writer.frameQueueCv.notify_all();
    }

    for (auto& [cameraName, writer] : m_videoWriters)
    {
        if (writer.flushThread.joinable())
        {
            writer.flushThread.join();
            writer.flushThread = std::thread();
        }
    }

    // close all the thread associated to the exogenous image logging
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

    // close the thread associated to the text logging polling
    m_lookForNewLogsIsRunning = false;
    if (m_lookForNewLogsThread.joinable())
    {
        m_lookForNewLogsThread.join();
        m_lookForNewLogsThread = std::thread();
    }

    m_lookForNewExogenousSignalIsRunning = false;
    if (m_lookForNewExogenousSignalThread.joinable())
    {
        m_lookForNewExogenousSignalThread.join();
        m_lookForNewExogenousSignalThread = std::thread();
    }

    return true;
}

void BipedalLocomotion::YarpRobotLoggerDevice::waitForAcquisitionThreadsToPause()
{
    // First we request all acquisition threads to pause and we wait for them to be paused
    log()->info("[YarpRobotLoggerDevice::waitForAcquisitionThreadsToPause] Pausing acquisition "
                "threads...");
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
            const bool captureActive = writer.recordVideoIsRunning && !writer.paused;

            bool queueEmpty = true;
            if (writer.flushThreadShouldRun)
            {
                std::lock_guard<std::mutex> lock(writer.frameQueueMutex);
                queueEmpty = writer.frameQueue.empty();
            }

            const bool flushActive = writer.flushThreadShouldRun
                                     && (!writer.flushPaused || !queueEmpty);

            if (captureActive || flushActive)
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
    log()->info("[YarpRobotLoggerDevice::waitForAcquisitionThreadsToPause] All acquisition threads "
                "paused.");
}

void BipedalLocomotion::YarpRobotLoggerDevice::resumeAcquisitionThreads()
{
    for (auto& [cameraName, writer] : m_videoWriters)
    {
        if (writer.recordVideoIsRunning)
        {
            writer.requestPause = false;
            writer.paused = false;
            writer.flushPaused = false;
            writer.frameQueueCv.notify_all();
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

    std::string actualFileName;
    log()->info("{} Saving data to .mat file...", logPrefix);
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
        // Calling callback manually since it is called only when the saving is automatic
        output = this->saveCallback(actualFileName, robometry::SaveCallbackSaveMethod::periodic);
    }

    // If it lasted less than 1 second we wait a bit to avoid that
    // we save files with the same name
    duration = BipedalLocomotion::clock().now() - startTime;
    using namespace std::chrono_literals;
    if (duration < 1s)
    {
        BipedalLocomotion::clock().sleepFor(1s - duration);
    }

    log()->info("{} Saving data took {}.", logPrefix, std::chrono::duration<double>(duration));

    return output;
}
