/**
 * @file YarpRobotLoggerDevice.cpp
 * @copyright 2020, 2021 Istituto Italiano di Tecnologia (IIT), 2026 Generative Bionics S.R.L.
 * This software may be modified and distributed under the terms of the BSD-3-Clause license.
 */

#include <algorithm>
#include <cctype>
#include <chrono>
#include <optional>
#include <sstream>

#include <Eigen/Geometry>

#include <process.hpp>

#include <yarp/conf/version.h>
#include <yarp/dev/IFrameGrabberImage.h>
#include <yarp/dev/IRGBDSensor.h>
#include <yarp/eigen/Eigen.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/profiler/NetworkProfiler.h>

#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>
#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h>
#include <BipedalLocomotion/System/Clock.h>
#include <BipedalLocomotion/System/YarpClock.h>
#include <BipedalLocomotion/TextLogging/Logger.h>
#include <BipedalLocomotion/TextLogging/LoggerBuilder.h>
#include <BipedalLocomotion/TextLogging/YarpLogger.h>
#include <BipedalLocomotion/YarpRobotLoggerDevice.h>

#include <BipedalLocomotion/RobotLogger/ExogenousSignalsLogger.h>
#include <BipedalLocomotion/RobotLogger/ImageRecorder.h>
#include <BipedalLocomotion/RobotLogger/TelemetryBuffer.h>

using namespace BipedalLocomotion;
using namespace BipedalLocomotion::RobotLogger;
using BipedalLocomotion::RobotInterface::BatteryStatus;
using BipedalLocomotion::RobotInterface::YarpCameraBridge;

namespace
{
constexpr auto treeDelimiter = "::";

const std::vector<std::string> ftElementNames = {"f_x", "f_y", "f_z", "mu_x", "mu_y", "mu_z"};
const std::vector<std::string> gyroElementNames = {"omega_x", "omega_y", "omega_z"};
const std::vector<std::string> accelerometerElementNames = {"a_x", "a_y", "a_z"};
const std::vector<std::string> orientationElementNames = {"r", "p", "y"};
const std::vector<std::string> magnetometerElementNames = {"mag_x", "mag_y", "mag_z"};
const std::vector<std::string> temperatureElementNames = {"temperature"};
const std::vector<std::string> batteryElementNames
    = {"voltage", "current", "charge", "temperature"};

void setFactories()
{
    // Use the yarp clock in blf
    BipedalLocomotion::System::ClockBuilder::setFactory(
        std::make_shared<BipedalLocomotion::System::YarpClockFactory>());

    // the logging message are streamed using yarp
    BipedalLocomotion::TextLogging::LoggerBuilder::setFactory(
        std::make_shared<BipedalLocomotion::TextLogging::YarpLoggerFactory>());
}

void findAndReplaceAll(std::string& data, const std::string& toSearch, const std::string& replace)
{
    std::size_t position = data.find(toSearch);
    while (position != std::string::npos)
    {
        data.replace(position, toSearch.size(), replace);
        position = data.find(toSearch, position + replace.size());
    }
}

/** Color images of a camera. The mutex is shared with the depth images of the same camera. */
class CameraColorSource final : public IImageSource
{
public:
    CameraColorSource(YarpCameraBridge& bridge,
                      std::string camera,
                      std::shared_ptr<std::mutex> mutex)
        : m_bridge(bridge)
        , m_camera(std::move(camera))
        , m_mutex(std::move(mutex))
    {
    }

    bool read(cv::Mat& image) final
    {
        std::lock_guard lock(*m_mutex);
        if (!m_bridge.getColorImage(m_camera, m_buffer))
        {
            return false;
        }
        // the buffer may share the memory with the bridge
        image = m_buffer.clone();
        return true;
    }

private:
    YarpCameraBridge& m_bridge;
    std::string m_camera;
    std::shared_ptr<std::mutex> m_mutex;
    cv::Mat m_buffer;
};

class CameraDepthSource final : public IImageSource
{
public:
    CameraDepthSource(YarpCameraBridge& bridge,
                      std::string camera,
                      std::shared_ptr<std::mutex> mutex,
                      double scale)
        : m_bridge(bridge)
        , m_camera(std::move(camera))
        , m_mutex(std::move(mutex))
        , m_scale(scale)
    {
    }

    bool read(cv::Mat& image) final
    {
        std::lock_guard lock(*m_mutex);
        if (!m_bridge.getDepthImage(m_camera, m_buffer))
        {
            return false;
        }
        m_buffer.convertTo(image, CV_16UC1, m_scale);
        return true;
    }

private:
    YarpCameraBridge& m_bridge;
    std::string m_camera;
    std::shared_ptr<std::mutex> m_mutex;
    double m_scale;
    cv::Mat m_buffer;
};

/**
 * Build the RobotCameraBridge group from the attached devices: the devices exposing IRGBDSensor
 * are rgbd cameras, the ones exposing only IFrameGrabberImage are rgb cameras. It returns nullptr
 * if there is no camera.
 */
std::shared_ptr<ParametersHandler::StdImplementation>
getAttachedCameras(const yarp::dev::PolyDriverList& poly)
{
    std::vector<std::string> rgbCameras;
    std::vector<std::string> rgbdCameras;
    for (int i = 0; i < poly.size(); i++)
    {
        yarp::dev::IRGBDSensor* rgbdSensor{nullptr};
        yarp::dev::IFrameGrabberImage* frameGrabber{nullptr};
        if (poly[i]->poly->view(rgbdSensor) && rgbdSensor != nullptr)
        {
            rgbdCameras.push_back(poly[i]->key);
        } else if (poly[i]->poly->view(frameGrabber) && frameGrabber != nullptr)
        {
            rgbCameras.push_back(poly[i]->key);
        }
    }

    if (rgbCameras.empty() && rgbdCameras.empty())
    {
        return nullptr;
    }

    auto cameras = std::make_shared<ParametersHandler::StdImplementation>();
    if (!rgbCameras.empty())
    {
        cameras->setParameter("rgb_cameras_list", rgbCameras);
    }
    if (!rgbdCameras.empty())
    {
        cameras->setParameter("rgbd_cameras_list", rgbdCameras);
    }

    auto group = std::make_shared<ParametersHandler::StdImplementation>();
    group->setParameter("stream_cameras", true);
    group->setGroup("Cameras", cameras);

    auto join = [](const std::vector<std::string>& names) {
        std::ostringstream stream;
        for (std::size_t i = 0; i < names.size(); i++)
        {
            stream << (i == 0 ? "" : ", ") << names[i];
        }
        return stream.str();
    };
    log()->info("[YarpRobotLoggerDevice::attachAll] Cameras found in the attached devices. RGB: "
                "[{}]. RGBD: [{}].",
                join(rgbCameras),
                join(rgbdCameras));
    return group;
}

} // namespace

YarpRobotLoggerDevice::YarpRobotLoggerDevice(double period,
                                             yarp::os::ShouldUseSystemClock useSystemClock)
    : yarp::os::PeriodicThread(period, useSystemClock, yarp::os::PeriodicThreadClock::Absolute)
{
    setFactories();
}

YarpRobotLoggerDevice::YarpRobotLoggerDevice()
    : YarpRobotLoggerDevice(0.01)
{
}

YarpRobotLoggerDevice::~YarpRobotLoggerDevice()
{
    // run() and the periodic save use the components
    this->stop();
    if (m_buffer != nullptr)
    {
        m_buffer->stopPeriodicSave();
    }
}

bool YarpRobotLoggerDevice::open(yarp::os::Searchable& config)
{
    constexpr auto logPrefix = "[YarpRobotLoggerDevice::open]";
    auto params = std::make_shared<ParametersHandler::YarpImplementation>(config);

    auto getOptionalParameter = [&params, logPrefix](const std::string& name, auto& value) {
        if (!params->getParameter(name, value))
        {
            log()->info("{} The parameter '{}' is not provided. Default value: {}.",
                        logPrefix,
                        name,
                        value);
        }
    };

    double devicePeriod{0.01};
    if (params->getParameter("sampling_period_in_s", devicePeriod))
    {
        this->setPeriod(devicePeriod);
    }

    std::string portPrefix{"/yarp-robot-logger"};
    getOptionalParameter("port_prefix", portPrefix);
    getOptionalParameter("auto_start_logging", m_autoStartLogging);
    if (!params->getParameter("maximum_admissible_time_step", m_acceptableStep))
    {
        log()->info("{} The parameter 'maximum_admissible_time_step' is not provided. The time "
                    "step is not checked.",
                    logPrefix);
    }

    m_buffer = std::make_shared<TelemetryBuffer>();
    if (!m_buffer->initialize(params))
    {
        log()->error("{} Unable to initialize the telemetry buffer.", logPrefix);
        return false;
    }
    m_buffer->setSaveCallback([this](const std::string& fileName) { this->onFileSaved(fileName); });

    bool logRobotData{true};
    getOptionalParameter("log_robot_data", logRobotData);
    if (logRobotData && !this->setupRobotSensorBridge(params->getGroup("RobotSensorBridge")))
    {
        log()->error("{} Unable to setup the robot sensor bridge.", logPrefix);
        return false;
    }

    bool logCameras{true};
    getOptionalParameter("log_cameras", logCameras);
    if (logCameras && params->getGroup("RobotCameraBridge").lock() == nullptr)
    {
        log()->info("{} The group 'RobotCameraBridge' is not provided. The cameras will be "
                    "retrieved from the attached devices.",
                    logPrefix);
        m_cameraParams = params;
    } else if (logCameras && !this->setupCameras(params, params->getGroup("RobotCameraBridge")))
    {
        log()->error("{} Unable to setup the cameras. The cameras will not be logged.", logPrefix);
        m_cameraRecorders.clear();
        m_cameraBridge.reset();
    }

    getOptionalParameter("log_text", m_logText);
    if (m_logText)
    {
        if (!params->getParameter("text_logging_subnames", m_textLoggingSubnames))
        {
            log()->info("{} The parameter 'text_logging_subnames' is not provided. All the text "
                        "logging ports will be considered.",
                        logPrefix);
        }
        m_textLoggingPortName = portPrefix + "/text_logging:i";
        // do not drop the messages arriving between two calls of run()
        m_textLoggingPort.setStrict(true);
    }

    bool logCodeStatus{true};
    getOptionalParameter("log_code_status", logCodeStatus);
    if (logCodeStatus && !params->getParameter("code_status_cmds", m_codeStatusCommands))
    {
        log()->info("{} The parameter 'code_status_cmds' is not provided. No command will be "
                    "executed.",
                    logPrefix);
    }

    m_exogenousSignals = std::make_unique<ExogenousSignalsLogger>();
    if (!m_exogenousSignals->initialize(params->getGroup("ExogenousSignals"), m_buffer, portPrefix))
    {
        log()->error("{} Unable to initialize the exogenous signals.", logPrefix);
        return false;
    }

    bool logFrames{false};
    getOptionalParameter("log_frames", logFrames);
    if (logFrames && !this->setupFrameTransforms(config.findGroup("Transforms")))
    {
        log()->error("{} Unable to setup the frames logging. The frames will not be logged.",
                     logPrefix);
        m_frameTransform = nullptr;
        m_frameTransformDevice.close();
    }

    const std::string rpcPortName = portPrefix + "/commands/rpc:i";
    this->yarp().attachAsServer(m_rpcPort);
    if (!m_rpcPort.open(rpcPortName))
    {
        log()->error("{} Unable to open the port {}.", logPrefix, rpcPortName);
        return false;
    }

    const std::string statusPortName = portPrefix + "/status:o";
    if (!m_statusPort.open(statusPortName))
    {
        log()->error("{} Unable to open the port {}.", logPrefix, statusPortName);
        return false;
    }

    log()->info("{} Logger configuration completed.", logPrefix);

    if (m_robotSensorBridge != nullptr || m_cameraBridge != nullptr)
    {
        log()->info("{} Waiting for the attach phase before starting the logging.", logPrefix);
        return true;
    }

    return this->startDevice();
}

void YarpRobotLoggerDevice::addJointSignal(const std::string& name,
                                           std::function<bool(Eigen::Ref<Eigen::VectorXd>)> read)
{
    m_jointSignals.push_back({name, std::move(read)});
}

template <int Size, typename Reader>
void YarpRobotLoggerDevice::addSensorSignal(const std::string& group,
                                            const std::vector<std::string>& elementNames,
                                            std::function<const std::vector<std::string>&()> sensors,
                                            Reader reader)
{
    SensorSignal signal;
    signal.group = group;
    signal.elementNames = elementNames;
    signal.sensors = std::move(sensors);
    signal.buffer.resize(Size);
    signal.read = [reader](const std::string& sensor, Eigen::VectorXd& output) -> bool {
        Eigen::Matrix<double, Size, 1> measurement;
        if (!reader(sensor, measurement))
        {
            return false;
        }
        output = measurement;
        return true;
    };
    m_sensorSignals.push_back(std::move(signal));
}

bool YarpRobotLoggerDevice::setupRobotSensorBridge(
    std::weak_ptr<const ParametersHandler::IParametersHandler> params)
{
    constexpr auto logPrefix = "[YarpRobotLoggerDevice::setupRobotSensorBridge]";

    auto ptr = params.lock();
    if (ptr == nullptr)
    {
        log()->error("{} The group 'RobotSensorBridge' is not provided.", logPrefix);
        return false;
    }

    m_robotSensorBridge = std::make_unique<RobotInterface::YarpSensorBridge>();
    if (!m_robotSensorBridge->initialize(ptr))
    {
        log()->error("{} Unable to configure the sensor bridge.", logPrefix);
        return false;
    }

    std::unordered_map<std::string, bool> stream = {{"stream_joint_states", false},
                                                    {"stream_joint_accelerations", true},
                                                    {"stream_motor_states", false},
                                                    {"stream_motor_temperature", false},
                                                    {"stream_motor_PWM", false},
                                                    {"stream_pids", false},
                                                    {"stream_inertials", false},
                                                    {"stream_cartesian_wrenches", false},
                                                    {"stream_forcetorque_sensors", false},
                                                    {"stream_temperatures", false},
                                                    {"stream_battery", false}};
    for (auto& [name, value] : stream)
    {
        if (!ptr->getParameter(name, value))
        {
            log()->info("{} The parameter '{}' is not provided. Default value: {}.",
                        logPrefix,
                        name,
                        value);
        }
    }

    // the bridge is never replaced after this point
    auto& bridge = *m_robotSensorBridge;

    if (stream["stream_joint_states"])
    {
        addJointSignal("joints_state::positions",
                       [&bridge](auto v) { return bridge.getJointPositions(v); });
        addJointSignal("joints_state::velocities",
                       [&bridge](auto v) { return bridge.getJointVelocities(v); });
        if (stream["stream_joint_accelerations"])
        {
            addJointSignal("joints_state::accelerations",
                           [&bridge](auto v) { return bridge.getJointAccelerations(v); });
        }
        addJointSignal("joints_state::torques",
                       [&bridge](auto v) { return bridge.getJointTorques(v); });
    }

    if (stream["stream_motor_states"])
    {
        addJointSignal("motors_state::positions",
                       [&bridge](auto v) { return bridge.getMotorPositions(v); });
        addJointSignal("motors_state::velocities",
                       [&bridge](auto v) { return bridge.getMotorVelocities(v); });
        addJointSignal("motors_state::accelerations",
                       [&bridge](auto v) { return bridge.getMotorAccelerations(v); });
        addJointSignal("motors_state::currents",
                       [&bridge](auto v) { return bridge.getMotorCurrents(v); });
        if (stream["stream_motor_temperature"])
        {
            addJointSignal("motors_state::temperatures",
                           [&bridge](auto v) { return bridge.getMotorTemperatures(v); });
        }
    }

    if (stream["stream_motor_PWM"])
    {
        addJointSignal("motors_state::PWM", [&bridge](auto v) { return bridge.getMotorPWMs(v); });
    }

    if (stream["stream_pids"])
    {
        addJointSignal("PIDs", [&bridge](auto v) { return bridge.getPidPositions(v); });
    }

    if (stream["stream_forcetorque_sensors"])
    {
        addSensorSignal<6>(
            "FTs",
            ftElementNames,
            [&bridge]() -> const auto& { return bridge.getSixAxisForceTorqueSensorsList(); },
            [&bridge](const std::string& name, auto& v) {
                return bridge.getSixAxisForceTorqueMeasurement(name, v);
            });
    }

    if (stream["stream_inertials"])
    {
        addSensorSignal<3>(
            "gyros",
            gyroElementNames,
            [&bridge]() -> const auto& { return bridge.getGyroscopesList(); },
            [&bridge](const std::string& name, auto& v) {
                return bridge.getGyroscopeMeasure(name, v);
            });
        addSensorSignal<3>(
            "accelerometers",
            accelerometerElementNames,
            [&bridge]() -> const auto& { return bridge.getLinearAccelerometersList(); },
            [&bridge](const std::string& name, auto& v) {
                return bridge.getLinearAccelerometerMeasurement(name, v);
            });
        addSensorSignal<3>(
            "orientations",
            orientationElementNames,
            [&bridge]() -> const auto& { return bridge.getOrientationSensorsList(); },
            [&bridge](const std::string& name, auto& v) {
                return bridge.getOrientationSensorMeasurement(name, v);
            });
        addSensorSignal<3>(
            "magnetometers",
            magnetometerElementNames,
            [&bridge]() -> const auto& { return bridge.getMagnetometersList(); },
            [&bridge](const std::string& name, auto& v) {
                return bridge.getMagnetometerMeasurement(name, v);
            });
    }

    if (stream["stream_cartesian_wrenches"])
    {
        addSensorSignal<6>(
            "cartesian_wrenches",
            ftElementNames,
            [&bridge]() -> const auto& { return bridge.getCartesianWrenchesList(); },
            [&bridge](const std::string& name, auto& v) {
                return bridge.getCartesianWrench(name, v);
            });
    }

    if (stream["stream_temperatures"])
    {
        addSensorSignal<1>(
            "temperatures",
            temperatureElementNames,
            [&bridge]() -> const auto& { return bridge.getTemperatureSensorsList(); },
            [&bridge](const std::string& name, auto& v) {
                return bridge.getTemperature(name, v(0));
            });
    }

    if (stream["stream_battery"])
    {
        addSensorSignal<4>(
            "batteries",
            batteryElementNames,
            [&bridge]() -> const auto& { return bridge.getBatteriesList(); },
            [&bridge](const std::string& name, auto& v) {
                BatteryStatus status;
                if (!bridge.getBatteryStatus(name, status))
                {
                    return false;
                }
                v << status.voltage, status.current, status.charge, status.temperature;
                return true;
            });
    }

    return true;
}

bool YarpRobotLoggerDevice::setupCameras(
    std::shared_ptr<const ParametersHandler::IParametersHandler> params,
    std::weak_ptr<const ParametersHandler::IParametersHandler> cameraBridgeGroup)
{
    constexpr auto logPrefix = "[YarpRobotLoggerDevice::setupCameras]";

    auto group = cameraBridgeGroup.lock();
    if (group == nullptr)
    {
        log()->error("{} The group 'RobotCameraBridge' is not provided.", logPrefix);
        return false;
    }

    m_cameraBridge = std::make_unique<YarpCameraBridge>();
    if (!m_cameraBridge->initialize(group))
    {
        log()->error("{} Unable to configure the camera bridge.", logPrefix);
        return false;
    }

    std::string videoEncoder;
    const bool hasVideoEncoder = params->getParameter("video_encoder", videoEncoder);

    auto addRecorder
        = [this, logPrefix](std::shared_ptr<ParametersHandler::StdImplementation> handler,
                            std::unique_ptr<IImageSource> source) {
              auto recorder = std::make_shared<ImageRecorder>();
              if (!recorder->initialize(handler, std::move(source), m_buffer))
              {
                  log()->error("{} Unable to initialize the image recorder.", logPrefix);
                  return false;
              }
              m_cameraRecorders.push_back(std::move(recorder));
              return true;
          };

    // Each option is a list with one value per camera. A single value is used for all the cameras
    // and a missing (or empty) option takes the default value.
    auto getCameraOption
        = [&params, logPrefix](const std::string& name, std::size_t size, auto defaultValue)
        -> std::optional<std::vector<decltype(defaultValue)>> {
        using Type = decltype(defaultValue);
        std::vector<Type> values;
        Type value;
        if (!params->getParameter(name, values) || values.empty())
        {
            values = {params->getParameter(name, value) ? value : defaultValue};
        }
        if (values.size() == 1)
        {
            values.resize(size, values.front());
        }
        if (values.size() != size)
        {
            log()->error("{} The parameter '{}' must contain one value or one value per camera "
                         "({}).",
                         logPrefix,
                         name,
                         size);
            return std::nullopt;
        }
        return values;
    };

    auto addCameras = [&](const std::vector<std::string>& cameras, bool isRGBD) {
        const std::string prefix = isRGBD ? "rgbd_cameras_" : "rgb_cameras_";
        const std::size_t size = cameras.size();

        const auto fps = getCameraOption(prefix + "fps", size, 30);
        const auto rgbSaveModes
            = getCameraOption(prefix + "rgb_save_mode", size, std::string("video"));
        const auto depthScales = getCameraOption(prefix + "depth_scale", size, 1000);
        const auto depthSaveModes
            = getCameraOption(prefix + "depth_save_mode", size, std::string("video"));
        if (!fps || !rgbSaveModes || (isRGBD && (!depthScales || !depthSaveModes)))
        {
            return false;
        }

        for (std::size_t i = 0; i < size; i++)
        {
            const auto& camera = cameras[i];
            if ((*fps)[i] <= 0)
            {
                log()->error("{} The fps of the camera {} must be positive.", logPrefix, camera);
                return false;
            }

            auto handler = std::make_shared<ParametersHandler::StdImplementation>();
            handler->setParameter("name", camera);
            handler->setParameter("fps", static_cast<double>((*fps)[i]));
            if (hasVideoEncoder)
            {
                handler->setParameter("video_encoder", videoEncoder);
            }

            // the color and the depth images of a camera are not read at the same time
            auto mutex = std::make_shared<std::mutex>();

            handler->setParameter("image_type", std::string("rgb"));
            handler->setParameter("channel", "camera::" + camera + "::rgb");
            handler->setParameter("save_mode", (*rgbSaveModes)[i]);
            if (!addRecorder(handler,
                             std::make_unique<CameraColorSource>(*m_cameraBridge, camera, mutex)))
            {
                return false;
            }

            if (isRGBD)
            {
                handler->setParameter("image_type", std::string("depth"));
                handler->setParameter("channel", "camera::" + camera + "::depth");
                handler->setParameter("save_mode", (*depthSaveModes)[i]);
                if (!addRecorder(handler,
                                 std::make_unique<CameraDepthSource>(*m_cameraBridge,
                                                                     camera,
                                                                     mutex,
                                                                     (*depthScales)[i])))
                {
                    return false;
                }
            }
        }
        return true;
    };

    const auto& metadata = m_cameraBridge->getMetaData();
    if (metadata.bridgeOptions.isRGBCameraEnabled
        && !addCameras(metadata.sensorsList.rgbCamerasList, false))
    {
        return false;
    }

    return !metadata.bridgeOptions.isRGBDCameraEnabled
           || addCameras(metadata.sensorsList.rgbdCamerasList, true);
}

bool YarpRobotLoggerDevice::setupFrameTransforms(const yarp::os::Bottle& config)
{
    constexpr auto logPrefix = "[YarpRobotLoggerDevice::setupFrameTransforms]";

    if (config.isNull())
    {
        log()->error("{} The group 'Transforms' is not provided.", logPrefix);
        return false;
    }

    auto params = std::make_shared<ParametersHandler::YarpImplementation>(config);
    std::vector<std::string> parents;
    if (!params->getParameter("parent_frames", parents) || parents.empty())
    {
        log()->error("{} The parameter 'parent_frames' is missing or empty.", logPrefix);
        return false;
    }
    m_parentFrames.insert(parents.begin(), parents.end());

    // the group is passed as it is to the device
    yarp::os::Bottle& deviceGroup = config.findGroup("TransformClientDevice");
    if (deviceGroup.isNull())
    {
        log()->error("{} The group 'TransformClientDevice' is not provided.", logPrefix);
        return false;
    }

    if (!m_frameTransformDevice.open(deviceGroup))
    {
        log()->error("{} Unable to open the transform client.", logPrefix);
        return false;
    }

    if (!m_frameTransformDevice.view(m_frameTransform) || m_frameTransform == nullptr)
    {
        log()->error("{} Unable to view the IFrameTransform interface.", logPrefix);
        return false;
    }

    return true;
}

bool YarpRobotLoggerDevice::attachAll(const yarp::dev::PolyDriverList& poly)
{
    constexpr auto logPrefix = "[YarpRobotLoggerDevice::attachAll]";

    if (m_robotSensorBridge != nullptr && !m_robotSensorBridge->setDriversList(poly))
    {
        log()->error("{} Could not attach the drivers list to the sensor bridge.", logPrefix);
        return false;
    }

    if (m_cameraParams != nullptr && m_cameraBridge == nullptr)
    {
        auto cameras = getAttachedCameras(poly);
        if (cameras == nullptr)
        {
            log()->info("{} No camera found in the attached devices.", logPrefix);
        } else if (this->isRunning())
        {
            log()->warn("{} The cameras are not logged since the logger started before the attach "
                        "phase. Please provide the group 'RobotCameraBridge'.",
                        logPrefix);
        } else if (!this->setupCameras(m_cameraParams, cameras))
        {
            log()->error("{} Unable to setup the cameras.", logPrefix);
            m_cameraRecorders.clear();
            m_cameraBridge.reset();
            return false;
        }
    }

    if (m_cameraBridge != nullptr && !m_cameraBridge->setDriversList(poly))
    {
        log()->error("{} Could not attach the drivers list to the camera bridge.", logPrefix);
        return false;
    }

    log()->info("{} Attach completed.", logPrefix);

    return this->isRunning() || this->startDevice();
}

bool YarpRobotLoggerDevice::detachAll()
{
    if (this->isRunning())
    {
        this->stop();
    }
    return true;
}

bool YarpRobotLoggerDevice::close()
{
    this->stop();

    // no more commands while closing
    m_rpcPort.close();

    {
        std::lock_guard lock(m_sessionMutex);
        if (m_state == DeviceState::Recording)
        {
            this->stopSession(true, "");
        }
    }

    m_statusPort.close();
    return true;
}

bool YarpRobotLoggerDevice::startDevice()
{
    constexpr auto logPrefix = "[YarpRobotLoggerDevice::startDevice]";

    if (m_autoStartLogging)
    {
        std::lock_guard lock(m_sessionMutex);
        if (!this->startSession())
        {
            return false;
        }
    }

    if (!this->isRunning() && !this->start())
    {
        log()->error("{} Unable to start the periodic thread.", logPrefix);
        return false;
    }

    if (!m_autoStartLogging)
    {
        log()->info("{} Device started in Idle state. Use 'startRecording' to begin.", logPrefix);
    }
    return true;
}

bool YarpRobotLoggerDevice::startSession()
{
    constexpr auto logPrefix = "[YarpRobotLoggerDevice::startSession]";

    m_buffer->clear();
    m_frames.clear();

    bool ok = m_robotSensorBridge == nullptr || this->addRobotChannels();
    ok = ok && (!m_logText || this->startTextLogging());
    ok = ok && m_exogenousSignals->start();
    for (const auto& recorder : m_cameraRecorders)
    {
        ok = ok && recorder->start();
    }

    if (!ok)
    {
        log()->error("{} Unable to start the recording.", logPrefix);
        this->stopSession(false, "");
        return false;
    }

    m_firstRun = true;
    m_state = DeviceState::Recording;
    m_buffer->startPeriodicSave();

    log()->info("{} The logger has started recording.", logPrefix);
    return true;
}

bool YarpRobotLoggerDevice::stopSession(bool save, const std::string& tag)
{
    constexpr auto logPrefix = "[YarpRobotLoggerDevice::stopSession]";

    std::string prefix;
    if (save && !this->getFileNamePrefix(tag, prefix))
    {
        return false;
    }

    m_state = DeviceState::Saving;

    // wait for the logging cycle in progress
    {
        std::lock_guard lock(m_runMutex);
    }

    m_buffer->stopPeriodicSave();

    const auto recorders = this->getImageRecorders();
    for (const auto& recorder : recorders)
    {
        recorder->stopAcquisition();
    }

    std::string fileName;
    if (!save || !m_buffer->save(prefix, fileName))
    {
        for (const auto& recorder : recorders)
        {
            recorder->discard();
        }
        log()->info("{} No data saved.", logPrefix);
    }

    for (const auto& recorder : m_cameraRecorders)
    {
        recorder->stop();
    }
    m_exogenousSignals->stop();
    this->stopTextLogging();

    // release the memory
    m_buffer->clear();

    m_state = DeviceState::Idle;
    log()->info("{} The device is now in Idle state.", logPrefix);
    return true;
}

void YarpRobotLoggerDevice::onFileSaved(const std::string& fileName)
{
    for (const auto& recorder : this->getImageRecorders())
    {
        recorder->rotate(fileName);
    }

    this->saveCodeStatus(fileName);
}

std::vector<std::shared_ptr<ImageRecorder>> YarpRobotLoggerDevice::getImageRecorders() const
{
    std::vector<std::shared_ptr<ImageRecorder>> recorders = m_cameraRecorders;
    const auto& exogenous = m_exogenousSignals->getImageRecorders();
    recorders.insert(recorders.end(), exogenous.begin(), exogenous.end());
    return recorders;
}

bool YarpRobotLoggerDevice::getFileNamePrefix(const std::string& tag, std::string& prefix) const
{
    prefix = TelemetryBuffer::defaultFilePrefix;
    if (tag.empty())
    {
        return true;
    }

    std::string editedTag = tag;
    for (auto& c : editedTag)
    {
        if (c == ' ')
        {
            c = '_';
        } else if (!std::isalnum(static_cast<unsigned char>(c)) && c != '_')
        {
            log()->error("[YarpRobotLoggerDevice::getFileNamePrefix] The tag can contain only "
                         "alphanumeric characters, underscores or spaces (tag = \"{}\").",
                         tag);
            return false;
        }
    }

    prefix += "_" + editedTag;
    return true;
}

bool YarpRobotLoggerDevice::addRobotChannels()
{
    constexpr auto logPrefix = "[YarpRobotLoggerDevice::addRobotChannels]";

    if (!m_robotSensorBridgeReady)
    {
        // the sensor bridge could be not ready right after the attach
        using namespace std::chrono_literals;
        BipedalLocomotion::clock().sleepFor(2000ms);
        m_robotSensorBridgeReady = true;
    }

    if (!m_robotSensorBridge->getJointsList(m_jointsList))
    {
        log()->error("{} Could not get the joints list.", logPrefix);
        return false;
    }
    m_jointsBuffer.resize(m_jointsList.size());

    if (!m_buffer->setDescriptionList(m_jointsList))
    {
        log()->error("{} Unable to set the joints list.", logPrefix);
        return false;
    }

    for (const auto& signal : m_jointSignals)
    {
        if (!m_buffer->addChannel(signal.name, m_jointsList.size(), m_jointsList))
        {
            log()->error("{} Unable to add the channel {}.", logPrefix, signal.name);
            return false;
        }
    }

    for (const auto& signal : m_sensorSignals)
    {
        for (const auto& sensor : signal.sensors())
        {
            const std::string name = signal.group + treeDelimiter + sensor;
            if (!m_buffer->addChannel(name, signal.elementNames.size(), signal.elementNames))
            {
                log()->error("{} Unable to add the channel {}.", logPrefix, name);
                return false;
            }
        }
    }

    return true;
}

void YarpRobotLoggerDevice::recordRobotData(double time)
{
    if (!m_robotSensorBridge->advance())
    {
        log()->error("[YarpRobotLoggerDevice::recordRobotData] Could not advance the sensor "
                     "bridge.");
    }

    for (const auto& signal : m_jointSignals)
    {
        if (signal.read(m_jointsBuffer))
        {
            m_buffer->push(signal.name, m_jointsBuffer, time);
        }
    }

    for (auto& signal : m_sensorSignals)
    {
        for (const auto& sensor : signal.sensors())
        {
            if (signal.read(sensor, signal.buffer))
            {
                m_buffer->push(signal.group + treeDelimiter + sensor, signal.buffer, time);
            }
        }
    }
}

bool YarpRobotLoggerDevice::startTextLogging()
{
    if (!m_textLoggingPort.open(m_textLoggingPortName))
    {
        log()->error("[YarpRobotLoggerDevice::startTextLogging] Unable to open the port {}.",
                     m_textLoggingPortName);
        return false;
    }

    m_lookForNewLogsIsRunning = true;
    m_lookForNewLogsThread = std::thread([this] { this->lookForNewLogs(); });
    return true;
}

void YarpRobotLoggerDevice::stopTextLogging()
{
    if (!m_lookForNewLogsThread.joinable())
    {
        return;
    }

    m_lookForNewLogsIsRunning = false;
    m_lookForNewLogsThread.join();

    for (const auto& port : m_textLoggingPortNames)
    {
        yarp::os::Network::disconnect(port, m_textLoggingPortName);
    }
    m_textLoggingPortNames.clear();
    m_textLogChannels.clear();
    m_pendingTextLogs.clear();
    m_textLoggingPort.close();
}

void YarpRobotLoggerDevice::lookForNewLogs()
{
    using namespace std::chrono_literals;
    constexpr auto textLoggingPortPrefix = "/log/";
    constexpr auto period = 2s;
    constexpr auto sleepStep = 100ms;

    auto hasSubname = [this](const std::string& port) {
        if (m_textLoggingSubnames.empty())
        {
            return true;
        }
        for (const auto& subname : m_textLoggingSubnames)
        {
            if (port.find(subname) != std::string::npos)
            {
                return true;
            }
        }
        return false;
    };

    yarp::profiler::NetworkProfiler::ports_name_set ports;
    while (m_lookForNewLogsIsRunning)
    {
        ports.clear();
        yarp::profiler::NetworkProfiler::getPortsList(ports);
        for (const auto& port : ports)
        {
            if (port.name.rfind(textLoggingPortPrefix, 0) == 0
                && m_textLoggingPortNames.find(port.name) == m_textLoggingPortNames.end()
                && hasSubname(port.name) && yarp::os::Network::exists(port.name))
            {
                m_textLoggingPortNames.insert(port.name);
                yarp::os::Network::connect(port.name, m_textLoggingPortName, "udp");
            }
        }

        for (auto slept = 0ms; slept < period && m_lookForNewLogsIsRunning; slept += sleepStep)
        {
            std::this_thread::sleep_for(sleepStep);
        }
    }
}

bool YarpRobotLoggerDevice::storeTextLog(const std::string& channel,
                                         const TextLoggingEntry& entry,
                                         double time)
{
    if (m_textLogChannels.find(channel) == m_textLogChannels.end())
    {
        if (!m_buffer->addStoredChannel(channel, {1, 1}))
        {
            return false;
        }
        m_textLogChannels.insert(channel);
    }
    m_buffer->push(channel, entry, time);
    return true;
}

void YarpRobotLoggerDevice::recordTextLogs(double time)
{
    // the messages whose channel could not be added while a file was being written
    if (!m_pendingTextLogs.empty())
    {
        auto pending = std::move(m_pendingTextLogs);
        m_pendingTextLogs.clear();
        for (const auto& [channel, entry] : pending)
        {
            if (!this->storeTextLog(channel, entry, time))
            {
                m_pendingTextLogs.emplace_back(channel, entry);
            }
        }
    }

    while (m_textLoggingPort.getPendingReads() > 0)
    {
        yarp::os::Bottle* bottle = m_textLoggingPort.read(false);
        if (bottle == nullptr)
        {
            break;
        }

        const auto entry = TextLoggingEntry::deserializeMessage(*bottle, std::to_string(time));
        if (!entry.isValid)
        {
            continue;
        }

        std::string channel = entry.portSystem + treeDelimiter + entry.portPrefix + treeDelimiter
                              + entry.processName + treeDelimiter + "p" + entry.processPID;
        // matlab does not support the character - in the name of a struct field
        std::replace(channel.begin(), channel.end(), '-', '_');

        if (!this->storeTextLog(channel, entry, time))
        {
            m_pendingTextLogs.emplace_back(channel, entry);
        }
    }
}

void YarpRobotLoggerDevice::updateFrames()
{
    for (auto& [name, frame] : m_frames)
    {
        frame.active = false;
    }

    // the vector is not cleared by getAllFrameIds
    m_allFrames.clear();
    if (!m_frameTransform->getAllFrameIds(m_allFrames))
    {
        return;
    }

    for (const auto& id : m_allFrames)
    {
        if (m_parentFrames.find(id) != m_parentFrames.end())
        {
            continue;
        }

        const auto known = m_frames.find(id);
        if (known != m_frames.end())
        {
            known->second.active = true;
            continue;
        }

        for (const auto& parent : m_parentFrames)
        {
#if YARP_VERSION_COMPARE(<, 3, 11, 0)
            const bool canTransform = m_frameTransform->canTransform(id, parent);
#else
            bool ok = false;
            const bool canTransform = m_frameTransform->canTransform(id, parent, ok) && ok;
#endif
            if (!canTransform)
            {
                continue;
            }

            FrameDescriptor frame;
            frame.parent = parent;
            frame.positionChannel = "frames::" + parent + "::" + id + "::position";
            frame.orientationChannel = "frames::" + parent + "::" + id + "::orientation";

            // if the channels cannot be added now the frame is added at the next call
            if (m_buffer->addChannel(frame.positionChannel, 3, {"x", "y", "z"})
                && m_buffer->addChannel(frame.orientationChannel, 4, {"qx", "qy", "qz", "qw"}))
            {
                m_frames.emplace(id, frame);
            }
            break;
        }
    }
}

void YarpRobotLoggerDevice::recordFrames(double time)
{
    this->updateFrames();

    for (const auto& [id, frame] : m_frames)
    {
        if (!frame.active
            || !m_frameTransform->getTransform(id, frame.parent, m_frameTransformMatrix))
        {
            continue;
        }

        const Eigen::Matrix4d transform = yarp::eigen::toEigen(m_frameTransformMatrix);
        const Eigen::Vector3d position = transform.topRightCorner<3, 1>();
        const Eigen::Quaterniond quaternion(Eigen::Matrix3d(transform.topLeftCorner<3, 3>()));
        Eigen::Vector4d orientation;
        orientation << quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w();

        m_buffer->push(frame.positionChannel, position, time);
        m_buffer->push(frame.orientationChannel, orientation, time);
    }
}

void YarpRobotLoggerDevice::saveCodeStatus(const std::string& fileName) const
{
    constexpr auto logPrefix = "[YarpRobotLoggerDevice::saveCodeStatus]";

    if (m_codeStatusCommands.empty())
    {
        return;
    }

    const auto start = std::chrono::steady_clock::now();
    for (const auto& commandTemplate : m_codeStatusCommands)
    {
        std::string command = commandTemplate;
        findAndReplaceAll(command, "{filename}", fileName);

        log()->info("{} Running the code status command: {}", logPrefix, command);

        std::stringstream output;
        TinyProcessLib::Process process(command, "", [&output](const char* bytes, size_t n) {
            output << std::string(bytes, n);
        });
        const int exitStatus = process.get_exit_status();
        if (exitStatus != 0)
        {
            log()->warn("{} The command '{}' exited with status {}. Output: {}",
                        logPrefix,
                        command,
                        exitStatus,
                        output.str());
        }
    }

    log()->info("{} Status of the code saved in {}.",
                logPrefix,
                std::chrono::duration<double>(std::chrono::steady_clock::now() - start));
}

void YarpRobotLoggerDevice::run()
{
    constexpr auto logPrefix = "[YarpRobotLoggerDevice::run]";
    const std::chrono::nanoseconds t = BipedalLocomotion::clock().now();

    if (m_state != DeviceState::Recording)
    {
        m_previousTimestamp = t;
        m_firstRun = true;
        return;
    }

    if (!m_firstRun && t - m_previousTimestamp > m_acceptableStep)
    {
        log()->warn("{} The time step is too big. Previous timestamp: {}. Current timestamp: {}. "
                    "Time step: {}.",
                    logPrefix,
                    std::chrono::duration<double>(m_previousTimestamp),
                    std::chrono::duration<double>(t),
                    std::chrono::duration<double>(t - m_previousTimestamp));
        m_previousTimestamp = t;
        return;
    }
    m_previousTimestamp = t;
    m_firstRun = false;

    const double time = std::chrono::duration<double>(t).count();
    {
        std::lock_guard lock(m_runMutex);
        if (m_state != DeviceState::Recording)
        {
            return;
        }

        m_buffer->beginCycle(time);

        if (m_robotSensorBridge != nullptr)
        {
            this->recordRobotData(time);
        }
        m_exogenousSignals->record(time);
        if (m_logText)
        {
            this->recordTextLogs(time);
        }
        if (m_frameTransform != nullptr)
        {
            this->recordFrames(time);
        }

        m_buffer->endCycle();
    }

    yarp::os::Bottle& status = m_statusPort.prepare();
    status.clear();
    status.addFloat64(time);
    m_statusPort.write();

    const double runDuration
        = std::chrono::duration<double>(BipedalLocomotion::clock().now() - t).count();
    if (runDuration > this->getPeriod())
    {
        log()->warn("{} The run method took {} seconds, more than the period of {} seconds.",
                    logPrefix,
                    runDuration,
                    this->getPeriod());
    }

    yInfoThrottle(5) << logPrefix << " Logging data...";
}

bool YarpRobotLoggerDevice::startRecording()
{
    constexpr auto logPrefix = "[YarpRobotLoggerDevice::startRecording]";
    std::lock_guard lock(m_sessionMutex);

    if (m_state != DeviceState::Idle)
    {
        log()->error("{} Cannot start recording: the device is in the {} state.",
                     logPrefix,
                     this->getState());
        return false;
    }
    return this->startSession();
}

bool YarpRobotLoggerDevice::saveRecording(const std::string& tag)
{
    constexpr auto logPrefix = "[YarpRobotLoggerDevice::saveRecording]";
    std::lock_guard lock(m_sessionMutex);

    if (m_state != DeviceState::Recording)
    {
        log()->error("{} Cannot save: the device is not recording.", logPrefix);
        return false;
    }

    std::string prefix;
    if (!this->getFileNamePrefix(tag, prefix))
    {
        return false;
    }

    std::string fileName;
    return m_buffer->save(prefix, fileName);
}

bool YarpRobotLoggerDevice::saveAndStopRecording(const std::string& tag)
{
    constexpr auto logPrefix = "[YarpRobotLoggerDevice::saveAndStopRecording]";
    std::lock_guard lock(m_sessionMutex);

    if (m_state != DeviceState::Recording)
    {
        log()->error("{} Cannot save and stop: the device is not recording.", logPrefix);
        return false;
    }
    return this->stopSession(true, tag);
}

bool YarpRobotLoggerDevice::discardRecording()
{
    constexpr auto logPrefix = "[YarpRobotLoggerDevice::discardRecording]";
    std::lock_guard lock(m_sessionMutex);

    if (m_state != DeviceState::Recording)
    {
        log()->error("{} Cannot discard: the device is not recording.", logPrefix);
        return false;
    }
    return this->stopSession(false, "");
}

std::string YarpRobotLoggerDevice::getState()
{
    switch (m_state.load())
    {
    case DeviceState::Idle:
        return "Idle";
    case DeviceState::Recording:
        return "Recording";
    case DeviceState::Saving:
        return "Saving";
    }
    return "Unknown";
}
