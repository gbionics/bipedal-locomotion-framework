/**
 * @copyright 2020, 2021 Istituto Italiano di Tecnologia (IIT), 2026 Generative Bionics S.R.L.
 * This software may be modified and distributed under the terms of the BSD-3-Clause license.
 */

#include <cctype>
#include <chrono>

#include <yarp/os/LogStream.h>

#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h>
#include <BipedalLocomotion/System/Clock.h>
#include <BipedalLocomotion/System/YarpClock.h>
#include <BipedalLocomotion/TextLogging/Logger.h>
#include <BipedalLocomotion/TextLogging/LoggerBuilder.h>
#include <BipedalLocomotion/TextLogging/YarpLogger.h>
#include <BipedalLocomotion/YarpRobotLoggerDevice.h>

#include <BipedalLocomotion/RobotLogger/CamerasRecorder.h>
#include <BipedalLocomotion/RobotLogger/CodeStatusSaver.h>
#include <BipedalLocomotion/RobotLogger/DataSink.h>
#include <BipedalLocomotion/RobotLogger/DataStorage.h>
#include <BipedalLocomotion/RobotLogger/ExogenousSignalsLogger.h>
#include <BipedalLocomotion/RobotLogger/FrameTransformLogger.h>
#include <BipedalLocomotion/RobotLogger/ImageRecorder.h>
#include <BipedalLocomotion/RobotLogger/RealTimeStreamer.h>
#include <BipedalLocomotion/RobotLogger/RobotDataLogger.h>
#include <BipedalLocomotion/RobotLogger/TextLogCollector.h>

using namespace BipedalLocomotion;
using namespace BipedalLocomotion::RobotLogger;

namespace
{
void setFactories()
{
    // Use the yarp clock in blf
    BipedalLocomotion::System::ClockBuilder::setFactory(
        std::make_shared<BipedalLocomotion::System::YarpClockFactory>());

    // the logging message are streamed using yarp
    BipedalLocomotion::TextLogging::LoggerBuilder::setFactory(
        std::make_shared<BipedalLocomotion::TextLogging::YarpLoggerFactory>());
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
    if (m_storage != nullptr)
    {
        m_storage->stopPeriodicSave();
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

    bool enableRealTimeLogging{false};
    if (!params->getParameter("enable_real_time_logging", enableRealTimeLogging))
    {
        log()->error("{} Unable to get the 'enable_real_time_logging' parameter.", logPrefix);
        return false;
    }

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

    // storage
    m_storage = std::make_unique<DataStorage>();
    if (!m_storage->initialize(params->getGroup("Telemetry"), devicePeriod))
    {
        log()->error("{} Unable to initialize the data storage.", logPrefix);
        return false;
    }
    m_storage->setSaveCallback([this](const std::string& fileName, DataStorage::SaveMethod) {
        this->onFileSaved(fileName);
    });

    if (enableRealTimeLogging)
    {
        m_realTimeStreamer = std::make_unique<RealTimeStreamer>();
        if (!m_realTimeStreamer->initialize(params->getGroup("REAL_TIME_STREAMING")))
        {
            log()->error("{} Unable to initialize the real time streaming.", logPrefix);
            return false;
        }
    } else
    {
        log()->info("{} Real time logging not activated.", logPrefix);
    }
    m_sink = std::make_unique<DataSink>(*m_storage, m_realTimeStreamer.get());

    // robot data
    bool logRobotData{true};
    getOptionalParameter("log_robot_data", logRobotData);
    if (logRobotData)
    {
        m_robotDataLogger = std::make_unique<RobotDataLogger>();
        if (!m_robotDataLogger->initialize(params->getGroup("RobotSensorBridge")))
        {
            log()->error("{} Unable to initialize the robot data logging.", logPrefix);
            return false;
        }
    }

    // cameras
    bool logCameras{true};
    getOptionalParameter("log_cameras", logCameras);
    if (logCameras)
    {
        m_camerasRecorder = std::make_unique<CamerasRecorder>();
        if (!m_camerasRecorder->initialize(params))
        {
            log()->error("{} Unable to initialize the cameras recording. The cameras will not be "
                         "logged.",
                         logPrefix);
            m_camerasRecorder.reset();
        }
    }

    // text logs
    bool logText{true};
    getOptionalParameter("log_text", logText);
    if (logText)
    {
        std::vector<std::string> subnames;
        if (!params->getParameter("text_logging_subnames", subnames))
        {
            log()->info("{} The parameter 'text_logging_subnames' is not provided. All the text "
                        "logging ports will be considered.",
                        logPrefix);
        }
        m_textLogCollector = std::make_unique<TextLogCollector>(portPrefix + "/text_logging:i",
                                                                subnames);
    }

    // code status
    bool logCodeStatus{true};
    getOptionalParameter("log_code_status", logCodeStatus);
    if (logCodeStatus)
    {
        std::vector<std::string> commands;
        if (!params->getParameter("code_status_cmds", commands))
        {
            log()->info("{} The parameter 'code_status_cmds' is not provided. No command will be "
                        "executed.",
                        logPrefix);
        }
        m_codeStatusSaver = std::make_unique<CodeStatusSaver>(commands);
    }

    // exogenous signals
    m_exogenousSignalsLogger = std::make_unique<ExogenousSignalsLogger>();
    if (!m_exogenousSignalsLogger->initialize(params->getGroup("ExogenousSignals")))
    {
        log()->error("{} Unable to initialize the exogenous signals.", logPrefix);
        return false;
    }

    // frame transforms
    bool logFrames{false};
    getOptionalParameter("log_frames", logFrames);
    if (logFrames)
    {
        m_frameTransformLogger = std::make_unique<FrameTransformLogger>();
        if (!m_frameTransformLogger->initialize(config.findGroup("Transforms")))
        {
            log()->error("{} Unable to initialize the frames logging. The frames will not be "
                         "logged.",
                         logPrefix);
            m_frameTransformLogger.reset();
        }
    }

    // ports
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

    if (m_robotDataLogger != nullptr || m_camerasRecorder != nullptr)
    {
        log()->info("{} Waiting for the attach phase before starting the logging.", logPrefix);
        return true;
    }

    return this->startDevice();
}

bool YarpRobotLoggerDevice::attachAll(const yarp::dev::PolyDriverList& poly)
{
    constexpr auto logPrefix = "[YarpRobotLoggerDevice::attachAll]";

    if (m_robotDataLogger != nullptr && !m_robotDataLogger->setDriversList(poly))
    {
        return false;
    }

    if (m_camerasRecorder != nullptr && !m_camerasRecorder->setDriversList(poly))
    {
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

    m_storage->clear();
    if (m_frameTransformLogger != nullptr)
    {
        m_frameTransformLogger->reset();
    }

    bool ok = true;
    if (m_robotDataLogger != nullptr)
    {
        ok = m_robotDataLogger->prepare(*m_sink);
    }

    if (ok && m_realTimeStreamer != nullptr)
    {
        ok = m_realTimeStreamer->addRobotMetadata(
            m_robotDataLogger != nullptr ? m_robotDataLogger->jointsList()
                                         : std::vector<std::string>{});
    }

    ok = ok && (m_textLogCollector == nullptr || m_textLogCollector->start());
    ok = ok && m_exogenousSignalsLogger->start(*m_storage);
    ok = ok && (m_camerasRecorder == nullptr || m_camerasRecorder->start(*m_storage));

    if (!ok)
    {
        log()->error("{} Unable to start the recording.", logPrefix);
        this->stopSession(false, "");
        return false;
    }

    m_firstRun = true;
    m_state = DeviceState::Recording;
    m_storage->startPeriodicSave();

    log()->info("{} The logger has started recording.", logPrefix);
    return true;
}

bool YarpRobotLoggerDevice::stopSession(bool save, const std::string& tag)
{
    constexpr auto logPrefix = "[YarpRobotLoggerDevice::stopSession]";

    std::string prefix;
    if (save && !this->fileNamePrefix(tag, prefix))
    {
        return false;
    }

    m_state = DeviceState::Saving;

    // wait for the logging cycle in progress
    {
        std::lock_guard lock(m_runMutex);
    }

    m_storage->stopPeriodicSave();

    const auto recorders = this->imageRecorders();
    for (auto* recorder : recorders)
    {
        recorder->stopAcquisition();
    }

    std::string fileName;
    if (!save || !m_storage->save(prefix, DataStorage::SaveMethod::last_call, fileName))
    {
        for (auto* recorder : recorders)
        {
            recorder->discard();
        }
        log()->info("{} No data saved.", logPrefix);
    }

    if (m_camerasRecorder != nullptr)
    {
        m_camerasRecorder->stop();
    }
    m_exogenousSignalsLogger->stop();
    if (m_textLogCollector != nullptr)
    {
        m_textLogCollector->stop();
    }

    // release the memory
    m_storage->clear();

    m_state = DeviceState::Idle;
    log()->info("{} The device is now in Idle state.", logPrefix);
    return true;
}

void YarpRobotLoggerDevice::onFileSaved(const std::string& fileName)
{
    for (auto* recorder : this->imageRecorders())
    {
        recorder->rotate(fileName);
    }

    if (m_codeStatusSaver != nullptr)
    {
        m_codeStatusSaver->save(fileName);
    }
}

std::vector<ImageRecorder*> YarpRobotLoggerDevice::imageRecorders()
{
    std::vector<ImageRecorder*> recorders;
    if (m_camerasRecorder != nullptr)
    {
        recorders = m_camerasRecorder->recorders();
    }
    const auto exogenous = m_exogenousSignalsLogger->imageRecorders();
    recorders.insert(recorders.end(), exogenous.begin(), exogenous.end());
    return recorders;
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

        if (m_realTimeStreamer != nullptr)
        {
            m_realTimeStreamer->beginCycle(time);
        }

        if (m_robotDataLogger != nullptr)
        {
            m_robotDataLogger->record(*m_sink, time);
        }
        m_exogenousSignalsLogger->record(*m_sink, time);
        if (m_textLogCollector != nullptr)
        {
            m_textLogCollector->record(*m_storage, time);
        }
        if (m_frameTransformLogger != nullptr)
        {
            m_frameTransformLogger->record(*m_sink, time);
        }

        if (m_realTimeStreamer != nullptr)
        {
            m_realTimeStreamer->endCycle();
        }
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

bool YarpRobotLoggerDevice::fileNamePrefix(const std::string& tag, std::string& prefix) const
{
    prefix = DataStorage::defaultFilePrefix;
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
            log()->error("[YarpRobotLoggerDevice::fileNamePrefix] The tag can contain only "
                         "alphanumeric characters, underscores or spaces (tag = \"{}\").",
                         tag);
            return false;
        }
    }

    prefix += "_" + editedTag;
    return true;
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
    if (!this->fileNamePrefix(tag, prefix))
    {
        return false;
    }

    std::string fileName;
    return m_storage->save(prefix, DataStorage::SaveMethod::periodic, fileName);
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
