/**
 * @file YarpRobotLoggerDevice.h
 * @copyright 2020,2021 Istituto Italiano di Tecnologia (IIT), 2026 Generative Bionics S.R.L.
 * This software may be modified and distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_FRAMEWORK_YARP_ROBOT_LOGGER_DEVICE_H
#define BIPEDAL_LOCOMOTION_FRAMEWORK_YARP_ROBOT_LOGGER_DEVICE_H

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IFrameTransform.h>
#include <yarp/dev/IMultipleWrapper.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/Port.h>
#include <yarp/sig/Matrix.h>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/RobotInterface/YarpCameraBridge.h>
#include <BipedalLocomotion/RobotInterface/YarpSensorBridge.h>
#include <BipedalLocomotion/YarpTextLoggingUtilities.h>

#include <YarpRobotLoggerDeviceCommands.h>

namespace BipedalLocomotion
{

namespace RobotLogger
{
class ExogenousSignalsLogger;
class ImageRecorder;
class TelemetryBuffer;
} // namespace RobotLogger

/**
 * YarpRobotLoggerDevice logs the robot data, the exogenous signals streamed by other applications,
 * the text logs, the frame transforms and the cameras. The data is stored by a
 * RobotLogger::TelemetryBuffer in mat files and, optionally, streamed in real time.
 */
class YarpRobotLoggerDevice : public yarp::dev::DeviceDriver,
                              public yarp::dev::IMultipleWrapper,
                              public yarp::os::PeriodicThread,
                              public YarpRobotLoggerDeviceCommands
{
public:
    enum class DeviceState
    {
        Idle,
        Recording,
        Saving
    };

    YarpRobotLoggerDevice(double period,
                          yarp::os::ShouldUseSystemClock useSystemClock
                          = yarp::os::ShouldUseSystemClock::No);
    YarpRobotLoggerDevice();
    ~YarpRobotLoggerDevice();

    virtual bool open(yarp::os::Searchable& config) final;
    virtual bool close() final;
    virtual bool attachAll(const yarp::dev::PolyDriverList& poly) final;
    virtual bool detachAll() final;
    virtual void run() final;

    // RPC commands
    virtual bool startRecording() override;
    virtual bool saveRecording(const std::string& tag = "") override;
    virtual bool saveAndStopRecording(const std::string& tag = "") override;
    virtual bool discardRecording() override;
    virtual std::string getState() override;

private:
    /** A quantity with one element per joint, e.g., joint positions. */
    struct JointSignal
    {
        std::string name;
        std::function<bool(Eigen::Ref<Eigen::VectorXd>)> read;
    };

    /** A quantity measured by a set of sensors, e.g., gyroscopes. */
    struct SensorSignal
    {
        std::string group;
        std::vector<std::string> elementNames;
        std::function<const std::vector<std::string>&()> sensors;
        std::function<bool(const std::string&, Eigen::VectorXd&)> read;
        Eigen::VectorXd buffer;
    };

    struct FrameDescriptor
    {
        std::string parent;
        std::string positionChannel;
        std::string orientationChannel;
        bool active{true};
    };

    bool setupRobotSensorBridge(std::weak_ptr<const ParametersHandler::IParametersHandler> params);
    bool setupCameras(std::shared_ptr<const ParametersHandler::IParametersHandler> params);
    bool setupFrameTransforms(const yarp::os::Bottle& config);

    void
    addJointSignal(const std::string& name, std::function<bool(Eigen::Ref<Eigen::VectorXd>)> read);
    template <int Size, typename Reader>
    void addSensorSignal(const std::string& group,
                         const std::vector<std::string>& elementNames,
                         std::function<const std::vector<std::string>&()> sensors,
                         Reader reader);

    /** Start the periodic thread and, if requested, the recording. */
    bool startDevice();
    bool startSession();
    /** Stop the recording. If save is false the data is discarded. */
    bool stopSession(bool save, const std::string& tag);
    void onFileSaved(const std::string& fileName);
    std::vector<std::shared_ptr<RobotLogger::ImageRecorder>> getImageRecorders() const;
    /** Get the file name prefix associated to a tag. It returns false if the tag is invalid. */
    bool getFileNamePrefix(const std::string& tag, std::string& prefix) const;

    bool addRobotChannels();
    void recordRobotData(double time);

    bool startTextLogging();
    void stopTextLogging();
    void lookForNewLogs();
    bool storeTextLog(const std::string& channel, const TextLoggingEntry& entry, double time);
    void recordTextLogs(double time);

    void updateFrames();
    void recordFrames(double time);

    void saveCodeStatus(const std::string& fileName) const;

    std::atomic<DeviceState> m_state{DeviceState::Idle};
    std::mutex m_sessionMutex; /**< Serializes the state transitions. */
    std::mutex m_runMutex; /**< Held by run() while logging. */

    bool m_autoStartLogging{true};
    std::chrono::nanoseconds m_previousTimestamp{0};
    std::chrono::nanoseconds m_acceptableStep{std::chrono::nanoseconds::max()};
    bool m_firstRun{true};

    std::shared_ptr<RobotLogger::TelemetryBuffer> m_buffer;
    std::unique_ptr<RobotLogger::ExogenousSignalsLogger> m_exogenousSignals;

    // robot data, disabled if the bridge is null
    std::unique_ptr<RobotInterface::YarpSensorBridge> m_robotSensorBridge;
    std::vector<JointSignal> m_jointSignals;
    std::vector<SensorSignal> m_sensorSignals;
    std::vector<std::string> m_jointsList;
    Eigen::VectorXd m_jointsBuffer;
    bool m_robotSensorBridgeReady{false};

    // cameras, disabled if the bridge is null. The recorders use the bridge.
    std::unique_ptr<RobotInterface::YarpCameraBridge> m_cameraBridge;
    std::vector<std::shared_ptr<RobotLogger::ImageRecorder>> m_cameraRecorders;

    // text logging
    bool m_logText{true};
    std::string m_textLoggingPortName;
    std::vector<std::string> m_textLoggingSubnames;
    yarp::os::BufferedPort<yarp::os::Bottle> m_textLoggingPort;
    std::atomic<bool> m_lookForNewLogsIsRunning{false};
    std::thread m_lookForNewLogsThread;
    std::unordered_set<std::string> m_textLoggingPortNames; /**< Used only by the thread. */
    std::unordered_set<std::string> m_textLogChannels;
    std::vector<std::pair<std::string, TextLoggingEntry>> m_pendingTextLogs;

    // frame transforms, disabled if m_frameTransform is null
    yarp::dev::PolyDriver m_frameTransformDevice;
    yarp::dev::IFrameTransform* m_frameTransform{nullptr}; /**< Owned by m_frameTransformDevice. */
    std::unordered_set<std::string> m_parentFrames;
    std::unordered_map<std::string, FrameDescriptor> m_frames;
    std::vector<std::string> m_allFrames;
    yarp::sig::Matrix m_frameTransformMatrix;

    std::vector<std::string> m_codeStatusCommands;

    yarp::os::Port m_rpcPort;
    yarp::os::BufferedPort<yarp::os::Bottle> m_statusPort;
};

} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_FRAMEWORK_YARP_ROBOT_LOGGER_DEVICE_H
