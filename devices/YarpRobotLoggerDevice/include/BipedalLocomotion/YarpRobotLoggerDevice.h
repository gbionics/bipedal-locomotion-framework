/**
 * @copyright 2020,2021 Istituto Italiano di Tecnologia (IIT), 2026 Generative Bionics S.R.L.
 * This software may be modified and distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_FRAMEWORK_YARP_ROBOT_LOGGER_DEVICE_H
#define BIPEDAL_LOCOMOTION_FRAMEWORK_YARP_ROBOT_LOGGER_DEVICE_H

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IMultipleWrapper.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/Port.h>

#include <YarpRobotLoggerDeviceCommands.h>

namespace BipedalLocomotion
{

namespace RobotLogger
{
class CamerasRecorder;
class CodeStatusSaver;
class DataSink;
class DataStorage;
class ExogenousSignalsLogger;
class FrameTransformLogger;
class ImageRecorder;
class RealTimeStreamer;
class RobotDataLogger;
class TextLogCollector;
} // namespace RobotLogger

/**
 * YarpRobotLoggerDevice logs the robot data, the exogenous signals streamed by other applications,
 * the text logs, the frame transforms and the cameras. The data is saved in mat files and videos.
 *
 * The device coordinates the following components:
 * - RobotLogger::DataStorage buffers the data and saves it periodically in mat files;
 * - RobotLogger::RealTimeStreamer streams the data on a yarp port;
 * - RobotLogger::RobotDataLogger logs the data of the attached robot devices;
 * - RobotLogger::ExogenousSignalsLogger logs the signals streamed by other applications;
 * - RobotLogger::TextLogCollector logs the yarp text logs;
 * - RobotLogger::FrameTransformLogger logs the frame transforms;
 * - RobotLogger::CamerasRecorder records the cameras;
 * - RobotLogger::CodeStatusSaver stores the status of the code together with the data.
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
    /** Start the periodic thread and, if requested, the recording. */
    bool startDevice();

    bool startSession();

    /** Stop the recording. If save is false the data is discarded. */
    bool stopSession(bool save, const std::string& tag);

    void onFileSaved(const std::string& fileName);

    std::vector<RobotLogger::ImageRecorder*> imageRecorders();

    /** Get the file name prefix associated to a tag. It returns false if the tag is invalid. */
    bool fileNamePrefix(const std::string& tag, std::string& prefix) const;

    std::atomic<DeviceState> m_state{DeviceState::Idle};
    std::mutex m_sessionMutex; /**< Serializes the state transitions. */
    std::mutex m_runMutex; /**< Held by run() while logging. */

    bool m_autoStartLogging{true};
    std::chrono::nanoseconds m_previousTimestamp{0};
    std::chrono::nanoseconds m_acceptableStep{std::chrono::nanoseconds::max()};
    bool m_firstRun{true};

    // the storage must outlive the components pushing data into it
    std::unique_ptr<RobotLogger::DataStorage> m_storage;
    std::unique_ptr<RobotLogger::RealTimeStreamer> m_realTimeStreamer;
    std::unique_ptr<RobotLogger::DataSink> m_sink;
    std::unique_ptr<RobotLogger::RobotDataLogger> m_robotDataLogger;
    std::unique_ptr<RobotLogger::CamerasRecorder> m_camerasRecorder;
    std::unique_ptr<RobotLogger::ExogenousSignalsLogger> m_exogenousSignalsLogger;
    std::unique_ptr<RobotLogger::TextLogCollector> m_textLogCollector;
    std::unique_ptr<RobotLogger::FrameTransformLogger> m_frameTransformLogger;
    std::unique_ptr<RobotLogger::CodeStatusSaver> m_codeStatusSaver;

    yarp::os::Port m_rpcPort;
    yarp::os::BufferedPort<yarp::os::Bottle> m_statusPort;
};

} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_FRAMEWORK_YARP_ROBOT_LOGGER_DEVICE_H
