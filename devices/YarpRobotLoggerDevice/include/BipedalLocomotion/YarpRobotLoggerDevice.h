/**
 * @copyright 2020,2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_FRAMEWORK_YARP_ROBOT_LOGGER_DEVICE_H
#define BIPEDAL_LOCOMOTION_FRAMEWORK_YARP_ROBOT_LOGGER_DEVICE_H

#include <atomic>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IFrameTransform.h>
#include <yarp/dev/IMultipleWrapper.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/Vector.h>

#include <robometry/BufferManager.h>

#include <trintrin/msgs/HumanState.h>
#include <trintrin/msgs/WearableTargets.h>
#include <trintrin/msgs/WearableData.h>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/RobotInterface/YarpCameraBridge.h>
#include <BipedalLocomotion/RobotInterface/YarpSensorBridge.h>
#include <BipedalLocomotion/YarpUtilities/VectorsCollection.h>
#include <BipedalLocomotion/YarpUtilities/VectorsCollectionClient.h>
#include <BipedalLocomotion/YarpUtilities/VectorsCollectionServer.h>

#include <YarpRobotLoggerDeviceCommands.h>

namespace BipedalLocomotion
{

class YarpRobotLoggerDevice : public yarp::dev::DeviceDriver,
                              public yarp::dev::IMultipleWrapper,
                              public yarp::os::PeriodicThread,
                              public YarpRobotLoggerDeviceCommands
{
public:
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

private:
    std::chrono::nanoseconds m_previousTimestamp;
    std::chrono::nanoseconds m_acceptableStep{std::chrono::nanoseconds::max()};
    bool m_firstRun{true};
    std::atomic<bool> m_requestPause{false};
    std::atomic<bool> m_paused{false};

    using ft_t = Eigen::Matrix<double, 6, 1>;
    using gyro_t = Eigen::Matrix<double, 3, 1>;
    using accelerometer_t = Eigen::Matrix<double, 3, 1>;
    using orientation_t = Eigen::Matrix<double, 3, 1>;
    using magnemetometer_t = Eigen::Matrix<double, 3, 1>;
    using analog_sensor_t = Eigen::Matrix<double, 12, 1>;

    std::unique_ptr<BipedalLocomotion::RobotInterface::YarpSensorBridge> m_robotSensorBridge;
    std::unique_ptr<BipedalLocomotion::RobotInterface::YarpCameraBridge> m_cameraBridge;

    bool m_sendDataRT;
    BipedalLocomotion::YarpUtilities::VectorsCollectionServer m_vectorCollectionRTDataServer;

    template <typename T> struct ExogenousSignal
    {
        std::mutex mutex;
        std::string remote;
        std::string local;
        std::string carrier;
        std::string signalName;
        yarp::os::BufferedPort<T> port;
        bool dataArrived{false};
        std::atomic<bool> connected{false};

        bool connect()
        {
            return yarp::os::Network::connect(remote, local, carrier);
        }

        void disconnect()
        {
            if (connected)
            {
                yarp::os::Network::disconnect(remote, local);
            }
        }
    };

    struct VectorsCollectionSignal
    {
        std::mutex mutex;
        BipedalLocomotion::YarpUtilities::VectorsCollectionClient client;
        BipedalLocomotion::YarpUtilities::VectorsCollectionMetadata metadata;
        std::string signalName;
        bool dataArrived{false};
        std::atomic<bool> connected{false};

        bool connect();
        void disconnect();
    };

    template <typename T> struct ExogenousSignalWithMetadata : ExogenousSignal<T>
    {
        BipedalLocomotion::YarpUtilities::VectorsCollectionMetadata metadata;
        BipedalLocomotion::YarpUtilities::VectorsCollection convertedSignal;
    };

    std::unordered_map<std::string, VectorsCollectionSignal> m_vectorsCollectionSignals;
    std::unordered_map<std::string, ExogenousSignal<yarp::sig::Vector>> m_vectorSignals;
    std::unordered_map<std::string, ExogenousSignal<yarp::os::Bottle>> m_stringSignals;
    std::unordered_map<std::string, ExogenousSignal<yarp::sig::ImageOf<yarp::sig::PixelRgb>>>
        m_imageSignals;
    std::unordered_map<std::string, ExogenousSignalWithMetadata<trintrin::msgs::HumanState>>
        m_humanStateSignals;
    std::unordered_map<std::string, ExogenousSignalWithMetadata<trintrin::msgs::WearableTargets>>
        m_wearableTargetsSignals;
    std::unordered_map<std::string, ExogenousSignalWithMetadata<trintrin::msgs::WearableData>>
        m_wearableDataSignals;

    std::atomic<bool> m_lookForNewExogenousSignalIsRunning{false};
    std::thread m_lookForNewExogenousSignalThread;

    std::vector<std::string> m_rgbCamerasList;
    std::vector<std::string> m_rgbdCamerasList;
    struct VideoWriter
    {
        enum class SaveMode
        {
            Video,
            Frame
        };

        struct ImageSaver
        {
            std::mutex mutex;
            std::shared_ptr<cv::VideoWriter> writer;
            cv::Mat frame;
            SaveMode saveMode{SaveMode::Video};
            std::filesystem::path framesPath;
        };

        struct FrameBundle
        {
            unsigned int frameIndex{0};
            double timestamp{0.0};
            cv::Mat rgbFrame;
            cv::Mat depthFrame;
            bool hasRgb{false};
            bool hasDepth{false};
        };

        std::shared_ptr<ImageSaver> rgb;
        std::shared_ptr<ImageSaver> depth;
        int depthScale{1};

        std::thread videoThread;
        std::thread flushThread;
        std::atomic<bool> recordVideoIsRunning{false};
        int fps{-1};
        std::atomic<unsigned int> frameIndex{0};
        std::atomic<bool> resetIndex{false};
        std::atomic<bool> paused{false};
        std::atomic<bool> requestPause{false};

        std::condition_variable frameQueueCv;
        std::mutex frameQueueMutex;
        std::deque<FrameBundle> frameQueue;
        std::size_t maxBufferedBundles{0};
        std::atomic<bool> flushThreadShouldRun{false};
        std::atomic<bool> flushPaused{false};
        std::atomic<bool> queueOverflowNotified{false};
    };

    std::string m_videoCodecCode{"mp4v"};
    std::unordered_map<std::string, VideoWriter> m_videoWriters;
    std::unordered_map<std::string, VideoWriter> m_exogenousImageWriters;

    const std::string m_textLoggingPortName = "/YarpRobotLoggerDevice/TextLogging:i";
    std::unordered_set<std::string> m_textLoggingPortNames;
    yarp::os::BufferedPort<yarp::os::Bottle> m_textLoggingPort;
    std::atomic<bool> m_lookForNewLogsIsRunning{false};
    std::unordered_set<std::string> m_textLogsStoredInManager;
    std::thread m_lookForNewLogsThread;

    struct FrameDescriptor
    {
        std::string parent; /**< name of the parent frame */
        std::string positionChannelName; /**< name of the channel associated to the frame */
        std::string orientationChannelName; /**< name of the channel associated to the frame */
        bool active{true}; /**< is the frame active? */
    };

    yarp::dev::PolyDriver m_tfDevice;
    yarp::dev::IFrameTransform* m_tf{nullptr};
    std::unordered_set<std::string> m_tfRootFrames;
    std::vector<std::string> m_allFrames;
    std::unordered_map<std::string, FrameDescriptor> m_tfChildFrames;
    yarp::sig::Matrix m_tfMatrix;

    std::vector<std::string> m_jointList;
    Eigen::VectorXd m_jointSensorBuffer;
    ft_t m_ftBuffer;
    gyro_t m_gyroBuffer;
    accelerometer_t m_acceloremeterBuffer;
    orientation_t m_orientationBuffer;
    magnemetometer_t m_magnemetometerBuffer;
    analog_sensor_t m_analogSensorBuffer;
    double m_ftTemperatureBuffer;

    bool m_streamMotorStates{false};
    bool m_streamJointStates{false};
    bool m_streamJointAccelerations{true};
    bool m_streamMotorTemperature{false};
    bool m_streamMotorPWM{false};
    bool m_streamPIDs{false};
    bool m_streamInertials{false};
    bool m_streamCartesianWrenches{false};
    bool m_streamFTSensors{false};
    bool m_streamTemperatureSensors{false};
    bool m_logText{true};
    bool m_logCodeStatus{true};
    bool m_logCameras{true};
    bool m_logFrames{false};
    bool m_logRobot{true};
    std::vector<std::string> m_textLoggingSubnames;
    std::vector<std::string> m_codeStatusCmdPrefixes;

    std::mutex m_bufferManagerMutex;
    robometry::BufferManager m_bufferManager;

    const std::string m_rpcPortName{"/commands/rpc:i"}; /**< name of Remote
                                                                            Procedure Call port. */
    yarp::os::Port m_rpcPort; /**< Remote Procedure Call port. */

    std::string m_statusPortName{"/status:o"};
    yarp::os::BufferedPort<yarp::os::Bottle> m_statusPort; /**< Port used to send the status of the
                                                              device. */

    void lookForNewLogs();
    void lookForExogenousSignals();

    bool addChannel(const std::string& nameKey,
                    std::size_t vectorSize,
                    const std::vector<std::string>& metadata = {});

    bool populateCamerasData(const std::string& logPrefix,
                             std::shared_ptr<const ParametersHandler::IParametersHandler> params,
                             const std::string& fpsParamName,
                             const std::vector<std::string>& cameraNames);

    bool hasSubstring(const std::string& str, const std::vector<std::string>& substrings) const;
    void recordVideo(const std::string& cameraName, VideoWriter& writer);
    void flushVideoQueue(const std::string& cameraName, VideoWriter& writer);
    void saveExogenousImages(const std::string& signalName,
                             VideoWriter& writer,
                             ExogenousSignal<yarp::sig::ImageOf<yarp::sig::PixelRgb>>& signal);
    void saveCodeStatus(const std::string& logPrefix, const std::string& fileName) const;
    void unpackIMU(Eigen::Ref<const analog_sensor_t> signal,
                   Eigen::Ref<accelerometer_t> accelerometer,
                   Eigen::Ref<gyro_t> gyro,
                   Eigen::Ref<orientation_t> orientation);
    bool setupRobotSensorBridge(std::weak_ptr<const ParametersHandler::IParametersHandler> params);
    bool setupRobotCameraBridge(std::weak_ptr<const ParametersHandler::IParametersHandler> params);
    bool setupTelemetry(std::weak_ptr<const ParametersHandler::IParametersHandler> params,
                        const double& devicePeriod);
    bool setupExogenousInputs(std::weak_ptr<const ParametersHandler::IParametersHandler> params);
    bool setupTransformInputs(const yarp::os::Bottle& config);
    bool updateChildTransformList();
    bool saveCallback(const std::string& fileName, const robometry::SaveCallbackSaveMethod& method);
    bool openVideoWriter(
        std::shared_ptr<VideoWriter::ImageSaver> imageSaver,
        const std::string& camera,
        const std::string& imageType,
        const std::unordered_map<std::string, std::pair<std::size_t, std::size_t>>& imgDimensions);
    bool createFramesFolder(std::shared_ptr<VideoWriter::ImageSaver> imageSaver,
                            const std::string& camera,
                            const std::string& imageType);
    bool startLogging();
    bool prepareRobotLogging();
    bool prepareCameraLogging();
    bool prepareExogenousImageLogging();
    bool prepareRTStreaming();

    const std::string defaultFilePrefix = "robot_logger_device";

    const std::string robotRtRootName = "robot_realtime";

    const std::string jointStatePositionsName = "joints_state::positions";
    const std::string jointStateVelocitiesName = "joints_state::velocities";
    const std::string jointStateAccelerationsName = "joints_state::accelerations";
    const std::string jointStateTorquesName = "joints_state::torques";

    const std::string motorStatePositionsName = "motors_state::positions";
    const std::string motorStateVelocitiesName = "motors_state::velocities";
    const std::string motorStateAccelerationsName = "motors_state::accelerations";
    const std::string motorStateCurrentsName = "motors_state::currents";
    const std::string motorStateTemperaturesName = "motors_state::temperatures";
    const std::string motorStatePwmName = "motors_state::PWM";

    const std::string motorStatePidsName = "PIDs";

    const std::string ftsName = "FTs";

    const std::vector<std::string> ftElementNames = {"f_x", "f_y", "f_z", "mu_x", "mu_y", "mu_z"};

    const std::string gyrosName = "gyros";
    const std::vector<std::string> gyroElementNames = {"omega_x", "omega_y", "omega_z"};

    const std::string accelerometersName = "accelerometers";
    const std::vector<std::string> accelerometerElementNames = {"a_x", "a_y", "a_z"};

    const std::string orientationsName = "orientations";
    const std::vector<std::string> orientationElementNames = {"r", "p", "y"};

    const std::string magnetometersName = "magnetometers";
    const std::vector<std::string> magnetometerElementNames = {"mag_x", "mag_y", "mag_z"};

    const std::string cartesianWrenchesName = "cartesian_wrenches";
    const std::vector<std::string> cartesianWrenchNames = {ftElementNames[0],
                                                           ftElementNames[1],
                                                           ftElementNames[2],
                                                           ftElementNames[3],
                                                           ftElementNames[4],
                                                           ftElementNames[5]};

    const std::string temperatureName = "temperatures";
    const std::vector<std::string> temperatureNames = {"temperature"};

    const std::string robotName = "yarp_robot_name";

    const std::string robotDescriptionList = "description_list";

    const std::string timestampsName = "timestamps";

    void waitForAcquisitionThreadsToPause();

    void resumeAcquisitionThreads();

    virtual bool saveData(const std::string& tag = "") override;
};

} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_FRAMEWORK_YARP_ROBOT_LOGGER_DEVICE_H
