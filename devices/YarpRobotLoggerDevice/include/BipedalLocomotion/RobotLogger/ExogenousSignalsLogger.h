/**
 * @copyright 2026 Generative Bionics S.R.L. This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_ROBOT_LOGGER_EXOGENOUS_SIGNALS_LOGGER_H
#define BIPEDAL_LOCOMOTION_ROBOT_LOGGER_EXOGENOUS_SIGNALS_LOGGER_H

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ContactStyle.h>
#include <yarp/os/Network.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/Vector.h>

#include <trintrin/msgs/HumanState.h>
#include <trintrin/msgs/WearableData.h>
#include <trintrin/msgs/WearableTargets.h>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/YarpUtilities/VectorsCollection.h>
#include <BipedalLocomotion/YarpUtilities/VectorsCollectionClient.h>

#include <BipedalLocomotion/RobotLogger/DataSink.h>
#include <BipedalLocomotion/RobotLogger/ImageRecorder.h>

namespace BipedalLocomotion
{
namespace RobotLogger
{

/**
 * Data shared by all the exogenous signals. The members that are not atomic are accessed only by
 * the logging thread. The monitor thread only manages the connection.
 */
struct ExogenousSignalBase
{
    std::mutex mutex;
    std::string signalName;
    bool dataArrived{false};
    std::atomic<bool> connected{false};
    std::atomic<unsigned int> connectionId{0}; /**< Incremented at every new connection. */
    unsigned int loggedConnectionId{0}; /**< Connection associated to channelNames. */
    std::unordered_map<std::string, std::string> channelNames; /**< signal key -> channel. */
    std::string connectionIdChannel;
};

template <typename T> struct ExogenousSignal : ExogenousSignalBase
{
    std::string remote;
    std::string local;
    std::string carrier;
    yarp::os::BufferedPort<T> port;

    bool connect()
    {
        return yarp::os::Network::connect(remote, local, carrier);
    }

    bool isConnected() const
    {
        yarp::os::ContactStyle style;
        style.quiet = true;
        style.timeout = 1.0;
        return yarp::os::Network::isConnected(remote, local, style);
    }

    void disconnect()
    {
        yarp::os::Network::disconnect(remote, local);
    }
};

template <typename T> struct ExogenousSignalWithMetadata : ExogenousSignal<T>
{
    YarpUtilities::VectorsCollectionMetadata metadata;
    YarpUtilities::VectorsCollection convertedSignal;
};

struct VectorsCollectionSignal : ExogenousSignalBase
{
    YarpUtilities::VectorsCollectionClient client;
    YarpUtilities::VectorsCollectionMetadata metadata;

    bool connect()
    {
        return client.connect();
    }

    bool isConnected() const
    {
        return client.isConnected();
    }

    void disconnect()
    {
        client.disconnect();
    }
};

/**
 * ExogenousSignalsLogger logs the signals streamed by other applications.
 *
 * A monitor thread connects the signals as soon as their ports are available, it detects when an
 * application is closed and reconnects the signal when the application is started again. Every
 * new connection increments the connection id of the signal, which is logged together with each
 * sample in the channel `exogenous_signals_connection_id::<signal_name>`. If the structure of a
 * signal changes in a new connection, the data is stored in the channel
 * `<channel>_connection_<id>`.
 */
class ExogenousSignalsLogger
{
public:
    ~ExogenousSignalsLogger();

    /**
     * @param params the `ExogenousSignals` group. If empty no signal is logged.
     */
    bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> params);

    /** Start monitoring the connections and recording the image signals. */
    bool start(DataStorage& storage);

    void record(DataSink& sink, double time);

    std::vector<ImageRecorder*> imageRecorders();

    /** Stop the monitor and the image recorders and disconnect all the signals. */
    void stop();

private:
    using ImageSignal = ExogenousSignal<yarp::sig::ImageOf<yarp::sig::PixelRgb>>;

    void monitorConnections();
    template <typename Signals> void monitorConnections(Signals& signals);

    bool resetOnNewConnection(ExogenousSignalBase& signal);
    bool addChannel(DataSink& sink,
                    ExogenousSignalBase& signal,
                    const std::string& key,
                    std::size_t size,
                    const std::vector<std::string>& elementNames);
    bool addConnectionIdChannel(DataSink& sink, ExogenousSignalBase& signal);
    void logConnectionId(DataSink& sink, const ExogenousSignalBase& signal, double time);

    void logVectorsCollections(DataSink& sink, double time);
    void logVectors(DataSink& sink, double time);
    void logStrings(DataSink& sink, double time);
    template <typename Signals> void logSignalsWithMetadata(Signals& signals, DataSink& sink, double time);

    std::unordered_map<std::string, VectorsCollectionSignal> m_vectorsCollectionSignals;
    std::unordered_map<std::string, ExogenousSignal<yarp::sig::Vector>> m_vectorSignals;
    std::unordered_map<std::string, ExogenousSignal<yarp::os::Bottle>> m_stringSignals;
    std::unordered_map<std::string, ImageSignal> m_imageSignals;
    std::unordered_map<std::string, ExogenousSignalWithMetadata<trintrin::msgs::HumanState>>
        m_humanStateSignals;
    std::unordered_map<std::string, ExogenousSignalWithMetadata<trintrin::msgs::WearableTargets>>
        m_wearableTargetsSignals;
    std::unordered_map<std::string, ExogenousSignalWithMetadata<trintrin::msgs::WearableData>>
        m_wearableDataSignals;

    std::vector<std::unique_ptr<ImageRecorder>> m_imageRecorders;

    std::atomic<bool> m_monitorIsRunning{false};
    std::thread m_monitorThread;
};

} // namespace RobotLogger
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_ROBOT_LOGGER_EXOGENOUS_SIGNALS_LOGGER_H
