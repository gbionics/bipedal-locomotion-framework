/**
 * @file ExogenousSignalsLogger.cpp
 * @copyright 2026 Generative Bionics S.R.L. This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>

#include <Eigen/Core>

#include <opencv2/imgproc.hpp>

#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Contact.h>
#include <yarp/os/Network.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/Vector.h>

#include <trintrin/msgs/HumanState.h>
#include <trintrin/msgs/WearableData.h>
#include <trintrin/msgs/WearableTargets.h>

#include <BipedalLocomotion/MessageConversionUtilities.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>
#include <BipedalLocomotion/TextLogging/Logger.h>
#include <BipedalLocomotion/YarpUtilities/VectorsCollection.h>
#include <BipedalLocomotion/YarpUtilities/VectorsCollectionClient.h>

#include <BipedalLocomotion/RobotLogger/ExogenousSignalsLogger.h>

using namespace BipedalLocomotion::RobotLogger;
using namespace BipedalLocomotion;

namespace
{
constexpr auto connectionIdName = "exogenous_signals_connection_id";

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
    yarp::os::Contact remoteContact; /**< Contact of the remote port at connection time. */

    bool connect()
    {
        remoteContact = yarp::os::Network::queryName(remote);
        return remoteContact.isValid() && yarp::os::Network::connect(remote, local, carrier);
    }

    // The remote port is not contacted, since it may block the application streaming the signal.
    bool isConnected()
    {
        if (port.getInputCount() == 0)
        {
            return false;
        }
        // a restarted application registers the port with a different contact
        const yarp::os::Contact contact = yarp::os::Network::queryName(remote);
        return contact.isValid() && contact.getHost() == remoteContact.getHost()
               && contact.getPort() == remoteContact.getPort();
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

using ImageSignal = ExogenousSignal<yarp::sig::ImageOf<yarp::sig::PixelRgb>>;

/** Images streamed on a yarp port. The read blocks until an image arrives. */
class PortImageSource final : public IImageSource
{
public:
    explicit PortImageSource(yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>>& port)
        : m_port(port)
    {
    }

    bool read(cv::Mat& image) final
    {
        auto* yarpImage = m_port.read(true);
        if (yarpImage == nullptr)
        {
            return false;
        }

        const cv::Mat rgb(yarpImage->height(),
                          yarpImage->width(),
                          CV_8UC3,
                          yarpImage->getRawImage(),
                          yarpImage->getRowSize());
        cv::cvtColor(rgb, image, cv::COLOR_RGB2BGR);
        return true;
    }

    void interrupt() final
    {
        m_port.interrupt();
    }

    void resume() final
    {
        m_port.resume();
    }

private:
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>>& m_port;
};

struct SignalDescription
{
    std::string signalName;
    std::string remote;
    std::string local;
    std::string carrier;
};

/**
 * Read the description of the signal stored in the group named input. Only `remote` is required,
 * `signal_name` defaults to the group name, `local` to
 * `<port_prefix>/exogenous_signals/<signal_name>` and `carrier` to udp.
 */
bool getSignalDescription(const ParametersHandler::IParametersHandler& handler,
                          const std::string& input,
                          const std::string& portPrefix,
                          SignalDescription& description)
{
    constexpr auto logPrefix = "[ExogenousSignalsLogger::initialize]";

    auto group = handler.getGroup(input).lock();
    if (group == nullptr || !group->getParameter("remote", description.remote))
    {
        log()->error("{} Unable to get the parameter 'remote' of the input {}.", logPrefix, input);
        return false;
    }

    description.signalName = input;
    group->getParameter("signal_name", description.signalName);
    description.local = portPrefix + "/exogenous_signals/" + description.signalName;
    group->getParameter("local", description.local);
    description.carrier = "udp";
    group->getParameter("carrier", description.carrier);
    return true;
}

std::vector<std::string> getInputs(const ParametersHandler::IParametersHandler& handler,
                                   const std::string& listName)
{
    std::vector<std::string> inputs;
    if (!handler.getParameter(listName, inputs))
    {
        log()->debug("[ExogenousSignalsLogger::initialize] The parameter '{}' is not provided. "
                     "Assuming none.",
                     listName);
    }
    return inputs;
}

template <typename Signals>
bool openSignals(const ParametersHandler::IParametersHandler& handler,
                 const std::string& listName,
                 const std::string& portPrefix,
                 Signals& signals)
{
    constexpr auto logPrefix = "[ExogenousSignalsLogger::initialize]";

    for (const auto& input : getInputs(handler, listName))
    {
        SignalDescription description;
        if (!getSignalDescription(handler, input, portPrefix, description))
        {
            return false;
        }

        auto& signal = signals[description.remote];
        signal.signalName = description.signalName;
        signal.remote = description.remote;
        signal.local = description.local;
        signal.carrier = description.carrier;
        if (!signal.port.open(description.local))
        {
            log()->error("{} Unable to open the port {}.", logPrefix, description.local);
            return false;
        }
    }
    return true;
}

} // namespace

struct ExogenousSignalsLogger::Impl
{
    std::shared_ptr<TelemetryBuffer> buffer;

    std::unordered_map<std::string, VectorsCollectionSignal> vectorsCollectionSignals;
    std::unordered_map<std::string, ExogenousSignal<yarp::sig::Vector>> vectorSignals;
    std::unordered_map<std::string, ExogenousSignal<yarp::os::Bottle>> stringSignals;
    std::unordered_map<std::string, ImageSignal> imageSignals;
    std::unordered_map<std::string, ExogenousSignalWithMetadata<trintrin::msgs::HumanState>>
        humanStateSignals;
    std::unordered_map<std::string, ExogenousSignalWithMetadata<trintrin::msgs::WearableTargets>>
        wearableTargetsSignals;
    std::unordered_map<std::string, ExogenousSignalWithMetadata<trintrin::msgs::WearableData>>
        wearableDataSignals;

    std::vector<std::shared_ptr<ImageRecorder>> imageRecorders;

    std::atomic<bool> monitorIsRunning{false};
    std::thread monitorThread;

    template <typename Signals> void monitorConnections(Signals& signals)
    {
        constexpr auto logPrefix = "[ExogenousSignalsLogger::monitorConnections]";

        // The signal mutex is not locked: the network operations may take time and the logging
        // thread reads a signal only if it is connected.
        for (auto& [remote, signal] : signals)
        {
            if (signal.connected)
            {
                if (signal.isConnected())
                {
                    continue;
                }

                // e.g., the application streaming the signal has been closed
                signal.connected = false;
                signal.disconnect();
                log()->warn("{} The exogenous signal '{}' has been disconnected. It will be "
                            "reconnected as soon as the port {} is available again.",
                            logPrefix,
                            signal.signalName,
                            remote);
                continue;
            }

            if (signal.connect())
            {
                // the id must be updated before the logging thread reads the signal
                const unsigned int id = ++signal.connectionId;
                signal.connected = true;
                log()->info("{} Connected to the exogenous signal '{}' (connection {}).",
                            logPrefix,
                            signal.signalName,
                            id);
            }
        }
    }

    void monitorConnections()
    {
        using namespace std::chrono_literals;
        constexpr auto period = 1s;
        constexpr auto sleepStep = 50ms;

        while (monitorIsRunning)
        {
            this->monitorConnections(vectorsCollectionSignals);
            this->monitorConnections(vectorSignals);
            this->monitorConnections(stringSignals);
            this->monitorConnections(imageSignals);
            this->monitorConnections(humanStateSignals);
            this->monitorConnections(wearableTargetsSignals);
            this->monitorConnections(wearableDataSignals);

            for (auto slept = 0ms; slept < period && monitorIsRunning; slept += sleepStep)
            {
                std::this_thread::sleep_for(sleepStep);
            }
        }
    }

    /** Forget the channels of the previous connection. It returns true if they are forgotten. */
    bool resetOnNewConnection(ExogenousSignalBase& signal)
    {
        const unsigned int connectionId = signal.connectionId.load();
        if (connectionId == signal.loggedConnectionId)
        {
            return false;
        }

        signal.loggedConnectionId = connectionId;
        signal.dataArrived = false;
        signal.channelNames.clear();
        signal.connectionIdChannel.clear();
        return true;
    }

    bool addChannel(ExogenousSignalBase& signal,
                    const std::string& key,
                    std::size_t size,
                    const std::vector<std::string>& elementNames)
    {
        if (signal.channelNames.find(key) != signal.channelNames.end())
        {
            return true;
        }

        // The channel may exist from a previous connection. It is reused only if the structure
        // of the signal did not change.
        const bool changed = !buffer->isChannelCompatible(key, size, elementNames);
        const std::string channel
            = changed ? key + "_connection_" + std::to_string(signal.loggedConnectionId) : key;

        if (!buffer->addChannel(channel, size, elementNames))
        {
            return false;
        }

        if (changed)
        {
            log()->warn("[ExogenousSignalsLogger::addChannel] The structure of '{}' changed in "
                        "the connection {} of the exogenous signal '{}'. The data will be stored "
                        "in '{}'.",
                        key,
                        signal.loggedConnectionId,
                        signal.signalName,
                        channel);
        }

        signal.channelNames[key] = channel;
        return true;
    }

    bool addConnectionIdChannel(ExogenousSignalBase& signal)
    {
        const std::string key = std::string(connectionIdName) + treeDelim + signal.signalName;
        if (!this->addChannel(signal, key, 1, {"connection_id"}))
        {
            return false;
        }
        signal.connectionIdChannel = signal.channelNames[key];
        return true;
    }

    void logConnectionId(const ExogenousSignalBase& signal, double time)
    {
        Eigen::Matrix<double, 1, 1> connectionId;
        connectionId << static_cast<double>(signal.loggedConnectionId);
        buffer->push(signal.connectionIdChannel, connectionId, time);
    }

    void logVectorsCollections(double time)
    {
        for (auto& [remote, signal] : vectorsCollectionSignals)
        {
            if (!signal.connected)
            {
                continue;
            }

            std::lock_guard lock(signal.mutex);
            if (this->resetOnNewConnection(signal))
            {
                signal.metadata.vectors.clear();
            }

            const auto* collection = signal.client.readData(false);
            if (collection == nullptr)
            {
                continue;
            }

            // the metadata is retrieved with the first data and every time the server updates it
            if ((signal.metadata.vectors.empty() || signal.client.isNewMetadataAvailable())
                && !signal.client.getMetadata(signal.metadata) && signal.metadata.vectors.empty())
            {
                continue;
            }

            if (!signal.dataArrived)
            {
                signal.dataArrived = this->addConnectionIdChannel(signal);
                if (!signal.dataArrived)
                {
                    continue;
                }
            }

            bool logged = false;
            for (const auto& [key, vector] : collection->vectors)
            {
                const std::string name = signal.signalName + treeDelim + key;
                auto channel = signal.channelNames.find(name);
                if (channel == signal.channelNames.end())
                {
                    const auto metadata = signal.metadata.vectors.find(key);
                    if (metadata == signal.metadata.vectors.cend()
                        || !this->addChannel(signal, name, vector.size(), metadata->second))
                    {
                        continue;
                    }
                    channel = signal.channelNames.find(name);
                }
                buffer->push(channel->second, vector, time);
                logged = true;
            }

            if (logged)
            {
                this->logConnectionId(signal, time);
            }
        }
    }

    void logVectors(double time)
    {
        for (auto& [remote, signal] : vectorSignals)
        {
            if (!signal.connected)
            {
                continue;
            }

            std::lock_guard lock(signal.mutex);
            this->resetOnNewConnection(signal);

            const yarp::sig::Vector* vector = signal.port.read(false);
            if (vector == nullptr)
            {
                continue;
            }

            if (!signal.dataArrived)
            {
                signal.dataArrived = this->addChannel(signal, signal.signalName, vector->size(), {})
                                     && this->addConnectionIdChannel(signal);
                if (!signal.dataArrived)
                {
                    continue;
                }
            }

            buffer->push(signal.channelNames[signal.signalName], *vector, time);
            this->logConnectionId(signal, time);
        }
    }

    void logStrings(double time)
    {
        // the strings are not streamed in real time
        for (auto& [remote, signal] : stringSignals)
        {
            if (!signal.connected)
            {
                continue;
            }

            std::lock_guard lock(signal.mutex);
            this->resetOnNewConnection(signal);

            const yarp::os::Bottle* bottle = signal.port.read(false);
            if (bottle == nullptr)
            {
                continue;
            }

            if (!signal.dataArrived)
            {
                signal.dataArrived = buffer->addStoredChannel(signal.signalName, {1})
                                     && this->addConnectionIdChannel(signal);
                if (!signal.dataArrived)
                {
                    continue;
                }
            }

            buffer->push(signal.signalName, bottle->toString(), time);
            this->logConnectionId(signal, time);
        }
    }

    template <typename Signals> void logSignalsWithMetadata(Signals& signals, double time)
    {
        for (auto& [remote, signal] : signals)
        {
            if (!signal.connected)
            {
                continue;
            }

            std::lock_guard lock(signal.mutex);
            if (this->resetOnNewConnection(signal))
            {
                signal.metadata.vectors.clear();
            }

            const auto* message = signal.port.read(false);
            if (message == nullptr)
            {
                continue;
            }

            if (!signal.dataArrived)
            {
                if (signal.metadata.vectors.empty())
                {
                    extractMetadata(*message, signal.signalName, signal.metadata);
                }

                bool channelsAdded = this->addConnectionIdChannel(signal);
                for (const auto& [key, elementNames] : signal.metadata.vectors)
                {
                    channelsAdded
                        = channelsAdded
                          && this->addChannel(signal, key, elementNames.size(), elementNames);
                }
                signal.dataArrived = channelsAdded;
                if (!signal.dataArrived)
                {
                    continue;
                }
            }

            convertToVectorsCollection(*message, signal.signalName, signal.convertedSignal);
            for (const auto& [key, vector] : signal.convertedSignal.vectors)
            {
                const auto channel = signal.channelNames.find(key);
                if (channel != signal.channelNames.end())
                {
                    buffer->push(channel->second, vector, time);
                }
            }
            this->logConnectionId(signal, time);
        }
    }

    template <typename Signals> static void disconnect(Signals& signals)
    {
        for (auto& [name, signal] : signals)
        {
            if (signal.connected)
            {
                signal.disconnect();
            }
            signal.connected = false;
            signal.connectionId = 0;

            std::lock_guard lock(signal.mutex);
            signal.loggedConnectionId = 0;
            signal.dataArrived = false;
            signal.channelNames.clear();
            signal.connectionIdChannel.clear();
        }
    }
};

ExogenousSignalsLogger::ExogenousSignalsLogger()
    : m_pimpl(std::make_unique<Impl>())
{
}

ExogenousSignalsLogger::~ExogenousSignalsLogger()
{
    this->stop();
}

bool ExogenousSignalsLogger::initialize(
    std::weak_ptr<const ParametersHandler::IParametersHandler> handler,
    std::shared_ptr<TelemetryBuffer> buffer,
    const std::string& portPrefix)
{
    constexpr auto logPrefix = "[ExogenousSignalsLogger::initialize]";

    if (buffer == nullptr)
    {
        log()->error("{} The buffer is not valid.", logPrefix);
        return false;
    }
    m_pimpl->buffer = buffer;

    auto ptr = handler.lock();
    if (ptr == nullptr)
    {
        log()->info("{} No exogenous signal will be logged.", logPrefix);
        return true;
    }

    for (const auto& input : getInputs(*ptr, "vectors_collection_exogenous_inputs"))
    {
        SignalDescription description;
        if (!getSignalDescription(*ptr, input, portPrefix, description))
        {
            return false;
        }

        auto clientHandler = std::make_shared<ParametersHandler::StdImplementation>();
        clientHandler->setParameter("remote", description.remote);
        clientHandler->setParameter("local", description.local);
        clientHandler->setParameter("carrier", description.carrier);

        auto& signal = m_pimpl->vectorsCollectionSignals[description.remote];
        signal.signalName = description.signalName;
        if (!signal.client.initialize(clientHandler))
        {
            log()->error("{} Unable to initialize the client of the input {}.", logPrefix, input);
            return false;
        }
    }

    if (!openSignals(*ptr, "vectors_exogenous_inputs", portPrefix, m_pimpl->vectorSignals)
        || !openSignals(*ptr, "string_exogenous_inputs", portPrefix, m_pimpl->stringSignals)
        || !openSignals(*ptr, "image_exogenous_inputs", portPrefix, m_pimpl->imageSignals)
        || !openSignals(*ptr,
                        "human_state_exogenous_inputs",
                        portPrefix,
                        m_pimpl->humanStateSignals)
        || !openSignals(*ptr,
                        "wearable_targets_exogenous_inputs",
                        portPrefix,
                        m_pimpl->wearableTargetsSignals)
        || !openSignals(*ptr,
                        "wearable_data_exogenous_inputs",
                        portPrefix,
                        m_pimpl->wearableDataSignals))
    {
        return false;
    }

    for (auto& [remote, signal] : m_pimpl->imageSignals)
    {
        auto recorderHandler = std::make_shared<ParametersHandler::StdImplementation>();
        recorderHandler->setParameter("name", signal.signalName);
        recorderHandler->setParameter("image_type", "rgb");
        recorderHandler->setParameter("channel",
                                      "exogenous_images::" + signal.signalName + "::rgb");
        recorderHandler->setParameter("save_mode", "frame");

        auto recorder = std::make_shared<ImageRecorder>();
        if (!recorder->initialize(recorderHandler,
                                  std::make_unique<PortImageSource>(signal.port),
                                  buffer))
        {
            log()->error("{} Unable to initialize the recorder of the image signal {}.",
                         logPrefix,
                         signal.signalName);
            return false;
        }
        m_pimpl->imageRecorders.push_back(std::move(recorder));
    }

    return true;
}

bool ExogenousSignalsLogger::start()
{
    for (const auto& recorder : m_pimpl->imageRecorders)
    {
        if (!recorder->start())
        {
            return false;
        }
    }

    m_pimpl->monitorIsRunning = true;
    m_pimpl->monitorThread = std::thread([this] { m_pimpl->monitorConnections(); });
    return true;
}

void ExogenousSignalsLogger::record(double time)
{
    m_pimpl->logVectorsCollections(time);
    m_pimpl->logVectors(time);
    m_pimpl->logStrings(time);
    m_pimpl->logSignalsWithMetadata(m_pimpl->humanStateSignals, time);
    m_pimpl->logSignalsWithMetadata(m_pimpl->wearableTargetsSignals, time);
    m_pimpl->logSignalsWithMetadata(m_pimpl->wearableDataSignals, time);
}

void ExogenousSignalsLogger::stop()
{
    m_pimpl->monitorIsRunning = false;
    if (m_pimpl->monitorThread.joinable())
    {
        m_pimpl->monitorThread.join();
    }

    for (const auto& recorder : m_pimpl->imageRecorders)
    {
        recorder->stop();
    }

    Impl::disconnect(m_pimpl->vectorsCollectionSignals);
    Impl::disconnect(m_pimpl->vectorSignals);
    Impl::disconnect(m_pimpl->stringSignals);
    Impl::disconnect(m_pimpl->imageSignals);
    Impl::disconnect(m_pimpl->humanStateSignals);
    Impl::disconnect(m_pimpl->wearableTargetsSignals);
    Impl::disconnect(m_pimpl->wearableDataSignals);
}

const std::vector<std::shared_ptr<ImageRecorder>>& ExogenousSignalsLogger::getImageRecorders() const
{
    return m_pimpl->imageRecorders;
}
