/**
 * @copyright 2026 Generative Bionics S.R.L. This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <chrono>

#include <Eigen/Core>

#include <opencv2/imgproc.hpp>

#include <BipedalLocomotion/MessageConversionUtilities.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <BipedalLocomotion/RobotLogger/ExogenousSignalsLogger.h>

using namespace BipedalLocomotion::RobotLogger;

namespace
{
constexpr auto connectionIdName = "exogenous_signals_connection_id";

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

template <typename Signals>
bool openSignals(std::shared_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> ptr,
                 const std::string& listName,
                 bool required,
                 Signals& signals)
{
    constexpr auto logPrefix = "[ExogenousSignalsLogger::initialize]";
    using BipedalLocomotion::log;

    std::vector<std::string> inputs;
    if (!ptr->getParameter(listName, inputs))
    {
        if (required)
        {
            log()->error("{} Unable to get the parameter '{}'.", logPrefix, listName);
            return false;
        }
        log()->info("{} The parameter '{}' is not provided. Assuming none.", logPrefix, listName);
        return true;
    }

    for (const auto& input : inputs)
    {
        auto group = ptr->getGroup(input).lock();
        std::string local, remote, carrier, signalName;
        if (group == nullptr || !group->getParameter("local", local)
            || !group->getParameter("remote", remote) || !group->getParameter("carrier", carrier)
            || !group->getParameter("signal_name", signalName))
        {
            log()->error("{} Unable to get the parameters of the input {}.", logPrefix, input);
            return false;
        }

        auto& signal = signals[remote];
        signal.signalName = signalName;
        signal.remote = remote;
        signal.local = local;
        signal.carrier = carrier;
        if (!signal.port.open(local))
        {
            log()->error("{} Unable to open the port {}.", logPrefix, local);
            return false;
        }
    }
    return true;
}
} // namespace

ExogenousSignalsLogger::~ExogenousSignalsLogger()
{
    this->stop();
}

bool ExogenousSignalsLogger::initialize(
    std::weak_ptr<const ParametersHandler::IParametersHandler> params)
{
    constexpr auto logPrefix = "[ExogenousSignalsLogger::initialize]";

    auto ptr = params.lock();
    if (ptr == nullptr)
    {
        log()->info("{} No exogenous signal will be logged.", logPrefix);
        return true;
    }

    std::vector<std::string> inputs;
    if (!ptr->getParameter("vectors_collection_exogenous_inputs", inputs))
    {
        log()->error("{} Unable to get the parameter 'vectors_collection_exogenous_inputs'.",
                     logPrefix);
        return false;
    }

    for (const auto& input : inputs)
    {
        auto group = ptr->getGroup(input).lock();
        std::string remote, signalName;
        if (group == nullptr || !group->getParameter("remote", remote)
            || !group->getParameter("signal_name", signalName))
        {
            log()->error("{} Unable to get the parameters of the input {}.", logPrefix, input);
            return false;
        }

        auto& signal = m_vectorsCollectionSignals[remote];
        signal.signalName = signalName;
        if (!signal.client.initialize(group))
        {
            log()->error("{} Unable to initialize the client of the input {}.", logPrefix, input);
            return false;
        }
    }

    return openSignals(ptr, "vectors_exogenous_inputs", true, m_vectorSignals)
           && openSignals(ptr, "string_exogenous_inputs", false, m_stringSignals)
           && openSignals(ptr, "image_exogenous_inputs", false, m_imageSignals)
           && openSignals(ptr, "human_state_exogenous_inputs", false, m_humanStateSignals)
           && openSignals(ptr,
                          "wearable_targets_exogenous_inputs",
                          false,
                          m_wearableTargetsSignals)
           && openSignals(ptr, "wearable_data_exogenous_inputs", false, m_wearableDataSignals);
}

bool ExogenousSignalsLogger::start(DataStorage& storage)
{
    for (auto& [name, signal] : m_imageSignals)
    {
        ImageRecorderOptions options;
        options.name = signal.signalName;
        options.imageType = "rgb";
        options.channel = "exogenous_images::" + signal.signalName + "::rgb";
        options.saveMode = ImageSaveMode::Frames;
        auto recorder
            = std::make_unique<ImageRecorder>(options,
                                              std::make_unique<PortImageSource>(signal.port),
                                              storage);
        if (!recorder->start())
        {
            return false;
        }
        m_imageRecorders.push_back(std::move(recorder));
    }

    m_monitorIsRunning = true;
    m_monitorThread = std::thread([this] { this->monitorConnections(); });
    return true;
}

std::vector<ImageRecorder*> ExogenousSignalsLogger::imageRecorders()
{
    std::vector<ImageRecorder*> recorders;
    for (auto& recorder : m_imageRecorders)
    {
        recorders.push_back(recorder.get());
    }
    return recorders;
}

void ExogenousSignalsLogger::stop()
{
    m_monitorIsRunning = false;
    if (m_monitorThread.joinable())
    {
        m_monitorThread.join();
    }

    m_imageRecorders.clear();

    auto disconnect = [](auto& signals) {
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
    };

    disconnect(m_vectorsCollectionSignals);
    disconnect(m_vectorSignals);
    disconnect(m_stringSignals);
    disconnect(m_imageSignals);
    disconnect(m_humanStateSignals);
    disconnect(m_wearableTargetsSignals);
    disconnect(m_wearableDataSignals);
}

template <typename Signals> void ExogenousSignalsLogger::monitorConnections(Signals& signals)
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

void ExogenousSignalsLogger::monitorConnections()
{
    using namespace std::chrono_literals;
    constexpr auto period = 1s;
    constexpr auto sleepStep = 50ms;

    while (m_monitorIsRunning)
    {
        this->monitorConnections(m_vectorsCollectionSignals);
        this->monitorConnections(m_vectorSignals);
        this->monitorConnections(m_stringSignals);
        this->monitorConnections(m_imageSignals);
        this->monitorConnections(m_humanStateSignals);
        this->monitorConnections(m_wearableTargetsSignals);
        this->monitorConnections(m_wearableDataSignals);

        for (auto slept = 0ms; slept < period && m_monitorIsRunning; slept += sleepStep)
        {
            std::this_thread::sleep_for(sleepStep);
        }
    }
}

bool ExogenousSignalsLogger::resetOnNewConnection(ExogenousSignalBase& signal)
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

bool ExogenousSignalsLogger::addChannel(DataSink& sink,
                                        ExogenousSignalBase& signal,
                                        const std::string& key,
                                        std::size_t size,
                                        const std::vector<std::string>& elementNames)
{
    if (signal.channelNames.find(key) != signal.channelNames.end())
    {
        return true;
    }

    // The channel may exist from a previous connection. It is reused only if the structure of
    // the signal did not change.
    std::string channel = key;
    const auto existing = sink.storage().getChannel(key);
    const bool changed = existing.has_value()
                         && !(*existing == DataSink::vectorDescriptor(size, elementNames));
    if (changed)
    {
        channel = key + "_connection_" + std::to_string(signal.loggedConnectionId);
    }

    if (!sink.addChannel(channel, size, elementNames))
    {
        return false;
    }

    if (changed)
    {
        log()->warn("[ExogenousSignalsLogger::addChannel] The structure of '{}' changed in the "
                    "connection {} of the exogenous signal '{}'. The data will be stored in '{}'.",
                    key,
                    signal.loggedConnectionId,
                    signal.signalName,
                    channel);
    }

    signal.channelNames[key] = channel;
    return true;
}

bool ExogenousSignalsLogger::addConnectionIdChannel(DataSink& sink, ExogenousSignalBase& signal)
{
    const std::string key = std::string(connectionIdName) + treeDelim + signal.signalName;
    if (!this->addChannel(sink, signal, key, 1, {"connection_id"}))
    {
        return false;
    }
    signal.connectionIdChannel = signal.channelNames[key];
    return true;
}

void ExogenousSignalsLogger::logConnectionId(DataSink& sink,
                                             const ExogenousSignalBase& signal,
                                             double time)
{
    Eigen::Matrix<double, 1, 1> connectionId;
    connectionId << static_cast<double>(signal.loggedConnectionId);
    sink.log(signal.connectionIdChannel, connectionId, time);
}

void ExogenousSignalsLogger::logVectorsCollections(DataSink& sink, double time)
{
    for (auto& [remote, signal] : m_vectorsCollectionSignals)
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
            signal.dataArrived = this->addConnectionIdChannel(sink, signal);
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
                    || !this->addChannel(sink, signal, name, vector.size(), metadata->second))
                {
                    continue;
                }
                channel = signal.channelNames.find(name);
            }
            sink.log(channel->second, vector, time);
            logged = true;
        }

        if (logged)
        {
            this->logConnectionId(sink, signal, time);
        }
    }
}

void ExogenousSignalsLogger::logVectors(DataSink& sink, double time)
{
    for (auto& [remote, signal] : m_vectorSignals)
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
            signal.dataArrived
                = this->addChannel(sink, signal, signal.signalName, vector->size(), {})
                  && this->addConnectionIdChannel(sink, signal);
            if (!signal.dataArrived)
            {
                continue;
            }
        }

        sink.log(signal.channelNames[signal.signalName], *vector, time);
        this->logConnectionId(sink, signal, time);
    }
}

void ExogenousSignalsLogger::logStrings(DataSink& sink, double time)
{
    // the strings are not streamed in real time
    for (auto& [remote, signal] : m_stringSignals)
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
            signal.dataArrived = sink.storage().addChannel(signal.signalName, {{1}, {}})
                                 && this->addConnectionIdChannel(sink, signal);
            if (!signal.dataArrived)
            {
                continue;
            }
        }

        sink.storage().push(signal.signalName, bottle->toString(), time);
        this->logConnectionId(sink, signal, time);
    }
}

template <typename Signals>
void ExogenousSignalsLogger::logSignalsWithMetadata(Signals& signals, DataSink& sink, double time)
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

            bool channelsAdded = this->addConnectionIdChannel(sink, signal);
            for (const auto& [key, elementNames] : signal.metadata.vectors)
            {
                channelsAdded = channelsAdded
                                && this->addChannel(sink,
                                                    signal,
                                                    key,
                                                    elementNames.size(),
                                                    elementNames);
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
                sink.log(channel->second, vector, time);
            }
        }
        this->logConnectionId(sink, signal, time);
    }
}

void ExogenousSignalsLogger::record(DataSink& sink, double time)
{
    this->logVectorsCollections(sink, time);
    this->logVectors(sink, time);
    this->logStrings(sink, time);
    this->logSignalsWithMetadata(m_humanStateSignals, sink, time);
    this->logSignalsWithMetadata(m_wearableTargetsSignals, sink, time);
    this->logSignalsWithMetadata(m_wearableDataSignals, sink, time);
}
