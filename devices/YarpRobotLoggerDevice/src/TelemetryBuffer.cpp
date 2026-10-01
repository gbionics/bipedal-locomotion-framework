/**
 * @file TelemetryBuffer.cpp
 * @copyright 2026 Generative Bionics S.R.L. This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdlib>
#include <mutex>
#include <thread>
#include <unordered_map>

#include <robometry/BufferManager.h>

#include <BipedalLocomotion/TextLogging/Logger.h>
#include <BipedalLocomotion/YarpUtilities/VectorsCollectionServer.h>

#include <BipedalLocomotion/RobotLogger/TelemetryBuffer.h>

VISITABLE_STRUCT(BipedalLocomotion::TextLoggingEntry,
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

using namespace BipedalLocomotion::RobotLogger;
using namespace BipedalLocomotion;

namespace
{
constexpr auto realTimeRootName = "robot_realtime";
constexpr auto timestampsName = "timestamps";

std::string realTimeName(const std::string& name)
{
    return std::string(realTimeRootName) + "::" + name;
}
} // namespace

struct TelemetryBuffer::Impl
{
    struct ChannelDescriptor
    {
        std::vector<std::size_t> dimensions;
        std::vector<std::string> elementNames;
        bool streamed{false};
    };

    mutable std::mutex mutex; /**< Protects the structure of the buffer. */
    robometry::BufferManager bufferManager;
    robometry::BufferConfig config;
    std::unordered_map<std::string, ChannelDescriptor> channels;
    bool saveInProgress{false};

    std::filesystem::path logFolder;
    bool savePeriodically{true};

    std::mutex saveMutex;
    SaveCallback saveCallback;
    std::chrono::steady_clock::time_point lastSaveTime;

    std::thread periodicSaveThread;
    std::mutex periodicSaveMutex;
    std::condition_variable periodicSaveCv;
    bool stopPeriodicSave{false};

    std::unique_ptr<YarpUtilities::VectorsCollectionServer> realTimeServer; /**< Null if the real
                                                                               time streaming is
                                                                               disabled. */
    std::unordered_map<std::string, std::vector<std::string>> realTimeMetadata;

    static ChannelDescriptor
    vectorDescriptor(std::size_t size, const std::vector<std::string>& elementNames)
    {
        const bool hasNames = elementNames.size() == size;
        return {{size, 1}, hasNames ? elementNames : std::vector<std::string>{}};
    }

    bool addChannel(const std::string& name, const ChannelDescriptor& descriptor)
    {
        std::lock_guard lock(mutex);

        const auto channel = channels.find(name);
        if (channel != channels.end())
        {
            if (channel->second.dimensions == descriptor.dimensions
                && channel->second.elementNames == descriptor.elementNames)
            {
                return true;
            }
            log()->error("[TelemetryBuffer::addChannel] The channel {} already exists with a "
                         "different structure.",
                         name);
            return false;
        }

        // the buffer cannot be modified while it is written to file
        if (saveInProgress)
        {
            return false;
        }

        if (!bufferManager.addChannel({name, descriptor.dimensions, descriptor.elementNames}))
        {
            log()->error("[TelemetryBuffer::addChannel] Unable to add the channel {}.", name);
            return false;
        }

        channels.emplace(name, descriptor);
        return true;
    }

    /**
     * Add the real time metadata. Adding again the same metadata succeeds.
     * @param added true if the metadata has been added now.
     */
    bool addRealTimeMetadata(const std::string& name,
                             const std::vector<std::string>& metadata,
                             bool& added)
    {
        added = false;
        const std::string key = realTimeName(name);
        const auto existing = realTimeMetadata.find(key);
        if (existing != realTimeMetadata.end())
        {
            if (existing->second == metadata)
            {
                return true;
            }
            log()->error("[TelemetryBuffer::addRealTimeMetadata] The signal {} has been already "
                         "added with different metadata.",
                         key);
            return false;
        }

        if (!realTimeServer->populateMetadata(key, metadata))
        {
            log()->error("[TelemetryBuffer::addRealTimeMetadata] Unable to add the metadata of "
                         "{}.",
                         key);
            return false;
        }

        realTimeMetadata.emplace(key, metadata);
        added = true;
        return true;
    }

    bool save(const std::string& fileNamePrefix, std::string& savedFileName)
    {
        constexpr auto logPrefix = "[TelemetryBuffer::save]";

        std::lock_guard saveLock(saveMutex);

        // the files are indexed with a resolution of one second
        using namespace std::chrono_literals;
        const auto elapsed = std::chrono::steady_clock::now() - lastSaveTime;
        if (elapsed < 1s)
        {
            std::this_thread::sleep_for(1s - elapsed);
        }

        const auto start = std::chrono::steady_clock::now();
        {
            std::lock_guard lock(mutex);
            saveInProgress = true;
            bufferManager.setFileName(fileNamePrefix);
        }

        // the data is pushed in the meantime, the buffer manager moves it out with a short lock
        const bool ok = bufferManager.saveToFile(savedFileName);

        {
            std::lock_guard lock(mutex);
            bufferManager.setFileName(defaultFilePrefix);
            saveInProgress = false;
        }
        lastSaveTime = std::chrono::steady_clock::now();

        if (!ok)
        {
            log()->warn("{} No data saved. Either no data was logged or the file could not be "
                        "written in {}.",
                        logPrefix,
                        logFolder.string());
            return false;
        }

        log()->info("{} Data saved to file {}.mat in {}.",
                    logPrefix,
                    savedFileName,
                    std::chrono::duration<double>(lastSaveTime - start));

        if (saveCallback)
        {
            saveCallback(savedFileName);
        }

        return true;
    }

    void periodicSave()
    {
        const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(config.save_period));

        std::unique_lock lock(periodicSaveMutex);
        while (!periodicSaveCv.wait_for(lock, period, [this] { return stopPeriodicSave; }))
        {
            lock.unlock();
            std::string fileName;
            this->save(defaultFilePrefix, fileName);
            lock.lock();
        }
    }
};

TelemetryBuffer::TelemetryBuffer()
    : m_pimpl(std::make_unique<Impl>())
{
}

TelemetryBuffer::~TelemetryBuffer()
{
    this->stopPeriodicSave();
}

bool TelemetryBuffer::initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler)
{
    constexpr auto logPrefix = "[TelemetryBuffer::initialize]";

    auto ptr = handler.lock();
    if (ptr == nullptr)
    {
        log()->error("{} The handler is not valid.", logPrefix);
        return false;
    }

    double samplingPeriod{0.01};
    if (!ptr->getParameter("sampling_period_in_s", samplingPeriod))
    {
        log()->info("{} The parameter 'sampling_period_in_s' is not provided. Default value: {}.",
                    logPrefix,
                    samplingPeriod);
    }

    bool enableRealTimeLogging{false};
    if (!ptr->getParameter("enable_real_time_logging", enableRealTimeLogging))
    {
        log()->error("{} Unable to get the parameter 'enable_real_time_logging'.", logPrefix);
        return false;
    }

    robometry::BufferConfig config;
    if (!ptr->getParameter("robot_model_uri", config.yarp_robot_name))
    {
        log()->warn("{} The parameter 'robot_model_uri' is not provided. The robot model will not "
                    "be associated to the logged data.",
                    logPrefix);
    }
    config.filename = defaultFilePrefix;
    // the files are saved by this class, robometry is used only as buffer
    config.auto_save = false;
    config.save_periodically = false;
    config.file_indexing = "%Y_%m_%d_%H_%M_%S";
    config.mat_file_version = matioCpp::FileVersion::MAT7_3;
    config.save_period = 1800.0;

    std::string logFolder;
    auto telemetry = ptr->getGroup("Telemetry").lock();
    if (telemetry == nullptr)
    {
        log()->info("{} The group 'Telemetry' is not provided. The default values will be used.",
                    logPrefix);
    } else
    {
        if (!telemetry->getParameter("log_folder", logFolder))
        {
            log()->info("{} The parameter 'log_folder' is not provided. The files will be saved "
                        "in the current working directory.",
                        logPrefix);
        }
        if (!telemetry->getParameter("save_period", config.save_period))
        {
            log()->info("{} The parameter 'save_period' is not provided. Default value: {}.",
                        logPrefix,
                        config.save_period);
        }
        if (!telemetry->getParameter("save_periodically", m_pimpl->savePeriodically))
        {
            log()->info("{} The parameter 'save_periodically' is not provided. Default value: {}.",
                        logPrefix,
                        m_pimpl->savePeriodically);
        }
    }

    if (config.save_period <= 0 || samplingPeriod <= 0)
    {
        log()->error("{} The save period and the sampling period must be positive.", logPrefix);
        return false;
    }

    if (!logFolder.empty() && logFolder.front() == '~')
    {
        const char* home = std::getenv("HOME");
        if (home == nullptr)
        {
            log()->error("{} Unable to expand '~' in {}. HOME is not set.", logPrefix, logFolder);
            return false;
        }
        logFolder = std::string(home) + logFolder.substr(1);
    }

    m_pimpl->logFolder = logFolder.empty() ? std::filesystem::current_path()
                                           : std::filesystem::absolute(logFolder);

    std::error_code ec;
    std::filesystem::create_directories(m_pimpl->logFolder, ec);
    if (ec || !std::filesystem::is_directory(m_pimpl->logFolder))
    {
        log()->error("{} Unable to create the log folder {}. Error: {}.",
                     logPrefix,
                     m_pimpl->logFolder.string(),
                     ec.message());
        return false;
    }
    log()->info("{} The logged data will be saved in {}.", logPrefix, m_pimpl->logFolder.string());

    // robometry concatenates path and file name
    config.path = (m_pimpl->logFolder / "").string();

    // the buffer must contain the samples pushed between two saves, a 10% margin is added
    constexpr double margin = 0.1;
    config.n_samples
        = static_cast<std::size_t>(std::ceil((1 + margin) * config.save_period / samplingPeriod));

    {
        std::lock_guard lock(m_pimpl->mutex);
        m_pimpl->config = config;
        if (!m_pimpl->bufferManager.configure(m_pimpl->config))
        {
            log()->error("{} Unable to configure the buffer.", logPrefix);
            return false;
        }
    }

    if (!enableRealTimeLogging)
    {
        log()->info("{} Real time logging not activated.", logPrefix);
        return true;
    }

    m_pimpl->realTimeServer = std::make_unique<YarpUtilities::VectorsCollectionServer>();
    if (!m_pimpl->realTimeServer->initialize(ptr->getGroup("REAL_TIME_STREAMING")))
    {
        log()->error("{} Unable to initialize the real time streaming. Please check the group "
                     "'REAL_TIME_STREAMING'.",
                     logPrefix);
        return false;
    }

    bool added{false};
    if (!m_pimpl->addRealTimeMetadata("yarp_robot_name", {config.yarp_robot_name}, added)
        || !m_pimpl->addRealTimeMetadata(timestampsName, {timestampsName}, added)
        || !m_pimpl->realTimeServer->finalizeMetadata())
    {
        log()->error("{} Unable to add the real time metadata.", logPrefix);
        return false;
    }

    log()->info("{} Real time logging activated.", logPrefix);
    return true;
}

const std::filesystem::path& TelemetryBuffer::getLogFolder() const
{
    return m_pimpl->logFolder;
}

void TelemetryBuffer::setSaveCallback(SaveCallback callback)
{
    std::lock_guard lock(m_pimpl->saveMutex);
    m_pimpl->saveCallback = std::move(callback);
}

bool TelemetryBuffer::setDescriptionList(const std::vector<std::string>& descriptionList)
{
    {
        std::lock_guard lock(m_pimpl->mutex);
        m_pimpl->config.description_list = descriptionList;
        m_pimpl->bufferManager.setDescriptionList(descriptionList);
    }

    if (m_pimpl->realTimeServer == nullptr)
    {
        return true;
    }

    bool added{false};
    if (!m_pimpl->addRealTimeMetadata("description_list", descriptionList, added))
    {
        return false;
    }
    return !added || m_pimpl->realTimeServer->finalizeMetadata();
}

bool TelemetryBuffer::addChannel(const std::string& name,
                                 std::size_t size,
                                 const std::vector<std::string>& elementNames)
{
    auto descriptor = Impl::vectorDescriptor(size, elementNames);
    descriptor.streamed = m_pimpl->realTimeServer != nullptr;
    if (!m_pimpl->addChannel(name, descriptor))
    {
        return false;
    }

    if (!descriptor.streamed)
    {
        return true;
    }

    std::vector<std::string> metadata = descriptor.elementNames;
    for (std::size_t i = metadata.size(); i < size; i++)
    {
        metadata.push_back("element_" + std::to_string(i));
    }

    bool added{false};
    if (!m_pimpl->addRealTimeMetadata(name, metadata, added))
    {
        return false;
    }
    // make the new metadata available to the clients
    return !added || m_pimpl->realTimeServer->finalizeMetadata();
}

bool TelemetryBuffer::addStoredChannel(const std::string& name,
                                       const std::vector<std::size_t>& dimensions,
                                       const std::vector<std::string>& elementNames)
{
    return m_pimpl->addChannel(name, {dimensions, elementNames, false});
}

bool TelemetryBuffer::isChannelCompatible(const std::string& name,
                                          std::size_t size,
                                          const std::vector<std::string>& elementNames) const
{
    const auto descriptor = Impl::vectorDescriptor(size, elementNames);

    std::lock_guard lock(m_pimpl->mutex);
    const auto channel = m_pimpl->channels.find(name);
    return channel == m_pimpl->channels.end()
           || (channel->second.dimensions == descriptor.dimensions
               && channel->second.elementNames == descriptor.elementNames);
}

void TelemetryBuffer::push(const std::string& name, iDynTree::Span<const double> data, double time)
{
    bool streamed{false};
    {
        std::lock_guard lock(m_pimpl->mutex);
        m_pimpl->bufferManager.push_back(std::vector<double>(data.begin(), data.end()), time, name);
        const auto channel = m_pimpl->channels.find(name);
        streamed = channel != m_pimpl->channels.end() && channel->second.streamed;
    }

    if (streamed)
    {
        m_pimpl->realTimeServer->populateData(realTimeName(name), data);
    }
}

void TelemetryBuffer::push(const std::string& name, unsigned int data, double time)
{
    std::lock_guard lock(m_pimpl->mutex);
    m_pimpl->bufferManager.push_back(data, time, name);
}

void TelemetryBuffer::push(const std::string& name, const std::string& data, double time)
{
    std::lock_guard lock(m_pimpl->mutex);
    m_pimpl->bufferManager.push_back(data, time, name);
}

void TelemetryBuffer::push(const std::string& name, const TextLoggingEntry& data, double time)
{
    std::lock_guard lock(m_pimpl->mutex);
    m_pimpl->bufferManager.push_back(data, time, name);
}

void TelemetryBuffer::beginCycle(double time)
{
    if (m_pimpl->realTimeServer == nullptr)
    {
        return;
    }

    m_pimpl->realTimeServer->prepareData();
    m_pimpl->realTimeServer->clearData();
    const std::vector<double> timestamp{time};
    m_pimpl->realTimeServer->populateData(realTimeName(timestampsName), timestamp);
}

void TelemetryBuffer::endCycle()
{
    if (m_pimpl->realTimeServer != nullptr)
    {
        m_pimpl->realTimeServer->sendData();
    }
}

bool TelemetryBuffer::save(const std::string& fileNamePrefix, std::string& savedFileName)
{
    return m_pimpl->save(fileNamePrefix, savedFileName);
}

void TelemetryBuffer::startPeriodicSave()
{
    if (!m_pimpl->savePeriodically || m_pimpl->periodicSaveThread.joinable())
    {
        return;
    }

    {
        std::lock_guard lock(m_pimpl->periodicSaveMutex);
        m_pimpl->stopPeriodicSave = false;
    }
    m_pimpl->periodicSaveThread = std::thread([this] { m_pimpl->periodicSave(); });
}

void TelemetryBuffer::stopPeriodicSave()
{
    if (!m_pimpl->periodicSaveThread.joinable())
    {
        return;
    }

    {
        std::lock_guard lock(m_pimpl->periodicSaveMutex);
        m_pimpl->stopPeriodicSave = true;
    }
    m_pimpl->periodicSaveCv.notify_one();
    m_pimpl->periodicSaveThread.join();
}

void TelemetryBuffer::clear()
{
    std::lock_guard lock(m_pimpl->mutex);
    m_pimpl->bufferManager.clear();
    m_pimpl->channels.clear();
    m_pimpl->bufferManager.configure(m_pimpl->config);
}
