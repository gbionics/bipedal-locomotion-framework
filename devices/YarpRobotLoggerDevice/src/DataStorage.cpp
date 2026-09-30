/**
 * @copyright 2026 Generative Bionics S.R.L. This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <cmath>
#include <cstdlib>

#include <BipedalLocomotion/TextLogging/Logger.h>

#include <BipedalLocomotion/RobotLogger/DataStorage.h>

using namespace BipedalLocomotion::RobotLogger;

DataStorage::~DataStorage()
{
    this->stopPeriodicSave();
}

bool DataStorage::initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> params,
                             double samplingPeriod)
{
    constexpr auto logPrefix = "[DataStorage::initialize]";

    robometry::BufferConfig config;
    if (const char* robotName = std::getenv("YARP_ROBOT_NAME"))
    {
        config.yarp_robot_name = robotName;
    }
    config.filename = defaultFilePrefix;
    // the files are saved by this class, robometry is used only as buffer
    config.auto_save = false;
    config.save_periodically = false;
    config.file_indexing = "%Y_%m_%d_%H_%M_%S";
    config.mat_file_version = matioCpp::FileVersion::MAT7_3;
    config.save_period = 1800.0;

    std::string logFolder;
    auto ptr = params.lock();
    if (ptr == nullptr)
    {
        log()->info("{} The telemetry parameters are not provided. The default values will be "
                    "used.",
                    logPrefix);
    } else
    {
        if (!ptr->getParameter("log_folder", logFolder))
        {
            log()->info("{} The 'log_folder' parameter is not provided. The files will be saved "
                        "in the current working directory.",
                        logPrefix);
        }
        if (!ptr->getParameter("save_period", config.save_period))
        {
            log()->info("{} The 'save_period' parameter is not provided. Default value: {}.",
                        logPrefix,
                        config.save_period);
        }
        if (!ptr->getParameter("save_periodically", m_savePeriodically))
        {
            log()->info("{} The 'save_periodically' parameter is not provided. Default value: {}.",
                        logPrefix,
                        m_savePeriodically);
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

    m_logFolder = logFolder.empty() ? std::filesystem::current_path()
                                    : std::filesystem::absolute(logFolder);

    std::error_code ec;
    std::filesystem::create_directories(m_logFolder, ec);
    if (ec || !std::filesystem::is_directory(m_logFolder))
    {
        log()->error("{} Unable to create the log folder {}. Error: {}.",
                     logPrefix,
                     m_logFolder.string(),
                     ec.message());
        return false;
    }
    log()->info("{} The logged data will be saved in {}.", logPrefix, m_logFolder.string());

    // robometry concatenates path and file name
    config.path = (m_logFolder / "").string();

    // the buffer must contain the samples pushed between two saves, a 10% margin is added
    constexpr double margin = 0.1;
    config.n_samples
        = static_cast<std::size_t>(std::ceil((1 + margin) * config.save_period / samplingPeriod));

    std::lock_guard lock(m_mutex);
    m_config = config;
    return m_bufferManager.configure(m_config);
}

const std::filesystem::path& DataStorage::logFolder() const
{
    return m_logFolder;
}

void DataStorage::setSaveCallback(SaveCallback callback)
{
    std::lock_guard lock(m_saveMutex);
    m_saveCallback = std::move(callback);
}

void DataStorage::setDescriptionList(const std::vector<std::string>& descriptionList)
{
    std::lock_guard lock(m_mutex);
    m_config.description_list = descriptionList;
    m_bufferManager.setDescriptionList(descriptionList);
}

bool DataStorage::addChannel(const std::string& name, const ChannelDescriptor& descriptor)
{
    std::lock_guard lock(m_mutex);

    const auto channel = m_channels.find(name);
    if (channel != m_channels.end())
    {
        if (channel->second == descriptor)
        {
            return true;
        }
        log()->error("[DataStorage::addChannel] The channel {} already exists with a different "
                     "structure.",
                     name);
        return false;
    }

    // the buffer manager tree cannot be modified while it is written to file
    if (m_saveInProgress)
    {
        return false;
    }

    if (!m_bufferManager.addChannel({name, descriptor.dimensions, descriptor.elementNames}))
    {
        log()->error("[DataStorage::addChannel] Unable to add the channel {}.", name);
        return false;
    }

    m_channels.emplace(name, descriptor);
    return true;
}

std::optional<ChannelDescriptor> DataStorage::getChannel(const std::string& name) const
{
    std::lock_guard lock(m_mutex);
    const auto channel = m_channels.find(name);
    if (channel == m_channels.end())
    {
        return std::nullopt;
    }
    return channel->second;
}

bool DataStorage::save(const std::string& fileNamePrefix,
                       SaveMethod method,
                       std::string& savedFileName)
{
    constexpr auto logPrefix = "[DataStorage::save]";

    std::lock_guard saveLock(m_saveMutex);

    // The files are indexed with a resolution of one second.
    using namespace std::chrono_literals;
    const auto elapsed = std::chrono::steady_clock::now() - m_lastSaveTime;
    if (elapsed < 1s)
    {
        std::this_thread::sleep_for(1s - elapsed);
    }

    const auto start = std::chrono::steady_clock::now();
    {
        std::lock_guard lock(m_mutex);
        m_saveInProgress = true;
        m_bufferManager.setFileName(fileNamePrefix);
    }

    const bool ok = m_bufferManager.saveToFile(savedFileName);

    {
        std::lock_guard lock(m_mutex);
        m_bufferManager.setFileName(defaultFilePrefix);
        m_saveInProgress = false;
    }
    m_lastSaveTime = std::chrono::steady_clock::now();

    if (!ok)
    {
        log()->warn("{} No data saved. Either no data was logged or the file could not be "
                    "written in {}.",
                    logPrefix,
                    m_logFolder.string());
        return false;
    }

    log()->info("{} Data saved to file {}.mat in {}.",
                logPrefix,
                savedFileName,
                std::chrono::duration<double>(m_lastSaveTime - start));

    if (m_saveCallback)
    {
        m_saveCallback(savedFileName, method);
    }

    return true;
}

void DataStorage::periodicSave()
{
    const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(m_config.save_period));

    std::unique_lock lock(m_periodicSaveMutex);
    while (!m_periodicSaveCv.wait_for(lock, period, [this] { return m_stopPeriodicSave; }))
    {
        lock.unlock();
        std::string fileName;
        this->save(defaultFilePrefix, SaveMethod::periodic, fileName);
        lock.lock();
    }
}

void DataStorage::startPeriodicSave()
{
    if (!m_savePeriodically || m_periodicSaveThread.joinable())
    {
        return;
    }

    {
        std::lock_guard lock(m_periodicSaveMutex);
        m_stopPeriodicSave = false;
    }
    m_periodicSaveThread = std::thread([this] { this->periodicSave(); });
}

void DataStorage::stopPeriodicSave()
{
    if (!m_periodicSaveThread.joinable())
    {
        return;
    }

    {
        std::lock_guard lock(m_periodicSaveMutex);
        m_stopPeriodicSave = true;
    }
    m_periodicSaveCv.notify_one();
    m_periodicSaveThread.join();
}

void DataStorage::clear()
{
    std::lock_guard lock(m_mutex);
    m_bufferManager.clear();
    m_channels.clear();
    m_bufferManager.configure(m_config);
}
