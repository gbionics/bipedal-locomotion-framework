/**
 * @copyright 2026 Generative Bionics S.R.L. This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_ROBOT_LOGGER_DATA_STORAGE_H
#define BIPEDAL_LOCOMOTION_ROBOT_LOGGER_DATA_STORAGE_H

#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <robometry/BufferManager.h>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>

namespace BipedalLocomotion
{
namespace RobotLogger
{

struct ChannelDescriptor
{
    std::vector<std::size_t> dimensions;
    std::vector<std::string> elementNames;

    bool operator==(const ChannelDescriptor& other) const
    {
        return dimensions == other.dimensions && elementNames == other.elementNames;
    }
};

/**
 * DataStorage buffers the logged data and saves it in mat files. A file is written without
 * blocking the threads that push the data: the data of each channel is moved out of the buffer
 * with a short lock and then written.
 * All the methods are thread safe.
 */
class DataStorage
{
public:
    using SaveMethod = robometry::SaveCallbackSaveMethod;

    /** Called after every successful save with the path of the file (without extension). */
    using SaveCallback = std::function<void(const std::string& fileName, SaveMethod method)>;

    static constexpr auto defaultFilePrefix = "robot_logger_device";

    ~DataStorage();

    /**
     * Initialize the storage.
     * @param params parameters (it may be empty). The following parameters are optional:
     * | Parameter Name    |   Type   | Description                                             |
     * |:-----------------:|:--------:|:-------------------------------------------------------:|
     * | `log_folder`      | `string` | Folder of the files. Default the current directory.     |
     * | `save_period`     | `double` | Period in seconds of the periodic save. Default 1800.   |
     * | `save_periodically` | `bool` | Enable the periodic save. Default true.                 |
     * @param samplingPeriod period in seconds at which the data is pushed.
     */
    bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> params,
                    double samplingPeriod);

    const std::filesystem::path& logFolder() const;

    void setSaveCallback(SaveCallback callback);

    void setDescriptionList(const std::vector<std::string>& descriptionList);

    /**
     * Add a channel. Adding a channel that already exists with the same descriptor succeeds.
     * @return false also while a file is being written. In this case the caller should retry.
     */
    bool addChannel(const std::string& name, const ChannelDescriptor& descriptor);

    std::optional<ChannelDescriptor> getChannel(const std::string& name) const;

    template <typename T> void push(const std::string& name, const T& data, double time)
    {
        std::lock_guard lock(m_mutex);
        m_bufferManager.push_back(data, time, name);
    }

    /**
     * Save the buffered data in a file.
     * @param fileNamePrefix prefix of the file name.
     * @param method method passed to the save callback.
     * @param savedFileName path of the saved file without extension.
     * @return false if no file has been saved.
     */
    bool save(const std::string& fileNamePrefix, SaveMethod method, std::string& savedFileName);

    void startPeriodicSave();

    void stopPeriodicSave();

    /** Remove all the channels and their data. */
    void clear();

private:
    void periodicSave();

    mutable std::mutex m_mutex; /**< Protects the structure of the buffer manager. */
    robometry::BufferManager m_bufferManager;
    robometry::BufferConfig m_config;
    std::unordered_map<std::string, ChannelDescriptor> m_channels;
    bool m_saveInProgress{false};

    std::filesystem::path m_logFolder;
    bool m_savePeriodically{true};
    SaveCallback m_saveCallback;

    std::mutex m_saveMutex;
    std::chrono::steady_clock::time_point m_lastSaveTime;

    std::thread m_periodicSaveThread;
    std::mutex m_periodicSaveMutex;
    std::condition_variable m_periodicSaveCv;
    bool m_stopPeriodicSave{false};
};

} // namespace RobotLogger
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_ROBOT_LOGGER_DATA_STORAGE_H
