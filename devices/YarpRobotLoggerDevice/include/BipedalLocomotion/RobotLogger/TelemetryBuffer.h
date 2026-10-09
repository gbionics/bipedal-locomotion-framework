/**
 * @file TelemetryBuffer.h
 * @copyright 2026 Generative Bionics S.R.L. This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_ROBOT_LOGGER_TELEMETRY_BUFFER_H
#define BIPEDAL_LOCOMOTION_ROBOT_LOGGER_TELEMETRY_BUFFER_H

#include <filesystem>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <iDynTree/Span.h>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/YarpTextLoggingUtilities.h>

namespace BipedalLocomotion
{
namespace RobotLogger
{

/**
 * TelemetryBuffer stores the logged data in memory and saves it in mat files. If enabled, the
 * numerical channels are also streamed on a yarp port, e.g., for the robot-log-visualizer.
 * A file is written without blocking the threads pushing the data, so no sample is lost while
 * saving.
 * @note All the methods are thread safe except beginCycle(), endCycle() and the push of the
 * streamed channels, that must be called by the same thread.
 */
class TelemetryBuffer
{
public:
    static constexpr auto defaultFilePrefix = "robot_logger_device";

    /**
     * Function called after each successful save.
     * @param fileName path of the saved file without extension.
     */
    using SaveCallback = std::function<void(const std::string& fileName)>;

    TelemetryBuffer();

    ~TelemetryBuffer();

    // clang-format off
    /**
     * Initialize the buffer.
     * @param handler pointer to the parameters handler.
     * @note The following parameters are used:
     * |      Parameter Name        |   Type   |                    Description                     | Mandatory |
     * |:--------------------------:|:--------:|:--------------------------------------------------:|:---------:|
     * |   `sampling_period_in_s`   | `double` | Period at which the data is pushed. Default 0.01.  |    No     |
     * | `enable_real_time_logging` |  `bool`  | Stream the numerical channels on a yarp port. Default false. | No |
     * |     `robot_model_uri`      | `string` | URI of the robot model, saved as `yarp_robot_name`.|    No     |
     * |       `port_prefix`        | `string` | Used for the default real time port. Default `/yarp-robot-logger`. | No |
     * The optional group `Telemetry` contains:
     * |      Parameter Name        |   Type   |                    Description                     | Mandatory |
     * |:--------------------------:|:--------:|:--------------------------------------------------:|:---------:|
     * |        `log_folder`        | `string` | Folder of the files. Default the working directory.|    No     |
     * |       `save_period`        | `double` | Period of the periodic save in seconds. Default 1800.|  No     |
     * |    `save_periodically`     |  `bool`  |       Enable the periodic save. Default true.      |    No     |
     * If `enable_real_time_logging` is true, the optional group `REAL_TIME_STREAMING` contains the
     * parameters of YarpUtilities::VectorsCollectionServer. If it is not provided the data is
     * streamed on `<port_prefix>/rt_logging`.
     * @return true in case of success, false otherwise.
     */
    bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler);
    // clang-format on

    /**
     * Get the folder where the files are saved.
     */
    const std::filesystem::path& getLogFolder() const;

    /**
     * Set the function called after each successful save.
     */
    void setSaveCallback(SaveCallback callback);

    /**
     * Set the list of the joints of the robot.
     */
    bool setDescriptionList(const std::vector<std::string>& descriptionList);

    /**
     * Add a channel containing a vector of doubles. If enabled, the channel is also streamed in
     * real time. Adding a channel that exists with the same structure succeeds.
     * @param name name of the channel.
     * @param size size of the vector.
     * @param elementNames name of each element. If its size is not `size`, it is ignored.
     * @return false in case of failure or while a file is being written. In the latter case the
     * channel can be added later.
     */
    bool addChannel(const std::string& name,
                    std::size_t size,
                    const std::vector<std::string>& elementNames = {});

    /**
     * Add a channel that is only stored, e.g., a channel containing text.
     * @return false in case of failure or while a file is being written. In the latter case the
     * channel can be added later.
     */
    bool addStoredChannel(const std::string& name,
                          const std::vector<std::size_t>& dimensions,
                          const std::vector<std::string>& elementNames = {});

    /**
     * Check if a vector channel can be added with addChannel(), i.e., if it does not exist or it
     * exists with the same structure.
     */
    bool isChannelCompatible(const std::string& name,
                             std::size_t size,
                             const std::vector<std::string>& elementNames) const;

    /**
     * Push a vector. If the channel is streamed, the vector is also sent in the current cycle.
     */
    void push(const std::string& name, iDynTree::Span<const double> data, double time);

    void push(const std::string& name, unsigned int data, double time);

    void push(const std::string& name, const std::string& data, double time);

    void push(const std::string& name, const TextLoggingEntry& data, double time);

    /**
     * Begin a real time streaming cycle. It must be called before pushing the streamed channels.
     */
    void beginCycle(double time);

    /**
     * Send the data streamed in the current cycle.
     */
    void endCycle();

    /**
     * Save the buffered data.
     * @param fileNamePrefix prefix of the file name.
     * @param savedFileName path of the saved file without extension.
     * @return false if no file has been saved.
     */
    bool save(const std::string& fileNamePrefix, std::string& savedFileName);

    /**
     * Start saving the data every `save_period` seconds, if enabled.
     */
    void startPeriodicSave();

    void stopPeriodicSave();

    /**
     * Remove all the channels and their data.
     */
    void clear();

private:
    struct Impl;
    std::unique_ptr<Impl> m_pimpl;
};

} // namespace RobotLogger
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_ROBOT_LOGGER_TELEMETRY_BUFFER_H
