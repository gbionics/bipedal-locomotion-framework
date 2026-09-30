/**
 * @copyright 2026 Generative Bionics S.R.L. This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_ROBOT_LOGGER_REAL_TIME_STREAMER_H
#define BIPEDAL_LOCOMOTION_ROBOT_LOGGER_REAL_TIME_STREAMER_H

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <iDynTree/Span.h>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/YarpUtilities/VectorsCollectionServer.h>

namespace BipedalLocomotion
{
namespace RobotLogger
{

/**
 * RealTimeStreamer streams the logged data on a yarp port, e.g., for the robot-log-visualizer.
 * All the signals are streamed under the root `robot_realtime`.
 * It is not thread safe, the data must be populated by a single thread.
 */
class RealTimeStreamer
{
public:
    static constexpr auto rootName = "robot_realtime";

    /**
     * @param params parameters. It requires the `remote` parameter containing the port prefix.
     */
    bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> params);

    /**
     * Add a signal. Adding again a signal with the same metadata succeeds.
     * @note If elementNames does not have `size` elements, default names are used.
     */
    bool addSignal(const std::string& name,
                   std::size_t size,
                   const std::vector<std::string>& elementNames);

    /** Add the metadata describing the robot. */
    bool addRobotMetadata(const std::vector<std::string>& jointsList);

    void beginCycle(double time);

    void populate(const std::string& name, iDynTree::Span<const double> data);

    void endCycle();

private:
    enum class MetadataStatus
    {
        Added,
        AlreadyPresent,
        Error
    };

    MetadataStatus addMetadata(const std::string& key, const std::vector<std::string>& metadata);

    YarpUtilities::VectorsCollectionServer m_server;
    std::unordered_map<std::string, std::vector<std::string>> m_metadata;
};

} // namespace RobotLogger
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_ROBOT_LOGGER_REAL_TIME_STREAMER_H
