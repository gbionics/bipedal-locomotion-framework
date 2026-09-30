/**
 * @copyright 2026 Generative Bionics S.R.L. This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_ROBOT_LOGGER_TEXT_LOG_COLLECTOR_H
#define BIPEDAL_LOCOMOTION_ROBOT_LOGGER_TEXT_LOG_COLLECTOR_H

#include <atomic>
#include <string>
#include <thread>
#include <unordered_set>
#include <utility>
#include <vector>

#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>

#include <BipedalLocomotion/YarpTextLoggingUtilities.h>

#include <BipedalLocomotion/RobotLogger/DataStorage.h>

namespace BipedalLocomotion
{
namespace RobotLogger
{

/**
 * TextLogCollector stores the text messages published by the yarp applications on the `/log/*`
 * ports. A thread periodically looks for new ports and connects them.
 */
class TextLogCollector
{
public:
    /**
     * @param portName name of the port receiving the messages.
     * @param subnames only the ports containing one of these strings are connected. If empty
     * all the `/log/*` ports are connected.
     */
    TextLogCollector(std::string portName, std::vector<std::string> subnames);

    ~TextLogCollector();

    bool start();

    /** Store the messages received since the last call. */
    void record(DataStorage& storage, double time);

    void stop();

private:
    void lookForNewLogs();
    bool store(DataStorage& storage, const std::string& channel, const TextLoggingEntry& entry, double time);

    std::string m_portName;
    std::vector<std::string> m_subnames;
    yarp::os::BufferedPort<yarp::os::Bottle> m_port;

    std::atomic<bool> m_isRunning{false};
    std::thread m_thread;
    std::unordered_set<std::string> m_connectedPorts; /**< Accessed only by m_thread. */

    std::unordered_set<std::string> m_channels;
    std::vector<std::pair<std::string, TextLoggingEntry>> m_pendingMessages;
};

} // namespace RobotLogger
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_ROBOT_LOGGER_TEXT_LOG_COLLECTOR_H
