/**
 * @copyright 2026 Generative Bionics S.R.L. This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <chrono>

#include <yarp/os/Network.h>
#include <yarp/profiler/NetworkProfiler.h>

#include <BipedalLocomotion/TextLogging/Logger.h>

#include <BipedalLocomotion/RobotLogger/TextLogCollector.h>

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

namespace
{
void replaceAll(std::string& data, char toSearch, char replace)
{
    for (auto& c : data)
    {
        if (c == toSearch)
        {
            c = replace;
        }
    }
}
} // namespace

TextLogCollector::TextLogCollector(std::string portName, std::vector<std::string> subnames)
    : m_portName(std::move(portName))
    , m_subnames(std::move(subnames))
{
    // do not drop the messages arriving between two calls of log()
    m_port.setStrict(true);
}

TextLogCollector::~TextLogCollector()
{
    this->stop();
}

bool TextLogCollector::start()
{
    if (!m_port.open(m_portName))
    {
        log()->error("[TextLogCollector::start] Unable to open the port {}.", m_portName);
        return false;
    }

    m_isRunning = true;
    m_thread = std::thread([this] { this->lookForNewLogs(); });
    return true;
}

void TextLogCollector::stop()
{
    if (!m_thread.joinable())
    {
        return;
    }

    m_isRunning = false;
    m_thread.join();

    for (const auto& port : m_connectedPorts)
    {
        yarp::os::Network::disconnect(port, m_portName);
    }
    m_connectedPorts.clear();
    m_channels.clear();
    m_pendingMessages.clear();
    m_port.close();
}

void TextLogCollector::lookForNewLogs()
{
    using namespace std::chrono_literals;
    constexpr auto textLoggingPortPrefix = "/log/";
    constexpr auto period = 2s;
    constexpr auto sleepStep = 100ms;

    auto hasSubname = [this](const std::string& port) {
        if (m_subnames.empty())
        {
            return true;
        }
        for (const auto& subname : m_subnames)
        {
            if (port.find(subname) != std::string::npos)
            {
                return true;
            }
        }
        return false;
    };

    yarp::profiler::NetworkProfiler::ports_name_set ports;
    while (m_isRunning)
    {
        ports.clear();
        yarp::profiler::NetworkProfiler::getPortsList(ports);
        for (const auto& port : ports)
        {
            if (port.name.rfind(textLoggingPortPrefix, 0) == 0
                && m_connectedPorts.find(port.name) == m_connectedPorts.end()
                && hasSubname(port.name) && yarp::os::Network::exists(port.name))
            {
                m_connectedPorts.insert(port.name);
                yarp::os::Network::connect(port.name, m_portName, "udp");
            }
        }

        for (auto slept = 0ms; slept < period && m_isRunning; slept += sleepStep)
        {
            std::this_thread::sleep_for(sleepStep);
        }
    }
}

bool TextLogCollector::store(DataStorage& storage,
                             const std::string& channel,
                             const TextLoggingEntry& entry,
                             double time)
{
    if (m_channels.find(channel) == m_channels.end())
    {
        if (!storage.addChannel(channel, {{1, 1}, {}}))
        {
            return false;
        }
        m_channels.insert(channel);
    }
    storage.push(channel, entry, time);
    return true;
}

void TextLogCollector::record(DataStorage& storage, double time)
{
    // the messages whose channel could not be created while a file was being written
    if (!m_pendingMessages.empty())
    {
        auto pending = std::move(m_pendingMessages);
        m_pendingMessages.clear();
        for (const auto& [channel, entry] : pending)
        {
            if (!this->store(storage, channel, entry, time))
            {
                m_pendingMessages.emplace_back(channel, entry);
            }
        }
    }

    while (m_port.getPendingReads() > 0)
    {
        yarp::os::Bottle* bottle = m_port.read(false);
        if (bottle == nullptr)
        {
            break;
        }

        const auto entry = TextLoggingEntry::deserializeMessage(*bottle, std::to_string(time));
        if (!entry.isValid)
        {
            continue;
        }

        std::string channel = entry.portSystem + "::" + entry.portPrefix
                              + "::" + entry.processName + "::p" + entry.processPID;
        // matlab does not support the character - in the name of a struct field
        replaceAll(channel, '-', '_');

        if (!this->store(storage, channel, entry, time))
        {
            m_pendingMessages.emplace_back(channel, entry);
        }
    }
}
