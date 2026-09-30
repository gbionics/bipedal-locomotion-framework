/**
 * @copyright 2026 Generative Bionics S.R.L. This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <cstdlib>

#include <BipedalLocomotion/TextLogging/Logger.h>

#include <BipedalLocomotion/RobotLogger/RealTimeStreamer.h>

using namespace BipedalLocomotion::RobotLogger;

namespace
{
constexpr auto treeDelimiter = "::";
constexpr auto timestampsName = "timestamps";

std::string fullName(const std::string& name)
{
    return std::string(RealTimeStreamer::rootName) + treeDelimiter + name;
}
} // namespace

bool RealTimeStreamer::initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> params)
{
    constexpr auto logPrefix = "[RealTimeStreamer::initialize]";

    auto ptr = params.lock();
    if (ptr == nullptr)
    {
        log()->error("{} The 'REAL_TIME_STREAMING' group is not provided.", logPrefix);
        return false;
    }

    if (!m_server.initialize(ptr))
    {
        log()->error("{} Unable to initialize the vectors collection server.", logPrefix);
        return false;
    }

    std::string remote;
    ptr->getParameter("remote", remote);
    log()->info("{} Real time logging activated on the yarp port {}.", logPrefix, remote);
    return true;
}

RealTimeStreamer::MetadataStatus
RealTimeStreamer::addMetadata(const std::string& key, const std::vector<std::string>& metadata)
{
    const auto existing = m_metadata.find(key);
    if (existing != m_metadata.end())
    {
        if (existing->second == metadata)
        {
            return MetadataStatus::AlreadyPresent;
        }
        log()->error("[RealTimeStreamer::addMetadata] The signal {} has been already added with "
                     "different metadata.",
                     key);
        return MetadataStatus::Error;
    }

    if (!m_server.populateMetadata(key, metadata))
    {
        log()->error("[RealTimeStreamer::addMetadata] Unable to add the metadata of {}.", key);
        return MetadataStatus::Error;
    }

    m_metadata.emplace(key, metadata);
    return MetadataStatus::Added;
}

bool RealTimeStreamer::addSignal(const std::string& name,
                                 std::size_t size,
                                 const std::vector<std::string>& elementNames)
{
    std::vector<std::string> metadata = elementNames;
    if (metadata.size() != size)
    {
        metadata.clear();
        for (std::size_t i = 0; i < size; i++)
        {
            metadata.push_back("element_" + std::to_string(i));
        }
    }

    const auto status = this->addMetadata(fullName(name), metadata);
    if (status == MetadataStatus::Added)
    {
        // make the new metadata available to the clients
        return m_server.finalizeMetadata();
    }
    return status == MetadataStatus::AlreadyPresent;
}

bool RealTimeStreamer::addRobotMetadata(const std::vector<std::string>& jointsList)
{
    const char* robotName = std::getenv("YARP_ROBOT_NAME");

    std::vector<std::pair<std::string, std::vector<std::string>>> metadata;
    metadata.emplace_back("yarp_robot_name",
                          std::vector<std::string>{robotName == nullptr ? "" : robotName});
    metadata.emplace_back(timestampsName, std::vector<std::string>{timestampsName});
    if (!jointsList.empty())
    {
        metadata.emplace_back("description_list", jointsList);
    }

    bool added = false;
    for (const auto& [name, value] : metadata)
    {
        const auto status = this->addMetadata(fullName(name), value);
        if (status == MetadataStatus::Error)
        {
            return false;
        }
        added = added || status == MetadataStatus::Added;
    }

    return !added || m_server.finalizeMetadata();
}

void RealTimeStreamer::beginCycle(double time)
{
    m_server.prepareData();
    m_server.clearData();
    const std::vector<double> timestamp{time};
    m_server.populateData(fullName(timestampsName), timestamp);
}

void RealTimeStreamer::populate(const std::string& name, iDynTree::Span<const double> data)
{
    m_server.populateData(fullName(name), data);
}

void RealTimeStreamer::endCycle()
{
    m_server.sendData();
}
