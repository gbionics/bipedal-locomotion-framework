/**
 * @copyright 2026 Generative Bionics S.R.L. This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_ROBOT_LOGGER_DATA_SINK_H
#define BIPEDAL_LOCOMOTION_ROBOT_LOGGER_DATA_SINK_H

#include <string>
#include <vector>

#include <BipedalLocomotion/RobotLogger/DataStorage.h>
#include <BipedalLocomotion/RobotLogger/RealTimeStreamer.h>

namespace BipedalLocomotion
{
namespace RobotLogger
{

/**
 * DataSink sends the numerical signals both to the storage and, if available, to the real time
 * streamer. The signals are vectors of doubles.
 */
class DataSink
{
public:
    DataSink(DataStorage& storage, RealTimeStreamer* streamer)
        : m_storage(storage)
        , m_streamer(streamer)
    {
    }

    /** Descriptor of the storage channel associated to a vector signal. */
    static ChannelDescriptor vectorDescriptor(std::size_t size,
                                              const std::vector<std::string>& elementNames)
    {
        const bool hasNames = elementNames.size() == size;
        return {{size, 1}, hasNames ? elementNames : std::vector<std::string>{}};
    }

    /**
     * Add a vector signal.
     * @return false also if the channel cannot be added now, see DataStorage::addChannel.
     */
    bool addChannel(const std::string& name,
                    std::size_t size,
                    const std::vector<std::string>& elementNames = {})
    {
        if (!m_storage.addChannel(name, vectorDescriptor(size, elementNames)))
        {
            return false;
        }
        return m_streamer == nullptr || m_streamer->addSignal(name, size, elementNames);
    }

    template <typename T> void log(const std::string& name, const T& data, double time)
    {
        m_storage.push(name, data, time);
        if (m_streamer != nullptr)
        {
            m_streamer->populate(name, data);
        }
    }

    DataStorage& storage()
    {
        return m_storage;
    }

private:
    DataStorage& m_storage;
    RealTimeStreamer* m_streamer;
};

} // namespace RobotLogger
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_ROBOT_LOGGER_DATA_SINK_H
