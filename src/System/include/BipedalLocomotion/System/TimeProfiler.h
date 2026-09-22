/**
 * @file TimeProfiler.h
 * @authors Guglielmo Cervettini, Giulio Romualdi
 * @copyright 2024 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_SYSTEM_TIME_PROFILER_H
#define BIPEDAL_LOCOMOTION_SYSTEM_TIME_PROFILER_H

// std
#include <chrono>
#include <map>
#include <memory>
#include <string>

namespace BipedalLocomotion
{
namespace System
{
/**
 * Timer is a simple class that can be used to measure the time between two events.
 */
class Timer
{
    std::chrono::time_point<std::chrono::steady_clock> m_initTime; /**< Init time. */
    std::chrono::time_point<std::chrono::steady_clock> m_endTime; /**< End time. */
    std::chrono::nanoseconds m_totalDuration{0}; /**< Duration accumulated since the last reset. */
    std::chrono::nanoseconds m_maxDuration{0}; /**< Longest sample since the last reset. */
    unsigned int m_samples{0}; /**< Number of samples accumulated since the last reset. */
    bool m_hasFreshSample{false}; /**< True once setEndTime() closed a not yet evaluated sample. */

public:
    /**
     * Reset the statistics, i.e. the accumulated duration, the maximum and the sample count.
     */
    void reset();

    /**
     * Set initial time.
     */
    void setInitTime();

    /**
     * Set final time.
     */
    void setEndTime();

    /**
     * Accumulate the interval between the last setInitTime() and setEndTime() and update the
     * maximum.
     * @note A cycle in which the timer has not been closed contributes no sample, instead of
     * counting the previous interval twice.
     */
    void evaluateDuration();

    /**
     * Get the mean of the samples accumulated since the last reset.
     * @return average duration, zero when no sample has been accumulated.
     */
    std::chrono::nanoseconds getAverageDuration() const;

    /**
     * Get the longest sample accumulated since the last reset.
     * @return maximum duration, zero when no sample has been accumulated.
     */
    const std::chrono::nanoseconds& getMaxDuration() const;

    /**
     * Get the sum of the samples accumulated since the last reset.
     * @return total duration.
     */
    const std::chrono::nanoseconds& getTotalDuration() const;

    /**
     * Get how many samples have been accumulated since the last reset.
     * @return number of samples.
     */
    unsigned int getSamples() const;
};

/**
 * TimeProfiler is a simple class that can be used to profile the code.
 */
class TimeProfiler
{
    int m_counter{0}; /**< Counter useful to print the profiling quantities only every m_maxCounter
                      times. */
    int m_maxCounter{1}; /**< The profiling quantities will be printed every maxCounter cycles. */
    std::map<std::string, Timer> m_timers; /**< Dictionary that contains all the timers. */
public:
    /**
     * Set the output period.
     * @param maxCounter is the period (expressed in cycles).
     */
    void setPeriod(int maxCounter);

    /**
     * Add a new timer
     * @param key is the name of the timer.
     * @return true/false in case of success/failure.
     */
    bool addTimer(const std::string& key);

    /**
     * Set the init time for the timer named "key"
     * @param key is the name of the timer.
     * @return true/false in case of success/failure.
     */
    bool setInitTime(const std::string& key);

    /**
     * Set the end time for the timer named "key"
     * @param key is the name of the timer.
     * @return true/false in case of success/failure.
     */
    bool setEndTime(const std::string& key);

    /**
     * Print the profiling quantities.
     */
    void profiling();
};

/**
 * ScopedTimer closes a TimeProfiler timer when the scope it lives in ends.
 *
 * The profiler measures the interval between setInitTime() and setEndTime(), so a timer left open
 * by an early return, a break or an exception does not simply lose one sample: it keeps a stale
 * initial time and skews the measurements that follow. Declaring a ScopedTimer instead of pairing
 * the two calls by hand removes the possibility.
 *
 * @code{.cpp}
 * {
 *     ScopedTimer timer(profiler, "Inference");
 *     if (!network.advance()) return false; // the timer is closed anyway
 * }
 * profiler.profiling();
 * @endcode
 */
class ScopedTimer
{
public:
    /**
     * Constructor. It sets the initial time of the timer named @p key.
     * @param profiler profiler owning the timer. It must outlive the ScopedTimer.
     * @param key name of the timer, as passed to TimeProfiler::addTimer().
     */
    ScopedTimer(TimeProfiler& profiler, std::string key);

    /**
     * Destructor. It sets the end time of the timer.
     */
    ~ScopedTimer();

    ScopedTimer(const ScopedTimer&) = delete;
    ScopedTimer& operator=(const ScopedTimer&) = delete;
    ScopedTimer(ScopedTimer&&) = delete;
    ScopedTimer& operator=(ScopedTimer&&) = delete;

private:
    TimeProfiler& m_profiler;
    std::string m_key;
};

}; // namespace System
}; // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_SYSTEM_TIME_PROFILER_H
