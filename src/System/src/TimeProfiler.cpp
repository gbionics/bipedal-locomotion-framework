/**
 * @file TimeProfiler.cpp
 * @authors Guglielmo Cervettini, Giulio Romualdi
 * @copyright 2024 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <string>
#include <utility>

#include <BipedalLocomotion/System/TimeProfiler.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

using namespace BipedalLocomotion::System;

std::chrono::nanoseconds Timer::getAverageDuration() const
{
    if (m_samples == 0)
    {
        return std::chrono::nanoseconds(0);
    }
    return m_totalDuration / m_samples;
}

const std::chrono::nanoseconds& Timer::getMaxDuration() const
{
    return m_maxDuration;
}

const std::chrono::nanoseconds& Timer::getTotalDuration() const
{
    return m_totalDuration;
}

unsigned int Timer::getSamples() const
{
    return m_samples;
}

void Timer::reset()
{
    m_totalDuration = std::chrono::nanoseconds(0);
    m_maxDuration = std::chrono::nanoseconds(0);
    m_samples = 0;
}

void Timer::setInitTime()
{
    m_initTime = std::chrono::steady_clock::now();
}

void Timer::setEndTime()
{
    m_endTime = std::chrono::steady_clock::now();
    m_hasFreshSample = true;
}

void Timer::evaluateDuration()
{
    // A timer that has not been closed on this cycle carries the interval of the previous one,
    // which would otherwise be counted twice.
    if (!m_hasFreshSample)
    {
        return;
    }
    m_hasFreshSample = false;

    const std::chrono::nanoseconds duration = m_endTime - m_initTime;
    m_totalDuration += duration;
    m_maxDuration = std::max(m_maxDuration, duration);
    m_samples++;
}

void TimeProfiler::setPeriod(int maxCounter)
{
    m_maxCounter = maxCounter;
}

bool TimeProfiler::addTimer(const std::string& key)
{
    auto timer = m_timers.find(key);
    if (timer != m_timers.end())
    {
        log()->error("[TimeProfiler::addTimer] The timer named {} already exists.", key);
        return false;
    }

    m_timers.insert(std::make_pair(key, Timer()));
    return true;
}

bool TimeProfiler::setInitTime(const std::string& key)
{
    auto timer = m_timers.find(key);
    if (timer == m_timers.end())
    {
        log()->error("[TimeProfiler::setInitTime] Unable to find the timer named {}.", key);
        return false;
    }

    timer->second.setInitTime();
    return true;
}

bool TimeProfiler::setEndTime(const std::string& key)
{
    auto timer = m_timers.find(key);
    if (timer == m_timers.end())
    {
        log()->error("[TimeProfiler::setEndTime] Unable to find the timer named {}.", key);
        return false;
    }

    timer->second.setEndTime();
    return true;
}

void TimeProfiler::profiling()
{
    std::ostringstream infoStream;
    infoStream << std::fixed << std::setprecision(3);

    m_counter++;
    const bool print = m_counter >= m_maxCounter;

    bool first = true;
    for (auto& [key, timer] : m_timers)
    {
        timer.evaluateDuration();
        if (!print)
        {
            continue;
        }

        const auto toMs = [](const std::chrono::nanoseconds& duration) {
            return std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(duration)
                .count();
        };

        // The average is over the samples of this very timer, and not over the cycles of the
        // profiler, so that a timer which is not hit on every cycle is not diluted by the ones it
        // skipped.
        infoStream << (first ? "" : " | ") << key << ": avg " << toMs(timer.getAverageDuration())
                   << " ms, max " << toMs(timer.getMaxDuration()) << " ms";
        first = false;

        timer.reset();
    }

    if (print)
    {
        m_counter = 0;
        log()->info("[TimeProfiler::profiling] {}", infoStream.str());
    }
}

ScopedTimer::ScopedTimer(TimeProfiler& profiler, std::string key)
    : m_profiler(profiler)
    , m_key(std::move(key))
{
    m_profiler.setInitTime(m_key);
}

ScopedTimer::~ScopedTimer()
{
    m_profiler.setEndTime(m_key);
}
