/**
 * @file AdvanceableRunner.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_SYSTEM_ADVANCEABLE_RUNNER_H
#define BIPEDAL_LOCOMOTION_SYSTEM_ADVANCEABLE_RUNNER_H

#include <BipedalLocomotion/GenericContainer/TemplateHelpers.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/System/Advanceable.h>
#include <BipedalLocomotion/System/Barrier.h>
#include <BipedalLocomotion/System/Clock.h>
#include <BipedalLocomotion/System/SharedResource.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <atomic>
#include <memory>
#include <optional>
#include <thread>
#include <type_traits>

namespace BipedalLocomotion
{
namespace System
{

/**
 * AdvanceableRunner is an helper class that can be used to run a advanceable at a given period.
 * Different AdvanceableRunners can communicate trough the SharedResource class. The
 * AdvanceableRunner can be used only with Advanceable. Do not try to use if with a class that does
 * not inherit from advanceable, a compile time assert will be thrown. This is a visual example of
 * usage <br/> <img
 * src="https://user-images.githubusercontent.com/16744101/112768882-7c137280-901e-11eb-8c24-05b7cc3fa5e5.png"
 * alt="AdvanceableRunner" width="500"> Here we want to run two advanceables in parallel with a
 * different sampling time. The AdvanceableRunner is a class that helps you in creating a periodic
 * thread. The SharedResource is the way that can be used to communicate between two advanceables
 * running in separate threads (i.e. the wired between two AdvanceableRunners)
 */
template <class _Advanceable> class AdvanceableRunner
{
    static_assert(BipedalLocomotion::is_base_of_template<System::Advanceable, _Advanceable>::value,
                  "The _Advanceable class must be derived from System::Advanceable class.");

public:
    using Input = typename _Advanceable::Input;
    using Output = typename _Advanceable::Output;

private:
    bool m_isInitialized{false}; /**< True if the runner is initialized */
    std::chrono::nanoseconds m_dT{std::chrono::nanoseconds::zero()}; /**< Period of the runner */
    std::atomic<bool> m_isRunning{false}; /**> If True the runner is running */
    int m_maximumNumberOfAcceptedDeadlineMiss{-1}; /**< Maximum number of accepted deadline miss */

    std::unique_ptr<_Advanceable> m_advanceable; /**< Advanceable contained in the runner */
    typename SharedResource<Input>::Ptr m_input; /**< Input shared resource */
    typename SharedResource<Output>::Ptr m_output; /**< Output shared resource */

    struct Info
    {
        unsigned int deadlineMiss{0}; /**< Number of deadline miss */
        std::chrono::nanoseconds dT{std::chrono::nanoseconds::zero()}; /**< Period of the runner */
        std::string name{}; /**< Name associated to the runner */
    };
    std::mutex m_infoMutex; /**< Mutex used to protect the information struct */
    Info m_info; /**< Information struct */

public:
    // clang-format off
    /**
     * Initialize the AdvanceableRunner class
     * @param handler pointer to a parameter handler
     * @note The following parameters are required
     * |                Parameter Name              |   Type   |                                             Description                                       | Mandatory |
     * |:------------------------------------------:|:--------:|:---------------------------------------------------------------------------------------------:|:---------:|
     * |               `sampling_time`              | `double` |            Strictly positive number representing the sampling time in seconds                 |    Yes    |
     * |                   `name`                   | `string` |                          Name associated to the runner (used in the logs)                     |    Yes    |
     * | `maximum_number_of_accepted_deadline_miss` |  `int`   | Number of accepted deadline miss. if negative the check is not considered. Default value `-1` |     No    |
     * @return true in case of success, false otherwise.
     */
    bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler);
    // clang-format on

    /**
     * Set the advanceable inside the runner.
     * @param advanceable an unique pointer representing the advanceable
     * @return true in case of success, false otherwise
     */
    bool setAdvanceable(std::unique_ptr<_Advanceable> advanceable);

    /**
     * Set the input resource
     * @param resource pointer representing the input resource
     * @return true in case of success, false otherwise
     */
    bool setInputResource(std::shared_ptr<SharedResource<Input>> resource);

    /**
     * Set the output resource
     * @param resource pointer representing the output resource
     * @return true in case of success, false otherwise
     */
    bool setOutputResource(std::shared_ptr<SharedResource<Output>> resource);

    /**
     * Get some info of the runner,
     * @return A copy of the Info struct
     */
    Info getInfo();

    /**
     * Run the advanceable runner. The function create a periodic thread running with a period of
     * m_dT seconds.
     * @param barrier is an optional parameter that can be used to synchronize the startup of all
     * the AdvanceableRunner spawned by the process.
     * @return a thread of containing the running process. If the runner was not correctly
     * initialized the thread is invalid.
     */
    std::thread run(std::shared_ptr<Barrier> barrier = nullptr);

    /**
     * Stop the thread
     * @return true in case of success, false otherwise
     */
    void stop();

    /**
     * Check if the AdvanceableRunner is running
     * @return true if the thread is running, false otherwise.
     */
    bool isRunning() const;
};

template <class _Advanceable>
bool AdvanceableRunner<_Advanceable>::initialize(
    std::weak_ptr<const ParametersHandler::IParametersHandler> handler)
{
    constexpr auto errorPrefix = "[AdvanceableRunner::initialize]";

    if (m_isInitialized)
    {
        log()->error("{} AdvanceableRunner is already initialized.", errorPrefix);
        return false;
    }

    auto ptr = handler.lock();
    if (ptr == nullptr)
    {
        log()->error("{} The parameters handler is not valid.", errorPrefix);
        return false;
    }

    if (!ptr->getParameter("sampling_time", m_info.dT))
    {
        log()->error("{} Unable to get the sampling time.", errorPrefix);
        return false;
    }

    if (!ptr->getParameter("name", m_info.name))
    {
        log()->error("{} Unable to get the name of the runner.", errorPrefix);
        return false;
    }

    m_dT = m_info.dT;

    if (!ptr->getParameter("maximum_number_of_accepted_deadline_miss",
                           m_maximumNumberOfAcceptedDeadlineMiss))
    {
        log()->info("{} maximum_number_of_accepted_deadline_miss parameter not found. The default "
                    "parameter will be used: {}.",
                    errorPrefix,
                    m_maximumNumberOfAcceptedDeadlineMiss);
    }

    m_isInitialized = true;
    return true;
}

template <class _Advanceable>
bool AdvanceableRunner<_Advanceable>::setAdvanceable(std::unique_ptr<_Advanceable> advanceable)
{
    constexpr auto errorPrefix = "[AdvanceableRunner::setAdvanceable]";

    if (m_isRunning)
    {
        log()->error("{} The advanceable cannot be changed while the runner is running.",
                     errorPrefix);
        return false;
    }

    if (advanceable == nullptr)
    {
        log()->error("{} The advanceable is not valid.", errorPrefix);
        return false;
    }

    m_advanceable = std::move(advanceable);
    return true;
}

template <class _Advanceable>
bool AdvanceableRunner<_Advanceable>::setInputResource(std::shared_ptr<SharedResource<Input>> input)
{
    constexpr auto logPrefix = "[AdvanceableRunner::setInputResource]";

    if (m_isRunning)
    {
        log()->error("{} The input cannot be changed while the runner is running.", logPrefix);
        return false;
    }

    if (input == nullptr)
    {
        log()->error("{} The input is not valid.", logPrefix);
        return false;
    }

    this->m_input = input;

    return true;
}

template <class _Advanceable>
bool AdvanceableRunner<_Advanceable>::setOutputResource(
    std::shared_ptr<SharedResource<Output>> output)
{
    constexpr auto logPrefix = "[AdvanceableRunner::setOutputResource]";

    if (m_isRunning)
    {
        log()->error("{} The output cannot be changed while the runner is running.", logPrefix);
        return false;
    }

    if (output == nullptr)
    {
        log()->error("{} The output is not valid.", logPrefix);
        return false;
    }

    this->m_output = output;

    return true;
}

template <class _Advanceable>
std::thread AdvanceableRunner<_Advanceable>::run(std::shared_ptr<Barrier> barrier)
{
    constexpr auto logPrefix = "[AdvanceableRunner::run]";

    if (!m_isInitialized)
    {
        log()->warn("{} - {} The AdvanceableRunner class is not initialized. An invalid thread "
                    "will be returned",
                    logPrefix,
                    m_info.name);

        assert(false && "[AdvanceableRunner::run] The AdvanceableRunner is not initialized.");
        return std::thread();
    }

    if (m_input == nullptr || m_output == nullptr)
    {
        log()->warn("{} - {} The shared resources are not valid. An invalid thread will be "
                    "returned.",
                    logPrefix,
                    m_info.name);
        assert(false && "[AdvanceableRunner::run] The shared resources are not valid.");
        return std::thread();
    }

    if (m_advanceable == nullptr)
    {
        log()->warn("{} - {} The advanceable is not valid. An invalid thread will be returned.",
                    logPrefix,
                    m_info.name);
        assert(false && "[AdvanceableRunner::run] The advanceable is not valid.");
        return std::thread();
    }

    if (m_isRunning)
    {
        log()->warn("{} - {} The thread is already running. An invalid thread will be returned.",
                    logPrefix,
                    m_info.name);
        assert(false && "[AdvanceableRunner::run] The thread is already running.");
        return std::thread();
    }

    // run the thread
    m_isRunning = true;
    auto function = [&](std::shared_ptr<Barrier> barrier) -> bool {
        constexpr auto logPrefix = "[AdvanceableRunner::run]";

        // synchronize the threads
        if (!(barrier == nullptr))
        {
            log()->debug("{} - {} This thread is waiting for the other threads.",
                         logPrefix,
                         m_info.name);
            barrier->wait();
        }

        auto time = BipedalLocomotion::clock().now();
        auto oldTime = time;
        auto wakeUpTime = time;

        // the main periodic function
        while (this->m_isRunning)
        {
            // detect if a clock has been reset
            oldTime = time;
            time = BipedalLocomotion::clock().now();
            // if the current time is lower than old time, the timer has been reset.
            if ((time - oldTime).count() < 1e-12)
            {
                wakeUpTime = time;
            }

            // advance the wake-up time
            wakeUpTime += m_dT;

            if (!this->m_advanceable->setInput(this->m_input->get()))
            {
                m_isRunning = false;
                log()->error("{} - {} Unable to set the input to the advanceable.",
                             logPrefix,
                             m_info.name);
                break;
            }

            // advance the advanceable
            if (!this->m_advanceable->advance())
            {
                m_isRunning = false;
                log()->error("{} - {} Unable to advance the advanceable.", logPrefix, m_info.name);
                break;
            }
            assert(this->m_advanceable->isOutputValid());

            // set the output
            this->m_output->set(this->m_advanceable->getOutput());

            // check if the deadline is missed
            if (wakeUpTime < BipedalLocomotion::clock().now())
            {
                unsigned int deadlineMiss = 0;

                // scope to reduce the amount of time in which the mutex is locked
                {
                    std::lock_guard<std::mutex> guard(m_infoMutex);
                    m_info.deadlineMiss++;
                    deadlineMiss = m_info.deadlineMiss;
                }

                if (m_maximumNumberOfAcceptedDeadlineMiss >= 0
                    && deadlineMiss
                           > static_cast<unsigned int>(m_maximumNumberOfAcceptedDeadlineMiss))
                {
                    // we have to close the runner
                    m_isRunning = false;
                    log()->error("{} - {} Experienced {} deadline misses. The maximum accepted "
                                 "number is {}.",
                                 logPrefix,
                                 m_info.name,
                                 deadlineMiss,
                                 m_maximumNumberOfAcceptedDeadlineMiss);
                    break;
                }
            }

            // release the CPU
            BipedalLocomotion::clock().yield();

            BipedalLocomotion::clock().sleepUntil(wakeUpTime);
        }

        unsigned int deadlineMiss = 0;
        // scope to reduce the amount of time in which the mutex is locked
        {
            std::lock_guard<std::mutex> guard(m_infoMutex);
            deadlineMiss = m_info.deadlineMiss;
        }

        log()->info("{} - {}: Closing the AdvanceableRunner. Number of deadline miss {}.",
                    logPrefix,
                    m_info.name,
                    deadlineMiss);

        // the return value of the thread function is discarded by std::thread
        if (!this->m_advanceable->close())
        {
            log()->error("{} - {} Unable to close the advanceable.", logPrefix, m_info.name);
            return false;
        }
        return true;
    };

    return std::thread(function, barrier);
}

template <class _Advanceable> void AdvanceableRunner<_Advanceable>::stop()
{
    // m_isRunning is an atomic<bool> the mutex is not required here
    m_isRunning = false;
}

template <class _Advanceable> bool AdvanceableRunner<_Advanceable>::isRunning() const
{
    // m_isRunning is an atomic<bool> the mutex is not required here
    return m_isRunning;
}

template <class _Advanceable>
typename AdvanceableRunner<_Advanceable>::Info AdvanceableRunner<_Advanceable>::getInfo()
{
    std::lock_guard<std::mutex> guard(m_infoMutex);
    return m_info;
}

} // namespace System
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_SYSTEM_ADVANCEABLE_RUNNER_H
