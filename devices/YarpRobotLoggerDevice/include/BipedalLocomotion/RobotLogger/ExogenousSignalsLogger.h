/**
 * @file ExogenousSignalsLogger.h
 * @copyright 2026 Generative Bionics S.R.L. This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_ROBOT_LOGGER_EXOGENOUS_SIGNALS_LOGGER_H
#define BIPEDAL_LOCOMOTION_ROBOT_LOGGER_EXOGENOUS_SIGNALS_LOGGER_H

#include <memory>
#include <vector>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/RobotLogger/ImageRecorder.h>
#include <BipedalLocomotion/RobotLogger/TelemetryBuffer.h>

namespace BipedalLocomotion
{
namespace RobotLogger
{

/**
 * ExogenousSignalsLogger logs the signals streamed by other applications.
 *
 * A monitor thread connects the signals as soon as their ports are available, it detects when an
 * application is closed and it reconnects the signal when the application is started again. Every
 * new connection increments the connection id of the signal, which is logged together with each
 * sample in the channel `exogenous_signals_connection_id::<signal_name>`. If the structure of a
 * signal changes in a new connection, the data is stored in the channel
 * `<channel>_connection_<id>`.
 */
class ExogenousSignalsLogger
{
public:
    ExogenousSignalsLogger();

    ~ExogenousSignalsLogger();

    // clang-format off
    /**
     * Initialize the logger.
     * @param handler pointer to the parameters handler of the group `ExogenousSignals`. If it is
     * not valid, no signal is logged.
     * @param buffer buffer where the signals are stored.
     * @note The following parameters are used:
     * |            Parameter Name               |       Type       |                Description                 | Mandatory |
     * |:---------------------------------------:|:----------------:|:------------------------------------------:|:---------:|
     * |  `vectors_collection_exogenous_inputs`  | `vector<string>` | Signals streamed with a VectorsCollectionServer. |  Yes  |
     * |       `vectors_exogenous_inputs`        | `vector<string>` |    Signals streamed as yarp::sig::Vector.  |    Yes    |
     * |        `string_exogenous_inputs`        | `vector<string>` |    Signals streamed as yarp::os::Bottle.   |    No     |
     * |        `image_exogenous_inputs`         | `vector<string>` |       Signals streamed as rgb images.      |    No     |
     * |     `human_state_exogenous_inputs`      | `vector<string>` |  Signals streamed as trintrin HumanState.  |    No     |
     * |   `wearable_targets_exogenous_inputs`   | `vector<string>` | Signals streamed as trintrin WearableTargets. | No     |
     * |    `wearable_data_exogenous_inputs`     | `vector<string>` | Signals streamed as trintrin WearableData. |    No     |
     * Each signal is described by a group containing `signal_name` and `remote`. The vectors
     * collections contain the parameters of YarpUtilities::VectorsCollectionClient, the other
     * signals contain `local` and `carrier`.
     * @return true in case of success, false otherwise.
     */
    bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler,
                    std::shared_ptr<TelemetryBuffer> buffer);

    // clang-format on

    /**
     * Start monitoring the connections and recording the images.
     */
    bool start();

    /**
     * Store the data received since the last call.
     */
    void record(double time);

    /**
     * Stop the monitor and the image recorders and disconnect all the signals.
     */
    void stop();

    /**
     * Get the recorders of the image signals.
     */
    const std::vector<std::shared_ptr<ImageRecorder>>& getImageRecorders() const;

private:
    struct Impl;
    std::unique_ptr<Impl> m_pimpl;
};

} // namespace RobotLogger
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_ROBOT_LOGGER_EXOGENOUS_SIGNALS_LOGGER_H
