/**
 * @copyright 2026 Generative Bionics S.R.L. This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_ROBOT_LOGGER_ROBOT_DATA_LOGGER_H
#define BIPEDAL_LOCOMOTION_ROBOT_LOGGER_ROBOT_DATA_LOGGER_H

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include <yarp/dev/PolyDriverList.h>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/RobotInterface/YarpSensorBridge.h>

#include <BipedalLocomotion/RobotLogger/DataSink.h>

namespace BipedalLocomotion
{
namespace RobotLogger
{

/**
 * RobotDataLogger logs the robot data (joints, motors, IMUs, FT sensors, ...) retrieved through
 * a YarpSensorBridge.
 */
class RobotDataLogger
{
public:
    /**
     * @param params parameters of the YarpSensorBridge plus the `stream_*` flags selecting the
     * logged quantities.
     */
    bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> params);

    bool setDriversList(const yarp::dev::PolyDriverList& poly);

    /** Add the channels. It must be called at the beginning of each recording session. */
    bool prepare(DataSink& sink);

    void record(DataSink& sink, double time);

    const std::vector<std::string>& jointsList() const;

private:
    /** A quantity with one element per joint, e.g., joint positions. */
    struct JointSignal
    {
        std::string name;
        std::function<bool(Eigen::Ref<Eigen::VectorXd>)> read;
    };

    /** A quantity measured by a set of sensors, e.g., gyroscopes. */
    struct SensorSignal
    {
        std::string group;
        std::vector<std::string> elementNames;
        std::function<const std::vector<std::string>&()> sensors;
        std::function<bool(const std::string&, Eigen::VectorXd&)> read;
        Eigen::VectorXd buffer;
    };

    void addJointSignal(const std::string& name,
                        std::function<bool(Eigen::Ref<Eigen::VectorXd>)> read);

    template <int Size, typename Reader>
    void addSensorSignal(const std::string& group,
                         const std::vector<std::string>& elementNames,
                         std::function<const std::vector<std::string>&()> sensors,
                         Reader reader);

    std::unique_ptr<RobotInterface::YarpSensorBridge> m_bridge;
    std::vector<JointSignal> m_jointSignals;
    std::vector<SensorSignal> m_sensorSignals;
    std::vector<std::string> m_jointsList;
    Eigen::VectorXd m_jointsBuffer;
    bool m_bridgeReady{false};
};

} // namespace RobotLogger
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_ROBOT_LOGGER_ROBOT_DATA_LOGGER_H
