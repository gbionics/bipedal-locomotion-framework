/**
 * @copyright 2026 Generative Bionics S.R.L. This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <chrono>
#include <unordered_map>

#include <BipedalLocomotion/System/Clock.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <BipedalLocomotion/RobotLogger/RobotDataLogger.h>

using namespace BipedalLocomotion::RobotLogger;
using BipedalLocomotion::RobotInterface::BatteryStatus;

namespace
{
constexpr auto treeDelimiter = "::";

const std::vector<std::string> ftElementNames = {"f_x", "f_y", "f_z", "mu_x", "mu_y", "mu_z"};
const std::vector<std::string> gyroElementNames = {"omega_x", "omega_y", "omega_z"};
const std::vector<std::string> accelerometerElementNames = {"a_x", "a_y", "a_z"};
const std::vector<std::string> orientationElementNames = {"r", "p", "y"};
const std::vector<std::string> magnetometerElementNames = {"mag_x", "mag_y", "mag_z"};
const std::vector<std::string> temperatureElementNames = {"temperature"};
const std::vector<std::string> batteryElementNames = {"voltage", "current", "charge", "temperature"};
} // namespace

void RobotDataLogger::addJointSignal(const std::string& name,
                                     std::function<bool(Eigen::Ref<Eigen::VectorXd>)> read)
{
    m_jointSignals.push_back({name, std::move(read)});
}

template <int Size, typename Reader>
void RobotDataLogger::addSensorSignal(const std::string& group,
                                      const std::vector<std::string>& elementNames,
                                      std::function<const std::vector<std::string>&()> sensors,
                                      Reader reader)
{
    SensorSignal signal;
    signal.group = group;
    signal.elementNames = elementNames;
    signal.sensors = std::move(sensors);
    signal.buffer.resize(Size);
    signal.read = [reader](const std::string& sensor, Eigen::VectorXd& output) -> bool {
        Eigen::Matrix<double, Size, 1> measurement;
        if (!reader(sensor, measurement))
        {
            return false;
        }
        output = measurement;
        return true;
    };
    m_sensorSignals.push_back(std::move(signal));
}

bool RobotDataLogger::initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> params)
{
    constexpr auto logPrefix = "[RobotDataLogger::initialize]";

    auto ptr = params.lock();
    if (ptr == nullptr)
    {
        log()->error("{} The 'RobotSensorBridge' group is not provided.", logPrefix);
        return false;
    }

    m_bridge = std::make_unique<RobotInterface::YarpSensorBridge>();
    if (!m_bridge->initialize(ptr))
    {
        log()->error("{} Unable to configure the sensor bridge.", logPrefix);
        return false;
    }

    std::unordered_map<std::string, bool> stream = {{"stream_joint_states", false},
                                                    {"stream_joint_accelerations", true},
                                                    {"stream_motor_states", false},
                                                    {"stream_motor_temperature", false},
                                                    {"stream_motor_PWM", false},
                                                    {"stream_pids", false},
                                                    {"stream_inertials", false},
                                                    {"stream_cartesian_wrenches", false},
                                                    {"stream_forcetorque_sensors", false},
                                                    {"stream_temperatures", false},
                                                    {"stream_battery", false}};
    for (auto& [name, value] : stream)
    {
        if (!ptr->getParameter(name, value))
        {
            log()->info("{} The '{}' parameter is not found. Default value: {}.",
                        logPrefix,
                        name,
                        value);
        }
    }

    auto* bridge = m_bridge.get();

    if (stream["stream_joint_states"])
    {
        addJointSignal("joints_state::positions",
                       [bridge](auto v) { return bridge->getJointPositions(v); });
        addJointSignal("joints_state::velocities",
                       [bridge](auto v) { return bridge->getJointVelocities(v); });
        if (stream["stream_joint_accelerations"])
        {
            addJointSignal("joints_state::accelerations",
                           [bridge](auto v) { return bridge->getJointAccelerations(v); });
        }
        addJointSignal("joints_state::torques",
                       [bridge](auto v) { return bridge->getJointTorques(v); });
    }

    if (stream["stream_motor_states"])
    {
        addJointSignal("motors_state::positions",
                       [bridge](auto v) { return bridge->getMotorPositions(v); });
        addJointSignal("motors_state::velocities",
                       [bridge](auto v) { return bridge->getMotorVelocities(v); });
        addJointSignal("motors_state::accelerations",
                       [bridge](auto v) { return bridge->getMotorAccelerations(v); });
        addJointSignal("motors_state::currents",
                       [bridge](auto v) { return bridge->getMotorCurrents(v); });
        if (stream["stream_motor_temperature"])
        {
            addJointSignal("motors_state::temperatures",
                           [bridge](auto v) { return bridge->getMotorTemperatures(v); });
        }
    }

    if (stream["stream_motor_PWM"])
    {
        addJointSignal("motors_state::PWM", [bridge](auto v) { return bridge->getMotorPWMs(v); });
    }

    if (stream["stream_pids"])
    {
        addJointSignal("PIDs", [bridge](auto v) { return bridge->getPidPositions(v); });
    }

    if (stream["stream_forcetorque_sensors"])
    {
        addSensorSignal<6>(
            "FTs",
            ftElementNames,
            [bridge]() -> const auto& { return bridge->getSixAxisForceTorqueSensorsList(); },
            [bridge](const std::string& name, auto& v) {
                return bridge->getSixAxisForceTorqueMeasurement(name, v);
            });
    }

    if (stream["stream_inertials"])
    {
        addSensorSignal<3>(
            "gyros",
            gyroElementNames,
            [bridge]() -> const auto& { return bridge->getGyroscopesList(); },
            [bridge](const std::string& name, auto& v) {
                return bridge->getGyroscopeMeasure(name, v);
            });
        addSensorSignal<3>(
            "accelerometers",
            accelerometerElementNames,
            [bridge]() -> const auto& { return bridge->getLinearAccelerometersList(); },
            [bridge](const std::string& name, auto& v) {
                return bridge->getLinearAccelerometerMeasurement(name, v);
            });
        addSensorSignal<3>(
            "orientations",
            orientationElementNames,
            [bridge]() -> const auto& { return bridge->getOrientationSensorsList(); },
            [bridge](const std::string& name, auto& v) {
                return bridge->getOrientationSensorMeasurement(name, v);
            });
        addSensorSignal<3>(
            "magnetometers",
            magnetometerElementNames,
            [bridge]() -> const auto& { return bridge->getMagnetometersList(); },
            [bridge](const std::string& name, auto& v) {
                return bridge->getMagnetometerMeasurement(name, v);
            });
    }

    if (stream["stream_cartesian_wrenches"])
    {
        addSensorSignal<6>(
            "cartesian_wrenches",
            ftElementNames,
            [bridge]() -> const auto& { return bridge->getCartesianWrenchesList(); },
            [bridge](const std::string& name, auto& v) {
                return bridge->getCartesianWrench(name, v);
            });
    }

    if (stream["stream_temperatures"])
    {
        addSensorSignal<1>(
            "temperatures",
            temperatureElementNames,
            [bridge]() -> const auto& { return bridge->getTemperatureSensorsList(); },
            [bridge](const std::string& name, auto& v) {
                return bridge->getTemperature(name, v(0));
            });
    }

    if (stream["stream_battery"])
    {
        addSensorSignal<4>(
            "batteries",
            batteryElementNames,
            [bridge]() -> const auto& { return bridge->getBatteriesList(); },
            [bridge](const std::string& name, auto& v) {
                BatteryStatus status;
                if (!bridge->getBatteryStatus(name, status))
                {
                    return false;
                }
                v << status.voltage, status.current, status.charge, status.temperature;
                return true;
            });
    }

    return true;
}

bool RobotDataLogger::setDriversList(const yarp::dev::PolyDriverList& poly)
{
    if (!m_bridge->setDriversList(poly))
    {
        log()->error("[RobotDataLogger::setDriversList] Could not attach the drivers list to the "
                     "sensor bridge.");
        return false;
    }
    return true;
}

bool RobotDataLogger::prepare(DataSink& sink)
{
    constexpr auto logPrefix = "[RobotDataLogger::prepare]";

    if (!m_bridgeReady)
    {
        // the sensor bridge could be not ready right after the attach
        using namespace std::chrono_literals;
        BipedalLocomotion::clock().sleepFor(2000ms);
        m_bridgeReady = true;
    }

    if (!m_bridge->getJointsList(m_jointsList))
    {
        log()->error("{} Could not get the joints list.", logPrefix);
        return false;
    }
    m_jointsBuffer.resize(m_jointsList.size());
    sink.storage().setDescriptionList(m_jointsList);

    for (const auto& signal : m_jointSignals)
    {
        if (!sink.addChannel(signal.name, m_jointsList.size(), m_jointsList))
        {
            log()->error("{} Unable to add the channel {}.", logPrefix, signal.name);
            return false;
        }
    }

    for (const auto& signal : m_sensorSignals)
    {
        for (const auto& sensor : signal.sensors())
        {
            const std::string name = signal.group + treeDelimiter + sensor;
            if (!sink.addChannel(name, signal.elementNames.size(), signal.elementNames))
            {
                log()->error("{} Unable to add the channel {}.", logPrefix, name);
                return false;
            }
        }
    }

    return true;
}

void RobotDataLogger::record(DataSink& sink, double time)
{
    if (!m_bridge->advance())
    {
        log()->error("[RobotDataLogger::record] Could not advance the sensor bridge.");
    }

    for (const auto& signal : m_jointSignals)
    {
        if (signal.read(m_jointsBuffer))
        {
            sink.log(signal.name, m_jointsBuffer, time);
        }
    }

    for (auto& signal : m_sensorSignals)
    {
        for (const auto& sensor : signal.sensors())
        {
            if (signal.read(sensor, signal.buffer))
            {
                sink.log(signal.group + treeDelimiter + sensor, signal.buffer, time);
            }
        }
    }
}

const std::vector<std::string>& RobotDataLogger::jointsList() const
{
    return m_jointsList;
}
