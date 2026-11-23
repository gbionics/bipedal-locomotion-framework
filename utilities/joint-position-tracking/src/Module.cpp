/**
 * @file Module.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <chrono>
#include <fstream>
#include <iomanip>

#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h>
#include <BipedalLocomotion/RobotInterface/YarpHelper.h>
#include <BipedalLocomotion/RobotInterface/YarpRobotControl.h>
#include <BipedalLocomotion/RobotInterface/YarpSensorBridge.h>

#include <Eigen/Dense>

#include <BipedalLocomotion/JointPositionTracking/Module.h>

#include <yarp/dev/IEncoders.h>

using Vector1d = Eigen::Matrix<double, 1, 1>;

using namespace BipedalLocomotion;
using namespace BipedalLocomotion::JointPositionTracking;

double Module::getPeriod()
{
    return std::chrono::duration<double>(m_dT).count();
}

bool Module::createPolydriver(std::shared_ptr<ParametersHandler::IParametersHandler> handler)
{
    constexpr auto logPrefix = "[Module::createPolydriver]";
    auto ptr = handler->getGroup("ROBOT_INTERFACE").lock();
    if (ptr == nullptr)
    {
        log()->error("{} Unable to find the group ROBOT_INTERFACE.", logPrefix);
        return false;
    }
    ptr->setParameter("local_prefix", this->getName());
    m_controlBoard = RobotInterface::constructRemoteControlBoardRemapper(ptr);
    if (!m_controlBoard.isValid())
    {
        log()->error("{} Unable to create the polydriver.", logPrefix);
        return false;
    }

    // check the number of controlled joints
    int controlBoardDOFs = 0;
    yarp::dev::IEncoders* axis;
    m_controlBoard.poly->view(axis);
    if (axis != nullptr)
    {
        axis->getAxes(&controlBoardDOFs);
    }

    if (controlBoardDOFs != 1)
    {
        log()->error("{} The current implementation can be used to control only one joint. "
                     "Please be sure that the size of the joint_list and remote_control_boards is "
                     "equal to one.",
                     logPrefix);
        return false;
    }

    return true;
}

bool Module::initializeRobotControl(std::shared_ptr<ParametersHandler::IParametersHandler> handler)
{
    if (!m_robotControl.initialize(handler->getGroup("ROBOT_CONTROL")))
    {
        log()->error("[Module::initializeRobotControl] Unable to initialize the control board");
        return false;
    }
    if (!m_robotControl.setDriver(m_controlBoard.poly))
    {
        log()->error("[Module::initializeRobotControl] Unable to set the driver");
        return false;
    }

    return true;
}

bool Module::instantiateSensorBridge(std::shared_ptr<ParametersHandler::IParametersHandler> handler)
{
    if (!m_sensorBridge.initialize(handler->getGroup("SENSOR_BRIDGE")))
    {
        log()->error("[Module::initializeSensorBridge] Unable to initialize the sensor bridge");
        return false;
    }

    yarp::dev::PolyDriverList list;
    list.push(m_controlBoard.poly.get(), m_controlBoard.key.c_str());
    if (!m_sensorBridge.setDriversList(list))
    {
        log()->error("[Module::initializeSensorBridge] Unable to set the driver list");
        return false;
    }

    return true;
}

bool Module::configure(yarp::os::ResourceFinder& rf)
{
    m_currentJointPos.resize(1);

    auto parametersHandler = std::make_shared<ParametersHandler::YarpImplementation>(rf);

    std::string name;
    if (!parametersHandler->getParameter("name", name))
    {
        log()->error("[Module::configure] Unable to find the parameter 'name'.");
        return false;
    }
    this->setName(name.c_str());

    if (!parametersHandler->getParameter("sampling_time", m_dT))
    {
        log()->error("[Module::configure] Unable to find the parameter 'sampling_time'.");
        return false;
    }

    double maxValue = 0;
    if (!parametersHandler->getParameter("max_angle_deg", maxValue))
    {
        log()->error("[Module::configure] Unable to find the parameter 'max_angle_deg'.");
        return false;
    }

    maxValue *= M_PI / 180;

    double minValue = 0;
    if (!parametersHandler->getParameter("min_angle_deg", minValue))
    {
        log()->error("[Module::configure] Unable to find the parameter 'min_angle_deg'.");
        return false;
    }

    minValue *= M_PI / 180;

    double trajectoryDuration = 5;
    if (!parametersHandler->getParameter("trajectory_duration", trajectoryDuration))
    {
        log()->error("[Module::configure] Unable to find the parameter 'trajectory_duration'.");
        return false;
    }

    this->createPolydriver(parametersHandler);
    this->initializeRobotControl(parametersHandler);
    this->instantiateSensorBridge(parametersHandler);

    m_setPoints.push_back((maxValue + minValue) / 2);
    m_setPoints.push_back(maxValue);
    m_setPoints.push_back(minValue);
    m_setPoints.push_back((maxValue + minValue) / 2);

    m_spline.setAdvanceTimeStep(m_dT);
    m_spline.setInitialConditions(Eigen::VectorXd::Zero(1), Eigen::VectorXd::Zero(1));
    m_spline.setFinalConditions(Eigen::VectorXd::Zero(1), Eigen::VectorXd::Zero(1));

    m_timeKnots.clear();
    m_timeKnots.push_back(std::chrono::nanoseconds::zero());
    m_timeKnots.push_back(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(trajectoryDuration)));

    if (!m_sensorBridge.advance())
    {
        log()->error("[Module::configure] Unable to read the sensor.");
        return false;
    }
    m_sensorBridge.getJointPositions(m_currentJointPos);

    // the trajectory will bring the robot in the initial configuration
    m_setPoints.push_back(m_currentJointPos[0]);
    m_currentSetPoint = m_setPoints.begin();

    m_trajectoryKnots.push_back(m_currentJointPos);
    m_trajectoryKnots.push_back(Vector1d::Constant(*m_currentSetPoint));

    m_spline.setKnots(m_trajectoryKnots, m_timeKnots);

    m_initTrajectoryTime = yarp::os::Time::now();

    // switch in position direct control
    if (!m_robotControl.setControlMode(RobotInterface::ControlMode::PositionDirect))
    {
        log()->error("[Module::configure] Unable to switch in position direct control.");
        return false;
    }

    log()->info("[Module::configure] Module configured.");

    return true;
}

bool Module::generateNewTrajectory()
{
    double initTrajectory = *m_currentSetPoint;

    // the trajectory is ended
    if (std::next(m_currentSetPoint, 1) == m_setPoints.end())
    {
        return false;
    }

    std::advance(m_currentSetPoint, 1);
    double endTrajectory = *m_currentSetPoint;

    m_trajectoryKnots[0] = Vector1d::Constant(initTrajectory);
    m_trajectoryKnots[1] = Vector1d::Constant(endTrajectory);

    m_spline.setKnots(m_trajectoryKnots, m_timeKnots);

    m_initTrajectoryTime = yarp::os::Time::now();

    return true;
}

bool Module::updateModule()
{
    constexpr auto logPrefix = "[Module::updateModule]";

    if (!m_sensorBridge.advance())
    {
        log()->error("{} Unable to read the sensor.", logPrefix);
        return false;
    }

    if (!m_sensorBridge.getJointPositions(m_currentJointPos))
    {
        log()->error("{} Unable to get the joint position.", logPrefix);
        return false;
    }

    // set the reference
    if (!m_robotControl.setReferences(m_spline.getOutput().position,
                                      RobotInterface::ControlMode::PositionDirect,
                                      m_currentJointPos))
    {
        log()->error("{} Unable to set the reference.", logPrefix);
        return false;
    }

    m_logJointPos.push_back(m_currentJointPos[0]);
    m_logDesiredJointPos.push_back(m_spline.getOutput().position[0]);

    // advance the spline
    if (!m_spline.advance())
    {
        log()->error("{} Unable to advance the spline.", logPrefix);
        return false;
    }

    const double now = yarp::os::Time::now();
    if (now - m_initTrajectoryTime > std::chrono::duration<double>(m_timeKnots.back()).count() + 2)
    {
        std::cout << "[Module::updateModule] Generate a new trajectory." << std::endl;

        if (!generateNewTrajectory())
        {
            std::cerr << "[Module::updateModule] Experiment finished." << std::endl;
            return false;
        }
    }

    return true;
}

bool Module::close()
{
    log()->info("[Module::close] Storing the dataset.");

    // set the file name
    std::time_t t = std::time(nullptr);
    std::tm tm = *std::localtime(&t);

    std::ofstream stream; /**< std stream. */
    std::stringstream fileName;

    fileName << "Dataset_" << m_robotControl.getJointList().front() << "_"
             << std::put_time(&tm, "%Y_%m_%d_%H_%M_%S") << ".txt";
    stream.open(fileName.str().c_str());

    const auto minSize = std::min(m_logJointPos.size(), m_logDesiredJointPos.size());

    stream << "desired_joint, measured_joint" << std::endl;
    for (int i = 0; i < minSize; i++)
        stream << m_logDesiredJointPos[i] << ", " << m_logJointPos[i] << std::endl;

    stream.close();

    log()->info("[Module::close] Dataset stored in {}", fileName.str());

    // switch back in position control
    if (!m_robotControl.setControlMode(RobotInterface::ControlMode::Position))
    {
        log()->error("[Module::close] Unable to switch back in position control.");
        return false;
    }

    return true;
}
