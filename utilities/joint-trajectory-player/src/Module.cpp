/**
 * @file Module.cpp
 * @authors Ines Sorrentino
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <iomanip>

#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h>
#include <BipedalLocomotion/RobotInterface/YarpHelper.h>
#include <BipedalLocomotion/RobotInterface/YarpRobotControl.h>
#include <BipedalLocomotion/RobotInterface/YarpSensorBridge.h>

#include <Eigen/Dense>

#include <BipedalLocomotion/JointTrajectoryPlayer/Module.h>

#include <yarp/dev/IEncoders.h>

#include <BipedalLocomotion/Conversions/matioCppConversions.h>

using namespace BipedalLocomotion;
using namespace BipedalLocomotion::JointTrajectoryPlayer;

double Module::getPeriod()
{
    return m_dT;
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

    return true;
}

bool Module::initializeRobotControl(std::shared_ptr<ParametersHandler::IParametersHandler> handler)
{
    constexpr auto logPrefix = "[Module::initializeRobotControl]";
    if (!m_robotControl.initialize(handler->getGroup("ROBOT_CONTROL")))
    {
        log()->error("{} Unable to initialize the robot control.", logPrefix);
        return false;
    }
    if (!m_robotControl.setDriver(m_controlBoard.poly))
    {
        log()->error("{} Unable to set the driver.", logPrefix);
        return false;
    }

    return true;
}

bool Module::instantiateSensorBridge(std::shared_ptr<ParametersHandler::IParametersHandler> handler)
{
    constexpr auto logPrefix = "[Module::instantiateSensorBridge]";
    if (!m_sensorBridge.initialize(handler->getGroup("SENSOR_BRIDGE")))
    {
        log()->error("{} Unable to initialize the sensor bridge.", logPrefix);
        return false;
    }

    yarp::dev::PolyDriverList list;
    list.push(m_controlBoard.poly.get(), m_controlBoard.key.c_str());
    if (!m_sensorBridge.setDriversList(list))
    {
        log()->error("{} Unable to set the drivers list.", logPrefix);
        return false;
    }

    return true;
}

bool Module::readStateFromFile(const std::string& filename, const std::size_t numFields)
{
    constexpr auto logPrefix = "[Module::readStateFromFile]";
    std::deque<Eigen::VectorXd> data;

    matioCpp::File input(filename);

    if (!input.isOpen())
    {
        log()->error("{} Unable to open the file {}.", logPrefix, filename);
        return false;
    } else
    {
        m_traj = input.read("traj").asMultiDimensionalArray<double>(); // Read a multi dimensional
                                                                       // array named "traj"
        if (!m_traj.isValid())
        {
            log()->error("{} Unable to read the trajectory from the file.", logPrefix);
            return false;
        }

        return true;
    }
}

bool Module::configure(yarp::os::ResourceFinder& rf)
{
    constexpr auto logPrefix = "[Module::configure]";
    auto parametersHandler = std::make_shared<ParametersHandler::YarpImplementation>(rf);

    std::string name;
    if (!parametersHandler->getParameter("name", name))
    {
        log()->error("{} Unable to find the parameter 'name'.", logPrefix);
        return false;
    }
    this->setName(name.c_str());

    if (!parametersHandler->getParameter("sampling_time", m_dT))
    {
        log()->error("{} Unable to find the parameter 'sampling_time'.", logPrefix);
        return false;
    }

    std::string trajectoryFile;
    if (!parametersHandler->getParameter("trajectory_file", trajectoryFile))
    {
        log()->error("{} Unable to find the parameter 'trajectory_file'.", logPrefix);
        return false;
    }

    if (!this->createPolydriver(parametersHandler))
    {
        log()->error("{} Unable to create the polydriver.", logPrefix);
        return false;
    }
    if (!this->initializeRobotControl(parametersHandler))
    {
        log()->error("{} Unable to initialize the robot control.", logPrefix);
        return false;
    }
    if (!this->instantiateSensorBridge(parametersHandler))
    {
        log()->error("{} Unable to instantiate the sensor bridge.", logPrefix);
        return false;
    }

    m_numOfJoints = m_robotControl.getJointList().size();
    if (m_numOfJoints == 0)
    {
        log()->error("{} Are you trying to control a robot without joints?", logPrefix);
        return false;
    }

    m_axisList = m_robotControl.getJointList();

    m_currentJointPos.resize(m_numOfJoints);
    m_currentJointVel.resize(m_numOfJoints);
    m_currentMotorCurr.resize(m_numOfJoints);

    if (!readStateFromFile(trajectoryFile, m_numOfJoints))
    {
        log()->error("{} Unable to read the trajectory from the file.", logPrefix);
        return false;
    }

    log()->info("{} Module configured.", logPrefix);

    // switch to position control
    if (!m_robotControl.setControlMode(RobotInterface::ControlMode::Position))
    {
        log()->error("{} Unable to set the control mode to position.", logPrefix);
        return false;
    }

    // Reach the first position of the desired trajectory in position control
    if (!m_robotControl.setReferences(Conversions::toEigen(m_traj).row(m_idxTraj),
                                      RobotInterface::ControlMode::Position))
    {
        log()->error("{} Error while setting the reference position.", logPrefix);
        return false;
    }

    m_state = State::positioning;

    return true;
}

bool Module::updateModule()
{
    constexpr auto logPrefix = "[Module::updateModule]";
    bool isMotionDone;
    bool isTimeExpired;
    std::vector<std::pair<std::string, double>> jointlist;

    switch (m_state)
    {
    case State::positioning:
        if (!m_robotControl.checkMotionDone(isMotionDone, isTimeExpired, jointlist))
        {
            log()->error("{} Unable to check if the motion is done.", logPrefix);
            return false;
        }
        if (isTimeExpired)
        {
            log()->error("{} The timer expired.", logPrefix);
            for (int i = 0; i < jointlist.size(); i++)
            {
                log()->error("{} Joint {} --> Error {} rad",
                             logPrefix,
                             jointlist[i].first,
                             jointlist[i].second);
            }
            return false;
        }
        if (isMotionDone)
        {
            log()->info("{} Positioning done.", logPrefix);

            // switch in position direct control
            if (!m_robotControl.setControlMode(RobotInterface::ControlMode::PositionDirect))
            {
                log()->error("{} Unable to switch in position direct control.", logPrefix);
                return false;
            }

            m_state = State::running;
        }
        break;

    case State::running:
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

        if (!m_sensorBridge.getJointVelocities(m_currentJointVel))
        {
            log()->error("{} Unable to get the joint velocity.", logPrefix);
            return false;
        }

        if (!m_sensorBridge.getMotorCurrents(m_currentMotorCurr))
        {
            log()->error("{} Unable to get the motor currents.", logPrefix);
            return false;
        }

        // log data
        m_log["time"].push_back(yarp::os::Time::now());
        for (int i = 0; i < m_numOfJoints; i++)
        {
            m_log[m_axisList[i] + "_pos"].push_back(m_currentJointPos[i]);
        }

        m_log["time"].push_back(yarp::os::Time::now());
        for (int i = 0; i < m_numOfJoints; i++)
        {
            m_log[m_axisList[i] + "_vel"].push_back(m_currentJointVel[i]);
        }

        for (int i = 0; i < m_numOfJoints; i++)
        {
            m_log[m_axisList[i] + "_curr"].push_back(m_currentMotorCurr[i]);
        }

        m_idxTraj++;
        if (m_idxTraj == Conversions::toEigen(m_traj).rows())
        {
            log()->info("{} Trajectory ended.", logPrefix);
            return false;
        }

        // set the reference
        if (!m_robotControl.setReferences(Conversions::toEigen(m_traj).row(m_idxTraj),
                                          RobotInterface::ControlMode::PositionDirect))
        {
            log()->error("{} Unable to set the reference.", logPrefix);
            return false;
        }

        break;

    default:
        log()->error("{} Unknown state.", logPrefix);
        return false;
    }

    return true;
}

bool Module::close()
{
    log()->info("[Module::close] Storing the dataset.");

    // set the file name
    std::time_t t = std::time(nullptr);
    std::tm tm = *std::localtime(&t);

    std::stringstream fileName;

    fileName << "Dataset_Measured_" << m_robotControl.getJointList().front() << "_"
             << std::put_time(&tm, "%Y_%m_%d_%H_%M_%S") << ".mat";

    matioCpp::File file = matioCpp::File::Create(fileName.str());

    for (auto& [key, value] : m_log)
    {
        matioCpp::Vector<double> out(key);
        out = value;
        file.write(out);
    }

    log()->info("[Module::close] Dataset stored in {}.", fileName.str());

    // switch back in position control
    if (!m_robotControl.setControlMode(RobotInterface::ControlMode::Position))
    {
        log()->error("[Module::close] Unable to switch back in position control.");
        return false;
    }

    return true;
}
