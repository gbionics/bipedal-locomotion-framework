/**
 * @file YarpSensorBridgeImpl.h
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_ROBOT_INTERFACE_YARP_SENSOR_BRIDGE_IMPL_H
#define BIPEDAL_LOCOMOTION_ROBOT_INTERFACE_YARP_SENSOR_BRIDGE_IMPL_H

#include <BipedalLocomotion/GenericContainer/Vector.h>
#include <BipedalLocomotion/RobotInterface/YarpSensorBridge.h>
#include <BipedalLocomotion/System/Clock.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

// YARP os
#include <yarp/os/Time.h>

// YARP sig
#include <yarp/sig/Image.h>
#include <yarp/sig/Vector.h>

// YARP Sensor Interfaces
#include <yarp/dev/IAmplifierControl.h>
#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/ICurrentControl.h>
#include <yarp/dev/IGenericSensor.h>
#include <yarp/dev/IMotor.h>
#include <yarp/dev/IMotorEncoders.h>
#include <yarp/dev/IPidControl.h>
#include <yarp/dev/ITorqueControl.h>
#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>

// YARP Camera Interfaces
#include <yarp/dev/IRGBDSensor.h>

// YARP Control Board Interfaces
#include <yarp/dev/IAxisInfo.h>
#include <yarp/dev/IEncodersTimed.h>

// yarp eigen
#include <yarp/eigen/Eigen.h>

// std
#include <algorithm>
#include <cmath>
#include <set>

namespace BipedalLocomotion
{
namespace RobotInterface
{

struct YarpSensorBridge::Impl
{
    using StampedYARPVector = std::pair<yarp::sig::Vector, double>;

    /**
     * Struct holding remapped remote control board interfaces
     */
    struct ControlBoardRemapperInterfaces
    {
        yarp::dev::IEncodersTimed* encoders{nullptr};
        yarp::dev::IAxisInfo* axis{nullptr};
        yarp::dev::ICurrentControl* currsensors{nullptr};
        yarp::dev::ITorqueControl* torques{nullptr};
        yarp::dev::IMotorEncoders* motorEncoders{nullptr};
        yarp::dev::IMotor* motor{nullptr};
        yarp::dev::IPidControl* pids{nullptr};
        yarp::dev::IAmplifierControl* amp{nullptr};
    };

    ControlBoardRemapperInterfaces controlBoardRemapperInterfaces;

    /**
     * Struct holding remapped MAS interfaces -inertial sensors related
     */
    struct MASInertialsInterface
    {
        yarp::dev::IThreeAxisLinearAccelerometers* accelerometers{nullptr};
        yarp::dev::IThreeAxisGyroscopes* gyroscopes{nullptr};
        yarp::dev::IThreeAxisMagnetometers* magnetometers{nullptr};
        yarp::dev::IOrientationSensors* orientationSensors{nullptr};
    };

    MASInertialsInterface masInertialsInterface;

    /**
     * Struct holding remapped MAS interfaces - FT sensors related
     */
    struct MASForceTorquesInterface
    {
        yarp::dev::ISixAxisForceTorqueSensors* sixAxisFTSensors{nullptr};
        yarp::dev::ITemperatureSensors* temperatureSensors{nullptr};
    };
    MASForceTorquesInterface masForceTorquesInterface;

    struct MASSensorIndexMaps
    {
        std::unordered_map<std::string, std::size_t> accelerometer;
        std::unordered_map<std::string, std::size_t> gyroscopes;
        std::unordered_map<std::string, std::size_t> magnetometers;
        std::unordered_map<std::string, std::size_t> orientationSensors;
        std::unordered_map<std::string, std::size_t> sixAxisFTSensors;
        std::unordered_map<std::string, std::size_t> temperatureSensors;
    };

    MASSensorIndexMaps masSensorIndexMaps;

    /**
     * Struct holding measurements polled from remapped remote control board interfaces
     */
    struct ControlBoardRemapperMeasures
    {
        Eigen::VectorXi remappedJointIndices;
        Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic> remappedJointPermutationMatrix;
        Eigen::VectorXd jointPositions;
        Eigen::VectorXd jointVelocities;
        Eigen::VectorXd jointAccelerations;
        Eigen::VectorXd motorCurrents;
        Eigen::VectorXd jointTorques;
        Eigen::VectorXd motorPositions;
        Eigen::VectorXd motorVelocities;
        Eigen::VectorXd motorAccelerations;
        Eigen::VectorXd pidPositions;
        Eigen::VectorXd pidPositionErrors;
        Eigen::VectorXd motorPWMs;
        Eigen::VectorXd motorTemperatures;

        Eigen::VectorXd jointPositionsUnordered;
        Eigen::VectorXd jointVelocitiesUnordered;
        Eigen::VectorXd jointAccelerationsUnordered;
        Eigen::VectorXd motorCurrentsUnordered;
        Eigen::VectorXd jointTorquesUnordered;
        Eigen::VectorXd motorPositionsUnordered;
        Eigen::VectorXd motorVelocitiesUnordered;
        Eigen::VectorXd motorAccelerationsUnordered;
        Eigen::VectorXd motorTemperaturesUnordered;
        Eigen::VectorXd pidPositionsUnordered;
        Eigen::VectorXd pidPositionErrorsUnordered;
        Eigen::VectorXd motorPWMsUnordered;

        double receivedTimeInSeconds;
    };

    ControlBoardRemapperMeasures controlBoardRemapperMeasures;

    /**< map of IMU sensors attached through generic sensor interfaces */
    std::unordered_map<std::string, yarp::dev::IGenericSensor*> analogIMUInterface;

    /**< map of cartesian wrench streams attached through generic sensor interfaces */
    std::unordered_map<std::string, yarp::dev::IGenericSensor*> cartesianWrenchInterface;

    /**< map of six axis force torque sensors attached through analog sensor interfaces */
    std::unordered_map<std::string, yarp::dev::IAnalogSensor*> analogSixAxisFTSensorsInterface;

    /** < map holding analog IMU sensor measurements (Used only for analog sensor interfaces) */
    std::unordered_map<std::string, StampedYARPVector> IMUMeasures;

    /**< map holding six axis force torque measures */
    std::unordered_map<std::string, StampedYARPVector> FTMeasures;

    /**< maps holding three axis inertial sensor measures */
    std::unordered_map<std::string, StampedYARPVector> gyroMeasures;
    std::unordered_map<std::string, StampedYARPVector> accelerometerMeasures;
    std::unordered_map<std::string, StampedYARPVector> orientationMeasures;
    std::unordered_map<std::string, StampedYARPVector> magnetometerMeasures;

    /**< map holding cartesian wrench measures */
    std::unordered_map<std::string, StampedYARPVector> cartesianWrenchMeasures;

    /**< map holding temperature measures */
    std::unordered_map<std::string, StampedYARPVector> temperatureMeasures;

    const int nrChannelsInYARPGenericIMUSensor{12};
    const int nrChannelsInYARPGenericCartesianWrench{6};
    const int nrChannelsInYARPAnalogSixAxisFTSensor{6};

    std::vector<std::string> failedSensorReads;
    SensorBridgeMetaData metaData; /**< struct holding meta data **/
    bool bridgeInitialized{false}; /**< flag set to true if the bridge is successfully initialized
                                    */
    bool driversAttached{false}; /**< flag set to true if the bridge is successfully attached to
                                    required device drivers */
    bool checkForNAN{false}; /**< flag to enable search for NANs in the incoming measurement buffers
                              */
    bool streamJointAccelerations{true}; /**< flag to enable reading joint accelerations from
                                            encoders */
    bool allJointsAreRevolutes{false}; /**< flag to indicate if all the joints are revolute */
    bool allJointsArePrismatics{false}; /**< flag to indicate if all the joints are prismatic */

    using SubConfigLoader = bool (YarpSensorBridge::Impl::*)(
        std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler>,
        SensorBridgeMetaData&);

    /**
     * Utility function to get index from vector
     */
    bool getIndexFromVector(const std::vector<std::string>& vec, //
                            const std::string& query,
                            int& index)
    {
        auto iter{std::lower_bound(vec.begin(), vec.end(), query)};
        if (iter == vec.end())
        {
            return false;
        }

        index = iter - vec.begin();
        return true;
    }

    /**
     * Utility function to check for nan in vector
     */
    bool nanExistsInVec(Eigen::Ref<const Eigen::VectorXd> vec,
                        std::string_view logPrefix,
                        const std::string& sensorName)
    {
        if (vec.array().isNaN().any())
        {
            log()->error("{} NAN values read from {}, use previous measurement.",
                         logPrefix,
                         sensorName);
            return true;
        }
        return false;
    }

    /**
     * Checks is a stream is enabled in configuration and
     * loads the relevant stream group from configuration
     */
    bool subConfigLoader(const std::string& enableStreamString,
                         const std::string& streamGroupString,
                         const SubConfigLoader loader,
                         std::weak_ptr<const ParametersHandler::IParametersHandler> handler,
                         SensorBridgeMetaData& metaData,
                         bool& enableStreamFlag)
    {
        constexpr auto logPrefix = "[YarpSensorBridge::Impl::subConfigLoader]";

        auto ptr = handler.lock();
        if (ptr == nullptr)
        {
            log()->error("{} The handler is not pointing to an already initialized memory.",
                         logPrefix);
            return false;
        }

        enableStreamFlag = false;
        if (ptr->getParameter(enableStreamString, enableStreamFlag) && enableStreamFlag)
        {
            auto groupHandler = ptr->getGroup(streamGroupString);
            if (!(this->*loader)(groupHandler, metaData))
            {
                log()->error("{} {} group could not be initialized from the configuration file.",
                             logPrefix,
                             streamGroupString);
                return false;
            }
        }

        return true;
    }

    /**
     * Configure remote control board remapper meta data
     * Related to kinematics and other joint/motor relevant quantities
     */
    bool configureRemoteControlBoardRemapper(
        std::weak_ptr<const ParametersHandler::IParametersHandler> handler,
        SensorBridgeMetaData& metaData)
    {
        // This means that the joints list is already configured
        if (metaData.bridgeOptions.nrJoints > 0)
        {
            return true;
        }

        // clear the joint list
        metaData.sensorsList.jointsList.clear();

        constexpr auto logPrefix = "[YarpSensorBridge::Impl::configureRemoteControlBoardRemapper]";
        auto ptr = handler.lock();

        // in general if a group is not defined means there is an error in the configuration file.
        // In this particlar case since the only parameter is optional, having an empty
        // configuration file is not a problem.
        if (ptr == nullptr)
        {
            // Notice in case of other REQUIRED parameters here a return false is necessary.
            log()->info("{} The parameter 'joints_list' in not available in the configuration. The "
                        "order of the joints will be the one passed in RemoteControlBoardRemapper.",
                        logPrefix);
            return true;
        }

        if (!ptr->getParameter("joints_list", metaData.sensorsList.jointsList))
        {
            log()->info("{} The parameter 'joints_list' in not available in the configuration. The "
                        "order of the joints will be the one passed in RemoteControlBoardRemapper.",
                        logPrefix);
            return true;
        }

        metaData.bridgeOptions.nrJoints = metaData.sensorsList.jointsList.size();
        return true;
    }

    /**
     * Configure inertial sensors meta data
     */
    bool
    configureInertialSensors(std::weak_ptr<const ParametersHandler::IParametersHandler> handler,
                             SensorBridgeMetaData& metaData)
    {
        constexpr auto logPrefix = "[YarpSensorBridge::Impl::configureInertialSensors]";
        auto ptr = handler.lock();
        if (ptr == nullptr)
        {
            return false;
        }

        if (ptr->getParameter("imu_list", metaData.sensorsList.IMUsList))
        {
            metaData.bridgeOptions.isIMUEnabled = true;
        }

        if (ptr->getParameter("accelerometers_list", metaData.sensorsList.linearAccelerometersList))
        {
            metaData.bridgeOptions.isLinearAccelerometerEnabled = true;
        }

        if (ptr->getParameter("gyroscopes_list", metaData.sensorsList.gyroscopesList))
        {
            metaData.bridgeOptions.isGyroscopeEnabled = true;
        }

        if (ptr->getParameter("orientation_sensors_list",
                              metaData.sensorsList.orientationSensorsList))
        {
            metaData.bridgeOptions.isOrientationSensorEnabled = true;
        }

        if (ptr->getParameter("magnetometers_list", metaData.sensorsList.magnetometersList))
        {
            metaData.bridgeOptions.isMagnetometerEnabled = true;
        }

        return true;
    }

    /**
     * Configure six axis force torque sensors meta data
     */
    bool configureSixAxisForceTorqueSensors(
        std::weak_ptr<const ParametersHandler::IParametersHandler> handler,
        SensorBridgeMetaData& metaData)
    {
        constexpr auto logPrefix = "[YarpSensorBridge::Impl::configureSixAxisForceTorqueSensors]";
        auto ptr = handler.lock();
        if (ptr == nullptr)
        {
            return false;
        }

        if (!ptr->getParameter("sixaxis_forcetorque_sensors_list",
                               metaData.sensorsList.sixAxisForceTorqueSensorsList))
        {
            log()->error("{} Required parameter \"sixaxis_forcetorque_sensors_list\" not available "
                         "in the configuration.",
                         logPrefix);
            return false;
        }

        return true;
    }

    /**
     * Configure cartesian wrenches meta data
     */
    bool
    configureCartesianWrenches(std::weak_ptr<const ParametersHandler::IParametersHandler> handler,
                               SensorBridgeMetaData& metaData)
    {
        constexpr auto logPrefix = "[YarpSensorBridge::Impl::configureCartesianWrenchSensors] ";
        auto ptr = handler.lock();
        if (ptr == nullptr)
        {
            return false;
        }

        if (!ptr->getParameter("cartesian_wrenches_list",
                               metaData.sensorsList.cartesianWrenchesList))
        {
            log()->error("{} Required parameter \"cartesian_wrenches_list\" not available "
                         "in the configuration.",
                         logPrefix);
            return false;
        }

        return true;
    }

    /**
     * Configure temperature sensors meta data
     */
    bool
    configureTemperatureSensors(std::weak_ptr<const ParametersHandler::IParametersHandler> handler,
                                SensorBridgeMetaData& metaData)
    {
        constexpr auto logPrefix = "[YarpSensorBridge::Impl::configureTemperatureSensors] ";
        auto ptr = handler.lock();
        if (ptr == nullptr)
        {
            return false;
        }

        if (!ptr->getParameter("temperature_sensors_list",
                               metaData.sensorsList.temperatureSensorsList))
        {
            log()->error("{} Required parameter \"temperature_sensors_list\" not available "
                         "in the configuration.",
                         logPrefix);
            return false;
        }
        return true;
    }

    /**
     * Attach device with IGenericSensor or IAnalogSensor interfaces
     * Important assumptions here,
     * - Any generic sensor with 12 channels is a IMU sensor
     * - Any generic sensor with 6 channels is a cartesian wrench sensor
     * - Any analog sensor with 6 channels is a six axis force torque sensor
     */
    template <typename SensorType>
    bool attachGenericOrAnalogSensor(const yarp::dev::PolyDriverList& devList,
                                     const std::string& sensorName,
                                     const int& nrChannelsInSensor,
                                     std::unordered_map<std::string, SensorType*>& sensorMap)
    {
        constexpr auto logPrefix = "[YarpSensorBridge::Impl::attachGenericOrAnalogSensor]";
        for (int devIdx = 0; devIdx < devList.size(); devIdx++)
        {
            if (sensorName != devList[devIdx]->key)
            {
                continue;
            }

            SensorType* sensorInterface{nullptr};
            if (devList[devIdx]->poly->view(sensorInterface))
            {
                if (sensorInterface == nullptr)
                {
                    log()->error("{} {} Could not view interface.", logPrefix, sensorName);
                    return false;
                }

                int nrChannels;
                if constexpr (std::is_same_v<SensorType, yarp::dev::IAnalogSensor>)
                {
                    nrChannels = sensorInterface->getChannels();
                } else if constexpr (std::is_same_v<SensorType, yarp::dev::IGenericSensor>)
                {
                    sensorInterface->getChannels(&nrChannels);
                }

                if (nrChannels != nrChannelsInSensor)
                {
                    log()->error("{} {} Mismatch in the expected number of channels in the sensor "
                                 "stream.",
                                 logPrefix,
                                 sensorName);
                    return false;
                }

                sensorMap[devList[devIdx]->key] = sensorInterface;
            } else
            {
                log()->error("{} {} Could not view interface.", logPrefix, sensorName);
                return false;
            }
        }
        return true;
    }

    template <typename SensorType>
    bool attachAllGenericOrAnalogSensors(const yarp::dev::PolyDriverList& devList,
                                         std::unordered_map<std::string, SensorType*>& sensorMap,
                                         const int& nrChannelsInSensor,
                                         const std::vector<std::string>& sensorList,
                                         std::string_view interfaceType)
    {
        constexpr auto logPrefix = "[YarpSensorBridge::Impl::attachAllGenericOrAnalogSensors]";

        bool allSensorsAttachedSuccessful{true};
        for (const auto& genericSensorCartesianWrench : sensorList)
        {
            if (!attachGenericOrAnalogSensor(devList,
                                             genericSensorCartesianWrench,
                                             nrChannelsInSensor,
                                             sensorMap))
            {
                allSensorsAttachedSuccessful = false;
                break;
            }
        }

        if (!allSensorsAttachedSuccessful || (sensorMap.size() != sensorList.size()))
        {
            log()->error("{} Could not attach all desired {}.", logPrefix, interfaceType);
            return false;
        }

        return true;
    }

    /**
     * Attach Remapped Multiple Analog Sensor Interfaces and check available sensors
     */
    template <typename MASSensorType>
    bool attachAndCheckMASSensors(const yarp::dev::PolyDriverList& devList,
                                  MASSensorType*& sensorInterface,
                                  const std::vector<std::string>& sensorList,
                                  const std::string_view interfaceName)
    {
        constexpr auto logPrefix = "[YarpSensorBridge::Impl::attachAndCheckMASSensors]";

        if (!attachRemappedMASSensor(devList, sensorInterface))
        {
            log()->error("{} Could not find {} interface.", logPrefix, interfaceName);
            return false;
        }

        if (!checkAttachedMASSensors(devList, sensorInterface, sensorList))
        {
            log()->error("{} Could not find at least one of the required sensors.", logPrefix);
            return false;
        }

        return true;
    }

    /**
     * Attach Remapped Multiple Analog Sensor Interfaces
     * Looks for a specific MAS Sensor interface in the attached MAS Remapper
     */
    template <typename MASSensorType>
    bool attachRemappedMASSensor(const yarp::dev::PolyDriverList& devList,
                                 MASSensorType*& masSensorInterface)
    {
        constexpr auto logPrefix = "[YarpSensorBridge::Impl::attachRemappedMASSensor]";
        bool broken{false};
        for (int devIdx = 0; devIdx < devList.size(); devIdx++)
        {
            MASSensorType* sensorInterface{nullptr};
            devList[devIdx]->poly->view(sensorInterface);
            if (sensorInterface == nullptr)
            {
                continue;
            }

            // break out of the loop if we find relevant MAS interface
            masSensorInterface = sensorInterface;
            broken = true;
            break;
        }

        if (!broken || masSensorInterface == nullptr)
        {
            log()->error("{} Could not view interface.", logPrefix);
            return false;
        }
        return true;
    }

    /**
     * Check if all the desired MAS sensors are available in the attached MAS interface
     */
    template <typename MASSensorType>
    bool checkAttachedMASSensors(const yarp::dev::PolyDriverList& devList,
                                 MASSensorType*& sensorInterface,
                                 const std::vector<std::string>& sensorList)
    {
        constexpr auto logPrefix = "[YarpSensorBridge::Impl::checkAttachedMASSensors]";

        auto nrSensors{getNumberOfMASSensors(sensorInterface)};
        if (nrSensors != sensorList.size())
        {
            log()->error("{} Expecting the same number of MAS sensors attached to the Bridge as "
                         "many mentioned in the initialization step. Number of MAS sensor in the "
                         "interface: {}. "
                         "Number of sensor in initialization: {}.",
                         logPrefix,
                         nrSensors,
                         sensorList.size());
            return false;
        }

        for (const auto& masSensorName : sensorList)
        {
            bool sensorFound{false};
            for (std::size_t attachedIdx = 0; attachedIdx < nrSensors; attachedIdx++)
            {
                std::string attachedSensorName;
                if (!getMASSensorName(sensorInterface, attachedIdx, attachedSensorName))
                {
                    continue;
                }

                if (attachedSensorName == masSensorName)
                {
                    sensorFound = true;

                    // update mas sensor index map for lookup
                    if constexpr (std::is_same_v<MASSensorType, yarp::dev::IThreeAxisGyroscopes>)
                    {
                        masSensorIndexMaps.gyroscopes[masSensorName] = attachedIdx;
                    } else if constexpr (std::is_same_v<MASSensorType,
                                                        yarp::dev::IThreeAxisLinearAccelerometers>)
                    {
                        masSensorIndexMaps.accelerometer[masSensorName] = attachedIdx;
                    } else if constexpr (std::is_same_v<MASSensorType,
                                                        yarp::dev::IThreeAxisMagnetometers>)
                    {
                        masSensorIndexMaps.magnetometers[masSensorName] = attachedIdx;
                    } else if constexpr (std::is_same_v<MASSensorType,
                                                        yarp::dev::IOrientationSensors>)
                    {
                        masSensorIndexMaps.orientationSensors[masSensorName] = attachedIdx;
                    } else if constexpr (std::is_same_v<MASSensorType,
                                                        yarp::dev::ISixAxisForceTorqueSensors>)
                    {
                        masSensorIndexMaps.sixAxisFTSensors[masSensorName] = attachedIdx;
                    } else if constexpr (std::is_same_v<MASSensorType,
                                                        yarp::dev::ITemperatureSensors>)
                    {
                        masSensorIndexMaps.temperatureSensors[masSensorName] = attachedIdx;
                    }
                    break;
                }
            }

            if (!sensorFound)
            {
                log()->error("{} Bridge could not attach to MAS Sensor {}.",
                             logPrefix,
                             masSensorName);
            }
        }

        return true;
    }

    /**
     *  Get number of MAS Sensors
     */
    template <typename MASSensorType>
    std::size_t getNumberOfMASSensors(MASSensorType*& sensorInterface)
    {
        if (sensorInterface == nullptr)
        {
            return 0;
        }

        if constexpr (std::is_same_v<MASSensorType, yarp::dev::IThreeAxisGyroscopes>)
        {
            return sensorInterface->getNrOfThreeAxisGyroscopes();
        } else if constexpr (std::is_same_v<MASSensorType,
                                            yarp::dev::IThreeAxisLinearAccelerometers>)
        {
            return sensorInterface->getNrOfThreeAxisLinearAccelerometers();
        } else if constexpr (std::is_same_v<MASSensorType, yarp::dev::IThreeAxisMagnetometers>)
        {
            return sensorInterface->getNrOfThreeAxisMagnetometers();
        } else if constexpr (std::is_same_v<MASSensorType, yarp::dev::IOrientationSensors>)
        {
            return sensorInterface->getNrOfOrientationSensors();
        } else if constexpr (std::is_same_v<MASSensorType, yarp::dev::ISixAxisForceTorqueSensors>)
        {
            return sensorInterface->getNrOfSixAxisForceTorqueSensors();
        } else if constexpr (std::is_same_v<MASSensorType, yarp::dev::ITemperatureSensors>)
        {
            return sensorInterface->getNrOfTemperatureSensors();
        }

        return 0;
    }

    /**
     *  Get name of MAS Sensors
     */
    template <typename MASSensorType>
    bool getMASSensorName(MASSensorType*& sensorInterface,
                          const std::size_t& sensIdx,
                          std::string& sensorName)
    {
        if (sensorInterface == nullptr)
        {
            return false;
        }

        if constexpr (std::is_same_v<MASSensorType, yarp::dev::IThreeAxisGyroscopes>)
        {
            return sensorInterface->getThreeAxisGyroscopeName(sensIdx, sensorName);
        } else if constexpr (std::is_same_v<MASSensorType,
                                            yarp::dev::IThreeAxisLinearAccelerometers>)
        {
            return sensorInterface->getThreeAxisLinearAccelerometerName(sensIdx, sensorName);
        } else if constexpr (std::is_same_v<MASSensorType, yarp::dev::IThreeAxisMagnetometers>)
        {
            return sensorInterface->getThreeAxisMagnetometerName(sensIdx, sensorName);
        } else if constexpr (std::is_same_v<MASSensorType, yarp::dev::IOrientationSensors>)
        {
            return sensorInterface->getOrientationSensorName(sensIdx, sensorName);
        } else if constexpr (std::is_same_v<MASSensorType, yarp::dev::ISixAxisForceTorqueSensors>)
        {
            return sensorInterface->getSixAxisForceTorqueSensorName(sensIdx, sensorName);
        } else if constexpr (std::is_same_v<MASSensorType, yarp::dev::ITemperatureSensors>)
        {
            return sensorInterface->getTemperatureSensorName(sensIdx, sensorName);
        }
        return true;
    }

    /**
     * Get all sensor names in a MAS Inerface
     */
    template <typename MASSensorType>
    std::vector<std::string> getAllSensorsInMASInterface(MASSensorType* sensorInterface)
    {
        if (sensorInterface == nullptr)
        {
            return {};
        }

        std::vector<std::string> availableSensorNames;
        if constexpr (std::is_same_v<MASSensorType, yarp::dev::IThreeAxisGyroscopes>)
        {
            auto nrSensors = sensorInterface->getNrOfThreeAxisGyroscopes();
            for (std::size_t sensIdx = 0; sensIdx < nrSensors; sensIdx++)
            {
                std::string sensName;
                sensorInterface->getThreeAxisGyroscopeName(sensIdx, sensName);
                availableSensorNames.push_back(sensName);
            }
        } else if constexpr (std::is_same_v<MASSensorType,
                                            yarp::dev::IThreeAxisLinearAccelerometers>)
        {
            auto nrSensors = sensorInterface->getNrOfThreeAxisLinearAccelerometers();
            for (std::size_t sensIdx = 0; sensIdx < nrSensors; sensIdx++)
            {
                std::string sensName;
                sensorInterface->getThreeAxisLinearAccelerometerName(sensIdx, sensName);
                availableSensorNames.push_back(sensName);
            }
        } else if constexpr (std::is_same_v<MASSensorType, yarp::dev::IThreeAxisMagnetometers>)
        {
            auto nrSensors = sensorInterface->getNrOfThreeAxisMagnetometers();
            for (std::size_t sensIdx = 0; sensIdx < nrSensors; sensIdx++)
            {
                std::string sensName;
                sensorInterface->getThreeAxisMagnetometerName(sensIdx, sensName);
                availableSensorNames.push_back(sensName);
            }
        } else if constexpr (std::is_same_v<MASSensorType, yarp::dev::IOrientationSensors>)
        {
            auto nrSensors = sensorInterface->getNrOfOrientationSensors();
            for (std::size_t sensIdx = 0; sensIdx < nrSensors; sensIdx++)
            {
                std::string sensName;
                sensorInterface->getOrientationSensorName(sensIdx, sensName);
                availableSensorNames.push_back(sensName);
            }
        } else if constexpr (std::is_same_v<MASSensorType, yarp::dev::ISixAxisForceTorqueSensors>)
        {
            auto nrSensors = sensorInterface->getNrOfSixAxisForceTorqueSensors();
            for (std::size_t sensIdx = 0; sensIdx < nrSensors; sensIdx++)
            {
                std::string sensName;
                sensorInterface->getSixAxisForceTorqueSensorName(sensIdx, sensName);
                availableSensorNames.push_back(sensName);
            }
        } else if constexpr (std::is_same_v<MASSensorType, yarp::dev::ITemperatureSensors>)
        {
            auto nrSensors = sensorInterface->getNrOfTemperatureSensors();
            for (std::size_t sensIdx = 0; sensIdx < nrSensors; sensIdx++)
            {
                std::string sensName;
                sensorInterface->getTemperatureSensorName(sensIdx, sensName);
                availableSensorNames.push_back(sensName);
            }
        }
        return availableSensorNames;
    }

    /**
     * Check if sensor is available in the relevant sensor map
     */
    template <typename SensorType>
    bool checkSensor(const std::unordered_map<std::string, SensorType*>& sensorMap,
                     const std::string& sensorName)
    {
        if (sensorMap.find(sensorName) == sensorMap.end())
        {
            return false;
        }

        return true;
    }

    /**
     * Check if sensor is available in the relevant sensor measurement map
     */
    template <typename YARPDataType>
    bool checkValidSensorMeasure(std::string_view logPrefix,
                                 const std::unordered_map<std::string, YARPDataType>& sensorMap,
                                 const std::string& sensorName)
    {
        if (!checkValid(logPrefix))
        {
            return false;
        }

        if (sensorMap.find(sensorName) == sensorMap.end())
        {
            log()->error("{} {} sensor unavailable in the measurement map.", logPrefix, sensorName);
            return false;
        }

        return true;
    }

    /**
     * Check if the bridge is successfully initialized and attached to required device drivers
     */
    bool checkValid(const std::string_view methodName)
    {
        if (!(bridgeInitialized && driversAttached))
        {
            log()->error("{} SensorBridge is not ready. Please initialize and set drivers list.",
                         methodName);
            return false;
        }
        return true;
    }

    template <typename ControlBoardInterface>
    bool checkControlBoardSensor(const std::string logPrefix,
                                 ControlBoardInterface* interface,
                                 const bool& streamConfig,
                                 Eigen::Ref<const Eigen::VectorXd> measureBuffer)
    {
        if (!checkValid(logPrefix))
        {
            return false;
        }

        if (!streamConfig)
        {
            log()->error("{} Configuration flag set to {}. Please set this flag to true before "
                         "calling this method.",
                         logPrefix,
                         streamConfig);
            return false;
        }

        if (interface == nullptr)
        {
            log()->error("{} Failed to attach to relevant drivers. Unable to retrieve "
                         "measurements.",
                         logPrefix);
        }

        if (measureBuffer.size() == 0)
        {
            log()->error("{} Measurement buffers seem empty. Unable to retrieve measurements.",
                         logPrefix);
        }

        return true;
    }

    /**
     *  Attach generic IMU sensor types and MAS inertials
     */
    bool attachAllInertials(const yarp::dev::PolyDriverList& devList)
    {
        constexpr auto logPrefix = "[YarpSensorBridge::Impl::attachAllInertials]";
        if (metaData.bridgeOptions.isIMUEnabled)
        {
            // check if a generic sensor has 12 channels implying it is a IMU sensor through a
            // GenericSensor Interfaces
            std::string_view interfaceType{"Generic IMU Interface"};
            if (!attachAllGenericOrAnalogSensors(devList,
                                                 analogIMUInterface,
                                                 nrChannelsInYARPGenericIMUSensor,
                                                 metaData.sensorsList.IMUsList,
                                                 interfaceType))
            {
                log()->error("{} Unable to attach the imus as generic or analog sensors.",
                             logPrefix);
                return false;
            }
        }

        if (metaData.bridgeOptions.isLinearAccelerometerEnabled)
        {
            std::string_view interfaceType{"IThreeAxisLinearAccelerometers"};
            if (!attachAndCheckMASSensors(devList,
                                          masInertialsInterface.accelerometers,
                                          metaData.sensorsList.linearAccelerometersList,
                                          interfaceType))
            {
                log()->error("{} Unable to attach the accelerometer as MAS.", logPrefix);
                return false;
            }
        }

        if (metaData.bridgeOptions.isGyroscopeEnabled)
        {
            std::string_view interfaceType{"IThreeAxisGyroscopes"};
            if (!attachAndCheckMASSensors(devList,
                                          masInertialsInterface.gyroscopes,
                                          metaData.sensorsList.gyroscopesList,
                                          interfaceType))
            {
                log()->error("{} Unable to attach the gyros as MAS.", logPrefix);
                return false;
            }
        }

        if (metaData.bridgeOptions.isOrientationSensorEnabled)
        {
            std::string_view interfaceType{"IOrientationSensors"};
            if (!attachAndCheckMASSensors(devList,
                                          masInertialsInterface.orientationSensors,
                                          metaData.sensorsList.orientationSensorsList,
                                          interfaceType))
            {
                log()->error("{} Unable to attach the orientation as MAS.", logPrefix);
                return false;
            }
        }

        if (metaData.bridgeOptions.isMagnetometerEnabled)
        {
            std::string_view interfaceType{"IThreeAxisMagnetometers"};
            if (!attachAndCheckMASSensors(devList,
                                          masInertialsInterface.magnetometers,
                                          metaData.sensorsList.magnetometersList,
                                          interfaceType))
            {
                log()->error("{} Unable to attach the magnetoemeters as MAS.", logPrefix);
                return false;
            }
        }

        return true;
    }

    /**
     * Attach a remapped control board and check the availability of desired interface
     * Further, resize joint data buffers and check if
     * the control board joints list and the desired joints list match
     * Also, maintain a remapping index buffer for adapting to arbitrary joint list serializations
     */
    bool attachRemappedRemoteControlBoard(const yarp::dev::PolyDriverList& devList)
    {
        if (!metaData.bridgeOptions.isJointSensorsEnabled
            && !metaData.bridgeOptions.isPWMControlEnabled
            && !metaData.bridgeOptions.isMotorSensorsEnabled
            && !metaData.bridgeOptions.isPIDsEnabled)
        {
            // do nothing
            return true;
        }

        constexpr auto logPrefix = "[YarpSensorBridge::Impl::attachRemappedRemoteControlBoard]";
        // expects only one remotecontrolboard device attached to it, if found break!
        // if there multiple remote control boards, then  use a remapper to create a single
        // remotecontrolboard
        bool okJointsSensor = !metaData.bridgeOptions.isJointSensorsEnabled;
        bool okPWM = !metaData.bridgeOptions.isPWMControlEnabled;
        bool okMotorsSensor = !metaData.bridgeOptions.isMotorSensorsEnabled;
        bool okPID = !metaData.bridgeOptions.isPIDsEnabled;
        bool okMotorTemperature = !metaData.bridgeOptions.isMotorTemperatureSensorEnabled;

        for (int devIdx = 0; devIdx < devList.size(); devIdx++)
        {
            yarp::dev::PolyDriver* poly = devList[devIdx]->poly;

            if (!okJointsSensor && metaData.bridgeOptions.isJointSensorsEnabled)
            {
                okJointsSensor = poly->view(controlBoardRemapperInterfaces.axis);
                okJointsSensor = okJointsSensor
                                 && poly->view(controlBoardRemapperInterfaces.encoders);
                okJointsSensor = okJointsSensor
                                 && poly->view(controlBoardRemapperInterfaces.torques);
            }
            if (!okPWM && metaData.bridgeOptions.isPWMControlEnabled)
            {
                okPWM = devList[devIdx]->poly->view(controlBoardRemapperInterfaces.amp);
            }
            if (!okMotorsSensor && metaData.bridgeOptions.isMotorSensorsEnabled)
            {
                okMotorsSensor
                    = devList[devIdx]->poly->view(controlBoardRemapperInterfaces.motorEncoders);
                okMotorsSensor
                    = okMotorsSensor
                      && devList[devIdx]->poly->view(controlBoardRemapperInterfaces.currsensors);
            }
            if (!okPID && metaData.bridgeOptions.isPIDsEnabled)
            {
                okPID = devList[devIdx]->poly->view(controlBoardRemapperInterfaces.pids);
            }

            if (!okMotorTemperature && metaData.bridgeOptions.isMotorTemperatureSensorEnabled)
            {
                okMotorTemperature
                    = devList[devIdx]->poly->view(controlBoardRemapperInterfaces.motor);
            }

            if (okPID && okJointsSensor && okMotorsSensor && okPWM && okMotorTemperature)
            {
                if (!compareControlBoardJointsList())
                {
                    log()->error("{} Could not attach to remapped control board interface.",
                                 logPrefix);
                    return false;
                }

                return true;
            }
        }

        auto getSensorStatus = [](bool isRequired, bool isFound) -> const char* {
            if (isRequired)
            {
                return isFound ? "required and found" : "required but not found";
            }
            return "not required";
        };

        log()->error("{} Could not find a remapped remote control board with the desired "
                     "interfaces. Here the status of the interfaces. "
                     "Joint sensors: {}, "
                     "Motor sensors: {}, ",
                     "PID sensors: {}, ",
                     "PWM sensors: {}.",
                     logPrefix,
                     getSensorStatus(metaData.bridgeOptions.isJointSensorsEnabled, okJointsSensor),
                     getSensorStatus(metaData.bridgeOptions.isMotorSensorsEnabled, okMotorsSensor),
                     getSensorStatus(metaData.bridgeOptions.isPIDsEnabled, okPID),
                     getSensorStatus(metaData.bridgeOptions.isPWMControlEnabled, okPWM));

        return false;
    }

    /**
     * resize and set control board buffers to zero
     */
    void resetControlBoardBuffers()
    {
        // default joint variables
        // firstly resize all the controlboard data buffers
        if (metaData.bridgeOptions.isJointSensorsEnabled)
        {
            controlBoardRemapperMeasures.remappedJointPermutationMatrix.resize(
                metaData.bridgeOptions.nrJoints);
            controlBoardRemapperMeasures.remappedJointIndices.resize(
                metaData.bridgeOptions.nrJoints);

            controlBoardRemapperMeasures.jointPositions.resize(metaData.bridgeOptions.nrJoints);
            controlBoardRemapperMeasures.jointVelocities.resize(metaData.bridgeOptions.nrJoints);
            controlBoardRemapperMeasures.jointAccelerations.resize(metaData.bridgeOptions.nrJoints);
            controlBoardRemapperMeasures.jointTorques.resize(metaData.bridgeOptions.nrJoints);

            controlBoardRemapperMeasures.jointPositionsUnordered.resize(
                metaData.bridgeOptions.nrJoints);
            controlBoardRemapperMeasures.jointVelocitiesUnordered.resize(
                metaData.bridgeOptions.nrJoints);
            controlBoardRemapperMeasures.jointAccelerationsUnordered.resize(
                metaData.bridgeOptions.nrJoints);
            controlBoardRemapperMeasures.jointTorquesUnordered.resize(
                metaData.bridgeOptions.nrJoints);

            // zero buffers
            controlBoardRemapperMeasures.remappedJointPermutationMatrix.setIdentity();
            controlBoardRemapperMeasures.jointPositions.setZero();
            controlBoardRemapperMeasures.jointVelocities.setZero();
            controlBoardRemapperMeasures.jointAccelerations.setZero();
            controlBoardRemapperMeasures.jointTorques.setZero();
        }

        if (metaData.bridgeOptions.isPWMControlEnabled)
        {
            // firstly resize all the controlboard data buffers
            controlBoardRemapperMeasures.motorPWMs.resize(metaData.bridgeOptions.nrJoints);

            controlBoardRemapperMeasures.motorPWMsUnordered.resize(metaData.bridgeOptions.nrJoints);

            // zero buffers
            controlBoardRemapperMeasures.motorPWMs.setZero();
        }

        if (metaData.bridgeOptions.isMotorTemperatureSensorEnabled)
        {
            // firstly resize all the controlboard data buffers
            controlBoardRemapperMeasures.motorTemperatures.resize(metaData.bridgeOptions.nrJoints);

            controlBoardRemapperMeasures.motorTemperaturesUnordered.resize(
                metaData.bridgeOptions.nrJoints);

            // zero buffers
            controlBoardRemapperMeasures.motorTemperatures.setZero();
        }

        if (metaData.bridgeOptions.isMotorSensorsEnabled)
        {
            // firstly resize all the controlboard data buffers
            controlBoardRemapperMeasures.motorPositions.resize(metaData.bridgeOptions.nrJoints);
            controlBoardRemapperMeasures.motorVelocities.resize(metaData.bridgeOptions.nrJoints);
            controlBoardRemapperMeasures.motorAccelerations.resize(metaData.bridgeOptions.nrJoints);
            controlBoardRemapperMeasures.motorCurrents.resize(metaData.bridgeOptions.nrJoints);

            controlBoardRemapperMeasures.motorPositionsUnordered.resize(
                metaData.bridgeOptions.nrJoints);
            controlBoardRemapperMeasures.motorVelocitiesUnordered.resize(
                metaData.bridgeOptions.nrJoints);
            controlBoardRemapperMeasures.motorAccelerationsUnordered.resize(
                metaData.bridgeOptions.nrJoints);
            controlBoardRemapperMeasures.motorCurrentsUnordered.resize(
                metaData.bridgeOptions.nrJoints);

            // zero buffers
            controlBoardRemapperMeasures.motorPositions.setZero();
            controlBoardRemapperMeasures.motorVelocities.setZero();
            controlBoardRemapperMeasures.motorAccelerations.setZero();
            controlBoardRemapperMeasures.motorCurrents.setZero();
        }

        if (metaData.bridgeOptions.isPIDsEnabled)
        {
            // firstly resize all the controlboard data buffers
            controlBoardRemapperMeasures.pidPositions.resize(metaData.bridgeOptions.nrJoints);
            controlBoardRemapperMeasures.pidPositionErrors.resize(metaData.bridgeOptions.nrJoints);

            controlBoardRemapperMeasures.pidPositionsUnordered.resize(
                metaData.bridgeOptions.nrJoints);
            controlBoardRemapperMeasures.pidPositionErrorsUnordered.resize(
                metaData.bridgeOptions.nrJoints);

            // zero buffers
            controlBoardRemapperMeasures.pidPositions.setZero();
            controlBoardRemapperMeasures.pidPositionErrors.setZero();
        }
    }

    /**
     * check and match control board joints with the sensorbridge joints list
     */
    bool compareControlBoardJointsList()
    {
        constexpr auto logPrefix = "[YarpSensorBridge::Impl::compareControlBoardJointsList]";

        // get the names of all the joints available in the attached remote control board remapper
        std::vector<std::string> controlBoardJoints;
        std::vector<JointType> controlBoardJointTypes;
        int controlBoardDOFs;
        controlBoardRemapperInterfaces.encoders->getAxes(&controlBoardDOFs);

        std::string joint;
        yarp::dev::JointTypeEnum jType;
        for (int DOF = 0; DOF < controlBoardDOFs; DOF++)
        {
            controlBoardRemapperInterfaces.axis->getAxisName(DOF, joint);
            controlBoardRemapperInterfaces.axis->getJointType(DOF, jType);
            controlBoardJoints.push_back(joint);
            if (jType != yarp::dev::VOCAB_JOINTTYPE_REVOLUTE
                && jType != yarp::dev::VOCAB_JOINTTYPE_PRISMATIC)
            {
                log()->error("{} Joint type not supported. Only revolute and prismatic joints are "
                             "supported.",
                             logPrefix);
                return false;
            }

            controlBoardJointTypes.push_back(yarp::dev::VOCAB_JOINTTYPE_REVOLUTE == jType
                                                 ? JointType::REVOLUTE
                                                 : JointType::PRISMATIC);
        }

        // if the joint list is empty means that the user did not pass a "joints_list" while the
        // RemoteControlBoard is configured. In this case the joints list of the SensorBridge is the
        // same of the polydriver.
        if (metaData.sensorsList.jointsList.empty())
        {
            metaData.sensorsList.jointsList = controlBoardJoints;
            metaData.bridgeOptions.nrJoints = metaData.sensorsList.jointsList.size();
        }

        metaData.sensorsList.jointsTypeList = controlBoardJointTypes;

        this->allJointsArePrismatics
            = std::all_of(metaData.sensorsList.jointsTypeList.begin(),
                          metaData.sensorsList.jointsTypeList.end(), //
                          [](const auto& jointType) { return jointType == JointType::PRISMATIC; });
        this->allJointsAreRevolutes
            = std::all_of(metaData.sensorsList.jointsTypeList.begin(),
                          metaData.sensorsList.jointsTypeList.end(), //
                          [](const auto& jointType) { return jointType == JointType::REVOLUTE; });

        // reset the control board buffers
        this->resetControlBoardBuffers();

        // check if the joints in the desired joint list are available in the controlboard joints
        // list if available get the control board index at which the desired joint is available
        // this is required in order to remap the control board joints on to the desired joints
        for (int desiredDOF = 0; desiredDOF < metaData.sensorsList.jointsList.size(); desiredDOF++)
        {
            const auto& jointInSensorList = metaData.sensorsList.jointsList[desiredDOF];
            auto& remappedJointIndex
                = controlBoardRemapperMeasures.remappedJointIndices[desiredDOF];
            // find the joint named jointInSensorList into the controlBoardJoints vector
            const auto it = std::find_if(controlBoardJoints.begin(), //
                                         controlBoardJoints.end(), //
                                         [&](const auto& joint) { //
                                             return jointInSensorList == joint;
                                         });

            // check if the joint is found
            if (it != controlBoardJoints.end())
            {
                remappedJointIndex = std::distance(controlBoardJoints.begin(), it);
            } else
            {
                log()->error("{} Could not find a desired joint from the configuration in the "
                             "attached control board remapper.",
                             logPrefix);
                return false;
            }
        }

        controlBoardRemapperMeasures.remappedJointPermutationMatrix.indices()
            = controlBoardRemapperMeasures.remappedJointIndices;

        log()->info("{} Found all joints with the remapped index", logPrefix);
        for (int idx = 0; idx < controlBoardRemapperMeasures.remappedJointIndices.size(); idx++)
        {
            log()->info("{} Remapped Index: {}, Joint name: {}.",
                        logPrefix,
                        controlBoardRemapperMeasures.remappedJointIndices[idx],
                        controlBoardJoints[controlBoardRemapperMeasures.remappedJointIndices[idx]]);
        }
        return true;
    }

    bool attachAllSixAxisForceTorqueSensors(const yarp::dev::PolyDriverList& devList)
    {
        if (!metaData.bridgeOptions.isSixAxisForceTorqueSensorEnabled)
        {
            // do nothing
            return true;
        }

        constexpr auto logPrefix = "[YarpSensorBridge::Impl::attachAllSixAxisForceTorqueSensors]";

        std::vector<std::string> analogFTSensors;
        std::vector<std::string> masFTSensors;
        // attach MAS sensors
        if (attachRemappedMASSensor(devList, masForceTorquesInterface.sixAxisFTSensors))
        {
            // get MAS FT sensor names
            masFTSensors = getAllSensorsInMASInterface(masForceTorquesInterface.sixAxisFTSensors);
            auto allFTsInMetaData = metaData.sensorsList.sixAxisForceTorqueSensorsList;

            // compare with sensorList - those not available in the MAS interface list
            // are assumed to be analog FT sensors
            std::sort(masFTSensors.begin(), masFTSensors.end());
            std::sort(allFTsInMetaData.begin(), allFTsInMetaData.end());
            std::set_difference(allFTsInMetaData.begin(),
                                allFTsInMetaData.end(),
                                masFTSensors.begin(),
                                masFTSensors.end(),
                                std::back_inserter(analogFTSensors));
        } else
        {
            // if there are no MAS FT sensors then all the FT sensors in the configuration
            // are analog FT sensors
            analogFTSensors = metaData.sensorsList.sixAxisForceTorqueSensorsList;
        }

        if (!checkAttachedMASSensors(devList,
                                     masForceTorquesInterface.sixAxisFTSensors,
                                     masFTSensors))
        {
            log()->error("{} Could not find atleast one of the required MAS FT sensors.",
                         logPrefix);
            return false;
        }

        // check if a generic sensor has 12 channels implying it is a IMU sensor through a
        // GenericSensor Interfaces
        std::string_view interfaceType{"Analog Six Axis FT Interface"};
        if (!attachAllGenericOrAnalogSensors(devList,
                                             analogSixAxisFTSensorsInterface,
                                             nrChannelsInYARPAnalogSixAxisFTSensor,
                                             analogFTSensors,
                                             interfaceType))
        {
            return false;
        }

        return true;
    }

    bool attachAllTemperatureSensors(const yarp::dev::PolyDriverList& devList)
    {
        if (!metaData.bridgeOptions.isTemperatureSensorEnabled)
        {
            // do nothing
            return true;
        }

        std::string_view interfaceType{"ITemperatureSensors"};
        return attachAndCheckMASSensors(devList,
                                        masForceTorquesInterface.temperatureSensors,
                                        metaData.sensorsList.temperatureSensorsList,
                                        interfaceType);
    }

    /**
     * Attach to cartesian wrench interface
     */
    bool attachCartesianWrenchInterface(const yarp::dev::PolyDriverList& devList)
    {
        if (!metaData.bridgeOptions.isCartesianWrenchEnabled)
        {
            // do nothing
            return true;
        }

        if (metaData.bridgeOptions.isCartesianWrenchEnabled)
        {
            // check if a generic sensor has 6 channels implying it is a cartesian wrench sensor
            // through a GenericSensor Interfaces
            std::string_view interfaceType{"Cartesian Wrench Interface"};
            if (!attachAllGenericOrAnalogSensors(devList,
                                                 cartesianWrenchInterface,
                                                 nrChannelsInYARPGenericCartesianWrench,
                                                 metaData.sensorsList.cartesianWrenchesList,
                                                 interfaceType))
            {
                return false;
            }
        }
        return true;
    }

    /**
     * utility function
     */
    inline double deg2rad(double deg)
    {
        return deg * M_PI / 180.0;
    }

    /**
     * Read generic or analog sensor stream and update internal measurement buffer
     */
    template <typename SensorType>
    bool
    readAnalogOrGenericSensor(const std::string& sensorName,
                              const int& nrChannelsInYARPSensor,
                              std::unordered_map<std::string, SensorType*>& interfaceMap,
                              std::unordered_map<std::string, StampedYARPVector>& measurementMap,
                              bool checkForNan = false)
    {
        bool ok{true};
        constexpr auto logPrefix = "[YarpSensorBridge::Impl::readAnalogOrGenericSensor]";
        if (!checkSensor(interfaceMap, sensorName))
        {
            return false;
        }

        auto iter = interfaceMap.find(sensorName);
        auto interface = iter->second;

        yarp::sig::Vector sensorMeasure;
        sensorMeasure.resize(nrChannelsInYARPSensor);

        if constexpr (std::is_same_v<SensorType, yarp::dev::IAnalogSensor>)
        {
            auto retValue = interface->read(sensorMeasure);
            ok = ok && (retValue == yarp::dev::IAnalogSensor::AS_OK);
        } else if constexpr (std::is_same_v<SensorType, yarp::dev::IGenericSensor>)
        {
            ok = ok && interface->read(sensorMeasure);
        }

        if (!ok)
        {
            log()->error("{} Unable to read from {}, use previous measurement.",
                         logPrefix,
                         sensorName);
            return false;
        }

        if (checkForNan)
        {
            if (nanExistsInVec(yarp::eigen::toEigen(sensorMeasure), logPrefix, sensorName))
            {
                return false;
            }
        }

        // for IMU measurements convert angular velocity  measurements to radians per s
        // and convert orientation measurements to radians
        // enforcing the assumption that if dimensions of sensor channel is 12 then its an IMU
        // sensor
        if (nrChannelsInYARPSensor == nrChannelsInYARPGenericIMUSensor)
        {
            // See http://wiki.icub.org/wiki/Inertial_Sensor
            // 0, 1 and 2 indices correspond to orientation measurements
            sensorMeasure[0] = deg2rad(sensorMeasure[0]);
            sensorMeasure[1] = deg2rad(sensorMeasure[1]);
            sensorMeasure[2] = deg2rad(sensorMeasure[2]);
            // 6, 7 and 8 indices correspond to angular velocity measurements
            sensorMeasure[6] = deg2rad(sensorMeasure[6]);
            sensorMeasure[7] = deg2rad(sensorMeasure[7]);
            sensorMeasure[8] = deg2rad(sensorMeasure[8]);
        }

        measurementMap[sensorName].first = sensorMeasure;
        measurementMap[sensorName].second = yarp::os::Time::now();
        return true;
    }

    template <typename MASSensorType>
    bool readMASSensor(MASSensorType* interface,
                       const std::string& sensorName,
                       const std::size_t sensorIdx,
                       std::unordered_map<std::string, StampedYARPVector>& measurementMap,
                       bool checkForNan = false)
    {
        constexpr auto logPrefix = "[YarpSensorBridge::Impl::readMASSensor]";

        yarp::sig::Vector sensorMeasure;
        bool ok{false};
        double txTimeStamp;
        if constexpr (std::is_same_v<MASSensorType, yarp::dev::IThreeAxisGyroscopes>)
        {
            constexpr int dim{3};
            sensorMeasure.resize(dim);
            ok = interface->getThreeAxisGyroscopeMeasure(sensorIdx, sensorMeasure, txTimeStamp);

            // convert YARP measures from deg/s to rad/s
            for (std::size_t elemIdx = 0; elemIdx < dim; elemIdx++)
            {
                sensorMeasure(elemIdx) = deg2rad(sensorMeasure(elemIdx));
            }
        } else if constexpr (std::is_same_v<MASSensorType,
                                            yarp::dev::IThreeAxisLinearAccelerometers>)
        {
            constexpr int dim{3};
            sensorMeasure.resize(dim);
            ok = interface->getThreeAxisLinearAccelerometerMeasure(sensorIdx,
                                                                   sensorMeasure,
                                                                   txTimeStamp);
        } else if constexpr (std::is_same_v<MASSensorType, yarp::dev::IThreeAxisMagnetometers>)
        {
            constexpr int dim{3};
            sensorMeasure.resize(dim);
            ok = interface->getThreeAxisMagnetometerMeasure(sensorIdx, sensorMeasure, txTimeStamp);
        } else if constexpr (std::is_same_v<MASSensorType, yarp::dev::IOrientationSensors>)
        {
            constexpr int dim{3};
            sensorMeasure.resize(dim);
            ok = interface->getOrientationSensorMeasureAsRollPitchYaw(sensorIdx,
                                                                      sensorMeasure,
                                                                      txTimeStamp);
            // convert YARP measures from deg to rad
            for (std::size_t elemIdx = 0; elemIdx < dim; elemIdx++)
            {
                sensorMeasure(elemIdx) = deg2rad(sensorMeasure(elemIdx));
            }
        } else if constexpr (std::is_same_v<MASSensorType, yarp::dev::ISixAxisForceTorqueSensors>)
        {
            constexpr int dim{6};
            sensorMeasure.resize(dim);
            ok = interface->getSixAxisForceTorqueSensorMeasure(sensorIdx,
                                                               sensorMeasure,
                                                               txTimeStamp);
        } else if constexpr (std::is_same_v<MASSensorType, yarp::dev::ITemperatureSensors>)
        {
            ok = interface->getTemperatureSensorMeasure(sensorIdx, sensorMeasure, txTimeStamp);
        }

        if (!ok)
        {
            log()->error("{} Unable to read from {}, use previous measurement.",
                         logPrefix,
                         sensorName);
            return false;
        }

        if (checkForNan
            && nanExistsInVec(yarp::eigen::toEigen(sensorMeasure), logPrefix, sensorName))
        {
            return false;
        }

        measurementMap[sensorName].first = sensorMeasure;
        measurementMap[sensorName].second = BipedalLocomotion::clock().now().count();

        return true;
    }

    template <typename MASSensorType>
    bool readAllMASSensors(MASSensorType* interface,
                           std::unordered_map<std::string, std::size_t> sensIdxMap,
                           std::unordered_map<std::string, StampedYARPVector>& measurementMap,
                           std::vector<std::string>& failedSensorReads,
                           bool checkForNan = false)
    {
        constexpr auto logPrefix = "[YarpSensorBridge::Impl::readAllMASSensors]";
        bool allSensorsReadCorrectly{true};
        failedSensorReads.clear();
        for (const auto& [sensorName, sensorIdx] : sensIdxMap)
        {
            bool ok = readMASSensor(interface, sensorName, sensorIdx, measurementMap, checkForNan);
            if (!ok)
            {
                log()->error("{} Read MAS sensor failed for {}.", logPrefix, sensorName);
                failedSensorReads.emplace_back(sensorName);
            }
            allSensorsReadCorrectly = ok && allSensorsReadCorrectly;
        }

        return allSensorsReadCorrectly;
    }

    /**
     * Read control board remapper interfaces
     */
    bool readControlBoardInterface(bool checkForNan = false)
    {
        constexpr auto logPrefix = "[YarpSensorBridge::Impl::readControlBoardInterface]";

        bool ok{true};

        ok = ok && readAllJointSensors(checkForNan, streamJointAccelerations);
        ok = ok && readAllMotorSensors(checkForNan);
        ok = ok && readAllPIDs(checkForNan);
        ok = ok && readAllMotorPWMs(checkForNan);

        if (!ok)
        {
            log()->error("{} Unable to read from control board interfaces.", logPrefix);
            return false;
        }

        controlBoardRemapperMeasures.receivedTimeInSeconds = yarp::os::Time::now();

        return true;
    }

    /**
     * Read joint sensors measurements
     */
    bool readAllJointSensors(bool checkForNan = false, bool streamJointAccelerations = true)
    {
        constexpr auto logPrefix = "[YarpSensorBridge::Impl::readAllJointSensors]";

        bool ok{true};

        if (controlBoardRemapperInterfaces.encoders)
        {
            ok = ok
                 && controlBoardRemapperInterfaces.encoders->getEncoders(
                     controlBoardRemapperMeasures.jointPositionsUnordered.data());
            ok = ok
                 && controlBoardRemapperInterfaces.encoders->getEncoderSpeeds(
                     controlBoardRemapperMeasures.jointVelocitiesUnordered.data());

            if (streamJointAccelerations)
            {
                ok = ok
                     && controlBoardRemapperInterfaces.encoders->getEncoderAccelerations(
                         controlBoardRemapperMeasures.jointAccelerationsUnordered.data());
            }

            if (ok)
            {
                if (checkForNan)
                {
                    if (nanExistsInVec(controlBoardRemapperMeasures.jointPositionsUnordered,
                                       logPrefix,
                                       "encoders"))
                    {
                        return false;
                    }

                    if (nanExistsInVec(controlBoardRemapperMeasures.jointVelocitiesUnordered,
                                       logPrefix,
                                       "encoder speeds"))
                    {
                        return false;
                    }

                    if (streamJointAccelerations)
                    {
                        if (nanExistsInVec(controlBoardRemapperMeasures.jointAccelerationsUnordered,
                                           logPrefix,
                                           "encoder accelerations"))
                        {
                            return false;
                        }
                    }
                }

                // Create aliases for clarity
                auto& measures = controlBoardRemapperMeasures;
                const auto& perm = measures.remappedJointPermutationMatrix;

                // Helper lambda for uniform remapping and scaling.
                auto remapAndScale = [&](auto& target,
                                         const auto& source,
                                         std::optional<double> scale = std::nullopt) {
                    if (!scale.has_value())
                    {
                        target.noalias() = perm * source;
                        return;
                    }
                    target.noalias() = perm * source * scale.value();
                };

                // Helper lambda for mixed joints: remap then convert each revolute joint.
                auto remapAndConvertMixed = [&](auto& target, const auto& source) {
                    target.noalias() = perm * source;
                    for (int i = 0; i < target.size(); ++i)
                    {
                        // Convert only revolute joints from degrees to radians.
                        if (metaData.sensorsList.jointsTypeList[i] == JointType::REVOLUTE)
                        {
                            target(i) = deg2rad(target(i));
                        }
                    }
                };

                // Process joint measures based on joint type configuration.
                if (this->allJointsAreRevolutes)
                {
                    remapAndScale(measures.jointPositions,
                                  measures.jointPositionsUnordered,
                                  M_PI / 180.0);
                    remapAndScale(measures.jointVelocities,
                                  measures.jointVelocitiesUnordered,
                                  M_PI / 180.0);
                    if (streamJointAccelerations)
                    {
                        remapAndScale(measures.jointAccelerations,
                                      measures.jointAccelerationsUnordered,
                                      M_PI / 180.0);
                    }
                } else if (this->allJointsArePrismatics)
                {
                    remapAndScale(measures.jointPositions, measures.jointPositionsUnordered);
                    remapAndScale(measures.jointVelocities, measures.jointVelocitiesUnordered);
                    if (streamJointAccelerations)
                    {
                        remapAndScale(measures.jointAccelerations,
                                      measures.jointAccelerationsUnordered);
                    }
                } else // Mixed joints: some revolute and some prismatic.
                {
                    remapAndConvertMixed(measures.jointPositions, measures.jointPositionsUnordered);
                    remapAndConvertMixed(measures.jointVelocities,
                                         measures.jointVelocitiesUnordered);
                    if (streamJointAccelerations)
                    {
                        remapAndConvertMixed(measures.jointAccelerations,
                                             measures.jointAccelerationsUnordered);
                    }
                }
            } else
            {
                log()->error("{} Unable to read from IEncodersTimed interface, use previous "
                             "measurement.",
                             logPrefix);
            }
        }

        if (controlBoardRemapperInterfaces.torques)
        {
            ok = controlBoardRemapperInterfaces.torques->getTorques(
                controlBoardRemapperMeasures.jointTorquesUnordered.data());

            if (ok)
            {
                if (checkForNan)
                {
                    if (nanExistsInVec(controlBoardRemapperMeasures.jointTorquesUnordered,
                                       logPrefix,
                                       "WBDJointTorques"))
                    {
                        return false;
                    }
                }

                controlBoardRemapperMeasures.jointTorques.noalias()
                    = controlBoardRemapperMeasures.remappedJointPermutationMatrix
                      * controlBoardRemapperMeasures.jointTorquesUnordered;
            } else
            {
                log()->error("{} Unable to read from ITorqueControl interface, use previous "
                             "measurement.",
                             logPrefix);
            }
        }

        return true;
    }

    /**
     * Read motor sensors measurements
     */
    bool readAllMotorSensors(bool checkForNan = false)
    {
        constexpr auto logPrefix = "[YarpSensorBridge::Impl::readAllMotorSensors]";

        bool ok{true};

        if (controlBoardRemapperInterfaces.motorEncoders)
        {
            ok = ok
                 && controlBoardRemapperInterfaces.motorEncoders->getMotorEncoders(
                     controlBoardRemapperMeasures.motorPositionsUnordered.data());
            ok = ok
                 && controlBoardRemapperInterfaces.motorEncoders->getMotorEncoderSpeeds(
                     controlBoardRemapperMeasures.motorVelocitiesUnordered.data());
            ok = ok
                 && controlBoardRemapperInterfaces.motorEncoders->getMotorEncoderAccelerations(
                     controlBoardRemapperMeasures.motorAccelerationsUnordered.data());

            if (ok)
            {
                if (checkForNan)
                {
                    if (nanExistsInVec(controlBoardRemapperMeasures.motorPositionsUnordered,
                                       logPrefix,
                                       "MotorEncodersPos"))
                    {
                        return false;
                    }

                    if (nanExistsInVec(controlBoardRemapperMeasures.motorVelocitiesUnordered,
                                       logPrefix,
                                       "MotorEncodersVel"))
                    {
                        return false;
                    }

                    if (nanExistsInVec(controlBoardRemapperMeasures.motorAccelerationsUnordered,
                                       logPrefix,
                                       "MotorEncodersAcc"))
                    {
                        return false;
                    }
                }

                controlBoardRemapperMeasures.motorPositions.noalias()
                    = controlBoardRemapperMeasures.remappedJointPermutationMatrix
                      * controlBoardRemapperMeasures.motorPositionsUnordered * M_PI / 180;

                controlBoardRemapperMeasures.motorVelocities.noalias()
                    = controlBoardRemapperMeasures.remappedJointPermutationMatrix
                      * controlBoardRemapperMeasures.motorVelocitiesUnordered * M_PI / 180;

                controlBoardRemapperMeasures.motorAccelerations.noalias()
                     = controlBoardRemapperMeasures.remappedJointPermutationMatrix
                       * controlBoardRemapperMeasures.motorAccelerationsUnordered * M_PI / 180;

            } else
            {
                log()->error("{} Unable to read from IMotorEncoders interface, use previous "
                             "measurement.",
                             logPrefix);
            }
        }

        if (controlBoardRemapperInterfaces.currsensors)
        {
            ok = controlBoardRemapperInterfaces.currsensors->getCurrents(
                controlBoardRemapperMeasures.motorCurrentsUnordered.data());

            if (ok)
            {
                if (checkForNan)
                {
                    if (nanExistsInVec(controlBoardRemapperMeasures.motorCurrentsUnordered,
                                       logPrefix,
                                       "CurrentSensors"))
                    {
                        return false;
                    }
                }

                controlBoardRemapperMeasures.motorCurrents.noalias()
                    = controlBoardRemapperMeasures.remappedJointPermutationMatrix
                      * controlBoardRemapperMeasures.motorCurrentsUnordered;
            } else
            {
                log()->error("{} Unable to read from ICurrentControl interface, use previous "
                             "measurement.",
                             logPrefix);
            }
        }

        if (controlBoardRemapperInterfaces.motor)
        {
            ok = controlBoardRemapperInterfaces.motor->getTemperatures(
                controlBoardRemapperMeasures.motorTemperaturesUnordered.data());

            if (ok)
            {
                if (checkForNan)
                {
                    if (nanExistsInVec(controlBoardRemapperMeasures.motorTemperaturesUnordered,
                                       logPrefix,
                                       "MotorTemperatures"))
                    {
                        return false;
                    }
                }

                controlBoardRemapperMeasures.motorTemperatures.noalias()
                    = controlBoardRemapperMeasures.remappedJointPermutationMatrix
                      * controlBoardRemapperMeasures.motorTemperaturesUnordered;
            } else
            {
                log()->error("{} Unable to read from IMotorTemperature interface, use previous "
                             "measurement.",
                             logPrefix);
            }
        }

        return true;
    }

    /**
     * Read motor encoders measurements
     */
    bool readAllMotorPWMs(bool checkForNan = false)
    {
        if (!metaData.bridgeOptions.isPWMControlEnabled)
        {
            // do nothing
            return true;
        }

        constexpr auto logPrefix = "[YarpSensorBridge::Impl::readAllMotorPWMs]";
        bool ok{true};
        for (int j = 0; j < controlBoardRemapperMeasures.motorPWMsUnordered.size(); j++)
        {
            ok &= controlBoardRemapperInterfaces.amp
                      ->getPWM(j, &(controlBoardRemapperMeasures.motorPWMsUnordered[j]));
        }

        if (!ok)
        {
            log()->error("{} Unable to read from amplifiers, use previous measurement.", logPrefix);
            return false;
        }

        if (checkForNan)
        {
            if (nanExistsInVec(controlBoardRemapperMeasures.motorPWMsUnordered,
                               logPrefix,
                               "MotorPWMs"))
            {
                return false;
            }
        }

        controlBoardRemapperMeasures.motorPWMs.noalias()
            = controlBoardRemapperMeasures.remappedJointPermutationMatrix
              * controlBoardRemapperMeasures.motorPWMsUnordered;

        return true;
    }

    /**
     * Read pid references
     */
    bool readAllPIDs(bool checkForNan = false)
    {
        if (!metaData.bridgeOptions.isPIDsEnabled)
        {
            // do nothing
            return true;
        }

        constexpr auto logPrefix = "[YarpSensorBridge::Impl::readAllPIDs]";
        bool ok{true};
        ok = ok
             && controlBoardRemapperInterfaces.pids
                    ->getPidReferences(yarp::dev::VOCAB_PIDTYPE_POSITION,
                                       controlBoardRemapperMeasures.pidPositionsUnordered.data());
        ok = ok
             && controlBoardRemapperInterfaces.pids
                    ->getPidErrors(yarp::dev::VOCAB_PIDTYPE_POSITION,
                                   controlBoardRemapperMeasures.pidPositionErrorsUnordered.data());

        if (!ok)
        {
            log()->error("{} Unable to read from pids, use previous measurement.", logPrefix);
            return false;
        }

        if (checkForNan)
        {
            if (nanExistsInVec(controlBoardRemapperMeasures.pidPositionsUnordered,
                               logPrefix,
                               "PIDs"))
            {
                return false;
            }
            if (nanExistsInVec(controlBoardRemapperMeasures.pidPositionErrorsUnordered,
                               logPrefix,
                               "PIDs"))
            {
                return false;
            }
        }

        controlBoardRemapperMeasures.pidPositions.noalias()
            = controlBoardRemapperMeasures.remappedJointPermutationMatrix
              * controlBoardRemapperMeasures.pidPositionsUnordered * M_PI / 180;
        controlBoardRemapperMeasures.pidPositionErrors.noalias()
            = controlBoardRemapperMeasures.remappedJointPermutationMatrix
              * controlBoardRemapperMeasures.pidPositionErrorsUnordered * M_PI / 180;

        return true;
    }

    bool readAllIMUs(std::vector<std::string>& failedSensorReads)
    {
        if (!metaData.bridgeOptions.isIMUEnabled)
        {
            // do nothing
            return true;
        }

        constexpr auto logPrefix = "[YarpSensorBridge::Impl::readAllIMUs]";
        bool allIMUsReadCorrectly{true};
        failedSensorReads.clear();
        for (auto const& imu : analogIMUInterface)
        {
            const auto& imuName = imu.first;
            bool ok = readAnalogOrGenericSensor(imuName,
                                                nrChannelsInYARPGenericIMUSensor,
                                                analogIMUInterface,
                                                IMUMeasures,
                                                checkForNAN);
            if (!ok)
            {
                log()->error("{} Read IMU failed for {}.", logPrefix, imuName);
                failedSensorReads.emplace_back(imuName);
            }
            allIMUsReadCorrectly = ok && allIMUsReadCorrectly;
        }

        return allIMUsReadCorrectly;
    }

    bool readAllCartesianWrenches(std::vector<std::string>& failedSensorReads)
    {
        if (!metaData.bridgeOptions.isCartesianWrenchEnabled)
        {
            // do nothing
            return true;
        }

        constexpr auto logPrefix = "[YarpSensorBridge::Impl::readAllCartesianWrenches]";

        bool allWrenchesReadCorrectly{true};
        failedSensorReads.clear();
        for (auto const& wrenchUnit : cartesianWrenchInterface)
        {
            const auto& wrenchName = wrenchUnit.first;
            bool ok = readAnalogOrGenericSensor(wrenchName,
                                                nrChannelsInYARPGenericCartesianWrench,
                                                cartesianWrenchInterface,
                                                cartesianWrenchMeasures,
                                                checkForNAN);
            if (!ok)
            {
                log()->error("{} Read cartesian wrench failed for {}.", logPrefix, wrenchName);
                failedSensorReads.emplace_back(wrenchName);
            }
            allWrenchesReadCorrectly = ok && allWrenchesReadCorrectly;
        }

        return allWrenchesReadCorrectly;
    }

    bool readAllMASLinearAccelerometers(std::vector<std::string>& failedSensorReads)
    {
        if (!metaData.bridgeOptions.isLinearAccelerometerEnabled)
        {
            // do nothing
            return true;
        }

        return readAllMASSensors(masInertialsInterface.accelerometers,
                                 masSensorIndexMaps.accelerometer,
                                 accelerometerMeasures,
                                 failedSensorReads,
                                 checkForNAN);
    }

    bool readAllMASGyroscopes(std::vector<std::string>& failedSensorReads)
    {
        if (!metaData.bridgeOptions.isGyroscopeEnabled)
        {
            // do nothing
            return true;
        }

        return readAllMASSensors(masInertialsInterface.gyroscopes,
                                 masSensorIndexMaps.gyroscopes,
                                 gyroMeasures,
                                 failedSensorReads,
                                 checkForNAN);
    }

    bool readAllMASOrientationSensors(std::vector<std::string>& failedSensorReads)
    {
        if (!metaData.bridgeOptions.isOrientationSensorEnabled)
        {
            // do nothing
            return true;
        }

        return readAllMASSensors(masInertialsInterface.orientationSensors,
                                 masSensorIndexMaps.orientationSensors,
                                 orientationMeasures,
                                 failedSensorReads,
                                 checkForNAN);
    }

    bool readAllMASMagnetometers(std::vector<std::string>& failedSensorReads)
    {
        if (!metaData.bridgeOptions.isMagnetometerEnabled)
        {
            // do nothing
            return true;
        }

        return readAllMASSensors(masInertialsInterface.magnetometers,
                                 masSensorIndexMaps.magnetometers,
                                 magnetometerMeasures,
                                 failedSensorReads,
                                 checkForNAN);
    }

    bool readAllMASSixAxisForceTorqueSensors(std::vector<std::string>& failedSensorReads)
    {
        if (!metaData.bridgeOptions.isSixAxisForceTorqueSensorEnabled)
        {
            // do nothing
            return true;
        }

        return readAllMASSensors(masForceTorquesInterface.sixAxisFTSensors,
                                 masSensorIndexMaps.sixAxisFTSensors,
                                 FTMeasures,
                                 failedSensorReads,
                                 checkForNAN);
    }

    bool readAllAnalogSixAxisForceTorqueSensors(std::vector<std::string>& failedSensorReads)
    {
        if (!metaData.bridgeOptions.isSixAxisForceTorqueSensorEnabled)
        {
            // do nothing
            return true;
        }

        constexpr auto logPrefix = "[YarpSensorBridge::Impl::"
                                   "readAllAnalogSixAxisForceTorqueSensors]";

        bool allFTsReadCorrectly{true};
        failedSensorReads.clear();
        for (auto const& FTUnit : analogSixAxisFTSensorsInterface)
        {
            const auto& FTName = FTUnit.first;
            bool ok = readAnalogOrGenericSensor(FTName,
                                                nrChannelsInYARPAnalogSixAxisFTSensor,
                                                analogSixAxisFTSensorsInterface,
                                                FTMeasures,
                                                checkForNAN);
            if (!ok)
            {
                log()->error("{} Read FT sensor failed for {}.", logPrefix, FTName);
                failedSensorReads.emplace_back(FTName);
            }
            allFTsReadCorrectly = ok && allFTsReadCorrectly;
        }

        return allFTsReadCorrectly;
    }

    bool readAllMASTemperatures(std::vector<std::string>& failedSensorReads)
    {
        if (!metaData.bridgeOptions.isTemperatureSensorEnabled)
        {
            // do nothing
            return true;
        }

        return readAllMASSensors(masForceTorquesInterface.temperatureSensors,
                                 masSensorIndexMaps.temperatureSensors,
                                 temperatureMeasures,
                                 failedSensorReads,
                                 checkForNAN);
    }

    bool readAllSensors(std::vector<std::string>& failedReadAllSensors)
    {
        failedReadAllSensors.clear();
        std::vector<std::string> failedReads;

        if (readControlBoardInterface(checkForNAN))
        {
            failedReadAllSensors.emplace_back(std::string("RemoteControlBoardRemapper"));
        }

        if (!readAllIMUs(failedReads))
        {
            failedReadAllSensors.insert(failedReadAllSensors.end(),
                                        failedReads.begin(),
                                        failedReads.end());
        }

        if (!readAllCartesianWrenches(failedReads))
        {
            failedReadAllSensors.insert(failedReadAllSensors.end(),
                                        failedReads.begin(),
                                        failedReads.end());
        }

        if (!readAllMASLinearAccelerometers(failedReads))
        {
            failedReadAllSensors.insert(failedReadAllSensors.end(),
                                        failedReads.begin(),
                                        failedReads.end());
        }

        if (!readAllMASGyroscopes(failedReads))
        {
            failedReadAllSensors.insert(failedReadAllSensors.end(),
                                        failedReads.begin(),
                                        failedReads.end());
        }

        if (!readAllMASOrientationSensors(failedReads))
        {
            failedReadAllSensors.insert(failedReadAllSensors.end(),
                                        failedReads.begin(),
                                        failedReads.end());
        }

        if (!readAllMASMagnetometers(failedReads))
        {
            failedReadAllSensors.insert(failedReadAllSensors.end(),
                                        failedReads.begin(),
                                        failedReads.end());
        }

        if (!readAllMASSixAxisForceTorqueSensors(failedReads))
        {
            failedReadAllSensors.insert(failedReadAllSensors.end(),
                                        failedReads.begin(),
                                        failedReads.end());
        }

        if (!readAllAnalogSixAxisForceTorqueSensors(failedReads))
        {
            failedReadAllSensors.insert(failedReadAllSensors.end(),
                                        failedReads.begin(),
                                        failedReads.end());
        }

        if (!readAllMASTemperatures(failedReads))
        {
            failedReadAllSensors.insert(failedReadAllSensors.end(),
                                        failedReads.begin(),
                                        failedReads.end());
        }

        return true;
    }
};

} // namespace RobotInterface
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_ROBOT_INTERFACE_YARP_SENSOR_BRIDGE_IMPL_H
