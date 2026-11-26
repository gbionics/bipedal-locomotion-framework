/**
 * @file YarpRobotControl.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <algorithm>
#include <atomic>
#include <cmath>
#include <future>
#include <limits>
#include <mutex>
#include <numeric>
#include <string>
#include <string_view>
#include <thread>
#include <unordered_map>

#include <yarp/dev/IAxisInfo.h>
#include <yarp/dev/IControlLimits.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/ICurrentControl.h>
#include <yarp/dev/IEncodersTimed.h>
#include <yarp/dev/IInteractionMode.h>
#include <yarp/dev/IPWMControl.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/ITorqueControl.h>
#include <yarp/dev/IVelocityControl.h>
#include <yarp/dev/PolyDriver.h>

#include <BipedalLocomotion/RobotInterface/JointType.h>
#include <BipedalLocomotion/RobotInterface/YarpHelper.h>
#include <BipedalLocomotion/RobotInterface/YarpRobotControl.h>
#include <BipedalLocomotion/System/Clock.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

using namespace BipedalLocomotion::RobotInterface;

namespace
{
constexpr double RevolutePositionMinVelocity = 3.0; // deg/s
constexpr double PrismaticPositionMinVelocity = 10.0; // mm/s

std::string_view toString(BipedalLocomotion::RobotInterface::ControlMode mode)
{
    using BipedalLocomotion::RobotInterface::ControlMode;
    switch (mode)
    {
    case ControlMode::Position:
        return "Position";
    case ControlMode::PositionDirect:
        return "PositionDirect";
    case ControlMode::Velocity:
        return "Velocity";
    case ControlMode::Torque:
        return "Torque";
    case ControlMode::PWM:
        return "PWM";
    case ControlMode::Current:
        return "Current";
    case ControlMode::Idle:
        return "Idle";
    case ControlMode::ForceIdle:
        return "ForceIdle";
    case ControlMode::HardwareFault:
        return "HardwareFault";
    default:
        return "Unknown";
    }
}
} // namespace

struct YarpRobotControl::Impl
{
    /**
     * JointsControlValuesAndMode contains the information regarding the desired joint values and
     * the control mode for each joint.
     */
    struct JointsControlValuesAndMode
    {
        /** Set containing the indices related to each control mode*/
        std::unordered_map<RobotInterface::ControlMode, std::vector<int>> index;

        /** Set containing the desired joint values related to each control mode. In case of
         * Position, PositionDirect and Velocity the joint values are expressed in deg or deg/s */
        std::unordered_map<RobotInterface::ControlMode, Eigen::VectorXd> value;
    };

    std::shared_ptr<yarp::dev::PolyDriver> robotDevice; /**< PolyDriver */

    // YARP Interfaces exposed by the RemoteControlBoardRemapper
    yarp::dev::IEncodersTimed* encodersInterface{nullptr}; /**< Encorders interface. */
    yarp::dev::IPositionDirect* positionDirectInterface{nullptr}; /**< Direct position control
                                                                     interface. */
    yarp::dev::IPositionControl* positionInterface{nullptr}; /**< Position control interface. */
    yarp::dev::IVelocityControl* velocityInterface{nullptr}; /**< Velocity control interface. */
    yarp::dev::ITorqueControl* torqueInterface{nullptr}; /**< Torque control interface. */
    yarp::dev::IPWMControl* pwmInterface{nullptr}; /**< PWM control interface. */
    yarp::dev::ICurrentControl* currentInterface{nullptr}; /**< Current control interface. */
    yarp::dev::IControlMode* controlModeInterface{nullptr}; /**< Control mode interface. */
    yarp::dev::IInteractionMode* interactionModeInterface{nullptr};
    yarp::dev::IAxisInfo* axisInfoInterface{nullptr}; /**< Axis info interface. */
    yarp::dev::IControlLimits* controlLimitsInterface{nullptr}; /**< Control limits interface. */

    std::vector<JointType> jointsTypeList; /**< list of joint types */
    bool allJointsAreRevolutes{false}; /**< flag to indicate if all the joints are revolute */
    bool allJointsArePrismatics{false}; /**< flag to indicate if all the joints are prismatic */


    std::size_t actuatedDOFs{0}; /**< Number of the actuated DoFs. */

    Eigen::VectorXd positionFeedback; /**< Current joint position [rad]. */

    /** Control mode requested for each joint, initialized with the control modes read in
     * setDriver(). It is the desired control mode and not the actual one: it is updated also if
     * the device does not accept the request (e.g. a joint in hardware fault). Use
     * YarpSensorBridge to read the actual control modes. */
    std::vector<RobotInterface::ControlMode> desiredControlModes;
    std::vector<yarp::conf::vocab32_t> controlModesYarp; /**< Vector containing the map between the
                                                             joint and the current yarp control mode
                                                          */

    std::vector<std::string> axesName; /**< List containing the mapping between the joints index and
                                          the their name */

    JointsControlValuesAndMode desiredJointValuesAndMode; /**< Struct containing the information
                                                             regarding the desired joint value and
                                                             the control mode */
    std::vector<double> positionControlRefSpeeds; /**< Vector containing the ref speed in
                                                     deg/seconds for the position control joints. */

    std::chrono::nanoseconds positioningDuration{0}; /**< Duration of the trajectory generated when
                                                        the joint is controlled in position mode */
    std::chrono::nanoseconds startPositionControlInstant{0}; /**< Initial time instant of the
                                                                  trajectory generated when the
                                                                  joint is controlled in position
                                                                  mode */
    double positioningTolerance{0.0}; /**< Max Admissible error for position control joint [rad] */
    double positionDirectMaxAdmissibleError{0.0}; /**< Max admissible error for position direct
                                                     control joint [rad] */

    std::size_t maxReadingAttempts{100}; /**< Max number of attempts used for reading from the yarp
                                            interfaces. */
    std::size_t readingTimeout{500}; /**< Timeout used while reading from the yarp interfaces in
                                        microseconds. */

    /** Protects the scratch buffers below; acquired by the public subset-control API before any
     *  call to setReferencesSubset(), so that method itself does not need to lock. */
    std::mutex subsetMutex;
    Eigen::VectorXd subsetCurrentPos; /**< Scratch: current joint positions for subset position mode. */
    std::vector<double> subsetRefSpeeds; /**< Scratch: reference speeds for subset position mode. */
    Eigen::VectorXd subsetRefs; /**< Scratch: converted reference values for subset calls. */
    std::vector<int> subsetJointIndices; /**< Scratch: joint indices for name-based subset calls. */

    /** Time of the last warning about the references not accepted by the device [ns]. */
    std::atomic<std::chrono::nanoseconds::rep> lastReferencesWarning{
        std::numeric_limits<std::chrono::nanoseconds::rep>::min()};

    bool getControlModes()
    {
        constexpr auto errorPrefix = "[YarpRobotControl::Impl::getControlModes]";

        if (this->controlModeInterface == nullptr)
        {
            log()->error("{} The control mode I/F is not ready.", errorPrefix);
            return false;
        }

        // try to read the control mode
        std::size_t counter = 0;
        while (!this->controlModeInterface->getControlModes(this->controlModesYarp.data()))
        {
            if (++counter == this->maxReadingAttempts)
            {
                log()->error("{} Error while reading the control mode.", errorPrefix);
                return false;
            }

            // Sleep for some while
            std::this_thread::sleep_for(std::chrono::microseconds(this->readingTimeout));
        }

        for (std::size_t i = 0; i < this->actuatedDOFs; i++)
        {
            this->desiredControlModes[i] = fromYarpControlMode(this->controlModesYarp[i]);
        }

        return true;
    }

    /**
     * Return true if the control mode can be set to a joint. HardwareFault can only be read.
     */
    static bool canBeSet(const RobotInterface::ControlMode& mode)
    {
        return mode != RobotInterface::ControlMode::HardwareFault;
    }

    /**
     * Rebuild the indices of the joints associated to each control mode from desiredControlModes.
     */
    void updateControlModeIndices()
    {
        for (auto& [mode, indices] : this->desiredJointValuesAndMode.index)
        {
            indices.clear();
        }

        for (std::size_t i = 0; i < this->actuatedDOFs; i++)
        {
            this->desiredJointValuesAndMode.index[this->desiredControlModes[i]].push_back(i);
        }

        for (const auto& [mode, indices] : this->desiredJointValuesAndMode.index)
        {
            this->desiredJointValuesAndMode.value[mode].resize(indices.size());
        }

        this->positionControlRefSpeeds.resize(
            this->desiredJointValuesAndMode.index[RobotInterface::ControlMode::Position].size());
    }

    /**
     * Warn that the device did not accept the references of a control mode. The warning is
     * throttled since it can be raised at every control cycle, e.g. while a joint is in hardware
     * fault.
     */
    void warnReferencesNotAccepted(std::string_view errorPrefix, RobotInterface::ControlMode mode)
    {
        constexpr auto period = std::chrono::nanoseconds(std::chrono::seconds(1)).count();
        const auto now = BipedalLocomotion::clock().now().count();
        auto last = this->lastReferencesWarning.load();
        if ((last != std::numeric_limits<std::chrono::nanoseconds::rep>::min()
             && now - last < period)
            || !this->lastReferencesWarning.compare_exchange_strong(last, now))
        {
            return;
        }
        log()->warn("{} The device did not accept the references of some of the joints in "
                    "control mode {}, e.g. because they are in hardware fault. The references of "
                    "the other joints and of the other control modes are sent anyway.",
                    errorPrefix,
                    toString(mode));
    }

    bool setControlModes(const std::vector<RobotInterface::ControlMode>& controlModes)
    {
        constexpr auto errorPrefix = "[YarpRobotControl::Impl::setControlModes]";

        if (controlModes.size() != this->actuatedDOFs)
        {
            log()->error("{} The size of the control modes ({}) does not match the number of "
                         "actuated DoFs ({}).",
                         errorPrefix,
                         controlModes.size(),
                         this->actuatedDOFs);
            return false;
        }

        std::vector<int> jointIndices(this->actuatedDOFs);
        std::iota(jointIndices.begin(), jointIndices.end(), 0);
        return this->setControlModesSubset(controlModes, jointIndices);
    }

    bool setControlModesSubset(const std::vector<RobotInterface::ControlMode>& controlModes,
                               const std::vector<int>& jointIndices)
    {
        constexpr auto errorPrefix = "[YarpRobotControl::Impl::setControlModes]";

        if (this->controlModeInterface == nullptr)
        {
            log()->error("{} The control mode I/F is not ready.", errorPrefix);
            return false;
        }

        if (controlModes.size() != jointIndices.size())
        {
            log()->error("{} The size of the control modes ({}) does not match the size of the "
                         "joint indices ({}).",
                         errorPrefix,
                         controlModes.size(),
                         jointIndices.size());
            return false;
        }

        for (std::size_t i = 0; i < jointIndices.size(); i++)
        {
            const int idx = jointIndices[i];
            if (idx < 0 || static_cast<std::size_t>(idx) >= this->actuatedDOFs)
            {
                log()->error("{} Joint index {} is out of range [0, {}).",
                             errorPrefix,
                             idx,
                             this->actuatedDOFs);
                return false;
            }

            if (!canBeSet(controlModes[i]))
            {
                log()->error("{} The HardwareFault control mode cannot be set (joint '{}'). Use "
                             "ForceIdle to acknowledge a hardware fault.",
                             errorPrefix,
                             this->axesName[idx]);
                return false;
            }
        }

        std::vector<yarp::conf::vocab32_t> modes(jointIndices.size());
        for (std::size_t i = 0; i < jointIndices.size(); i++)
        {
            modes[i] = toYarpControlMode(controlModes[i]);
        }

        // The request is sent for all the joints: the device sets the joints that accept the
        // control mode also when some others do not (e.g. a joint in hardware fault only accepts
        // ForceIdle).
        bool allJoints = jointIndices.size() == this->actuatedDOFs;
        for (std::size_t i = 0; allJoints && i < jointIndices.size(); i++)
        {
            allJoints = jointIndices[i] == static_cast<int>(i);
        }
        const bool ok = allJoints
                            ? this->controlModeInterface->setControlModes(modes.data())
                            : this->controlModeInterface->setControlModes(static_cast<int>(
                                                                              jointIndices.size()),
                                                                          jointIndices.data(),
                                                                          modes.data());

        // The desired control modes are stored also if the device did not accept some of them,
        // so that the references of the other joints keep being accepted.
        for (std::size_t i = 0; i < jointIndices.size(); i++)
        {
            this->desiredControlModes[jointIndices[i]] = controlModes[i];
        }
        this->updateControlModeIndices();

        if (!ok)
        {
            log()->warn("{} The device did not accept the control mode of some of the joints, "
                        "e.g. because they are in hardware fault and only accept ForceIdle. The "
                        "other joints are set.",
                        errorPrefix);
            return false;
        }

        return true;
    }

    bool setInteractionModesSubset(
        const std::vector<RobotInterface::InteractionMode>& interactionModes,
        const std::vector<int>& jointIndices)
    {
        constexpr auto errorPrefix = "[YarpRobotControl::setInteractionMode]";
        if (this->interactionModeInterface == nullptr)
        {
            log()->error("{} The interaction mode interface is not ready.", errorPrefix);
            return false;
        }
        if (interactionModes.size() != jointIndices.size())
        {
            log()->error("{} The number of interaction modes ({}) does not match the number of "
                         "joint indices ({}).",
                         errorPrefix, interactionModes.size(), jointIndices.size());
            return false;
        }

        std::vector<yarp::dev::InteractionModeEnum> modes;
        modes.reserve(interactionModes.size());
        for (std::size_t i = 0; i < jointIndices.size(); ++i)
        {
            const int idx = jointIndices[i];
            if (idx < 0 || static_cast<std::size_t>(idx) >= this->actuatedDOFs)
            {
                log()->error("{} Joint index {} is out of range [0, {}).",
                             errorPrefix, idx, this->actuatedDOFs);
                return false;
            }
            const auto mode = toYarpInteractionMode(interactionModes[i]);
            if (mode == yarp::dev::VOCAB_IM_UNKNOWN)
            {
                log()->error("{} Unknown interaction mode cannot be set for joint '{}'.",
                             errorPrefix, this->axesName[idx]);
                return false;
            }
            modes.push_back(mode);
        }

        if (jointIndices.empty())
        {
            return true;
        }

        bool allJoints = jointIndices.size() == this->actuatedDOFs;
        for (std::size_t i = 0; allJoints && i < jointIndices.size(); ++i)
        {
            allJoints = jointIndices[i] == static_cast<int>(i);
        }
        auto indices = jointIndices;
        const bool ok = allJoints
                            ? this->interactionModeInterface->setInteractionModes(modes.data())
                            : this->interactionModeInterface->setInteractionModes(
                                  static_cast<int>(indices.size()), indices.data(), modes.data());
        if (!ok)
        {
            log()->error("{} The device failed to set the interaction modes; some joints may "
                         "have been set.", errorPrefix);
        }
        return ok;
    }

    bool getJointPos()
    {
        constexpr auto errorPrefix = "[YarpRobotControl::Impl::getJointPos]";

        if (this->encodersInterface == nullptr)
        {
            log()->error("{} The control mode I/F is not ready.", errorPrefix);
            return false;
        }

        // try to read the joint encoders
        std::size_t counter = 0;
        while (!this->encodersInterface->getEncoders(this->positionFeedback.data()))
        {
            if (++counter == this->maxReadingAttempts)
            {
                log()->error("{} Error while reading the encoders.", errorPrefix);
                return false;
            }

            // Sleep for some while
            std::this_thread::sleep_for(std::chrono::microseconds(this->readingTimeout));
        }

        // Check if all joints are revolute
        if (this->allJointsAreRevolutes)
        {
            // convert the joint position in radians
            this->positionFeedback *= M_PI / 180.0;
        }
        else if (!this->allJointsArePrismatics)
        {
            for (std::size_t i = 0; i < this->positionFeedback.size(); i++)
            {
                if (this->jointsTypeList[i] == JointType::REVOLUTE)
                {
                    this->positionFeedback[i] *= M_PI / 180.0;
                }
            }
        }

        return true;
    }

    bool setDriver(std::shared_ptr<yarp::dev::PolyDriver> robotDevice)
    {
        constexpr auto errorPrefix = "[YarpRobotControl::Impl::setDriver]";

        this->interactionModeInterface = nullptr;
        if (robotDevice == nullptr)
        {
            log()->error("{} The robotDevice is pointing to an non initialized memory.",
                         errorPrefix);
            return false;
        }

        // obtain the interfaces
        if (!robotDevice->view(encodersInterface) || encodersInterface == nullptr)
        {
            log()->error("{} Cannot load the IEncodersTimed interface.", errorPrefix);
            return false;
        }

        if (!robotDevice->view(positionInterface) || positionInterface == nullptr)
        {
            log()->error("{} Cannot load the IPositionControl interface.", errorPrefix);
            return false;
        }

        if (!robotDevice->view(positionDirectInterface) || positionDirectInterface == nullptr)
        {
            log()->error("{} Cannot load the IPositionDirect interface.", errorPrefix);
            return false;
        }

        if (!robotDevice->view(velocityInterface) || velocityInterface == nullptr)
        {
            log()->error("{} Cannot load the IVelocityControl interface.", errorPrefix);
            return false;
        }

        if (!robotDevice->view(torqueInterface) || torqueInterface == nullptr)
        {
            log()->error("{} Cannot load the ITorqueControl interface.", errorPrefix);
            return false;
        }

        if (!robotDevice->view(pwmInterface) || pwmInterface == nullptr)
        {
            log()->error("{} Cannot load the IPWMControl interface.", errorPrefix);
            return false;
        }

        if (!robotDevice->view(currentInterface) || currentInterface == nullptr)
        {
            log()->error("{} Cannot load the ICurrentControl interface.", errorPrefix);
            return false;
        }

        if (!robotDevice->view(controlModeInterface) || controlModeInterface == nullptr)
        {
            log()->error("{} Cannot load the IControlMode interface.", errorPrefix);
            return false;
        }

        yarp::dev::IInteractionMode* interactionMode{nullptr};
        if (!robotDevice->view(interactionMode) || interactionMode == nullptr)
        {
            interactionMode = nullptr;
            log()->warn("{} IInteractionMode is unavailable; interaction modes cannot be set.",
                        errorPrefix);
        }

        if (!robotDevice->view(axisInfoInterface) || axisInfoInterface == nullptr)
        {
            log()->error("{} Cannot load the IAxisInfo interface.", errorPrefix);
            return false;
        }

        if (!robotDevice->view(controlLimitsInterface) || controlLimitsInterface == nullptr)
        {
            log()->error("{} Cannot load the IControlMode interface.", errorPrefix);
            return false;
        }

        // get the number of degree of freedom
        int dofs = 0;
        if (!encodersInterface->getAxes(&dofs))
        {
            log()->error("{} Cannot get the actuated DoFs.", errorPrefix);
            return false;
        }
        this->actuatedDOFs = dofs;

        // resize vector
        this->positionFeedback.resize(this->actuatedDOFs);
        this->desiredControlModes.resize(this->actuatedDOFs);
        this->controlModesYarp.resize(this->actuatedDOFs);
        this->axesName.resize(this->actuatedDOFs);

        // reserve vector for subset control
        this->subsetCurrentPos.resize(this->actuatedDOFs);
        this->subsetRefSpeeds.reserve(this->actuatedDOFs);
        this->subsetRefs.resize(this->actuatedDOFs);
        this->subsetJointIndices.reserve(this->actuatedDOFs);

        // populate the axesName vector
        for (int i = 0; i < this->actuatedDOFs; i++)
        {
            this->axisInfoInterface->getAxisName(i, this->axesName[i]);
        }

        // store the polydriver
        this->robotDevice = robotDevice;

        if (!this->getControlModes())
        {
            log()->error("{} Unable to get the control modes.", errorPrefix);
            return false;
        }

        // clear all the stored control modes
        this->desiredJointValuesAndMode.index[RobotInterface::ControlMode::Position].clear();
        this->desiredJointValuesAndMode.index[RobotInterface::ControlMode::PositionDirect].clear();
        this->desiredJointValuesAndMode.index[RobotInterface::ControlMode::Velocity].clear();
        this->desiredJointValuesAndMode.index[RobotInterface::ControlMode::Torque].clear();
        this->desiredJointValuesAndMode.index[RobotInterface::ControlMode::PWM].clear();
        this->desiredJointValuesAndMode.index[RobotInterface::ControlMode::Current].clear();

        // store the joint associated to a specific control mode
        for (std::size_t i = 0; i < this->actuatedDOFs; i++)
        {
            this->desiredJointValuesAndMode.index[this->desiredControlModes[i]].push_back(i);
        }

        // resize the desired joint value vector associated to each control mode
        for (const auto& [mode, indices] : this->desiredJointValuesAndMode.index)
        {
            this->desiredJointValuesAndMode.value[mode].resize(indices.size());
        }

        // resize the reference speed for the position control mode
        // The size of the vector is equal to the size of the joints in position control
        this->positionControlRefSpeeds.resize(
            this->desiredJointValuesAndMode.index[RobotInterface::ControlMode::Position].size());

        // get the names of all the joints available in the attached remote control board remapper
        std::vector<std::string> controlBoardJoints;
        std::vector<JointType> controlBoardJointTypes;
        yarp::dev::JointTypeEnum jType;
        std::string joint;

        for (int DOF = 0; DOF < this->axesName.size(); DOF++)
        {
            this->axisInfoInterface->getAxisName(DOF, joint);
            this->axisInfoInterface->getJointType(DOF, jType);
            controlBoardJoints.push_back(joint);
            if (jType != yarp::dev::VOCAB_JOINTTYPE_REVOLUTE
                && jType != yarp::dev::VOCAB_JOINTTYPE_PRISMATIC)
            {
                log()->error("{} Joint type not supported. Only revolute and prismatic joints are "
                             "supported.",
                             errorPrefix);
                return false;
            }

            controlBoardJointTypes.push_back(yarp::dev::VOCAB_JOINTTYPE_REVOLUTE == jType
                                                 ? JointType::REVOLUTE
                                                 : JointType::PRISMATIC);
        }

        this->jointsTypeList = controlBoardJointTypes;

        this->allJointsArePrismatics
            = std::all_of(controlBoardJointTypes.begin(),
                          controlBoardJointTypes.end(), //
                          [](const auto& jointType) { return jointType == JointType::PRISMATIC; });
        this->allJointsAreRevolutes
            = std::all_of(controlBoardJointTypes.begin(),
                          controlBoardJointTypes.end(), //
                          [](const auto& jointType) { return jointType == JointType::REVOLUTE; });

        this->interactionModeInterface = interactionMode;
        return true;
    }

    /**
     * Return the the worst position error for the joint controlled in position direct.
     * The first value is the index while the second is the error in radians.
     */
    std::pair<int, double>
    getWorstPositionDirectError(Eigen::Ref<const Eigen::VectorXd> desiredJointValues,
                                Eigen::Ref<const Eigen::VectorXd> jointPositions) const
    {
        // clear the std::pair
        std::pair<int, double> worstError{0, 0.0};

        for (int i = 0; i < this->actuatedDOFs; i++)
        {
            // we are interested only if the joint is in PositionDirect
            if (this->desiredControlModes[i] == RobotInterface::ControlMode::PositionDirect)
            {
                const double jointError = std::abs(jointPositions[i] - desiredJointValues[i]);

                if (jointError > worstError.second)
                {
                    worstError.first = i;
                    worstError.second = jointError;
                }
            }
        }

        return worstError;
    }

    std::function<bool(const int, const int*, const double*)>
    control(const RobotInterface::ControlMode& mode)
    {
        assert(mode != RobotInterface::ControlMode::Unknown);
        assert(mode != RobotInterface::ControlMode::Idle);

        switch (mode)
        {
        case RobotInterface::ControlMode::Position:
            return [&](const int nJoints, const int* joints, const double* refs) -> bool {
                return this->positionInterface->positionMove(nJoints, joints, refs);
            };

        case RobotInterface::ControlMode::PositionDirect:
            return [&](const int nJoints, const int* joints, const double* refs) -> bool {
                return this->positionDirectInterface->setPositions(nJoints, joints, refs);
            };

        case RobotInterface::ControlMode::Velocity:
            return [&](const int nJoints, const int* joints, const double* refs) -> bool {
                return this->velocityInterface->velocityMove(nJoints, joints, refs);
            };

        case RobotInterface::ControlMode::Torque:
            return [&](const int nJoints, const int* joints, const double* refs) -> bool {
                return this->torqueInterface->setRefTorques(nJoints, joints, refs);
            };

        case RobotInterface::ControlMode::Current:
            return [&](const int nJoints, const int* joints, const double* refs) -> bool {
                return this->currentInterface->setRefCurrents(nJoints, joints, refs);
            };

        case RobotInterface::ControlMode::PWM:
            return [&](const int nJoints, const int* joints, const double* refs) -> bool {
                bool ok = true;
                for (int i = 0; i < nJoints; i++)
                {
                    ok = ok && this->pwmInterface->setRefDutyCycle(joints[i], refs[i]);
                }
                return ok;
            };

        default:
            return nullptr;
        }

        return nullptr;
    }

    bool setReferences(Eigen::Ref<const Eigen::VectorXd> jointValues,
                       std::optional<Eigen::Ref<const Eigen::VectorXd>> currentJointValues)
    {
        constexpr auto errorPrefix = "[YarpRobotControl::Impl::setReferences]";

        // the following checks are performed only if the robot is controlled in position direct or
        // in position mode
        if (!this->desiredJointValuesAndMode.index[RobotInterface::ControlMode::Position].empty()
            || !this->desiredJointValuesAndMode.index[RobotInterface::ControlMode::PositionDirect]
                    .empty())
        {

            if (jointValues.size() != this->actuatedDOFs)
            {
                log()->error("{} The size of the joint values is different from the number of "
                             "actuated DoFs. Expected size: {}. Received size: {}.",
                             errorPrefix,
                             this->actuatedDOFs,
                             jointValues.size());
                return false;
            }

            if (currentJointValues.has_value())
            {
                if (currentJointValues->size() != this->actuatedDOFs)
                {
                    log()->error("{} The size of the current joint values is different from the "
                                 "number of actuated DoFs. Expected size: {}. Received size: {}.",
                                 errorPrefix,
                                 this->actuatedDOFs,
                                 currentJointValues->size());
                    return false;
                }
                this->positionFeedback = currentJointValues.value();
            } else
            {
                if (!this->getJointPos())
                {
                    log()->error("{} Unable to get the joint position.", errorPrefix);
                    return false;
                }
            }

            const auto worstError = this->getWorstPositionDirectError(jointValues, //
                                                                      this->positionFeedback);

            if (worstError.second > this->positionDirectMaxAdmissibleError)
            {
                if (this->jointsTypeList[worstError.first] == JointType::REVOLUTE)
                {
                    log()->error("{} The worst error between the current and the desired position of "
                        "the "
                        "joint named '{}' is greater than {} deg. Error = {} deg.",
                        errorPrefix,
                        this->axesName[worstError.first],
                        180 / M_PI * this->positionDirectMaxAdmissibleError,
                        180 / M_PI * worstError.second);
                }
                else
                {
                    log()->error("{} The worst error between the current and the desired position of "
                        "the "
                        "joint named '{}' is greater than {} deg. Error = {} deg.",
                        errorPrefix,
                        this->axesName[worstError.first],
                        this->positionDirectMaxAdmissibleError,
                        worstError.second);
                }

                return false;
            }
        }

        bool allAccepted = true;
        for (const auto& [mode, indices] : this->desiredJointValuesAndMode.index)
        {
            // if indices vector is empty no joint is controlled with this specific control mode
            if (indices.empty())
            {
                continue;
            }

            if (mode == RobotInterface::ControlMode::Idle
                || mode == RobotInterface::ControlMode::ForceIdle)
            {
                continue;
            }

            else if (mode == RobotInterface::ControlMode::HardwareFault)
            {
                // the joints were in hardware fault when the driver was set and no control mode
                // was requested since then: like the idle ones, they do not receive references
                continue;

            } else if (mode == RobotInterface::ControlMode::Unknown)
            {
                std::string joints = "";
                for (const auto& index : indices)
                {
                    joints += " '" + this->axesName[index] + "'";
                }

                log()->error("{} The following joints does not have a specified control "
                             "mode:{}. Please set a feasible control mode.",
                             errorPrefix,
                             joints);
                return false;

            } else if (mode == RobotInterface::ControlMode::Position)
            {
                const double positioningDurationSeconds
                    = std::chrono::duration<double>(this->positioningDuration).count();
                for (int i = 0; i < indices.size(); i++)
                {
                    const auto jointError
                        = std::abs(jointValues[indices[i]] - this->positionFeedback[indices[i]]);

                    double scaling;
                    double minVelocity;
                    if (this->jointsTypeList[i] == JointType::REVOLUTE)
                    {
                        scaling = 180 / M_PI;
                        minVelocity = RevolutePositionMinVelocity;
                    }
                    else {
                        scaling = 1;
                        minVelocity = PrismaticPositionMinVelocity;
                    }
                    this->positionControlRefSpeeds[i]
                        = std::max(minVelocity,
                                   scaling * (jointError / positioningDurationSeconds));
                }

                if (!this->positionInterface->setRefSpeeds(indices.size(),
                                                           indices.data(),
                                                           this->positionControlRefSpeeds.data()))
                {
                    log()->error("{} Unable to set the reference speed for the position control "
                                 "joints.",
                                 errorPrefix);
                    return false;
                }

                this->startPositionControlInstant = BipedalLocomotion::clock().now();
            }

            // Define an helper to check if the control mode is torque, pwm or current
            // This is in general valide for each kind of joint
            auto isScalingNotRequired = [](RobotInterface::ControlMode mode) -> bool {
                return mode == RobotInterface::ControlMode::Torque
                       || mode == RobotInterface::ControlMode::PWM
                       || mode == RobotInterface::ControlMode::Current;
            };

            for (size_t i = 0; i < indices.size(); ++i) {
                const auto jointIdx = indices[i];
                const auto jt = this->jointsTypeList[i];
                double scaling = 1.0;

                // If the joint is prismatic, the scaling factor is 1. If the joint is revolute,
                // and the control mode is not torque, pwm or current, then the scaling factor
                // is 180/M_PI to convert from radians to degrees. Otherwise, the scaling
                // factor is 1.
                if (jt == JointType::REVOLUTE && !isScalingNotRequired(mode))
                {
                    scaling = 180.0 / M_PI;
                }

                this->desiredJointValuesAndMode.value[mode][i]
                    = scaling * jointValues[jointIdx];
            }

            if (!this->control(mode)(indices.size(),
                                     indices.data(),
                                     this->desiredJointValuesAndMode.value[mode].data()))

            {
                // the device applied the references to the joints that accept them: the other
                // control modes are still commanded
                this->warnReferencesNotAccepted(errorPrefix, mode);
                allAccepted = false;
            }
        }
        return allAccepted;
    }

    bool checkControlMode(const std::vector<RobotInterface::ControlMode>& controlModes) const
    {
        return controlModes == this->desiredControlModes;
    }

    bool checkControlMode(const RobotInterface::ControlMode& mode) const
    {
        return std::all_of(this->desiredControlModes.begin(),
                           this->desiredControlModes.end(),
                           [&mode](const auto& m) { return m == mode; });
    }

    bool setReferencesSubset(Eigen::Ref<const Eigen::VectorXd> jointValues,
                             const std::vector<int>& jointIndices,
                             const RobotInterface::ControlMode& controlMode,
                             std::optional<Eigen::Ref<const Eigen::VectorXd>> currentJointValues)
    {
        constexpr auto errorPrefix = "[YarpRobotControl::Impl::setReferencesSubset]";

        const int n = static_cast<int>(jointIndices.size());

        if (jointValues.size() != n)
        {
            log()->error("{} desiredJointValues size ({}) does not match jointIndices size ({}).",
                         errorPrefix,
                         jointValues.size(),
                         n);
            return false;
        }

        if (controlMode == RobotInterface::ControlMode::Unknown
            || controlMode == RobotInterface::ControlMode::Idle
            || controlMode == RobotInterface::ControlMode::ForceIdle
            || controlMode == RobotInterface::ControlMode::HardwareFault)
        {
            log()->error("{} Cannot set references for Idle, ForceIdle, HardwareFault or Unknown "
                         "control mode.",
                         errorPrefix);
            return false;
        }

        // Validate indices
        for (const int idx : jointIndices)
        {
            if (idx < 0 || static_cast<std::size_t>(idx) >= this->actuatedDOFs)
            {
                log()->error("{} Joint index {} is out of range [0, {}).",
                             errorPrefix,
                             idx,
                             this->actuatedDOFs);
                return false;
            }
        }

        auto isScalingNotRequired = [](RobotInterface::ControlMode mode) -> bool {
            return mode == RobotInterface::ControlMode::Torque
                   || mode == RobotInterface::ControlMode::PWM
                   || mode == RobotInterface::ControlMode::Current;
        };

        // For PositionDirect: check the error for the subset joints
        if (controlMode == RobotInterface::ControlMode::PositionDirect)
        {
            if (currentJointValues.has_value())
            {
                if (currentJointValues->size() != n)
                {
                    log()->error(
                        "{} currentJointValues size ({}) does not match jointIndices size ({}).",
                        errorPrefix,
                        currentJointValues->size(),
                        n);
                    return false;
                }
                for (int i = 0; i < n; i++)
                {
                    const double error
                        = std::abs((*currentJointValues)[i] - jointValues[i]);
                    if (error > this->positionDirectMaxAdmissibleError)
                    {
                        if (this->jointsTypeList[jointIndices[i]] == JointType::REVOLUTE)
                        {
                            log()->error(
                                "{} The error between the current and the desired position of joint "
                                "'{}' is greater than {} deg. Error = {} deg.",
                                errorPrefix,
                                this->axesName[jointIndices[i]],
                                180.0 / M_PI * this->positionDirectMaxAdmissibleError,
                                180.0 / M_PI * error);
                        } else
                        {
                            log()->error(
                                "{} The error between the current and the desired position of joint "
                                "'{}' is greater than {}. Error = {}.",
                                errorPrefix,
                                this->axesName[jointIndices[i]],
                                this->positionDirectMaxAdmissibleError,
                                error);
                        }
                        return false;
                    }
                }
            } else
            {
                if (!this->getJointPos())
                {
                    log()->error("{} Unable to get the joint position.", errorPrefix);
                    return false;
                }
                for (int i = 0; i < n; i++)
                {
                    const double error
                        = std::abs(this->positionFeedback[jointIndices[i]] - jointValues[i]);
                    if (error > this->positionDirectMaxAdmissibleError)
                    {
                        if (this->jointsTypeList[jointIndices[i]] == JointType::REVOLUTE)
                        {
                            log()->error(
                                "{} The error between the current and the desired position of joint "
                                "'{}' is greater than {} deg. Error = {} deg.",
                                errorPrefix,
                                this->axesName[jointIndices[i]],
                                180.0 / M_PI * this->positionDirectMaxAdmissibleError,
                                180.0 / M_PI * error);
                        } else
                        {
                            log()->error(
                                "{} The error between the current and the desired position of joint "
                                "'{}' is greater than {}. Error = {}.",
                                errorPrefix,
                                this->axesName[jointIndices[i]],
                                this->positionDirectMaxAdmissibleError,
                                error);
                        }
                        return false;
                    }
                }
            }
        }

        // For Position mode: compute reference speeds
        if (controlMode == RobotInterface::ControlMode::Position)
        {
            this->subsetCurrentPos.resize(n);
            if (currentJointValues.has_value())
            {
                if (currentJointValues->size() != n)
                {
                    log()->error(
                        "{} currentJointValues size ({}) does not match jointIndices size ({}).",
                        errorPrefix,
                        currentJointValues->size(),
                        n);
                    return false;
                }
                this->subsetCurrentPos = *currentJointValues;
            } else
            {
                if (!this->getJointPos())
                {
                    log()->error("{} Unable to get the joint position.", errorPrefix);
                    return false;
                }
                for (int i = 0; i < n; i++)
                {
                    this->subsetCurrentPos[i] = this->positionFeedback[jointIndices[i]];
                }
            }

            const double positioningDurationSeconds
                = std::chrono::duration<double>(this->positioningDuration).count();
            this->subsetRefSpeeds.resize(n);
            for (int i = 0; i < n; i++)
            {
                const double jointError = std::abs(jointValues[i] - this->subsetCurrentPos[i]);
                double scaling, minVelocity;
                if (this->jointsTypeList[jointIndices[i]] == JointType::REVOLUTE)
                {
                    scaling = 180.0 / M_PI;
                    minVelocity = RevolutePositionMinVelocity;
                } else
                {
                    scaling = 1.0;
                    minVelocity = PrismaticPositionMinVelocity;
                }
                this->subsetRefSpeeds[i] = std::max(minVelocity,
                                        scaling * (jointError / positioningDurationSeconds));
            }

            if (!this->positionInterface->setRefSpeeds(n,
                                                       jointIndices.data(),
                                                       this->subsetRefSpeeds.data()))
            {
                log()->error("{} Unable to set the reference speed for the position control joints.",
                             errorPrefix);
                return false;
            }

            this->startPositionControlInstant = BipedalLocomotion::clock().now();
        }

        // Build desired values with unit conversion
        this->subsetRefs.resize(n);
        for (int i = 0; i < n; i++)
        {
            double scaling = 1.0;
            if (this->jointsTypeList[jointIndices[i]] == JointType::REVOLUTE
                && !isScalingNotRequired(controlMode))
            {
                scaling = 180.0 / M_PI;
            }
            this->subsetRefs[i] = scaling * jointValues[i];
        }

        if (!this->control(controlMode)(n, jointIndices.data(), this->subsetRefs.data()))
        {
            // the device applied the references to the joints that accept them
            this->warnReferencesNotAccepted(errorPrefix, controlMode);
            return false;
        }

        return true;
    }
};

YarpRobotControl::YarpRobotControl()
    : m_pimpl(std::make_unique<Impl>())
{
}

YarpRobotControl::~YarpRobotControl() = default;

bool YarpRobotControl::setDriver(std::shared_ptr<yarp::dev::PolyDriver> robotDevice)
{
    return m_pimpl->setDriver(robotDevice);
}

bool YarpRobotControl::initialize(std::weak_ptr<ParametersHandler::IParametersHandler> handler)
{
    constexpr auto errorPrefix = "[YarpRobotControl::initialize]";

    auto ptr = handler.lock();
    if (ptr == nullptr)
    {
        log()->error("{} The handler is not pointing to an already initialized memory.",
                     errorPrefix);
        return false;
    }

    // optional parameters
    int temp = 0;
    if (ptr->getParameter("reading_timeout", temp))
    {
        // the reading timeout has to be a positive number
        if (temp < 0)
        {
            log()->error("{} 'reading_timeout' parameter has to be a positive number.",
                         errorPrefix);
            return false;
        }
        m_pimpl->readingTimeout = temp;
    }

    if (ptr->getParameter("max_reading_attempts", temp))
    {
        // the max_reading_attempts has to be a strictly positive number
        if (temp <= 0)
        {
            log()->error("{} 'max_reading_attempts' parameter has to be a strictly positive "
                         "number.",
                         errorPrefix);

            return false;
        }
        m_pimpl->maxReadingAttempts = temp;
    }

    // mandatory parameters
    using namespace std::chrono_literals;
    bool ok = ptr->getParameter("positioning_duration", m_pimpl->positioningDuration)
              && (m_pimpl->positioningDuration > 0s);
    ok = ok && ptr->getParameter("positioning_tolerance", m_pimpl->positioningTolerance)
         && (m_pimpl->positioningTolerance > 0);
    ok = ok
         && ptr->getParameter("position_direct_max_admissible_error",
                              m_pimpl->positionDirectMaxAdmissibleError)
         && (m_pimpl->positionDirectMaxAdmissibleError > 0);

    return ok;
}

bool YarpRobotControl::setControlMode(const std::vector<RobotInterface::ControlMode>& controlModes)
{
    if (!m_pimpl->setControlModes(controlModes))
    {
        log()->error("[YarpRobotControl::setControlMode] Unable to set the control modes.");
        return false;
    }

    return true;
}

bool YarpRobotControl::setControlMode(const RobotInterface::ControlMode& mode)
{
    // create a vector containing the same control mode for all the joints
    const std::vector<RobotInterface::ControlMode> controlModes(m_pimpl->actuatedDOFs, mode);
    return this->setControlMode(controlModes);
}

bool YarpRobotControl::setControlMode(const std::vector<RobotInterface::ControlMode>& controlModes,
                                      const std::vector<int>& jointIndices)
{
    return m_pimpl->setControlModesSubset(controlModes, jointIndices);
}

bool YarpRobotControl::setControlMode(const std::vector<RobotInterface::ControlMode>& controlModes,
                                      const std::vector<std::string>& jointNames)
{
    constexpr auto errorPrefix = "[YarpRobotControl::setControlMode]";

    std::vector<int> jointIndices;
    jointIndices.reserve(jointNames.size());
    for (const auto& name : jointNames)
    {
        const auto it = std::find(m_pimpl->axesName.begin(), m_pimpl->axesName.end(), name);
        if (it == m_pimpl->axesName.end())
        {
            log()->error("{} Joint '{}' not found in the joint list.", errorPrefix, name);
            return false;
        }
        jointIndices.push_back(static_cast<int>(std::distance(m_pimpl->axesName.begin(), it)));
    }

    return m_pimpl->setControlModesSubset(controlModes, jointIndices);
}

bool YarpRobotControl::setInteractionMode(
    const std::vector<RobotInterface::InteractionMode>& interactionModes)
{
    if (m_pimpl->interactionModeInterface == nullptr)
    {
        log()->error("[YarpRobotControl::setInteractionMode] The interaction mode interface is "
                     "not ready.");
        return false;
    }
    if (interactionModes.size() != m_pimpl->actuatedDOFs)
    {
        log()->error("[YarpRobotControl::setInteractionMode] The number of interaction modes ({}) "
                     "does not match the number of joints ({}).",
                     interactionModes.size(), m_pimpl->actuatedDOFs);
        return false;
    }
    std::vector<int> jointIndices(m_pimpl->actuatedDOFs);
    std::iota(jointIndices.begin(), jointIndices.end(), 0);
    return m_pimpl->setInteractionModesSubset(interactionModes, jointIndices);
}

bool YarpRobotControl::setInteractionMode(const RobotInterface::InteractionMode& interactionMode)
{
    if (m_pimpl->interactionModeInterface == nullptr)
    {
        log()->error("[YarpRobotControl::setInteractionMode] The interaction mode interface is "
                     "not ready.");
        return false;
    }
    return setInteractionMode(
        std::vector<RobotInterface::InteractionMode>(m_pimpl->actuatedDOFs, interactionMode));
}

bool YarpRobotControl::setInteractionMode(
    const std::vector<RobotInterface::InteractionMode>& interactionModes,
    const std::vector<int>& jointIndices)
{
    return m_pimpl->setInteractionModesSubset(interactionModes, jointIndices);
}

bool YarpRobotControl::setInteractionMode(
    const std::vector<RobotInterface::InteractionMode>& interactionModes,
    const std::vector<std::string>& jointNames)
{
    constexpr auto errorPrefix = "[YarpRobotControl::setInteractionMode]";
    std::vector<int> jointIndices;
    jointIndices.reserve(jointNames.size());
    for (const auto& name : jointNames)
    {
        const auto it = std::find(m_pimpl->axesName.begin(), m_pimpl->axesName.end(), name);
        if (it == m_pimpl->axesName.end())
        {
            log()->error("{} Joint '{}' not found in the joint list.", errorPrefix, name);
            return false;
        }
        jointIndices.push_back(static_cast<int>(std::distance(m_pimpl->axesName.begin(), it)));
    }
    return m_pimpl->setInteractionModesSubset(interactionModes, jointIndices);
}

std::future<bool>
YarpRobotControl::setControlModeAsync(const std::vector<RobotInterface::ControlMode>& controlModes)
{
    // lambda function to set the control mode
    auto setControlMode
        = [this, controlModes]() -> bool { return this->setControlMode(controlModes); };

    return std::async(std::launch::async, setControlMode);
}

std::future<bool> YarpRobotControl::setControlModeAsync(const RobotInterface::ControlMode& mode)
{
    // lambda function to set the control mode
    auto setControlMode = [this, mode]() -> bool { return this->setControlMode(mode); };

    return std::async(std::launch::async, setControlMode);
}

bool YarpRobotControl::setReferences(
    Eigen::Ref<const Eigen::VectorXd> desiredJointValues,
    const std::vector<RobotInterface::ControlMode>& controlModes,
    std::optional<Eigen::Ref<const Eigen::VectorXd>> currentJointValues)
{
    if (!m_pimpl->checkControlMode(controlModes))
    {
        log()->error("[YarpRobotControl::setReferences] Control modes are not the expected one. "
                     "Please call setControlMode before calling this function.");
        return false;
    }

    return m_pimpl->setReferences(desiredJointValues, currentJointValues);
}

bool YarpRobotControl::setReferences(
    Eigen::Ref<const Eigen::VectorXd> desiredJointValues,
    const RobotInterface::ControlMode& mode,
    std::optional<Eigen::Ref<const Eigen::VectorXd>> currentJointValues)
{
    if (!m_pimpl->checkControlMode(mode))
    {
        log()->error("[YarpRobotControl::setReferences] Control mode is not the expected one. "
                     "Please call setControlMode before calling this function.");
        return false;
    }

    return m_pimpl->setReferences(desiredJointValues, currentJointValues);
}

bool YarpRobotControl::checkMotionDone(bool& motionDone,
                                       bool& isTimeExpired,
                                       std::vector<std::pair<std::string, double>>& info)
{
    constexpr auto errorPrefix = "[YarpRobotControl::checkMotionDone]";

    if (!m_pimpl->positionInterface->checkMotionDone(&motionDone))
    {
        log()->error("{} Unable to check if the motion is terminated from the Yarp interface.",
                     errorPrefix);
        return false;
    }

    if (!m_pimpl->getJointPos())
    {
        log()->error("{} Unable to get the joint position.", errorPrefix);
        return false;
    }

    info.clear();

    const auto& jointPositionControlIndex
        = m_pimpl->desiredJointValuesAndMode.index[RobotInterface::ControlMode::Position];

    const auto& jointPositionControlDesiredValue
        = m_pimpl->desiredJointValuesAndMode.value[RobotInterface::ControlMode::Position];

    for (int i = 0; i < jointPositionControlIndex.size(); i++)
    {
        const double desiredJointPosRad = jointPositionControlDesiredValue[i] * M_PI / 180;
        const double error = std::abs(desiredJointPosRad
                                      - m_pimpl->positionFeedback[jointPositionControlIndex[i]]);

        if (error > m_pimpl->positioningTolerance)
        {
            info.push_back({m_pimpl->axesName[jointPositionControlIndex[i]], error});
        }
    }

    using namespace std::chrono_literals;
    const std::chrono::nanoseconds now = BipedalLocomotion::clock().now();
    constexpr std::chrono::nanoseconds timeTolerance{1s};
    if (now - m_pimpl->startPositionControlInstant > m_pimpl->positioningDuration + timeTolerance)
    {
        isTimeExpired = true;
    } else
    {
        isTimeExpired = false;
    }
    motionDone = motionDone && info.empty();

    return true;
}

std::vector<std::string> YarpRobotControl::getJointList() const
{
    return m_pimpl->axesName;
}

bool YarpRobotControl::isValid() const
{
    return m_pimpl->robotDevice != nullptr;
}

bool YarpRobotControl::getJointLimits(Eigen::Ref<Eigen::VectorXd> lowerLimits,
                                      Eigen::Ref<Eigen::VectorXd> upperLimits) const
{

    constexpr auto errorPrefix = "[YarpRobotControl::getJointLimits]";

    if (lowerLimits.size() != m_pimpl->actuatedDOFs)
    {
        log()->error("{} The size of the first input vector is not "
                     "correct. Expected size: {}. Received size: {}.",
                     errorPrefix,
                     m_pimpl->actuatedDOFs,
                     lowerLimits.size());
        return false;
    }

    if (upperLimits.size() != m_pimpl->actuatedDOFs)
    {
        log()->error("{} The size of the second input vector is not "
                     "correct. Expected size: {}. Received size: {}.",
                     errorPrefix,
                     m_pimpl->actuatedDOFs,
                     upperLimits.size());
        return false;
    }

    for (int i = 0; i < m_pimpl->actuatedDOFs; i++)
    {
        m_pimpl->controlLimitsInterface->getLimits(i, &lowerLimits[i], &upperLimits[i]);
    }
    return true;
}

bool YarpRobotControl::setReferences(
    Eigen::Ref<const Eigen::VectorXd> desiredJointValues,
    const std::vector<int>& jointIndices,
    const RobotInterface::ControlMode& controlMode,
    std::optional<Eigen::Ref<const Eigen::VectorXd>> currentJointValues)
{
    std::lock_guard<std::mutex> lock(m_pimpl->subsetMutex);
    return m_pimpl->setReferencesSubset(desiredJointValues,
                                        jointIndices,
                                        controlMode,
                                        currentJointValues);
}

bool YarpRobotControl::setReferences(
    Eigen::Ref<const Eigen::VectorXd> desiredJointValues,
    const std::vector<std::string>& jointNames,
    const RobotInterface::ControlMode& controlMode,
    std::optional<Eigen::Ref<const Eigen::VectorXd>> currentJointValues)
{
    constexpr auto errorPrefix = "[YarpRobotControl::setReferences]";

    std::lock_guard<std::mutex> lock(m_pimpl->subsetMutex);
    m_pimpl->subsetJointIndices.clear();
    m_pimpl->subsetJointIndices.reserve(jointNames.size());
    for (const auto& name : jointNames)
    {
        const auto it
            = std::find(m_pimpl->axesName.begin(), m_pimpl->axesName.end(), name);
        if (it == m_pimpl->axesName.end())
        {
            log()->error("{} Joint '{}' not found in the joint list.", errorPrefix, name);
            return false;
        }
        m_pimpl->subsetJointIndices.push_back(
            static_cast<int>(std::distance(m_pimpl->axesName.begin(), it)));
    }

    return m_pimpl->setReferencesSubset(desiredJointValues,
                                        m_pimpl->subsetJointIndices,
                                        controlMode,
                                        currentJointValues);
}
