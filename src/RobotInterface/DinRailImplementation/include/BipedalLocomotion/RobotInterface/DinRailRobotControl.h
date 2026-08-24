/**
 * @file DinRailRobotControl.h
 * @authors Giulio Romualdi
 * @copyright Generative Bionics S.R.L. This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_ROBOT_INTERFACE_DINRAIL_ROBOT_CONTROL_H
#define BIPEDAL_LOCOMOTION_ROBOT_INTERFACE_DINRAIL_ROBOT_CONTROL_H

#include <memory>
#include <vector>

#include <Eigen/Dense>

#include <yarp/dev/PolyDriver.h>

#include <BipedalLocomotion/RobotInterface/YarpRobotControl.h>

namespace BipedalLocomotion
{
namespace RobotInterface
{

/**
 * DinRailRobotControl extends YarpRobotControl with support for the
 * dinrail::IImpedanceAllSetPointsControl interface exposed by DinRailControlBoardNWCYarp.
 *
 * All standard control modes (Position, PositionDirect, Velocity, Torque, PWM, Current)
 * are inherited from YarpRobotControl without modification.  The new method
 * setImpedanceSetPoints() additionally allows setting all impedance control
 * setpoints (position, velocity, torque feedforward, stiffness, damping) in a
 * single call, as defined by the MIT-mode / IImpedanceAllSetPointsControl interface.
 *
 * @note The dinrail::IImpedanceAllSetPointsControl interface uses the same units
 *       as the corresponding YARP interfaces (degrees / degrees per second for
 *       revolute joints).  setImpedanceSetPoints() therefore performs the same
 *       radian-to-degree conversion for position and velocity of revolute joints
 *       that YarpRobotControl applies to the other control modes.
 *       Torque, stiffness and damping are forwarded without unit conversion.
 *
 * @warning SensorBridge: since DinRailControlBoardNWCYarp already exposes the
 *          standard YARP encoder and sensor interfaces, YarpSensorBridge can be
 *          used directly and no separate DinRail sensor bridge is required.
 */
class DinRailRobotControl : public YarpRobotControl
{
    struct Impl;
    std::unique_ptr<Impl> m_pimpl;

public:
    /**
     * Constructor.
     */
    DinRailRobotControl();

    /**
     * Destructor.
     */
    ~DinRailRobotControl() override;

    /**
     * Initialize the interface.
     * @param handler pointer to a parameter handler interface.
     * @note Accepts the same parameters as YarpRobotControl::initialize().
     * @return True/False in case of success/failure.
     */
    bool initialize(std::weak_ptr<ParametersHandler::IParametersHandler> handler) override;

    /**
     * Set the driver required to control the robot.
     *
     * In addition to all YARP interfaces acquired by YarpRobotControl, this
     * method also acquires the dinrail::IImpedanceAllSetPointsControl interface
     * from the device.  The device is expected to be an instance of
     * DinRailControlBoardNWCYarp (or a compatible device that exposes
     * dinrail::IImpedanceAllSetPointsControl).
     *
     * @param robotDevice device used to control the robot.
     * @return True/False in case of success/failure.
     */
    bool setDriver(std::shared_ptr<yarp::dev::PolyDriver> robotDevice) override;

    /**
     * Set all impedance control setpoints (MIT mode) for all controlled joints.
     *
     * This method maps to dinrail::IImpedanceAllSetPointsControl::setSetPoints()
     * for all joints.  Position and velocity inputs are expected in SI units
     * (radians and radians per second); they are converted to degrees /
     * degrees-per-second internally for revolute joints before being forwarded
     * to the underlying device.  For revolute joints, position and velocity are
     * converted from radians to degrees, stiffness from Nm/rad to Nm/deg, and
     * damping from Nms/rad to Nms/deg (all multiplied by π/180).
     * Torque feedforward is always forwarded in Nm without conversion.
     *
     * @param position Position setpoints for all controlled joints [rad].
     * @param velocity Velocity setpoints for all controlled joints [rad/s].
     * @param torque Torque feedforward setpoints for all controlled joints [Nm].
     * @param stiffness Stiffness setpoints for all controlled joints [Nm/rad].
     * @param damping Damping setpoints for all controlled joints [Nms/rad].
     * @return True on success, false otherwise.
     */
    bool setImpedanceSetPoints(Eigen::Ref<const Eigen::VectorXd> position,
                               Eigen::Ref<const Eigen::VectorXd> velocity,
                               Eigen::Ref<const Eigen::VectorXd> torque,
                               Eigen::Ref<const Eigen::VectorXd> stiffness,
                               Eigen::Ref<const Eigen::VectorXd> damping);

    /**
     * Set impedance control setpoints (MIT mode) for a subset of joints.
     *
     * Mirrors the behaviour of IRobotControl::setReferences() with joint indices:
     * only the joints listed in @p jointIndices are updated; all other joints
     * retain the values they were last commanded with.
     *
     * Each input vector must have exactly `jointIndices.size()` elements.  The
     * same radian-to-degree conversion rules as the full overload apply.
     *
     * @param position   Position setpoints for the subset joints [rad].
     * @param velocity   Velocity setpoints for the subset joints [rad/s].
     * @param torque     Torque feedforward setpoints for the subset joints [Nm].
     * @param stiffness  Stiffness setpoints for the subset joints [Nm/rad].
     * @param damping    Damping setpoints for the subset joints [Nms/rad].
     * @param jointIndices Indices (into the full control-board joint list) of the
     *                     joints to update.
     * @return True on success, false otherwise.
     * @note The full overload must be called at least once before this overload
     *       is used, so that the cached state for the non-indexed joints is
     *       properly initialised.
     */
    bool setImpedanceSetPoints(Eigen::Ref<const Eigen::VectorXd> position,
                               Eigen::Ref<const Eigen::VectorXd> velocity,
                               Eigen::Ref<const Eigen::VectorXd> torque,
                               Eigen::Ref<const Eigen::VectorXd> stiffness,
                               Eigen::Ref<const Eigen::VectorXd> damping,
                               const std::vector<int>& jointIndices);

    /**
     * Set impedance control setpoints (MIT mode) for a subset of joints identified by name.
     *
     * Resolves each name in @p jointNames to its index in the controlled joint list and
     * delegates to the index-based overload.  All other joints retain their last commanded
     * setpoints.  Each input vector must have exactly `jointNames.size()` elements.
     * The same unit-conversion rules as the full overload apply.
     *
     * @param position   Position setpoints for the named joints [rad].
     * @param velocity   Velocity setpoints for the named joints [rad/s].
     * @param torque     Torque feedforward setpoints for the named joints [Nm].
     * @param stiffness  Stiffness setpoints for the named joints [Nm/rad].
     * @param damping    Damping setpoints for the named joints [Nms/rad].
     * @param jointNames Names of the joints to update (must be in the controlled joint list).
     * @return True on success, false if any name is not found or inputs are inconsistent.
     * @note The full overload must be called at least once before this overload
     *       is used.
     */
    bool setImpedanceSetPoints(Eigen::Ref<const Eigen::VectorXd> position,
                               Eigen::Ref<const Eigen::VectorXd> velocity,
                               Eigen::Ref<const Eigen::VectorXd> torque,
                               Eigen::Ref<const Eigen::VectorXd> stiffness,
                               Eigen::Ref<const Eigen::VectorXd> damping,
                               const std::vector<std::string>& jointNames);
};

} // namespace RobotInterface
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_ROBOT_INTERFACE_DINRAIL_ROBOT_CONTROL_H
