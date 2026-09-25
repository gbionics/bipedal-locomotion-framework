/**
 * @file CentroidalMPC.cpp
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */
#include <chrono>
#include <limits>
#include <string>
#include <unordered_map>

#include <casadi/casadi.hpp>
#include <casadi/config.h>

#ifdef BLF_CENTROIDAL_MPC_USE_OPENMP
#include <omp.h>
#endif

#include <BipedalLocomotion/Contacts/Contact.h>
#include <BipedalLocomotion/Conversions/CasadiConversions.h>
#include <BipedalLocomotion/Math/Constants.h>
#include <BipedalLocomotion/Math/LinearizedFrictionCone.h>
#include <BipedalLocomotion/ReducedModelControllers/CentroidalMPC.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

using namespace BipedalLocomotion::ReducedModelControllers;
using namespace BipedalLocomotion::Contacts;

constexpr bool casadiVersionIsAtLeast360
    = (CASADI_MAJOR_VERSION > 3) || (CASADI_MAJOR_VERSION == 3 && CASADI_MINOR_VERSION >= 6);

inline double chronoToSeconds(const std::chrono::nanoseconds& d)
{
    return std::chrono::duration<double>(d).count();
}

/**
 * Limit the OpenMP threads of the calling thread for the lifetime of the object. For problems of
 * the size of the MPC the OpenMP version of the linear solver (e.g., MUMPS) is much slower than the
 * sequential one.
 */
struct SingleThreadedOpenMPScope
{
#ifdef BLF_CENTROIDAL_MPC_USE_OPENMP
    const int previousNumberOfThreads{omp_get_max_threads()};

    SingleThreadedOpenMPScope()
    {
        omp_set_num_threads(1);
    }

    ~SingleThreadedOpenMPScope()
    {
        omp_set_num_threads(previousNumberOfThreads);
    }
#endif
};

std::vector<std::string> extractVariablesName(const std::vector<casadi::MX>& variables)
{
    std::vector<std::string> variablesName;
    variablesName.reserve(variables.size());
    for (const auto& variable : variables)
    {
        variablesName.push_back(variable.name());
    }

    return variablesName;
}

struct CentroidalMPC::Impl
{
    casadi::Opti opti; /**< CasADi opti stack */
    casadi::Function controller;
    std::chrono::nanoseconds currentTime{std::chrono::nanoseconds::zero()};

    CentroidalMPCOutput output;
    Contacts::ContactPhaseList contactPhaseList;
    Math::LinearizedFrictionCone frictionCone;

    enum class FSM
    {
        Idle,
        Initialized,
        OutputValid,
        OutputInvalid,
    };

    FSM fsm{FSM::Idle};

    struct CasadiCorner
    {
        casadi::DM position;
        casadi::MX force;
        casadi::MX previousForce; /**< Only for fatrop. Column k is a copy of force(:, k). */
        casadi::MX forceLinearizationPoint; /**< Only for sqp. */

        std::string cornerName;

        CasadiCorner(const std::string& cornerName)
            : cornerName(cornerName)
        {
        }

        CasadiCorner() = default;

        CasadiCorner(const std::string& cornerName, const Corner& other)
            : cornerName(cornerName)
        {
            this->operator=(other);
        }

        CasadiCorner& operator=(const Corner& other)
        {
            this->position.resize(3, 1);
            this->position(0, 0) = other.position(0);
            this->position(1, 0) = other.position(1);
            this->position(2, 0) = other.position(2);

            this->force = casadi::MX::sym(cornerName + "_force", other.force.size(), 1);

            return *this;
        }
    };

    struct CasadiContact
    {
        casadi::MX position;
        casadi::MX linearVelocity;
        casadi::MX orientation;
        casadi::MX isEnabled;
        std::vector<CasadiCorner> corners;

        std::string contactName;

        CasadiContact(const std::string& contactName)
            : contactName(contactName)
        {
        }

        CasadiContact& operator=(const DiscreteGeometryContact& other)
        {
            corners.resize(other.corners.size());

            for (int i = 0; i < other.corners.size(); i++)
            {
                this->corners[i].cornerName = contactName + "_" + std::to_string(i);
                this->corners[i] = other.corners[i];
            }

            this->orientation = casadi::MX::sym(contactName + "_orientation", 3 * 3);
            this->position = casadi::MX::sym(contactName + "_position", 3);
            this->linearVelocity = casadi::MX::sym(contactName + "_linear_velocity", 3);
            this->isEnabled = casadi::MX::sym(contactName + "_is_enable");

            return *this;
        }

        CasadiContact(const std::string& contactName, const DiscreteGeometryContact& other)
            : contactName(contactName)
        {
            this->operator=(other);
        }
    };

    struct CasadiContactWithConstraints : CasadiContact
    {

        CasadiContactWithConstraints(const std::string& contactName)
            : CasadiContact(contactName)
        {
        }

        casadi::MX currentPosition;
        casadi::MX nominalPosition;
        casadi::MX upperLimitPosition;
        casadi::MX lowerLimitPosition;
        casadi::MX positionLinearizationPoint; /**< Only for sqp. */
    };

    struct OptimizationSettings
    {
        int solverVerbosity{0}; /**< Verbosity of ipopt */
        std::string ipoptLinearSolver{"mumps"}; /**< Linear solved used by ipopt */
        double ipoptTolerance{1e-8}; /**< Tolerance of ipopt
                                        (https://coin-or.github.io/Ipopt/OPTIONS.html#OPT_tol) */
        int ipoptMaxIteration{3000}; /**< Maximum number of iteration */
        double fatropTolerance{1e-8}; /**< Convergence tolerance of fatrop */

        int horizon; /**<Number of samples used in the horizon */
        std::chrono::nanoseconds samplingTime; /**< Sampling time of the planner */
        std::chrono::nanoseconds timeHorizon; /**< Duration of the horizon */
        bool isWarmStartEnabled{false}; /**< True if the user wants to warm start the CoM, angular
                                           momentum and contact. */
        bool isCseEnabled{false}; /**< True if the Common subexpression elimination casadi option is
                                       enabled. */

        std::string solverName{"ipopt"}; /**< Name of the solver used by the MPC. */
        int numberOfQPIterations{1}; /**< Maximum number of QPs solved at each advance by the sqp
                                        solver. 1 corresponds to the real-time iteration scheme. */
        double sqpTolerance{1e-4}; /**< The sqp stops when the step is smaller than this value. */
        double osqpTolerance{1e-5}; /**< Absolute and relative tolerance of osqp. */
        int osqpMaxIteration{4000}; /**< Maximum number of iterations of osqp. */
    };

    OptimizationSettings optiSettings; /**< Settings */

    /**
     * OptimizationVariables contains the optimization variables expressed as CasADi elements.
     */
    struct OptimizationVariables
    {
        casadi::MX com;
        casadi::MX dcom;
        casadi::MX angularMomentum;
        std::map<std::string, CasadiContactWithConstraints> contacts;

        casadi::MX comReference;
        casadi::MX angularMomentumReference;
        casadi::MX comCurrent;
        casadi::MX dcomCurrent;
        casadi::MX angularMomentumCurrent;
        casadi::MX externalForce;
        casadi::MX externalTorque;
        casadi::MX gravity;
        casadi::MX comLinearizationPoint; /**< Only for sqp. */
    };
    OptimizationVariables optiVariables; /**< Optimization variables */

    struct ContactsInputs
    {
        casadi::DM* currentPosition;
        casadi::DM* orientation;
        casadi::DM* nominalPosition;
        casadi::DM* upperLimitPosition;
        casadi::DM* lowerLimitPosition;
        casadi::DM* isEnabled;
    };
    struct ControllerInputs
    {
        std::map<std::string, ContactsInputs> contacts;

        casadi::DM* comReference;
        casadi::DM* angularMomentumReference;
        casadi::DM* comCurrent;
        casadi::DM* dcomCurrent;
        casadi::DM* angularMomentumCurrent;
        casadi::DM* externalForce;
        casadi::DM* externalTorque;
        casadi::DM* gravity;
    };
    ControllerInputs controllerInputs; /**< The pointers will point to the vectorized input */

    struct ContactInitialGuess
    {
        casadi::DM* contactLocation;
        std::vector<casadi::DM*> contactForce;
    };

    struct InitialGuess
    {
        std::map<std::string, ContactInitialGuess> contactsInitialGuess;

        casadi::DM* com;
        casadi::DM* angularMomentum;
    };
    InitialGuess initialGuess;

    /**
     * Current iterate of the sqp solver. It is used both as linearization point and as warm start
     * of osqp. After each advance it is shifted of one sample (real-time iteration scheme).
     */
    struct SqpContactIterate
    {
        casadi::DM* position;
        casadi::DM* linearVelocity;
        std::vector<casadi::DM*> force;
        casadi::DM* positionLinearizationPoint;
        std::vector<casadi::DM*> forceLinearizationPoint;
    };

    struct SqpIterate
    {
        casadi::DM* com;
        casadi::DM* dcom;
        casadi::DM* angularMomentum;
        casadi::DM* comLinearizationPoint;
        std::map<std::string, SqpContactIterate> contacts;
        casadi::DM* multipliers;
        bool isValid{false};
    };
    SqpIterate sqpIterate;

    /**
     * Rows of the constraints multipliers to be copied from the next stage when the sqp iterate is
     * shifted.
     */
    struct MultipliersShift
    {
        casadi_int destination;
        casadi_int source;
        casadi_int size;
    };
    std::vector<MultipliersShift> multipliersShift;

    bool isSqp() const
    {
        return this->optiSettings.solverName == "sqp";
    }


    std::vector<casadi::DM> vectorizedOptiInputs;

    struct Weights
    {
        Eigen::Vector3d com;
        double contactPosition;
        Eigen::Vector3d forceRateOfChange;
        double angularMomentum;
        double contactForceSymmetry;
    };
    Weights weights;

    struct ContactBoundingBox
    {
        Eigen::Vector3d upperLimit;
        Eigen::Vector3d lowerLimit;
    };

    std::unordered_map<std::string, ContactBoundingBox> contactBoundingBoxes;

    bool loadContactCorners(std::shared_ptr<const ParametersHandler::IParametersHandler> ptr,
                            DiscreteGeometryContact& contact)
    {
        constexpr auto errorPrefix = "[CentroidalMPC::Impl::loadContactCorners]";

        int numberOfCorners;
        if (!ptr->getParameter("number_of_corners", numberOfCorners))
        {
            log()->error("{} Unable to get the number of corners.", errorPrefix);
            return false;
        }
        contact.corners.resize(numberOfCorners);

        for (std::size_t j = 0; j < numberOfCorners; j++)
        {
            if (!ptr->getParameter("corner_" + std::to_string(j), contact.corners[j].position))
            {
                // prepare the error
                std::string cornesNames;
                for (std::size_t k = 0; k < numberOfCorners; k++)
                {
                    cornesNames += " corner_" + std::to_string(k);
                }

                log()->error("{} Unable to load the corner number {}. Please provide the corners "
                             "having the following names:{}.",
                             errorPrefix,
                             j,
                             cornesNames);

                return false;
            }
        }

        return true;
    }

    bool loadParameters(std::shared_ptr<const ParametersHandler::IParametersHandler> ptr)
    {
        constexpr auto logPrefix = "[CentroidalMPC::Impl::loadParameters]";

        auto getParameter
            = [logPrefix](std::shared_ptr<const ParametersHandler::IParametersHandler> ptr,
                          const std::string& paramName,
                          auto& param) -> bool {
            if (!ptr->getParameter(paramName, param))
            {
                log()->error("{} Unable to load the parameter named '{}'.", logPrefix, paramName);
                return false;
            }
            return true;
        };

        auto getOptionalParameter
            = [logPrefix](std::shared_ptr<const ParametersHandler::IParametersHandler> ptr,
                          const std::string& paramName,
                          auto& param) -> void {
            if (!ptr->getParameter(paramName, param))
            {
                log()->info("{} Unable to load the parameter named '{}'. The default one will be "
                            "used '{}'.",
                            logPrefix,
                            paramName,
                            param);
            }
        };

        bool ok = getParameter(ptr, "sampling_time", this->optiSettings.samplingTime);
        ok = ok && getParameter(ptr, "time_horizon", this->optiSettings.timeHorizon);

        if (!ok)
        {
            return false;
        }
        this->optiSettings.horizon
            = this->optiSettings.timeHorizon / this->optiSettings.samplingTime;

        int numberOfMaximumContacts = 0;
        ok = ok && getParameter(ptr, "number_of_maximum_contacts", numberOfMaximumContacts);

        for (std::size_t i = 0; i < numberOfMaximumContacts; i++)
        {
            auto contactHandler = ptr->getGroup("CONTACT_" + std::to_string(i)).lock();

            if (contactHandler == nullptr)
            {
                log()->error("{} Unable to load the contact {}. Please be sure that CONTACT_{} "
                             "group exists.",
                             logPrefix,
                             i,
                             i);
                return false;
            }

            std::string contactName;
            ok = ok && getParameter(contactHandler, "contact_name", contactName);
            if (!ok)
            {
                return false;
            }

            // set the contact name
            this->output.contacts[contactName].name = contactName;
            ok = ok
                 && getParameter(contactHandler,
                                 "bounding_box_upper_limit",
                                 this->contactBoundingBoxes[contactName].upperLimit);
            ok = ok
                 && getParameter(contactHandler,
                                 "bounding_box_lower_limit",
                                 this->contactBoundingBoxes[contactName].lowerLimit);

            if (!this->loadContactCorners(contactHandler, this->output.contacts[contactName]))
            {
                log()->error("{} Unable to load the contact corners for the contact {}.",
                             logPrefix,
                             i);
                return false;
            }
        }

        ok = ok && getParameter(ptr, "com_weight", this->weights.com);
        ok = ok && getParameter(ptr, "contact_position_weight", this->weights.contactPosition);
        ok = ok
             && getParameter(ptr, "force_rate_of_change_weight", this->weights.forceRateOfChange);
        ok = ok && getParameter(ptr, "angular_momentum_weight", this->weights.angularMomentum);
        ok = ok
             && getParameter(ptr,
                             "contact_force_symmetry_weight",
                             this->weights.contactForceSymmetry);

        // initialize the friction cone
        ok = ok && frictionCone.initialize(ptr);
        ok = ok && getParameter(ptr, "solver_name", this->optiSettings.solverName);
        if (!ok)
        {
            return false;
        }
        if (this->optiSettings.solverName != "ipopt" && this->optiSettings.solverName != "sqp"
            && this->optiSettings.solverName != "fatrop")
        {
            log()->error("{} The solver name '{}' is not supported. The supported solvers are "
                         "'ipopt', 'fatrop' and 'sqp'.",
                         logPrefix,
                         this->optiSettings.solverName);
            return false;
        }

        if (this->optiSettings.solverName == "ipopt")
        {
            getOptionalParameter(ptr, "linear_solver", this->optiSettings.ipoptLinearSolver);
            getOptionalParameter(ptr, "ipopt_tolerance", this->optiSettings.ipoptTolerance);
            getOptionalParameter(ptr, "ipopt_max_iteration", this->optiSettings.ipoptMaxIteration);
        } else if (this->optiSettings.solverName == "fatrop")
        {
            getOptionalParameter(ptr, "fatrop_tolerance", this->optiSettings.fatropTolerance);
        } else
        {
            getOptionalParameter(ptr,
                                 "number_of_qp_iterations",
                                 this->optiSettings.numberOfQPIterations);
            getOptionalParameter(ptr, "sqp_tolerance", this->optiSettings.sqpTolerance);
            getOptionalParameter(ptr, "osqp_tolerance", this->optiSettings.osqpTolerance);
            getOptionalParameter(ptr, "osqp_max_iteration", this->optiSettings.osqpMaxIteration);
        }

        getOptionalParameter(ptr, "solver_verbosity", this->optiSettings.solverVerbosity);
        getOptionalParameter(ptr, "is_warm_start_enabled", this->optiSettings.isWarmStartEnabled);
        getOptionalParameter(ptr, "is_cse_enabled", this->optiSettings.isCseEnabled);

        // the sqp solver is always warm started with the previous solution
        if (this->isSqp())
        {
            this->optiSettings.isWarmStartEnabled = false;
        }

        return ok;
    }

    casadi::Function ode()
    {
        const bool isLinearized = this->isSqp();
        // Convert DiscreteGeometryContact into a casadiContact object
        std::map<std::string, CasadiContact> casadiContacts;

        for (const auto& [key, contact] : this->output.contacts)
        {
            CasadiContact temp(key);
            temp = contact;
            auto [contactIt, outcome] = casadiContacts.emplace(key, temp);
        }

        // we assume mass equal to 1
        constexpr double mass = 1;

        casadi::MX com = casadi::MX::sym("com_in", 3);
        casadi::MX dcom = casadi::MX::sym("dcom_in", 3);
        casadi::MX angularMomentum = casadi::MX::sym("angular_momentum_in", 3);

        casadi::MX externalForce = casadi::MX::sym("external_force", 3);
        casadi::MX externalTorque = casadi::MX::sym("external_torque", 3);

        casadi::MX ddcom = casadi::MX::sym("ddcom", 3);
        casadi::MX angularMomentumDerivative = casadi::MX::sym("angular_momentum_derivative", 3);

        casadi::MX gravity = casadi::MX::sym("gravity", 3);

        ddcom = gravity + externalForce / mass;
        angularMomentumDerivative = externalTorque;

        std::vector<casadi::MX> input;
        input.push_back(externalForce);
        input.push_back(externalTorque);
        input.push_back(com);
        input.push_back(dcom);
        input.push_back(angularMomentum);
        input.push_back(gravity);

        // In the sqp the bilinear term of the angular momentum dynamics is linearized around
        // the current iterate, so that each subproblem is a convex QP (Gauss-Newton).
        casadi::MX comLinearizationPoint = casadi::MX::sym("com_linearization_point", 3);
        if (isLinearized)
        {
            input.push_back(comLinearizationPoint);
        }

        for (const auto& [key, contact] : casadiContacts)
        {
            input.push_back(contact.position);
            input.push_back(contact.orientation);
            input.push_back(contact.isEnabled);
            input.push_back(contact.linearVelocity);

            casadi::MX positionLinearizationPoint
                = casadi::MX::sym(key + "_position_linearization_point", 3);
            if (isLinearized)
            {
                input.push_back(positionLinearizationPoint);
            }

            for (const auto& corner : contact.corners)
            {
                using namespace casadi;
                ddcom += contact.isEnabled / mass * corner.force;

                const MX cornerPosition
                    = MX::mtimes(MX::reshape(contact.orientation, 3, 3), corner.position);
                const MX leverArm = cornerPosition + contact.position - com;
                input.push_back(corner.force);

                if (!isLinearized)
                {
                    angularMomentumDerivative
                        += contact.isEnabled * MX::cross(leverArm, corner.force);
                    continue;
                }

                const MX forceLinearizationPoint
                    = MX::sym(corner.cornerName + "_force_linearization_point", 3);
                const MX leverArmLinearizationPoint
                    = cornerPosition + positionLinearizationPoint - comLinearizationPoint;
                angularMomentumDerivative
                    += contact.isEnabled
                       * (MX::cross(leverArmLinearizationPoint, corner.force)
                          + MX::cross(leverArm - leverArmLinearizationPoint,
                                      forceLinearizationPoint));
                input.push_back(forceLinearizationPoint);
            }
        }

        const double dT = chronoToSeconds(this->optiSettings.samplingTime);

        std::vector<std::string> outputName{"com", "dcom", "angular_momentum"};
        std::vector<casadi::MX> rhs{com + dcom * dT,
                                    dcom + ddcom * dT,
                                    angularMomentum + angularMomentumDerivative * dT};

        for (const auto& [key, contact] : casadiContacts)
        {
            rhs.push_back(contact.position + (1 - contact.isEnabled) * contact.linearVelocity * dT);
            outputName.push_back(key);
        }

        return casadi::Function("centroidal_dynamics",
                                std::move(input),
                                std::move(rhs),
                                extractVariablesName(input),
                                std::move(outputName));
    }

    casadi::Function contactPositionError()
    {
        casadi::MX contactPosition = casadi::MX::sym("contact_position", 3);
        casadi::MX nominalContactPosition = casadi::MX::sym("nominal_contact_position", 3);
        casadi::MX contactOrientation = casadi::MX::sym("contact_orientation", 3 * 3);

        // the orientation is stored as a vectorized version of the matrix. We need to reshape it
        casadi::MX rhs = casadi::MX::mtimes(casadi::MX::reshape(contactOrientation, 3, 3).T(),
                                            contactPosition - nominalContactPosition);

        return casadi::Function("contact_position_error",
                                {contactPosition, nominalContactPosition, contactOrientation},
                                {rhs},
                                extractVariablesName({contactPosition, //
                                                      nominalContactPosition,
                                                      contactOrientation}),
                                {"error"});
    }

    casadi::Function frictionConeConstraint()
    {
        // Assumption: the Eigen matrix is stored as column-major
        const Eigen::MatrixXd A = this->frictionCone.getA();
        casadi::DM frictionConeMatrix = casadi::DM::zeros(A.rows(), A.cols());
        std::memcpy(frictionConeMatrix.ptr(), A.data(), sizeof(double) * A.size());

        casadi::MX orientation = casadi::MX::sym("contact_orientation", 3 * 3);
        casadi::MX isEnabled = casadi::MX::sym("is_enabled");
        casadi::MX force = casadi::MX::sym("force", 3);

        // The force of a non active contact does not enter the dynamics, hence the constraint is
        // disabled to avoid a degenerate apex of the cone. The linearized cone is a pyramid, so it
        // already implies a non-negative normal force in the contact frame.
        casadi::MX rhs = isEnabled
                         * casadi::MX::mtimes(frictionConeMatrix,
                                              casadi::MX::mtimes(casadi::MX::reshape(orientation,
                                                                                     3,
                                                                                     3)
                                                                     .T(),
                                                                 force));

        return casadi::Function("friction_cone",
                                {orientation, isEnabled, force},
                                {rhs},
                                extractVariablesName({orientation, isEnabled, force}),
                                {"constraint"});
    }

    void resizeControllerInputs()
    {
        constexpr int vector3Size = 3;
        const int stateHorizon = this->optiSettings.horizon + 1;

        // resize the CoM Trajectory
        this->output.comTrajectory.resize(stateHorizon);
        this->output.comVelocityTrajectory.resize(stateHorizon);
        this->output.angularMomentumTrajectory.resize(stateHorizon);

        // In case of no warmstart the variables are:
        // - centroidalVariables = 8: external force + external torque + com current + dcom current
        //                            + current angular momentum + gravity + com reference
        //                            + angular momentum reference
        // - contactVariables = 6: for each contact we have current position + nominal position +
        //                         orientation + is enabled + upper limit in position
        //                         + lower limit in position
        constexpr std::size_t centroidalVariables = 8;
        constexpr std::size_t contactVariables = 6;

        std::size_t vectorizedOptiInputsSize = centroidalVariables + //
                                               (this->output.contacts.size() * contactVariables);

        if (this->optiSettings.isWarmStartEnabled)
        {
            // in this case we need to add the com, the angular momentum and the contact location
            constexpr std::size_t centroidalVariablesWarmStart = 2;
            constexpr std::size_t contactVariablesWarmStart = 1;
            vectorizedOptiInputsSize += centroidalVariablesWarmStart + //
                                        (this->output.contacts.size() * contactVariablesWarmStart);

            for (const auto& [key, contact] : this->output.contacts)
            {
                for (const auto& corner : contact.corners)
                {
                    // for each corner we have the force
                    vectorizedOptiInputsSize += 1;
                }
            }
        }

        if (this->isSqp())
        {
            // com, dcom, angular momentum and com linearization point + for each contact the
            // position, the velocity, the position linearization point and for each corner the
            // force and the force linearization point
            vectorizedOptiInputsSize += 5; // +1 for the multipliers added by createController
            for (const auto& [key, contact] : this->output.contacts)
            {
                vectorizedOptiInputsSize += 3 + 2 * contact.corners.size();
            }
        }

        // we reserve in advance so the push_back will not invalidate the pointers
        // Indeed the standard guarantees that if the new size() is greater than capacity() then all
        // iterators and references (including the end() iterator) are invalidated. Otherwise only
        // the end() iterator is invalidated.
        // https://en.cppreference.com/w/cpp/container/vector/push_back
        this->vectorizedOptiInputs.reserve(vectorizedOptiInputsSize);

        // prepare the controller inputs struct
        // The order matches the one required by createController
        this->vectorizedOptiInputs.push_back(casadi::DM::zeros(vector3Size, //
                                                               this->optiSettings.horizon));
        this->controllerInputs.externalForce = &this->vectorizedOptiInputs.back();

        this->vectorizedOptiInputs.push_back(casadi::DM::zeros(vector3Size, //
                                                               this->optiSettings.horizon));
        this->controllerInputs.externalTorque = &this->vectorizedOptiInputs.back();

        this->vectorizedOptiInputs.push_back(casadi::DM::zeros(vector3Size));
        this->controllerInputs.comCurrent = &this->vectorizedOptiInputs.back();

        this->vectorizedOptiInputs.push_back(casadi::DM::zeros(vector3Size));
        this->controllerInputs.dcomCurrent = &this->vectorizedOptiInputs.back();

        this->vectorizedOptiInputs.push_back(casadi::DM::zeros(vector3Size));
        this->controllerInputs.angularMomentumCurrent = &this->vectorizedOptiInputs.back();

        this->vectorizedOptiInputs.push_back(casadi::DM::zeros(vector3Size));
        this->controllerInputs.gravity = &this->vectorizedOptiInputs.back();

        this->vectorizedOptiInputs.push_back(casadi::DM::zeros(vector3Size, stateHorizon));
        this->controllerInputs.comReference = &this->vectorizedOptiInputs.back();

        this->vectorizedOptiInputs.push_back(casadi::DM::zeros(vector3Size, stateHorizon));
        this->controllerInputs.angularMomentumReference = &this->vectorizedOptiInputs.back();

        if (this->optiSettings.isWarmStartEnabled)
        {
            this->vectorizedOptiInputs.push_back(casadi::DM::zeros(vector3Size, stateHorizon));
            this->initialGuess.com = &this->vectorizedOptiInputs.back();

            this->vectorizedOptiInputs.push_back(casadi::DM::zeros(vector3Size, stateHorizon));
            this->initialGuess.angularMomentum = &this->vectorizedOptiInputs.back();
        }

        for (const auto& [key, contact] : this->output.contacts)
        {
            // The current position of the contact
            this->vectorizedOptiInputs.push_back(casadi::DM::zeros(vector3Size));
            this->controllerInputs.contacts[key].currentPosition
                = &this->vectorizedOptiInputs.back();

            // The nominal contact position is a parameter that regularize the solution
            this->vectorizedOptiInputs.push_back(casadi::DM::zeros(vector3Size, stateHorizon));
            this->controllerInputs.contacts[key].nominalPosition
                = &this->vectorizedOptiInputs.back();

            // The orientation is stored as a vectorized version of the rotation matrix
            this->vectorizedOptiInputs.push_back(casadi::DM::zeros(9, stateHorizon));
            this->controllerInputs.contacts[key].orientation = &this->vectorizedOptiInputs.back();

            // Maximum admissible contact force. It is expressed in the contact body frame
            this->vectorizedOptiInputs.push_back(casadi::DM::zeros(1, this->optiSettings.horizon));
            this->controllerInputs.contacts[key].isEnabled = &this->vectorizedOptiInputs.back();

            // Upper limit of the position of the contact. It is expressed in the contact body frame
            this->vectorizedOptiInputs.push_back(
                casadi::DM::zeros(vector3Size, this->optiSettings.horizon));
            this->controllerInputs.contacts[key].upperLimitPosition
                = &this->vectorizedOptiInputs.back();

            // Lower limit of the position of the contact. It is expressed in the contact body frame
            this->vectorizedOptiInputs.push_back(
                casadi::DM::zeros(vector3Size, this->optiSettings.horizon));
            this->controllerInputs.contacts[key].lowerLimitPosition
                = &this->vectorizedOptiInputs.back();

            if (this->optiSettings.isWarmStartEnabled)
            {
                this->vectorizedOptiInputs.push_back(casadi::DM::zeros(vector3Size, stateHorizon));
                this->initialGuess.contactsInitialGuess[key].contactLocation
                    = &this->vectorizedOptiInputs.back();

                for (const auto& corner : contact.corners)
                {
                    this->vectorizedOptiInputs.push_back(
                        casadi::DM::zeros(vector3Size, this->optiSettings.horizon));
                    this->initialGuess.contactsInitialGuess[key].contactForce.push_back(
                        &this->vectorizedOptiInputs.back());
                }
            }
        }

        if (this->isSqp())
        {
            auto addInput = [this](int rows, int cols) {
                this->vectorizedOptiInputs.push_back(casadi::DM::zeros(rows, cols));
                return &this->vectorizedOptiInputs.back();
            };
            const int horizon = this->optiSettings.horizon;

            this->sqpIterate.com = addInput(vector3Size, stateHorizon);
            this->sqpIterate.dcom = addInput(vector3Size, stateHorizon);
            this->sqpIterate.angularMomentum = addInput(vector3Size, stateHorizon);
            this->sqpIterate.comLinearizationPoint = addInput(vector3Size, stateHorizon);
            for (const auto& [key, contact] : this->output.contacts)
            {
                auto& c = this->sqpIterate.contacts[key];
                c.position = addInput(vector3Size, stateHorizon);
                c.linearVelocity = addInput(vector3Size, horizon);
                for (std::size_t i = 0; i < contact.corners.size(); i++)
                {
                    c.force.push_back(addInput(vector3Size, horizon));
                }
                c.positionLinearizationPoint = addInput(vector3Size, stateHorizon);
                for (std::size_t i = 0; i < contact.corners.size(); i++)
                {
                    c.forceLinearizationPoint.push_back(addInput(vector3Size, horizon));
                }
            }
        }

        assert(vectorizedOptiInputsSize
               == this->vectorizedOptiInputs.size() + (this->isSqp() ? 1 : 0));
    }

    void populateOptiVariables()
    {
        constexpr int vector3Size = 3;
        const int horizon = this->optiSettings.horizon;
        const int stateHorizon = horizon + 1;
        const bool usePreviousForceState = this->optiSettings.solverName == "fatrop";

        // the casadi contacts depends on the maximum number of contacts
        for (const auto& [key, contact] : this->output.contacts)
        {
            auto [contactIt, outcome]
                = this->optiVariables.contacts.insert_or_assign(key,
                                                                CasadiContactWithConstraints(key));

            auto& c = contactIt->second;

            // each contact has a different number of corners
            c.corners.resize(contact.corners.size());

            // the orientation is a parameter. The orientation is stored as a vectorized version of
            // the rotation matrix. The last column is used only to express the bounding box of the
            // contact position at the end of the horizon.
            c.orientation = this->opti.parameter(9, stateHorizon);

            // Upper limit of the position of the contact. It is expressed in the contact body frame
            c.upperLimitPosition = this->opti.parameter(vector3Size, horizon);

            // Lower limit of the position of the contact. It is expressed in the contact body frame
            c.lowerLimitPosition = this->opti.parameter(vector3Size, horizon);

            // Maximum admissible contact force. It is expressed in the contact body frame
            c.isEnabled = this->opti.parameter(1, horizon);

            // The nominal contact position is a parameter that regularize the solution
            c.nominalPosition = this->opti.parameter(vector3Size, stateHorizon);

            c.currentPosition = this->opti.parameter(vector3Size);

            for (int j = 0; j < contact.corners.size(); j++)
            {
                c.corners[j].position
                    = casadi::DM(std::vector<double>(contact.corners[j].position.data(),
                                                     contact.corners[j].position.data()
                                                         + contact.corners[j].position.size()));
            }
        }

        // The decision variables are created stage by stage, i.e., [x_0, u_0, x_1, u_1, ..., x_N]
        // where x_k contains the CoM, its velocity, the angular momentum and the contact positions
        // and u_k the contact velocities and forces. This ordering is required by structure
        // exploiting solvers (e.g., fatrop) and does not affect the others.
        std::vector<casadi::MX> com, dcom, angularMomentum;
        std::map<std::string, std::vector<casadi::MX>> position, linearVelocity;
        std::map<std::string, std::vector<std::vector<casadi::MX>>> force, previousForce;
        for (int k = 0; k < stateHorizon; k++)
        {
            com.push_back(this->opti.variable(vector3Size));
            dcom.push_back(this->opti.variable(vector3Size));
            angularMomentum.push_back(this->opti.variable(vector3Size));
            for (const auto& [key, contact] : this->optiVariables.contacts)
            {
                position[key].push_back(this->opti.variable(vector3Size));
            }

            // fatrop requires a stage-wise separable cost. The force of the previous stage is
            // added to the state to express the rate of change of the force.
            if (usePreviousForceState && k > 0 && k < horizon)
            {
                for (const auto& [key, contact] : this->optiVariables.contacts)
                {
                    previousForce[key].resize(contact.corners.size());
                    for (auto& cornerForce : previousForce[key])
                    {
                        cornerForce.push_back(this->opti.variable(vector3Size));
                    }
                }
            }

            if (k == horizon)
            {
                break;
            }

            for (const auto& [key, contact] : this->optiVariables.contacts)
            {
                linearVelocity[key].push_back(this->opti.variable(vector3Size));
                force[key].resize(contact.corners.size());
                for (auto& cornerForce : force[key])
                {
                    cornerForce.push_back(this->opti.variable(vector3Size));
                }
            }
        }

        this->optiVariables.com = casadi::MX::horzcat(com);
        this->optiVariables.dcom = casadi::MX::horzcat(dcom);
        this->optiVariables.angularMomentum = casadi::MX::horzcat(angularMomentum);
        for (auto& [key, c] : this->optiVariables.contacts)
        {
            c.position = casadi::MX::horzcat(position[key]);
            c.linearVelocity = casadi::MX::horzcat(linearVelocity[key]);
            for (int j = 0; j < c.corners.size(); j++)
            {
                c.corners[j].force = casadi::MX::horzcat(force[key][j]);
                if (usePreviousForceState)
                {
                    c.corners[j].previousForce = casadi::MX::horzcat(previousForce[key][j]);
                }
            }
        }

        this->optiVariables.comCurrent = this->opti.parameter(vector3Size);
        this->optiVariables.dcomCurrent = this->opti.parameter(vector3Size);
        this->optiVariables.angularMomentumCurrent = this->opti.parameter(vector3Size);
        this->optiVariables.comReference = this->opti.parameter(vector3Size, stateHorizon);
        this->optiVariables.angularMomentumReference
            = this->opti.parameter(vector3Size, stateHorizon);
        this->optiVariables.externalForce = this->opti.parameter(vector3Size, //
                                                                 this->optiSettings.horizon);
        this->optiVariables.externalTorque = this->opti.parameter(vector3Size, //
                                                                  this->optiSettings.horizon);
        this->optiVariables.gravity = this->opti.parameter(vector3Size);

        if (this->isSqp())
        {
            this->optiVariables.comLinearizationPoint
                = this->opti.parameter(vector3Size, stateHorizon);
            for (auto& [key, c] : this->optiVariables.contacts)
            {
                c.positionLinearizationPoint = this->opti.parameter(vector3Size, stateHorizon);
                for (auto& corner : c.corners)
                {
                    corner.forceLinearizationPoint = this->opti.parameter(vector3Size, horizon);
                }
            }
        }
    }

    /**
     * Setup the optimization problem options
     */
    void setupOptiOptions()
    {
        casadi::Dict casadiOptions;
        casadi::Dict solverOptions;
        if (this->optiSettings.solverName == "ipopt")
        {
            if (this->optiSettings.solverVerbosity != 0)
            {
                casadi_int ipoptVerbosity
                    = static_cast<long long>(optiSettings.solverVerbosity - 1);
                solverOptions["print_level"] = ipoptVerbosity;
                casadiOptions["print_time"] = true;
            } else
            {
                solverOptions["print_level"] = 0;
                casadiOptions["print_time"] = false;
            }

            solverOptions["max_iter"] = this->optiSettings.ipoptMaxIteration;
            solverOptions["tol"] = this->optiSettings.ipoptTolerance;
            solverOptions["linear_solver"] = this->optiSettings.ipoptLinearSolver;
            solverOptions["sb"] = "yes";
            casadiOptions["expand"] = true;
            casadiOptions["error_on_fail"] = true;

            this->opti.solver("ipopt", casadiOptions, solverOptions);
            return;
        }

        if (this->optiSettings.solverName == "fatrop")
        {
            solverOptions["print_level"] = this->optiSettings.solverVerbosity;
            solverOptions["tol"] = this->optiSettings.fatropTolerance;
            // the default value (1e2) leads to more iterations for this problem
            solverOptions["mu_init"] = 1e-1;

            casadiOptions["print_time"] = this->optiSettings.solverVerbosity != 0;
            casadiOptions["expand"] = true;
            casadiOptions["error_on_fail"] = true;
            casadiOptions["structure_detection"] = "auto";
            casadiOptions["fatrop"] = solverOptions;

            this->opti.solver("fatrop", casadiOptions);
            return;
        }

        // sqp: each subproblem is a convex QP solved by osqp
        casadi::Dict osqpOptions;
        osqpOptions["verbose"] = this->optiSettings.solverVerbosity != 0;
        osqpOptions["eps_abs"] = this->optiSettings.osqpTolerance;
        osqpOptions["eps_rel"] = this->optiSettings.osqpTolerance;
        osqpOptions["max_iter"] = this->optiSettings.osqpMaxIteration;
        osqpOptions["polish"] = true;

        casadiOptions["expand"] = true;
        casadiOptions["error_on_fail"] = false;
        casadiOptions["warm_start_primal"] = true;
        casadiOptions["warm_start_dual"] = true;
        this->opti.solver("osqp", casadiOptions, osqpOptions);
    }

    casadi::Function createController()
    {
        using Sl = casadi::Slice;

        if (this->isSqp())
        {
            this->opti = casadi::Opti("conic");
        }

        this->populateOptiVariables();

        // get the variables to simplify the readability
        auto& com = this->optiVariables.com;
        auto& dcom = this->optiVariables.dcom;
        auto& angularMomentum = this->optiVariables.angularMomentum;
        auto& externalForce = this->optiVariables.externalForce;
        auto& externalTorque = this->optiVariables.externalTorque;
        auto& gravity = this->optiVariables.gravity;
        auto& contacts = this->optiVariables.contacts;
        const int horizon = this->optiSettings.horizon;
        const bool usePreviousForceState = this->optiSettings.solverName == "fatrop";

        auto dynamics = this->ode();
        auto contactPositionError = this->contactPositionError();
        auto frictionConeConstraint = this->frictionConeConstraint();

        // The bounding box of the position at instant k is expressed in the frame of the contact at
        // the same instant. The limits are set to infinity by setContactPhaseList when the
        // constraint is redundant.
        // each constraint is labeled with a name and a stage to shift the multipliers in the sqp
        struct ConstraintBlock
        {
            casadi_int offset;
            casadi_int size;
        };
        std::map<std::pair<std::string, int>, ConstraintBlock> constraintBlocks;
        casadi_int numberOfConstraints = 0;
        auto addConstraint = [&](const casadi::MX& constraint,
                                 casadi_int size,
                                 const std::string& name,
                                 int k) {
            this->opti.subject_to(constraint);
            constraintBlocks[{name, k}] = {numberOfConstraints, size};
            numberOfConstraints += size;
        };

        auto addBoundingBoxConstraint = [&](const CasadiContactWithConstraints& contact, int k) {
            auto error = contactPositionError({contact.position(Sl(), k),
                                               contact.nominalPosition(Sl(), k),
                                               contact.orientation(Sl(), k)});
            addConstraint(contact.lowerLimitPosition(Sl(), k - 1) <= error[0]
                              <= contact.upperLimitPosition(Sl(), k - 1),
                          error[0].numel(),
                          "bounding_box_" + contact.contactName,
                          k);
        };

        // The constraints are added stage by stage, i.e., [dynamics_0, path_0, dynamics_1, ...],
        // as required by structure exploiting solvers (e.g., fatrop).
        for (int k = 0; k < horizon; k++)
        {
            std::vector<casadi::MX> odeInput{externalForce(Sl(), k),
                                             externalTorque(Sl(), k),
                                             com(Sl(), k),
                                             dcom(Sl(), k),
                                             angularMomentum(Sl(), k),
                                             gravity};
            if (this->isSqp())
            {
                odeInput.push_back(this->optiVariables.comLinearizationPoint(Sl(), k));
            }
            for (const auto& [key, contact] : contacts)
            {
                odeInput.push_back(contact.position(Sl(), k));
                odeInput.push_back(contact.orientation(Sl(), k));
                odeInput.push_back(contact.isEnabled(Sl(), k));
                odeInput.push_back(contact.linearVelocity(Sl(), k));
                if (this->isSqp())
                {
                    odeInput.push_back(contact.positionLinearizationPoint(Sl(), k));
                }
                for (const auto& corner : contact.corners)
                {
                    odeInput.push_back(corner.force(Sl(), k));
                    if (this->isSqp())
                    {
                        odeInput.push_back(corner.forceLinearizationPoint(Sl(), k));
                    }
                }
            }
            const auto next = dynamics(odeInput);

            // the order must match the one of the state variables
            std::vector<casadi::MX> gap{com(Sl(), k + 1) - next[0],
                                        dcom(Sl(), k + 1) - next[1],
                                        angularMomentum(Sl(), k + 1) - next[2]};
            std::size_t contactIndex = 3;
            for (const auto& [key, contact] : contacts)
            {
                gap.push_back(contact.position(Sl(), k + 1) - next[contactIndex++]);
            }
            if (usePreviousForceState && k + 1 < horizon)
            {
                for (const auto& [key, contact] : contacts)
                {
                    for (const auto& corner : contact.corners)
                    {
                        gap.push_back(corner.previousForce(Sl(), k) - corner.force(Sl(), k));
                    }
                }
            }
            const casadi::MX gapVector = casadi::MX::vertcat(gap);
            addConstraint(gapVector == 0, gapVector.numel(), "dynamics", k);

            if (k == 0)
            {
                // set the feedback
                addConstraint(this->optiVariables.comCurrent == com(Sl(), 0), 3, "com_0", 0);
                addConstraint(this->optiVariables.dcomCurrent == dcom(Sl(), 0), 3, "dcom_0", 0);
                addConstraint(this->optiVariables.angularMomentumCurrent
                                  == angularMomentum(Sl(), 0),
                              3,
                              "angular_momentum_0",
                              0);
                for (const auto& [key, contact] : contacts)
                {
                    addConstraint(contact.currentPosition == contact.position(Sl(), 0),
                                  3,
                                  key + "_0",
                                  0);
                }
            }

            for (const auto& [key, contact] : contacts)
            {
                if (k > 0)
                {
                    addBoundingBoxConstraint(contact, k);
                }

                // TODO please if you want to add heel to toe motion you should define a
                // contact.maximumNormalForce for each corner. At this stage is too premature.
                for (const auto& corner : contact.corners)
                {
                    addConstraint(frictionConeConstraint({contact.orientation(Sl(), k),
                                                          contact.isEnabled(Sl(), k),
                                                          corner.force(Sl(), k)})[0]
                                      <= 0,
                                  this->frictionCone.getA().rows(),
                                  "friction_cone_" + corner.cornerName,
                                  k);
                }
            }
        }

        for (const auto& [key, contact] : contacts)
        {
            addBoundingBoxConstraint(contact, horizon);
        }

        // create the cost function
        auto& comReference = this->optiVariables.comReference;
        auto& angularMomentumReference = this->optiVariables.angularMomentumReference;

        // (max - mix) * exp(-i) + min
        casadi::DM weightCoMZ = casadi::DM::zeros(1, com.columns());
        const double min = this->weights.com(2) / 2;
        for (int i = 0; i < com.columns(); i++)
        {
            weightCoMZ(Sl(), i) = (this->weights.com(2) - min) * std::exp(-i) + min;
        }

        casadi::MX cost
            = this->weights.angularMomentum
                  * casadi::MX::sumsqr(angularMomentum - angularMomentumReference)
              + this->weights.com(0) * casadi::MX::sumsqr(com(0, Sl()) - comReference(0, Sl()))
              + this->weights.com(1) * casadi::MX::sumsqr(com(1, Sl()) - comReference(1, Sl()))
              + casadi::MX::sumsqr(weightCoMZ * (com(2, Sl()) - comReference(2, Sl())));

        casadi::MX averageForce;
        for (const auto& [key, contact] : this->optiVariables.contacts)
        {
            cost += this->weights.contactPosition
                    * casadi::MX::sumsqr(contact.nominalPosition - contact.position);

            // The velocity of an active contact does not affect the problem. Penalizing it keeps
            // the Hessian non-singular without changing the solution.
            cost += casadi::MX::sumsqr(casadi::MX::repmat(contact.isEnabled, 3, 1)
                                       * contact.linearVelocity);

            averageForce = casadi::MX::vertcat(
                {contact.isEnabled * contact.corners[0].force(0, Sl()) / contact.corners.size(),
                 contact.isEnabled * contact.corners[0].force(1, Sl()) / contact.corners.size(),
                 contact.isEnabled * contact.corners[0].force(2, Sl()) / contact.corners.size()});
            for (int i = 1; i < contact.corners.size(); i++)
            {
                averageForce += casadi::MX::vertcat(
                    {contact.isEnabled * contact.corners[i].force(0, Sl()) / contact.corners.size(),
                     contact.isEnabled * contact.corners[i].force(1, Sl()) / contact.corners.size(),
                     contact.isEnabled * contact.corners[i].force(2, Sl())
                         / contact.corners.size()});
            }

            for (const auto& corner : contact.corners)
            {
                const casadi::MX forceRateOfChange
                    = usePreviousForceState
                          ? corner.force(Sl(), Sl(1, horizon)) - corner.previousForce
                          : casadi::MX::diff(corner.force.T()).T();

                cost += this->weights.contactForceSymmetry
                        * casadi::MX::sumsqr(corner.force - averageForce);

                cost += this->weights.forceRateOfChange(0)
                        * casadi::MX::sumsqr(forceRateOfChange(0, Sl()));
                cost += this->weights.forceRateOfChange(1)
                        * casadi::MX::sumsqr(forceRateOfChange(1, Sl()));
                cost += this->weights.forceRateOfChange(2)
                        * casadi::MX::sumsqr(forceRateOfChange(2, Sl()));
            }
        }

        this->opti.minimize(cost);

        this->setupOptiOptions();

        // prepare the casadi function
        std::vector<casadi::MX> input;
        std::vector<casadi::MX> output;
        std::vector<std::string> inputName;
        std::vector<std::string> outputName;

        auto concatenateInput
            = [&input, &inputName](const casadi::MX& inputVariable, std::string inputVariableName) {
                  input.push_back(inputVariable);
                  inputName.push_back(std::move(inputVariableName));
              };

        auto concatenateOutput = [&output, &outputName](const casadi::MX& outputVariable,
                                                        std::string outputVariableName) {
            output.push_back(outputVariable);
            outputName.push_back(std::move(outputVariableName));
        };

        concatenateInput(this->optiVariables.externalForce, "external_force");
        concatenateInput(this->optiVariables.externalTorque, "external_torque");
        concatenateInput(this->optiVariables.comCurrent, "com_current");
        concatenateInput(this->optiVariables.dcomCurrent, "dcom_current");
        concatenateInput(this->optiVariables.angularMomentumCurrent, "angular_momentum_current");
        concatenateInput(this->optiVariables.gravity, "gravity");

        concatenateInput(this->optiVariables.comReference, "com_reference");
        concatenateInput(this->optiVariables.angularMomentumReference,
                         "angular_momentum_reference");

        // if warm start is enabled we need to add the initial guess for the com and the angular
        // momentum
        if (this->optiSettings.isWarmStartEnabled)
        {
            concatenateInput(this->optiVariables.com, "com_warmstart");
            concatenateInput(this->optiVariables.angularMomentum, "angular_momentum_warmstart");
        }

        for (const auto& [key, contact] : this->optiVariables.contacts)
        {
            concatenateInput(contact.currentPosition, "contact_" + key + "_current_position");
            concatenateInput(contact.nominalPosition, "contact_" + key + "_nominal_position");
            concatenateInput(contact.orientation, "contact_" + key + "_orientation_input");
            concatenateInput(contact.isEnabled, "contact_" + key + "is_enable_in");
            concatenateInput(contact.upperLimitPosition,
                             "contact_" + key + "_upper_limit_position");
            concatenateInput(contact.lowerLimitPosition,
                             "contact_" + key + "_lower_limit_position");

            // if warm start is enabled we need to add the initial guess for the contact position
            // and the force
            if (this->optiSettings.isWarmStartEnabled)
            {
                concatenateInput(contact.position, "contact_" + key + "_position_warmstart");

                std::size_t cornerIndex = 0;
                for (const auto& corner : contact.corners)
                {
                    concatenateInput(corner.force,
                                     "contact_" + key + "_corner_" + std::to_string(cornerIndex)
                                         + "_force_warmstart");
                    cornerIndex++;
                }
            }

            concatenateOutput(contact.isEnabled, "contact_" + key + "_is_enable");
            concatenateOutput(contact.position, "contact_" + key + "_position");
            concatenateOutput(contact.orientation, "contact_" + key + "_orientation");

            std::size_t cornerIndex = 0;
            for (const auto& corner : contact.corners)
            {
                concatenateOutput(corner.force,
                                  "contact_" + key + "_corner_" + std::to_string(cornerIndex)
                                      + "_force");
                cornerIndex++;
            }
        }

        concatenateOutput(this->optiVariables.com, "com");
        concatenateOutput(this->optiVariables.dcom, "dcom");
        concatenateOutput(this->optiVariables.angularMomentum, "angular_momentum");

        // the order must match the one of resizeControllerInputs
        if (this->isSqp())
        {
            concatenateInput(this->optiVariables.com, "com_iterate");
            concatenateInput(this->optiVariables.dcom, "dcom_iterate");
            concatenateInput(this->optiVariables.angularMomentum, "angular_momentum_iterate");
            concatenateInput(this->optiVariables.comLinearizationPoint, "com_linearization_point");
            for (const auto& [key, contact] : this->optiVariables.contacts)
            {
                const std::string prefix = "contact_" + key;
                concatenateInput(contact.position, prefix + "_position_iterate");
                concatenateInput(contact.linearVelocity, prefix + "_linear_velocity_iterate");
                for (std::size_t i = 0; i < contact.corners.size(); i++)
                {
                    concatenateInput(contact.corners[i].force,
                                     prefix + "_corner_" + std::to_string(i) + "_force_iterate");
                }
                concatenateInput(contact.positionLinearizationPoint,
                                 prefix + "_position_linearization_point");
                for (std::size_t i = 0; i < contact.corners.size(); i++)
                {
                    concatenateInput(contact.corners[i].forceLinearizationPoint,
                                     prefix + "_corner_" + std::to_string(i)
                                         + "_force_linearization_point");
                }

                concatenateOutput(contact.linearVelocity, prefix + "_linear_velocity");
            }

            const casadi::MX multipliers = this->opti.lam_g();
            assert(multipliers.numel() == numberOfConstraints);
            this->vectorizedOptiInputs.push_back(casadi::DM::zeros(multipliers.sparsity()));
            this->sqpIterate.multipliers = &this->vectorizedOptiInputs.back();
            concatenateInput(multipliers, "multipliers_iterate");
            concatenateOutput(multipliers, "multipliers");

            this->multipliersShift.clear();
            for (const auto& [label, block] : constraintBlocks)
            {
                const auto next = constraintBlocks.find({label.first, label.second + 1});
                if (next != constraintBlocks.end() && next->second.size == block.size)
                {
                    this->multipliersShift.push_back(
                        {block.offset, next->second.offset, block.size});
                }
            }
        }

        casadi::Dict toFunctionOptions, jitOptions;
        if constexpr (casadiVersionIsAtLeast360)
        {
            toFunctionOptions["cse"] = this->optiSettings.isCseEnabled;
        }

        return this->opti
            .to_function("controller", input, output, inputName, outputName, toFunctionOptions);
    }

    /**
     * Initialize the sqp iterate with the references, the nominal contacts and gravity-compensating
     * contact forces.
     */
    void initializeSqpIterate()
    {
        using namespace BipedalLocomotion::Conversions;
        const double dT = chronoToSeconds(this->optiSettings.samplingTime);
        const int horizon = this->optiSettings.horizon;

        toEigen(*this->sqpIterate.com) = toEigen(*this->controllerInputs.comReference);
        auto dcom = toEigen(*this->sqpIterate.dcom);
        const auto comReference = toEigen(*this->controllerInputs.comReference);
        dcom.leftCols(horizon) = (comReference.rightCols(horizon) - comReference.leftCols(horizon)) / dT;
        dcom.rightCols<1>() = dcom.col(horizon - 1);
        toEigen(*this->sqpIterate.angularMomentum)
            = toEigen(*this->controllerInputs.angularMomentumReference);

        Eigen::VectorXd numberOfActiveCorners = Eigen::VectorXd::Zero(horizon);
        for (const auto& [key, contact] : this->output.contacts)
        {
            numberOfActiveCorners += static_cast<double>(contact.corners.size())
                                     * toEigen(*this->controllerInputs.contacts[key].isEnabled)
                                           .transpose();
        }

        const Eigen::Vector3d gravity = toEigen(*this->controllerInputs.gravity);
        for (auto& [key, c] : this->sqpIterate.contacts)
        {
            const auto isEnabled = toEigen(*this->controllerInputs.contacts[key].isEnabled);
            toEigen(*c.position) = toEigen(*this->controllerInputs.contacts[key].nominalPosition);
            toEigen(*c.linearVelocity).setZero();
            toEigen(*this->sqpIterate.multipliers).setZero();
            for (auto* force : c.force)
            {
                auto f = toEigen(*force);
                for (int k = 0; k < horizon; k++)
                {
                    f.col(k) = (isEnabled(k) > 0.5 && numberOfActiveCorners(k) > 0)
                                   ? Eigen::Vector3d(-gravity / numberOfActiveCorners(k))
                                   : Eigen::Vector3d::Zero();
                }
            }
        }
        this->sqpIterate.isValid = true;
    }

    void setSqpLinearizationPoint()
    {
        *this->sqpIterate.comLinearizationPoint = *this->sqpIterate.com;
        for (auto& [key, c] : this->sqpIterate.contacts)
        {
            *c.positionLinearizationPoint = *c.position;
            for (std::size_t i = 0; i < c.force.size(); i++)
            {
                *c.forceLinearizationPoint[i] = *c.force[i];
            }
        }
    }

    /**
     * Store the solution of the QP as new iterate.
     * @return the infinity norm of the step or a negative number if the solution is not valid.
     */
    double updateSqpIterate(const std::vector<casadi::DM>& solution)
    {
        using namespace BipedalLocomotion::Conversions;
        double step = 0;
        auto update = [&step](casadi::DM& iterate, const casadi::DM& value) {
            step = std::max(step, (toEigen(iterate) - toEigen(value)).lpNorm<Eigen::Infinity>());
            iterate = value;
        };

        // the order is the one of the outputs in createController
        auto it = solution.cbegin();
        for (auto& [key, c] : this->sqpIterate.contacts)
        {
            std::advance(it, 1); // is enabled
            update(*c.position, *it++);
            std::advance(it, 1); // orientation
            for (auto* force : c.force)
            {
                update(*force, *it++);
            }
        }
        update(*this->sqpIterate.com, *it++);
        update(*this->sqpIterate.dcom, *it++);
        update(*this->sqpIterate.angularMomentum, *it++);
        for (auto& [key, c] : this->sqpIterate.contacts)
        {
            update(*c.linearVelocity, *it++);
        }
        *this->sqpIterate.multipliers = *it;

        return std::isfinite(step) ? step : -1;
    }

    /**
     * Shift the iterate of one sample to warm start the next control cycle.
     */
    void shiftSqpIterate()
    {
        using namespace BipedalLocomotion::Conversions;
        auto shift = [](casadi::DM& value) {
            auto v = toEigen(value);
            const int n = v.cols() - 1;
            v.leftCols(n) = v.rightCols(n).eval();
        };

        shift(*this->sqpIterate.com);
        shift(*this->sqpIterate.dcom);
        shift(*this->sqpIterate.angularMomentum);
        for (auto& [key, c] : this->sqpIterate.contacts)
        {
            shift(*c.position);
            shift(*c.linearVelocity);
            for (auto* force : c.force)
            {
                shift(*force);
            }
        }

        auto multipliers = toEigen(*this->sqpIterate.multipliers);
        const Eigen::VectorXd previousMultipliers = multipliers;
        for (const auto& block : this->multipliersShift)
        {
            multipliers.middleRows(block.destination, block.size)
                = previousMultipliers.middleRows(block.source, block.size);
        }
    }

    bool solveSqp(std::vector<casadi::DM>& solution)
    {
        // when the iterate is (re)initialized the sqp runs until convergence to get a good
        // linearization point for the next control cycles
        constexpr int maxNumberOfQPIterationsAtInitialization = 20;
        int numberOfQPIterations = this->optiSettings.numberOfQPIterations;
        if (!this->sqpIterate.isValid)
        {
            this->initializeSqpIterate();
            numberOfQPIterations
                = std::max(numberOfQPIterations, maxNumberOfQPIterationsAtInitialization);
        }

        for (int i = 0; i < numberOfQPIterations; i++)
        {
            this->setSqpLinearizationPoint();
            solution = this->controller(this->vectorizedOptiInputs);
            const double step = this->updateSqpIterate(solution);
            if (step < 0)
            {
                this->sqpIterate.isValid = false;
                return false;
            }
            if (step < this->optiSettings.sqpTolerance)
            {
                break;
            }
        }

        return true;
    }
};

bool CentroidalMPC::initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler)
{
    constexpr auto errorPrefix = "[CentroidalMPC::initialize]";
    auto ptr = handler.lock();

    if (ptr == nullptr)
    {
        log()->error("{} The parameter handler is not valid.", errorPrefix);
        return false;
    }

    if (!m_pimpl->loadParameters(ptr))
    {
        log()->error("{} Unable to load the parameters.", errorPrefix);
        return false;
    }

    m_pimpl->resizeControllerInputs();
    m_pimpl->controller = m_pimpl->createController();
    m_pimpl->fsm = Impl::FSM::Initialized;

    return true;
}

CentroidalMPC::~CentroidalMPC() = default;

CentroidalMPC::CentroidalMPC()
{
    m_pimpl = std::make_unique<Impl>();
}

const CentroidalMPCOutput& CentroidalMPC::getOutput() const
{
    return m_pimpl->output;
}

bool CentroidalMPC::isOutputValid() const
{
    return m_pimpl->fsm == Impl::FSM::OutputValid;
}

bool CentroidalMPC::advance()
{
    constexpr auto errorPrefix = "[CentroidalMPC::advance]";
    assert(m_pimpl);

    using Sl = casadi::Slice;

    if (m_pimpl->fsm == Impl::FSM::Idle)
    {
        log()->error("{} The controller is not initialized please call initialize() method.",
                     errorPrefix);
        return false;
    }

    // invalidate the output
    m_pimpl->fsm = Impl::FSM::OutputInvalid;

    // compute the output
    std::vector<casadi::DM> controllerOutput;
    try
    {
        SingleThreadedOpenMPScope singleThreadedScope;
        if (m_pimpl->isSqp())
        {
            if (!m_pimpl->solveSqp(controllerOutput))
            {
                log()->error("{} The sqp returned an invalid solution.", errorPrefix);
                return false;
            }
        } else
        {
            controllerOutput = m_pimpl->controller(m_pimpl->vectorizedOptiInputs);
        }
    } catch (const std::exception& e)
    {
        log()->error("{} Unable to solve the problem. The following exception has been thrown {}.",
                     errorPrefix,
                     e.what());
        return false;
    }

    // get the solution
    auto it = controllerOutput.begin();

    ContactListMap contactListMap = m_pimpl->output.contactPhaseList.lists();
    for (auto& [key, contact] : m_pimpl->output.contacts)
    {
        auto contactListIt = contactListMap.find(key);
        if (contactListIt == contactListMap.end())
        {
            log()->error("{} Unable to find the contact list named {}. Please call "
                         "setContactPhaseList() before advance().",
                         errorPrefix,
                         key);
            return false;
        }
        ContactList& contactList = contactListIt->second;

        // this is required for toEigen
        using namespace BipedalLocomotion::Conversions;

        int index = toEigen(*it).size();
        const int size = toEigen(*it).size();
        for (int i = 0; i < size; i++)
        {
            // read it as: "if the contact is active at a given time instant"
            if (toEigen(*it)(i) > 0.5)
            {
                // if the contact is active now
                if (i == 0)
                {
                    break;
                } // in this case we break if the contact is active and at the previous time
                  // step it was not active
                else if (toEigen(*it)(i - 1) < 0.5)
                {
                    index = i;
                    break;
                }
            }
        }

        // check if now we are in contact
        const double isEnabled = toEigen(*it)(0);

        /// Position
        std::advance(it, 1);
        contact.pose.translation(toEigen(*it).leftCols<1>());

        // In this case the contact is not active and there will be a next planned contact
        if (index < size)
        {
            const std::chrono::nanoseconds nextPlannedContactTime
                = m_pimpl->currentTime + m_pimpl->optiSettings.samplingTime * index;

            auto nextPlannedContact = contactList.getPresentContact(nextPlannedContactTime);
            if (nextPlannedContact == contactList.end())
            {
                log()->error("[CentroidalMPC::advance] Unable to get the next planned contact");
                return false;
            }

            PlannedContact modifiedNextPlannedContact = *nextPlannedContact;

            // only the position is modified by the MPC
            modifiedNextPlannedContact.pose.translation(toEigen(*it).col(index));

            if (!contactList.editContact(nextPlannedContact, modifiedNextPlannedContact))
            {
                log()->error("{} Unable to edit the next planned contact at time {}. The contact "
                             "list contains the following contacts: {}",
                             errorPrefix,
                             std::chrono::duration_cast<std::chrono::milliseconds>(
                                 nextPlannedContactTime),
                             contactList.toString());
                return false;
            }
        }

        std::advance(it, 1);

        // get the orientation
        contact.pose.quat(Eigen::Quaterniond(
            Eigen::Map<const Eigen::Matrix3d>(toEigen(*it).leftCols<1>().data())));

        // get the forces
        std::advance(it, 1);

        for (std::size_t cornerIndex = 0; cornerIndex < contact.corners.size(); cornerIndex++)
        {
            if (m_pimpl->optiSettings.isWarmStartEnabled)
            {
                toEigen(*m_pimpl->initialGuess.contactsInitialGuess[key].contactForce[cornerIndex])
                    .leftCols(m_pimpl->optiSettings.horizon - 1)
                    = toEigen(*it).rightCols(m_pimpl->optiSettings.horizon - 1);
                toEigen(*m_pimpl->initialGuess.contactsInitialGuess[key].contactForce[cornerIndex])
                    .rightCols<1>()
                    = toEigen(*it).rightCols<1>();
            }
            if (isEnabled > 0.5)
            {
                contact.corners[cornerIndex].force = toEigen(*it).leftCols<1>();
            } else
            {
                contact.corners[cornerIndex].force.setZero();
            }

            std::advance(it, 1);
        }
    }

    // update the contact phase list
    m_pimpl->output.contactPhaseList.setLists(contactListMap);

    for (int i = 0; i < m_pimpl->output.comTrajectory.size(); i++)
    {
        using namespace BipedalLocomotion::Conversions;
        m_pimpl->output.comTrajectory[i] = toEigen(*it).col(i);
    }

    std::advance(it, 1);
    for (int i = 0; i < m_pimpl->output.comVelocityTrajectory.size(); i++)
    {
        using namespace BipedalLocomotion::Conversions;
        m_pimpl->output.comVelocityTrajectory[i] = toEigen(*it).col(i);
    }

    std::advance(it, 1);
    for (int i = 0; i < m_pimpl->output.angularMomentumTrajectory.size(); i++)
    {
        using namespace BipedalLocomotion::Conversions;
        m_pimpl->output.angularMomentumTrajectory[i] = toEigen(*it).col(i);
    }

    if (m_pimpl->isSqp())
    {
        m_pimpl->shiftSqpIterate();
    }

    // advance the time
    m_pimpl->currentTime += m_pimpl->optiSettings.samplingTime;

    // Make the output valid
    m_pimpl->fsm = Impl::FSM::OutputValid;

    return true;
}

bool CentroidalMPC::setReferenceTrajectory(const std::vector<Eigen::Vector3d>& com,
                                           const std::vector<Eigen::Vector3d>& angularMomentum)
{
    constexpr auto errorPrefix = "[CentroidalMPC::setReferenceTrajectory]";

    const int stateHorizon = m_pimpl->optiSettings.horizon + 1;

    if (m_pimpl->fsm == Impl::FSM::Idle)
    {
        log()->error("{} The controller is not initialized please call initialize() method.",
                     errorPrefix);
        return false;
    }

    if (com.size() < stateHorizon)
    {
        log()->error("{} The CoM trajectory vector should have at least {} elements. Provided "
                     "size: {}.",
                     errorPrefix,
                     stateHorizon,
                     com.size());
        return false;
    }

    if (angularMomentum.size() < stateHorizon)
    {
        log()->error("{} The angular momentum trajectory vector should have at least {} elements. "
                     "Provided size: {}.",
                     errorPrefix,
                     stateHorizon,
                     angularMomentum.size());
        return false;
    }

    // Since Eigen vector is a contiguous we can copy the CoM and the angular momentum references by
    // columns.
    for (int i = 0; i < stateHorizon; i++)
    {
        using namespace BipedalLocomotion::Conversions;
        toEigen(*(m_pimpl->controllerInputs.comReference)).col(i)
            = Eigen::Map<const Eigen::Vector3d>(com[i].data());
        toEigen(*(m_pimpl->controllerInputs.angularMomentumReference)).col(i)
            = Eigen::Map<const Eigen::Vector3d>(angularMomentum[i].data());
    }

    // if the warmstart is enabled then the reference is used also as warmstart
    if (m_pimpl->optiSettings.isWarmStartEnabled)
    {
        using namespace BipedalLocomotion::Conversions;
        toEigen(*(m_pimpl->initialGuess.com)) = toEigen(*(m_pimpl->controllerInputs.comReference));
        toEigen(*(m_pimpl->initialGuess.angularMomentum))
            = toEigen(*(m_pimpl->controllerInputs.angularMomentumReference));
    }

    return true;
}

bool CentroidalMPC::setGravity(Eigen::Ref<const Eigen::Vector3d> gravity)
{
    constexpr auto errorPrefix = "[CentroidalMPC::setGravity]";
    assert(m_pimpl);

    if (m_pimpl->fsm == Impl::FSM::Idle)
    {
        log()->error("{} The controller is not initialized please call initialize() method.",
                     errorPrefix);
        return false;
    }

    auto& inputs = m_pimpl->controllerInputs;

    using namespace BipedalLocomotion::Conversions;
    toEigen(*inputs.gravity) = gravity;

    return true;
}

bool CentroidalMPC::setState(Eigen::Ref<const Eigen::Vector3d> com,
                             Eigen::Ref<const Eigen::Vector3d> dcom,
                             Eigen::Ref<const Eigen::Vector3d> angularMomentum)
{
    const Math::Wrenchd dummy = Math::Wrenchd::Zero();
    return this->setState(com, dcom, angularMomentum, dummy);
}

bool CentroidalMPC::setState(Eigen::Ref<const Eigen::Vector3d> com,
                             Eigen::Ref<const Eigen::Vector3d> dcom,
                             Eigen::Ref<const Eigen::Vector3d> angularMomentum,
                             const Math::Wrenchd& externalWrench)
{
    Eigen::Vector3d gravity = Eigen::Vector3d::Zero();
    gravity[2] = -BipedalLocomotion::Math::StandardAccelerationOfGravitation;
    return this->setState(com, dcom, angularMomentum, externalWrench, gravity);
}

bool CentroidalMPC::setState(Eigen::Ref<const Eigen::Vector3d> com,
                             Eigen::Ref<const Eigen::Vector3d> dcom,
                             Eigen::Ref<const Eigen::Vector3d> angularMomentum,
                             const Math::Wrenchd& externalWrench,
                             Eigen::Ref<const Eigen::Vector3d> gravity)
{
    constexpr auto errorPrefix = "[CentroidalMPC::setState]";
    assert(m_pimpl);

    if (m_pimpl->fsm == Impl::FSM::Idle)
    {
        log()->error("{} The controller is not initialized please call initialize() method.",
                     errorPrefix);
        return false;
    }

    auto& inputs = m_pimpl->controllerInputs;

    using namespace BipedalLocomotion::Conversions;
    toEigen(*inputs.comCurrent) = com;
    toEigen(*inputs.dcomCurrent) = dcom;
    toEigen(*inputs.angularMomentumCurrent) = angularMomentum;

    toEigen(*inputs.externalForce).setZero();
    toEigen(*inputs.externalTorque).setZero();

    toEigen(*inputs.externalForce).leftCols<1>() = externalWrench.force();
    toEigen(*inputs.externalTorque).leftCols<1>() = externalWrench.torque();

    toEigen(*inputs.gravity) = gravity;

    return true;
}

bool CentroidalMPC::setContactPhaseList(const Contacts::ContactPhaseList& contactPhaseList)
{
    constexpr auto errorPrefix = "[CentroidalMPC::setContactPhaseList]";
    assert(m_pimpl);

    if (m_pimpl->fsm == Impl::FSM::Idle)
    {
        log()->error("{} The controller is not initialized please call initialize() method.",
                     errorPrefix);
        return false;
    }

    if (contactPhaseList.size() == 0)
    {
        log()->error("{} The contactPhaseList is empty.", errorPrefix);
        return false;
    }

    for (const auto& [key, list] : contactPhaseList.lists())
    {
        if (!list.areContactsSampled(m_pimpl->optiSettings.samplingTime))
        {
            log()->error("{} The contact list {} is not sampled at the sampling time {}. Please "
                         "resample the contacts lists before calling this method.",
                         errorPrefix,
                         key,
                         std::chrono::duration_cast<std::chrono::milliseconds>(
                             m_pimpl->optiSettings.samplingTime));
            return false;
        }
    }

    m_pimpl->contactPhaseList = contactPhaseList;

    // The orientation is stored as a vectorized version of the rotation matrix
    const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();

    auto& inputs = m_pimpl->controllerInputs;

    // clear previous data
    for (const auto& [key, contact] : m_pimpl->output.contacts)
    {
        using namespace BipedalLocomotion::Conversions;

        // initialize the current contact pose to zero. If the contact is active the current
        // position will be set later on
        toEigen(*inputs.contacts[key].currentPosition).setZero();

        // initialize all the orientation to the identity
        toEigen(*inputs.contacts[key].orientation).colwise()
            = Eigen::Map<const Eigen::VectorXd>(identity.data(), identity.cols() * identity.rows());

        // Upper limit of the position of the contact. It is expressed in the contact body frame
        toEigen(*inputs.contacts[key].upperLimitPosition).setZero();

        // Lower limit of the position of the contact. It is expressed in the contact body frame
        toEigen(*inputs.contacts[key].lowerLimitPosition).setZero();

        // Maximum admissible contact force. It is expressed in the contact body frame
        toEigen(*inputs.contacts[key].isEnabled).setZero();

        // The nominal contact position is a parameter that regularize the solution
        toEigen(*inputs.contacts[key].nominalPosition).setZero();
    }

    const std::chrono::nanoseconds absoluteTimeHorizon
        = m_pimpl->currentTime + m_pimpl->optiSettings.timeHorizon;

    // find the contactPhase associated to the current time
    auto initialPhase = contactPhaseList.getPresentPhase(m_pimpl->currentTime);
    if (initialPhase == contactPhaseList.end())
    {
        log()->error("{} Unable to find the contact phase related to the current at time {}. The "
                     "contact "
                     "list contains the following contacts: {}",
                     errorPrefix,
                     std::chrono::duration_cast<std::chrono::milliseconds>(m_pimpl->currentTime),
                     contactPhaseList.toString());
        return false;
    }

    // find the contactPhase associated to the end time. getPresentPhase returns the latest phase
    // if the time is after the end of the list, and the initial phase exists, so this is valid.
    const auto finalPhase = contactPhaseList.getPresentPhase(absoluteTimeHorizon);

    int index = 0;
    for (auto it = initialPhase; it != std::next(finalPhase); std::advance(it, 1))
    {
        const std::chrono::nanoseconds tInitial = std::max(m_pimpl->currentTime, it->beginTime);

        // the final phase is extended up to the end of the horizon
        const std::chrono::nanoseconds tFinal
            = it == finalPhase ? absoluteTimeHorizon : std::min(absoluteTimeHorizon, it->endTime);

        const std::chrono::nanoseconds duration = tFinal - tInitial;
        const int numberOfSamples = duration / m_pimpl->optiSettings.samplingTime;

        for (const auto& [key, contact] : it->activeContacts)
        {
            using namespace BipedalLocomotion::Conversions;

            auto inputContact = inputs.contacts.find(key);
            if (inputContact == inputs.contacts.end())
            {
                log()->error("{} Unable to find the input contact named {}.", errorPrefix, key);
                return false;
            }

            toEigen(*(inputContact->second.nominalPosition))
                .middleCols(index, numberOfSamples + 1)
                .colwise()
                = contact->pose.translation();

            // this is required to reshape the matrix into a vector
            const Eigen::Matrix3d orientation = contact->pose.quat().toRotationMatrix();
            toEigen(*(inputContact->second.orientation))
                .middleCols(index, numberOfSamples + 1)
                .colwise()
                = Eigen::Map<const Eigen::VectorXd>(orientation.data(), orientation.size());

            constexpr double isEnabled = 1;
            toEigen(*(inputContact->second.isEnabled))
                .middleCols(index, numberOfSamples)
                .setConstant(isEnabled);
        }

        index += numberOfSamples;
    }

    assert(index == m_pimpl->optiSettings.horizon);

    // set the current contact position to for the active contact only
    for (auto& [key, contact] : inputs.contacts)
    {
        using namespace BipedalLocomotion::Conversions;

        toEigen(*contact.currentPosition) = toEigen(*contact.nominalPosition).leftCols<1>();

        // if warmstart is enabled the contact location is used as warmstart to initialize the
        // problem
        if (m_pimpl->optiSettings.isWarmStartEnabled)
        {
            toEigen(*(m_pimpl->initialGuess.contactsInitialGuess[key].contactLocation))
                = toEigen(*contact.nominalPosition);
        }
    }

    // The position of an active contact is constant, hence the bounding box is enforced only when
    // the contact is established (i.e., at the first active sample after a swing phase). In all
    // the other instants the constraint is either redundant or meaningless, and keeping it would
    // make the constraint Jacobian rank deficient.
    constexpr double infinity = std::numeric_limits<double>::infinity();
    const int horizon = m_pimpl->optiSettings.horizon;
    for (auto& [key, contact] : inputs.contacts)
    {
        using namespace BipedalLocomotion::Conversions;

        const auto& boundingBox = m_pimpl->contactBoundingBoxes.at(key);
        const auto isEnabled = toEigen(*contact.isEnabled);
        auto upperLimit = toEigen(*contact.upperLimitPosition);
        auto lowerLimit = toEigen(*contact.lowerLimitPosition);
        const bool isActiveAtTheEnd = finalPhase->activeContacts.count(key) > 0;

        for (int i = 0; i < horizon; i++)
        {
            const bool isActiveAtNextSample
                = (i + 1 < horizon) ? isEnabled(i + 1) > 0.5 : isActiveAtTheEnd;
            if (isEnabled(i) < 0.5 && isActiveAtNextSample)
            {
                upperLimit.col(i) = boundingBox.upperLimit;
                lowerLimit.col(i) = boundingBox.lowerLimit;
            } else
            {
                upperLimit.col(i).setConstant(infinity);
                lowerLimit.col(i).setConstant(-infinity);
            }
        }
    }

    // we store the contact phase list for the output
    m_pimpl->output.contactPhaseList = contactPhaseList;

    return true;
}
