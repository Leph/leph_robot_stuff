#ifndef LEPH_MODEL_INVERSEDYNAMICSSIMPLIFIED_HPP
#define LEPH_MODEL_INVERSEDYNAMICSSIMPLIFIED_HPP

#include <vector>
#include <map>
#include <string>
#include <iostream>
#include <Eigen/Dense>
#include <leph_model/Model.hpp>

namespace leph {

/**
 * InverseDynamicsSimplified
 *
 * Whole body Inverse Dynamics
 * using Quadratic Programming formulation
 * inspired from S. Feng et al. 2014 work.
 * ("Optimization Based Full Body Control 
 * for the Atlas Robot").
 * The robot's model is assumed to have a 
 * floating base on which no contact is applied.
 */
class InverseDynamicsSimplified
{
    public:
        
        /**
         * Initialization with a floating Model.
         * The model instance must be valid during
         * the whole use of InverseDynamics.
         *
         * @param model Reference to the 
         * model with floating base.
         */
        InverseDynamicsSimplified(Model& model);

        /**
         * Declare a named 6D contact plane 
         * constraint in world frame.
         *
         * @param nameFrame RBDL body name of the frame
         * whose center is contacting in world frame.
         * The 6 degrees of freedom of the contact point 
         * will be enforced to be fixed.
         * @param areaCoPMinX Center of pressure allowed 
         * minimum X bound expressed in local contact frame.
         * @param areaCoPMaxX Center of pressure allowed
         * maximum X bound expressed in local contact frame.
         * @param areaCoPMinY Center of pressure allowed
         * minimum Y bound expressed in local contact frame.
         * @param areaCoPMaxY Center of pressure allowed
         * maximum Y bound expressed in local contact frame.
         */
        void addContactPlane(
            const std::string& nameFrame,
            double areaCoPMinX,
            double areaCoPMaxX,
            double areaCoPMinY,
            double areaCoPMaxY);

        /**
         * Declare a named 3D contact point 
         * constraint in world frame.
         *
         * @param nameFrame RBDL body name of the frame
         * whose center is contacting in world frame.
         * The 3 position degrees of freedom of 
         * the contact point will be enforced to be fixed.
         */
        void addContactPoint(
            const std::string& nameFrame);

        /**
         * Declare the direct tracking (in acceleration 
         * level) of a joint.
         *
         * @param nameDOF Model name of tracked
         * angular joint.
         */
        void addTargetJoint(
            const std::string& nameJoint);

        /**
         * Declare a named kinematics target in Cartesian space.
         * Either 3d position or 3d orientation will be tracked
         * on specified frame's center in world frame.
         *
         * @param nameFrame RBDL body name of the frame
         * which kinematics state is enforced in world frame.
         */
        void addTargetPosition(
            const std::string& nameFrame);
        void addTargetOrientation(
            const std::string& nameFrame);

        /**
         * Declare the tracking of either the
         * linear or angular centroidal 
         * (center of mass) momentum withing
         * the cost term
         */
        void addTargetMomentumLinear();
        void addTargetMomentumAngular();

        /**
         * Assign regularization penalty over
         * optimized variables.
         * All weights must be positive or zero.
         *
         * @param penaltyAccBase Penalty weight
         * over linear floating base DOF accelerations.
         * @param penaltyAccJoint Penalty weight
         * over angular floating base and joints
         * DOF accelerations.
         */
        void setRegularization(
            double penaltyAccBase,
            double penaltyAccJoint);

        /**
         * Assign to a declared contact the 
         * associated friction coefficient for 
         * approximated friction pyramid constraint.
         *
         * @param nameContact Name of
         * previously declared contact RBDL frame.
         * @param frictionCoef Positive friction coefficient.
         */
        void setFrictionCoef(
            const std::string& nameContact,
            double frictionCoef);
        
        /**
         * Set or get to a declared point contact
         * the surface orientation
         *
         * @param nameContact Name of previously 
         * declared point contact RBDL frame.
         * @param surfaceMat Contact surface normal
         * orientation expressed in world frame.
         */
        void setContactPointOrientation(
            const std::string& nameContact,
            const Eigen::Matrix3d& surfaceMat);
        const Eigen::Matrix3d& getContactPointOrientation(
            const std::string& nameContact);

        /**
         * Set the desired contact target wrench and
         * associated weight for regularization.
         *
         * @param nameContact Name of previously 
         * declared point contact RBDL frame.
         * @param wrench Desired contact wrench. 
         * For point contact, only the linear part is used.
         * @param weightTorque Weighting applied on
         * all contact angular moments
         * @param weightForceTangential Weighting applied on
         * all tangential linear contact forces.
         * @param weightForceNormal Weighting applied on
         * all normal linear contact forces.
         */
        void setContactWrenchTarget(
            const std::string& nameContact,
            const Eigen::Vector6d& wrench);
        void setContactWrenchWeight(
            const std::string& nameContact,
            double weightTorque,
            double weightForceTangential,
            double weightForceNormal);

        /**
         * Set or get the minimum contact normal force limit.
         * Used as hard inequality constraint.
         *
         * @param nameContact Name of previously 
         * declared point contact RBDL frame.
         * @param forceMin Constraint for minimum
         * contact normal force.
         */
        void setContactMinimumNormalForce(
            const std::string& nameContact,
            double forceMin);
        double getContactMinimumNormalForce(
            const std::string& nameContact);

        /**
         * Assigned named target as well as 
         * centroidal momentum weight in cost minimization.
         *
         * @param nameTarget Name of
         * previously declared target RBDL frame.
         * @param weightLinear Positive weight
         * for contact or target position 
         * acceleration constraint.
         * @param weightAngular Positive weight
         * for contact or target orientation
         * acceleration constraint.
         */
        void setWeightJoint(
            const std::string& nameJoint,
            double weightAngular);
        void setWeightPosition(
            const std::string& nameTarget,
            double weightLinear);
        void setWeightOrientation(
            const std::string& nameTarget,
            double weightAngular);
        void setWeightMomentumLinear(
            double weightMomentumLinear);
        void setWeightMomentumAngular(
            double weightMomentumAngular);

        /**
         * Reset and initialize internal 
         * data structure and set torque limits
         * to model (URDF) limits.
         */
        void init();

        /**
         * Enable or disable a declared
         * named contact for dynamics computation.
         *
         * @param nameContact Name of
         * previously declared contact RBDL frame.
         * @param isEnabled If true, the
         * given contact is enable, else
         * it is disable.
         */
        void setContact(
            const std::string& nameContact,
            bool isEnabled);

        /**
         * Directly assign target joint angular
         * acceleration.
         *
         * @param nameJoint Previously declared
         * model joint name.
         * @param acc Target angular joint acceleration.
         */
        void setEffortJoint(
            const std::string& nameJoint,
            double acc);

        /**
         * Directly assign target linear or angular
         * Cartesian acceleration of target.
         *
         * @param nameTarget Name of previously
         * declared target.
         * @param acc Linear or angular acceleration 
         * vector of frame's center expressed 
         * in world frame.
         */
        void setEffortPosition(
            const std::string& nameTarget,
            const Eigen::Vector3d& acc);
        void setEffortOrientation(
            const std::string& nameTarget,
            const Eigen::Vector3d& acc);

        /**
         * Directly assign target linear or angular
         * rate of change of centroidal momentum.
         *
         * @param momentumDiff Linear or angular
         * momentum rate of change vector.
         */
        void setEffortMomentumLinear(
            const Eigen::Vector3d& momentumDiff);
        void setEffortMomentumAngular(
            const Eigen::Vector3d& momentumDiff);

        /**
         * Compute and assign target joint 
         * acceleration with PD control.
         * WARNING: The internal model's position and 
         * velocity state are used and are 
         * assumed to be updated.
         *
         * @param nameJoint Previously declared
         * model joint name.
         * @param gainErrorPos P gain over position error.
         * @param targetPos Desired angular joint position.
         * @param gainErrorVel D gain over velocity error.
         * @param targetVel Desired angular joint velocity.
         * @param targetAcc Optional desired angular joint
         * acceleration as feedforward term.
         * @return computed acceleration effort (which is also
         * assigned to target effort).
         */
        double computeEffortJoint(
            const std::string& nameJoint,
            double gainErrorPos,
            double targetPos,
            double gainErrorVel,
            double targetVel,
            double targetAcc = 0.0);

        /**
         * Compute and assign target linear or 
         * angular Cartesian acceleration with PD control.
         * WARNING: The internal model's position and 
         * velocity state are used and are 
         * assumed to be updated.
         *
         * @param nameTarget Name of previously
         * declared target.
         * @param gainErrorPos P gain over position error.
         * @param targetPos Desired Cartesian position vector 
         * of frame expressed in world frame.
         * @param targetMat Desired Cartesian orientation matrix 
         * of frame expressed in world frame.
         * @param gainErrorVel D gain over velocity error.
         * @param targetVel Desired Cartesian linear or angular
         * velocity vector of frame expressed in world frame.
         * @param targetAcc Optional desired Cartesian linear or
         * angular acceleration of frame expressed in world 
         * frame as feedforward term.
         * @return computed acceleration effort (which is also
         * assigned to target effort).
         */
        const Eigen::Vector3d& computeEffortPosition(
            const std::string& nameTarget,
            double gainErrorPos,
            const Eigen::Vector3d& targetPos,
            double gainErrorVel,
            const Eigen::Vector3d& targetVel,
            const Eigen::Vector3d& targetAcc = Eigen::Vector3d::Zero());
        const Eigen::Vector3d& computeEffortOrientation(
            const std::string& nameTarget,
            double gainErrorPos,
            const Eigen::Matrix3d& targetMat,
            double gainErrorVel,
            const Eigen::Vector3d& targetVel,
            const Eigen::Vector3d& targetAcc = Eigen::Vector3d::Zero());

        /**
         * Compute and assign target linear or 
         * angular centroidal momentum rate of change
         * with PD control.
         * WARNING: The internal model's position and 
         * velocity state are used and are 
         * assumed to be updated.
         *
         * @param gainErrorPos P gain over position error.
         * @param targetCoMPos Desired Cartesian position 
         * vector of center of mass expressed in world frame.
         * @param gainErrorVel D gain over velocity error.
         * @param targetCoMVel Desired Cartesian velocity
         * vector of center of mass expressed in world frame.
         * @param targetCoMAcc Optional desired Cartesian
         * acceleration of center of mass as feedforward term
         * expressed in world frame.
         * @param targetMomentumAngular Desired CoM angular 
         * momentum expressed in floating base frame.
         * @return computed momentum rate of change effort 
         * (which is also assigned to target effort).
         */
        const Eigen::Vector3d& computeEffortMomentumLinear(
            double gainErrorPos,
            const Eigen::Vector3d& targetCoMPos,
            double gainErrorVel,
            const Eigen::Vector3d& targetCoMVel,
            const Eigen::Vector3d& targetCoMAcc = Eigen::Vector3d::Zero());
        const Eigen::Vector3d& computeEffortMomentumAngular(
            double gainErrorVel,
            const Eigen::Vector3d& targetMomentumAngular,
            const Eigen::Vector3d& targetMomentumAcc = Eigen::Vector3d::Zero());

        /**
         * Set dynamics optional parameters.
         * (see member comments)
         */
        void setUseNoVelocityForDynamics(bool flag);
        void setCoefNonDiagonalMassMatrix(double coef);
        void setDisableInequalityConstraints(bool flag);

        /**
         * Read and write access to internal base and joint
         * torque bias user defined vector
         */
        const Eigen::VectorXd& refBiasTorques() const;
        Eigen::VectorXd& refBiasTorques();

        /**
         * Run the QP optimization process.
         * WARNING: The internal model's position and 
         * velocity state are used and are 
         * assumed to be set and updated.
         *
         * @return true if optimization 
         * has been successful and internal 
         * computed solution is valid.
         */
        bool run();

        /**
         * Read only access to last computed 
         * accelerations, torques and contact plane
         * wrenches or contact point forces QP solution.
         *
         * @param nameContact User name of a defined 
         * contact RBDL frame.
         * @return access to degrees of freedom acceleration 
         * (sizeVectVel()), joint torque vector (sizeJoint()) 
         * and wrench (6d) or forces (3d) expressed in local frame 
         * at given plane or point contact point.
         * If contact is not enabled, zero wrench 
         * vector is returned.
         */
        Eigen::VectorBlock<const Eigen::VectorXd, -1> accelerations() const;
        Eigen::VectorBlock<const Eigen::VectorXd, -1> torques() const;
        Eigen::VectorBlock<const Eigen::VectorXd, -1> contactWrench(
            const std::string& nameContact) const;
        Eigen::VectorBlock<const Eigen::VectorXd, -1> contactForce(
            const std::string& nameContact) const;

        /**
         * Compute and return the generated joint torques split
         * according to the equation of motion in the
         * feedback part (accelerations and mass matrix) 
         * and the feedforward part (gravity and contact forces).
         * @return partial degree of freedom torque 
         * vectors (sizeVectVel()).
         */
        Eigen::VectorXd torquesFeedbackPart() const;
        Eigen::VectorXd torquesFeedforwardPart() const;

    private:

        /**
         * Structure for external contact point.
         *
         * @param isEnabled if false, the contact
         * is not actually used in dynamics computation.
         * @param isPoint True if the contact is a 3 DOFs
         * contact point.
         * @param frameId RBDL body id 
         * of fixed frame. The contacting point 
         * is at the frame's center in world frame.
         * @param frictionCoef Contact surface friction
         * pyramid coefficient (tangential force constraints).
         * @param areaCoPMinX Center of pressure allowed 
         * minimum X bound expressed in local contact frame.
         * @param areaCoPMaxX Center of pressure allowed
         * maximum X bound expressed in local contact frame.
         * @param areaCoPMinY Center of pressure allowed
         * minimum Y bound expressed in local contact frame.
         * @param areaCoPMaxY Center of pressure allowed
         * maximum Y bound expressed in local contact frame.
         * @param normalForceMin Minimum normal contact force.
         * @param surfaceMat Orientation in world frame of 
         * contact surface normal (only for point contact).
         * @param targetWrench Desired wrench used for contact 
         * regularization (zero by default).
         * @param weightWrench Weighting for contact 
         * wrench in cost function
         */
        struct Contact_t {
            bool isEnabled;
            bool isPoint;
            size_t frameId;
            double frictionCoef;
            double areaCoPMinX;
            double areaCoPMaxX;
            double areaCoPMinY;
            double areaCoPMaxY;
            double normalForceMin;
            Eigen::Matrix3d surfaceMat;
            Eigen::Vector6d targetWrench;
            Eigen::Vector6d weightWrench;
        };

        /**
         * Structure for direct joint
         * level control enforced as cost in QP.
         *
         * @param index Joint index in internal 
         * model degrees of freedom.
         * @param targetAcc Control angular 
         * joint acceleration.
         * @param weight Positive weight over
         * joint control acceleration target cost.
         */
        struct Joint_t {
            size_t index;
            double targetAcc;
            double weight;
        };

        /**
         * Structure for frame whose
         * linear position or orientation expressed
         * in world frame is enforced as cost in QP.
         *
         * @param frameId RBDL body id of frame whose
         * center position or orientation is tracked.
         * @param targetAcc Control linear or angular 
         * target acceleration of frame's center 
         * expressed in world frame.
         * @param weight Positive weight over linear or 
         * angular control acceleration cost.
         */
        struct Position_t {
            size_t frameId;
            Eigen::Vector3d targetAcc;
            double weight;
        };
        struct Orientation_t {
            size_t frameId;
            Eigen::Vector3d targetAcc;
            double weight;
        };
        
        /**
         * Pointer to the working floating 
         * leph::Model instance
         */
        Model* _model;

        /**
         * Problem sizes
         */
        size_t _numberContactPlane;
        size_t _numberContactPoint;
        size_t _numberTarget;
        size_t _numberMomentum;
        size_t _sizeDOF;
        size_t _sizeJoint;
        size_t _sizeSol;
        
        /**
         * Mapping from joint or frame names 
         * to indexes in internal container
         */
        std::map<std::string, size_t> _mappingContact;
        std::map<std::string, size_t> _mappingTargetJoint;
        std::map<std::string, size_t> _mappingTargetPosition;
        std::map<std::string, size_t> _mappingTargetOrientation;

        /**
         * If true, either the tracking of centroidal 
         * linear or angular momentum is enabled
         */
        bool _isCentroidalMomentumLinear;
        bool _isCentroidalMomentumAngular;

        /**
         * If either centroidal momentum is tracked,
         * target linear or angular momentum rate of change
         * (differentiate of momentum)
         */
        Eigen::Vector3d _targetDiffMomentumLinear;
        Eigen::Vector3d _targetDiffMomentumAngular;

        /**
         * Positive weight over linear and angular 
         * centroidal control cost.
         */
        double _centroidalWeightLinear;
        double _centroidalWeightAngular;

        /**
         * Regularization weights over degrees of 
         * freedom base linear acceleration, angular
         * base and joints.
         * Contact wrench regularization are hold in 
         * Contact_t structure.
         */
        double _penaltyAccBase;
        double _penaltyAccJoint;

        /**
         * If true, some weight have been
         * updated and the internal problem
         * weights matrix need to be rebuilt
         */
        bool _isWeightsDirty;
        
        /**
         * Container for contacts and pose 
         * targets (cost minimization)
         */
        std::vector<Contact_t> _contacts;
        std::vector<Joint_t> _targetsJoint;
        std::vector<Position_t> _targetsPosition;
        std::vector<Orientation_t> _targetsOrientation;
        
        /**
         * Mapping from RBDL frame body Id 
         * to a cache of computed 6D Jacobian matrices 
         * of src frame expressed either in world frame
         * or in body frame (use for contacts).         
         * (computed at frame center, point = zero).
         */
        std::map<size_t, Eigen::MatrixXd> _cacheJacobiansWorld;
        std::map<size_t, Eigen::MatrixXd> _cacheJacobiansBody;

        /**
         * Mapping from RBDL frame body Id 
         * to a cache of computed 6D generalized velocity vector
         * (JDot*QDot computed from point acceleration).
         * of src frame expressed in world frame.         
         * (computed at frame center, point = zero).
         */
        std::map<size_t, Eigen::Vector6d> _cacheJacDotQDot;

        /**
         * Cached equation of motion terms and conversion from
         * reduced solution to degrees of freedom torques
         */
        Eigen::MatrixXd _cacheM;
        Eigen::VectorXd _cacheC;
        Eigen::VectorXd _cacheG;
        Eigen::MatrixXd _solToTauMat;
        Eigen::VectorXd _solToTauVec;

        /**
         * User defined base and joint space torques bias
         * added on gravity vector side of the equation of motion
         */
        Eigen::VectorXd _torqueBias;

        /**
         * Temporary matrix and vector for
         * problem formulation
         */
        Eigen::MatrixXd _tmpTauIneqMat;
        Eigen::VectorXd _tmpTauIneqVec;

        /**
         * Diagonal matrix containing 
         * all costs weighting
         */
        Eigen::DiagonalMatrix<double, Eigen::Dynamic> _weights;
        
        /**
         * Matrix and vector defining the 
         * minimisation problem min ||Ax-b||^2.
         */
        Eigen::MatrixXd _costMat;
        Eigen::VectorXd _costVec;

        /**
         * Allocation of QP problem matrices
         */
        Eigen::MatrixXd _problemCostMat;
        Eigen::VectorXd _problemCostVec;
        Eigen::MatrixXd _problemEqMat;
        Eigen::VectorXd _problemEqVec;
        Eigen::MatrixXd _problemIneqMat;
        Eigen::VectorXd _problemIneqVec;
        Eigen::VectorXd _problemSolution;

        /**
         * Zero wrench and force used because 
         * of type issues in contactWrench()
         */
        Eigen::VectorXd _zeroWrench;
        Eigen::VectorXd _zeroForce;

        /**
         * Store last computed torque 
         * for regularization
         */
        Eigen::VectorXd _lastJointTorques;

        /**
         * Optional parameters for control.
         * @param _useNoVelocityForDynamics Disable the use
         * of velocity for the equation of motion (non linear forces)
         * and for bias accelerations.
         * @param _coefNonDiagonalMassMatrix Multiplicative coefficient
         * on off diagonal mass matrix elements.
         * @param _disableInequalityConstraints If true, the inequality
         * constraints are skip for QP solving.
         */
        bool _useNoVelocityForDynamics;
        double _coefNonDiagonalMassMatrix;
        bool _disableInequalityConstraints;
        
        /**
         * Init Jacobian and JDotQDot
         * cache for given frame.
         *
         * @param frameId RBDL body id.
         * @param initJacWorld If true, caching
         * of Jacobian computed in world frame 
         * and JDotQDot is enabled.
         * @param initJacBody Ir true, caching
         * of Jacobian computed in body frame 
         * is enabled.
         */
        void initCache(
            size_t frameId, 
            bool initJacWorld, bool initJacBody);

        /**
         * Assign the _weights matrix the 
         * target weights stored in internal
         * data structure
         */
        void assignWeights();
};

}

#endif

