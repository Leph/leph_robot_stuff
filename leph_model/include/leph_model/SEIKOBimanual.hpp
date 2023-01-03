#ifndef LEPH_MODEL_SEIKOBIMANUAL_HPP
#define LEPH_MODEL_SEIKOBIMANUAL_HPP

#include <vector>
#include <map>
#include <string>
#include <Eigen/Dense>
#include <leph_model/Model.hpp>
#include <leph_model/PinocchioInterface.hpp>

namespace leph {

/**
 * SEIKOBimanual
 *
 * Implementation if Sequential Equilibrium and
 * Inverse Kinematic Optimization method updated
 * for fixed based bimanual manipulation tasks
 */
class SEIKOBimanual
{
    public:

        /**
         * Initialization with a Model.
         * The model instance must be valid during
         * the whole use of SEIKOBimanual.
         *
         * @param model Reference to the model
         * whose joints will be optimized. Stored
         * DOF position are used and updated.
         * Must have a floating base.
         */
        SEIKOBimanual(Model& model);

        /**
         * Define the end effectors 
         * frame names as in the URDF
         */
        void setHandFrameNames(
            const std::string& nameFrameHandLeft,
            const std::string& nameFrameHandRight);

        /**
         * Enable or disable the bimanual mode.
         * If false, the hands are assumed to be commanded 
         * independently and both the equilibrium and 
         * kinematic constrains are disabled.
         */
        void setBimanualMode(bool isEnabled);

        /**
         * Return current bimanual mode
         */
        bool getBimanualMode() const;

        /**
         * Define the relative pose of the right hand 
         * as well as the position of the manipulated object's 
         * center of mass is expressed with respect to the left hands.
         * (only used in bimanual mode).
         */
        void setBimanualObjectTransforms(
            const Eigen::Vector3d& posRightHandInLeft,
            const Eigen::Matrix3d& matRightHandInLeft,
            const Eigen::Vector3d& posCoMObjectInLeft);

        /**
         * Return bimanual object transform
         */
        const Eigen::Vector3d& getBimanualPosRightInLeft() const;
        const Eigen::Matrix3d& getBimanualMatRightInLeft() const;
        const Eigen::Vector3d& getBimanualPosCoMInLeft() const;

        /**
         * Define the wrench applied on the manipulated
         * object at its center of mass (including gravity).
         * (only used in bimanual mode).
         */
        void setBimanualObjectWrench(
            const Eigen::Vector6d& wrenchObjectInWorld);

        /**
         * Read and write access to target pose for
         * left and right hands expressed in world frame.
         * Automatically override in bimanual mode.
         */
        Eigen::Vector3d& refTargetPosLeftHand();
        Eigen::Vector3d& refTargetPosRightHand();
        Eigen::Matrix3d& refTargetMatLeftHand();
        Eigen::Matrix3d& refTargetMatRightHand();

        /**
         * Read and write access to target pose for
         * the manipulated object expressed in world frame.
         * (Only used in bimanual mode).
         */
        Eigen::Vector3d& refTargetPosObject();
        Eigen::Matrix3d& refTargetMatObject();

        /**
         * Read access to desired wrench 
         * for left and right hands
         */
        Eigen::Vector6d& refTargetWrenchLeft();
        Eigen::Vector6d& refTargetWrenchRight();

        /**
         * Define Cartesian position and orientation 
         * clamping and cost weights
         */
        void setCartesianClamps(double clampPos, double clampMat);
        void setCartesianPoseWeights(double weightPos, double weightMat);

        /**
         * Define cost weights for joint torque 
         * and contact wrenches regularization
         */
        void setTorqueWrenchWeights(
            double weightJointTorque,
            const Eigen::Vector6d& weightWrench);

        /**
         * Define cost weights for joint velocity
         * and contact wrench changes regularization
         */
        void setDeltaWeights(
            double weightVelocity, double weightDeltaWrench);

        /**
         * Define friction pyramid coefficient, center of 
         * pressure range, minimal and maximal normal force 
         * limits used for object equilibrium.
         * (Only used in bimanual mode).
         */
        void setBimanualLimits(
            double frictionCoef, double copRange, 
            double normalForceMin, double normalForceMax);

        /**
         * Redefine named joint lower and upper position limits,
         * velocity as well as maximum absolute torque
         */
        void setJointLimitPos(
            const std::string& name,
            double limitLower, double limitUpper);
        void setJointLimitVel(
            double limitVelAbs);
        void setJointLimitVel(
            const std::string& name,
            double limitVelAbs);
        void setJointLimitTau(
            const std::string& name,
            double limitTauAbs);

        /**
         * Define default joint target position
         * for regularization
         */
        void setJointPosTarget(
            const Eigen::VectorXd& pos);
        
        /**
         * Define cost weight for default joint posture
         */
        void setJointPosWeight(double weightPos);
        void setJointPosWeight(
            const std::string& name, double weightPos);

        /**
         * Define joint position clamping
         */
        void setJointPosClamp(double clamp);
        
        /**
         * Define scaling factor between [0:1] applied on 
         * dynamics constraint limits
         */
        void setRatioLimits(double ratio);

        /**
         * Define inequality constraints override.
         * If true, all inequality constraints are disabled.
         */
        void setDisableConstraints(bool isDisabled);

        /**
         * If true, the joint torque constraints
         * are disabled
         */
        void setDisableJointTorqueConstraints(bool isDisabled);
        
        /**
         * Define inequality constraints loose 
         * adaptation to initial state.
         */
        void setAdaptativeConstraints(bool isDisabled);

        /**
         * Assign joint target positions and all contact
         * target poses from current model state
         */
        void resetTargetsFromModel();

        /**
         * Provide custom joint gravity vector for next iteration
         *
         * @param vectG Joint gravity vector of sizeJoint().
         */
        void setOverrideGravityVector(const Eigen::VectorXd& vectG);
        
        /**
         * Solve the quadratic program to compute 
         * the SEIKO step for bimanual fixed based.
         *
         * @param dt Time step in seconds.
         * @return true if the QP succeed, else
         * false if no feasible solution is found.
         */
        bool run(double dt);
        
        /**
         * Update model position and velocity state 
         * to integrate computed kinematics motion.
         * Update internal joint torques and contact
         * wrenches and forces to integrate computed 
         * contact wrenches and forces delta.
         *
         * @param dt Time step in seconds.
         */
        void integrateComputedDelta(double dt);

        /**
         * Retrieve delta changes computed by SEIKO
         * of joint position, joint torque and contact wrenches
         */
        const Eigen::VectorXd& deltaDOFPosition() const;
        const Eigen::VectorXd& deltaJointTorque() const;
        const Eigen::Vector6d& deltaWrenchLeft() const;
        const Eigen::Vector6d& deltaWrenchRight() const;

        /**
         * Read write access to integrated joint torque 
         * and contact wrenches state
         */
        const Eigen::VectorXd& stateJointTorque() const;
        const Eigen::Vector6d& stateWrenchLeft() const;
        const Eigen::Vector6d& stateWrenchRight() const;
        Eigen::VectorXd& stateJointTorque();
        Eigen::Vector6d& stateWrenchLeft();
        Eigen::Vector6d& stateWrenchRight();

        /**
         * Return last computed remaining bias (error)
         * for equilibrium and kinematics constraints.
         * (only if bimanual mode is enabled).
         */
        const Eigen::Vector6d& getErrorConstraintEquilibrium() const;
        const Eigen::Vector6d& getErrorConstraintKinematics() const;

        /**
         * Compute and return debug ratio between [0:1].
         * - Joint position ratio w.r.t kinematic bounds.
         * - Joint effort ratio w.r.t torque bounds.
         * - Hand contact planes center of pressure 
         * and friction pyramid limit ratio.
         * @return name to value container. All values are within [0:1].
         */
        std::map<std::string, double> computeIKIDRatios() const;

    private:

        /**
         * Pointer to used model instance
         */
        Model* _model;

        /**
         * Hold problem sizes
         */
        size_t _sizeDOF;
        size_t _sizeJoint;

        /**
         * Joint constraint limits.
         * Lower and upper angle position,
         * absolute maximum joint velocity and
         * absolute maximum joint torque.
         */
        Eigen::VectorXd _jointLimitPosLower;
        Eigen::VectorXd _jointLimitPosUpper;
        Eigen::VectorXd _jointLimitVelAbs;
        Eigen::VectorXd _jointLimitTauAbs;

        /**
         * Joint angular target position and 
         * weighting as regularization in IK cost.
         * Position weight attract toward 
         * given reference posture.
         * Velocity weight penalize joint velocity in IK.
         */
        Eigen::VectorXd _jointTargetPos;
        Eigen::VectorXd _jointWeightPos;
        Eigen::VectorXd _jointWeightVel;

        /**
         * Maximum absolute angular position error
         * used for joint position target
         */
        double _jointClampPos;

        /**
         * Minimal and maximal normal force limits
         */
        double _limitNormalForceMin;
        double _limitNormalForceMax;

        /**
         * Last computed generalized gravity
         * vector for all degrees of freedom
         */
        Eigen::VectorXd _gravityVector;

        /**
         * Last successfully computed joint 
         * torques change from IKID 
         */
        Eigen::VectorXd _jointComputedDeltaTau;

        /**
         * Last successfully computed degrees 
         * of freedom position change in IK
         */
        Eigen::VectorXd _jointComputedDeltaDOF;

        /**
         * Integrated joint torque state from SEIKO
         */
        Eigen::VectorXd _jointStateTau;
        
        /**
         * Scaling factor between [0:1] applied on 
         * dynamics constraint limits used in ID and IK
         */
        double _ratioLimitsDynamics;

        /**
         * If true, all inequality constraints
         * are disabled in both ID and IKID
         */
        bool _isDisabledConstraints;

        /**
         * If true, joint torque constraints
         * are disabled
         */
        bool _isDisabledJointTorqueConstraints;

        /**
         * If true, the constraint bounds are adapted such that
         * the initial state (before change optimization) is always 
         * inside the constraints even if it is violating the reference limits
         */
        bool _isAdaptativeConstraints;

        /**
         * Pinocchio model interface instance
         */
        PinocchioInterface _pinocchio;

        /**
         * Bimanual hands contact configuration
         */
        //End effectors frame names in URDF
        std::string _nameFrameHandLeft;
        std::string _nameFrameHandRight;
        //Target pose for left and right hands
        //expressed in world frame.
        //Automatically override in non bimanual mode.
        Eigen::Vector3d _targetPosLeftInWorld;
        Eigen::Vector3d _targetPosRightInWorld;
        Eigen::Matrix3d _targetMatLeftInWorld;
        Eigen::Matrix3d _targetMatRightInWorld;
        //Target pose for the manipulated object
        //expressed in world frame.
        //Only used in bimanual mode.
        Eigen::Vector3d _targetPosObjectInWorld;
        Eigen::Matrix3d _targetMatObjectInWorld;
        //Target wrench for left and right hands in local frame.
        Eigen::Vector6d _targetWrenchLeft;
        Eigen::Vector6d _targetWrenchRight;
        //Cartesian position and orientation clamping
        double _clampPosCart;
        double _clampMatCart;
        //Cartesian position and orientation error cost weight
        double _weightPosCart;
        double _weightMatCart;
        //Cost weight on joint torque regularization
        double _weightJointTorque;
        //Cost weight for contact wrench changes regularization
        double _weightDeltaWrenches;
        //Cost weight for contact wrenches regularization
        Eigen::Vector6d _weightWrenchHands;
        //If false, the hands are assumed to be commanded 
        //independently and both the equilibrium and 
        //kinematic constrains are disabled
        bool _isBimanualEnabled;
        //In bimanual mode, the relative pose 
        //of the right hand as well as the 
        //position of the object's center of mass is 
        //expressed with respect to the left hands
        Eigen::Vector3d _posRightHandInLeft;
        Eigen::Matrix3d _matRightHandInLeft;
        Eigen::Vector3d _posCoMObjectInLeft;
        //In bimanual mode, wrench applied on the manipulated
        //object at its center of mass (including gravity)
        Eigen::Vector6d _wrenchObjectInWorld;
        //Friction pyramid coefficient and center of 
        //pressure range limits used for object's equilibrium 
        //in bimanual mode
        double _limitFrictionObject;
        double _limitCoPObject;
        //Computed contact wrench changes from SEIKO
        Eigen::Vector6d _deltaWrenchHandLeft;
        Eigen::Vector6d _deltaWrenchHandRight;
        //Integrated contact wrenches state from SEIKO
        Eigen::Vector6d _stateWrenchHandLeft;
        Eigen::Vector6d _stateWrenchHandRight;

        /**
         * Last computed remaining bias (error)
         * for equilibrium and kinematics constraints.
         * (only if bimanual mode is enabled).
         */
        Eigen::Vector6d _biasConstraintEquilibrium;
        Eigen::Vector6d _biasConstraintKinematics;

        /**
         * If this, a custom joint gravity vector has been provided 
         * for next iteration
         */
        bool _isOverrideGravityVector;

        /**
         * Build matrix and vector defining static
         * inverse dynamics for joint torque and
         * contact wrench constraints.
         * Decision variables are assumed to be
         * - joint torque
         * - enabled contact plane wrenches
         *
         * @param problemIneqMat Constraints inequality 
         * matrix to be written.
         * @param problemIneqVec Constraints inequality 
         * vector to be written.
         * @param ratioLimit Coefficient used to scale down 
         * joint torque and contact absolute limits.
         */
        void buildStaticIDInequalities(
            Eigen::MatrixXd& problemIneqMat,
            Eigen::VectorXd& problemIneqVec,
            double ratioLimit);

        /**
         * Build and write to given matrix and vector
         * a contact inequality constraints.
         * The following matrix block range size are written:
         * If contact is plane: 18x6.
         * Input matrix and vector are assumed to be
         * reset to zero.
         *
         * @param problemIneqMat Inequality matrix to be written.
         * Must be already initialized and large enough.
         * @param problemIneqVec Inequality vector to be written.
         * Must be already initialized and large enough.
         * @param offsetRow Offset in matrix and vector 
         * row to start writing.
         * @param offsetCol Offset in matrix col to 
         * start writing.
         * @param ratioLimit Coefficient used to scale down 
         * joint torque and contact absolute limits.
         * @param isLeftHand If true, the left hand 
         * constraint are being built.
         */
        void writeContactConstraints(
            Eigen::MatrixXd& problemIneqMat,
            Eigen::VectorXd& problemIneqVec,
            size_t offsetRow,
            size_t offsetCol,
            double ratioLimit,
            bool isLeftHand);
};

}

#endif

