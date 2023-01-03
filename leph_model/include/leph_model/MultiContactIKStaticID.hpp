#ifndef LEPH_MODEL_MULTICONTACTIKSTATICID_HPP
#define LEPH_MODEL_MULTICONTACTIKSTATICID_HPP

#include <vector>
#include <map>
#include <string>
#include <sstream>
#include <Eigen/Dense>
#include <leph_model/Model.hpp>
#include <leph_model/PinocchioInterface.hpp>

namespace leph {

/**
 * MultiContactIKStaticID
 *
 * Uneven and multi-contact inverse kinematics
 * QP based implementation enforcing static
 * inverse dynamics joint torque and contact 
 * wrenches feasibility constraints.
 */
class MultiContactIKStaticID
{
    public:

        /**
         * Base structure for Cartesian frame on robot 
         * kinematics tree used as position controlled 
         * or external contact point
         */
        struct ContactBase_t {
            //Cartesian robot frame id and name
            size_t frameId;
            std::string frameName;
            //If true, the frame is currently
            //used as external contact.
            //If false, its position is tracked by IK.
            bool isEnabled;
            //Cartesian position and orientation 
            //target expressed in world frame
            Eigen::Vector3d targetPos;
            Eigen::Matrix3d targetMat;
            //Cost weighting for IK position and 
            //orientation target
            double weightPos;
            double weightMat;
            //Maximum absolute position or 
            //orientation error used for target
            double clampPos;
            double clampMat;
            //Contact friction coefficient constraint
            double frictionCoef;
            //Contact normal force minimum 
            //and maximum bounds
            double normalForceMin;
            double normalForceMax;
            //Cache for computed frame Jacobian
            //expressed in world and local body frame
            Eigen::MatrixXd jacobianWorld;
            Eigen::MatrixXd jacobianBody;
        };

        /**
         * Structure for plane 6-DOFs contact 
         * constraints (position and orientation)
         */
        struct ContactPlane_t : public ContactBase_t {
            //Contact center of pressure absolute bounds
            //along local X/Y axis (tangent forces)
            Eigen::Vector2d limitCOP;
            //Target contact wrench and associated 
            //weight as ID wrench cost in static 
            //inverse dynamics 
            Eigen::Vector6d targetWrench;
            Eigen::Vector6d weightWrench;
            //Last successfully computed contact change wrench 
            //(if enabled or zero if disabled) and integrated
            //wrench state
            Eigen::Vector6d computedDeltaWrench;
            Eigen::Vector6d stateWrench;
            //Measured contact wrench used for special 
            //inequality robustness feature
            Eigen::Vector6d readWrench;
        };

        /**
         * Structure for point 3-DOFs contact 
         * constraints (only position)
         */
        struct ContactPoint_t : public ContactBase_t {
            //Contact surface orientation expressed in world 
            //frame (aligning Z axis as surface normal)
            Eigen::Matrix3d contactMat;
            //Target contact force and associated 
            //weight as ID force cost in static 
            //inverse dynamics
            Eigen::Vector3d targetForce;
            Eigen::Vector3d weightForce;
            //Last successfully computed contact change force 
            //(if enabled or zero if disabled) and integrated
            //force state
            Eigen::Vector3d computedDeltaForce;
            Eigen::Vector3d stateForce;
            //Measured contact force used for special 
            //inequality robustness feature
            Eigen::Vector3d readForce;
        };

        /**
         * Initialization with a Model.
         * The model instance must be valid during
         * the whole use of MultiContactIKStaticID.
         *
         * @param model Reference to the model
         * whose joints will be optimized. Stored
         * DOF position are used and updated.
         * Must have a floating base.
         */
        MultiContactIKStaticID(Model& model);

        /**
         * Define a new potential contact plane
         * (6-DOFs) or contact point (3-DOFs) 
         * with default configuration.
         *
         * @param frameName Model frame name in kinematics tree.
         */
        void addContactPlane(const std::string& frameName);
        void addContactPoint(const std::string& frameName);

        /**
         * Assign joint target positions and all contact
         * target poses from current model state
         */
        void resetTargetsFromModel();

        /**
         * Read only access to base, plane or 
         * point contact data structure.
         *
         * @param frameName Model frame name previously declared.
         * @return read only access to internal contact structure.
         */
        const ContactBase_t& stateContactBase(
            const std::string& frameName) const;
        const ContactPlane_t& stateContactPlane(
            const std::string& frameName) const;
        const ContactPoint_t& stateContactPoint(
            const std::string& frameName) const;

        /**
         * Assign measured contact wrench and force
         * used for inequalities robustness feature.
         *
         * @param frameName Model frame name previously declared.
         * @param readWrench,readForce measured contact force from sensors.
         */
        void setReadContactWrench(
            const std::string& frameName,
            const Eigen::Vector6d& readWrench);
        void setReadContactForce(
            const std::string& frameName,
            const Eigen::Vector3d& readForce);

        /**
         * Change contact state of the previously defined 
         * Cartesian frame (either plane or point).
         *
         * @param frameName Model frame name previously declared.
         * @param isEnabled If true, the given contact will
         * be set as external contact. If false, the contact
         * will be free to be position controlled.
         * @return true if the contact state is actually changed.
         */
        bool toggleContact(
            const std::string& frameName,
            bool isEnabled);

        /**
         * Mark the given contact frame as being switched.
         * Only one contact can be switched at the same time.
         *
         * @param frameName Model frame name previously declared.
         */
        void switchingAsk(const std::string& frameName);

        /**
         * Clear previously set switching contact if any.
         */
        void switchingClear();

        /**
         * Setter for joint related parameters.
         * If no joint name is specified, the given value
         * is assigned to all joints.
         * TODO XXX
         */
        void setJointLimitPos(
            const std::string& name,
            double limitLower, double limitUpper);
        void setJointLimitVelAbs(double maxVel);
        void setJointTargetPos(
            const Eigen::VectorXd& pos);
        void setJointWeightPos(double weightVel);
        void setJointWeightVel(double weightVel);
        void setJointWeightTau(double weightTau);
        void setWeightCoefIDInIK(double coef);
        void setRatioLimits(double ratio);
        void setDisableConstraints(bool isDisabled);

        /**
         * Setter for Cartesian contact related parameters.
         * If no frame name is specified, the given value
         * is assigned to all contacts.
         * TODO XXX
         */
        Eigen::Vector3d& refTargetPos(
            const std::string& frameName);
        Eigen::Matrix3d& refTargetMat(
            const std::string& frameName);
        void setWeightPose(
            const std::string& frameName,
            double weightPos, double weightMat);
        void setNormalForceLimits(
            const std::string& frameName,
            double normalForceMin, double normalForceMax);
        
        /**
         * Setter for Cartesian plane contact related parameters.
         * If no frame name is specified, the given value
         * is assigned to all plane contacts.
         * TODO XXX
         */
        void setTargetWrench(
            const std::string& frameName,
            const Eigen::Vector6d& targetWrench);
        void setWeightWrench(
            const std::string& frameName,
            const Eigen::Vector6d& weightWrench);

        /**
         * Setter for Cartesian point contact related parameters.
         * If no frame name is specified, the given value
         * is assigned to all point contacts.
         * TODO XXX
         */
        void setContactPlaneLimitCOP(
            const std::string& frameName,
            double limitX, double limitY);
        void setContactLimitFriction(
            const std::string& frameName,
            double frictionCoef);
        void setContactMat(
            const std::string& frameName,
            const Eigen::Matrix3d& contactMat);
        void setTargetForce(
            const std::string& frameName,
            const Eigen::Vector3d& targetForce);
        void setWeightForce(
            const std::string& frameName,
            const Eigen::Vector3d& weightForce);

        /**
         * Solve the quadratic program to compute the 
         * static inverse dynamics with current model state.
         *
         * @param isSwitch If true, the switching contact
         * is taken into account in ID computation (either
         * included or excluded). Else, normal computation is done.
         * @return true if the QP succeed, else false if 
         * no feasible solution is found.
         */
        bool runInverseDynamics(bool isSwitch = false);
        bool runInverseDynamicsFast(bool isSwitch = false);

        /**
         * Solve the quadratic program to compute 
         * the combined inverse dynamics and inverse 
         * kinematics with current model state.
         *
         * @param dt Time step in seconds.
         * @return true if the QP succeed, else
         * false if no feasible solution is found.
         */
        bool runCombinedIKID(double dt);
        bool runCombinedIKIDFast(double dt);

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
         * Read only access to last successfully 
         * computed changes in IKID optimization.
         *
         * @param frameName Model frame name previously 
         * declared as plane for wrench or point for force.
         * @return computed changes for kinematics degree of 
         * freedom positions, joint torques, contact
         * plane wrenches and contact point forces.
         */
        const Eigen::VectorXd& deltaDOFPosition() const;
        const Eigen::VectorXd& deltaJointTorque() const;
        const Eigen::Vector6d& deltaContactWrench(
            const std::string& frameName) const;
        const Eigen::Vector3d& deltaContactForce(
            const std::string& frameName) const;

        /**
         * Read and write access internally integrated
         * state in IKID or last successfully computed
         * joint torques in static ID.
         *
         * @param frameName Model frame name previously 
         * declared as plane for wrench or point for force.
         *
         * @return reference toward state for joint torques, 
         * contact plane wrenches and contact point forces.
         */
        const Eigen::VectorXd& stateJointTorque() const;
        Eigen::VectorXd& stateJointTorque();
        const Eigen::Vector6d& stateContactWrench(
            const std::string& frameName) const;
        Eigen::Vector6d& stateContactWrench(
            const std::string& frameName);
        const Eigen::Vector3d& stateContactForce(
            const std::string& frameName) const;
        Eigen::Vector3d& stateContactForce(
            const std::string& frameName);

        /**
         * Check if computed static ID solution are within 
         * the feasibility inequalities limits.
         *
         * @param vectorIDSolution The torque/wrench/force 
         * inverse dynamics solution vector to check.
         * @param ratioLimit Coefficient used to scale down 
         * joint torque and contact absolute limits.
         * @param isSwitch If false, the given vector must
         * be from normal ID size, else switch ID size is assumed.
         * @param ratioMax If not null, the maximum constraint
         * value with respect to limit ratio.
         * @param ss If not null, a string message is craft and 
         * appended for all or only violated limits.
         * @param printAll If true, all limits string will
         * be output in ss. Else, only violated limits.
         * @return false if the constraints are violated.
         */
        bool checkLimitsComputedIDSolution(
            const Eigen::VectorXd& vectorIDSolution,
            double ratioLimit,
            bool isSwitch,
            double* ratioMax = nullptr,
            std::ostringstream* ss = nullptr,
            bool printAll = false) const;

        /**
         * Compute and return debug ratio between [0:1].
         * - Joint position ratio w.r.t kinematic bounds.
         * - Joint effort ratio w.r.t torque bounds.
         * - Enabled contact planes center of pressure 
         * and friction pyramid limit ratio.
         * - Enabled contact points friction pyramid limit ratio.
         * - All enabled contacts normal force distribution ratio.
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
        size_t _sizePlaneOn;
        size_t _sizePointOn;
        size_t _sizePlaneOff;
        size_t _sizePointOff;

        /**
         * Container for defined Cartesian 
         * contact planes and points
         */
        std::vector<ContactPlane_t> _contactsPlane;
        std::vector<ContactPoint_t> _contactsPoint;

        /**
         * Mapping from defined contact frame names
         * to index in contact container.
         * If index > 0, then index-1 is inside _contactsPlane.
         * If index < 0, then -index-1 is inside _contactsPoint.
         */
        std::map<std::string, int> _mappingContacts;

        /**
         * Contact switching state.
         * Only one contact can be switched at the same time.
         * @param _switchState 
         * If == 0, no switching contact.
         * If == 1, the contact is currently disabled but
         * is asked to be enabled.
         * If == -1, the contact is currently enabled but
         * is asked to be disabled.
         * @param _switchIndexPlane Hold the index in container 
         * of the plane contact being switched or -1.
         * @param _switchIndexPoint Hold the index in container 
         * of the point contact being switched or -1.
         */
        int _switchState;
        int _switchIndexPlane;
        int _switchIndexPoint;

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
         * Joint target torque and weighting 
         * in static inverse dynamics
         */
        Eigen::VectorXd _jointTargetTau;
        Eigen::VectorXd _jointWeightTau;

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
         * Integrated joint torque state from IKID or last 
         * successfully computed joint torque from ID  
         */
        Eigen::VectorXd _jointStateTau;
        
        /**
         * Coefficient applied on static inverse 
         * dynamics weights to get joint torque and 
         * contact force/wrench cost weight in combined IKID
         */
        double _coefIDWeightInIK;

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
         * Retrieve a contact structure from its name.
         *
         * @param frameName Existing contact name.
         * Either plane, point or both.
         * @return the contact reference
         * within container.
         */
        ContactBase_t& getContactBase(
            const std::string& frameName);
        ContactPlane_t& getContactPlane(
            const std::string& frameName);
        ContactPoint_t& getContactPoint(
            const std::string& frameName);

        /**
         * Build matrix and vector defining static
         * inverse dynamics for joint torque and
         * contact wrench constraints.
         * Decision variables are assumed to be
         * - joint torque
         * - enabled contact plane wrenches
         * - enabled contact point forces
         *
         * @param problemIneqMat Constraints inequality 
         * matrix to be written.
         * @param problemIneqVec Constraints inequality 
         * vector to be written.
         * @param ratioLimit Coefficient used to scale down 
         * joint torque and contact absolute limits.
         * @param isSwitch If true, inequalities constraints
         * for switching inverse dynamics are written.
         */
        void buildStaticIDInequalities(
            Eigen::MatrixXd& problemIneqMat,
            Eigen::VectorXd& problemIneqVec,
            double ratioLimit,
            bool isSwitch);

        /**
         * Build and write to given matrix and vector
         * a (plane or point) contact inequality constraints.
         * The following matrix block range size are written:
         * If contact is plane: 18x6.
         * If contact is point: 6x3.
         * Input matrix and vector are assumed to be
         * reset to zero.
         *
         * @param problemIneqMat Inequality matrix to be written.
         * Must be already initialized and large enough.
         * @param problemIneqVec Inequality vector to be written.
         * Must be already initialized and large enough.
         * @param isContactPlane If true, a contact plane
         * index is assumed and 18 rows are written. Else,
         * a contact point is assumed and 6 rows are written.
         * @param indexContact Contact index to look for
         * the limits. Either plane or point.
         * @param offsetRow Offset in matrix and vector 
         * row to start writing.
         * @param offsetCol Offset in matrix col to 
         * start writing.
         * @param ratioLimit Coefficient used to scale down 
         * joint torque and contact absolute limits.
         */
        void writeContactConstraints(
            Eigen::MatrixXd& problemIneqMat,
            Eigen::VectorXd& problemIneqVec,
            bool isContactPlane,
            size_t indexContact,
            size_t offsetRow,
            size_t offsetCol,
            double ratioLimit);

        /**
         * Write to given matrix and vectors the kinematics
         * frame pose or orientation cost formulation.
         * Position and orientation: 
         * The matrix block 6 x sizeDOF is written.
         * Orientation only:
         * The matrix block 3 x sizeDOF is written.
         *
         * @param contactRef Reference to either a 
         * disabled plane or point contact structure.
         * @param costMat Cost matrix to be written.
         * Mulst be initialized and large enough.
         * @param costVec Cost vector to be written.
         * Mulst be initialized and large enough.
         * @param weights Weight diagonal matrix to be written.
         * @param offsetRow Offset in matrix and vector rows
         * @param addPosition if true, both position and orientation
         * is written (6 rows). 
         * If false, only orientation is written (3 rows).
         */
        void writePoseTarget(
            const ContactBase_t& contactRef,
            Eigen::MatrixXd& costMat,
            Eigen::VectorXd& costVec,
            Eigen::DiagonalMatrix<double, Eigen::Dynamic>& weights,
            size_t offsetRow, 
            bool isAddPosition);

        /**
         * Compute static dynamics differentiation with 
         * numerical finite difference with respect to 
         * given degree of freedom index.
         *
         * @param statePosition Current DOF position state
         * (the kinematics model is assigned).
         * @param indexDOF DOF index to differentiate.
         * @param delta Numerical delta for finite difference.
         * @param gravityVector Joint space gravity differentiated
         * vector to be assigned.
         */
        void computeDiffDynamics(
            const Eigen::VectorXd& statePosition,
            size_t indexDOF,
            double delta,
            Eigen::VectorXd& gravityVector,
            std::map<size_t, Eigen::MatrixXd>& jacobianPlaneTransposed,
            std::map<size_t, Eigen::MatrixXd>& jacobianPointTransposed);

        /**
         * Check if given joint torque, contact wrench or force
         * are within the feasibility inequality limits.
         *
         * @param torque View over joint torque vector.
         * @param wrench View over the 6d contact wrench vector.
         * @param force View over the 3d contact force vector.
         * @param indexContactPlane Index in contact plane 
         * container used to retrieve limits.
         * @param indexContactPoint Index in contact point
         * container used to retrieve limits.
         * @param ratioLimit Coefficient used to scale down 
         * joint torque and contact absolute limits.
         * @param ratioMax If not null, the maximum constraint
         * value with respect to limit ratio.
         * @param ss If not null, a string message is craft and 
         * appended for all or only violated limits.
         * @param printAll If true, all limits string will
         * be output in ss. Else, only violated limits.
         * appended for all limits.
         * @return false if the constraints are violated.
         */
        bool checkLimitsJointTorque(
            Eigen::VectorBlock<const Eigen::VectorXd, -1> torque,
            double ratioLimit,
            double* ratioMax = nullptr,
            std::ostringstream* ss = nullptr,
            bool printAll = false) const;
        bool checkLimitsContactWrench(
            Eigen::VectorBlock<const Eigen::VectorXd, 6> wrench,
            size_t indexContactPlane,
            double ratioLimit,
            double* ratioMax = nullptr,
            std::ostringstream* ss = nullptr,
            bool printAll = false) const;
        bool checkLimitsContactForce(
            Eigen::VectorBlock<const Eigen::VectorXd, 3> force,
            size_t indexContactPoint,
            double ratioLimit,
            double* ratioMax = nullptr,
            std::ostringstream* ss = nullptr,
            bool printAll = false) const;

        /**
         * @return true if the given contact plane or point
         * index should be included in the switching static
         * inverse dynamics inequalities
         */
        bool isPlaneIndexInSwitch(size_t index) const;
        bool isPointIndexInSwitch(size_t index) const;
};

}

#endif

