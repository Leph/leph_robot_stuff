#ifndef LEPH_MODEL_SEIKOFLOATINGBASE_HPP
#define LEPH_MODEL_SEIKOFLOATINGBASE_HPP

#include <vector>
#include <map>
#include <string>
#include <Eigen/Dense>
#include <leph_model/Model.hpp>
#include <leph_model/PinocchioInterface.hpp>

namespace leph {

/**
 * SEIKOFloatingBase
 *
 * Sequential Equilibrium Inverse Kinematic Optimization 
 * implementation for floating base system and 
 * for multi-contact applications
 */
class SEIKOFloatingBase
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
            //Maximum absolute velocity limit 
            //for the normal contact force
            double limitVelNormalForce;
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
        };
        
        /**
         * Empty initialization
         */
        SEIKOFloatingBase();

        /**
         * Initialization with a Model.
         * The model instance must be valid during
         * the whole use of SEIKOFloatingBase.
         *
         * @param model Reference to the model
         * whose joints will be optimized. Stored
         * DOF position are used and updated.
         * Must have a floating base.
         */
        SEIKOFloatingBase(Model& model);

        /**
         * Initialization with a Model.
         * The model instance must be valid during
         * the whole use of SEIKOFloatingBase.
         *
         * @param model Reference to the model
         * whose joints will be optimized. Stored
         * DOF position are used and updated.
         * Must have a floating base.
         */
        void init(Model& model);
        
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
         * Read only access to the map (name to index) of defined
         * planes (positive index) and points (negative index)
         */
        const std::map<std::string, int>& getMappingContacts() const;
        
        /**
         * Setter for joint related parameters.
         */
        void setJointLimitPos(
            const Eigen::VectorXd& limitLower, 
            const Eigen::VectorXd& limitUpper);
        void setJointLimitVelAbs(const Eigen::VectorXd& maxVel);
        void setJointLimitTauAbs(const Eigen::VectorXd& maxTau);
        void setJointTargetPos(const Eigen::VectorXd& targetPos);
        void setJointWeightPos(const Eigen::VectorXd& weightPos);
        void setJointWeightVel(const Eigen::VectorXd& weightVel);
        void setJointWeightTau(const Eigen::VectorXd& weightTau);
        void setJointClampPos(double clampPos);
        
        /**
         * Setter for Cartesian contact related parameters.
         */
        Eigen::Vector3d& refTargetPos(
            const std::string& frameName);
        Eigen::Matrix3d& refTargetMat(
            const std::string& frameName);
        void setWeightPose(
            const std::string& frameName,
            double weightPos, double weightMat);
        void setClampPose(
            const std::string& frameName,
            double clampPos, double clampMat);
        void setContactLimitFriction(
            const std::string& frameName,
            double frictionCoef);
        void setNormalForceLimits(
            const std::string& frameName,
            double normalForceMin, double normalForceMax);
        void setLimitVelNormalForce(
            const std::string& frameName,
            double maxVel);
        
        /**
         * Setter for Cartesian plane contact related parameters.
         */
        void setContactPlaneLimitCOP(
            const std::string& frameName,
            double limitX, double limitY);
        void setTargetWrench(
            const std::string& frameName,
            const Eigen::Vector6d& targetWrench);
        void setWeightWrench(
            const std::string& frameName,
            const Eigen::Vector6d& weightWrench);

        /**
         * Setter for Cartesian point contact related parameters.
         */
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
         * Access to external wrench on floating base
         * (using torque-force convention).
         */
        const Eigen::Vector6d& refExternalBaseWrench() const;
        Eigen::Vector6d& refExternalBaseWrench();
        
        /**
         * Solve the quadratic program to compute the 
         * static inverse dynamics with current model state.
         *
         * @return true if the QP succeed, else false if 
         * no feasible solution is found.
         */
        bool runInverseDynamics();
        
        /**
         * Solve the quadratic program to compute 
         * the combined inverse dynamics and inverse 
         * kinematics SEIKO with current model state.
         *
         * @param dt Time step in seconds.
         * @return true if the QP succeed, else
         * false if no feasible solution is found.
         */
        bool runSEIKO(double dt);
        
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
         * Read only access to internally integrated
         * state in SEIKO or last successfully computed
         * joint torques in static ID.
         *
         * @param frameName Model frame name previously 
         * declared as plane for wrench or point for force.
         *
         * @return reference toward state for joint torques, 
         * contact plane wrenches and contact point forces.
         */
        const Eigen::VectorXd& stateJointTorque() const;
        const Eigen::Vector6d& stateContactWrench(
            const std::string& frameName) const;
        const Eigen::Vector3d& stateContactForce(
            const std::string& frameName) const;

        /**
         * Direct access to normal force
         * for both plane and point contact.
         *
         * @param frameName Model frame name previously 
         * declared as plane or point.
         *
         * @return last integrated/computed contact normal force
         */
        double stateNormalForce(
            const std::string& frameName) const;

        /**
         * Return last computed remaining bias (error) 
         * for equilibrium equality constraints
         */
        const Eigen::Vector6d& getErrorConstraintEquilibrium() const;

        /**
         * Compute and return constraint ratio between [0:1].
         * - Joint position ratio w.r.t kinematic bounds.
         * - Joint effort ratio w.r.t torque bounds.
         * - Enabled contact planes center of pressure 
         * and friction pyramid limit ratio.
         * - Enabled contact points friction pyramid limit ratio.
         * - All enabled contacts normal force distribution ratio.
         * @return name to value container. All values are within [0:1].
         */
        std::map<std::string, double> computeConstraintRatios() const;

    protected:
        
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
         * torques change from SEIKO
         */
        Eigen::VectorXd _jointComputedDeltaTau;

        /**
         * Last successfully computed degrees 
         * of freedom position change in SEIKO
         */
        Eigen::VectorXd _jointComputedDeltaDOF;

        /**
         * Integrated joint torque state from SEIKO or last 
         * successfully computed joint torque from ID  
         */
        Eigen::VectorXd _jointStateTau;

        /**
         * Additional external torque-force wrench applied on 
         * the floating base to be added to the equilibrium equation
         */
        Eigen::Vector6d _externalBaseWrench;
        
        /**
         * Last computed remaining bias (error) 
         * for equilibrium equality constraints
         */
        Eigen::Vector6d _biasConstraintEquilibrium;
        
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
         */
        void buildStaticIDInequalities(
            Eigen::MatrixXd& problemIneqMat,
            Eigen::VectorXd& problemIneqVec);
        
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
         */
        void writeContactConstraints(
            Eigen::MatrixXd& problemIneqMat,
            Eigen::VectorXd& problemIneqVec,
            bool isContactPlane,
            size_t indexContact,
            size_t offsetRow,
            size_t offsetCol);
        
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
};

}

#endif

