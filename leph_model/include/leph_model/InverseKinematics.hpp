#ifndef LEPH_MODEL_INVERSEKINEMATICS_HPP
#define LEPH_MODEL_INVERSEKINEMATICS_HPP

#include <vector>
#include <map>
#include <string>
#include <Eigen/Dense>
#include <leph_model/Model.hpp>

namespace leph {

/**
 * InverseKinematics
 *
 * Iterative whole body Inverse Kinematics
 * based on Quadratic Programming formulation
 * on velocity level.
 * Accuracy of trajectory tracking is
 * depending on time step length and weights 
 * applied on target cost and damping.
 */
class InverseKinematics
{
    public:

        /**
         * Initialization with a Model.
         * The model instance must be valid during
         * the whole use of InverseKinematics.
         *
         * @param model Reference to the model
         * whose joints will be optimized.
         */
        InverseKinematics(Model& model);

        /**
         * Declare a joint to be optimized.
         * Only declared joint will be taken into account.
         *
         * @param nameJoint Model name of added joint.
         */
        void addJoint(const std::string& nameJoint);

        /**
         * Declare a named kinematics target (cost minimization).
         * Either kinematics position, orientation or specific 
         * scalar dimension will be enforced on a specific frame.
         * Special frame name "COM" stands for the center of mass.
         *
         * @param dim For scalar target, the row index
         * defining the dimension in the Jacobian matrix.
         * @param nameTarget User defined name for the target.
         * @param nameFrameSrc RBDL body name of the frame
         * which kinematics state is enforced.
         * @param nameFrameDst RBDL body name of the frame
         * in which target kinematics state will be expressed.
         */
        void addTargetScalar(
            const std::string& nameTarget,
            unsigned int dim,
            const std::string& nameFrameSrc,
            const std::string& nameFrameDst);
        void addTargetPosition(
            const std::string& nameTarget,
            const std::string& nameFrameSrc,
            const std::string& nameFrameDst);
        void addTargetOrientation(
            const std::string& nameTarget,
            const std::string& nameFrameSrc,
            const std::string& nameFrameDst);

        /**
         * Declare a named kinematics lower or upper 
         * scalar limit as inequality constraints.
         * A specific scalar dimension will be enforced 
         * on a specific frame.
         * Special frame name "COM" stands for the center of mass.
         *
         * @param dim For scalar target, the row index
         * defining the dimension in the Jacobian matrix.
         * @param nameLimit User defined name for the limit.
         * @param nameFrameSrc RBDL body name of the frame
         * which kinematics state is enforced.
         * @param nameFrameDst RBDL body name of the frame
         * in which target kinematics state will be expressed.
         */
        void addLimitLower(
            const std::string& nameLimit,
            unsigned int dim,
            const std::string& nameFrameSrc,
            const std::string& nameFrameDst);
        void addLimitUpper(
            const std::string& nameLimit,
            unsigned int dim,
            const std::string& nameFrameSrc,
            const std::string& nameFrameDst);

        /**
         * Reset and initialize internal 
         * data structure. Reset joint limits
         * to model (URDF) limits.
         * Seting weights, limits and targets can only 
         * be assigned after this method is called.
         */
        void init();

        /**
         * Set delta joint regularization weight.
         * Used to improve problem stability.
         *
         * @param weight Positive damping target weight.
         */
        void setDamping(double weight);

        /**
         * Set the absolute maximum angular change
         * for all joint during one call of 
         * optimization in run().
         * Used to improve problem stability.
         *
         * @param max Positive maximum angular
         * change in radian.
         */
        void setMaxChange(double max);

        /**
         * Override target weight in
         * cost minimization.
         *
         * @param nameTarget User name of
         * previously declared target.
         * @param weight Positive weight
         * for target minimized cost.
         */
        void setWeightScalar(
            const std::string& nameTarget,
            double weight);
        void setWeightPosition(
            const std::string& nameTarget,
            double weight);
        void setWeightOrientation(
            const std::string& nameTarget,
            double weight);

        /**
         * Override joint lower and upper bound 
         * (inequality) constraint.
         *
         * @param nameJoint Model joint name that
         * must have been declared.
         * @param limitLower Lower angle limit in radian.
         * Must be lower than limitUpper.
         * @param limitUpper upper angle limit in radian.
         * Must be greater than limitLower.
         */
        void setJointLimit(
            const std::string& nameJoint,
            double limitLower,
            double limitUpper);

        /**
         * Set a new position, orientation 
         * or scalar target for previously 
         * declared target.
         *
         * @param nameTarget Frame name of
         * previously declared target.
         * @param val Scalar target expressed
         * in declared frameDst.
         * @param pos Point position to be enforce 
         * expressed in declared frameDst.
         * @param mat Orientation in rotation matrix 
         * format expressed in declared frameDst.
         */
        void setTargetScalar(
            const std::string& nameTarget,
            double val);
        void setTargetPosition(
            const std::string& nameTarget,
            const Eigen::Vector3d& pos);
        void setTargetOrientation(
            const std::string& nameTarget,
            const Eigen::Matrix3d& mat);

        /**
         * Assign a lower or upper scalar bound
         * to a declared limit.
         *
         * @param nameLimit Frame name of
         * previously declared limit.
         * @param val Scalar limit value expressed
         * in frameDst.
         */
        void setLimitLower(
            const std::string& nameLimit,
            double val);
        void setLimitUpper(
            const std::string& nameLimit,
            double val);

        /**
         * Run the optimization process.
         * If the solver is successful,
         * the underlying model's joint 
         * positions are updated.
         * Note: model kinematics state is 
         * not updated (recomputed).
         * This update will be needed to 
         * re-run the InverseKinematics.
         *
         * @return true if optimization 
         * has succeed.
         */
        bool run();

        /**
         * Print on standard output debugging
         * information about internal states
         */
        void print() const;

    private:

        /**
         * Target and constraint structure
         * for unidimensional scalar, 3d
         * position or orientation.
         * @param frameSrc Frame (RBDL body id)
         * at which the Jacobian is computed.
         * @param frameDst If not zero,
         * optional frame in which the target or limit
         * is expressed (world ROOT frame by default).
         * @param dim Index of the dimension 
         * in Jacobian matrix (in 0:6).
         * @param isOrientation True is the
         * scalar is an element of orientation axis.
         */
        struct Scalar_t {
            size_t frameSrc;
            size_t frameDst;
            unsigned int dim;
            double val;
            bool isOrientation;
        };
        struct Position_t {
            size_t frameSrc;
            size_t frameDst;
            Eigen::Vector3d pos;
        };
        struct Orientation_t {
            size_t frameSrc;
            size_t frameDst;
            Eigen::Matrix3d mat;
        };

        /**
         * Pointer to the working leph::Model instance
         */
        Model* _model;

        /**
         * Mapping from names (joint names or user specified names) 
         * to indexes in internal container.
         */
        std::map<std::string, size_t> _mappingJoint;
        std::map<std::string, size_t> _mappingTargetScalar;
        std::map<std::string, size_t> _mappingTargetPosition;
        std::map<std::string, size_t> _mappingTargetOrientation;
        std::map<std::string, size_t> _mappingLimitLower;
        std::map<std::string, size_t> _mappingLimitUpper;
        
        /**
         * Container of declared joints 
         * to be optimized for
         */
        std::vector<size_t> _jointsIndex;

        /**
         * Container for target value (cost minimized)
         * for unidimensional, position and orientation.
         */
        std::vector<Scalar_t> _targetsScalar;
        std::vector<Position_t> _targetsPosition;
        std::vector<Orientation_t> _targetsOrientation;

        /**
         * Container for lower and upper unidimensional limits
         */
        std::vector<Scalar_t> _limitsLower;
        std::vector<Scalar_t> _limitsUpper;
        
        /**
         * Problem typical size
         */
        size_t _sizeDOF;
        size_t _sizeTarget;
        size_t _sizeLimit;

        /**
         * Mapping from RBDL couple bodyId (src, dst) 
         * to a cache of computed 6D Jacobian matrices 
         * at src expressed in dst frame.         
         * * (computed at frame center, point = zero).
         * Special bodyId src -1 is used for 3D CoM Jacobian.
         * Both full and restricted to declared 
         * joints are stored.
         */
        std::map<std::pair<size_t, size_t>, Eigen::MatrixXd> 
            _cacheJacobiansFull;
        std::map<std::pair<size_t, size_t>, Eigen::MatrixXd> 
            _cacheJacobiansCut;

        /**
         * Mapping from RBDL couple bodyId (src, dst) 
         * to a cache of computed position vectors or 
         * orientation rotation matrices at src expressed
         * in dst frame.
         * (computed at frame center, point = zero).
         * Special bodyId src -1 is used for CoM position.
         */
        std::map<std::pair<size_t, size_t>, Eigen::Vector3d> 
            _cachePositions;
        std::map<std::pair<size_t, size_t>, Eigen::Matrix3d> 
            _cacheOrientations;

        /**
         * Hold the current angular position of
         * all declared joints
         */
        Eigen::VectorXd _currentJointPositions;

        /**
         * Vectors holding the lower and upper
         * angular bounds (in radian) of each
         * declared joints.
         * Same order than joints container.
         */
        Eigen::VectorXd _jointsLimitLower;
        Eigen::VectorXd _jointsLimitUpper;

        /**
         * Diagonal matrix containing 
         * all targets weighting
         */
        Eigen::DiagonalMatrix<double, Eigen::Dynamic> _weights;

        /**
         * Absolute maximum angular change 
         * (dq) bound for all joints
         */
        double _jointsLimitMaxChange;

        /**
         * Matrix and vector defining the 
         * minimisation problem min ||Ax-b||^2.
         */
        Eigen::MatrixXd _costTargetMat;
        Eigen::VectorXd _costTargetVec;

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
         * Init cache Jacobian, position
         * and orientation for given frame.
         *
         * @param idSrc, idDst RBDL source and 
         * destination bodies id.
         * @param initJac If true, the Jacobian
         * matrix caching is initialized for id.
         * @param initPos If true, the position
         * vector caching is initialized for id.
         * @param initMat If true, the orientation
         * matrix caching is initialized for id.
         */
        void initCache(
            size_t idSrc, size_t idDst, 
            bool initJac, bool initPos, bool initMat);
};

}

#endif

