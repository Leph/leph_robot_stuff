#ifndef LEPH_MODEL_SEIKOWRAPPER_HPP
#define LEPH_MODEL_SEIKOWRAPPER_HPP

#include <string>
#include <map>
#include <Eigen/Dense>
#include <leph_model/SEIKOFloatingBase.hpp>
#include <leph_maths/FilterPoseCommand.hpp>

namespace leph {

/**
 * SEIKOWrapper
 *
 * Sequential Equilibrium Inverse Kinematic Optimization 
 * wrapper with contact switching and target filtering features
 */
class SEIKOWrapper : public SEIKOFloatingBase
{
    public:

        /**
         * Structure for end-effector command state
         */
        struct EndEffector_t {
            //Frame name
            std::string name;
            //If true, the contact is a point contact,
            //else a plane contact
            bool isPoint;
            //If true, the contact is trying to be disabled
            bool isSwitchingDisable;
            //Desired normal force limits configuration.
            //Actual limits are changed to allow contact switching.
            double limitNormalForceMin;
            double limitNormalForceMax;
            //Config contact force or wrench weights when 
            //the contact is enabled and during disabling
            double weightForceEnabled;
            double weightForceDisabling;
            //Last provided raw pose commands
            bool isClutch;
            Eigen::Vector3d rawPos;
            Eigen::Matrix3d rawMat;
            //Last provided velocity commands
            Eigen::Vector3d velLin;
            Eigen::Vector3d velAng;
            //Command filter
            FilterPoseCommand filterPose;
        };

        /**
         * Constructor inheritance
         */
        using SEIKOFloatingBase::SEIKOFloatingBase;
        
        /**
         * Define and setup contact frames.
         * Must be called only once.
         */
        void setup();

        /**
         * Reset SEIKO and internal state
         * to default.
         *
         * @return true if the reset static ID is successful.
         */
        bool reset();

        /**
         * Set raw input pose command for end-effector
         *
         * @param frameName Defined contact frame name.
         * @param isClutch If true, pose motion integration
         * is enabled. Anchor coordinate frame is reset and offset 
         * when clutch is enabled.
         * @param posRaw Absolute raw position with A as up.
         * @param matRaw Absolute raw orientation with Z as up.
         */
        void setTargetPose(
            const std::string& frameName,
            bool isClutch,
            const Eigen::Vector3d& rawPos, 
            const Eigen::Matrix3d& rawMat);

        /**
         * Set velocity pose command for end-effector
         *
         * @param frameName Defined contact frame name.
         * @param velLin Linear velocity command.
         * @param velAng Angular velocity command.
         */
        void setTargetVel(
            const std::string& frameName,
            const Eigen::Vector3d& velLin, 
            const Eigen::Vector3d& velAng);

        /**
         * Ask for contact switching procedure.
         *
         * @param frameName Defined contact frame name.
         * @param isEnabled True if the contact should be
         * enabled, false if it should be disabled.
         */
        void askSwitching(
            const std::string& frameName,
            bool isEnabled);

        /**
         * Update command filters, update switching procedure, 
         * compute a SEIKO step, integrate the resulting deltas,
         * update the underlying model and compute ahead commands.
         *
         * @param dt_task SEIKO task time step.
         * @param dt_ahead Time step to compute ahead command.
         * @return true if SEIKO QP is successful.
         */
        bool update(double dt_task, double dt_ahead);

        /**
         * @return read access to last computed 
         * ahead joint position vector.
         */
        const Eigen::VectorXd& getAheadJointPos() const;

        /**
         * Read only access to effector command structure.
         * @param frameName Defined contact frame name.
         * @return read only reference to effector structure.
         */
        const EndEffector_t& stateEffector(
            const std::string& frameName) const;

        /**
         * Check if a contact is switching.
         * @param frameName Defined contact frame name.
         * @return true if either the contact is currently 
         * enabling or disabling.
         */
        bool stateIsContactSwitching(
            const std::string& frameName) const;

    protected:
        
        /**
         * Map containing command state for all 
         * declared end-effectors
         */
        std::map<std::string, EndEffector_t> _commands;

        /**
         * Specific coefficient to reduce angular 
         * limit margin on some joints
         */
        Eigen::VectorXd _jointPosMarginCoef;

        /**
         * Normalized weight vectors for forces and wrenches
         */
        Eigen::Vector6d _vectWeightWrench;
        Eigen::Vector3d _vectWeightForce;

        /**
         * Smoothing initialization ratio in [0:1]
         * applied on computed velocity after reset
         */
        double _initRatio;

        /**
         * Last computed joint position vector
         * for ahead time step from SEIKO output
         */
        Eigen::VectorXd _aheadJointPos;

        /**
         * Define a contact to underlying SEIKO and wrapper
         * @param nameFrame URDF frame name to configure as contact.
         * @param isPoint If true the contact is point, else is plane.
         */
        void addContact(
            const std::string nameFrame, 
            bool isPoint);

        /**
         * Update and return the position vector
         * ahead from last SEIKO computed output.
         *
         * @param dt Ahead time step from last SEIKO state.
         * @return position vector of size sizeJoint().
         */
        void computeAheadJointPos(double dt);
};

}

#endif

