#ifndef LEPH_MODEL_LEGIKTALOS_HPP
#define LEPH_MODEL_LEGIKTALOS_HPP

#include <vector>
#include <Eigen/Dense>
#include <leph_model/Model.hpp>

namespace leph {

/**
 * LegIKTalos
 *
 * Analytic Inverse Kinematic for 
 * the leg of Talos robot
 */
class LegIKTalos
{
    public:

        /**
         * Empty initialization
         */
        LegIKTalos();

        /**
         * Initialization with model instance.
         *
         * @param model Reference to the Talos model
         * whose leg joints will be assigned.
         */
        LegIKTalos(Model& model);

        /**
         * Compute analytical inverse kinematics LegIK and update
         * left or right legs joint of internal model.
         * @param pos Foot center desired position in base frame.
         * @param mat Foot center desired orientation in base frame.
         * @param boundIKDistance If not null, a signed metric from
         * how close to Kinematic bound if assigned. If positive, the
         * IK is feasible, infeasible if negative.
         *
         * @return True is returned and joint angles are updated 
         * if given desired pose is feasible, else the IK failed.
         */
        bool legIKLeft(
            const Eigen::Vector3d& pos,
            const Eigen::Matrix3d& mat,
            double* boundIKDistance = nullptr);
        bool legIKRight(
            const Eigen::Vector3d& pos,
            const Eigen::Matrix3d& mat,
            double* boundIKDistance = nullptr);

        /**
         * Compute analytical inverse kinematic of both Talos legs.
         * Set both legs joint positions such that given base and flying foot pose
         * expressed in support foot is realized.
         * @param isLeftFoot If true, the support foot is assumed to be the left foot. 
         * Else the right foot.
         * @param basePos Base desired position in support foot frame.
         * @param baseMat Base desired orientation in support foot frame.
         * @param basePos Flying other foot desired position in support foot frame.
         * @param baseMat Flying other foot desired orientation in support foot frame.
         * @param boundIKDistance If not null, a signed metric from
         * how close to Kinematic bound if assigned. If positive, the
         * IK is feasible, infeasible if negative.
         *
         * @return True is returned and joint angles are updated 
         * if given desired poses are feasible, else the IK failed.
         */
        bool baseFootIK(
            bool isLeftFoot,
            const Eigen::Vector3d& basePos, 
            const Eigen::Matrix3d& baseMat,
            const Eigen::Vector3d& footPos,
            const Eigen::Matrix3d& footMat,
            double* boundIKDistance = nullptr);

        /**
         * Compute iterative inverse kinematic with the same interface 
         * as baseFootIK(), but the Center of Mass position is provided
         * instead of the base position in support foot frame.
         */
        bool CoMFootIK(
            bool isLeftFoot,
            const Eigen::Vector3d& comPos, 
            const Eigen::Matrix3d& baseMat,
            const Eigen::Vector3d& footPos,
            const Eigen::Matrix3d& footMat,
            double* boundIKDistance = nullptr);

    private:

        /**
         * Model instance
         */
        Model* _model;

        /**
         * Indexes in model of degrees of freedom of 
         * left and right leg joints from top to bottom
         */
        std::vector<size_t> _indexLegLeft;
        std::vector<size_t> _indexLegRight;

        /**
         * Translation from base to hips
         */
        Eigen::Vector3d _transBaseToHipLeft;
        Eigen::Vector3d _transBaseToHipRight;

        /**
         * Leg length
         */
        double _legHipToKnee;
        double _legKneeToAnkle;
        double _legAnkleToGround;

        /**
         * Indexes of frames in model
         */
        size_t _indexFrameBase;
        size_t _indexFrameFootLeft;
        size_t _indexFrameFootRight;
};

}

#endif

