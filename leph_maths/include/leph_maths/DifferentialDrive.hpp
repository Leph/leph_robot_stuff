#ifndef LEPH_MATHS_DIFFERENTIALDRIVE_HPP
#define LEPH_MATHS_DIFFERENTIALDRIVE_HPP

#include <leph_maths/FilterExponential.hpp>
#include <leph_maths/TrajectoryBangBangAcc.hpp>

namespace leph {

/**
 * DifferentialDrive
 *
 * Controller and odometry estimation for 
 * two wheels differential drive
 */
class DifferentialDrive
{
    public:

        /**
         * Empty initialization
         */
        DifferentialDrive();

        /**
         * Set filter and controller parameters
         *
         * @param wheelRadius Geometric wheel radius.
         * @param wheelDistance Geometric lateral distance 
         * between base and wheels.
         * @param cutoffFreq Command filter cutoff frequency.
         * @param maxVelLin Command maximum linear velocity.
         * @param maxVelAng Command maximum angular velocity.
         * @param maxAccLin Command maximum linear acceleration.
         * @param maxAccAng Command maximum angular acceleration.
         */
        void setParameters(
            double wheelRadius,
            double wheelDistance,
            double cutoffFreq,
            double maxVelLin,
            double maxVelAng,
            double maxAccLin,
            double maxAccAng);

        /**
         * Update command filters and compute wheel effort.
         * 
         * @param dt Time step.
         * @param cmdVelLin Target forward linear velocity.
         * @param cmdVelAng Target planar angular velocity.
         */
        void update(
            double dt,
            double cmdVelLin,
            double cmdVelAng);

        /**
         * @return left and right velocity 
         * wheel effort
         */
        double getEffortWheelLeft() const;
        double getEffortWheelRight() const;

        /**
         * Reset velocity command to zero
         */
        void reset();

    private:

        /**
         * Geometric parameters
         */
        double _wheelRadius;
        double _wheelDistance;

        /**
         * Absolute maximum linear and angular 
         * velocity command
         */
        double _maxVelLin;
        double _maxVelAng;

        /**
         * Filters for linear and angular 
         * velocity command
         */
        FilterExponential<double> _filterLowpassLin;
        FilterBangBangAcc<double> _filterBangbangLin;
        FilterExponential<double> _filterLowpassAng;
        FilterBangBangAcc<double> _filterBangbangAng;
};

}

#endif

