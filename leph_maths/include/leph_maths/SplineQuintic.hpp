#ifndef LEPH_MATHS_SPLINEQUINTIC_HPP
#define LEPH_MATHS_SPLINEQUINTIC_HPP

#include <vector>
#include <leph_maths/Spline.hpp>

namespace leph {

/**
 * SplineQuintic
 *
 * Implementation of 5th
 * order polynomial splines
 */
class SplineQuintic : public Spline
{
    public:
        
        /**
         * Via point structure
         */
        struct Point {
            double time;
            double position;
            double velocity;
            double acceleration;
        };

        /**
         * Add a new via point
         * position, velocity and acceleration 
         * at given time.
         * Internal spline parts are recomputed.
         *
         * @param time Time of the via point.
         * @param pos Position of the via point.
         * @param vel Velocity of the via point.
         * @param acc Acceleration of the via point.
         */
        void addPoint(
            double time, 
            double pos,
            double vel = 0.0,
            double acc = 0.0);
        
        /**
         * Access to points container
         */
        const std::vector<Point>& points() const;
        std::vector<Point>& points();
        
        /**
         * Recompute splines 
         * interpolation model
         */
        void computeSplines();
    
    private:

        /**
         * Points container
         */
        std::vector<Point> _points;
};

}

#endif

