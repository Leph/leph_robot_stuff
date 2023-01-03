#ifndef LEPH_MATHS_SPLINELINEAR_HPP
#define LEPH_MATHS_SPLINELINEAR_HPP

#include <vector>
#include <leph_maths/Spline.hpp>

namespace leph {

/**
 * SplineLinear
 *
 * Implementation of first 
 * order polynomial splines
 */
class SplineLinear : public Spline
{
    public:
        
        /**
         * Via point structure
         */
        struct Point {
            double time;
            double position;
        };

        /**
         * Add a new via point
         * position at given time.
         * Internal spline parts are recomputed.
         *
         * @param time Time of the via point.
         * @param pos Position of the via point.
         */
        void addPoint(double time, double pos);
        
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

