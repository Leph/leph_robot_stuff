#ifndef LEPH_MATHS_SPLINE_HPP
#define LEPH_MATHS_SPLINE_HPP

#include <vector>
#include <leph_maths/Polynomial.hpp>

namespace leph {

/**
 * Spline
 *
 * Generic one dimensional 
 * polynomial spline generator
 */
class Spline 
{
    public:

        /**
         * Internal spline part with 
         * a polynomial only valid 
         * on a time interval
         */
        struct Spline_t {
            Polynomial polynom;
            double min;
            double max;
        };

        /**
         * Evaluation of the spline position and 
         * its first, second, third and fourth 
         * derivatives.
         *
         * @param t The evaluation time.
         * @return spline evaluation at t.
         */
        double pos(double t) const;
        double vel(double t) const;
        double acc(double t) const;
        double jerk(double t) const;

        /**
         * @return minimum and maximum time 
         * abscissa value for which the 
         * spline is defined
         */
        double min() const;
        double max() const;

        /**
         * @return the number of internal 
         * polynomial parts
         */
        size_t size() const;

        /**
         * Access to spline part. 
         *
         * @param index Spline part index
         * from 0 to size().
         * @return Spline part reference.
         */
        const Spline_t& part(size_t index) const;

    protected:

        /**
         * Spline parts container.
         * The parts are assumed to be sorted
         * by time and non overlapping.
         */
        std::vector<Spline_t> _splines;

    private:
        
        /**
         * Return spline interpolation of given value and
         * used given polynomial evaluation function
         * (member function pointer)
         *
         * @param t The evaluation time.
         * @return spline value evaluated at t.
         */
        double interpolation(double t, 
            double(Polynomial::*func)(double) const) const;
};

}

#endif

