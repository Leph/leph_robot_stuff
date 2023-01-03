#ifndef LEPH_MATHS_POLYFIT_HPP
#define LEPH_MATHS_POLYFIT_HPP

#include <Eigen/Dense>
#include <leph_maths/Polynomial.hpp>
#include <leph_maths/LinearRegression.hpp>

namespace leph {

/**
 * PolyFit
 *
 * Fit one dimensional data
 * points by a polynomial of desired degree
 * and minimizing least square error
 */
class PolyFit
{
    public:

        /**
         * Initialization with polynomial
         * fitting degree.
         *
         * @param degree Degree of the polynomial 
         * to fit (should be greater or equals 1).
         */
        PolyFit(unsigned int degree);

        /**
         * Add a data point to be fitted.
         *
         * @param t Time abscissa (X).
         * @param val Associated value (Y).
         * @param weight Optional point weight.
         */
        void add(double t, double val, double weight = 1.0);

        /**
         * @return computed fitted polynomial 
         * in least square sense
         */
        Polynomial fit();

        /**
         * @return direct access to internal
         * Linear Regression instance
         */
        const LinearRegression& regression() const;

    private:

        /**
         * Desired fitting degree
         */
        unsigned int _degree;

        /**
         * Internal Linear Regression instance
         */
        LinearRegression _regression;
};

}

#endif

