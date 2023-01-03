#ifndef LEPH_MATHS_RECURSIVELEASTSQUARES_HPP
#define LEPH_MATHS_RECURSIVELEASTSQUARES_HPP

#include <Eigen/Dense>

namespace leph {

/**
 * RecursiveLeastSquares
 *
 * Implementation of the multivariate, multidimensional
 * recursive least squares algorithm.
 * Compute recursively a linear regression 
 * with a forgetting factor.
 * Implement directional forgeting
 * as well as regularization.
 */
class RecursiveLeastSquares
{
    public:
        
        /**
         * Default initialization
         */
        RecursiveLeastSquares();
        
        /**
         * Reset internal state.
         *
         * @param initParams Initial parameters value.
         * @param initCovarianceInvDiag Initial coraviance 
         * inverse matrix diagonal value.
         */
        void reset(
            const Eigen::MatrixXd initParams,
            double initCovarianceInvDiag);

        /**
         * @return the data dimension of 
         * appended input and output vectors
         */
        size_t dimIn() const;
        size_t dimOut() const;
        
        /**
         * Add a couple of (input, output) data point
         * and immediately update the internal model.
         *
         * @param input Vector of data input (#dimIn x 1).
         * @param output Vector of output data (#dimOur x 1).
         * @param forgettingFactor Forgetting weight within [0:1]
         * used to discount previously added data.
         */
        void append(
            const Eigen::VectorXd& input, 
            const Eigen::VectorXd& output, 
            double forgettingFactor);

        /**
         * @return currently estimated
         * linear parameters
         */
        const Eigen::MatrixXd& params() const;

        /**
         * @return currenlty estimated 
         * input covariance matrix as well as its inverse
         */
        const Eigen::MatrixXd& covariance() const;
        const Eigen::MatrixXd& covarianceInverse() const;

        /**
         * Compute and return the output vector 
         * given the input vector and internally 
         * fitted parameters.
         *
         * @param input Input data vector (#dimIn x 1).
         * @return computed output vector (#dimOut x 1).
         */
        Eigen::VectorXd predict(const Eigen::VectorXd& input) const;

    private:
        
        /**
         * Estimated model parameters matrix (#dimOut x #dimIn)
         */
        Eigen::MatrixXd _params;
        
        /**
         * Estimated input covariance matrix 
         * and its inverse (#dimIn x #dimIn)
         */
        Eigen::MatrixXd _cov;
        Eigen::MatrixXd _cov2; //TODO XXX
        Eigen::MatrixXd _covInv;
};

}

#endif

