#ifndef LEPH_MATHS_LINEARREGRESSION_HPP
#define LEPH_MATHS_LINEARREGRESSION_HPP

#include <iostream>
#include <Eigen/Dense>

namespace leph {

/**
 * LinearRegression
 *
 * Multivariate multidimensional
 * optionally weighted linear regression.
 * Least square fit without Bayesian.
 * Implements data (inputs and outputs) 
 * mean scaling and Tikhonov (ridge) 
 * regularization.
 * See https://en.wikipedia.org/wiki/Tikhonov_regularization
 */
class LinearRegression
{
    public:

        /**
         * Default initialization
         */
        LinearRegression();
        
        /**
         * Reset all internal registered 
         * inputs/outputs data
         */
        void clear();

        /**
         * @return the number of registered 
         * inputs/outputs data points
         */
        size_t count() const;

        /**
         * @return the data dimension of 
         * registered input and output vectors
         * as well as regularization.
         */
        size_t dimIn() const;
        size_t dimOut() const;
        size_t dimReg() const;

        /**
         * Set and define the ridge regularisation
         * term by its matrices and weights.
         * min ||regMat * params.transpose() - regBias||^2_W.
         *
         * @param regMat: Transformation matrix 
         * (#dimReg x #dimIn).
         * @param regBias: Bias matrix 
         * (#dimReg x #dimOut).
         * @param regWeight: Weight diagonal matrix 
         * (#dimReg x #dimReg).
         */
        void setRegularization(
            const Eigen::MatrixXd& regMat,
            const Eigen::MatrixXd& regBias,
            const Eigen::DiagonalMatrix<
                double, Eigen::Dynamic>& regWeight);

        /**
         * Add a couple of (input, output) data point.
         *
         * @param input Vector of data input (#dimIn x 1).
         * @param output Vector of output data (#dimOur x 1).
         * @param weight Optional weighting coefficient
         * over the data point.
         */
        void append(
            const Eigen::VectorXd& input, 
            const Eigen::VectorXd& output, 
            double weight = 1.0);

        /**
         * Compute the linear regression and return
         * the fitted model parameters matrix
         */
        const Eigen::MatrixXd& fit();

        /**
         * Return fitted linear parameters
         */
        const Eigen::MatrixXd& params() const;

        /**
         * Compute and return the output vector 
         * given the input vector and internally 
         * fitted parameters.
         *
         * @param input Input data vector (#dimIn x 1).
         * @return computed output vector (#dimOut x 1).
         */
        Eigen::VectorXd predict(const Eigen::VectorXd& input) const;

        /**
         * Read only access to internal 
         * registered data.
         * @return input or output data matrices.
         */
        const Eigen::MatrixXd& inputs() const;
        const Eigen::MatrixXd& outputs() const;

        /**
         * @return the data
         * residuals after fitting
         */
        Eigen::MatrixXd residuals() const;

        /**
         * @return the vector (#dimOut) mean (and root of) of 
         * squared residuals (distance between learning 
         * points and fitted model predictions)
         */
        Eigen::VectorXd meanSquaredError() const;
        Eigen::VectorXd rootMeanSquaredError() const;

        /**
         * @return the maximum residual vector (#dimOut x 1)
         * (maximum error between leaning points 
         * and fitted model predictions)
         */
        Eigen::VectorXd maxError() const;

        /**
         * @return the estimated output variance
         * vector (#dimOut x 1)
         */
        Eigen::VectorXd variance() const;

        /**
         * Print regression informations
         * (only available after the regression is done)
         *
         * @param os Optional output stream.
         */
        void print(std::ostream& os = std::cout) const;

    private:

        /**
         * Inputs data Matrix (#dimIn x #sizeData)
         */
        Eigen::MatrixXd _inputs;

        /**
         * Outputs data Matrix (#dimOut x #sizeData)
         */
        Eigen::MatrixXd _outputs;

        /**
         * Computed model parameters matrix (#dimOut x #dimIn)
         */
        Eigen::MatrixXd _params;

        /**
         * Ridge regularization parameters 
         * matrices transform and bias such that:
         * min ||_regMat * _params - _regBias||^2 
         * with a diagonal weighting matrix defining
         * the regularization penalties.
         * @param _regMat: (#dimReg x #dimOut)
         * @param _regBias: (#dimReg x #dimIn)
         * @param _regWeight: (#dimReg x #dimReg)
         */
        Eigen::MatrixXd _regMat;
        Eigen::MatrixXd _regBias;
        Eigen::DiagonalMatrix<double, Eigen::Dynamic> _regWeight;
};

}

#endif

