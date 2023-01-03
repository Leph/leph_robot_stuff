#ifndef LEPH_MATHS_SOFTMAX_H
#define LEPH_MATHS_SOFTMAX_H

#include <cmath>
#include <Eigen/Dense>

namespace leph {

/**
 * Compute smooth maximum or smooth infinity 
 * norm from given array of values.
 * 
 * @param vect Input vector of values.
 * @param coef Smoothness regularization parameter.
 * Must be positive. Typical value: 0.01.
 * @return the maximum or infinity norm scalar.
 */
inline double SoftMaxSigned(
    const Eigen::VectorXd& vect, double coef)
{
    double offsetMax = vect.maxCoeff();
    double sum = 0.0;
    for (size_t i=0;i<(size_t)vect.size();i++) {
        sum += std::exp((vect(i) - offsetMax)/coef);
    }
    
    return coef*std::log(sum) + offsetMax;
}
inline double SoftMaxAbsolute(
    const Eigen::VectorXd& vect, double coef)
{
    double offsetMax = 
        vect.lpNorm<Eigen::Infinity>()
        * vect.lpNorm<Eigen::Infinity>();
    double sum = 0.0;
    for (size_t i=0;i<(size_t)vect.size();i++) {
        sum += std::exp((vect(i)*vect(i) - offsetMax)/coef);
    }
    
    return std::sqrt(coef*std::log(sum) + offsetMax);
}

/**
 * Compute the partial derivatives of the smooth
 * maximum or smooth infinity norm functions 
 * at given array of values.
 *
 * @param vect Input vector of values.
 * @param coef Smoothness regularization parameter.
 * Must be positive. Typical value: 0.01.
 * @return the maximum or infinity norm partial derivatives.
 * Same size as input vector.
 */
inline Eigen::VectorXd SoftMaxSignedDiff(
    const Eigen::VectorXd& vect, double coef)
{
    double offsetMax = vect.maxCoeff();
    double sum = 0.0;
    for (size_t i=0;i<(size_t)vect.size();i++) {
        sum += std::exp((vect(i) - offsetMax)/coef);
    }
    Eigen::VectorXd diff = Eigen::VectorXd::Zero(vect.size());
    if (std::fabs(sum) > 1e-6) {
        for (size_t i=0;i<(size_t)vect.size();i++) {
            diff(i) = std::exp((vect(i) - offsetMax)/coef)/sum;
        }
    }

    return diff;
}
inline Eigen::VectorXd SoftMaxAbsoluteDiff(
    const Eigen::VectorXd& vect, double coef)
{
    double offsetMax = 
        vect.lpNorm<Eigen::Infinity>()
        * vect.lpNorm<Eigen::Infinity>();
    double sum = 0.0;
    for (size_t i=0;i<(size_t)vect.size();i++) {
        sum += std::exp((vect(i)*vect(i) - offsetMax)/coef);
    }
    double norm = std::sqrt(coef*std::log(sum) + offsetMax);
    Eigen::VectorXd diff = Eigen::VectorXd::Zero(vect.size());
    if (std::fabs(norm*sum) > 1e-6) {
        for (size_t i=0;i<(size_t)vect.size();i++) {
            diff(i) = vect(i)*std::exp((vect(i)*vect(i) - offsetMax)/coef)/(norm*sum);
        }
    }
    
    return diff;
}

}

#endif

