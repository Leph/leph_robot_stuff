#ifndef LEPH_MATHS_SYLVESTER_H
#define LEPH_MATHS_SYLVESTER_H

#include <Eigen/Dense>

namespace leph {

/**
 * Solve the Sylvester equation:
 * AX + XB = C
 *
 * @param A Input square matrix (nxn)
 * @param B Input square matrix (mxm)
 * @param C Input rectangle matrix (nxm)
 * @param X Output resulting rectangle matrix (nxm)
 * @return true if the solver is successful, else
 * false when the two A and B matrices share at 
 * least one common eigenvalues.
 */
bool Sylvester(
    const Eigen::MatrixXd& A,
    const Eigen::MatrixXd& B,
    const Eigen::MatrixXd& C,
    Eigen::MatrixXd& X);

}

#endif

