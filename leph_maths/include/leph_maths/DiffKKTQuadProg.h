#ifndef LEPH_MATHS_DIFFKKTQUADPROG_H
#define LEPH_MATHS_DIFFKKTQUADPROG_H

#include <vector>
#include <stdexcept>
#include <Eigen/Dense>

namespace leph {

/**
 * Compute the partial derivatives with respect to either 
 * a single or several variables of the solution of a quadratic 
 * programming (QP) problem based on the differentiation of 
 * the Karush-Kuhn-Tucker (KKT) conditions.
 * See "Optnet: Differentiable optimization as a layer in neural networks"
 * 2017, B. Brandon, JZ. Kolte.
 * 
 * The QP formulation used is the following:
 * minimize(sol) 0.5*sol'*costMat*sol + costVec'*sol
 * Such that:
 * eqMat*sol + eqVec = 0
 * ineqMat*sol + ineqVec >= 0
 *
 * The KKT conditions are (note the minus):
 * costMat*sol + costVec - eqMat'*dualEq - ineqMat'*dualIneq = 0
 * eqMat*sol + eqVec = 0
 * diag(dualIneq)*(ineqMat*sol + ineqVec) = 0
 *
 * @param solution The optimal solution vector the QP (satisfying the KKT conditions).
 * @param dualEq The Lagrangian multipliers associated to the 
 * QP equality constraints (in the same layout as the eqMat matrix).
 * @param dualIneqActive The non zero Lagrangian multipliers 
 * associated to the QP active inequality constraints 
 * (layout given by dualIneqIndex).
 * @param dualIneqIndex Index mapping from active 
 * inequalities to indexes into the ineqMat matrix.
 * @param costMat QP cost matrix.
 * @param eqMat QP equality constraints matrix.
 * @param ineqMat QP inequality constraints matrix.
 * @param ineqVec QP inequality constraints vector.
 */

/**
 * @param diffCostMat Derivative of the QP cost matrix
 * with respect to the single differentiation variable.
 * @param diffCostVec Derivative of the QP cost vector
 * with respect to the single differentiation variable.
 * @param diffEqMat Derivative of the QP equality constraints 
 * matrix with respect to the single differentiation variable.
 * @param diffEqVec Derivative of the QP equality constraints 
 * vector with respect to the single differentiation variable.
 * @param diffIneqMat Derivative of the QP inequality constraints 
 * matrix with respect to the single differentiation variable.
 * @param diffIneqVec Derivative of the QP inequality constraints 
 * vector with respect to the single differentiation variable.
 * @return the vector of partial derivatives of the QP solution
 * with respect to the single differentiation variable.
 */
inline Eigen::VectorXd DiffKKTQuadProg(
    const Eigen::VectorXd& solution,
    const Eigen::VectorXd& dualEq,
    const Eigen::VectorXd& dualIneqActive,
    const Eigen::VectorXi& dualIneqIndex,
    const Eigen::MatrixXd& costMat,
    const Eigen::MatrixXd& eqMat,
    const Eigen::MatrixXd& ineqMat,
    const Eigen::VectorXd& ineqVec,
    const Eigen::MatrixXd& diffCostMat,
    const Eigen::VectorXd& diffCostVec,
    const Eigen::MatrixXd& diffEqMat,
    const Eigen::VectorXd& diffEqVec,
    const Eigen::MatrixXd& diffIneqMat,
    const Eigen::VectorXd& diffIneqVec)
{
    //Check sizes
    size_t sizeSol = solution.size();
    size_t sizeEq = dualEq.size();
    size_t sizeIneq = dualIneqActive.size();
    if (
        (size_t)costMat.rows() != sizeSol ||
        (size_t)costMat.cols() != sizeSol ||
        (size_t)eqMat.rows() != sizeEq ||
        (size_t)eqMat.cols() != sizeSol ||
        (size_t)ineqMat.rows() < sizeIneq ||
        (size_t)ineqMat.cols() != sizeSol ||
        (size_t)ineqVec.size() != (size_t)ineqMat.rows() ||
        (size_t)diffCostMat.rows() != sizeSol ||
        (size_t)diffCostMat.cols() != sizeSol ||
        (size_t)diffCostVec.size() != sizeSol ||
        (size_t)diffEqMat.rows() != sizeEq ||
        (size_t)diffEqMat.cols() != sizeSol ||
        (size_t)diffEqVec.size() != sizeEq ||
        (size_t)diffIneqMat.rows() != (size_t)ineqMat.rows() ||
        (size_t)diffIneqMat.cols() != sizeSol ||
        (size_t)diffIneqVec.size() != (size_t)ineqMat.rows()
    ) {
        throw std::logic_error(
            "leph::DiffKKTQuadProg: "
            "Invalid input matrices or vectors size.");
    }

    //Extract and select active rows in 
    //inequality matrices and vectors
    Eigen::MatrixXd cutIneqMat(sizeIneq, sizeSol);
    Eigen::VectorXd cutIneqVec(sizeIneq);
    Eigen::MatrixXd cutDiffIneqMat(sizeIneq, sizeSol);
    Eigen::VectorXd cutDiffIneqVec(sizeIneq);
    for (size_t i=0;i<(size_t)dualIneqIndex.size();i++) {
        cutIneqMat.row(i) = ineqMat.row(dualIneqIndex(i));
        cutIneqVec(i) = ineqVec(dualIneqIndex(i));
        cutDiffIneqMat.row(i) = diffIneqMat.row(dualIneqIndex(i));
        cutDiffIneqVec(i) = diffIneqVec(dualIneqIndex(i));
    }
    
    //Build diagonal matrices
    Eigen::DiagonalMatrix<double, Eigen::Dynamic> diagDualIneq = 
        Eigen::DiagonalMatrix<double, Eigen::Dynamic>(dualIneqActive);
    Eigen::DiagonalMatrix<double, Eigen::Dynamic> diagErrorIneq = 
        Eigen::DiagonalMatrix<double, Eigen::Dynamic>(
            cutIneqMat*solution + cutIneqVec);

    //Build the derivatives matrix of the KKT conditions
    Eigen::MatrixXd tmpMat = Eigen::MatrixXd::Zero(
        sizeSol+sizeEq+sizeIneq, sizeSol+sizeEq+sizeIneq);
    Eigen::VectorXd tmpVec = Eigen::VectorXd::Zero(
        sizeSol+sizeEq+sizeIneq);
    tmpMat.block(0, 0, sizeSol, sizeSol) = costMat;
    tmpMat.block(0, sizeSol, sizeSol, sizeEq) = -eqMat.transpose();
    tmpMat.block(0, sizeSol+sizeEq, sizeSol, sizeIneq) = 
        -cutIneqMat.transpose();
    tmpMat.block(sizeSol, 0, sizeEq, sizeSol) = eqMat;
    tmpMat.block(sizeSol+sizeEq, 0, sizeIneq, sizeSol) = 
        diagDualIneq*cutIneqMat;
    tmpMat.block(sizeSol+sizeEq, sizeSol+sizeEq, sizeIneq, sizeIneq) = 
        diagErrorIneq;
    //Build the derivative vector
    tmpVec.segment(0, sizeSol) = 
        -diffCostMat*solution 
        -diffCostVec 
        +diffEqMat.transpose()*dualEq
        +cutDiffIneqMat.transpose()*dualIneqActive;
    tmpVec.segment(sizeSol, sizeEq) = 
        -diffEqMat*solution
        -diffEqVec;
    tmpVec.segment(sizeSol+sizeEq, sizeIneq) = 
        -1.0*diagDualIneq*(cutDiffIneqMat*solution + cutDiffIneqVec);

    //Solve the system for the 
    //solution's partial derivatives
    Eigen::VectorXd tmpDiff = tmpMat.colPivHouseholderQr().solve(tmpVec);
    return tmpDiff.segment(0, sizeSol); //XXX ratio 0.5 ???
}

/**
 * @param diffCostMat Derivative of the QP cost matrix
 * with respect to the set of differentiation variables.
 * If empty, the derivatives are supposed to be zero.
 * @param diffCostVec Derivative of the QP cost vector
 * with respect to the set of differentiation variables.
 * If empty, the derivatives are supposed to be zero.
 * @param diffEqMat Derivative of the QP equality constraints 
 * matrix with respect to the set of differentiation variables.
 * If empty, the derivatives are supposed to be zero.
 * @param diffEqVec Derivative of the QP equality constraints 
 * vector with respect to the set of differentiation variables.
 * If empty, the derivatives are supposed to be zero.
 * @param diffIneqMat Derivative of the QP inequality constraints 
 * matrix with respect to the set of differentiation variables.
 * If empty, the derivatives are supposed to be zero.
 * @param diffIneqVec Derivative of the QP inequality constraints 
 * vector with respect to the set of differentiation variables.
 * If empty, the derivatives are supposed to be zero.
 * @return the matrix of partial derivatives of the QP solution
 * with respect to the set of differentiation variables.
 */
inline Eigen::MatrixXd DiffKKTQuadProg(
    const Eigen::VectorXd& solution,
    const Eigen::VectorXd& dualEq,
    const Eigen::VectorXd& dualIneqActive,
    const Eigen::VectorXi& dualIneqIndex,
    const Eigen::MatrixXd& costMat,
    const Eigen::MatrixXd& eqMat,
    const Eigen::MatrixXd& ineqMat,
    const Eigen::VectorXd& ineqVec,
    const std::vector<Eigen::MatrixXd>& diffCostMat,
    const std::vector<Eigen::VectorXd>& diffCostVec,
    const std::vector<Eigen::MatrixXd>& diffEqMat,
    const std::vector<Eigen::VectorXd>& diffEqVec,
    const std::vector<Eigen::MatrixXd>& diffIneqMat,
    const std::vector<Eigen::VectorXd>& diffIneqVec)
{
    //Retrieve the number of partial derivative 
    //dimensions to compute on
    size_t sizeDiff = 0;
    if (diffCostMat.size() > 0 && sizeDiff == 0) {
        sizeDiff = diffCostMat.size();
    }
    if (diffCostVec.size() > 0 && sizeDiff == 0) {
        sizeDiff = diffCostVec.size();
    }
    if (diffEqMat.size() > 0 && sizeDiff == 0) {
        sizeDiff = diffEqMat.size();
    }
    if (diffEqVec.size() > 0 && sizeDiff == 0) {
        sizeDiff = diffEqVec.size();
    }
    if (diffIneqMat.size() > 0 && sizeDiff == 0) {
        sizeDiff = diffIneqMat.size();
    }
    if (diffIneqVec.size() > 0 && sizeDiff == 0) {
        sizeDiff = diffIneqVec.size();
    }
    //Check problem sizes
    size_t sizeSol = solution.size();
    size_t sizeEq = dualEq.size();
    size_t sizeIneqAll = ineqMat.rows();
    size_t sizeIneqActive = dualIneqActive.size();
    if (
        (size_t)costMat.rows() != sizeSol ||
        (size_t)costMat.cols() != sizeSol ||
        (size_t)eqMat.rows() != sizeEq ||
        (size_t)eqMat.cols() != sizeSol ||
        (size_t)ineqMat.rows() < sizeIneqActive ||
        (size_t)ineqMat.cols() != sizeSol ||
        (size_t)ineqVec.size() != sizeIneqAll
    ) {
        throw std::logic_error(
            "leph::DiffKKTQuadProg: "
            "Invalid input problem matrices or vectors size.");
    }
    //Check differentiation dimensions
    if (
        (diffCostMat.size() > 0 && diffCostMat.size() != sizeDiff) ||
        (diffCostVec.size() > 0 && diffCostVec.size() != sizeDiff) ||
        (diffEqMat.size() > 0 && diffEqMat.size() != sizeDiff) ||
        (diffEqVec.size() > 0 && diffEqVec.size() != sizeDiff) ||
        (diffIneqMat.size() > 0 && diffIneqMat.size() != sizeDiff) ||
        (diffIneqVec.size() > 0 && diffIneqVec.size() != sizeDiff)
    ) {
        throw std::logic_error(
            "leph::DiffKKTQuadProg: "
            "Diff input sets size mismatch.");
    }
    //Check differentiation matrices and vectors sizes
    for (size_t i=0;i<diffCostMat.size();i++) {
        if (
            (size_t)diffCostMat[i].rows() != sizeSol ||
            (size_t)diffCostMat[i].cols() != sizeSol
        ) {
            throw std::logic_error(
                "leph::DiffKKTQuadProg: "
                "Invalid cost matrix size.");
        }
    }
    for (size_t i=0;i<diffCostVec.size();i++) {
        if ((size_t)diffCostVec[i].size() != sizeSol) {
            throw std::logic_error(
                "leph::DiffKKTQuadProg: "
                "Invalid cost vector size.");
        }
    }
    for (size_t i=0;i<diffEqMat.size();i++) {
        if (
            (size_t)diffEqMat[i].rows() != sizeEq ||
            (size_t)diffEqMat[i].cols() != sizeSol
        ) {
            throw std::logic_error(
                "leph::DiffKKTQuadProg: "
                "Invalid equality matrix size.");
        }
    }
    for (size_t i=0;i<diffEqVec.size();i++) {
        if ((size_t)diffEqVec[i].size() != sizeEq) {
            throw std::logic_error(
                "leph::DiffKKTQuadProg: "
                "Invalid equality vector size.");
        }
    }
    for (size_t i=0;i<diffIneqMat.size();i++) {
        if (
            (size_t)diffIneqMat[i].rows() != sizeIneqActive ||
            (size_t)diffIneqMat[i].cols() != sizeSol
        ) {
            throw std::logic_error(
                "leph::DiffKKTQuadProg: "
                "Invalid inequality matrix size.");
        }
    }
    for (size_t i=0;i<diffIneqVec.size();i++) {
        if ((size_t)diffIneqVec[i].size() != sizeIneqActive) {
            throw std::logic_error(
                "leph::DiffKKTQuadProg: "
                "Invalid inequality vector size.");
        }
    }
    
    //Extract and select active rows in 
    //inequality matrices and vectors
    Eigen::MatrixXd cutIneqMat(sizeIneqActive, sizeSol);
    Eigen::VectorXd cutIneqVec(sizeIneqActive);
    for (size_t i=0;i<sizeIneqActive;i++) {
        cutIneqMat.row(i) = ineqMat.row(dualIneqIndex(i));
        cutIneqVec(i) = ineqVec(dualIneqIndex(i));
    }
    std::vector<Eigen::MatrixXd> cutDiffIneqMat;
    std::vector<Eigen::VectorXd> cutDiffIneqVec;
    cutDiffIneqMat.reserve(sizeDiff);
    cutDiffIneqVec.reserve(sizeDiff);
    for (size_t i=0;i<sizeDiff;i++) {
        cutDiffIneqMat.push_back(
            Eigen::MatrixXd::Zero(sizeIneqActive, sizeSol));
        cutDiffIneqVec.push_back(
            Eigen::VectorXd::Zero(sizeIneqActive));
        if (diffIneqMat.size() == sizeDiff) {
            for (size_t j=0;j<sizeIneqActive;j++) {
                cutDiffIneqMat[i].row(j) = 
                    diffIneqMat[i].row(dualIneqIndex(j));
            }
        } 
        if (diffIneqVec.size() == sizeDiff) {
            for (size_t j=0;j<sizeIneqActive;j++) {
                cutDiffIneqVec[i](j) = 
                    diffIneqVec[i](dualIneqIndex(j));
            }
        } 
    }
    
    //Build diagonal matrices
    Eigen::DiagonalMatrix<double, Eigen::Dynamic> diagDualIneq = 
        Eigen::DiagonalMatrix<double, Eigen::Dynamic>(dualIneqActive);
    Eigen::DiagonalMatrix<double, Eigen::Dynamic> diagErrorIneq = 
        Eigen::DiagonalMatrix<double, Eigen::Dynamic>(
            cutIneqMat*solution + cutIneqVec);

    //Build the derivatives matrix of the KKT conditions
    Eigen::MatrixXd tmpMat = Eigen::MatrixXd::Zero(
        sizeSol+sizeEq+sizeIneqActive, sizeSol+sizeEq+sizeIneqActive);
    tmpMat.block(0, 0, sizeSol, sizeSol) = costMat;
    tmpMat.block(0, sizeSol, sizeSol, sizeEq) = -eqMat.transpose();
    tmpMat.block(0, sizeSol+sizeEq, sizeSol, sizeIneqActive) = 
        -cutIneqMat.transpose();
    tmpMat.block(sizeSol, 0, sizeEq, sizeSol) = eqMat;
    tmpMat.block(sizeSol+sizeEq, 0, sizeIneqActive, sizeSol) = 
        diagDualIneq*cutIneqMat;
    tmpMat.block(sizeSol+sizeEq, sizeSol+sizeEq, 
        sizeIneqActive, sizeIneqActive) = diagErrorIneq;

    //Compute matrix decomposition
    auto decomposition = tmpMat.ldlt();

    Eigen::MatrixXd diffSol = Eigen::MatrixXd::Zero(sizeSol, sizeDiff);
    for (size_t i=0;i<sizeDiff;i++) {
        //Build the derivative vector
        Eigen::VectorXd tmpVec = Eigen::VectorXd::Zero(
            sizeSol+sizeEq+sizeIneqActive);
        if (diffCostMat.size() == sizeDiff) {
            tmpVec.segment(0, sizeSol) -= diffCostMat[i]*solution;
        }
        if (diffCostVec.size() == sizeDiff) {
            tmpVec.segment(0, sizeSol) -= diffCostVec[i];
        }
        if (diffEqMat.size() == sizeDiff) {
            tmpVec.segment(0, sizeSol) += diffEqMat[i].transpose()*dualEq;
            tmpVec.segment(sizeSol, sizeEq) -= diffEqMat[i]*solution;
        }
        if (diffEqVec.size() == sizeDiff) {
            tmpVec.segment(sizeSol, sizeEq) -= diffEqVec[i];
        }
        tmpVec.segment(0, sizeSol) += 
            cutDiffIneqMat[i].transpose()*dualIneqActive;
        tmpVec.segment(sizeSol+sizeEq, sizeIneqActive) -= 
            diagDualIneq*cutDiffIneqMat[i]*solution;
        tmpVec.segment(sizeSol+sizeEq, sizeIneqActive) -= 
            diagDualIneq*cutDiffIneqVec[i];

        //Solve the system and retrieve the 
        //solution's partial derivatives
        diffSol.col(i) = decomposition.solve(tmpVec).segment(0, sizeSol);
    }

    return diffSol; //XXX ratio 0.5 ???
}

}

#endif

