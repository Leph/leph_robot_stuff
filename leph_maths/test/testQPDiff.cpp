#include <iostream>
#include <Eigen/Dense>
#include <leph_maths/DiffKKTQuadProg.h>
#include <leph_eiquadprog/leph_eiquadprog.hpp>

int main()
{
    size_t sizeSol = 2;
    size_t sizeEq = 0;
    size_t sizeIneq = 1;
    Eigen::MatrixXd problemCostMat = 
        Eigen::MatrixXd::Zero(sizeSol, sizeSol);
    Eigen::VectorXd problemCostVec = 
        Eigen::VectorXd::Zero(sizeSol);
    Eigen::MatrixXd problemEqMat = 
        Eigen::MatrixXd::Zero(sizeEq, sizeSol);
    Eigen::VectorXd problemEqVec = 
        Eigen::VectorXd::Zero(sizeEq);
    Eigen::MatrixXd problemIneqMat = 
        Eigen::MatrixXd::Zero(sizeIneq, sizeSol);
    Eigen::VectorXd problemIneqVec = 
        Eigen::VectorXd::Zero(sizeIneq);
    Eigen::VectorXd problemSolution = 
        Eigen::VectorXd::Zero(sizeSol);
    Eigen::VectorXd problemDual =  
        Eigen::VectorXd::Zero(sizeEq + sizeIneq);

    problemCostMat <<
        1.0, 0.0,
        0.0, 1.0;
    problemIneqMat << 
        0.0, 1.0;
    problemIneqVec <<
        -0.8;
    
    Eigen::VectorXd dualEq;
    Eigen::VectorXd dualIneqActive;
    Eigen::VectorXi dualIneqIndex;
    double cost = Eigen::solve_quadprog(
        problemCostMat,
        problemCostVec,
        problemEqMat.transpose(),
        problemEqVec,
        problemIneqMat.transpose(),
        problemIneqVec,
        problemSolution,
        &dualEq, &dualIneqActive, &dualIneqIndex);
    problemDual.segment(0, sizeEq) = dualEq;
    for (size_t i=0;i<(size_t)dualIneqIndex.size();i++) {
        problemDual(sizeEq + dualIneqIndex(i)) = dualIneqActive(i);
    }

    std::cout << "Cost=" << cost << " Sol=" << problemSolution.transpose() << " Dual=" << problemDual.transpose() << std::endl;
    std::cout << "KKT1: " << (problemCostMat*problemSolution - problemEqMat.transpose()*problemDual.segment(0, sizeEq) - problemIneqMat.transpose()*problemDual.segment(sizeEq, sizeIneq)).transpose() << std::endl;
    std::cout << "KKT2: " << (problemEqMat*problemSolution + problemEqVec).transpose() << std::endl;
    Eigen::MatrixXd diag = Eigen::DiagonalMatrix<double, Eigen::Dynamic>(problemDual.segment(sizeEq, sizeIneq));
    std::cout << "KKT3: " << (diag*(problemIneqMat*problemSolution + problemIneqVec)).transpose() << std::endl;

    Eigen::VectorXd diffSol = leph::DiffKKTQuadProg(
        problemSolution,
        dualEq,
        dualIneqActive,
        dualIneqIndex,
        problemCostMat,
        problemEqMat,
        problemIneqMat,
        problemIneqVec,
        Eigen::MatrixXd::Zero(sizeSol, sizeSol),
        Eigen::VectorXd::Zero(sizeSol),
        Eigen::MatrixXd::Zero(sizeEq, sizeSol),
        Eigen::VectorXd::Zero(sizeEq),
        Eigen::MatrixXd::Zero(sizeIneq, sizeSol),
        Eigen::VectorXd::Ones(sizeIneq));
    std::cout << "Analytical diff: " << diffSol.transpose() << std::endl;

    //Numerically compute the 
    //QP solution differentiation
    {
        Eigen::solve_quadprog(
            problemCostMat,
            problemCostVec,
            problemEqMat.transpose(),
            problemEqVec,
            problemIneqMat.transpose(),
            problemIneqVec,
            problemSolution,
            &problemDual);
        Eigen::VectorXd sol1 = problemSolution;
        problemIneqVec(0) += 0.001;
        Eigen::solve_quadprog(
            problemCostMat,
            problemCostVec,
            problemEqMat.transpose(),
            problemEqVec,
            problemIneqMat.transpose(),
            problemIneqVec,
            problemSolution,
            &problemDual);
        Eigen::VectorXd sol2 = problemSolution;
        std::cout << "Numerical diff: " 
            << ((1.0/0.001)*(sol2-sol1)).transpose() << std::endl;;
    }

    return 0;
}

