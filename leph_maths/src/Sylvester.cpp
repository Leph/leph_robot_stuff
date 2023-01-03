#include <stdexcept>
#include <leph_maths/Sylvester.h>

extern "C" {

/**
 * Binding to LAPACK function.
 * DTRSYL solves the real Sylvester matrix equation.
 * For documentation, see:
 * http://apfel.mathematik.uni-ulm.de/~lehn/lapack/plain-lapack-3.3.1/dtrsyl.html
 */
void dtrsyl_(
    const char* transa, 
    const char* transb, 
    const int* isgn, 
    const int* m, 
    const int* n, 
    const double* a, 
    const int* lda, 
    const double* b, 
    const int* ldb, 
    double* c, 
    const int* ldc, 
    double* scale, 
    int* info);

}

namespace leph {

bool Sylvester(
    const Eigen::MatrixXd& A,
    const Eigen::MatrixXd& B,
    const Eigen::MatrixXd& C,
    Eigen::MatrixXd& X)
{
    //Check sizes
    if (
        A.rows() != A.cols() ||
        B.rows() != B.cols()
    ) {
        throw std::logic_error(
            "leph::Sylvester: "
            "Input problem matrices not squared.");
    }
    if (
        C.rows() != A.rows() ||
        C.cols() != B.cols()
    ) {
        throw std::logic_error(
            "leph::Sylvester: "
            "Input bias matrix invalid size.");
    }

    //Compute the Schur decomposition of problem 
    //matrices such that A = Q.U.Q^{-1}
    //with Q unitary matrix (Q^{-1} = Q^{T})
    //with U quasi upper triangular
    Eigen::RealSchur<Eigen::MatrixXd> schurA(A);
    Eigen::RealSchur<Eigen::MatrixXd> schurB(B);

    //Reduce the problem such that A and B matrices
    //are triangular using the Schur decomposition
    //A.X + X.B = C
    //(QA.UA.QA^{-1}).X + X.(QB.UB.QB^{-1}) = C
    //UA.(QA^{-1}.X.QB) + (QA^{-1}.X.QB).UB = QA^{-1}.C.QB
    Eigen::MatrixXd Y = schurA.matrixU().transpose()*C*schurB.matrixU();

    //LAPACK arguments
    char transa = 'N';
    char transb = 'N';
    int isgn = 1;
    int m = C.rows();
    int n = C.cols();
    int lda = schurA.matrixT().outerStride();
    int ldb = schurB.matrixT().outerStride();
    int ldc = Y.outerStride();
    double scale = 1.0;
    int info = 0;
    //Call LAPACK solver
    dtrsyl_(
        &transa, &transb, &isgn, &m, &n, 
        schurA.matrixT().data(), &lda, 
        schurB.matrixT().data(), &ldb, 
        Y.data(), &ldc,
        &scale, &info);
    //Check result success
    if (info < 0) {
        throw std::logic_error(
            "leph::Sylvester: "
            "Invalid argument from LAPACK "
            "(this should never happen).");
    } else if (info == 1) {
        return false;
    }

    //Convert back the result 
    //to original formulation
    Y /= scale;
    X = schurA.matrixU()*Y*schurB.matrixU().transpose();

    return true;
}

}

