#include <stdexcept>
#include <leph_maths/RecursiveLeastSquares.hpp>

#include <iostream> //XXX TODO

namespace leph {

//TODO XXX
static void printEigenMat(const Eigen::MatrixXd& mat)
{
    auto svd = mat.jacobiSvd(Eigen::ComputeFullV | Eigen::ComputeFullU);
    std::cout << "==== Print SVD ====" << std::endl;
    std::cout << mat << std::endl;
    std::cout << "--- U:" << std::endl;
    std::cout << svd.matrixU() << std::endl;
    std::cout << "--- S:" << std::endl;
    std::cout << svd.singularValues() << std::endl;
    std::cout << "--- V:" << std::endl;
    std::cout << svd.matrixV() << std::endl;
}
        
RecursiveLeastSquares::RecursiveLeastSquares() :
    _params(),
    _cov(),
    _cov2(),
    _covInv()
{
}
        
void RecursiveLeastSquares::reset(
    const Eigen::MatrixXd initParams,
    double initCovarianceInvDiag)
{
    size_t sizeIn = initParams.cols();
    _params = initParams;
    _covInv = initCovarianceInvDiag*
        Eigen::MatrixXd::Identity(sizeIn, sizeIn);
    _cov = (1.0/initCovarianceInvDiag)*
        Eigen::MatrixXd::Identity(sizeIn, sizeIn);
    _cov2 = (1.0/initCovarianceInvDiag)*
        Eigen::MatrixXd::Identity(sizeIn, sizeIn);
}

size_t RecursiveLeastSquares::dimIn() const
{
    return _params.cols();
}
size_t RecursiveLeastSquares::dimOut() const
{
    return _params.rows();
}

void RecursiveLeastSquares::append(
    const Eigen::VectorXd& input, 
    const Eigen::VectorXd& output, 
    double forgettingFactor)
{
    //Check of internal state is initialized
    if (dimIn() == 0 || dimOut() == 0) {
        throw std::logic_error(
            "leph::RecursiveLeastSquares::append: "
            "Not initialized.");
    }

    //Check data sizes
    size_t sizeIn = input.size();
    size_t sizeOut = output.size();
    if (sizeIn != dimIn() || sizeOut != dimOut()) {
        throw std::logic_error(
            "leph::RecursiveLeastSquares::append: "
            "Invalid input or output vector size.");
    }

    Eigen::MatrixXd xxt = input*input.transpose();

    double alpha = 1.0/(input.transpose()*_cov*input);

    Eigen::MatrixXd regA = Eigen::MatrixXd::Zero(sizeOut, sizeIn);
    regA(0,0) = 1e-8;
    regA(0,1) = 0.001;
    Eigen::MatrixXd regB = Eigen::MatrixXd::Zero(sizeOut, sizeIn);
    regB(0,0) = 1.0;
    regB(0,1) = 0.0;

    /*

    Eigen::MatrixXd updatedCovInv = (
        Eigen::MatrixXd::Identity(sizeIn, sizeIn)
        - (1.0-forgettingFactor)*alpha*_covInv*xxt)*_covInv;
    _covInv = updatedCovInv + xxt;
    Eigen::MatrixXd updatedCov = 
        _cov + (1.0-forgettingFactor)/forgettingFactor*alpha*xxt;

    _cov = 
        updatedCov
        - updatedCov*xxt*updatedCov
        /(1.0 + input.transpose()*updatedCov*input);
    */
    Eigen::MatrixXd F = 
        Eigen::MatrixXd::Identity(sizeIn, sizeIn)
        - (1.0-forgettingFactor)*(alpha*_cov*xxt + 0.0*Eigen::MatrixXd::Identity(sizeIn, sizeIn));
    std::cout << "F:" << std::endl;
    std::cout << F << std::endl;

    F = 0.9*Eigen::MatrixXd::Identity(sizeIn, sizeIn);

    _cov = F*_cov + xxt /*+ regA.transpose()*regA*/;
    _cov2 = 0.995*_cov2 + xxt + regA.transpose()*regA;
    _covInv = _cov.inverse() + 0.0*Eigen::MatrixXd::Identity(sizeIn, sizeIn);

    Eigen::VectorXd error = 
        output - _params*input;
    _params = 
        _params + error*input.transpose()*_covInv
        + 0.001*(regB - _params)*_cov2.inverse();
        //+ 0.0005*(regB - _params);
        //+ (regB.transpose() - _params*regA.transpose())*regA*_covInv;
    std::cout << "params:" << std::endl;
    std::cout << _params << std::endl;
    std::cout << "regB:" << std::endl;
    std::cout << regB << std::endl;
    std::cout << "+++++++DeltaBefore:" << std::endl;
    std::cout << (regB - _params) << std::endl;
    std::cout << "+++++++:DeltaAfter" << std::endl;
    std::cout << (regB - _params)*_cov2.inverse() << std::endl;

    std::cout << "In: " << input.transpose() << " Out: " << output.transpose() << " Factor: " << forgettingFactor << std::endl;
    std::cout << "xxt: " << std::endl << xxt << std::endl;
    std::cout << "ErrorBefore: " << error << std::endl;
    std::cout << "ErrorAfter : " << (output - _params*input) << std::endl;
    std::cout << "Covariance:" << std::endl;
    std::cout << _cov << std::endl;
    std::cout << "CovarianceInverse:" << std::endl;
    std::cout << _covInv << std::endl;
    std::cout << "Params:" << _params << std::endl;

    printEigenMat(_cov);
}
        
const Eigen::MatrixXd& RecursiveLeastSquares::params() const
{
    return _params;
}
        
const Eigen::MatrixXd& RecursiveLeastSquares::covariance() const
{
    return _cov;
}
const Eigen::MatrixXd& RecursiveLeastSquares::covarianceInverse() const
{
    return _covInv;
}
        
Eigen::VectorXd RecursiveLeastSquares::predict(
    const Eigen::VectorXd& input) const
{
    return _params*input;
}

}

