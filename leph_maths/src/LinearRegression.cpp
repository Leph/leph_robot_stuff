#include <cmath>
#include <stdexcept>
#include <leph_maths/LinearRegression.hpp>

namespace leph {

LinearRegression::LinearRegression() :
    _inputs(),
    _outputs(),
    _params(),
    _regMat(),
    _regBias(),
    _regWeight()
{
}
        
void LinearRegression::clear()
{
    _inputs = Eigen::MatrixXd();
    _outputs = Eigen::MatrixXd();
    _params = Eigen::MatrixXd();
}
        
size_t LinearRegression::count() const
{
    return _inputs.cols();
}

size_t LinearRegression::dimIn() const
{
    return _inputs.rows();
}
size_t LinearRegression::dimOut() const
{
    return _outputs.rows();
}
size_t LinearRegression::dimReg() const
{
    return _regMat.rows();
}

void LinearRegression::setRegularization(
    const Eigen::MatrixXd& regMat,
    const Eigen::MatrixXd& regBias,
    const Eigen::DiagonalMatrix<double, Eigen::Dynamic>& regWeight)
{
    if (
        regMat.rows() != regBias.rows() ||
        regMat.rows() != regWeight.rows()
    ) {
        throw std::logic_error(
            "leph::LinearRegression::setRegularization: "
            "Mismatch in row sizes.");
    }
    if (
        (dimIn() != 0 && (size_t)regMat.cols() != dimIn()) ||
        (dimOut() != 0 && (size_t)regBias.cols() != dimOut())
    ) {
        throw std::logic_error(
            "leph::LinearRegression::setRegularization: "
            "Invalid col sizes.");
    }
    _regMat = regMat;
    _regBias = regBias;
    _regWeight = regWeight;
}

void LinearRegression::append(
    const Eigen::VectorXd& input, 
    const Eigen::VectorXd& output, 
    double weight)
{
    size_t size = count();
    size_t sizeIn = dimIn();
    size_t sizeOut = dimOut();
    double sqrtWeight = std::sqrt(weight);
    if (size == 0) {
        _inputs = sqrtWeight*input;
        _outputs = sqrtWeight*output;
    } else {
        if (sizeIn != (size_t)input.size()) {
            throw std::logic_error(
                "leph::LinearRegression::append: "
                "Invalid input dimension.");
        }
        if (sizeOut != (size_t)output.size()) {
            throw std::logic_error(
                "leph::LinearRegression::append: "
                "Invalid output dimension.");
        }
        _inputs.conservativeResize(Eigen::NoChange_t(), size+1);
        _inputs.rightCols(1) = sqrtWeight*input;
        _outputs.conservativeResize(Eigen::NoChange_t(), size+1);
        _outputs.rightCols(1) = sqrtWeight*output;
    }
}

const Eigen::MatrixXd& LinearRegression::fit()
{
    //Empty check
    if (count() == 0 || dimIn() == 0) {
        throw std::logic_error(
            "leph::LinearRegression::fit: Empty data");
    }
    
    //Compute terms
    Eigen::MatrixXd XXt = _inputs*_inputs.transpose();
    Eigen::MatrixXd YXt = _outputs*_inputs.transpose();

    //Solve the fitting analytically
    bool isSuccess;
    if (dimReg() == 0) {
        //Classic linear regression 
        //without regularization
        Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(XXt);
        isSuccess = (qr.info() == Eigen::Success);
        if (isSuccess) {
            _params = YXt*qr.inverse();
        }
    } else {
        //Multivariate linear regression with
        //the regularization term
        Eigen::MatrixXd AtA = 
            _regMat.transpose()*_regWeight*_regMat;
        Eigen::MatrixXd BtA =
            _regBias.transpose()*_regWeight*_regMat;
        Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(XXt + AtA);
        isSuccess = (qr.info() == Eigen::Success);
        if (isSuccess) {
            _params = (YXt + BtA)*qr.inverse();
        }
    }
    if (!isSuccess) {
        throw std::runtime_error(
            "leph::LinearRegression:fit: "
            "Failed to fit the data.");
    }

    return _params;
}
 
const Eigen::MatrixXd& LinearRegression::params() const
{
    return _params;
}
        
Eigen::VectorXd LinearRegression::predict(
    const Eigen::VectorXd& input) const
{
    if (input.size() != _params.cols()) {
        throw std::logic_error(
            "leph::LinearRegression::predict: "
            "Model not fitted.");
    }

    return _params*input;
}

const Eigen::MatrixXd& LinearRegression::inputs() const
{
    return _inputs;
}
const Eigen::MatrixXd& LinearRegression::outputs() const
{
    return _outputs;
}
        
Eigen::MatrixXd LinearRegression::residuals() const
{
    if (count() == 0 || dimIn() == 0) {
        throw std::logic_error(
            "leph::LinearRegression::residuals: "
            "Empty data.");
    }
    if (dimIn() != (size_t)_params.cols()) {
        throw std::logic_error(
            "leph::LinearRegression::residuals: "
            "Model not fitted.");
    }

    return 
        _outputs - _params*_inputs;
}
        
Eigen::VectorXd LinearRegression::meanSquaredError() const
{
    Eigen::MatrixXd res = residuals();
    return (1.0/count())*res.rowwise().squaredNorm();
}
Eigen::VectorXd LinearRegression::rootMeanSquaredError() const
{
    return meanSquaredError().cwiseSqrt();
}
        
Eigen::VectorXd LinearRegression::maxError() const
{
    Eigen::MatrixXd res = residuals();
    Eigen::VectorXd maxVect(dimOut());
    for (size_t i=0;i<(size_t)maxVect.size();i++) {
        maxVect(i) = res.row(i).cwiseAbs().maxCoeff();
    }
    return maxVect;
}
        
Eigen::VectorXd LinearRegression::variance() const
{
    if (count() > 1) {
        Eigen::MatrixXd res = residuals();
        Eigen::VectorXd mean = res.cwiseAbs().rowwise().mean();
        for (size_t i=0;i<(size_t)res.cols();i++) {
            res.col(i) -= mean;
        }
        return (1.0/(count()-1.0))*res.rowwise().squaredNorm();
    } else {
        return Eigen::VectorXd::Zero(dimOut());
    }
}

void LinearRegression::print(std::ostream& os) const
{
    if (count() == 0 || dimIn() == 0) {
        throw std::logic_error(
            "leph::LinearRegression::print: "
            "Empty data.");
    }
    if (dimIn() != (size_t)_params.size()) {
        throw std::logic_error(
            "leph::LinearRegression::print: "
            "Model not fitted.");
    }

    os << "LinearRegression count=" << count() 
        << " dimIn=" << dimIn() 
        << " dimOut=" << dimOut() 
        << " dimReg=" << dimReg() 
        << std::endl;
    os << "Params: " << std::endl << _params << std::endl;
    os << "MSE   : " << meanSquaredError().transpose() << std::endl;
    os << "RMSE  : " << rootMeanSquaredError().transpose() << std::endl;
    os << "MaxE  : " << maxError().transpose() << std::endl;
    os << "Var   : " << variance().transpose() << std::endl;
    os << "Std   : " << variance().cwiseSqrt().transpose() << std::endl;
}

}

