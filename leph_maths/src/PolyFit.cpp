#include <stdexcept>
#include <leph_maths/PolyFit.hpp>

namespace leph {

PolyFit::PolyFit(unsigned int degree) :
    _degree(degree),
    _regression()
{
    if (_degree <= 0) {
        throw std::logic_error("leph::PolyFit: Invalid degree.");
    }
}
        
void PolyFit::add(double t, double val, double weight)
{
    Eigen::VectorXd in(_degree + 1);
    Eigen::VectorXd out(1);
    double expT = 1.0;
    for (size_t k=0;k<_degree+1;k++) {
        in(k) = expT;
        expT *= t;
    }
    out(0) = val;
    _regression.append(in, out, weight);
}
        
Polynomial PolyFit::fit()
{
    _regression.fit();
    
    Polynomial polynom;
    polynom.getCoefs().resize(_degree + 1);
    for (size_t k=0;k<_degree+1;k++) {
        polynom.getCoefs()[k] = 
            _regression.params()(0, k);
    }

    return polynom;
}
        
const LinearRegression& PolyFit::regression() const
{
    return _regression;
}

}

