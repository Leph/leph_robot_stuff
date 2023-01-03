#include <iomanip>
#include <stdexcept>
#include <leph_maths/Spline.hpp>

namespace leph {

double Spline::pos(double t) const
{
    return interpolation(t, &Polynomial::pos);
}
double Spline::vel(double t) const
{
    return interpolation(t, &Polynomial::vel);
}
double Spline::acc(double t) const
{
    return interpolation(t, &Polynomial::acc);
}
double Spline::jerk(double t) const
{
    return interpolation(t, &Polynomial::jerk);
}
        
double Spline::min() const
{
    if (_splines.size() == 0) {
        return 0.0;
    } else {
        return _splines.front().min;
    }
}
double Spline::max() const
{
    if (_splines.size() == 0) {
        return 0.0;
    } else {
        return _splines.back().max;
    }
}
        
size_t Spline::size() const
{
    return _splines.size();
}
        
const Spline::Spline_t& Spline::part(size_t index) const
{
    return _splines.at(index);
}
        
double Spline::interpolation(double t, 
    double(Polynomial::*func)(double) const) const
{
    //Empty case
    if (_splines.size() == 0) {
        return 0.0;
    }
    //Bound asked abscissa into spline range
    if (t <= _splines.front().min) {
        t = _splines.front().min;
    }
    if (t >= _splines.back().max) {
        t = _splines.back().max;
    }
    //Bijection spline search
    size_t indexLow = 0;
    size_t indexUp = _splines.size()-1;
    while (indexLow != indexUp) {
        size_t index = (indexUp+indexLow)/2;
        if (t < _splines[index].min) {
            indexUp = index-1;
        } else if (t > _splines[index].max) {
            indexLow = index+1;
        } else {
            indexUp = index;
            indexLow = index;
        }
    }
    //Compute and return spline value
    return (_splines[indexUp].polynom.*func)
        (t - _splines[indexUp].min);
}

}

