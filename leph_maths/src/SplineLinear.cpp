#include <stdexcept>
#include <algorithm>
#include <leph_maths/SplineLinear.hpp>

namespace leph {

void SplineLinear::addPoint(double time, double position)
{
    _points.push_back({time, position});
    computeSplines();
}

const std::vector<SplineLinear::Point>& SplineLinear::points() const
{
    return _points;
}
std::vector<SplineLinear::Point>& SplineLinear::points()
{
    return _points;
}

void SplineLinear::computeSplines() 
{
    //Reset internal spline parts
    Spline::_splines.clear();
    if (_points.size() < 2) {
        return;
    }

    //Sort points container by increasing time
    std::sort(
        _points.begin(), 
        _points.end(), 
        [](const Point& p1, const Point& p2) -> bool { 
            return p1.time < p2.time;
        });

    //Build first order polynomial
    //with position continuity
    for (size_t i=1;i<_points.size();i++) {
        double time = _points[i].time - _points[i-1].time;
        if (time > 1e-6) {
            Polynomial poly(1);
            poly(0) = _points[i-1].position;
            poly(1) = (_points[i].position - _points[i-1].position)/time;
            Spline::_splines.push_back({
                poly,
                _points[i-1].time,
                _points[i].time
            });
        }
    }
}

}

