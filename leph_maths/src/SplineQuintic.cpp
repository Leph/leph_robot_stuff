#include <algorithm>
#include <leph_maths/SplineQuintic.hpp>

namespace leph {

void SplineQuintic::addPoint(
    double time, 
    double pos, 
    double vel, 
    double acc)
{
    _points.push_back({time, pos, vel, acc});
    computeSplines();
}

const std::vector<SplineQuintic::Point>& SplineQuintic::points() const
{
    return _points;
}
std::vector<SplineQuintic::Point>& SplineQuintic::points()
{
    return _points;
}

void SplineQuintic::computeSplines() 
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

    //Build fifth order polynomial
    //with position, velocity and acceleration continuity
    for (size_t i=1;i<_points.size();i++) {
        double time = _points[i].time - _points[i-1].time;
        if (time > 1e-6) {
            Polynomial poly;
            poly.fitQuintic(
                time,
                _points[i-1].position, _points[i-1].velocity, _points[i-1].acceleration,
                _points[i].position, _points[i].velocity, _points[i].acceleration);
            Spline::_splines.push_back({
                poly,
                _points[i-1].time,
                _points[i].time
            });
        }
    }
}

}

