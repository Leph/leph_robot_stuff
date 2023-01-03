#include <algorithm>
#include <leph_maths/SplineCubic.hpp>

namespace leph {

void SplineCubic::addPoint(
    double time, 
    double pos, 
    double vel)
{
    _points.push_back({time, pos, vel});
    computeSplines();
}

const std::vector<SplineCubic::Point>& SplineCubic::points() const
{
    return _points;
}
std::vector<SplineCubic::Point>& SplineCubic::points()
{
    return _points;
}
        
void SplineCubic::computeSplines() 
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

    //Build third order polynomial
    //with position and velocity continuity
    for (size_t i=1;i<_points.size();i++) {
        double time = _points[i].time - _points[i-1].time;
        if (time > 1e-6) {
            Polynomial poly;
            poly.fitCubic(
                time,
                _points[i-1].position, _points[i-1].velocity,
                _points[i].position, _points[i].velocity);
            Spline::_splines.push_back({
                poly,
                _points[i-1].time,
                _points[i].time
            });
        }
    }
}

}

