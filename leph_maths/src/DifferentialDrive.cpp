#include <stdexcept>
#include <leph_maths/DifferentialDrive.hpp>
#include <leph_maths/Clamp.h>

namespace leph {

DifferentialDrive::DifferentialDrive() :
    _wheelRadius(0.0),
    _wheelDistance(0.0),
    _maxVelLin(0.0),
    _maxVelAng(0.0),
    _filterLowpassLin(),
    _filterBangbangLin(),
    _filterLowpassAng(),
    _filterBangbangAng()
{
}

void DifferentialDrive::setParameters(
    double wheelRadius,
    double wheelDistance,
    double cutoffFreq,
    double maxVelLin,
    double maxVelAng,
    double maxAccLin,
    double maxAccAng)
{
    //Check parameters
    if (
        wheelRadius <= 0.0 ||
        wheelDistance <= 0.0 ||
        cutoffFreq <= 0.0 ||
        maxVelLin <= 0.0 ||
        maxVelAng <= 0.0 ||
        maxAccLin <= 0.0 ||
        maxAccAng <= 0.0
    ) {
        throw std::logic_error(
            "leph::DifferentialDrive::setParameters: "
            "Invalid parameters.");
    }

    //Assign parameters
    _wheelRadius = wheelRadius;
    _wheelDistance = wheelDistance;
    _filterLowpassLin.cutoffFrequency() = cutoffFreq;
    _filterLowpassAng.cutoffFrequency() = cutoffFreq;
    _maxVelLin = maxVelLin;
    _maxVelAng = maxVelAng;
    _filterBangbangLin.maxVel() = maxAccLin;
    _filterBangbangAng.maxVel() = maxAccAng;
    _filterBangbangLin.maxAcc() = 2.0*maxAccLin;
    _filterBangbangAng.maxAcc() = 2.0*maxAccAng;
}

void DifferentialDrive::update(
    double dt,
    double cmdVelLin,
    double cmdVelAng)
{
    //Check time step
    if (dt < 0.0) {
        throw std::logic_error(
            "leph::DifferentialDrive::update: "
            "Invalid time step.");
    }

    //Clamp maximum velocity
    cmdVelLin = ClampAbsolute(cmdVelLin, _maxVelLin);
    cmdVelAng = ClampAbsolute(cmdVelAng, _maxVelAng);

    //Filter command
    _filterLowpassLin.update(cmdVelLin, dt);
    _filterLowpassAng.update(cmdVelAng, dt);
    _filterBangbangLin.update(_filterLowpassLin.value(), dt);
    _filterBangbangAng.update(_filterLowpassAng.value(), dt);
}

double DifferentialDrive::getEffortWheelLeft() const
{
    double velLin = _filterBangbangLin.value();
    double velAng = _filterBangbangAng.value();
    return (1.0/_wheelRadius)*(velLin - velAng*_wheelDistance);
}
double DifferentialDrive::getEffortWheelRight() const
{
    double velLin = _filterBangbangLin.value();
    double velAng = _filterBangbangAng.value();
    return (1.0/_wheelRadius)*(velLin + velAng*_wheelDistance);
}
        
void DifferentialDrive::reset()
{
    _filterLowpassLin.reset(0.0);
    _filterLowpassAng.reset(0.0);
    _filterBangbangLin.reset(0.0);
    _filterBangbangAng.reset(0.0);
}

}

