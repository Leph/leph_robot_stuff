#include <leph_model/ActuatorSEA.hpp>
#include <leph_maths/Sign.h>
#include <leph_maths/Clamp.h>

namespace leph {

ActuatorSEA::ActuatorSEA() :
    _motorPos(0.0),
    _motorVel(0.0),
    _motorIsStiction(true),
    _jointPos(0.0),
    _jointVel(0.0)
{
    //Parameters default configuration
    resetParameters();
}
        
void ActuatorSEA::resetParameters()
{
    //Parameters described in:
    //"Implementation, Identification and Control
    //of an Efficient Electric Actuator for Humanoid Robots"
    _paramMotorInertia = 1.38e-5;
    _paramMotorFrictionLinear = 0.003;
    _paramMotorFrictionOffset = 0.1;
    _paramSpringStiffness = 588.0;
    _paramJointInertia = 8.5e-4;
    _paramJointFrictionLinear = 0.278;
    _paramControlSpringRatio = 1.0;
    _paramControlSpringOffset = 0.0;
    _paramControlGainPos = 0.001;
    _paramControlGainVel = 0.00000001;
    _paramControlMaxEffort = 2000.0;
}

double ActuatorSEA::stateMotorPos() const
{
    return _motorPos;
}
double& ActuatorSEA::stateMotorPos()
{
    return _motorPos;
}
double ActuatorSEA::stateMotorVel() const
{
    return _motorVel;
}
double& ActuatorSEA::stateMotorVel()
{
    return _motorVel;
}
bool ActuatorSEA::stateMotorIsStiction() const
{
    return _motorIsStiction;
}
bool& ActuatorSEA::stateMotorIsStiction()
{
    return _motorIsStiction;
}
double ActuatorSEA::stateJointPos() const
{
    return _jointPos;
}
double& ActuatorSEA::stateJointPos()
{
    return _jointPos;
}
double ActuatorSEA::stateJointVel() const
{
    return _jointVel;
}
double& ActuatorSEA::stateJointVel()
{
    return _jointVel;
}

double ActuatorSEA::springDeflectionPos() const
{
    return _jointPos - _motorPos;
}
double ActuatorSEA::springDeflectionVel() const
{
    return _jointVel - _motorVel;
}
double ActuatorSEA::springTorque() const
{
    return _paramSpringStiffness*springDeflectionPos();
}
double ActuatorSEA::springTorqueVel() const
{
    return _paramSpringStiffness*springDeflectionVel();
}
        
double ActuatorSEA::computeFrictionMotor(double vel) const
{
    if (
        _paramMotorFrictionLinear <= 0.0 || 
        _paramMotorFrictionOffset <= 0.0
    ) {
        throw std::logic_error(
            "leph::ActuatorSEA::computeFrictionMotor: "
            "Invalid friction parameters.");
    }

    return 
        -_paramMotorFrictionLinear*vel 
        -_paramMotorFrictionOffset*leph::Sign(vel);
}
double ActuatorSEA::computeFrictionJoint(double vel) const
{
    if (_paramJointFrictionLinear <= 0.0) {
        throw std::logic_error(
            "leph::ActuatorSEA::computeFrictionJoint: "
            "Invalid friction parameters.");
    }

    return -_paramJointFrictionLinear*vel;
}

double ActuatorSEA::computeControl(
    double torqueDesired,
    double torqueSpring,
    double torqueSpringVel) const
{
    if (
        _paramControlSpringRatio <= 0.0 || 
        _paramControlGainPos <= 0.0 ||
        _paramControlGainVel <= 0.0 ||
        _paramControlMaxEffort <= 0.0
    ) {
        throw std::logic_error(
            "leph::ActuatorSEA::computeControl: "
            "Invalid controller parameters.");
    }

    //Simulate error in spring stiffness model
    //from internal controller point of view
    double readTorqueSpring = 
        _paramControlSpringRatio*torqueSpring 
        + _paramControlSpringOffset;
    double readTorqueSpringVel = 
        _paramControlSpringRatio*torqueSpringVel;

    //PD control and feedforward term
    double effort = 
        torqueDesired
        + _paramControlGainPos*(torqueDesired-readTorqueSpring)
        - _paramControlGainVel*readTorqueSpringVel;

    //Bound control torque
    return ClampAbsolute(effort, _paramControlMaxEffort);
}

void ActuatorSEA::update(
    double dt,
    double torqueDesired,
    double torqueExternal)
{
    if (
        _paramMotorInertia <= 0.0 || 
        _paramJointInertia <= 0.0 || 
        _paramSpringStiffness <= 0.0
    ) {
        throw std::logic_error(
            "leph::ActuatorSEA::update: "
            "Invalid dynamics parameters.");
    }

    //Compute torque applied on motor and joint load
    double torqueFrictionMotor = computeFrictionMotor(_motorVel);
    double torqueFrictionJoint = computeFrictionJoint(_motorVel);
    double torqueSpring = springTorque();
    double torqueControl = computeControl(
        torqueDesired, torqueSpring, springTorqueVel());

    //Compute system acceleration
    double accMotor = 
        (torqueSpring + torqueFrictionMotor - torqueControl)
        /_paramMotorInertia;
    double accJoint = 
        (torqueExternal + torqueFrictionJoint - torqueSpring)
        /_paramJointInertia;

    //Integrate system state
    _motorVel += dt*accMotor;
    _jointVel += dt*accJoint;
    _motorPos += dt*_motorVel;
    _jointPos += dt*_jointVel;
}

}

