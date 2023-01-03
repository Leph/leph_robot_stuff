#include <cmath>
#include <leph_model/ForwardSimulation.hpp>
#include <leph_maths/Angle.h>
#include <leph_maths/IntegrateDOFVect.h>

namespace leph {

ForwardSimulation::ForwardSimulation(Model& model) :
    _model(&model),
    _positions(Eigen::VectorXd::Zero(_model->sizeVectPos())),
    _velocities(Eigen::VectorXd::Zero(_model->sizeVectVel())),
    _torques(Eigen::VectorXd::Zero(_model->sizeVectVel()))
{
    if (_positions.size() <= 7) {
        throw std::runtime_error(
            "leph::ForwardSimulation: Invalid state vector size.");
    }

    //Load initial position state
    _positions = _model->getDOFPosVect();
}
        
const Eigen::VectorXd& ForwardSimulation::positions() const
{
    return _positions;
}
Eigen::VectorXd& ForwardSimulation::positions()
{
    return _positions;
}
const Eigen::VectorXd& ForwardSimulation::velocities() const
{
    return _velocities;
}
Eigen::VectorXd& ForwardSimulation::velocities()
{
    return _velocities;
}
const Eigen::VectorXd& ForwardSimulation::torques() const
{
    return _torques;
}
Eigen::VectorXd& ForwardSimulation::torques()
{
    return _torques;
}

void ForwardSimulation::update(
    double dt,
    RBDL::ConstraintSet& constraints)
{
    //Compute directly next velocity 
    //using impulsive dynamics
    Eigen::VectorXd nextVel = 
        _model->forwardImpulsiveDynamicsContacts(
            dt, 
            constraints,
            _positions,
            _velocities,
            _torques);
    
    //Compute next state with semi-implicit 
    //Euler integration 
    _velocities = nextVel;
    //Integration of position state.
    //Special handling of floating base 
    //quaternion orientation.
    _positions = IntegrateDOFVect(
        _positions, dt*_velocities);

    //Check numerical validity
    if (
        !_positions.allFinite() ||
        !_velocities.allFinite() ||
        !_torques.allFinite()
    ) {
        throw std::runtime_error(
            "leph::ForwardSimulation::update: "
            "Numerical instability");
    }
    
    //Normalize joint angular position
    for (size_t i=0;i<_model->sizeJoint();i++) {
        _positions(6+i) = AngleBound(_positions(6+i));
        //Check again numerical stability
        if (
            std::fabs(_positions(6+i)) > 1e10 || 
            std::fabs(_velocities(6+i)) > 1e10
        ) {
            throw std::runtime_error(
                "leph::ForwardSimulation::update: "
                "Numerical instability");
        }
    }
    
    //Assign model new position 
    //and velocity state
    _model->setDOFPosVect(_positions);
    _model->setDOFVelVect(_velocities);
}

}

