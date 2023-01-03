#include <stdexcept>
#include <leph_model/SEIKOFloatingBase.hpp>
#include <leph_eiquadprog/leph_eiquadprog.hpp>
#include <leph_maths/AxisAngle.h>
#include <leph_maths/Clamp.h>
#include <leph_maths/IntegrateDOFVect.h>

namespace leph {

SEIKOFloatingBase::SEIKOFloatingBase() :
    _model(nullptr),
    _sizeDOF(0),
    _sizeJoint(0),
    _sizePlaneOn(0),
    _sizePointOn(0),
    _sizePlaneOff(0),
    _sizePointOff(0),
    _contactsPlane(),
    _contactsPoint(),
    _mappingContacts(),
    _jointLimitPosLower(),
    _jointLimitPosUpper(),
    _jointLimitVelAbs(),
    _jointLimitTauAbs(),
    _jointTargetPos(),
    _jointWeightPos(),
    _jointWeightVel(),
    _jointClampPos(),
    _jointTargetTau(),
    _jointWeightTau(),
    _gravityVector(),
    _jointComputedDeltaTau(),
    _jointComputedDeltaDOF(),
    _jointStateTau(),
    _biasConstraintEquilibrium(),
    _externalBaseWrench(),
    _pinocchio()
{
}

SEIKOFloatingBase::SEIKOFloatingBase(Model& model) :
    _model(&model),
    _sizeDOF(0),
    _sizeJoint(0),
    _sizePlaneOn(0),
    _sizePointOn(0),
    _sizePlaneOff(0),
    _sizePointOff(0),
    _contactsPlane(),
    _contactsPoint(),
    _mappingContacts(),
    _jointLimitPosLower(),
    _jointLimitPosUpper(),
    _jointLimitVelAbs(),
    _jointLimitTauAbs(),
    _jointTargetPos(),
    _jointWeightPos(),
    _jointWeightVel(),
    _jointClampPos(),
    _jointTargetTau(),
    _jointWeightTau(),
    _gravityVector(),
    _jointComputedDeltaTau(),
    _jointComputedDeltaDOF(),
    _jointStateTau(),
    _biasConstraintEquilibrium(),
    _externalBaseWrench(),
    _pinocchio(model)
{
    init(model);
}
        
void SEIKOFloatingBase::init(Model& model)
{
    _model = &model;
    _sizeDOF = 0;
    _sizeJoint = 0;
    _sizePlaneOn = 0;
    _sizePointOn = 0;
    _sizePlaneOff = 0;
    _sizePointOff = 0;
    //Reset
    _contactsPlane.clear();
    _contactsPoint.clear();
    _mappingContacts.clear();
    //Joint default initialization
    _sizeDOF = _model->sizeDOF();
    _sizeJoint = _model->sizeJoint();
    //Limits initialization
    _jointLimitPosLower = _model->jointLimitsLower();
    _jointLimitPosUpper = _model->jointLimitsUpper();
    _jointLimitVelAbs = _model->jointLimitsVelocity();
    _jointLimitTauAbs = _model->jointLimitsTorque();
    //Targets and weights default initialization
    _jointTargetPos = _model->getJointPosVect();
    _jointWeightPos = 1.0*Eigen::VectorXd::Ones(_sizeJoint);
    _jointWeightVel = 1.0*Eigen::VectorXd::Ones(_sizeJoint);
    _jointClampPos = 0.2;
    _jointTargetTau = Eigen::VectorXd::Zero(_sizeJoint);
    _jointWeightTau = 1.0*Eigen::VectorXd::Ones(_sizeJoint);
    _gravityVector = Eigen::VectorXd::Zero(_sizeDOF);
    _jointComputedDeltaTau = Eigen::VectorXd::Zero(_sizeJoint);
    _jointComputedDeltaDOF = Eigen::VectorXd::Zero(_sizeDOF);
    _jointStateTau = Eigen::VectorXd::Zero(_sizeJoint);
    //Equilibrium bias and external wrench
    _biasConstraintEquilibrium = Eigen::Vector6d::Zero();
    _externalBaseWrench = Eigen::Vector6d::Zero();
    //Reset Pinocchio
    _pinocchio = PinocchioInterface(model);
}

void SEIKOFloatingBase::addContactPlane(const std::string& frameName)
{
    //Check if given name is already registered
    if (_mappingContacts.count(frameName) != 0) {
        throw std::logic_error(
            "leph::SEIKOFloatingBase::addContactPlane: "
            "Already defined frame name: " 
            + frameName);
    }

    //Default configuration
    ContactPlane_t contact;
    contact.frameId = _model->getIndexFrame(frameName);
    contact.frameName = frameName;
    contact.isEnabled = false;
    contact.targetPos = _model->position(frameName, "ROOT");
    contact.targetMat = _model->orientation(frameName, "ROOT");
    contact.weightPos = 100.0;
    contact.weightMat = 1.0;
    contact.clampPos = 0.01;
    contact.clampMat = 0.01;
    contact.frictionCoef = 0.4;
    contact.normalForceMin = 1e-2;
    contact.normalForceMax = 1e9;
    contact.limitVelNormalForce = 1e9;
    contact.jacobianWorld = Eigen::MatrixXd::Zero(6, _sizeDOF);
    contact.jacobianBody = Eigen::MatrixXd::Zero(6, _sizeDOF);
    contact.limitCOP.x() = 0.01;
    contact.limitCOP.y() = 0.01;
    contact.targetWrench = Eigen::Vector6d::Zero();
    contact.weightWrench = 1e-6*Eigen::Vector6d::Ones();
    contact.computedDeltaWrench = Eigen::Vector6d::Zero();
    contact.stateWrench = Eigen::Vector6d::Zero();

    //Add new contact plane
    _contactsPlane.push_back(contact);
    _mappingContacts.insert({frameName, _contactsPlane.size()});
    _sizePlaneOff++;
}
void SEIKOFloatingBase::addContactPoint(const std::string& frameName)
{
    //Check if given name is already registered
    if (_mappingContacts.count(frameName) != 0) {
        throw std::logic_error(
            "leph::SEIKOFloatingBase::addContactPoint: "
            "Already defined frame name: " 
            + frameName);
    }

    //Default configuration
    ContactPoint_t contact;
    contact.frameId = _model->getIndexFrame(frameName);
    contact.frameName = frameName;
    contact.isEnabled = false;
    contact.targetPos = _model->position(frameName, "ROOT");
    contact.targetMat = _model->orientation(frameName, "ROOT");
    contact.weightPos = 100.0;
    contact.weightMat = 1.0;
    contact.clampPos = 0.01;
    contact.clampMat = 0.01;
    contact.frictionCoef = 0.4;
    contact.normalForceMin = 1e-2;
    contact.normalForceMax = 1e9;
    contact.limitVelNormalForce = 1e9;
    contact.jacobianWorld = Eigen::MatrixXd::Zero(6, _sizeDOF);
    contact.jacobianBody = Eigen::MatrixXd::Zero(6, _sizeDOF);
    contact.contactMat = Eigen::Matrix3d::Identity();
    contact.targetForce = Eigen::Vector3d::Zero();
    contact.weightForce = 1e-6*Eigen::Vector3d::Ones();
    contact.computedDeltaForce = Eigen::Vector3d::Zero();
    contact.stateForce = Eigen::Vector3d::Zero();

    //Add new contact point
    _contactsPoint.push_back(contact);
    _mappingContacts.insert({frameName, -_contactsPoint.size()});
    _sizePointOff++;
}

void SEIKOFloatingBase::resetTargetsFromModel()
{
    setJointTargetPos(_model->getJointPosVect());
    for (size_t i=0;i<_contactsPlane.size();i++) {
        _contactsPlane[i].targetPos = 
            _model->position(_contactsPlane[i].frameId, 0);
        _contactsPlane[i].targetMat = 
            _model->orientation(_contactsPlane[i].frameId, 0);
    }
    for (size_t i=0;i<_contactsPoint.size();i++) {
        _contactsPoint[i].targetPos = 
            _model->position(_contactsPoint[i].frameId, 0);
        _contactsPoint[i].targetMat = 
            _model->orientation(_contactsPoint[i].frameId, 0);
    }
}

bool SEIKOFloatingBase::toggleContact(
    const std::string& frameName,
    bool isEnabled)
{
    if (_mappingContacts.count(frameName) == 0) {
        throw std::logic_error(
            "leph::SEIKOFloatingBase::toggleContact: "
            "Unknown contact frame name: "
            + frameName);
    }
    int index = _mappingContacts.at(frameName);

    bool isChanged = false;
    if (index > 0) {
        //Plane contact
        index = index-1;
        if (isEnabled && !_contactsPlane.at(index).isEnabled) {
            _contactsPlane.at(index).isEnabled = true;
            _sizePlaneOn++;
            _sizePlaneOff--;
            isChanged = true;
            //Reset position and orientation on plane contact enabling
            _contactsPlane.at(index).targetPos = 
                _model->position(_contactsPlane.at(index).frameId, 0);
            _contactsPlane.at(index).targetMat = 
                _model->orientation(_contactsPlane.at(index).frameId, 0);
        }
        if (!isEnabled && _contactsPlane.at(index).isEnabled) {
            _contactsPlane.at(index).isEnabled = false;
            _sizePlaneOn--;
            _sizePlaneOff++;
            isChanged = true;
        }
    } else if (index < 0) {
        //Point contact
        index = -index-1;
        if (isEnabled && !_contactsPoint.at(index).isEnabled) {
            _contactsPoint.at(index).isEnabled = true;
            _sizePointOn++;
            _sizePointOff--;
            isChanged = true;
            //Reset only position on point contact enabling
            _contactsPoint.at(index).targetPos = 
                _model->position(_contactsPoint.at(index).frameId, 0);
        }
        if (!isEnabled && _contactsPoint.at(index).isEnabled) {
            _contactsPoint.at(index).isEnabled = false;
            _sizePointOn--;
            _sizePointOff++;
            isChanged = true;
        }
    }

    return isChanged;
}

const SEIKOFloatingBase::ContactBase_t& SEIKOFloatingBase::stateContactBase(
    const std::string& frameName) const
{
    if (_mappingContacts.count(frameName) == 0) {
        throw std::logic_error(
            "leph::SEIKOFloatingBase::stateContactBase: "
            "Unknown contact frame name: "
            + frameName);
    }

    int index = _mappingContacts.at(frameName);
    if (index > 0) {
        return _contactsPlane.at(index-1);
    } else {
        return _contactsPoint.at(-index-1);
    }
}
const SEIKOFloatingBase::ContactPlane_t& SEIKOFloatingBase::stateContactPlane(
    const std::string& frameName) const
{
    if (_mappingContacts.count(frameName) == 0) {
        throw std::logic_error(
            "leph::SEIKOFloatingBase::stateContactPlane: "
            "Unknown contact frame name: "
            + frameName);
    }

    int index = _mappingContacts.at(frameName);
    if (index > 0) {
        return _contactsPlane.at(index-1);
    } else {
        throw std::logic_error(
            "leph::SEIKOFloatingBase::stateContactPlane: "
            "Contact is not plane: "
            + frameName);
    }
}
const SEIKOFloatingBase::ContactPoint_t& SEIKOFloatingBase::stateContactPoint(
    const std::string& frameName) const
{
    if (_mappingContacts.count(frameName) == 0) {
        throw std::logic_error(
            "leph::SEIKOFloatingBase::stateContactPoint: "
            "Unknown contact frame name: "
            + frameName);
    }

    int index = _mappingContacts.at(frameName);
    if (index > 0) {
        throw std::logic_error(
            "leph::SEIKOFloatingBase::stateContactPoint: "
            "Contact is not point: "
            + frameName);
    } else {
        return _contactsPoint.at(-index-1);
    }
}
        
const std::map<std::string, int>& SEIKOFloatingBase::getMappingContacts() const
{
    return _mappingContacts;
}

void SEIKOFloatingBase::setJointLimitPos(
    const Eigen::VectorXd& limitLower, 
    const Eigen::VectorXd& limitUpper)
{
    if (
        limitLower.size() != _sizeJoint || 
        limitUpper.size() != _sizeJoint || 
        (limitUpper-limitLower).minCoeff() < 0.0
    ) {
        throw std::logic_error(
            "leph::SEIKOFloatingBase::setJointLimitPos: "
            "Invalid bounds");
    }
    _jointLimitPosLower = limitLower;
    _jointLimitPosUpper = limitUpper;
}
void SEIKOFloatingBase::setJointLimitVelAbs(const Eigen::VectorXd& maxVel)
{
    if (
        maxVel.size() != _sizeJoint || 
        maxVel.minCoeff() < 0.0
    ) {
        throw std::logic_error(
            "leph::SEIKOFloatingBase::setJointLimitVelAbs: "
            "Invalid bounds");
    }
    _jointLimitVelAbs = maxVel;
}
void SEIKOFloatingBase::setJointLimitTauAbs(const Eigen::VectorXd& maxTau)
{
    if (
        maxTau.size() != _sizeJoint || 
        maxTau.minCoeff() < 0.0
    ) {
        throw std::logic_error(
            "leph::SEIKOFloatingBase::setJointLimitTauAbs: "
            "Invalid bounds");
    }
    _jointLimitTauAbs = maxTau;
}
void SEIKOFloatingBase::setJointTargetPos(const Eigen::VectorXd& targetPos)
{
    if (targetPos.size() != _sizeJoint) {
        throw std::logic_error(
            "leph::SEIKOFloatingBase::setJointTargetPos: "
            "Invalid size");
    }
    _jointTargetPos = targetPos;
}
void SEIKOFloatingBase::setJointWeightPos(const Eigen::VectorXd& weightPos)
{
    if (
        weightPos.size() != _sizeJoint || 
        weightPos.minCoeff() < 0.0
    ) {
        throw std::logic_error(
            "leph::SEIKOFloatingBase::setJointWeightPos: "
            "Invalid weights");
    }
    _jointWeightPos = weightPos;
}
void SEIKOFloatingBase::setJointWeightVel(const Eigen::VectorXd& weightVel)
{
    if (
        weightVel.size() != _sizeJoint || 
        weightVel.minCoeff() < 0.0
    ) {
        throw std::logic_error(
            "leph::SEIKOFloatingBase::setJointWeightVel: "
            "Invalid weights");
    }
    _jointWeightVel = weightVel;
}
void SEIKOFloatingBase::setJointWeightTau(const Eigen::VectorXd& weightTau)
{
    if (
        weightTau.size() != _sizeJoint || 
        weightTau.minCoeff() < 0.0
    ) {
        throw std::logic_error(
            "leph::SEIKOFloatingBase::setJointWeightTau: "
            "Invalid weights");
    }
    _jointWeightTau = weightTau;
}
void SEIKOFloatingBase::setJointClampPos(double clampPos)
{
    if (clampPos < 0.0) {
        throw std::logic_error(
            "leph::SEIKOFloatingBase::setJointClampPos: "
            "Negative parameter");
    }
    _jointClampPos = clampPos;
}
        
Eigen::Vector3d& SEIKOFloatingBase::refTargetPos(
    const std::string& frameName)
{
    return getContactBase(frameName).targetPos;
}
Eigen::Matrix3d& SEIKOFloatingBase::refTargetMat(
    const std::string& frameName)
{
    return getContactBase(frameName).targetMat;
}
void SEIKOFloatingBase::setWeightPose(
    const std::string& frameName,
    double weightPos, double weightMat)
{
    if (weightPos < 0.0 || weightMat < 0.0) {
        throw std::logic_error(
            "leph::SEIKOFloatingBase::setWeightPose: "
            "Invalid weights");
    }
    getContactBase(frameName).weightPos = weightPos;
    getContactBase(frameName).weightMat = weightMat;
}
void SEIKOFloatingBase::setClampPose(
    const std::string& frameName,
    double clampPos, double clampMat)
{
    if (clampPos < 0.0 || clampMat < 0.0) {
        throw std::logic_error(
            "leph::SEIKOFloatingBase::setClampPose: "
            "Invalid clamps");
    }
    getContactBase(frameName).clampPos = clampPos;
    getContactBase(frameName).clampMat = clampMat;
}
void SEIKOFloatingBase::setContactLimitFriction(
    const std::string& frameName,
    double frictionCoef)
{
    if (frictionCoef < 0.0) {
        throw std::logic_error(
            "leph::SEIKOFloatingBase::setContactLimitFriction: "
            "Invalid coef");
    }
    getContactBase(frameName).frictionCoef = frictionCoef;
}
void SEIKOFloatingBase::setNormalForceLimits(
    const std::string& frameName,
    double normalForceMin, double normalForceMax)
{
    if (
        normalForceMax < 0.0 ||
        normalForceMin > normalForceMax
    ) {
        throw std::logic_error(
            "leph::SEIKOFloatingBase::setNormalForceLimits: "
            "Invalid limits");
    }
    ContactBase_t& contact = getContactBase(frameName);
    contact.normalForceMin = normalForceMin;
    contact.normalForceMax = normalForceMax;
}
void SEIKOFloatingBase::setLimitVelNormalForce(
    const std::string& frameName,
    double maxVel)
{
    if (maxVel < 0.0) {
        throw std::logic_error(
            "leph::SEIKOFloatingBase::setNormalForceLimits: "
            "Invalid limits");
    }
    getContactBase(frameName).limitVelNormalForce = maxVel;
}

void SEIKOFloatingBase::setContactPlaneLimitCOP(
    const std::string& frameName,
    double limitX, double limitY)
{
    if (limitX < 0.0 || limitY < 0.0) {
        throw std::logic_error(
            "leph::SEIKOFloatingBase::setContactPlaneLimitCOP: "
            "Invalid limits");
    }
    Eigen::Vector2d limit;
    limit.x() = limitX;
    limit.y() = limitY;
    getContactPlane(frameName).limitCOP = limit;
}
void SEIKOFloatingBase::setTargetWrench(
    const std::string& frameName,
    const Eigen::Vector6d& targetWrench)
{
    ContactPlane_t& contact = getContactPlane(frameName);
    contact.targetWrench = targetWrench;
}
void SEIKOFloatingBase::setWeightWrench(
    const std::string& frameName,
    const Eigen::Vector6d& weightWrench)
{
    if (weightWrench.minCoeff() < 0.0) {
        throw std::logic_error(
            "leph::SEIKOFloatingBase::setWeightWrench: "
            "Invalid weights");
    }
    ContactPlane_t& contact = getContactPlane(frameName);
    contact.weightWrench = weightWrench;
}

void SEIKOFloatingBase::setContactMat(
    const std::string& frameName,
    const Eigen::Matrix3d& contactMat)
{
    getContactPoint(frameName).contactMat = contactMat;
}
void SEIKOFloatingBase::setTargetForce(
    const std::string& frameName,
    const Eigen::Vector3d& targetForce)
{
    ContactPoint_t& contact = getContactPoint(frameName);
    contact.targetForce = targetForce;
}
void SEIKOFloatingBase::setWeightForce(
    const std::string& frameName,
    const Eigen::Vector3d& weightForce)
{
    if (weightForce.minCoeff() < 0.0) {
        throw std::logic_error(
            "leph::SEIKOFloatingBase::setWeightForce: "
            "Invalid weights");
    }
    ContactPoint_t& contact = getContactPoint(frameName);
    contact.weightForce = weightForce;
}

const Eigen::Vector6d& SEIKOFloatingBase::refExternalBaseWrench() const
{
    return _externalBaseWrench;
}
Eigen::Vector6d& SEIKOFloatingBase::refExternalBaseWrench()
{
    return _externalBaseWrench;
}

bool SEIKOFloatingBase::runInverseDynamics()
{
    //Define problem sizes for 
    //static inverse dynamics
    size_t sizeSol = 
        //Plane contact wrenches
        6*_sizePlaneOn +
        //Point contact forces
        3*_sizePointOn;
    size_t sizeCost = 
        //Targets for joint torques and 
        //contact wrenches and forces
        _sizeJoint +
        6*_sizePlaneOn +
        3*_sizePointOn;
    size_t sizeEq =
        //Floating base upper part of 
        //static equation of motion
        6;
    size_t sizeIneq =
        //Joint torques bounds
        2*_sizeJoint + 
        //Contact plane constraints
        18*_sizePlaneOn +
        //Contact point constraints
        6*_sizePointOn;
    
    //Matrices initialization
    Eigen::DiagonalMatrix<double, Eigen::Dynamic> weights(sizeCost);
    weights.setZero();
    Eigen::MatrixXd costMat = Eigen::MatrixXd::Zero(sizeCost, sizeSol);
    Eigen::VectorXd costVec = Eigen::VectorXd::Zero(sizeCost);
    Eigen::MatrixXd problemCostMat = Eigen::MatrixXd::Zero(sizeSol, sizeSol);
    Eigen::VectorXd problemCostVec = Eigen::VectorXd::Zero(sizeSol);
    Eigen::MatrixXd problemEqMat = Eigen::MatrixXd::Zero(sizeEq, sizeSol);
    Eigen::VectorXd problemEqVec = Eigen::VectorXd::Zero(sizeEq);
    Eigen::MatrixXd problemIneqMat = Eigen::MatrixXd::Zero(sizeIneq, sizeSol);
    Eigen::VectorXd problemIneqVec = Eigen::VectorXd::Zero(sizeIneq);
    Eigen::VectorXd problemSolution = Eigen::VectorXd::Zero(sizeSol);
    
    //Compute joint gravity vector
    _gravityVector = _model->computeGravityVector();

    //Compute plane and point contact Jacobian matrices
    for (size_t i=0;i<_contactsPlane.size();i++) {
        if (_contactsPlane[i].isEnabled) {
            _contactsPlane[i].jacobianBody = _model->pointJacobian(
                _contactsPlane[i].frameId, _contactsPlane[i].frameId);
        }
    }
    for (size_t i=0;i<_contactsPoint.size();i++) {
        if (_contactsPoint[i].isEnabled) {
            _contactsPoint[i].jacobianWorld = _model->pointJacobian(
                _contactsPoint[i].frameId, 0);
        }
    }
    
    //Build the linear relationship from reduced solution 
    //(contact wrenches/forces) to computed joint torques
    //using the static equation of motion.
    //solToTauMat*deltaSol + solToTauVec = S*(dtau+tau)
    Eigen::MatrixXd solToTauMat = Eigen::MatrixXd::Zero(_sizeDOF, sizeSol);
    Eigen::VectorXd solToTauVec = Eigen::VectorXd::Zero(_sizeDOF);
    size_t offsetTmpCol = 0;
    //Contact plane wrenches
    for (size_t i=0;i<_contactsPlane.size();i++) {
        if (_contactsPlane[i].isEnabled) {
            solToTauMat.block(0, offsetTmpCol, _sizeDOF, 6) = 
                -_contactsPlane[i].jacobianBody.transpose();
            offsetTmpCol += 6;
        }
    }
    //Contact point forces
    for (size_t i=0;i<_contactsPoint.size();i++) {
        if (_contactsPoint[i].isEnabled) {
            solToTauMat.block(0, offsetTmpCol, _sizeDOF, 3) =
                -(_contactsPoint[i].jacobianWorld
                    .block(3, 0, 3, _sizeDOF).transpose()
                * _contactsPoint[i].contactMat);
            offsetTmpCol += 3;
        }
    }
    //Gravity vector
    solToTauVec = _gravityVector;
    //Add external wrench on floating base
    solToTauVec.segment(0, 3) -= _externalBaseWrench.segment(3, 3);
    solToTauVec.segment(3, 3) -= _externalBaseWrench.segment(0, 3);
    
    //Static inverse dynamics matrices setup
    //Cost matrix
    size_t offsetCostRow = 0;
    //Joint torques using linear relation from contact 
    //wrenches/forces with lower part of the equation of motion
    costMat.block(offsetCostRow, 0, _sizeJoint, sizeSol) = 
        solToTauMat.block(6, 0, _sizeJoint, sizeSol);
    costVec.segment(offsetCostRow, _sizeJoint) = 
        _jointTargetTau - solToTauVec.segment(6, _sizeJoint);
    weights.diagonal().segment(offsetCostRow, _sizeJoint) = 
        _jointWeightTau;
    offsetCostRow += _sizeJoint;
    //Contact plane wrenches
    size_t offsetCostCol = 0;
    for (size_t i=0;i<_contactsPlane.size();i++) {
        if (_contactsPlane[i].isEnabled) {
            costMat.block(offsetCostRow, offsetCostCol, 6, 6).setIdentity();
            costVec.segment(offsetCostRow, 6) = 
                _contactsPlane[i].targetWrench;
            weights.diagonal().segment(offsetCostRow, 6) = 
                _contactsPlane[i].weightWrench;
            offsetCostRow += 6;
            offsetCostCol += 6;
        }
    }
    //Contact point forces
    for (size_t i=0;i<_contactsPoint.size();i++) {
        if (_contactsPoint[i].isEnabled) {
            costMat.block(offsetCostRow, offsetCostCol, 3, 3).setIdentity();
            costVec.segment(offsetCostRow, 3) = 
                _contactsPoint[i].targetForce;
            weights.diagonal().segment(offsetCostRow, 3) = 
                _contactsPoint[i].weightForce;
            offsetCostRow += 3;
            offsetCostCol += 3;
        }
    }

    //Build equality constraints matrix
    //Upper floating base rows of static equation of motion
    problemEqMat = solToTauMat.block(0, 0, 6, sizeSol);
    problemEqVec = solToTauVec.segment(0, 6);

    //Build inequality constraints matrix
    Eigen::MatrixXd tmpIDIneqMat;
    Eigen::VectorXd tmpIDIneqVec;
    //Build constraints matrix and vector over 
    //joint torques and contact wrenches/forces
    buildStaticIDInequalities(tmpIDIneqMat, tmpIDIneqVec);
    //Build mapping from solution (only contact wrenches/forces)
    //to extented vector with joint torques and contacts
    Eigen::MatrixXd tmpMappingMat = 
        Eigen::MatrixXd::Zero(_sizeJoint + sizeSol, sizeSol);
    Eigen::VectorXd tmpMappingVec = 
        Eigen::VectorXd::Zero(_sizeJoint + sizeSol);
    tmpMappingMat.block(0, 0, _sizeJoint, sizeSol) = 
        solToTauMat.block(6, 0, _sizeJoint, sizeSol);
    tmpMappingMat.block(_sizeJoint, 0, sizeSol, sizeSol).setIdentity();
    tmpMappingVec.segment(0, _sizeJoint) = solToTauVec.segment(6, _sizeJoint);
    //Build problem inequalities
    problemIneqMat = tmpIDIneqMat*tmpMappingMat;
    problemIneqVec = tmpIDIneqMat*tmpMappingVec + tmpIDIneqVec;

    //Build the weighted distance of linear target costs
    problemCostMat = costMat.transpose()*weights*costMat;
    problemCostVec = -costMat.transpose()*weights*costVec;

    //Solve the QP problem
    double cost = Eigen::solve_quadprog(
        problemCostMat,
        problemCostVec,
        problemEqMat.transpose(),
        problemEqVec,
        problemIneqMat.transpose(),
        problemIneqVec,
        problemSolution);

    //Check solver success
    if (std::isnan(cost) || std::isinf(cost)) {
        //Return failure
        return false;
    } else {
        //Copy computed solution to internal state
        size_t offsetSolRow = 0;
        //Contact wrenches
        for (size_t i=0;i<_contactsPlane.size();i++) {
            if (_contactsPlane[i].isEnabled) {
                _contactsPlane[i].stateWrench = 
                    problemSolution.segment(offsetSolRow, 6);
                offsetSolRow += 6;
            } else {
                _contactsPlane[i].stateWrench.setZero();
            }
        }
        //Contact forces
        for (size_t i=0;i<_contactsPoint.size();i++) {
            if (_contactsPoint[i].isEnabled) {
                _contactsPoint[i].stateForce = 
                    problemSolution.segment(offsetSolRow, 3);
                offsetSolRow += 3;
            } else {
                _contactsPoint[i].stateForce.setZero();
            }
        }
        //Compute the torque from botton joint 
        //part of static equation of motion
        _jointStateTau = 
            solToTauMat.block(6, 0, _sizeJoint, sizeSol)*problemSolution 
            + solToTauVec.segment(6, _sizeJoint);
        //Return success
        return true;
    }
}

bool SEIKOFloatingBase::runSEIKO(double dt)
{
    //Define problem sizes
    size_t sizeSolID = 
        //Delta plane contact wrenches
        6*_sizePlaneOn + 
        //Delta point contact forces
        3*_sizePointOn;
    size_t sizeSol = 
        //Delta position for each DOF
        _sizeDOF +
        //Delta static wrenches and forces
        sizeSolID;
    size_t sizeCost = 
        //Velocity regularization
        _sizeDOF +
        //Joint position target
        _sizeJoint +
        //Disabled pose and point contact target
        6*(_sizePlaneOff + _sizePointOff) +
        //Enabled point contact orientation target
        3*_sizePointOn +
        //Joint torque minimization
        _sizeJoint +
        //Contact plane wrenches minimization
        6*_sizePlaneOn +
        //Contact point forces minimization
        3*_sizePointOn;
    size_t sizeEq =
        //Enabled pose contact constraint
        6*_sizePlaneOn +
        //Enabled point contact constraint
        3*_sizePointOn +
        //Upper floating base part of 
        //the static equation of motion
        6;
    size_t sizeIDIneq = 
        //Joint torques bounds
        2*_sizeJoint + 
        //Contact plane constraints
        18*_sizePlaneOn +
        //Contact point constraints
        6*_sizePointOn;
    size_t sizeIneq =
        //Joint position bounds
        2*_sizeJoint +
        //Joint velocity bounds
        2*_sizeJoint +
        //Normal force change (velocity) bounds
        2*(_sizePlaneOn +_sizePointOn) +
        //Static inverse dynamics feasibility
        sizeIDIneq;
    
    //Matrices initialization
    Eigen::DiagonalMatrix<double, Eigen::Dynamic> weights = 
        Eigen::DiagonalMatrix<double, Eigen::Dynamic>(sizeCost);
    weights.setZero();
    Eigen::MatrixXd costMat = Eigen::MatrixXd::Zero(sizeCost, sizeSol);
    Eigen::VectorXd costVec = Eigen::VectorXd::Zero(sizeCost);
    Eigen::MatrixXd problemCostMat = Eigen::MatrixXd::Zero(sizeSol, sizeSol);
    Eigen::VectorXd problemCostVec = Eigen::VectorXd::Zero(sizeSol);
    Eigen::MatrixXd problemEqMat = Eigen::MatrixXd::Zero(sizeEq, sizeSol);
    Eigen::VectorXd problemEqVec = Eigen::VectorXd::Zero(sizeEq);
    Eigen::MatrixXd problemIneqMat = Eigen::MatrixXd::Zero(sizeIneq, sizeSol);
    Eigen::VectorXd problemIneqVec = Eigen::VectorXd::Zero(sizeIneq);
    Eigen::VectorXd problemSolution = Eigen::VectorXd::Zero(sizeSol);

    //Update pinocchio kinematics
    _pinocchio.updateKinematics(_model->getDOFPosVect());

    //Compute plane and point contact Jacobian matrices
    for (size_t i=0;i<_contactsPlane.size();i++) {
        _contactsPlane[i].jacobianWorld = _model->pointJacobian(
            _contactsPlane[i].frameId, 0);
        if (_contactsPlane[i].isEnabled) {
            _contactsPlane[i].jacobianBody = _model->pointJacobian(
                _contactsPlane[i].frameId, _contactsPlane[i].frameId);
        }
    }
    for (size_t i=0;i<_contactsPoint.size();i++) {
        _contactsPoint[i].jacobianWorld = _model->pointJacobian(
            _contactsPoint[i].frameId, 0);
    }
    
    //Compute joint gravity vector
    _gravityVector = _pinocchio.gravityVector();

    //Build current ID state vector
    Eigen::VectorXd currentIDVect = 
        Eigen::VectorXd::Zero(_sizeJoint + sizeSolID);
    size_t offsetSolID = 0;
    currentIDVect.segment(offsetSolID, _sizeJoint) = _jointStateTau;
    offsetSolID += _sizeJoint;
    for (size_t i=0;i<_contactsPlane.size();i++) {
        if (_contactsPlane[i].isEnabled) {
            currentIDVect.segment(offsetSolID, 6) = 
                _contactsPlane[i].stateWrench;
            offsetSolID += 6;
        }
    }
    for (size_t i=0;i<_contactsPoint.size();i++) {
        if (_contactsPoint[i].isEnabled) {
            currentIDVect.segment(offsetSolID, 3) = 
                _contactsPoint[i].stateForce;
            offsetSolID += 3;
        }
    }

    //Build the linear relationship from reduced solution 
    //(delta position and delta contact wrenches/forces)
    //to computed joint torques (after integration)
    //using the differentiated static equation of motion.
    //solToTauMat*deltaSol + solToTauVec = S*(dtau+tau)
    Eigen::MatrixXd solToTauMat = Eigen::MatrixXd::Zero(_sizeDOF, sizeSol);
    Eigen::VectorXd solToTauVec = Eigen::VectorXd::Zero(_sizeDOF);
    size_t offsetTmpCol = _sizeDOF;
    for (size_t i=0;i<_contactsPlane.size();i++) {
        if (_contactsPlane[i].isEnabled) {
            solToTauMat.block(0, 0, _sizeDOF, _sizeDOF) -= 
                _pinocchio.diffHessianWrenchInLocalProduct(
                    _contactsPlane[i].frameName, 
                    _contactsPlane[i].stateWrench);
            solToTauMat.block(0, offsetTmpCol, _sizeDOF, 6) = 
                -_contactsPlane[i].jacobianBody.transpose();
            solToTauVec -= 
                _contactsPlane[i].jacobianBody.transpose()
                * _contactsPlane[i].stateWrench;
            offsetTmpCol += 6;
        }
    }
    for (size_t i=0;i<_contactsPoint.size();i++) {
        if (_contactsPoint[i].isEnabled) {
            solToTauMat.block(0, 0, _sizeDOF, _sizeDOF) -= 
                _pinocchio.diffHessianForceInWorldProduct(
                    _contactsPoint[i].frameName, 
                    (_contactsPoint[i].contactMat*_contactsPoint[i].stateForce));
            solToTauMat.block(0, offsetTmpCol, _sizeDOF, 3) = 
                -_contactsPoint[i].jacobianWorld
                    .block(3, 0, 3, _sizeDOF).transpose()
                * _contactsPoint[i].contactMat;
            solToTauVec -=
                _contactsPoint[i].jacobianWorld.block(3, 0, 3, _sizeDOF).transpose()
                * _contactsPoint[i].contactMat
                * _contactsPoint[i].stateForce;
            offsetTmpCol += 3;
        }
    }
    solToTauMat.block(0, 0, _sizeDOF, _sizeDOF) += 
        _pinocchio.diffGravityVector();
    solToTauVec += _gravityVector;
    //Add external wrench on floating base
    solToTauVec.segment(0, 3) -= _externalBaseWrench.segment(3, 3);
    solToTauVec.segment(3, 3) -= _externalBaseWrench.segment(0, 3);

    //Build cost
    size_t offsetCostRow = 0;
    //Velocity regularization
    costMat.block(offsetCostRow, 0, _sizeDOF, _sizeDOF) = 
        Eigen::MatrixXd::Identity(_sizeDOF, _sizeDOF);
    costVec.segment(offsetCostRow, _sizeDOF) = 
        Eigen::VectorXd::Zero(_sizeDOF);
    //Nearly no regularization for floating base DOF
    weights.diagonal().segment(offsetCostRow, 6) = 
        1e-8*Eigen::VectorXd::Ones(6);
    //Joint case
    weights.diagonal().segment(offsetCostRow+6, _sizeJoint) = 
        dt*_jointWeightVel;
    offsetCostRow += _sizeDOF;
    //Joint position target
    costMat.block(offsetCostRow, 6, _sizeJoint, _sizeJoint) = 
        Eigen::MatrixXd::Identity(_sizeJoint, _sizeJoint);
    costVec.segment(offsetCostRow, _sizeJoint) = ClampVectorAllComponent(
        _jointTargetPos - _model->getJointPosVect(), _jointClampPos);
    weights.diagonal().segment(offsetCostRow, _sizeJoint) = 
        _jointWeightPos;
    offsetCostRow += _sizeJoint;
    //Disable pose contact target
    for (size_t i=0;i<_contactsPlane.size();i++) {
        if (!_contactsPlane[i].isEnabled) {
            writePoseTarget(
                _contactsPlane[i],
                costMat, costVec, weights, 
                offsetCostRow,
                true);
            offsetCostRow += 6;
        }
    }
    //Disable point contact target
    for (size_t i=0;i<_contactsPoint.size();i++) {
        if (!_contactsPoint[i].isEnabled) {
            writePoseTarget(
                _contactsPoint[i],
                costMat, costVec, weights, 
                offsetCostRow,
                true);
            offsetCostRow += 6;
        }
    }
    //Enabled point contact orientation target
    for (size_t i=0;i<_contactsPoint.size();i++) {
        if (_contactsPoint[i].isEnabled) {
            writePoseTarget(
                _contactsPoint[i],
                costMat, costVec, weights, 
                offsetCostRow,
                false);
            offsetCostRow += 3;
        }
    }
    //Delta torques wrenches and forces
    //Joint torques using linear relation from contact 
    //wrenches/forces with lower part of the equation of motion
    costMat.block(offsetCostRow, 0, _sizeJoint, sizeSol) = 
        solToTauMat.block(6, 0, _sizeJoint, sizeSol);
    costVec.segment(offsetCostRow, _sizeJoint) = 
        _jointTargetTau - solToTauVec.segment(6, _sizeJoint);
    weights.diagonal().segment(offsetCostRow, _sizeJoint) = 
        _jointWeightTau;
    offsetCostRow += _sizeJoint;
    //Contact plane wrenches
    size_t offsetCostCol = _sizeDOF;
    for (size_t i=0;i<_contactsPlane.size();i++) {
        if (_contactsPlane[i].isEnabled) {
            costMat.block(offsetCostRow, offsetCostCol, 6, 6) = 
                Eigen::MatrixXd::Identity(6, 6);
            costVec.segment(offsetCostRow, 6) = 
                _contactsPlane[i].targetWrench 
                - _contactsPlane[i].stateWrench;
            weights.diagonal().segment(offsetCostRow, 6) = 
                _contactsPlane[i].weightWrench;
            offsetCostRow += 6;
            offsetCostCol += 6;
        }
    }
    //Contact point forces
    for (size_t i=0;i<_contactsPoint.size();i++) {
        if (_contactsPoint[i].isEnabled) {
            costMat.block(offsetCostRow, offsetCostCol, 3, 3) = 
                Eigen::MatrixXd::Identity(3, 3);
            costVec.segment(offsetCostRow, 3) = 
                _contactsPoint[i].targetForce
                - _contactsPoint[i].stateForce;
            weights.diagonal().segment(offsetCostRow, 3) = 
                _contactsPoint[i].weightForce;
            offsetCostRow += 3;
            offsetCostCol += 3;
        }
    }

    //Build equality constraints
    size_t offsetEq = 0;
    //Enabled pose contacts
    for (size_t i=0;i<_contactsPlane.size();i++) {
        if (_contactsPlane[i].isEnabled) {
            const size_t& frameId = _contactsPlane[i].frameId;
            Eigen::Vector3d deltaMat = MatrixToAxis(
                    _contactsPlane[i].targetMat
                    * _model->orientation(frameId, 0).transpose());
            Eigen::Vector3d deltaPos = 
                _contactsPlane[i].targetPos
                - _model->position(frameId, 0);
            problemEqMat.block(offsetEq, 0, 6, _sizeDOF) = 
                _contactsPlane[i].jacobianWorld;
            problemEqVec.segment(offsetEq+0, 3) = -deltaMat;
            problemEqVec.segment(offsetEq+3, 3) = -deltaPos;
            offsetEq += 6;
        }
    }
    //Enabled point contacts
    for (size_t i=0;i<_contactsPoint.size();i++) {
        if (_contactsPoint[i].isEnabled) {
            const size_t& frameId = _contactsPoint[i].frameId;
            Eigen::Vector3d deltaPos = 
                _contactsPoint[i].targetPos
                - _model->position(frameId, 0);
            problemEqMat.block(offsetEq, 0, 3, _sizeDOF) = 
                _contactsPoint[i].jacobianWorld.block(3, 0, 3, _sizeDOF);
            problemEqVec.segment(offsetEq, 3) = -deltaPos;
            offsetEq += 3;
        }
    }
    //Static equation of motion
    problemEqMat.block(offsetEq, 0, 6, sizeSol) = 
        solToTauMat.block(0, 0, 6, sizeSol);
    _biasConstraintEquilibrium = solToTauVec.segment(0, 6);
    problemEqVec.segment(offsetEq, 6) = 
        _biasConstraintEquilibrium;
    
    //Build inequality constraints 
    size_t offsetIneq = 0;
    //Joint kinematics bounds
    for (size_t i=0;i<_sizeJoint;i++) {
        //10 because increase safety...
        problemIneqMat(offsetIneq+0, 6+i) = 10.0;
        problemIneqMat(offsetIneq+1, 6+i) = -10.0;
        problemIneqVec(offsetIneq+0) = 
            _model->getDOFPos(i+6) - _jointLimitPosLower(i);
        problemIneqVec(offsetIneq+1) = 
            -_model->getDOFPos(i+6) + _jointLimitPosUpper(i);
        offsetIneq += 2;
    }
    //Joint velocity bounds
    for (size_t i=0;i<_sizeJoint;i++) {
        problemIneqMat(offsetIneq+0, 6+i) = 1.0;
        problemIneqMat(offsetIneq+1, 6+i) = -1.0;
        problemIneqVec(offsetIneq+0) = 
            dt*_jointLimitVelAbs(i);
        problemIneqVec(offsetIneq+1) = 
            dt*_jointLimitVelAbs(i);
        offsetIneq += 2;
    }
    //Normal force velocity bounds
    size_t offsetIneqCol = _sizeDOF;
    for (size_t i=0;i<_contactsPlane.size();i++) {
        if (_contactsPlane[i].isEnabled) {
            problemIneqMat(offsetIneq+0, offsetIneqCol+5) = 1.0;
            problemIneqMat(offsetIneq+1, offsetIneqCol+5) = -1.0;
            problemIneqVec(offsetIneq+0) = 
                dt*_contactsPlane[i].limitVelNormalForce;
            problemIneqVec(offsetIneq+1) = 
                dt*_contactsPlane[i].limitVelNormalForce;
            offsetIneq += 2;
            offsetIneqCol += 6;
        }
    }
    for (size_t i=0;i<_contactsPoint.size();i++) {
        if (_contactsPoint[i].isEnabled) {
            problemIneqMat(offsetIneq+0, offsetIneqCol+2) = 1.0;
            problemIneqMat(offsetIneq+1, offsetIneqCol+2) = -1.0;
            problemIneqVec(offsetIneq+0) = 
                dt*_contactsPoint[i].limitVelNormalForce;
            problemIneqVec(offsetIneq+1) = 
                dt*_contactsPoint[i].limitVelNormalForce;
            offsetIneq += 2;
            offsetIneqCol += 3;
        }
    }
    //Static torques wrenches forces
    Eigen::MatrixXd tmpIDIneqMat;
    Eigen::VectorXd tmpIDIneqVec;
    //Build constraints matrix and vector over 
    //joint torques and contact wrenches/forces
    buildStaticIDInequalities(tmpIDIneqMat, tmpIDIneqVec);
    //Build mapping from ID solution (only contact wrenches/forces)
    //to extented vector with joint torques and contacts
    Eigen::MatrixXd tmpMappingMat = 
        Eigen::MatrixXd::Zero(_sizeJoint + sizeSolID, sizeSol);
    Eigen::VectorXd tmpMappingVec = 
        Eigen::VectorXd::Zero(_sizeJoint + sizeSolID);
    tmpMappingMat.block(0, 0, _sizeJoint, sizeSol) = 
        solToTauMat.block(6, 0, _sizeJoint, sizeSol);
    tmpMappingMat.block(_sizeJoint, _sizeDOF, sizeSolID, sizeSolID).setIdentity();
    tmpMappingVec.segment(0, _sizeJoint) = solToTauVec.segment(6, _sizeJoint);
    tmpMappingVec.segment(_sizeJoint, sizeSolID) = 
        currentIDVect.segment(_sizeJoint, sizeSolID);
    //Build problem inequalities
    problemIneqMat.block(offsetIneq, 0, sizeIDIneq, sizeSol) = 
        tmpIDIneqMat*tmpMappingMat;
    problemIneqVec.segment(offsetIneq, sizeIDIneq) = 
        tmpIDIneqMat*tmpMappingVec + tmpIDIneqVec;
    offsetIneq += sizeIDIneq;

    //Build the weighted distance of linear target costs
    problemCostMat = costMat.transpose()*weights*costMat;
    problemCostVec = -costMat.transpose()*weights*costVec;
    
    //Solve the QP problem
    double cost = Eigen::solve_quadprog(
        problemCostMat,
        problemCostVec,
        problemEqMat.transpose(),
        problemEqVec,
        problemIneqMat.transpose(),
        problemIneqVec,
        problemSolution);
    
    //Check solver success
    if (std::isnan(cost) || std::isinf(cost)) {
        //Set all changes to zero
        //DOF position
        _jointComputedDeltaDOF.setZero();
        //Joint torques
        _jointComputedDeltaTau.setZero();
        //Contact wrenches
        for (size_t i=0;i<_contactsPlane.size();i++) {
            _contactsPlane[i].computedDeltaWrench.setZero();
        }
        //Contact forces
        for (size_t i=0;i<_contactsPoint.size();i++) {
            _contactsPoint[i].computedDeltaForce.setZero();
        }
        //Return failure
        return false;
    } else {
        //Copy the computed changes
        size_t offsetSolRow = 0;
        //DOF position
        _jointComputedDeltaDOF = 
            problemSolution.segment(offsetSolRow, _sizeDOF);
        offsetSolRow += _sizeDOF;
        //Contact wrenches
        for (size_t i=0;i<_contactsPlane.size();i++) {
            if (_contactsPlane[i].isEnabled) {
                _contactsPlane[i].computedDeltaWrench = 
                    problemSolution.segment(offsetSolRow, 6);
                offsetSolRow += 6;
            } else {
                _contactsPlane[i].computedDeltaWrench.setZero();
            }
        }
        //Contact forces
        for (size_t i=0;i<_contactsPoint.size();i++) {
            if (_contactsPoint[i].isEnabled) {
                _contactsPoint[i].computedDeltaForce = 
                    problemSolution.segment(offsetSolRow, 3);
                offsetSolRow += 3;
            } else {
                _contactsPoint[i].computedDeltaForce.setZero();
            }
        }
        //Joint torques
        _jointComputedDeltaTau = 
            solToTauMat.block(6, 0, _sizeJoint, sizeSol)*problemSolution 
            + solToTauVec.segment(6, _sizeJoint)
            - _jointStateTau;
        //Return success
        return true;
    }
}

void SEIKOFloatingBase::integrateComputedDelta(double dt)
{
    //Integrate position
    _model->setDOFPosVect(IntegrateDOFVect(
        _model->getDOFPosVect(), _jointComputedDeltaDOF));
    //Assign velocity
    _model->setDOFVelVect((1.0/dt)*_jointComputedDeltaDOF);
    //Model update
    _model->updateState();

    //Integrate joint torques
    _jointStateTau += _jointComputedDeltaTau;
    
    //Integrate contact wrenches
    for (size_t i=0;i<_contactsPlane.size();i++) {
        _contactsPlane[i].stateWrench += 
            _contactsPlane[i].computedDeltaWrench;
    }
    //Integrate contact forces
    for (size_t i=0;i<_contactsPoint.size();i++) {
        _contactsPoint[i].stateForce += 
            _contactsPoint[i].computedDeltaForce;
    }
}        

const Eigen::VectorXd& SEIKOFloatingBase::deltaDOFPosition() const
{
    return _jointComputedDeltaDOF;
}
const Eigen::VectorXd& SEIKOFloatingBase::deltaJointTorque() const
{
    return _jointComputedDeltaTau;
}
const Eigen::Vector6d& SEIKOFloatingBase::deltaContactWrench(
    const std::string& frameName) const
{
    return stateContactPlane(frameName).computedDeltaWrench;
}
const Eigen::Vector3d& SEIKOFloatingBase::deltaContactForce(
    const std::string& frameName) const
{
    return stateContactPoint(frameName).computedDeltaForce;
}

const Eigen::VectorXd& SEIKOFloatingBase::stateJointTorque() const
{
    return _jointStateTau;
}
const Eigen::Vector6d& SEIKOFloatingBase::stateContactWrench(
    const std::string& frameName) const
{
    return stateContactPlane(frameName).stateWrench;
}
const Eigen::Vector3d& SEIKOFloatingBase::stateContactForce(
    const std::string& frameName) const
{
    return stateContactPoint(frameName).stateForce;
}

double SEIKOFloatingBase::stateNormalForce(
    const std::string& frameName) const
{
    if (_mappingContacts.count(frameName) == 0) {
        throw std::logic_error(
            "leph::SEIKOFloatingBase::stateNormalForce: "
            "Unknown contact frame name: "
            + frameName);
    }
    int index = _mappingContacts.at(frameName);

    if (index > 0) {
        return _contactsPlane.at(index-1).stateWrench(5);
    } else {
        return _contactsPoint.at(-index-1).stateForce(2);
    }
}

const Eigen::Vector6d& SEIKOFloatingBase::getErrorConstraintEquilibrium() const
{
    return _biasConstraintEquilibrium;
}

std::map<std::string, double> SEIKOFloatingBase::computeConstraintRatios() const
{
    //Container for computed ratio
    std::map<std::string, double> container;

    //Joint position
    Eigen::VectorXd neutral = 
        0.5*_jointLimitPosLower + 0.5*_jointLimitPosUpper;
    Eigen::VectorXd range = 
        _jointLimitPosUpper - neutral;
    Eigen::VectorXd position = 
        _model->getJointPosVect();
    for (size_t i=0;i<_sizeJoint;i++) {
        double ratio = std::fabs(neutral(i)-position(i))/range(i);
        container.insert(std::make_pair(
            "pos " + _model->getNameJoint(i), 
            ratio));
    }

    //Joint torque
    for (size_t i=0;i<_sizeJoint;i++) {
        double ratio = 
            std::fabs(_jointStateTau(i))/_jointLimitTauAbs(i);
        container.insert(std::make_pair(
            "tau " + _model->getNameJoint(i), 
            ratio));
    }

    //Total applied normal force
    double sumNormalForce = 0.0;
    for (size_t i=0;i<_contactsPlane.size();i++) {
        if (_contactsPlane[i].isEnabled) {
            sumNormalForce += _contactsPlane[i].stateWrench(5);
        }
    }
    for (size_t i=0;i<_contactsPoint.size();i++) {
        if (_contactsPoint[i].isEnabled) {
            sumNormalForce += _contactsPoint[i].stateForce(2);
        }
    }
    
    //Contact wrench
    for (size_t i=0;i<_contactsPlane.size();i++) {
        if (_contactsPlane[i].isEnabled) {
            const Eigen::Vector6d& wrench = _contactsPlane[i].stateWrench;
            double ratioCOPX = 0.0;
            double ratioCOPY = 0.0;
            double ratioFrictionX = 0.0;
            double ratioFrictionY = 0.0;
            if (std::fabs(wrench(5)) > 1e-3) {
                ratioCOPX = 
                    std::fabs(wrench(1)) /
                    std::fabs(wrench(5)*_contactsPlane[i].limitCOP.x());
                ratioCOPY = 
                    std::fabs(wrench(0)) /
                    std::fabs(wrench(5)*_contactsPlane[i].limitCOP.y());
                ratioFrictionX = 
                    std::fabs(wrench(3)) /
                    std::fabs(wrench(5)*_contactsPlane[i].frictionCoef);
                ratioFrictionY = 
                    std::fabs(wrench(4)) /
                    std::fabs(wrench(5)*_contactsPlane[i].frictionCoef);
            }
            container.insert(std::make_pair(
                "copX " + _contactsPlane[i].frameName,
                ratioCOPX));
            container.insert(std::make_pair(
                "copY " + _contactsPlane[i].frameName,
                ratioCOPY));
            container.insert(std::make_pair(
                "frictionX " + _contactsPlane[i].frameName,
                ratioFrictionX));
            container.insert(std::make_pair(
                "frictionY " + _contactsPlane[i].frameName,
                ratioFrictionY));
            container.insert(std::make_pair(
                "forceNormal " + _contactsPlane[i].frameName,
                wrench(5)/sumNormalForce));
        } 
    }
    
    //Contact force
    for (size_t i=0;i<_contactsPoint.size();i++) {
        if (_contactsPoint[i].isEnabled) {
            const Eigen::Vector3d& force = _contactsPoint[i].stateForce;
            sumNormalForce += force(2);
            double ratioFrictionX = 0.0;
            double ratioFrictionY = 0.0;
            if (std::fabs(force(2)) > 1e-3) {
                ratioFrictionX = 
                    std::fabs(force(0)) /
                    std::fabs(force(2)*_contactsPoint[i].frictionCoef);
                ratioFrictionY = 
                    std::fabs(force(1)) /
                    std::fabs(force(2)*_contactsPoint[i].frictionCoef);
            }
            container.insert(std::make_pair(
                "frictionX " + _contactsPoint[i].frameName,
                ratioFrictionX));
            container.insert(std::make_pair(
                "frictionY " + _contactsPoint[i].frameName,
                ratioFrictionY));
            container.insert(std::make_pair(
                "forceNormal " + _contactsPoint[i].frameName,
                force(2)/sumNormalForce));
        } 
    }

    return container;
}

void SEIKOFloatingBase::buildStaticIDInequalities(
    Eigen::MatrixXd& problemIneqMat,
    Eigen::VectorXd& problemIneqVec)
{
    //Define problem sizes
    size_t sizeIDSol = 
        //Joint torques
        _sizeJoint +
        //Plane contact wrenches
        6*_sizePlaneOn +
        //Point contact forces
        3*_sizePointOn;
    size_t sizeIDIneq =
        //Joint torques bounds
        2*_sizeJoint + 
        //Contact plane constraints
        18*_sizePlaneOn +
        //Contact point constraints
        6*_sizePointOn;

    //Matrix and vector reset
    if (
        (size_t)problemIneqMat.rows() == sizeIDIneq && 
        (size_t)problemIneqMat.cols() == sizeIDSol
    ) {
        problemIneqMat.setZero();
    } else {
        problemIneqMat = Eigen::MatrixXd::Zero(sizeIDIneq, sizeIDSol);
    }
    if ((size_t)problemIneqVec.size() == sizeIDIneq) {
        problemIneqVec.setZero();
    } else {
        problemIneqVec = Eigen::VectorXd::Zero(sizeIDIneq);
    }

    //Build inequality constraints
    size_t offsetRow = 0;
    size_t offsetCol = 0;
    //Joint torque limits
    for (size_t i=0;i<_sizeJoint;i++) {
        problemIneqMat(offsetRow+0, offsetCol) = 1.0;
        problemIneqMat(offsetRow+1, offsetCol) = -1.0;
        problemIneqVec(offsetRow+0) = _jointLimitTauAbs(i);
        problemIneqVec(offsetRow+1) = _jointLimitTauAbs(i);
        offsetRow += 2;
        offsetCol += 1;
    }
    //Contact plane constraints
    for (size_t i=0;i<_contactsPlane.size();i++) {
        if (_contactsPlane[i].isEnabled) {
            writeContactConstraints(
                problemIneqMat, problemIneqVec,
                true, i,
                offsetRow, offsetCol);
            offsetRow += 18;
            offsetCol += 6;
        }
    }
    //Contact point constraints
    for (size_t i=0;i<_contactsPoint.size();i++) {
        if (_contactsPoint[i].isEnabled) {
            writeContactConstraints(
                problemIneqMat, problemIneqVec,
                false, i,
                offsetRow, offsetCol);
            offsetRow += 6;
            offsetCol += 3;
        }
    }
}

SEIKOFloatingBase::ContactBase_t& SEIKOFloatingBase::getContactBase(
    const std::string& frameName)
{
    if (_mappingContacts.count(frameName) == 0) {
        throw std::logic_error(
            "leph::SEIKOFloatingBase::getContactBase: "
            "Unknown contact frame name: "
            + frameName);
    }

    int index = _mappingContacts.at(frameName);
    if (index > 0) {
        return _contactsPlane.at(index-1);
    } else {
        return _contactsPoint.at(-index-1);
    }
}
SEIKOFloatingBase::ContactPlane_t& SEIKOFloatingBase::getContactPlane(
    const std::string& frameName)
{
    if (_mappingContacts.count(frameName) == 0) {
        throw std::logic_error(
            "leph::SEIKOFloatingBase::getContactPlane: "
            "Unknown contact frame name: "
            + frameName);
    }

    int index = _mappingContacts.at(frameName);
    if (index > 0) {
        return _contactsPlane.at(index-1);
    } else {
        throw std::logic_error(
            "leph::SEIKOFloatingBase::getContactPlane: "
            "Contact is not plane: "
            + frameName);
    }
}
SEIKOFloatingBase::ContactPoint_t& SEIKOFloatingBase::getContactPoint(
    const std::string& frameName)
{
    if (_mappingContacts.count(frameName) == 0) {
        throw std::logic_error(
            "leph::SEIKOFloatingBase::getContactPoint: "
            "Unknown contact frame name: "
            + frameName);
    }

    int index = _mappingContacts.at(frameName);
    if (index > 0) {
        throw std::logic_error(
            "leph::SEIKOFloatingBase::getContactPoint: "
            "Contact is not point: "
            + frameName);
    } else {
        return _contactsPoint.at(-index-1);
    }
}

void SEIKOFloatingBase::writeContactConstraints(
    Eigen::MatrixXd& problemIneqMat,
    Eigen::VectorXd& problemIneqVec,
    bool isContactPlane,
    size_t indexContact,
    size_t offsetRow,
    size_t offsetCol)
{
    //Check input matrix and vector sizes
    if (
        (isContactPlane && offsetRow+18 > (size_t)problemIneqMat.rows()) ||
        (isContactPlane && offsetRow+18 > (size_t)problemIneqVec.size()) ||
        (isContactPlane && offsetCol+6 > (size_t)problemIneqMat.cols()) ||
        (!isContactPlane && offsetRow+6 > (size_t)problemIneqMat.rows()) ||
        (!isContactPlane && offsetRow+6 > (size_t)problemIneqVec.size()) ||
        (!isContactPlane && offsetCol+3 > (size_t)problemIneqMat.cols())
    ) {
        throw std::logic_error(
            "leph::SEIKOFloatingBase::writeContactConstraints: "
            "Invalid input matrix or vector sizes.");
    }
    //Check contact container index
    if (
        (isContactPlane && indexContact >= _contactsPlane.size()) ||
        (!isContactPlane && indexContact >= _contactsPoint.size())
    ) {
        throw std::logic_error(
            "leph::SEIKOFloatingBase::writeContactConstraints: "
            "Invalid contact index: " 
            + std::to_string(indexContact));
    }

    //Retrieve limits
    double normalForceMin = 0.0;
    double normalForceMax = 0.0;
    double frictionCoef = 0.0;
    double copMaxX = 0.0;
    double copMaxY = 0.0;
    if (isContactPlane) {
        normalForceMin = 
            _contactsPlane[indexContact].normalForceMin;
        normalForceMax = 
            _contactsPlane[indexContact].normalForceMax;
        frictionCoef = 
            _contactsPlane[indexContact].frictionCoef;
        copMaxX = 
            _contactsPlane[indexContact].limitCOP.x();
        copMaxY = 
            _contactsPlane[indexContact].limitCOP.y();
    } else {
        normalForceMin = 
            _contactsPoint[indexContact].normalForceMin;
        normalForceMax = 
            _contactsPoint[indexContact].normalForceMax;
        frictionCoef = 
            _contactsPoint[indexContact].frictionCoef;
    }
    double frictionCoefX = frictionCoef;
    double frictionCoefY = frictionCoef;

    //Contact plane constraints
    if (isContactPlane) {
        //Normal contact force bounds
        problemIneqMat(offsetRow+0, offsetCol+5) = 1.0;
        problemIneqVec(offsetRow+0) = -normalForceMin;
        problemIneqMat(offsetRow+1, offsetCol+5) = -1.0;
        problemIneqVec(offsetRow+1) = normalForceMax;
        //Contact friction pyramid
        problemIneqMat(offsetRow+2, offsetCol+3) = 1.0;
        problemIneqMat(offsetRow+2, offsetCol+5) = frictionCoefX;
        problemIneqMat(offsetRow+3, offsetCol+4) = 1.0;
        problemIneqMat(offsetRow+3, offsetCol+5) = frictionCoefY;
        problemIneqMat(offsetRow+4, offsetCol+3) = -1.0;
        problemIneqMat(offsetRow+4, offsetCol+5) = frictionCoefX;
        problemIneqMat(offsetRow+5, offsetCol+4) = -1.0;
        problemIneqMat(offsetRow+5, offsetCol+5) = frictionCoefY;
        //Plane center of pressure bounds
        problemIneqMat(offsetRow+6, offsetCol+1) = -1.0;
        problemIneqMat(offsetRow+6, offsetCol+5) = copMaxX;
        problemIneqMat(offsetRow+7, offsetCol+0) = 1.0;
        problemIneqMat(offsetRow+7, offsetCol+5) = copMaxY;
        problemIneqMat(offsetRow+8, offsetCol+1) = 1.0;
        problemIneqMat(offsetRow+8, offsetCol+5) = copMaxX;
        problemIneqMat(offsetRow+9, offsetCol+0) = -1.0;
        problemIneqMat(offsetRow+9, offsetCol+5) = copMaxY;
        //Plane torsional torque bounds
        //
        //Inequality over contact torque yaw no friction bounds.
        //See Stéphane Caron paper: 
        //"Stability of Surface Contacts for Humanoid Robots:
        //Closed-Form Formulae of the Contact Wrench Cone
        //for Rectangular Support Areas"
        //
        //tau_min <= tau_z <= tau_max
        //tau_min = -mu.(X+Y).f_z + |Y.f_x - mu.tau_x| + |X.f_y - mu.tau_y|
        //tau_max = +mu.(X+Y).f_z - |Y.f_x + mu.tau_x| - |X.f_y + mu.tau_y|
        //
        //Minimum bound
        problemIneqMat.block(offsetRow+10, offsetCol+0, 1, 6) <<
            frictionCoef, frictionCoef, 1.0, 
            -copMaxY, -copMaxX, frictionCoef*(copMaxX+copMaxY);
        problemIneqMat.block(offsetRow+11, offsetCol+0, 1, 6) <<
            frictionCoef, -frictionCoef, 1.0, 
            -copMaxY, copMaxX, frictionCoef*(copMaxX+copMaxY);
        problemIneqMat.block(offsetRow+12, offsetCol+0, 1, 6) <<
            -frictionCoef, -frictionCoef, 1.0, 
            copMaxY, copMaxX, frictionCoef*(copMaxX+copMaxY);
        problemIneqMat.block(offsetRow+13, offsetCol+0, 1, 6) <<
            -frictionCoef, frictionCoef, 1.0, 
            copMaxY, -copMaxX, frictionCoef*(copMaxX+copMaxY);
        //Maximum bound
        problemIneqMat.block(offsetRow+14, offsetCol+0, 1, 6) <<
            -frictionCoef, -frictionCoef, -1.0, 
            -copMaxY, -copMaxX, frictionCoef*(copMaxX+copMaxY);
        problemIneqMat.block(offsetRow+15, offsetCol+0, 1, 6) <<
            -frictionCoef, frictionCoef, -1.0, 
            -copMaxY, copMaxX, frictionCoef*(copMaxX+copMaxY);
        problemIneqMat.block(offsetRow+16, offsetCol+0, 1, 6) <<
            frictionCoef, frictionCoef, -1.0, 
            copMaxY, copMaxX, frictionCoef*(copMaxX+copMaxY);
        problemIneqMat.block(offsetRow+17, offsetCol+0, 1, 6) <<
            frictionCoef, -frictionCoef, -1.0, 
            copMaxY, -copMaxX, frictionCoef*(copMaxX+copMaxY);
    } 
    //Contact point constraints
    else {
        //Normal contact force bounds
        problemIneqMat(offsetRow+0, offsetCol+2) = 1.0;
        problemIneqVec(offsetRow+0) = -normalForceMin;
        problemIneqMat(offsetRow+1, offsetCol+2) = -1.0;
        problemIneqVec(offsetRow+1) = normalForceMax;
        //Contact friction pyramid
        problemIneqMat(offsetRow+2, offsetCol+0) = 1.0;
        problemIneqMat(offsetRow+2, offsetCol+2) = frictionCoefX;
        problemIneqMat(offsetRow+3, offsetCol+1) = 1.0;
        problemIneqMat(offsetRow+3, offsetCol+2) = frictionCoefY;
        problemIneqMat(offsetRow+4, offsetCol+0) = -1.0;
        problemIneqMat(offsetRow+4, offsetCol+2) = frictionCoefX;
        problemIneqMat(offsetRow+5, offsetCol+1) = -1.0;
        problemIneqMat(offsetRow+5, offsetCol+2) = frictionCoefY;
    }
}

void SEIKOFloatingBase::writePoseTarget(
    const ContactBase_t& contactRef,
    Eigen::MatrixXd& costMat,
    Eigen::VectorXd& costVec,
    Eigen::DiagonalMatrix<double, Eigen::Dynamic>& weights,
    size_t offsetRow,
    bool isAddPosition)
{
    const size_t& frameId = contactRef.frameId;
    //Compute clamped Cartesian orientation error vector
    Eigen::Vector3d deltaMat = ClampVectorNorm(
        MatrixToAxis(
            contactRef.targetMat
            * _model->orientation(frameId, 0).transpose()), 
        contactRef.clampMat);
    //Position and orientation case
    if (isAddPosition) {
        //Compute clamped Cartesian position 
        Eigen::Vector3d deltaPos = ClampVectorNorm(
            contactRef.targetPos - _model->position(frameId, 0), 
            contactRef.clampPos);
        //Assign matrix, vector and weight
        costMat.block(offsetRow, 0, 6, _sizeDOF) =
            contactRef.jacobianWorld;
        costVec.segment(offsetRow+0, 3) = deltaMat;
        costVec.segment(offsetRow+3, 3) = deltaPos;
        weights.diagonal().segment(offsetRow+0, 3) = 
            contactRef.weightMat*Eigen::VectorXd::Ones(3);
        weights.diagonal().segment(offsetRow+3, 3) = 
            contactRef.weightPos*Eigen::VectorXd::Ones(3);
    }
    //Only orientation case
    else {
        //Assign matrix, vector and weight
        costMat.block(offsetRow, 0, 3, _sizeDOF) =
            contactRef.jacobianWorld.block(0, 0, 3, _sizeDOF);
        costVec.segment(offsetRow+0, 3) = deltaMat;
        weights.diagonal().segment(offsetRow+0, 3) = 
            contactRef.weightMat*Eigen::VectorXd::Ones(3);
    }
}

}

