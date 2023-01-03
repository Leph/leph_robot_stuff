#include <stdexcept>
#include <leph_model/MultiContactIKStaticID.hpp>
#include <leph_eiquadprog/leph_eiquadprog.hpp>
#include <leph_maths/AxisAngle.h>
#include <leph_maths/Clamp.h>
#include <leph_maths/IntegrateDOFVect.h>

namespace leph {

MultiContactIKStaticID::MultiContactIKStaticID(Model& model) :
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
    _switchState(0),
    _switchIndexPlane(-1),
    _switchIndexPoint(-1),
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
    _coefIDWeightInIK(0.00001),
    _ratioLimitsDynamics(1.0),
    _isDisabledConstraints(false),
    _isAdaptativeConstraints(false),
    _pinocchio(model)
{
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
    _jointWeightPos = 2.5*Eigen::VectorXd::Ones(_sizeJoint);
    _jointWeightVel = 10000.0*Eigen::VectorXd::Ones(_sizeJoint);
    _jointClampPos = 0.1;
    _jointTargetTau = Eigen::VectorXd::Zero(_sizeJoint);
    _jointWeightTau = 1.0*Eigen::VectorXd::Ones(_sizeJoint);
    _gravityVector = Eigen::VectorXd::Zero(_sizeDOF);
    _jointComputedDeltaTau = Eigen::VectorXd::Zero(_sizeJoint);
    _jointComputedDeltaDOF = Eigen::VectorXd::Zero(_sizeDOF);
    _jointStateTau = Eigen::VectorXd::Zero(_sizeJoint);
}
        
void MultiContactIKStaticID::addContactPlane(const std::string& frameName)
{
    //Check if given name is already registered
    if (_mappingContacts.count(frameName) != 0) {
        throw std::logic_error(
            "leph::MultiContactIKStaticID::addContactPlane: "
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
    contact.jacobianWorld = Eigen::MatrixXd::Zero(6, _sizeDOF);
    contact.jacobianBody = Eigen::MatrixXd::Zero(6, _sizeDOF);
    contact.limitCOP.x() = 0.12;
    contact.limitCOP.y() = 0.07;
    contact.targetWrench = Eigen::Vector6d::Zero();
    contact.weightWrench = 1e-6*Eigen::Vector6d::Ones();
    contact.computedDeltaWrench = Eigen::Vector6d::Zero();
    contact.stateWrench = Eigen::Vector6d::Zero();
    contact.readWrench = Eigen::Vector6d::Zero();

    //Add new contact plane
    _contactsPlane.push_back(contact);
    _mappingContacts.insert({frameName, _contactsPlane.size()});
    _sizePlaneOff++;
}
void MultiContactIKStaticID::addContactPoint(const std::string& frameName)
{
    //Check if given name is already registered
    if (_mappingContacts.count(frameName) != 0) {
        throw std::logic_error(
            "leph::MultiContactIKStaticID::addContactPoint: "
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
    contact.jacobianWorld = Eigen::MatrixXd::Zero(6, _sizeDOF);
    contact.jacobianBody = Eigen::MatrixXd::Zero(6, _sizeDOF);
    contact.contactMat = Eigen::Matrix3d::Identity();
    contact.targetForce = Eigen::Vector3d::Zero();
    contact.weightForce = 1e-6*Eigen::Vector3d::Ones();
    contact.computedDeltaForce = Eigen::Vector3d::Zero();
    contact.stateForce = Eigen::Vector3d::Zero();
    contact.readForce = Eigen::Vector3d::Zero();

    //Add new contact point
    _contactsPoint.push_back(contact);
    _mappingContacts.insert({frameName, -_contactsPoint.size()});
    _sizePointOff++;
}
        
void MultiContactIKStaticID::resetTargetsFromModel()
{
    _jointTargetPos = _model->getJointPosVect();
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

const MultiContactIKStaticID::ContactBase_t& MultiContactIKStaticID::stateContactBase(
    const std::string& frameName) const
{
    if (_mappingContacts.count(frameName) == 0) {
        throw std::logic_error(
            "leph::MultiContactIKStaticID::stateContactBase: "
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
const MultiContactIKStaticID::ContactPlane_t& MultiContactIKStaticID::stateContactPlane(
    const std::string& frameName) const
{
    if (_mappingContacts.count(frameName) == 0) {
        throw std::logic_error(
            "leph::MultiContactIKStaticID::stateContactPlane: "
            "Unknown contact frame name: "
            + frameName);
    }

    int index = _mappingContacts.at(frameName);
    if (index > 0) {
        return _contactsPlane.at(index-1);
    } else {
        throw std::logic_error(
            "leph::MultiContactIKStaticID::stateContactPlane: "
            "Contact is not plane: "
            + frameName);
    }
}
const MultiContactIKStaticID::ContactPoint_t& MultiContactIKStaticID::stateContactPoint(
    const std::string& frameName) const
{
    if (_mappingContacts.count(frameName) == 0) {
        throw std::logic_error(
            "leph::MultiContactIKStaticID::stateContactPoint: "
            "Unknown contact frame name: "
            + frameName);
    }

    int index = _mappingContacts.at(frameName);
    if (index > 0) {
        throw std::logic_error(
            "leph::MultiContactIKStaticID::stateContactPoint: "
            "Contact is not point: "
            + frameName);
    } else {
        return _contactsPoint.at(-index-1);
    }
}

void MultiContactIKStaticID::setReadContactWrench(
    const std::string& frameName,
    const Eigen::Vector6d& readWrench)
{
    if (_mappingContacts.count(frameName) == 0) {
        throw std::logic_error(
            "leph::MultiContactIKStaticID::setReadContactWrench: "
            "Unknown contact frame name: "
            + frameName);
    }

    int index = _mappingContacts.at(frameName);
    if (index > 0) {
        _contactsPlane.at(index-1).readWrench = readWrench;
    } else {
        throw std::logic_error(
            "leph::MultiContactIKStaticID::setReadContactWrench: "
            "Contact is not plane: "
            + frameName);
    }
}
void MultiContactIKStaticID::setReadContactForce(
    const std::string& frameName,
    const Eigen::Vector3d& readForce)
{
    if (_mappingContacts.count(frameName) == 0) {
        throw std::logic_error(
            "leph::MultiContactIKStaticID::setReadContactForce: "
            "Unknown contact frame name: "
            + frameName);
    }

    int index = _mappingContacts.at(frameName);
    if (index > 0) {
        throw std::logic_error(
            "leph::MultiContactIKStaticID::setReadContactForce: "
            "Contact is not point: "
            + frameName);
    } else {
        _contactsPoint.at(-index-1).readForce = readForce;
    }
}
        
bool MultiContactIKStaticID::toggleContact(
    const std::string& frameName,
    bool isEnabled)
{
    if (_mappingContacts.count(frameName) == 0) {
        throw std::logic_error(
            "leph::MultiContactIKStaticID::toggleContact: "
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
        
void MultiContactIKStaticID::switchingAsk(const std::string& frameName)
{
    if (_switchState != 0) {
        throw std::logic_error(
            "leph::MultiContactIKStaticID::switchingAsk: "
            "Switching already asked.");
    }

    int index = _mappingContacts.at(frameName);
    if (index > 0) {
        _switchIndexPlane = index-1;
        _switchIndexPoint = -1;
        if (_contactsPlane.at(_switchIndexPlane).isEnabled) {
            _switchState = -1;
        } else {
            _switchState = 1;
        }
    } else if (index < 0) {
        _switchIndexPlane = -1;
        _switchIndexPoint = -index-1;
        if (_contactsPoint.at(_switchIndexPoint).isEnabled) {
            _switchState = -1;
        } else {
            _switchState = 1;
        }
    }
}
void MultiContactIKStaticID::switchingClear()
{
    _switchState = 0;
    _switchIndexPlane = -1;
    _switchIndexPoint = -1;
}

///////////////////////////////////////////////////////////XXX
void MultiContactIKStaticID::setJointLimitPos(
    const std::string& name,
    double limitLower, double limitUpper)
{
    size_t index = _model->getIndexDOF(name);
    _jointLimitPosLower(index-6) = limitLower;
    _jointLimitPosUpper(index-6) = limitUpper;
}
void MultiContactIKStaticID::setJointLimitVelAbs(double maxVel)
{
    _jointLimitVelAbs = maxVel*Eigen::VectorXd::Ones(_sizeJoint);
}
void MultiContactIKStaticID::setJointTargetPos(
    const Eigen::VectorXd& pos)
{
    if ((size_t)pos.size() != _sizeJoint) {
        throw std::logic_error(
            "leph::MultiContactIKStaticID::setJointTargetPos: "
            "Invalid vector size.");
    }
    _jointTargetPos = pos;
}
void MultiContactIKStaticID::setJointWeightPos(double weightPos)
{
    _jointWeightPos = weightPos*Eigen::VectorXd::Ones(_sizeJoint);
}
void MultiContactIKStaticID::setJointWeightVel(double weightVel)
{
    _jointWeightVel = weightVel*Eigen::VectorXd::Ones(_sizeJoint);
}
void MultiContactIKStaticID::setJointWeightTau(double weightTau)
{
    _jointWeightTau = weightTau*Eigen::VectorXd::Ones(_sizeJoint);
}
void MultiContactIKStaticID::setWeightCoefIDInIK(double coef)
{
    _coefIDWeightInIK = coef;
}
void MultiContactIKStaticID::setRatioLimits(double ratio)
{
    _ratioLimitsDynamics = ratio;
}
void MultiContactIKStaticID::setDisableConstraints(bool isDisabled)
{
    _isDisabledConstraints = isDisabled;
}
Eigen::Vector3d& MultiContactIKStaticID::refTargetPos(
    const std::string& frameName)
{
    return getContactBase(frameName).targetPos;
}
Eigen::Matrix3d& MultiContactIKStaticID::refTargetMat(
    const std::string& frameName)
{
    return getContactBase(frameName).targetMat;
}
void MultiContactIKStaticID::setWeightPose(
    const std::string& frameName,
    double weightPos, double weightMat)
{
    getContactBase(frameName).weightPos = weightPos;
    getContactBase(frameName).weightMat = weightMat;
}
void MultiContactIKStaticID::setNormalForceLimits(
    const std::string& frameName,
    double normalForceMin, double normalForceMax)
{
    getContactBase(frameName).normalForceMin = normalForceMin;
    getContactBase(frameName).normalForceMax = normalForceMax;
}
void MultiContactIKStaticID::setContactPlaneLimitCOP(
    const std::string& frameName,
    double limitX, double limitY)
{
    Eigen::Vector2d limit;
    limit.x() = limitX;
    limit.y() = limitY;
    getContactPlane(frameName).limitCOP = limit;
}
void MultiContactIKStaticID::setContactLimitFriction(
    const std::string& frameName,
    double frictionCoef)
{
    getContactBase(frameName).frictionCoef = frictionCoef;
}
void MultiContactIKStaticID::setContactMat(
    const std::string& frameName,
    const Eigen::Matrix3d& contactMat)
{
    getContactPoint(frameName).contactMat = contactMat;
}
void MultiContactIKStaticID::setTargetWrench(
    const std::string& frameName,
    const Eigen::Vector6d& targetWrench)
{
    ContactPlane_t& contact = getContactPlane(frameName);
    contact.targetWrench = targetWrench;
}
void MultiContactIKStaticID::setWeightWrench(
    const std::string& frameName,
    const Eigen::Vector6d& weightWrench)
{
    ContactPlane_t& contact = getContactPlane(frameName);
    contact.weightWrench = weightWrench;
}
void MultiContactIKStaticID::setTargetForce(
    const std::string& frameName,
    const Eigen::Vector3d& targetForce)
{
    ContactPoint_t& contact = getContactPoint(frameName);
    contact.targetForce = targetForce;
}
void MultiContactIKStaticID::setWeightForce(
    const std::string& frameName,
    const Eigen::Vector3d& weightForce)
{
    ContactPoint_t& contact = getContactPoint(frameName);
    contact.weightForce = weightForce;
}
///////////////////////////////////////////////////////////XXX

bool MultiContactIKStaticID::runInverseDynamics(
    bool isSwitch)
{
    //Define problem sizes for 
    //static inverse dynamics
    size_t sizeSol = 
        //Joint torques
        _sizeJoint +
        //Plane contact wrenches
        6*_sizePlaneOn +
        //Point contact forces
        3*_sizePointOn;
    size_t sizeCost = 
        //Targets for joint torques and 
        //contact wrenches and forces
        sizeSol;
    size_t sizeEq =
        //Static equation of motion
        _sizeDOF;
    size_t sizeIneq =
        //Joint torques bounds
        2*_sizeJoint + 
        //Contact plane constraints
        18*_sizePlaneOn +
        //Contact point constraints
        6*_sizePointOn;
    //Contact switching inverse dynamics
    //include or exclude switching contact
    if (isSwitch && _switchState > 0) {
        if (_switchIndexPlane >= 0) {
            sizeSol += 6;
            sizeCost += 6;
            sizeIneq += 18;
        } else {
            sizeSol += 3;
            sizeCost += 3;
            sizeIneq += 6;
        }
    } else if (isSwitch && _switchState < 0) {
        if (_switchIndexPlane >= 0) {
            sizeSol -= 6;
            sizeCost -= 6;
            sizeIneq -= 18;
        } else {
            sizeSol -= 3;
            sizeCost -= 3;
            sizeIneq -= 6;
        }
    }
    
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
        if (
            (!isSwitch && _contactsPlane[i].isEnabled) ||
            (isSwitch && isPlaneIndexInSwitch(i))
        ) {
            _contactsPlane[i].jacobianBody = _model->pointJacobian(
                _contactsPlane[i].frameId, _contactsPlane[i].frameId);
        }
    }
    for (size_t i=0;i<_contactsPoint.size();i++) {
        if (
            (!isSwitch && _contactsPoint[i].isEnabled) || 
            (isSwitch && isPointIndexInSwitch(i))
        ) {
            _contactsPoint[i].jacobianWorld = _model->pointJacobian(
                _contactsPoint[i].frameId, 0);
        }
    }
    
    //Static inverse dynamics matrices setup
    //Cost matrix
    size_t offsetCost = 0;
    costMat.setIdentity();
    //Joint torque
    costVec.segment(offsetCost, _sizeJoint) = 
        _jointTargetTau;
    weights.diagonal().segment(offsetCost, _sizeJoint) = 
        _jointWeightTau;
    offsetCost += _sizeJoint;
    //Contact plane wrenches
    for (size_t i=0;i<_contactsPlane.size();i++) {
        if (
            (!isSwitch && _contactsPlane[i].isEnabled) || 
            (isSwitch && isPlaneIndexInSwitch(i))
        ) {
            costVec.segment(offsetCost, 6) = 
                _contactsPlane[i].targetWrench;
            weights.diagonal().segment(offsetCost, 6) = 
                _contactsPlane[i].weightWrench;
            offsetCost += 6;
        }
    }
    //Contact point forces
    for (size_t i=0;i<_contactsPoint.size();i++) {
        if (
            (!isSwitch && _contactsPoint[i].isEnabled) || 
            (isSwitch && isPointIndexInSwitch(i))
        ) {
            costVec.segment(offsetCost, 3) = 
                _contactsPoint[i].targetForce;
            weights.diagonal().segment(offsetCost, 3) = 
                _contactsPoint[i].weightForce;
            offsetCost += 3;
        }
    }

    //Build equality constraints matrix
    //Static equation of motion
    size_t offsetEq = 0;
    problemEqMat.block(6, offsetEq, _sizeJoint, _sizeJoint).setIdentity();
    offsetEq += _sizeJoint;
    for (size_t i=0;i<_contactsPlane.size();i++) {
        if (
            (!isSwitch && _contactsPlane[i].isEnabled) ||
            (isSwitch && isPlaneIndexInSwitch(i))
        ) {
            problemEqMat.block(0, offsetEq, _sizeDOF, 6) = 
                _contactsPlane[i].jacobianBody.transpose();
            offsetEq += 6;
        }
    }
    for (size_t i=0;i<_contactsPoint.size();i++) {
        if (
            (!isSwitch && _contactsPoint[i].isEnabled) ||
            (isSwitch && isPointIndexInSwitch(i))
        ) {
            problemEqMat.block(0, offsetEq, _sizeDOF, 3) = 
                _contactsPoint[i].jacobianWorld
                    .block(3, 0, 3, _sizeDOF).transpose()
                * _contactsPoint[i].contactMat;
            offsetEq += 3;
        }
    }
    //Gravity vector
    problemEqVec = -_gravityVector;

    //Build inequality constraints matrix
    buildStaticIDInequalities(
        problemIneqMat, 
        problemIneqVec, 
        _ratioLimitsDynamics,
        isSwitch);

    //Build the weighted distance of linear target costs
    problemCostMat = costMat.transpose()*weights*costMat;
    problemCostVec = -costMat.transpose()*weights*costVec;

    //Solve the QP problem
    double cost = 0.0;
    if (_isDisabledConstraints) {
        cost = Eigen::solve_quadprog(
            problemCostMat,
            problemCostVec,
            problemEqMat.transpose(),
            problemEqVec,
            Eigen::MatrixXd::Zero(sizeSol, 0),
            Eigen::VectorXd::Zero(0),
            problemSolution);
    } else {
        cost = Eigen::solve_quadprog(
            problemCostMat,
            problemCostVec,
            problemEqMat.transpose(),
            problemEqVec,
            problemIneqMat.transpose(),
            problemIneqVec,
            problemSolution);
    }

    //Check solver success
    if (std::isnan(cost) || std::isinf(cost)) {
        //Return failure
        return false;
    } else {
        //Copy computed solution to internal state
        size_t offsetSol = 0;
        //Joint torques
        _jointStateTau = problemSolution.segment(offsetSol, _sizeJoint);
        offsetSol += _sizeJoint;
        //Contact wrenches
        for (size_t i=0;i<_contactsPlane.size();i++) {
            if (
                (!isSwitch && _contactsPlane[i].isEnabled) ||
                (isSwitch && isPlaneIndexInSwitch(i))
            ) {
                _contactsPlane[i].stateWrench = 
                    problemSolution.segment(offsetSol, 6);
                offsetSol += 6;
            } else {
                _contactsPlane[i].stateWrench.setZero();
            }
        }
        //Contact forces
        for (size_t i=0;i<_contactsPoint.size();i++) {
            if (
                (!isSwitch && _contactsPoint[i].isEnabled) ||
                (isSwitch && isPointIndexInSwitch(i))
            ) {
                _contactsPoint[i].stateForce = 
                    problemSolution.segment(offsetSol, 3);
                offsetSol += 3;
            } else {
                _contactsPoint[i].stateForce.setZero();
            }
        }
        //Return success
        return true;
    }
}
bool MultiContactIKStaticID::runInverseDynamicsFast(
    bool isSwitch)
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
    //Contact switching inverse dynamics
    //include or exclude switching contact
    if (isSwitch && _switchState > 0) {
        if (_switchIndexPlane >= 0) {
            sizeSol += 6;
            sizeCost += 6;
            sizeIneq += 18;
        } else {
            sizeSol += 3;
            sizeCost += 3;
            sizeIneq += 6;
        }
    } else if (isSwitch && _switchState < 0) {
        if (_switchIndexPlane >= 0) {
            sizeSol -= 6;
            sizeCost -= 6;
            sizeIneq -= 18;
        } else {
            sizeSol -= 3;
            sizeCost -= 3;
            sizeIneq -= 6;
        }
    }
    
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
        if (
            (!isSwitch && _contactsPlane[i].isEnabled) ||
            (isSwitch && isPlaneIndexInSwitch(i))
        ) {
            _contactsPlane[i].jacobianBody = _model->pointJacobian(
                _contactsPlane[i].frameId, _contactsPlane[i].frameId);
        }
    }
    for (size_t i=0;i<_contactsPoint.size();i++) {
        if (
            (!isSwitch && _contactsPoint[i].isEnabled) || 
            (isSwitch && isPointIndexInSwitch(i))
        ) {
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
        if (
            (!isSwitch && _contactsPlane[i].isEnabled) || 
            (isSwitch && isPlaneIndexInSwitch(i))
        ) {
            solToTauMat.block(0, offsetTmpCol, _sizeDOF, 6) = 
                -_contactsPlane[i].jacobianBody.transpose();
            offsetTmpCol += 6;
        }
    }
    //Contact point forces
    for (size_t i=0;i<_contactsPoint.size();i++) {
        if (
            (!isSwitch && _contactsPoint[i].isEnabled) || 
            (isSwitch && isPointIndexInSwitch(i))
        ) {
            solToTauMat.block(0, offsetTmpCol, _sizeDOF, 3) =
                -(_contactsPoint[i].jacobianWorld
                    .block(3, 0, 3, _sizeDOF).transpose()
                * _contactsPoint[i].contactMat);
            offsetTmpCol += 3;
        }
    }
    //Gravity vector
    solToTauVec = _gravityVector;
    
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
        if (
            (!isSwitch && _contactsPlane[i].isEnabled) || 
            (isSwitch && isPlaneIndexInSwitch(i))
        ) {
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
        if (
            (!isSwitch && _contactsPoint[i].isEnabled) || 
            (isSwitch && isPointIndexInSwitch(i))
        ) {
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
    buildStaticIDInequalities(
        tmpIDIneqMat, tmpIDIneqVec,
        _ratioLimitsDynamics, isSwitch);
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
    double cost = 0.0;
    if (_isDisabledConstraints) {
        cost = Eigen::solve_quadprog(
            problemCostMat,
            problemCostVec,
            problemEqMat.transpose(),
            problemEqVec,
            Eigen::MatrixXd::Zero(sizeSol, 0),
            Eigen::VectorXd::Zero(0),
            problemSolution);
    } else {
        cost = Eigen::solve_quadprog(
            problemCostMat,
            problemCostVec,
            problemEqMat.transpose(),
            problemEqVec,
            problemIneqMat.transpose(),
            problemIneqVec,
            problemSolution);
    }

    //Check solver success
    if (std::isnan(cost) || std::isinf(cost)) {
        //Return failure
        return false;
    } else {
        //Copy computed solution to internal state
        size_t offsetSolRow = 0;
        //Contact wrenches
        for (size_t i=0;i<_contactsPlane.size();i++) {
            if (
                (!isSwitch && _contactsPlane[i].isEnabled) ||
                (isSwitch && isPlaneIndexInSwitch(i))
            ) {
                _contactsPlane[i].stateWrench = 
                    problemSolution.segment(offsetSolRow, 6);
                offsetSolRow += 6;
            } else {
                _contactsPlane[i].stateWrench.setZero();
            }
        }
        //Contact forces
        for (size_t i=0;i<_contactsPoint.size();i++) {
            if (
                (!isSwitch && _contactsPoint[i].isEnabled) ||
                (isSwitch && isPointIndexInSwitch(i))
            ) {
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
        
bool MultiContactIKStaticID::runCombinedIKID(double dt)
{
    //Define problem sizes
    size_t sizeSolID = 
        //Delta joint torques
        _sizeJoint + 
        //Delta plane contact wrenches
        6*_sizePlaneOn + 
        //Delta point contact forces
        3*_sizePointOn;
    size_t sizeSol = 
        //Delta position for each DOF
        _sizeDOF +
        //Delta static torques wrenches and forces
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
        //Static equation of motion
        _sizeDOF;
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
    _gravityVector = _model->computeGravityVector();

    //Store model position and velocity state
    Eigen::VectorXd statePosition = _model->getDOFPosVect();
    Eigen::VectorXd stateVelocity = _model->getDOFVelVect();
    
    //Compute dynamics differentiation
    //Initialization
    Eigen::MatrixXd diffGravity = Eigen::MatrixXd::Zero(_sizeDOF, _sizeDOF);
    std::map<size_t, Eigen::MatrixXd> diffHessianPlane;
    std::map<size_t, Eigen::MatrixXd> diffHessianPoint;
    for (size_t i=0;i<_contactsPlane.size();i++) {
        if (_contactsPlane[i].isEnabled) {
            diffHessianPlane.insert(std::make_pair(
                i, Eigen::MatrixXd::Zero(_sizeDOF, _sizeDOF)));
        }
    }
    for (size_t i=0;i<_contactsPoint.size();i++) {
        if (_contactsPoint[i].isEnabled) {
            diffHessianPoint.insert(std::make_pair(
                i, Eigen::MatrixXd::Zero(_sizeDOF, _sizeDOF)));
        }
    }
    //Numerical differentiation
    for (size_t i=0;i<_sizeDOF;i++) {
        Eigen::VectorXd gravityVectorDiff;
        std::map<size_t, Eigen::MatrixXd> jacobianPlaneDiffT;
        std::map<size_t, Eigen::MatrixXd> jacobianPointDiffT;
        computeDiffDynamics(
            statePosition, i, 1e-6,
            gravityVectorDiff, jacobianPlaneDiffT, jacobianPointDiffT);
        //Build gravity vector diff matrix
        diffGravity.col(i) = gravityVectorDiff;
        //Build plane hessian diff matrix
        for (size_t j=0;j<_contactsPlane.size();j++) {
            if (_contactsPlane[j].isEnabled) {
                diffHessianPlane.at(j).col(i) = 
                    jacobianPlaneDiffT.at(j)
                    * _contactsPlane[j].stateWrench;
            }
        }
        //Build point hessian diff matrix
        for (size_t j=0;j<_contactsPoint.size();j++) {
            if (_contactsPoint[j].isEnabled) {
                diffHessianPoint.at(j).col(i) = 
                    jacobianPointDiffT.at(j)
                    * _contactsPoint[j].stateForce;
            }
        }
    }

    //Restore model position and velocity state
    _model->setDOFPosVect(statePosition);
    _model->setDOFVelVect(stateVelocity);
    _model->updateState();

    //Build current ID state vector
    Eigen::VectorXd currentIDVect = Eigen::VectorXd::Zero(sizeSolID);
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
    
    //Build cost
    size_t offsetCost = 0;
    //Velocity regularization
    costMat.block(offsetCost, 0, _sizeDOF, _sizeDOF) = 
        Eigen::MatrixXd::Identity(_sizeDOF, _sizeDOF);
    costVec.segment(offsetCost, _sizeDOF) = 
        Eigen::VectorXd::Zero(_sizeDOF);
    //Nearly no regularization for floating base DOF
    weights.diagonal().segment(offsetCost, 6) = 
        1e-8*Eigen::VectorXd::Ones(6);
    //Joint case
    weights.diagonal().segment(offsetCost+6, _sizeJoint) = 
        dt*_jointWeightVel;
    offsetCost += _sizeDOF;
    //Joint position target
    costMat.block(offsetCost, 6, _sizeJoint, _sizeJoint) = 
        Eigen::MatrixXd::Identity(_sizeJoint, _sizeJoint);
    costVec.segment(offsetCost, _sizeJoint) = ClampVectorAllComponent(
        _jointTargetPos - _model->getJointPosVect(), _jointClampPos);
    weights.diagonal().segment(offsetCost, _sizeJoint) = 
        _jointWeightPos;
    offsetCost += _sizeJoint;
    //Disable pose contact target
    for (size_t i=0;i<_contactsPlane.size();i++) {
        if (!_contactsPlane[i].isEnabled) {
            writePoseTarget(
                _contactsPlane[i],
                costMat, costVec, weights, 
                offsetCost,
                true);
            offsetCost += 6;
        }
    }
    //Disable point contact target
    for (size_t i=0;i<_contactsPoint.size();i++) {
        if (!_contactsPoint[i].isEnabled) {
            writePoseTarget(
                _contactsPoint[i],
                costMat, costVec, weights, 
                offsetCost,
                true);
            offsetCost += 6;
        }
    }
    //Enabled point contact orientation target
    for (size_t i=0;i<_contactsPoint.size();i++) {
        if (_contactsPoint[i].isEnabled) {
            writePoseTarget(
                _contactsPoint[i],
                costMat, costVec, weights, 
                offsetCost,
                false);
            offsetCost += 3;
        }
    }
    //Delta torques wrenches and forces
    costMat.block(offsetCost, _sizeDOF, sizeSolID, sizeSolID) = 
        Eigen::MatrixXd::Identity(sizeSolID, sizeSolID);
    //Joint torque
    costVec.segment(offsetCost, _sizeJoint) = 
        _jointTargetTau - _jointStateTau;
    weights.diagonal().segment(offsetCost, _sizeJoint) = 
        _coefIDWeightInIK*_jointWeightTau;
    offsetCost += _sizeJoint;
    //Contact plane wrenches
    for (size_t i=0;i<_contactsPlane.size();i++) {
        if (_contactsPlane[i].isEnabled) {
            costVec.segment(offsetCost, 6) = 
                _contactsPlane[i].targetWrench 
                - _contactsPlane[i].stateWrench;
            weights.diagonal().segment(offsetCost, 6) = 
                _coefIDWeightInIK*_contactsPlane[i].weightWrench;
            offsetCost += 6;
        }
    }
    //Contact point forces
    for (size_t i=0;i<_contactsPoint.size();i++) {
        if (_contactsPoint[i].isEnabled) {
            costVec.segment(offsetCost, 3) = 
                _contactsPoint[i].targetForce
                - _contactsPoint[i].stateForce;
            weights.diagonal().segment(offsetCost, 3) = 
                _coefIDWeightInIK*_contactsPoint[i].weightForce;
            offsetCost += 3;
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
    size_t offsetIDCol = 0;
    //Delta positions
    problemEqMat.block(offsetEq, offsetIDCol, _sizeDOF, _sizeDOF) = 
        diffGravity;
    for (size_t i=0;i<_contactsPlane.size();i++) {
        if (_contactsPlane[i].isEnabled) {
            problemEqMat.block(offsetEq, offsetIDCol, _sizeDOF, _sizeDOF) -= 
                diffHessianPlane.at(i);
        }
    }
    for (size_t i=0;i<_contactsPoint.size();i++) {
        if (_contactsPoint[i].isEnabled) {
            problemEqMat.block(offsetEq, offsetIDCol, _sizeDOF, _sizeDOF) -= 
                diffHessianPoint.at(i);
        }
    }
    offsetIDCol += _sizeDOF;
    //Delta torques
    problemEqMat.block(offsetEq+6, offsetIDCol, _sizeJoint, _sizeJoint) = 
        -Eigen::MatrixXd::Identity(_sizeJoint, _sizeJoint);
    offsetIDCol += _sizeJoint;
    //Delta wrenches
    for (size_t i=0;i<_contactsPlane.size();i++) {
        if (_contactsPlane[i].isEnabled) {
            problemEqMat.block(offsetEq, offsetIDCol, _sizeDOF, 6) = 
                -_contactsPlane[i].jacobianBody.transpose();
            offsetIDCol += 6;
        }
    }
    //Delta forces
    for (size_t i=0;i<_contactsPoint.size();i++) {
        if (_contactsPoint[i].isEnabled) {
            problemEqMat.block(offsetEq, offsetIDCol, _sizeDOF, 3) = 
                -_contactsPoint[i].jacobianWorld
                    .block(3, 0, 3, _sizeDOF).transpose()
                * _contactsPoint[i].contactMat;
            offsetIDCol += 3;
        }
    }
    //Set equality bias to static equilibrium error vector
    problemEqVec.segment(offsetEq, _sizeDOF).setZero();
    problemEqVec.segment(offsetEq, _sizeDOF) += _gravityVector;
    problemEqVec.segment(offsetEq+6, _sizeJoint) -= 
        _jointStateTau;
    for (size_t i=0;i<_contactsPlane.size();i++) {
        if (_contactsPlane[i].isEnabled) {
            problemEqVec.segment(offsetEq, _sizeDOF) -= 
                _contactsPlane[i].jacobianBody.transpose()
                * _contactsPlane[i].stateWrench;
        }
    }
    for (size_t i=0;i<_contactsPoint.size();i++) {
        if (_contactsPoint[i].isEnabled) {
            problemEqVec.segment(offsetEq, _sizeDOF) -=
                _contactsPoint[i].jacobianWorld.block(3, 0, 3, _sizeDOF).transpose()
                * _contactsPoint[i].contactMat
                * _contactsPoint[i].stateForce;
        }
    }

    //Build inequality constraints 
    size_t offsetIneq = 0;
    //Joint kinematics bounds
    for (size_t i=0;i<_sizeJoint;i++) {
        //XXX 10 because increase safety...
        problemIneqMat(offsetIneq+0, 6+i) = 10.0;
        problemIneqMat(offsetIneq+1, 6+i) = -10.0;
        problemIneqVec(offsetIneq+0) = 
            _model->getDOFPos(i+6) - 
            _ratioLimitsDynamics*_jointLimitPosLower(i);
        problemIneqVec(offsetIneq+1) = 
            -_model->getDOFPos(i+6) + 
            _ratioLimitsDynamics*_jointLimitPosUpper(i);
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
    //Static torques wrenches forces
    Eigen::MatrixXd tmpIDIneqMat;
    Eigen::VectorXd tmpIDIneqVec;
    buildStaticIDInequalities(
        tmpIDIneqMat, tmpIDIneqVec,
        _ratioLimitsDynamics, false);
    problemIneqMat.block(offsetIneq, _sizeDOF, sizeIDIneq, sizeSolID) = 
        tmpIDIneqMat;
    problemIneqVec.segment(offsetIneq, sizeIDIneq) = 
        tmpIDIneqMat*currentIDVect + tmpIDIneqVec;
    offsetIneq += sizeIDIneq;

    //Build the weighted distance of linear target costs
    problemCostMat = costMat.transpose()*weights*costMat;
    problemCostVec = -costMat.transpose()*weights*costVec;
    
    //Solve the QP problem
    double cost = 0.0;
    if (_isDisabledConstraints) {
        cost = Eigen::solve_quadprog(
            problemCostMat,
            problemCostVec,
            problemEqMat.transpose(),
            problemEqVec,
            Eigen::MatrixXd::Zero(sizeSol, 0),
            Eigen::VectorXd::Zero(0),
            problemSolution);
    } else {
        cost = Eigen::solve_quadprog(
            problemCostMat,
            problemCostVec,
            problemEqMat.transpose(),
            problemEqVec,
            problemIneqMat.transpose(),
            problemIneqVec,
            problemSolution);
    }
    
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
        //Joint torques
        _jointComputedDeltaTau = 
            problemSolution.segment(offsetSolRow, _sizeJoint);
        offsetSolRow += _sizeJoint;
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
        //Return success
        return true;
    }
}
bool MultiContactIKStaticID::runCombinedIKIDFast(double dt)
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
        _coefIDWeightInIK*_jointWeightTau;
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
                _coefIDWeightInIK*_contactsPlane[i].weightWrench;
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
                _coefIDWeightInIK*_contactsPoint[i].weightForce;
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
    problemEqVec.segment(offsetEq, 6) = 
        solToTauVec.segment(0, 6);
    
    //Build inequality constraints 
    size_t offsetIneq = 0;
    //Joint kinematics bounds
    for (size_t i=0;i<_sizeJoint;i++) {
        //XXX 10 because increase safety...
        problemIneqMat(offsetIneq+0, 6+i) = 10.0;
        problemIneqMat(offsetIneq+1, 6+i) = -10.0;
        problemIneqVec(offsetIneq+0) = 
            _model->getDOFPos(i+6) - 
            _ratioLimitsDynamics*_jointLimitPosLower(i);
        problemIneqVec(offsetIneq+1) = 
            -_model->getDOFPos(i+6) + 
            _ratioLimitsDynamics*_jointLimitPosUpper(i);
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
    //Static torques wrenches forces
    Eigen::MatrixXd tmpIDIneqMat;
    Eigen::VectorXd tmpIDIneqVec;
    //Build constraints matrix and vector over 
    //joint torques and contact wrenches/forces
    buildStaticIDInequalities(
        tmpIDIneqMat, tmpIDIneqVec,
        _ratioLimitsDynamics, false);
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
    double cost = 0.0;
    if (_isDisabledConstraints) {
        cost = Eigen::solve_quadprog(
            problemCostMat,
            problemCostVec,
            problemEqMat.transpose(),
            problemEqVec,
            Eigen::MatrixXd::Zero(sizeSol, 0),
            Eigen::VectorXd::Zero(0),
            problemSolution);
    } else {
        cost = Eigen::solve_quadprog(
            problemCostMat,
            problemCostVec,
            problemEqMat.transpose(),
            problemEqVec,
            problemIneqMat.transpose(),
            problemIneqVec,
            problemSolution);
    }
    
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
        
void MultiContactIKStaticID::integrateComputedDelta(double dt)
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
        
const Eigen::VectorXd& MultiContactIKStaticID::deltaDOFPosition() const
{
    return _jointComputedDeltaDOF;
}
const Eigen::VectorXd& MultiContactIKStaticID::deltaJointTorque() const
{
    return _jointComputedDeltaTau;
}
const Eigen::Vector6d& MultiContactIKStaticID::deltaContactWrench(
    const std::string& frameName) const
{
    return stateContactPlane(frameName).computedDeltaWrench;
}
const Eigen::Vector3d& MultiContactIKStaticID::deltaContactForce(
    const std::string& frameName) const
{
    return stateContactPoint(frameName).computedDeltaForce;
}

const Eigen::VectorXd& MultiContactIKStaticID::stateJointTorque() const
{
    return _jointStateTau;
}
Eigen::VectorXd& MultiContactIKStaticID::stateJointTorque()
{
    return _jointStateTau;
}
const Eigen::Vector6d& MultiContactIKStaticID::stateContactWrench(
    const std::string& frameName) const
{
    return stateContactPlane(frameName).stateWrench;
}
Eigen::Vector6d& MultiContactIKStaticID::stateContactWrench(
    const std::string& frameName)
{
    return getContactPlane(frameName).stateWrench;
}
const Eigen::Vector3d& MultiContactIKStaticID::stateContactForce(
    const std::string& frameName) const
{
    return stateContactPoint(frameName).stateForce;
}
Eigen::Vector3d& MultiContactIKStaticID::stateContactForce(
    const std::string& frameName)
{
    return getContactPoint(frameName).stateForce;
}

bool MultiContactIKStaticID::checkLimitsComputedIDSolution(
    const Eigen::VectorXd& vectorIDSolution,
    double ratioLimit,
    bool isSwitch,
    double* ratioMax,
    std::ostringstream* ss,
    bool printAll) const
{
    double tmpRatioMax = 0.0;
    if (ratioMax != nullptr) {
        *ratioMax = 0.0;
    }
    size_t offsetRow = 0;
    bool isValid = true;
    bool isTorqueValid = checkLimitsJointTorque(
        vectorIDSolution.segment(offsetRow, _sizeJoint), 
        ratioLimit, &tmpRatioMax, ss, printAll);
    isValid = isValid && isTorqueValid;
    if (ratioMax != nullptr) {
        *ratioMax = std::max(*ratioMax, tmpRatioMax);
    }
    offsetRow += _sizeJoint;
    for (size_t i=0;i<_contactsPlane.size();i++) {
        if (
            (!isSwitch && _contactsPlane[i].isEnabled) ||
            (isSwitch && isPlaneIndexInSwitch(i))
        ) {
            bool isWrenchValid = checkLimitsContactWrench(
                vectorIDSolution.segment<6>(offsetRow, 6),
                i, ratioLimit, &tmpRatioMax, ss, printAll);
            isValid = isValid && isWrenchValid;
            if (ratioMax != nullptr) {
                *ratioMax = std::max(*ratioMax, tmpRatioMax);
            }
            offsetRow += 6;
        }
    }
    for (size_t i=0;i<_contactsPoint.size();i++) {
        if (
            (!isSwitch && _contactsPoint[i].isEnabled) ||
            (isSwitch && isPointIndexInSwitch(i))
        ) {
            bool isForceValid = checkLimitsContactForce(
                vectorIDSolution.segment<3>(offsetRow, 3), 
                i, ratioLimit, &tmpRatioMax, ss, printAll);
            isValid = isValid && isForceValid;
            if (ratioMax != nullptr) {
                *ratioMax = std::max(*ratioMax, tmpRatioMax);
            }
            offsetRow += 3;
        }
    }

    return isValid;
}

std::map<std::string, double> MultiContactIKStaticID::computeIKIDRatios() const
{
    //Container for computed ratio
    std::map<std::string, double> container;

    //Joint position
    Eigen::VectorXd neutral = 
        0.5*_ratioLimitsDynamics*_jointLimitPosLower 
        + 0.5*_ratioLimitsDynamics*_jointLimitPosUpper;
    Eigen::VectorXd range = 
        _ratioLimitsDynamics*_jointLimitPosUpper - neutral;
    Eigen::VectorXd position = 
        _model->getJointPosVect();
    for (size_t i=0;i<_sizeJoint;i++) {
        double ratio = std::fabs(neutral(i)-position(i))/range(i);
        container.insert(std::make_pair(
            "pos " + _model->getNameDOF(i+6), 
            ratio));
    }

    //Joint torque
    for (size_t i=0;i<_sizeJoint;i++) {
        double ratio = 
            std::fabs(_jointStateTau(i))/_jointLimitTauAbs(i);
        container.insert(std::make_pair(
            "tau " + _model->getNameDOF(i+6), 
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
            Eigen::Vector6d wrench = _contactsPlane[i].stateWrench;
            double ratioCOPX = 0.0;
            double ratioCOPY = 0.0;
            double ratioFrictionX = 0.0;
            double ratioFrictionY = 0.0;
            if (std::fabs(wrench(5)) > 1e-3) {
                ratioCOPX = std::fabs(
                    wrench(1) /
                    (wrench(5)*_contactsPlane[i].limitCOP.x()));
                ratioCOPY = std::fabs(
                    wrench(0) /
                    (wrench(5)*_contactsPlane[i].limitCOP.y()));
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
            Eigen::Vector3d force = _contactsPoint[i].stateForce;
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

MultiContactIKStaticID::ContactBase_t& MultiContactIKStaticID::getContactBase(
    const std::string& frameName)
{
    if (_mappingContacts.count(frameName) == 0) {
        throw std::logic_error(
            "leph::MultiContactIKStaticID::getContactBase: "
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
MultiContactIKStaticID::ContactPlane_t& MultiContactIKStaticID::getContactPlane(
    const std::string& frameName)
{
    if (_mappingContacts.count(frameName) == 0) {
        throw std::logic_error(
            "leph::MultiContactIKStaticID::getContactPlane: "
            "Unknown contact frame name: "
            + frameName);
    }

    int index = _mappingContacts.at(frameName);
    if (index > 0) {
        return _contactsPlane.at(index-1);
    } else {
        throw std::logic_error(
            "leph::MultiContactIKStaticID::getContactPlane: "
            "Contact is not plane: "
            + frameName);
    }
}
MultiContactIKStaticID::ContactPoint_t& MultiContactIKStaticID::getContactPoint(
    const std::string& frameName)
{
    if (_mappingContacts.count(frameName) == 0) {
        throw std::logic_error(
            "leph::MultiContactIKStaticID::getContactPoint: "
            "Unknown contact frame name: "
            + frameName);
    }

    int index = _mappingContacts.at(frameName);
    if (index > 0) {
        throw std::logic_error(
            "leph::MultiContactIKStaticID::getContactPoint: "
            "Contact is not point: "
            + frameName);
    } else {
        return _contactsPoint.at(-index-1);
    }
}

void MultiContactIKStaticID::buildStaticIDInequalities(
    Eigen::MatrixXd& problemIneqMat,
    Eigen::VectorXd& problemIneqVec,
    double ratioLimit,
    bool isSwitch)
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
    //Contact switching case
    if (isSwitch) {
        if (_switchState > 0) {
            if (_switchIndexPlane >= 0) {
                sizeIDSol += 6;
                sizeIDIneq += 18;
            } else {
                sizeIDSol += 3;
                sizeIDIneq += 6;
            }
        } else if (_switchState < 0) {
            if (_switchIndexPlane >= 0) {
                sizeIDSol -= 6;
                sizeIDIneq -= 18;
            } else {
                sizeIDSol -= 3;
                sizeIDIneq -= 6;
            }
        }
    }

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
        problemIneqVec(offsetRow+0) = ratioLimit*_jointLimitTauAbs(i);
        problemIneqVec(offsetRow+1) = ratioLimit*_jointLimitTauAbs(i);
        offsetRow += 2;
        offsetCol += 1;
    }
    //Contact plane constraints
    for (size_t i=0;i<_contactsPlane.size();i++) {
        if (
            (!isSwitch && _contactsPlane[i].isEnabled) ||
            (isSwitch && isPlaneIndexInSwitch(i))
        ) {
            writeContactConstraints(
                problemIneqMat, problemIneqVec,
                true, i,
                offsetRow, offsetCol,
                ratioLimit);
            offsetRow += 18;
            offsetCol += 6;
        }
    }
    //Contact point constraints
    for (size_t i=0;i<_contactsPoint.size();i++) {
        if (
            (!isSwitch && _contactsPoint[i].isEnabled) ||
            (isSwitch && isPointIndexInSwitch(i))
        ) {
            writeContactConstraints(
                problemIneqMat, problemIneqVec,
                false, i,
                offsetRow, offsetCol,
                ratioLimit);
            offsetRow += 6;
            offsetCol += 3;
        }
    }
}

void MultiContactIKStaticID::writeContactConstraints(
    Eigen::MatrixXd& problemIneqMat,
    Eigen::VectorXd& problemIneqVec,
    bool isContactPlane,
    size_t indexContact,
    size_t offsetRow,
    size_t offsetCol,
    double ratioLimit)
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
            "leph::MultiContactIKStaticID::writeContactConstraints: "
            "Invalid input matrix or vector sizes.");
    }
    //Check contact container index
    if (
        (isContactPlane && indexContact >= _contactsPlane.size()) ||
        (!isContactPlane && indexContact >= _contactsPoint.size())
    ) {
        throw std::logic_error(
            "leph::MultiContactIKStaticID::writeContactConstraints: "
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
            ratioLimit*_contactsPlane[indexContact].frictionCoef;
        copMaxX = 
            ratioLimit*_contactsPlane[indexContact].limitCOP.x();
        copMaxY = 
            ratioLimit*_contactsPlane[indexContact].limitCOP.y();
    } else {
        normalForceMin = 
            _contactsPoint[indexContact].normalForceMin;
        normalForceMax = 
            _contactsPoint[indexContact].normalForceMax;
        frictionCoef = 
            ratioLimit*_contactsPoint[indexContact].frictionCoef;
    }
    double frictionCoefX = frictionCoef;
    double frictionCoefY = frictionCoef;

    //Retrieve current state (from last computed solution)
    double stateNormal = 0.0;
    double stateCoPX = 0.0;
    double stateCoPY = 0.0;
    double stateFrictionX = 0.0;
    double stateFrictionY = 0.0;
    if (isContactPlane) {
        const Eigen::Vector6d& stateWrench = 
            _contactsPlane[indexContact].stateWrench;
        stateNormal = stateWrench(5);
        if (std::fabs(stateWrench(5)) > 1e-4) {
            stateCoPX = std::fabs(stateWrench(1)/stateWrench(5));
            stateCoPY = std::fabs(stateWrench(0)/stateWrench(5));
            stateFrictionX = std::fabs(stateWrench(3)/stateWrench(5));
            stateFrictionY = std::fabs(stateWrench(4)/stateWrench(5));
        }
    } else {
        const Eigen::Vector3d& stateForce = 
            _contactsPoint[indexContact].stateForce;
        stateNormal = stateForce(2);
        if (std::fabs(stateForce(2)) > 1e-4) {
            stateFrictionX = std::fabs(stateForce(0)/stateForce(2));
            stateFrictionY = std::fabs(stateForce(1)/stateForce(2));
        }
    }
    //Retrieve measured state (from force-torque sensors)
    double readNormal = 0.0;
    double readCoPX = 0.0;
    double readCoPY = 0.0;
    double readFrictionX = 0.0;
    double readFrictionY = 0.0;
    if (isContactPlane) {
        const Eigen::Vector6d& readWrench = 
            _contactsPlane[indexContact].readWrench;
        readNormal = readWrench(5);
        if (std::fabs(readWrench(5)) > 1e-4) {
            readCoPX = std::fabs(readWrench(1)/readWrench(5));
            readCoPY = std::fabs(readWrench(0)/readWrench(5));
            readFrictionX = std::fabs(readWrench(3)/readWrench(5));
            readFrictionY = std::fabs(readWrench(4)/readWrench(5));
        }
    } else {
        const Eigen::Vector3d& readForce = 
            _contactsPoint[indexContact].readForce;
        readNormal = readForce(2);
        if (std::fabs(readForce(2)) > 1e-4) {
            readFrictionX = std::fabs(readForce(0)/readForce(2));
            readFrictionY = std::fabs(readForce(1)/readForce(2));
        }
    }
    
    /*
    //Compute (positive) limit margins clampted to adapt to current state
    double marginStateNormalMin = std::max(stateNormal-normalForceMin, 0.0);
    double marginStateNormalMax = std::max(normalForceMax-stateNormal, 0.0);
    double marginStateFrictionX = std::max(frictionCoefX-stateFrictionX, 0.0);
    double marginStateFrictionY = std::max(frictionCoefY-stateFrictionY, 0.0);
    double marginStateCoPX = std::max(copMaxX-stateCoPX, 0.0);
    double marginStateCoPY = std::max(copMaxY-stateCoPY, 0.0);
    
    //Compute (positive) limit margins clampted to adapt to measured state
    double marginReadNormalMin = std::max(readNormal-normalForceMin, 0.0);
    double marginReadNormalMax = std::max(normalForceMax-readNormal, 0.0);
    double marginReadFrictionX = std::max(frictionCoefX-readFrictionX, 0.0);
    double marginReadFrictionY = std::max(frictionCoefY-readFrictionY, 0.0);
    double marginReadCoPX = std::max(copMaxX-readCoPX, 0.0);
    double marginReadCoPY = std::max(copMaxY-readCoPY, 0.0);
    */

    //Loose limits to current one if adaptive option is set
    /*
    if (_isAdaptativeConstraints) {
        //TODO XXX
    }
    */
    //Assign used limits
    //TODO XXX
    /*
    marginReadNormalMin = 1e3;
    marginReadNormalMax = 1e3;
    marginReadFrictionX = 1e3;
    marginReadFrictionY = 1e3;
    marginReadCoPX = 1e3;
    marginReadCoPY = 1e3;
    */
    /*
    normalForceMin = stateNormal - std::min(marginStateNormalMin, marginReadNormalMax);
    normalForceMin = std::max(0.0, normalForceMin);
    normalForceMax = stateNormal + std::min(marginStateNormalMax, marginReadNormalMax);
    */
    /*
    copMaxX = stateCoPX + std::min(marginStateCoPX, marginReadCoPX);
    copMaxY = stateCoPY + std::min(marginStateCoPY, marginReadCoPY);
    frictionCoefX = stateFrictionX + std::min(marginStateFrictionX, marginReadFrictionX);
    frictionCoefY = stateFrictionY + std::min(marginStateFrictionY, marginReadFrictionY);
    frictionCoef = std::max(frictionCoefX, frictionCoefY);
    */

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

void MultiContactIKStaticID::writePoseTarget(
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

void MultiContactIKStaticID::computeDiffDynamics(
    const Eigen::VectorXd& statePosition,
    size_t indexDOF,
    double delta,
    Eigen::VectorXd& gravityVector,
    std::map<size_t, Eigen::MatrixXd>& jacobianPlaneTransposed,
    std::map<size_t, Eigen::MatrixXd>& jacobianPointTransposed)
{
    //Create finite difference degree 
    //of freedom position vector
    Eigen::VectorXd tmpVel1 = Eigen::VectorXd::Zero(_sizeDOF);
    Eigen::VectorXd tmpVel2 = Eigen::VectorXd::Zero(_sizeDOF);
    tmpVel1(indexDOF) += delta;
    tmpVel2(indexDOF) -= delta;
    Eigen::VectorXd tmpPos1 = IntegrateDOFVect(statePosition, tmpVel1);
    Eigen::VectorXd tmpPos2 = IntegrateDOFVect(statePosition, tmpVel2);
    //Sample first evaluation
    _model->setDOFPosVect(tmpPos1);
    _model->setDOFVelVect(Eigen::VectorXd::Zero(_sizeDOF));
    _model->updateState();
    Eigen::VectorXd tmpG1 = _model->computeGravityVector();
    std::map<size_t, Eigen::MatrixXd> tmpJacPlane1;
    std::map<size_t, Eigen::MatrixXd> tmpJacPoint1;
    for (size_t i=0;i<_contactsPlane.size();i++) {
        if (_contactsPlane[i].isEnabled || (int)i == _switchIndexPlane) {
            tmpJacPlane1.insert(std::make_pair(i,
                _model->pointJacobian(
                    _contactsPlane[i].frameId, 
                    _contactsPlane[i].frameId)
                .transpose()));
        }
    }
    for (size_t i=0;i<_contactsPoint.size();i++) {
        if (_contactsPoint[i].isEnabled || (int)i == _switchIndexPoint) {
            tmpJacPoint1.insert(std::make_pair(i, 
                (_model->pointJacobian(
                    _contactsPoint[i].frameId, 0)
                .block(3, 0, 3, _sizeDOF).transpose())
                *  _contactsPoint[i].contactMat));
        }
    }
    //Sample second evaluation
    _model->setDOFPosVect(tmpPos2);
    _model->setDOFVelVect(Eigen::VectorXd::Zero(_sizeDOF));
    _model->updateState();
    Eigen::VectorXd tmpG2 = _model->computeGravityVector();
    std::map<size_t, Eigen::MatrixXd> tmpJacPlane2;
    std::map<size_t, Eigen::MatrixXd> tmpJacPoint2;
    for (size_t i=0;i<_contactsPlane.size();i++) {
        if (_contactsPlane[i].isEnabled || (int)i == _switchIndexPlane) {
            tmpJacPlane2.insert(std::make_pair(i,
                _model->pointJacobian(
                    _contactsPlane[i].frameId, 
                    _contactsPlane[i].frameId)
                .transpose()));
        }
    }
    for (size_t i=0;i<_contactsPoint.size();i++) {
        if (_contactsPoint[i].isEnabled || (int)i == _switchIndexPoint) {
            tmpJacPoint2.insert(std::make_pair(i, 
                (_model->pointJacobian(
                    _contactsPoint[i].frameId, 0)
                .block(3, 0, 3, _sizeDOF).transpose())
                *  _contactsPoint[i].contactMat));
        }
    }
    //Compute finite difference derivatives
    gravityVector = 1.0/(2.0*delta)*(tmpG1 - tmpG2);
    for (const auto& it : tmpJacPlane1) {
        jacobianPlaneTransposed.insert(std::make_pair(it.first,
            1.0/(2.0*delta)*(tmpJacPlane1.at(it.first) - tmpJacPlane2.at(it.first))));
    }
    for (const auto& it : tmpJacPoint1) {
        jacobianPointTransposed.insert(std::make_pair(it.first,
            1.0/(2.0*delta)*(tmpJacPoint1.at(it.first) - tmpJacPoint2.at(it.first))));
    }
}
        
bool MultiContactIKStaticID::checkLimitsJointTorque(
    Eigen::VectorBlock<const Eigen::VectorXd, -1> torque,
    double ratioLimit,
    double* ratioMax,
    std::ostringstream* ss,
    bool printAll) const
{
    if ((size_t)torque.size() != _sizeJoint) {
        throw std::logic_error(
            "leph::MultiContactIKStaticID::checkLimitsJointTorque: "
            "Invalid vector size.");
    }

    double tmpRatioMax = 0.0;
    for (size_t i=0;i<_sizeJoint;i++) {
        double ratio = std::fabs(torque(i))/_jointLimitTauAbs(i);
        if (ss != nullptr && (ratio > ratioLimit || printAll)) {
            *ss 
                << "Joint torque: " << _model->getNameDOF(i+6) 
                << ": " << ratio << std::endl;
        }
        tmpRatioMax = std::max(tmpRatioMax, ratio);
    }

    if (ratioMax != nullptr) {
        *ratioMax = tmpRatioMax;
    }
    return (tmpRatioMax <= ratioLimit);
}
bool MultiContactIKStaticID::checkLimitsContactWrench(
    Eigen::VectorBlock<const Eigen::VectorXd, 6> wrench,
    size_t indexContactPlane,
    double ratioLimit,
    double* ratioMax,
    std::ostringstream* ss,
    bool printAll) const
{
    const ContactPlane_t& contact = _contactsPlane.at(indexContactPlane);
    //Normal force
    if (
        wrench(5) < ratioLimit*contact.normalForceMin || 
        wrench(5) > ratioLimit*contact.normalForceMax
    ) {
        if (ratioMax != nullptr) {
            *ratioMax = 0.0;
        }
        if (ss != nullptr) {
            *ss 
                << "Contact plane normal force: " 
                << contact.frameName << std::endl;
        }
        return false;
    }
    double tmpRatioMax = 0.0;
    
    //Friction pyramid
    double mu = contact.frictionCoef;
    double forceZ = std::fabs(wrench(5));
    double ratioFrictionX = std::fabs(wrench(3))/(forceZ*mu);
    double ratioFrictionY = std::fabs(wrench(4))/(forceZ*mu);
    tmpRatioMax = std::max(tmpRatioMax, ratioFrictionX);
    tmpRatioMax = std::max(tmpRatioMax, ratioFrictionY);
    if (ss != nullptr && (ratioFrictionX > ratioLimit || printAll)) {
        *ss 
            << "Contact plane friction pyramid X: " 
            << contact.frameName << ": " 
            << ratioFrictionX << std::endl;
    }
    if (ss != nullptr && (ratioFrictionY > ratioLimit || printAll)) {
        *ss 
            << "Contact plane friction pyramid Y: " 
            << contact.frameName << ": " 
            << ratioFrictionY << std::endl;
    }
    
    //Center of pressure
    double copx = contact.limitCOP.x();
    double copy = contact.limitCOP.y();
    double ratioCOPX = std::fabs(wrench(1))/(forceZ*copx);
    double ratioCOPY = std::fabs(wrench(0))/(forceZ*copy);
    tmpRatioMax = std::max(tmpRatioMax, ratioCOPX);
    tmpRatioMax = std::max(tmpRatioMax, ratioCOPY);
    if (ss != nullptr && (ratioCOPX > ratioLimit || printAll)) {
        *ss 
            << "Contact plane center of pressure X: " 
            << contact.frameName << ": " 
            << ratioCOPX << std::endl;
    }
    if (ss != nullptr && (ratioCOPY > ratioLimit || printAll)) {
        *ss 
            << "Contact plane center of pressure Y: " 
            << contact.frameName << ": " 
            << ratioCOPY << std::endl;
    }

    //Torsional torque
    mu *= ratioLimit;
    copx *= ratioLimit;
    copy *= ratioLimit;
    double tauMin = 
        - mu*(copx+copy)*forceZ
        + std::fabs(copy*wrench(3) - mu*wrench(0))
        + std::fabs(copx*wrench(4) - mu*wrench(1));
    double tauMax = 
        + mu*(copx+copy)*forceZ
        - std::fabs(copy*wrench(3) + mu*wrench(0))
        - std::fabs(copx*wrench(4) + mu*wrench(1));
    double tauMiddle = 0.5*tauMin + 0.5*tauMax;
    double tauRange = tauMax - tauMiddle;
    double ratioTau = ratioLimit*std::fabs(tauMiddle-wrench(2))/tauRange;
    tmpRatioMax = std::max(tmpRatioMax, ratioTau);
    if (ss != nullptr && (ratioTau > ratioLimit || printAll)) {
        *ss 
            << "Contact plane torsional torque: " << contact.frameName 
            << ": " << ratioTau 
            << " --- minTau=" << tauMin << " tauMax=" << tauMax << std::endl;
    }

    if (ratioMax != nullptr) {
        *ratioMax = tmpRatioMax;
    }
    return (tmpRatioMax <= ratioLimit);
}
bool MultiContactIKStaticID::checkLimitsContactForce(
    Eigen::VectorBlock<const Eigen::VectorXd, 3> force,
    size_t indexContactPoint,
    double ratioLimit,
    double* ratioMax,
    std::ostringstream* ss,
    bool printAll) const
{
    const ContactPoint_t& contact = _contactsPoint.at(indexContactPoint);
    //Normal force
    if (
        force(2) < ratioLimit*contact.normalForceMin || 
        force(2) > ratioLimit*contact.normalForceMax
    ) {
        if (ratioMax != nullptr) {
            *ratioMax = 0.0;
        }
        if (ss != nullptr) {
            *ss 
                << "Contact point normal force: " 
                << contact.frameName << std::endl;
        }
        return false;
    }

    //Friction pyramid
    const double& mu = contact.frictionCoef;
    double forceZ = std::fabs(force(2));
    double ratioFrictionX = std::fabs(force(0))/(forceZ*mu);
    double ratioFrictionY = std::fabs(force(1))/(forceZ*mu);
    if (ss != nullptr && (ratioFrictionX > ratioLimit || printAll)) {
        *ss 
            << "Contact point friction pyramid X: " 
            << contact.frameName << ": " 
            << ratioFrictionX << std::endl;
    }
    if (ss != nullptr && (ratioFrictionY > ratioLimit || printAll)) {
        *ss 
            << "Contact point friction pyramid Y: " 
            << contact.frameName << ": " 
            << ratioFrictionY << std::endl;
    }

    double tmpRatioMax = std::max(ratioFrictionX, ratioFrictionY);
    if (ratioMax != nullptr) {
        *ratioMax = tmpRatioMax;
    }
    return (tmpRatioMax <= ratioLimit);
}

bool MultiContactIKStaticID::isPlaneIndexInSwitch(size_t index) const
{
    if (_switchState == 1) {
        return 
            _contactsPlane[index].isEnabled || 
            ((int)index == _switchIndexPlane);
    } else if (_switchState == -1) {
        return 
            _contactsPlane[index].isEnabled && 
            ((int)index != _switchIndexPlane);
    } else {
        return _contactsPlane[index].isEnabled;
    }
}
bool MultiContactIKStaticID::isPointIndexInSwitch(size_t index) const
{
    if (_switchState == 1) {
        return 
            _contactsPoint[index].isEnabled || 
            ((int)index == _switchIndexPoint);
    } else if (_switchState == -1) {
        return 
            _contactsPoint[index].isEnabled && 
            ((int)index != _switchIndexPoint);
    } else {
        return _contactsPoint[index].isEnabled;
    }
}

}

