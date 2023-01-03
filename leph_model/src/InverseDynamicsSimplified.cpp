#include <leph_model/InverseDynamicsSimplified.hpp>
#include <leph_maths/AxisAngle.h>
#include <leph_eiquadprog/leph_eiquadprog.hpp>

namespace leph {

InverseDynamicsSimplified::InverseDynamicsSimplified(Model& model) :
    _model(&model),
    _numberContactPlane(0),
    _numberContactPoint(0),
    _numberTarget(0),
    _numberMomentum(0),
    _sizeDOF(0),
    _sizeJoint(0),
    _sizeSol(0),
    _mappingContact(),
    _mappingTargetJoint(),
    _mappingTargetPosition(),
    _mappingTargetOrientation(),
    _isCentroidalMomentumLinear(false),
    _isCentroidalMomentumAngular(false),
    _targetDiffMomentumLinear(0.0, 0.0, 0.0),
    _targetDiffMomentumAngular(0.0, 0.0, 0.0),
    _centroidalWeightLinear(0.0),
    _centroidalWeightAngular(0.0),
    _penaltyAccBase(0.0),
    _penaltyAccJoint(0.0),
    _isWeightsDirty(false),
    _contacts(),
    _targetsJoint(),
    _targetsPosition(),
    _targetsOrientation(),
    _cacheJacobiansWorld(),
    _cacheJacobiansBody(),
    _cacheJacDotQDot(),
    _cacheM(),
    _cacheC(),
    _cacheG(),
    _solToTauMat(),
    _solToTauVec(),
    _torqueBias(),
    _tmpTauIneqMat(),
    _tmpTauIneqVec(),
    _weights(),
    _costMat(),
    _costVec(),
    _problemCostMat(),
    _problemCostVec(),
    _problemEqMat(),
    _problemEqVec(),
    _problemIneqMat(),
    _problemIneqVec(),
    _problemSolution(),
    _zeroWrench(Eigen::VectorXd::Zero(6)),
    _zeroForce(Eigen::VectorXd::Zero(3)),
    _lastJointTorques(),
    _useNoVelocityForDynamics(false),
    _coefNonDiagonalMassMatrix(1.0),
    _disableInequalityConstraints(false)
{
    _lastJointTorques = Eigen::VectorXd::Zero(_model->sizeJoint());
}

void InverseDynamicsSimplified::addContactPlane(
    const std::string& nameFrame,
    double areaCoPMinX,
    double areaCoPMaxX,
    double areaCoPMinY,
    double areaCoPMaxY)
{
    if (_mappingContact.count(nameFrame) > 0) {
        throw std::logic_error(
            "leph::InverseDynamicsSimplified::addContactPlane: "
            "Contact already declared: " 
            + nameFrame);
    }
    if (_model->getMappingFrames().count(nameFrame) == 0) {
        throw std::logic_error(
            "leph::InverseDynamicsSimplified::addContactPlane: "
            "Invalid frame name: "
            + nameFrame);
    }
    _contacts.push_back({
        false,
        false,
        _model->getMappingFrames().at(nameFrame),
        1e6,
        areaCoPMinX, 
        areaCoPMaxX, 
        areaCoPMinY, 
        areaCoPMaxY,
        0.0,
        Eigen::Matrix3d::Identity(),
        Eigen::Vector6d::Zero(),
        1e-8*Eigen::Vector6d::Ones()});
    _mappingContact.insert(
        {nameFrame, _contacts.size()-1});
}

void InverseDynamicsSimplified::addContactPoint(
    const std::string& nameFrame)
{
    if (_mappingContact.count(nameFrame) > 0) {
        throw std::logic_error(
            "leph::InverseDynamicsSimplified::addContactPoint: "
            "Contact already declared: " 
            + nameFrame);
    }
    if (_model->getMappingFrames().count(nameFrame) == 0) {
        throw std::logic_error(
            "leph::InverseDynamicsSimplified::addContactPoint: "
            "Invalid frame name: "
            + nameFrame);
    }
    _contacts.push_back({
        false,
        true,
        _model->getMappingFrames().at(nameFrame),
        1e6,
        0.0, 0.0, 0.0, 0.0,
        0.0,
        Eigen::Matrix3d::Identity(),
        Eigen::Vector6d::Zero(),
        1e-8*Eigen::Vector6d::Ones()});
    _mappingContact.insert(
        {nameFrame, _contacts.size()-1});
}

void InverseDynamicsSimplified::addTargetJoint(
    const std::string& nameJoint)
{
    if (_mappingTargetJoint.count(nameJoint) > 0) {
        throw std::logic_error(
            "leph::InverseDynamicsSimplified::addTargetJoint: "
            "Joint target already declared: " 
            + nameJoint);
    }
    if (_model->getMappingDOFs().count(nameJoint) == 0) {
        throw std::logic_error(
            "leph::InverseDynamicsSimplified::addTargetJoint: "
            "Invalid joint name: "
            + nameJoint);
    }
    _targetsJoint.push_back({
        _model->getIndexDOF(nameJoint),
        0.0, 0.0});
    _mappingTargetJoint.insert(
        {nameJoint, _targetsJoint.size()-1});
}

void InverseDynamicsSimplified::addTargetPosition(
    const std::string& nameFrame)
{
    if (_mappingTargetPosition.count(nameFrame) > 0) {
        throw std::logic_error(
            "leph::InverseDynamicsSimplified::addTargetPosition: "
            "Target already declared: "
            + nameFrame);
    }
    if (_model->getMappingFrames().count(nameFrame) == 0) {
        throw std::logic_error(
            "leph::InverseDynamicsSimplified::addTargetPosition: "
            "Invalid frame name: "
            + nameFrame);
    }
    _targetsPosition.push_back({
        _model->getMappingFrames().at(nameFrame),
        Eigen::Vector3d::Zero(),
        0.0});
    _mappingTargetPosition.insert(
        {nameFrame, _targetsPosition.size()-1});
}
void InverseDynamicsSimplified::addTargetOrientation(
    const std::string& nameFrame)
{
    if (_mappingTargetOrientation.count(nameFrame) > 0) {
        throw std::logic_error(
            "leph::InverseDynamicsSimplified::addTargetOrientation: "
            "Target already declared: " 
            + nameFrame);
    }
    if (_model->getMappingFrames().count(nameFrame) == 0) {
        throw std::logic_error(
            "leph::InverseDynamicsSimplified::addTargetOrientation: "
            "Invalid frame name: "
            + nameFrame);
    }
    _targetsOrientation.push_back({
        _model->getMappingFrames().at(nameFrame), 
        Eigen::Vector3d::Zero(),
        0.0});
    _mappingTargetOrientation.insert(
        {nameFrame, _targetsOrientation.size()-1});
}
        
void InverseDynamicsSimplified::addTargetMomentumLinear()
{
    _isCentroidalMomentumLinear = true;
    _targetDiffMomentumLinear.setZero();
}
void InverseDynamicsSimplified::addTargetMomentumAngular()
{
    _isCentroidalMomentumAngular = true;
    _targetDiffMomentumAngular.setZero();
}

void InverseDynamicsSimplified::setRegularization(
    double penaltyAccBase,
    double penaltyAccJoint)
{
    if (
        penaltyAccBase < 0 ||
        penaltyAccJoint < 0
    ) {
        throw std::logic_error(
            "leph::InverseDynamicsSimplified::setRegularization: "
            "Invalid negative weight.");
    }
    _penaltyAccBase = penaltyAccBase;
    _penaltyAccJoint = penaltyAccJoint;
    _isWeightsDirty = true;
}

void InverseDynamicsSimplified::setFrictionCoef(
    const std::string& nameContact,
    double frictionCoef)
{
    if (_mappingContact.count(nameContact) == 0) {
        throw std::logic_error(
            "leph::InverseDynamicsSimplified::setFrictionCoef: "
            "Contact name not declared: " + nameContact);
    }
    if (frictionCoef < 0) {
        throw std::logic_error(
            "leph::InverseDynamicsSimplified::setFrictionCoef: "
            "Invalid negative coefficient.");
    }
    size_t index = _mappingContact.at(nameContact);
    _contacts[index].frictionCoef = frictionCoef;
}

void InverseDynamicsSimplified::setContactPointOrientation(
    const std::string& nameContact,
    const Eigen::Matrix3d& surfaceMat)
{
    if (_mappingContact.count(nameContact) == 0) {
        throw std::logic_error(
            "leph::InverseDynamicsSimplified::setContactPointOrientation: "
            "Contact name not declared: " + nameContact);
    }
    size_t index = _mappingContact.at(nameContact);
    _contacts[index].surfaceMat = surfaceMat;
}
const Eigen::Matrix3d& InverseDynamicsSimplified::getContactPointOrientation(
    const std::string& nameContact)
{
    if (_mappingContact.count(nameContact) == 0) {
        throw std::logic_error(
            "leph::InverseDynamicsSimplified::getContactPointOrientation: "
            "Contact name not declared: " + nameContact);
    }
    size_t index = _mappingContact.at(nameContact);
    return _contacts[index].surfaceMat;
}

void InverseDynamicsSimplified::setContactWrenchTarget(
    const std::string& nameContact,
    const Eigen::Vector6d& wrench)
{
    if (_mappingContact.count(nameContact) == 0) {
        throw std::logic_error(
            "leph::InverseDynamicsSimplified::setContactWrenchTarget: "
            "Contact name not declared: " + nameContact);
    }
    size_t index = _mappingContact.at(nameContact);
    _contacts[index].targetWrench = wrench;
}
void InverseDynamicsSimplified::setContactWrenchWeight(
    const std::string& nameContact,
    double weightTorque,
    double weightForceTangential,
    double weightForceNormal)
{
    if (
        weightTorque <= 0.0 ||
        weightForceTangential <= 0.0 ||
        weightForceNormal <= 0.0
    ) {
        throw std::logic_error(
            "leph::InverseDynamicsSimplified::setContactWrenchWeight: "
            "Invalid weights: " + nameContact);
    }
    if (_mappingContact.count(nameContact) == 0) {
        throw std::logic_error(
            "leph::InverseDynamicsSimplified::setContactWrenchWeight: "
            "Contact name not declared: " + nameContact);
    }
    size_t index = _mappingContact.at(nameContact);
    _contacts[index].weightWrench(0) = weightTorque;
    _contacts[index].weightWrench(1) = weightTorque;
    _contacts[index].weightWrench(2) = weightTorque;
    _contacts[index].weightWrench(3) = weightForceTangential;
    _contacts[index].weightWrench(4) = weightForceTangential;
    _contacts[index].weightWrench(5) = weightForceNormal;
    _isWeightsDirty = true;
}

void InverseDynamicsSimplified::setContactMinimumNormalForce(
    const std::string& nameContact,
    double forceMin)
{
    if (_mappingContact.count(nameContact) == 0) {
        throw std::logic_error(
            "leph::InverseDynamicsSimplified::setContactMinimumNormalForce: "
            "Contact name not declared: " + nameContact);
    }
    size_t index = _mappingContact.at(nameContact);
    _contacts[index].normalForceMin = forceMin;
}
double InverseDynamicsSimplified::getContactMinimumNormalForce(
    const std::string& nameContact)
{
    if (_mappingContact.count(nameContact) == 0) {
        throw std::logic_error(
            "leph::InverseDynamicsSimplified::getContactMinimumNormalForce: "
            "Contact name not declared: " + nameContact);
    }
    size_t index = _mappingContact.at(nameContact);
    return _contacts[index].normalForceMin;
}

void InverseDynamicsSimplified::setWeightJoint(
    const std::string& nameJoint,
    double weightAngular)
{
    if (_mappingTargetJoint.count(nameJoint) == 0) {
        throw std::logic_error(
            "leph::InverseDynamicsSimplified::setWeightJoint: "
            "Target name not declared: " + nameJoint);
    }
    if (weightAngular < 0) {
        throw std::logic_error(
            "leph::InverseDynamicsSimplified::setWeightJoint: "
            "Invalid negative weight.");
    }
    size_t index = _mappingTargetJoint.at(nameJoint);
    _targetsJoint[index].weight = weightAngular;
    _isWeightsDirty = true;
}
void InverseDynamicsSimplified::setWeightPosition(
    const std::string& nameTarget,
    double weightLinear)
{
    if (_mappingTargetPosition.count(nameTarget) == 0) {
        throw std::logic_error(
            "leph::InverseDynamicsSimplified::setWeightPosition: "
            "Target name not declared: " + nameTarget);
    }
    if (weightLinear < 0) {
        throw std::logic_error(
            "leph::InverseDynamicsSimplified::setWeightPosition: "
            "Invalid negative weight.");
    }
    size_t index = _mappingTargetPosition.at(nameTarget);
    _targetsPosition[index].weight = weightLinear;
    _isWeightsDirty = true;
}
void InverseDynamicsSimplified::setWeightOrientation(
    const std::string& nameTarget,
    double weightAngular)
{
    if (_mappingTargetOrientation.count(nameTarget) == 0) {
        throw std::logic_error(
            "leph::InverseDynamicsSimplified::setWeightOrientation: "
            "Target name not declared: " + nameTarget);
    }
    if (weightAngular < 0) {
        throw std::logic_error(
            "leph::InverseDynamicsSimplified::setWeightOrientation: "
            "Invalid negative weight.");
    }
    size_t index = _mappingTargetOrientation.at(nameTarget);
    _targetsOrientation[index].weight = weightAngular;
    _isWeightsDirty = true;
}
void InverseDynamicsSimplified::setWeightMomentumLinear(
    double weightMomentumLinear)
{
    if (!_isCentroidalMomentumLinear) {
        throw std::logic_error(
            "leph::InverseDynamicsSimplified::setWeightMomentumLinear: "
            "Centroidal linear momentum not declared.");
    }
    _centroidalWeightLinear = weightMomentumLinear;
    _isWeightsDirty = true;
}
void InverseDynamicsSimplified::setWeightMomentumAngular(
    double weightMomentumAngular)
{
    if (!_isCentroidalMomentumAngular) {
        throw std::logic_error(
            "leph::InverseDynamicsSimplified::setWeightMomentumAngular: "
            "Centroidal angular momentum not declared.");
    }
    _centroidalWeightAngular = weightMomentumAngular; 
    _isWeightsDirty = true;
}

void InverseDynamicsSimplified::init()
{
    //Reset caching
    _cacheJacobiansWorld.clear();
    _cacheJacobiansBody.clear();
    _cacheJacDotQDot.clear();
    //Cache matrices allocation
    for (size_t i=0;i<_contacts.size();i++) {
        initCache(_contacts[i].frameId, true, true);
    }
    for (size_t i=0;i<_targetsPosition.size();i++) {
        initCache(_targetsPosition[i].frameId, true, false);
    }
    for (size_t i=0;i<_targetsOrientation.size();i++) {
        initCache(_targetsOrientation[i].frameId, true, false);
    }

    //Problem matrices layout
    //Solutions: 
    //- (sizeDOF) DOF accelerations
    //- (6*numberContactPlane) Cartesian contact wrench in contact frame
    //- (3*numberContactPoint) Cartesian contact force in surface contact frame
    //NOTE: the contact wrenches and forces are actually ordered in 
    //solution vector using the same order as in the contact container.
    //Costs:
    //- (sizeSol) Regularization (optional joint acceleration target)
    //- (3*numberTarget) Target acceleration constraints
    //- (optional 3) Target centroidal angular momentum
    //- (optional 3) Target centroidal linear momentum
    //Equality constraints:
    //- (6) Upper part of the equation of motion with contacts
    //- (6*numberContactPlane) Contact plane acceleration constraints
    //- (3*numberContactPoint) Contact point acceleration constraints
    //Inequality constraints:
    //- (2*sizeJoint) Joint torques min and max bounds
    //- (numberContactPlane) Normal unilateral forces positive
    //- (numberContactPoint) Normal unilateral forces positive
    //- (4*numberContactPlane) Center of pressure position bounds
    //- (4*numberContactPlane) Linearized friction cone (pyramid)
    //- (4*numberContactPoint) Linearized friction cone (pyramid)
    //- (8*numberContactPlane) Stephane Caron contact yaw torque bounds
    
    //Compute problem sizes
    _numberContactPlane = 0;
    _numberContactPoint = 0;
    for (size_t i=0;i<_contacts.size();i++) {
        if (_contacts[i].isEnabled) {
            if (_contacts[i].isPoint) {
                _numberContactPoint++;
            } else {
                _numberContactPlane++;
            }
        }
    }
    _numberTarget = 
        _targetsPosition.size() + _targetsOrientation.size();
    _numberMomentum = 0;
    if (_isCentroidalMomentumLinear) {
        _numberMomentum++;
    }
    if (_isCentroidalMomentumAngular) {
        _numberMomentum++;
    }
    _sizeDOF = 
        _model->sizeVectVel();
    _sizeJoint = 
        _model->sizeJoint();
    _sizeSol = 
        _sizeDOF + 6*_numberContactPlane + 3*_numberContactPoint;
    size_t sizeCost = 
        _sizeSol + 
        3*_numberTarget + 3*_numberMomentum;
    size_t sizeEq = 
        6 + 6*_numberContactPlane + 3*_numberContactPoint;
    size_t sizeIneq = 
        2*_sizeJoint + _numberContactPlane + _numberContactPoint +
        4*_numberContactPlane + 4*_numberContactPoint +
        4*_numberContactPlane + 8*_numberContactPlane;

    //Problem matrices allocation
    _weights = Eigen::DiagonalMatrix<double, -1>(sizeCost);
    _weights.setZero();
    _costMat = Eigen::MatrixXd::Zero(sizeCost, _sizeSol);
    _costVec = Eigen::VectorXd::Zero(sizeCost);
    _problemCostMat = Eigen::MatrixXd::Zero(_sizeSol, _sizeSol);
    _problemCostVec = Eigen::VectorXd::Zero(_sizeSol);
    _problemEqMat = Eigen::MatrixXd::Zero(sizeEq, _sizeSol);
    _problemEqVec = Eigen::VectorXd::Zero(sizeEq);
    _problemIneqMat = Eigen::MatrixXd::Zero(sizeIneq, _sizeSol);
    _problemIneqVec = Eigen::VectorXd::Zero(sizeIneq);
    _problemSolution = Eigen::VectorXd::Zero(_sizeSol);
    _cacheM = Eigen::MatrixXd::Zero(_sizeDOF, _sizeDOF);
    _cacheC = Eigen::VectorXd::Zero(_sizeDOF);
    _cacheG = Eigen::VectorXd::Zero(_sizeDOF);
    _solToTauMat = Eigen::MatrixXd::Zero(_sizeDOF, _sizeSol);
    _solToTauVec = Eigen::VectorXd::Zero(_sizeDOF);
    _torqueBias = Eigen::VectorXd::Zero(_sizeDOF);
    _tmpTauIneqMat = Eigen::MatrixXd::Zero(2*_sizeJoint, _sizeJoint);
    _tmpTauIneqVec = Eigen::VectorXd::Zero(2*_sizeJoint);

    //Build weights diagonal matrix from 
    //internal structures
    assignWeights();
}

void InverseDynamicsSimplified::setContact(
    const std::string& nameContact,
    bool isEnabled)
{
    if (_mappingContact.count(nameContact) == 0) {
        throw std::logic_error(
            "leph::InverseDynamicsSimplified::setContact: "
            "Contact name not declared: " + nameContact);
    }
    size_t indexContact = _mappingContact.at(nameContact);
    //Rebuild the problem only if something has changed
    bool isChanged = 
        _contacts[indexContact].isEnabled != isEnabled;
    if (isChanged) {
        _contacts[indexContact].isEnabled = isEnabled;
        //Re build and init all the problems matrices
        //since the number of active contact has changed
        init();
    }
}

void InverseDynamicsSimplified::setEffortJoint(
    const std::string& nameJoint,
    double acc)
{
    if (_mappingTargetJoint.count(nameJoint) == 0) {
        throw std::logic_error(
            "leph::InverseDynamicsSimplified::setEffortJoint: "
            "Joint name not declared: " + nameJoint);
    }
    size_t index = _mappingTargetJoint.at(nameJoint);
    _targetsJoint[index].targetAcc = acc;
}

void InverseDynamicsSimplified::setEffortPosition(
    const std::string& nameTarget,
    const Eigen::Vector3d& acc)
{
    if (_mappingTargetPosition.count(nameTarget) == 0) {
        throw std::logic_error(
            "leph::InverseDynamicsSimplified::setEffortPosition: "
            "Target name not declared: " + nameTarget);
    }
    size_t index = _mappingTargetPosition.at(nameTarget);
    _targetsPosition[index].targetAcc = acc;
}
void InverseDynamicsSimplified::setEffortOrientation(
    const std::string& nameTarget,
    const Eigen::Vector3d& acc)
{
    if (_mappingTargetOrientation.count(nameTarget) == 0) {
        throw std::logic_error(
            "leph::InverseDynamicsSimplified::setEffortOrientation: "
            "Target name not declared: " + nameTarget);
    }
    size_t index = _mappingTargetOrientation.at(nameTarget);
    _targetsOrientation[index].targetAcc = acc;
}

void InverseDynamicsSimplified::setEffortMomentumLinear(
    const Eigen::Vector3d& momentumDiff)
{
    if (!_isCentroidalMomentumLinear) {
        throw std::logic_error(
            "leph::InverseDynamicsSimplified::setEffortMomentumLinear: "
            "Centroidal linear momentum not declared.");
    }

    _targetDiffMomentumLinear = momentumDiff;
}
void InverseDynamicsSimplified::setEffortMomentumAngular(
    const Eigen::Vector3d& momentumDiff)
{
    if (!_isCentroidalMomentumAngular) {
        throw std::logic_error(
            "leph::InverseDynamicsSimplified::setEffortMomentumAngular: "
            "Centroidal angular momentum not declared.");
    }
    
    _targetDiffMomentumAngular = momentumDiff;
}

double InverseDynamicsSimplified::computeEffortJoint(
    const std::string& nameJoint,
    double gainErrorPos,
    double targetPos,
    double gainErrorVel,
    double targetVel,
    double targetAcc)
{
    if (_mappingTargetJoint.count(nameJoint) == 0) {
        throw std::logic_error(
            "leph::InverseDynamicsSimplified::computeEffortJoint: "
            "Joint name not declared: " + nameJoint);
    }
    
    size_t indexInStruct = _mappingTargetJoint.at(nameJoint);
    double currentPos = _model->getDOFPosVect()(
        _targetsJoint[indexInStruct].index);
    double currentVel = _model->getDOFVelVect()(
        _targetsJoint[indexInStruct].index);
    _targetsJoint[indexInStruct].targetAcc = 
        gainErrorPos*(targetPos - currentPos) +
        gainErrorVel*(targetVel - currentVel) +
        targetAcc;

    return _targetsJoint[indexInStruct].targetAcc;
}

const Eigen::Vector3d& InverseDynamicsSimplified::computeEffortPosition(
    const std::string& nameTarget,
    double gainErrorPos,
    const Eigen::Vector3d& targetPos,
    double gainErrorVel,
    const Eigen::Vector3d& targetVel,
    const Eigen::Vector3d& targetAcc)
{
    if (_mappingTargetPosition.count(nameTarget) == 0) {
        throw std::logic_error(
            "leph::InverseDynamicsSimplified::computeEffortPosition: "
            "Target name not declared: " + nameTarget);
    }

    size_t index = _mappingTargetPosition.at(nameTarget);
    size_t frameId = _targetsPosition[index].frameId;
    Eigen::Vector3d currentPos = 
        _model->position(frameId, 0);
    Eigen::Vector3d currentVel = 
        _model->pointVelocity(frameId, 0).segment(3, 3);

    _targetsPosition[index].targetAcc = 
        gainErrorPos*(targetPos - currentPos) + 
        gainErrorVel*(targetVel - currentVel) +
        targetAcc;

    return _targetsPosition[index].targetAcc;
}
const Eigen::Vector3d& InverseDynamicsSimplified::computeEffortOrientation(
    const std::string& nameTarget,
    double gainErrorPos,
    const Eigen::Matrix3d& targetMat,
    double gainErrorVel,
    const Eigen::Vector3d& targetVel,
    const Eigen::Vector3d& targetAcc)
{
    if (_mappingTargetOrientation.count(nameTarget) == 0) {
        throw std::logic_error(
            "leph::InverseDynamicsSimplified::computeEffortOrientation: "
            "Target name not declared: " + nameTarget);
    }

    size_t index = _mappingTargetOrientation.at(nameTarget);
    size_t frameId = _targetsOrientation[index].frameId;
    Eigen::Matrix3d currentMat = 
        _model->orientation(frameId, 0);
    Eigen::Vector3d currentVel = 
        _model->pointVelocity(frameId, 0).segment(0, 3);

    _targetsOrientation[index].targetAcc = 
        gainErrorPos*MatrixToAxis(targetMat*currentMat.transpose()) +
        gainErrorVel*(targetVel - currentVel) +
        targetAcc;

    return _targetsOrientation[index].targetAcc;
}

const Eigen::Vector3d& InverseDynamicsSimplified::computeEffortMomentumLinear(
    double gainErrorPos,
    const Eigen::Vector3d& targetCoMPos,
    double gainErrorVel,
    const Eigen::Vector3d& targetCoMVel,
    const Eigen::Vector3d& targetCoMAcc)
{
    if (!_isCentroidalMomentumLinear) {
        throw std::logic_error(
            "leph::InverseDynamicsSimplified::computeEffortMomentumLinear: "
            "Centroidal linear momentum not declared.");
    }
    //Check that the internal model is up to date since 
    //the raw call to RBDL::CalcCenterOfMass update kinematics 
    //flags is set to false for performances
    if (_model->isDirty()) {
        throw std::logic_error(
            "leph::InverseDynamicsSimplified::computeEffortMomentumLinear:" 
            "Internal model is not updated.");
    }

    //Compute center of mass current position 
    //and velocity in world frame
    double mass;
    RBDLMath::Vector3d currentCoMPosInWorld;
    RBDLMath::Vector3d currentCoMVelInWorld;
    RBDL::Utils::CalcCenterOfMass(
        _model->getRBDLModel(), 
        _model->getDOFPosVect(), _model->getDOFVelVect(), nullptr, 
        mass, 
        currentCoMPosInWorld, &currentCoMVelInWorld, nullptr, 
        nullptr, nullptr, false);

    //Compute target linear momentum rate of 
    //change of CoM expressed in world frame
    _targetDiffMomentumLinear = mass*(
        gainErrorPos*(targetCoMPos - currentCoMPosInWorld) +
        gainErrorVel*(targetCoMVel - currentCoMVelInWorld) +
        targetCoMAcc);

    return _targetDiffMomentumLinear;
}
const Eigen::Vector3d& InverseDynamicsSimplified::computeEffortMomentumAngular(
    double gainErrorVel,
    const Eigen::Vector3d& targetMomentumAngular,
    const Eigen::Vector3d& targetMomentumAcc)
{
    if (!_isCentroidalMomentumAngular) {
        throw std::logic_error(
            "leph::InverseDynamicsSimplified::computeEffortMomentumAngular: "
            "Centroidal angular momentum not declared.");
    }
    //Check that the internal model is up to date since 
    //the raw call to RBDL::CalcCenterOfMass update kinematics 
    //flags is set to false for performances
    if (_model->isDirty()) {
        throw std::logic_error(
            "leph::InverseDynamicsSimplified::computeEffortMomentumAngular:" 
            "Internal model is not updated.");
    }

    double mass;
    RBDLMath::Vector3d currentCoMPosInWorld;
    RBDLMath::Vector3d currentCoMMomentumAngular;
    RBDL::Utils::CalcCenterOfMass(
        _model->getRBDLModel(), 
        _model->getDOFPosVect(), _model->getDOFVelVect(), nullptr, 
        mass, 
        currentCoMPosInWorld, nullptr, nullptr, 
        &currentCoMMomentumAngular, nullptr, false);
    
    //Compute target angular momentum rate of 
    //change around CoM expressed in world frame
    _targetDiffMomentumAngular =
        gainErrorVel*(targetMomentumAngular - currentCoMMomentumAngular) +
        targetMomentumAcc;
    
    return _targetDiffMomentumAngular;
}

void InverseDynamicsSimplified::setUseNoVelocityForDynamics(bool flag)
{
    _useNoVelocityForDynamics = flag;
}
void InverseDynamicsSimplified::setCoefNonDiagonalMassMatrix(double coef)
{
    _coefNonDiagonalMassMatrix = coef;
}
void InverseDynamicsSimplified::setDisableInequalityConstraints(bool flag)
{
    _disableInequalityConstraints = flag;
}

const Eigen::VectorXd& InverseDynamicsSimplified::refBiasTorques() const
{
    return _torqueBias;
}
Eigen::VectorXd& InverseDynamicsSimplified::refBiasTorques()
{
    return _torqueBias;
}
 
bool InverseDynamicsSimplified::run()
{
    //Rebuild the weights diagonal matrix if they
    //has been updated in internal structures
    if (_isWeightsDirty) {
        assignWeights();
    }

    //Compute needed Jacobian matrices
    //expressed in world frame
    for (auto& it : _cacheJacobiansWorld) {
        it.second = _model->pointJacobian(it.first, 0);
    }
    //Compute needed Jacobian matrices
    //expressed in body frame
    for (auto& it : _cacheJacobiansBody) {
        it.second = _model->pointJacobian(it.first, it.first);
    }
    //Compute needed generalized velocity 
    //matrices expressed in world frame
    for (auto& it : _cacheJacDotQDot) {
        if (_useNoVelocityForDynamics) {
            it.second.setZero();
        } else {
            it.second = _model->pointAcceleration(
                it.first, 0, Eigen::VectorXd::Zero(_sizeDOF));
        }
    }
    
    //Compute equation of motion inertia M 
    //matrix, generalized and gravity forces C, G vectors
    _cacheM.setZero();
    _cacheC.setZero();
    _cacheG.setZero();
    _model->computeEquationOfMotion(
        _cacheM, _cacheC, &_cacheG);
    for (int j=0;j<_cacheM.cols();j++) {
        for (int i=0;i<_cacheM.rows();i++) {
            if (i != j) {
                _cacheM(i,j) *= _coefNonDiagonalMassMatrix;
            }
        }
    }

    //If needed, compute centroidal momentum 
    //matrix and bias vector
    Eigen::MatrixXd CMM;
    Eigen::VectorXd CMMDotQDot;
    if (_isCentroidalMomentumLinear || _isCentroidalMomentumAngular) {
        if (_useNoVelocityForDynamics) {
            _model->computeCentroidalDynamics(
                _cacheM, _cacheG, _cacheG, CMM, CMMDotQDot);
        } else {
            _model->computeCentroidalDynamics(
                _cacheM, _cacheC, _cacheG, CMM, CMMDotQDot);
        }
    }

    //Build matrix and vector converting solution vector to 
    //degree of freedom torques with the equation of motion
    _solToTauMat.setZero();
    _solToTauVec.setZero();
    size_t offsetCol = _sizeDOF;
    _solToTauMat.block(0, 0, _sizeDOF, _sizeDOF) = _cacheM;
    for (size_t i=0;i<_contacts.size();i++) {
        if (_contacts[i].isEnabled && !_contacts[i].isPoint) {
            _solToTauMat.block(0, offsetCol, _sizeDOF, 6) = 
                -_cacheJacobiansBody.at(_contacts[i].frameId).transpose();
            offsetCol += 6;
        }
        if (_contacts[i].isEnabled && _contacts[i].isPoint) {
            _solToTauMat.block(0, offsetCol, _sizeDOF, 3) = 
                -_cacheJacobiansWorld.at(_contacts[i].frameId)
                    .block(3,0,3,_sizeDOF).transpose()
                *_contacts[i].surfaceMat;
            offsetCol += 3;
        }
    }
    if (_useNoVelocityForDynamics) {
        _solToTauVec = _cacheG + _torqueBias;
    } else {
        _solToTauVec = _cacheC + _torqueBias;
    }

    //Build cost matrices
    //Reset regularization
    _costMat.block(0, 0, _sizeDOF, _sizeDOF).setIdentity();
    _costVec.segment(0, _sizeDOF).setZero();
    //Direct joint control
    for (size_t i=0;i<_targetsJoint.size();i++) {
        _costVec(_targetsJoint[i].index) = _targetsJoint[i].targetAcc;
    }
    //Contact wrench target
    size_t offsetRow = _sizeDOF;
    offsetCol = _sizeDOF;
    for (size_t i=0;i<_contacts.size();i++) {
        if (_contacts[i].isEnabled && !_contacts[i].isPoint) {
            _costMat.block(offsetRow, offsetCol, 6, 6).setIdentity();
            _costVec.segment(offsetRow, 6) = 
                _contacts[i].targetWrench;
            offsetRow += 6;
            offsetCol += 6;
        }
        if (_contacts[i].isEnabled && _contacts[i].isPoint) {
            _costMat.block(offsetRow, offsetCol, 3, 3).setIdentity();
            _costVec.segment(offsetRow, 3) = 
                _contacts[i].targetWrench.segment(3, 3);
            offsetRow += 3;
            offsetCol += 3;
        }
    }
    //Acceleration target position constraints
    for (size_t i=0;i<_targetsPosition.size();i++) {
        size_t frameId = _targetsPosition[i].frameId;
        _costMat.block(offsetRow, 0, 3, _sizeDOF) = 
            _cacheJacobiansWorld.at(frameId).block(3, 0, 3, _sizeDOF);
        _costVec.segment(offsetRow, 3) = 
            _targetsPosition[i].targetAcc -
            _cacheJacDotQDot.at(frameId).segment(3, 3);
        offsetRow += 3;
    }
    //Acceleration target orientation constraints
    for (size_t i=0;i<_targetsOrientation.size();i++) {
        size_t frameId = _targetsOrientation[i].frameId;
        _costMat.block(offsetRow, 0, 3, _sizeDOF) = 
            _cacheJacobiansWorld.at(frameId).block(0, 0, 3, _sizeDOF);
        _costVec.segment(offsetRow, 3) = 
            _targetsOrientation[i].targetAcc -
            _cacheJacDotQDot.at(frameId).segment(0, 3);
        offsetRow += 3;
    }
    //Centroidal angular momentum rate of change target
    if (_isCentroidalMomentumAngular) {
        _costMat.block(offsetRow, 0, 3, _sizeDOF) =
            CMM.block(0, 0, 3, _sizeDOF);
        _costVec.segment(offsetRow, 3) =
            _targetDiffMomentumAngular - CMMDotQDot.segment(0, 3);
        offsetRow += 3;
    }
    //Centroidal linear momentum rate of change target
    if (_isCentroidalMomentumLinear) {
        _costMat.block(offsetRow, 0, 3, _sizeDOF) =
            CMM.block(3, 0, 3, _sizeDOF);
        _costVec.segment(offsetRow, 3) =
            _targetDiffMomentumLinear - CMMDotQDot.segment(3, 3);
        offsetRow += 3;
    }
    
    //Built equality constraints
    //Enforce equation of motion with contacts.
    //Since contact wrenches are assumed to be expressed 
    //in local contact frame, body Jacobian is used to 
    //convert them in joint-space torques
    offsetRow = 0;
    _problemEqMat.block(offsetRow, 0, 6, _sizeSol) = 
        _solToTauMat.block(0, 0, 6, _sizeSol);
    _problemEqVec.segment(offsetRow, 6) = _solToTauVec.segment(0, 6);
    offsetRow += 6;
    //Zero acceleration contact constraints
    for (size_t i=0;i<_contacts.size();i++) {
        if (_contacts[i].isEnabled && !_contacts[i].isPoint) {
            size_t frameId = _contacts[i].frameId;
            _problemEqMat.block(offsetRow, 0, 6, _sizeDOF) = 
                _cacheJacobiansWorld.at(frameId);
            _problemEqVec.segment(offsetRow, 6) = 
                -_cacheJacDotQDot.at(frameId);
            offsetRow += 6;
        }
        if (_contacts[i].isEnabled && _contacts[i].isPoint) {
            size_t frameId = _contacts[i].frameId;
            _problemEqMat.block(offsetRow, 0, 3, _sizeDOF) = 
                _cacheJacobiansWorld.at(frameId).block(3,0,3,_sizeDOF);
            _problemEqVec.segment(offsetRow, 3) = 
                -_cacheJacDotQDot.at(frameId).segment(3, 3);
            offsetRow += 3;
        }
    }
    
    //Build inequality constraints
    //Inequality torque bounds
    _tmpTauIneqMat.setZero();
    _tmpTauIneqVec.setZero();
    for (size_t i=0;i<_sizeJoint;i++) {
        _tmpTauIneqMat(2*i+0, i) = 1.0;
        _tmpTauIneqMat(2*i+1, i) = -1.0;
        //Initialize torque limits from URDF bounds
        _tmpTauIneqVec(2*i+0) = _model->jointLimitsTorque()(i);
        _tmpTauIneqVec(2*i+1) = _model->jointLimitsTorque()(i);
    }
    _problemIneqMat.block(0, 0, 2*_sizeJoint, _sizeSol) = 
        _tmpTauIneqMat*_solToTauMat.block(6, 0, _sizeJoint, _sizeSol);
    _problemIneqVec.segment(0, 2*_sizeJoint) = 
        _tmpTauIneqMat*_solToTauVec.segment(6, _sizeJoint) + _tmpTauIneqVec;
    offsetRow = 2*_sizeJoint;
    //Inequality unilateral positive constraints
    offsetCol = _sizeDOF;
    for (size_t i=0;i<_contacts.size();i++) {
        if (_contacts[i].isEnabled && !_contacts[i].isPoint) {
            _problemIneqMat(offsetRow, offsetCol+5) = 1.0;
            _problemIneqVec(offsetRow) = -_contacts[i].normalForceMin;
            offsetRow += 1;
            offsetCol += 6;
        }
        if (_contacts[i].isEnabled && _contacts[i].isPoint) {
            _problemIneqMat(offsetRow, offsetCol+2) = 1.0;
            _problemIneqVec(offsetRow) = -_contacts[i].normalForceMin;
            offsetRow += 1;
            offsetCol += 3;
        }
    }
    //Inequality center of pressure support area bounds
    //for plane contacts
    offsetCol = _sizeDOF;
    for (size_t i=0;i<_contacts.size();i++) {
        if (_contacts[i].isEnabled && !_contacts[i].isPoint) {
            _problemIneqMat(offsetRow+0, offsetCol+1) = -1.0;
            _problemIneqMat(offsetRow+0, offsetCol+5) = -_contacts[i].areaCoPMinX;
            _problemIneqMat(offsetRow+1, offsetCol+0) = 1.0;
            _problemIneqMat(offsetRow+1, offsetCol+5) = -_contacts[i].areaCoPMinY;
            _problemIneqMat(offsetRow+2, offsetCol+1) = 1.0;
            _problemIneqMat(offsetRow+2, offsetCol+5) = _contacts[i].areaCoPMaxX;
            _problemIneqMat(offsetRow+3, offsetCol+0) = -1.0;
            _problemIneqMat(offsetRow+3, offsetCol+5) = _contacts[i].areaCoPMaxY;
            offsetRow += 4;
            offsetCol += 6;
        }
        if (_contacts[i].isEnabled && _contacts[i].isPoint) {
            offsetCol += 3;
        }
    }
    //Inequality over tangential contact force with 
    //linearized friction cone (pyramid) on plane and point contact
    offsetCol = _sizeDOF;
    for (size_t i=0;i<_contacts.size();i++) {
        if (_contacts[i].isEnabled && !_contacts[i].isPoint) {
            double frictionCoef = _contacts[i].frictionCoef;
            _problemIneqMat(offsetRow+0, offsetCol+3) = 1.0;
            _problemIneqMat(offsetRow+0, offsetCol+5) = frictionCoef;
            _problemIneqMat(offsetRow+1, offsetCol+4) = 1.0;
            _problemIneqMat(offsetRow+1, offsetCol+5) = frictionCoef;
            _problemIneqMat(offsetRow+2, offsetCol+3) = -1.0;
            _problemIneqMat(offsetRow+2, offsetCol+5) = frictionCoef;
            _problemIneqMat(offsetRow+3, offsetCol+4) = -1.0;
            _problemIneqMat(offsetRow+3, offsetCol+5) = frictionCoef;
            offsetRow += 4;
            offsetCol += 6;
        }
        if (_contacts[i].isEnabled && _contacts[i].isPoint) {
            double frictionCoef = _contacts[i].frictionCoef;
            _problemIneqMat(offsetRow+0, offsetCol+0) = 1.0;
            _problemIneqMat(offsetRow+0, offsetCol+2) = frictionCoef;
            _problemIneqMat(offsetRow+1, offsetCol+1) = 1.0;
            _problemIneqMat(offsetRow+1, offsetCol+2) = frictionCoef;
            _problemIneqMat(offsetRow+2, offsetCol+0) = -1.0;
            _problemIneqMat(offsetRow+2, offsetCol+2) = frictionCoef;
            _problemIneqMat(offsetRow+3, offsetCol+1) = -1.0;
            _problemIneqMat(offsetRow+3, offsetCol+2) = frictionCoef;
            offsetRow += 4;
            offsetCol += 3;
        }
    }
    //Inequality over contact torque yaw no friction bounds.
    //See stéphane Caron paper: 
    //"Stability of Surface Contacts for Humanoid Robots:
    //Closed-Form Formulae of the Contact Wrench Cone
    //for Rectangular Support Areas"
    offsetCol = _sizeDOF;
    for (size_t i=0;i<_contacts.size();i++) {
        if (_contacts[i].isEnabled && !_contacts[i].isPoint) {
            //tau_min <= tau_z <= tau_max
            //tau_min = -mu.(X+Y).f_z + |Y.f_x - mu.tau_x| + |X.f_y - mu.tau_y|
            //tau_max = +mu.(X+Y).f_z - |Y.f_x + mu.tau_x| - |X.f_y + mu.tau_y|
            double mu = _contacts[i].frictionCoef;
            double X = std::min(
                -_contacts[i].areaCoPMinX, 
                _contacts[i].areaCoPMaxX);
            double Y = std::min(
                -_contacts[i].areaCoPMinY, 
                _contacts[i].areaCoPMaxY);
            //Minimum bound
            _problemIneqMat.block(offsetRow+0, offsetCol, 1, 6) 
                << mu, mu, 1.0, -Y, -X, mu*(X+Y);
            _problemIneqMat.block(offsetRow+1, offsetCol, 1, 6) 
                << -mu, mu, 1.0, Y, -X, mu*(X+Y);
            _problemIneqMat.block(offsetRow+2, offsetCol, 1, 6) 
                << -mu, -mu, 1.0, Y, X, mu*(X+Y);
            _problemIneqMat.block(offsetRow+3, offsetCol, 1, 6) 
                << mu, -mu, 1.0, -Y, X, mu*(X+Y);
            //Maximun bound
            _problemIneqMat.block(offsetRow+4, offsetCol, 1, 6) 
                << mu, mu, -1.0, Y, X, mu*(X+Y);
            _problemIneqMat.block(offsetRow+5, offsetCol, 1, 6) 
                << -mu, mu, -1.0, -Y, X, mu*(X+Y);
            _problemIneqMat.block(offsetRow+6, offsetCol, 1, 6) 
                << -mu, -mu, -1.0, -Y, -X, mu*(X+Y);
            _problemIneqMat.block(offsetRow+7, offsetCol, 1, 6) 
                << mu, -mu, -1.0, Y, -X, mu*(X+Y);
            offsetRow += 8;
            offsetCol += 6;
        }
        if (_contacts[i].isEnabled && _contacts[i].isPoint) {
            offsetCol += 3;
        }
    }

    //Build the weighted distance of linear target costs
    _problemCostMat = _costMat.transpose()*_weights*_costMat;
    _problemCostVec = -_costMat.transpose()*_weights*_costVec;
    
    //Solve the QP problem
    double cost = 0.0;
    if (_disableInequalityConstraints) {
        cost = Eigen::solve_quadprog(
            _problemCostMat,
            _problemCostVec,
            _problemEqMat.transpose(),
            _problemEqVec,
            Eigen::MatrixXd(_sizeSol, 0),
            Eigen::VectorXd(0),
            _problemSolution);
    } else {
        cost = Eigen::solve_quadprog(
            _problemCostMat,
            _problemCostVec,
            _problemEqMat.transpose(),
            _problemEqVec,
            _problemIneqMat.transpose(),
            _problemIneqVec,
            _problemSolution);
    }
    
    //Check QP solver result
    if (std::isnan(cost) || std::isinf(cost)) {
        _problemSolution.setZero();
        _lastJointTorques.setZero();
        return false;
    } else {
        _lastJointTorques = 
            (_solToTauMat*_problemSolution + _solToTauVec)
            .segment(6, _sizeJoint);
        return true;
    }
}

Eigen::VectorBlock<const Eigen::VectorXd, -1> 
    InverseDynamicsSimplified::accelerations() const
{
    return _problemSolution.segment(0, _sizeDOF);
}
Eigen::VectorBlock<const Eigen::VectorXd, -1> 
    InverseDynamicsSimplified::torques() const
{
    return _lastJointTorques.segment(0, _sizeJoint);
}
Eigen::VectorBlock<const Eigen::VectorXd, -1> 
    InverseDynamicsSimplified::contactWrench(const std::string& nameContact) const
{
    if (_mappingContact.count(nameContact) == 0) {
        throw std::logic_error(
            "leph::InverseDynamicsSimplified::contactWrench: "
            "Contact name not declared: " + nameContact);
    }
    size_t index = _mappingContact.at(nameContact);
    if (_contacts[index].isPoint) {
        throw std::logic_error(
            "leph::InverseDynamicsSimplified::contactWrench: "
            "Contact is not plane: " + nameContact);
    }
    if (!_contacts[index].isEnabled) {
        return _zeroWrench.segment(0, 6);
    }
    size_t offsetRow = _sizeDOF;
    for (size_t i=0;i<_contacts.size();i++) {
        if (_contacts[i].isEnabled && !_contacts[i].isPoint) {
            if (index == i) {
                break;
            }
            offsetRow += 6;
        }
        if (_contacts[i].isEnabled && _contacts[i].isPoint) {
            offsetRow += 3;
        }
    }
    return _problemSolution.segment(offsetRow, 6);
}
Eigen::VectorBlock<const Eigen::VectorXd, -1> 
    InverseDynamicsSimplified::contactForce(const std::string& nameContact) const
{
    if (_mappingContact.count(nameContact) == 0) {
        throw std::logic_error(
            "leph::InverseDynamicsSimplified::contactForce: "
            "Contact name not declared: " + nameContact);
    }
    size_t index = _mappingContact.at(nameContact);
    if (!_contacts[index].isPoint) {
        throw std::logic_error(
            "leph::InverseDynamicsSimplified::contactForce: "
            "Contact is not point: " + nameContact);
    }
    if (!_contacts[index].isEnabled) {
        return _zeroForce.segment(0, 3);
    }
    size_t offsetRow = _sizeDOF;
    for (size_t i=0;i<_contacts.size();i++) {
        if (_contacts[i].isEnabled && !_contacts[i].isPoint) {
            offsetRow += 6;
        }
        if (_contacts[i].isEnabled && _contacts[i].isPoint) {
            if (index == i) {
                break;
            }
            offsetRow += 3;
        }
    }
    return _problemSolution.segment(offsetRow, 3);
}

Eigen::VectorXd InverseDynamicsSimplified::torquesFeedbackPart() const
{
    return _cacheM * accelerations();
}
Eigen::VectorXd InverseDynamicsSimplified::torquesFeedforwardPart() const
{
    Eigen::VectorXd tau;
    if (_useNoVelocityForDynamics) {
        tau = _cacheG + _torqueBias;
    } else {
        tau = _cacheC + _torqueBias;
    }
    size_t offsetRow = _sizeDOF;
    for (size_t i=0;i<_contacts.size();i++) {
        if (_contacts[i].isEnabled && !_contacts[i].isPoint) {
            tau -= 
                _cacheJacobiansBody.at(_contacts[i].frameId).transpose()
                *_problemSolution.segment(offsetRow, 6);
            offsetRow += 6;
        }
        if (_contacts[i].isEnabled && _contacts[i].isPoint) {
            tau -= 
                _cacheJacobiansWorld.at(_contacts[i].frameId)
                    .block(3,0,3,_sizeDOF).transpose()
                *_contacts[i].surfaceMat
                *_problemSolution.segment(offsetRow, 3);
            offsetRow += 3;
        }
    }
    return tau;
}
        
void InverseDynamicsSimplified::initCache(
    size_t frameId, bool initJacWorld, bool initJacBody)
{
    if (initJacWorld) {
        _cacheJacobiansWorld[frameId] = 
            Eigen::MatrixXd::Zero(6, _model->sizeVectVel());
        _cacheJacDotQDot[frameId] = 
            Eigen::Vector6d::Zero();
    }
    if (initJacBody) {
        _cacheJacobiansBody[frameId] = 
            Eigen::MatrixXd::Zero(6, _model->sizeVectVel());
    }
}
        
void InverseDynamicsSimplified::assignWeights()
{
    //Reset all weights to 1
    _weights.setIdentity();
    _isWeightsDirty = false;

    size_t offsetRow = 0;
    //Variables regularization
    _weights.diagonal().segment(offsetRow, 6) *= _penaltyAccBase;
    offsetRow += 6;
    _weights.diagonal().segment(offsetRow, _sizeJoint) *= _penaltyAccJoint;
    offsetRow += _sizeJoint;
    for (size_t i=0;i<_contacts.size();i++) {
        if (_contacts[i].isEnabled && !_contacts[i].isPoint) {
            _weights.diagonal().segment(offsetRow, 6) = 
                _contacts[i].weightWrench;
            offsetRow += 6;
        }
        if (_contacts[i].isEnabled && _contacts[i].isPoint) {
            _weights.diagonal().segment(offsetRow, 3) = 
                _contacts[i].weightWrench.segment(3, 3);
            offsetRow += 3;
        }
    }
    //Direct joint acceleration target 
    //overloads regularization weights
    for (size_t i=0;i<_targetsJoint.size();i++) {
        _weights.diagonal()(_targetsJoint[i].index) = _targetsJoint[i].weight;
    }
    //Position acceleration target
    for (size_t i=0;i<_targetsPosition.size();i++) {
        _weights.diagonal().segment(offsetRow, 3) *= _targetsPosition[i].weight;
        offsetRow += 3;
    }
    //Orientation acceleration target
    for (size_t i=0;i<_targetsOrientation.size();i++) {
        _weights.diagonal().segment(offsetRow, 3) *= _targetsOrientation[i].weight;
        offsetRow += 3;
    }
    //Centroidal momentum target
    if (_isCentroidalMomentumAngular) {
        _weights.diagonal().segment(offsetRow, 3) *= _centroidalWeightAngular;
        offsetRow += 3;
    }
    if (_isCentroidalMomentumLinear) {
        _weights.diagonal().segment(offsetRow, 3) *= _centroidalWeightLinear;
        offsetRow += 3;
    }
}

}

