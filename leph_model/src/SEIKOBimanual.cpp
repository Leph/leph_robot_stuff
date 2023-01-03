#include <stdexcept>
#include <leph_model/SEIKOBimanual.hpp>
#include <leph_eiquadprog/leph_eiquadprog.hpp>
#include <leph_maths/AxisAngle.h>
#include <leph_maths/Clamp.h>
#include <leph_maths/IntegrateDOFVect.h>

namespace leph {

SEIKOBimanual::SEIKOBimanual(Model& model) :
    _model(&model),
    _sizeDOF(0),
    _sizeJoint(0),
    _jointLimitPosLower(),
    _jointLimitPosUpper(),
    _jointLimitVelAbs(),
    _jointLimitTauAbs(),
    _jointTargetPos(),
    _jointWeightPos(),
    _jointWeightVel(),
    _jointClampPos(),
    _limitNormalForceMin(0.0),
    _limitNormalForceMax(1e5),
    _gravityVector(),
    _jointComputedDeltaTau(),
    _jointComputedDeltaDOF(),
    _jointStateTau(),
    _ratioLimitsDynamics(1.0),
    _isDisabledConstraints(false),
    _isDisabledJointTorqueConstraints(false),
    _isAdaptativeConstraints(false),
    _pinocchio(model),
    _nameFrameHandLeft(""),
    _nameFrameHandRight(""),
    _targetPosLeftInWorld(Eigen::Vector3d::Zero()),
    _targetPosRightInWorld(Eigen::Vector3d::Zero()),
    _targetMatLeftInWorld(Eigen::Matrix3d::Identity()),
    _targetMatRightInWorld(Eigen::Matrix3d::Identity()),
    _targetPosObjectInWorld(Eigen::Vector3d::Zero()),
    _targetMatObjectInWorld(Eigen::Matrix3d::Identity()),
    _targetWrenchLeft(Eigen::Vector6d::Zero()),
    _targetWrenchRight(Eigen::Vector6d::Zero()),
    _clampPosCart(0.01),
    _clampMatCart(0.01),
    _weightPosCart(1000.0),
    _weightMatCart(100.0),
    _weightJointTorque(1e-4),
    _weightDeltaWrenches(10.0),
    _weightWrenchHands(Eigen::Vector6d::Ones()),
    _isBimanualEnabled(false),
    _posRightHandInLeft(Eigen::Vector3d::Zero()),
    _matRightHandInLeft(Eigen::Matrix3d::Identity()),
    _posCoMObjectInLeft(Eigen::Vector3d::Zero()),
    _wrenchObjectInWorld(Eigen::Vector6d::Zero()),
    _limitFrictionObject(0.4),
    _limitCoPObject(0.05),
    _deltaWrenchHandLeft(Eigen::Vector6d::Zero()),
    _deltaWrenchHandRight(Eigen::Vector6d::Zero()),
    _stateWrenchHandLeft(Eigen::Vector6d::Zero()),
    _stateWrenchHandRight(Eigen::Vector6d::Zero()),
    _biasConstraintEquilibrium(Eigen::Vector6d::Zero()),
    _biasConstraintKinematics(Eigen::Vector6d::Zero()),
    _isOverrideGravityVector(false)
{
    //Joint default initialization
    _sizeDOF = _model->sizeDOF();
    _sizeJoint = _model->sizeJoint();
    //Limits initialization
    _jointLimitPosLower = _model->jointLimitsLower();
    _jointLimitPosUpper = _model->jointLimitsUpper();
    _jointLimitVelAbs = 1e4*Eigen::VectorXd::Ones(_sizeJoint);
    _jointLimitTauAbs = _model->jointLimitsTorque();
    //Targets and weights default initialization
    _jointTargetPos = _model->getJointPosVect();
    _jointWeightPos = 2.5*Eigen::VectorXd::Ones(_sizeJoint);
    _jointWeightVel = 10000.0*Eigen::VectorXd::Ones(_sizeJoint);
    _jointClampPos = 0.1;
    _gravityVector = Eigen::VectorXd::Zero(_sizeDOF);
    _jointComputedDeltaTau = Eigen::VectorXd::Zero(_sizeJoint);
    _jointComputedDeltaDOF = Eigen::VectorXd::Zero(_sizeDOF);
    _jointStateTau = Eigen::VectorXd::Zero(_sizeJoint);
}
        
void SEIKOBimanual::setHandFrameNames(
    const std::string& nameFrameHandLeft,
    const std::string& nameFrameHandRight)
{
    _nameFrameHandLeft = nameFrameHandLeft;
    _nameFrameHandRight = nameFrameHandRight;
}
        
void SEIKOBimanual::setBimanualMode(bool isEnabled)
{
    _isBimanualEnabled = isEnabled;
}
        
bool SEIKOBimanual::getBimanualMode() const
{
    return _isBimanualEnabled;
}

void SEIKOBimanual::setBimanualObjectTransforms(
    const Eigen::Vector3d& posRightHandInLeft,
    const Eigen::Matrix3d& matRightHandInLeft,
    const Eigen::Vector3d& posCoMObjectInLeft)
{
    _posRightHandInLeft = posRightHandInLeft;
    _matRightHandInLeft = matRightHandInLeft;
    _posCoMObjectInLeft = posCoMObjectInLeft;
}

const Eigen::Vector3d& SEIKOBimanual::getBimanualPosRightInLeft() const
{
    return _posRightHandInLeft;
}
const Eigen::Matrix3d& SEIKOBimanual::getBimanualMatRightInLeft() const
{
    return _matRightHandInLeft;
}
const Eigen::Vector3d& SEIKOBimanual::getBimanualPosCoMInLeft() const
{
    return _posCoMObjectInLeft;
}
 
void SEIKOBimanual::setBimanualObjectWrench(
    const Eigen::Vector6d& wrenchObjectInWorld)
{
    _wrenchObjectInWorld = wrenchObjectInWorld;
}

Eigen::Vector3d& SEIKOBimanual::refTargetPosLeftHand()
{
    return _targetPosLeftInWorld;
}
Eigen::Vector3d& SEIKOBimanual::refTargetPosRightHand()
{
    return _targetPosRightInWorld;
}
Eigen::Matrix3d& SEIKOBimanual::refTargetMatLeftHand()
{
    return _targetMatLeftInWorld;
}
Eigen::Matrix3d& SEIKOBimanual::refTargetMatRightHand()
{
    return _targetMatRightInWorld;
}
Eigen::Vector3d& SEIKOBimanual::refTargetPosObject()
{
    return _targetPosObjectInWorld;
}
Eigen::Matrix3d& SEIKOBimanual::refTargetMatObject()
{
    return _targetMatObjectInWorld;
}

Eigen::Vector6d& SEIKOBimanual::refTargetWrenchLeft()
{
    return _targetWrenchLeft;
}
Eigen::Vector6d& SEIKOBimanual::refTargetWrenchRight()
{
    return _targetWrenchRight;
}
        
void SEIKOBimanual::setCartesianClamps(double clampPos, double clampMat)
{
    _clampPosCart = clampPos;
    _clampMatCart = clampMat;
}
void SEIKOBimanual::setCartesianPoseWeights(
    double weightPos, double weightMat)
{
    _weightPosCart = weightPos;
    _weightMatCart = weightMat;
}

void SEIKOBimanual::setTorqueWrenchWeights(
    double weightJointTorque,
    const Eigen::Vector6d& weightWrench)
{
    _weightJointTorque = weightJointTorque;
    _weightWrenchHands = weightWrench;
}

void SEIKOBimanual::setDeltaWeights(
    double weightVelocity, double weightDeltaWrench)
{
    _jointWeightVel = weightVelocity*Eigen::VectorXd::Ones(_sizeJoint);
    _weightDeltaWrenches = weightDeltaWrench;
}

void SEIKOBimanual::setBimanualLimits(
    double frictionCoef, double copRange,
    double normalForceMin, double normalForceMax)
{
    _limitFrictionObject = frictionCoef;
    _limitCoPObject = copRange;
    _limitNormalForceMin = normalForceMin;
    _limitNormalForceMax = normalForceMax;
}

void SEIKOBimanual::setJointLimitPos(
    const std::string& name,
    double limitLower, double limitUpper)
{
    size_t index = _model->getIndexDOF(name);
    _jointLimitPosLower(index-6) = limitLower;
    _jointLimitPosUpper(index-6) = limitUpper;
}
void SEIKOBimanual::setJointLimitVel(
    double limitVelAbs)
{
    _jointLimitVelAbs = 
        limitVelAbs*Eigen::VectorXd::Ones(_model->sizeJoint());
}
void SEIKOBimanual::setJointLimitVel(
    const std::string& name,
    double limitVelAbs)
{
    size_t index = _model->getIndexDOF(name);
    _jointLimitVelAbs(index-6) = limitVelAbs;
}
void SEIKOBimanual::setJointLimitTau(
    const std::string& name,
    double limitTauAbs)
{
    size_t index = _model->getIndexDOF(name);
    _jointLimitTauAbs(index-6) = limitTauAbs;
}

void SEIKOBimanual::setJointPosTarget(
    const Eigen::VectorXd& pos)
{
    if ((size_t)pos.size() != _sizeJoint) {
        throw std::logic_error(
            "leph::SEIKOBimanual::setJointTargetPos: "
            "Invalid vector size.");
    }
    _jointTargetPos = pos;
}

void SEIKOBimanual::setJointPosWeight(double weightPos)
{
    _jointWeightPos = weightPos*Eigen::VectorXd::Ones(_sizeJoint);
}
void SEIKOBimanual::setJointPosWeight(
    const std::string& name, double weightPos)
{
    _jointWeightPos(_model->getIndexDOF(name)-6) = weightPos;
}

void SEIKOBimanual::setJointPosClamp(double clamp)
{
    _jointClampPos = clamp;
}

void SEIKOBimanual::setRatioLimits(double ratio)
{
    _ratioLimitsDynamics = ratio;
}

void SEIKOBimanual::setDisableConstraints(bool isDisabled)
{
    _isDisabledConstraints = isDisabled;
}
void SEIKOBimanual::setDisableJointTorqueConstraints(bool isDisabled)
{
    _isDisabledJointTorqueConstraints = isDisabled;
}

void SEIKOBimanual::setAdaptativeConstraints(bool isDisabled)
{
    _isAdaptativeConstraints = isDisabled;
}

void SEIKOBimanual::resetTargetsFromModel()
{
    _jointTargetPos = _model->getJointPosVect();
    _targetPosObjectInWorld = 
        _model->position(_nameFrameHandLeft, "ROOT", _posCoMObjectInLeft);
    _targetMatObjectInWorld = 
        _model->orientation(_nameFrameHandLeft, "ROOT");
    _targetPosLeftInWorld = 
        _model->position(_nameFrameHandLeft, "ROOT");
    _targetPosRightInWorld = 
        _model->position(_nameFrameHandRight, "ROOT");
    _targetMatLeftInWorld = 
        _model->orientation(_nameFrameHandLeft, "ROOT");
    _targetMatRightInWorld = 
        _model->orientation(_nameFrameHandRight, "ROOT");
}
        
void SEIKOBimanual::setOverrideGravityVector(const Eigen::VectorXd& vectG)
{
    if (vectG.size() != _sizeJoint) {
        throw std::logic_error(
            "leph::SEIKOBimanual::setOverrideGravityVector: "
            "Invalid vector size.");
    }
    _gravityVector.segment(6, _sizeJoint) = vectG;
    _isOverrideGravityVector = true;
}

bool SEIKOBimanual::run(double dt)
{
    //Define problem sizes
    size_t sizeSolID = 
        //Delta hand contact wrenches
        6*2;
    size_t sizeSol = 
        //Delta position for each joint
        _sizeJoint +
        //Delta static wrenches of hands
        sizeSolID;
    size_t sizeCost = 
        //Velocity regularization
        _sizeJoint +
        //Delta wrenches regularization
        6*2 +
        //Joint position target
        _sizeJoint +
        //Left and right hand pose target
        6*2 +
        //Joint torques minimization
        _sizeJoint +
        //Hands contact wrenches minimization
        6*2;
    size_t sizeEq = 0;
    if (_isBimanualEnabled) {
        sizeEq =
        //Object static equilibrium equation
        //over hands contact wrenches
        6 +
        //Hands position kinematic closure
        6;
    } else {
        //Enforce null contact force in non bimanual mode
        //when no wrench target is set
        if (_targetWrenchLeft.squaredNorm() < 1e-3) {
            sizeEq += 6;
        }
        if (_targetWrenchRight.squaredNorm() < 1e-3) {
            sizeEq += 6;
        }
    }
    size_t sizeIDIneq = 
        //Joint torques bounds
        2*_sizeJoint + 
        //Contact plane constraints
        18*2;
    size_t sizeIneq =
        //Joint position bounds
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

    //Compute joint gravity vector
    if (!_isOverrideGravityVector) {
        _gravityVector = _pinocchio.gravityVector();
    }

    //Store model position and velocity state
    Eigen::VectorXd statePosition = _model->getDOFPosVect();
    Eigen::VectorXd stateVelocity = _model->getDOFVelVect();
    
    //Restore model position and velocity state
    _model->setDOFPosVect(statePosition);
    _model->setDOFVelVect(stateVelocity);
    _model->updateState();

    //Build current ID state vector
    Eigen::VectorXd currentIDVect = 
        Eigen::VectorXd::Zero(_sizeJoint + sizeSolID);
    size_t offsetSolID = 0;
    currentIDVect.segment(offsetSolID, _sizeJoint) = _jointStateTau;
    offsetSolID += _sizeJoint;
    currentIDVect.segment(offsetSolID, 6) = _stateWrenchHandLeft;
    offsetSolID += 6;
    currentIDVect.segment(offsetSolID, 6) = _stateWrenchHandRight;
    offsetSolID += 6;

    //Build the linear relationship from reduced solution 
    //(delta position and delta contact wrenches/forces)
    //to computed joint torques (after integration)
    //using the differentiated static equation of motion.
    //solToTauMat*deltaSol + solToTauVec = dtau+tau
    Eigen::MatrixXd solToTauMat = Eigen::MatrixXd::Zero(_sizeJoint, sizeSol);
    Eigen::VectorXd solToTauVec = Eigen::VectorXd::Zero(_sizeJoint);
    size_t offsetTmpCol = _sizeJoint;
    //Left hand
    solToTauMat.block(0, 0, _sizeJoint, _sizeJoint) -= 
        _pinocchio.diffHessianWrenchInLocalProduct(
            _nameFrameHandLeft,
            _stateWrenchHandLeft).block(6, 6, _sizeJoint, _sizeJoint);
    solToTauMat.block(0, offsetTmpCol, _sizeJoint, 6) = 
        -_model->pointJacobian(_nameFrameHandLeft, _nameFrameHandLeft)
        .block(0, 6, 6, _sizeJoint).transpose();
    solToTauVec -= 
        _model->pointJacobian(_nameFrameHandLeft, _nameFrameHandLeft)
        .block(0, 6, 6, _sizeJoint).transpose()
        * _stateWrenchHandLeft;
    offsetTmpCol += 6;
    //Right hand
    solToTauMat.block(0, 0, _sizeJoint, _sizeJoint) -= 
        _pinocchio.diffHessianWrenchInLocalProduct(
            _nameFrameHandRight,
            _stateWrenchHandRight).block(6, 6, _sizeJoint, _sizeJoint);
    solToTauMat.block(0, offsetTmpCol, _sizeJoint, 6) = 
        -_model->pointJacobian(_nameFrameHandRight, _nameFrameHandRight)
        .block(0, 6, 6, _sizeJoint).transpose();
    solToTauVec -= 
        _model->pointJacobian(_nameFrameHandRight, _nameFrameHandRight)
        .block(0, 6, 6, _sizeJoint).transpose()
        * _stateWrenchHandRight;
    offsetTmpCol += 6;
    solToTauMat.block(0, 0, _sizeJoint, _sizeJoint) += 
        _pinocchio.diffGravityVector().block(6, 6, _sizeJoint, _sizeJoint);
    solToTauVec += _gravityVector.segment(6, _sizeJoint);

    //Assign back to left and right target pose the manipulated 
    //object target pose when bimanual mode is enabed
    if (_isBimanualEnabled) {
        Eigen::Vector3d posCoMObjectInRight = 
            _matRightHandInLeft.transpose() *
            (_posCoMObjectInLeft - _posRightHandInLeft);
        _targetPosLeftInWorld = 
            _targetPosObjectInWorld - 
            _model->orientation(_nameFrameHandLeft, "ROOT")*_posCoMObjectInLeft;
        _targetMatLeftInWorld = 
            _targetMatObjectInWorld;
        _targetPosRightInWorld = 
            _targetPosObjectInWorld - 
            _model->orientation(_nameFrameHandRight, "ROOT")*posCoMObjectInRight;
        _targetMatRightInWorld = 
            _targetMatObjectInWorld*_matRightHandInLeft;
    }

    //Build cost
    size_t offsetCostRow = 0;
    //Velocity regularization
    costMat.block(offsetCostRow, 0, _sizeJoint, _sizeJoint) = 
        Eigen::MatrixXd::Identity(_sizeJoint, _sizeJoint);
    costVec.segment(offsetCostRow, _sizeJoint) = 
        Eigen::VectorXd::Zero(_sizeJoint);
    weights.diagonal().segment(offsetCostRow, _sizeJoint) = 
        dt*_jointWeightVel;
    offsetCostRow += _sizeJoint;
    //Delta hand wrenches regularization
    costMat.block(offsetCostRow, _sizeJoint, 12, 12) = 
        Eigen::MatrixXd::Identity(12, 12);
    costVec.segment(offsetCostRow, 12) = 
        Eigen::VectorXd::Zero(12);
    weights.diagonal().segment(offsetCostRow, 12) = 
        _weightDeltaWrenches*Eigen::VectorXd::Ones(12);
    offsetCostRow += 12;
    //Joint position target
    costMat.block(offsetCostRow, 0, _sizeJoint, _sizeJoint) = 
        Eigen::MatrixXd::Identity(_sizeJoint, _sizeJoint);
    costVec.segment(offsetCostRow, _sizeJoint) = ClampVectorAllComponent(
        _jointTargetPos - _model->getJointPosVect(), _jointClampPos);
    weights.diagonal().segment(offsetCostRow, _sizeJoint) = 
        _jointWeightPos;
    offsetCostRow += _sizeJoint;
    //Left hand pose target
    {
        //Compute clamped Cartesian orientation error vector
        Eigen::Vector3d deltaMat = ClampVectorNorm(
            MatrixToAxis(
                _targetMatLeftInWorld
                * _model->orientation(_nameFrameHandLeft, "ROOT").transpose()), 
            _clampMatCart);
        //Compute clamped Cartesian position 
        Eigen::Vector3d deltaPos = ClampVectorNorm(
            _targetPosLeftInWorld -
            _model->position(_nameFrameHandLeft, "ROOT"), 
            _clampPosCart);
        //Assign matrix, vector and weight
        costMat.block(offsetCostRow, 0, 6, _sizeJoint) =
            _model->pointJacobian(_nameFrameHandLeft, "ROOT")
            .block(0,6,6,_sizeJoint);
        costVec.segment(offsetCostRow+0, 3) = deltaMat;
        costVec.segment(offsetCostRow+3, 3) = deltaPos;
        weights.diagonal().segment(offsetCostRow+0, 3) = 
            _weightMatCart*Eigen::VectorXd::Ones(3);
        weights.diagonal().segment(offsetCostRow+3, 3) = 
            _weightPosCart*Eigen::VectorXd::Ones(3);
        offsetCostRow += 6;
    }
    //Right hand pose target
    {
        //Compute clamped Cartesian orientation error vector
        Eigen::Vector3d deltaMat = ClampVectorNorm(
            MatrixToAxis(
                _targetMatRightInWorld
                * _model->orientation(_nameFrameHandRight, "ROOT").transpose()), 
            _clampMatCart);
        //Compute clamped Cartesian position 
        Eigen::Vector3d deltaPos = ClampVectorNorm(
            _targetPosRightInWorld -
            _model->position(_nameFrameHandRight, "ROOT"), 
            _clampPosCart);
        //Assign matrix, vector and weight
        costMat.block(offsetCostRow, 0, 6, _sizeJoint) =
            _model->pointJacobian(_nameFrameHandRight, "ROOT")
            .block(0,6,6,_sizeJoint);
        costVec.segment(offsetCostRow+0, 3) = deltaMat;
        costVec.segment(offsetCostRow+3, 3) = deltaPos;
        weights.diagonal().segment(offsetCostRow+0, 3) = 
            _weightMatCart*Eigen::VectorXd::Ones(3);
        weights.diagonal().segment(offsetCostRow+3, 3) = 
            _weightPosCart*Eigen::VectorXd::Ones(3);
        offsetCostRow += 6;
    }
    //Delta torques wrenches and forces
    //Joint torques using linear relation from contact 
    //wrenches/forces with lower part of the equation of motion
    costMat.block(offsetCostRow, 0, _sizeJoint, sizeSol) = 
        solToTauMat.block(0, 0, _sizeJoint, sizeSol);
    costVec.segment(offsetCostRow, _sizeJoint) = 
        -solToTauVec.segment(0, _sizeJoint);
    weights.diagonal().segment(offsetCostRow, _sizeJoint) = 
        _weightJointTorque*Eigen::VectorXd::Ones(_sizeJoint);
    offsetCostRow += _sizeJoint;
    //Left hand contact wrench minimization
    size_t offsetCostCol = _sizeJoint;
    costMat.block(offsetCostRow, offsetCostCol, 6, 6) = 
        Eigen::MatrixXd::Identity(6, 6);
    costVec.segment(offsetCostRow, 6) = 
        _targetWrenchLeft - _stateWrenchHandLeft;
    weights.diagonal().segment(offsetCostRow, 6) = 
        _weightWrenchHands;
    offsetCostRow += 6;
    offsetCostCol += 6;
    //Right hand contact wrench minimization
    costMat.block(offsetCostRow, offsetCostCol, 6, 6) = 
        Eigen::MatrixXd::Identity(6, 6);
    costVec.segment(offsetCostRow, 6) = 
        _targetWrenchRight - _stateWrenchHandRight;
    weights.diagonal().segment(offsetCostRow, 6) = 
        _weightWrenchHands;
    offsetCostRow += 6;
    offsetCostCol += 6;

    //Build equality constraints
    size_t offsetEq = 0;
    if (_isBimanualEnabled) {
        //Static equilibrium equation over hands contact wrenches
        Eigen::Vector3d posCoMInRight = _model->position(
            _nameFrameHandLeft, _nameFrameHandRight, _posCoMObjectInLeft);
        Eigen::Matrix3d matComInRight = _model->orientation(
            _nameFrameHandLeft, _nameFrameHandRight);
        Eigen::Matrix6d transformLeftToCoM = _model->getWrenchTransform(
            _posCoMObjectInLeft, Eigen::Matrix3d::Identity());
        Eigen::Matrix6d transformRightToCoM = _model->getWrenchTransform(
            posCoMInRight, matComInRight);
        Eigen::Matrix3d matCoMInWorld = _model->orientation(
            _nameFrameHandLeft, "ROOT");
        Eigen::Vector6d wrenchGravityInCoM = Eigen::Vector6d::Zero();
        wrenchGravityInCoM.segment(0, 3) = 
            matCoMInWorld.transpose()*_wrenchObjectInWorld.segment(0, 3);
        wrenchGravityInCoM.segment(3, 3) = 
            matCoMInWorld.transpose()*_wrenchObjectInWorld.segment(3, 3);
        Eigen::MatrixXd rotatedVectorJacAngLeft = _pinocchio.diffRotatedVector(
            "ROOT", _nameFrameHandLeft, _wrenchObjectInWorld.segment(0, 3));
        Eigen::MatrixXd rotatedVectorJacLinLeft = _pinocchio.diffRotatedVector(
            "ROOT", _nameFrameHandLeft, _wrenchObjectInWorld.segment(3, 3));
        Eigen::MatrixXd rotatedVectorJacAngRight = _pinocchio.diffRotatedVector(
            "ROOT", _nameFrameHandRight, _wrenchObjectInWorld.segment(0, 3));
        Eigen::MatrixXd rotatedVectorJacLinRight = _pinocchio.diffRotatedVector(
            "ROOT", _nameFrameHandRight, _wrenchObjectInWorld.segment(3, 3));
        problemEqMat.block(offsetEq+0, 0, 3, _sizeJoint) = 
            -0.5*rotatedVectorJacAngLeft.block(0, 6, 3,_sizeJoint) 
            -0.5*matComInRight.transpose()*rotatedVectorJacAngRight.block(0, 6, 3,_sizeJoint);
        problemEqMat.block(offsetEq+3, 0, 3, _sizeJoint) = 
            -0.5*rotatedVectorJacLinLeft.block(0, 6, 3,_sizeJoint) 
            -0.5*matComInRight.transpose()*rotatedVectorJacLinRight.block(0, 6, 3,_sizeJoint);
        problemEqMat.block(offsetEq, _sizeJoint+0, 6, 6) = 
            transformLeftToCoM;
        problemEqMat.block(offsetEq, _sizeJoint+6, 6, 6) = 
            transformRightToCoM;
        _biasConstraintEquilibrium = 
            transformLeftToCoM*_stateWrenchHandLeft +
            transformRightToCoM*_stateWrenchHandRight
            - wrenchGravityInCoM;
        problemEqVec.segment(offsetEq, 6) = _biasConstraintEquilibrium;
        offsetEq += 6;
        //Hands kinematic closure
        _biasConstraintKinematics.segment(0, 3) = MatrixToAxis(
            _matRightHandInLeft *
            _model->orientation(_nameFrameHandRight, _nameFrameHandLeft)
                .transpose());
        _biasConstraintKinematics.segment(3, 3) =
            _posRightHandInLeft
            - _model->position(_nameFrameHandRight, _nameFrameHandLeft);
        problemEqMat.block(offsetEq, 0, 6, _sizeJoint) = 
            _model->pointJacobian(_nameFrameHandRight, _nameFrameHandLeft)
                .block(0,6,6,_sizeJoint);
        problemEqVec.segment(offsetEq, 6) = -_biasConstraintKinematics;
        offsetEq += 6;
    } else {
        //Enforce null contact force in non bimanual mode
        //when no wrench target is set
        if (_targetWrenchLeft.squaredNorm() < 1e-3) {
            problemEqMat.block(offsetEq, _sizeJoint+0, 6, 6) = 
                Eigen::MatrixXd::Identity(6, 6);
            problemEqVec.segment(offsetEq, 6) = _stateWrenchHandLeft;
            offsetEq += 6;
        }
        if (_targetWrenchRight.squaredNorm() < 1e-3) {
            problemEqMat.block(offsetEq, _sizeJoint+6, 6, 6) = 
                Eigen::MatrixXd::Identity(6, 6);
            problemEqVec.segment(offsetEq, 6) = _stateWrenchHandRight;
            offsetEq += 6;
        }
        _biasConstraintEquilibrium.setZero();
        _biasConstraintKinematics.setZero();

    }
    
    //Build inequality constraints 
    size_t offsetIneq = 0;
    //Joint kinematics bounds
    for (size_t i=0;i<_sizeJoint;i++) {
        //Retrieve angular bounds
        double limitLow = _ratioLimitsDynamics*_jointLimitPosLower(i);
        double limitUp = _ratioLimitsDynamics*_jointLimitPosUpper(i);
        if (_isAdaptativeConstraints) {
            limitLow = std::min(limitLow, _model->getDOFPosVect()(i+6));
            limitUp = std::max(limitUp, _model->getDOFPosVect()(i+6));
        }
        //Bound joint position and velocity
        problemIneqMat(offsetIneq+0, i) = 1.0;
        problemIneqMat(offsetIneq+1, i) = -1.0;
        problemIneqVec(offsetIneq+0) = 
            -std::max(limitLow-_model->getDOFPos(i+6), -_jointLimitVelAbs(i)*dt);
        problemIneqVec(offsetIneq+1) = 
            std::min(limitUp-_model->getDOFPos(i+6), _jointLimitVelAbs(i)*dt);
        offsetIneq += 2;
    }
    //Static torques wrenches forces
    Eigen::MatrixXd tmpIDIneqMat;
    Eigen::VectorXd tmpIDIneqVec;
    //Build constraints matrix and vector over 
    //joint torques and contact wrenches/forces
    buildStaticIDInequalities(
        tmpIDIneqMat, tmpIDIneqVec,
        _ratioLimitsDynamics);
    //Build mapping from ID solution (only contact wrenches/forces)
    //to extented vector with joint torques and contacts
    Eigen::MatrixXd tmpMappingMat = 
        Eigen::MatrixXd::Zero(_sizeJoint + sizeSolID, sizeSol);
    Eigen::VectorXd tmpMappingVec = 
        Eigen::VectorXd::Zero(_sizeJoint + sizeSolID);
    tmpMappingMat.block(0, 0, _sizeJoint, sizeSol) = solToTauMat;
    tmpMappingMat.block(_sizeJoint, _sizeJoint, sizeSolID, sizeSolID).setIdentity();
    tmpMappingVec.segment(0, _sizeJoint) = solToTauVec;
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
        _deltaWrenchHandLeft.setZero();
        _deltaWrenchHandRight.setZero();
        //Return failure
        return false;
    } else {
        //Copy the computed changes
        size_t offsetSolRow = 0;
        //DOF position
        _jointComputedDeltaDOF.segment(0,6).setZero();
        _jointComputedDeltaDOF.segment(6,_sizeJoint) = 
            problemSolution.segment(offsetSolRow, _sizeJoint);
        offsetSolRow += _sizeJoint;
        //Contact wrenches
        _deltaWrenchHandLeft = problemSolution.segment(offsetSolRow, 6);
        offsetSolRow += 6;
        _deltaWrenchHandRight = problemSolution.segment(offsetSolRow, 6);
        offsetSolRow += 6;
        //Joint torques
        _jointComputedDeltaTau = 
            solToTauMat*problemSolution 
            + solToTauVec
            - _jointStateTau;
        //Return success
        return true;
    }
}

void SEIKOBimanual::integrateComputedDelta(double dt)
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
    _stateWrenchHandLeft += _deltaWrenchHandLeft;
    _stateWrenchHandRight += _deltaWrenchHandRight;

    //Reset override state
    _isOverrideGravityVector = false;
}        

const Eigen::VectorXd& SEIKOBimanual::deltaDOFPosition() const
{
    return _jointComputedDeltaDOF;
}
const Eigen::VectorXd& SEIKOBimanual::deltaJointTorque() const
{
    return _jointComputedDeltaTau;
}
const Eigen::Vector6d& SEIKOBimanual::deltaWrenchLeft() const
{
    return _deltaWrenchHandLeft;
}
const Eigen::Vector6d& SEIKOBimanual::deltaWrenchRight() const
{
    return _deltaWrenchHandRight;
}

const Eigen::VectorXd& SEIKOBimanual::stateJointTorque() const
{
    return _jointStateTau;
}
const Eigen::Vector6d& SEIKOBimanual::stateWrenchLeft() const
{
    return _stateWrenchHandLeft;
}
const Eigen::Vector6d& SEIKOBimanual::stateWrenchRight() const
{
    return _stateWrenchHandRight;
}
Eigen::VectorXd& SEIKOBimanual::stateJointTorque()
{
    return _jointStateTau;
}
Eigen::Vector6d& SEIKOBimanual::stateWrenchLeft()
{
    return _stateWrenchHandLeft;
}
Eigen::Vector6d& SEIKOBimanual::stateWrenchRight()
{
    return _stateWrenchHandRight;
}

const Eigen::Vector6d& SEIKOBimanual::getErrorConstraintEquilibrium() const
{
    return _biasConstraintEquilibrium;
}
const Eigen::Vector6d& SEIKOBimanual::getErrorConstraintKinematics() const
{
    return _biasConstraintKinematics;
}

std::map<std::string, double> SEIKOBimanual::computeIKIDRatios() const
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
            "pos_" + _model->getNameDOF(i+6), 
            ratio));
    }

    //Joint torque
    for (size_t i=0;i<_sizeJoint;i++) {
        double ratio = 
            std::fabs(_jointStateTau(i))/_jointLimitTauAbs(i);
        container.insert(std::make_pair(
            "tau_" + _model->getNameDOF(i+6), 
            ratio));
    }

    //Contact wrench
    {
        Eigen::Vector6d wrench = _stateWrenchHandLeft;
        double ratioCOPX = 0.0;
        double ratioCOPY = 0.0;
        double ratioFrictionX = 0.0;
        double ratioFrictionY = 0.0;
        if (std::fabs(wrench(5)) > 1e-3) {
            ratioCOPX = std::fabs(
                wrench(1) /
                (wrench(5)*_limitCoPObject));
            ratioCOPY = std::fabs(
                wrench(0) /
                (wrench(5)*_limitCoPObject));
            ratioFrictionX = 
                std::fabs(wrench(3)) /
                std::fabs(wrench(5)*_limitFrictionObject);
            ratioFrictionY = 
                std::fabs(wrench(4)) /
                std::fabs(wrench(5)*_limitFrictionObject);
        }
        container.insert(std::make_pair(
            "copX_left", ratioCOPX));
        container.insert(std::make_pair(
            "copY_left", ratioCOPY));
        container.insert(std::make_pair(
            "frictionX_left", ratioFrictionX));
        container.insert(std::make_pair(
            "frictionY_left", ratioFrictionY));
    } 
    {
        Eigen::Vector6d wrench = _stateWrenchHandRight;
        double ratioCOPX = 0.0;
        double ratioCOPY = 0.0;
        double ratioFrictionX = 0.0;
        double ratioFrictionY = 0.0;
        if (std::fabs(wrench(5)) > 1e-3) {
            ratioCOPX = std::fabs(
                wrench(1) /
                (wrench(5)*_limitCoPObject));
            ratioCOPY = std::fabs(
                wrench(0) /
                (wrench(5)*_limitCoPObject));
            ratioFrictionX = 
                std::fabs(wrench(3)) /
                std::fabs(wrench(5)*_limitFrictionObject);
            ratioFrictionY = 
                std::fabs(wrench(4)) /
                std::fabs(wrench(5)*_limitFrictionObject);
        }
        container.insert(std::make_pair(
            "copX_right", ratioCOPX));
        container.insert(std::make_pair(
            "copY_right", ratioCOPY));
        container.insert(std::make_pair(
            "frictionX_right", ratioFrictionX));
        container.insert(std::make_pair(
            "frictionY_right", ratioFrictionY));
    } 
    
    return container;
}

void SEIKOBimanual::buildStaticIDInequalities(
    Eigen::MatrixXd& problemIneqMat,
    Eigen::VectorXd& problemIneqVec,
    double ratioLimit)
{
    //Define problem sizes
    size_t sizeIDSol = 
        //Joint torques
        _sizeJoint +
        //Hands plane contact wrenches
        6*2;
    size_t sizeIDIneq =
        //Joint torques bounds
        2*_sizeJoint + 
        //Hand contact plane constraints
        18*2;

    //Matrix and vector reset
    problemIneqMat = Eigen::MatrixXd::Zero(sizeIDIneq, sizeIDSol);
    problemIneqVec = Eigen::VectorXd::Zero(sizeIDIneq);

    //Build inequality constraints
    size_t offsetRow = 0;
    size_t offsetCol = 0;
    //Joint torque limits
    for (size_t i=0;i<_sizeJoint;i++) {
        problemIneqMat(offsetRow+0, offsetCol) = 1.0;
        problemIneqMat(offsetRow+1, offsetCol) = -1.0;
        double limitTau = ratioLimit*_jointLimitTauAbs(i);
        if (_isAdaptativeConstraints) {
            limitTau = std::max(limitTau, std::fabs(_jointStateTau(i)));
        } 
        if (_isDisabledJointTorqueConstraints) {
            limitTau = 1e3;
        }
        problemIneqVec(offsetRow+0) = limitTau;
        problemIneqVec(offsetRow+1) = limitTau;
        offsetRow += 2;
        offsetCol += 1;
    }
    //Contact plane constraints
    //Left hand
    writeContactConstraints(
        problemIneqMat, problemIneqVec,
        offsetRow, offsetCol,
        ratioLimit, true);
    offsetRow += 18;
    offsetCol += 6;
    //Right hand
    writeContactConstraints(
        problemIneqMat, problemIneqVec,
        offsetRow, offsetCol,
        ratioLimit, false);
    offsetRow += 18;
    offsetCol += 6;
}

void SEIKOBimanual::writeContactConstraints(
    Eigen::MatrixXd& problemIneqMat,
    Eigen::VectorXd& problemIneqVec,
    size_t offsetRow,
    size_t offsetCol,
    double ratioLimit,
    bool isLeftHand)
{
    //Retrieve limits
    double limitForceMin = _limitNormalForceMin;
    double limitForceMax = _limitNormalForceMax;
    double frictionCoefX = ratioLimit*_limitFrictionObject;
    double frictionCoefY = ratioLimit*_limitFrictionObject;
    double frictionCoef = ratioLimit*_limitFrictionObject;
    double copMaxX = ratioLimit*_limitCoPObject;
    double copMaxY = ratioLimit*_limitCoPObject;
    if (_isAdaptativeConstraints) {
        if (isLeftHand) {
            limitForceMin = std::min(limitForceMin, _stateWrenchHandLeft(5));
            limitForceMax = std::max(limitForceMax, _stateWrenchHandLeft(5));
            if (_stateWrenchHandLeft(5) > 1e-3) {
                copMaxX = std::max(copMaxX, 
                    std::fabs(_stateWrenchHandLeft(1)/_stateWrenchHandLeft(5)));
                copMaxY = std::max(copMaxY, 
                    std::fabs(_stateWrenchHandLeft(0)/_stateWrenchHandLeft(5)));
                frictionCoefX = std::max(frictionCoefX, 
                    std::fabs(_stateWrenchHandLeft(3)/_stateWrenchHandLeft(5)));
                frictionCoefY = std::max(frictionCoefX, 
                    std::fabs(_stateWrenchHandLeft(4)/_stateWrenchHandLeft(5)));
            }
        } else {
            limitForceMin = std::min(limitForceMin, _stateWrenchHandRight(5));
            limitForceMax = std::max(limitForceMax, _stateWrenchHandRight(5));
            if (_stateWrenchHandRight(5) > 1e-3) {
                copMaxX = std::max(copMaxX, 
                    std::fabs(_stateWrenchHandRight(1)/_stateWrenchHandRight(5)));
                copMaxY = std::max(copMaxY, 
                    std::fabs(_stateWrenchHandRight(0)/_stateWrenchHandRight(5)));
                frictionCoefX = std::max(frictionCoefX, 
                    std::fabs(_stateWrenchHandRight(3)/_stateWrenchHandRight(5)));
                frictionCoefY = std::max(frictionCoefX, 
                    std::fabs(_stateWrenchHandRight(4)/_stateWrenchHandRight(5)));
            }
        }
    }
    //Contact plane constraints
    //Normal contact force bounds
    problemIneqMat(offsetRow+0, offsetCol+5) = 1.0;
    problemIneqVec(offsetRow+0) = -limitForceMin;
    problemIneqMat(offsetRow+1, offsetCol+5) = -1.0;
    problemIneqVec(offsetRow+1) = limitForceMax;
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

}

