#include <iostream>
#include <stdexcept>
#include <algorithm>
#include <leph_model/InverseKinematics.hpp>
#include <leph_eiquadprog/leph_eiquadprog.hpp>
#include <leph_maths/AxisAngle.h>

namespace leph {

InverseKinematics::InverseKinematics(Model& model) :
    _model(&model),
    _mappingJoint(),
    _mappingTargetScalar(),
    _mappingTargetPosition(),
    _mappingTargetOrientation(),
    _mappingLimitLower(),
    _mappingLimitUpper(),
    _jointsIndex(),
    _targetsScalar(),
    _targetsPosition(),
    _targetsOrientation(),
    _limitsLower(),
    _limitsUpper(),
    _sizeDOF(0),
    _sizeTarget(0),
    _sizeLimit(0),
    _cacheJacobiansFull(),
    _cacheJacobiansCut(),
    _cachePositions(),
    _cacheOrientations(),
    _currentJointPositions(),
    _jointsLimitLower(),
    _jointsLimitUpper(),
    _weights(),
    _jointsLimitMaxChange(0.0),
    _costTargetMat(),
    _costTargetVec(),
    _problemCostMat(),
    _problemCostVec(),
    _problemEqMat(),
    _problemEqVec(),
    _problemIneqMat(),
    _problemIneqVec(),
    _problemSolution()
{
}
        
void InverseKinematics::addJoint(const std::string& nameJoint)
{
    if (
        _model->getMappingDOFs().count(nameJoint) == 0 ||
        _model->getMappingDOFs().at(nameJoint) < 6
    ) {
        throw std::logic_error(
            "leph::InverseKinematics::addJoint: "
            "Joint name unknown: " 
            + nameJoint);
    }
    if (_mappingJoint.count(nameJoint) > 0) {
        throw std::logic_error(
            "leph::InverseKinematics::addJoint: "
            "Joint name already declared: " 
            + nameJoint);
    }
    _jointsIndex.push_back(
        _model->getMappingDOFs().at(nameJoint));
    _mappingJoint.insert(
        {nameJoint, _jointsIndex.size()-1});
}

void InverseKinematics::addTargetScalar(
    const std::string& nameTarget,
    unsigned int dim,
    const std::string& nameFrameSrc,
    const std::string& nameFrameDst)
{
    if (_mappingTargetScalar.count(nameTarget) > 0) {
        throw std::logic_error(
            "leph::InverseKinematics::addTargetScalar: "
            "Target already declared: " 
            + nameTarget);
    }
    if (
        (nameFrameSrc != "COM" &&
        _model->getMappingFrames().count(nameFrameSrc) == 0) ||
        _model->getMappingFrames().count(nameFrameDst) == 0
    ) {
        throw std::logic_error(
            "leph::InverseKinematics::addTargetScalar: "
            "Invalid frame name.");
    }
    if (
        (nameFrameSrc == "COM" && dim >= 3) ||
        (nameFrameSrc != "COM" && dim >= 6)
    ) {
        throw std::logic_error(
            "leph::InverseKinematics::addTargetScalar: "
            "Invalid dimension: " + std::to_string(dim));
    }
    size_t frameIdSrc;
    if (nameFrameSrc == "COM") {
        frameIdSrc = (size_t)-1;
    } else {
        frameIdSrc = _model->getMappingFrames().at(nameFrameSrc);
    }
    _targetsScalar.push_back({
        frameIdSrc,
        _model->getMappingFrames().at(nameFrameDst), 
        dim, 0.0, 
        (nameFrameSrc != "COM" && dim < 3)});
    _mappingTargetScalar.insert(
        {nameTarget, _targetsScalar.size()-1});
}
void InverseKinematics::addTargetPosition(
    const std::string& nameTarget,
    const std::string& nameFrameSrc,
    const std::string& nameFrameDst)
{
    if (_mappingTargetPosition.count(nameTarget) > 0) {
        throw std::logic_error(
            "leph::InverseKinematics::addTargetPosition: "
            "Target already declared: " 
            + nameTarget);
    }
    if (
        (nameFrameSrc != "COM" &&
        _model->getMappingFrames().count(nameFrameSrc) == 0) ||
        _model->getMappingFrames().count(nameFrameDst) == 0
    ) {
        throw std::logic_error(
            "leph::InverseKinematics::addTargetPosition: "
            "Invalid frame name.");
    }
    size_t frameIdSrc;
    if (nameFrameSrc == "COM") {
        frameIdSrc = (size_t)-1;
    } else {
        frameIdSrc = _model->getMappingFrames().at(nameFrameSrc);
    }
    _targetsPosition.push_back({
        frameIdSrc,
        _model->getMappingFrames().at(nameFrameDst), 
        Eigen::Vector3d::Zero()});
    _mappingTargetPosition.insert(
        {nameTarget, _targetsPosition.size()-1});
}
void InverseKinematics::addTargetOrientation(
    const std::string& nameTarget,
    const std::string& nameFrameSrc,
    const std::string& nameFrameDst)
{
    if (_mappingTargetOrientation.count(nameTarget) > 0) {
        throw std::logic_error(
            "leph::InverseKinematics::addTargetOrientation: "
            "Target already declared: " 
            + nameTarget);
    }
    if (
        _model->getMappingFrames().count(nameFrameSrc) == 0 ||
        _model->getMappingFrames().count(nameFrameDst) == 0
    ) {
        throw std::logic_error(
            "leph::InverseKinematics::addTargetOrientation: "
            "Invalid frame name.");
    }
    _targetsOrientation.push_back({
        _model->getMappingFrames().at(nameFrameSrc),
        _model->getMappingFrames().at(nameFrameDst), 
        Eigen::Matrix3d::Zero()});
    _mappingTargetOrientation.insert(
        {nameTarget, _targetsOrientation.size()-1});
}

void InverseKinematics::addLimitLower(
    const std::string& nameLimit,
    unsigned int dim,
    const std::string& nameFrameSrc,
    const std::string& nameFrameDst)
{
    if (_mappingLimitLower.count(nameLimit) > 0) {
        throw std::logic_error(
            "leph::InverseKinematics::addLimitLower: "
            "Limit already declared: " 
            + nameLimit);
    }
    if (
        (nameFrameSrc != "COM" &&
        _model->getMappingFrames().count(nameFrameSrc) == 0) ||
        _model->getMappingFrames().count(nameFrameDst) == 0
    ) {
        throw std::logic_error(
            "leph::InverseKinematics::addLimitLower: "
            "Invalid frame name.");
    }
    if (
        (nameFrameSrc == "COM" && dim >= 3) ||
        (nameFrameSrc != "COM" && dim >= 6)
    ) {
        throw std::logic_error(
            "leph::InverseKinematics::addLimitLower: "
            "Invalid dimension: " + std::to_string(dim));
    }
    size_t frameIdSrc;
    if (nameFrameSrc == "COM") {
        frameIdSrc = (size_t)-1;
    } else {
        frameIdSrc = _model->getMappingFrames().at(nameFrameSrc);
    }
    _limitsLower.push_back({
        frameIdSrc,
        _model->getMappingFrames().at(nameFrameDst), 
        dim, 0.0, 
        (nameFrameSrc != "COM" && dim < 3)});
    _mappingLimitLower.insert(
        {nameLimit, _limitsLower.size()-1});
}
void InverseKinematics::addLimitUpper(
    const std::string& nameLimit,
    unsigned int dim,
    const std::string& nameFrameSrc,
    const std::string& nameFrameDst)
{
    if (_mappingLimitUpper.count(nameLimit) > 0) {
        throw std::logic_error(
            "leph::InverseKinematics::addLimitUpper: "
            "Limit already declared: " 
            + nameLimit);
    }
    if (
        (nameFrameSrc != "COM" &&
        _model->getMappingFrames().count(nameFrameSrc) == 0) ||
        _model->getMappingFrames().count(nameFrameDst) == 0
    ) {
        throw std::logic_error(
            "leph::InverseKinematics::addLimitUpper: "
            "Invalid frame name.");
    }
    if (
        (nameFrameSrc == "COM" && dim >= 3) ||
        (nameFrameSrc != "COM" && dim >= 6)
    ) {
        throw std::logic_error(
            "leph::InverseKinematics::addLimitUpper: "
            "Invalid dimension: " + std::to_string(dim));
    }
    size_t frameIdSrc;
    if (nameFrameSrc == "COM") {
        frameIdSrc = (size_t)-1;
    } else {
        frameIdSrc = _model->getMappingFrames().at(nameFrameSrc);
    }
    _limitsUpper.push_back({
        frameIdSrc,
        _model->getMappingFrames().at(nameFrameDst), 
        dim, 0.0, 
        (nameFrameSrc != "COM" && dim < 3)});
    _mappingLimitUpper.insert(
        {nameLimit, _limitsUpper.size()-1});
}
        
void InverseKinematics::init()
{
    //Sort declared joints by growing RBDL index
    //(hope to optimize cache use for Jacobian 
    //matrix columns extraction)
    std::vector<size_t> tmpIndexes = _jointsIndex;
    std::sort(tmpIndexes.begin(), tmpIndexes.end());
    std::map<std::string, size_t> tmpMapping;
    for (const auto& it : _mappingJoint) {
        size_t j = 0;
        while (tmpIndexes[j] != _jointsIndex[it.second]) {
            j++;
        }
        tmpMapping.insert({it.first, j});
    }
    _jointsIndex = tmpIndexes;
    _mappingJoint = tmpMapping;
    
    //Compute problem sizes
    _sizeDOF = 
        _jointsIndex.size();
    _sizeTarget = 
        _sizeDOF +
        _targetsScalar.size() + 
        3*_targetsPosition.size() + 
        3*_targetsOrientation.size();
    _sizeLimit = 
        2*_sizeDOF +
        _limitsLower.size() +
        _limitsUpper.size();

    //Cache matrices allocation
    for (size_t i=0;i<_targetsScalar.size();i++) {
        if (_targetsScalar[i].isOrientation) {
            initCache(
                _targetsScalar[i].frameSrc, 
                _targetsScalar[i].frameDst, 
                true, false, true);
        } else {
            initCache(
                _targetsScalar[i].frameSrc, 
                _targetsScalar[i].frameDst, 
                true, true, false);
        }
    }
    for (size_t i=0;i<_targetsPosition.size();i++) {
        initCache(
            _targetsPosition[i].frameSrc, 
            _targetsPosition[i].frameDst, 
            true, true, false);
    }
    for (size_t i=0;i<_targetsOrientation.size();i++) {
        initCache(
            _targetsOrientation[i].frameSrc, 
            _targetsOrientation[i].frameDst, 
            true, false, true);
    }
    for (size_t i=0;i<_limitsLower.size();i++) {
        if (_limitsLower[i].isOrientation) {
            initCache(
                _limitsLower[i].frameSrc, 
                _limitsLower[i].frameDst, 
                true, false, true);
        } else {
            initCache(
                _limitsLower[i].frameSrc, 
                _limitsLower[i].frameDst, 
                true, true, false);
        }
    }
    for (size_t i=0;i<_limitsUpper.size();i++) {
        if (_limitsUpper[i].isOrientation) {
            initCache(
                _limitsUpper[i].frameSrc, 
                _limitsUpper[i].frameDst, 
                true, false, true);
        } else {
            initCache(
                _limitsUpper[i].frameSrc, 
                _limitsUpper[i].frameDst, 
                true, true, false);
        }
    }

    //Problem matrices allocation
    _currentJointPositions = Eigen::VectorXd::Zero(_sizeDOF);
    _jointsLimitLower = Eigen::VectorXd::Zero(_sizeDOF);
    _jointsLimitUpper = Eigen::VectorXd::Zero(_sizeDOF);
    _weights = Eigen::DiagonalMatrix<double, -1>(_sizeTarget);
    _weights.setIdentity();
    _jointsLimitMaxChange = M_PI/2.0;
    _costTargetMat = Eigen::MatrixXd::Zero(_sizeTarget, _sizeDOF);
    _costTargetVec = Eigen::VectorXd::Zero(_sizeTarget);
    _problemCostMat = Eigen::MatrixXd::Zero(_sizeDOF, _sizeDOF);
    _problemCostVec = Eigen::VectorXd::Zero(_sizeDOF);
    _problemEqMat = Eigen::MatrixXd::Zero(_sizeDOF, 0);
    _problemEqVec = Eigen::VectorXd::Zero(0);
    _problemIneqMat = Eigen::MatrixXd::Zero(_sizeDOF, _sizeLimit);
    _problemIneqVec = Eigen::VectorXd::Zero(_sizeLimit);
    _problemSolution = Eigen::VectorXd::Zero(_sizeDOF);

    //Setup static problem matrices
    //Joint change regularization
    _costTargetMat.block(0, 0, _sizeDOF, _sizeDOF) = 
        Eigen::MatrixXd::Identity(_sizeDOF, _sizeDOF);
    _costTargetVec.segment(0, _sizeDOF) = 
        Eigen::VectorXd::Zero(_sizeDOF);
    //Joint position limits
    _problemIneqMat.block(0, 0, _sizeDOF, _sizeDOF) = 
        Eigen::MatrixXd::Identity(_sizeDOF, _sizeDOF);
    _problemIneqMat.block(0, _sizeDOF, _sizeDOF, _sizeDOF) = 
        -Eigen::MatrixXd::Identity(_sizeDOF, _sizeDOF);

    //Initialize joint limits from URDF bounds
    for (size_t i=0;i<_jointsIndex.size();i++) {
        _jointsLimitLower(i) = 
            _model->jointLimitsLower()(_jointsIndex[i]-6);
        _jointsLimitUpper(i) = 
            _model->jointLimitsUpper()(_jointsIndex[i]-6);
    }
}
        
void InverseKinematics::setDamping(double weight)
{
    if (weight < 0.0) {
        throw std::logic_error(
            "leph::InverseKinematics::setDamping: "
            "Invalid weight: " + std::to_string(weight));
    }
    _weights.diagonal().segment(0, _sizeDOF) = 
        weight*Eigen::VectorXd::Ones(_sizeDOF);
}
        
void InverseKinematics::setMaxChange(double max)
{
    if (max <= 0.0) {
        throw std::logic_error(
            "leph::InverseKinematics::setMaxChange: "
            "Invalid bound: " + std::to_string(max));
    }
    _jointsLimitMaxChange = max;
}

void InverseKinematics::setWeightScalar(
    const std::string& nameTarget,
    double weight)
{
    if (_mappingTargetScalar.count(nameTarget) == 0) {
        throw std::logic_error(
            "leph::InverseKinematics::setWeightScalar: "
            "Target name not declared: " + nameTarget);
    }
    if (weight < 0.0) {
        throw std::logic_error(
            "leph::InverseKinematics::setWeightScalar: "
            "Invalid weight: " + std::to_string(weight));
    }
    size_t index = _mappingTargetScalar.at(nameTarget);
    _weights.diagonal()(_sizeDOF+index) = weight;
}
void InverseKinematics::setWeightPosition(
    const std::string& nameTarget,
    double weight)
{
    if (_mappingTargetPosition.count(nameTarget) == 0) {
        throw std::logic_error(
            "leph::InverseKinematics::setWeightPosition: "
            "Target name not declared: " + nameTarget);
    }
    if (weight < 0.0) {
        throw std::logic_error(
            "leph::InverseKinematics::setWeightPosition: "
            "Invalid weight: " + std::to_string(weight));
    }
    size_t index = _mappingTargetPosition.at(nameTarget);
    size_t sizeScalar = _targetsScalar.size();
    _weights.diagonal().segment(_sizeDOF+sizeScalar+3*index, 3) = 
        weight*Eigen::Vector3d::Ones();
}
void InverseKinematics::setWeightOrientation(
    const std::string& nameTarget,
    double weight)
{
    if (_mappingTargetOrientation.count(nameTarget) == 0) {
        throw std::logic_error(
            "leph::InverseKinematics::setWeightOrientation: "
            "Target name not declared: " + nameTarget);
    }
    if (weight < 0.0) {
        throw std::logic_error(
            "leph::InverseKinematics::setWeightOrientation: "
            "Invalid weight: " + std::to_string(weight));
    }
    size_t index = _mappingTargetOrientation.at(nameTarget);
    size_t sizeScalar = _targetsScalar.size();
    size_t sizePosition = _targetsPosition.size();
    _weights.diagonal().segment(
        _sizeDOF+sizeScalar+3*sizePosition+3*index, 3) = 
        weight*Eigen::Vector3d::Ones();
}
        
void InverseKinematics::setJointLimit(
    const std::string& nameJoint,
    double limitLower,
    double limitUpper)
{
    if (_mappingJoint.count(nameJoint) == 0) {
        throw std::logic_error(
            "leph::InverseKinematics::setJointLimit: "
            "Joint name not declared: " + nameJoint);
    }
    if (limitUpper < limitLower) {
        throw std::logic_error(
            "leph::InverseKinematics::setJointLimit: "
            "Invalid bounds.");
    }
    size_t index = _mappingJoint.at(nameJoint);
    _jointsLimitLower(index) = limitLower;
    _jointsLimitUpper(index) = limitUpper;
}

void InverseKinematics::setTargetScalar(
    const std::string& nameTarget,
    double val)
{
    if (_mappingTargetScalar.count(nameTarget) == 0) {
        throw std::logic_error(
            "leph::InverseKinematics::setTargetScalar: "
            "Target name not declared: " + nameTarget);
    }
    size_t index = _mappingTargetScalar.at(nameTarget);
    _targetsScalar[index].val = val;
}
void InverseKinematics::setTargetPosition(
    const std::string& nameTarget,
    const Eigen::Vector3d& pos)
{
    if (_mappingTargetPosition.count(nameTarget) == 0) {
        throw std::logic_error(
            "leph::InverseKinematics::setTargetPosition: "
            "Target name not declared: " + nameTarget);
    }
    size_t index = _mappingTargetPosition.at(nameTarget);
    _targetsPosition[index].pos = pos;
}
void InverseKinematics::setTargetOrientation(
    const std::string& nameTarget,
    const Eigen::Matrix3d& mat)
{
    if (_mappingTargetOrientation.count(nameTarget) == 0) {
        throw std::logic_error(
            "leph::InverseKinematics::setTargetOrientation: "
            "Target name not declared: " + nameTarget);
    }
    size_t index = _mappingTargetOrientation.at(nameTarget);
    _targetsOrientation[index].mat = mat;
}

void InverseKinematics::setLimitLower(
    const std::string& nameLimit,
    double val)
{
    if (_mappingLimitLower.count(nameLimit) == 0) {
        throw std::logic_error(
            "leph::InverseKinematics::setLimitLower: "
            "Limit name not declared: " + nameLimit);
    }
    size_t index = _mappingLimitLower.at(nameLimit);
    _limitsLower[index].val = val;
}
void InverseKinematics::setLimitUpper(
    const std::string& nameLimit,
    double val)
{
    if (_mappingLimitUpper.count(nameLimit) == 0) {
        throw std::logic_error(
            "leph::InverseKinematics::setLimitUpper: "
            "Limit name not declared: " + nameLimit);
    }
    size_t index = _mappingLimitUpper.at(nameLimit);
    _limitsUpper[index].val = val;
}
        
bool InverseKinematics::run()
{
    //Retrieve current declared joint position
    for (size_t i=0;i<_jointsIndex.size();i++) {
        _currentJointPositions(i) = 
            _model->getDOFPos(_jointsIndex[i]);
    }

    //Compute needed Jacobian matrices
    for (auto& it : _cacheJacobiansFull) {
        if (it.first.first == (size_t)-1) {
            it.second = _model->comJacobian(
                it.first.second);
        } else {
            it.second = _model->pointJacobian(
                it.first.first, it.first.second);
        }
        //Compute restricted Jacobian 
        //to select declared joints
        Eigen::MatrixXd& ref = _cacheJacobiansCut.at(it.first);
        for (size_t i=0;i<_jointsIndex.size();i++) {
            ref.col(i) = it.second.col(_jointsIndex[i]);
        }
    }
    //Compute needed position vectors
    for (auto& it : _cachePositions) {
        if (it.first.first == (size_t)-1) {
            it.second = _model->centerOfMass(
                it.first.second);
        } else {
            it.second = _model->position(
                it.first.first, it.first.second);
        }
    }
    //Compute needed orientation matrices
    for (auto& it : _cacheOrientations) {
        it.second = _model->orientation(
            it.first.first, it.first.second);
    }

    //Build target summed distance
    //User declared scalar targets
    size_t sizeScalar = _targetsScalar.size();
    size_t sizePosition = _targetsPosition.size();
    size_t sizeOrientation = _targetsOrientation.size();
    for (size_t i=0;i<sizeScalar;i++) {
        size_t idSrc = _targetsScalar[i].frameSrc;
        size_t idDst = _targetsScalar[i].frameDst;
        std::pair<size_t, size_t> id(idSrc, idDst);
        unsigned int dim = _targetsScalar[i].dim;
        if (_targetsScalar[i].isOrientation) {
            _costTargetMat.block(_sizeDOF+i, 0, 1, _sizeDOF) =
                _cacheJacobiansCut.at(id).block(dim, 0, 1, _sizeDOF);
            _costTargetVec(_sizeDOF+i) =
                _targetsScalar[i].val - leph::MatrixToAxis(_cacheOrientations.at(id))(dim);
        } else {
            _costTargetMat.block(_sizeDOF+i, 0, 1, _sizeDOF) =
                _cacheJacobiansCut.at(id).block(dim, 0, 1, _sizeDOF);
            _costTargetVec(_sizeDOF+i) =
                _targetsScalar[i].val - _cachePositions.at(id)(dim);
        }
    }
    //User declared position targets
    for (size_t i=0;i<sizePosition;i++) {
        size_t idSrc = _targetsPosition[i].frameSrc;
        size_t idDst = _targetsPosition[i].frameDst;
        std::pair<size_t, size_t> id(idSrc, idDst);
        if (idSrc == (size_t)-1) {
            _costTargetMat.block(_sizeDOF+sizeScalar+3*i, 0, 3, _sizeDOF) = 
                _cacheJacobiansCut.at(id).block(0, 0, 3, _sizeDOF);
        } else {
            _costTargetMat.block(_sizeDOF+sizeScalar+3*i, 0, 3, _sizeDOF) = 
                _cacheJacobiansCut.at(id).block(3, 0, 3, _sizeDOF);
        }
        _costTargetVec.segment(_sizeDOF+sizeScalar+3*i, 3) = 
            _targetsPosition[i].pos - _cachePositions.at(id);
    }
    //User declared orientation targets
    for (size_t i=0;i<sizeOrientation;i++) {
        size_t idSrc = _targetsOrientation[i].frameSrc;
        size_t idDst = _targetsOrientation[i].frameDst;
        std::pair<size_t, size_t> id(idSrc, idDst);
        _costTargetMat.block(_sizeDOF+sizeScalar+3*sizePosition+3*i, 0, 3, _sizeDOF) = 
            _cacheJacobiansCut.at(id).block(0, 0, 3, _sizeDOF);
        _costTargetVec.segment(_sizeDOF+sizeScalar+3*sizePosition+3*i, 3) = 
            leph::MatrixToAxis(
                _targetsOrientation[i].mat*_cacheOrientations.at(id).transpose());
    }

    //Build the weighted distance of linear target cost 
    _problemCostMat = _costTargetMat.transpose()*_weights*_costTargetMat;
    _problemCostVec = -_costTargetMat.transpose()*_weights*_costTargetVec;

    //Build lower and upper limit inequality constraints
    //Joint position limits
    _problemIneqVec.segment(0, _sizeDOF) = 
        (_currentJointPositions-_jointsLimitLower)
            .array().min(_jointsLimitMaxChange);
    _problemIneqVec.segment(_sizeDOF, _sizeDOF) = 
        (_jointsLimitUpper-_currentJointPositions)
            .array().min(_jointsLimitMaxChange);
    //User declared lower limits
    size_t sizeLower = _limitsLower.size();
    size_t sizeUpper = _limitsUpper.size();
    for (size_t i=0;i<sizeLower;i++) {
        size_t idSrc = _limitsLower[i].frameSrc;
        size_t idDst = _limitsLower[i].frameDst;
        std::pair<size_t, size_t> id(idSrc, idDst);
        unsigned int dim = _limitsLower[i].dim;
        if (_limitsLower[i].isOrientation) {
            _problemIneqMat.block(0, 2*_sizeDOF+i, _sizeDOF, 1) =
                _cacheJacobiansCut.at(id).block(dim, 0, 1, _sizeDOF).transpose();
            _problemIneqVec(2*_sizeDOF+i) =
                leph::MatrixToAxis(_cacheOrientations.at(id))(dim) - _limitsLower[i].val;
        } else {
            _problemIneqMat.block(0, 2*_sizeDOF+i, _sizeDOF, 1) =
                _cacheJacobiansCut.at(id).block(dim, 0, 1, _sizeDOF).transpose();
            _problemIneqVec(2*_sizeDOF+i) =
                _cachePositions.at(id)(dim) - _limitsLower[i].val;
        }
    }
    //User declared upper limits
    for (size_t i=0;i<sizeUpper;i++) {
        size_t idSrc = _limitsUpper[i].frameSrc;
        size_t idDst = _limitsUpper[i].frameDst;
        std::pair<size_t, size_t> id(idSrc, idDst);
        unsigned int dim = _limitsUpper[i].dim;
        if (_limitsUpper[i].isOrientation) {
            _problemIneqMat.block(0, 2*_sizeDOF+sizeLower+i, _sizeDOF, 1) =
                -_cacheJacobiansCut.at(id).block(dim, 0, 1, _sizeDOF).transpose();
            _problemIneqVec(2*_sizeDOF+sizeLower+i) =
                _limitsUpper[i].val - leph::MatrixToAxis(_cacheOrientations.at(id))(dim);
        } else {
            _problemIneqMat.block(0, 2*_sizeDOF+sizeLower+i, _sizeDOF, 1) =
                -_cacheJacobiansCut.at(id).block(dim, 0, 1, _sizeDOF).transpose();
            _problemIneqVec(2*_sizeDOF+sizeLower+i) =
                _limitsUpper[i].val - _cachePositions.at(id)(dim);
        }
    }

    //Solve the QP problem
    double cost = Eigen::solve_quadprog(
        _problemCostMat,
        _problemCostVec,
        _problemEqMat,
        _problemEqVec,
        _problemIneqMat,
        _problemIneqVec,
        _problemSolution);

    //Assign the model's joint state only
    //if the solution is valid (not infeasible problem)
    if (!std::isnan(cost) && !std::isinf(cost)) {
        _currentJointPositions += _problemSolution;
        for (size_t i=0;i<_jointsIndex.size();i++) {
            _model->setDOFPos(
                _jointsIndex[i], 
                _currentJointPositions(i));
        }
        return true;
    } else {
        return false;
    }
}
        
void InverseKinematics::print() const
{
    std::cout << "=== Sizes:" << std::endl;
    std::cout << "sizeDOF=   " << _sizeDOF << std::endl;
    std::cout << "sizeTarget=" << _sizeTarget << std::endl;
    std::cout << "sizeLimit= " << _sizeLimit << std::endl;
    std::cout << "=== Mapping Joints:" << std::endl;
    for (const auto& it : _mappingJoint) {
        std::cout 
            << "name=" << it.first 
            << " index=" << it.second
            << " id=" << _jointsIndex[it.second]
            << std::endl;
    }
    std::cout << "=== Mapping Target Scalar:" << std::endl;
    for (const auto& it : _mappingTargetScalar) {
        std::cout 
            << "name=" << it.first
            << " index=" << it.second
            << " frameSrc=" << _targetsScalar[it.second].frameSrc
            << " dim=" << _targetsScalar[it.second].dim
            << " isOrientation=" << _targetsScalar[it.second].isOrientation
            << " val=" << _targetsScalar[it.second].val
            << std::endl;
    }
    std::cout << "=== Mapping Target Position:" << std::endl;
    for (const auto& it : _mappingTargetPosition) {
        std::cout 
            << "name=" << it.first
            << " index=" << it.second
            << " frameSrc=" << _targetsPosition[it.second].frameSrc
            << " pos=" << _targetsPosition[it.second].pos.transpose()
            << std::endl;
    }
    std::cout << "=== Mapping Target Orientation:" << std::endl;
    for (const auto& it : _mappingTargetOrientation) {
        std::cout 
            << "name=" << it.first
            << " index=" << it.second
            << " frameSrc=" << _targetsOrientation[it.second].frameSrc
            << " mat=" << std::endl 
            << _targetsOrientation[it.second].mat
            << std::endl;
    }
    std::cout << "=== Mapping Limit Lower:" << std::endl;
    for (const auto& it : _mappingLimitLower) {
        std::cout 
            << "name=" << it.first
            << " index=" << it.second
            << " frameSrc=" << _limitsLower[it.second].frameSrc
            << " dim=" << _limitsLower[it.second].dim
            << " isOrientation=" << _limitsLower[it.second].isOrientation
            << " val=" << _limitsLower[it.second].val
            << std::endl;
    }
    std::cout << "=== Mapping Limit Upper:" << std::endl;
    for (const auto& it : _mappingLimitUpper) {
        std::cout 
            << "name=" << it.first
            << " index=" << it.second
            << " frameSrc=" << _limitsUpper[it.second].frameSrc
            << " dim=" << _limitsUpper[it.second].dim
            << " isOrientation=" << _limitsUpper[it.second].isOrientation
            << " val=" << _limitsUpper[it.second].val
            << std::endl;
    }
    std::cout << "=== Cache JacobiansFull:" << std::endl;
    for (const auto& it : _cacheJacobiansFull) {
        std::cout 
            << "idSrc=" << it.first.first 
            << " idDst=" << it.first.second 
            << " rows=" << it.second.rows() 
            << " cols=" << it.second.cols() 
            << std::endl;
    }
    std::cout << "=== Cache JacobiansCut:" << std::endl;
    for (const auto& it : _cacheJacobiansCut) {
        std::cout 
            << "idSrc=" << it.first.first 
            << " idDst=" << it.first.second 
            << " rows=" << it.second.rows() 
            << " cols=" << it.second.cols() 
            << std::endl;
    }
    std::cout << "=== Cache Positions:" << std::endl;
    for (const auto& it : _cachePositions) {
        std::cout 
            << "idSrc=" << it.first.first 
            << " idDst=" << it.first.second 
            << " rows=" << it.second.rows() 
            << " cols=" << it.second.cols() 
            << std::endl;
    }
    std::cout << "=== Cache Orientations:" << std::endl;
    for (const auto& it : _cacheOrientations) {
        std::cout 
            << "idSrc=" << it.first.first 
            << " idDst=" << it.first.second 
            << " rows=" << it.second.rows() 
            << " cols=" << it.second.cols() 
            << std::endl;
    }
    std::cout << "=== Joint Limit Lower:" << std::endl;
    std::cout 
        << "rows=" << _jointsLimitLower.rows() 
        << " cols=" << _jointsLimitLower.cols() 
        << std::endl;
    std::cout << _jointsLimitLower.transpose() << std::endl;
    std::cout << "=== Joint Limit Upper:" << std::endl;
    std::cout 
        << "rows=" << _jointsLimitUpper.rows() 
        << " cols=" << _jointsLimitUpper.cols() 
        << std::endl;
    std::cout << _jointsLimitUpper.transpose() << std::endl;
    std::cout << "=== Weights:" << std::endl;
    std::cout 
        << "rows=" << _weights.rows() 
        << " cols=" << _weights.cols() 
        << std::endl;
    std::cout << _weights.diagonal().transpose() << std::endl;
    std::cout << "=== CostTargetMat:" << std::endl;
    std::cout 
        << "rows=" << _costTargetMat.rows() 
        << " cols=" << _costTargetMat.cols() 
        << std::endl;
    std::cout << _costTargetMat << std::endl;
    std::cout << "=== CostTargetVec:" << std::endl;
    std::cout 
        << "rows=" << _costTargetVec.rows() 
        << " cols=" << _costTargetVec.cols() 
        << std::endl;
    std::cout << _costTargetVec.transpose() << std::endl;
    
    std::cout << "=== IneqMap:" << std::endl;
    std::cout 
        << "rows=" << _problemIneqMat.rows() 
        << " cols=" << _problemIneqMat.cols() 
        << std::endl;
    std::cout << _problemIneqMat.transpose() << std::endl;
    std::cout << "=== IneqVec:" << std::endl;
    std::cout 
        << "rows=" << _problemIneqVec.rows() 
        << " cols=" << _problemIneqVec.cols() 
        << std::endl;
    std::cout << _problemIneqVec.transpose() << std::endl;
}

void InverseKinematics::initCache(
    size_t idSrc, size_t idDst,
    bool initJac, bool initPos, bool initMat)
{
    if (initJac) {
        if (idSrc == (size_t)-1) {
            //CoM case
            _cacheJacobiansFull[std::make_pair(idSrc, idDst)] = 
                Eigen::MatrixXd::Zero(3, _model->sizeVectVel());
            _cacheJacobiansCut[std::make_pair(idSrc, idDst)] = 
                Eigen::MatrixXd::Zero(3, _sizeDOF);
        } else {
            //Other cases
            _cacheJacobiansFull[std::make_pair(idSrc, idDst)] = 
                Eigen::MatrixXd::Zero(6, _model->sizeVectVel());
            _cacheJacobiansCut[std::make_pair(idSrc, idDst)] = 
                Eigen::MatrixXd::Zero(6, _sizeDOF);
        }
    }
    if (initPos) {
        _cachePositions[std::make_pair(idSrc, idDst)] = 
            Eigen::Vector3d::Zero();
    }
    if (initMat) {
        _cacheOrientations[std::make_pair(idSrc, idDst)] = 
            Eigen::Matrix3d::Identity();
    }
}

}

