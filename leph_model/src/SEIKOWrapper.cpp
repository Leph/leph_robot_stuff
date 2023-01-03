#include <leph_model/SEIKOWrapper.hpp>
#include <leph_maths/Angle.h>

namespace leph {

void SEIKOWrapper::setup()
{
    //Reset internal state
    _jointPosMarginCoef = Eigen::VectorXd::Ones(this->_sizeJoint);
    _vectWeightWrench.setZero();
    _vectWeightForce.setZero();
    _aheadJointPos = Eigen::VectorXd::Zero(this->_sizeJoint);
}
        
bool SEIKOWrapper::reset()
{
    //Reset SEIKO to underlying model
    this->resetTargetsFromModel();

    //Reset forces and wrenches target
    for (auto& it : _commands) {
        if (it.second.isPoint) {
            this->setTargetForce(it.first,
                Eigen::Vector3d::Zero());
        } else {
            this->setTargetWrench(it.first,
                Eigen::Vector6d::Zero());
        }
    }
    
    //Reset default point contact orientation
    for (auto& it : _commands) {
        if (it.second.isPoint) {
            this->setContactMat(it.first,
                this->_model->orientation(it.first, "ROOT"));
        } 
    }

    //Reset command pose filters
    //and current velocity commands
    for (auto& it : _commands) {
        it.second.isClutch = false;
        it.second.velLin.setZero();
        it.second.velAng.setZero();
        it.second.filterPose.reset(
            this->_model->position(it.first, "ROOT"),
            this->_model->orientation(it.first, "ROOT"));
    }
    
    //Run static ID to reset SEIKO 
    //internal forces and wrenches
    bool isSuccess = this->runInverseDynamics();

    //Reset smoothing init ratio
    _initRatio = 0.0;

    return isSuccess;
}

void SEIKOWrapper::setTargetPose(
    const std::string& frameName,
    bool isClutch,
    const Eigen::Vector3d& rawPos, 
    const Eigen::Matrix3d& rawMat)
{
    if (_commands.count(frameName) == 0) {
        throw std::logic_error(
            "leph::SEIKOWrapper::setTargetPose: "
            "Undeclared frame name: " + frameName);
    }
    _commands.at(frameName).isClutch = isClutch;
    _commands.at(frameName).rawPos = rawPos;
    _commands.at(frameName).rawMat = rawMat;
}
        
void SEIKOWrapper::setTargetVel(
    const std::string& frameName,
    const Eigen::Vector3d& velLin, 
    const Eigen::Vector3d& velAng)
{
    if (_commands.count(frameName) == 0) {
        throw std::logic_error(
            "leph::SEIKOWrapper::setTargetVel: "
            "Undeclared frame name: " + frameName);
    }
    _commands.at(frameName).velLin = velLin;
    _commands.at(frameName).velAng = velAng;
}

void SEIKOWrapper::askSwitching(
    const std::string& frameName,
    bool isEnabled)
{
    if (_commands.count(frameName) == 0) {
        throw std::logic_error(
            "leph::SEIKOWrapper::askSwitching: "
            "Undeclared frame name: " + frameName);
    }
    EndEffector_t& contact = _commands.at(frameName);
    if (isEnabled) {
        //Ask for enabling contact
        if (
            !contact.isSwitchingDisable && 
            !this->stateContactBase(frameName).isEnabled
        ) {
            if (contact.isPoint) {
                //Reset default surface orientation
                this->setContactMat(frameName,
                    this->_model->orientation(frameName, "ROOT"));
                //Reset force weight
                this->setWeightForce(frameName,
                    contact.weightForceEnabled*_vectWeightForce);
            } else {
                //Reset wrench weight
                this->setWeightWrench(frameName,
                    contact.weightForceEnabled*_vectWeightWrench);
            }
            //Set min force to zero
            this->setNormalForceLimits(frameName,
                0.0, contact.limitNormalForceMax);
            //Toggle SEIKO contact
            this->toggleContact(frameName, true);
        }
    } else {
        //Ask for disabling procedure
        if (
            !contact.isSwitchingDisable && 
            this->stateContactBase(frameName).isEnabled
        ) {
            //Start switching procedure
            contact.isSwitchingDisable = true;
            if (contact.isPoint) {
                //Set hight force weight
                this->setWeightForce(frameName,
                    contact.weightForceDisabling*_vectWeightForce);
            } else {
                //Set hight wrench weight
                this->setWeightWrench(frameName,
                    contact.weightForceDisabling*_vectWeightWrench);
            }
            //Set min force to zero
            this->setNormalForceLimits(frameName,
                0.0, contact.limitNormalForceMax);
        }
    }
}
        
bool SEIKOWrapper::update(double dt_task, double dt_ahead)
{
    //Switching state update
    for (auto& it : _commands) {
        const ContactBase_t& contact = this->stateContactBase(it.first);
        double forceNormal = this->stateNormalForce(it.first);
        //For enabled contact, check if their min force limit 
        //can be raised to desired one
        if (
            !it.second.isSwitchingDisable &&
            contact.isEnabled && 
            contact.normalForceMin < it.second.limitNormalForceMin &&
            forceNormal > contact.normalForceMin+1e-2
        ) {
            this->setNormalForceLimits(
                it.first,
                std::min(forceNormal-1e-2, it.second.limitNormalForceMin), 
                it.second.limitNormalForceMax);
        }
        //Check if disabling contacts can be fully disabled
        if (
            it.second.isSwitchingDisable &&
            contact.isEnabled && 
            forceNormal < 0.1*dt_task*contact.limitVelNormalForce
        ) {
            it.second.isSwitchingDisable = false;
            this->toggleContact(it.first, false);
        }
    }

    //Update command filtering and send command to SEIKO
    for (auto& it : _commands) {
        const ContactBase_t& contact = this->stateContactBase(it.first);
        if (contact.isEnabled) {
            //Reset filters to current pose for enabled contact
            it.second.filterPose.reset(
                this->_model->position(it.first, "ROOT"),
                this->_model->orientation(it.first, "ROOT"));
        } else {
            //Update filtering and send command 
            //to SEIKO for enabled contact
            it.second.filterPose.update(
                dt_task, 
                it.second.isClutch,
                it.second.rawPos, it.second.rawMat,
                it.second.velLin, it.second.velAng,
                this->_model->position(it.first, "ROOT"),
                this->_model->orientation(it.first, "ROOT"));
            this->refTargetPos(it.first) = it.second.filterPose.valuePos();
            this->refTargetMat(it.first) = it.second.filterPose.valueMat();
        }
        it.second.velLin.setZero();
        it.second.velAng.setZero();
    }

    //Compute SEIKO step
    bool isSuccess = this->runSEIKO(dt_task);

    if (isSuccess) {
        //Apply smoothing ratio on all (linear) computed delta
        _initRatio = ClampRange(_initRatio, 0.0, 1.0);
        this->_jointComputedDeltaDOF *= _initRatio;
        this->_jointComputedDeltaTau *= _initRatio;
        for (size_t i=0;i<this->_contactsPlane.size();i++) {
            this->_contactsPlane[i].computedDeltaWrench *= _initRatio;
        }
        for (size_t i=0;i<this->_contactsPoint.size();i++) {
            this->_contactsPoint[i].computedDeltaForce *= _initRatio;
        }
        //Update initialization smoothing
        _initRatio += dt_task/2.0;
        _initRatio = ClampRange(_initRatio, 0.0, 1.0);
        //Integrate computed deltas
        this->integrateComputedDelta(dt_task);
        //Update underlying model
        this->_model->updateState();
        //Compute ahead command from current delta as velocity
        computeAheadJointPos(dt_ahead);
    }

    return isSuccess;
}

const Eigen::VectorXd& SEIKOWrapper::getAheadJointPos() const
{
    return _aheadJointPos;
}

const SEIKOWrapper::EndEffector_t& SEIKOWrapper::stateEffector(
    const std::string& frameName) const
{
    if (_commands.count(frameName) == 0) {
        throw std::logic_error(
            "leph::SEIKOWrapper::stateEffector: "
            "Undeclared frame name: " + frameName);
    }
    return _commands.at(frameName);
}

bool SEIKOWrapper::stateIsContactSwitching(
    const std::string& frameName) const
{
    if (_commands.count(frameName) == 0) {
        throw std::logic_error(
            "leph::SEIKOWrapper::stateIsContactSwitching: "
            "Undeclared frame name: " + frameName);
    }
    const ContactBase_t& contact = this->stateContactBase(frameName);
    const EndEffector_t& effector = _commands.at(frameName);
    bool isDisabling = 
        contact.isEnabled && 
        effector.isSwitchingDisable;
    bool isEnabling = 
        contact.isEnabled && 
        !effector.isSwitchingDisable && 
        contact.normalForceMin < effector.limitNormalForceMin;
    return isDisabling || isEnabling;
}

void SEIKOWrapper::addContact(
    const std::string nameFrame, 
    bool isPoint)
{
    if (isPoint) {
        this->addContactPoint(nameFrame);
    } else {
        this->addContactPlane(nameFrame);
    }
    _commands.insert(std::make_pair(nameFrame, EndEffector_t{
        nameFrame, isPoint,
        false, 0.0, 0.0, 0.0, 0.0,
        false, 
        Eigen::Vector3d::Zero(),
        Eigen::Matrix3d::Identity(),
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero(),
        FilterPoseCommand(),
    }));
}

void SEIKOWrapper::computeAheadJointPos(double dt)
{
    //Reset to last computed state
    _aheadJointPos = this->_model->getJointPosVect();
    //Retrieve computed joint velocity
    const auto& jointVel = this->_model->getJointVelVect();
    //Integrate time step
    _aheadJointPos += dt*jointVel;
}        

}

