#include <leph_model/SEIKOTalos.hpp>
#include <leph_model/TalosDOFs.h>
#include <leph_maths/Angle.h>

namespace leph {

void SEIKOTalos::setup()
{
    //Frame names
    _nameFootLeft = "left_sole_link";
    _nameFootRight = "right_sole_link";
    _nameHandLeft = "left_hand_frame";
    _nameHandRight = "right_hand_frame";
    
    //Setup contacts
    this->addContact(_nameFootLeft, false);
    this->addContact(_nameFootRight, false);
    this->addContact(_nameHandLeft, true);
    this->addContact(_nameHandRight, true);

    //Call to base setup
    this->SEIKOWrapper::setup();
    
    //Specific configuration for joint angular limit margin
    _jointPosMarginCoef.setOnes();
    if (this->_model->getMappingDOFs().count("head_1_joint") != 0) {
        _jointPosMarginCoef(this->_model->getIndexJoint("head_1_joint")) = 0.1;
    }
    if (this->_model->getMappingDOFs().count("head_2_joint") != 0) {
        _jointPosMarginCoef(this->_model->getIndexJoint("head_2_joint")) = 0.1;
    }
    if (this->_model->getMappingDOFs().count("torso_2_joint") != 0) {
        _jointPosMarginCoef(this->_model->getIndexJoint("torso_2_joint")) = 0.7;
    }
}
        
bool SEIKOTalos::reset()
{
    //Assign SEIKO and filters configuration
    //SEIKO parameters
    double jointPosMargin = DegToRad(15.0);
    double jointVelLimit = 0.2;
    double jointTauLimitRatio = 0.8;
    double jointWeightPos = 2.0;
    double jointWeightVel = 100000.0;
    double jointWeightTau = 0.00002;
    double jointClampPos = DegToRad(10.0);
    double weightPosFoot = 1000.0;
    double weightPosHand = 1000.0;
    double weightMatFoot = 20.0;
    double weightMatHand = 20.0;
    double weightWrenchEnabled = 0.001;
    double weightForceEnabled = 0.001;
    double weightWrenchDisabling = 1e3;
    double weightForceDisabling = 1e3;
    double forceMinFoot = 100.0;
    double forceMaxFoot = 1000.0;
    double forceMinHand = 10.0;
    double forceMaxHand = 200.0;
    double limitVelNormalForceFoot = 100.0;
    double limitVelNormalForceHand = 10.0;
    double frictionCoef = 0.6;
    double clampPos = 0.02;
    double clampMat = DegToRad(10.0);
    double limitCOPX = 0.04;
    double limitCOPY = 0.03;

    //Pose filter parameters
    bool isLocalFrame = false;
    double scalingLin = 1.0;
    double clampRadiusLin = 0.03;
    double clampRadiusAng = 0.2;
    double cutoffFreq = 2.0;
    double maxVelLin = 0.2;
    double maxAccLin = 0.4;
    double maxVelAng = 0.5;
    double maxAccAng = 1.0;
    
    //Contact wrench and force weight vectors
    //normalization. Put less (regularization) weight 
    //on normal forces.
    _vectWeightWrench(0) = 1.0;
    _vectWeightWrench(1) = 1.0;
    _vectWeightWrench(2) = 1.0;
    _vectWeightWrench(3) = 1.0;
    _vectWeightWrench(4) = 1.0;
    _vectWeightWrench(5) = 0.01;
    _vectWeightForce(0) = 1.0;
    _vectWeightForce(1) = 1.0;
    _vectWeightForce(2) = 0.01;
    
    //Joint parameters
    this->setJointLimitPos(
        this->_model->jointLimitsLower() + jointPosMargin*_jointPosMarginCoef,
        this->_model->jointLimitsUpper() - jointPosMargin*_jointPosMarginCoef);
    this->setJointLimitVelAbs(
        jointVelLimit*Eigen::VectorXd::Ones(this->_sizeJoint));
    this->setJointLimitTauAbs(
        jointTauLimitRatio*this->_model->jointLimitsTorque());
    this->setJointWeightPos(
        jointWeightPos*Eigen::VectorXd::Ones(this->_sizeJoint));
    this->setJointWeightVel(
        jointWeightVel*Eigen::VectorXd::Ones(this->_sizeJoint));
    this->setJointWeightTau(
        jointWeightTau*Eigen::VectorXd::Ones(this->_sizeJoint));
    this->setJointClampPos(
        jointClampPos);

    //Cartesian pose parameters
    this->setWeightPose(_nameFootLeft,
        weightPosFoot, weightMatFoot);
    this->setWeightPose(_nameFootRight,
        weightPosFoot, weightMatFoot);
    this->setWeightPose(_nameHandLeft,
        weightPosHand, weightMatHand);
    this->setWeightPose(_nameHandRight,
        weightPosHand, weightMatHand);
    this->setClampPose(_nameFootLeft,
        clampPos, clampMat);
    this->setClampPose(_nameFootRight,
        clampPos, clampMat);
    this->setClampPose(_nameHandLeft,
        clampPos, clampMat);
    this->setClampPose(_nameHandRight,
        clampPos, clampMat);
    this->setContactLimitFriction(_nameFootLeft,
        frictionCoef);
    this->setContactLimitFriction(_nameFootRight,
        frictionCoef);
    this->setContactLimitFriction(_nameHandLeft,
        frictionCoef);
    this->setContactLimitFriction(_nameHandRight,
        frictionCoef);
    this->setNormalForceLimits(_nameFootLeft,
        forceMinFoot, forceMaxFoot);
    this->setNormalForceLimits(_nameFootRight,
        forceMinFoot, forceMaxFoot);
    this->setNormalForceLimits(_nameHandLeft,
        forceMinHand, forceMaxHand);
    this->setNormalForceLimits(_nameHandRight,
        forceMinHand, forceMaxHand);
    this->setLimitVelNormalForce(_nameFootLeft,
        limitVelNormalForceFoot);
    this->setLimitVelNormalForce(_nameFootRight,
        limitVelNormalForceFoot);
    this->setLimitVelNormalForce(_nameHandLeft,
        limitVelNormalForceHand);
    this->setLimitVelNormalForce(_nameHandRight,
        limitVelNormalForceHand);

    //Plane contact parameters
    this->setContactPlaneLimitCOP(_nameFootLeft,
        limitCOPX, limitCOPY);
    this->setContactPlaneLimitCOP(_nameFootRight,
        limitCOPX, limitCOPY);
    this->setWeightWrench(_nameFootLeft,
        weightWrenchEnabled*_vectWeightWrench);
    this->setWeightWrench(_nameFootRight,
        weightWrenchEnabled*_vectWeightWrench);

    //Point contact parameters
    this->setWeightForce(_nameHandLeft,
        weightForceEnabled*_vectWeightForce);
    this->setWeightForce(_nameHandRight,
        weightForceEnabled*_vectWeightForce);

    //Contact command structure parameters
    for (auto& it : _commands) {
        if (it.second.isPoint) {
            it.second.limitNormalForceMin = forceMinHand;
            it.second.limitNormalForceMax = forceMaxHand;
            it.second.weightForceEnabled = weightForceEnabled;
            it.second.weightForceDisabling = weightForceDisabling;
        } else {
            it.second.limitNormalForceMin = forceMinFoot;
            it.second.limitNormalForceMax = forceMaxFoot;
            it.second.weightForceEnabled = weightWrenchEnabled;
            it.second.weightForceDisabling = weightWrenchDisabling;
        }
        //Configure command pose filters
        it.second.filterPose.setParameters(
            isLocalFrame,
            scalingLin,
            clampRadiusLin, clampRadiusAng,
            cutoffFreq,
            maxVelLin, maxAccLin,
            maxVelAng, maxAccAng);
    }
    
    //Reset default contact state
    this->toggleContact(_nameFootLeft, true);
    this->toggleContact(_nameFootRight, true);
    this->toggleContact(_nameHandLeft, false);
    this->toggleContact(_nameHandRight, false);

    //Call base reset
    return this->SEIKOWrapper::reset();
}
        
const std::string& SEIKOTalos::nameFrameFootLeft() const
{
    return _nameFootLeft;
}
const std::string& SEIKOTalos::nameFrameFootRight() const
{
    return _nameFootRight;
}
const std::string& SEIKOTalos::nameFrameHandLeft() const
{
    return _nameHandLeft;
}
const std::string& SEIKOTalos::nameFrameHandRight() const
{
    return _nameHandRight;
}

}

