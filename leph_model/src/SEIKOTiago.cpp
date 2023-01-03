#include <leph_model/SEIKOTiago.hpp>
#include <leph_maths/Angle.h>

namespace leph {

void SEIKOTiago::setup()
{
    //Frame names
    _nameHand = "gripper_tip_frame";
    _nameBase = "base_footprint";
    
    //Setup contacts
    this->addContact(_nameBase, false);
    this->addContact(_nameHand, true);
    
    //Call to base setup
    this->SEIKOWrapper::setup();
    
    //Specific configuration for prismatic joint limit margin
    _jointPosMarginCoef.setOnes();
    _jointPosMarginCoef(this->_model->getIndexJoint("torso_lift_joint")) = 0.2;
    _jointPosMarginCoef(this->_model->getIndexJoint("arm_1_joint")) = 0.1;
}
        
bool SEIKOTiago::reset()
{
    //Assign SEIKO and filters configuration
    //SEIKO parameters
    double jointPosMargin = DegToRad(15.0);
    double jointVelLimitRatio = 0.4;
    double jointTauLimitRatio = 0.8;
    double jointWeightPos = 1.0;
    double jointWeightVel = 10000.0;
    double jointWeightTau = 0.000001;
    double jointClampPos = DegToRad(10.0);
    double weightPosBase = 1000.0;
    double weightPosHand = 1000.0;
    double weightMatBase = 1000.0;
    double weightMatHand = 40.0;
    double weightWrenchEnabled = 0.0001;
    double weightForceEnabled = 0.0001;
    double weightWrenchDisabling = 1e3;
    double weightForceDisabling = 1e3;
    double forceMinBase = 1.0;
    double forceMaxBase = 1000.0;
    double forceMinHand = 1.0;
    double forceMaxHand = 100.0;
    double limitVelNormalForceBase = 1000.0;
    double limitVelNormalForceHand = 100.0;
    double frictionCoef = 0.5;
    double clampPos = 0.02;
    double clampMat = DegToRad(10.0);

    //Pose filter parameters
    bool isLocalFrame = true;
    double scalingLin = 2.0;
    double clampRadiusLin = 0.02;
    double clampRadiusAng = 0.1;
    double cutoffFreq = 10.0;
    double maxVelLin = 0.2;
    double maxAccLin = 2.0;
    double maxVelAng = 1.0;
    double maxAccAng = 10.0;
    
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
        jointVelLimitRatio*this->_model->jointLimitsVelocity());
    this->setJointLimitTauAbs(
        jointTauLimitRatio*this->_model->jointLimitsTorque());
    this->setJointWeightPos(
        jointWeightPos*Eigen::VectorXd::Ones(this->_sizeJoint));
    this->setJointWeightVel(
        jointWeightVel * 
        (1.0/this->_model->jointLimitsVelocity().maxCoeff())*this->_model->jointLimitsVelocity());
    this->setJointWeightTau(
        jointWeightTau*Eigen::VectorXd::Ones(this->_sizeJoint));
    this->setJointClampPos(
        jointClampPos);

    //Cartesian pose parameters
    this->setWeightPose(_nameBase,
        weightPosBase, weightMatBase);
    this->setWeightPose(_nameHand,
        weightPosHand, weightMatHand);
    this->setClampPose(_nameBase,
        clampPos, clampMat);
    this->setClampPose(_nameHand,
        clampPos, clampMat);
    this->setContactLimitFriction(_nameBase,
        frictionCoef);
    this->setContactLimitFriction(_nameHand,
        frictionCoef);
    this->setNormalForceLimits(_nameBase,
        forceMinBase, forceMaxBase);
    this->setNormalForceLimits(_nameHand,
        forceMinHand, forceMaxHand);
    this->setLimitVelNormalForce(_nameBase,
        limitVelNormalForceBase);
    this->setLimitVelNormalForce(_nameHand,
        limitVelNormalForceHand);

    //Plane contact parameters
    this->setContactPlaneLimitCOP(_nameBase,
        0.25, 0.25);
    this->setWeightWrench(_nameBase,
        weightWrenchEnabled*_vectWeightWrench);

    //Point contact parameters
    this->setWeightForce(_nameHand,
        weightForceEnabled*_vectWeightForce);

    //Contact command structure parameters
    for (auto& it : _commands) {
        if (it.second.isPoint) {
            it.second.limitNormalForceMin = forceMinHand;
            it.second.limitNormalForceMax = forceMaxHand;
            it.second.weightForceEnabled = weightForceEnabled;
            it.second.weightForceDisabling = weightForceDisabling;
        } else {
            it.second.limitNormalForceMin = forceMinBase;
            it.second.limitNormalForceMax = forceMaxBase;
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
    this->toggleContact(_nameHand, false);
    this->toggleContact(_nameBase, true);

    //Call base reset
    bool isIDSuccess = this->SEIKOWrapper::reset();

    //Override default posture
    this->setJointTargetPos(
        0.5*this->_model->jointLimitsLower() + 
        0.5*this->_model->jointLimitsUpper());

    return isIDSuccess;
}
        
const std::string& SEIKOTiago::nameFrameHand() const
{
    return _nameHand;
}
const std::string& SEIKOTiago::nameFrameBase() const
{
    return _nameBase;
}

}

