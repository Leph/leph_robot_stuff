#include <leph_model/ModelHumanoidFixed.hpp>

namespace leph {

ModelHumanoidFixed::ModelHumanoidFixed(
    const std::string& filename,
    const std::string& nameFootLeft,
    const std::string& nameFootRight) :
    _nameFootLeft(nameFootLeft),
    _nameFootRight(nameFootRight),
    _modelLeft(filename, nameFootLeft),
    _modelRight(filename, nameFootRight),
    _isSupportLeft(true)
{
}
        
bool ModelHumanoidFixed::isSupportLeft() const
{
    return _isSupportLeft;
}
        
void ModelHumanoidFixed::setSupport(bool isLeft)
{
    if (_isSupportLeft == isLeft) {
        return;
    }

    Eigen::Vector3d footPos;
    Eigen::Matrix3d footMat;
    if (_isSupportLeft) {
        //Update the model if needed
        if (_modelLeft.isDirty()) {
            _modelLeft.updateState();
        }
        //Compute current flying foot 
        //pose in world frame
        footPos = _modelLeft.position(_nameFootRight, "ROOT");
        footMat = _modelLeft.orientation(_nameFootRight, "ROOT");
        //Copy degrees of freedom state
        _modelRight.importState(_modelLeft);
        //Assign base pose
        _modelRight.setBasePos(footPos);
        _modelRight.setBaseQuat(Eigen::Quaterniond(footMat));
        _isSupportLeft = false;
    } else {
        //Update the model if needed
        if (_modelRight.isDirty()) {
            _modelRight.updateState();
        }
        //Compute current flying foot 
        //pose in world frame
        footPos = _modelRight.position(_nameFootLeft, "ROOT");
        footMat = _modelRight.orientation(_nameFootLeft, "ROOT");
        //Copy degrees of freedom state
        _modelLeft.importState(_modelRight);
        //Assign base pose
        _modelLeft.setBasePos(footPos);
        _modelLeft.setBaseQuat(Eigen::Quaterniond(footMat));
        _isSupportLeft = true;
    }
}

const Model& ModelHumanoidFixed::get() const
{
    if (_isSupportLeft) {
        return _modelLeft;
    } else {
        return _modelRight;
    }
}
Model& ModelHumanoidFixed::get()
{
    if (_isSupportLeft) {
        return _modelLeft;
    } else {
        return _modelRight;
    }
}

const Model& ModelHumanoidFixed::getModelLeft() const
{
    return _modelLeft;
}
Model& ModelHumanoidFixed::getModelLeft()
{
    return _modelLeft;
}
const Model& ModelHumanoidFixed::getModelRight() const
{
    return _modelRight;
}
Model& ModelHumanoidFixed::getModelRight()
{
    return _modelRight;
}

}

