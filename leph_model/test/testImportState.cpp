#include <iostream>
#include <leph_model/Model.hpp>
#include <leph_utils/Filesystem.h>
#include <leph_model/TalosDOFs.h>

int main()
{
    //Models initialization
    leph::Model modelFixed(
        leph::SystemResolvePath(
            "package://leph_talos_description/urdf/talos_fast_no_gripper.urdf"),
        "left_sole_link");
    leph::Model modelFloat(
        leph::SystemResolvePath(
            "package://leph_talos_description/urdf/talos_fast_no_gripper.urdf"),
        "base_link");
    
    //Set state for fixed model
    for (const auto& it : modelFixed.getMappingDOFs()) {
        if (leph::PostureDefault.count(it.first) != 0) {
            modelFixed.setDOFPos(it.first, leph::PostureDefault.at(it.first));
        }
    }
    for (size_t i=0;i<modelFixed.sizeDOF();i++) {
        modelFixed.setDOFVel(i, 0.05+0.01*i);
    }
    modelFixed.updateState();
    modelFixed.setBaseToMatchFramePose(
        "left_sole_link",
        Eigen::Vector3d(0.0, 0.0, 0.0),
        Eigen::Vector3d(0.3, 0.4, 0.0),
        Eigen::Quaterniond(Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ())));
    modelFixed.updateState();

    //Import and convert state from fixed to float model
    modelFloat.importState(modelFixed);
    modelFloat.updateState();

    //Check position and velocity in world frame
    Eigen::Vector3d posFixed = modelFixed.position("wrist_left_ft_link", "ROOT");
    Eigen::Vector3d posFloat = modelFloat.position("wrist_left_ft_link", "ROOT");
    Eigen::Vector3d velFixed = modelFixed.pointVelocity("wrist_left_ft_link", "ROOT").segment(3, 3);
    Eigen::Vector3d velFloat = modelFloat.pointVelocity("wrist_left_ft_link", "ROOT").segment(3, 3);
    std::cout << posFixed.transpose() << " -- vs -- " << posFloat.transpose() << std::endl;
    std::cout << velFixed.transpose() << " -- vs -- " << velFloat.transpose() << std::endl;

    return 0;
}

