#include <iostream>
#include <leph_model/Model.hpp>
#include <leph_utils/Filesystem.h>

int main()
{
    leph::Model model(
        leph::SystemResolvePath(
            "package://leph_talos_description/urdf/talos_fast_no_gripper.urdf"),
        "base_link");
    model.setDOFPos("leg_left_1_joint", M_PI);
    model.setDOFPos("leg_left_5_joint", -0.6);
    model.setDOFPos("leg_left_4_joint", 0.0);
    model.setDOFPos("leg_left_3_joint", 0.6);
    model.updateState();

    Eigen::Matrix3d mat = model.orientation("base_link", "left_sole_link");
    Eigen::Vector3d vec = model.position("base_link", "left_sole_link");
    std::cout << "Rotation SrcInDst:" << std::endl;
    std::cout << mat << std::endl;
    std::cout << "Translation SrcInDst:" << std::endl;
    std::cout << vec.transpose() << std::endl;
    std::cout << "==== Wrench Transform:" << std::endl;
    Eigen::Matrix6d tmpTransform;
    for (unsigned int i=0;i<6;i++) {
        Eigen::Vector6d wrench = Eigen::Vector6d::Zero();
        wrench(i) = 1.0;
        tmpTransform.col(i) = model.wrenchTransform("base_link", "left_sole_link", wrench);
    }
    std::cout << tmpTransform << std::endl;
    std::cout << "==== Wrench Transform:" << std::endl;
    std::cout << model.getWrenchTransform(
        model.position("left_sole_link", "base_link"), 
        model.orientation("left_sole_link", "base_link")) << std::endl;

    return 0;
}

