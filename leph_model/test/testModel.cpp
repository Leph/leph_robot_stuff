#include <iostream>
#include <leph_model/Model.hpp>
#include <leph_utils/Filesystem.h>

int main()
{
    leph::Model model(
        leph::SystemResolvePath(
            "package://leph_talos_description/urdf/talos_fast_no_gripper.urdf"));

    std::cout << "Mapping DoFs:" << std::endl;
    for (const auto& it : model.getMappingDOFs()) {
        std::cout << it.first << ": " << it.second << std::endl;
    }
    std::cout << std::endl;
    std::cout << "Mapping Frames:" << std::endl;
    for (const auto& it : model.getMappingFrames()) {
        std::cout << it.first << ": " << it.second << std::endl;
    }
    std::cout << std::endl;
    std::cout << "Visuals:" << std::endl;
    for (const auto& it : model.getMappingVisuals()) {
        std::cout << it.first << ": " << it.second.meshFilename << std::endl;
    }
    std::cout << std::endl;
    std::cout << "sizeDOF:    " << model.sizeDOF() << std::endl;
    std::cout << "sizeJoint:  " << model.sizeJoint() << std::endl;
    std::cout << "sizeVectPos:" << model.sizeVectPos() << std::endl;
    std::cout << "sizeVectVel:" << model.sizeVectVel() << std::endl;
    std::cout << "Mass: " << model.massSum() << std::endl;

    return 0;
}

