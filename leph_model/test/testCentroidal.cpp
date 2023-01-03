#include <iostream>
#include <Eigen/Dense>
#include <leph_model/Model.hpp>
#include <leph_utils/Filesystem.h>

template <typename T, typename U>
inline void checkEqual(T v1, U v2, const std::string& msg = "")
{
    if ((v1-v2).norm() > 1e-6) {
        std::cout
            << "Test [" << msg << "] failed! "
            << "error=" << (v1-v2).norm() << std::endl;
    }
}

int main()
{
    leph::Model model(
        leph::SystemResolvePath(
            "package://leph_talos_description/urdf/talos_fast_no_gripper.urdf"),
        "base_link");

    double mass = model.massSum();
    std::cout << "Total mass: " << mass << std::endl;

    {
        model.setBaseToMatchFramePose(
            "base_link", Eigen::Vector3d::Zero(),
            Eigen::Vector3d(0.0, 0.0, 0.0),
            Eigen::Quaterniond(Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ())));
        model.updateState();
        Eigen::MatrixXd H;
        Eigen::VectorXd C;
        Eigen::VectorXd G;
        model.computeEquationOfMotion(H, C, &G);
        Eigen::MatrixXd CMM;
        Eigen::VectorXd CMMDotQDot;
        model.computeCentroidalDynamics(H, C, G, CMM, CMMDotQDot);
        std::cout << "-----" << std::endl;
        std::cout << "CMM:" << std::endl;
        std::cout << CMM.block(0, 0, 6, 6) << std::endl;

        for (int i=0;i<6;i++) {
            Eigen::VectorXd vel = Eigen::VectorXd::Zero(model.sizeVectVel());
            vel(i) = 1.0;
            model.setDOFVelVect(vel);
            model.updateState();
            leph::RBDLMath::Vector3d currentCoMPosInWorld;
            leph::RBDLMath::Vector3d currentCoMVelInWorld;
            leph::RBDLMath::Vector3d currentCoMMomentumAngular;
            leph::RBDL::Utils::CalcCenterOfMass(
                model.getRBDLModel(), 
                model.getDOFPosVect(), model.getDOFVelVect(), nullptr, 
                mass, 
                currentCoMPosInWorld, &currentCoMVelInWorld, nullptr, 
                &currentCoMMomentumAngular, nullptr, false);
            std::cout 
                << i << ": " 
                << currentCoMMomentumAngular.transpose() << " " 
                << (mass*currentCoMVelInWorld).transpose() << std::endl;
            checkEqual(currentCoMMomentumAngular, CMM.block(0, i, 3, 1));
            checkEqual(mass*currentCoMVelInWorld, CMM.block(3, i, 3, 1));
        }
    }
    {
        model.setBaseToMatchFramePose(
            "base_link", Eigen::Vector3d::Zero(),
            Eigen::Vector3d(1.0, 0.5, 0.2),
            Eigen::Quaterniond(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ())));
        model.updateState();
        Eigen::MatrixXd H;
        Eigen::VectorXd C;
        Eigen::VectorXd G;
        model.computeEquationOfMotion(H, C, &G);
        Eigen::MatrixXd CMM;
        Eigen::VectorXd CMMDotQDot;
        model.computeCentroidalDynamics(H, C, G, CMM, CMMDotQDot);
        std::cout << "-----" << std::endl;
        std::cout << "CMM:" << std::endl;
        std::cout << CMM.block(0, 0, 6, 6) << std::endl;
        
        for (int i=0;i<6;i++) {
            Eigen::VectorXd vel = Eigen::VectorXd::Zero(model.sizeVectVel());
            vel(i) = 1.0;
            model.setDOFVelVect(vel);
            model.updateState();
            leph::RBDLMath::Vector3d currentCoMPosInWorld;
            leph::RBDLMath::Vector3d currentCoMVelInWorld;
            leph::RBDLMath::Vector3d currentCoMMomentumAngular;
            leph::RBDL::Utils::CalcCenterOfMass(
                model.getRBDLModel(), 
                model.getDOFPosVect(), model.getDOFVelVect(), nullptr, 
                mass, 
                currentCoMPosInWorld, &currentCoMVelInWorld, nullptr, 
                &currentCoMMomentumAngular, nullptr, false);
            std::cout 
                << i << ": " 
                << currentCoMMomentumAngular.transpose() << " " 
                << (mass*currentCoMVelInWorld).transpose() << std::endl;
            checkEqual(currentCoMMomentumAngular, CMM.block(0, i, 3, 1));
            checkEqual(mass*currentCoMVelInWorld, CMM.block(3, i, 3, 1));
        }
    }

    return 0;
}

