#include <iostream>
#include <Eigen/Dense>
#include <leph_model/Model.hpp>
#include <leph_utils/Filesystem.h>
#include <leph_maths/IntegrateDOFVect.h>
#include <leph_maths/AxisAngle.h>

template <typename T, typename U>
inline void checkEqual(T v1, U v2, const std::string& msg = "")
{
    if ((v1-v2).norm() > 1e-4) {
        std::cout
            << "Test [" << msg << "] failed! "
            << "error=" << (v1-v2).norm() << std::endl;
    }
}

void testJacobian(const std::string frameSrc, const std::string& frameDst)
{
    leph::Model model(
        leph::SystemResolvePath(
            "package://leph_talos_description/urdf/talos_fast_no_gripper.urdf"),
        "base_link");
    model.setDOFPos("leg_left_1_joint", M_PI_4);
    model.setDOFPos("leg_left_5_joint", -0.6);
    model.setDOFPos("leg_left_4_joint", 0.0);
    model.setDOFPos("leg_left_3_joint", 0.3);
    model.setDOFPos("leg_right_1_joint", -0.5);
    model.setDOFPos("leg_right_5_joint", -0.2);
    model.setDOFPos("leg_right_4_joint", 0.0);
    model.setDOFPos("leg_right_3_joint", 0.1);
    model.updateState();
    model.setBaseToMatchFramePose(
        "base_link", Eigen::Vector3d::Zero(),
        Eigen::Vector3d(0.5, 0.2, 0.0),
        Eigen::Quaterniond(Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ())));
    model.updateState();

    Eigen::MatrixXd jacobianDiff = Eigen::MatrixXd::Zero(6, model.sizeVectVel());
    Eigen::MatrixXd jacobianModel = model.pointJacobian(frameSrc, frameDst);
    Eigen::VectorXd statePosition = model.getDOFPosVect();
    const double delta = 1e-6;
    for (size_t i=0;i<model.sizeVectVel();i++) {
        Eigen::VectorXd tmpVel = Eigen::VectorXd::Zero(model.sizeVectVel());
        tmpVel(i) += delta;
        Eigen::VectorXd tmpPos = leph::IntegrateDOFVect(statePosition, tmpVel);
        //Sample first evaluation
        model.setDOFPosVect(tmpPos);
        model.updateState();
        Eigen::Vector3d pos1 = model.position(frameSrc, frameDst);
        Eigen::Matrix3d mat1 = model.orientation(frameSrc, frameDst);
        //Sample second evaluation
        model.setDOFPosVect(statePosition);
        model.updateState();
        Eigen::Vector3d pos2 = model.position(frameSrc, frameDst);
        Eigen::Matrix3d mat2 = model.orientation(frameSrc, frameDst);
        //Compute Jacobian
        jacobianDiff.block(0,i,3,1) = (1.0/delta)*leph::MatrixToAxis(mat1*mat2.transpose());
        jacobianDiff.block(3,i,3,1) = (1.0/delta)*(pos1 - pos2);
    }
    model.setDOFPosVect(statePosition);
    model.updateState();
    std::cout << "------" << std::endl;
    std::cout << "model:" << std::endl;
    std::cout << jacobianModel << std::endl;
    std::cout << "diff:" << std::endl;
    std::cout << jacobianDiff << std::endl;
    checkEqual(jacobianModel, jacobianDiff, 
        "jacobian " + frameSrc + std::string(" ") + frameDst);
}

void testJacobianBody(const std::string frame)
{
    leph::Model model(
        leph::SystemResolvePath(
            "package://leph_talos_description/urdf/talos_fast_no_gripper.urdf"),
        "base_link");
    model.setDOFPos("leg_left_1_joint", M_PI_4);
    model.setDOFPos("leg_left_5_joint", -0.6);
    model.setDOFPos("leg_left_4_joint", 0.0);
    model.setDOFPos("leg_left_3_joint", 0.3);
    model.setDOFPos("leg_right_1_joint", -0.5);
    model.setDOFPos("leg_right_5_joint", -0.2);
    model.setDOFPos("leg_right_4_joint", 0.0);
    model.setDOFPos("leg_right_3_joint", 0.1);
    model.updateState();
    model.setBaseToMatchFramePose(
        "base_link", Eigen::Vector3d::Zero(),
        Eigen::Vector3d(0.5, 0.2, 0.0),
        Eigen::Quaterniond(Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ())));
    model.updateState();

    Eigen::MatrixXd jacobianDiff = Eigen::MatrixXd::Zero(6, model.sizeVectVel());
    Eigen::MatrixXd jacobianModel = model.pointJacobian(frame, frame);
    Eigen::VectorXd statePosition = model.getDOFPosVect();
    const double delta = 1e-6;
    for (size_t i=0;i<model.sizeVectVel();i++) {
        Eigen::VectorXd tmpVel = Eigen::VectorXd::Zero(model.sizeVectVel());
        tmpVel(i) += delta;
        Eigen::VectorXd tmpPos = leph::IntegrateDOFVect(statePosition, tmpVel);
        //Sample first evaluation
        model.setDOFPosVect(tmpPos);
        model.updateState();
        Eigen::Vector3d pos1 = model.position(frame, "ROOT");
        Eigen::Matrix3d mat1 = model.orientation(frame, "ROOT");
        //Sample second evaluation
        model.setDOFPosVect(statePosition);
        model.updateState();
        Eigen::Vector3d pos2 = model.position(frame, "ROOT");
        Eigen::Matrix3d mat2 = model.orientation(frame, "ROOT");
        //Compute Jacobian
        jacobianDiff.block(0,i,3,1) = mat2.transpose()*((1.0/delta)*leph::MatrixToAxis(mat1*mat2.transpose()));
        jacobianDiff.block(3,i,3,1) = mat2.transpose()*((1.0/delta)*(pos1 - pos2));
    }
    model.setDOFPosVect(statePosition);
    model.updateState();
    std::cout << "------" << std::endl;
    std::cout << "model:" << std::endl;
    std::cout << jacobianModel << std::endl;
    std::cout << "diff:" << std::endl;
    std::cout << jacobianDiff << std::endl;
    checkEqual(jacobianModel, jacobianDiff, 
        "jacobian body " + frame);
}

int main()
{
    testJacobian("left_sole_link", "ROOT");
    testJacobianBody("left_sole_link");
    testJacobian("left_sole_link", "right_sole_link");

    return 0;
}

