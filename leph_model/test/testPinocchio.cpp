#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <pinocchio/multibody/model.hpp>
#pragma GCC diagnostic pop
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/kinematics-derivatives.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/rnea-derivatives.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <iostream>
#include <string>
#include <leph_model/TalosDOFs.h>
#include <leph_model/Model.hpp>
#include <leph_model/PinocchioInterface.hpp>
#include <leph_utils/Filesystem.h>
#include <leph_utils/Chrono.hpp>
#include <leph_maths/IntegrateDOFVect.h>

template <typename T, typename U>
inline void checkEqual(T v1, U v2, const std::string& msg = "")
{
    if ((v1-v2).norm() > 1e-6) {
        std::cout
            << "Test [" << msg << "] failed! "
            << "error=" << (v1-v2).norm() << std::endl;
    }
}

void testPinocchioInterface()
{
    leph::Chrono chrono;
    
    //Set URDF path
    const std::string modelFilename = leph::SystemResolvePath(
        "package://leph_talos_description/urdf/talos_fast_no_gripper.urdf");
    
    //Load RBDL model
    chrono.start("RBDL load");
    leph::Model modelRBDL(
        modelFilename, "base_link");
    chrono.stop("RBDL load");
    //Define test posture
    for (const auto& it : modelRBDL.getMappingDOFs()) {
        if (leph::PostureDefault.count(it.first) != 0) {
            modelRBDL.setDOFPos(it.first, leph::PostureDefault.at(it.first));
        }
    }
    modelRBDL.updateState();
    modelRBDL.setBaseToMatchFramePose(
        "left_sole_link",
        Eigen::Vector3d(0.0, 0.0, 0.0),
        Eigen::Vector3d(0.3, 0.0, 0.0),
        Eigen::Quaterniond(
            Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ())
            * Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitY())));
    chrono.start("RBDL kinematics");
    modelRBDL.updateState();
    chrono.stop("RBDL kinematics");

    //Load model PinocchioInterface
    chrono.start("Pinocchio load");
    leph::PinocchioInterface modelPinocchio(modelRBDL);
    chrono.stop("Pinocchio load");
    chrono.start("Pinocchio kinematics");
    modelPinocchio.updateKinematics(modelRBDL.getDOFPosVect());
    chrono.stop("Pinocchio kinematics");
    
    //Test kinematics
    //Position
    chrono.start("RBDL position");
    Eigen::Vector3d pos1 = modelRBDL.position(
        "wrist_left_ft_link", "left_sole_link", Eigen::Vector3d(0.1, 0.0, -0.1));
    chrono.stop("RBDL position");
    chrono.start("Pinocchio position");
    Eigen::Vector3d pos2 = modelPinocchio.position(
        "wrist_left_ft_link", "left_sole_link", Eigen::Vector3d(0.1, 0.0, -0.1));
    chrono.stop("Pinocchio position");
    checkEqual(pos1, pos2, "position");
    //Orientation
    chrono.start("RBDL orientation");
    Eigen::Matrix3d mat1 = modelRBDL.orientation("wrist_left_ft_link", "left_sole_link");
    chrono.stop("RBDL orientation");
    chrono.start("Pinocchio orientation");
    Eigen::Matrix3d mat2 = modelPinocchio.orientation("wrist_left_ft_link", "left_sole_link");
    chrono.stop("Pinocchio orientation");
    checkEqual(mat1, mat2, "orientation");

    //Test Jacobian world
    chrono.start("RBDL jacobian");
    Eigen::MatrixXd jac1 = modelRBDL.pointJacobian("wrist_left_ft_link", "ROOT");
    chrono.stop("RBDL jacobian");
    chrono.start("Pinocchio jacobian");
    Eigen::MatrixXd jac2 = modelPinocchio.frameJacobian("wrist_left_ft_link", "ROOT");
    chrono.stop("Pinocchio jacobian");
    checkEqual(jac1, jac2, "jacobian world");
    
    //Test Jacobian local
    Eigen::MatrixXd jac3 = modelRBDL.pointJacobian("wrist_left_ft_link", "wrist_left_ft_link");
    Eigen::MatrixXd jac4 = modelPinocchio.frameJacobian("wrist_left_ft_link", "wrist_left_ft_link");
    checkEqual(jac3, jac4, "jacobian local");
    Eigen::MatrixXd jac5 = modelRBDL.pointJacobian("ROOT", "ROOT");
    Eigen::MatrixXd jac6 = modelPinocchio.frameJacobian("ROOT", "ROOT");
    checkEqual(jac5, jac6, "jacobian local zero");
    
    //Test Jacobian inverted
    Eigen::MatrixXd jac7 = modelRBDL.pointJacobian("ROOT", "wrist_left_ft_link");
    Eigen::MatrixXd jac8 = modelPinocchio.frameJacobian("ROOT", "wrist_left_ft_link");
    checkEqual(jac7, jac8, "jacobian relative");
    Eigen::MatrixXd jac9 = modelRBDL.pointJacobian("left_sole_link", "wrist_left_ft_link");
    Eigen::MatrixXd jac10 = modelPinocchio.frameJacobian("left_sole_link", "wrist_left_ft_link");
    checkEqual(jac9, jac10, "jacobian relative 2");

    //Test gravity vector
    chrono.start("RBDL gravity");
    Eigen::VectorXd g1 = modelRBDL.computeGravityVector();
    chrono.stop("RBDL gravity");
    chrono.start("Pinocchio gravity");
    Eigen::VectorXd g2 = modelPinocchio.gravityVector();
    chrono.stop("Pinocchio gravity");
    checkEqual(g1, g2, "gravity");

    //Test gravity partial derivatives
    chrono.start("RBDL diff gravity");
    Eigen::MatrixXd diffG1 = Eigen::MatrixXd::Zero(
        modelRBDL.sizeDOF(), modelRBDL.sizeDOF());
    Eigen::VectorXd statePosition = modelRBDL.getDOFPosVect();
    const double delta = 1e-6;
    for (size_t i=0;i<modelRBDL.sizeVectVel();i++) {
        Eigen::VectorXd tmpVel1 = Eigen::VectorXd::Zero(modelRBDL.sizeDOF());
        Eigen::VectorXd tmpVel2 = Eigen::VectorXd::Zero(modelRBDL.sizeDOF());
        tmpVel1(i) += delta;
        tmpVel2(i) -= delta;
        Eigen::VectorXd tmpPos1 = leph::IntegrateDOFVect(statePosition, tmpVel1);
        Eigen::VectorXd tmpPos2 = leph::IntegrateDOFVect(statePosition, tmpVel2);
        //Sample first evaluation
        modelRBDL.setDOFPosVect(tmpPos1);
        modelRBDL.setDOFVelVect(Eigen::VectorXd::Zero(modelRBDL.sizeDOF()));
        modelRBDL.updateState();
        Eigen::VectorXd tmpG1 = modelRBDL.computeGravityVector();
        //Sample second evaluation
        modelRBDL.setDOFPosVect(tmpPos2);
        modelRBDL.setDOFVelVect(Eigen::VectorXd::Zero(modelRBDL.sizeDOF()));
        modelRBDL.updateState();
        Eigen::VectorXd tmpG2 = modelRBDL.computeGravityVector();
        //Compute finite difference
        diffG1.col(i) = 1.0/(2.0*delta)*(tmpG1 - tmpG2);
    }
    modelRBDL.setDOFPosVect(statePosition);
    modelRBDL.setDOFVelVect(Eigen::VectorXd::Zero(modelRBDL.sizeDOF()));
    modelRBDL.updateState();
    chrono.stop("RBDL diff gravity");
    chrono.start("Pinocchio diff gravity");
    Eigen::MatrixXd diffG2 = modelPinocchio.diffGravityVector();
    chrono.stop("Pinocchio diff gravity");
    checkEqual(diffG1, diffG2, "diff gravity");

    //Test Hessian product derivatives in local frame
    Eigen::Vector6d wrenchLocal = Eigen::Vector6d::Zero();
    wrenchLocal(2) = 33.0;
    wrenchLocal(3) = 7.0;
    wrenchLocal(4) = 30.0;
    wrenchLocal(5) = 200.0;
    chrono.start("RBDL diff Hessian local");
    Eigen::MatrixXd hessianProdLocalRBDL = Eigen::MatrixXd::Zero(
        modelRBDL.sizeDOF(), modelRBDL.sizeDOF());
    for (size_t indexDOF=0;indexDOF<modelRBDL.sizeVectVel();indexDOF++) {
        Eigen::VectorXd tmpVel1 = Eigen::VectorXd::Zero(modelRBDL.sizeDOF());
        Eigen::VectorXd tmpVel2 = Eigen::VectorXd::Zero(modelRBDL.sizeDOF());
        tmpVel1(indexDOF) += delta;
        tmpVel2(indexDOF) -= delta;
        Eigen::VectorXd tmpPos1 = leph::IntegrateDOFVect(statePosition, tmpVel1);
        Eigen::VectorXd tmpPos2 = leph::IntegrateDOFVect(statePosition, tmpVel2);
        //Sample first evaluation
        modelRBDL.setDOFPosVect(tmpPos1);
        modelRBDL.setDOFVelVect(Eigen::VectorXd::Zero(modelRBDL.sizeDOF()));
        modelRBDL.updateState();
        Eigen::MatrixXd jac1 = modelRBDL.pointJacobian(
            "left_sole_link", "left_sole_link");
        Eigen::VectorXd vec1 = jac1.transpose()*wrenchLocal;
        //Sample second evaluation
        modelRBDL.setDOFPosVect(tmpPos2);
        modelRBDL.setDOFVelVect(Eigen::VectorXd::Zero(modelRBDL.sizeDOF()));
        modelRBDL.updateState();
        Eigen::MatrixXd jac2 = modelRBDL.pointJacobian(
            "left_sole_link", "left_sole_link");
        Eigen::VectorXd vec2 = jac2.transpose()*wrenchLocal;
        //Compute
        Eigen::MatrixXd vec3 = 1.0/(2.0*delta)*(vec1 - vec2);
        hessianProdLocalRBDL.col(indexDOF) = vec3;
    }
    chrono.stop("RBDL diff Hessian local");
    chrono.start("Pinocchio diff Hessian local");
    Eigen::MatrixXd hessianProdLocalPinocchio = 
        modelPinocchio.diffHessianWrenchInLocalProduct("left_sole_link", wrenchLocal);
    chrono.stop("Pinocchio diff Hessian local");
    checkEqual(hessianProdLocalRBDL, hessianProdLocalPinocchio, "diff Hessian local");
    
    //Test Hessian product derivatives in world frame
    Eigen::Vector6d wrenchWorld = Eigen::Vector6d::Zero();
    wrenchWorld(3) = 7.0;
    wrenchWorld(4) = 30.0;
    wrenchWorld(5) = 200.0;
    chrono.start("RBDL diff Hessian world");
    Eigen::MatrixXd hessianProdWorldRBDL = Eigen::MatrixXd::Zero(
        modelRBDL.sizeDOF(), modelRBDL.sizeDOF());
    for (size_t indexDOF=0;indexDOF<modelRBDL.sizeVectVel();indexDOF++) {
        Eigen::VectorXd tmpVel1 = Eigen::VectorXd::Zero(modelRBDL.sizeDOF());
        Eigen::VectorXd tmpVel2 = Eigen::VectorXd::Zero(modelRBDL.sizeDOF());
        tmpVel1(indexDOF) += delta;
        tmpVel2(indexDOF) -= delta;
        Eigen::VectorXd tmpPos1 = leph::IntegrateDOFVect(statePosition, tmpVel1);
        Eigen::VectorXd tmpPos2 = leph::IntegrateDOFVect(statePosition, tmpVel2);
        //Sample first evaluation
        modelRBDL.setDOFPosVect(tmpPos1);
        modelRBDL.setDOFVelVect(Eigen::VectorXd::Zero(modelRBDL.sizeDOF()));
        modelRBDL.updateState();
        Eigen::MatrixXd jac1 = modelRBDL.pointJacobian(
            "left_sole_link", "ROOT");
        Eigen::VectorXd vec1 = jac1.transpose()*wrenchWorld;
        //Sample second evaluation
        modelRBDL.setDOFPosVect(tmpPos2);
        modelRBDL.setDOFVelVect(Eigen::VectorXd::Zero(modelRBDL.sizeDOF()));
        modelRBDL.updateState();
        Eigen::MatrixXd jac2 = modelRBDL.pointJacobian(
            "left_sole_link", "ROOT");
        Eigen::VectorXd vec2 = jac2.transpose()*wrenchWorld;
        //Compute
        Eigen::MatrixXd vec3 = 1.0/(2.0*delta)*(vec1 - vec2);
        hessianProdWorldRBDL.col(indexDOF) = vec3;
    }
    chrono.stop("RBDL diff Hessian world");
    chrono.start("Pinocchio diff Hessian world");
    Eigen::MatrixXd hessianProdWorldPinocchio = 
        modelPinocchio.diffHessianForceInWorldProduct("left_sole_link", wrenchWorld.segment(3, 3));
    chrono.stop("Pinocchio diff Hessian world");
    checkEqual(hessianProdWorldRBDL, hessianProdWorldPinocchio, "diff Hessian world");

    //Test rotated vector differentiation
    Eigen::Vector3d vectInFrame(0.0, 0.0, 1.0);
    chrono.start("RBDL diff rotated vector");
    Eigen::MatrixXd rotationProductDiffRBDL = Eigen::MatrixXd::Zero(3, modelRBDL.sizeDOF());
    for (size_t indexDOF=0;indexDOF<modelRBDL.sizeVectVel();indexDOF++) {
        Eigen::VectorXd tmpVel1 = Eigen::VectorXd::Zero(modelRBDL.sizeDOF());
        Eigen::VectorXd tmpVel2 = Eigen::VectorXd::Zero(modelRBDL.sizeDOF());
        tmpVel1(indexDOF) += delta;
        tmpVel2(indexDOF) -= delta;
        Eigen::VectorXd tmpPos1 = leph::IntegrateDOFVect(statePosition, tmpVel1);
        Eigen::VectorXd tmpPos2 = leph::IntegrateDOFVect(statePosition, tmpVel2);
        //Sample first evaluation
        modelRBDL.setDOFPosVect(tmpPos1);
        modelRBDL.setDOFVelVect(Eigen::VectorXd::Zero(modelRBDL.sizeDOF()));
        modelRBDL.updateState();
        Eigen::VectorXd vec1 = modelRBDL.orientation("left_sole_link", "ROOT")*vectInFrame;
        //Sample second evaluation
        modelRBDL.setDOFPosVect(tmpPos2);
        modelRBDL.setDOFVelVect(Eigen::VectorXd::Zero(modelRBDL.sizeDOF()));
        modelRBDL.updateState();
        Eigen::VectorXd vec2 = modelRBDL.orientation("left_sole_link", "ROOT")*vectInFrame;
        //Compute
        Eigen::VectorXd vec3 = 1.0/(2.0*delta)*(vec1 - vec2);
        rotationProductDiffRBDL.col(indexDOF) = vec3;
    }
    chrono.stop("RBDL diff rotated vector");
    modelRBDL.setDOFPosVect(statePosition);
    modelRBDL.setDOFVelVect(Eigen::VectorXd::Zero(modelRBDL.sizeDOF()));
    modelRBDL.updateState();
    chrono.start("Pinocchio diff rotated vector");
    Eigen::MatrixXd rotationProductDiffPino = modelPinocchio.diffRotatedVector(
        "left_sole_link", "ROOT", vectInFrame);
    chrono.stop("Pinocchio diff rotated vector");
    checkEqual(rotationProductDiffRBDL, rotationProductDiffPino, "diff rotated vector");
    
    chrono.print();
}

int main()
{
    testPinocchioInterface();
    
    return 0;
}

