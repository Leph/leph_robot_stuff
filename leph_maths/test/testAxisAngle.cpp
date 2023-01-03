#include <iostream>
#include <Eigen/Dense>
#include <leph_maths/AxisAngle.h>

void test1(const Eigen::Matrix3d& mat1)
{
    Eigen::Vector3d axis = leph::MatrixToAxis(mat1);
    Eigen::Matrix3d mat2 = leph::AxisToMatrix(axis);
    if ((mat1-mat2).norm() > 1e-6) {
        std::cout << "Error conversion" << std::endl;
    }
}

void test2(const Eigen::Vector3d& axis1)
{
    Eigen::Matrix3d mat = leph::AxisToMatrix(axis1);
    Eigen::Vector3d axis2 = leph::MatrixToAxis(mat);
    if ((axis1-axis2).norm() > 1e-6) {
        std::cout << "Error conversion" << std::endl;
    }
}

int main()
{
    test1(Eigen::Matrix3d::Identity());
    test1(Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitX()).toRotationMatrix());
    test1(
        Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitX()).toRotationMatrix() *
        Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitY()).toRotationMatrix());
    test1(
        Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX()).toRotationMatrix() *
        Eigen::AngleAxisd(M_PI_2+0.2, Eigen::Vector3d::UnitY()).toRotationMatrix());
    test1(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix());
    test1(Eigen::AngleAxisd(-M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix());
    test1(Eigen::AngleAxisd(M_PI+0.1, Eigen::Vector3d::UnitX()).toRotationMatrix());

    test2(Eigen::Vector3d(0.0, 0.0, 0.0));
    test2(Eigen::Vector3d(-0.1, 0.0, 0.0));
    test2(Eigen::Vector3d(1.0, 0.0, 0.0));
    test2(Eigen::Vector3d(1.0, 2.0, 0.0));
    test2(Eigen::Vector3d(M_PI_2, 0.0, 0.0));
    test2(Eigen::Vector3d(M_PI_2+0.1, 0.0, 0.0));
    test2(Eigen::Vector3d(M_PI, 0.0, 0.0));

    return 0;
}

