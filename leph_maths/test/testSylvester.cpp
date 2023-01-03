#include <iostream>
#include <Eigen/Dense>
#include <leph_maths/Sylvester.h>

int main()
{
    Eigen::MatrixXd A = Eigen::MatrixXd::Random(2,2);
    Eigen::MatrixXd B = Eigen::MatrixXd::Random(3,3);
    Eigen::MatrixXd C = Eigen::MatrixXd::Random(2,3);
    std::cout << "Input matrix A:" << std::endl;
    std::cout << A << std::endl;
    std::cout << "Input matrix B:" << std::endl;
    std::cout << B << std::endl;
    std::cout << "Input matrix C:" << std::endl;
    std::cout << C << std::endl;
    Eigen::MatrixXd X;
    bool isSuccess = leph::Sylvester(A, B, C, X);
    std::cout << "IsSuccess: " << isSuccess << std::endl;
    std::cout << "Solution: " << std::endl;
    std::cout << X << std::endl;
    std::cout << "Check: " << std::endl;
    std::cout << A*X + X*B << std::endl;

    return 0;
}

