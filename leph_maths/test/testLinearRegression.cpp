#include <iostream>
#include <random>
#include <algorithm>
#include <Eigen/Dense>
#include <leph_maths/LinearRegression.hpp>
#include <leph_plot/Plot.hpp>

static std::random_device rd;
static std::default_random_engine gen(rd());

static double function(double x, double y)
{
    return 2.0*x + 1.0*y + 0.5;
}

static double noise()
{
    std::uniform_real_distribution<double> dist(-1.0, 1.0);
    return dist(gen);
}

int main()
{
    leph::Plot plot;
    leph::LinearRegression reg;
    Eigen::DiagonalMatrix<double, Eigen::Dynamic> regWeight(1);
    Eigen::MatrixXd regMat(1, 3);
    Eigen::MatrixXd regBias(1, 1);
    regWeight.diagonal() << 10.0;
    regMat << 1.0, 1.0, 1.0;
    regBias << 3.5;
    reg.setRegularization(regMat, regBias, regWeight);

    for (size_t k=0;k<1000;k++) {
        Eigen::VectorXd input(3);
        input(0) = 1.0;
        input(1) = noise();
        input(2) = noise();
        Eigen::VectorXd output(1);
        output(0) = function(input(1), input(2)) + 0.5*noise();
        reg.append(input, output);
        plot.add(
            "x", input(1),
            "y", input(2),
            "target", output(0)
        );
    }
    
    //Compute linear model
    reg.fit();
    reg.print();
    std::cout << "Params sum: " <<
        reg.params()(0,0) + reg.params()(0,1) + reg.params()(0,2)
        << std::endl;
    plot.plot("x", "y", "all").show();

    return 0;
}

