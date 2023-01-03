#include <iostream>
#include <Eigen/Dense>
#include <leph_maths/SoftMax.h>
#include <leph_plot/Plot.hpp>

int main()
{
    Eigen::VectorXd values = Eigen::VectorXd::Zero(5);
    values << 0.0, 0.2, 0.3, -0.4, -0.5;
    std::cout << leph::SoftMaxSigned(values, 0.01) << std::endl;
    std::cout << leph::SoftMaxSignedDiff(values, 0.01).transpose() << std::endl;
    std::cout << leph::SoftMaxAbsolute(values, 0.01) << std::endl;
    std::cout << leph::SoftMaxAbsoluteDiff(values, 0.01).transpose() << std::endl;

    leph::Plot plot;
    for (double v=-10.0;v<10.0;v+=0.002) {
        values(4) = v;
        double normSgn = leph::SoftMaxSigned(values, 0.01);
        double normAbs = leph::SoftMaxAbsolute(values, 0.01);
        plot.add(
            "normAbs", normAbs,
            "normSgn", normSgn,
            "value", v
        );
    }
    plot.plot("value", "all").show();

    Eigen::VectorXd state = Eigen::VectorXd::Zero(5);
    state << 0.0, 0.2, -2.0, 4.0, 10.0;
    for (size_t k=0;k<10000;k++) {
        Eigen::VectorXd diffState = Eigen::VectorXd::Zero(5);
        for (size_t i=0;i<5;i++) {
            diffState(i) += state.sum() - 1.0;
        }
        diffState += 0.1*leph::SoftMaxAbsoluteDiff(state, 0.01);
        state -= 0.02*diffState;
        std::cout << "sum=" << state.sum() << " state=" << state.transpose() << std::endl;
    }

    return 0;
}

