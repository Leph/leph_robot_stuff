#include <iostream>
#include <leph_maths/RecursiveLeastSquares.hpp>
#include <leph_plot/Plot.hpp>
#include <leph_utils/Random.hpp>

void testSimple()
{
    leph::RecursiveLeastSquares model;
    model.reset(Eigen::MatrixXd::Zero(1, 3), 1e3);

    leph::Random random;
    leph::Plot plot;
    double xx = 0.0;
    double yy = 0.0;
    double targetParam0 = 0.5;
    double targetParam1 = 1.0;
    double targetParam2 = 2.0;
    double dt = 0.001;
    for (double t=0.0;t<10.0;t+=dt) {
        //State space motion
        double phaseState = t;
        /*
        while (phaseState > 1.0) {
            phaseState -= 1.0;
        }
        if (phaseState <= 0.2) {
            //Random walk
            xx += random.randGaussian(0.0, 0.01);
            yy += random.randGaussian(0.0, 0.01);
        } else if (phaseState <= 0.4) {
            //Sinus walk
            xx += 0.010*std::sin(2.0*M_PI*(phaseState-0.4)*(5.0+10.0*(0.4-phaseState)/0.2)+0.1);
            yy += 0.012*std::sin(2.0*M_PI*(phaseState-0.4)*(5.0+15.0*(0.4-phaseState)/0.2)+0.2);
        } else if (phaseState <= 0.6) {
            //No motion
        } else if (phaseState <= 0.8) {
            //Only one direction excitation
            xx += random.randGaussian(0.0, 0.01);
        } else if (phaseState <= 1.0) {
            //One dimension excitation
            double delta = random.randGaussian(0.0, 0.01);
            xx += 1.0*delta;
            yy += 0.5*delta;
        }
        */
        xx = 0.5*std::sin(2.0*M_PI*10.0*t+0.1) + random.randGaussian(0.0, 0.01);
        if (t < 5.0) {
            yy = 0.5*std::sin(2.0*M_PI*11.0*t+0.2) + random.randGaussian(0.0, 0.01);
        }
        //Bound state to unit square
        if (xx > 1.0) {
            xx = 1.0;
        }
        if (yy > 1.0) {
            yy = 1.0;
        }
        if (xx < -1.0) {
            xx = -1.0;
        }
        if (yy < -1.0) {
            yy = -1.0;
        }
        //Change of target parameters
        /*
        double phaseTarget = t/4.0;
        while (phaseTarget > 1.0) {
            phaseTarget -= 1.0;
        }
        if (phaseTarget <= 0.4) {
            //Random walk
            //targetParam0 += random.randGaussian(0.0, 0.01);
            //targetParam1 += random.randGaussian(0.0, 0.01);
            targetParam2 += random.randGaussian(0.0, 0.01);
        } else if (phaseTarget <= 0.8) {
            //Smooth oscillation
            //targetParam0 += 0.010*std::sin(2.0*M_PI*phaseTarget*(30.0+30.0*(0.8-phaseTarget)/0.2)+0.1);
            //targetParam1 += 0.012*std::sin(2.0*M_PI*phaseTarget*(30.0+32.0*(0.8-phaseTarget)/0.4)+0.2);
            targetParam2 += 0.014*std::sin(2.0*M_PI*phaseTarget*(30.0+34.0*(0.8-phaseTarget)/0.4)+0.3);
        } else if (phaseTarget <= 0.9 && phaseTarget+0.5*dt > 0.9) {
            //Step
            //targetParam0 += random.randGaussian(0.0, 1.0);
            //targetParam1 += random.randGaussian(0.0, 1.0);
            targetParam2 += random.randGaussian(0.0, 1.0);
        }
        */
        if (t < 8.0) {
            targetParam2 += 0.001;
        }
        //Compute signal
        double targetSignal = 
            targetParam2*xx + targetParam1*yy + targetParam0*1.0;
        double noise = random.randGaussian(0.0, 0.01);
        double valueSignal = targetSignal + noise;
        //Model update
        Eigen::VectorXd input(3);
        Eigen::VectorXd output(1);
        input << 1.0, xx, yy;
        output << valueSignal;
        std::cout << "-------------------" << std::endl;
        std::cout << "Time: " << t << " PhaseState: " << phaseState << std::endl;
        std::cout << "TargetParams: " << targetParam0 << " " << targetParam1 << " " << targetParam2 << std::endl;
        Eigen::VectorXd predictBefore = model.predict(input);
        model.append(input, output, 0.9);
        Eigen::VectorXd predictAfter = model.predict(input);
        //Plot
        plot.add(
            "x", xx,
            "y", yy,
            "signal", valueSignal,
            "target", targetSignal,
            "prediction_before", predictBefore(0),
            "prediction_after", predictAfter(0),
            "target_param0", targetParam0,
            "target_param1", targetParam1,
            "target_param2", targetParam2,
            "computed_param0", model.params()(0, 0),
            "computed_param1", model.params()(0, 1),
            "computed_param2", model.params()(0, 2),
            "covariance_min", model.covariance().cwiseAbs().minCoeff(),
            "covariance_max", model.covariance().cwiseAbs().maxCoeff(),
            "time", t
        );
    }
    //Plot
    plot
        .plot("time", "x")
        .plot("time", "y")
        .show();
    plot
        .plot("x", "y", leph::Plot::LinesPoints, "time")
        .show();
    plot
        .plot("time", "covariance_min")
        .plot("time", "covariance_max")
        .show();
    plot
        .plot("time", "target_param0")
        .plot("time", "target_param1")
        .plot("time", "target_param2")
        .plot("time", "computed_param0")
        .plot("time", "computed_param1")
        .plot("time", "computed_param2")
        .show();
    plot
        .plot("x", "y", "signal", leph::Plot::Points)
        .plot("x", "y", "target", leph::Plot::Points)
        .plot("x", "y", "prediction_before", leph::Plot::Points)
        .plot("x", "y", "prediction_after", leph::Plot::Points)
        .show();
    plot
        .plot("time", "signal")
        .plot("time", "target")
        .plot("time", "prediction_before")
        .plot("time", "prediction_after")
        .show();

}

void testSimplePlane()
{
    leph::RecursiveLeastSquares model;
    model.reset(Eigen::MatrixXd::Zero(1, 2), 1e2);
    
    leph::Random random;
    leph::Plot plot;
    double xx = 0.0;
    double yy = 0.0;
    double targetParamX = 1.5;
    double targetParamY = 2.0;
    double dt = 0.001;
    for (double t=0.0;t<1.2;t+=dt) {
        //Input exploration
        /*
        if (t < 1.0) {
            xx = random.randGaussian(0.0, 0.5);
            yy = random.randGaussian(0.0, 0.5);
        } else {
            xx = random.randGaussian(0.0, 0.5);
        }
        */
        if (t < 1.0) {
            xx = 0.5*std::sin(2.0*M_PI*t*20.0+0.15);
            yy = 0.5*std::sin(2.0*M_PI*t*28.0+1.1);
        } else if (t < 1.5) {
            xx = 0.5*std::sin(2.0*M_PI*t*20.0+0.15);
            yy = xx;
        } else {
            xx = 0.5*std::sin(2.0*M_PI*t*20.0+0.15);
            yy = 0.5*std::sin(2.0*M_PI*t*28.0+1.1);
        }
        //Parameter change
        targetParamX += dt;
        targetParamX += dt*dt;
        //targetParamY += 0.2*dt;
        //Compute signal
        double targetSignal = targetParamX*xx + targetParamY*yy;
        double noise = random.randGaussian(0.0, 0.001);
        double valueSignal = targetSignal + noise;
        //Model update
        Eigen::VectorXd input(2);
        Eigen::VectorXd output(1);
        input << xx, yy;
        output << valueSignal;
        std::cout << "-------------------" << std::endl;
        std::cout << "Time: " << t << std::endl;
        std::cout << "TargetParams: " << targetParamX << " " << targetParamY << std::endl;
        Eigen::VectorXd predictBefore = model.predict(input);
        model.append(input, output, 0.9);
        Eigen::VectorXd predictAfter = model.predict(input);
        //Plot
        Eigen::MatrixXd xxt = input*input.transpose();
        plot.add(
            "x", xx,
            "y", yy,
            "signal", valueSignal,
            "target", targetSignal,
            "target_paramX", targetParamX,
            "target_paramY", targetParamY,
            "computed_paramX", model.params()(0, 0),
            "computed_paramY", model.params()(0, 1),
            "covInv_0_0", model.covarianceInverse()(0, 0),
            "covInv_1_1", model.covarianceInverse()(1, 1),
            "cov_0_0", model.covariance()(0, 0),
            "cov_1_1", model.covariance()(1, 1),
            "xxt_0_0", xxt(0, 0),
            "xxt_1_1", xxt(1, 1),
            "time", t
        );
    }
    //Plot
    plot
        .plot("time", "x")
        .plot("time", "y")
        .show();
    plot
        .plot("time", "cov_0_0")
        .plot("time", "cov_1_1")
        .plot("time", "covInv_0_0")
        .plot("time", "covInv_1_1")
        .plot("time", "xxt_0_0")
        .plot("time", "xxt_1_1")
        .show();
    plot
        .plot("time", "target_paramX")
        .plot("time", "target_paramY")
        .plot("time", "computed_paramX")
        .plot("time", "computed_paramY")
        .show();
}

int main()
{
    testSimplePlane();

    return 0;
}

