#include <iostream>
#include <iomanip>
#include <Eigen/Dense>
#include <leph_maths/AutoRegressive.h>
#include <leph_plot/Plot.hpp>

void testResursiveUnfording()
{
    //Problem sizes
    int sizeHistory = 3;
    int sizeHorizon = 10;

    //Model parameters
    double paramsConst = 0.05;
    Eigen::VectorXd paramsState(sizeHistory);
    paramsState << 0.7, 0.2, 1.0;
    Eigen::VectorXd paramsAction(sizeHistory);
    paramsAction << 0.15, -0.1, -0.01;

    //Initial state
    Eigen::VectorXd dataStateInit(sizeHistory);
    Eigen::VectorXd dataActionInit(sizeHistory);
    for (int i=0;i<sizeHistory;i++) {
        dataStateInit(i) = -(double)i/50.0;
        dataActionInit(i) = 0.0;
    }
    //Time series prediction
    Eigen::VectorXd dataStatePredict(sizeHorizon);
    Eigen::VectorXd dataActionPredict(sizeHorizon-1);
    for (int i=0;i<sizeHorizon;i++) {
        //Build param vector
        Eigen::VectorXd tmpParams(2*sizeHistory+1);
        tmpParams(0) = paramsConst;
        tmpParams.segment(1, sizeHistory) = paramsState;
        tmpParams.segment(1+sizeHistory, sizeHistory) = paramsAction;
        //Build input vector
        Eigen::VectorXd input(2*sizeHistory+1);
        input(0) = 1.0;
        for (int j=0;j<sizeHistory;j++) {
            if (i-1-j < 0) {
                input(1+j) = dataStateInit(j-i);
                input(1+sizeHistory+j) = dataActionInit(j-i);
            } else {
                input(1+j) = dataStatePredict(i-1-j);
                input(1+sizeHistory+j) = dataActionPredict(i-1-j);
            }
        }
        //Compute prediction
        dataStatePredict(i) = tmpParams.transpose()*input;
        //Compute next action
        if (i < sizeHorizon-1) {
            dataActionPredict(i) = std::sin(((double)i)/15.0);
        }
    }
    //Ploting
    leph::Plot plot;
    for (int i=sizeHistory-1;i>=0;i--) {
        plot.add(
            "time", -((double)i)-1.0,
            "pos", dataStateInit(i),
            "tau", dataActionInit(i));
    }
    for (int i=0;i<sizeHorizon-1;i++) {
        plot.add(
            "time", (double)i,
            "pos", dataStatePredict(i),
            "tau", dataActionPredict(i));
    }
    plot.add(
        "time", (double)sizeHorizon-1,
        "pos", dataStatePredict(sizeHorizon-1));
    plot.plot("time", "all").show();

    //Compute recursive coefficients
    Eigen::VectorXd coefConst;
    Eigen::MatrixXd coefAction;
    Eigen::MatrixXd coefInitAction;
    Eigen::MatrixXd coefInitState;
    leph::AutoRegressiveModelMatrices(
        sizeHorizon, sizeHistory, 
        paramsConst, paramsState, paramsAction,
        coefConst, coefInitState, coefInitAction, coefAction);

    //Compute state prediction
    Eigen::VectorXd dataStatePredict2(sizeHorizon);
    dataStatePredict2 = 
        coefAction*dataActionPredict + 
        coefInitState*dataStateInit + 
        coefInitAction*dataActionInit + 
        coefConst;
    std::cout << "Results:" << std::endl;
    std::cout << dataStatePredict.transpose() << std::endl;
    std::cout << dataStatePredict2.transpose() << std::endl;
    if ((dataStatePredict-dataStatePredict2).norm() > 1e-6) {
        std::cout << "Error mismath vectors" << std::endl;
    }
}

int main()
{
    testResursiveUnfording();

    return 0;
}

