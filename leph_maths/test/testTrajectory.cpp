#include <iostream>
#include <leph_plot/Plot.hpp>
#include <leph_maths/TrajectoryBangBangAcc.hpp>

void plotTraj(
    leph::Plot& plot, 
    double posInit, double velInit, 
    double posEnd, double velEnd, 
    const std::string& name)
{
    double maxVel = 1.0;
    double maxAcc = 0.5;

    leph::TrajectoryBangBangAcc traj(
        posInit, velInit, 
        posEnd, velEnd,
        maxVel, maxAcc);

    for (double t=traj.min();t<traj.max();t+=0.01) {
        plot.add(
            "pos-" + name, traj.pos(t),
            "vel-" + name, traj.vel(t),
            "acc-" + name, traj.acc(t),
            "time", t
        );
    }
}

void testReplanning()
{
    double posCurrent = 0.0;
    double velCurrent = 0.0;
    double posEnd = 6.0;
    double velEnd = 0.0;
    
    double maxVel = 1.0;
    double maxAcc = 0.5;

    leph::Plot plot;
    for (double t=0.0;t<10.0;t+=0.01) {
        leph::TrajectoryBangBangAcc traj(
            posCurrent, velCurrent, 
            posEnd, velEnd,
            maxVel, maxAcc);
        posCurrent = traj.pos(0.01);
        velCurrent = traj.vel(0.01);
        plot.add(
            "pos", posCurrent,
            "vel", velCurrent,
            "time", t
        );
    }
    plotTraj(plot, 0.0, 0.0, posEnd, velEnd, "trajRef");
    plot
        .plot("time", "all")
        .show();
}

void testReplanning2()
{
    double posCurrent = 0.0;
    double velCurrent = 0.0;
    double posEnd = 0.5;
    double velEnd = 0.8;
    
    double maxVel = 1.0;
    double maxAcc = 0.5;

    leph::Plot plot;
    for (double t=0.0;t<10.0;t+=0.01) {
        leph::TrajectoryBangBangAcc traj(
            posCurrent, velCurrent, 
            posEnd, velEnd,
            maxVel, maxAcc);
        posCurrent = traj.pos(0.01);
        velCurrent = traj.vel(0.01);
        plot.add(
            "pos", posCurrent,
            "vel", velCurrent,
            "time", t
        );
    }
    plotTraj(plot, 0.0, 0.0, posEnd, velEnd, "trajRef");
    plot
        .plot("time", "all")
        .show();
}

void testStopping()
{
    double posCurrent = 0.0;
    double velCurrent = 1.0;
    
    double maxVel = 1.0;
    double maxAcc = 0.5;

    leph::Plot plot;
    for (double t=0.0;t<10.0;t+=0.01) {
        leph::TrajectoryBangBangAcc traj(
            posCurrent, velCurrent, 
            posCurrent, 0.0,
            maxVel, maxAcc);
        posCurrent = traj.pos(0.01);
        velCurrent = traj.vel(0.01);
        plot.add(
            "pos", posCurrent,
            "vel", velCurrent,
            "time", t
        );
    }
    plot
        .plot("time", "all")
        .show();
}

void testIntersection()
{
    leph::Plot plot;
    
    double posTarget = 0.4;
    double velTarget = 0.01;
    double dt = 0.01;
    
    double maxVel = 0.1;
    double maxAcc = 0.2;
    /*
    for (double pos=0.0;pos<0.5;pos+=0.005) {
        for (double ratio=0.5;ratio<=1.0;ratio+=0.005) {
            leph::TrajectoryBangBangAcc tmpTraj(
                0.0, 0.0, pos, velTarget, 
                ratio*maxVel, ratio*maxAcc);
            plot.add("pos", pos, "time", tmpTraj.max(), "ratio", ratio);
        }
    }
    plot.plot("pos", "time", leph::Plot::Points, "ratio").show();
    exit(0);
    */
    
    double timeLow = 0.0;
    double timeUp = 100.0;
    double timeMiddle;
    double posMiddle;
    for (int k=0;k<10;k++) {
        timeMiddle = 0.5*timeLow + 0.5*timeUp;
        posMiddle = posTarget+timeMiddle*velTarget;
        leph::TrajectoryBangBangAcc traj(
            0.0, 0.0, posMiddle, velTarget, 
            maxVel, maxAcc);
        if (traj.max() > timeMiddle) {
            timeLow = timeMiddle;
        } else {
            timeUp = timeMiddle;
        }
    }
    std::cout << timeMiddle << " " << posMiddle << std::endl;

    double timeIntersection = 6.0;
    double posIntersection = posTarget+timeIntersection*velTarget;
    double ratioUp = 1.0;
    double ratioLow = 0.0;
    double ratioMiddle;
    for (int k=0;k<10;k++) {
        ratioMiddle = 0.5*ratioUp + 0.5*ratioLow;
        leph::TrajectoryBangBangAcc traj(
            0.0, 0.0, posIntersection, velTarget, 
            ratioMiddle*maxVel, ratioMiddle*maxAcc);
        if (traj.max() > timeIntersection) {
            ratioLow = ratioMiddle;
        } else {
            ratioUp = ratioMiddle;
        }
    }
    std::cout << ratioMiddle << std::endl;

    leph::TrajectoryBangBangAcc traj(
        0.0, 0.0, posMiddle, velTarget, 
        //0.0, 0.0, posTarget+0.1, velTarget, 
        maxVel, maxAcc);
    /*
    leph::TrajectoryBangBangAcc traj(
        0.0, 0.0, posIntersection, velTarget, 
        ratioMiddle*maxVel, ratioMiddle*maxAcc);
    */
    std::cout << traj.min() << " " << traj.max() << std::endl;

    for (double t=0.0;t<15.0;t+=dt) {
        posTarget += dt*velTarget;
        double posHand = traj.pos(t);
        if (t > traj.max()) {
            posHand = traj.pos(traj.max()) + (t-traj.max())*velTarget;
        }
        plot.add(
            "target", posTarget,
            "hand", posHand,
            "time", t
        );
    }
    plot.plot("time", "all").show();
}

int main()
{
    leph::Plot plot;

    plotTraj(plot, 0.0, 0.0, 0.5, 0.0, "traj0");
    plotTraj(plot, 0.0, 0.0, 1.0, 0.0, "traj1");
    plotTraj(plot, 0.0, 0.0, 3.0, 0.0, "traj2");
    plot
        .multiplot(3, 1)
        .plot("time", "pos-*")
        .nextPlot()
        .plot("time", "vel-*")
        .nextPlot()
        .plot("time", "acc-*")
        .show();
    plot
        .plot("time", "pos-*")
        .show();
    plot
        .plot("time", "vel-*")
        .show();
    plot.clear();
    
    plotTraj(plot, 0.0, 0.0, -0.8, 0.5, "traj1");
    plotTraj(plot, 0.0, 0.0, -0.6, 0.5, "traj2");
    plotTraj(plot, 0.0, 0.0, -0.4, 0.5, "traj3");
    plotTraj(plot, 0.0, 0.0, -0.2, 0.5, "traj4");
    plotTraj(plot, 0.0, 0.0, 0.0, 0.5, "traj5");
    plotTraj(plot, 0.0, 0.0, 0.2, 0.5, "traj6");
    plotTraj(plot, 0.0, 0.0, 0.4, 0.5, "traj7");
    plotTraj(plot, 0.0, 0.0, 0.6, 0.5, "traj8");
    plotTraj(plot, 0.0, 0.0, 0.8, 0.5, "traj9");
    plot
        .multiplot(3, 1)
        .plot("time", "pos-*")
        .nextPlot()
        .plot("time", "vel-*")
        .nextPlot()
        .plot("time", "acc-*")
        .show();
    plot
        .plot("time", "pos-*")
        .show();
    plot
        .plot("time", "vel-*")
        .show();
    plot.clear();
    
    testReplanning();
    testReplanning2();
    testStopping();
    testIntersection();

    return 0;
}

