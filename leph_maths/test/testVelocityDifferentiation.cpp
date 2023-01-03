#include <iostream>
#include <cmath>
#include <leph_plot/Plot.hpp>
#include <leph_maths/FilterEncoderDifferentiation.hpp>
#include <leph_maths/Sign.h>
#include <leph_maths/FilterExponential.hpp>
#include <leph_utils/Chrono.hpp>

void testEncoderDifferentiation()
{
    double dt = 0.002;
    double quantum = 0.005;

    leph::FilterEncoderDifferentiation filter;
    filter.posQuantum = 0.99*quantum;
    filter.freqSampling = 500.0;

    leph::Plot plot;
    leph::Chrono chrono;
    for (double t=0.0;t<30.0;t+=dt) {
        //Trajectory simulation
        double freq = 1.0;
        if (t >= 0.0 && t <= 10.0) freq = 0.1;
        if (t >= 10.0 && t <= 11.0) freq = 0.0;
        if (t >= 11.0 && t <= 15.0) freq = 0.5;
        if (t >= 15.0 && t <= 16.0) freq = 0.0;
        if (t >= 16.0 && t <= 20.0) freq = 2.0;
        if (t >= 20.0 && t <= 21.0) freq = 0.0;
        if (t >= 21.0 && t <= 25.0) freq = 6.0;
        if (t >= 25.0 && t <= 30.0) freq = 0.2;
        double pos = 0.2005*std::sin(2.0*M_PI*freq*t);
        double vel = 0.2005*2.0*M_PI*freq*std::cos(2.0*M_PI*freq*t);
        double read = std::floor(pos/quantum)*quantum;
        //Filtering
        chrono.start("filtering");
        filter.update(read, 15, 10.0);
        chrono.stop("filtering");
        //Plotting
        plot.add(
            "pos", pos,
            "vel", vel,
            "posEstimated", filter.posEstimated,
            "velEstimated", filter.velEstimated,
            "velFiltered", filter.velFiltered,
            "read", read,
            "t", t
        );
    }
    chrono.print();
    plot.plot("t", "all").show();
}

void testFilteringWithControl()
{
    double dt = 0.002;
    double mass1 = 50.0;
    double mass2 = 10.0;
    double stiffness2 = 5000.0;
    double pos1 = -0.04;
    double pos2 = -0.04;
    double vel1 = 0.0;
    double vel2 = 0.0;
    double frictionVel = 12.0;
    double frictionStatic = 4.0;
    double quantum = 0.0007;

    leph::Plot plot;
    std::vector<double> history;
    leph::FilterExponential<double> filterPos(5.0);
    leph::FilterExponential<double> filterVel(5.0);
    leph::FilterExponential<double> filterControl(50.0);
    for (double t=0.0;t<12.0;t+=dt) {
        double target = std::sin(2.0*M_PI*0.1*(t-3.0));
        if (target >= 0.0) target = 0.04;
        if (target < 0.0) target = -0.04;
        double read = std::floor(pos1/quantum)*quantum;
        history.push_back(read);
        filterPos.update(read, dt);
        double posFiltered = filterPos.value();
        double velFiltered = 0.0;
        if (history.size() > 5) {
            double diff = (history.at(history.size()-1-0)-history.at(history.size()-1-5))/(5*dt);
            filterVel.update(diff, dt);
            velFiltered = filterVel.value();
        }
        double control = 1500.0*(target-posFiltered) - 800.0*velFiltered;
        filterControl.update(control, dt);
        double acc2 = 1.0/mass2*(
            stiffness2*(pos1-pos2) 
            - frictionStatic*leph::Sign(vel2)
            - frictionVel*vel2);
        double acc1 = 1.0/mass1*(
            - stiffness2*(pos1-pos2) 
            - mass1*9.81*std::sin(pos1) 
            - frictionStatic*leph::Sign(vel1) 
            - frictionVel*vel1 
            + filterControl.value());
        vel1 += dt*acc1;
        vel2 += dt*acc2;
        pos1 += dt*vel1;
        pos2 += dt*vel2;
        //Plotting
        plot.add(
            "target", target,
            "control", control,
            "controlFiltered", filterControl.value(),
            "pos", pos1,
            "pos2", pos2,
            "read", read,
            "posFiltered", posFiltered,
            "vel", vel1,
            "velFiltered", velFiltered,
            "t", t
        );
    }
    plot.plot("t", "all").show();
}

int main()
{
    testFilteringWithControl();
    testEncoderDifferentiation();

    return 0;
}

