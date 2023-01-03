#include <iostream>
#include <cmath>
#include <thread>
#include <leph_maths/Sign.h>
#include <leph_maths/FilterIIR.hpp>
#include <leph_maths/SplineCubic.hpp>
#include <leph_maths/Clamp.h>
#include <leph_plot/Plot.hpp>
#include <leph_utils/Random.hpp>
#include <leph_maths/ControlDampedIntegral.hpp>

void testControlDampedIntegral(leph::Plot& plot)
{
    double dt = 0.001;
    double pos = 0.0;
    double vel = 0.0;
    double mass = 10.0;
    double gravity = 9.81;
    double frictionVel = 0.5;
    double frictionStatic = 0.4;
    double positionResolution = 0.005;
    double noiseForce = 0.5;
    double noiseVelocity = 0.008;
    double maximumEffort = 10.0;
    maximumEffort = 20000.0;
    noiseForce = 1e-6;
    frictionStatic = 0.0;
    frictionVel = 0.9;
    
    leph::SplineCubic splineTarget;
    splineTarget.addPoint(0.0, 0.00);
    splineTarget.addPoint(2.0, 0.40);
    splineTarget.addPoint(2.5, -0.30);
    splineTarget.addPoint(4.0, -0.30);
    splineTarget.addPoint(4.5, -0.35);
    splineTarget.addPoint(5.0, -0.25);
    splineTarget.addPoint(5.5, -0.35);
    splineTarget.addPoint(5.8, -0.25);
    splineTarget.addPoint(6.0, -0.30);
    splineTarget.addPoint(7.0, -0.30);
    splineTarget.addPoint(7.0, -0.60);
    splineTarget.addPoint(12.0, -0.60);
    splineTarget.addPoint(12.0, 0.20);
    splineTarget.addPoint(15.0, 0.20);
    
    //Control configuration
    leph::ControlDampedIntegral control(1.0/dt);
    control.posNeutral = 0.0;
    control.gainStabilizationP = 2000.0;
    control.gainStabilizationD = 2*1.0*std::sqrt(mass*control.gainStabilizationP);
    control.gainIntegralP = 0.0*310.0;
    control.gainIntegralD = 0.0*80.0;
    control.effortMaximum = maximumEffort;
    control.freqFilterStatePos = 100.0;
    control.freqFilterStateVel = 50.0;
    control.freqFilterErrorPos = 100.0;
    control.freqFilterErrorVel = 50.0;
    control.orderFilters = 3;

    leph::Random random;
    for (double t=0.0;t<14.0;t+=dt) {
        //Simulation sensor reading
        double posRead = std::floor(pos/positionResolution)*positionResolution;
        double velRead = vel + random.randGaussian(0.0, noiseVelocity);
        //Reference target position
        double target = splineTarget.pos(t);
        //Control policy
        control.posNeutral = target;
        double controlEffort = control.update(posRead, velRead);
        //Model simulation update
        double forceGravity = -mass*gravity*std::sin(pos);
        double forceControl = controlEffort;
        double forceDisturbance = 0.0;
        double forceFrictionVel = -frictionVel*vel;
        double forceFrictionStatic = -frictionStatic*leph::Sign(vel);
        double forceNoise = random.randGaussian(0.0, noiseForce);
        double acc = (
            forceGravity + forceControl + forceFrictionVel + 
            forceFrictionStatic + forceNoise + forceDisturbance)/mass;
        vel += dt*acc;
        pos += dt*vel;
        //Plot
        plot.add(
            "antiwindup", control.effortAntiWindup,
            "integrator", control.effortIntegral,
            "control", control.effortControl,
            "target", target,
            "pos", pos,
            "t", t
        );
    }
}

int main()
{
    leph::Plot plot;
    testControlDampedIntegral(plot);
    plot.plot("t", "all").show();

    return 0;
}

