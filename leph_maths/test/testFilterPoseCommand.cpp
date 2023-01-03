#include <iostream>
#include <leph_maths/FilterPoseCommand.hpp>
#include <leph_utils/Random.hpp>
#include <leph_plot/Plot.hpp>

/**
 * Test FilterPoseCommand on a 1d example trajectory
 */
int main()
{
    double limitRadius = 0.3;
    double dt = 0.01;

    //Filter parameters
    leph::FilterPoseCommand filter;
    Eigen::Vector3d pos = Eigen::Vector3d(0.2, 0.0, 0.0);
    filter.setParameters(
        false, 1.0,
        0.05, 0.1,
        10.0,
        0.1, 1.0,
        0.1, 0.2);
    filter.reset(
        pos,
        Eigen::Matrix3d::Identity());

    //Simulation loop
    leph::Plot plot;
    leph::Random random;
    for (double t=0.0;t<12.0;t+=dt) {
        //Velocity command
        Eigen::Vector3d velLin = Eigen::Vector3d::Zero();
        Eigen::Vector3d velAng = Eigen::Vector3d::Zero();
        if (t >= 0.5 && t <= 4.0) {
            velLin.x() = 0.1*std::sin(2.0*M_PI*t*(0.5+0.1*(t-0.5)));
        }
        if (t >= 4.0 && t <= 8.0) {
            velLin.x() = (0.1+0.1*(t-4.0))*std::sin(2.0*M_PI*t*(0.5+0.2*(t-4.0)));
        }
        if (t >= 8.0 && t <= 12.0) {
            velLin.x() = 0.8*std::sin(2.0*M_PI*t*0.2+M_PI);
        }
        velLin.x() += random.randGaussian(0.0, 0.02);
        //Integration
        filter.update(
            dt, 
            false, Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity(),
            velLin, velAng, 
            pos, Eigen::Matrix3d::Identity());
        Eigen::Vector3d newPos = filter.valuePos();
        //Simulate limit
        if (newPos.norm() > limitRadius) {
            newPos = limitRadius*newPos.normalized();
        }
        //COmpute filtered velocity
        Eigen::Vector3d filteredVel = (1.0/dt)*(newPos-pos);
        pos = newPos;
        //Plot
        plot.add(
            "vel_x", velLin.x(),
            "vel_z", velLin.z(),
            "pos_x", pos.x(),
            "pos_z", pos.z(),
            "filteredPos_x", filter.valuePos().x(),
            "filteredVel_x", filteredVel.x(),
            "time", t);
    }
    plot.plot("time", "all").show();

    return 0;
}

