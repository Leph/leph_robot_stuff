#include <iostream>
#include <random>
#include <Eigen/Dense>
#include <leph_maths/SplineCubic.hpp>
#include <leph_maths/FilterExponential.hpp>
#include <leph_maths/FilterIIR.hpp>
#include <leph_maths/FilterDifferentiator.hpp>
#include <leph_maths/FilterDifferentiator2.hpp>
#include <leph_maths/FilterBacklash.hpp>
#include <leph_maths/TrajectoryBangBangAcc.hpp>
#include <leph_plot/Plot.hpp>

static std::random_device rd;
static std::default_random_engine gen(rd());

static double noise()
{
    std::uniform_real_distribution<double> dist(-1.0, 1.0);
    return dist(gen);
}

static void testFilterExponential()
{
    //Test exponential filter with
    //two update time step frequency.
    //The same exponential decay is expected.
    leph::FilterExponential<double> filter1;
    leph::FilterExponential<double> filter2;
    filter1.reset(0.0);
    filter1.cutoffFrequency() = 2.0;
    filter2.reset(0.0);
    filter2.cutoffFrequency() = 2.0;

    leph::Plot plot;
    for (double t=0.0;t<1.0;t+=0.001) {
        filter1.update(1.0, 0.001);
        plot.add(
            "time", t,
            "filter1", filter1.value()
        );
    }
    for (double t=0.0;t<1.0;t+=0.01) {
        filter2.update(1.0, 0.01);
        plot.add(
            "time", t,
            "filter2", filter2.value()
        );
    }
    plot.plot("time", "all").show();
    
    //Test noise smoothing with 
    //the exponential filter and two
    //sampling frequency
    leph::FilterExponential<double> filter3;
    leph::FilterExponential<double> filter4;
    filter3.cutoffFrequency() = 10.0;
    filter4.cutoffFrequency() = 5.0;
    plot.clear();
    for (double t=0.0;t<3.0;t+=0.004) {
        double target = std::sin(2.0*M_PI*t);
        double signal = target + 0.1*noise();
        filter3.update(signal, 0.004);
        plot.add(
            "time", t,
            "target", target,
            "signal", signal,
            "filtered3", filter3.value()
        );
    }
    for (double t=0.0;t<3.0;t+=0.001) {
        double target = std::sin(2.0*M_PI*t);
        double signal = target + 0.1*noise();
        filter4.update(signal, 0.001);
        plot.add(
            "time", t,
            "filtered4", filter4.value()
        );
    }
    plot.plot("time", "all").show();
}

static void testFilterIIR()
{
    leph::FilterIIR<double> filter1;
    leph::FilterIIR<double> filter2;
    leph::FilterIIR<double> filter3;
    leph::FilterIIR<double> filter4;
    filter1.init(
        mkfilter::FilterType_t::Bessel,
        mkfilter::PassType_t::Lowpass,
        0.0, 
        2, 1.0/0.005, 20.0, 0.0);
    filter2.init(
        mkfilter::FilterType_t::Bessel,
        mkfilter::PassType_t::Lowpass,
        0.0, 
        4, 1.0/0.005, 20.0, 0.0);
    filter3.init(
        mkfilter::FilterType_t::Bessel,
        mkfilter::PassType_t::Lowpass,
        0.0, 
        6, 1.0/0.005, 20.0, 0.0);
    filter4.init(
        mkfilter::FilterType_t::Bessel,
        mkfilter::PassType_t::Lowpass,
        0.0, 
        8, 1.0/0.005, 20.0, 0.0);

    leph::Plot plot;
    for (double t=0.0;t<3.0;t+=0.005) {
        double target = std::sin(2.0*M_PI*t*(1.0+0.1*std::sin(2.0*M_PI*4.0*t)));
        double signal = target + 0.1*noise();
        filter1.update(signal);
        filter2.update(signal);
        filter3.update(signal);
        filter4.update(signal);
        plot.add(
            "target", target,
            "signal", signal,
            "filtered1", filter1.value(),
            "filtered2", filter2.value(),
            "filtered3", filter3.value(),
            "filtered4", filter4.value(),
            "time", t
        );
    }
    plot.plot("time", "all").show();
}

void testZeroPhaseFilter()
{
    std::vector<double> signalTarget;
    std::vector<double> signalOriginal;
    std::vector<double> signalFiltered1;
    std::vector<double> signalFiltered2;

    //Generate data
    for (double t=0.0;t<3.0;t+=0.005) {
        double target = 
            std::sin(2.0*M_PI*t*(1.0+0.1*std::sin(2.0*M_PI*4.0*t)));
        double signal = 
            target + 0.1*noise();
        signalTarget.push_back(target);
        signalOriginal.push_back(signal);
        signalFiltered1.push_back(0.0);
        signalFiltered2.push_back(0.0);
    }
    
    leph::FilterIIR<double> filter;
    filter.init(
        mkfilter::FilterType_t::Butterworth,
        mkfilter::PassType_t::Lowpass, 0.0,
        4, 1.0/0.005, 15.0, 0.0);

    //Forward filter
    filter.reset();
    for (int i=0;i<(int)signalOriginal.size();i++) {
        filter.update(signalOriginal[i]);
        signalFiltered1[i] = filter.value();
    }
    
    //Backward filter
    filter.reset();
    for (int i=signalOriginal.size()-1;i>=0;i--) {
        filter.update(signalFiltered1[i]);
        signalFiltered2[i] = filter.value();
    }

    //Plot
    leph::Plot plot;
    for (int i=0;i<(int)signalOriginal.size();i++) {
        plot.add(
            "target", signalTarget[i],
            "signal", signalOriginal[i],
            "filteredForward", signalFiltered1[i],
            "filteredZeroPhase", signalFiltered2[i],
            "time", (double)i
        );
    }
    plot.plot("time", "all").show();
}
    
void testCompareExponantialAndIIR()
{
    //Compare FilterExponential and order1 FilterIIR
    leph::FilterExponential<double> filterEXP;
    filterEXP.cutoffFrequency() = 10.0;
    leph::FilterIIR<double> filterIIR1;
    filterIIR1.init(
        mkfilter::FilterType_t::Bessel,
        mkfilter::PassType_t::Lowpass, 0.0,
        1, 1.0/0.005, 10.0, 0.0);
    leph::FilterIIR<double> filterIIR3;
    filterIIR3.init(
        mkfilter::FilterType_t::Bessel,
        mkfilter::PassType_t::Lowpass, 0.0,
        3, 1.0/0.005, 10.0, 0.0);
    leph::Plot plot;
    for (double t=0.0;t<2.0;t+=0.005) {
        double target = std::sin(2.0*M_PI*t*(1.0+0.1*std::sin(2.0*M_PI*4.0*t)));
        double signal = target + 0.1*noise();
        filterEXP.update(signal, 0.005);
        filterIIR1.update(signal);
        filterIIR3.update(signal);
        plot.add(
            "target", target,
            "signal", signal,
            "filteredEXP", filterEXP.value(),
            "filteredIIROrder1", filterIIR1.value(),
            "filteredIIROrder3", filterIIR3.value(),
            "time", t
        );
    }
    plot.plot("time", "all").show();
}

void testFilterDifferentiator()
{
    //Test noisy signal differentiation
    leph::FilterDifferentiator filterDiff1(2, 1.0/0.005);
    leph::FilterDifferentiator filterDiff2(6, 1.0/0.005);
    leph::FilterIIR<double> filterIIR1;
    leph::FilterIIR<double> filterIIR2;
    filterIIR1.init(
        mkfilter::FilterType_t::Bessel,
        mkfilter::PassType_t::Lowpass, 0.0,
        6, 1.0/0.005, 10.0, 0.0);
    filterIIR2.init(
        mkfilter::FilterType_t::Bessel,
        mkfilter::PassType_t::Lowpass, 0.0,
        2, 1.0/0.005, 10.0, 0.0);
    
    leph::FilterIIR<double> filterIIRPre;
    filterIIRPre.init(
        mkfilter::FilterType_t::Bessel,
        mkfilter::PassType_t::Lowpass, 0.0,
        4, 1.0/0.005, 10.0, 0.0);
    
    leph::Plot plot;
    for (double t=0.0;t<2.0;t+=0.005) {
        double target = 
            std::sin(2.0*M_PI*t*(1.0+0.1*std::sin(2.0*M_PI*4.0*t)));
        double signal = 
            target + 0.1*noise();
        double diff = 
            1.6*M_PI*std::cos(M_PI*t*(0.2*std::sin(8.0*M_PI*t)+2.0))
            *(0.125*std::sin(8.0*M_PI*t)+M_PI*t*std::cos(8.0*M_PI*t)+1.25);
        filterDiff1.update(signal);
        filterDiff2.update(signal);
        filterIIR1.update(filterDiff1.value());
        filterIIR2.update(filterDiff2.value());
        plot.add(
            "target", target,
            "signal", signal,
            "diff", diff,
            "filterDiffRaw1", filterDiff1.value(),
            "filterDiffRaw2", filterDiff2.value(),
            "filterDiffIIR1", filterIIR1.value(),
            "filterDiffIIR2", filterIIR2.value(),
            "time", t
        );
    }
    plot.plot("time", "all").show();
    plot.clear();
    filterIIR1.reset();
    filterIIR2.reset();
    filterDiff1.reset();
    filterDiff2.reset();

    leph::SplineCubic spline;
    spline.addPoint(0.0, 0.2);
    spline.addPoint(0.2, 0.2);
    spline.addPoint(0.8, 0.5);
    spline.addPoint(0.825, 0.495);
    spline.addPoint(0.85, 0.5);
    spline.addPoint(1.1, 0.25);
    spline.addPoint(1.4, -0.1);
    spline.addPoint(1.6, 0.3);
    spline.addPoint(1.7, 0.3);
    spline.addPoint(1.72, 0.28);
    spline.addPoint(1.74, 0.32);
    spline.addPoint(1.76, 0.28);
    spline.addPoint(1.78, 0.32);
    spline.addPoint(1.80, 0.3);
    spline.addPoint(2.0, 0.3);
    for (double t=0.0;t<2.0;t+=0.005) {
        double target = spline.pos(t);
        double signal = (double)((int)(target/0.02)*0.02);
        double diff = spline.vel(t);
        filterIIRPre.update(signal);
        filterDiff1.update(filterIIRPre.value());
        filterDiff2.update(signal);
        filterIIR1.update(filterDiff1.value());
        filterIIR2.update(filterDiff2.value());
        plot.add(
            "target", target,
            "signal", signal,
            "derivative", diff,
            "filterIIKPre", filterIIRPre.value(),
            "filterDiff", filterDiff1.value(),
            "filterIIRPost", filterIIR1.value(),
            "time", t
        );
    }
    plot.plot("time", "all").show();
}

void testFilterDifferentiator2()
{
    //Test noisy signal differentiation
    leph::FilterDifferentiator2<double> filterDiff;
    filterDiff.cutoffFrequency() = 2.0;
    filterDiff.timeDelta() = 0.02;
    
    leph::Plot plot;
    for (double t=0.0;t<2.0;t+=0.005) {
        double target = 
            std::sin(2.0*M_PI*t*(1.0+0.1*std::sin(2.0*M_PI*4.0*t)));
        double signal = 
            target + 0.1*noise();
        double diff = 
            1.6*M_PI*std::cos(M_PI*t*(0.2*std::sin(8.0*M_PI*t)+2.0))
            *(0.125*std::sin(8.0*M_PI*t)+M_PI*t*std::cos(8.0*M_PI*t)+1.25);
        filterDiff.update(signal, 0.005);
        plot.add(
            "target", target,
            "signal", signal,
            "diff-target", diff,
            "diff-filtered", filterDiff.value(),
            "time", t
        );
    }
    plot.plot("time", "all").show();
    plot.clear();
}

void testFilterBacklash()
{
    leph::FilterBacklash filter(0.05);
    leph::Plot plot;
    for (double t=0.0;t<2.0;t+=0.005) {
        double target = 
            std::sin(2.0*M_PI*t)*std::sin(2.0*M_PI*8.0*(1.0-std::sin(2.0*M_PI*t*0.5))*t);
        filter.update(t, target);
        plot.add(
            "target", target,
            "filterredValue", filter.value(),
            "time", t
        );
    }
    plot.plot("time", "all").show();
}

void testFilterBangBangAcc()
{
    leph::FilterExponential<double> filterEXP;
    filterEXP.cutoffFrequency() = 5.0;
    leph::FilterBangBangAcc<double> filterBangBangAcc1;
    leph::FilterBangBangAcc<Eigen::Vector3d> filterBangBangAcc2;
    filterBangBangAcc1.maxVel() = 30.0;
    filterBangBangAcc1.maxAcc() = 600.0;
    filterBangBangAcc2.maxVel() = 30.0;
    filterBangBangAcc2.maxAcc() = 600.0;
    leph::Plot plot;
    double lastFilteredPos = 0.0;
    for (double t=0.0;t<2.0;t+=0.005) {
        double target = std::sin(2.0*M_PI*t*(1.0+0.1*std::sin(2.0*M_PI*4.0*t)));
        if (t > 1.0) {
            target += 2.0;
        }
        if (t > 1.5) {
            //Test chaning the max velocity during max vel is reached
            filterBangBangAcc1.maxVel() = 10.0;
        }
        double signal = target + 0.1*noise();
        filterEXP.update(signal, 0.005);
        filterBangBangAcc1.update(signal, 0.005);
        filterBangBangAcc2.update(Eigen::Vector3d(filterEXP.value(), 0.0, 0.0), 0.005);
        double filteredVel = (filterBangBangAcc1.value()-lastFilteredPos)/0.005;
        lastFilteredPos = filterBangBangAcc1.value();
        plot.add(
            "target", target,
            "signal", signal,
            "filteredEXP", filterEXP.value(),
            "filteredBangBang1", filterBangBangAcc1.value(),
            "filteredBangBang2", filterBangBangAcc2.value()(0),
            //"filteredVel", filteredVel,
            "time", t
        );
    }
    plot.plot("time", "all").show();
}

int main()
{
    testFilterExponential();
    testFilterIIR();
    testZeroPhaseFilter();
    testCompareExponantialAndIIR();
    testFilterDifferentiator();
    testFilterDifferentiator2();
    testFilterBacklash();
    testFilterBangBangAcc();

    return 0;
}

