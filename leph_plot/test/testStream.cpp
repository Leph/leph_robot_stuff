#include <iostream>
#include <leph_plot/Plot.hpp>
#include <chrono>
#include <thread>
#include <cmath>

int main()
{
    leph::Plot plot;
    //Test stream plot
    for (double t=0;t<10.0;t+=0.1) {
        double val = std::sin(t);
        std::cout << t << " " << val << std::endl;
        plot.add(
            "t", t, 
            "val", val);
        plot.filterLowerThan("t", t-3.0);
        plot.plot("t", "val").stream();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
    plot.closeWindow();
    plot.clear();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    //Test stream multiplot
    for (double t=0;t<10.0;t+=0.1) {
        double val1 = std::sin(t);
        double val2 = std::cos(t);
        double val3 = std::cos(2*t);
        plot.add(
            "t", t, 
            "val1", val1,
            "val2", val2,
            "val3", val3
        );
        plot.filterLowerThan("t", t-3.0);
        plot
            .multiplot(3, 1)
            .plot("t", "val1")
            .nextPlot()
            .plot("t", "val2")
            .nextPlot()
            .plot("t", "val3")
            .stream();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
    plot.closeWindow();
    plot.clear();
    std::this_thread::sleep_for(std::chrono::seconds(1));

    return 0;
}

