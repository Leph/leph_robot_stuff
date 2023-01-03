#include <iostream>
#include <leph_plot/Plot.hpp>

int main()
{
    leph::Plot plot;
    for (int i=0;i<100;i++) {
        plot.add(
            "aa:toto", i,
            "aa:titi", -i,
            "bb:tata", 1.0,
            "bb:tete", -0.5*i
        );
    }
    plot.add("cc:tutu", 0.0);
    plot.add("cc:tutu", 0.0);
    plot
        .rangeX(0.0, 120.0).rangeY(-20.0, 20.0)
        .plot("index", "all")
        .plot("index", "aa:*", leph::Plot::Lines)
        .plot("index", "aa:titi", leph::Plot::Points)
        .plot("index", "bb:tata", leph::Plot::Points, "aa:toto")
        .show();
    plot
        .rangeX(0.0, 120.0).rangeY(-20.0, 20.0)
        .plot("index", "all")
        .plot("index", "aa:titi", leph::Plot::Points)
        .plot("index", "bb:tata", leph::Plot::Points, "aa:toto")
        .show("/tmp/testPlot.plot");
    plot
        .rangeUniform()
        .plot("index", "all")
        .show();

    plot.add(
        "t", 1.0,
        "val", 1.0
    );
    plot.add(
        "t", 2.0,
        "val", 3.0
    );
    plot.add(
        "t", 3.0,
        "val", 5.0
    );
    plot
        .rangeX(0.0, 120.0).rangeY(-20.0, 20.0)
        .multiplot(1, 2)
        .plot("index", "bb:tata", leph::Plot::Points, "aa:toto")
        .nextPlot()
        .plot("index", "aa:titi", leph::Plot::Points)
        .nextPlot()
        .plot("t", "val")
        .plot("index", "aa:toto", leph::Plot::Points)
        .show();
    plot.writeData();

    return 0;
}

