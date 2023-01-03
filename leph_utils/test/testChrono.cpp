#include <iostream>
#include <leph_utils/Chrono.hpp>

int main()
{
    leph::Chrono chrono;
    chrono.start("init");
    chrono.stop("init");
    chrono.start("all");
    for (size_t i=0;i<1000;i++) {
        chrono.start("iteration");
        chrono.start("loop1");
        for (volatile size_t j=0;j<10000;j++) {
        }
        chrono.stop("loop1");
        chrono.start("loop2");
        for (volatile size_t j=0;j<20000;j++) {
        }
        chrono.stop("loop2");
        chrono.start("loop3");
        for (volatile size_t j=0;j<50000;j++) {
        }
        chrono.stop("loop3");
        chrono.start("loop4");
        if (i < 100) {
            chrono.start("cond_loop1");
            for (volatile size_t j=0;j<50000;j++) {
            }
            chrono.stop("cond_loop1");
        } else {
            chrono.start("cond_loop2");
            for (volatile size_t j=0;j<50000;j++) {
            }
            chrono.stop("cond_loop2");
        }
        chrono.stop("loop4");
        chrono.stop("iteration");
    }
    chrono.stop("all");
    chrono.start("end");
    chrono.stop("end");
    chrono.print();

    return 0;
}

