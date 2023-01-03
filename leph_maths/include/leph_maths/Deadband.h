#ifndef LEPH_MATHS_DEADBAND_H
#define LEPH_MATHS_DEADBAND_H

namespace leph {

/**
 * Deadband filter around zero.
 *
 * @param value Value to be filtered.
 * @param width Half range width (centered around zero)
 * on which the input value is zeroed.
 * @param isContinuous If false, the neutral zone is simply applied
 * and a discontinuity at +/- width value is present. 
 * If true, an offset is applied on value to have smooth linear slope.
 *
 * @return the filtered value.
 */
inline double Deadband(
    double value, double width, bool isContinuous)
{
    if (width < 0.0) {
        width = 0.0;
    }

    if (isContinuous) {
        if (value >= width) {
            return value - width;
        } else if (value <= -width) {
            return value + width;
        } else {
            return 0.0;
        }
    } else {
        if (std::fabs(value) <= width) {
            return 0.0;
        } else {
            return value;
        }
    }
}

}

#endif

