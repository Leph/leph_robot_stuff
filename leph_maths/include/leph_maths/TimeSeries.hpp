#ifndef LEPH_MATHS_TIMESERIES_HPP
#define LEPH_MATHS_TIMESERIES_HPP

#include <stdexcept>
#include <vector>
#include <cmath>
#include <leph_maths/Angle.h>

namespace leph {

/**
 * TimeSeries
 *
 * Container for time indexed sequence
 * of generic type T. Implement value insertion, 
 * retrieving and interpolation mechanisms.
 */
template <typename T>
class TimeSeries
{
    public:
        
        /**
         * Structure for timed value
         * time is in second
         */
        struct Point {
            double time;
            T value;
        };

        /**
         * Empty initialization
         */
        TimeSeries() :
            _data()
        {
        }
        
        /**
         * @return the number of 
         * currently stored data points
         */
        inline size_t size() const
        {
            return _data.size();
        }
        
        /**
         * Reset the series data to empty
         */
        inline void clear()
        {
            _data.clear();
        }
        
        /**
         * @return oldest and newest
         * inserted point's time
         */
        inline double timeMin() const
        {
            return at(0).time;
        }
        inline double timeMax() const
        {
            return at(size()-1).time;
        }
        
        /**
         * Find and return the index, point or value 
         * whose timestamp is just lower or upper 
         * given time.
         *
         * @param time Timestamp to be 
         * searched into the container.
         */
        inline size_t lowerIndex(double time) const
        {
            size_t indexLow;
            size_t indexUp;
            bisectionSearch(time, indexLow, indexUp);

            return indexLow;
        }
        inline size_t upperIndex(double time) const
        {
            size_t indexLow;
            size_t indexUp;
            bisectionSearch(time, indexLow, indexUp);

            return indexUp;
        }
        inline const Point& lowerPoint(double time) const
        {
            size_t indexLow;
            size_t indexUp;
            bisectionSearch(time, indexLow, indexUp);

            return at(indexLow);
        }
        inline const Point& upperPoint(double time) const
        {
            size_t indexLow;
            size_t indexUp;
            bisectionSearch(time, indexLow, indexUp);

            return at(indexUp);
        }
        inline const T& lowerValue(double time) const
        {
            size_t indexLow;
            size_t indexUp;
            bisectionSearch(time, indexLow, indexUp);

            return at(indexLow).value;
        }
        inline const T& upperValue(double time) const
        {
            size_t indexLow;
            size_t indexUp;
            bisectionSearch(time, indexLow, indexUp);

            return at(indexUp).value;
        }
        
        /**
         * @return last added point's 
         * time and value
         */
        inline double lastTime() const
        {
            return at(size()-1).time;
        }
        inline const T& lastValue() const
        {
            return at(size()-1).value;
        }
        
        /**
         * Direct access to a Point in the container
         * given its index.
         *
         * @param index Valid point index between 0 and size()-1.
         */
        inline const Point& operator[](size_t index) const
        {
            if (index >= size()) {
                throw std::logic_error(
                    "leph::TimeSeries::at: Invalid index: " 
                    + std::to_string(index));
            }
            return _data.at(index);
        }
        inline const Point& at(size_t index) const
        {
            return operator[](index);
        }
        
        /**
         * Compute linear interpolation between
         * lower and upper contained values at given time.
         *
         * @param time Time to interpolate at.
         * @return linearly interpolated value between
         * lower and upper contained values.
         */
        inline T interpolate(double time) const
        {
            size_t indexLow;
            size_t indexUp;
            bisectionSearch(time, indexLow, indexUp);

            //Linear interpolation
            double d1 = at(indexLow).time - at(indexUp).time;
            double d2 = time - at(indexUp).time;
            return ((d1-d2)/d1)*at(indexUp).value + (d2/d1)*at(indexLow).value;
        }

        /**
         * Compute linear interpolation between
         * lower and upper contained values at given time
         * specialized for (cyclic) angular value.
         *
         * @param time Time to interpolate at.
         * @return linearly interpolated value between
         * lower and upper contained values.
         */
        inline T interpolateAngle(double time) const;
        
        /**
         * Append given timed point to the series.
         * Throw error if time is not strictly larger
         * than last stored time.
         *
         * @param time New point time (must be inscreasing).
         * @param value New point value
         */
        inline void append(double time, const T& value)
        {
            if (std::isnan(time)) {
                throw std::logic_error(
                    "leph::TimeSeries::append: Adding NaN time.");
            }
            if (_data.size() > 0 && time <= lastTime()) {
                throw std::logic_error(
                    "leph::TimeSeries::append: Adding past time.");
            }

            //Insert the point
            _data.push_back({time, value});
        }

    private:

        /**
         * Data points container
         */
        std::vector<Point> _data;
        
        /**
         * Process a bijection search onto data internal
         * values container for upper and lower index
         * between given time;
         *
         * @param time Time to search in the container.
         * @param indexLow Output lower index.
         * @param indexUp Output upper index.
         */
        inline void bisectionSearch(
            double time, 
            size_t& indexLow, size_t& indexUp) const
        {
            if (size() < 2) {
                throw std::logic_error(
                    "leph::TimeSeries:bisectionSearch: "
                    "Not enough points.");
            }
            if (time < timeMin() || time > timeMax()) {
                throw std::logic_error(
                    "leph::TimeSeries:bisectionSearch: "
                    "Time out of bound.");
            }

            //Bijection search
            indexLow = 0;
            indexUp = size()-1;
            while (indexUp-indexLow != 1) {
                size_t i = (indexLow+indexUp)/2;
                if (time >= _data.at(i).time) {
                    indexLow = i;
                } else {
                    indexUp = i;
                }
            }
        }
};

template <>
inline double TimeSeries<double>::interpolateAngle(double time) const
{
    size_t indexLow;
    size_t indexUp;
    bisectionSearch(time, indexLow, indexUp);

    //Linear interpolation
    double d1 = at(indexLow).time - at(indexUp).time;
    double d2 = time - at(indexUp).time;
    return AngleWeightedAverage(
        d1-d2, at(indexUp).value, 
        d2, at(indexLow).value);
}

}

#endif

