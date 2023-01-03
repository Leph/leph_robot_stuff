#ifndef LEPH_MATHS_FILTERBACKLASH_HPP
#define LEPH_MATHS_FILTERBACKLASH_HPP

#include <cmath>
#include <stdexcept>

namespace leph {

/**
 * FilterBacklash
 *
 * Implement a filter backlash
 * on joint velocity.
 * Based on MIT and IHMC similar filters.
 * https://github.com/ihmcrobotics/ihmc-open-robotics-software/blob/develop/ihmc-robotics-toolkit/src/main/java/us/ihmc/robotics/math/filters/BacklashCompensatingVelocityYoVariable.java
 * https://github.com/openhumanoids/pronto/blob/cf091a4218649a6f0ae269ad22be086fcbe67950/estimate_tools/src/backlash_filter_tools/velocity_backlash_filter.h
 */
class FilterBacklash
{
    public:

        /**
         * Initialization.
         *
         * @param timeSlop Time length in 
         * seconds of the ramping up slop state.
         */
        FilterBacklash(double timeSlop) :
            _isInitialized(false),
            _state(State_t::FORWARD_OK),
            _timeAtSloppy(0.0),
            _timeSlop(timeSlop),
            _filteredVel(0.0)
        {
            if (_timeSlop < 0.0) {
                _timeSlop = 0.0;
            }
            reset();
        }
        
        /**
         * Update the internal filtered 
         * velocity.
         *
         * @param time Value timestamp 
         * in seconds.
         * @param value The new input 
         * velocity to be filtered.
         */
        void update(double time, double inputVel)
        {
            if (!_isInitialized) {
                reset(inputVel);
            }

            if (_state == State_t::FORWARD_OK) {
                if (inputVel < 0.0) {
                    _timeAtSloppy = time;
                    _state = State_t::BACKWARD_SLOP;
                }
            } else if (_state == State_t::BACKWARD_OK) {
                if (inputVel > 0.0) {
                    _timeAtSloppy = time;
                    _state = State_t::FORWARD_SLOP;
                }
            } else if (_state == State_t::FORWARD_SLOP) {
                if (inputVel < 0.0) {
                    _timeAtSloppy = time;
                    _state = State_t::BACKWARD_SLOP;
                } else if (time - _timeAtSloppy > _timeSlop) {
                    _timeAtSloppy = 0.0;
                    _state = State_t::FORWARD_OK;
                }
            } else if (_state == State_t::BACKWARD_SLOP) {
                if (inputVel > 0.0) {
                    _timeAtSloppy = time;
                    _state = State_t::FORWARD_SLOP;
                } else if (time - _timeAtSloppy > _timeSlop) {
                    _timeAtSloppy = 0.0;
                    _state = State_t::BACKWARD_OK;
                }
            } else {
                _filteredVel = 0.0;
            }

            if (
                _timeSlop > 0.0 &&
                (_state == State_t::FORWARD_SLOP ||
                _state == State_t::BACKWARD_SLOP)
            ) {
                double ratio = (time - _timeAtSloppy)/_timeSlop;
                if (ratio < 0.0) {
                    ratio = 0.0;
                }
                if (ratio > 1.0) {
                    ratio = 1.0;
                }
                _filteredVel = ratio*inputVel;
            } else {
                _filteredVel = inputVel;
            }
        }
        
        /**
         * @return read access to filtered value
         */
        const double& value() const
        {
            return _filteredVel;
        }
        
        /**
         * Set the internal state as 
         * uninitialized. Its value will be 
         * assigned at next call of update().
         */
        void reset()
        {
            _isInitialized = false;
        }

        /**
         * Reset the internal state to
         * given value.
         *
         * @param value Value to be assign
         * to current internal state.
         */
        void reset(double value)
        {
            _isInitialized = true;
            if (value >= 0.0) {
                _state = State_t::FORWARD_OK;
            } else {
                _state = State_t::BACKWARD_OK;
            }
            _timeAtSloppy = 0.0;
            _filteredVel = value;
        }

    private:

        /**
         * Filter state
         */
        enum class State_t {
            //Normal positive velocity
            FORWARD_OK,
            //Normal negative velocity
            BACKWARD_OK,
            //Positive velocity while
            //the slop (ramping up) is active
            FORWARD_SLOP,
            //Negative velocity while 
            //the slop (ramping up) is active
            BACKWARD_SLOP,
        };
        
        /**
         * If false, the current state is 
         * not yet initialized and should be
         * set at next update() call.
         */
        bool _isInitialized;

        /**
         * Current filter state
         */
        State_t _state;

        /**
         * Last time when either 
         * FORWARD_SLOP or BACKWARD_SLOP
         * state was entered.
         */
        double _timeAtSloppy;

        /**
         * Time length in seconds 
         * of the ramping up slop state
         */
        double _timeSlop;

        /**
         * Computed filtered velocity
         */
        double _filteredVel;
};

}

#endif

