#ifndef LEPH_MATHS_FILTERDIFFERENTIATOR_HPP
#define LEPH_MATHS_FILTERDIFFERENTIATOR_HPP

#include <cmath>
#include <vector>
#include <stdexcept>

namespace leph {

/**
 * FilterDifferentiator
 *
 * Implement a smooth differentiator 
 * filter based on: 
 * http://www.holoborodko.com/pavel/numerical-methods/numerical-derivative/smooth-low-noise-differentiators/
 * by Pavel Holoborodko
 * More precisely, http://www.holoborodko.com/pavel/wp-content/plugins/download-monitor/download.php?id=7
 */
class FilterDifferentiator
{
    public:

        /**
         * Initialization.
         *
         * @param historySize Number of used 
         * past data points to compute the derivative 
         * (between 1 and 10).
         * @param samplingFreq Input signal sampling 
         * frequency in Hz.
         */
        FilterDifferentiator(
            unsigned int historySize, 
            double samplingFreq) :
            _historySize(historySize),
            _samplingFreq(samplingFreq),
            _diffValue(0.0),
            _isInitialized(false),
            _head(0),
            _bufferIn()
        {
            if (historySize < 1 || historySize > 10) {
                throw std::logic_error(
                    "leph::FilterDifferentiator: Invalid history size");
            }
            _bufferIn.resize(_historySize+1);
            reset();
        }

        /**
         * Update the internal filtered 
         * state with in input value.
         *
         * @param value The new input value 
         * to be differentiated.
         */
        void update(double input)
        {
            if (!_isInitialized) {
                reset(input);
            }
            double dt = 1.0/_samplingFreq;

            //Append the new value to 
            //input rolling buffer
            _bufferIn[rollingIndex(0)] = input;

            //Implement the one sided differentiator
            //from Pavel Holoborodko
            if (_historySize == 1) {
                _diffValue = (
                    _bufferIn[rollingIndex(0)]
                    - _bufferIn[rollingIndex(1)]
                    )/(dt);
            } else if (_historySize == 2) {
                _diffValue = (
                    _bufferIn[rollingIndex(0)]
                    - _bufferIn[rollingIndex(2)]
                    )/(2.0*dt);
            } else if (_historySize == 3) {
                _diffValue = (
                    _bufferIn[rollingIndex(0)]
                    + _bufferIn[rollingIndex(1)]
                    - _bufferIn[rollingIndex(2)]
                    - _bufferIn[rollingIndex(3)]
                    )/(4.0*dt);
            } else if (_historySize == 4) {
                _diffValue = (
                    _bufferIn[rollingIndex(0)]
                    + 2.0*_bufferIn[rollingIndex(1)]
                    - 2.0*_bufferIn[rollingIndex(3)]
                    - _bufferIn[rollingIndex(4)]
                    )/(8.0*dt);
            } else if (_historySize == 5) {
                _diffValue = (
                    _bufferIn[rollingIndex(0)]
                    + 3.0*_bufferIn[rollingIndex(1)]
                    + 2.0*_bufferIn[rollingIndex(2)]
                    - 2.0*_bufferIn[rollingIndex(3)]
                    - 3.0* _bufferIn[rollingIndex(4)]
                    - _bufferIn[rollingIndex(5)]
                    )/(16.0*dt);
            } else if (_historySize == 6) {
                _diffValue = (
                    _bufferIn[rollingIndex(0)]
                    + 4.0*_bufferIn[rollingIndex(1)]
                    + 5.0*_bufferIn[rollingIndex(2)]
                    - 5.0*_bufferIn[rollingIndex(4)]
                    - 4.0*_bufferIn[rollingIndex(5)]
                    - _bufferIn[rollingIndex(6)]
                    )/(32.0*dt);
            } else if (_historySize == 7) {
                _diffValue = (
                    _bufferIn[rollingIndex(0)]
                    + 5.0*_bufferIn[rollingIndex(1)]
                    + 9.0*_bufferIn[rollingIndex(2)]
                    + 5.0*_bufferIn[rollingIndex(3)]
                    - 5.0*_bufferIn[rollingIndex(4)]
                    - 9.0*_bufferIn[rollingIndex(5)]
                    - 5.0*_bufferIn[rollingIndex(6)]
                    - _bufferIn[rollingIndex(7)]
                    )/(64.0*dt);
            } else if (_historySize == 8) {
                _diffValue = (
                    _bufferIn[rollingIndex(0)]
                    + 6.0*_bufferIn[rollingIndex(1)]
                    + 14.0*_bufferIn[rollingIndex(2)]
                    + 14.0*_bufferIn[rollingIndex(3)]
                    - 14.0*_bufferIn[rollingIndex(5)]
                    - 14.0*_bufferIn[rollingIndex(6)]
                    - 6.0*_bufferIn[rollingIndex(7)]
                    - _bufferIn[rollingIndex(8)]
                    )/(128.0*dt);
            } else if (_historySize == 9) {
                _diffValue = (
                    _bufferIn[rollingIndex(0)]
                    + 7.0*_bufferIn[rollingIndex(1)]
                    + 20.0*_bufferIn[rollingIndex(2)]
                    + 28.0*_bufferIn[rollingIndex(3)]
                    + 14.0*_bufferIn[rollingIndex(4)]
                    - 14.0*_bufferIn[rollingIndex(5)]
                    - 28.0*_bufferIn[rollingIndex(6)]
                    - 20.0*_bufferIn[rollingIndex(7)]
                    - 7.0*_bufferIn[rollingIndex(8)]
                    - _bufferIn[rollingIndex(9)]
                    )/(256.0*dt);
            } else if (_historySize == 10) {
                _diffValue = (
                    _bufferIn[rollingIndex(0)]
                    + 8.0*_bufferIn[rollingIndex(1)]
                    + 27.0*_bufferIn[rollingIndex(2)]
                    + 48.0*_bufferIn[rollingIndex(3)]
                    + 42.0*_bufferIn[rollingIndex(4)]
                    - 42.0*_bufferIn[rollingIndex(6)]
                    - 48.0*_bufferIn[rollingIndex(7)]
                    - 27.0*_bufferIn[rollingIndex(8)]
                    - 8.0*_bufferIn[rollingIndex(9)]
                    - _bufferIn[rollingIndex(10)]
                    )/(512.0*dt);
            } else {
                _diffValue = 0.0;
            }
            
            //Update rolling buffer index
            _head = (_head+1) % _bufferIn.size();
        }
        
        /**
         * @return read access to filtered 
         * differentiated value.
         */
        const double& value() const
        {
            return _diffValue;
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
            _diffValue = 0.0;
            _isInitialized = true;
            _head = 0;
            for (size_t i=0;i<_historySize+1;i++) {
                _bufferIn[i] = value;
            }
        }

    private:

        /**
         * Number of used past data 
         * points to compute the derivative 
         * (between 2 and 10)
         */
        unsigned int _historySize;

        /**
         * Input signal sampling 
         * frequency in Hz
         */
        double _samplingFreq;

        /**
         * The last computed signal
         * differentiation
         */
        double _diffValue;
        
        /**
         * If false, the current state is 
         * not yet initialized and should be
         * set at next update() call.
         */
        bool _isInitialized;

        /**
         * Begin index inside rolling buffer.
         * Index to the next cell to write.
         */
        int _head;
        
        /**
         * Rolling buffer for 
         * input values
         */
        std::vector<double> _bufferIn;
        
        /**
         * Utility function computing buffer 
         * index in rolling buffer.
         *
         * @param pos Expected position relative to head.
         * @return the index in rolling buffer.
         */
        size_t rollingIndex(int pos) const
        {
            int size = _bufferIn.size();
            int index = _head - pos;
            while (index < 0) {
                index += size;
            }
            return index % size;
        }
};

}

#endif

