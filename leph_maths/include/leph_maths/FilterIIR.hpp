#ifndef LEPH_MATHS_FILTERIIR_HPP
#define LEPH_MATHS_FILTERIIR_HPP

#include <cmath>
#include <vector>
#include <stdexcept>
#include <mkfilter/MKFilter.h>

namespace leph {

/**
 * FilterIIR
 *
 * Implement type generic Infinite Impulse Response
 * filter where the coefficients are generated
 * using the mkfilter command from 
 * Tony Fisher, fisher@minster.york.ac.uk 
 *
 * Running the filter is RT safe but 
 * not computing the IIR coefficients.
 */
template <typename T>
class FilterIIR
{
    public:

        /**
         * Default constructor.
         * The internal filter is not initialized.
         * init() must be cal before update().
         */
        FilterIIR() :
            _bufferIn(),
            _bufferOut(),
            _coefIn(),
            _coefOut(),
            _gain(0.0),
            _head(0),
            _isInitialized(false),
            _paramTypeFilter(),
            _paramTypePass(),
            _paramRipple(0.0),
            _paramOrder(0),
            _paramFreqSampling(0.0),
            _paramFreqCorner1(0.0),
            _paramFreqCorner2(0.0),
            _valueZero()
        {
            //Set to zero
            _valueZero *= 0.0;
        }

        /**
         * Initialize and generate 
         * the IIR filter coefficients.
         * Note: NOT real time.
         *
         * @param filter Filter type.
         * @param pass Pass type.
         * @param ripple Passband ripple in dB only 
         * for Chebyshev filter type. Else not used.
         * @param order Filter order in 1:10.
         * @param freqSampling Input sampling frequency in Hz.
         * @param freqCorner1 First corder frequency in Hz.
         * @param freqCorner2 Second corner frequency in Hz.
         * Upper corner frequency used for Bandpass and 
         * Bandstop pass types. Else not used.
         * 0 < freqCorner1/freqSampling < 0.5
         * 0 < freqCorner2/freqSampling < 0.5
         * freqCorner1/freqSampling < freqCorner2/freqSampling
         */
        void init(
            mkfilter::FilterType_t filter, 
            mkfilter::PassType_t pass, 
            double ripple,
            unsigned int order, 
            double freqSampling,
            double freqCorner1,
            double freqCorner2)
        {
            //Assign parameters
            _paramTypeFilter = filter;
            _paramTypePass = pass;
            _paramRipple = ripple;
            _paramOrder = order;
            _paramFreqSampling = freqSampling;
            _paramFreqCorner1 = freqCorner1;
            _paramFreqCorner2 = freqCorner2;
            //Build filter
            _coefIn.clear();
            _coefOut.clear();
            _gain = 0.0;
            _bufferIn.clear();
            _bufferOut.clear();
            mkfilter::MKFilter(
                filter, pass, ripple, order, 
                freqCorner1/freqSampling, 
                freqCorner2/freqSampling, 
                _coefIn, _coefOut, _gain);
            _bufferIn.assign(_coefIn.size(), T());
            _bufferOut.assign(_coefOut.size(), T());
            reset(); 
        }

        /**
         * Update the internal filtered 
         * state with in input value.
         *
         * @param value The new input value to be filtered.
         */
        void update(const T& input)
        {
            if (
                _coefIn.size() == 0 || 
                _coefOut.size() == 0
            ) {
                throw std::logic_error(
                    "leph::FilterIIR::update: "
                    "Filter not initialized.");
            }

            //Initialization if needed
            if (!_isInitialized) {
                reset(input);
            }

            //Append the new value to 
            //input rolling buffer
            _bufferIn[rollingIndex(0)] = (1.0/_gain)*input;

            //Compute next filtered output value
            //Note: use *= to reset to reset to zero for 
            //both double and Eigen vector (template type)
            _bufferOut[rollingIndex(0)] *= 0.0; 
            for (int i=0;i<(int)_bufferIn.size();i++) {
                _bufferOut[rollingIndex(0)] += _coefIn[i]*_bufferIn[rollingIndex(i)];
            }
            for (int i=1;i<(int)_bufferOut.size();i++) {
                _bufferOut[rollingIndex(0)] += _coefOut[i]*_bufferOut[rollingIndex(i)];
            }

            //Update rolling buffer index
            _head = (_head+1) % _bufferIn.size();
        }
        
        /**
         * @return the last output filtered value
         * or zero if not yet initialized
         */
        const T& value() const
        {
            if (_isInitialized) {
                return _bufferOut[rollingIndex(1)];
            } else {
                return _valueZero;
            }
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
         * Should be called after init().
         *
         * @param value Value to be assign
         * to current internal state.
         */
        void reset(const T& value)
        {
            if (
                _coefIn.size() == 0 || 
                _coefOut.size() == 0
            ) {
                throw std::logic_error(
                    "leph::FilterIIR::reset: "
                    "Filter not initialized.");
            }

            _isInitialized = true;
            _head = 0;
            for (size_t i=0;i<_bufferIn.size();i++) {
                _bufferIn[i] = (1.0/_gain)*value;
            }
            for (size_t i=0;i<_bufferOut.size();i++) {
                _bufferOut[i] = value;
            }
        }

        /**
         * @return current filter parameters
         * set with init()).
         */
        mkfilter::FilterType_t getParamTypeFilter() const
        {
            return _paramTypeFilter;
        }
        mkfilter::PassType_t getParamTypePass() const
        {
            return _paramTypePass;
        }
        double getParamRipple() const
        {
            return _paramRipple;
        }
        unsigned int getParamOrder() const
        {
            return _paramOrder;
        }
        double getParamFreqSampling() const
        {
            return _paramFreqSampling;
        }
        double getParamFreqCorner1() const
        {
            return _paramFreqCorner1;
        }
        double getParamFreqCorner2() const
        {
            return _paramFreqCorner2;
        }

    private:

        /**
         * Rolling buffer for input 
         * and output values
         */
        std::vector<T> _bufferIn;
        std::vector<T> _bufferOut;

        /**
         * Infinite impulse response coefficient
         * over input and output values
         */
        std::vector<double> _coefIn;
        std::vector<double> _coefOut;

        /**
         * Infinite impulse response 
         * gain over input data
         */
        double _gain;

        /**
         * Begin index inside rolling buffers.
         * Index to the next cell to write.
         */
        int _head;

        /**
         * If false, the current state is 
         * not yet initialized and should be
         * set at next update() call.
         */
        bool _isInitialized;

        /**
         * Current filter parameters
         */
        mkfilter::FilterType_t _paramTypeFilter;
        mkfilter::PassType_t _paramTypePass;
        double _paramRipple;
        unsigned int _paramOrder;
        double _paramFreqSampling;
        double _paramFreqCorner1;
        double _paramFreqCorner2;

        /**
         * Constant default zero value used 
         * to be returned in value() function
         */
        T _valueZero;
        
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

