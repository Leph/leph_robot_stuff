#ifndef LEPH_MATHS_FILTERENCODERDIFFERENTIATION_HPP
#define LEPH_MATHS_FILTERENCODERDIFFERENTIATION_HPP

#include <cmath>
#include <stdexcept>
#include <leph_utils/CircularBuffer.hpp>
#include <leph_maths/Clamp.h>

namespace leph {

/**
 * FilterEncoderDifferentiation
 *
 * Differentiate velocity from discrete
 * encoder positions applying edges detection
 * and finite difference then low pass filter.
 * Real time safe.
 */
class FilterEncoderDifferentiation
{
    public:

        /**
         * Position encoder resolution quantum
         */
        double posQuantum;

        /**
         * Time sampling and update frequency
         */
        double freqSampling;

        /**
         * Encoder position and velocity estimated 
         * and low pass filtered computed signals
         */
        double posEstimated;
        double velEstimated;
        double velFiltered;

    public:

        /**
         * Default initialization
         */
        FilterEncoderDifferentiation() :
            posQuantum(0.0),
            freqSampling(500.0),
            posEstimated(0.0),
            velEstimated(0.0),
            velFiltered(0.0),
            _cycle(0),
            _lastPosition(0.0),
            _edges(),
            _isInitialized(false)
        {
            reset();
        }

        /**
         * Update internal state filter and compute
         * estimated and filtered encoder signals.
         *
         * @param readPosition Read discrete 
         * position measurement.
         * @param minTimeCycles The desired minimum number 
         * of time ticks between edge finite difference.
         * @param cutoffFreq Final low pass filter 
         * on velocity cutoff frequency.
         */
        void update(
            double readPosition, 
            unsigned int minTimeCycles,
            double cutoffFreq)
        {
            //Bound parameter values
            if (minTimeCycles < 1) {
                minTimeCycles = 1;
            }
            if (cutoffFreq < 0.0) {
                cutoffFreq = 0.0;
            }

            //Run initialization if needed
            bool needInit = !_isInitialized;
            if (needInit) {
                reset(readPosition);
            }

            //Update cycle counter
            _cycle++;

            //Position change detection
            if (std::fabs(readPosition-_lastPosition) >= posQuantum) {
                //Append detected edge in circular buffer
                struct Edge_t edge;
                edge.cycle = _cycle;
                edge.value = 0.5*readPosition + 0.5*_lastPosition;
                _edges.append(edge);
                _lastPosition = readPosition;
            }

            //Select the past edge index to use 
            //for differentiation.
            //Going back in time until minimum time 
            //tick interval is meet.
            unsigned int indexPast = 1;
            while (
                indexPast+1 < _edges.size() && 
                (_edges.get(0).cycle 
                    - _edges.get(indexPast+1).cycle) < minTimeCycles
            ) {
                indexPast++;
            }

            if (indexPast < _edges.size()) {
                //Velocity estimation from edge finite differences
                velEstimated = 
                    freqSampling
                    *(_edges.get(0).value - _edges.get(indexPast).value)
                    /(double)(_edges.get(0).cycle - _edges.get(indexPast).cycle);
                //Retrieve previous edge (excluding current one if any)
                const Edge_t edgeLast = 
                    (_cycle > _edges.get(0).cycle) ? 
                    _edges.get(0) : 
                    _edges.get(1);
                //Compute current position from 
                //last edge and estimated velocity
                posEstimated = 
                    edgeLast.value 
                    + velEstimated*(double)(_cycle - edgeLast.cycle)/freqSampling;
                //Clamp estimated position to 
                //comply with measurement
                posEstimated = ClampRange(
                    posEstimated, 
                    readPosition - posQuantum, 
                    readPosition + posQuantum);
                //Correct velocity estimation (apply only if 
                //the position has been clamped)
                velEstimated = 
                    freqSampling
                    *(posEstimated - edgeLast.value)
                    /(double)(_cycle - edgeLast.cycle);
            } else {
                posEstimated = readPosition;
                velEstimated = 0.0;
            }

            //Exponential low pass filtering (see FilterExponential)
            double dt = 1.0/freqSampling;
            double omega = 2.0*M_PI*cutoffFreq;
            double coeff = (1.0-omega*dt/2.0)/(1.0+omega*dt/2.0);
            coeff = ClampRange(coeff, 0.0, 1.0);
            //Do not apply low pass filter on initialization
            if (!needInit) {
                velFiltered = coeff*velFiltered + (1.0-coeff)*velEstimated;
            } 
        }

        /**
         * Reset the internal state with given 
         * initial encoder position value
         */
        void reset(double initPosition)
        {
            reset();
            _lastPosition = initPosition;
            _isInitialized = true;
        }

        /**
         * Reset the internal filter state
         */
        void reset()
        {
            posEstimated = 0.0;
            velEstimated = 0.0;
            velFiltered = 0.0;
            _cycle = 0;
            _lastPosition = 0.0;
            _edges.clear();
            _isInitialized = false;
        }

    private:

        /**
         * Structure for encoder edge events
         */
        struct Edge_t {
            unsigned long cycle;
            double value;
        };

        /**
         * Current cycle counter
         */
        unsigned long _cycle;

        /**
         * Last encoder position value
         */
        double _lastPosition;

        /**
         * Circular buffer of detected edges
         */
        CircularBuffer<Edge_t, 100> _edges;

        /**
         * If false, the current state is 
         * not yet initialized and should be
         * set at next update() call.
         */
        bool _isInitialized;
};

}

#endif

