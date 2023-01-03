#ifndef LEPH_MATHS_CONTROLDAMPEDINTEGRAL_HPP
#define LEPH_MATHS_CONTROLDAMPEDINTEGRAL_HPP

#include <cmath>
#include <leph_maths/Clamp.h>
#include <leph_maths/FilterIIR.hpp>

namespace leph {

/**
 * ControlDampedIntegral
 *
 * Implement a single degree of freedom 
 * integral control in parallel with a spring damper 
 * stabilization
 */
class ControlDampedIntegral
{
    public:

        /**
         * Neutral position for the 
         * stabilization spring damper control
         */
        double posNeutral;

        /**
         * Target position for the 
         * integral control
         */
        double posTarget;

        /**
         * Gains for position and derivative 
         * for the spring damper stabilization
         */
        double gainStabilizationP;
        double gainStabilizationD;

        /**
         * Gains for position and derivative 
         * for the integration part
         */
        double gainIntegralP;
        double gainIntegralD;

        /**
         * Maximum control effort saturation limit
         */
        double effortMaximum;

        /**
         * Cutoff frequency associated with 
         * the low pass filters 
         * (use to easily update through RhIO)
         */
        double freqFilterStatePos;
        double freqFilterStateVel;
        double freqFilterErrorPos;
        double freqFilterErrorVel;
        
        /**
         * Filter order
         * (only updated at cutoff frequency change)
         */
        unsigned int orderFilters;

        /**
         * Integrated control effort term
         */
        double effortIntegral;

        /**
         * Anti windup control effort term
         */
        double effortAntiWindup;

        /**
         * Computed and clamped control effort term
         */
        double effortControl;

    public:

        /**
         * Initialization.
         *
         * @param frequencySampling Control and filtering 
         * sampling frequency
         */
        ControlDampedIntegral(double frequencySampling) :
            posNeutral(0.0),
            posTarget(0.0),
            gainStabilizationP(0.0),
            gainStabilizationD(0.0),
            gainIntegralP(0.0),
            gainIntegralD(0.0),
            effortMaximum(1e9),
            freqFilterStatePos(0.0),
            freqFilterStateVel(0.0),
            freqFilterErrorPos(0.0),
            freqFilterErrorVel(0.0),
            orderFilters(3),
            effortIntegral(0.0),
            effortAntiWindup(0.0),
            effortControl(0.0),
            _frequencySampling(frequencySampling),
            _filterStatePos(),
            _filterStateVel(),
            _filterErrorPos(),
            _filterErrorVel(),
            _previousErrorPos(0.0)
        {
        }

        /**
         * Update and compute the integral control.
         *
         * @param posRead Raw position signal from sensor.
         * @param velRead Raw velocity signal from sensor.
         * @return computed control effort.
         */
        double update(double posRead, double velRead)
        {
            //Reset the filters if cutoff frequency has changed
            configureFilters();
            //Update filtering
            double dt = 1.0/_frequencySampling;
            double errorPos = posTarget - posRead;
            double errorVel = (errorPos - _previousErrorPos)/dt;
            _previousErrorPos = errorPos;
            _filterStatePos.update(posNeutral - posRead);
            _filterStateVel.update(velRead);
            _filterErrorPos.update(errorPos);
            _filterErrorVel.update(errorVel);
            //Compute control scheme
            double effortDelta = 
                gainIntegralP*_filterErrorPos.value()
                + gainIntegralD*_filterErrorVel.value();
            effortIntegral += dt*effortDelta + effortAntiWindup;
            effortControl = 
                getEffortStabilizationPD()
                + effortIntegral;
            effortAntiWindup = ClampDistFromAbsolute(effortControl, effortMaximum);
            effortControl = ClampAbsolute(effortControl, effortMaximum);

            return effortControl;
        }

        /**
         * @return control effort from stabilization
         * parallel spring and damper part
         */
        double getEffortStabilizationP() const
        {
            return gainStabilizationP*_filterStatePos.value();
        }
        double getEffortStabilizationD() const
        {
            return -gainStabilizationD*_filterStateVel.value();
        }
        double getEffortStabilizationPD() const
        {
            return 
                getEffortStabilizationP() + 
                getEffortStabilizationD();
        }

        /**
         * @return filtered internal signals
         */
        double getFilteredStatePos() const
        {
            return _filterStatePos.value();
        }
        double getFilteredStateVel() const
        {
            return _filterStateVel.value();
        }
        double getFilteredErrorPos() const
        {
            return _filterErrorPos.value();
        }
        double getFilteredErrorVel() const
        {
            return _filterErrorVel.value();
        }

    private:

        /**
         * Control and filtering fixed frequency
         */
        double _frequencySampling;

        /**
         * Filter instances for current position, 
         * current velocity, current error and its derivatives
         */
        FilterIIR<double> _filterStatePos;
        FilterIIR<double> _filterStateVel;
        FilterIIR<double> _filterErrorPos;
        FilterIIR<double> _filterErrorVel;

        /**
         * Previous position error used for
         * derivative computation
         */
        double _previousErrorPos;

        /**
         * Reset and configure the filters of the cutoff 
         * frequency has changed
         */
        void configureFilters()
        {
            bool isChangedStatePos = (std::fabs(
                _filterStatePos.getParamFreqCorner1()
                - freqFilterStatePos) > 1e-6);
            bool isChangedStateVel = (std::fabs(
                _filterStateVel.getParamFreqCorner1()
                - freqFilterStateVel) > 1e-6);
            bool isChangedErrorPos = (std::fabs(
                _filterErrorPos.getParamFreqCorner1()
                - freqFilterErrorPos) > 1e-6);
            bool isChangedErrorVel = (std::fabs(
                _filterErrorVel.getParamFreqCorner1()
                - freqFilterErrorVel) > 1e-6);

            if (freqFilterStatePos > 0.0 && isChangedStatePos) {
                _filterStatePos.init(
                    mkfilter::FilterType_t::Butterworth,
                    mkfilter::PassType_t::Lowpass,
                    0.0, 
                    orderFilters, 
                    _frequencySampling, freqFilterStatePos, 0.0);
            }
            if (freqFilterStateVel > 0.0 && isChangedStateVel) {
                _filterStateVel.init(
                    mkfilter::FilterType_t::Butterworth,
                    mkfilter::PassType_t::Lowpass,
                    0.0, 
                    orderFilters, 
                    _frequencySampling, freqFilterStateVel, 0.0);
            }
            if (freqFilterErrorPos > 0.0 && isChangedErrorPos) {
                _filterErrorPos.init(
                    mkfilter::FilterType_t::Butterworth,
                    mkfilter::PassType_t::Lowpass,
                    0.0, 
                    orderFilters, 
                    _frequencySampling, freqFilterErrorPos, 0.0);
            }
            if (freqFilterErrorVel > 0.0 && isChangedErrorVel) {
                _filterErrorVel.init(
                    mkfilter::FilterType_t::Butterworth,
                    mkfilter::PassType_t::Lowpass,
                    0.0, 
                    orderFilters, 
                    _frequencySampling, freqFilterErrorVel, 0.0);
            }
        }
};

}

#endif

