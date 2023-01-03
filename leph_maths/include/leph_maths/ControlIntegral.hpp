#ifndef LEPH_MATHS_CONTROLINTEGRAL_HPP
#define LEPH_MATHS_CONTROLINTEGRAL_HPP

#include <stdexcept>
#include <cmath>
#include <leph_maths/Sign.h>
#include <leph_maths/Clamp.h>

namespace leph {

/**
 * ControlIntegral
 *
 * Implement error integral control
 * with safety checks and limits.
 */
class ControlIntegral
{
    public:

        /**
         * Empty initialization
         */
        ControlIntegral() :
            _integrator(0.0),
            _maxIntegral(0.0),
            _maxDeltaNormal(0.0),
            _rangeProportionalDelta(0.0),
            _maxVelocity(0.0),
            _maxDeltaSafety(0.0)
        {
        }

        /**
         * Set the integral controller parameters.
         *
         * @param maxIntegral Maximum absolute 
         * bound for the integral term.
         * @param maxDeltaNormal Maximum (absolute) integral 
         * term change per second during normal control.
         * @param rangeProportionalDelta Range size 
         * (centered around zero) in witch the integral 
         * term change is proportional to position error.
         * @param maxVelocity Absolute velocity threshold 
         * above which the safety mode integral term 
         * reduction is applied.
         * @param maxDeltaSafety Maximum (absolute) integral 
         * term change per second during safety mode.
         */
        void setConfig(
            double maxIntegral,
            double maxDeltaNormal,
            double rangeProportionalDelta,
            double maxVelocity,
            double maxDeltaSafety)
        {
            if (
                maxIntegral <= 0.0 ||
                maxDeltaNormal <= 0.0 ||
                rangeProportionalDelta <= 0.0 ||
                maxVelocity <= 0.0 ||
                maxDeltaSafety <= 0.0
            ) {
                throw std::logic_error(
                    "leph::ControlIntegral::setConfig: "
                    "Invalid (negative) parameters.");
            }
            _maxIntegral = maxIntegral;
            _maxDeltaNormal = maxDeltaNormal;
            _rangeProportionalDelta = rangeProportionalDelta;
            _maxVelocity = maxVelocity;
            _maxDeltaSafety = maxDeltaSafety;
        }

        /**
         * Update in internal integral term.
         *
         * @param dt Time since last update in seconds.
         * @param errorPos Error to be integrated.
         * @param currentVel Current (filtered) velocity.
         */
        void update(
            double dt,
            double errorPos,
            double currentVel = 0.0)
        {
            //Compute integral change
            double delta = 0.0;
            if (std::fabs(errorPos) < _rangeProportionalDelta) {
                delta = (_maxDeltaNormal/_rangeProportionalDelta)*errorPos;
            } else {
                delta = Sign(errorPos)*_maxDeltaNormal;
            }

            //If the velocity is too high, the integral
            //term absolute value is quickly reduced
            //until zero
            if (std::fabs(currentVel) >= _maxVelocity) {
                if (dt*_maxDeltaSafety > std::fabs(_integrator)) {
                    delta = -_integrator/dt;
                } else {
                    delta = -Sign(_integrator)*_maxDeltaSafety;
                }
            }
            
            //Integrate the accumulator with time step
            _integrator += dt*delta;
            //Clamp the absolute value of the integral term
            _integrator = ClampAbsolute(_integrator, _maxIntegral);
        }

        /**
         * @return the computed integral term
         */
        double integralTerm() const
        {
            return _integrator;
        }

        /**
         * Reset the internal integral 
         * term to given value
         */
        void reset(double value = 0.0)
        {
            _integrator = value;
        }

    private:

        /**
         * Internal integral term
         * being accumulated
         */
        double _integrator;

        /**
         * Maximum absolute bound
         * for the integral term
         */
        double _maxIntegral;

        /**
         * Maximum (absolute) integral term 
         * change per second during
         * normal control
         */
        double _maxDeltaNormal;

        /**
         * Range size (centered around zero)
         * in witch the integral term change
         * is proportional to position error.
         * Outside this position error range, 
         * the integral term change is bound to 
         * +/- _maxDeltaNormal.
         */
        double _rangeProportionalDelta;

        /**
         * Absolute velocity threshold above
         * which the safety mode integral term 
         * reduction is applied
         */
        double _maxVelocity;
        
        /**
         * Maximum (absolute) integral 
         * term change per second during
         * safety mode
         */
        double _maxDeltaSafety;
};

}

#endif

