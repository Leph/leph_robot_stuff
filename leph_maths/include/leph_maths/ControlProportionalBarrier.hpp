#ifndef LEPH_MATHS_CONTROLPROPORTIONALBARRIER_HPP
#define LEPH_MATHS_CONTROLPROPORTIONALBARRIER_HPP

#include <stdexcept>
#include <cmath>
#include <leph_maths/Clamp.h>
#include <leph_maths/Sign.h>

namespace leph {

/**
 * ControlProportionalBarrier
 *
 * Proportional controller with variable gain 
 * used to implement a potential barrier at
 * joint position limits.
 * Real time safe.
 */
class ControlProportionalBarrier
{
    public:

        /**
         * Default initialization
         */
        ControlProportionalBarrier() :
            _boundEffort(0.0),
            _linearStiffness(0.0),
            _barrierPosLower(0.0),
            _barrierPosUpper(0.0),
            _barrierWidth(0.0)
        {
        }

        /**
         * Set controller parameters.
         *
         * @param boundEffort Absolute maximum control effort.
         * @param linearStiffness Classic proportional to error 
         * control gain away from any position barrier.
         * @param barrierPosLower Barrier lower position.
         * @param barrierPosUpper Barrier upper position.
         * @param barrierWidth Position interval where the 
         * control effort barrier is rising up from linear
         * to maximum effort.
         */
        void setParameters(
            double boundEffort,
            double linearStiffness,
            double barrierPosLower,
            double barrierPosUpper,
            double barrierWidth)
        {
            if (boundEffort >= 0.0) {
                _boundEffort = boundEffort;
            } else {
                _boundEffort = 0.0;
            }
            if (linearStiffness >= 0.0) {
                _linearStiffness = linearStiffness;
            } else {
                _linearStiffness = 0.0;
            }
            if (barrierWidth > 0.0 && barrierPosUpper > barrierPosLower) {
                _barrierPosLower = barrierPosLower;
                _barrierPosUpper = barrierPosUpper;
                _barrierWidth = barrierWidth;
                if (barrierPosUpper-barrierPosLower < 2.0*barrierWidth) {
                    _barrierWidth = 0.5*(barrierPosUpper-barrierPosLower);
                }
            } else {
                _barrierPosLower = 0.0;
                _barrierPosUpper = 0.0;
                _barrierWidth = 0.0;
            }
        }

        /**
         * Compute the control effort given
         * current and desired position.
         *
         * @param targetPos Desired target position.
         * @param currentPos Current measured position.
         * @return control effort.
         */
        double computeControlEffort(double targetPos, double currentPos)
        {
            //Barrier saturation cases
            if (_barrierWidth > 0.0 && currentPos > _barrierPosUpper) {
                return -_boundEffort;
            }
            if (_barrierWidth > 0.0 && currentPos < _barrierPosLower) {
                return _boundEffort;
            }

            //Clamp target position within barrier range
            if (_barrierWidth > 0.0 && targetPos >= _barrierPosUpper) {
                targetPos = _barrierPosUpper;
            }
            if (_barrierWidth > 0.0 && targetPos <= _barrierPosLower) {
                targetPos = _barrierPosLower;
            }

            //Compute linear (proportional) control effort
            double error = targetPos - currentPos;
            double effortLinear = _linearStiffness*error;

            //Compute lower and upper barrier effort
            //if parameters are in valid range
            double effortBarrier = 0.0;
            if (_barrierWidth > 0.0) { 
                if (currentPos >= _barrierPosUpper-_barrierWidth) {
                    double tmpDelta = (currentPos-_barrierPosUpper+_barrierWidth)/_barrierWidth;
                    double tmpCoef = _boundEffort - std::fabs(effortLinear);
                    effortBarrier -= tmpCoef*std::pow(tmpDelta, 3);
                }
                
                if (currentPos <= _barrierPosLower+_barrierWidth) {
                    double tmpDelta = (_barrierPosLower+_barrierWidth-currentPos)/_barrierWidth;
                    double tmpCoef = _boundEffort - std::fabs(effortLinear);
                    effortBarrier += tmpCoef*std::pow(tmpDelta, 3);
                }
            }
            
            //Compute total effort and clamp it
            //to maximum allowed effort
            double effort = effortLinear + effortBarrier;
            return ClampAbsolute(effort, _boundEffort);
        }

        /**
         * Compute a variable proportional gain.
         *
         * @param targetPos Desired target position.
         * @param currentPos Current measured position.
         * @return equivalent linear (variable) proportional gain.
         */
        double computeControlGain(double targetPos, double currentPos)
        {
            //Not initialized case
            if (_linearStiffness <= 0.0) {
                return 0.0;
            } 
            
            //Clamp target and current position
            //to valid range for barrier bounds
            if (_barrierWidth > 0.0) {
                if (currentPos <= _barrierPosLower) {
                    currentPos = _barrierPosLower;
                }
                if (currentPos >= _barrierPosUpper) {
                    currentPos = _barrierPosUpper;
                }
                if (targetPos <= _barrierPosLower + _barrierWidth) {
                    targetPos = _barrierPosLower + _barrierWidth;
                }
                if (targetPos >= _barrierPosUpper - _barrierWidth) {
                    targetPos = _barrierPosUpper - _barrierWidth;
                }
            }
            
            //No barrier case
            double error = targetPos - currentPos;
            if (_barrierWidth <= 0.0 || std::fabs(error) < 1e-6) {
                return _linearStiffness;
            } 
            
            //Compute associated linear gain 
            //from control effort
            double effort = computeControlEffort(targetPos, currentPos);
            double gain = std::fabs(effort/error);
            
            return gain;
        }

    private:

        /**
         * Absolute maximum control effort
         */
        double _boundEffort;

        /**
         * Classic proportional to error control 
         * gain away from any position barrier
         */
        double _linearStiffness;

        /**
         * Barrier lower and upper position 
         * defining linear control range
         */
        double _barrierPosLower;
        double _barrierPosUpper;

        /**
         * Position interval where the control 
         * effort barrier is rising up from linear
         * to maximum effort
         */
        double _barrierWidth;
};

}

#endif

