#ifndef LEPH_MATHS_FILTERELASTICITYVEL_HPP
#define LEPH_MATHS_FILTERELASTICITYVEL_HPP

#include <cmath>
#include <stdexcept>
#include <leph_maths/Clamp.h>

namespace leph {

/**
 * FilterElasticityVel
 *
 * Filter to correct joint velocity from
 * joint elasticity due to currently 
 * applied torque.
 */
class FilterElasticityVel
{
    public:

        /**
         * Initialization.
         *
         * @param elasticity The elasticity coefficient 
         * as the inverse of the spring stiffness coefficient.
         * @param maxDeflection Maximum angular
         * deflection in radian.
         */
        FilterElasticityVel(
            double elasticity = 0.0, 
            double maxDeflection = M_PI) :
            _maxDeflection(maxDeflection),
            _elasticity(elasticity),
            _filteredVel(0.0),
            _timePrevious(-1.0),
            _deflectionPrevious(0.0)
        {
            if (maxDeflection < 0.0) {
                throw std::logic_error(
                    "leph::FilterElasticity: Invalid maximum deflection.");
            }
        }

        /**
         * Set the elasticity coefficient
         */
        void setElasticity(double elasticity)
        {
            if (elasticity > 0.0) {
                _elasticity = elasticity;
            } else {
                _elasticity = 0.0;
            }
        }

        /**
         * Computed the corrected velocity.
         *
         * @param velocity Current joint 
         * velocity to be corrected.
         * @oaram torque Current torque being 
         * applied on the joint.
         */
        void update(double time, double velocity, double torque)
        {
            //Compute time step
            double dt = 0.0;
            if (_timePrevious >= 0.0) {
                dt = time - _timePrevious;
            }

            //Compute deflection differentiation
            double deflectionNow = ClampAbsolute(
                torque*_elasticity, _maxDeflection);
            double deflectionDiff = 0.0;
            if (dt > 1e-6) {
                deflectionDiff = (deflectionNow - _deflectionPrevious)/dt;
            }

            //Update internal state
            _filteredVel = velocity - deflectionDiff;
            _timePrevious = time;
            _deflectionPrevious = deflectionNow;
        }

        /**
         * @return read access to corrected velocity
         */
        double value() const
        {
            return _filteredVel;
        }

    private:

        /**
         * Maximum angular applied 
         * deflection in radian
         */
        double _maxDeflection;

        /**
         * The elasticity coefficient as 
         * the inverse of the spring stiffness
         * coefficient
         */
        double _elasticity;

        /**
         * Computed velocity after applied 
         * elasticity compensation applied
         */
        double _filteredVel;

        /**
         * Time in second at last 
         * computed _deflectionPrevious
         */
        double _timePrevious;

        /**
         * Last computed joint deflection used to 
         * estimate the deflection time differentiation
         */
        double _deflectionPrevious;
};

}

#endif

