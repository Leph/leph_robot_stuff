#ifndef LEPH_MATHS_FILTERELASTICITYPOS_HPP
#define LEPH_MATHS_FILTERELASTICITYPOS_HPP

#include <cmath>
#include <stdexcept>
#include <leph_maths/Clamp.h>

namespace leph {

/**
 * FilterElasticityPos
 *
 * Filter to correct joint position from
 * joint elasticity due to currently 
 * applied torque.
 */
class FilterElasticityPos
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
        FilterElasticityPos(
            double elasticity = 0.0, 
            double maxDeflection = M_PI) :
            _maxDeflection(maxDeflection),
            _elasticity(elasticity),
            _filteredPos(0.0)
        {
            if (maxDeflection < 0.0) {
                throw std::logic_error(
                    "leph::FilterElasticityPos: "
                    "Invalid maximum deflection.");
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
         * Computed the corrected position.
         *
         * @param position Current joint 
         * position to be corrected.
         * @oaram torque Current torque 
         * being applied on the joint.
         */
        void update(double position, double torque)
        {
            double deflection = ClampAbsolute(
                torque*_elasticity, _maxDeflection);
            _filteredPos = position - deflection;
        }

        /**
         * @return read access to corrected position
         */
        double value() const
        {
            return _filteredPos;
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
         * Computed position after applied 
         * elasticity compensation applied
         */
        double _filteredPos;
};

}

#endif

