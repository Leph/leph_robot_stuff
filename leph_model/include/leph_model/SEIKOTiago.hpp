#ifndef LEPH_MODEL_SEIKOTIAGO_HPP
#define LEPH_MODEL_SEIKOTIAGO_HPP

#include <leph_model/SEIKOWrapper.hpp>

namespace leph {

/**
 * SEIKOTiago
 *
 * Sequential Equilibrium Inverse Kinematic Optimization 
 * configured and specialized for Tiago robot.
 */
class SEIKOTiago : public SEIKOWrapper
{
    public:

        /**
         * Constructor inheritance
         */
        using SEIKOWrapper::SEIKOWrapper;
        
        /**
         * Define and setup contact frames.
         * Must be called only once.
         */
        void setup();

        /**
         * Reset SEIKO and internal state
         * to default.
         *
         * @return true if the reset static ID is successful.
         */
        bool reset();

        /**
         * Read access to frame names for end-effectors
         */
        const std::string& nameFrameHand() const;
        const std::string& nameFrameBase() const;

    private:

        /**
         * Hands and base contact name
         */
        std::string _nameHand;
        std::string _nameBase;
};

}

#endif

