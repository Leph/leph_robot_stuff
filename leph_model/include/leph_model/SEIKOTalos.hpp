#ifndef LEPH_MODEL_SEIKOTALOS_HPP
#define LEPH_MODEL_SEIKOTALOS_HPP

#include <leph_model/SEIKOWrapper.hpp>

namespace leph {

/**
 * SEIKOTalos
 *
 * Sequential Equilibrium Inverse Kinematic Optimization 
 * configured and specialized for Talos robot.
 */
class SEIKOTalos : public SEIKOWrapper
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
        const std::string& nameFrameFootLeft() const;
        const std::string& nameFrameFootRight() const;
        const std::string& nameFrameHandLeft() const;
        const std::string& nameFrameHandRight() const;

    private:

        /**
         * Feet and hands contact names
         */
        std::string _nameFootLeft;
        std::string _nameFootRight;
        std::string _nameHandLeft;
        std::string _nameHandRight;
};

}

#endif

