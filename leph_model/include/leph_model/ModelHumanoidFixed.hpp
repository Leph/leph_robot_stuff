#ifndef LEPH_MODEL_MODELHUMANOIDFIXED_HPP
#define LEPH_MODEL_MODELHUMANOIDFIXED_HPP

#include <string>
#include <Eigen/Dense>
#include <leph_model/Model.hpp>

namespace leph {

/**
 * ModelHumanoidFixed
 *
 * Represent a humanoid model where
 * at least one of its foot is always
 * assumed to be fixed with respect 
 * the world frame (no jumping/fly phase).
 * The class is a wrapper for two models
 * where their tree roots are located on
 * each foot (fixed) frame.
 * The two models are alternatively swapped
 * each time the support foot change.
 */
class ModelHumanoidFixed
{
    public:

        /**
         * Initialization with URDF file
         * and foot frame name.
         *
         * @param filename Path to URDF XML file.
         * @param nameFootLeft RBDL frame name
         * of left foot which will be fixed during
         * left support phase.
         * @param nameFootRight RBDL frame name
         * of right foot which will be fixed during
         * left support phase.
         */
        ModelHumanoidFixed(
            const std::string& filename,
            const std::string& nameFootLeft,
            const std::string& nameFootRight);

        /**
         * Current support state.
         *
         * @return true is the left foot
         * is currently assumed to be fixed and 
         * left model is used. Else, return false
         * for right support phase.
         */
        bool isSupportLeft() const;

        /**
         * Update and swap the support 
         * state if given support state is 
         * different from current one.
         *
         * @param isLeft If true, the left support 
         * state is asked. Else, the right support state.
         */
        void setSupport(bool isLeft);

        /**
         * Underlying model access.
         *
         * @return access to currently used model
         * (based upon support state).
         */
        const Model& get() const;
        Model& get();

        /**
         * Direct access to left and right model
         */
        const Model& getModelLeft() const;
        Model& getModelLeft();
        const Model& getModelRight() const;
        Model& getModelRight();

    private:
        
        /**
         * RBDL frame name for left 
         * and right feet
         */
        std::string _nameFootLeft;
        std::string _nameFootRight;

        /**
         * Left and right models
         * whose tree's root is located
         * on left and right foot frame
         */
        Model _modelLeft;
        Model _modelRight;

        /**
         * If true, the left model is used
         * and the left foot is assumed to be 
         * fixed in world frame.
         * Else, the right model is used and
         * the right foot is assumed to be fixed.
         */
        bool _isSupportLeft;
};

}

#endif

