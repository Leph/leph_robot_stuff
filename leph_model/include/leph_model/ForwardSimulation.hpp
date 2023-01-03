#ifndef LEPH_MODEL_FORWARDSIMULATION_HPP
#define LEPH_MODEL_FORWARDSIMULATION_HPP

#include <vector>
#include <Eigen/Dense>
#include <leph_model/Model.hpp>

namespace leph {

/**
 * ForwardSimulation
 *
 * Model simulation using forward impulsive
 * dynamics and classic semi-implicit 
 * Euler integration.
 */
class ForwardSimulation
{
    public:

        /**
         * Initialization with floating 
         * base Model instance.
         *
         * @param model Reference to the model
         * whose forward dynamics will be simulated.
         */
        ForwardSimulation(Model& model);

        /**
         * Access to current state.
         *
         * @return Read only or read-write 
         * reference to internal vectors
         */
        const Eigen::VectorXd& positions() const;
        Eigen::VectorXd& positions();
        const Eigen::VectorXd& velocities() const;
        Eigen::VectorXd& velocities();
        const Eigen::VectorXd& torques() const;
        Eigen::VectorXd& torques();

        /**
         * Update the position/velocity state
         * from current control torques over
         * the given dt time step.
         * Underlying model position and velocity is updated.
         *
         * @param dt Integration time step in seconds.
         * @param constraints Reference to RBDL contact 
         * constraint set to be enforced. The set need
         * to be well initialized and bound to internal model.
         */
        void update(
            double dt, 
            RBDL::ConstraintSet& constraints);

    private:

        /**
         * Model pointer instance
         */
        Model* _model;

        /**
         * Degrees of freedom position
         * and velocity state.
         * Floating base quaternion is 
         * included in position vector
         * (sizeVectPos() and sizeVectVel()).
         */
        Eigen::VectorXd _positions;
        Eigen::VectorXd _velocities;

        /**
         * Last used user input torque for each
         * degrees of freedom (sizeVectVel()).
         */
        Eigen::VectorXd _torques;
};

}

#endif

