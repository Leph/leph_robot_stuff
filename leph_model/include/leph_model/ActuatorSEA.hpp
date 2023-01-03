#ifndef LEPH_MODEL_ACTUATORSEA_HPP
#define LEPH_MODEL_ACTUATORSEA_HPP

namespace leph {

/**
 * ActuatorSEA
 *
 * Implementation for Series Elastic 
 * Actuator model for simulation.
 */
class ActuatorSEA
{
    public:

        /**
         * Initialization with 
         * default parameters
         */
        ActuatorSEA();

        /**
         * Reset internal parameters 
         * to default values
         */
        void resetParameters();

        /**
         * @return direct read-write 
         * access to internal state
         */
        double stateMotorPos() const;
        double& stateMotorPos();
        double stateMotorVel() const;
        double& stateMotorVel();
        bool stateMotorIsStiction() const;
        bool& stateMotorIsStiction();
        double stateJointPos() const;
        double& stateJointPos();
        double stateJointVel() const;
        double& stateJointVel();

        /**
         * @return computed spring deflection 
         * position and velocity.
         * @return computed spring torque
         * and its velocity from spring deflection.
         */
        double springDeflectionPos() const;
        double springDeflectionVel() const;
        double springTorque() const;
        double springTorqueVel() const;

        /**
         * Friction torque models.
         *
         * @param vel Current motor angular velocity.
         * @return computed friction torque of 
         * the motor and the load.
         */
        double computeFrictionMotor(
            double vel) const;
        double computeFrictionJoint(
            double vel) const;

        /**
         * Motor torque controller model.
         *
         * @param torqueDesired Commanded target torque.
         * @param torqueSpring Current spring torque.
         * @param torqueSpringVel Current spring torque derivative.
         * @return computed control torque from torque controller.
         */
        double computeControl(
            double torqueDesired,
            double torqueSpring,
            double torqueSpringVel) const;

        /**
         * Compute one simulation step and
         * update internal state.
         *
         * @param dt Integration time step.
         * @param torqueDesired Desired target 
         * torque for internal controller.
         * @param torqueExternal Torque applied on 
         * external load. Gravity is part of it.
         */
        void update(
            double dt,
            double torqueDesired,
            double torqueExternal);

    private:

        /**
         * Motor internal angular 
         * position and velocity state
         */
        double _motorPos;
        double _motorVel;

        /**
         * Internal motor stiction state.
         * If true, stiction is enabled.
         */
        bool _motorIsStiction;

        /**
         * External load (joint) angular
         * position and velocity state
         */
        double _jointPos;
        double _jointVel;

        /**
         * Internal dynamics parameters
         */
        //Internal motor inertia
        double _paramMotorInertia;
        //Motor friction linear term w.r.t velocity
        double _paramMotorFrictionLinear;
        //Motor friction offset term w.r.t velocity sign
        double _paramMotorFrictionOffset;
        //Torsional spring stiffness
        double _paramSpringStiffness;
        //External load inertia
        double _paramJointInertia;
        //Load friction linear term w.r.t velocity
        double _paramJointFrictionLinear;

        /**
         * Internal controller parameters
         */
        //Discrepancy linear ratio on spring 
        //stiffness used by motor controller
        double _paramControlSpringRatio;
        //Discrepancy offset on spring torque
        //used by motor controller
        double _paramControlSpringOffset;
        //Controller proportional gain on torque error
        double _paramControlGainPos;
        //Controller derivative gain on torque error
        double _paramControlGainVel;
        //Controller absolute maximum effort
        double _paramControlMaxEffort;

        /**
         * Internal sensor parameters
         */
        /*
        //Position encoder resolution
        double _paramEncoderResolution;
        //Position encoder offset (between 0 and resolution)
        double _paramEncoderOffset;
        */
};

}

#endif

