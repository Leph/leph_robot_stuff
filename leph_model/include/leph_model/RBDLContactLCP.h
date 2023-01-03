#ifndef LEPH_MODEL_RBDLCONTACTLCP_H
#define LEPH_MODEL_RBDLCONTACTLCP_H

#include <rbdl/rbdl.h>
#include <Eigen/Dense>

namespace leph {

/**
 * Solve the Linear Complimentary 
 * Problem used to compute which contact
 * constraints have to be activated or 
 * is going to release.
 * Use Moby-Drake LCP solver.
 * RBDL model, position, velocity and applied
 * torque are given in addition to the constraint
 * set for only considered contact.
 *
 * The constraint set hold unilateral (inequality)
 * for non penetration and bilateral (equality) 
 * constraints representing no-slip infinite friction.
 * All bilateral constraints must have non zero value
 * in associated vector isBilateralConstraint.
 * (Associated force lambda will not be constrained by
 * complimentary. Always non zero).
 *
 * See Featherstone's book Chapter 11.3, for
 * details and notations.
 *
 * The computed contact force lambda (with zero
 * and non zero elements) is assigned to the "force"
 * attribute of constraint set structure.
 * WARNING: Only vertical unilateral forces 
 * hold actual values.
 */
void RBDLContactLCP(
    RigidBodyDynamics::Model& model,
    const RigidBodyDynamics::Math::VectorNd& Q,
    const RigidBodyDynamics::Math::VectorNd& QDot,
    const RigidBodyDynamics::Math::VectorNd& Tau,
    RigidBodyDynamics::ConstraintSet& CS,
    const Eigen::VectorXi& isBilateralConstraint);

}

#endif

