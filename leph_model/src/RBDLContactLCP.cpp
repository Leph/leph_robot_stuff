#include <stdexcept>
#include <sstream>
#include <leph_model/RBDLContactLCP.h>
#include <mobydrakelcp/LCPSolver.hpp>

namespace leph {

namespace RBDL = RigidBodyDynamics;
namespace RBDLMath = RigidBodyDynamics::Math;

void RBDLContactLCP(
    RBDL::Model& model,
    const RBDLMath::VectorNd& Q,
    const RBDLMath::VectorNd& QDot,
    const RBDLMath::VectorNd& Tau,
    RBDL::ConstraintSet& CS,
    const Eigen::VectorXi& isBilateralConstraint)
{
    //Check sizes
    size_t sizePos = model.q_size;
    size_t sizeDOF = model.qdot_size;
    if (
        (size_t)Q.size() != sizePos ||
        (size_t)QDot.size() != sizeDOF ||
        (size_t)Tau.size() != sizeDOF ||
        (size_t)isBilateralConstraint.size() != CS.size()
    ) {
        throw std::logic_error(
            "leph::RBDLContactLCP: Invalid input size.");
    }
    if (CS.size() == 0) {
        throw std::logic_error(
            "leph::RBDLContactLCP: Empty constraint.");
    }

    //Compute H, C, and G matrices.
    RBDL::CalcConstrainedSystemVariables(
        model, Q, QDot, Tau, CS);

    //Count unilateral and bilateral constraints
    size_t sizeUnilateral = 0;
    size_t sizeBilateral = 0;
    for (size_t i=0;i<(size_t)isBilateralConstraint.size();i++) {
        if (isBilateralConstraint(i) == 0) {
            sizeUnilateral++;
        } else {
            sizeBilateral++;
        }
    }

    //Split constraint matrices in 
    //unilateral and bilateral
    RBDLMath::MatrixNd GUnilateral(sizeUnilateral, CS.G.cols());
    RBDLMath::MatrixNd GBilateral(sizeBilateral, CS.G.cols());
    RBDLMath::VectorNd gammaUnilateral(sizeUnilateral);
    RBDLMath::VectorNd gammaBilateral(sizeBilateral);
    size_t indexUnilateral = 0;
    size_t indexBilateral = 0;
    for (size_t i=0;i<(size_t)isBilateralConstraint.size();i++) {
        if (isBilateralConstraint(i) == 0) {
            GUnilateral.row(indexUnilateral) = CS.G.row(i);
            gammaUnilateral(indexUnilateral) = CS.gamma(i);
            indexUnilateral++;
        } else {
            GBilateral.row(indexBilateral) = CS.G.row(i);
            gammaBilateral(indexBilateral) = CS.gamma(i);
            indexBilateral++;
        }
    }
    
    //Check that given position and velocity 
    //complies with given constraint.
    //If not, an impulse need to be apply.
    RBDLMath::VectorNd ZetaUnilateral = GUnilateral * QDot;
    RBDLMath::VectorNd ZetaBilateral = GBilateral * QDot;
    if (
        ZetaUnilateral.lpNorm<Eigen::Infinity>() > 1e-3 ||
        ZetaBilateral.lpNorm<Eigen::Infinity>() > 1e-3
    ) {
        std::ostringstream ss;
        ss << " ZetaUnilateral: " << ZetaUnilateral.transpose();
        ss << " sizeUnilateral: " << sizeUnilateral;
        ss << " ZetaBilateral: " << ZetaBilateral.transpose();
        ss << " sizeBilateral: " << sizeBilateral;
        throw std::logic_error(
            "leph::RBDLContactLCP: "
            "Velocity not constrainted. Impulse needed: " 
            + ss.str());
    }

    //Featherstone's book chapter 11.3 and 11.5.
    //The LCP to solve is:
    //zetaDot = M*lambda + d
    //with: zetaDot(i) >=0, lambda(i) >= 0, zetaDot(i)*lambda(i) = 0.
    //lambda: contact force.
    //zetaDot: contact distance acceleration.
    //zeta: contact distance velocity = G*QDot.
    //M = G*Hinv*Gt.
    //d = G*Hinv*(Tau-C) + GDot*QDot.
    //GDot*QDot = -CS.gamma = -k (in Featherstone)
    //Note that in Featherstone's boot, 
    //notation is G = T.transpose().

    //Here, the unilateral constraints are solved 
    //while also enforcing the lateral no slip 
    //conditions (bilateral constraints).
    //(See Quentin Rouxel PhD thesis or Drumwright and Shell [2009]).
    //
    //G_t: Matrix of tangent bilateral constraints.
    //G_n: Matrix of normal unilateral constraints.
    //lambda_t: Vector of tangent bilateral forces.
    //lambda_n: Vector of normal unilateral forces.
    //
    //Equations to solves for lambda_t and lambda_n:
    //H.QDDot + C - K_t'.lambda_t - K_n'.lambda_n = Tau
    //G_n.QDDot - gamma_n = zetaDot_n
    //G_t.QDDot - gamma_t = 0
    //with complementary constraints.
    //
    //LCP problem to solve is zetaDot = M.lambda + d with:
    //TT = |G_n 0|
    //HH = |H   -G_t'|
    //     |K_t 0    |
    //CC = |Tau - C|
    //     |gamma_t|
    //M = TT.HH^{-1}.TT'
    //d = TT.HH^{-1}.CC - gamma_n
    
    //Build temporary problem matrices
    Eigen::MatrixXd HH = Eigen::MatrixXd::Zero(
        sizeDOF+sizeBilateral, sizeDOF+sizeBilateral);
    Eigen::MatrixXd TT = Eigen::MatrixXd::Zero(
        sizeUnilateral, sizeDOF+sizeBilateral);
    Eigen::VectorXd CC = Eigen::VectorXd::Zero(
        sizeDOF+sizeBilateral);
    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(
        sizeUnilateral, sizeUnilateral);
    Eigen::VectorXd D = Eigen::VectorXd::Zero(
        sizeUnilateral);
    Eigen::VectorXd lambda = Eigen::VectorXd::Zero(
        sizeUnilateral);
    HH.block(0, 0, sizeDOF, sizeDOF) = CS.H;
    HH.block(sizeDOF, 0, sizeBilateral, sizeDOF) = GBilateral;
    HH.block(0, sizeDOF, sizeDOF, sizeBilateral) = -GBilateral.transpose();
    TT.block(0, 0, sizeUnilateral, sizeDOF) = GUnilateral;
    CC.segment(0, sizeDOF) = Tau - CS.C;
    CC.segment(sizeDOF, sizeBilateral) = gammaBilateral;
    //Compute HH matrix inverse
    auto HHQR = HH.colPivHouseholderQr();
    if (!HHQR.isInvertible()) {
        throw std::logic_error(
            "leph::RBDLContactLCP: Non invertible HH matrix.");
    }
    Eigen::MatrixXd HHinv = HHQR.inverse();
    //Build LCP problem matrices
    M = TT*HHinv*TT.transpose();
    D = TT*HHinv*CC - gammaUnilateral;

    //Compute LCP solution
    Drake::MobyLCPSolver solver;
    bool isSuccess = solver.
        SolveLcpLemkeRegularized(M, D, &lambda);
    if (!isSuccess) {
        throw std::logic_error(
            "leph::RBDLContactLCP: LCP not successful.");
    }

    //Assign computed lambda to ConstraintSet
    indexUnilateral = 0;
    indexBilateral = 0;
    for (size_t i=0;i<(size_t)isBilateralConstraint.size();i++) {
        if (isBilateralConstraint(i) == 0) {
            CS.force(i) = lambda(indexUnilateral);
            indexUnilateral++;
        } else {
            CS.force(i) = 1.0;
            indexBilateral++;
        }
    }
}

}

