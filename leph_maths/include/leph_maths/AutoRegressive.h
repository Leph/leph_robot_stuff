#ifndef LEPH_MATH_AUTOREGRESSIVE_H
#define LEPH_MATH_AUTOREGRESSIVE_H

#include <Eigen/Dense>

namespace leph {

/**
 * Compute the matrices unfolding the prediction
 * of the recursive uni-dimentional and affine 
 * auto regressive model:
 * State(t+1) = 
 *     ParamState*State(t-l...t) + 
 *     ParamAction*Action(t-l...t) + 
 *     ParamAffine.
 * 
 * @param sizeHorizon The number of prediction 
 * steps to be evaluated.
 * @param sizeHistory The number of past time 
 * steps used by the auto regressive model.
 * Must be at least 1.
 * @param paramConst Affine parameter 
 * of the auto regressive model.
 * @param paramsState Linear parameters over 
 * past states in time reversed order. 
 * (sizeHistory x 1).
 * @param paramAction Linear parameters over
 * past actions in time reversed order.
 * (sizeHistory x 1).
 *
 * Assigned output vector and matrices
 * of the auto regressive model computed such that:
 * dataStatePredict = 
 *     coefAction*dataActionControl + 
 *     coefInitState*dataInitState + 
 *     coefInitAction*dataInitAction + 
 *     coefConst;
 * Where:
 * - dataStatePredict: (sizeHorizon x 1) 
 *   Computed states prediction ordered 
 *   in time step order.
 * - dataActionControl: (sizeHorizon-1 x 1)
 *   Control actions ordered in time step order.
 * - dataInitState: (sizeHistory x 1)
 *   Past initial states ordered in reverse 
 *   time step order.
 * - dataInitAction: (sizeHistory x 1)
 *   Past initial actions ordered in reverse 
 *   time step order.
 * @param coefConst (output) (sizeHorizon x 1).
 * @param coefInitState (output) (sizeHorizon x sizeHistory).
 * @param coefInitAction (output) (sizeHorizon x sizeHistory).
 * @param coefAction (output) (sizeHorizon x sizeHorizon-1).
 */
inline void AutoRegressiveModelMatrices(
    size_t sizeHorizon,
    size_t sizeHistory,
    double paramConst,
    const Eigen::VectorXd& paramState,
    const Eigen::VectorXd& paramAction,
    Eigen::VectorXd& coefConst,
    Eigen::MatrixXd& coefInitState,
    Eigen::MatrixXd& coefInitAction,
    Eigen::MatrixXd& coefAction)
{
    //Recursice coefficients vector 
    //and matrices initialization
    coefConst = Eigen::VectorXd::Zero(sizeHorizon);
    coefAction = Eigen::MatrixXd::Zero(sizeHorizon, sizeHorizon-1);
    coefInitAction = Eigen::MatrixXd::Zero(sizeHorizon, sizeHistory);
    coefInitState = Eigen::MatrixXd::Zero(sizeHorizon, sizeHistory);
    
    //Coefficients initialization
    coefConst(0) = paramConst;
    coefInitAction.row(0) = paramAction;
    coefInitState.row(0) = paramState;
    
    //Recursive coefficients formulae
    //Const
    for (int t=1;t<(int)sizeHorizon;t++) {
        coefConst(t) += paramConst;
        int index = std::min(t-1, (int)sizeHistory-1);
        for (int i=0;i<=index;i++) {
            coefConst(t) += paramState(i)*coefConst(t-1-i);
        }
    }
    //InitState
    for (int t=1;t<(int)sizeHorizon;t++) {
        for (int i=0;i<(int)sizeHistory;i++) {
            int index = std::min(t-1, (int)sizeHistory-1);
            if (i >= 0 && i <= (int)sizeHistory-1-t) {
                for (int j=0;j<=index;j++) {
                    coefInitState(t,i) += paramState(j)*coefInitState(t-1-j,i);
                }
                coefInitState(t,i) += paramState(i+t);
            } else {
                for (int j=0;j<=index;j++) {
                    coefInitState(t,i) += paramState(j)*coefInitState(t-1-j,i);
                }
            }
        }
    }
    //InitAction
    for (int t=1;t<(int)sizeHorizon;t++) {
        for (int i=0;i<(int)sizeHistory;i++) {
            int index = std::min(t-1, (int)sizeHistory-1);
            if (i >= 0 && i <= (int)sizeHistory-1-t) {
                for (int j=0;j<=index;j++) {
                    coefInitAction(t,i) += paramState(j)*coefInitAction(t-1-j,i);
                }
                coefInitAction(t,i) += paramAction(i+t);
            } else {
                for (int j=0;j<=index;j++) {
                    coefInitAction(t,i) += paramState(j)*coefInitAction(t-1-j,i);
                }
            }
        }
    }
    //Action
    for (int t=1;t<(int)sizeHorizon;t++) {
        int index = std::min(t-1, (int)sizeHistory-1);
        for (int i=0;i<=t-2-index;i++) {
            for (int j=0;j<=index;j++) {
                coefAction(t,i) += paramState(j)*coefAction(t-1-j,i);
            }
        }
        for (int i=t-1-index;i<=t-2;i++) {
            for (int j=0;j<=t-2-i;j++) {
                coefAction(t,i) += paramState(j)*coefAction(t-1-j,i);
            }
            coefAction(t,i) += paramAction(t-1-i);
        }
        coefAction(t,t-1) = paramAction(0);
    }
}

}

#endif

