#include <cmath>
#include <leph_model/LegIKTalos.hpp>

namespace LegIK {

#define IKDEBUG(command)  
#define IKMSG(command) command

class Vector3D : public std::vector<double> {
    public:
        Vector3D();
        Vector3D(double x1, double x2, double x3);
        Vector3D(const Vector3D & other);
        double length();
        void normalize();

        friend Vector3D operator*(double x, const Vector3D & v);
        friend Vector3D operator+(const Vector3D & v1, const Vector3D & v2);
        friend Vector3D operator-(const Vector3D & v1, const Vector3D & v2);
        friend double scalar_prod(const Vector3D & v1, const Vector3D & v2);
        friend Vector3D vect_prod(const Vector3D & v1, const Vector3D & v2);
};

/*****************************************************************************/

class Frame3D : public std::vector<Vector3D> {
    public:
        /* canonical frame */
        Frame3D();
        Frame3D(const Frame3D & other);
        /* Ordre habituel des angles d'euler : precession, nutation et rotation propre */
        static Frame3D from_euler(double euler_psi, double euler_theta, double euler_phi);
        static Frame3D from_vectors(Vector3D e1, Vector3D e2, Vector3D e3);
};

/*****************************************************************************/

class Position {
    public:
        double theta[6];
        Position();
        Position(double theta0, double theta1, double theta2, 
            double theta3, double theta4, double theta5);
};

/*****************************************************************************/

/**
 * If inverseKnee is true, the knee convention sign
 * in inverted and the knee is bending in the other
 * side.
 * If forceForward is false, we do not assume that 
 * the leg is pointing forward. Usefull for 6DOF
 * arm when target point is imposing the orientation
 * of the plane P.
 * If not null, boundIKDistance is a signed "distance" 
 * from kinematics bound. If positive, the IK is valid.
 * If negative, the IK is out of bounds.
 * (Computed from condition threasholds).
 */
class IK {
    private:

        double L[3];

    public:

        IK(double L0, double L1, double L2);

        bool compute(
            Vector3D C, 
            Frame3D orientation, 
            Position & result, 
            double* boundIKDistance = nullptr,
            bool inverseKnee = false, 
            bool forceForward = true);
};

/*****************************************************************************/

#define DEG2RAD(x) ((x) * M_PI / 180.0)
#define ik_global_epsilon 0.0000001

namespace IKTools {
    inline double val_abs(double x) {
        return (x >= 0) ? x : -x;
    }

    inline bool is_zero(double x) {
        return val_abs(x) < ik_global_epsilon;
    }

    inline void bound(double min, double max, double & x) {
        if (x < min) x = min;
        if (x > max) x = max;
    }

    inline int sign(double x) { 
        return (x >= 0) ? 1 : -1; 
    }
}

using namespace IKTools;

/*****************************************************************************/

Vector3D::Vector3D() {
    for (int i=0; i<3; i++) this->push_back(0.0);
}

Vector3D::Vector3D(double x1, double x2, double x3) {
    push_back(x1);
    push_back(x2);
    push_back(x3);
}

Vector3D::Vector3D(const Vector3D & other) : 
    vector<double>(other)
{}

double Vector3D::length() {
    return sqrt((*this)[0]*(*this)[0] + (*this)[1]*(*this)[1] + (*this)[2]*(*this)[2]);
}

void Vector3D::normalize() {
    double l = length();
    if (is_zero(l)) return;
    *this = (1.0 / l) * *this;
}

Vector3D operator+ (const Vector3D & v1, const Vector3D & v2) {
    Vector3D result;
    for(int i=0; i<3; i++) 
        result[i] = v1[i] + v2[i];
    return result;
}

Vector3D operator- (const Vector3D & v1, const Vector3D & v2) {
    Vector3D result;
    for(int i=0; i<3; i++) 
        result[i] = v1[i] - v2[i];
    return result;
}

Vector3D operator * (double x, const Vector3D & v) {
    Vector3D result;
    for(int i=0; i<3; i++) 
        result[i] = x * v[i];
    return result;
}

double scalar_prod(const Vector3D & v1, const Vector3D & v2) {
    return v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2];
}

Vector3D vect_prod(const Vector3D & v1, const Vector3D & v2) {
    return Vector3D(v1[1]*v2[2] - v1[2]*v2[1],
        v1[2]*v2[0] - v1[0]*v2[2],
        v1[0]*v2[1] - v1[1]*v2[0]);
}

/*****************************************************************************/

Frame3D::Frame3D() {
  push_back(Vector3D(1,0,0));
  push_back(Vector3D(0,1,0));
  push_back(Vector3D(0,0,1));
}

Frame3D::Frame3D(const Frame3D & other) : 
  vector<Vector3D>(other) 
{}

Frame3D Frame3D::from_euler(double psi, double theta, double phi) {
  Frame3D res;
  res[0] = Vector3D(cos(phi)*cos(psi) - sin(phi)*cos(theta)*sin(psi),
		    cos(phi)*sin(psi) + sin(phi)*cos(theta)*cos(psi),
		    sin(phi)*sin(theta));
  res[1] = Vector3D(-sin(phi)*cos(psi) - cos(phi)*cos(theta)*sin(psi),
		    -sin(phi)*sin(psi) + cos(phi)*cos(theta)*cos(psi),
		    cos(phi)*sin(theta));
  res[2] = Vector3D(sin(theta)*sin(psi),
		    -sin(theta)*cos(psi),
		    cos(theta));
  return res;
}

Frame3D Frame3D::from_vectors(Vector3D e1, Vector3D e2, Vector3D e3) {
  Frame3D res;
  res[0] = e1;
  res[1] = e2;
  res[2] = e3;
  return res;
}

/*****************************************************************************/

Position::Position() {
  for (int i=0; i<6; i++) theta[i] = 0.0;
}

Position::Position(double theta0, double theta1, double theta2, 
		   double theta3, double theta4, double theta5) {
  theta[0] = theta0;
  theta[1] = theta1;
  theta[2] = theta2;
  theta[3] = theta3;
  theta[4] = theta4;
  theta[5] = theta5;
}

/*****************************************************************************/

IK::IK(double L0, double L1, double L2) {
    L[0] = L0; L[1] = L1; L[2] = L2;
}

bool IK::compute(
    Vector3D C, 
    Frame3D orientation, 
    Position & result,
    double* boundIKDistance,
    bool inverseKnee, 
    bool forceForward) 
{
    if (boundIKDistance != nullptr) {
        *boundIKDistance = std::fabs(L[0]);
        *boundIKDistance = 
            std::min(*boundIKDistance, std::fabs(L[1]));
    }
    if (is_zero(L[0]) || is_zero(L[1])) {
        return false;
    }

    Vector3D e1(1,0,0), e2(0,1,0), e3(0,0,1);

    /* step 1 : calcul de B */
    Vector3D B = C + L[2] * orientation[2];  
    double B_len = B.length();

    if (boundIKDistance != nullptr) {
        *boundIKDistance = 
            std::min(*boundIKDistance, std::fabs(B_len));
        *boundIKDistance = 
            std::min(*boundIKDistance, -B[2]);
    }
    if (B[2] >= 0 || is_zero(B_len)) {
        return false;
    }

    /* step 2 : calcul de phi */
    Vector3D phi;
    if (!is_zero(orientation[0][2])) {
        double a_pp = 1.0;
        double b_pp = -B[2] / orientation[0][2];
        phi = (a_pp * B) + (b_pp * orientation[0]);
        phi.normalize();
    } 
    else {
        phi = orientation[0]; 
        phi.normalize(); 
    }

    /* phi est orienté vers l'avant ou sur la gauche */
    double phi_e1 = scalar_prod(phi, e1);
    if (
        forceForward && 
        !(phi_e1 > 0 || (is_zero(phi_e1) && scalar_prod(phi, e2) >= 0))
    ) {
        phi = -1.0 * phi;
    }

    /* step 3 : calcul de \theta_0 */
    result.theta[0] = atan2(phi[1], phi[0]);

    /* step 4 : calcul de G */
    Vector3D G = scalar_prod(B,phi) * phi;

    /* step 5 : calcul de \theta_1 */
    Vector3D zeta = -1.0 * vect_prod(phi, e3);
    result.theta[1] = atan2(scalar_prod(B-G, zeta), -B[2]);

    /* step 6 : calcul de \theta_3 */
    double q = (L[0]*L[0] + L[1]*L[1] - B_len*B_len) / (2 * L[0] * L[1]);
    if (boundIKDistance != nullptr) {
        *boundIKDistance = 
            std::min(*boundIKDistance, 1.0-std::fabs(q));
    }
    if (q < (-1.0 - ik_global_epsilon) || q > (1.0 + ik_global_epsilon)) {
        return false;
    }

    bound(-1.0, 1.0, q);
    if (inverseKnee) {
        result.theta[3] = acos(q) - M_PI;
    } else {
        result.theta[3] = M_PI - acos(q);
    }

    /* step 7 : calcul de \omega */
    Vector3D omega(-sin(result.theta[0])*sin(result.theta[1]),
        cos(result.theta[0])*sin(result.theta[1]),
        -cos(result.theta[1]));

    /* step 8 : calcul de alpha */
    q = scalar_prod(B,omega) / B_len;
    bound(-1.0, 1.0, q); /* on a toujours |q| <= 1 */
    double alpha = sign(scalar_prod(vect_prod(B, omega), zeta)) * acos(q); 

    /* step 9 : calcul de l'angle (A \Omega B) */
    q = (L[0]*L[0] + B_len*B_len - L[1]*L[1]) / (2 * L[0] * B_len);
    if (boundIKDistance != nullptr) {
        *boundIKDistance = 
            std::min(*boundIKDistance, 1.0-std::fabs(q));
    }
    if (q < (-1.0 - ik_global_epsilon) || q > (1.0 + ik_global_epsilon)) {
        return false;
    }

    bound(-1.0, 1.0, q);
    double A_omega_B = acos(q);

    /* step 10 : calcul de theta_2 */
    if (inverseKnee) {
        result.theta[2] = alpha - A_omega_B;
    } else {
        result.theta[2] = alpha + A_omega_B;
    }

    /* step 11 : calcul de theta_4 */
    q = scalar_prod(phi, orientation[0]);
    bound(-1.0, 1.0, q);
    double beta = -sign(scalar_prod(vect_prod(phi, orientation[0]), zeta)) * acos(q);
    result.theta[4] = beta + result.theta[3] - result.theta[2];

    /* step 12 : calcul de theta_5 */
    Vector3D tau = vect_prod(phi, omega);
    q = scalar_prod(tau, orientation[1]);
    bound(-1.0, 1.0, q);
    result.theta[5] = sign(scalar_prod(vect_prod(tau, orientation[1]), orientation[0])) * acos(q);

    return true;
}

}

namespace leph {

LegIKTalos::LegIKTalos() :
    _model(nullptr),
    _indexLegLeft(),
    _indexLegRight(),
    _transBaseToHipLeft(Eigen::Vector3d::Zero()),
    _transBaseToHipRight(Eigen::Vector3d::Zero()),
    _legHipToKnee(0.0),
    _legKneeToAnkle(0.0),
    _legAnkleToGround(0.0),
    _indexFrameBase(-1),
    _indexFrameFootLeft(-1),
    _indexFrameFootRight(-1)
{
}

LegIKTalos::LegIKTalos(Model& model) :
    _model(&model),
    _indexLegLeft(),
    _indexLegRight(),
    _transBaseToHipLeft(Eigen::Vector3d::Zero()),
    _transBaseToHipRight(Eigen::Vector3d::Zero()),
    _legHipToKnee(0.0),
    _legKneeToAnkle(0.0),
    _legAnkleToGround(0.0),
    _indexFrameBase(-1),
    _indexFrameFootLeft(-1),
    _indexFrameFootRight(-1)
{
    //Retrieve joint indexes
    _indexLegLeft.push_back(_model->getIndexDOF("leg_left_1_joint"));
    _indexLegLeft.push_back(_model->getIndexDOF("leg_left_2_joint"));
    _indexLegLeft.push_back(_model->getIndexDOF("leg_left_3_joint"));
    _indexLegLeft.push_back(_model->getIndexDOF("leg_left_4_joint"));
    _indexLegLeft.push_back(_model->getIndexDOF("leg_left_5_joint"));
    _indexLegLeft.push_back(_model->getIndexDOF("leg_left_6_joint"));
    _indexLegRight.push_back(_model->getIndexDOF("leg_right_1_joint"));
    _indexLegRight.push_back(_model->getIndexDOF("leg_right_2_joint"));
    _indexLegRight.push_back(_model->getIndexDOF("leg_right_3_joint"));
    _indexLegRight.push_back(_model->getIndexDOF("leg_right_4_joint"));
    _indexLegRight.push_back(_model->getIndexDOF("leg_right_5_joint"));
    _indexLegRight.push_back(_model->getIndexDOF("leg_right_6_joint"));

    //Retrieve leg geometry
    Eigen::VectorXd statePos = _model->getDOFPosVect();
    _model->setJointPosVect(Eigen::VectorXd::Zero(_model->sizeJoint()));
    _model->updateState();
    _transBaseToHipLeft = _model->position("leg_left_1_link", "base_link");
    _transBaseToHipRight = _model->position("leg_right_1_link", "base_link");
    _legHipToKnee = _model->position("leg_left_3_link", "leg_left_4_link").z();
    _legKneeToAnkle = _model->position("leg_left_4_link", "leg_left_5_link").z();
    _legAnkleToGround = _model->position("leg_left_6_link", "left_sole_link").z();
    _model->setDOFPosVect(statePos);
    _model->updateState();

    //Retrieve frame indexes
    _indexFrameBase = _model->getIndexFrame("base_link");
    _indexFrameFootLeft = _model->getIndexFrame("left_sole_link");
    _indexFrameFootRight = _model->getIndexFrame("right_sole_link");
}

bool LegIKTalos::legIKLeft(
    const Eigen::Vector3d& pos,
    const Eigen::Matrix3d& mat,
    double* boundIKDistance)
{
    //LegIK initialization
    LegIK::IK ik(_legHipToKnee, _legKneeToAnkle, _legAnkleToGround);
    //Convert foot position from given 
    //target to LegIK base
    LegIK::Vector3D legIKTarget;
    legIKTarget[0] = pos.x() - _transBaseToHipLeft.x();
    legIKTarget[1] = pos.y() - _transBaseToHipLeft.y();
    legIKTarget[2] = pos.z() - _transBaseToHipLeft.z();
    //Convert orientation from given frame
    //to LegIK base
    LegIK::Frame3D legIKMatrix;
    Eigen::Matrix3d tmpMat = mat.transpose();
    legIKMatrix[0][0] = tmpMat(0, 0);
    legIKMatrix[0][1] = tmpMat(0, 1);
    legIKMatrix[0][2] = tmpMat(0, 2);
    legIKMatrix[1][0] = tmpMat(1, 0);
    legIKMatrix[1][1] = tmpMat(1, 1);
    legIKMatrix[1][2] = tmpMat(1, 2);
    legIKMatrix[2][0] = tmpMat(2, 0);
    legIKMatrix[2][1] = tmpMat(2, 1);
    legIKMatrix[2][2] = tmpMat(2, 2);
    
    //Run inverse kinematics
    LegIK::Position result;
    bool isSuccess = ik.compute(
        legIKTarget, legIKMatrix, result, boundIKDistance);

    //Update degrees of freedom on success
    if (isSuccess) {
        _model->setDOFPos(_indexLegLeft[0], result.theta[0]);
        _model->setDOFPos(_indexLegLeft[1], result.theta[1]);
        _model->setDOFPos(_indexLegLeft[2], -result.theta[2]);
        _model->setDOFPos(_indexLegLeft[3], result.theta[3]);
        _model->setDOFPos(_indexLegLeft[4], -result.theta[4]);
        _model->setDOFPos(_indexLegLeft[5], result.theta[5]);
    } 

    return isSuccess;
}
bool LegIKTalos::legIKRight(
    const Eigen::Vector3d& pos,
    const Eigen::Matrix3d& mat,
    double* boundIKDistance)
{
    //LegIK initialization
    LegIK::IK ik(_legHipToKnee, _legKneeToAnkle, _legAnkleToGround);
    //Convert foot position from given 
    //target to LegIK base
    LegIK::Vector3D legIKTarget;
    legIKTarget[0] = pos.x() - _transBaseToHipRight.x();
    legIKTarget[1] = pos.y() - _transBaseToHipRight.y();
    legIKTarget[2] = pos.z() - _transBaseToHipRight.z();
    //Convert orientation from given frame
    //to LegIK base
    LegIK::Frame3D legIKMatrix;
    Eigen::Matrix3d tmpMat = mat.transpose();
    legIKMatrix[0][0] = tmpMat(0, 0);
    legIKMatrix[0][1] = tmpMat(0, 1);
    legIKMatrix[0][2] = tmpMat(0, 2);
    legIKMatrix[1][0] = tmpMat(1, 0);
    legIKMatrix[1][1] = tmpMat(1, 1);
    legIKMatrix[1][2] = tmpMat(1, 2);
    legIKMatrix[2][0] = tmpMat(2, 0);
    legIKMatrix[2][1] = tmpMat(2, 1);
    legIKMatrix[2][2] = tmpMat(2, 2);
    
    //Run inverse kinematics
    LegIK::Position result;
    bool isSuccess = ik.compute(
        legIKTarget, legIKMatrix, result, boundIKDistance);

    //Update degrees of freedom on success
    if (isSuccess) {
        _model->setDOFPos(_indexLegRight[0], result.theta[0]);
        _model->setDOFPos(_indexLegRight[1], result.theta[1]);
        _model->setDOFPos(_indexLegRight[2], -result.theta[2]);
        _model->setDOFPos(_indexLegRight[3], result.theta[3]);
        _model->setDOFPos(_indexLegRight[4], -result.theta[4]);
        _model->setDOFPos(_indexLegRight[5], result.theta[5]);
    } 

    return isSuccess;
}

bool LegIKTalos::baseFootIK(
    bool isLeftFoot,
    const Eigen::Vector3d& basePos, 
    const Eigen::Matrix3d& baseMat,
    const Eigen::Vector3d& footPos,
    const Eigen::Matrix3d& footMat,
    double* boundIKDistance)
{
    //Frame transformations
    Eigen::Matrix3d matBaseToSupport = baseMat.transpose();
    Eigen::Vector3d posBaseToSupport = matBaseToSupport*(-basePos);
    Eigen::Matrix3d matBaseToFlying = matBaseToSupport*footMat;
    Eigen::Vector3d posBaseToFlying = posBaseToSupport + matBaseToSupport*footPos;

    //Compute left and right leg inverse kinematics
    //for both foot.
    bool isSuccessLeft = true;
    bool isSuccessRight = true;
    //Compute distance from IK bound
    double boundLeft = 0.0;
    double boundRight = 0.0;
    if (isLeftFoot) {
        isSuccessLeft = legIKLeft(
            posBaseToSupport, matBaseToSupport,
            &boundLeft);
        isSuccessRight = legIKRight(
            posBaseToFlying, matBaseToFlying,
            &boundRight);
    } else {
        isSuccessRight = legIKRight(
            posBaseToSupport, matBaseToSupport,
            &boundRight);
        isSuccessLeft = legIKLeft(
            posBaseToFlying, matBaseToFlying,
            &boundLeft);
    }
    //Assign IK bound
    if (boundIKDistance != nullptr) {
        *boundIKDistance = std::min(boundLeft, boundRight);
    }

    return isSuccessLeft && isSuccessRight;
}

bool LegIKTalos::CoMFootIK(
    bool isLeftFoot,
    const Eigen::Vector3d& comPos, 
    const Eigen::Matrix3d& baseMat,
    const Eigen::Vector3d& footPos,
    const Eigen::Matrix3d& footMat,
    double* boundIKDistance)
{
    bool isSuccess = false;
    Eigen::Vector3d offsetCoM = Eigen::Vector3d::Zero();
    for (size_t i=0;i<20;i++) {
        isSuccess = baseFootIK(
            isLeftFoot, 
            comPos + offsetCoM, 
            baseMat, 
            footPos, 
            footMat, 
            boundIKDistance);
        if (!isSuccess) {
            break;
        }
        _model->updateState();
        if (isLeftFoot) {
            offsetCoM = 
                offsetCoM + 
                0.5*(comPos - _model->centerOfMass(_indexFrameFootLeft));
        } else {
            offsetCoM = 
                offsetCoM + 
                0.5*(comPos - _model->centerOfMass(_indexFrameFootRight));
        }
    }
    return isSuccess;
}

}

