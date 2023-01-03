#ifndef LEPH_MATHS_POSE2D_H
#define LEPH_MATHS_POSE2D_H

#include <cmath>
#include <Eigen/Dense>
#include <leph_maths/Angle.h>

namespace leph {

/**
 * Utility tools for using planar pose vector
 * (x, y, theta) with x, y planar translation
 * in meters and theta the rotation angle in -PI:PI
 */

/**
 * Apply a planar transformation 
 * to given pose expressed in initial pose frame.
 * On initial pose frame, translation is applied 
 * first and then rotation in second place.
 *
 * @param pose Planar pose in (x, y, theta) format.
 * @param diff Transformation in (dx, dy, dtheta) 
 * format expressed in pose frame.
 * @return pose + diff Planar pose vector.
 */
inline Eigen::Vector3d Pose2dAdd(
    const Eigen::Vector3d& pose,
    const Eigen::Vector3d& diff)
{
    Eigen::Vector3d tmpPose = pose;
    double aa = pose.z();
    tmpPose.x() += diff.x()*std::cos(aa) - diff.y()*std::sin(aa);
    tmpPose.y() += diff.x()*std::sin(aa) + diff.y()*std::cos(aa);
    tmpPose.z() = AngleBound(tmpPose.z() + diff.z());

    return tmpPose;
}

/**
 * Compute from two planar poses the transformation
 * between the two expressed in starting frame.
 * Translation is assumed to be applied first 
 * and then the rotation to initial pose
 * to produce final pose.
 *
 * @param poseSrc Planar initial pose 
 * in (x, y, theta) format.
 * @param poseDst Planar final pose
 * in (x, y, theta) format.
 * @return poseDst - poseSrc
 */
inline Eigen::Vector3d Pose2dDiff(
    const Eigen::Vector3d& poseSrc,
    const Eigen::Vector3d& poseDst)
{
    //Transform in world
    double vectX = poseDst.x() - poseSrc.x();
    double vectY = poseDst.y() - poseSrc.y();
    double angle = AngleDistance(poseSrc.z(), poseDst.z()); 
    //Rotation back to source frame
    double aa = poseSrc.z();
    double vectInSrcX = vectX*cos(-aa) - vectY*sin(-aa);
    double vectInSrcY = vectX*sin(-aa) + vectY*cos(-aa);

    return Eigen::Vector3d(vectInSrcX, vectInSrcY, angle);
}

/**
 * Compute inverse planar transformation.
 * Translation is applied first and then rotation.
 *
 * @param diff Planar transformation in 
 * (dx, dy, dtheta) format.
 * @return inversed -diff planar transformation
 * (in translation first than rotation format (dx, dy, dtheta)).
 */
inline Eigen::Vector3d Pose2dInvDiff(
    const Eigen::Vector3d& diff)
{
    Eigen::Vector3d tmpDiff;
    double aa = diff.z();
    tmpDiff.x() = -diff.x()*std::cos(-aa) + diff.y()*std::sin(-aa);
    tmpDiff.y() = -diff.x()*std::sin(-aa) - diff.y()*std::cos(-aa);
    tmpDiff.z() = AngleBound(-diff.z()); 

    return tmpDiff;
}

}

#endif

