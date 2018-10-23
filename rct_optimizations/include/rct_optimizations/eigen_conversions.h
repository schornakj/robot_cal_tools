#ifndef RCT_EIGEN_CONVERSIONS_H
#define RCT_EIGEN_CONVERSIONS_H

#include "rct_optimizations/types.h"
#include <Eigen/Dense>

namespace rct_optimizations
{

Pose6d poseEigenToCal(const Eigen::Affine3d& pose);

Vec3d vecEigenToCal(const Eigen::Vector3d& vec);

Eigen::Affine3d poseCalToEigen(const Pose6d& pose);

Eigen::Vector3d vecCalToEigen(const Vec3d& vec);
}

#endif // RCT_EIGEN_CONVERSIONS_H
