#include <rct_optimizations/dh_chain_kinematic_calibration.h>
#include <rct_optimizations/ceres_math_utilities.h>
#include <rct_optimizations/eigen_conversions.h>
#include <rct_optimizations/covariance_analysis.h>
#include <rct_optimizations/maximum_likelihood.h>
#include <rct_optimizations/local_parameterization.h>

#include <ceres/ceres.h>

namespace rct_optimizations
{

// TODO: replace; duplicates a member function
Eigen::Isometry3d createTransform(const Eigen::Vector3d& t, const Eigen::Vector3d& aa)
{
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity() * Eigen::Translation3d(t);

  double aa_norm = aa.norm();
  if (aa_norm > std::numeric_limits<double>::epsilon())
  {
    result *= Eigen::AngleAxisd(aa_norm, aa.normalized());
  }

  return result;
}

} // namespace rct_optimizations

