#include "rct_optimizations/extrinsic_3d_camera_cartesian_gantry.h".h"

#include "rct_optimizations/ceres_math_utilities.h"
#include "rct_optimizations/eigen_conversions.h"
#include "rct_optimizations/types.h"

#include <ceres/ceres.h>
#include <iostream>

using namespace rct_optimizations;

namespace
{

class ObservationCost
{
public:
  /**
   * @brief A CERES cost function class that represents a single observation.
   * Each observation is:
   *  - One point on the calibration target as seen by the camera with the given
   * intrinsics
   *  - Associated with a given wrist position (which may have produced many
   * such calibrations)
   * @param obs
   * @param intr
   * @param wrist_pose
   * @param point_in_target
   */
  ObservationCost(const Eigen::Vector3d& obs, const Eigen::Affine3d& world_to_axis_y,
                  const Eigen::Affine3d& axis_y_to_wrist, const Eigen::Vector3d& point_in_target)
      : obs_(obs), x_axis_pose_(poseEigenToCal(world_to_axis_y)), wrist_pose_(poseEigenToCal(axis_y_to_wrist)), target_pt_(point_in_target)
  {}

  template <typename T>
  bool operator()(const T* pose_axis_x_to_axis_y, const T* pose_wrist_to_camera, const T* pose_base_to_target, T* residual) const
  {
    const T* camera_angle_axis = pose_wrist_to_camera + 0;
    const T* camera_position = pose_wrist_to_camera + 3;

    const T* target_angle_axis = pose_base_to_target + 0;
    const T* target_position = pose_base_to_target + 3;

    const T* misalignment_angle_axis = pose_axis_x_to_axis_y + 0;
    const T* misalignment_position = pose_axis_x_to_axis_y + 3;

    T world_point[3];  // Point in world coordinates
    T x_axis_point[3];    // Point in x-axis-relative coordinates
    T y_axis_base_point[3];    // Point in x-axis-relative coordinates
    T link_point[3];   // Point in link coordinates
    T camera_point[3]; // Point in camera coordinates

    // Transform points into camera coordinates
    T target_pt[3];
    target_pt[0] = T(target_pt_(0));
    target_pt[1] = T(target_pt_(1));
    target_pt[2] = T(target_pt_(2));
    transformPoint(target_angle_axis, target_position, target_pt, world_point);

    // Need to transform point from world into intermediate frame
    poseTransformPoint(x_axis_pose_, world_point, x_axis_point);
    transformPoint(misalignment_angle_axis, misalignment_position, x_axis_point, y_axis_base_point);

    // Then from intermediate frame into camera frame
    poseTransformPoint(wrist_pose_, y_axis_base_point, link_point);
    transformPoint(camera_angle_axis, camera_position, link_point, camera_point);

    residual[0] = camera_point[0] - obs_.x();
    residual[1] = camera_point[1] - obs_.y();
    residual[2] = camera_point[2] - obs_.z();

    return true;
  }

private:
  Eigen::Vector3d obs_;
  Pose6d x_axis_pose_;
  Pose6d wrist_pose_;
  Eigen::Vector3d target_pt_;
};

}

rct_optimizations::Extrinsic3DCameraGantryResult rct_optimizations::optimize(const rct_optimizations::Extrinsic3DCameraGantryProblem& params)
{
  assert(params.observations.size() == params.wrist_poses.size());

  Pose6d internal_base_to_target = poseEigenToCal(params.base_to_target_guess);
  Pose6d internal_axis_misalignment = poseEigenToCal(params.axis_misalignment_guess);
  Pose6d internal_camera_to_wrist = poseEigenToCal(params.wrist_to_camera_guess.inverse());



  ceres::Problem problem;

  for (std::size_t i = 0; i < params.wrist_poses.size(); ++i) // For each wrist pose / image set
  {
    for (std::size_t j = 0; j < params.observations[i].size(); ++j) // For each 3D point seen in the 2D image
    {
      // Define
      const auto& img_obs = params.observations[i][j].in_image;
      const auto& point_in_target = params.observations[i][j].in_target;
      const auto x_axis_to_base = params.x_axis_poses[i].inverse();
      const auto y_axis_to_x_axis = params.y_axis_poses[i].inverse();

      // Allocate Ceres data structures - ownership is taken by the ceres
      // Problem data structure
      auto* cost_fn = new ObservationCost(img_obs, x_axis_to_base, y_axis_to_x_axis, point_in_target);

      auto* cost_block = new ceres::AutoDiffCostFunction<ObservationCost, 3, 6, 6, 6>(cost_fn);

      problem.AddResidualBlock(cost_block, NULL, internal_axis_misalignment.values.data(),
                               internal_camera_to_wrist.values.data(), internal_base_to_target.values.data());
    }
  }

  ceres::Solver::Options options;
  ceres::Solver::Summary summary;

  ceres::Solve(options, &problem, &summary);

  Extrinsic3DCameraGantryResult result;
  result.converged = summary.termination_type == ceres::CONVERGENCE;
  result.base_to_target = poseCalToEigen(internal_base_to_target);
  result.axis_misalignment = poseCalToEigen(internal_axis_misalignment);
  result.wrist_to_camera = poseCalToEigen(internal_camera_to_wrist).inverse();
  result.initial_cost_per_obs = summary.initial_cost / summary.num_residuals;
  result.final_cost_per_obs = summary.final_cost / summary.num_residuals;

  return result;
}
