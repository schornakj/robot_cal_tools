#include "rct_optimizations/extrinsic_3d_camera_cartesian_gantry.h"

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
   *  - One point on the calibration target as seen by the depth camera
   *  - Associated with given positions of the X- and Y-axes of the gantry
   * @param observed_pt_world
   * @param world_to_axis_y
   * @param axis_y_to_wrist
   * @param point_in_target
   */
  ObservationCost(const Eigen::Vector3d& observed_pt_world, const Eigen::Affine3d& axis_x_to_world,
                  const Eigen::Affine3d& axis_y_to_axis_x, const Eigen::Vector3d& point_in_target)
      : observed_pt_world_(observed_pt_world),
        pose_x_axis_to_world_(poseEigenToCal(axis_x_to_world)),
        pose_x_axis_to_world_inv_(poseEigenToCal(axis_x_to_world.inverse())),
        pose_y_axis_to_x_axis_(poseEigenToCal(axis_y_to_axis_x)),
        pose_y_axis_to_x_axis_inv_(poseEigenToCal(axis_y_to_axis_x.inverse())),
        target_pt_object_(point_in_target)
  {}

  template <typename T>
  bool operator()(const T* transform_axis_misalignment, const T* transform_wrist_to_camera, const T* transform_base_to_target, const T* object_point_tweak, T* residual) const
  {
    const T* camera_angle_axis = transform_wrist_to_camera + 0;
    const T* camera_position = transform_wrist_to_camera + 3;

    const T* target_angle_axis = transform_base_to_target + 0;
    const T* target_position = transform_base_to_target + 3;

    const T* misalignment_angle_axis = transform_axis_misalignment + 0;
    const T* misalignment_position = transform_axis_misalignment + 3;

    T world_point[3];  // Point in world coordinates
    T x_axis_point[3];    // Point in x-axis-relative coordinates
    T y_axis_base_point[3];    // Point in y-axis-relative coordinates
    T wrist_point[3];   // Point in link coordinates
    T camera_point[3]; // Point in camera coordinates

    T y_axis_point_undistorted[3];
    T world_point_undistorted[3];

    // Transform points into camera coordinates
    T target_pt[3];
    target_pt[0] = T(target_pt_object_(0)) + object_point_tweak[0];
    target_pt[1] = T(target_pt_object_(1)) + object_point_tweak[1];
    target_pt[2] = T(target_pt_object_(2)) + object_point_tweak[2];
    transformPoint(target_angle_axis, target_position, target_pt, world_point);

    // Need to transform point from world into intermediate frame
    poseTransformPoint(pose_x_axis_to_world_, world_point, x_axis_point);

//    transformPoint(misalignment_angle_axis, misalignment_position, x_axis_point, y_axis_base_point);
    ceres::AngleAxisRotatePoint(misalignment_angle_axis, x_axis_point, y_axis_base_point);

    // Then from intermediate frame into camera frame
    poseTransformPoint(pose_y_axis_to_x_axis_, y_axis_base_point, wrist_point);

//    transformPoint(camera_angle_axis, camera_position, wrist_point, camera_point);
    ceres::AngleAxisRotatePoint(camera_angle_axis, wrist_point, camera_point);

    poseTransformPoint(pose_x_axis_to_world_inv_, camera_point, y_axis_point_undistorted);
    poseTransformPoint(pose_y_axis_to_x_axis_inv_, y_axis_point_undistorted, world_point_undistorted);

    // Cost needs to account for getting the image and object points close together without having the object point deviate too much from its initial position.
    residual[0] = world_point_undistorted[0] - observed_pt_world_.x() + object_point_tweak[0];
    residual[1] = world_point_undistorted[1] - observed_pt_world_.y() + object_point_tweak[1];
    residual[2] = world_point_undistorted[2] - observed_pt_world_.z() + object_point_tweak[2];

    return true;
  }

private:
  Eigen::Vector3d observed_pt_world_;
  Pose6d pose_x_axis_to_world_;
  Pose6d pose_x_axis_to_world_inv_;

  Pose6d pose_y_axis_to_x_axis_;
  Pose6d pose_y_axis_to_x_axis_inv_;
  Eigen::Vector3d target_pt_object_;
};

}

rct_optimizations::Extrinsic3DCameraGantryResult rct_optimizations::optimize(const rct_optimizations::Extrinsic3DCameraGantryProblem& params)
{
  assert((params.observations.size() == params.x_axis_poses.size()) && (params.observations.size() == params.y_axis_poses.size()));

  /*
   * Pass in initial guesses for each transform, as well as default values for the adjustments to each object point.
   * Each observation is a known object point and an observed image point for a particular corner of a particular marker.
   * Cost for each observation is a function of the transforms, the adjustment to the specific object point associated with that observation,
   * and the positions of the gantry axes.
   *
   */


  Pose6d internal_base_to_target = poseEigenToCal(params.base_to_target_guess);
  Pose6d internal_axis_misalignment = poseEigenToCal(params.axis_misalignment_guess);
  Pose6d internal_wrist_to_camera = poseEigenToCal(params.wrist_to_camera_guess);
//  Vec3d internal_tweak = vecEigenToCal(Eigen::Vector3d(0,0,0));


  std::map<std::tuple<int, int>, Vec3d> internal_tweaks;
  std::for_each(params.tweaks_guess.begin(), params.tweaks_guess.end(), [&](std::pair<std::tuple<int, int>, Eigen::Vector3d> p){
      Vec3d tweak = vecEigenToCal(p.second);
      internal_tweaks.insert(std::make_pair(p.first, tweak));
  });

  ceres::Problem problem;
  std::vector<ceres::ResidualBlockId> res_block_ids;

  for (std::size_t i = 0; i < params.x_axis_poses.size(); ++i) // For each gantry pose / image set
  {
    for (std::size_t j = 0; j < params.observations[i].size(); ++j) // For each 3D target point seen in the point cloud
    {
      // Define
      const auto& img_obs = params.observations[i][j].in_image;
      const auto& point_in_target = params.observations[i][j].in_target;
      const auto x_axis_to_base = params.x_axis_poses[i].inverse();
      const auto y_axis_to_x_axis = params.y_axis_poses[i].inverse();

      const auto obs_id = params.observations[i][j].id;
      const auto obs_corner_index = params.observations[i][j].index_corner;

      // Allocate Ceres data structures - ownership is taken by the ceres
      // Problem data structure
      auto* cost_fn = new ObservationCost(img_obs, x_axis_to_base, y_axis_to_x_axis, point_in_target);

      auto* cost_block = new ceres::AutoDiffCostFunction<ObservationCost, 3, 6, 6, 6, 3>(cost_fn);

      ceres::ResidualBlockId id = problem.AddResidualBlock(cost_block, NULL, internal_axis_misalignment.values.data(),
                                                           internal_wrist_to_camera.values.data(), internal_base_to_target.values.data(),
                                                           internal_tweaks.at(std::tuple<int,int>(obs_id,obs_corner_index)).values.data());

      res_block_ids.push_back(id);
    }
  }

  ceres::Problem::EvaluateOptions eval_options;
  eval_options.residual_blocks = res_block_ids;
  double total_cost = 0.0;
  std::vector<double> residuals;
  problem.Evaluate(eval_options, &total_cost, &residuals, nullptr, nullptr);
//  for (int i = 0; i < residuals.size(); i+=3)
//  {
//      std::cout << residuals[i] << " " << residuals[i+1] << " " << residuals[i+2] << std::endl;
//  }

  ceres::Solver::Options options;
  options.function_tolerance = 1e-9;
  ceres::Solver::Summary summary;

  ceres::Solve(options, &problem, &summary);

  Extrinsic3DCameraGantryResult result;
  result.converged = summary.termination_type == ceres::CONVERGENCE;
  result.base_to_target = poseCalToEigen(internal_base_to_target);
  result.axis_misalignment = poseCalToEigen(internal_axis_misalignment);
  result.wrist_to_camera = poseCalToEigen(internal_wrist_to_camera);
  result.initial_cost_per_obs = summary.initial_cost / summary.num_residuals;
  result.final_cost_per_obs = summary.final_cost / summary.num_residuals;

  std::for_each(internal_tweaks.begin(), internal_tweaks.end(), [&](std::pair<std::tuple<int, int>, Vec3d> p){
      Eigen::Vector3d tweak = vecCalToEigen(p.second);
      result.tweaks.insert(std::make_pair(p.first, tweak));
  });

  return result;
}
