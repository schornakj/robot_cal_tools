#include "rct_optimizations/extrinsic_3d_camera_gantry_multi_obs.h"

#include "rct_optimizations/ceres_math_utilities.h"
#include "rct_optimizations/eigen_conversions.h"
#include "rct_optimizations/types.h"

#include <ceres/ceres.h>
#include <iostream>
#include <Eigen/Dense>

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
  ObservationCost(const Eigen::Vector3d& translation_world_to_point_first,
                  const Eigen::Vector3d& translation_world_to_point_second,
                  const Eigen::Affine3d& world_to_x_first,
                  const Eigen::Affine3d& x_to_y_first,
                  const Eigen::Affine3d& world_to_x_second,
                  const Eigen::Affine3d& x_to_y_second)
      : observed_pt_first_(translation_world_to_point_first),
        observed_pt_second_(translation_world_to_point_second),

        transform_world_to_x_first_(poseEigenToCal(world_to_x_first)),
        transform_world_to_x_second_(poseEigenToCal(world_to_x_second)),

        transform_x_to_world_first_(poseEigenToCal(world_to_x_first.inverse())),
        transform_x_to_world_second_(poseEigenToCal(world_to_x_second.inverse())),

        transform_x_to_y_first_(poseEigenToCal(x_to_y_first)),
        transform_x_to_y_second_(poseEigenToCal(x_to_y_second)),

        transform_y_to_x_first_(poseEigenToCal(x_to_y_first.inverse())),
        transform_y_to_x_second_(poseEigenToCal(x_to_y_second.inverse()))
  {
      // BUG: transform_pt_to_wrist_first_ seems incorrect
      Eigen::Affine3d transform_world_to_point_first = Eigen::Affine3d::Identity();
      transform_world_to_point_first.translation() = translation_world_to_point_first;
      Eigen::Affine3d transform_wrist_to_point_first = (world_to_x_first * x_to_y_first).inverse() * transform_world_to_point_first;
      transform_pt_to_wrist_first_ = poseEigenToCal(transform_wrist_to_point_first.inverse());

//      std::cout << "1 - World to Point: " << std::endl << transform_world_to_point_first.matrix() << std::endl;
//      std::cout << "1 - World to X: " << std::endl << world_to_x_first.matrix() << std::endl;
//      std::cout << "1 - X to Y: " << std::endl << x_to_y_first.matrix() << std::endl;
//      std::cout << "1 - Wrist to Pt: " << std::endl << transform_wrist_to_point_first.matrix() << std::endl;

      Eigen::Affine3d transform_world_to_point_second = Eigen::Affine3d::Identity();
      transform_world_to_point_second.translation() = translation_world_to_point_second;
      Eigen::Affine3d transform_wrist_to_point_second = (world_to_x_second * x_to_y_second).inverse() * transform_world_to_point_second;
      transform_pt_to_wrist_second_ = poseEigenToCal(transform_wrist_to_point_second.inverse());

//      std::cout << "2 - World to Point: " << std::endl << transform_world_to_point_second.matrix() << std::endl;
//      std::cout << "2 - World to X: " << std::endl << world_to_x_second.matrix() << std::endl;
//      std::cout << "2 - X to Y: " << std::endl << x_to_y_second.matrix() << std::endl;
//      std::cout << "2 - Wrist to Pt: " << std::endl << transform_wrist_to_point_second.matrix() << std::endl;
//      std::cout << transform_wrist_to_point_second.matrix() << std::endl;

  }

    template <typename T>
    bool operator()(const T* transform_axis_misalignment, const T* transform_wrist_misalignment, T* residual) const
    {
        const T* wrist_angle_axis = transform_wrist_misalignment + 0;
        const T* wrist_position = transform_wrist_misalignment + 3;

        const T* misalignment_angle_axis = transform_axis_misalignment + 0;
        const T* misalignment_position = transform_axis_misalignment + 3;

        T obs_pt_first[3];
        obs_pt_first[0] = T(observed_pt_first_[0]);
        obs_pt_first[1] = T(observed_pt_first_[1]);
        obs_pt_first[2] = T(observed_pt_first_[2]);

//        std::cout << "Obs Pt 1: " << std::endl << obs_pt_first[0] << std::endl << obs_pt_first[1] << std::endl << obs_pt_first[2] << std::endl;

        T shoulder_pt_first_uncorr[3];
        T shoulder_pt_first_corr[3];
        T wrist_pt_first_uncorr[3];
        T wrist_pt_first_corr[3];
        T target_pt_corr_first[3];

        T shoulder_pt_first_back[3];
        T obs_pt_first_back[3];

        poseTransformPoint(transform_x_to_world_first_, obs_pt_first, shoulder_pt_first_uncorr);
//        std::cout << "1 shoulder_uncorr: " << std::endl << shoulder_pt_first_uncorr[0] << std::endl << shoulder_pt_first_uncorr[1] << std::endl << shoulder_pt_first_uncorr[2] << std::endl;
        transformPoint(misalignment_angle_axis, misalignment_position, shoulder_pt_first_uncorr, shoulder_pt_first_corr);
//        std::cout << "1 shoulder_corr: " << std::endl << shoulder_pt_first_corr[0] << std::endl << shoulder_pt_first_corr[1] << std::endl << shoulder_pt_first_corr[2] << std::endl;
        poseTransformPoint(transform_y_to_x_first_, shoulder_pt_first_corr, wrist_pt_first_uncorr);
//        std::cout << "1 wrist_uncorr: " << std::endl << wrist_pt_first_uncorr[0] << std::endl << wrist_pt_first_uncorr[1] << std::endl << wrist_pt_first_uncorr[2] << std::endl;
        transformPoint(wrist_angle_axis, wrist_position, wrist_pt_first_uncorr, wrist_pt_first_corr);
//        std::cout << "1 wrist_corr: " << std::endl << wrist_pt_first_corr[0] << std::endl << wrist_pt_first_corr[1] << std::endl << wrist_pt_first_corr[2] << std::endl;
        poseTransformPoint(transform_x_to_y_first_, wrist_pt_first_corr, shoulder_pt_first_back);
//        std::cout << "1 shoulder_back: " << std::endl << shoulder_pt_first_back[0] << std::endl << shoulder_pt_first_back[1] << std::endl << shoulder_pt_first_back[2] << std::endl;
        poseTransformPoint(transform_world_to_x_first_, shoulder_pt_first_back, obs_pt_first_back);
//        std::cout << "1 obs_pt_back: " << std::endl << obs_pt_first_back[0] << std::endl << obs_pt_first_back[1] << std::endl << obs_pt_first_back[2] << std::endl;



//        poseTransformPoint(transform_pt_to_wrist_first_, wrist_pt_first_corr, target_pt_corr_first);

//        std::cout << "Corr Pt 1: " << std::endl << target_pt_corr_first[0] << std::endl << target_pt_corr_first[1] << std::endl << target_pt_corr_first[2] << std::endl;

        T obs_pt_second[3];
        obs_pt_second[0] = T(observed_pt_second_[0]);
        obs_pt_second[1] = T(observed_pt_second_[1]);
        obs_pt_second[2] = T(observed_pt_second_[2]);

//        std::cout << "Obs Pt 2: " << std::endl << obs_pt_second[0] << std::endl << obs_pt_second[1] << std::endl << obs_pt_second[2] << std::endl;


        T shoulder_pt_second_uncorr[3];
        T shoulder_pt_second_corr[3];
        T wrist_pt_second_uncorr[3];
        T wrist_pt_second_corr[3];
        T target_pt_corr_second[3];

        T shoulder_pt_second_back[3];
        T obs_pt_second_back[3];

        poseTransformPoint(transform_x_to_world_second_, obs_pt_second, shoulder_pt_second_uncorr);
//        std::cout << "2 shoulder_uncorr: " << std::endl << shoulder_pt_second_uncorr[0] << std::endl << shoulder_pt_second_uncorr[1] << std::endl << shoulder_pt_second_uncorr[2] << std::endl;
        transformPoint(misalignment_angle_axis, misalignment_position, shoulder_pt_second_uncorr, shoulder_pt_second_corr);
//        std::cout << "2 shoulder_corr: " << std::endl << shoulder_pt_second_corr[0] << std::endl << shoulder_pt_second_corr[1] << std::endl << shoulder_pt_second_corr[2] << std::endl;
        poseTransformPoint(transform_y_to_x_second_, shoulder_pt_second_corr, wrist_pt_second_uncorr);
//        std::cout << "2 wrist_uncorr: " << std::endl << wrist_pt_second_uncorr[0] << std::endl << wrist_pt_second_uncorr[1] << std::endl << wrist_pt_second_uncorr[2] << std::endl;
        transformPoint(wrist_angle_axis, wrist_position, wrist_pt_second_uncorr, wrist_pt_second_corr);
//        std::cout << "2 wrist_corr: " << std::endl << wrist_pt_second_corr[0] << std::endl << wrist_pt_second_corr[1] << std::endl << wrist_pt_second_corr[2] << std::endl;
        poseTransformPoint(transform_x_to_y_second_, wrist_pt_second_corr, shoulder_pt_second_back);
//        std::cout << "2 shoulder_back: " << std::endl << shoulder_pt_second_back[0] << std::endl << shoulder_pt_second_back[1] << std::endl << shoulder_pt_second_back[2] << std::endl;
        poseTransformPoint(transform_world_to_x_second_, shoulder_pt_second_back, obs_pt_second_back);
//        std::cout << "2 obs_pt_back: " << std::endl << obs_pt_second_back[0] << std::endl << obs_pt_second_back[1] << std::endl << obs_pt_second_back[2] << std::endl;



//        poseTransformPoint(transform_pt_to_wrist_second_, wrist_pt_second_corr, target_pt_corr_second);

//        std::cout << "Corr Pt 2: " << std::endl << target_pt_corr_second[0] << std::endl << target_pt_corr_second[1] << std::endl << target_pt_corr_second[2] << std::endl;

        residual[0] = obs_pt_second_back[0] - obs_pt_first_back[0];
        residual[1] = obs_pt_second_back[1] - obs_pt_first_back[1];
        residual[2] = obs_pt_second_back[2] - obs_pt_first_back[2];

//        std::cout << "Res: " << std::endl << residual[0] << std::endl << residual[1] << std::endl << residual[2] << std::endl;


        return true;
    }

private:
  Eigen::Vector3d observed_pt_first_;
  Eigen::Vector3d observed_pt_second_;

  Pose6d transform_pt_to_wrist_first_, transform_pt_to_wrist_second_;

  Pose6d transform_x_to_world_first_, transform_x_to_world_second_;

  Pose6d transform_world_to_x_first_, transform_world_to_x_second_;

  Pose6d transform_wrist_to_pt_first_, transform_wrist_to_pt_second_;

  Pose6d transform_x_to_y_first_, transform_x_to_y_second_;
  Pose6d transform_y_to_x_first_, transform_y_to_x_second_;
};

}

rct_optimizations::Extrinsic3DCameraGantryMultiObsResult rct_optimizations::optimize(const rct_optimizations::Extrinsic3DCameraGantryMultiObsProblem& params)
{

  /*
   * Pass in initial guesses for each transform, as well as default values for the adjustments to each object point.
   * Each observation is a known object point and an observed image point for a particular corner of a particular marker.
   * Cost for each observation is a function of the transforms, the adjustment to the specific object point associated with that observation,
   * and the positions of the gantry axes.
   *
   */


  Pose6d internal_axis_misalignment = poseEigenToCal(params.axis_misalignment_guess);
  Pose6d internal_wrist_to_camera = poseEigenToCal(params.wrist_to_camera_guess);

  ceres::Problem problem;
  std::vector<ceres::ResidualBlockId> res_block_ids;

  for (std::size_t i = 0; i < params.observations.size(); ++i) // For each gantry pose / image set
  {
//      std::cout << "OBS " << i << std::endl;

      // Define
      const auto& img_obs_first = params.observations[i].point_first;
      const auto& img_obs_second = params.observations[i].point_second;

      const auto base_to_x_axis_first = params.observations[i].pose_axis_x_first;
      const auto base_to_y_axis_first = params.observations[i].pose_axis_y_first;

      const auto base_to_x_axis_second = params.observations[i].pose_axis_x_second;
      const auto base_to_y_axis_second = params.observations[i].pose_axis_y_second;

      const auto obs_id = params.observations[i].id;
      const auto obs_corner_index = params.observations[i].index_corner;

      // Allocate Ceres data structures - ownership is taken by the ceres
      // Problem data structure
      auto* cost_fn = new ObservationCost(img_obs_first, img_obs_second, base_to_x_axis_first, base_to_y_axis_first, base_to_x_axis_second, base_to_y_axis_second);

      auto* cost_block = new ceres::AutoDiffCostFunction<ObservationCost, 3, 6, 6>(cost_fn);

      ceres::ResidualBlockId id = problem.AddResidualBlock(cost_block, NULL,
                                                           internal_axis_misalignment.values.data(),
                                                           internal_wrist_to_camera.values.data());

      res_block_ids.push_back(id);

  }

  ceres::Problem::EvaluateOptions eval_options;
  eval_options.residual_blocks = res_block_ids;
  double total_cost = 0.0;
  std::vector<double> residuals;
  problem.Evaluate(eval_options, &total_cost, &residuals, nullptr, nullptr);
//  std::cout << "----------------" << std::endl << "RESIDUALS" << std::endl;
//  for (int i = 0; i < residuals.size(); i+=3)
//  {
//      std::cout << sqrt(residuals[i]) << " " << sqrt(residuals[i+1]) << " " << sqrt(residuals[i+2]) << std::endl;
//  }

  ceres::Solver::Options options;
  options.function_tolerance = 1e-9;
  ceres::Solver::Summary summary;

  ceres::Solve(options, &problem, &summary);

  Extrinsic3DCameraGantryMultiObsResult result;
  result.converged = summary.termination_type == ceres::CONVERGENCE;
  result.axis_misalignment = poseCalToEigen(internal_axis_misalignment);
  result.axis_misalignment_inv = poseCalToEigen(internal_axis_misalignment).inverse();

  result.wrist_to_camera = poseCalToEigen(internal_wrist_to_camera);
  result.wrist_to_camera_inv = poseCalToEigen(internal_wrist_to_camera).inverse();

  result.initial_cost_per_obs = sqrt(summary.initial_cost / summary.num_residuals);
  result.final_cost_per_obs = sqrt(summary.final_cost / summary.num_residuals);

  return result;
}
