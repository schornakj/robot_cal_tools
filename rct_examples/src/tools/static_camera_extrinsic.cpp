// Utilities for loading data sets and calib parameters from YAML files via ROS
#include "rct_ros_tools/data_set.h"
#include "rct_ros_tools/parameter_loaders.h"
#include "rct_ros_tools/target_loaders.h"
#include "rct_ros_tools/print_utils.h"
// To find 2D  observations from images
#include <rct_image_tools/modified_circle_grid_finder.h>
#include <rct_image_tools/image_utils.h>
// The calibration function for 'static camera' on robot wrist
#include <rct_optimizations/extrinsic_hand_eye.h>

#include <opencv2/highgui.hpp>
#include <ros/ros.h>

#include <opencv2/imgproc.hpp>
#include <rct_optimizations/ceres_math_utilities.h>
#include <rct_optimizations/pnp.h>

using namespace rct_optimizations;
using namespace rct_image_tools;
using namespace rct_ros_tools;

static void reproject(const Eigen::Isometry3d& wrist_to_target, const Eigen::Isometry3d& base_to_camera,
                      const Observation2D3D& obs, const CameraIntrinsics& intr,
                      const ModifiedCircleGridTarget& target, const cv::Mat& image)
{
  Eigen::Isometry3d camera_to_target = base_to_camera.inverse() * (obs.to_target_mount * wrist_to_target);
  std::vector<cv::Point2d> reprojections = getReprojections(camera_to_target, intr, target.createPoints());

  cv::Mat frame = image.clone();
  drawReprojections(reprojections, 3, cv::Scalar(0, 0, 255), frame);

  // We want to compute the "positional error" as well
  // So first we compute the "camera to target" transform based on the calibration...
  PnPProblem pb;
  pb.camera_to_target_guess = camera_to_target;
  pb.correspondences = obs.correspondence_set;
  pb.intr = intr;
  PnPResult r = optimize(pb);

  printOptResults(r.converged, r.initial_cost_per_obs, r.final_cost_per_obs);
  printNewLine();

  printTransformDiff(camera_to_target, r.camera_to_target, "Camera", "Target", "PNP Diff");
  printNewLine();

  cv::imshow("repr", frame);
  cv::waitKey();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "camera_on_wrist_extrinsic");
  ros::NodeHandle pnh("~");

  // Load the data set path from ROS param
  std::string data_path;
  if (!pnh.getParam("data_path", data_path))
  {
    ROS_ERROR("Must set 'data_path' parameter");
    return 1;
  }

  // Attempt to load the data set from the specified path
  ExtrinsicDataSet data_set;
  try
  {
    data_set = parseFromFile(data_path);
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM("Failed to parse data set from path " << data_path << ": " << e.what());
    return 2;
  }

  try
  {
    ModifiedCircleGridTarget target = TargetLoader<ModifiedCircleGridTarget>::load(pnh, "target_definition");

    CameraIntrinsics intr = loadIntrinsics(pnh, "intrinsics");

    // Lets create a class that will search for the target in our raw images.
    ModifiedCircleGridTargetFinder target_finder(target);

    // Now we construct our problem:
    ExtrinsicHandEyeProblem2D3D problem_def;
    // Our camera intrinsics to use
    problem_def.intr = intr;

    // Our 'base to camera guess': A camera off to the side, looking at a point centered in front of the robot
    problem_def.camera_mount_to_camera_guess = loadPose(pnh, "base_to_camera_guess");
    problem_def.target_mount_to_target_guess = loadPose(pnh, "wrist_to_target_guess");

    // Finally, we need to process our images into correspondence sets: for each dot in the
    // target this will be where that dot is in the target and where it was seen in the image.
    // Repeat for each image. We also tell where the wrist was when the image was taken.
    ExtrinsicDataSet found_images;
    problem_def.observations.reserve(data_set.images.size());
    for (std::size_t i = 0; i < data_set.images.size(); ++i)
    {
      // Try to find the circle grid in this image:
      rct_image_tools::TargetFeatures target_features;
      try
      {
        target_features = target_finder.findTargetFeatures(data_set.images[i]);

        // Show the points we detected
        cv::imshow("points", target_finder.drawTargetFeatures(data_set.images[i], target_features));
        cv::waitKey();
      }
      catch (const std::runtime_error& ex)
      {
        ROS_WARN_STREAM("Unable to find the circle grid in image " << i << "' :" << ex.what() << "'");
        cv::imshow("points", data_set.images[i]);
        cv::waitKey();
        continue;
      }

      // cache found image data
      found_images.images.push_back(data_set.images[i]);
      found_images.tool_poses.push_back(data_set.tool_poses[i]);

      Observation2D3D obs;

      // So for each image we need to:
      //// 1. Record the wrist position
      obs.to_target_mount = data_set.tool_poses[i];
      obs.to_camera_mount = Eigen::Isometry3d::Identity();

      //// And finally add that to the problem
      obs.correspondence_set = target.createCorrespondences(target_features);

      problem_def.observations.push_back(obs);
    }

    // Run optimization
    ExtrinsicHandEyeResult opt_result = optimize(problem_def);

    // Report results
    printOptResults(opt_result.converged, opt_result.initial_cost_per_obs, opt_result.final_cost_per_obs);
    printNewLine();

    Eigen::Isometry3d c = opt_result.camera_mount_to_camera;
    printTransform(c, "Base", "Camera", "BASE TO CAMERA");
    printNewLine();

    Eigen::Isometry3d t = opt_result.target_mount_to_target;
    printTransform(t, "Wrist", "Target", "WRIST_TO_TARGET");
    printNewLine();

    for (std::size_t i = 0; i < found_images.images.size(); ++i)
    {
      printTitle("REPROJECT IMAGE " + std::to_string(i));
      reproject(opt_result.target_mount_to_target, opt_result.camera_mount_to_camera, problem_def.observations[i], intr,
                target, found_images.images[i]);
    }
  }
  catch (const std::exception& ex)
  {
    ROS_ERROR_STREAM(ex.what());
    return -1;
  }

  return 0;
}
