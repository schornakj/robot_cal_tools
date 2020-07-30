#pragma once

#include <rct_optimizations/types.h>
#include <rct_optimizations/dh_kinematic/dh_chain.h>
#include <rct_optimizations/dh_kinematic/dual_dh_chain_cost_base.h>
#include <rct_optimizations/ceres_math_utilities.h>
#include <rct_optimizations/covariance_analysis.h>

namespace rct_optimizations
{

struct KinematicCalibrationProblemPose3D
{
  KinematicCalibrationProblemPose3D(DHChain camera_chain_, DHChain target_chain_)
    : camera_chain(std::move(camera_chain_))
    , target_chain(std::move(target_chain_))
    , camera_mount_to_camera_guess(Eigen::Isometry3d::Identity())
    , target_mount_to_target_guess(Eigen::Isometry3d::Identity())
    , camera_base_to_target_base_guess(Eigen::Isometry3d::Identity())
  {
  }

  DHChain camera_chain;
  DHChain target_chain;
  KinematicMeasurement::Set observations;
  Eigen::Isometry3d camera_mount_to_camera_guess;
  Eigen::Isometry3d target_mount_to_target_guess;
  Eigen::Isometry3d camera_base_to_target_base_guess;

  /* Create an array of masks
   * 0. Camera DH parameters (size joints x 4)
   * 1. Target DH parameters (size joints x 4)
   * 2. Camera mount to camera position (size 3)
   * 3. Camera mount to camera angle axis (size 3)
   * 4. Target mount to target position (size 3)
   * 5. Target mount to target angle axis (size 3)
   * 6. Camera base to target base position (size 3)
   * 7. Target mount to target base angle axis (size 3)
   */
  std::array<std::vector<int>, 8> mask;

  std::string label_camera_mount_to_camera = "camera_mount_to_camera";
  std::string label_target_mount_to_target = "target_mount_to_target";
  std::string label_camera_base_to_target = "camera_base_to_target";
};

class DualDHChainCostPose3D : public DualDHChainCost
{
  public:
  DualDHChainCostPose3D(const KinematicMeasurement &measurement,
       const DHChain &camera_chain,
       const DHChain &target_chain,
       const Eigen::VectorXd &camera_chain_joints,
       const Eigen::VectorXd &target_chain_joints)
    : DualDHChainCost (camera_chain, target_chain, camera_chain_joints, target_chain_joints)
    , measurement_camera_to_target_(measurement)
  {
  }

  template<typename T>
  bool operator()(T const *const *parameters, T *residual) const
  {
    // Step 1: Load the data
    // The first parameter is a pointer to the DH parameter offsets of the camera kinematic chain
    Eigen::Map<const Eigen::Matrix<T, Eigen::Dynamic, 4>> camera_chain_dh_offsets(parameters[0], camera_chain_.dof(), 4);

    // The next parameter is a pointer to the DH parameter offsets of the target kinematic chain
    Eigen::Map<const Eigen::Matrix<T, Eigen::Dynamic, 4>> target_chain_dh_offsets(parameters[1], target_chain_.dof(), 4);

    // The next two parameters are pointers to the position and angle axis of the transform from the camera mount to the camera
    std::size_t cm_to_c_idx = 2;
    const Isometry3<T> camera_mount_to_camera = createTransform(parameters, cm_to_c_idx);

    // The next two parameters are pointers to the position and angle axis of the transform from the target mount to the target
    std::size_t tm_to_t_idx = cm_to_c_idx + 2;
    const Isometry3<T> target_mount_to_target = createTransform(parameters, tm_to_t_idx);

    // The next two parameters are pointers to the position and angle axis of the transform from the camera chain base to the target chain base
    std::size_t cb_to_tb_idx = tm_to_t_idx + 2;
    const Isometry3<T> camera_base_to_target_base = createTransform(parameters, cb_to_tb_idx);

    // Step 2: Transformation math
    // Build the transforms from the camera chain base out to the camera
    Isometry3<T> camera_chain_fk = camera_chain_.getFK<T>(camera_chain_joints_.cast<T>(),
                                                          camera_chain_dh_offsets);
    Isometry3<T> camera_base_to_camera = camera_chain_fk * camera_mount_to_camera;

    // Build the transforms from the camera chain base out to the target
    Isometry3<T> target_chain_fk = target_chain_.getFK<T>(target_chain_joints_.cast<T>(),
                                                          target_chain_dh_offsets);
    Isometry3<T> camera_base_to_target = camera_base_to_target_base * target_chain_fk
                                         * target_mount_to_target;

    // Now that we have two transforms in the same frame, get the difference between the expected and observed pose of the target
    Isometry3<T> camera_to_target = camera_base_to_camera.inverse() * camera_base_to_target;

    Isometry3<T> camera_to_target_measured = measurement_camera_to_target_.camera_to_target.cast<T>();

    Isometry3<T> tform_error = camera_to_target_measured * camera_to_target.inverse();

    // Calculate elements of residual from error transform
    residual[0] = tform_error.translation().x();
    residual[1] = tform_error.translation().y();
    residual[2] = tform_error.translation().z();

    // compute residual from orientation as nominal vs actual locations of points rotated by error transform
    Vector4<T> p1, p2;
    p1 << T(2.0), T(0.0), T(0.0), T(1.0);
    p2 << T(0.0), T(2.0), T(0.0), T(1.0);

    Vector4<T> p1_t = tform_error * p1;
    Vector4<T> p2_t = tform_error * p2;

    residual[3] = p1_t.x() - p1.x();
    residual[4] = p1_t.y() - p1.y();
    residual[5] = p1_t.z() - p1.z();

    residual[6] = p2_t.x() - p2.x();
    residual[7] = p2_t.y() - p2.y();
    residual[8] = p2_t.z() - p2.z();

    return true;
  }

  protected:
  KinematicMeasurement measurement_camera_to_target_;
};

KinematicCalibrationResult optimize(const KinematicCalibrationProblemPose3D &problem);

} // namespace rct_optimizations
