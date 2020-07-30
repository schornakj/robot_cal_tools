#pragma once

#include <rct_optimizations/types.h>
#include <rct_optimizations/dh_kinematic/dh_chain.h>
#include <rct_optimizations/covariance_analysis.h>

namespace rct_optimizations
{
/**
   * @brief Create a mask of parameter indices from a matrix of boolean values
   * The indices are calculated in column-wise order because Eigen stores it's values internally in column-wise order by default
   * @param mask
   * @return
   */
inline std::vector<int> createDHMask(const Eigen::Array<bool, Eigen::Dynamic, 4>& mask)
{
  std::vector<int> out;
  out.reserve(mask.size());

  const Eigen::Index rows = mask.rows();
  for (Eigen::Index row = 0; row < mask.rows(); ++row)
  {
    for (Eigen::Index col = 0; col < mask.cols(); ++col)
    {
      if (mask(row, col))
      {
        out.push_back(rows * col + row);
      }
    }
  }

  return out;
}


struct KinematicCalibrationResult
{
  bool converged;
  double initial_cost_per_obs;
  double final_cost_per_obs;

  Eigen::Isometry3d camera_mount_to_camera;
  Eigen::Isometry3d target_mount_to_target;
  Eigen::Isometry3d camera_base_to_target_base;
  Eigen::MatrixX4d camera_chain_dh_offsets;
  Eigen::MatrixX4d target_chain_dh_offsets;

  CovarianceResult covariance;
};

class DualDHChainCost
{
public:
  DualDHChainCost(const DHChain &camera_chain,
                  const DHChain &target_chain,
                  const Eigen::VectorXd &camera_chain_joints,
                  const Eigen::VectorXd &target_chain_joints)
    : camera_chain_(camera_chain)
    , target_chain_(target_chain)
    , camera_chain_joints_(camera_chain_joints)
    , target_chain_joints_(target_chain_joints)

  {
  }

  template<typename T>
  static Isometry3<T> createTransform(T const *const *params, const std::size_t idx)
  {
    Eigen::Map<const Vector3<T>> t(params[idx]);
    Eigen::Map<const Vector3<T>> aa(params[idx + 1]);

    Isometry3<T> result = Isometry3<T>::Identity() * Eigen::Translation<T, 3>(t);

    T aa_norm = aa.norm();
    if (aa_norm > std::numeric_limits<T>::epsilon())
    {
      result *= Eigen::AngleAxis<T>(aa_norm, aa.normalized());
    }
    return result;
  }

  static std::vector<double *> constructParameters(Eigen::MatrixX4d &camera_chain_dh_offsets,
                                                   Eigen::MatrixX4d &target_chain_dh_offsets,
                                                   Eigen::Vector3d &camera_mount_to_camera_position,
                                                   Eigen::Vector3d &camera_mount_to_camera_angle_axis,
                                                   Eigen::Vector3d &target_mount_to_target_position,
                                                   Eigen::Vector3d &target_mount_to_target_angle_axis,
                                                   Eigen::Vector3d &camera_chain_base_to_target_chain_base_position,
                                                   Eigen::Vector3d &camera_chain_base_to_target_chain_base_angle_axis)
  {
    std::vector<double *> parameters;
    parameters.push_back(camera_chain_dh_offsets.data());
    parameters.push_back(target_chain_dh_offsets.data());
    parameters.push_back(camera_mount_to_camera_position.data());
    parameters.push_back(camera_mount_to_camera_angle_axis.data());
    parameters.push_back(target_mount_to_target_position.data());
    parameters.push_back(target_mount_to_target_angle_axis.data());
    parameters.push_back(camera_chain_base_to_target_chain_base_position.data());
    parameters.push_back(camera_chain_base_to_target_chain_base_angle_axis.data());
    return parameters;
  }

  static std::vector<std::vector<std::string>> constructParameterLabels(const std::vector<std::array<std::string, 4>>& camera_chain_labels,
                                                                        const std::vector<std::array<std::string, 4>>& target_chain_labels,
                                                                        const std::array<std::string, 3>& camera_mount_to_camera_position_labels,
                                                                        const std::array<std::string, 3>& camera_mount_to_camera_angle_axis_labels,
                                                                        const std::array<std::string, 3>& target_mount_to_target_position_labels,
                                                                        const std::array<std::string, 3>& target_mount_to_target_angle_axis_labels,
                                                                        const std::array<std::string, 3>& camera_chain_base_to_target_chain_base_position_labels,
                                                                        const std::array<std::string, 3>& camera_chain_base_to_target_chain_base_angle_axis_labels)
  {
    std::vector<std::vector<std::string>> param_labels;
    std::vector<std::string> cc_labels_concatenated;
    for (auto cc_label : camera_chain_labels)
    {
      cc_labels_concatenated.insert(cc_labels_concatenated.end(), cc_label.begin(), cc_label.end());
    }
    param_labels.push_back(cc_labels_concatenated);

    std::vector<std::string> tc_labels_concatenated;
    for (auto tc_label : target_chain_labels)
    {
      tc_labels_concatenated.insert(tc_labels_concatenated.end(), tc_label.begin(), tc_label.end());
    }
    param_labels.push_back(tc_labels_concatenated);

    param_labels.emplace_back(camera_mount_to_camera_position_labels.begin(), camera_mount_to_camera_position_labels.end());
    param_labels.emplace_back(camera_mount_to_camera_angle_axis_labels.begin(), camera_mount_to_camera_angle_axis_labels.end());
    param_labels.emplace_back(target_mount_to_target_position_labels.begin(), target_mount_to_target_position_labels.end());
    param_labels.emplace_back(target_mount_to_target_angle_axis_labels.begin(), target_mount_to_target_angle_axis_labels.end());
    param_labels.emplace_back(camera_chain_base_to_target_chain_base_position_labels.begin(), camera_chain_base_to_target_chain_base_position_labels.end());
    param_labels.emplace_back(camera_chain_base_to_target_chain_base_angle_axis_labels.begin(), camera_chain_base_to_target_chain_base_angle_axis_labels.end());
    return param_labels;
  }

protected:
  const DHChain &camera_chain_;
  const DHChain &target_chain_;

  Eigen::VectorXd camera_chain_joints_;
  Eigen::VectorXd target_chain_joints_;
};
}
