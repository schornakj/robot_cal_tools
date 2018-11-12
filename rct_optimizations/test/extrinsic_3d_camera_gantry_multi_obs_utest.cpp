#include <gtest/gtest.h>
#include <rct_optimizations/extrinsic_3d_camera_gantry_multi_obs.h>
#include <rct_optimizations/ceres_math_utilities.h>
#include <rct_optimizations/types.h>

#include <rct_optimizations_tests/utilities.h>
#include <rct_optimizations_tests/observation_creator.h>

static void print_results(const rct_optimizations::Extrinsic3DCameraGantryMultiObsResult& r)
{
    // Report results
    std::cout << "Did converge?: " << r.converged << "\n";
    std::cout << "Initial cost?: " << r.initial_cost_per_obs << "\n";
    std::cout << "Final cost?: " << r.final_cost_per_obs << "\n";

    Eigen::Affine3d axis_mis = r.axis_misalignment;
    Eigen::Affine3d cam_mis = r.wrist_to_camera;

    std::cout << "Estimated Axis Misalignment:\n";
    std::cout << axis_mis.matrix() << "\n";
    std::cout << "Estimated Camera Misalignment:\n";
    std::cout << cam_mis.matrix() << "\n";
}

double f_rand(double min, double max)
{
    double f = (double)rand() / max;
    return min + f * (max - min);
}

Eigen::Vector3d apply_misalignment(const Eigen::Vector3d& world_point_in,
                                   const Eigen::Affine3d& tform_world_to_x,
                                   const Eigen::Affine3d& tform_x_to_y,
                                   const Eigen::Affine3d& tform_axis_misalignment,
                                   const Eigen::Affine3d& tform_wrist_misalignment)
{
    Eigen::Affine3d tform_world_to_wrist_perturbed = tform_world_to_x * tform_axis_misalignment * tform_x_to_y * tform_wrist_misalignment;
    Eigen::Affine3d tform_world_to_wrist_unperturbed = tform_world_to_x * tform_x_to_y;
    return tform_world_to_wrist_perturbed * tform_world_to_wrist_unperturbed.inverse() * world_point_in;
}

void run_test()
{

    std::vector<Eigen::Vector3d> world_points {Eigen::Vector3d(0.5, 0.25, -1.0),
                                               Eigen::Vector3d(1.0, 0.25, -1.0),
                                               Eigen::Vector3d(1.0, 0.75, -1.0),
                                               Eigen::Vector3d(0.5, 0.75, -1.0)};

    Eigen::Affine3d transform_world_to_x_first = Eigen::Affine3d::Identity();
    Eigen::Affine3d transform_x_to_y_first = Eigen::Affine3d::Identity();
    Eigen::Affine3d transform_world_to_x_second = Eigen::Affine3d::Identity();
    Eigen::Affine3d transform_x_to_y_second = Eigen::Affine3d::Identity();

    transform_world_to_x_first.translation() = Eigen::Vector3d(0.5, 0.0, 0.0);
    transform_x_to_y_first.translation() = Eigen::Vector3d(0.0, 0.5, 0.0);

    transform_world_to_x_second.translation() = Eigen::Vector3d(1.0, 0.0, 0.0);
    transform_x_to_y_second.translation() = Eigen::Vector3d(0.0, 0.5, 0.0);

    const double spatial_noise = 0.0;
    const double angular_noise = 1. * M_PI / 180.0;

    Eigen::Affine3d true_axis_misalignment = rct_optimizations::test::perturbPose(Eigen::Affine3d::Identity(), spatial_noise, angular_noise);
    std::cout << "real axis misalignment" << std::endl << true_axis_misalignment.matrix() << std::endl;

    Eigen::Affine3d true_wrist_misalignment = rct_optimizations::test::perturbPose(Eigen::Affine3d::Identity(), spatial_noise, angular_noise);
    std::cout << "real wrist misalignment" << std::endl << true_wrist_misalignment.matrix() << std::endl;

    Eigen::Affine3d tform_perturbed = transform_world_to_x_first * true_axis_misalignment * transform_x_to_y_first * true_wrist_misalignment;

    std::cout << "first point in world frame" << std::endl << world_points[0] << std::endl;

    Eigen::Vector3d first_pt_in_wrist_frame = transform_world_to_x_first * transform_x_to_y_first * world_points[0];

    std::cout << "first point in wrist frame" << std::endl << first_pt_in_wrist_frame << std::endl;

    Eigen::Vector3d first_pt_world_frame_perturbed = apply_misalignment(world_points[0], transform_world_to_x_first, transform_x_to_y_first, true_axis_misalignment, true_wrist_misalignment);

    std::cout << "first point in world frame perturbed" << std::endl << first_pt_world_frame_perturbed << std::endl;

    rct_optimizations::Extrinsic3DCameraGantryMultiObsProblem problem_def;
    problem_def.axis_misalignment_guess = Eigen::Affine3d::Identity();
    problem_def.wrist_to_camera_guess = Eigen::Affine3d::Identity();
    problem_def.base_to_target_guess = Eigen::Affine3d::Identity();

    rct_optimizations::CorrespondenceARMulti3D3D corr_base;


// TODO: Add pairs of image points for each object point

    for(std::size_t i = 0; i < world_points.size(); i++)
    {
        std::cout << "CORRESPONDENCE " << i << std::endl;
        rct_optimizations::CorrespondenceARMulti3D3D corr_new;
        corr_new.pose_axis_x_first = transform_world_to_x_first;
        corr_new.pose_axis_y_first = transform_x_to_y_first;
        corr_new.pose_axis_x_second = transform_world_to_x_second;
        corr_new.pose_axis_y_second = transform_x_to_y_second;
        corr_new.id = 0;
        corr_new.index_corner = i;
        corr_new.point_first = apply_misalignment(world_points[i], transform_world_to_x_first, transform_x_to_y_first, true_axis_misalignment, true_wrist_misalignment);
        corr_new.point_second = apply_misalignment(world_points[i], transform_world_to_x_second, transform_x_to_y_second, true_axis_misalignment, true_wrist_misalignment);

        std::cout << "X axis 1: " << std::endl << corr_new.pose_axis_x_first.matrix() << std::endl;
        std::cout << "Y axis 1: " << std::endl << corr_new.pose_axis_y_first.matrix() << std::endl;
        std::cout << "X axis 2: " << std::endl << corr_new.pose_axis_x_second.matrix() << std::endl;
        std::cout << "Y axis 2: " << std::endl << corr_new.pose_axis_y_second.matrix() << std::endl;

        std::cout << "Actual factual position of this point:" << std::endl << world_points[i] << std::endl;
        std::cout << "View from 1st pose:" << std::endl << corr_new.point_first << std::endl;
        std::cout << "Viewed from 2nd pose:" << std::endl << corr_new.point_second << std::endl;
        problem_def.observations.push_back(corr_new);
        std::cout << std::endl;
    }

    auto result = rct_optimizations::optimize(problem_def);

    EXPECT_TRUE(result.converged);
    EXPECT_TRUE(result.final_cost_per_obs < 1.0);

    EXPECT_TRUE(result.axis_misalignment.isApprox(true_axis_misalignment, 1e-6));
    EXPECT_TRUE(result.wrist_to_camera.isApprox(true_wrist_misalignment, 1e-6));

    print_results(result);
}

TEST(CameraOnGantry, placeholder)
{
    run_test();
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
