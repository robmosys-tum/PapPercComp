#include "chair_manipulation_grasp_detection_advanced/point_cloud_registration.h"
#include "chair_manipulation_grasp_detection_advanced/utils.h"
#include "chair_manipulation_grasp_detection_advanced/stopwatch.h"

namespace chair_manipulation
{
void PointCloudRegistrationParameters::load(ros::NodeHandle& nh)
{
  lambda_ = nh.param<double>("lambda", cpd::DEFAULT_LAMBDA);
  beta_ = nh.param<double>("beta", cpd::DEFAULT_BETA);
  max_iterations_ = nh.param<int>("max_iterations", cpd::DEFAULT_MAX_ITERATIONS);
  pre_voxel_grid_leaf_size_ = nh.param<double>("pre_voxel_grid_leaf_size", 0.04);
  post_voxel_grid_leaf_size_ = nh.param<double>("post_voxel_grid_leaf_size", 0.02);
}

PointCloudRegistration::PointCloudRegistration(PointCloudRegistrationParameters params) : params_(std::move(params))
{
  pre_voxel_filter_.setLeafSize(params_.pre_voxel_grid_leaf_size_, params_.pre_voxel_grid_leaf_size_,
                                params_.pre_voxel_grid_leaf_size_);
  post_voxel_filter_.setLeafSize(params_.post_voxel_grid_leaf_size_, params_.post_voxel_grid_leaf_size_,
                                 params_.post_voxel_grid_leaf_size_);
}

void PointCloudRegistration::setInputSource(const PointNormalCloudConstPtr& source_cloud)
{
  source_cloud_ = source_cloud;
}

void PointCloudRegistration::setInputTarget(const PointNormalCloudConstPtr& target_cloud)
{
  target_cloud_ = target_cloud;
}

void PointCloudRegistration::align(PointNormalCloud& aligned_cloud, NonrigidTransform& transform)
{
  Stopwatch stopwatch;

  auto filtered_source_cloud = PointNormalCloudPtr{ new PointNormalCloud };
  pre_voxel_filter_.setInputCloud(source_cloud_);
  pre_voxel_filter_.filter(*filtered_source_cloud);

  auto source_eigen_cloud = std::make_shared<EigenCloud>();
  auto target_eigen_cloud = std::make_shared<EigenCloud>();
  utils::pointCloudToEigen(*filtered_source_cloud, *source_eigen_cloud);
  utils::pointCloudToEigen(*target_cloud_, *target_eigen_cloud);

  cpd::Nonrigid nonrigid;
  nonrigid.lambda(params_.lambda_);
  nonrigid.beta((params_.beta_));
  nonrigid.max_iterations(params_.max_iterations_);
  nonrigid.normalize(false);

  std::size_t iter = 0;
  nonrigid.add_callback(
      [&](const auto& result) { ROS_DEBUG_STREAM_NAMED("point_cloud_registration", "CPD iteration " << ++iter); });

  ROS_DEBUG_STREAM_NAMED("point_cloud_registration", "=== CPD registration ===");
  ROS_DEBUG_STREAM_NAMED("point_cloud_registration", "Max number of iterations: " << params_.max_iterations_);
  ROS_DEBUG_STREAM_NAMED("point_cloud_registration", "Number of source cloud points: " << source_eigen_cloud->rows());
  ROS_DEBUG_STREAM_NAMED("point_cloud_registration", "Number of target cloud points: " << target_eigen_cloud->rows());

  stopwatch.start();
  auto result = nonrigid.run(*target_eigen_cloud, *source_eigen_cloud);
  stopwatch.stop();

  ROS_DEBUG_STREAM_NAMED("point_cloud_registration", "CPD finished.");
  ROS_DEBUG_STREAM_NAMED("point_cloud_registration", "It took" << stopwatch.elapsedSeconds() << "s.");

  auto w = std::make_shared<Eigen::MatrixXd>(result.w);
  transform = NonrigidTransform{ source_eigen_cloud, w, params_.beta_ };
  utils::transformPointCloud(*source_cloud_, aligned_cloud, transform);
}

}  // namespace chair_manipulation
