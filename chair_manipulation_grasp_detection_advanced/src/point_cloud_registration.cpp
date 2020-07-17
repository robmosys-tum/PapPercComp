#include <chair_manipulation_grasp_detection_advanced/point_cloud_registration.h>

namespace chair_manipulation
{
void PointCloudRegistrationParameters::load(ros::NodeHandle& nh)
{
  lambda_ = nh.param<double>("lambda", cpd::DEFAULT_LAMBDA);
  beta_ = nh.param<double>("beta", cpd::DEFAULT_BETA);
  max_iterations_ = nh.param<int>("max_iterations", cpd::DEFAULT_MAX_ITERATIONS);
}

PointCloudRegistration::PointCloudRegistration(PointCloudRegistrationParameters params) : params_(std::move(params))
{
  nonrigid_.lambda(params_.lambda_);
  nonrigid_.beta((params.beta_));
  nonrigid_.max_iterations(params.max_iterations_);
  nonrigid_.normalize(false);
  nonrigid_.add_callback([&](const cpd::NonrigidResult& result) {
    double runtime_seconds = (result.runtime.count() / 1000000.);
    ROS_DEBUG_STREAM_NAMED("point_cloud_registration", "CPD iteration " << result.iterations);
    ROS_DEBUG_STREAM_NAMED("point_cloud_registration", "CPD runtime " << runtime_seconds << "s.");
  });
}

void PointCloudRegistration::setInputSource(const PointCloudRegistration::PointCloudConstPtr& source_cloud)
{
  source_cloud_ = source_cloud;
}

void PointCloudRegistration::setInputTarget(const PointCloudRegistration::PointCloudConstPtr& target_cloud)
{
  target_cloud_ = target_cloud;
}

void PointCloudRegistration::align(PointCloud& aligned_cloud, NonrigidTransform& transform)
{
  auto result = nonrigid_.run(*target_cloud_, *source_cloud_);
  aligned_cloud = result.points;
  auto w = std::make_shared<Eigen::MatrixXd>(result.w);
  transform = NonrigidTransform{ source_cloud_, w, params_.beta_ };
}

}  // namespace chair_manipulation
