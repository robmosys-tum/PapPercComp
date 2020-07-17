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
  auto result = nonrigid_.run(*source_cloud_, *target_cloud_);
  aligned_cloud = result.points;
  transform = NonrigidTransform{result.points, result.w, params_.beta_};
}

}  // namespace chair_manipulation
