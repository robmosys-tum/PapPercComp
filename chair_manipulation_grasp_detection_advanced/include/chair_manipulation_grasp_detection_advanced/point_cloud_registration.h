#ifndef CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_POINT_CLOUD_REGISTRATION_H
#define CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_POINT_CLOUD_REGISTRATION_H

#include "nonrigid_transform.h"
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cpd/nonrigid.hpp>

namespace chair_manipulation
{
struct PointCloudRegistrationParameters
{
  void load(ros::NodeHandle& nh);

  double lambda_;
  double beta_;
  int max_iterations_;
};

class PointCloudRegistration
{
public:
  using PointCloud = Eigen::MatrixXd;
  using PointCloudPtr = std::shared_ptr<PointCloud>;
  using PointCloudConstPtr = std::shared_ptr<const PointCloud>;

  explicit PointCloudRegistration(PointCloudRegistrationParameters params);

  void setInputSource(const PointCloudConstPtr& source_cloud);

  void setInputTarget(const PointCloudConstPtr& target_cloud);

  void align(PointCloud& aligned_cloud, NonrigidTransform& transform);

private:
  PointCloudRegistrationParameters params_;
  PointCloudConstPtr source_cloud_;
  PointCloudConstPtr target_cloud_;
};

}

#endif  // CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_POINT_CLOUD_REGISTRATION_H
