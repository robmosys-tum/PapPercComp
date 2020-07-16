#ifndef CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_POINT_CLOUD_REGISTRATION_H
#define CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_POINT_CLOUD_REGISTRATION_H

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
  using PointT = pcl::PointNormal;
  using PointCloud = pcl::PointCloud<PointT>;
  using PointCloudPtr = PointCloud::Ptr;
  using PointCloudConstPtr = PointCloud::ConstPtr;

  explicit PointCloudRegistration(PointCloudRegistrationParameters params);

  void setInputSource(const PointCloudConstPtr& source_cloud);

  void setInputTarget(const PointCloudConstPtr& target_cloud);

  void align(PointCloud& aligned_cloud);

private:
  PointCloudRegistrationParameters params_;
  PointCloudConstPtr source_cloud_;
  PointCloudConstPtr target_cloud_;
  cpd::Nonrigid nonrigid_;
};

}

#endif  // CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_POINT_CLOUD_REGISTRATION_H
