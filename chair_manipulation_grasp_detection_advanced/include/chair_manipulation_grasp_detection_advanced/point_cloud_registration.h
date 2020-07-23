#ifndef CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_POINT_CLOUD_REGISTRATION_H
#define CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_POINT_CLOUD_REGISTRATION_H

#include "nonrigid_transform.h"
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <cpd/nonrigid.hpp>

namespace chair_manipulation
{
struct PointCloudRegistrationParameters
{
  void load(ros::NodeHandle& nh);

  double lambda_;
  double beta_;
  int max_iterations_;
  double pre_voxel_grid_leaf_size_;
  double post_voxel_grid_leaf_size_;
};

class PointCloudRegistration
{
public:
  using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
  using PointCloudPtr = PointCloud::Ptr;
  using PointNormalCloud = pcl::PointCloud<pcl::PointNormal>;
  using PointNormalCloudPtr = PointNormalCloud::Ptr;
  using PointNormalCloudConstPtr = PointNormalCloud::ConstPtr;
  using EigenCloud = Eigen::MatrixXd;

  explicit PointCloudRegistration(PointCloudRegistrationParameters params);

  void setInputSource(const PointNormalCloudConstPtr& source_cloud);

  void setInputTarget(const PointNormalCloudConstPtr& target_cloud);

  void align(PointNormalCloud& aligned_cloud, NonrigidTransform& transform);

private:
  PointCloudRegistrationParameters params_;
  PointNormalCloudConstPtr source_cloud_;
  PointNormalCloudConstPtr target_cloud_;
  pcl::VoxelGrid<pcl::PointNormal> pre_voxel_filter_;
  pcl::VoxelGrid<pcl::PointNormal> post_voxel_filter_;
};

}

#endif  // CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_POINT_CLOUD_REGISTRATION_H
