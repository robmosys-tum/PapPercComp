#ifndef CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_GRASP_SAMPLER_H
#define CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_GRASP_SAMPLER_H

#include "model.h"
#include "grasp.h"
#include "ros/ros.h"
#include <random>
#include <pcl/search/kdtree.h>
#include <geometry_msgs/Pose.h>

namespace chair_manipulation
{
struct GraspSamplerParameters
{
  void load(ros::NodeHandle& nh);

  double grasp_quality_threshold_;
  double max_antipodal_normal_angle_;
  double max_antipodal_position_angle_;
  double max_equator_normal_angle_;
  double gripper_pad_distance_;
  double gripper_pad_length_;
};

class GraspSampler
{
public:
  using PointCloud = Model::PointCloud;
  using PointT = PointCloud::PointType;

  explicit GraspSampler(const GraspSamplerParameters& params) : params_(params)
  {
  }

  void sampleGrasps(const Model& model, std::size_t sample_trials, std::vector<Grasp>& grasps);

  bool sampleGraspPose(const PointCloud::ConstPtr& point_cloud, geometry_msgs::Pose& grasp_pose);

  bool findGraspPoseAt(const PointCloud::ConstPtr& point_cloud, const PointCloud::PointType& point,
                       geometry_msgs::Pose& grasp_pose);

private:
  GraspSamplerParameters params_;

  std::default_random_engine random_generator_;

  using SearchMethod = pcl::search::KdTree<PointCloud::PointType>;
  SearchMethod search_method_;

  /**
   * Angle between the normal of the reference point and the normal of the antipodal point
   */
  double computeAntipodalNormalAngle(const PointT& reference_point, const PointT& antipodal_point);

  /**
   * Angle between the normal of the reference point and the connecting line from the antipodal point
   * to the reference point
   */
  double computeAntipodalPositionAngle(const PointT& reference_point, const PointT& antipodal_point);

  /**
   * Angle between the normal of reference point and the normal of the equator point minus pi/2
   */
  double computeEquatorNormalAngle(const PointT& reference_point, const PointT& equator_point);

  /**
   * The cost associated with some antipodal point
   */
  double computeAntipodalCost(const PointT& reference_point, const PointT& antipodal_point);

  /**
   * The cost associated with some equator point
   */
  double computeEquatorCost(const PointT& reference_point, const PointT& equator_point);
};

}  // namespace chair_manipulation

#endif  // CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_GRASP_SAMPLER_H
