#include "chair_manipulation_grasp_detection_advanced/grasp_sampler.h"
#include "chair_manipulation_grasp_detection_advanced/utils.h"
#include "chair_manipulation_grasp_detection_advanced/exception.h"

namespace chair_manipulation
{
GraspSampler::GraspSampler(ros::NodeHandle& nh)
{
  grasp_quality_threshold_ = nh.param<double>("grasp_quality_threshold", 0.7);
  max_antipodal_normal_angle_ = nh.param<double>("max_antipodal_normal_angle", 0.1);
  max_antipodal_position_angle_ = nh.param<double>("max_antipodal_position_angle", 0.1);
  max_equator_normal_angle_ = nh.param<double>("max_equator_normal_angle", 0.1);
  gripper_pad_distance_ = nh.param<double>("gripper_pad_distance", 0.1);
  gripper_pad_length_ = nh.param<double>("gripper_pad_length", 0.2);
}

void GraspSampler::sampleGrasps(const Model& model, std::size_t sample_trials, std::vector<Grasp>& grasps)
{
  const auto& mesh = model.mesh_;
  const auto& point_cloud = model.point_cloud_;
  for (std::size_t s = 0; s < sample_trials; s++)
  {
    geometry_msgs::Pose grasp_pose;
    if (!sampleGraspPose(point_cloud, grasp_pose))
      continue;
  }
}

bool GraspSampler::sampleGraspPose(const pcl::PointCloud<pcl::PointNormal>::ConstPtr& point_cloud,
                                   geometry_msgs::Pose& grasp_pose)
{
  // Uniformly sample random point from
  std::uniform_int_distribution<std::size_t> distribution(0, point_cloud->size() - 1);
  std::size_t i = distribution(random_generator_);
  auto point_i = (*point_cloud)[i];
  return findGraspPoseAt(point_cloud, point_i, grasp_pose);
}

bool GraspSampler::findGraspPoseAt(const pcl::PointCloud<pcl::PointNormal>::ConstPtr& point_cloud,
                                   const pcl::PointNormal& reference_point, geometry_msgs::Pose& grasp_pose)
{
  // Get neighboring points in radius equal to the gripper pad distance
  std::vector<int> indices;
  std::vector<float> sqr_distances;
  search_method_.setInputCloud(point_cloud);
  search_method_.radiusSearch(reference_point, gripper_pad_distance_, indices, sqr_distances);

  // Filter out the points where the antipodal normal and position angle is greater than the given threshold
  const auto filter_max_antipodal_angle = [&](int j) {
    auto antipodal_point = (*point_cloud)[j];
    return computeAntipodalNormalAngle(reference_point, antipodal_point) > max_antipodal_normal_angle_ &&
           computeAntipodalPositionAngle(reference_point, antipodal_point) > max_antipodal_position_angle_;
  };
  indices.erase(std::remove_if(indices.begin(), indices.end(), filter_max_antipodal_angle), indices.end());
  if (indices.empty())
    return false;

  // From the filtered indices, find the point that minimizes the sum of the antipodal angles.
  const auto antipodal_cost_comp = [&](int j1, int j2) {
    auto antipodal_point1 = (*point_cloud)[j1];
    auto antipodal_point2 = (*point_cloud)[j2];
    return computeAntipodalCost(reference_point, antipodal_point1) <
           computeAntipodalCost(reference_point, antipodal_point2);
  };
  auto it = std::min_element(indices.begin(), indices.end(), antipodal_cost_comp);

  auto antipodal_index = *it;
  auto antipodal_point = (*point_cloud)[antipodal_index];

  // Position of the tcp frame
  Eigen::Vector3f center_position = (reference_point.getVector3fMap() + antipodal_point.getVector3fMap()) / 2;
  auto center_point = reference_point;
  center_point.x = center_position.x();
  center_point.y = center_position.y();
  center_point.z = center_position.z();

  // Get neighboring points of the center point within the radius of the gripper pad length
  indices = std::vector<int>{};
  sqr_distances = std::vector<float>{};
  search_method_.radiusSearch(center_point, gripper_pad_length_, indices, sqr_distances);

  // Filter out the points where the equator normal angle is larger than the given threshold
  const auto filter_max_equator_angle = [&](int k) {
    auto equator_point = (*point_cloud)[k];
    return computeEquatorNormalAngle(reference_point, equator_point) > max_equator_normal_angle_;
  };
  indices.erase(std::remove_if(indices.begin(), indices.end(), filter_max_equator_angle), indices.end());
  if (indices.empty())
    return false;

  // From the filtered indices, find the point that minimizes the sum of the equator angle and the distance
  // between the point and the center
  const auto equator_cost_comp = [&](int k1, int k2) {
    auto equator_point1 = (*point_cloud)[k1];
    auto equator_point2 = (*point_cloud)[k2];
    return computeEquatorCost(reference_point, equator_point1) < computeEquatorCost(reference_point, equator_point2);
  };
  it = std::min_element(indices.begin(), indices.end(), equator_cost_comp);

  auto equator_index = *it;
  auto equator_point = (*point_cloud)[equator_index];

  // The y-axis points in the direction of the normal of the reference point
  Eigen::Vector3f y_direction = reference_point.getNormalVector3fMap();
  // Orthonormalization of the equator inverse normal gives the z-axis
  Eigen::Vector3f equator_normal_inverted = -equator_point.getNormalVector3fMap();
  Eigen::Vector3f z_direction =
      (equator_normal_inverted - utils::projection(y_direction, equator_normal_inverted)).normalized();
  Eigen::Quaternionf orientation;
  utils::directionsYZToQuaternion(y_direction, z_direction, orientation);

  // Output
  grasp_pose.position.x = center_position.x();
  grasp_pose.position.y = center_position.y();
  grasp_pose.position.z = center_position.z();
  grasp_pose.orientation.x = orientation.x();
  grasp_pose.orientation.y = orientation.y();
  grasp_pose.orientation.z = orientation.z();
  grasp_pose.orientation.w = orientation.w();

  return true;
}

double GraspSampler::computeAntipodalNormalAngle(const PointT& reference_point, const PointT& antipodal_point)
{
  return std::acos(-reference_point.getNormalVector3fMap().dot(antipodal_point.getNormalVector3fMap()));
}

double GraspSampler::computeAntipodalPositionAngle(const PointT& reference_point, const PointT& antipodal_point)
{
  return std::acos(reference_point.getNormalVector3fMap().dot(
      (reference_point.getVector3fMap() - antipodal_point.getVector3fMap()).normalized()));
}

double GraspSampler::computeEquatorNormalAngle(const PointT& reference_point, const PointT& equator_point)
{
  return std::abs(std::acos(reference_point.getNormalVector3fMap().dot(equator_point.getNormalVector3fMap())) - M_PI_2);
}

double GraspSampler::computeAntipodalCost(const PointT& reference_point, const PointT& antipodal_point)
{
  return computeAntipodalNormalAngle(reference_point, antipodal_point) +
         computeAntipodalPositionAngle(reference_point, antipodal_point);
}

double GraspSampler::computeEquatorCost(const PointT& reference_point, const PointT& equator_point)
{
  // Normalize angle from [0, pi/2] to [0, 1].
  // Normalize distance from [0, gripper_pad_length] to [0, 1].
  const double sqr_gripper_pad_length = gripper_pad_length_ * gripper_pad_length_;
  auto distance = (reference_point.getVector3fMap() - equator_point.getVector3fMap()).norm();
  return computeEquatorNormalAngle(reference_point, equator_point) / M_PI_2 + distance / sqr_gripper_pad_length;
}

}  // namespace chair_manipulation