#include "chair_manipulation_grasp_detection_advanced/grasp_sampler.h"
#include "chair_manipulation_grasp_detection_advanced/utils.h"
#include "chair_manipulation_grasp_detection_advanced/exception.h"
#include "chair_manipulation_grasp_detection_advanced/transform.h"
#include "chair_manipulation_grasp_detection_advanced/stopwatch.h"
#include <geometric_shapes/shapes.h>

namespace chair_manipulation
{
void GraspSamplerParameters::load(ros::NodeHandle& nh)
{
  max_antipodal_normal_angle_ = nh.param<double>("max_antipodal_normal_angle", 0.1);
  max_antipodal_position_angle_ = nh.param<double>("max_antipodal_position_angle", 0.1);
  max_equator_normal_angle_ = nh.param<double>("max_equator_normal_angle", 0.1);
  gripper_pad_distance_ = nh.param<double>("gripper_pad_distance", 0.1);
  gripper_pad_length_ = nh.param<double>("gripper_pad_length", 0.2);
}

void GraspSampler::sampleGraspHypotheses(const Model& model, std::size_t sample_trials,
                                         std::vector<GraspHypothesis>& hypotheses)
{
  Stopwatch stopwatch_contacts;
  const auto& mesh = model.getMesh();
  const auto& point_cloud = model.getPointCloud();

  gripper_->addCollisionObject(mesh);

  // Add a ground plane which is the x-y-plane offset by the minimum z-coordinate
  auto ground_plane = std::make_shared<shapes::Plane>(0., 0., 1., 0.);
  Eigen::Isometry3d ground_plane_pose = Eigen::Translation3d{ model.getMin() } * Eigen::Isometry3d::Identity();
  gripper_->addCollisionObject(ground_plane, ground_plane_pose);

  ROS_DEBUG_STREAM_NAMED("grasp_sampler", "Start sampling grasp hypotheses.");
  for (std::size_t s = 0; s < sample_trials; s++)
  {
    ROS_DEBUG_STREAM_NAMED("grasp_sampler", "");
    ROS_DEBUG_STREAM_NAMED("grasp_sampler", "=== Sample " << s << "/" << sample_trials << " ===");
    ROS_DEBUG_STREAM_NAMED("grasp_sampler", "");

    Eigen::Isometry3d grasp_pose;
    bool success = sampleGraspPose(point_cloud, grasp_pose);
    ROS_DEBUG_STREAM_NAMED("grasp_sampler", "Sampled grasp pose [" << utils::poseToStr(grasp_pose) << "]");
    if (!success)
    {
      ROS_DEBUG_STREAM_NAMED("grasp_sampler", "Failed to sample grasp pose.");
      continue;
    }

    gripper_->setTcpPose(grasp_pose);
    gripper_->setStateOpen();
    GraspHypothesis hypothesis;
    stopwatch_contacts.start();
    success = gripper_->grasp(hypothesis.contacts_);
    stopwatch_contacts.stop();
    ROS_DEBUG_STREAM_NAMED("grasp_sampler", "Computed grasp contacts.");
    ROS_DEBUG_STREAM_NAMED("grasp_sampler", "It took " << stopwatch_contacts.elapsedSeconds() << "s.");
    if (!success)
    {
      ROS_DEBUG_STREAM_NAMED("grasp_sampler", "Reject sampled pose because it results in a collision.");
      continue;
    }

    hypothesis.pose_ = grasp_pose;
    hypotheses.push_back(hypothesis);
    ROS_DEBUG_STREAM_NAMED("grasp_sampler", "Added sampled grasp hypothesis.");
  }

  gripper_->clearCollisionObjects();

  ROS_DEBUG_STREAM_NAMED("grasp_sampler", "Finished sampling grasp hypotheses.");
  ROS_DEBUG_STREAM_NAMED("grasp_sampler", "Found " << hypotheses.size() << " grasp hypotheses.");
}

void GraspSampler::sampleGraspHypothesesFromPrior(const Model& model, const std::vector<MultiArmGrasp>& prior_grasps,
                                                  std::size_t sample_trials, double sample_radius,
                                                  std::vector<GraspHypothesis>& hypotheses)
{
  Stopwatch stopwatch_contacts;

  const auto& mesh = model.getMesh();
  const auto& point_cloud = model.getPointCloud();

  gripper_->addCollisionObject(mesh);

  // Add a ground plane which is the x-y-plane offset by the minimum z-coordinate
  auto ground_plane = std::make_shared<shapes::Plane>(0., 0., 1., 0.);
  Eigen::Isometry3d ground_plane_pose = Eigen::Translation3d{ model.getMin() } * Eigen::Isometry3d::Identity();
  gripper_->addCollisionObject(ground_plane, ground_plane_pose);

  std::uniform_int_distribution<std::size_t> prior_index_distribution(0, prior_grasps.size() - 1);
  search_method_.setInputCloud(point_cloud);

  for (std::size_t s = 0; s < sample_trials; s++)
  {
    ROS_DEBUG_STREAM_NAMED("grasp_sampler", "");
    ROS_DEBUG_STREAM_NAMED("grasp_sampler", "=== Sample " << s << "/" << sample_trials << " ===");
    ROS_DEBUG_STREAM_NAMED("grasp_sampler", "");

    // Choose a random prior grasp
    auto prior_index = prior_index_distribution(random_generator_);
    const auto& prior_grasp = prior_grasps[prior_index];

    // For each pose of that randomly chosen grasp, randomly choose a point that's inside
    // the given radius and use that to find the grasp hypothesis
    for (const auto& pose : prior_grasp.poses_)
    {
      Eigen::Vector3d position = pose.translation();
      pcl::PointXYZ point(position.x(), position.y(), position.z());
      std::vector<int> indices;
      std::vector<float> sqr_distances;
      search_method_.radiusSearchT(point, sample_radius, indices, sqr_distances);
      if (indices.empty())
      {
        ROS_WARN_STREAM_NAMED("grasp_sampler",
                              "Failed to find nearby points for pose [" << utils::poseToStr(pose) << "]");
        continue;
      }

      std::uniform_int_distribution<std::size_t> nearest_index_distribution(0, indices.size() - 1);
      auto nearby_index = nearest_index_distribution(random_generator_);
      const auto& nearby_point = (*point_cloud)[indices[nearby_index]];
      ROS_DEBUG_STREAM_NAMED("grasp_sampler", "Considering nearby point ["
                                                  << utils::vectorToStr(nearby_point.getVector3fMap().cast<double>())
                                                  << "]");

      Eigen::Isometry3d grasp_pose;
      bool success = findGraspPoseAt(point_cloud, nearby_point, grasp_pose);
      ROS_DEBUG_STREAM_NAMED("grasp_sampler", "Sampled grasp pose [" << utils::poseToStr(grasp_pose) << "]");
      if (!success)
      {
        ROS_WARN_STREAM_NAMED("grasp_sampler", "Failed to sample grasp pose.");
        continue;
      }

      gripper_->setTcpPose(grasp_pose);
      gripper_->setStateOpen();
      GraspHypothesis hypothesis;
      stopwatch_contacts.start();
      success = gripper_->grasp(hypothesis.contacts_);
      stopwatch_contacts.stop();
      ROS_DEBUG_STREAM_NAMED("grasp_sampler", "Computed grasp contacts.");
      ROS_DEBUG_STREAM_NAMED("grasp_sampler", "It took " << stopwatch_contacts.elapsedSeconds() << "s.");
      if (!success)
      {
        ROS_DEBUG_STREAM_NAMED("grasp_sampler", "Reject sampled pose because it results in a collision.");
        continue;
      }

      hypothesis.pose_ = grasp_pose;
      hypotheses.push_back(hypothesis);
      ROS_DEBUG_STREAM_NAMED("grasp_sampler", "Added sampled grasp hypothesis.");
    }
  }

  gripper_->clearCollisionObjects();

  ROS_DEBUG_STREAM_NAMED("grasp_sampler", "Finished sampling grasp hypotheses.");
  ROS_DEBUG_STREAM_NAMED("grasp_sampler", "Found " << hypotheses.size() << " grasp hypotheses.");
}

bool GraspSampler::sampleGraspPose(const PointCloudConstPtr& point_cloud, Eigen::Isometry3d& grasp_pose)
{
  // Uniformly sample random point from
  std::uniform_int_distribution<std::size_t> distribution(0, point_cloud->size() - 1);
  std::size_t i = distribution(random_generator_);
  auto point_i = (*point_cloud)[i];
  return findGraspPoseAt(point_cloud, point_i, grasp_pose);
}

bool GraspSampler::findGraspPoseAt(const PointCloudConstPtr& point_cloud, const PointT& reference_point,
                                   Eigen::Isometry3d& grasp_pose)
{
  // Get neighboring points in radius equal to the gripper pad distance
  std::vector<int> indices;
  std::vector<float> sqr_distances;
  search_method_.setInputCloud(point_cloud);
  search_method_.radiusSearch(reference_point, params_.gripper_pad_distance_, indices, sqr_distances);

  // Filter out the points where the antipodal normal and position angle is greater than the given threshold
  const auto filter_max_antipodal_angle = [&](int j) {
    auto antipodal_point = (*point_cloud)[j];
    return computeAntipodalNormalAngle(reference_point, antipodal_point) > params_.max_antipodal_normal_angle_ &&
           computeAntipodalPositionAngle(reference_point, antipodal_point) > params_.max_antipodal_position_angle_;
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
  search_method_.radiusSearch(center_point, params_.gripper_pad_length_, indices, sqr_distances);

  // Filter out the points where the equator normal angle is larger than the given threshold
  const auto filter_max_equator_angle = [&](int k) {
    auto equator_point = (*point_cloud)[k];
    return computeEquatorNormalAngle(reference_point, equator_point) > params_.max_equator_normal_angle_;
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
  Eigen::Vector3f y_axis = reference_point.getNormalVector3fMap();
  // Orthonormalization of the equator inverse normal gives the z-axis
  Eigen::Vector3f equator_normal_inverted = -equator_point.getNormalVector3fMap();
  Eigen::Vector3f z_axis = (equator_normal_inverted - utils::projection(y_axis, equator_normal_inverted)).normalized();

  // Finally, convert to double
  Eigen::Isometry3d rotation, translation;
  rotation = transform::fromYZAxes(y_axis.cast<double>(), z_axis.cast<double>());
  translation = Eigen::Translation3d{ center_position.cast<double>() };
  grasp_pose = translation * rotation;

  return true;
}

double GraspSampler::computeAntipodalNormalAngle(const PointT& reference_point, const PointT& antipodal_point) const
{
  return std::acos(-reference_point.getNormalVector3fMap().dot(antipodal_point.getNormalVector3fMap()));
}

double GraspSampler::computeAntipodalPositionAngle(const PointT& reference_point, const PointT& antipodal_point) const
{
  return std::acos(reference_point.getNormalVector3fMap().dot(
      (reference_point.getVector3fMap() - antipodal_point.getVector3fMap()).normalized()));
}

double GraspSampler::computeEquatorNormalAngle(const PointT& reference_point, const PointT& equator_point) const
{
  return std::abs(std::acos(reference_point.getNormalVector3fMap().dot(equator_point.getNormalVector3fMap())) - M_PI_2);
}

double GraspSampler::computeAntipodalCost(const PointT& reference_point, const PointT& antipodal_point) const
{
  return computeAntipodalNormalAngle(reference_point, antipodal_point) +
         computeAntipodalPositionAngle(reference_point, antipodal_point);
}

double GraspSampler::computeEquatorCost(const PointT& reference_point, const PointT& equator_point) const
{
  // Normalize angle from [0, pi/2] to [0, 1].
  // Normalize distance from [0, gripper_pad_length] to [0, 1].
  const double sqr_gripper_pad_length = params_.gripper_pad_length_ * params_.gripper_pad_length_;
  auto distance = (reference_point.getVector3fMap() - equator_point.getVector3fMap()).norm();
  return computeEquatorNormalAngle(reference_point, equator_point) / M_PI_2 + distance / sqr_gripper_pad_length;
}

}  // namespace chair_manipulation