#include "chair_manipulation_grasp_detection_advanced/grasp_synthesizer.h"
#include "chair_manipulation_grasp_detection_advanced/exception.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

namespace chair_manipulation
{
GraspSynthesizer::GraspSynthesizer(GraspSynthesizerParameters params, GraspQualityWeights weights)
    : params_(std::move(params)), weights_(std::move(weights))
{
  // Get arm base poses (with respect to the world frame) using tf2
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);
  arm_base_poses_.resize(params.num_arms_);
  for (std::size_t i = 0; i < params_.num_arms_; i++)
  {
    const auto& arm_base_frame = params_.arm_base_frames_[i];
    geometry_msgs::TransformStamped msg;
    msg = tf_buffer.lookupTransform(params_.world_frame_, arm_base_frame, ros::Time{ 0 }, ros::Duration{ 3. });
    arm_base_poses_[i] = tf2::transformToEigen(msg);
  }
}

void GraspSynthesizer::synthesize(const std::vector<GraspHypothesis>& hypotheses, const Model& model,
                                  std::size_t max_num_grasps, std::vector<MultiArmGrasp>& synthesized_grasps) const
{
  if (hypotheses.size() < params_.num_arms_)
    throw exception::IllegalArgument{ "The number of hypotheses must be at least as large as the number of arms." };

  std::vector<GraspCandidate> candidates;
  generateGraspCandidates(hypotheses, candidates);

  double weight_sum = weights_.epsilon1_ + weights_.v1_ + weights_.distance_ + weights_.reachability_;
  if (weight_sum == 0)
    throw exception::Parameter{ "The sum of the weights must not be zero." };

  for (const auto& candidate : candidates)
  {
    double grasp_quality = 0;

    if (weights_.epsilon1_ != 0. || weights_.v1_ != 0.)
    {
      auto wrench_space = wrenchSpaceFromHypotheses(candidate, model);
      if (!wrench_space.isForceClosure())
        continue;
      grasp_quality += weights_.epsilon1_ * wrench_space.getEpsilon1Quality();
      grasp_quality += weights_.v1_ * wrench_space.getV1Quality();
    }

    if (weights_.distance_ != 0.)
    {
      double distance = computeNormalizedPairwiseDistanceSum(candidate, model);
      grasp_quality += weights_.distance_ * distance;
    }

    if (weights_.reachability_ != 0.)
    {
      double reachability = computeReachability(candidate);
      grasp_quality += reachability;
    }

    grasp_quality /= weight_sum;
    if (grasp_quality >= params_.grasp_quality_threshold_)
    {
      MultiArmGrasp grasp;
      grasp.quality_ = grasp_quality;
      for (const auto& hypothesis : candidate)
      {
        grasp.poses_.push_back(hypothesis->pose_);
      }
      synthesized_grasps.push_back(grasp);
    }
  }

  std::sort(synthesized_grasps.begin(), synthesized_grasps.end(),
            [&](const auto& grasp1, const auto& grasp2) { return grasp1.quality_ < grasp2.quality_; });

  if (synthesized_grasps.size() > max_num_grasps)
    synthesized_grasps.resize(max_num_grasps);
}

void GraspSynthesizer::generateGraspCandidates(const std::vector<GraspHypothesis>& hypotheses,
                                               std::vector<GraspCandidate>& candidates,
                                               const GraspCandidate& curr_candidate, std::size_t arm_index,
                                               std::size_t hypothesis_index) const
{
  for (std::size_t j = hypothesis_index; j < hypotheses.size() - params_.num_arms_ + arm_index + 1; j++)
  {
    auto new_candidate = curr_candidate;
    auto hypothesis = &hypotheses[j];
    new_candidate.push_back(hypothesis);
    if (arm_index == params_.num_arms_ - 1)
      candidates.push_back(new_candidate);
    else
      generateGraspCandidates(hypotheses, candidates, new_candidate, arm_index + 1, j + 1);
  }
}

WrenchSpace GraspSynthesizer::wrenchSpaceFromHypotheses(const GraspCandidate& candidate, const Model& model) const
{
  std::size_t num_contacts = 0;
  for (const auto& hypothesis : candidate)
    num_contacts += hypothesis->contacts_.size();

  std::vector<Contact> contacts;
  contacts.resize(num_contacts);

  std::size_t i = 0;
  for (const auto& hypothesis : candidate)
  {
    for (const auto& contact : hypothesis->contacts_)
      contacts[i++] = contact;
  }

  return WrenchSpace(contacts, model, params_.friction_coefficient_, params_.num_friction_edges_);
}

double GraspSynthesizer::computeNormalizedPairwiseDistanceSum(const GraspSynthesizer::GraspCandidate& candidate,
                                                              const Model& model) const
{
  double max_distance = (model.getMax() - model.getMin()).norm();
  double sum = 0.;
  std::size_t n = 0;
  for (std::size_t i = 0; i < candidate.size(); i++)
  {
    for (std::size_t j = 0; j < candidate.size(); j++)
    {
      sum += (candidate[i]->pose_.translation() - candidate[j]->pose_.translation()).norm() / max_distance;
      n++;
    }
  }
  return sum / n;
}

double GraspSynthesizer::computeReachability(const GraspCandidate& candidate) const
{
  double reachability = 0;

  // We want every grasp to be assigned to a unique arm so this tells us whether grasp i
  // is already assigned to arm i where we assign grasp i to the arm that is closest to it.
  std::vector<bool> assigned(params_.num_arms_, false);

  for (const auto& grasp : candidate)
  {
    double min_distance = std::numeric_limits<double>::max();
    std::size_t arm_index = 0;

    for (std::size_t i = 0; i < params_.num_arms_; i++)
    {
      const auto& arm_base_pose = arm_base_poses_[i];
      double distance = (grasp->pose_.translation() - arm_base_pose.translation()).norm();
      if (distance < min_distance)
      {
        min_distance = distance;
        arm_index = i;
      }
    }

    // If the arm is already assigned we reject the grasp by setting output to negative infinity
    if (assigned[arm_index])
      return -std::numeric_limits<double>::infinity();

    // The closer the grasp is to the base, the higher the computed score, normalized to [0, 1].
    reachability += 1. - (std::min(min_distance, params_.max_arm_radius_) / params_.max_arm_radius_);
  }
  return reachability / params_.num_arms_;
}

void GraspQualityWeights::load(ros::NodeHandle& nh)
{
  epsilon1_ = nh.param<double>("epsilon1", 1.0);
  v1_ = nh.param<double>("v1", 1.0);
  distance_ = nh.param<double>("distance", 1.0);
  reachability_ = nh.param<double>("reachability", 1.0);
}

void GraspSynthesizerParameters::load(ros::NodeHandle& nh)
{
  num_arms_ = nh.param<int>("num_arms", 2);
  grasp_quality_threshold_ = nh.param<double>("grasp_quality_threshold", 0.7);
  friction_coefficient_ = nh.param<double>("friction_coefficient", 0.8);
  num_friction_edges_ = nh.param<int>("num_friction_edges", 8);
  max_arm_radius_ = nh.param<double>("max_arm_radius", 1.0);
  world_frame_ = nh.param<std::string>("world_frame", "world");

  XmlRpc::XmlRpcValue arm_base_frames_array;
  if (!nh.getParam("arm_base_frames", arm_base_frames_array) ||
      arm_base_frames_array.getType() != XmlRpc::XmlRpcValue::TypeArray || arm_base_frames_array.size() == 0)
    throw exception::Parameter{ "Failed to load parameter 'arm_base_frames'." };

  int num_base_frames = arm_base_frames_array.size();
  arm_base_frames_.resize(num_base_frames);
  for (int i = 0; i < num_base_frames; i++)
  {
    XmlRpc::XmlRpcValue item = arm_base_frames_array[i];
    arm_base_frames_[i] = (std::string)item;
  }
}

}  // namespace chair_manipulation