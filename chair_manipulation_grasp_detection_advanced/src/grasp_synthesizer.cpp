#include "chair_manipulation_grasp_detection_advanced/grasp_synthesizer.h"
#include "chair_manipulation_grasp_detection_advanced/exception.h"
#include "chair_manipulation_grasp_detection_advanced/stopwatch.h"
#include "chair_manipulation_grasp_detection_advanced/utils.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

namespace chair_manipulation
{
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

GraspSynthesizer::GraspSynthesizer(GraspSynthesizerParameters params, GraspQualityWeights weights)
  : params_(std::move(params)), weights_(std::move(weights))
{
  if (weights_.reachability_ != 0.)
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
}

void GraspSynthesizer::synthesize(const std::vector<GraspHypothesis>& hypotheses, const Model& model,
                                  std::size_t max_num_grasps, std::vector<MultiArmGrasp>& synthesized_grasps) const
{
  ROS_DEBUG_STREAM_NAMED("grasp_synthesizer", "Synthesizing " << hypotheses.size() << " grasp hypotheses.");
  Stopwatch stopwatch_total, stopwatch_candidate, stopwatch_wrench;
  stopwatch_total.start();

  if (hypotheses.size() < params_.num_arms_)
    throw exception::IllegalArgument{ "The number of hypotheses must be at least as large as the number of arms." };

  std::vector<GraspCandidate> candidates;
  generateGraspCandidates(hypotheses, candidates);
  ROS_DEBUG_STREAM_NAMED("grasp_synthesizer", "Generated " << candidates.size() << " grasp candidates.");

  double weight_sum = weights_.epsilon1_ + weights_.v1_ + weights_.distance_ + weights_.reachability_;
  if (weight_sum == 0)
    throw exception::Parameter{ "The sum of the weights must not be zero." };

  for (std::size_t i = 0; i < candidates.size(); i++)
  {
    ROS_DEBUG_STREAM_NAMED("grasp_synthesizer", "");
    ROS_DEBUG_STREAM_NAMED("grasp_synthesizer", "=== Candidate " << (i + 1) << "/" << candidates.size() << " ===");
    ROS_DEBUG_STREAM_NAMED("grasp_synthesizer", "");

    stopwatch_candidate.start();

    const auto& candidate = candidates[i];
    double epsilon1 = 0;
    double v1 = 0;
    double distance = 0;
    double reachability = 0;

    for (std::size_t j = 0; j < candidate.size(); j++)
      ROS_DEBUG_STREAM_NAMED("grasp_synthesizer",
                             "pose of arm " << j << ": [" << utils::poseToStr(candidate[j]->pose_) << "]");

    if (weights_.epsilon1_ != 0. || weights_.v1_ != 0.)
    {
      stopwatch_wrench.start();
      auto wrench_space = wrenchSpaceFromHypotheses(candidate, model);
      stopwatch_wrench.stop();
      ROS_DEBUG_STREAM_NAMED("grasp_synthesizer", "Computed wrench space.");
      ROS_DEBUG_STREAM_NAMED("grasp_synthesizer", "It took " << stopwatch_wrench.elapsedSeconds() << "s.");
      if (!wrench_space.isForceClosure())
      {
        ROS_DEBUG_STREAM_NAMED("grasp_synthesizer", "No force closure - omit this candidate.");
        continue;
      }
      epsilon1 = wrench_space.getEpsilon1Quality();
      v1 = wrench_space.getV1Quality();
    }

    if (weights_.distance_ != 0.)
      distance = computeNormalizedPairwiseDistanceSum(candidate, model);

    if (weights_.reachability_ != 0.)
      reachability = computeReachability(candidate);

    double epsilon1_weighted = weights_.epsilon1_ * epsilon1;
    double v1_weighted = weights_.v1_ * v1;
    double distance_weighted = weights_.distance_ * distance;
    double reachability_weighted = weights_.reachability_ * reachability;

    double grasp_quality = epsilon1_weighted + v1_weighted + distance_weighted + reachability_weighted;
    grasp_quality /= weight_sum;

    MultiArmGrasp grasp;
    grasp.quality_ = grasp_quality;
    for (const auto& hypothesis : candidate)
      grasp.poses_.push_back(hypothesis->pose_);

    synthesized_grasps.push_back(grasp);

    stopwatch_candidate.stop();

    ROS_DEBUG_STREAM_NAMED("grasp_synthesizer", "epsilon1: " << epsilon1);
    ROS_DEBUG_STREAM_NAMED("grasp_synthesizer", "v1: " << v1);
    ROS_DEBUG_STREAM_NAMED("grasp_synthesizer", "distance: " << distance);
    ROS_DEBUG_STREAM_NAMED("grasp_synthesizer", "reachability: " << reachability);

    ROS_DEBUG_STREAM_NAMED("grasp_synthesizer", "epsilon1 weighted: " << epsilon1_weighted);
    ROS_DEBUG_STREAM_NAMED("grasp_synthesizer", "v1 weighted: " << v1_weighted);
    ROS_DEBUG_STREAM_NAMED("grasp_synthesizer", "distance weighted: " << distance_weighted);
    ROS_DEBUG_STREAM_NAMED("grasp_synthesizer", "reachability weighted: " << reachability_weighted);

    ROS_DEBUG_STREAM_NAMED("grasp_synthesizer", "grasp quality: " << grasp_quality);

    ROS_DEBUG_STREAM_NAMED("grasp_synthesizer",
                           "Processing current candidate took " << stopwatch_candidate.elapsedSeconds() << "s.");
  }

  ROS_DEBUG_STREAM_NAMED("grasp_synthesizer", "Sorting " << synthesized_grasps.size()
                                                         << " remaining synthesized grasps by grasp quality in "
                                                            "descending order.");

  std::sort(synthesized_grasps.begin(), synthesized_grasps.end(),
            [&](const auto& grasp1, const auto& grasp2) { return grasp1.quality_ > grasp2.quality_; });

  if (synthesized_grasps.size() > max_num_grasps)
  {
    ROS_DEBUG_STREAM_NAMED("grasp_synthesizer",
                           "Need to clip the number of returned grasps to " << max_num_grasps << ".");
    synthesized_grasps.resize(max_num_grasps);
  }

  stopwatch_total.stop();
  ROS_DEBUG_STREAM_NAMED("grasp_synthesizer",
                         "Synthesizing took " << stopwatch_total.elapsedSeconds() << "s in total.");
}  // namespace chair_manipulation

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

  ROS_DEBUG_STREAM_NAMED("grasp_synthesizer", "contacts: [" << utils::contactsToStr(contacts) << "]");

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

}  // namespace chair_manipulation