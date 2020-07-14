#ifndef CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_GRASP_SYNTHESIZER_H
#define CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_GRASP_SYNTHESIZER_H

#include "grasp_hypothesis.h"
#include "model.h"
#include "multi_arm_grasp.h"
#include "wrench.h"
#include <ros/ros.h>

#include <utility>

namespace chair_manipulation
{
struct GraspQualityWeights
{
  void load(ros::NodeHandle& nh);

  double epsilon1_;
  double v1_;
  double distance_;
  double reachability_;
};

struct GraspSynthesizerParameters
{
  void load(ros::NodeHandle& nh);

  int num_arms_;
  double friction_coefficient_;
  int num_friction_edges_;
  double max_arm_radius_;
  std::string world_frame_;
  std::vector<std::string> arm_base_frames_;
};

class GraspSynthesizer
{
public:
  GraspSynthesizer(GraspSynthesizerParameters params, GraspQualityWeights weights);

  void synthesize(const std::vector<GraspHypothesis>& hypotheses, const Model& model, std::size_t max_num_grasps,
                  std::vector<MultiArmGrasp>& synthesized_grasps) const;

private:
  using GraspCandidate = std::vector<const GraspHypothesis*>;

  GraspSynthesizerParameters params_;
  GraspQualityWeights weights_;
  std::vector<Eigen::Isometry3d> arm_base_poses_;

  void generateGraspCandidates(const std::vector<GraspHypothesis>& hypotheses, std::vector<GraspCandidate>& candidates,
                               const GraspCandidate& curr_candidate = GraspCandidate{}, std::size_t arm_index = 0,
                               std::size_t hypothesis_index = 0) const;

  WrenchSpace wrenchSpaceFromHypotheses(const GraspCandidate& candidate, const Model& model) const;

  double computeNormalizedPairwiseDistanceSum(const GraspCandidate& candidate, const Model& model) const;

  double computeReachability(const GraspCandidate& candidate) const;
};

using GraspSynthesizerPtr = std::shared_ptr<GraspSynthesizer>;
using GraspSynthesizerConstPtr = std::shared_ptr<const GraspSynthesizer>;

}  // namespace chair_manipulation

#endif  // CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_GRASP_SYNTHESIZER_H
