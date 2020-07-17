#ifndef CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_GRASP_DETECTION_PIPELINE_H
#define CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_GRASP_DETECTION_PIPELINE_H

#include <ros/ros.h>

namespace chair_manipulation
{
struct GraspDetectionParameters
{
  void load(ros::NodeHandle& nh);

  int num_sample_trials_;
  std::vector<std::string> grasp_frames_;
};

void run_grasp_detection_pipeline();

}  // namespace chair_manipulation

#endif  // CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_GRASP_DETECTION_PIPELINE_H
