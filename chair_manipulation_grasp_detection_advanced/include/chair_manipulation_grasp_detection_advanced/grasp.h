#ifndef CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_GRASP_H
#define CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_GRASP_H

#include <geometry_msgs/Pose.h>

namespace chair_manipulation
{
struct Grasp
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Grasp(const Eigen::Isometry3d& pose, double quality) : pose_(pose), quality_(quality)
  {
  }

  /**
   * Pose of tool center point (tcp) where
   *  - the Y-axis is parallel to the gripper pads and points towards the contact point
   *  - the Z-axis is parallel to the gripper and is perpendicular to the Y-axis
   *  - the X-axis is perpendicular to the Y- and X-axis (and therefore perpendicular to the gripper pads)
   */
  Eigen::Isometry3d pose_;

  /**
   * The grasp quality
   */
  double quality_;
};

}  // namespace chair_manipulation

#endif  // CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_GRASP_H
