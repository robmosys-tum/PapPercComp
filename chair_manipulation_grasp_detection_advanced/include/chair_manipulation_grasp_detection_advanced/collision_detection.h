#ifndef CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_COLLISION_DETECTION_H
#define CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_COLLISION_DETECTION_H

#include <ros/ros.h>
#include <moveit/planning_scene/planning_scene.h>
#include "model.h"

namespace chair_manipulation
{
class CollisionDetection
{
public:
  explicit CollisionDetection(ros::NodeHandle& nh);

  bool checkCollision(const Model& model, const geometry_msgs::Pose &grasp_pose);

private:
  std::string world_frame_;
  std::string gripper_root_frame_;
  std::string tcp_frame_;
  std::unique_ptr<planning_scene::PlanningScene> scene_;
};

}  // namespace chair_manipulation

#endif  // CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_COLLISION_DETECTION_H
