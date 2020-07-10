#ifndef CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_GRIPPER_H
#define CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_GRIPPER_H

#include <ros/ros.h>
#include <moveit/planning_scene/planning_scene.h>
#include "model.h"
#include "contact.h"

namespace chair_manipulation
{
struct FingerGroup
{
  std::string group_name_;
  std::string open_group_state_name_;
  std::string closed_group_state_name_;
};

struct GripperParameters
{
  void load(ros::NodeHandle& nh);

  std::string base_frame_;
  std::string tcp_frame_;
  std::string gripper_description_;
  std::string gripper_semantic_description_;
  std::vector<FingerGroup> finger_groups_;
};

class Gripper
{
public:
  using GripperParametersConstPtr = std::shared_ptr<const GripperParameters>;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static constexpr double CONTACT_THRESHOLD = 0.001;

  explicit Gripper(const GripperParametersConstPtr& params);

  void setTcpPose(const Eigen::Isometry3d& tcp_pose);

  void addCollisionObject(const shapes::ShapeConstPtr& object,
                          const Eigen::Isometry3d& pose = Eigen::Isometry3d::Identity());

  void clearCollisionObjects();

  void setStateOpen();

  bool isColliding();

  bool grasp(std::vector<Contact>& contacts);

  moveit::core::RobotStateConstPtr getRobotState() const
  {
    return robot_state_;
  }

  const Eigen::Isometry3d& getBasePose() const
  {
    return base_pose_;
  }

private:
  GripperParametersConstPtr params_;

  moveit::core::RobotModelPtr robot_model_;
  moveit::core::RobotStatePtr robot_state_;
  collision_detection::CollisionEnvPtr cenv_;
  collision_detection::AllowedCollisionMatrixPtr acm_;

  Eigen::Isometry3d tcp_to_base_;
  Eigen::Isometry3d base_pose_;

  using VariableValues = std::map<std::string, double>;
  std::map<std::string, VariableValues> open_group_state_values_;
  std::map<std::string, VariableValues> closed_group_state_values_;

  unsigned long long running_id_ = 0;

  void loadGroupStates();

  bool moveToContacts(const std::map<std::string, double>& open_values,
                         const std::map<std::string, double>& closed_values, const std::string& group_name,
                         std::vector<Contact>& contacts);

  void updateState();
};

}  // namespace chair_manipulation

#endif  // CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_GRIPPER_H
