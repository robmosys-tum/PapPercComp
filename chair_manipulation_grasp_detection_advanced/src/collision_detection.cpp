#include "chair_manipulation_grasp_detection_advanced/collision_detection.h"
#include "chair_manipulation_grasp_detection_advanced/exception.h"
#include <urdf_parser/urdf_parser.h>

namespace chair_manipulation
{
// Convenient class to add a model to the scene and automatically remove it when the object of class gets destroyed
class SceneModelScope
{
public:
  SceneModelScope(planning_scene::PlanningScene& scene, const Model& model) : scene_(scene)
  {
    Eigen::Isometry3d id = Eigen::Isometry3d::Identity();
    scene.getWorldNonConst()->addToObject("model", model.mesh_, id);
  }

  ~SceneModelScope()
  {
    scene_.getWorldNonConst()->removeObject("model");
  }

private:
  planning_scene::PlanningScene& scene_;
};

CollisionDetection::CollisionDetection(ros::NodeHandle& nh)
{
  world_frame_ = nh.param<std::string>("world_frame", "world");;
  gripper_root_frame_ = nh.param<std::string>("gripper_base_frame", "robotiq_arg2f_140_base_link");
  tcp_frame_ = nh.param<std::string>("tcp_frame", "tcp");

  std::string gripper_description;
  if (!nh.getParam("gripper_description", gripper_description))
  {
    throw exception::Parameter{ "Parameter 'gripper_description' not found." };
  }

  std::string gripper_semantic_description;
  if (!nh.getParam("gripper_semantic_description", gripper_semantic_description))
  {
    throw exception::Parameter{ "Parameter 'gripper_semantic_description' not found." };
  }

  urdf::ModelInterfaceSharedPtr urdf_model = urdf::parseURDF(gripper_description);
  srdf::ModelSharedPtr srdf_model{ new srdf::Model };
  srdf_model->initString(*urdf_model, gripper_semantic_description);

  scene_ = std::make_unique<planning_scene::PlanningScene>(urdf_model, srdf_model);
}

bool CollisionDetection::checkCollision(const Model& model, const geometry_msgs::Pose& grasp_pose)
{
  SceneModelScope model_scope{*scene_, model};
  ROS_INFO_STREAM("Planning frame: " << scene_->getPlanningFrame());



}

}  // namespace chair_manipulation