#include "chair_manipulation_grasp_planning/utils.h"

namespace chair_manipulation
{
void addGroundPlane(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
                    const std::string& frame)
{
  std::vector<std::string> object_names;
  object_names = planning_scene_interface.getKnownObjectNames();
  if (std::find(object_names.begin(), object_names.end(), "ground_plane") != object_names.end())
  {
    ROS_WARN("Ground plane already added to collision objects.");
    return;
  }

  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = frame;
  collision_object.header.stamp = ros::Time::now();
  collision_object.id = "ground_plane";

  shape_msgs::Plane plane;
  plane.coef[0] = 0.0;
  plane.coef[1] = 0.0;
  plane.coef[2] = 1.0;
  plane.coef[3] = 0.0;

  geometry_msgs::Pose plane_pose;
  plane_pose.orientation.w = 1.0;
  plane_pose.position.x = 0.0;
  plane_pose.position.y = 0.0;
  plane_pose.position.z = -0.01;

  collision_object.planes.push_back(plane);
  collision_object.plane_poses.push_back(plane_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  planning_scene_interface.addCollisionObjects(collision_objects);
}

}  // namespace chair_manipulation