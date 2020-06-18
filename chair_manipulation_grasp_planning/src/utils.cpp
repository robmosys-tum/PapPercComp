#include "chair_manipulation_grasp_planning/utils.h"

namespace chair_manipulation
{

void add_ground_plane(moveit::planning_interface::MoveGroupInterface &group,
                      moveit::planning_interface::PlanningSceneInterface &planning_scene_interface)
{
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = group.getPlanningFrame();
    collision_object.id = "ground_plane";

    shape_msgs::Plane plane;
    plane.coef[0] = 0.0;
    plane.coef[1] = 0.0;
    plane.coef[2] = 1.0;
    plane.coef[3] = 0.0;

    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.0;
    box_pose.position.y = 0.0;
    box_pose.position.z = -0.1;

    collision_object.planes.push_back(plane);
    collision_object.plane_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);

    planning_scene_interface.addCollisionObjects(collision_objects);
}

}