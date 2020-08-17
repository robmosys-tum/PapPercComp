#include <ros/ros.h>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <pluginlib/class_loader.h>
#include <boost/scoped_ptr.hpp>
#include "std_msgs/String.h"
#include "classbox.h"


ros::NodeHandle* handle;

struct Coordinates {
    float x;
    float y;
};

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 *
 */

int chatterCallback(const robot_move::classbox classbox ) {

    if (classbox.disease == "undefined" || classbox.disease_score < 3) {
        ROS_INFO("no disease or low score");
        ROS_INFO("score %i", classbox.disease_score);
        return 0;
    }

    Coordinates coordinates;

    coordinates.x = (classbox.xmin + classbox.xmax) / 2;
    coordinates.y = (classbox.ymin + classbox.ymax) / 2;

    ROS_INFO("I heard %f", coordinates.x);

    ros::NodeHandle node_handle("~");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    const std::string PLANNING_GROUP = "arm";

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

    ROS_INFO("Model frame: %s", robot_model->getModelFrame().c_str());

    moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(robot_model));
    const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);


    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

    planning_scene->getCurrentStateNonConst().setToDefaultValues(joint_model_group, "ready");

    boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
    planning_interface::PlannerManagerPtr planner_instance;
    std::string planner_plugin_name;


    if (!node_handle.getParam("planning_plugin", planner_plugin_name))
        ROS_FATAL_STREAM("Could not find planner plugin name");
    try
    {
        planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
                "moveit_core", "planning_interface::PlannerManager"));
    }
    catch (pluginlib::PluginlibException& ex)
    {
        ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
    }
    try
    {
        planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
        if (!planner_instance->initialize(robot_model, node_handle.getNamespace()))
            ROS_FATAL_STREAM("Could not initialize planner instance");
        ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
    }
    catch (pluginlib::PluginlibException& ex)
    {
        const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
        std::stringstream ss;
        for (std::size_t i = 0; i < classes.size(); ++i)
            ss << classes[i] << " ";
        ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
                                                             << "Available plugins: " << ss.str());
    }

    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
    visual_tools.loadRobotStatePub("/display_robot_state");
    visual_tools.enableBatchPublishing();
    visual_tools.deleteAllMarkers();  // clear all old markers
    visual_tools.trigger();

/* Remote control is an introspection tool that allows users to step through a high level script
via buttons and keyboard shortcuts in RViz */
    visual_tools.loadRemoteControl();

/* RViz provides many types of markers, in this demo we will use text, cylinders, and spheres*/
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.75;
    visual_tools.publishText(text_pose, "Motion Planning API Demo", rvt::WHITE, rvt::XLARGE);

/* Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations */
    visual_tools.trigger();

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    geometry_msgs::Pose pose;
    geometry_msgs::Pose orig_pose =  move_group.getCurrentPose().pose;
    ROS_INFO("value %s", move_group.getEndEffectorLink().c_str());
    pose = orig_pose;
    pose.position.x = coordinates.x;
    pose.position.y = coordinates.y;
    pose.position.z = 0.2;
    move_group.setPoseTarget(pose);

    move_group.move();

    return 0;
}

void chatterCallback2(const robot_move::classbox classbox ) {
    chatterCallback(classbox);
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_move");

    ros::NodeHandle node_handle;

    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::Subscriber sub = node_handle.subscribe("chatter", 1000, chatterCallback2);

    ros::waitForShutdown();

    return 0;
}