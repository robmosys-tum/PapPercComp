#include <ros/ros.h>

//MoveIt imports
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <boost/scoped_ptr.hpp>

//classbox message header file
#include "classbox.h"

// coordinates struct to save the center of the published classbox
struct Coordinates {
    float x;
    float y;
};

/**
 * Subscriber callback function
 *
*/
int chatterCallback(const robot_move::classbox classbox ) {
    if (classbox.disease == "undefined" || classbox.disease_score < 3) { // if no fruit disease or classification score is too low.
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

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP); //initalize arm move grou

    // load urdf model (data injected from launch file)
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

    ROS_INFO("Model frame: %s", robot_model->getModelFrame().c_str());

    moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(robot_model)); //get start statrt for robot
    const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP); //initalize joint model group with planning group (currenlty it is the arm)

    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

    planning_scene->getCurrentStateNonConst().setToDefaultValues(joint_model_group, "ready");

    boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
    planning_interface::PlannerManagerPtr planner_instance;
    std::string planner_plugin_name;


    if (!node_handle.getParam("planning_plugin", planner_plugin_name))
        ROS_FATAL_STREAM("Could not find planner plugin name");
    try {
        planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
                "moveit_core", "planning_interface::PlannerManager"));
    }
    catch (pluginlib::PluginlibException& ex) {
        ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
    }
    try {
        planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
        if (!planner_instance->initialize(robot_model, node_handle.getNamespace()))
            ROS_FATAL_STREAM("Could not initialize planner instance");
        ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
    }
    catch (pluginlib::PluginlibException& ex) {
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

//    plann interface object
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

//  pose object to set the new position for the end-effector
    geometry_msgs::PoseStamped pose;

    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;

    geometry_msgs::PoseStamped orig_pose =  move_group.getCurrentPose();
    ROS_INFO("value %s", move_group.getEndEffectorLink().c_str());
    pose = orig_pose;

//  set the coordinates of the end-effector
    pose.pose.position.x = coordinates.x;
    pose.pose.position.y = coordinates.y;
    pose.pose.position.z = 0.2;

//  create pose goal on link 6 (link connecting the end-effector)
    moveit_msgs::Constraints pose_goal =
            kinematic_constraints::constructGoalConstraints("link_6", pose);

    req.group_name = PLANNING_GROUP;
    req.goal_constraints.push_back(pose_goal);

    planning_interface::PlanningContextPtr context = //create planning context ptr from the pose goal request
            planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
    context->solve(res); //solve the motion planning problem for the robot

    if (res.error_code_.val != res.error_code_.SUCCESS) {
        ROS_ERROR("Could not compute plan successfully");
        return 0;
    }

    moveit_msgs::MotionPlanResponse response;
    res.getMessage(response); //obtain motion planning response

    //rotate joints according to the plan
    robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
//  set the planning scene state to match the new robot state after joints' rotation
    planning_scene->setCurrentState(*robot_state.get());
//  publish the new robot state to rviz
    visual_tools.publishRobotState(planning_scene->getCurrentStateNonConst(), rviz_visual_tools::GREEN);
    visual_tools.trigger();

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

//  create a subscriber listening on the topic chatter
    ros::Subscriber sub = node_handle.subscribe("chatter", 1000, chatterCallback2);

    ros::waitForShutdown();

    return 0;
}