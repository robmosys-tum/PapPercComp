#include <ros/ros.h>
#include "chair_manipulation_grasp_detection_advanced/grasp_database_creator.h"
#include "chair_manipulation_grasp_detection_advanced/exception.h"

using namespace chair_manipulation;

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "create_grasp_database_node");
  ros::NodeHandle nh_priv{ "~" };
  auto output_filename = nh_priv.param<std::string>("output_filename", "grasp_database.yaml");
  try
  {
    GraspDatabase database;
    GraspDatabaseCreatorParameters params;
    params.load(nh_priv);
    GraspDatabaseCreator creator{ params };
    creator.createGraspDatabase(database);
    database.save(output_filename);
  }
  catch (const exception::Runtime& e)
  {
    ROS_ERROR_STREAM_NAMED("create_grasp_database", e.what());
  }

  return 0;
}