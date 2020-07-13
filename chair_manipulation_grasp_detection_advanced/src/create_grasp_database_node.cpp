#include <ros/ros.h>
#include "chair_manipulation_grasp_detection_advanced/grasp_database_creator.h"
#include "chair_manipulation_grasp_detection_advanced/exception.h"

using namespace chair_manipulation;

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "create_grasp_database_node");
  try
  {
    ros::NodeHandle nh_priv{"~"};
    std::string gripper_urdf, gripper_srdf, grasp_database_filename;
    if (nh_priv.getParam("gripper_urdf", gripper_urdf))
      throw exception::Parameter{"Failed to load parameter 'gripper_urdf'."};
    if (nh_priv.getParam("gripper_srdf", gripper_srdf))
      throw exception::Parameter{"Failed to load parameter 'gripper_srdf'."};
    if (nh_priv.getParam("grasp_database_filename", grasp_database_filename))
      throw exception::Parameter{"Failed to load parameter 'grasp_database_filename'."};

    GripperParameters gripper_params;
    ros::NodeHandle gripper_nh{"~gripper"};
    gripper_params.load(gripper_nh);
    auto gripper = std::make_shared<Gripper>(std::move(gripper_params), gripper_urdf, gripper_srdf);

    GraspSamplerParameters grasp_sampler_params;
    ros::NodeHandle grasp_sampler_nh{ "~grasp_sampler" };
    grasp_sampler_params.load(grasp_sampler_nh);
    auto grasp_sampler = std::make_shared<GraspSampler>(std::move(grasp_sampler_params), gripper);

    GraspQualityWeights grasp_quality_weights;
    ros::NodeHandle grasp_quality_weights_nh{ "~weights_offline" };
    grasp_quality_weights.load(grasp_quality_weights_nh);

    GraspSynthesizerParameters grasp_synthesizer_params;
    ros::NodeHandle grasp_synthesizer_nh{ "~grasp_synthesizer" };
    grasp_synthesizer_params.load(grasp_synthesizer_nh);
    auto grasp_synthesizer =
        std::make_shared<GraspSynthesizer>(std::move(grasp_synthesizer_params), std::move(grasp_quality_weights));

    GraspDatabaseCreatorParameters grasp_database_creator_params;
    ros::NodeHandle grasp_database_creator_nh{ "~grasp_database_creator" };
    grasp_database_creator_params.load(grasp_database_creator_nh);
    GraspDatabaseCreator creator{ std::move(grasp_database_creator_params), grasp_sampler, grasp_synthesizer };

    GraspDatabase database;
    creator.createGraspDatabase(database);
    ros::NodeHandle database_nh{"grasp_database"};
    database.store(database_nh);

    std::string dump_cmd = "rosparam dump grasp_database > " + grasp_database_filename;
    std::system(dump_cmd.c_str());
  }
  catch (const exception::Runtime& e)
  {
    ROS_ERROR_STREAM_NAMED("create_grasp_database", e.what());
  }

  return 0;
}