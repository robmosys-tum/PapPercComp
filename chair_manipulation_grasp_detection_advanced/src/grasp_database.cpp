#include "chair_manipulation_grasp_detection_advanced/grasp_database.h"
#include "chair_manipulation_grasp_detection_advanced/exception.h"
#include "chair_manipulation_grasp_detection_advanced/utils.h"
#include <tf2_eigen/tf2_eigen.h>

namespace chair_manipulation
{
void GraspDatabase::load(ros::NodeHandle& nh)
{
  XmlRpc::XmlRpcValue element_items;
  if (!nh.getParam("elements", element_items) || element_items.getType() != XmlRpc::XmlRpcValue::TypeArray ||
      element_items.size() == 0)
    throw exception::Parameter{ "Failed to load parameter 'elements'." };

  auto num_elements = element_items.size();
  elements_.resize(num_elements);
  for (std::size_t i = 0; i < num_elements; i++)
  {
    const auto& element_item = element_items[i];
    auto& element = elements_[i];
    element->mesh_filename_ = utils::loadStringParameter(element_item, "mesh_filename");
    element->point_cloud_filename_ = utils::loadStringParameter(element_item, "point_cloud_filename");

    if (!element_item.hasMember("grasps"))
      throw exception::Parameter{ "Failed to load parameter 'grasps'." };

    const auto& grasp_items = element_item["grasps"];
    auto num_grasps = grasp_items.size();
    element->grasps_.resize(num_grasps);
    for (std::size_t j = 0; j < num_grasps; j++)
    {
      const auto& grasp_item = grasp_items[j];
      auto& grasp = element->grasps_[j];
      grasp.quality_ = utils::loadDoubleParameter(grasp_item, "quality");

      if (!grasp_item.hasMember("poses"))
        throw exception::Parameter{ "Failed to load parameter 'poses'." };

      const auto& pose_items = grasp_item["poses"];
      auto num_poses = pose_items.size();
      grasp.poses_.resize(num_poses);
      for (std::size_t k = 0; k < num_poses; k++)
      {
        auto pose_item = pose_items[k];
        auto& pose = grasp.poses_[k];
        auto pose_str = (std::string)pose_item;
        std::istringstream iss_pose{ pose_str };
        std::vector<std::string> entries{ std::istream_iterator<std::string>{ iss_pose },
                                          std::istream_iterator<std::string>{} };
        if (entries.size() != 7)
          throw exception::Parameter{"Invalid pose."};

        geometry_msgs::Pose msg;
        msg.position.x = std::stod(entries[0]);
        msg.position.y = std::stod(entries[1]);
        msg.position.z = std::stod(entries[2]);
        msg.orientation.x = std::stod(entries[3]);
        msg.orientation.y = std::stod(entries[4]);
        msg.orientation.z = std::stod(entries[5]);
        msg.orientation.w = std::stod(entries[6]);
        tf2::fromMsg(msg, pose);
      }
    }
  }
}

void GraspDatabase::store(ros::NodeHandle& nh) const
{
  XmlRpc::XmlRpcValue element_items;
  element_items.setSize(elements_.size());
  for (std::size_t i = 0; i < elements_.size(); i++)
  {
    const auto& element = elements_[i];
    XmlRpc::XmlRpcValue element_item;
    element_item["mesh_filename"] = XmlRpc::XmlRpcValue{ element->mesh_filename_ };
    element_item["point_cloud_filename"] = XmlRpc::XmlRpcValue{ element->point_cloud_filename_ };

    const auto& grasps = element->grasps_;
    XmlRpc::XmlRpcValue grasp_items;
    grasp_items.setSize(grasps.size());
    for (std::size_t j = 0; j < grasps.size(); j++)
    {
      const auto& grasp = grasps[j];
      XmlRpc::XmlRpcValue grasp_item;
      grasp_item["quality"] = grasp.quality_;

      const auto& poses = grasp.poses_;
      XmlRpc::XmlRpcValue pose_items;
      pose_items.setSize(poses.size());
      for (std::size_t k = 0; k < poses.size(); k++)
      {
        const auto& pose = poses[k];
        auto msg = tf2::toMsg(pose);
        std::ostringstream oss_pose;
        oss_pose << msg.position.x << " " << msg.position.y << " " << msg.position.z << " " << msg.orientation.x << " "
                 << msg.orientation.y << " " << msg.orientation.z << " " << msg.orientation.w;
        XmlRpc::XmlRpcValue pose_item{ oss_pose.str() };
        pose_items[k] = pose_item;
      }
      grasp_item["poses"] = pose_items;
      grasp_items[j] = grasp_item;
    }
    element_item["grasps"] = grasp_items;
    element_items[i] = element_item;
  }
  nh.setParam("elements", element_items);
}

}  // namespace chair_manipulation