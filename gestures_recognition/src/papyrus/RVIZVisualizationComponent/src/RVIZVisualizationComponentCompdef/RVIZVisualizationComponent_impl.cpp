// --------------------------------------------------------
// Code generated by Papyrus C++
// --------------------------------------------------------

#define RVIZVisualizationComponentCompdef_RVIZVisualizationComponent_impl_BODY

/************************************************************
 RVIZVisualizationComponent_impl class body
 ************************************************************/

// include associated header file
//#include "../../src/RVIZVisualizationComponentCompdef/RVIZVisualizationComponent_impl.h"
#include "geometry_msgs/msg/pose_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

// Derived includes directives
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;
using namespace std;
using namespace std::chrono_literals;

class RVIZVisualization : public rclcpp::Node
{
	public:
	RVIZVisualization() : Node("RVIZ_Visualization")
		{
			subscriber_ = this->create_subscription<geometry_msgs::msg::PoseArray>("hand_joints", 10, std::bind(&RVIZVisualization::topic_callback, this, _1));
			publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("hand_markers", 10);
			//timer_ = this->create_wall_timer(500ms, std::bind(&RVIZVisualization::timer_callback, this));
		}

	private:
		void topic_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) const
		{

			visualization_msgs::msg::MarkerArray markerVector;

			// Set our initial shape type to be a cube
			uint32_t shape = visualization_msgs::msg::Marker::SPHERE;

			for(size_t i = 0; i < msg->poses.size(); i++)
			{
				visualization_msgs::msg::Marker marker;
				// Set the frame ID and timestamp.  See the TF tutorials for information on these.
				marker.header.frame_id = "/base_link";
				//TODO: find ros2 equivalent of ros::Time::now()
				//marker.header.stamp = ros::Time::now();

				// Set the namespace and id for this marker.  This serves to create a unique ID
				// Any marker sent with the same namespace and id will overwrite the old one
				marker.ns = "basic_shapes";
				marker.id = i;

				// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
				marker.type = shape;

				// Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
				marker.action = visualization_msgs::msg::Marker::ADD;

				//TODO: Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
				marker.pose.position.x = msg->poses[i].position.x;
				marker.pose.position.y = msg->poses[i].position.y;
				marker.pose.position.z = msg->poses[i].position.z;
				marker.pose.orientation.x = 0.0;
				marker.pose.orientation.y = 0.0;
				marker.pose.orientation.z = 0.0;
				marker.pose.orientation.w = 1.0;



				// Set the scale of the marker -- 1x1x1 here means 1m on a side
				marker.scale.x = 0.03;
				marker.scale.y = 0.03;
				marker.scale.z = 0.03;

				if (i==0)
				{
					marker.color.r = 0.0f;
					marker.color.g = 0.0f;
					marker.color.b = 1.0f;
					marker.color.a = 1.0;
				}
				else if (i>0 && i<=4)
				{
					marker.color.r = 0.0f;
					marker.color.g = 1.0f;
					marker.color.b = 0.0f;
					marker.color.a = 1.0;
				}
				else if (i>4 && i<=8)
				{
					marker.color.r = 1.0f;
					marker.color.g = 0.0f;
					marker.color.b = 0.0f;
					marker.color.a = 1.0;
								}
				else if (i>8 && i<=12)
				{
					marker.color.r = 1.0f;
					marker.color.g = 1.0f;
					marker.color.b = 0.0f;
					marker.color.a = 1.0;
				}
				else if (i>12 && i<=16)
				{
					marker.color.r = 0.0f;
					marker.color.g = 1.0f;
					marker.color.b = 1.0f;
					marker.color.a = 1.0;
				}
				else if (i>16 && i<=20)
				{
					marker.color.r = 1.0f;
					marker.color.g = 0.0f;
					marker.color.b = 1.0f;
					marker.color.a = 1.0;
				}



				//TODO: Find ros2 equivalent of ros::Duration()
				//marker.lifetime = ros::Duration();

				// emplace back marker into markerVector
				markerVector.markers.emplace_back(marker);

			}

			//TODO: markerArray does not contain a header
			//markerVector.header.frame_id = std::string("/base_link");
			publisher_->publish(markerVector);
		}

		rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscriber_;
		//rclcpp::TimerBase::SharedPtr timer_;
	    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
};




int main(int argc, char ** argv)
{

  printf("hello world gesture recognition subscriber package\n");

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RVIZVisualization>());
  rclcpp::shutdown();

  return 0;
}



/************************************************************
 End of RVIZVisualizationComponent_impl class body
 ************************************************************/
