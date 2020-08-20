// --------------------------------------------------------
// Code generated by Papyrus C++
// --------------------------------------------------------

#define CameraDriverComponentCompdef_CameraDriverComponent_BODY

/************************************************************
 CameraDriverComponent class body
 ************************************************************/

// include associated header file
#include "CameraDriverComponentCompdef/CameraDriverComponent.h"

// Derived includes directives
#include "../../src/CameraDriverComponentCompdef/CameraDriverComponent_impl.h"
#include "rclcpp/rclcpp.hpp"

// Include from Include declaration (body)
// declare options
rclcpp::NodeOptions cameradrivercomponent_options;

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);

	auto cameradrivercomponent = std::make_shared<
			CameraDriverComponentCompdef::CameraDriverComponent_impl>(
			cameradrivercomponent_options);

	RCLCPP_INFO(cameradrivercomponent->get_logger(),
			"CameraDriverComponent has been initialized");

	rclcpp::executors::MultiThreadedExecutor executor;

	executor.add_node(cameradrivercomponent->get_node_base_interface());

	executor.spin();
	rclcpp::shutdown();
}

// End of Include declaration (body)

namespace CameraDriverComponentCompdef {

// static attributes (if any)

/**
 * 
 * @param options 
 */
CameraDriverComponent::CameraDriverComponent(rclcpp::NodeOptions /*in*/options) :
		rclcpp_lifecycle::LifecycleNode("CameraDriverComponent", options) {
	video_output_recv_ =
			create_subscription<sensor_msgs::msg::Image>("video_output",
					rclcpp::QoS(rclcpp::KeepLast(100)).best_effort(),
					std::bind(
							&CameraDriverComponentCompdef::CameraDriverComponent_impl::publishVideo,
							(CameraDriverComponent_impl*) this,
							std::placeholders::_1));

}

} // of namespace CameraDriverComponentCompdef

/************************************************************
 End of CameraDriverComponent class body
 ************************************************************/
