// --------------------------------------------------------
// Code generated by Papyrus C++
// --------------------------------------------------------

#define Garbage_classificationCompdef_Garbage_classification_BODY

/************************************************************
 Garbage_classification class body
 ************************************************************/

// include associated header file
#include "Garbage_classificationCompdef/Garbage_classification.h"

// Derived includes directives
#include "Garbage_classificationCompdef/Garbage_classification_impl.h"
#include "rclcpp/rclcpp.hpp"

// Include from Include declaration (body)
// declare options
rclcpp::NodeOptions garbage_classification_options;

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);

	auto garbage_classification = std::make_shared
			< Garbage_classificationCompdef::Garbage_classification_impl
			> (garbage_classification_options);

	RCLCPP_INFO(garbage_classification->get_logger(),
			"Garbage_classification has been initialized");

	rclcpp::executors::MultiThreadedExecutor executor;

	executor.add_node(garbage_classification->get_node_base_interface());

	executor.spin();
	rclcpp::shutdown();
}

// End of Include declaration (body)

namespace Garbage_classificationCompdef {

// static attributes (if any)

/**
 * 
 * @param options 
 */
Garbage_classification::Garbage_classification(
		rclcpp::NodeOptions /*in*/options) :
		rclcpp_lifecycle::LifecycleNode("Garbage_classification", options) {
	classification_pub_ = create_publisher < std_msgs::msg::String
			> ("classification", 1);
	// directly activate a publisher
	classification_pub_->on_activate();

	image_sub_ =
			create_subscription < sensor_msgs::msg::Image
					> ("image", rclcpp::QoS(rclcpp::KeepLast(100)).best_effort(), std::bind(
							&Garbage_classificationCompdef::Garbage_classification_impl::classify,
							(Garbage_classification_impl*) this,
							std::placeholders::_1));

}

} // of namespace Garbage_classificationCompdef

/************************************************************
 End of Garbage_classification class body
 ************************************************************/
