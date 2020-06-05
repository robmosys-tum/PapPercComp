// --------------------------------------------------------
// Code generated by Papyrus C++
// --------------------------------------------------------

#define KitchenUtensilClassifierCompdef_KitchenUtensilClassifier_BODY

/************************************************************
 KitchenUtensilClassifier class body
 ************************************************************/

// include associated header file
#include "KitchenUtensilClassifierCompdef/KitchenUtensilClassifier.h"

// Derived includes directives
#include "KitchenUtensilClassifierCompdef/KitchenUtensilClassifier_impl.h"
#include "rclcpp/rclcpp.hpp"

// Include from Include declaration (body)
// declare options
rclcpp::NodeOptions kitchenutensilclassifier_options;

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);

	auto kitchenutensilclassifier = std::make_shared<
			KitchenUtensilClassifierCompdef::KitchenUtensilClassifier_impl>(
			kitchenutensilclassifier_options);

	RCLCPP_INFO(kitchenutensilclassifier->get_logger(),
			"KitchenUtensilClassifier has been initialized");

	rclcpp::executors::MultiThreadedExecutor executor;

	executor.add_node(kitchenutensilclassifier->get_node_base_interface());

	executor.spin();
	rclcpp::shutdown();
}

// End of Include declaration (body)

namespace KitchenUtensilClassifierCompdef {

// static attributes (if any)

/**
 * 
 * @param options 
 */
KitchenUtensilClassifier::KitchenUtensilClassifier(
		rclcpp::NodeOptions /*in*/options) :
		rclcpp_lifecycle::LifecycleNode("KitchenUtensilClassifier", options) {
	utensilClass_pub_ = create_publisher<std_msgs::msg::String>("utensilClass",
			1);
	// directly activate a publisher
	utensilClass_pub_->on_activate();

	image_sub_ =
			create_subscription<sensor_msgs::msg::Image>("image",
					rclcpp::QoS(rclcpp::KeepLast(100)).best_effort(),
					std::bind(
							&KitchenUtensilClassifierCompdef::KitchenUtensilClassifier_impl::classifyKitchenUtensil,
							(KitchenUtensilClassifier_impl*) this,
							std::placeholders::_1));

}

} // of namespace KitchenUtensilClassifierCompdef

/************************************************************
 End of KitchenUtensilClassifier class body
 ************************************************************/
