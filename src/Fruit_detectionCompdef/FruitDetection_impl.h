// --------------------------------------------------------
// Code generated by Papyrus C++
// --------------------------------------------------------

#ifndef FRUIT_DETECTIONCOMPDEF_FRUITDETECTION_IMPL_H
#define FRUIT_DETECTIONCOMPDEF_FRUITDETECTION_IMPL_H

/************************************************************
 FruitDetection_impl class header
 ************************************************************/

#include "Fruit_detectionCompdef/Pkg_Fruit_detectionCompdef.h"

#include "Fruit_detectionCompdef/FruitDetection.h"
#include "sensor_msgs/msg/image.hpp"

namespace ros2Library {
namespace rclcpp {
class NodeOptions;
}
}

namespace Fruit_detectionCompdef {

/************************************************************/
/**
 * This is a skeleton class generated for component FruitDetection
 * Copy it into the source folder as an initial base (or copy parts
 * of it whenever you add modify the component).
 * 
 */
class FruitDetection_impl: public FruitDetection {
public:

	/**
	 * 
	 * @param options 
	 */
	FruitDetection_impl(rclcpp::NodeOptions /*in*/options);

	/**
	 * 
	 * @param image 
	 */
	void FruitDetectionHandler(
			const sensor_msgs::msg::Image::SharedPtr /*in*/image);

};
/************************************************************/
/* External declarations (package visibility)               */
/************************************************************/

/* Inline functions                                         */

} // of namespace Fruit_detectionCompdef

/************************************************************
 End of FruitDetection_impl class header
 ************************************************************/

#endif