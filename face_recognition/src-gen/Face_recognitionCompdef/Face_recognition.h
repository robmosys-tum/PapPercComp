// --------------------------------------------------------
// Code generated by Papyrus C++
// --------------------------------------------------------

#ifndef FACE_RECOGNITIONCOMPDEF_FACE_RECOGNITION_H
#define FACE_RECOGNITIONCOMPDEF_FACE_RECOGNITION_H

/************************************************************
 Face_recognition class header
 ************************************************************/

#include "Face_recognitionCompdef/Pkg_Face_recognitionCompdef.h"

#include "rclcpp_lifecycle/lifecycle_node.hpp"

// Include from Include stereotype (header)
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/opencv.hpp"
#include <iostream>

using namespace std;
using namespace cv;

// End of Include stereotype (header)

namespace ros2Library {
namespace rclcpp {
class NodeOptions;
}
}
namespace ros2Library {
namespace rclcpp {
class Subscription;
}
}
/*
namespace sensor_msgs {
namespace msg {
struct Image;
}
}
*/

namespace Face_recognitionCompdef {

/************************************************************/
/**
 * 
 */
class Face_recognition: public rclcpp_lifecycle::LifecycleNode {
public:

	/**
	 * 
	 */
	std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Image>> Image_sub;
	/**
	 * 
	 */
	std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Pose2D>> Pose_pub;
	/**
	 * 
	 * @param image sensor_msgs::msg::Image::SharedPtr
	 */
	void FaceDetectionHandler(const sensor_msgs::msg::Image::SharedPtr /*in*/image);

	/**
	 * 
	 * @param options 
	 */
	Face_recognition(rclcpp::NodeOptions /*in*/options);

};
/************************************************************/
/* External declarations (package visibility)               */
/************************************************************/

/* Inline functions                                         */

} // of namespace Face_recognitionCompdef

/************************************************************
 End of Face_recognition class header
 ************************************************************/

#endif
