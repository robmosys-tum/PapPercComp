// --------------------------------------------------------
// Code generated by Papyrus C++
// --------------------------------------------------------

#define Rgbd_surface_reconstructionCompdef_Rgbd_surface_reconstruction_impl_BODY

/************************************************************
 Rgbd_surface_reconstruction_impl class body
 ************************************************************/

// include associated header file
#include "Rgbd_surface_reconstructionCompdef/Rgbd_surface_reconstruction_impl.h"

// Derived includes directives
#include "rclcpp/rclcpp.hpp"

namespace Rgbd_surface_reconstructionCompdef {

// static attributes (if any)

/**
 * 
 * @param options 
 */
Rgbd_surface_reconstruction_impl::Rgbd_surface_reconstruction_impl(
		rclcpp::NodeOptions /*in*/options) :
		Rgbd_surface_reconstruction(options) {
}

/**
 * 
 * @param image 
 * @param image 
 */
void Rgbd_surface_reconstruction_impl::depth_image_handler(
		const sensor_msgs::msg::Image::SharedPtr /*in*/image,
		const sensor_msgs::msg::Image::SharedPtr /*in*/image) {
}

/**
 * 
 */
void Rgbd_surface_reconstruction_impl::color_image_handler() {
}

} // of namespace Rgbd_surface_reconstructionCompdef

/************************************************************
 End of Rgbd_surface_reconstruction_impl class body
 ************************************************************/
