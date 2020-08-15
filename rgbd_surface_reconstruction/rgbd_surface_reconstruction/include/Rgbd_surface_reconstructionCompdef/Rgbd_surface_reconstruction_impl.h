// --------------------------------------------------------
// Code generated by Papyrus C++
// --------------------------------------------------------

#ifndef RGBD_SURFACE_RECONSTRUCTIONCOMPDEF_RGBD_SURFACE_RECONSTRUCTION_IMPL_H
#define RGBD_SURFACE_RECONSTRUCTIONCOMPDEF_RGBD_SURFACE_RECONSTRUCTION_IMPL_H

/************************************************************
 Rgbd_surface_reconstruction_impl class header
 ************************************************************/

#include <queue>

#include "Rgbd_surface_reconstructionCompdef/Pkg_Rgbd_surface_reconstructionCompdef.h"

#include "Rgbd_surface_reconstructionCompdef/Rgbd_surface_reconstruction.h"
#include "sensor_msgs/msg/image.hpp"

//Kinect Fusion
//#include "tsdf_fusion.hcu"
#include "kinectfusion.h"
#include "Rgbd_surface_reconstructionCompdef/depth_camera.h"

namespace ros2Library {
namespace rclcpp {
class NodeOptions;
}
}

namespace Rgbd_surface_reconstructionCompdef {

/************************************************************/
/**
 * This is a skeleton class generated for component Rgbd_surface_reconstruction
 * Copy it into the source folder as an initial base (or copy parts
 * of it whenever you add modify the component).
 * 
 */
class Rgbd_surface_reconstruction_impl: public Rgbd_surface_reconstruction {
public:

	/**
	 * 
	 * @param options 
	 */
	Rgbd_surface_reconstruction_impl(rclcpp::NodeOptions /*in*/options);

	~Rgbd_surface_reconstruction_impl();

	/**
	 * 
	 * @param image 
	 */
	void depth_image_handler(
			const sensor_msgs::msg::Image::SharedPtr /*in*/image);
	
	void color_image_handler(
			const sensor_msgs::msg::Image::SharedPtr /*in*/image);
	
	void pose_handler(
			const geometry_msgs::msg::PoseStamped::SharedPtr /*in*/pose);
	
	bool _finished = false;
	void process_frames();

private:
	//TSDFFusion _tsdf_fusion;
	KinectCamera _camera {};

	kinectfusion::Pipeline* _kinect_pipeline_ptr;

	float _voxel_scale;
	int3 _volume_size;
	float3 _offset;
	bool _use_output_frame;
	bool _use_kinect_noise_model;
	int _use_every_nth_frame;
	int _export_frame;
	std::string _export_name;

	int _frame_counter = 0;

	bool _depth_image_initialized = false;
	bool _color_image_initialized = false;
	bool _pose_initialized = false;

	sensor_msgs::msg::Image::SharedPtr _depth_image;
	sensor_msgs::msg::Image::SharedPtr _color_image;
	geometry_msgs::msg::PoseStamped::SharedPtr _pose;

	std::queue<sensor_msgs::msg::Image::SharedPtr> _depth_images;
	std::queue<sensor_msgs::msg::Image::SharedPtr> _color_images;
	std::queue<geometry_msgs::msg::PoseStamped::SharedPtr> _poses;

};
/************************************************************/
/* External declarations (package visibility)               */
/************************************************************/

/* Inline functions                                         */

} // of namespace Rgbd_surface_reconstructionCompdef

/************************************************************
 End of Rgbd_surface_reconstruction_impl class header
 ************************************************************/

#endif