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

// abs()
#include <stdlib.h>

//#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"


namespace Rgbd_surface_reconstructionCompdef {

// static attributes (if any)
int  encoding2mat_type(const std::string & encoding);
void image_message_to_cv(cv::Mat &frame, sensor_msgs::msg::Image::SharedPtr msg);
void pose_to_transform(
    geometry_msgs::msg::PoseStamped::SharedPtr pose, float* matrix_array);

/**
 * 
 * @param options 
 */
Rgbd_surface_reconstruction_impl::Rgbd_surface_reconstruction_impl(
		rclcpp::NodeOptions /*in*/options) :
		Rgbd_surface_reconstruction(options) {

	_voxel_scale = this->declare_parameter("voxel_scale", 2.5f);
	_volume_size.x = this->declare_parameter("volume_size_x", 300);
	_volume_size.y = this->declare_parameter("volume_size_y", 800);
	_volume_size.z = this->declare_parameter("volume_size_z", 300);	
	_offset.x = this->declare_parameter("offset_x", 0.f);
	_offset.y = this->declare_parameter("offset_y", 800.f);
	_offset.z = this->declare_parameter("offset_z", -800.f);
	_use_output_frame = this->declare_parameter("use_output_frame", true);
	_use_kinect_noise_model = this->declare_parameter("use_kinect_noise_model", false);
	_use_every_nth_frame = this->declare_parameter("use_every_nth_frame", 25);
	_export_frame = this->declare_parameter("export_frame", 1400);
	_export_name = this->declare_parameter("export_name", "model");
	
	kinectfusion::GlobalConfiguration configuration;
	configuration.voxel_scale = _voxel_scale;
	configuration.volume_size = _volume_size;
	configuration.model_offset = _offset;
	configuration.use_output_frame = _use_output_frame;
	configuration.use_kinect_noise_model = _use_kinect_noise_model;
	configuration.triangles_buffer_size = 12000000;

	_kinect_pipeline_ptr = new kinectfusion::Pipeline(_camera.get_parameters(), configuration);
}

Rgbd_surface_reconstruction_impl::~Rgbd_surface_reconstruction_impl()
{
}

/**
 * 
 * @param image 
 */
void Rgbd_surface_reconstruction_impl::depth_image_handler(
		const sensor_msgs::msg::Image::SharedPtr /*in*/image) {
	//_depth_image = image;
	_depth_images.push(image);

	if(!_depth_image_initialized)
		_depth_image_initialized = true;

	//process_frames();
}

void Rgbd_surface_reconstruction_impl::pose_handler(
		const geometry_msgs::msg::PoseStamped::SharedPtr /*in*/pose) {
	//_pose = pose;

	_poses.push(pose);

	if(!_pose_initialized)
		_pose_initialized = true;
	
	//process_frames();

}

/**
 * 
 * @param image 
 */
void Rgbd_surface_reconstruction_impl::color_image_handler(
		const sensor_msgs::msg::Image::SharedPtr /*in*/image) {
	//_color_image = image;
	_color_images.push(image);
	if(!_color_image_initialized)
		_color_image_initialized = true;

}

void Rgbd_surface_reconstruction_impl::process_frames(){
	if(_poses.size() < 2 || _depth_images.size() < 2 || _color_images.size() < 2)
		return;

	//align_queues
	bool aligned = false;
	_pose = _poses.front();
	_poses.pop();
	_depth_image = _depth_images.front();
	_depth_images.pop();
	_color_image = _color_images.front();
	_color_images.pop();

	while(!aligned){
		
		double depth_image_timestamp = _depth_image->header.stamp.sec + _depth_image->header.stamp.nanosec/1000000000.0;
		double color_image_timestamp = _color_image->header.stamp.sec + _color_image->header.stamp.nanosec/1000000000.0;
		double pose_timestamp = _pose->header.stamp.sec + _pose->header.stamp.nanosec/1000000000.0;

		double diff_pose = depth_image_timestamp - pose_timestamp;
		double diff_color = depth_image_timestamp - color_image_timestamp;

		//If two messages are closer than 16ms we assume they belong together.
		if (abs(diff_pose) <= 0.015 && abs(diff_color) <= 0.015 ){
			aligned = true;
			break;
		}

		if (diff_pose  > -0.015){
		std::cout << "Skipping pose " << diff_pose << std::endl;
			if(_poses.empty())
				return;
			_pose = _poses.front();
			_poses.pop();
		}
		else if (diff_color  > -0.015){
			std::cout << "Skipping color" << std::endl;
			if(_color_images.empty())
				return;
			_color_image = _color_images.front();
			_color_images.pop();

		}
		else if (diff_pose < 0.015 || diff_color < 0.015){
			std::cout << "Skipping depth" << std::endl;
			if(_depth_images.empty())
				return;
			_depth_image = _depth_images.front();
			_depth_images.pop();
		}
	} 

	cv::Mat depth_map, color_map;

	image_message_to_cv(depth_map, _depth_image);
	image_message_to_cv(color_map, _color_image);

	//cv::imshow("depth_map", depth_map);
	//cv::imshow("color_map", color_map);
	//cv::waitKey(1);

	float m[16];
	pose_to_transform(_pose, m);
	Eigen::Matrix4f current_pose;
	current_pose << m[ 0], m[ 1], m[ 2], m[ 3],
					m[ 4], m[ 5], m[ 6], m[ 7],
					m[ 8], m[ 9], m[10], m[11],
					m[12], m[13], m[14], m[15];
	
	//std::cout << current_pose << std::endl;

	if (_frame_counter % _use_every_nth_frame == 0){
		std::cout << "Processing frame: " << _frame_counter << std::endl;
		_kinect_pipeline_ptr->process_frame(depth_map, color_map, current_pose);
	} else {
		std::cout << "  Skipping frame: " << _frame_counter << std::endl;
	}

	_frame_counter ++;
	cv::imshow("Pipeline Output", _kinect_pipeline_ptr->get_last_model_frame());
	cv::waitKey(1);

	//_tsdf_fusion.integrate_frame(depth_map, matrix_array);

	// Retrieve camera poses
	//auto poses = _kinect_pipeline_ptr->get_poses();

	if (_frame_counter == _export_frame)
	{
		// Export surface mesh
		auto mesh = _kinect_pipeline_ptr->extract_mesh();
		std::string filename = "./" + _export_name 
							 + (_use_kinect_noise_model ? "_new" : "_org")
							 + "_" + std::to_string(_voxel_scale) 
							 + "_" + std::to_string(_export_frame)
							 + "_" + std::to_string(_use_every_nth_frame)
							 + ".ply";
		kinectfusion::export_ply(filename, mesh);
		std::cout << "Exported to " << filename << std::endl;
		_finished = true;
	}


}

// from ROS2 image_tools demo [https://github.com/ros2/demos/blob/eloquent/image_tools/src/showimage.cpp]
int encoding2mat_type(const std::string & encoding) {
	if (encoding == "mono8") {
	return CV_8UC1;
	} else if (encoding == "bgr8") {
	return CV_8UC3;
	} else if (encoding == "mono16") {
	return CV_16SC1;
	} else if (encoding == "rgba8") {
	return CV_8UC4;
	} else if (encoding == "bgra8") {
	return CV_8UC4;
	} else if (encoding == "32FC1") {
	return CV_32FC1;
	} else if (encoding == "rgb8") {
	return CV_8UC3;
	} else {
	throw std::runtime_error("Unsupported encoding type");
	}
}

void image_message_to_cv(cv::Mat &frame, sensor_msgs::msg::Image::SharedPtr msg) {
	cv::Mat tmp_frame(
        msg->height, msg->width, encoding2mat_type(msg->encoding),
        const_cast<unsigned char *>(msg->data.data()), msg->step);
	
	frame = tmp_frame;
}

void pose_to_transform(
    geometry_msgs::msg::PoseStamped::SharedPtr pose, float* matrix_array){

    tf2::Quaternion q(pose->pose.orientation.x, pose->pose.orientation.y,
                     pose->pose.orientation.z, pose->pose.orientation.w);

    tf2::Matrix3x3 m(q);

    std::vector<float> matrix;

    for(int i = 0; i < 3; i++)
        matrix.push_back(m[0][i]);

    matrix.push_back(pose->pose.position.x * 1000);

    for(int i = 0; i < 3; i++)
        matrix.push_back(m[1][i]);

    matrix.push_back(pose->pose.position.y * 1000);

    for(int i = 0; i < 3; i++)
        matrix.push_back(m[2][i]);

    matrix.push_back(pose->pose.position.z * 1000);

    for(int i = 0; i < 3; i++)
        matrix.push_back(0.0);

    matrix.push_back(1.0);

	std::copy(matrix.begin(), matrix.end(), matrix_array); 
}

} // of namespace Rgbd_surface_reconstructionCompdef

/************************************************************
 End of Rgbd_surface_reconstruction_impl class body
 ************************************************************/
