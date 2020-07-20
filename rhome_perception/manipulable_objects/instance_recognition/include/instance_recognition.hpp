/*
 * instance_recognition.h
 *
 *  Created on: 07-2019
 *  ROS2 version on: 06-2020
 *      Author: luz.martinez@tum.de
 */

#ifndef INSTANCERECOGNITION_H_
#define INSTANCERECOGNITION_H_

// C, C++
#include <iostream>
#include <string>
#include <cmath>
#include <limits>
#include <map>
#include <list>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/filesystem.hpp>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "boost/bind.hpp"
// #include <rclcpp/package.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
// #include <tf/transform_listener.h>
// #include <pcl_ros/transforms.h>
// #include <pcl_ros/point_cloud.h>
// #include <visualization_msgs/Marker.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

// OpenCV
// #include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
// #include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>
// #include "opencv2/core/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/features2d/features2d.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>


// #include "opencv2/nonfree/features2d.hpp"
// #include "opencv2/nonfree/nonfree.hpp"
// #include "opencv2/xfeatures2d/nonfree.hpp"
// #include <opencv2/xfeatures2d/nonfree.hpp>
#include "opencv4/opencv2/xfeatures2d.hpp"
#include "opencv4/opencv2/xfeatures2d/nonfree.hpp"


// srvs and msgs
#include <rhome_metadata/srv/onoff.hpp>
#include <rhome_metadata/srv/queryobject.hpp>
#include <rhome_metadata/srv/getinformation.hpp>
// #include <robmosys_srvs/objectinformation.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rhome_metadata/msg/roi.hpp>
#include <rhome_metadata/msg/objects.hpp>
#include <sensor_msgs/msg/camera_info.hpp>


#include "lbplibrary.hpp"


namespace rhome_perception {

using std::string;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using namespace std::chrono_literals;
using namespace boost::filesystem;


struct box_obj {
  cv::Point p1;
  cv::Point p2;
  cv::Point p3;
  cv::Point p4;
};


class InstanceRecognition  {


private:
	rclcpp::Node::SharedPtr _nh;
	// std::string _name;
	bool _enabled;
	std::vector<float> colorcamera_info;
	std::string db_path;
	bool train_succes;
	std::vector<cv::Mat> db_dscr;
	std::vector<std::string> db_label;
	std::vector<std::vector<cv::KeyPoint>> db_kp;
	std::vector<int> db_width;
	std::vector<int> db_heigth;
	int level;

	// - - - - - Parameters - - - - - - -
	std::string _base_frame;
	std::string _rgbd_frame;
	std::string _rgb_frame;
	std::string _depth_frame;
	std::string _rgb_topic;
	std::string _depth_topic;
	std::string _depth_info_topic;

	sensor_msgs::msg::Image::SharedPtr rgb_in;
	sensor_msgs::msg::Image::SharedPtr depth_in;
	std::string image_frameid;

	cv::Mat ImageIn, DepthIn;
	std::string _algorithm;
	std::string _compare_hist;
	std::vector<std::string> recognition_alg;
	std::vector<std::string> compare_hist_methods;
	cv::Mat descriptors;
	
	//Detection Information
	std::vector<cv::Mat> detected_imgs;
	std::vector<geometry_msgs::msg::PoseStamped> detected_pose;
	std::vector<rhome_metadata::msg::Roi> detected_roi;
	std::vector<std::string> detected_label;
	cv::Mat depth_img;

	//LBP
	lbplibrary::LBP *lbp;

	//Features and Matching
	cv::Ptr<cv::Feature2D> detector;
	cv::Ptr<cv::Feature2D> extractor;
	cv::Ptr<cv::DescriptorMatcher> matcher;

	// publishers
	rclcpp::Publisher<rhome_metadata::msg::Objects>::SharedPtr _objects_pub;

	//Subscribers 
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr  _subs_rgb;
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr  _subs_depth;
	rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr  _subs_dinfo;

	// services
	rclcpp::Service<rhome_metadata::srv::Getinformation>::SharedPtr _getobjects_server;
	rclcpp::Service<rhome_metadata::srv::Queryobject>::SharedPtr _queryobjects_server;



public:
	bool _is_on;
	bool ready_rgb;
	bool ready_depth;
	bool roi_ready;

	InstanceRecognition(rclcpp::Node::SharedPtr nh);
	virtual ~InstanceRecognition();

	void run();


private:

	// - - - - - - S u b s c r i b e r   C a l l b a c k s  - - - - - - - - -
	void _process_depth(const sensor_msgs::msg::Image::SharedPtr img);
	void _process_rgb(const sensor_msgs::msg::Image::SharedPtr img);
	void _process_depthinfo(const sensor_msgs::msg::CameraInfo::SharedPtr info);

	// - - - - - - - - - - - - S e r v i c e s - - - - - - - - -
	void getobjects_service( const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<rhome_metadata::srv::Getinformation::Request> req, std::shared_ptr<rhome_metadata::srv::Getinformation::Response> res);
	void queryobjects_service( const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<rhome_metadata::srv::Queryobject::Request> req, std::shared_ptr<rhome_metadata::srv::Queryobject::Response> res);


	// - - - - -  Functions  - - - - - - - - - -
	bool load_database();
	cv::Mat get_histogram(cv::Mat1b const& image);
	cv::Mat extract_dscr(cv::Mat image_input, std::vector<cv::KeyPoint> &kp);
	std::vector<int>  matching_db(cv::Mat dscr_input, std::vector< std::vector<cv::DMatch> > &matches);
	bool intersection(cv::Point o1, cv::Point p1, cv::Point o2, cv::Point p2);
	void remove_duplicates(std::vector<box_obj> &objs, std::vector<cv::Rect> &objs_rects);
	void point_to_position(cv::Point point2d, geometry_msgs::msg::Point &point3d);
	geometry_msgs::msg::PoseStamped point2d_to_pose(cv::Point point_in);
	void publish_objects();
};

} /* namespace  */
#endif /* INSTANCERECOGNITION_H_ */



