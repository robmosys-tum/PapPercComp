// --------------------------------------------------------
// Code generated by Papyrus C++
// --------------------------------------------------------

#define Fruit_detectionCompdef_FruitDetection_impl_BODY

/************************************************************
 FruitDetection_impl class body
 ************************************************************/

// include associated header file
#include "Fruit_detectionCompdef/FruitDetection_impl.h"

// Derived includes directives
#include "rclcpp/rclcpp.hpp"

static auto img_msg = sensor_msgs::msg::Image();

namespace Fruit_detectionCompdef {

// static attributes (if any)

/**
 * 
 * @param options 
 */
FruitDetection_impl::FruitDetection_impl(rclcpp::NodeOptions /*in*/options) :
		FruitDetection(options) {
			this->detectionClient = this->create_client<fruit_detection::srv::Detection>("DetectionService");	
			this->diseaseClient = this->create_client<fruit_detection::srv::Classification>("DiseaseService");
			while (!this->diseaseClient->wait_for_service(std::chrono::seconds(5))) {
				if (!rclcpp::ok()) {
					RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
					rclcpp::shutdown();
				}
				RCLCPP_INFO(this->get_logger(), "Disease service not available, waiting again...");
  			}
			while (!this->detectionClient->wait_for_service(std::chrono::seconds(5))) {
				if (!rclcpp::ok()) {
					RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
					rclcpp::shutdown();
				}
				RCLCPP_INFO(this->get_logger(), "Detection service not available, waiting again...");
  			}
}

/**
 * 
 * @param image 
 */
void FruitDetection_impl::FruitDetectionHandler(
		const sensor_msgs::msg::Image::SharedPtr /*in*/image) {
			img_msg = *image;
			//cv::Mat cvImage(img_msg.height, img_msg.width, CV_8UC3, img_msg.data.data());
			cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
			const cv::Mat imageCopy = cv_ptr->image;
				
			if(!imageCopy.empty() && this->test) {
				RCLCPP_INFO(this->get_logger(), "Non empty image received");
				char c = (char)cv::waitKey(10);
				if( c == 27 || c == 'q' || c == 'Q' ) {
					rclcpp::shutdown();
				}
				RCLCPP_INFO(this->get_logger(), "Starting Object Detection");
				auto im = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::RGB8, cv::imread("/home/phil/Downloads/1.png"));
				auto ptr = std::make_shared<cv_bridge::CvImage>(im);
				this->detectFruits(ptr);
				this->test=false;
				//TODO publish correct data
				// std_msgs::msg::String msg;
				// msg.data = diseases[0];
				// Disease_pub_->publish(msg);
			}


}

/**
 * 
 * @param cv_ptr
 * @param boxes
 */
void FruitDetection_impl::classifyDisease(cv_bridge::CvImagePtr cv_ptr, std::vector<fruit_detection::msg::ClassBox> boxes){
	if(!this->diseaseClient->wait_for_service(std::chrono::seconds(1))){
		RCLCPP_ERROR(this->get_logger(), "Classification Service not available, shutting down");
		rclcpp::shutdown();
	}
	auto request = std::make_shared<fruit_detection::srv::Classification::Request>();
	request->img = *cv_ptr->toImageMsg();
	request->boxes = boxes;
	RCLCPP_INFO(this->get_logger(), "Sending Request");
	this->diseaseClient->async_send_request(request, [this, cv_ptr](rclcpp::Client<fruit_detection::srv::Classification>::SharedFuture future){
		std::vector<fruit_detection::msg::ClassBox> boxes= future.get()->new_boxes;
		this->drawAndPublish(cv_ptr, boxes);
	});
}

/**
 * 
 *@param cv_ptr 
 */
void FruitDetection_impl::detectFruits(cv_bridge::CvImagePtr cv_ptr){
	if(!this->detectionClient->wait_for_service(std::chrono::seconds(1))){
		RCLCPP_ERROR(this->get_logger(), "Detection service not available, shutting down");
		rclcpp::shutdown();
	}
	auto request = std::make_shared<fruit_detection::srv::Detection::Request>();
	request->img = *cv_ptr->toImageMsg();
	RCLCPP_INFO(this->get_logger(), "Sending detection request");
	this->detectionClient->async_send_request(request, [this, cv_ptr](rclcpp::Client<fruit_detection::srv::Detection>::SharedFuture future){
		RCLCPP_INFO(this->get_logger(), "Received detection results");
		std::vector<fruit_detection::msg::ClassBox> boxes= future.get()->classes;
		RCLCPP_INFO(this->get_logger(), "Start disease classification");
		this->classifyDisease(cv_ptr, boxes);
	});
}

void FruitDetection_impl::drawAndPublish(cv_bridge::CvImagePtr cv_ptr, std::vector<fruit_detection::msg::ClassBox> boxes){
	cv::Mat img = cv_ptr->image;
	cv::Size sz = img.size();
	int imageWidth = sz.width;
	int imageHeight = sz.height;
	for(auto box: boxes){
		cv::Scalar color(0, 255, 0);
		cv::Point top_left(box.xmin * imageWidth, box.ymax * imageHeight);
		cv::Point bottom_right(box.xmax * imageWidth, box.ymin * imageHeight);
		cv::rectangle(img, top_left, bottom_right, color, 3);

		std::string label = "";
		label.append(box.fruit.data());
		label.append(": ");
		label.append(std::to_string(box.fruit_score));
		label.append("%, ");
		label.append(box.disease.data());
		label.append(": ");
		label.append(std::to_string(box.disease_score));
		label.append("%");
		int fontFace = cv::FONT_HERSHEY_SIMPLEX;
		double fontScale = 0.8;
		int thickness = 2;
		int baseline=0;
		cv::Size textSize = cv::getTextSize(label, fontFace,
                            fontScale, thickness, &baseline);
		cv::Point textOrg(box.xmin * imageWidth, (box.ymin * imageHeight) + textSize.height);
		putText(img, label, textOrg, fontFace, fontScale, color, thickness, 0);

	}
	RCLCPP_INFO(this->get_logger(), "Drawing Image");
	cv::imshow("LABELED", img);
	cv::waitKey(0);
}
} // of namespace Fruit_detectionCompdef

/************************************************************
 End of FruitDetection_impl class body
 ************************************************************/
