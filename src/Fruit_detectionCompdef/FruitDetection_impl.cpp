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
std::vector<std::tuple<std::string, int, int, int, int>> detectFruits(cv::Mat &img);
std::string classifyDisease(cv::Mat &img);
void drawBox(cv::Mat &img, std::tuple<std::string, int, int, int, int> boxes,  std::string);


namespace Fruit_detectionCompdef {

// static attributes (if any)

/**
 * 
 * @param options 
 */
FruitDetection_impl::FruitDetection_impl(rclcpp::NodeOptions /*in*/options) :
		FruitDetection(options) {
}

/**
 * 
 * @param image 
 */
void FruitDetection_impl::FruitDetectionHandler(
		const sensor_msgs::msg::Image::SharedPtr /*in*/image) {

			img_msg = *image;
			cv::Mat cvImage(img_msg.height, img_msg.width, CV_8UC3, img_msg.data.data());
			cv::Mat imageCopy = cvImage;
			if(!imageCopy.empty()) {
				RCLCPP_INFO(this->get_logger(), "Non empty image received");
				char c = (char)cv::waitKey(10);
				if( c == 27 || c == 'q' || c == 'Q' ) {
					rclcpp::shutdown();
				}

				RCLCPP_INFO(this->get_logger(), "Starting Object Detection");
				auto boxes = detectFruits(imageCopy);
				std::vector<std::string> diseases;
				RCLCPP_INFO(this->get_logger(), "Detection Completed");

				RCLCPP_INFO(this->get_logger(), "Starting Disease Classification");
				for(auto const & box : boxes){
					cv::Mat singleFruitImage = imageCopy(cv::Rect(std::get<1>(box),std::get<2>(box),std::get<3>(box),std::get<4>(box)));
					diseases.push_back(classifyDisease(singleFruitImage));
				}
				RCLCPP_INFO(this->get_logger(), "Classification Complete");
				for(std::vector<int>::size_type i = 0; i != boxes.size(); i++) {
				   drawBox(imageCopy, boxes[i], diseases[i]);
				}
				//TODO publish correct data
				std_msgs::msg::String msg;
				msg.data = diseases[0];
				Disease_pub_->publish(msg);
			}


}

} // of namespace Fruit_detectionCompdef

/************************************************************
 End of FruitDetection_impl class body
 ************************************************************/
//SHOULD RETURN list of (label, xmin, xmax, ymin, ymax)
std::vector<std::tuple<std::string, int, int, int, int>> detectFruits(cv::Mat &img){
	auto boundingBoxes = std::vector<std::tuple<std::string, int, int, int, int>>();
	return boundingBoxes;
}

//should RETURN disease label of cropped label
std::string classifyDisease(cv::Mat &img){
	std::string diseaseClass = "";
	return diseaseClass;
}

//should draw boxes on each detected fruit and add class/disease description
void drawBox(cv::Mat &img, std::tuple<std::string, int, int, int, int> boxes,  std::string){
	auto test = 0;
}

