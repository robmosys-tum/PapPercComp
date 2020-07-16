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

namespace Fruit_detectionCompdef {
    // static attributes (if any)
static std::map<std::string, cv::Scalar> colorMap = {
    {"Fruit", cv::Scalar(0, 255, 0)},
    {"Apple", cv::Scalar(0, 0, 255)},
    {"Grape", cv::Scalar(255, 0, 0)},
    {"Common fig", cv::Scalar(255, 0, 255)},
    {"Pear", cv::Scalar(255, 255, 0)},
    {"Strawberry", cv::Scalar(0, 255, 255)},
    {"Tomato", cv::Scalar(147, 219, 112)},
    {"Lemon", cv::Scalar(66, 166, 181)},
    {"Banana", cv::Scalar(51, 115, 184)},
    {"Orange", cv::Scalar(0, 127, 255)},
    {"Peach", cv::Scalar(120, 31, 135)},
    {"Mango", cv::Scalar(23, 23, 140)},
    {"Pineapple", cv::Scalar(38, 66, 107)},
    {"Grapefruit", cv::Scalar(255, 0, 0)},
    {"Pomegranate", cv::Scalar(222, 176, 56)},
    {"Watermelon", cv::Scalar(153, 50, 204)},
    {"Cantaloup", cv::Scalar(50, 127, 205)}
};

FruitDetection_impl::FruitDetection_impl(rclcpp::NodeOptions /*in*/ options) : FruitDetection(options) {
    this->detectionClient = this->create_client<fruit_detection::srv::Detection>("DetectionService");
    this->diseaseClient = this->create_client<fruit_detection::srv::Classification>("DiseaseService");
    while (!this->diseaseClient->wait_for_service(std::chrono::seconds(5))) {
        if (!rclcpp::ok()) {
            rclcpp::shutdown();
        }
        RCLCPP_INFO(this->get_logger(),
                    "Disease service not available, waiting again...");
    }
    while (!this->detectionClient->wait_for_service(std::chrono::seconds(5))) {
        if (!rclcpp::ok()) {
            rclcpp::shutdown();
        }
        RCLCPP_INFO(this->get_logger(),
                    "Detection service not available, waiting again...");
    }
    RCLCPP_INFO(this->get_logger(), "Services Ready");
}

void FruitDetection_impl::FruitDetectionHandler(
    const sensor_msgs::msg::Image::SharedPtr /*in*/ image) {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    const cv::Mat imageCopy = cv_ptr->image;

    if (!imageCopy.empty()) {
        RCLCPP_INFO(this->get_logger(), "Non empty image received");
        char c = static_cast<char>(cv::waitKey(10));
        if (c == 27 || c == 'q' || c == 'Q') {
            rclcpp::shutdown();
        }
        RCLCPP_INFO(this->get_logger(), "Starting Object Detection");
        this->detectFruits(cv_ptr);
    } else {
        RCLCPP_INFO(this->get_logger(), "Image empty");
    }
}

void FruitDetection_impl::classifyDisease(cv_bridge::CvImagePtr cv_ptr,
                            std::vector<fruit_detection::msg::ClassBox> boxes) {
    if (!this->diseaseClient->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(this->get_logger(),
            "Classification Service not available, shutting down");
        rclcpp::shutdown();
    }
    auto request =
            std::make_shared<fruit_detection::srv::Classification::Request>();
    request->img = *cv_ptr->toImageMsg();
    request->boxes = boxes;
    RCLCPP_INFO(this->get_logger(), "Sending Request");
    this->diseaseClient->async_send_request(request, [this, cv_ptr]
    (rclcpp::Client<fruit_detection::srv::Classification>::SharedFuture future) {
        std::vector<fruit_detection::msg::ClassBox> boxes = future.get()->new_boxes;
        this->drawAndPublish(cv_ptr, boxes);
    });
}

void FruitDetection_impl::detectFruits(cv_bridge::CvImagePtr cv_ptr) {
    if (!this->detectionClient->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(this->get_logger(),
            "Detection service not available, shutting down");
        rclcpp::shutdown();
    }
    auto request = std::make_shared<fruit_detection::srv::Detection::Request>();
    request->img = *cv_ptr->toImageMsg();
    RCLCPP_INFO(this->get_logger(), "Sending detection request");
    this->detectionClient->async_send_request(request, [this, cv_ptr]
    (rclcpp::Client<fruit_detection::srv::Detection>::SharedFuture future) {
        RCLCPP_INFO(this->get_logger(), "Received detection results");
        std::vector<fruit_detection::msg::ClassBox> boxes = future.get()->classes;
        RCLCPP_INFO(this->get_logger(), "Start disease classification");
        this->classifyDisease(cv_ptr, boxes);
    });
}

void FruitDetection_impl::drawAndPublish(cv_bridge::CvImagePtr cv_ptr,
                            std::vector<fruit_detection::msg::ClassBox> boxes) {
    RCLCPP_INFO(this->get_logger(), "Publish Image");
    auto boxImage = fruit_detection::msg::BoxesImage();
    boxImage.img = *cv_ptr->toImageMsg();
    boxImage.boxes = boxes;
    this->BoxImage_pub_->publish(boxImage);
    RCLCPP_INFO(this->get_logger(), "Drawing Image");
    cv::Mat img = cv_ptr->image;
    cv::Size sz = cv::Size(512,512);
    int imageWidth = sz.width;
    int imageHeight = sz.height;
    cv::namedWindow("Labeled Image", cv::WINDOW_NORMAL);
    cv::resize(img, img, sz);
    RCLCPP_INFO(this->get_logger(), "Generate BoxImage");
    for (auto box : boxes) {
        auto color = colorMap[box.fruit.data()];
        cv::Point top_left(box.xmin * imageWidth, box.ymax * imageHeight);
        cv::Point bottom_right(box.xmax * imageWidth, box.ymin * imageHeight);
        cv::rectangle(img, top_left, bottom_right, color, 3);

        std::string label = createLabel(box);
        int fontFace = cv::FONT_HERSHEY_SIMPLEX;
        double fontScale = 0.5;
        int thickness = 1;
        int baseline = 0;
        cv::Size textSize = cv::getTextSize(label, fontFace,
                                            fontScale, thickness, &baseline);
        cv::Point textOrg(box.xmin * imageWidth,
                         (box.ymin * imageHeight) + textSize.height);
        putText(img, label, textOrg, fontFace, fontScale, color, thickness, 0);
    }
    cv::imshow("Labeled Image", img);
    cv::waitKey(5000);
}
}  // namespace Fruit_detectionCompdef

/************************************************************
 End of FruitDetection_impl class body
 ************************************************************/
