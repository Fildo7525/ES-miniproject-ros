#include "img_pub.hpp"
#include <vector>
#include <iostream>

// This represents the maximal motor rotation.
// 0.29 degrees per 1 impulse.
// Angles 300 - 360 are in invalid range.
#define MOTRO_ANGLES_TO_IMPULZES (1/0.29)

ImagePublisherNode::ImagePublisherNode()
	: rclcpp::Node("image_publisher_node")
	, capture(0)
{
	int status = XNn_inference_Initialize(&inf, "nn_inference");
	if (status != XST_SUCCESS) {
		std::cout << "Could not initialize IP block.\n";
		exit(1);
	}

	publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/image_raw", 10);
	// timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&ImagePublisherNode::publishImage, this));

	cv_bridge_ = std::make_shared<cv_bridge::CvImage>();
	motorPositionPublsher_ = this->create_publisher<SetPosition>("/set_position", 10);
	timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&ImagePublisherNode::publishMotorRotation, this));
}

std::vector<float> flatten(const cv::Mat &frame)
{
	if (frame.empty()) {
		std::cerr << "Failed to load the image!" << std::endl;
		return std::vector<float>();
	}

	// Flatten the image
	cv::Mat flattenedImage = frame.reshape(1, 1);

	// Copy the flattened image to a std::vector<float>
	std::vector<float> flattenedVector;
	flattenedVector.assign(flattenedImage.ptr<float>(0), flattenedImage.ptr<float>(0) + flattenedImage.cols * flattenedImage.rows);
	return flattenedVector;
}

int ImagePublisherNode::detectScrewType(cv::Mat frame)
{
	cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
	auto check = flatten(frame);

	while (!XNn_inference_IsReady(&inf)) {}
	XNn_inference_Write_Data_In_Words(&inf, 0, (word_type *)check.data(), IMG_SIZE);

	XNn_inference_Start(&inf);

	while (!XNn_inference_IsDone(&inf)) {}

	return XNn_inference_Get_Pred_out(&inf);
}

void ImagePublisherNode::publishImage()
{
	cv::Mat frame;
	capture >> frame;

	// Publish the ROS Image message to the /image_raw topic
	auto ros_image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
	publisher_->publish(*ros_image_msg);
	RCLCPP_INFO(this->get_logger(), "Image published");
}

void ImagePublisherNode::publishMotorRotation(int id, int angle)
{
	SetPosition msg;
	msg.id = id;
	msg.position = MOTRO_ANGLES_TO_IMPULZES * angle;
	motorPositionPublsher_->publish(msg);
}

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);
	auto node = std::make_shared<ImagePublisherNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}

