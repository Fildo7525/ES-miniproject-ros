#include "img_pub.hpp"
#include <vector>
#include <iostream>

// This represents the maximal motor rotation.
// 0.29 degrees per 1 impulse.
// Angles 300 - 360 are in invalid range.
#define MOTRO_ANGLES_TO_IMPULZES (1/0.29)
#define BASE_MOTOR_ID 1
#define CAMERA_MOTOR_ID 2

ImagePublisherNode::ImagePublisherNode()
	: rclcpp::Node("image_publisher_node")
	, m_idx(4)
	// , capture(0)
{
	int status = XNn_inference_Initialize(&m_inference, "nn_inference");
	if (status != XST_SUCCESS) {
		std::cout << "Could not initialize IP block.\n";
		exit(1);
	}

	m_cvBridge = std::make_shared<cv_bridge::CvImage>();
	m_motorPositionPublsher = this->create_publisher<SetPosition>("/set_position", 10);
	m_timer = this->create_wall_timer(std::chrono::seconds(1), [this] () { this->publishMotorRotation(BASE_MOTOR_ID, 0); });
}

ImagePublisherNode::~ImagePublisherNode()
{
	XNn_inference_Release(&m_inference);
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

	while (!XNn_inference_IsReady(&m_inference)) {}
	XNn_inference_Write_Data_In_Words(&m_inference, 0, (word_type *)check.data(), IMG_SIZE);

	XNn_inference_Start(&m_inference);

	while (!XNn_inference_IsDone(&m_inference)) {}

	return XNn_inference_Get_Pred_out(&m_inference);
}

void ImagePublisherNode::publishMotorRotation(int id, int angle)
{
	m_timer->cancel();

	sleep(2);

	angle = m_idx * 80;
	m_idx++;

	SetPosition msg;
	msg.id = id;
	msg.position = angle;
	RCLCPP_INFO(this->get_logger(), "Publishing [ID: %d] [Goal Position: %d]", msg.id, msg.position);
	m_motorPositionPublsher->publish(msg);

	m_timer->reset();
}

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);
	auto node = std::make_shared<ImagePublisherNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}

