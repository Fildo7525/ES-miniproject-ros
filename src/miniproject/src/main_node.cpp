#include "main_node.hpp"
#include <algorithm>
#include <iterator>
#include <vector>
#include <iostream>

// This represents the maximal motor rotation.
// 0.29 degrees per 1 impulse.
// Angles 300 - 360 are in invalid range.
#define MOTRO_ANGLES_TO_IMPULZES (1/0.29)
#define BASE_MOTOR_ID 1
#define CAMERA_MOTOR_ID 2

ImagePublisherNode::ImagePublisherNode()
	: rclcpp::Node("main_node")
	, m_idx(1)
{
	int status = XNn_inference_Initialize(&m_inference, "nn_inference");
	if (status != XST_SUCCESS) {
		std::cout << "Could not initialize IP block.\n";
		exit(1);
	}


	// publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/image_raw", 10);

	m_cameraSubscriber = this->create_subscription<sensor_msgs::msg::Image>(
		"/image_raw", 10, std::bind(&ImagePublisherNode::getImageCallback, this, std::placeholders::_1));

	m_publisher = this->create_publisher<sensor_msgs::msg::Image>("/processed_img", 10);

	m_cvBridge = std::make_shared<cv_bridge::CvImage>();
	m_motorPositionPublsher = this->create_publisher<SetPosition>("/set_position", 10);
	m_timer = this->create_wall_timer(std::chrono::seconds(1), [this] () { this->publishMotorRotation(BASE_MOTOR_ID, 0); });
	RCLCPP_INFO(this->get_logger(), "Constructed");
}

ImagePublisherNode::~ImagePublisherNode()
{
	RCLCPP_INFO(this->get_logger(), "Destroyed");
	XNn_inference_Release(&m_inference);
}

std::vector<float> ImagePublisherNode::flatten(cv::Mat image)
{
	// Flatten the image
	cv::Mat flattenedImage = image.reshape(1, 1);

	std::vector<float> flattenedVector(IMG_SIZE, 0);
	std::transform(flattenedImage.begin<char>(), flattenedImage.end<char>(), flattenedVector.begin(), [](char c) -> float {return c/255.;});

	return flattenedVector;
}

void ImagePublisherNode::publishImage(const cv::Mat &frame)
{
	// Publish the ROS Image message to the /image_raw topic
	auto ros_image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", frame).toImageMsg();
	m_publisher->publish(*ros_image_msg);
}

void ImagePublisherNode::getImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
	cv::Mat tmp;
	// convert ros Image to cv Image.
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
	m_frame = cv_ptr->image;
	cv::resize(cv_ptr->image, tmp, {32, 24}, 0, 0, cv::INTER_AREA);
	cv::cvtColor(tmp, tmp, cv::COLOR_BGR2GRAY);
	m_detectedScrewType = detectScrewType(tmp);
	// RCLCPP_INFO(this->get_logger(), "Screw type: %s", getName(m_detectedScrewType).c_str());
}

ImagePublisherNode::SetPosition constructMsg(int id, int angle)
{
	SetPosition msg;
	msg.id = id;
	msg.position = angle;
	return msg;
}

void ImagePublisherNode::findRotation()
{
	for (int i = 0; i < 128; i++) {
		m_motorPositionPublsher->publish(constructMsg(BASE_MOTOR_ID, i*8));
		if (checkIfFits(m_frame)) {
			m_motorPositionPublsher->publish(constructMsg(BASE_MOTOR_ID, (i-1)*8));
			return;
		}
	}
}

void ImagePublisherNode::publishMotorRotation(int id, int angle)
{
	m_timer->cancel();

	for (int i = 4; i < 7; i++) {
		m_motorPositionPublsher->publish(constructMsg(BASE_MOTOR_ID, i*80));

		auto screwType = detectScrewType(m_frame);
		if (screwType == ScrewType::Hexagonal || screwType == ScrewType::Nut) {
			findRotation();
			RCLCPP_INFO(this->get_logger(), "Rotation found");
			sleep(1);
		}
		else {
			RCLCPP_INFO(this->get_logger(), "Incorrect type of screw");
		}
	}
}


ImagePublisherNode::ScrewType ImagePublisherNode::detectScrewType(cv::Mat frame)
{
	auto check = flatten(frame);

	while (!XNn_inference_IsReady(&m_inference)) {}
	XNn_inference_Write_Data_In_Words(&m_inference, 0, (word_type *)check.data(), IMG_SIZE);

	XNn_inference_Start(&m_inference);

	while (!XNn_inference_IsDone(&m_inference)) {}

	return static_cast<ScrewType>(XNn_inference_Get_Pred_out(&m_inference));
}

std::string ImagePublisherNode::getName(ScrewType screwType)
{
	switch (screwType) {
		case ScrewType::Hexagonal:
			return "HEXAGONAL";
		case ScrewType::Nut:
			return "NUT";
		case ScrewType::Philips:
			return "PHILIPS";
		default:
			return "None";
	}
}

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);
	auto node = std::make_shared<ImagePublisherNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}

