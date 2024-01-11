// ROS2 headers
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

// OpenCV headers
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

// Dynamixel headers
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"

// FPGA IP block headers
#include "drivers/xnn_inference.h"

#define IMG_SIZE 32*24

class ImagePublisherNode
	: public rclcpp::Node
{
public:
	using SetPosition = dynamixel_sdk_custom_interfaces::msg::SetPosition;
	using GetPosition = dynamixel_sdk_custom_interfaces::srv::GetPosition;

	ImagePublisherNode();
	~ImagePublisherNode();

private:

	/**
	 * @brief Publishes a desired angle to the dynamixel control node.
	 *
	 * The motor rotation is defined in range 0 - 300 degrees. Thus the angle should
	 * be in this interval.
	 *
	 * @param id Id of the motor to control.
	 * @param anlge Whished final rotation of the motor
	 */
	void publishMotorRotation(int id, int anlge);
	int detectScrewType(cv::Mat frame);

	int m_idx;
	XNn_inference m_inference;
	cv::VideoCapture m_capture;
	rclcpp::TimerBase::SharedPtr m_timer;
	rclcpp::Publisher<SetPosition>::SharedPtr m_motorPositionPublsher;
	std::shared_ptr<cv_bridge::CvImage> m_cvBridge;
};

