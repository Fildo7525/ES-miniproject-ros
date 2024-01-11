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

private:
	void publishImage();

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

	XNn_inference inf;
	cv::VideoCapture capture;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<SetPosition>::SharedPtr motorPositionPublsher_;
	rclcpp::Publisher<GetPosition>::SharedPtr motorPositionSubscriber_;
	std::shared_ptr<cv_bridge::CvImage> cv_bridge_;
};

