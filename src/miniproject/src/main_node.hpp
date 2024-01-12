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

	enum class ScrewType
	{
		// Screwable
		Hexagonal,

		// Screwable
		Nut,

		// Unscrewable
		Philips
	};
	using SetPosition = dynamixel_sdk_custom_interfaces::msg::SetPosition;
	using GetPosition = dynamixel_sdk_custom_interfaces::srv::GetPosition;

	ImagePublisherNode();
	~ImagePublisherNode();

private:

	std::vector<float> flatten(cv::Mat image);

	void publishImage(const cv::Mat &frame);

	void getImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

	bool checkIfFits(const cv::Mat& frame);

	SetPosition constructMsg(int id, int angle);

	void findScrewRotation();

	/**
	 * @brief Publishes a desired angle to the dynamixel control node.
	 *
	 * The motor rotation is defined in range 0 - 300 degrees. Thus the angle should
	 * be in this interval.
	 *
	 * @param id Id of the motor to control.
	 */
	void publishBaseMotorRotation(int id);

	/**
	 * @brief Invokes the IP in FPGA and checks if the screw on the image is screwable.
	 *
	 * @param frame Frame capture from camera.
	 * @return Type of the screw.
	 */
	ScrewType detectScrewType(cv::Mat frame);

	std::string getName(ScrewType screwType);

private:
	int m_idx;
	cv::Mat m_frame;
	XNn_inference m_inference;
	std::ofstream file;
	ScrewType m_detectedScrewType;

	rclcpp::TimerBase::SharedPtr m_timer;

	// Subscriber to the /image_raw topic that the camera node created.
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_cameraSubscriber;

	// Publishers
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_processedImagePublisher;
	rclcpp::Publisher<SetPosition>::SharedPtr m_motorPositionPublsher;
	std::shared_ptr<cv_bridge::CvImage> m_cvBridge;
};

