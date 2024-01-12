#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ImagePublisherNode : public rclcpp::Node {
public:
	ImagePublisherNode() : Node("image_publisher_node"), capture(0) {
		publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/image_raw", 10);
		timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&ImagePublisherNode::publishImage, this));

		cv_bridge_ = std::make_shared<cv_bridge::CvImage>();
	}

private:
	void publishImage() {
		// Capture a frame from the camera (replace this with your camera capturing logic)
		// For example, using OpenCV to capture from a camera with index 0:
		capture >> frame;

		// Convert the OpenCV frame to a ROS Image message
		auto ros_image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();

		// Publish the ROS Image message to the /image_raw topic
		publisher_->publish(*ros_image_msg);
		RCLCPP_INFO(this->get_logger(), "Image published");
	}

	cv::VideoCapture capture;
	cv::Mat m_frame;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
	rclcpp::TimerBase::SharedPtr timer_;
	std::shared_ptr<cv_bridge::CvImage> cv_bridge_;
};

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);
	auto node = std::make_shared<ImagePublisherNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}

