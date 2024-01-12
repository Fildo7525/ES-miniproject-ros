from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		Node(
			package='dynamixel_sdk_examples',
			executable='read_write_node',
			output='screen',
			emulate_tty=True
		),
		Node(
			package='usb_cam',
			executable='usb_cam_node_exe',
			output='screen',
			emulate_tty=True
		),
		Node(
			package='miniproject',
			executable='main',
			output='screen',
			emulate_tty=True
		)
	])

