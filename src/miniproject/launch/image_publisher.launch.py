from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
	joy_config = LaunchConfiguration('joy_config', default='ps3')
	# according to the teleop_twist_joy documentatino the parameter default can be one of: atk3, ps3-holonomic, ps3, xbox, xd3.

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

