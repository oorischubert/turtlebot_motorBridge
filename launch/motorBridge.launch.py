from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlebot_motorBridge',
            namespace='motorBridge',
            executable='diffdrive_turtlebot',
            name='diffdrive_turtlebot',
            parameters=[{'device': '/dev/ttyUSB_esp32'}],
            output='screen',
            respawn=False,
            respawn_delay=2.0,
        ),
    ])

# ros2 run turtlebot_motorBridge low_level_bridge --ros-args -p device:=/dev/ttyUSB0