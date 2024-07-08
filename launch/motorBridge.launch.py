from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlebot_motorBridge',
            namespace='motorBridge',
            executable='low_level_bridge',
            name='low_level_bridge',
            parameters=[{'device': '/dev/ttyUSB0'}],
            output='screen',
            respawn=False,
            respawn_delay=2.0,
        ),
    ])

# ros2 run turtlebot_motorBridge low_level_bridge --ros-args -p device:=/dev/ttyUSB0