from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('robot_bringup'),
        'config',
        'bridge.yaml'
    )

    return LaunchDescription([
        Node(
            package='robot_bridge',
            executable='bridge_node',
            name='robot_bridge',
            parameters=[config],
            output='screen'
        ),
        # Later: add joystick, teleop, controllers here.
    ])
