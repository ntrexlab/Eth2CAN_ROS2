import launch
import launch.actions
import launch_ros.actions


# eth2can_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_dir = get_package_share_directory('eth2can')

    return LaunchDescription([
        Node(
            package='eth2can',
            executable='eth2can_node',
            name='eth2can_node',
            output='screen',
            parameters=[package_dir + '/yaml/config.yaml'],  # Load the YAML file
        ),
    ])