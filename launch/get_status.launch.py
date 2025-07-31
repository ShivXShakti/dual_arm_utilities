from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory    

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('dual_arm_utilities'),
        'config',
        'get_status.yaml'
    )
    return LaunchDescription([
        Node(
            package='dual_arm_utilities',
            executable='rough',
            name='status_node',
            output='screen',
            parameters=[config_file]  # relative to install space
        )
    ])
