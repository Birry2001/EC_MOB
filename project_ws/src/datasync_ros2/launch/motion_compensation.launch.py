from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('datasync_ros2')
    default_params = os.path.join(pkg_share, 'config', 'davis346.yaml')

    params_file = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='YAML parameter file for the motion compensation node'
    )

    node = Node(
        package='datasync_ros2',
        executable='motion_compensation_node',
        name='motion_compensation_node',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    return LaunchDescription([
        params_file,
        node,
    ])
