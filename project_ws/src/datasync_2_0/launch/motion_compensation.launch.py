from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")

    default_params = os.path.join(
        get_package_share_directory("datasync_2_0"),
        "config",
        "motion_compensation.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Path to motion compensation parameters",
            ),
            Node(
                package="datasync_2_0",
                executable="Motion_Compensation",
                name="motion_compensation",
                output="screen",
                parameters=[params_file],
            ),
        ]
    )
