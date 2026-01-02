import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_bag = LaunchConfiguration("use_bag")
    bag_path = LaunchConfiguration("bag_path")
    bag_loop = LaunchConfiguration("bag_loop")
    use_depth = LaunchConfiguration("use_depth")

    nav2_params = LaunchConfiguration("nav2_params")
    nav2_map = LaunchConfiguration("nav2_map")
    gazebo_world = LaunchConfiguration("gazebo_world")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_yaw = LaunchConfiguration("spawn_yaw")

    limo_car_share = get_package_share_directory("limo_car")
    limo_bringup_share = get_package_share_directory("limo_bringup")
    datasync_share = get_package_share_directory("datasync_2_0")

    default_map = os.path.join(limo_bringup_share, "maps", "mapAZ.yaml")
    default_nav2 = os.path.join(limo_bringup_share, "param", "nav2_ackermann.yaml")
    default_world = os.path.join(limo_car_share, "worlds", "empty_world.model")

    ackermann_state = os.path.join(limo_car_share, "launch", "ackermann.launch.py")
    ackermann_nav2 = os.path.join(
        limo_bringup_share, "launch", "humble", "limo_nav2_ackermann.launch.py"
    )
    event_pipeline = os.path.join(datasync_share, "launch", "rosbag_pipeline.launch.py")
    gazebo_launch = os.path.join(
        get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py"
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            "limo_ackermann",
            "-x",
            spawn_x,
            "-y",
            spawn_y,
            "-z",
            spawn_z,
            "-Y",
            spawn_yaw,
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulated time (Gazebo / rosbag)",
            ),
            DeclareLaunchArgument(
                "use_bag",
                default_value="true",
                description="Play rosbag instead of live DAVIS driver",
            ),
            DeclareLaunchArgument(
                "bag_path",
                default_value="/home/nochi/NOCHI/M2_PAR/Projet_de_synthese/Ros_bags/Event_datas/rosbag2_2025_12_19-13_54_18",
                description="Path to rosbag2 to play when use_bag is true",
            ),
            DeclareLaunchArgument(
                "bag_loop",
                default_value="false",
                description="Loop rosbag playback",
            ),
            DeclareLaunchArgument(
                "use_depth",
                default_value="false",
                description="Enable MiDaS depth estimation",
            ),
            DeclareLaunchArgument(
                "nav2_params",
                default_value=default_nav2,
                description="Nav2 ackermann params file",
            ),
            DeclareLaunchArgument(
                "nav2_map",
                default_value=default_map,
                description="Nav2 map YAML",
            ),
            DeclareLaunchArgument(
                "gazebo_world",
                default_value=default_world,
                description="Gazebo world file",
            ),
            DeclareLaunchArgument("spawn_x", default_value="0.0"),
            DeclareLaunchArgument("spawn_y", default_value="0.0"),
            DeclareLaunchArgument("spawn_z", default_value="0.0"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(ackermann_state),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gazebo_launch),
                launch_arguments={
                    "world": gazebo_world,
                }.items(),
            ),
            spawn_entity,
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(ackermann_nav2),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "map": nav2_map,
                    "params_file": nav2_params,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(event_pipeline),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "use_bag": use_bag,
                    "bag_path": bag_path,
                    "bag_loop": bag_loop,
                    "use_depth": use_depth,
                    "use_rviz": "false",
                }.items(),
            ),
        ]
    )
