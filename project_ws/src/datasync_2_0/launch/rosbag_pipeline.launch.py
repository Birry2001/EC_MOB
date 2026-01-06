from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_bag = LaunchConfiguration("use_bag")
    motion_params = LaunchConfiguration("motion_params")
    segmentation_params = LaunchConfiguration("segmentation_params")
    clustering_params = LaunchConfiguration("clustering_params")
    tf_params = LaunchConfiguration("tf_params")
    bag_path = LaunchConfiguration("bag_path")
    bag_loop = LaunchConfiguration("bag_loop")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    use_depth = LaunchConfiguration("use_depth")
    depth_params = LaunchConfiguration("depth_params")

    libcaer_launch = os.path.join(
        get_package_share_directory("libcaer_driver"),
        "launch",
        "driver_node.launch.py",
    )

    datasync_share = get_package_share_directory("datasync_2_0")
    default_motion = os.path.join(datasync_share, "config", "motion_compensation.yaml")
    default_rviz = os.path.join(datasync_share, "rviz", "rosbag_pipeline.rviz")

    segmentation_share = get_package_share_directory("event_segmentation")
    default_seg = os.path.join(segmentation_share, "config", "segmentation.yaml")

    clustering_share = get_package_share_directory("event_clustering")
    default_cluster = os.path.join(clustering_share, "config", "clustering.yaml")

    tf_share = get_package_share_directory("event_tf_static")
    default_tf = os.path.join(tf_share, "config", "static_tf.yaml")

    depth_share = get_package_share_directory("event_depth_midas")
    default_depth = os.path.join(depth_share, "config", "depth_midas.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_bag",
                default_value="true",
                description="If true, do not start the camera driver",
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
                "use_sim_time",
                default_value="true",
                description="Use simulated time when playing a bag",
            ),
            DeclareLaunchArgument(
                "motion_params",
                default_value=default_motion,
                description="Motion compensation params file",
            ),
            DeclareLaunchArgument(
                "segmentation_params",
                default_value=default_seg,
                description="Segmentation params file",
            ),
            DeclareLaunchArgument(
                "clustering_params",
                default_value=default_cluster,
                description="Clustering params file",
            ),
            DeclareLaunchArgument(
                "tf_params",
                default_value=default_tf,
                description="Static TF params file",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Launch RViz for visualization",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz,
                description="RViz config file",
            ),
            DeclareLaunchArgument(
                "use_depth",
                default_value="false",
                description="Enable MiDaS depth estimation from APS image",
            ),
            DeclareLaunchArgument(
                "depth_params",
                default_value=default_depth,
                description="MiDaS depth params file",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(libcaer_launch),
                condition=UnlessCondition(use_bag),
                launch_arguments={
                    "device_type": "davis",
                }.items(),
            ),
            ExecuteProcess(
                cmd=["ros2", "bag", "play", bag_path, "--clock", "--loop"],
                condition=IfCondition(
                    PythonExpression(
                        ["'", use_bag, "' == 'true' and '", bag_loop, "' == 'true'"]
                    )
                ),
                output="screen",
            ),
            ExecuteProcess(
                cmd=["ros2", "bag", "play", bag_path, "--clock"],
                condition=IfCondition(
                    PythonExpression(
                        ["'", use_bag, "' == 'true' and '", bag_loop, "' == 'false'"]
                    )
                ),
                output="screen",
            ),
            Node(
                package="event_tf_static",
                executable="event_tf_static_node",
                name="event_tf_static",
                output="screen",
                parameters=[tf_params, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="datasync_2_0",
                executable="Motion_Compensation",
                name="motion_compensation",
                output="screen",
                parameters=[motion_params, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="event_segmentation",
                executable="event_segmentation_node",
                name="event_segmentation",
                output="screen",
                parameters=[segmentation_params, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="event_clustering",
                executable="event_clustering_node",
                name="event_clustering",
                output="screen",
                parameters=[clustering_params, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="event_depth_midas",
                executable="event_depth_midas_node",
                name="event_depth_midas",
                output="screen",
                parameters=[depth_params, {"use_sim_time": use_sim_time}],
                condition=IfCondition(use_depth),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                condition=IfCondition(use_rviz),
            ),
        ]
    )
