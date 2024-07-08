import os
import sys
from glob import glob
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions



def generate_launch_description():
    pkg_dir = get_package_share_directory('icp_slam')
    rviz_config_file = os.path.join(pkg_dir, 'rviz','test.rviz')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true')
    use_sim_time = LaunchConfiguration('use_sim_time')
    list = [
        use_sim_time_arg,
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_file],
            # parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                "--x",
                "0.0",
                "--y",
                "0.0",
                "--z",
                "0.0",
                "--yaw",
                "0.0",
                "--pitch",
                "0.0",
                "--roll",
                "0.0",
                "--frame-id",
                "base_link",
                "--child-frame-id",
                "tugbot/scan_omni/scan_omni",
            ],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='icp_slam',
            executable='icp_scan_matcher',
            namespace='',
            output="screen",
            parameters=[{'use_sim_time': use_sim_time}, os.path.join(pkg_dir, "config", "sim_icp_scan_matcher_param.yaml")],
            respawn=True,
        ),
        Node(
            package='icp_slam',
            executable='map_builder',
            namespace='',
            output="screen",
            parameters=[{'use_sim_time': use_sim_time}, os.path.join(pkg_dir, "config", "map_builder_param.yaml")],
            respawn=True,
        ),
        Node(
            package='icp_slam',
            executable='tf_broadcaster',
            namespace='',
            output="screen",
            parameters=[{'use_sim_time': use_sim_time}, os.path.join(pkg_dir, "config", "tf_broadcaster_param.yaml")],
            respawn=True,
        ),
    ]

    return LaunchDescription(list)