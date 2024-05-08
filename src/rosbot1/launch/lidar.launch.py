import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode


def generate_launch_description():
    # Lifecycle manager configuration file
    lc_mgr_config_path = os.path.join(
        get_package_share_directory("ldlidar_node"), "params", "lifecycle_mgr.yaml"
    )

    lc_mgr_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager",
        output="screen",
        parameters=[
            # YAML files
            lc_mgr_config_path  # Parameters
        ],
    )

    lidar_config_path = os.path.join(
        get_package_share_directory('ldlidar_node'),
        'params',
        'ldlidar.yaml'
    )

    ldlidar_node = LifecycleNode(
        package="ldlidar_node",
        executable="ldlidar_node",
        name="ldlidar_node",
        namespace="",
        output="screen",
        parameters=[
            # YAML files
            lidar_config_path  # Parameters
        ],
    )

    return LaunchDescription([lc_mgr_node, ldlidar_node])
