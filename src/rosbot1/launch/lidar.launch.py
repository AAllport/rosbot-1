from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ldlidar',
            executable='ldlidar',
            name='ldlidar',
            output='screen',
            parameters=[
                {'serial_port': "/dev/serial1")},
                {'topic_name': "scan"},
                {'lidar_frame': "lidar_frame"},
                {'range_threshold': "0.005"}
            ]
        )
    ])
