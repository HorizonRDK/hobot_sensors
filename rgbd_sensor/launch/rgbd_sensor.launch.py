from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 启动图片发布pkg
        Node(
            package='rgbd_sensor',
            executable='rgbd_sensor',
            output='screen',
            parameters=[
                {"io_method": "ros"}
            ],
            arguments=['--ros-args', '--log-level', 'error']
        )
    ])
