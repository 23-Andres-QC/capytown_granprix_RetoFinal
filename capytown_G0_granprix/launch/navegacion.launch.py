"""Launch minimo: maze_solver + pare_detector + metrics_logger."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def _share_path(*parts):
    return os.path.join(get_package_share_directory('capytown_g0_granprix'), *parts)


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='capytown_g0_granprix',
            executable='pare_detector',
            name='pare_detector',
            output='screen',
            parameters=[_share_path('config', 'pare_params.yaml')],
        ),
        Node(
            package='capytown_g0_granprix',
            executable='metrics_logger',
            name='metrics_logger',
            output='screen',
            parameters=[_share_path('config', 'metricas_params.yaml')],
        ),
        Node(
            package='capytown_g0_granprix',
            executable='maze_solver',
            name='maze_solver',
            output='screen',
            parameters=[_share_path('config', 'navegacion_params.yaml')],
        ),
    ])
