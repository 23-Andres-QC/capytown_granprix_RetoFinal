"""Launch principal del CapyTown Gran Prix.

Uso:
    ros2 launch capytown_g0_granprix granprix.launch.py

El bringup de la base, LiDAR y camara va aparte segun el robot.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def _share_path(*parts):
    return os.path.join(get_package_share_directory('capytown_g0_granprix'), *parts)


def generate_launch_description():
    nav_params = _share_path('config', 'navegacion_params.yaml')
    pare_params = _share_path('config', 'pare_params.yaml')
    metricas_params = _share_path('config', 'metricas_params.yaml')

    maze_solver = Node(
        package='capytown_g0_granprix',
        executable='maze_solver',
        name='maze_solver',
        output='screen',
        parameters=[nav_params],
    )

    pare_detector = Node(
        package='capytown_g0_granprix',
        executable='pare_detector',
        name='pare_detector',
        output='screen',
        parameters=[pare_params],
    )

    metrics_logger = Node(
        package='capytown_g0_granprix',
        executable='metrics_logger',
        name='metrics_logger',
        output='screen',
        parameters=[metricas_params],
    )

    return LaunchDescription([
        pare_detector,
        metrics_logger,
        maze_solver,
    ])
