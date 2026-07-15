#!/usr/bin/env python3
"""Módulo visualizacion.launch."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Ejecuta generate launch description."""
    share = get_package_share_directory('capytown_g0_granprix')
    pare_params = os.path.join(share, 'config', 'pare_params.yaml')
    metricas_params = os.path.join(share, 'config', 'metricas_params.yaml')

    return LaunchDescription([


        Node(
            package='capytown_g0_granprix',
            executable='pare_detector',
            name='pare_detector',
            output='screen',
            parameters=[pare_params],
        ),
        Node(
            package='capytown_g0_granprix',
            executable='metrics_logger',
            name='metrics_logger',
            output='screen',
            parameters=[metricas_params],
        ),

        Node(
            package='capytown_g0_granprix',
            executable='visualizador_web',
            name='visualizador_web',
            output='screen',

            parameters=[{'topic_camera': '/pare/debug_image'}],
        ),
    ])
