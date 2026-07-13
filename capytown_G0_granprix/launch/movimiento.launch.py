#!/usr/bin/env python3
"""
movimiento.launch.py - Lanza maze_solver + cámara USB
Controla el movimiento del robot y detecta PARE
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def _share_path(*parts):
    return os.path.join(get_package_share_directory('capytown_g0_granprix'), *parts)


def generate_launch_description():
    return LaunchDescription([
        # Nodo maestro de navegación
        Node(
            package='capytown_g0_granprix',
            executable='maze_solver',
            name='maze_solver',
            output='screen',
            parameters=[_share_path('config', 'navegacion_params.yaml')],
        ),

        # Cámara USB (yuyv: sin decodificar, menor latencia que mjpeg2rgb)
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            output='screen',
            remappings=[('/usb_cam/image_raw', '/image_raw')],
            parameters=[{
                'video_device': '/dev/video0',
                'image_width': 640,
                'image_height': 480,
                'pixel_format': 'yuyv',
                'framerate': 30.0,
                'brightness': 50,
            }],
        ),
    ])
