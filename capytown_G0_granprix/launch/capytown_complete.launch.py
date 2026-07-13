#!/usr/bin/env python3
"""
capytown_complete.launch.py - Lanza el sistema completo:
  1. maze_solver (nodo maestro de navegación)
  2. usb_cam (cámara en vivo)
  3. visualizador_web (emisor JSON para web/index.html en la laptop)
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # ── Nodo maestro de navegación ────────────────────────────────────
        Node(
            package='capytown_g0_granprix',
            executable='maze_solver',
            name='maze_solver',
            output='screen',
        ),

        # ── Cámara USB (yuyv: sin decodificar, menor latencia que mjpeg2rgb) ──
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

        # ── Emisor web (la laptop abre web/index.html y hace fetch a /data) ──
        Node(
            package='capytown_g0_granprix',
            executable='visualizador_web',
            name='visualizador_web',
            output='screen',
        ),
    ])
