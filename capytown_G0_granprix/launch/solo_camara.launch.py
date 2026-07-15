#!/usr/bin/env python3
"""Módulo solo_camara.launch."""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    """Ejecuta generate launch description."""
    matar_camaras_previas = ExecuteProcess(
        cmd=['bash', '-lc', 'pkill -f "[u]sb_cam_node_exe" || true'],
        output='screen',
    )

    camara = Node(
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
    )


    fijar_servo = TimerAction(period=2.0, actions=[
        ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '-t', '3', '/servo_s1',
                 'std_msgs/msg/Int32', '{data: 0}'],
            output='screen'),
        ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '-t', '3', '/servo_s2',
                 'std_msgs/msg/Int32', '{data: -5}'],
            output='screen'),
    ])

    return LaunchDescription([
        matar_camaras_previas,
        TimerAction(period=1.0, actions=[camara]),
        fijar_servo,
    ])
