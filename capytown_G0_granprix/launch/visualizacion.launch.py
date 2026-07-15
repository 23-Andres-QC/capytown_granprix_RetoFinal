#!/usr/bin/env python3
"""
visualizacion.launch.py - Lanza el emisor web para visualizar desde la laptop.

El render ya no corre en la Pi: este launch expone los datos en /data, arranca
el detector rojo/amarillo/verde, el registro de métricas y el aviso sonoro para
mantener el flujo de trabajo de 3 comandos.
Abrir web/index.html en la laptop y conectar a http://<IP-del-carrito>:8080/data.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('capytown_g0_granprix')
    pare_params = os.path.join(share, 'config', 'pare_params.yaml')
    metricas_params = os.path.join(share, 'config', 'metricas_params.yaml')

    return LaunchDescription([
        # Detector rojo/amarillo/verde. También publica la secuencia del buzzer
        # en /beep. Se inicia aquí para que el
        # usuario no necesite ejecutar un cuarto comando/terminal.
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
        # Emisor JSON liviano. El frontend se abre en la laptop.
        Node(
            package='capytown_g0_granprix',
            executable='visualizador_web',
            name='visualizador_web',
            output='screen',
            # Mostrar la máscara/rectángulo del detector, no la imagen cruda.
            parameters=[{'topic_camera': '/pare/debug_image'}],
        ),
    ])
