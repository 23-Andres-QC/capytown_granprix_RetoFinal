import os
from glob import glob

from setuptools import setup


package_name = 'capytown_g0_granprix'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'docs'), glob('docs/*')),
        (os.path.join('share', package_name, 'web'), glob('web/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Grupo 0',
    maintainer_email='grupo0@example.com',
    description='CapyTown Gran Prix: navegacion autonoma de laberinto con fusion LiDAR + camara PARE.',
    license='MIT',
    entry_points={
        'console_scripts': [

            'maze_solver = capytown_g0_granprix.maze_solver:main',
            'pare_detector = capytown_g0_granprix.pare_detector:main',
            'metrics_logger = capytown_g0_granprix.metrics_logger:main',
            'visualizador_web = capytown_g0_granprix.visualizador_web:main',
        ],
    },
)
