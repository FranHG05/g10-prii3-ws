from setuptools import setup
import os
from glob import glob

package_name = 'g10_prii3_move_turtlebot'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 🔹 Esto asegura que los archivos .launch.py se copien al instalar el paquete
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='marolc',
    maintainer_email='molccom@epsa.upv.es',
    description='Nodo para dibujar el número del grupo en Gazebo usando TurtleBot3.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'draw_number = g10_prii3_move_turtlebot.draw_number:main',
        ],
    },
)
