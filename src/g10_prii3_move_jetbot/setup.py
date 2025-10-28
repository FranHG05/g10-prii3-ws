from setuptools import setup

package_name = 'g10_prii3_move_jetbot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='marolc',
    maintainer_email='molccom@epsa.upv.es',
    description='Movimientos y control del JetBot para dibujar el 10 con pausas y evitación de obstáculos',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'draw_number = g10_prii3_move_jetbot.draw_number:main',
            'collision_avoidance = g10_prii3_move_jetbot.collision_avoidance:main',
        ],
    },
)
