from glob import glob
import os
from setuptools import setup

package_name = 'g10_prii3_nav_turtlebot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'models'), glob('models/**/*', recursive=True)),
        (os.path.join('share', package_name, 'worlds/maps'), glob('worlds/maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Marcos',
    maintainer_email='marcos@example.com',
    description='TurtleBot3 navigation package with custom world',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'autonomous_navigation = g10_prii3_nav_turtlebot.autonomous_navigation:main'
        ],
    },
)
