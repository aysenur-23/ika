import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ika_mission'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'missions'), glob('missions/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'pyyaml'],
    zip_safe=True,
    maintainer='aslan',
    maintainer_email='aslan@todo.todo',
    description='IKA - Mission Manager (GPS waypoints)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_waypoint_mission = ika_mission.gps_waypoint_mission:main',
            'obstacle_avoider = ika_mission.obstacle_avoider_node:main',
        ],
    },
)
