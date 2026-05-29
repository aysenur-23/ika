import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ika_perception_dl'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aslan',
    maintainer_email='aslan@todo.todo',
    description='IKA - DL nesne tespiti (OAK-D Lite VPU spatial detection)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dl_perception_node = ika_perception_dl.dl_perception_node:main',
            'sim_detection_node = ika_perception_dl.sim_detection_node:main',
        ],
    },
)
