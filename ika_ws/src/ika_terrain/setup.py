import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ika_terrain'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='aslan',
    maintainer_email='aslan@todo.todo',
    description='IKA - Terrain Perception Node',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'terrain_perception_node = ika_terrain.terrain_perception_node:main',
        ],
    },
)
