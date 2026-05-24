from setuptools import find_packages, setup

package_name = 'ika_base_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='aslan',
    maintainer_email='aslan@todo.todo',
    description='IKA - Pi tarafi seri kopru, Arduino motor surucu',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'base_controller_node = ika_base_controller.base_controller_node:main',
        ],
    },
)
