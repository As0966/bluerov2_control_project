from setuptools import setup
import os
from glob import glob

package_name = 'bluerov2_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name, f'{package_name}.controllers'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bayisa',
    maintainer_email='bayisa@todo.todo',
    description='BlueROV2 controllers for Gazebo Harmonic simulation',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'controller_node = bluerov2_control.controller_node:main',
            'mission_control = bluerov2_control.mission_control:main',
            'wrench_forwarder = bluerov2_control.wrench_forwarder:main',
            'live_plotter = bluerov2_control.live_plotter:main',
        ],
    },
)
