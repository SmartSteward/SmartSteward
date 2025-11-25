from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'motion_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SmartSteward Team',
    maintainer_email='smartsteward@example.com',
    description='Motion control package for SmartSteward robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motion_controller_node = motion_control.motion_controller_node:main',
        ],
    },
)
