from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'cable_robot_description'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            glob('launch/*.launch.py')),
        ('share/' + package_name + '/config',
            glob('config/*.json')),
        ('share/' + package_name + '/urdf',
            glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Cable robot URDF builder package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gui_node = cable_robot_description.robot_builer.gui_node:main',
            'urdf_generator = cable_robot_description.robot_builer.urdf_generator:main',
        ],
    },
)