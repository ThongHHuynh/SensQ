import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'my_robot_docking'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml'),
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tom',
    maintainer_email='thonghuynh.0203@gmail.com',
    description='Nav2 staging and AprilTag visual docking server',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'docking_server = my_robot_docking.docking_server:main',
            'undock_server = my_robot_docking.undock_server:main',
            'velocity_arbiter = my_robot_docking.velocity_arbiter:main',
            'record_dock_pose = my_robot_docking.record_dock_pose:main',
        ],
    },
)
