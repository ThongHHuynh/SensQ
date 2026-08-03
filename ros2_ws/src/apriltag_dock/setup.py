from setuptools import setup
import os
from glob import glob

package_name = 'apriltag_dock'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tom',
    maintainer_email='thonghuynh.0203@gmail.com',
    description='AprilTag detection and precision docking for differential-drive robots',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'apriltag_detector_node = apriltag_dock.apriltag_detector_node:main',
            'dock_controller_node = apriltag_dock.dock_controller_node:main',
        ],
    },
)
