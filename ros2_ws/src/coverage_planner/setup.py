import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'coverage_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tom',
    maintainer_email='thonghuynh.0203@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'coverage_node = coverage_planner.coverage_node:main',
            'coverage_manager_node = coverage_planner.coverage_manager_node:main',
            'map_processor_node = coverage_planner.map_processor_node:main',
            'path_generator_node = coverage_planner.path_generator_node:main',
            'coverage_visualizer_node = coverage_planner.coverage_visualizer_node:main',
        ],
    },
)
