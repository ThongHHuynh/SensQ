import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'coverage_planner_archive'

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
    description='Coverage path planning for mobile robots using Boustrophedon cell decomposition',
    license='Proprietary',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'map_processor_node = coverage_planner_archive.map_processor_node:main',
            'coverage_visualizer_node = coverage_planner_archive.coverage_visualizer_node:main',
            'f2c_path_gen_node = coverage_planner_archive.f2c_path_gen:main',
            'open_coverage_path = coverage_planner_archive.open_coverage_path:main',
            'production_path_gen = coverage_planner_archive.production_path_gen:main',
            'coverage_executor_node = coverage_planner_archive.coverage_executor_node:main',
        ],
    },
)
