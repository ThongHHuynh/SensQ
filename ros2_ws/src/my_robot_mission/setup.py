import os
from glob import glob
from setuptools import find_packages, setup

package_name = "my_robot_mission"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="tom",
    maintainer_email="thonghuynh.0203@gmail.com",
    description="Mission sequencer orchestrating undock, coverage cleaning, and re-docking",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "mission_sequencer = my_robot_mission.mission_sequencer:main",
        ],
    },
)
