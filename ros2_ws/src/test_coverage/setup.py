from glob import glob
from setuptools import find_packages, setup


package_name = "test_coverage"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="tom",
    maintainer_email="thonghuynh.0203@gmail.com",
    description="Modular Boustrophedon coverage planning and Nav2 execution.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "coverage_planner = test_coverage.planner_node:main",
            "coverage_executor = test_coverage.executor_node:main",
        ],
    },
)
