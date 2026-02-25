from setuptools import setup
from glob import glob
import os

package_name = "moveitpy_execute_node"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.srdf")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ros",
    maintainer_email="ros@todo.todo",
    description="MoveItPy execution node",
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "pose_goal = moveitpy_execute_node.pose_goal:main",
            "trajectory_bridge = moveitpy_execute_node.trajectory_bridge:main",
        ],
    },
)
