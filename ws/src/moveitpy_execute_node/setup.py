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
    install_requires=["setuptools", "scipy"],
    zip_safe=True,
    maintainer="ros",
    maintainer_email="ros@todo.todo",
    description="MoveItPy execution node",
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "grasp_sequence = moveitpy_execute_node.grasp_sequence:main",
            "gripper_command = moveitpy_execute_node.gripper_command_node:main",
            "trajectory_bridge = moveitpy_execute_node.trajectory_bridge:main",
        ],
    },
)
