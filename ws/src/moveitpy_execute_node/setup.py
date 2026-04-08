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
    install_requires=["setuptools", "scipy", "PyYAML"],
    zip_safe=True,
    maintainer="ros",
    maintainer_email="ros@todo.todo",
    description="MoveItPy execution node",
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "executor_node = moveitpy_execute_node.executor_node:main",
            "trajectory_bridge = moveitpy_execute_node.trajectory_bridge:main",
            "gripper_command = moveitpy_execute_node.gripper_command_node:main",
            "demo_grasp_sequence = moveitpy_execute_node.demo_grasp_sequence:main",
            "demo_random_poses = moveitpy_execute_node.demo_random_poses:main",
            "demo_fixed_pose = moveitpy_execute_node.demo_fixed_pose:main",
            "grasp_with_candidates = moveitpy_execute_node.grasp_with_candidates:main",
            "ground_plane_scene = moveitpy_execute_node.ground_plane_scene:main",
            # Backward compatibility: old names run the new demos (executor must be running)
            "grasp_sequence = moveitpy_execute_node.demo_grasp_sequence:main",
            "pose_goal = moveitpy_execute_node.demo_random_poses:main",
        ],
    },
)
