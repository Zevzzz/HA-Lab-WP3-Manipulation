"""
Spawn the Panda into Gazebo via ros_gz_sim's create node.

Prereq: Start Gazebo first (Terminal 1):
  ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="-r empty.sdf"

Then run this launch (Terminal 2, after colcon build && source install/setup.bash):
  ros2 launch moveitpy_execute_node panda_gz_spawn.launch.py

In Docker/containers: ensure GZ_PARTITION matches. If spawn times out, run both in the same
shell or export GZ_PARTITION=ros before starting Gazebo.
"""
import os
import subprocess
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _build_urdf_and_spawn(context, *args, **kwargs):
    config_share = get_package_share_directory("moveit_resources_panda_moveit_config")
    pkg_share = get_package_share_directory("moveitpy_execute_node")
    xacro_path = os.path.join(pkg_share, "urdf", "panda_gazebo.urdf.xacro")
    initial_positions = os.path.join(config_share, "config", "initial_positions.yaml")
    ros2_controllers = os.path.join(config_share, "config", "ros2_controllers.yaml")

    if not os.path.isfile(xacro_path):
        raise FileNotFoundError(
            f"Gazebo Panda xacro not found: {xacro_path}. Rebuild with colcon build."
        )

    out_fd, urdf_path = tempfile.mkstemp(suffix=".urdf", prefix="panda_gz_")
    os.close(out_fd)

    cmd = [
        "xacro",
        xacro_path,
        f"config_dir:={config_share}",
        f"initial_positions_file:={initial_positions}",
        f"ros2_controllers_file:={ros2_controllers}",
        "-o",
        urdf_path,
    ]
    result = subprocess.run(cmd, env=os.environ.copy(), capture_output=True, text=True)
    if result.returncode != 0:
        raise RuntimeError(f"xacro failed: {result.stderr}")

    def _to_double(s):
        s = s.strip()
        if "." not in s and s.lstrip("-").isdigit():
            return s + ".0"
        return s

    entity_name = LaunchConfiguration("entity_name", default="panda").perform(context)
    x = _to_double(LaunchConfiguration("x", default="0.0").perform(context))
    y = _to_double(LaunchConfiguration("y", default="0.0").perform(context))
    z = _to_double(LaunchConfiguration("z", default="0.0").perform(context))

    world_name = LaunchConfiguration("world", default="empty").perform(context)
    print(f"[panda_gz_spawn] URDF: {urdf_path}, entity: {entity_name}, pose: ({x},{y},{z})")
    print(f"[panda_gz_spawn] Spawning via ros_gz_sim create node into world '{world_name}'")

    spawn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                get_package_share_directory("ros_gz_sim"),
                "/launch",
                "/gz_spawn_model.launch.py",
            ]
        ),
        launch_arguments={
            "world": world_name,
            "file": urdf_path,
            "entity_name": entity_name,
            "x": x,
            "y": y,
            "z": z,
        }.items(),
    )
    return [spawn_launch]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="empty", description="Gazebo world name (must match the running sim)"),
        DeclareLaunchArgument("entity_name", default_value="panda", description="Spawned model entity name"),
        DeclareLaunchArgument("x", default_value="0.0", description="Spawn position x (double)"),
        DeclareLaunchArgument("y", default_value="0.0", description="Spawn position y (double)"),
        DeclareLaunchArgument("z", default_value="0.0", description="Spawn position z (double)"),
        OpaqueFunction(function=_build_urdf_and_spawn),
    ])
