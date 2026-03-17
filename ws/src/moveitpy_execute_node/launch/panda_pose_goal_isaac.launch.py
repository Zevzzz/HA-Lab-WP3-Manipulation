"""
Launch MoveIt + executor_node + trajectory_bridge for Panda with Isaac Sim.

Isaac Sim must be running (your scene, Play, Action Graph publishing /joint_states,
/clock and subscribing to /joint_command). Start this launch in the container after
setting the same ROS_DOMAIN_ID (and DDS profile if needed) as your Isaac wrapper.

No controller_manager or spawners; the trajectory_bridge provides the action server
and forwards to /joint_command for Isaac. The executor_node exposes the ExecutePose
action for motion planning; optional demo (grasp or random) runs a test sequence.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    package_share = get_package_share_directory("moveit_resources_panda_moveit_config")
    rviz_config = os.path.join(package_share, "launch", "moveit.rviz")
    initial_positions = os.path.join(package_share, "config", "initial_positions.yaml")

    moveit_cpp_params = {
        "planning_pipelines": {
            "pipeline_names": [
                "ompl",
                "chomp",
                "pilz_industrial_motion_planner",
                "stomp",
            ],
            "namespace": "",
        },
        "plan_request_params": {
            "planning_pipeline": "ompl",
            "planner_id": "RRTConnect",
            "planning_time": 5.0,
            "planning_attempts": 1,
            "max_velocity_scaling_factor": 0.2,
            "max_acceleration_scaling_factor": 0.2,
        },
    }

    moveit_config = (
        MoveItConfigsBuilder(
            "panda",
            package_name="moveit_resources_panda_moveit_config",
        )
        .robot_description(
            file_path="config/panda.urdf.xacro",
            mappings={
                "ros2_control_hardware_type": "mock_components",
                "initial_positions_file": initial_positions,
            },
        )
        .robot_description_semantic(file_path="config/panda.srdf")
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner", "stomp"]
        )
        .to_moveit_configs()
    )

    use_sim_time = {"use_sim_time": True}

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), use_sim_time],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description, use_sim_time],
    )

    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0", "0", "0", "0", "0", "0", "world", "panda_link0"],
        parameters=[use_sim_time],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            use_sim_time,
        ],
    )

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="false",
        description="Launch RViz2 for visualization.",
    )
    rviz_group = GroupAction(
        [rviz_node],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    trajectory_bridge_node = Node(
        package="moveitpy_execute_node",
        executable="trajectory_bridge",
        name="trajectory_bridge",
        output="screen",
        parameters=[use_sim_time],
    )

    executor_node = Node(
        package="moveitpy_execute_node",
        executable="executor_node",
        name="moveit_py",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            moveit_cpp_params,
            use_sim_time,
        ],
    )

    demo_arg = DeclareLaunchArgument(
        "demo",
        default_value="grasp",
        description="Demo to run: 'none', 'grasp', or 'random'.",
    )

    demo_grasp_node = Node(
        package="moveitpy_execute_node",
        executable="demo_grasp_sequence",
        name="demo_grasp_sequence",
        output="screen",
        parameters=[use_sim_time],
        condition=LaunchConfigurationEquals("demo", "grasp"),
    )
    demo_random_node = Node(
        package="moveitpy_execute_node",
        executable="demo_random_poses",
        name="demo_random_poses",
        output="screen",
        parameters=[use_sim_time],
        condition=LaunchConfigurationEquals("demo", "random"),
    )

    return LaunchDescription([
        use_rviz_arg,
        demo_arg,
        static_tf_node,
        robot_state_publisher,
        move_group_node,
        trajectory_bridge_node,
        executor_node,
        rviz_group,
        demo_grasp_node,
        demo_random_node,
    ])
