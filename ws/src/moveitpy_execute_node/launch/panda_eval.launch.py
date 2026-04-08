"""
Panda + MoveIt + MoveItPy executor + ros2_control (stock trajectory execution).

use_isaac:=true
  - URDF hardware: topic_based_ros2_control/TopicBasedSystem (/isaac_joint_*)
  - isaac_ros2_control_relay: /joint_states -> /isaac_joint_states,
    /isaac_joint_commands -> /joint_command
  - use_sim_time true
  - Does NOT spawn joint_state_broadcaster (Isaac is the sole /joint_states publisher)

use_isaac:=false
  - mock_components/GenericSystem (offline / RViz check)
  - use_sim_time false
  - Spawns joint_state_broadcaster

MoveIt sends trajectories to panda_arm_controller (joint_trajectory_controller) via the
standard MoveItSimpleControllerManager -> FollowJointTrajectory (no custom bridge).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder


def _opaque_setup(context, *_args, **_kwargs):
    use_isaac = context.launch_configurations.get("use_isaac", "false").lower() in (
        "true",
        "1",
        "yes",
    )
    use_rviz = context.launch_configurations.get("use_rviz", "false").lower() in (
        "true",
        "1",
        "yes",
    )
    hw = "isaac" if use_isaac else "mock_components"
    use_sim_time = {"use_sim_time": use_isaac}

    package_share = get_package_share_directory("moveit_resources_panda_moveit_config")
    rviz_config = os.path.join(package_share, "launch", "moveit.rviz")
    initial_positions = os.path.join(package_share, "config", "initial_positions.yaml")
    ros2_controllers_path = os.path.join(package_share, "config", "ros2_controllers.yaml")

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
                "ros2_control_hardware_type": hw,
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

    move_group_params = moveit_config.to_dict()
    move_group_params["default_workspace_bounds"] = 0.6

    nodes = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_transform_publisher",
            output="log",
            arguments=["0", "0", "0", "0", "0", "0", "world", "panda_link0"],
            parameters=[use_sim_time],
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="both",
            parameters=[
                moveit_config.robot_description,
                use_sim_time,
                # Jazzy+: ros2_control_node subscribes to /robot_description (no param URDF).
                {"publish_robot_description": True},
            ],
        ),
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[move_group_params, use_sim_time],
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[moveit_config.robot_description, ros2_controllers_path],
            output="screen",
        ),
    ]

    # Mock: joint_state_broadcaster publishes /joint_states from GenericSystem.
    # Isaac: Isaac publishes /joint_states; relay feeds /isaac_joint_states only.
    if not use_isaac:
        nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager",
                    "/controller_manager",
                ],
            )
        )

    nodes.extend(
        [
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["panda_arm_controller", "-c", "/controller_manager"],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["panda_hand_controller", "-c", "/controller_manager"],
            ),
        ]
    )

    if use_isaac:
        nodes.append(
            Node(
                package="moveitpy_execute_node",
                executable="isaac_ros2_control_relay",
                name="isaac_ros2_control_relay",
                output="screen",
                parameters=[
                    use_sim_time,
                    {
                        "sim_joint_states_in": LaunchConfiguration("isaac_joint_states_in"),
                        "ros2_joint_states_out": "/isaac_joint_states",
                        "ros2_joint_commands_in": "/isaac_joint_commands",
                        "sim_joint_commands_out": LaunchConfiguration("isaac_joint_commands_out"),
                    },
                ],
            )
        )

    cartesian_tip = context.launch_configurations.get(
        "cartesian_tip_link", "panda_link8"
    )
    nodes.append(
        Node(
            package="moveitpy_execute_node",
            executable="executor_node",
            name="moveit_py",
            output="screen",
            parameters=[
                moveit_config.to_dict(),
                moveit_cpp_params,
                use_sim_time,
                {"cartesian_tip_link": cartesian_tip},
            ],
        )
    )

    if use_rviz:
        nodes.append(
            Node(
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
        )

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_isaac",
                default_value="true",
                description="true: topic_based_ros2_control + relay to Isaac topics. "
                "false: mock hardware for RViz / no sim.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="false",
                description="Launch RViz2.",
            ),
            DeclareLaunchArgument(
                "cartesian_tip_link",
                default_value="panda_link8",
                description="MoveIt pose_link for ExecutePose (panda_link8 or panda_hand).",
            ),
            DeclareLaunchArgument(
                "isaac_joint_states_in",
                default_value="/joint_states",
                description="JointState from Isaac Sim (relay -> /isaac_joint_states).",
            ),
            DeclareLaunchArgument(
                "isaac_joint_commands_out",
                default_value="/joint_command",
                description="JointState commands to Isaac Sim (from /isaac_joint_commands).",
            ),
            OpaqueFunction(function=_opaque_setup),
        ]
    )
