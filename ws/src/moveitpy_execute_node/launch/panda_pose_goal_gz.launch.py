"""
MoveIt + pose_goal + RViz for when the Panda is already running in Gazebo.

Do NOT start this until Gazebo is running and the Panda has been spawned
(panda_gz_spawn.launch.py). The gz_ros2_control plugin runs controller_manager
inside Gazebo; we still need spawners to load/activate the controllers.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    config_share = get_package_share_directory("moveit_resources_panda_moveit_config")
    pkg_share = get_package_share_directory("moveitpy_execute_node")
    rviz_config = os.path.join(config_share, "launch", "moveit.rviz")
    initial_positions = os.path.join(config_share, "config", "initial_positions.yaml")
    ros2_controllers = os.path.join(config_share, "config", "ros2_controllers.yaml")
    xacro_path = os.path.join(pkg_share, "urdf", "panda_gazebo.urdf.xacro")

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

    # Use panda_gazebo.urdf.xacro (same as spawn) so gz_ros2_control receives correct URDF
    moveit_config = (
        MoveItConfigsBuilder(
            "panda",
            package_name="moveit_resources_panda_moveit_config",
        )
        .robot_description(
            file_path=xacro_path,
            mappings={
                "config_dir": config_share,
                "initial_positions_file": initial_positions,
                "ros2_controllers_file": ros2_controllers,
            },
        )
        .robot_description_semantic(file_path=os.path.join(pkg_share, "config", "panda.srdf"))
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

    # use_sim_time: required for MoveIt to receive joint_states from Gazebo (sim time)
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

    pose_goal_node = Node(
        package="moveitpy_execute_node",
        executable="pose_goal",
        name="moveit_py",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            moveit_cpp_params,
            use_sim_time,
        ],
    )

    # Bridge Gazebo /clock to ROS so use_sim_time works
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        output="log",
        parameters=[
            {"bridge_names": ["clock_bridge"]},
            {"bridges.clock_bridge.ros_topic_name": "/clock"},
            {"bridges.clock_bridge.gz_topic_name": "/clock"},
            {"bridges.clock_bridge.ros_type_name": "rosgraph_msgs/msg/Clock"},
            {"bridges.clock_bridge.gz_type_name": "gz.msgs.Clock"},
            {"bridges.clock_bridge.direction": "GZ_TO_ROS"},
        ],
    )

    # Spawners load/activate controllers on the controller_manager inside Gazebo
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        parameters=[use_sim_time],
    )
    panda_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["panda_arm_controller", "-c", "/controller_manager"],
        parameters=[use_sim_time],
    )
    panda_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["panda_hand_controller", "-c", "/controller_manager"],
        parameters=[use_sim_time],
    )

    return LaunchDescription([
        clock_bridge,
        static_tf_node,
        robot_state_publisher,
        move_group_node,
        rviz_node,
        pose_goal_node,
        # Delay spawners so controller_manager (in Gazebo) is ready after robot_description
        TimerAction(
            period=5.0,
            actions=[
                joint_state_broadcaster_spawner,
                panda_arm_controller_spawner,
                panda_hand_controller_spawner,
            ],
        ),
    ])
