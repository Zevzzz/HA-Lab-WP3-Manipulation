# HA Lab WP3 Manipulation (WIP)

## Overview

Modular MoveItPy stack for the Panda arm: executor node exposes an **ExecutePose** action for motion planning and execution; a trajectory bridge forwards commands to Isaac Sim. Demos (grasp sequence, random poses) call the action to verify the pipeline. Planning uses OMPL. Intended for GraspGen evaluation, humanoid control, and RL policy training in Isaac Sim.

- **ROS stack**
  - **Executor node:** MoveItPy + **ExecutePose** action; any node sends a target pose and gets motion (demos, GraspGen eval, RL).
  - **Trajectory bridge:** FollowJointTrajectory → **/joint_command**, merges **/gripper_command**; no controller_manager when using Isaac.
  - **Demos:** Thin clients (grasp sequence, random poses); call the action + publish gripper.
  - **Packages:** Action type in **moveitpy_execute_node_msgs**; all logic in **moveitpy_execute_node**.
- **GraspGen**
  - **ZMQ server:** Runs from **deps/GraspGen** (`graspgen_server.py`, Franka Panda gripper config). Use **scripts/run_graspgen_server.sh** to launch.
  - **scripts/graspgen_request.py:** Standalone client — point cloud (.npy/.ply) → server → grasps YAML. No GraspGen package needed (pyzmq, msgpack).
  - **Integration:** Not wired yet; slot in via **PoseSource** or a client that calls ExecutePose with GraspGen output.
- **Isaac Sim**
  - **Launch:** **scripts/run_isaac_sim.sh** (sets ROS_DOMAIN_ID and Fast DDS profile).
  - **Panda assets:** **ws/src/moveitpy_execute_node/urdf/panda_isaac/** (USDs + URDF) for import into your scene.
  - **Scene:** Must publish **/joint_states** and **/clock**, subscribe to **/joint_command**. Use same ROS_DOMAIN_ID (and DDS profile) as the launch terminal.

## Build

From the workspace (e.g. `ws/`):
```bash
colcon build
source install/setup.bash
```

## How to run

```bash
# Terminal 1 - GraspGen Server (optional; for future GraspGen integration)
cd deps/GraspGen
bash docker/run.sh . --models ./GraspGenModels
python client-server/graspgen_server.py --gripper_config /models/checkpoints/graspgen_franka_panda.yml --port 5557

# Terminal 2 – start Isaac Sim
./scripts/run_isaac_sim.sh

# Terminal 3 – run executor + trajectory bridge + demo
colcon build && source install/setup.bash
ros2 launch moveitpy_execute_node panda_pose_goal_isaac.launch.py
```

- Use `demo:=grasp` (default), `demo:=random`, or `demo:=none` to choose the demo or run no demo.
- Add `use_rviz:=true` for RViz.

## TODO
- Potential tech debt – custom trajectory bridge instead of stock
- Integrate GraspGen
- Evaluate GraspGen functionality
