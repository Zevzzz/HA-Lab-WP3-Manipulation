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
  - **grasp_with_candidates:** ROS executable in **moveitpy_execute_node** — loads a grasps YAML (e.g. from GraspGen under `data/Mug/`), tries each candidate in order via **ExecutePose** until one succeeds (or all fail), logs one row to **data/logs/grasp_execution_results.csv** (timestamp, yaml path, total candidates, index used, failures, success, message). Run with `ros2 run moveitpy_execute_node grasp_with_candidates --path <yaml>`; optional `--log-dir`.
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
# Terminal 1 - GraspGen ZMQ Server (from repo root: source venv, then run server)
source scripts/venv/bin/activate
cd deps/GraspGen && bash docker/run.sh . --models ./GraspGenModels
python client-server/graspgen_server.py --gripper_config /models/checkpoints/graspgen_franka_panda.yml --port 5557

# Terminal 2 – start Isaac Sim
./scripts/run_isaac_sim.sh

# Terminal 3 – run executor + trajectory bridge + demo
colcon build && source install/setup.bash
ros2 launch moveitpy_execute_node panda_pose_goal_isaac.launch.py

# Terminal 4 – grasp from YAML (executor + Isaac must be running): tries candidates until one succeeds, logs to data/logs/grasp_execution_results.csv
ros2 run moveitpy_execute_node grasp_with_candidates --path /path/to/grasps.yaml
```

- Use `demo:=grasp` or `demo:=random` to run a demo (for debug); default is no demo (action server only).
- Add `use_rviz:=true` for RViz.
- `grasp_with_candidates`: optional `--log-dir` (default `data/logs`).

**Where to run**  
The ROS stack (executor, trajectory bridge, `grasp_with_candidates`) is intended to run **inside the container** (e.g. `docker compose run --rm sim bash`, then the commands above from `~/ws`). The container already has the ROS env—no venv needed. If you run ROS on the host instead, you’d need ROS 2 Jazzy and the workspace built there; a venv is usually not used for ROS nodes.

**Data in the container**  
Host `./data` is mounted at **`/home/ros/data`** only. From inside the container use **`/home/ros/data/Mug/...`** or **`/home/ros/data/logs/...`** (e.g. `--path /home/ros/data/Mug/Mug8192_grasps.yaml`). **Always run `docker compose` from the repo root** so `./data` is your repo’s data; running from another directory can make Docker use or create a different, empty `./data` and your files can disappear from view (or be replaced). If `data/Mug` was wiped, restore with `git restore data/Mug/`.

**scripts/venv**  
Use `source scripts/venv/bin/activate` when running **scripts/graspgen_request.py** or the GraspGen server on the host (pyzmq, msgpack, PyYAML, trimesh).

## TODO
- Potential tech debt – custom trajectory bridge instead of stock
- Integrate GraspGen
- Evaluate GraspGen functionality
