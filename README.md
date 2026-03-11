# HA Lab WP3 Manipulation (WIP)

## Overview

Small MoveItPy demo for the Panda arm: launches MoveIt + Isaac Sim and runs a simple
grasp sequence (open gripper → move to pre-grasp above a cube → close gripper → move to home).
Planning uses OMPL; a trajectory bridge forwards commands to Isaac Sim. Ready to slot in GraspGen later.

## Build

From the workspace (e.g. `ws/`):
```bash
colcon build
source install/setup.bash
```

## How to run

```bash
# Terminal 1 - GraspGen Server
cd deps/GraspGen
bash docker/run.sh . --models ./GraspGenModels
python client-server/graspgen_server.py --gripper_config /models/checkpoints/graspgen_franka_panda.yml --port 5557

# Terminal 2 – start Isaac Sim
./scripts/run_isaac_sim.sh

# Terminal 3 – run execute node 
ros2 launch moveitpy_execute_node panda_pose_goal_isaac.launch.py
```

Add `use_rviz:=true` to the launch command for RViz.

## TODO
- Potential tech debt – custom trajectory bridge instead of stock
- Integrate GraspGen
- Evaluate GraspGen functionality
