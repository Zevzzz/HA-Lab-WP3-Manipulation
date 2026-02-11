# HA Lab WP3 Manipulation (WIP)

## Overview

Small MoveItPy demo for the Panda arm that launches RViz + MoveIt and runs a looped
pose-goal motion from a Python node. The script samples random targets in a safe-ish
workspace, plans with OMPL, and executes on the fake controllers. If you need a
single fixed target or different bounds, edit `moveitpy_execute_node/pose_goal.py`.

## Build

From the workspace (e.g. `ws/`):
```bash
colcon build --packages-select moveitpy_execute_node
source install/setup.bash
```

## How to run

**RViz only (mock):** one terminal
```bash
ros2 launch moveitpy_execute_node panda_pose_goal.launch.py
```

**Gazebo (sim):** three terminals
```bash
# Terminal 1 – start Gazebo
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="-r empty.sdf"

# Terminal 2 – spawn Panda
ros2 launch moveitpy_execute_node panda_gz_spawn.launch.py

# Terminal 3 – MoveIt + pose_goal
ros2 launch moveitpy_execute_node panda_pose_goal_gz.launch.py
```

Full step-by-step: **[docs/HOW_TO_RUN.md](docs/HOW_TO_RUN.md)**.

## Gazebo setup

Background and setup details: [docs/GAZEBO_SETUP.md](docs/GAZEBO_SETUP.md).

## TODO

- Run in GZ and integrate
- Add GraspNet
- Evaluate GraspNet functionality
- Try Isaac Sim
