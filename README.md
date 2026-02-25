# HA Lab WP3 Manipulation (WIP)

## Overview

Small MoveItPy demo for the Panda arm that launches RViz + MoveIt and runs a looped
pose-goal motion from a Python node. The script samples random targets in a safe-ish
workspace, plans with OMPL, and executes on the fake controllers. If you need a
single fixed target or different bounds, edit `moveitpy_execute_node/pose_goal.py`.

## Build

From the workspace (e.g. `ws/`):
```bash
colcon build
source install/setup.bash
```

## How to run

**Isaac Sim Visualization:** two terminals
```bash
# Terminal 1 – start Isaac Sim
./scripts/run_isaac_sim.sh

# Terminal 2 - run execute node
ros2 launch moveitpy_execute_node panda_pose_goal_isaac.launch.py
```


## TODO
- Potential tech debt - custom trajectory bridge node instead of a stock version
- Add GraspGen
- Evaluate GraspNet functionality
