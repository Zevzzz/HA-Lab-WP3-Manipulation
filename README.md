# HA Lab WP3 Manipulation (WIP)

## Overview

Small MoveItPy demo for the Panda arm that launches RViz + MoveIt and runs a looped
pose-goal motion from a Python node. The script samples random targets in a safe-ish
workspace, plans with OMPL, and executes on the fake controllers. If you need a
single fixed target or different bounds, edit `moveitpy_execute_node/pose_goal.py`.

## Build + run

Build + run:
```bash
colcon build --packages-select moveitpy_execute_node
source install/setup.bash
ros2 launch moveitpy_execute_node panda_pose_goal.launch.py
```

## TODO

- Run in GZ and integrate
- Add GraspNet
- Evaluate GraspNet functionality
- Try Isaac Sim
