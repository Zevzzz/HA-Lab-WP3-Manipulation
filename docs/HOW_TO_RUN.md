# How to run everything

Two ways to run the Panda + MoveIt + pose_goal demo: **RViz only (mock)** or **Gazebo (sim)**.

---

## Build

From your workspace (e.g. `~/ws` or `/home/ros/ws`):

```bash
cd ~/ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select moveitpy_execute_node
source install/setup.bash
```

**After any change** to this package (launch files, Python, xacro, config): run the same `colcon build` and `source install/setup.bash` again. ROS 2 runs from the **install** space, not the source tree; without rebuilding, you still run the old version.

---

## Option A: RViz only (mock controllers)

Single terminal. No Gazebo. Good for testing planning and the demo script.

1. **Source and launch:**
   ```bash
   cd ~/ws
   source install/setup.bash
   ros2 launch moveitpy_execute_node panda_pose_goal.launch.py
   ```

2. **What you get:** RViz + MoveIt + a Python node that sends random pose goals; the arm “moves” in RViz using fake controllers.

3. **Stop:** Ctrl+C in the terminal (exits cleanly).

---

## Option B: Gazebo (full sim)

Three terminals. Gazebo runs the physics; the same pose_goal demo drives the arm in sim.

### Terminal 1 – Start Gazebo

```bash
cd ~/ws
source install/setup.bash
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="-r empty.sdf"
```

Leave this running. A Gazebo window should open with an empty world.

---

### Terminal 2 – Spawn the Panda

With Gazebo still running (only one Gazebo window). If you changed anything in the package since the last run, rebuild first (see **Build** above), then:

```bash
cd ~/ws
source install/setup.bash
ros2 launch moveitpy_execute_node panda_gz_spawn.launch.py
```

The Panda should appear in **that same** Gazebo window. Leave this terminal as-is (the spawn process will exit; the model stays in Gazebo).

**If a second, empty Gazebo window appears:** the spawn is talking to the wrong world. Close **all** Gazebo windows, then:

1. Start **only** Gazebo again (Terminal 1):  
   `ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="-r empty.sdf"`
2. In Terminal 2, try spawning with an explicit world name:
   ```bash
   ros2 launch moveitpy_execute_node panda_gz_spawn.launch.py world:=default
   ```
   If the Panda still doesn’t appear in the first window, try `world:=empty` instead.

**If you see “world not found” or xacro/URDF errors:** check the Terminal 2 output; the launch prints the world name and URDF path. Fix any missing-package or xacro errors and rebuild.

**If spawn times out or "Host unreachable" in Gazebo:** in Docker/containers, set `GZ_PARTITION` so both Gazebo and spawn use the same value: `export GZ_PARTITION=ros` before starting Terminal 1 and 2.

**"Waiting for robot_description" / "Waiting RM to load" spam in Gazebo:** expected until you run Terminal 3. Run `panda_pose_goal_gz.launch.py` to publish robot_description and stop the spam.

---

### Terminal 3 – MoveIt + pose_goal + RViz

With Gazebo running and the Panda spawned:

```bash
cd ~/ws
source install/setup.bash
ros2 launch moveitpy_execute_node panda_pose_goal_gz.launch.py
```

This starts: static TF (world → panda_link0), robot_state_publisher, move_group, RViz, and the pose_goal node. It does **not** start a separate controller manager (the gz_ros2_control plugin in Gazebo does that).

The arm in Gazebo should move to random pose goals; you can watch it in both Gazebo and RViz.

**Stop:** Ctrl+C in Terminal 3, then Ctrl+C in Terminal 1 when you’re done.

---

## Quick reference

| What you want        | Command (after `source install/setup.bash`) |
|----------------------|----------------------------------------------|
| RViz + mock only     | `ros2 launch moveitpy_execute_node panda_pose_goal.launch.py` |
| Gazebo (Terminal 1)  | `ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="-r empty.sdf"` |
| Spawn Panda (Term 2) | `ros2 launch moveitpy_execute_node panda_gz_spawn.launch.py` |
| MoveIt + pose in GZ (Term 3) | `ros2 launch moveitpy_execute_node panda_pose_goal_gz.launch.py` |

---

## If you use Docker

From the repo root (e.g. `Ha-Lab-WP3-Sim`):

```bash
docker compose run --rm sim bash
# inside container:
cd ~/ws
source install/setup.bash
# then run the commands above (Option A or Option B)
```

For Gazebo you need a display; use the same X11/display options you already use for RViz.
