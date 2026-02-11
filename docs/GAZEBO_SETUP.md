# Gazebo integration – step-by-step setup

This guide walks through setting up Gazebo (Gazebo Sim) and spawning the Panda arm so MoveIt can drive it in simulation. Your existing `pose_goal` demo will then run against the sim instead of mock controllers.

---

## Prerequisites (you already have these)

- ROS 2 Jazzy
- `ros-jazzy-ros-gz` (ros_gz_sim – launch Gazebo from ROS 2)
- `ros-jazzy-gz-ros2-control` (bridge between Gazebo and ros2_control)
- `moveit_resources_panda_moveit_config` (Panda URDF and MoveIt config)

For Jazzy, Gazebo Sim **Harmonic** is the matching version. The `ros-gz` packages bring it in.

---

## Step 1 – Verify Gazebo and ros_gz_sim

1. **Source ROS and (if you use it) your workspace:**
   ```bash
   source /opt/ros/jazzy/setup.bash
   # source install/setup.bash   # if you built a workspace
   ```

2. **Check that Gazebo Sim runs:**
   ```bash
   gz sim --version
   ```
   You should see a Harmonic version (e.g. 7.x).

3. **Launch an empty world from ROS 2:**
   ```bash
   ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="-r empty.sdf"
   ```
   The `-r` flag starts the simulation **running** (not paused). Without it, physics does not step, controllers never run, and execution fails.
   - A Gazebo window should open with an empty world.
   - Leave it running; we’ll spawn the robot into this world later.
   - To close: close the window or Ctrl+C in the terminal.

4. **Optional – run Gazebo headless (no GUI):**
   ```bash
   ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="-r -s empty.sdf"
   ```
   `-s` is server-only (no GUI). Keep `-r` for the simulation to run. Useful for CI or remote runs.

**Summary:** Gazebo Sim is installed; you can start a world with `ros_gz_sim` and `gz_args:="-r empty.sdf"`.

---

## Step 1b – Spawn the Panda (with Gazebo still running)

After Gazebo is running (Step 1), in a **second terminal**:

1. **Source your workspace** (so the spawn launch and xacro can find packages):
   ```bash
   source /opt/ros/jazzy/setup.bash
   source install/setup.bash   # from your ws directory
   ```

2. **Build the package** (once) if you haven’t:
   ```bash
   colcon build --packages-select moveitpy_execute_node
   source install/setup.bash
   ```

3. **Spawn the Panda** into the running world:
   ```bash
   ros2 launch moveitpy_execute_node panda_gz_spawn.launch.py
   ```
   Optional args: `world:=empty` (default), `entity_name:=panda`, `x:=0 y:=0 z:=0`. If your world has a different name, pass e.g. `world:=my_world`.

4. You should see the Panda model appear in the Gazebo window. The `gz_ros2_control` plugin in the model will start the controller manager and load `joint_state_broadcaster`, `panda_arm_controller`, and `panda_hand_controller` from the MoveIt Panda config.

**If the world name doesn’t match:** When you started Gazebo with `gz_args:="-r empty.sdf"`, the world name might be `empty` or `default`. If the spawn says the world wasn’t found, try:
   ```bash
   ros2 launch moveitpy_execute_node panda_gz_spawn.launch.py world:=default
   ```

---

## Step 2 – Understand the world file

- **Default “empty” world:** When you use `gz_args:="-r empty.sdf"`, Gazebo looks for a file named `empty.sdf` in its resource paths (e.g. from the Gazebo install or from `GZ_SIM_RESOURCE_PATH`).
- **Custom world:** You can use your own `.sdf` world file:
  - Put it in a package and add that package’s share dir to `GZ_SIM_RESOURCE_PATH`, or
  - Pass the full path in `gz_args`, e.g. `gz_args:="/path/to/my_world.sdf"`.

For this setup, the built-in `empty.sdf` is enough. Later you can add a ground plane, table, or objects for grasping.

---

## Step 3 – Robot model for Gazebo (Panda + gz_ros2_control)

MoveIt’s Panda config uses a xacro that currently points ros2_control at **mock** hardware. For Gazebo we need:

1. **ros2_control** in the URDF to use the **Gazebo** hardware type (so the gz_ros2_control plugin can drive the joints).
2. A **Gazebo plugin** in the URDF that loads `gz_ros2_control` and the controller config.

So we need a **Gazebo-specific robot description** (URDF/xacro) that:

- Keeps the same Panda links/joints/limits so MoveIt and Gazebo agree.
- Swaps the ros2_control `<hardware><plugin>` to `gz_ros2_control/GazeboSimSystem`.
- Adds a `<gazebo>` block with the plugin:
  - `libgz_ros2_control-system.so`
  - `gz_ros2_control::GazeboSimROS2ControlPlugin`
  - `<parameters>` pointing at the **same** controller YAML you use for MoveIt (e.g. `ros2_controllers.yaml` from the Panda config), so the plugin starts the same controllers (e.g. `panda_arm_controller`, `panda_hand_controller`, `joint_state_broadcaster`).

**Ways to get that description:**

- **Option A – xacro arg (if the Panda xacro supports it):**  
  The MoveIt Panda xacro may take something like `ros2_control_hardware_type`. If it does, we can pass `gz_ros2_control/GazeboSimSystem` and add a small wrapper xacro that includes the Panda xacro and adds the `<gazebo>` plugin block. That wrapper is your “Gazebo Panda” description.

- **Option B – Overlay package:**  
  Create a small package (e.g. `panda_gz_description`) that:
  - Includes a xacro that composes the Panda (from moveit_resources) and adds the Gazebo plugin + ros2_control block for `GazeboSimSystem`, or
  - Ships a standalone URDF/xacro that matches the Panda kinematics and adds Gazebo + gz_ros2_control.

We’ll implement one of these in the next phase; for now the important point is: **one robot description that uses GazeboSimSystem and the gz plugin, and the same controller YAML as MoveIt.**

---

## Step 4 – Spawn the Panda into the Gazebo world

Once you have the Gazebo-ready URDF (or xacro processed to URDF):

1. **Start Gazebo with the empty world** (Step 1):
   ```bash
   ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="-r empty.sdf"
   ```

2. **Spawn the Panda** using the spawn helper from `ros_gz_sim`:
   ```bash
   ros2 launch ros_gz_sim gz_spawn_model.launch.py \
     world:=empty \
     file:=/path/to/panda_gazebo.urdf \
     entity_name:=panda \
     x:=0 y:=0 z:=0
   ```
   - `world:=empty` must match the world name you started (often the base name of the SDF).
   - `file` = path to the **processed** URDF (or SDF) that contains the gz_ros2_control plugin.
   - If your model is xacro, process it first, e.g.:
     ```bash
     xacro /path/to/panda_gazebo.urdf.xacro -o /tmp/panda_gazebo.urdf
     ```
     then use `file:=/tmp/panda_gazebo.urdf`.

3. **What happens when the model spawns:**
   - Gazebo loads the model and runs the `gz_ros2_control` plugin.
   - The plugin parses the `<ros2_control>` block and starts a **controller_manager** inside the Gazebo process.
   - It loads the controllers from the YAML you gave in `<parameters>` (e.g. `joint_state_broadcaster`, `panda_arm_controller`, `panda_hand_controller`).
   - So you **do not** start `ros2_control_node` or controller spawners from your launch file when using Gazebo – the plugin does it.

**Summary:** Gazebo runs the world; you spawn one model whose URDF includes the gz_ros2_control plugin and controller YAML; that plugin runs the controller_manager and the same controllers MoveIt expects.

---

## Step 5 – TF, robot_state_publisher, and MoveIt

After the Panda is spawned in Gazebo:

- The **gz_ros2_control** plugin (via `joint_state_broadcaster`) publishes **`/joint_states`**.
- You still need a **TF tree** so MoveIt and RViz know where the robot is. Typically:
  - **world → panda_link0:** use a static transform (same as today: `world` to `panda_link0`).
  - **panda_link0 → … → panda_link8:** from robot_state_publisher, which subscribes to `/joint_states` and uses the **robot description** to compute link poses.

So in your **launch file** (when we wire everything together):

1. **Do not** start `ros2_control_node` or the controller spawners (Gazebo plugin does that).
2. **Do** start:
   - **static_transform_publisher:** `world` → `panda_link0` (same as current launch).
   - **robot_state_publisher** with the **same** Gazebo robot description (so it listens to `/joint_states` from Gazebo and publishes TF).
3. Then start **move_group**, **pose_goal**, and optionally **RViz** as you do now. They use:
   - `/robot_description` (the Gazebo Panda description)
   - `/joint_states` (from Gazebo)
   - TF (from robot_state_publisher + static transform).

**Important:** MoveIt and RViz need the **same** robot description that Gazebo uses (same links/joints). So use the same Gazebo-ready URDF/xacro for both spawn and for `robot_state_publisher` / MoveIt.

---

## Step 6 – Launch order (conceptual)

A single launch file for “Panda in Gazebo + MoveIt” would do something like:

1. Start **Gazebo** with the world (e.g. `empty.sdf`).
2. **Wait** for the world to be up (optional but robust: use a timer or a “world ready” check).
3. **Spawn** the Panda model (URDF with gz_ros2_control plugin).
4. Start **static_transform_publisher** (world → panda_link0).
5. Start **robot_state_publisher** with the Gazebo Panda description.
6. Start **move_group** with the same description and your MoveIt config (SRDF, pipelines, etc.).
7. Start **pose_goal** and **RViz** as you do today.

No `ros2_control_node` or controller spawner nodes; the plugin inside Gazebo is the controller_manager.

---

## Step 7 – Quick reference

| Component              | Where it runs        | Purpose |
|------------------------|----------------------|--------|
| Gazebo + world         | `ros_gz_sim`         | Physics sim; runs the world and the Panda model. |
| gz_ros2_control plugin | Inside Gazebo        | Starts controller_manager, loads controllers, bridges sim joints ↔ ros2_control. |
| /joint_states          | From plugin (broadcaster) | Joint positions/velocities from Gazebo. |
| robot_state_publisher  | ROS 2 node           | Subscribes to /joint_states, publishes TF from robot description. |
| static_transform_publisher | ROS 2 node       | world → panda_link0. |
| move_group             | ROS 2 node           | Motion planning (unchanged). |
| pose_goal              | ROS 2 node           | Sends goals and executes (unchanged). |

---

## Next implementation steps

1. **Add a Gazebo-ready Panda description** (xacro/URDF with `GazeboSimSystem` and the gz plugin, and controller YAML path).
2. **Add a launch file** that: starts Gazebo world → spawns Panda → starts static TF + robot_state_publisher → move_group → pose_goal (+ optional RViz).
3. **Test:** run the launch; in Gazebo you should see the arm; `pose_goal` should plan and execute and the arm should move in Gazebo.

If you want, the next step can be drafting the actual Gazebo Panda xacro and the combined launch file in your repo layout.
