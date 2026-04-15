# HA Lab WP3 Manipulation (WIP)

## Overview

End-to-end path from **object point clouds** → **GraspGen** (ZMQ) → **grasps YAML** → **MoveItPy** (ExecutePose) → **Isaac Sim** via a trajectory bridge. Panda planning uses OMPL; grasps are aligned to Franka `panda_hand` by default. Intended for GraspGen evaluation and later humanoid / RL hooks.

- **ROS:** `moveitpy_execute_node` — executor (MoveItPy + ExecutePose), trajectory bridge (`/joint_command`), `grasp_with_candidates`, optional ground collision at `z=0`.
- **GraspGen:** server in `deps/GraspGen`; client `scripts/graspgen_request.py` (no GraspGen Python package on the client).
- **Isaac:** scene publishes `/joint_states` + `/clock`, subscribes `/joint_command`; match `ROS_DOMAIN_ID` (see `scripts/run_isaac_sim.sh`). Panda USD/URDF under `ws/src/moveitpy_execute_node/urdf/panda_isaac/`.

## Build

```bash
cd ws && colcon build --symlink-install && source install/setup.bash
```

## Main workflow (obj → Isaac)

1. **Mesh → point cloud (CloudCompare)**  
   Sample the object surface; export **`.ply`** (vertices only is fine). Aim for **a few thousand points** on the outer surface (see *Point cloud & top-k* below).

2. **Check axes**  
   Grasp poses are in the **cloud frame after centroid + optional rotation**. The cloud should match how the object sits in sim (e.g. Z-up vs Y-up). Fix in CloudCompare (save rotated PLY) or use `--sim-frame-rpy-deg` when generating (see *Axes & frames*).

3. **Put files in `data/`**  
   Example: `data/Mug/Mug_2011.ply`. Logs go to `data/logs/` by default.

4. **GraspGen ZMQ server** (separate terminal; GPU machine typical)  
   ```bash
   ./scripts/run_graspgen_server.sh   # enters GraspGen docker env
   python client-server/graspgen_server.py \
     --gripper_config /models/checkpoints/graspgen_franka_panda.yml \
     --port 5557
   ```  
   (Paths may match your GraspGen layout; see *GraspGen server*.)

5. **Client: PLY → YAML** (repo root; use `scripts/venv` on host if not in container)  
   ```bash
   source scripts/venv/bin/activate
   python scripts/graspgen_request.py data/Mug/Mug_2011.ply --host <server_host> --port 5557 --topk 50
   ```  
   Writes `data/Mug/Mug_2011_grasps.yaml` and appends a row to `data/logs/graspgen_generations.csv`.

6. **Sanity-check grasps (optional)**  
   ```bash
   python scripts/visualize_grasps.py data/Mug/Mug_2011_grasps.yaml --center-pointcloud --only-index 0
   ```  
   RGB axes = X/Y/Z at centroid; red/green/blue = world X/Y/Z.

7. **Isaac Sim**  
   Play scene; joint bridge wired to ROS (see *Isaac & ROS*).

8. **ROS stack (container or host with same workspace)**  
   ```bash
   source ws/install/setup.bash
   ros2 launch moveitpy_execute_node panda_pose_goal_isaac.launch.py
   ```  
   Optional: `use_rviz:=true`, `add_ground_collision:=false` (ground on by default).

9. **Execute grasps**  
   ```bash
   ros2 run moveitpy_execute_node grasp_with_candidates --path /path/to/Mug_2011_grasps.yaml
   ```  
   Set object pose to match sim, e.g. `--object-center X Y Z`, `--object-yaw-deg …`, and frame alignment `--sim-from-pc-frame-rpy-deg …` if you did not bake rotation into the YAML (see *Axes & frames*). Logs: `data/logs/grasp_execution_results.csv`.

---

### GraspGen server

- `./scripts/run_graspgen_server.sh` — `cd deps/GraspGen` and runs `docker/run.sh` with `GraspGenModels` mounted; run **`graspgen_server.py` inside that environment** so CUDA/torch and checkpoints match upstream.
- Client and server **`--port` must match** (default **5557**).
- Server must stay running while `graspgen_request.py` runs (REQ/REP; one inference per request).

### Point cloud & top-k

- **Count:** Prefer **≥ ~2k–8k** points on the visible shell. Very small clouds can make the server’s outlier step drop everything — the client auto-disables outlier removal when **N < 2048**; you can force **`--no-remove-outliers`**.
- **`--topk`:** How many ranked grasps are returned (default **50**). Higher = more retries for `grasp_with_candidates`, slower generation.

### Axes & frames (common pain)

- YAML grasps are in **object centroid frame** (after mean subtraction; optional `--sim-frame-rpy-deg` at generate time).
- **Do not double-apply** the same rotation: if you used `--sim-frame-rpy-deg` in `graspgen_request.py`, skip `--sim-from-pc-frame-rpy-deg` on `grasp_with_candidates`.
- **Visualize:** If the PLY is not pre-centered, use `--center-pointcloud`. If grasps were generated with `--sim-frame-rpy-deg`, when visualizing against the **raw** PLY use the matching **`--rotate-pc-only-rpy-deg`** (see script help), not `--sim-from-pc-frame-rpy-deg` (that rotates grasps too).
- **Gripper convention:** Default applies a fixed GraspGen→Franka hand rotation; use `--no-align-graspgen-franka-fingers` only if your checkpoint already matches URDF.

### Isaac & ROS

- **`scripts/run_isaac_sim.sh`** — domain ID / DDS profile helper.
- **`panda_pose_goal_isaac.launch.py`:** `move_group` + `trajectory_bridge` + `executor_node` + static `world`→`panda_link0`; **`ground_plane_scene`** adds a floor collision at **`z=0` in `panda_link0`** (disable with `add_ground_collision:=false` if it fights your scene).
- **`panda_eval.launch.py`:** Alternative with `ros2_control` + optional Isaac relay (`use_isaac:=true`).
- **`--table-z` / `--object-center`:** Object vertical placement must match the sim table; YAML may include `object_half_height_m` for centroid height above table.

### Data paths in Docker

- If the sim container mounts repo **`./data` → `/home/ros/data`**, use paths like **`--path /home/ros/data/Mug/...`**. Run **`docker compose` from repo root** so the mount is this repo’s `data/`.

### Troubleshooting: “Planning failed” for every grasp

- Often **workspace bounds**: goals below the default min **z** are rejected. This repo’s Isaac launch sets **`default_workspace_bounds`** for table-height goals; align **`--table-z`** with the sim table, or raise the object.

## TODO

- GraspGen / humanoid eval metrics; tighten Isaac ↔ MoveIt bring-up docs.
