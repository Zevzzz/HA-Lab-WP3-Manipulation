# HA Lab WP3 Manipulation (WIP)

## Overview

- **Pipeline:** object PC → GraspGen (ZMQ) → grasps YAML → MoveItPy (ExecutePose) → Isaac (`/joint_command`).
- **ROS (container only):** `moveitpy_execute_node` — executor, trajectory bridge, `grasp_with_candidates`, optional ground at `z=0` in `panda_link0`.
- **GraspGen:** **ZMQ server** in `deps/GraspGen` (docker / GPU). **Host Python client** — `graspgen_request.py`, `visualize_grasps.py` — always run from repo root after **`source scripts/venv/bin/activate`** (pyzmq, Open3D, etc.; not the ROS container).
- **Isaac Sim (host):** Your stage needs a **ROS ↔ sim bridge**: publish **`/joint_states`** (arm + gripper) and **`/clock`**, subscribe to **`/joint_command`** so MoveIt’s trajectories drive the robot. **`./scripts/run_isaac_sim.sh`** sets **`ROS_DOMAIN_ID=0`** and the Fast DDS profile so discovery matches the dev container. Optional kit USD/URDF: `ws/src/moveitpy_execute_node/urdf/panda_isaac/`.

## Build (inside ROS container)

```bash
docker compose run --rm sim bash   # repo root
cd ~/ws && colcon build --symlink-install && source install/setup.bash
```

**Host — GraspGen Python (not ROS):** from repo root, activate venv before any client script:

```bash
source scripts/venv/bin/activate
```

No `colcon` for these.

## Main workflow (obj → Isaac)

1. **Isaac Sim (host) — import & place**  
   - Start Isaac (`ROS_DOMAIN_ID`, DDS profile):  
   ```bash
   ./scripts/run_isaac_sim.sh
   ```  
   - Import mesh: `File → Import` or drag `.obj` / `.usd`; place on table. **Do this before PLY axis alignment** — sim prim orientation is the reference.  
   - **Object centroid in sim:** After placement, note the **point cloud centroid** in **`panda_link0`** (meters) — e.g. Isaac prim/world readout + offset to centroid, or a small probe script. Use that for execution:  
     - **CLI:** `grasp_with_candidates --object-center X Y Z` (step 10), and `--object-yaw-deg` / `--object-rpy-deg` if the object is rotated on the table.  
     - **Code defaults:** `ws/src/moveitpy_execute_node/moveitpy_execute_node/grasp_with_candidates.py` — `DEFAULT_OBJECT_X_M`, `DEFAULT_OBJECT_Y_M`, `DEFAULT_TABLE_Z_M` (used with YAML `object_half_height_m` when `--object-center` is omitted).

2. **CloudCompare — mesh → PLY**  
   - Surface sample → export `.ply` (vertices OK).  
   - Target ~2k–8k points (see *Point cloud & top-k*).

3. **Check axes (CloudCompare)**  
   - Grasps = **centroid frame** of the PLY you send to GraspGen.  
   - Rotate / save the mesh or cloud in CloudCompare so axes match **how the object sits in Isaac** (no CLI rotation in `graspgen_request.py`).

4. **Put files in repo `data/`**  
   - One folder per object, e.g. `data/Mug/Mug_2011.ply`; after generation, `Mug_2011_grasps.yaml` lives beside it.  
   - CSV logs: `data/logs/graspgen_generations.csv`, `grasp_execution_results.csv`.  
   - **In Docker** the same tree is **`/home/ros/data`** (compose bind-mount) — use that prefix for `--path` inside the container.

5. **GraspGen server** (GPU machine / GraspGen docker)  
   ```bash
   source scripts/run_graspgen_server.sh

   python client-server/graspgen_server.py \
     --gripper_config /models/checkpoints/graspgen_franka_panda.yml \
     --port 5557
   ```

6. **PLY → YAML** (repo root, **venv on**)  
   ```bash
   source scripts/venv/bin/activate

   python scripts/graspgen_request.py data/Mug/Mug_2011.ply --port 5557 --topk 50
   ```

7. **Visualize (optional)** (same venv)  
   ```bash
   source scripts/venv/bin/activate

   python scripts/visualize_grasps.py data/Mug/Mug_2011_grasps.yaml --center-pointcloud --only-index 0
   ```  
   - RGB triad = grasp X/Y/Z; R/G/B = world X/Y/Z.

8. **Isaac — Play + ROS bridge**  
   - Same stage: **Play**; scene publishes `/joint_states`, `/clock` and subscribes `/joint_command`.

9. **ROS stack (container only — do not run `ros2` on bare host for this stack)**  
   ```bash
   docker compose run --rm sim bash   # repo root
   cd ~/ws && source install/setup.bash   # rebuild with colcon if src changed
   ros2 launch moveitpy_execute_node panda_pose_goal_isaac.launch.py
   ```  
   - Extras: `use_rviz:=true`, `add_ground_collision:=false`.

10. **Execute grasps** (same container shell)  
   ```bash
   ros2 run moveitpy_execute_node grasp_with_candidates --path /home/ros/data/Mug/Mug_2011_grasps.yaml
   ```  
   - Match sim: `--object-center`, `--object-yaw-deg`; `--sim-from-pc-frame-rpy-deg` only if a fixed PC↔sim offset remains after CloudCompare.  
   - Log: `data/logs/grasp_execution_results.csv`.

---

### GraspGen server & client

- **Server:** `source scripts/run_graspgen_server.sh` → GraspGen docker + `GraspGenModels`; run `graspgen_server.py` **inside** that container (not `scripts/venv`).
- **Client (`graspgen_request.py`, `visualize_grasps.py`):** on the **host**, repo root → **`source scripts/venv/bin/activate`** every time (deps: pyzmq, msgpack, Open3D, …).
- Client **`--port`** = server **`--port`** (default **5557**). REQ/REP — keep server running while the client runs.

### Point cloud & top-k

- **N:** ~2k–8k shell points; **N < 2048** → client turns off server outlier removal (or `--no-remove-outliers`).
- **Server outlier removal** can drop **all** points on some clouds (thin / flat parts, e.g. laptops) → fast error like `reshape ... [-1, 0, 3]`. Retry with **`--no-remove-outliers`** on `graspgen_request.py`.
- **`--topk`:** returned ranked grasps (default **50**); ↑ = more retries, slower gen.

### Axes & frames

- **Align in CloudCompare** (or your mesh tool) before sampling the PLY; `graspgen_request.py` only centers — no `--sim-frame-rpy-deg`.
- **`--sim-from-pc-frame-rpy-deg`** on `grasp_with_candidates` / `visualize_grasps.py`: optional runtime PC→sim rotation if something is still off (prefer fixing the PLY first).
- **`--rotate-pc-only-rpy-deg`** (visualize): rotate cloud only, not grasp frames (different from `--sim-from-pc-frame-rpy-deg`).
- **`--no-align-graspgen-franka-fingers`:** GraspGen’s TCP uses **+X = finger closing**; Franka **`panda_hand`** joints move along **±Y**. By default we apply a **fixed 90°-class rotation** so goals match MoveIt’s hand frame. Pass this flag **only** if your GraspGen checkpoint / YAML is **already** expressed in URDF `panda_hand` axes (double correction otherwise).

### Isaac & ROS / DDS

- Host Isaac + container ROS must share **`ROS_DOMAIN_ID`** (script uses **0**; compose default **0**).
- **`./scripts/run_isaac_sim.sh`** — exports `FASTRTPS_DEFAULT_PROFILES_FILE` to `docker_setup/fastdds_profile.xml` (file must exist).
- **`panda_pose_goal_isaac.launch.py`:** `move_group` + bridge + executor + `world`→`panda_link0`; **`ground_plane_scene`** = floor at **z=0** (`panda_link0`).
- **`panda_eval.launch.py`:** `ros2_control` + optional Isaac relay (`use_isaac:=true`).
- **`--table-z` / `--object-center`:** match sim table; YAML may have `object_half_height_m`.

### Data paths in Docker

- Compose: `./data` → **`/home/ros/data`** — use `--path /home/ros/data/Mug/...` in container.
- Run **`docker compose` from repo root** so that mount is this repo’s `data/`.

### Troubleshooting: every grasp “Planning failed”

- Usually **workspace z bounds** vs table height — align **`--table-z`** with sim; launch sets `default_workspace_bounds` for low table goals.

## TODO

- GraspGen / humanoid eval metrics; Isaac ↔ MoveIt notes.
