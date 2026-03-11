#!/usr/bin/env python3
"""
Visualize grasps from a YAML file (from graspgen_request.py) with the point cloud.
Usage: python visualize_grasps.py Mug8192_grasps.yaml [Mug8192.ply]
       python visualize_grasps.py path/to/grasps.yaml --pc path/to/pointcloud.ply
Requires: open3d, pyyaml, numpy. For .ply: trimesh.
"""

import argparse
import sys
from pathlib import Path

import numpy as np
import yaml

try:
    import open3d as o3d
except ImportError:
    print("Install Open3D: pip install open3d", file=sys.stderr)
    sys.exit(1)


def load_pc(path: Path) -> "o3d.geometry.PointCloud":
    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError(path)
    suf = path.suffix.lower()
    if suf == ".ply":
        pcd = o3d.io.read_point_cloud(str(path))
    elif suf == ".npy":
        pts = np.load(path)
        pts = np.asarray(pts, dtype=np.float64)[:, :3]
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)
    else:
        raise ValueError(f"Use .ply or .npy, got {suf}")
    return pcd


def pose_to_matrix(pos, ori) -> np.ndarray:
    """position (3,) + quat (x,y,z,w) -> 4x4."""
    T = np.eye(4)
    x, y, z, w = float(ori[0]), float(ori[1]), float(ori[2]), float(ori[3])
    T[0, 0] = 1 - 2 * (y * y + z * z)
    T[0, 1] = 2 * (x * y - z * w)
    T[0, 2] = 2 * (x * z + y * w)
    T[1, 0] = 2 * (x * y + z * w)
    T[1, 1] = 1 - 2 * (x * x + z * z)
    T[1, 2] = 2 * (y * z - x * w)
    T[2, 0] = 2 * (x * z - y * w)
    T[2, 1] = 2 * (y * z + x * w)
    T[2, 2] = 1 - 2 * (x * x + y * y)
    T[:3, 3] = [float(p) for p in pos]
    return T


def main():
    p = argparse.ArgumentParser(description="Visualize grasps YAML + point cloud")
    p.add_argument("yaml_path", type=Path, help="Grasps YAML (from graspgen_request.py)")
    p.add_argument("pc_path", type=Path, nargs="?", default=None, help="Point cloud .ply or .npy (default: same stem as yaml with .ply)")
    p.add_argument("--max-grasps", type=int, default=20, help="Show at most this many grasp frames (default 20)")
    args = p.parse_args()

    yaml_path = Path(args.yaml_path).resolve()
    if not yaml_path.is_file():
        print(f"Not found: {yaml_path}", file=sys.stderr)
        return 1

    with open(yaml_path) as f:
        data = yaml.safe_load(f)
    grasps_list = data.get("grasps") or []
    if not grasps_list:
        print("No grasps in YAML.", file=sys.stderr)
        return 1

    pc_path = args.pc_path
    if pc_path is None:
        # e.g. Mug8192_grasps.yaml -> Mug8192.ply
        stem = yaml_path.stem.replace("_grasps", "")
        for ext in (".ply", ".npy"):
            candidate = yaml_path.parent / (stem + ext)
            if candidate.is_file():
                pc_path = candidate
                break
        if pc_path is None:
            print("No point cloud path given and none found next to YAML.", file=sys.stderr)
            return 1
    pc_path = Path(pc_path).resolve()

    pcd = load_pc(pc_path)
    if len(pcd.points) == 0:
        print("Point cloud is empty.", file=sys.stderr)
        return 1

    frame_size = 0.03
    geoms = [pcd]
    n = min(args.max_grasps, len(grasps_list))
    for i in range(n):
        g = grasps_list[i]
        pos = g["position"]
        ori = g["orientation"]
        T = pose_to_matrix(pos, ori)
        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=frame_size, origin=[0, 0, 0])
        frame.transform(T)
        geoms.append(frame)

    print(f"Showing point cloud + {n} grasp frames (of {len(grasps_list)}). Close window to exit.")
    o3d.visualization.draw_geometries(geoms, window_name="Grasps")
    return 0


if __name__ == "__main__":
    sys.exit(main())

