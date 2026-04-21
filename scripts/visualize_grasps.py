#!/usr/bin/env python3
"""
Visualize grasps from a YAML file (from graspgen_request.py) with the point cloud.

World axes at the origin use Open3D's coordinate frame: red = +X, green = +Y, blue = +Z
(right-handed). Compare this triad to your Isaac / panda_link0 frame when checking conventions.

Usage: python visualize_grasps.py Mug8192_grasps.yaml [Mug8192.ply]
       python visualize_grasps.py path/to/grasps.yaml --only-index 0
       python visualize_grasps.py path/to/grasps.yaml --top 30
       python visualize_grasps.py path/to/grasps.yaml --pc path/to/pointcloud.ply
       python visualize_grasps.py path/to/grasps.yaml --center-pointcloud
       python visualize_grasps.py path/to/grasps.yaml --sim-from-pc-frame-rpy-deg -90 0 0

Use the same ``--sim-from-pc-frame-rpy-deg`` as ``grasp_with_candidates`` when checking alignment.

Requires: open3d, pyyaml, numpy, scipy. For .ply: trimesh.
"""

import argparse
import sys
from pathlib import Path
from typing import Optional

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

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


def rotation_matrix_from_rpy_xyz_deg(rpy_deg: tuple[float, float, float]) -> np.ndarray:
    return Rotation.from_euler("xyz", list(rpy_deg), degrees=True).as_matrix()


def main():
    p = argparse.ArgumentParser(description="Visualize grasps YAML + point cloud")
    p.add_argument("yaml_path", type=Path, help="Grasps YAML (from graspgen_request.py)")
    p.add_argument("pc_path", type=Path, nargs="?", default=None, help="Point cloud .ply or .npy (default: same stem as yaml with .ply)")
    p.add_argument("--max-grasps", type=int, default=None, metavar="N", help="Show at most N grasp frames (default: all)")
    p.add_argument(
        "--top",
        type=int,
        default=None,
        metavar="N",
        help="Show first N grasps (0-based indices 0 .. N-1). E.g. --top 30 → indices 0–29.",
    )
    p.add_argument(
        "--only-index",
        type=int,
        default=None,
        metavar="I",
        help="Show only grasp at index I (0-based). E.g. grasp_with_candidates 'Candidate 1' -> --only-index 0",
    )
    p.add_argument(
        "--world-axis-length",
        type=float,
        default=0.12,
        metavar="M",
        help="Length (m) of RGB world axes at origin (default: 0.12)",
    )
    p.add_argument(
        "--no-world-axes",
        action="store_true",
        help="Do not draw the world X/Y/Z triad at the origin",
    )
    p.add_argument(
        "--grasp-frame-size",
        type=float,
        default=0.03,
        metavar="M",
        help="Size (m) of each grasp coordinate frame (default: 0.03)",
    )
    p.add_argument(
        "--center-pointcloud",
        action="store_true",
        help="Subtract point cloud mean so origin matches GraspGen centroid frame (use if .ply was not saved centered)",
    )
    p.add_argument(
        "--sim-from-pc-frame-rpy-deg",
        nargs=3,
        type=float,
        default=None,
        metavar=("RX", "RY", "RZ"),
        help="Rotate centered cloud AND grasps (same as grasp_with_candidates). Intrinsic XYZ deg.",
    )
    p.add_argument(
        "--rotate-pc-only-rpy-deg",
        nargs=3,
        type=float,
        default=None,
        metavar=("RX", "RY", "RZ"),
        help="Rotate point cloud only; grasp frames unchanged (e.g. grasps in sim frame, PLY still in mesh export frame).",
    )
    args = p.parse_args()

    yaml_path = Path(args.yaml_path).resolve()
    if not yaml_path.is_file():
        print(f"Not found: {yaml_path}", file=sys.stderr)
        return 1

    with open(yaml_path) as f:
        data = yaml.safe_load(f)
    if args.sim_from_pc_frame_rpy_deg is not None and args.rotate_pc_only_rpy_deg is not None:
        print("Use only one of --sim-from-pc-frame-rpy-deg and --rotate-pc-only-rpy-deg.", file=sys.stderr)
        return 1

    align_rpy: Optional[tuple[float, float, float]] = None
    align_source = ""
    if args.sim_from_pc_frame_rpy_deg is not None:
        align_rpy = (
            float(args.sim_from_pc_frame_rpy_deg[0]),
            float(args.sim_from_pc_frame_rpy_deg[1]),
            float(args.sim_from_pc_frame_rpy_deg[2]),
        )
        align_source = "CLI --sim-from-pc-frame-rpy-deg"
    pc_only_rpy: Optional[tuple[float, float, float]] = None
    if args.rotate_pc_only_rpy_deg is not None:
        pc_only_rpy = (
            float(args.rotate_pc_only_rpy_deg[0]),
            float(args.rotate_pc_only_rpy_deg[1]),
            float(args.rotate_pc_only_rpy_deg[2]),
        )
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

    if args.center_pointcloud:
        pts = np.asarray(pcd.points)
        mean = pts.mean(axis=0)
        pcd.points = o3d.utility.Vector3dVector(pts - mean)
        print(f"Centered point cloud (subtracted mean {mean[0]:.6f}, {mean[1]:.6f}, {mean[2]:.6f}).")

    T_sim_from_pc = np.eye(4)
    transform_grasps = True
    if pc_only_rpy is not None and any(abs(x) > 1e-9 for x in pc_only_rpy):
        r_mat = rotation_matrix_from_rpy_xyz_deg(pc_only_rpy)
        pts = np.asarray(pcd.points)
        pcd.points = o3d.utility.Vector3dVector(pts @ r_mat.T)
        transform_grasps = False
        print(
            f"PC-only rotation rpy deg {pc_only_rpy} (--rotate-pc-only-rpy-deg): "
            "point cloud rotated; grasp frames unchanged."
        )
    elif align_rpy is not None and any(abs(x) > 1e-9 for x in align_rpy):
        r_mat = rotation_matrix_from_rpy_xyz_deg(align_rpy)
        T_sim_from_pc[:3, :3] = r_mat
        pts = np.asarray(pcd.points)
        pcd.points = o3d.utility.Vector3dVector(pts @ r_mat.T)
        print(
            f"PC→sim rotation rpy deg {align_rpy} ({align_source}): "
            "applied to point cloud and grasp frames (same as grasp_with_candidates)."
        )

    n_select = sum(
        1 for x in (args.only_index, args.max_grasps, args.top) if x is not None
    )
    if n_select > 1:
        print("Use only one of --only-index, --max-grasps, and --top.", file=sys.stderr)
        return 1
    if args.top is not None and args.top < 1:
        print("--top N requires N >= 1.", file=sys.stderr)
        return 1

    frame_size = args.grasp_frame_size
    geoms = [pcd]

    if not args.no_world_axes:
        world_axes = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=args.world_axis_length, origin=[0, 0, 0]
        )
        geoms.insert(0, world_axes)

    if args.only_index is not None:
        i = args.only_index
        if i < 0 or i >= len(grasps_list):
            print(f"--only-index {i} out of range [0, {len(grasps_list) - 1}].", file=sys.stderr)
            return 1
        indices = [i]
    else:
        if args.top is not None:
            n = min(args.top, len(grasps_list))
        elif args.max_grasps is not None:
            n = min(args.max_grasps, len(grasps_list))
        else:
            n = len(grasps_list)
        indices = list(range(n))

    for i in indices:
        g = grasps_list[i]
        pos = g["position"]
        ori = g["orientation"]
        t_grasp = pose_to_matrix(pos, ori)
        t_show = T_sim_from_pc @ t_grasp if transform_grasps else t_grasp
        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=frame_size, origin=[0, 0, 0])
        frame.transform(t_show)
        geoms.append(frame)

    if args.only_index is not None:
        print(f"Showing point cloud + grasp index {args.only_index} only (of {len(grasps_list)}). Close window to exit.")
    else:
        if not indices:
            print("No grasp indices to show (check --top / --max-grasps).", file=sys.stderr)
            return 1
        lo, hi = indices[0], indices[-1]
        print(
            f"Showing point cloud + {len(indices)} grasp frames "
            f"(indices {lo}–{hi} of {len(grasps_list)}). Close window to exit."
        )
    if not args.no_world_axes:
        print(
            "World axes at origin: red = +X, green = +Y, blue = +Z (Open3D right-handed). "
            "Grasp frames use the same RGB = local XYZ."
        )
    o3d.visualization.draw_geometries(geoms, window_name="Grasps")
    return 0


if __name__ == "__main__":
    sys.exit(main())

