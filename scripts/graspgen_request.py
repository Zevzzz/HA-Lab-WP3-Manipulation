#!/usr/bin/env python3
from __future__ import annotations

"""
Load point cloud from file, send to GraspGen ZMQ server, write returned grasps to YAML.

- Point cloud is centered (subtract mean) before sending, matching GraspGen's official client.
  Grasps are in the same orthonormal frame as that centered cloud — align axes in the tool
  that exports the PLY (e.g. CloudCompare), not in this script.
- Writes object_half_height_m (half of axis-aligned bbox height in z) on the centered cloud so
  downstream can place the object: object_center_z = table_z + object_half_height_m.

Uses the same wire protocol as grasp_gen.serving.zmq_client (msgpack + msgpack_numpy).
Deps: pyzmq, msgpack, msgpack-numpy, numpy, scipy, trimesh, PyYAML.
"""

import argparse
import csv
import sys
import time
from datetime import datetime
from pathlib import Path

import msgpack
import msgpack_numpy
import numpy as np
import yaml
import zmq
from scipy.spatial.transform import Rotation

msgpack_numpy.patch()

DEFAULT_LOG_DIR = Path("data/logs")
GENERATIONS_LOG_NAME = "graspgen_generations.csv"

YAML_KEY_OBJECT_HALF_HEIGHT_M = "object_half_height_m"


def load_point_cloud(path: Path) -> np.ndarray:
    """(N, 3) float32. Supports .npy and .ply (trimesh for vertices)."""
    path = Path(path).resolve()
    if not path.is_file():
        raise FileNotFoundError(path)
    suf = path.suffix.lower()
    if suf == ".npy":
        pc = np.asarray(np.load(path), dtype=np.float64)[:, :3]
    elif suf == ".ply":
        import trimesh
        geom = trimesh.load(str(path), process=False)
        pc = np.asarray(geom.vertices, dtype=np.float64)[:, :3]
    else:
        raise ValueError(f"Use .npy or .ply, got {suf}")
    if len(pc) < 3:
        raise ValueError("Too few points")
    return np.asarray(pc, dtype=np.float32)


def center_point_cloud(pc: np.ndarray) -> np.ndarray:
    """Center point cloud so origin is at centroid (matches GraspGen client)."""
    return np.asarray(pc - np.mean(pc, axis=0), dtype=pc.dtype)


def bbox_half_height_z(pc: np.ndarray) -> float:
    """Half of axis-aligned bbox extent in z (meters). pc is (N, 3)."""
    z_min, z_max = float(np.min(pc[:, 2])), float(np.max(pc[:, 2]))
    return (z_max - z_min) / 2.0


def request_grasps(
    point_cloud: np.ndarray,
    host: str,
    port: int,
    num_grasps: int,
    topk_num_grasps: int,
    *,
    remove_outliers: bool = True,
) -> tuple[np.ndarray, np.ndarray, dict]:
    """Send point cloud to GraspGen server; returns (grasps, confidences, timing_info).
    num_grasps/topk_num_grasps are set by main() from --topk (generate and return that many).
    """
    point_cloud = np.asarray(point_cloud, dtype=np.float32)
    if point_cloud.ndim != 2 or point_cloud.shape[1] != 3:
        raise ValueError(f"point_cloud must be (N, 3), got {point_cloud.shape}")

    payload = {
        "action": "infer",
        "point_cloud": point_cloud,
        "grasp_threshold": -1.0,
        "num_grasps": num_grasps,
        "topk_num_grasps": topk_num_grasps,
        "min_grasps": 1,
        "max_tries": 6,
        "remove_outliers": bool(remove_outliers),
    }

    ctx = zmq.Context()
    sock = ctx.socket(zmq.REQ)
    sock.setsockopt(zmq.RCVTIMEO, 60_000)
    sock.setsockopt(zmq.SNDTIMEO, 60_000)
    sock.setsockopt(zmq.LINGER, 0)
    sock.connect(f"tcp://{host}:{port}")
    n_pts = point_cloud.shape[0]
    print(f"Sending request to {host}:{port} ({n_pts} points, num_grasps={num_grasps}, topk={topk_num_grasps})...", flush=True)
    t0 = time.perf_counter()
    try:
        sock.send(msgpack.packb(payload, use_bin_type=True))
        print("Waiting for server (inference runs on server; client blocks here until response)...", flush=True)
        raw = sock.recv()
        elapsed = time.perf_counter() - t0
        print(f"Received response in {elapsed:.1f} s", flush=True)
        response = msgpack.unpackb(raw, raw=False)
    finally:
        sock.close()
        ctx.term()

    if "error" in response:
        raise RuntimeError(f"Server error: {response['error']}")
    grasps = np.asarray(response["grasps"], dtype=np.float64)
    confidences = np.asarray(response["confidences"], dtype=np.float64)
    timing_info = response.get("timing") or {}
    return grasps, confidences, timing_info


def matrix4_to_pose(T: np.ndarray) -> tuple[list[float], list[float]]:
    """4x4 transform -> (position [x,y,z], quat [x,y,z,w]) using scipy."""
    position = T[:3, 3].tolist()
    quat_xyzw = Rotation.from_matrix(T[:3, :3]).as_quat().tolist()  # scipy uses (x,y,z,w)
    return position, quat_xyzw


def _log_generation(
    log_dir: Path,
    filename: str,
    num_candidates: int,
    generation_time_s: float | None,
) -> None:
    """Append one row to data/logs/graspgen_generations.csv."""
    log_dir = Path(log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)
    log_file = log_dir / GENERATIONS_LOG_NAME
    write_header = not log_file.exists()
    with open(log_file, "a", newline="") as f:
        w = csv.writer(f)
        if write_header:
            w.writerow(["timestamp_iso", "filename", "num_candidates", "generation_time_s"])
        ts = datetime.utcnow().isoformat() + "Z"
        time_val = f"{generation_time_s:.4f}" if generation_time_s is not None else "n/a"
        w.writerow([ts, filename, num_candidates, time_val])


def main():
    p = argparse.ArgumentParser(description="GraspGen ZMQ client: point cloud -> grasps YAML")
    p.add_argument("path", type=Path, help="Point cloud .npy or .ply")
    p.add_argument("--host", default="localhost", help="Server host")
    p.add_argument("--port", type=int, default=5557, help="GraspGen server port (default 5557, match server --port)")
    p.add_argument("--topk", type=int, default=50, help="topk_num_grasps (-1 = use threshold)")
    p.add_argument("-o", "--output", type=Path, default=None, help="Output YAML path")
    p.add_argument("--frame-id", default="object", help="frame_id in YAML (use 'object' for centroid frame)")
    p.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR, help="Dir for generation log CSV (default: data/logs)")
    p.add_argument(
        "--no-remove-outliers",
        action="store_true",
        help="Send remove_outliers=false to the server (GraspGen can delete entire small clouds otherwise).",
    )
    args = p.parse_args()

    topk = args.topk if args.topk > 0 else -1
    num_grasps = topk if topk > 0 else 200
    topk_num_grasps = topk

    print(f"Loading point cloud from {args.path}...", flush=True)
    point_cloud = load_point_cloud(args.path)
    point_cloud_centered = center_point_cloud(point_cloud)
    print(f"Loaded {len(point_cloud)} points, centered.", flush=True)
    point_cloud_for_gen = point_cloud_centered

    n_send = int(point_cloud_for_gen.shape[0])
    remove_outliers = not args.no_remove_outliers
    if remove_outliers and n_send < 2048:
        print(
            f"Note: only {n_send} points — disabling server outlier removal (it often removes the whole cloud). "
            "Use a denser PLY or pass --no-remove-outliers explicitly.",
            flush=True,
        )
        remove_outliers = False

    grasps, confidences, timing_info = request_grasps(
        point_cloud_for_gen,
        args.host,
        args.port,
        num_grasps=num_grasps,
        topk_num_grasps=topk_num_grasps,
        remove_outliers=remove_outliers,
    )

    num_candidates = len(grasps)
    infer_ms = timing_info.get("infer_ms")
    generation_time_s = (infer_ms / 1000.0) if infer_ms is not None else None
    _log_generation(args.log_dir, args.path.name, num_candidates, generation_time_s)

    half_height_m = bbox_half_height_z(point_cloud_for_gen)

    out = args.output or args.path.with_stem(args.path.stem + "_grasps").with_suffix(".yaml")
    out.parent.mkdir(parents=True, exist_ok=True)
    candidates = []
    for i in range(len(grasps)):
        pos, ori = matrix4_to_pose(grasps[i])
        candidates.append({
            "position": [float(round(x, 6)) for x in pos],
            "orientation": [float(round(x, 6)) for x in ori],
            "confidence": float(round(confidences[i], 6)),
        })
    doc = {
        "frame_id": args.frame_id,
        "num_grasps": len(candidates),
        YAML_KEY_OBJECT_HALF_HEIGHT_M: round(half_height_m, 6),
        "grasps": candidates,
    }
    with open(out, "w") as f:
        yaml.safe_dump(doc, f, default_flow_style=False, sort_keys=False)

    print(f"Wrote {num_candidates} grasps to {out}")
    if generation_time_s is not None:
        print(f"Generation time: {generation_time_s:.3f} s (logged to {args.log_dir / GENERATIONS_LOG_NAME})")
    return 0


if __name__ == "__main__":
    sys.exit(main())

