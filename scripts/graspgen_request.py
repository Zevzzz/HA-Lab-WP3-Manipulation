#!/usr/bin/env python3
"""
Load point cloud from file, send to GraspGen ZMQ server, write returned grasps to YAML.
Uses the same wire protocol as grasp_gen.serving.zmq_client (msgpack + msgpack_numpy).
No GraspGen package required; deps: pyzmq, msgpack, msgpack-numpy, numpy, trimesh, PyYAML.
"""

import argparse
import sys
from pathlib import Path

import msgpack
import msgpack_numpy
import numpy as np
import yaml
import zmq

msgpack_numpy.patch()


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


def request_grasps(
    point_cloud: np.ndarray,
    host: str,
    port: int,
    num_grasps: int = 200,
    topk_num_grasps: int = 50,
) -> tuple[np.ndarray, np.ndarray]:
    """Protocol matches GraspGen zmq_client: action=infer, msgpack_numpy, use_bin_type=True."""
    point_cloud = np.asarray(point_cloud, dtype=np.float32)
    if point_cloud.ndim != 2 or point_cloud.shape[1] != 3:
        raise ValueError(f"point_cloud must be (N, 3), got {point_cloud.shape}")

    payload = {
        "action": "infer",
        "point_cloud": point_cloud,
        "grasp_threshold": -1.0,
        "num_grasps": num_grasps,
        "topk_num_grasps": topk_num_grasps,
        "min_grasps": 40,
        "max_tries": 6,
        "remove_outliers": True,
    }

    ctx = zmq.Context()
    sock = ctx.socket(zmq.REQ)
    sock.setsockopt(zmq.RCVTIMEO, 60_000)
    sock.setsockopt(zmq.SNDTIMEO, 60_000)
    sock.setsockopt(zmq.LINGER, 0)
    sock.connect(f"tcp://{host}:{port}")
    try:
        sock.send(msgpack.packb(payload, use_bin_type=True))
        raw = sock.recv()
        response = msgpack.unpackb(raw, raw=False)
    finally:
        sock.close()
        ctx.term()

    if "error" in response:
        raise RuntimeError(f"Server error: {response['error']}")
    grasps = np.asarray(response["grasps"], dtype=np.float64)
    confidences = np.asarray(response["confidences"], dtype=np.float64)
    return grasps, confidences


def _rot_to_quat(R):
    """3x3 rotation -> quat (x,y,z,w)."""
    t = R[0, 0] + R[1, 1] + R[2, 2]
    if t > 0:
        s = 0.5 / (t + 1) ** 0.5
        w, x = 0.25 / s, (R[2, 1] - R[1, 2]) * s
        y, z = (R[0, 2] - R[2, 0]) * s, (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > max(R[1, 1], R[2, 2]):
        s = 2 * (1 + R[0, 0] - R[1, 1] - R[2, 2]) ** 0.5
        w, x = (R[2, 1] - R[1, 2]) / s, 0.25 * s
        y, z = (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2 * (1 + R[1, 1] - R[0, 0] - R[2, 2]) ** 0.5
        w, x = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s
        y, z = 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = 2 * (1 + R[2, 2] - R[0, 0] - R[1, 1]) ** 0.5
        w = (R[1, 0] - R[0, 1]) / s
        x, y = (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return [x, y, z, w]


def matrix4_to_pose(T):
    """4x4 -> position [x,y,z], quat [x,y,z,w]."""
    return T[:3, 3].tolist(), _rot_to_quat(T[:3, :3])


def main():
    p = argparse.ArgumentParser(description="GraspGen ZMQ client: point cloud -> grasps YAML")
    p.add_argument("path", type=Path, help="Point cloud .npy or .ply")
    p.add_argument("--host", default="localhost", help="Server host")
    p.add_argument("--port", type=int, default=5556, help="Server port")
    p.add_argument("--topk", type=int, default=50, help="topk_num_grasps (-1 = use threshold)")
    p.add_argument("-o", "--output", type=Path, default=None, help="Output YAML path")
    p.add_argument("--frame-id", default="panda_link0", help="frame_id in YAML for ROS")
    args = p.parse_args()

    point_cloud = load_point_cloud(args.path)
    grasps, confidences = request_grasps(
        point_cloud, args.host, args.port,
        num_grasps=200,
        topk_num_grasps=args.topk if args.topk > 0 else -1,
    )

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
    doc = {"frame_id": args.frame_id, "num_grasps": len(candidates), "grasps": candidates}
    with open(out, "w") as f:
        yaml.safe_dump(doc, f, default_flow_style=False, sort_keys=False)

    print(f"Wrote {len(candidates)} grasps to {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

