#!/usr/bin/env python3
"""
Service-speed sweep for GraspGen (ZMQ) over the 4 evaluation objects.

For each object folder under ``data/`` (Laptop, Mug, RectPrism, Sphere):

  * Find all PLYs named ``*_<int>.ply``; sort ascending by that int.
  * Expect exactly 4, mapping to target slots [500, 2000, 5000, 10000] points.
  * Run **two OAT sweeps**:
      - Vary N_points across the 4 PLYs with topk fixed at --default-topk.
      - Vary topk across --topk-values with N_points fixed at the 3rd PLY (~5k).
  * Each (config) is repeated --num-trials times.

Every request is logged as one row to ``data/logs/service_speed.csv``
with timing, config, filename, number of candidates, and status.

After the **last trial** of each (sweep, object, setting) combo, if that
request succeeds, writes ``<ply_stem>_eval_<...>_grasps.yaml`` in the **same
folder as the source PLY** (one YAML per combo, overwrites on re-run).

Usage:
    source scripts/venv/bin/activate
    python scripts/service_speed_test.py --num-trials 10
"""

from __future__ import annotations

import argparse
import csv
import re
import sys
import time
import traceback
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Optional

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from graspgen_request import (  # noqa: E402
    YAML_KEY_OBJECT_HALF_HEIGHT_M,
    bbox_half_height_z,
    center_point_cloud,
    load_point_cloud,
    matrix4_to_pose,
    request_grasps,
)


OBJECT_FOLDERS = ["Laptop", "Mug", "RectPrism", "Sphere"]
TARGET_NPOINTS = [500, 2000, 5000, 10000]
DEFAULT_TOPK_VALUES = [50, 100, 200, 400]
DEFAULT_TOPK = 200
DEFAULT_NPOINTS_SLOT_IDX = 2  # 0-based index into TARGET_NPOINTS → 5k

DEFAULT_LOG_DIR = Path("data/logs")
DEFAULT_LOG_NAME = "service_speed.csv"
OUTLIER_REMOVAL_MIN_POINTS = 2048

PLY_NUM_RE = re.compile(r".*_(\d+)\.ply$", re.IGNORECASE)

CSV_HEADER = [
    "timestamp_iso",
    "sweep",             # n_points | topk
    "object",
    "ply_path",
    "ply_points_actual",
    "ply_slot_nominal",  # from TARGET_NPOINTS (slot label)
    "topk",
    "trial_idx",
    "elapsed_client_s",
    "infer_ms",
    "num_candidates",
    "remove_outliers",
    "ok",
    "error",
]


@dataclass
class RequestOutcome:
    ok: bool
    elapsed_s: float
    infer_ms: Optional[float]
    num_candidates: int
    remove_outliers: bool
    error: str
    grasps: Optional[np.ndarray] = None
    confidences: Optional[np.ndarray] = None


def find_plys_sorted(folder: Path) -> list[tuple[int, Path]]:
    """Return list of (int_from_filename, path) sorted ascending by the int."""
    matches: list[tuple[int, Path]] = []
    for p in folder.glob("*.ply"):
        m = PLY_NUM_RE.match(p.name)
        if m:
            matches.append((int(m.group(1)), p))
    matches.sort(key=lambda t: t[0])
    return matches


def _append_row(csv_path: Path, row: dict) -> None:
    write_header = not csv_path.exists()
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    with open(csv_path, "a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=CSV_HEADER)
        if write_header:
            w.writeheader()
        w.writerow(row)


def do_one_request(
    pc_centered: np.ndarray,
    *,
    host: str,
    port: int,
    topk: int,
) -> RequestOutcome:
    """One server request; on success includes grasps/confidences for YAML export."""
    n_pts = int(pc_centered.shape[0])
    remove_outliers = n_pts >= OUTLIER_REMOVAL_MIN_POINTS
    t0 = time.perf_counter()
    try:
        grasps, confidences, timing = request_grasps(
            pc_centered,
            host,
            port,
            num_grasps=topk,
            topk_num_grasps=topk,
            remove_outliers=remove_outliers,
        )
        elapsed = time.perf_counter() - t0
        infer_ms = timing.get("infer_ms") if isinstance(timing, dict) else None
        return RequestOutcome(
            ok=True,
            elapsed_s=elapsed,
            infer_ms=infer_ms,
            num_candidates=len(grasps),
            remove_outliers=remove_outliers,
            error="",
            grasps=grasps,
            confidences=confidences,
        )
    except Exception as e:  # noqa: BLE001
        elapsed = time.perf_counter() - t0
        return RequestOutcome(
            ok=False,
            elapsed_s=elapsed,
            infer_ms=None,
            num_candidates=0,
            remove_outliers=remove_outliers,
            error=f"{type(e).__name__}: {e}",
            grasps=None,
            confidences=None,
        )


def save_final_trial_yaml(
    *,
    ply_path: Path,
    pc_centered: np.ndarray,
    outcome: RequestOutcome,
    sweep: str,
    slot_nominal: int,
    topk: int,
) -> Optional[Path]:
    """Write grasps YAML beside the PLY; same schema as graspgen_request.py."""
    if not outcome.ok or outcome.grasps is None or outcome.confidences is None:
        return None
    if len(outcome.grasps) == 0:
        return None

    stem = ply_path.stem
    if sweep == "n_points":
        out_name = f"{stem}_eval_npts_slot{slot_nominal}_topk{topk}_grasps.yaml"
    else:
        out_name = f"{stem}_eval_topk{topk}_npts_slot{slot_nominal}_grasps.yaml"
    out_path = ply_path.parent / out_name

    half_height_m = bbox_half_height_z(pc_centered)
    candidates = []
    for i in range(len(outcome.grasps)):
        pos, ori = matrix4_to_pose(outcome.grasps[i])
        candidates.append({
            "position": [float(round(x, 6)) for x in pos],
            "orientation": [float(round(x, 6)) for x in ori],
            "confidence": float(round(float(outcome.confidences[i]), 6)),
        })
    doc = {
        "frame_id": "object",
        "num_grasps": len(candidates),
        YAML_KEY_OBJECT_HALF_HEIGHT_M: round(half_height_m, 6),
        "grasps": candidates,
    }
    with open(out_path, "w", encoding="utf-8") as f:
        yaml.safe_dump(doc, f, default_flow_style=False, sort_keys=False)
    return out_path


def sweep_object(
    *,
    object_name: str,
    folder: Path,
    num_trials: int,
    topk_values: list[int],
    default_topk: int,
    default_npoints_slot_idx: int,
    host: str,
    port: int,
    csv_path: Path,
) -> None:
    plys = find_plys_sorted(folder)
    if len(plys) < 4:
        print(
            f"[{object_name}] SKIP — expected 4 PLYs matching '*_<int>.ply', found {len(plys)} in {folder}",
            flush=True,
        )
        return
    if len(plys) > 4:
        print(
            f"[{object_name}] WARN — found {len(plys)} PLYs, using the 4 sorted ascending: "
            f"{[p.name for _n, p in plys[:4]]}",
            flush=True,
        )
        plys = plys[:4]

    print(f"[{object_name}] PLYs (sorted by filename int):", flush=True)
    for slot, (n_actual, path) in zip(TARGET_NPOINTS, plys):
        print(f"  slot≈{slot}: {path.name} (actual vertices tag={n_actual})", flush=True)

    clouds = []
    for (n_actual, path) in plys:
        pc = load_point_cloud(path)
        clouds.append((n_actual, path, center_point_cloud(pc)))

    # --- Sweep 1: vary N_points, topk fixed at default ---
    print(
        f"[{object_name}] sweep n_points @ topk={default_topk}, {num_trials} trials each",
        flush=True,
    )
    for slot, (n_actual, path, pc_c) in zip(TARGET_NPOINTS, clouds):
        for i in range(num_trials):
            out = do_one_request(pc_c, host=host, port=port, topk=default_topk)
            _append_row(
                csv_path,
                {
                    "timestamp_iso": datetime.utcnow().isoformat() + "Z",
                    "sweep": "n_points",
                    "object": object_name,
                    "ply_path": str(path),
                    "ply_points_actual": int(pc_c.shape[0]),
                    "ply_slot_nominal": slot,
                    "topk": default_topk,
                    "trial_idx": i,
                    "elapsed_client_s": f"{out.elapsed_s:.4f}",
                    "infer_ms": f"{out.infer_ms:.3f}" if out.infer_ms is not None else "",
                    "num_candidates": out.num_candidates,
                    "remove_outliers": out.remove_outliers,
                    "ok": out.ok,
                    "error": out.error,
                },
            )
            tag = "OK " if out.ok else "ERR"
            print(
                f"  [n_points slot={slot:>5} ply={path.name} trial={i+1}/{num_trials}] "
                f"{tag} client={out.elapsed_s:.3f}s infer={out.infer_ms if out.infer_ms is not None else '-'} "
                f"cands={out.num_candidates}",
                flush=True,
            )
            if i == num_trials - 1:
                yp = save_final_trial_yaml(
                    ply_path=path,
                    pc_centered=pc_c,
                    outcome=out,
                    sweep="n_points",
                    slot_nominal=slot,
                    topk=default_topk,
                )
                if yp is not None:
                    print(f"    saved YAML (final trial): {yp}", flush=True)
                elif out.ok:
                    print("    final trial OK but YAML not written (no grasps?)", flush=True)
                else:
                    print("    final trial failed — no YAML for this combo", flush=True)

    # --- Sweep 2: vary topk, N_points fixed at default slot (~5k) ---
    _n_tag, path_def, pc_c_def = clouds[default_npoints_slot_idx]
    slot_def = TARGET_NPOINTS[default_npoints_slot_idx]
    print(
        f"[{object_name}] sweep topk @ N_points slot≈{slot_def} ({path_def.name}), "
        f"{num_trials} trials each",
        flush=True,
    )
    for topk in topk_values:
        for i in range(num_trials):
            out = do_one_request(pc_c_def, host=host, port=port, topk=topk)
            _append_row(
                csv_path,
                {
                    "timestamp_iso": datetime.utcnow().isoformat() + "Z",
                    "sweep": "topk",
                    "object": object_name,
                    "ply_path": str(path_def),
                    "ply_points_actual": int(pc_c_def.shape[0]),
                    "ply_slot_nominal": slot_def,
                    "topk": topk,
                    "trial_idx": i,
                    "elapsed_client_s": f"{out.elapsed_s:.4f}",
                    "infer_ms": f"{out.infer_ms:.3f}" if out.infer_ms is not None else "",
                    "num_candidates": out.num_candidates,
                    "remove_outliers": out.remove_outliers,
                    "ok": out.ok,
                    "error": out.error,
                },
            )
            tag = "OK " if out.ok else "ERR"
            print(
                f"  [topk={topk:>4} trial={i+1}/{num_trials}] "
                f"{tag} client={out.elapsed_s:.3f}s infer={out.infer_ms if out.infer_ms is not None else '-'} "
                f"cands={out.num_candidates}",
                flush=True,
            )
            if i == num_trials - 1:
                yp = save_final_trial_yaml(
                    ply_path=path_def,
                    pc_centered=pc_c_def,
                    outcome=out,
                    sweep="topk",
                    slot_nominal=slot_def,
                    topk=topk,
                )
                if yp is not None:
                    print(f"    saved YAML (final trial): {yp}", flush=True)
                elif out.ok:
                    print("    final trial OK but YAML not written (no grasps?)", flush=True)
                else:
                    print("    final trial failed — no YAML for this combo", flush=True)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--num-trials", type=int, required=True, help="Trials per config (required).")
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=5557)
    ap.add_argument("--data-root", type=Path, default=Path("data"))
    ap.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    ap.add_argument("--log-name", default=DEFAULT_LOG_NAME)
    ap.add_argument(
        "--topk-values",
        type=int,
        nargs="+",
        default=DEFAULT_TOPK_VALUES,
        help=f"Values to sweep for topk. Default: {DEFAULT_TOPK_VALUES}",
    )
    ap.add_argument("--default-topk", type=int, default=DEFAULT_TOPK)
    ap.add_argument(
        "--default-npoints-slot-idx",
        type=int,
        default=DEFAULT_NPOINTS_SLOT_IDX,
        help=f"Index into sorted PLYs used when varying topk (default {DEFAULT_NPOINTS_SLOT_IDX} ≈ 5k).",
    )
    ap.add_argument(
        "--objects",
        nargs="+",
        default=OBJECT_FOLDERS,
        help=f"Object folder names under data-root. Default: {OBJECT_FOLDERS}",
    )
    args = ap.parse_args()

    if args.num_trials <= 0:
        ap.error("--num-trials must be > 0")

    t_wall0 = time.perf_counter()
    csv_path = args.log_dir / args.log_name
    print(f"Logging to {csv_path}", flush=True)

    for obj in args.objects:
        folder = args.data_root / obj
        if not folder.is_dir():
            print(f"[{obj}] SKIP — folder missing: {folder}", flush=True)
            continue
        try:
            sweep_object(
                object_name=obj,
                folder=folder,
                num_trials=args.num_trials,
                topk_values=args.topk_values,
                default_topk=args.default_topk,
                default_npoints_slot_idx=args.default_npoints_slot_idx,
                host=args.host,
                port=args.port,
                csv_path=csv_path,
            )
        except Exception:  # noqa: BLE001
            print(f"[{obj}] FAILED with exception:", flush=True)
            traceback.print_exc()
            continue

    total_s = time.perf_counter() - t_wall0
    print(
        f"Total runtime (all trials): {total_s:.2f} s ({total_s / 60.0:.2f} min)",
        flush=True,
    )
    print(f"Done. Rows appended to {csv_path}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
