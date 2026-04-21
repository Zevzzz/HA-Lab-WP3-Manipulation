#!/usr/bin/env python3
"""
Bake a mesh USD asset's **surface centroid** to its local origin, so the prim origin
coincides with the point GraspGen YAML grasps (mean-centered PLY) are defined around.

For each USD asset:
  c = surface-area-weighted centroid of the mesh (trimesh.Trimesh.centroid)
  mesh.points  -= c          # bake origin shift into geometry
  (optional) scene-prim ``xformOp:translate`` += world vector (c in world),
             so the object stays visually in place.

Idempotent: if ``||c|| < --eps`` the asset is left untouched.

Usage
-----
  scripts/venv/bin/python scripts/bake_centroid_to_usd_origin.py \\
      data/Laptop/Lowpoly_Notebook_2.usd

  # Also update a scene prim's translate to keep the object visually fixed:
  scripts/venv/bin/python scripts/bake_centroid_to_usd_origin.py \\
      data/Laptop/Lowpoly_Notebook_2.usd \\
      --scene isaac/scenes/HA_Grasping_Sim.usd \\
      --scene-prim /World/Targets/Lowpoly_Notebook_2

  # Verify only (no writes):
  scripts/venv/bin/python scripts/bake_centroid_to_usd_origin.py <usd> --dry-run

Notes
-----
- Backups are created next to each edited file as ``<name>.bak`` unless already present.
- Pass ``--mesh-prim <path>`` to restrict the edit to a single Mesh when the asset has
  several; by default all Mesh prims are offset by the same c computed across all of them.
"""

from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path
from typing import Iterable, Optional

import numpy as np
import trimesh
from pxr import Gf, Usd, UsdGeom, Vt


def _fan_triangulate(
    face_vertex_counts: np.ndarray,
    face_vertex_indices: np.ndarray,
) -> np.ndarray:
    tris: list[list[int]] = []
    k = 0
    for c in face_vertex_counts:
        ring = face_vertex_indices[k : k + c]
        for i in range(1, c - 1):
            tris.append([int(ring[0]), int(ring[i]), int(ring[i + 1])])
        k += int(c)
    return np.asarray(tris, dtype=np.int64) if tris else np.zeros((0, 3), dtype=np.int64)


def _iter_mesh_prims(stage: Usd.Stage, filter_path: Optional[str] = None) -> Iterable[Usd.Prim]:
    for p in stage.Traverse():
        if not p.IsA(UsdGeom.Mesh):
            continue
        if filter_path and p.GetPath().pathString != filter_path:
            continue
        yield p


def _load_mesh(prim: Usd.Prim) -> tuple[np.ndarray, np.ndarray]:
    m = UsdGeom.Mesh(prim)
    V = np.asarray(m.GetPointsAttr().Get(), dtype=np.float64)
    fvc = np.asarray(m.GetFaceVertexCountsAttr().Get(), dtype=np.int64)
    fvi = np.asarray(m.GetFaceVertexIndicesAttr().Get(), dtype=np.int64)
    F = _fan_triangulate(fvc, fvi)
    return V, F


def compute_surface_centroid(
    stage: Usd.Stage, mesh_prim_filter: Optional[str] = None
) -> tuple[np.ndarray, list[Usd.Prim]]:
    """Area-weighted centroid across every Mesh (optionally one) in the asset."""
    prims: list[Usd.Prim] = list(_iter_mesh_prims(stage, mesh_prim_filter))
    if not prims:
        raise RuntimeError("No Mesh prims found in USD.")
    V_all: list[np.ndarray] = []
    F_all: list[np.ndarray] = []
    off = 0
    for p in prims:
        V, F = _load_mesh(p)
        V_all.append(V)
        F_all.append(F + off)
        off += len(V)
    V_cat = np.vstack(V_all)
    F_cat = np.vstack(F_all) if F_all else np.zeros((0, 3), dtype=np.int64)
    tm = trimesh.Trimesh(vertices=V_cat, faces=F_cat, process=False)
    return np.asarray(tm.centroid, dtype=np.float64), prims


def apply_mesh_offset(prims: list[Usd.Prim], shift_xyz: np.ndarray) -> None:
    for p in prims:
        m = UsdGeom.Mesh(p)
        V = np.asarray(m.GetPointsAttr().Get(), dtype=np.float64)
        V2 = (V + shift_xyz).astype(np.float32)
        m.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(V2))


def _backup_once(path: Path) -> Optional[Path]:
    bak = path.with_suffix(path.suffix + ".bak")
    if bak.exists():
        return bak
    shutil.copy2(path, bak)
    return bak


def update_scene_translate(
    scene_path: Path,
    scene_prim_path: str,
    centroid_usd_local: np.ndarray,
    dry_run: bool,
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """
    Add ``T_local_to_world @ centroid_usd_local`` (vector) to ``xformOp:translate`` on the
    scene prim so the object stays visually in place after baking ``-c`` into the mesh.

    Returns (before, after) translate values.
    """
    stage = Usd.Stage.Open(str(scene_path))
    prim = stage.GetPrimAtPath(scene_prim_path)
    if not prim.IsValid():
        raise RuntimeError(f"Scene prim not found: {scene_prim_path}")
    xf = UsdGeom.Xformable(prim)
    T = xf.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    c = Gf.Vec3d(float(centroid_usd_local[0]), float(centroid_usd_local[1]), float(centroid_usd_local[2]))
    world_c = T.Transform(c)

    tr_ops = [o for o in xf.GetOrderedXformOps() if o.GetOpName() == "xformOp:translate"]
    if not tr_ops:
        raise RuntimeError(f"{scene_prim_path}: no xformOp:translate to update")
    tr = tr_ops[0]
    before = tuple(float(x) for x in tr.Get())
    after = (float(world_c[0]), float(world_c[1]), float(world_c[2]))

    if not dry_run:
        _backup_once(scene_path)
        tr.Set(Gf.Vec3d(*after))
        stage.GetRootLayer().Save()
    return before, after


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("usd", type=Path, help="USD asset file to edit (mesh origin → surface centroid).")
    parser.add_argument(
        "--mesh-prim",
        type=str,
        default=None,
        help="Restrict edit to a single Mesh prim path inside the USD (default: all).",
    )
    parser.add_argument(
        "--scene",
        type=Path,
        default=None,
        help="Optional scene USD whose prim translate should be updated so the object stays visually in place.",
    )
    parser.add_argument(
        "--scene-prim",
        type=str,
        default=None,
        help="Scene prim path that payload-references this asset. Required with --scene.",
    )
    parser.add_argument(
        "--eps",
        type=float,
        default=1e-6,
        help="Skip edit if ||centroid|| is below this (m in USD units). Default 1e-6.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Compute and print changes; write nothing.",
    )
    args = parser.parse_args()

    if args.scene is not None and not args.scene_prim:
        parser.error("--scene requires --scene-prim")

    usd_path: Path = args.usd.resolve()
    if not usd_path.is_file():
        print(f"USD not found: {usd_path}", file=sys.stderr)
        return 1

    stage = Usd.Stage.Open(str(usd_path))
    c, prims = compute_surface_centroid(stage, mesh_prim_filter=args.mesh_prim)
    print(f"Asset: {usd_path}")
    print(f"  meshes affected: {[p.GetPath().pathString for p in prims]}")
    print(f"  surface centroid in USD local = ({c[0]:.6f}, {c[1]:.6f}, {c[2]:.6f})")

    if float(np.linalg.norm(c)) < args.eps:
        print(f"  ||c|| < eps ({args.eps}); already centered. Nothing to do.")
        return 0

    if args.dry_run:
        print("  [dry-run] would translate mesh points by -c (bake origin to surface centroid).")
    else:
        _backup_once(usd_path)
        apply_mesh_offset(prims, shift_xyz=-c)
        stage.GetRootLayer().Save()
        stage2 = Usd.Stage.Open(str(usd_path))
        c2, _ = compute_surface_centroid(stage2, mesh_prim_filter=args.mesh_prim)
        print(f"  after bake: surface centroid = ({c2[0]:.3e}, {c2[1]:.3e}, {c2[2]:.3e})")
        if float(np.linalg.norm(c2)) > max(args.eps, 1e-4):
            print("  WARN: residual centroid > eps; check mesh data or --mesh-prim.", file=sys.stderr)

    if args.scene is not None:
        scene_path: Path = args.scene.resolve()
        if not scene_path.is_file():
            print(f"Scene not found: {scene_path}", file=sys.stderr)
            return 1
        before, after = update_scene_translate(scene_path, args.scene_prim, c, args.dry_run)
        tag = "[dry-run] would set" if args.dry_run else "set"
        print(f"Scene: {scene_path}")
        print(f"  prim {args.scene_prim}  xformOp:translate  before={before}  {tag}  after={after}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
