#!/usr/bin/env python3
"""
Interactive Open3D GUI: rotate one or many .ply files in 90° steps about world X/Y/Z, then Save.

Same rotation applies to every file (batch mode). Save overwrites all listed paths.

Requires: open3d, numpy, scipy (same stack as visualize_grasps.py).

  scripts/venv/bin/python scripts/rotate_ply_gui.py data/Mug/Mug_2011.ply
  scripts/venv/bin/python scripts/rotate_ply_gui.py data/Mug/Mug_*.ply
  scripts/venv/bin/python scripts/rotate_ply_gui.py a.ply b.ply c.ply

Red = +X, green = +Y, blue = +Z (world triad at origin + Open3D coordinate frames).
"""

from __future__ import annotations

import argparse
import colorsys
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import open3d as o3d
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering
from scipy.spatial.transform import Rotation as R3


def rot90_world_matrix(axis: int, sign: int) -> np.ndarray:
    """Active rotation: +sign * 90° about world X (0), Y (1), or Z (2)."""
    v = np.zeros(3, dtype=float)
    v[axis] = 1.0
    return R3.from_rotvec(v * (float(sign) * 0.5 * np.pi)).as_matrix()


@dataclass
class PlyEntry:
    path: Path
    points_orig: np.ndarray
    colors: np.ndarray | None
    normals: np.ndarray | None


def _load_ply(path: Path) -> PlyEntry:
    path = Path(path).resolve()
    base = o3d.io.read_point_cloud(str(path))
    pts = np.asarray(base.points, dtype=np.float64)
    if pts.size == 0:
        raise ValueError(f"Empty point cloud: {path}")
    colors = np.asarray(base.colors) if base.has_colors() else None
    normals = np.asarray(base.normals) if base.has_normals() else None
    return PlyEntry(path=path, points_orig=pts.copy(), colors=colors, normals=normals)


def _distinct_colors(n: int) -> np.ndarray:
    """RGB (n,3) in [0,1], vivid enough to tell clouds apart."""
    if n <= 0:
        return np.zeros((0, 3))
    hues = np.linspace(0.0, 1.0, n, endpoint=False)
    out = np.zeros((n, 3), dtype=np.float64)
    for i, h in enumerate(hues):
        r, g, b = colorsys.hsv_to_rgb(float(h), 0.85, 0.95)
        out[i] = (r, g, b)
    return out


class RotatePlyGui:
    AXES_NAME = "axes"

    def __init__(self, paths: list[Path]) -> None:
        self._entries: list[PlyEntry] = [_load_ply(p) for p in paths]
        self._R = np.eye(3)

        # Display offsets: single cloud = natural coords; batch = spread along +X after centering each cloud for view only.
        self._view_center: list[np.ndarray] = []
        self._view_offset: list[np.ndarray] = []
        n = len(self._entries)
        max_ext = 0.05
        for e in self._entries:
            lo = e.points_orig.min(axis=0)
            hi = e.points_orig.max(axis=0)
            max_ext = max(max_ext, float(np.linalg.norm(hi - lo)))
        margin = max(0.02, 0.15 * max_ext)
        step = max_ext + margin
        for i, e in enumerate(self._entries):
            c = np.mean(e.points_orig, axis=0)
            self._view_center.append(c.copy())
            if n == 1:
                self._view_offset.append(np.zeros(3))
            else:
                self._view_offset.append(np.array([i * step, 0.0, 0.0], dtype=np.float64))

        global_ext = max_ext + (n - 1) * step if n > 1 else max_ext
        self._axis_len = float(max(0.05, min(0.35, 0.22 * max(global_ext, 1e-6))))

        self._mat_pcd = rendering.MaterialRecord()
        self._mat_pcd.shader = "defaultUnlit"
        n_pts_total = sum(len(e.points_orig) for e in self._entries)
        self._mat_pcd.point_size = float(
            max(2.0, min(8.0, 120.0 / max(1, n_pts_total // max(1, n)) ** (1.0 / 3.0)))
        )

        self._mat_axes = rendering.MaterialRecord()
        self._mat_axes.shader = "defaultLit"

        title = self._entries[0].path.name if n == 1 else f"{n} PLYs (batch)"
        self.window = gui.Application.instance.create_window(f"Rotate PLY — {title}", 1280, 780)

        self._scene = gui.SceneWidget()
        self._scene.scene = rendering.Open3DScene(self.window.renderer)
        self._scene.set_view_controls(gui.SceneWidget.Controls.ROTATE_CAMERA)

        em = self.window.theme.font_size
        self._panel_w = int(22 * em)
        self._panel = gui.Vert(0, gui.Margins(em, em, em, em))
        self._panel.add_child(gui.Label("90° about world axes (all clouds)"))
        self._panel.add_child(gui.Label("Triad: red=X, green=Y, blue=Z"))
        if n > 1:
            self._panel.add_child(gui.Label(f"Batch: {n} files — same R, Save all"))

        for ax_idx, name in enumerate(("X", "Y", "Z")):
            row = gui.Horiz(0, gui.Margins(0, int(0.25 * em), 0, 0))
            bp = gui.Button(f"+90° {name}")
            bm = gui.Button(f"-90° {name}")
            bp.set_on_clicked(lambda a=ax_idx: self._on_rotate(a, 1))
            bm.set_on_clicked(lambda a=ax_idx: self._on_rotate(a, -1))
            row.add_child(bp)
            row.add_child(bm)
            self._panel.add_child(row)

        self._panel.add_fixed(int(em))
        br = gui.Button("Reset rotation")
        br.set_on_clicked(self._on_reset)
        self._panel.add_child(br)
        bs = gui.Button("Save all (overwrite)" if n > 1 else "Save (overwrite)")
        bs.set_on_clicked(self._on_save)
        self._panel.add_child(bs)
        for e in self._entries[:6]:
            self._panel.add_child(gui.Label(e.path.name))
        if n > 6:
            self._panel.add_child(gui.Label(f"... +{n - 6} more"))

        self.window.add_child(self._scene)
        self.window.add_child(self._panel)
        self.window.set_on_layout(self._on_layout)

        self._rebuild_scene(reset_camera=True)

    def _rotated_points_disk(self, idx: int) -> np.ndarray:
        """What gets written to disk (no view centering/offset)."""
        e = self._entries[idx]
        return e.points_orig @ self._R.T

    def _rotated_points_view(self, idx: int) -> np.ndarray:
        """What is drawn (batch: centered per cloud + spread)."""
        disk = self._rotated_points_disk(idx)
        if len(self._entries) == 1:
            return disk
        c0 = self._view_center[idx]
        return disk - (c0 @ self._R.T) + self._view_offset[idx]

    def _make_pcd_geometry(self, idx: int) -> o3d.geometry.PointCloud:
        e = self._entries[idx]
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(self._rotated_points_view(idx))
        if e.colors is not None and len(e.colors) == len(e.points_orig):
            pcd.colors = o3d.utility.Vector3dVector(e.colors)
        elif len(self._entries) > 1:
            pal = _distinct_colors(len(self._entries))
            rgb = np.tile(pal[idx], (len(e.points_orig), 1))
            pcd.colors = o3d.utility.Vector3dVector(rgb)
        if e.normals is not None and len(e.normals) == len(e.points_orig):
            pcd.normals = o3d.utility.Vector3dVector(e.normals @ self._R.T)
        return pcd

    def _rebuild_scene(self, *, reset_camera: bool) -> None:
        sc = self._scene.scene
        sc.clear_geometry()

        for i in range(len(self._entries)):
            sc.add_geometry(f"cloud_{i}", self._make_pcd_geometry(i), self._mat_pcd)

        axes = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=self._axis_len, origin=[0, 0, 0]
        )
        sc.add_geometry(self.AXES_NAME, axes, self._mat_axes)

        bbox = sc.bounding_box
        center = np.asarray(bbox.get_center(), dtype=np.float32).reshape(3, 1)
        if reset_camera:
            self._scene.setup_camera(60.0, bbox, center)
        self._scene.force_redraw()

    def _on_layout(self, layout_context: gui.LayoutContext) -> None:
        r = self.window.content_rect
        self._scene.frame = gui.Rect(r.x, r.y, r.width - self._panel_w, r.height)
        self._panel.frame = gui.Rect(self._scene.frame.get_right(), r.y, self._panel_w, r.height)

    def _on_rotate(self, axis: int, sign: int) -> None:
        self._R = rot90_world_matrix(axis, sign) @ self._R
        self._rebuild_scene(reset_camera=False)

    def _on_reset(self) -> None:
        self._R = np.eye(3)
        self._rebuild_scene(reset_camera=False)

    def _on_save(self) -> None:
        errors: list[str] = []
        for i, e in enumerate(self._entries):
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(self._rotated_points_disk(i))
            if e.colors is not None and len(e.colors) == len(e.points_orig):
                pcd.colors = o3d.utility.Vector3dVector(e.colors)
            if e.normals is not None and len(e.normals) == len(e.points_orig):
                pcd.normals = o3d.utility.Vector3dVector(e.normals @ self._R.T)
            ok = o3d.io.write_point_cloud(str(e.path), pcd, write_ascii=False)
            if not ok:
                errors.append(str(e.path))
        if errors:
            self.window.show_message_box("Error", "write_point_cloud failed for:\n" + "\n".join(errors))
            return
        n = len(self._entries)
        msg = f"Overwrote {n} file(s)." if n > 1 else f"Overwrote:\n{self._entries[0].path}"
        self.window.show_message_box("Saved", msg)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="GUI: apply 90° world rotations to one or more PLYs; Save overwrites each file."
    )
    parser.add_argument(
        "ply_paths",
        type=Path,
        nargs="+",
        help="One or more .ply paths (shell glob OK: data/Mug/*.ply)",
    )
    args = parser.parse_args()

    paths: list[Path] = []
    for p in args.ply_paths:
        p = Path(p)
        if p.suffix.lower() != ".ply":
            print(f"Skip (not .ply): {p}", file=sys.stderr)
            continue
        if not p.is_file():
            print(f"Skip (not found): {p}", file=sys.stderr)
            continue
        paths.append(p.resolve())

    if not paths:
        print("No valid .ply files.", file=sys.stderr)
        return 1

    paths.sort(key=lambda x: str(x))

    try:
        gui.Application.instance.initialize()
        _ = RotatePlyGui(paths)
    except ValueError as e:
        print(e, file=sys.stderr)
        return 1

    gui.Application.instance.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
