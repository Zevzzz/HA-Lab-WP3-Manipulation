#!/usr/bin/env python3
"""
Interactive Open3D GUI: rotate a .ply in 90° steps about world X/Y/Z, then Save to overwrite.

Requires: open3d, numpy, scipy (same stack as visualize_grasps.py).

  scripts/venv/bin/python scripts/rotate_ply_gui.py data/Laptop/Laptop_7997.ply

Red = +X, green = +Y, blue = +Z (world triad at origin + Open3D coordinate frames).
"""

from __future__ import annotations

import argparse
import sys
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


class RotatePlyGui:
    PCD_NAME = "cloud"
    AXES_NAME = "axes"

    def __init__(self, pcd_path: Path) -> None:
        self.path = Path(pcd_path).resolve()
        self._base = o3d.io.read_point_cloud(str(self.path))
        pts = np.asarray(self._base.points, dtype=np.float64)
        if pts.size == 0:
            raise ValueError(f"Empty point cloud: {self.path}")

        self._points_orig = pts.copy()
        self._colors = np.asarray(self._base.colors) if self._base.has_colors() else None
        self._normals = np.asarray(self._base.normals) if self._base.has_normals() else None

        self._R = np.eye(3)

        ext = float(np.linalg.norm(pts.max(axis=0) - pts.min(axis=0)))
        self._axis_len = float(max(0.05, min(0.35, 0.22 * max(ext, 1e-6))))

        self._mat_pcd = rendering.MaterialRecord()
        self._mat_pcd.shader = "defaultUnlit"
        n = len(pts)
        self._mat_pcd.point_size = float(max(2.0, min(8.0, 120.0 / max(1, n) ** (1.0 / 3.0))))

        self._mat_axes = rendering.MaterialRecord()
        self._mat_axes.shader = "defaultLit"

        self.window = gui.Application.instance.create_window(
            f"Rotate PLY — {self.path.name}", 1280, 780
        )

        self._scene = gui.SceneWidget()
        self._scene.scene = rendering.Open3DScene(self.window.renderer)
        self._scene.set_view_controls(gui.SceneWidget.Controls.ROTATE_CAMERA)

        em = self.window.theme.font_size
        self._panel_w = int(22 * em)
        self._panel = gui.Vert(0, gui.Margins(em, em, em, em))
        self._panel.add_child(gui.Label("90° about world axes (cloud)"))
        self._panel.add_child(gui.Label("Triad: red=X, green=Y, blue=Z"))

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
        bs = gui.Button("Save (overwrite)")
        bs.set_on_clicked(self._on_save)
        self._panel.add_child(bs)
        self._panel.add_child(gui.Label(str(self.path)))

        self.window.add_child(self._scene)
        self.window.add_child(self._panel)
        self.window.set_on_layout(self._on_layout)

        self._rebuild_scene(reset_camera=True)

    def _current_points(self) -> np.ndarray:
        return self._points_orig @ self._R.T

    def _make_pcd_geometry(self) -> o3d.geometry.PointCloud:
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(self._current_points())
        if self._colors is not None and len(self._colors) == len(self._points_orig):
            pcd.colors = o3d.utility.Vector3dVector(self._colors)
        if self._normals is not None and len(self._normals) == len(self._points_orig):
            pcd.normals = o3d.utility.Vector3dVector(self._normals @ self._R.T)
        return pcd

    def _rebuild_scene(self, *, reset_camera: bool) -> None:
        sc = self._scene.scene
        sc.clear_geometry()

        pcd = self._make_pcd_geometry()
        sc.add_geometry(self.PCD_NAME, pcd, self._mat_pcd)

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
        pcd = self._make_pcd_geometry()
        ok = o3d.io.write_point_cloud(str(self.path), pcd, write_ascii=False)
        if not ok:
            self.window.show_message_box("Error", "write_point_cloud returned False.")
            return
        self.window.show_message_box("Saved", f"Overwrote:\n{self.path}")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="GUI: apply 90° world rotations to a PLY; Save overwrites the file."
    )
    parser.add_argument("ply_path", type=Path, help="Path to .ply (overwritten on Save)")
    args = parser.parse_args()
    if args.ply_path.suffix.lower() != ".ply":
        print("Expected a .ply file.", file=sys.stderr)
        return 1

    try:
        gui.Application.instance.initialize()
        _ = RotatePlyGui(args.ply_path)
    except ValueError as e:
        print(e, file=sys.stderr)
        return 1

    gui.Application.instance.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
