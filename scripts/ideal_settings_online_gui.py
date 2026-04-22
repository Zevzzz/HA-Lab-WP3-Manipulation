#!/usr/bin/env python3
"""
Semi-automated **online** (sim + MoveIt) trials for the ideal-settings study.

Scans directory trees for eval ``*_grasps.yaml`` files (from ``service_speed_test.py``):

  * ``<stem>_eval_npts_slot<SLOT>_topk<TK>_grasps.yaml``
  * ``<stem>_eval_topk<TK>_npts_slot<SLOT>_grasps.yaml``

GUI flow:

  1. Pick **object** folder (e.g. Mug, RectPrism, Sphere, Laptop), then **N_points** (slot),
     then **TopK**, then (if multiple) a specific YAML.
  2. Press **Run grasp** as many times as you want. Each press runs
     ``grasp_with_candidates`` once and appends its exit code to the per-block history.
  3. Press **Log block → CSV**. A Tk dialog asks for **total trials** and **successes**
     (for deterministic sweeps, typically 1/1 or 0/1). One CSV row is written to
     ``data/logs/ideal_settings_online.csv`` including ``object_folder``, ``slot_nominal``,
     ``topk``, ``successes_reported``, ``trials_in_block``, and the list of
     ``trial_exit_codes`` seen since the previous log.

``Reset run counter`` clears the unsaved exit-code history without logging. Switching
object/slot/topk/YAML also clears it (nothing is carried across combos).

**Run from repo root** with venv::

    source scripts/venv/bin/activate
    python scripts/ideal_settings_online_gui.py data/Mug data/RectPrism data/Sphere data/Laptop

**Docker** (YAML must live under ``--docker-data-root``, default ``./data``)::

    python scripts/ideal_settings_online_gui.py \\
        data/Mug data/RectPrism data/Sphere data/Laptop --docker-exec
"""

from __future__ import annotations

import argparse
import csv
import re
import shlex
import socket
import subprocess
import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Optional

import numpy as np
import open3d as o3d
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering

RE_EVAL_NPTS = re.compile(
    r"^(.+)_eval_npts_slot(\d+)_topk(\d+)_grasps\.yaml$",
    re.IGNORECASE,
)
RE_EVAL_TOPK = re.compile(
    r"^(.+)_eval_topk(\d+)_npts_slot(\d+)_grasps\.yaml$",
    re.IGNORECASE,
)

DEFAULT_LOG_DIR = Path("data/logs")
LOG_NAME = "ideal_settings_online.csv"


def _combo_fill(combo: gui.Combobox, items: list[str], *, selected_index: int = 0) -> None:
    """Open3D Combobox: clear + add_item per entry (no set_items in older bindings)."""
    combo.clear_items()
    for s in items:
        combo.add_item(s)
    if not items:
        return
    i = max(0, min(int(selected_index), len(items) - 1))
    combo.selected_index = i

CSV_FIELDS = [
    "timestamp_iso",
    "scan_dirs",
    "object_folder",
    "yaml_host_path",
    "yaml_container_path",
    "slot_nominal",
    "topk",
    "filename_pattern",
    "successes_reported",
    "trials_in_block",
    "trial_exit_codes",
    "extra_args",
    "docker_mode",
    "hostname",
]


@dataclass(frozen=True)
class GraspYamlEntry:
    path: Path
    slot_nominal: int
    topk: int
    pattern: str  # eval_npts | eval_topk


def parse_grasp_yaml(path: Path) -> Optional[GraspYamlEntry]:
    name = path.name
    m = RE_EVAL_NPTS.match(name)
    if m:
        return GraspYamlEntry(
            path=path.resolve(),
            slot_nominal=int(m.group(2)),
            topk=int(m.group(3)),
            pattern="eval_npts",
        )
    m = RE_EVAL_TOPK.match(name)
    if m:
        return GraspYamlEntry(
            path=path.resolve(),
            slot_nominal=int(m.group(3)),
            topk=int(m.group(2)),
            pattern="eval_topk",
        )
    return None


def scan_catalog(scan_dirs: list[Path]) -> list[GraspYamlEntry]:
    seen: set[Path] = set()
    out: list[GraspYamlEntry] = []
    for root in scan_dirs:
        root = root.resolve()
        if not root.is_dir():
            continue
        for p in root.rglob("*_grasps.yaml"):
            if not p.is_file():
                continue
            pr = p.resolve()
            if pr in seen:
                continue
            ent = parse_grasp_yaml(p)
            if ent is None:
                continue
            seen.add(pr)
            out.append(ent)
    out.sort(key=lambda e: (e.path.parent.name, e.slot_nominal, e.topk, e.path.name))
    return out


def pick_yaml_for_combo(catalog: list[GraspYamlEntry], slot: int, topk: int) -> list[GraspYamlEntry]:
    return [e for e in catalog if e.slot_nominal == slot and e.topk == topk]


def host_path_to_container(grasp_path: Path, docker_data_root: Path) -> str:
    grasp_path = grasp_path.resolve()
    root = docker_data_root.resolve()
    rel = grasp_path.relative_to(root)
    return f"/home/ros/data/{rel.as_posix()}"


def append_log_row(log_dir: Path, row: dict) -> None:
    log_dir = Path(log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)
    fp = log_dir / LOG_NAME
    new_file = not fp.exists()
    with open(fp, "a", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=CSV_FIELDS)
        if new_file:
            w.writeheader()
        w.writerow(row)


def run_grasp_subprocess(
    *,
    grasp_path_host: Path,
    extra_args: list[str],
    repo_root: Path,
    docker_exec: bool,
    docker_data_root: Path,
    container_name: str,
    ws_setup: str,
) -> int:
    """Return process exit code (grasp_with_candidates: 0 success, 1 typical fail)."""
    if docker_exec:
        cpath = host_path_to_container(grasp_path_host, docker_data_root)
        inner = ["ros2", "run", "moveitpy_execute_node", "grasp_with_candidates", "--path", cpath]
        inner.extend(extra_args)
        inner_s = " ".join(shlex.quote(x) for x in inner)
        cmd = f"{ws_setup} && {inner_s}"
        full = ["docker", "exec", container_name, "bash", "-lc", cmd]
        r = subprocess.run(full, cwd=str(repo_root))
        return int(r.returncode)
    setup_bash = repo_root / "ws" / "install" / "setup.bash"
    if not setup_bash.is_file():
        raise FileNotFoundError(
            f"Missing {setup_bash}; build the workspace or pass --repo-root to the repo that contains ws/install."
        )
    inner = [
        "ros2",
        "run",
        "moveitpy_execute_node",
        "grasp_with_candidates",
        "--path",
        str(grasp_path_host.resolve()),
    ]
    inner.extend(extra_args)
    inner_s = " ".join(shlex.quote(x) for x in inner)
    cmd = f"source {shlex.quote(str(setup_bash))} && {inner_s}"
    r = subprocess.run(["bash", "-lc", cmd], cwd=str(repo_root))
    return int(r.returncode)


def prompt_block_tk(
    default_trials: int = 1,
    max_trials: int = 1000,
) -> Optional[tuple[int, int]]:
    """Modal dialog → (successes, trials) or None if cancelled.

    Asks for **total trials** first, then **successes** clamped to [0, trials].
    """
    import tkinter as tk
    from tkinter import simpledialog

    root = tk.Tk()
    root.withdraw()
    root.attributes("-topmost", True)
    try:
        trials = simpledialog.askinteger(
            "Ideal settings — online",
            "How many trials did you run for this block?",
            minvalue=1,
            maxvalue=max_trials,
            initialvalue=default_trials,
            parent=root,
        )
        if trials is None:
            return None
        succ = simpledialog.askinteger(
            "Ideal settings — online",
            f"How many of the {trials} trials succeeded?",
            minvalue=0,
            maxvalue=trials,
            initialvalue=0,
            parent=root,
        )
        if succ is None:
            return None
    finally:
        root.destroy()
    return int(succ), int(trials)


class IdealSettingsOnlineApp:
    # No fixed block size: press "Run grasp" as many times as you want, then "Log block"
    # and enter (successes, trials). Intended for deterministic pass/fail sweeps.
    DEFAULT_TRIALS_GUESS = 1

    def __init__(
        self,
        scan_dirs: list[Path],
        scan_dirs_joined: str,
        repo_root: Path,
        log_dir: Path,
        docker_exec: bool,
        docker_data_root: Path,
        container_name: str,
        ws_setup: str,
    ) -> None:
        self._scan_dirs_joined = scan_dirs_joined
        self._repo_root = repo_root.resolve()
        self._log_dir = Path(log_dir)
        self._docker = docker_exec
        self._docker_data_root = docker_data_root.resolve()
        self._container = container_name
        self._ws_setup = ws_setup

        self._catalog = scan_catalog(scan_dirs)
        self._object_names = sorted({e.path.parent.name for e in self._catalog})
        self._slots: list[int] = []
        self._topks: list[int] = []

        self._session_key: Optional[tuple[str, int, int, str]] = None
        self._exit_codes: list[int] = []
        # Open3D fires selection_changed when refilling slot before topk is updated — skip refresh mid-rebuild.
        self._rebuilding_combos: bool = False

        self.window = gui.Application.instance.create_window(
            "Ideal settings — online trials", 1100, 720
        )

        em = self.window.theme.font_size
        self._panel_w = int(34 * em)
        self._panel = gui.Vert(0, gui.Margins(em, em, em, em))

        self._panel.add_child(gui.Label("Ideal settings (eval YAMLs)"))
        self._panel.add_child(
            gui.Label(f"Scanned: {len(self._catalog)} YAML(s) under {scan_dirs_joined}")
        )
        if not self._catalog:
            self._panel.add_child(
                gui.Label("No eval-pattern YAMLs. Expected *_eval_npts_slot*_topk*_grasps.yaml etc.")
            )

        self._panel.add_fixed(int(0.5 * em))
        self._panel.add_child(gui.Label("Object (data subfolder)"))
        self._combo_object = gui.Combobox()
        _combo_fill(
            self._combo_object,
            self._object_names if self._object_names else ["(none)"],
        )
        self._panel.add_child(self._combo_object)

        self._panel.add_fixed(int(0.25 * em))
        self._panel.add_child(gui.Label("N_points (slot)"))
        self._combo_slot = gui.Combobox()
        self._panel.add_child(self._combo_slot)

        self._panel.add_fixed(int(0.25 * em))
        self._panel.add_child(gui.Label("TopK"))
        self._combo_topk = gui.Combobox()
        self._panel.add_child(self._combo_topk)

        self._panel.add_fixed(int(0.25 * em))
        self._panel.add_child(gui.Label("If multiple YAMLs match, pick file:"))
        self._combo_file = gui.Combobox()
        _combo_fill(self._combo_file, ["—"])
        self._panel.add_child(self._combo_file)

        # Must exist before _rebuild_slot_topk_for_object() → _on_refresh() touches them.
        self._panel.add_fixed(int(0.5 * em))
        self._status = gui.Label("Select object, then slot + topk.")
        self._panel.add_child(self._status)
        self._trial_lbl = gui.Label("Runs since last log: 0")
        self._panel.add_child(self._trial_lbl)

        self._rebuild_slots()

        self._combo_object.set_on_selection_changed(lambda _s, _i: self._on_object_changed())
        self._combo_slot.set_on_selection_changed(lambda _s, _i: self._on_slot_changed())
        self._combo_topk.set_on_selection_changed(lambda _s, _i: self._on_topk_changed())
        self._combo_file.set_on_selection_changed(lambda _s, _i: self._on_file_combo())

        self._panel.add_fixed(int(0.25 * em))
        self._panel.add_child(gui.Label("Extra grasp_with_candidates args (optional):"))
        self._extra = gui.TextEdit()
        self._extra.placeholder_text = '--object-center 0.5 0.0 0.12 --object-yaw-deg 0'
        self._panel.add_child(self._extra)

        self._panel.add_fixed(int(0.5 * em))
        br = gui.Button("Refresh match")
        br.set_on_clicked(self._on_refresh)
        self._panel.add_child(br)

        self._run_btn = gui.Button("Run grasp (MoveIt / sim)")
        self._run_btn.set_on_clicked(self._on_run)
        self._panel.add_child(self._run_btn)

        blog = gui.Button("Log block → CSV (ask successes / trials)")
        blog.set_on_clicked(self._on_log_block)
        self._panel.add_child(blog)

        breset = gui.Button("Reset run counter")
        breset.set_on_clicked(self._on_reset_counter)
        self._panel.add_child(breset)

        self._panel.add_fixed(int(em))
        self._panel.add_child(gui.Label(f"Log: {self._log_dir / LOG_NAME}"))
        self._panel.add_child(
            gui.Label(f"ros2: {'docker exec ' + self._container if self._docker else 'local ws/install'}")
        )

        self._scene = gui.SceneWidget()
        self._scene.scene = rendering.Open3DScene(self.window.renderer)
        self._scene.set_view_controls(gui.SceneWidget.Controls.ROTATE_CAMERA)
        axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.15)
        mat = rendering.MaterialRecord()
        mat.shader = "defaultLit"
        self._scene.scene.add_geometry("axes", axes, mat)
        bbox = self._scene.scene.bounding_box
        c = np.asarray(bbox.get_center(), dtype=np.float32).reshape(3, 1)
        self._scene.setup_camera(60.0, bbox, c)

        self.window.add_child(self._scene)
        self.window.add_child(self._panel)
        self.window.set_on_layout(self._on_layout)

        self._on_refresh()

    def _on_layout(self, ctx: gui.LayoutContext) -> None:
        r = self.window.content_rect
        self._scene.frame = gui.Rect(r.x, r.y, r.width - self._panel_w, r.height)
        self._panel.frame = gui.Rect(self._scene.frame.get_right(), r.y, self._panel_w, r.height)

    def _current_object_name(self) -> Optional[str]:
        if not self._object_names:
            return None
        oi = self._combo_object.selected_index
        if oi < 0 or oi >= len(self._object_names):
            return None
        return self._object_names[oi]

    def _filtered_catalog(self) -> list[GraspYamlEntry]:
        on = self._current_object_name()
        if not on:
            return []
        return [e for e in self._catalog if e.path.parent.name == on]

    # ---- Cascading dropdowns (every selectable combination maps to ≥1 YAML) ----

    def _slots_for_current_object(self) -> list[int]:
        return sorted({e.slot_nominal for e in self._filtered_catalog()})

    def _topks_for_object_slot(self, slot: int) -> list[int]:
        return sorted(
            {
                e.topk
                for e in self._filtered_catalog()
                if e.slot_nominal == slot
            }
        )

    def _files_for_object_slot_topk(self, slot: int, topk: int) -> list[GraspYamlEntry]:
        return pick_yaml_for_combo(self._filtered_catalog(), slot, topk)

    def _current_slot(self) -> Optional[int]:
        if not self._slots:
            return None
        st = (self._combo_slot.selected_text or "").strip()
        if st in ("(none)", "—", ""):
            return None
        try:
            v = int(st)
        except ValueError:
            return None
        return v if v in self._slots else None

    def _current_topk(self) -> Optional[int]:
        if not self._topks:
            return None
        st = (self._combo_topk.selected_text or "").strip()
        if st in ("(none)", "—", ""):
            return None
        try:
            v = int(st)
        except ValueError:
            return None
        return v if v in self._topks else None

    def _rebuild_slots(self) -> None:
        self._rebuilding_combos = True
        try:
            self._slots = self._slots_for_current_object()
            _combo_fill(
                self._combo_slot,
                [str(s) for s in self._slots] if self._slots else ["(none)"],
            )
        finally:
            self._rebuilding_combos = False
        self._rebuild_topks()

    def _rebuild_topks(self) -> None:
        self._rebuilding_combos = True
        try:
            slot = self._current_slot()
            self._topks = self._topks_for_object_slot(slot) if slot is not None else []
            _combo_fill(
                self._combo_topk,
                [str(t) for t in self._topks] if self._topks else ["(none)"],
            )
        finally:
            self._rebuilding_combos = False
        self._rebuild_files()

    def _rebuild_files(self) -> None:
        if self._rebuilding_combos:
            return
        obj = self._current_object_name()
        slot = self._current_slot()
        topk = self._current_topk()
        if obj is None or slot is None or topk is None:
            _combo_fill(self._combo_file, ["—"])
            self._status.text = "No valid selection."
            return
        matches = self._files_for_object_slot_topk(slot, topk)
        if not matches:
            _combo_fill(self._combo_file, ["—"])
            self._status.text = f"[{obj}] slot={slot} topk={topk}: no YAML (catalog inconsistency)."
            return
        labels = [f"{m.path.parent.name}/{m.path.name}" for m in matches]
        _combo_fill(self._combo_file, labels)
        self._status.text = f"[{obj}] slot={slot} topk={topk}: {len(matches)} match(es)."

    def _on_object_changed(self) -> None:
        self._session_key = None
        self._exit_codes = []
        self._trial_lbl.text = "Runs since last log: 0"
        self._rebuild_slots()

    def _on_slot_changed(self) -> None:
        self._rebuild_topks()

    def _on_topk_changed(self) -> None:
        self._rebuild_files()

    def _on_refresh(self) -> None:
        # Explicit refresh button: re-derive everything from catalog for the current object.
        self._rebuild_slots()

    def _on_file_combo(self) -> None:
        """Update status line with resolved path (optional feedback)."""
        ypath, slot, topk, matches = self._current_selection()
        if ypath is not None and slot is not None:
            self._status.text = f"slot={slot} topk={topk} → {ypath.name}"

    def _current_selection(
        self,
    ) -> tuple[Optional[Path], Optional[int], Optional[int], list[GraspYamlEntry]]:
        if not self._current_object_name():
            return None, None, None, []
        slot = self._current_slot()
        topk = self._current_topk()
        if slot is None or topk is None:
            return None, slot, topk, []
        matches = self._files_for_object_slot_topk(slot, topk)
        if not matches:
            return None, slot, topk, []
        idx = self._combo_file.selected_index
        if idx < 0 or idx >= len(matches):
            return None, slot, topk, matches
        return matches[idx].path, slot, topk, matches

    def _parse_extra_args(self) -> list[str]:
        raw = (self._extra.text_value or "").strip()
        if not raw:
            return []
        return shlex.split(raw)

    def _on_run(self) -> None:
        ypath, slot, topk, matches = self._current_selection()
        if slot is None or ypath is None or not ypath.is_file():
            self.window.show_message_box("Run", "Select a valid slot/topk with a matched YAML (Refresh if needed).")
            return

        if self._docker:
            try:
                host_path_to_container(ypath, self._docker_data_root)
            except ValueError:
                self.window.show_message_box(
                    "Docker path",
                    f"YAML must be under --docker-data-root:\n{self._docker_data_root}\nGot:\n{ypath}",
                )
                return

        key = (self._current_object_name() or "", slot, topk, str(ypath.resolve()))
        if key != self._session_key:
            self._session_key = key
            # Running under a new (obj, slot, topk, file) resets the per-block exit-code history,
            # but does NOT auto-log anything — press "Log block" when ready.
            self._exit_codes = []

        try:
            extras = self._parse_extra_args()
        except ValueError as e:
            self.window.show_message_box("Args", f"Could not parse extra args: {e}")
            return

        self._run_btn.enabled = False
        try:
            rc = run_grasp_subprocess(
                grasp_path_host=ypath,
                extra_args=extras,
                repo_root=self._repo_root,
                docker_exec=self._docker,
                docker_data_root=self._docker_data_root,
                container_name=self._container,
                ws_setup=self._ws_setup,
            )
        except Exception as e:  # noqa: BLE001
            self.window.show_message_box("Run failed", str(e))
            self._run_btn.enabled = True
            return

        self._exit_codes.append(rc)
        self._trial_lbl.text = f"Runs since last log: {len(self._exit_codes)} (last exit={rc})"
        self._run_btn.enabled = True

    def _on_reset_counter(self) -> None:
        self._session_key = None
        self._exit_codes = []
        self._trial_lbl.text = "Runs since last log: 0"

    def _on_log_block(self) -> None:
        ypath, slot, topk, matches = self._current_selection()
        if slot is None or topk is None or ypath is None:
            self.window.show_message_box(
                "Log block",
                "Select a valid object + slot/topk + YAML before logging.",
            )
            return
        try:
            extras = self._parse_extra_args()
        except ValueError as e:
            self.window.show_message_box("Args", f"Could not parse extra args: {e}")
            return

        cpath = ""
        if self._docker:
            try:
                cpath = host_path_to_container(ypath, self._docker_data_root)
            except ValueError:
                self.window.show_message_box(
                    "Docker path",
                    f"YAML must be under --docker-data-root:\n{self._docker_data_root}\nGot:\n{ypath}",
                )
                return

        default_trials = max(len(self._exit_codes), self.DEFAULT_TRIALS_GUESS)
        got = prompt_block_tk(default_trials=default_trials)
        if got is None:
            self.window.show_message_box("Log", "Cancelled — no CSV row written.")
            return
        successes, trials = got

        ent = next((e for e in self._catalog if e.path.resolve() == ypath.resolve()), None)
        pat = ent.pattern if ent else ""

        row = {
            "timestamp_iso": datetime.utcnow().isoformat() + "Z",
            "scan_dirs": self._scan_dirs_joined,
            "object_folder": self._current_object_name() or "",
            "yaml_host_path": str(ypath),
            "yaml_container_path": cpath,
            "slot_nominal": slot,
            "topk": topk,
            "filename_pattern": pat,
            "successes_reported": successes,
            "trials_in_block": trials,
            "trial_exit_codes": ",".join(str(x) for x in self._exit_codes),
            "extra_args": " ".join(shlex.quote(x) for x in extras),
            "docker_mode": self._docker,
            "hostname": socket.gethostname(),
        }
        append_log_row(self._log_dir, row)
        self.window.show_message_box(
            "Logged",
            f"Appended to {self._log_dir / LOG_NAME}\n"
            f"[{row['object_folder']}] slot={slot} topk={topk} "
            f"→ {successes}/{trials}",
        )
        # New block starts empty so you don't accidentally carry exit codes across combos.
        self._session_key = None
        self._exit_codes = []
        self._trial_lbl.text = "Runs since last log: 0 (new block)"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "scan_dirs",
        type=Path,
        nargs="+",
        help="Roots to rglob eval *_grasps.yaml (e.g. data/Mug data/RectPrism).",
    )
    ap.add_argument(
        "--repo-root",
        type=Path,
        default=Path("."),
        help="Repo root containing ws/install (default: current directory).",
    )
    ap.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    ap.add_argument(
        "--docker-exec",
        action="store_true",
        help="Run ros2 via docker exec (see --container-name).",
    )
    ap.add_argument(
        "--docker-data-root",
        type=Path,
        default=Path("data"),
        help="Host tree mounted at /home/ros/data (default: ./data). YAML path must be under it in docker mode.",
    )
    ap.add_argument(
        "--container-name",
        default="4d569d890f6f_ha_local_sim",
        help="Docker container name (docker ps). Compose often prefixes the service name.",
    )
    ap.add_argument(
        "--workspace-setup",
        default="source /home/ros/ws/install/setup.bash",
        help="Shell snippet before ros2 inside docker.",
    )
    args = ap.parse_args()

    repo = args.repo_root.resolve()
    joined = ";".join(str(p.resolve()) for p in args.scan_dirs)

    try:
        gui.Application.instance.initialize()
        _ = IdealSettingsOnlineApp(
            scan_dirs=list(args.scan_dirs),
            scan_dirs_joined=joined,
            repo_root=repo,
            log_dir=args.log_dir,
            docker_exec=bool(args.docker_exec),
            docker_data_root=args.docker_data_root,
            container_name=args.container_name,
            ws_setup=args.workspace_setup,
        )
    except Exception as e:  # noqa: BLE001
        print(e, file=sys.stderr)
        import traceback

        traceback.print_exc()
        return 1

    gui.Application.instance.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
