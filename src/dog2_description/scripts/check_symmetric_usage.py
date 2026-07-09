#!/usr/bin/env python3
"""Audit symmetric model usage: detect real-only hardcodes that leak into symmetric paths."""

from __future__ import annotations

import ast
import os
import subprocess
import sys
from pathlib import Path

WS_SRC = Path(__file__).resolve().parents[2]

FAILS: list[str] = []


def fail(msg: str) -> None:
    FAILS.append(msg)
    print(f"[FAIL] {msg}")


def pass_msg(msg: str) -> None:
    print(f"[PASS] {msg}")


def check_file_contains_urdf_hardcode(filepath: Path) -> None:
    """Flag files that hardcode dog2.urdf.xacro without model_variant."""
    text = filepath.read_text(encoding="utf-8")
    if "dog2_symmetric.urdf.xacro" in text or "get_urdf_xacro_filename" in text or "model_variant" in text:
        return
    if "'dog2.urdf.xacro'" in text or '"dog2.urdf.xacro"' in text or '"urdf", "dog2.urdf.xacro"' in text:
        fail(f"Hardcoded dog2.urdf.xacro without model_variant: {filepath}")


def check_model_variant_module() -> None:
    p = WS_SRC / "dog2_motion_control" / "dog2_motion_control" / "model_variant.py"
    if not p.exists():
        fail(f"Missing model_variant.py: {p}")
        return
    text = p.read_text()
    for name in ("normalize_model_variant", "get_urdf_xacro_filename", "get_leg_parameters"):
        if f"def {name}" not in text:
            fail(f"model_variant.py missing function: {name}")
    pass_msg("model_variant.py has all required functions")


def check_leg_params_symmetric_no_old_coords() -> None:
    p = WS_SRC / "dog2_motion_control" / "dog2_motion_control" / "leg_parameters_symmetric.py"
    text = p.read_text(encoding="utf-8")
    old_coords = ["0.1246", "0.1291"]
    for coord in old_coords:
        if coord in text:
            fail(f"leg_parameters_symmetric.py still references old CAD absolute coordinate: {coord}")
    pass_msg("leg_parameters_symmetric.py uses symmetric base_position coordinates only")


# 2026-07-08 清理：旧的 real-only 工具/演示 launch 已全部删除
# （原豁免白名单里的 19 个文件），不再需要豁免机制。
LEGACY_REAL_URDF_LAUNCHES: set[str] = set()


def check_launch_files() -> None:
    launch_dirs = [
        WS_SRC / "dog2_description" / "launch",
        WS_SRC / "dog2_motion_control" / "launch",
        WS_SRC / "dog2_mpc" / "launch",
        WS_SRC / "dog2_bringup" / "launch",
    ]
    for d in launch_dirs:
        if not d.exists():
            continue
        for f in sorted(d.iterdir()):
            if not f.suffix == ".py":
                continue
            text = f.read_text(encoding="utf-8")
            has_urdf_ref = "dog2.urdf.xacro" in text

            # Skip files that don't reference dog2.urdf.xacro at all
            if not has_urdf_ref:
                continue

            has_variant = "model_variant" in text
            rel = f.relative_to(WS_SRC)
            if has_variant:
                pass_msg(f"Launch has model_variant: {rel}")
            elif d.name == "launch" and d.parent.name == "dog2_description" and f.name in LEGACY_REAL_URDF_LAUNCHES:
                pass_msg(f"Legacy real-only launch allowed: {rel}")
            else:
                fail(f"Launch missing model_variant: {rel}")


def launch_show_args_check() -> None:
    """Check that key launches show model_variant."""
    import tempfile
    launches = [
        "dog2_description view_dog2_xacro.launch.py",
        "dog2_description gazebo_headless.launch.py",
        "dog2_description dog2_fortress_with_gui.launch.py",
        "dog2_motion_control spider_gazebo_position.launch.py",
        "dog2_motion_control spider_gazebo_mpc.launch.py",
        "dog2_motion_control spider_gazebo_complete.launch.py",
        "dog2_mpc complete_simulation.launch.py",
        "dog2_bringup control_stack.launch.py",
        "dog2_bringup effort_research_sim.launch.py",
    ]
    for launch in launches:
        cmd = ["ros2", "launch"] + launch.split() + ["--show-args"]
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
        if proc.returncode != 0:
            fail(f"ros2 launch --show-args failed for {launch}: {proc.stderr.strip()}")
            continue
        if "model_variant" in proc.stdout:
            pass_msg(f"model_variant found in --show-args: {launch}")
        else:
            fail(f"model_variant MISSING from --show-args: {launch}")


def main() -> int:
    print("=== Model variant module audit ===")
    check_model_variant_module()

    print("\n=== leg_parameters_symmetric audit ===")
    check_leg_params_symmetric_no_old_coords()

    print("\n=== Launch model_variant audit ===")
    check_launch_files()

    print("\n=== Launch --show-args audit ===")
    launch_show_args_check()

    if FAILS:
        print(f"\n{len(FAILS)} FAILURES:")
        for f in FAILS:
            print(f"  - {f}")
        return 1
    print("\n[PASS] All symmetric usage checks passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
