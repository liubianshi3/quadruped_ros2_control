#!/usr/bin/env python3

"""Validate dog2 joint semantic directions from the expanded xacro."""

from __future__ import annotations

import argparse
import math
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path


EPS = 1e-6


def fail(msg: str) -> None:
    print(f"[FAIL] {msg}")
    raise SystemExit(1)


def parse_vec(text: str) -> tuple[float, float, float]:
    parts = text.split()
    if len(parts) != 3:
        raise ValueError(f"Invalid vector text: {text!r}")
    return (float(parts[0]), float(parts[1]), float(parts[2]))


def mat_mul(a: tuple[tuple[float, ...], ...], b: tuple[tuple[float, ...], ...]) -> tuple[tuple[float, ...], ...]:
    return tuple(
        tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3))
        for i in range(3)
    )


def mat_vec(a: tuple[tuple[float, ...], ...], v: tuple[float, float, float]) -> tuple[float, float, float]:
    return tuple(sum(a[i][k] * v[k] for k in range(3)) for i in range(3))


def rpy_matrix(rpy: tuple[float, float, float]) -> tuple[tuple[float, ...], ...]:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = (
        (1.0, 0.0, 0.0),
        (0.0, cr, -sr),
        (0.0, sr, cr),
    )
    ry = (
        (cp, 0.0, sp),
        (0.0, 1.0, 0.0),
        (-sp, 0.0, cp),
    )
    rz = (
        (cy, -sy, 0.0),
        (sy, cy, 0.0),
        (0.0, 0.0, 1.0),
    )
    return mat_mul(rz, mat_mul(ry, rx))


def is_close_vec(a: tuple[float, float, float], b: tuple[float, float, float], tol: float = EPS) -> bool:
    return all(math.isclose(x, y, abs_tol=tol) for x, y in zip(a, b))


def run_xacro_to_urdf(xacro_path: Path) -> Path:
    with tempfile.NamedTemporaryFile(suffix=".urdf", delete=False) as tmp:
        urdf_path = Path(tmp.name)

    controllers_yaml = xacro_path.parents[1] / "config" / "ros2_controllers.yaml"
    cmd = [
        "xacro",
        str(xacro_path),
        f"controllers_yaml:={controllers_yaml}",
        "mass_scale:=1.0",
        "-o",
        str(urdf_path),
    ]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0:
        fail(
            "xacro expansion failed.\n"
            f"Command: {' '.join(cmd)}\n"
            f"stdout:\n{proc.stdout}\n"
            f"stderr:\n{proc.stderr}"
        )
    return urdf_path


def get_joint(robot: ET.Element, joint_name: str) -> ET.Element:
    joint = robot.find(f"./joint[@name='{joint_name}']")
    if joint is None:
        fail(f"Missing joint: {joint_name}")
    return joint


def get_joint_parent(robot: ET.Element, joint_name: str) -> str:
    parent = get_joint(robot, joint_name).find("parent")
    if parent is None:
        fail(f"Joint {joint_name} missing parent")
    link = parent.attrib.get("link")
    if not link:
        fail(f"Joint {joint_name} parent missing link")
    return link


def get_joint_rpy(robot: ET.Element, joint_name: str) -> tuple[float, float, float]:
    origin = get_joint(robot, joint_name).find("origin")
    if origin is None:
        fail(f"Joint {joint_name} missing origin")
    rpy_text = origin.attrib.get("rpy", "0 0 0")
    return parse_vec(rpy_text)


def get_joint_axis(robot: ET.Element, joint_name: str) -> tuple[float, float, float]:
    axis = get_joint(robot, joint_name).find("axis")
    if axis is None:
        fail(f"Joint {joint_name} missing axis")
    axis_text = axis.attrib.get("xyz")
    if axis_text is None:
        fail(f"Joint {joint_name} axis missing xyz")
    return parse_vec(axis_text)


def get_joint_parent(robot: ET.Element, joint_name: str) -> str:
    parent = get_joint(robot, joint_name).find("parent")
    if parent is None:
        fail(f"Joint {joint_name} missing parent")
    link = parent.attrib.get("link")
    if not link:
        fail(f"Joint {joint_name} parent missing link")
    return link


def get_joint_child(robot: ET.Element, joint_name: str) -> str:
    child = get_joint(robot, joint_name).find("child")
    if child is None:
        fail(f"Joint {joint_name} missing child")
    link = child.attrib.get("link")
    if not link:
        fail(f"Joint {joint_name} child missing link")
    return link


def build_parent_joint_map(robot: ET.Element) -> dict[str, str]:
    mapping: dict[str, str] = {}
    for joint in robot.findall("./joint"):
        name = joint.attrib.get("name")
        child = joint.find("child")
        child_link = child.attrib.get("link") if child is not None else None
        if not name or not child_link:
            continue
        mapping[child_link] = name
    return mapping


def link_rotation_in_root(
    robot: ET.Element,
    parent_joint_map: dict[str, str],
    link_name: str,
    root_link: str,
) -> tuple[tuple[float, ...], ...]:
    if link_name == root_link:
        return (
            (1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (0.0, 0.0, 1.0),
        )
    parent_joint_name = parent_joint_map.get(link_name)
    if parent_joint_name is None:
        fail(f"Link {link_name} is disconnected from root {root_link}")
    parent_link = get_joint_parent(robot, parent_joint_name)
    return mat_mul(
        link_rotation_in_root(robot, parent_joint_map, parent_link, root_link),
        rpy_matrix(get_joint_rpy(robot, parent_joint_name)),
    )


def joint_rotation_in_root(
    robot: ET.Element,
    parent_joint_map: dict[str, str],
    joint_name: str,
    root_link: str,
) -> tuple[tuple[float, ...], ...]:
    parent_link = get_joint_parent(robot, joint_name)
    return mat_mul(
        link_rotation_in_root(robot, parent_joint_map, parent_link, root_link),
        rpy_matrix(get_joint_rpy(robot, joint_name)),
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate dog2 joint semantic directions")
    parser.add_argument(
        "xacro_file",
        nargs="?",
        default="src/dog2_description/urdf/dog2.urdf.xacro",
        help="Path to dog2 xacro file",
    )
    args = parser.parse_args()

    xacro_path = Path(args.xacro_file).resolve()
    if not xacro_path.exists():
        fail(f"xacro file not found: {xacro_path}")

    urdf_path = run_xacro_to_urdf(xacro_path)
    try:
        root = ET.parse(urdf_path).getroot()
        parent_joint_map = build_parent_joint_map(root)

        prefixes = ("lf", "lh", "rh", "rf")
        rail_expected = (1.0, 0.0, 0.0)
        coxa_expected = (0.0, 0.0, -1.0)
        pitch_expected = (0.0, -1.0, 0.0)

        for prefix in prefixes:
            rail_r = joint_rotation_in_root(root, parent_joint_map, f"{prefix}_rail_joint", "base_link")
            rail_axis_base = mat_vec(rail_r, get_joint_axis(root, f"{prefix}_rail_joint"))
            if not is_close_vec(rail_axis_base, rail_expected):
                fail(f"{prefix}_rail_joint semantic axis mismatch: expected={rail_expected}, got={rail_axis_base}")

            coxa_r = joint_rotation_in_root(root, parent_joint_map, f"{prefix}_coxa_joint", "base_link")
            coxa_axis_base = mat_vec(coxa_r, get_joint_axis(root, f"{prefix}_coxa_joint"))
            if not is_close_vec(coxa_axis_base, coxa_expected):
                fail(f"{prefix}_coxa_joint semantic axis mismatch: expected={coxa_expected}, got={coxa_axis_base}")

            femur_r = joint_rotation_in_root(root, parent_joint_map, f"{prefix}_femur_joint", "base_link")
            femur_axis_base = mat_vec(femur_r, get_joint_axis(root, f"{prefix}_femur_joint"))
            if not is_close_vec(femur_axis_base, pitch_expected):
                fail(f"{prefix}_femur_joint semantic axis mismatch: expected={pitch_expected}, got={femur_axis_base}")

            tibia_r = joint_rotation_in_root(root, parent_joint_map, f"{prefix}_tibia_joint", "base_link")
            tibia_axis_base = mat_vec(tibia_r, get_joint_axis(root, f"{prefix}_tibia_joint"))
            if not is_close_vec(tibia_axis_base, pitch_expected):
                fail(f"{prefix}_tibia_joint semantic axis mismatch: expected={pitch_expected}, got={tibia_axis_base}")

        print("[PASS] Joint semantic checks passed.")
        return 0
    finally:
        try:
            urdf_path.unlink(missing_ok=True)
        except OSError:
            pass


if __name__ == "__main__":
    sys.exit(main())
