#!/usr/bin/env python3

"""
Validate current base-link / trunk / leg-mount boundary invariants for dog2.

This script enforces the rule that:
  - legacy `urdf_shift_*` bookkeeping is not present in the xacro source
  - `base_link` is the only URDF root and carries trunk inertial/collision/visual
  - legacy `base_footprint` / `base_offset_joint` / `base_link_cad` frames are absent
  - leg installation is expressed via `*_leg_mount_fixed` under `base_link`
"""

from __future__ import annotations

import argparse
import math
import re
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

EPS = 1e-4
STRICT_EPS = 1e-6
ZERO_VEC = (0.0, 0.0, 0.0)

EXPECTED_BASE_LINK_INERTIAL = (0.000225, 0.00253, 0.0)
EXPECTED_BASE_LINK_VISUAL = (-1.226975, 0.74953, -0.2649)
EXPECTED_BASE_LINK_COLLISION = (-0.00586, -0.000001, -0.006837)
EXPECTED_BASE_LINK_COLLISION_BOX_SIZE = (0.342, 0.160, 0.100333)
FORBIDDEN_LINKS = ("base_footprint", "base_link_cad")
FORBIDDEN_JOINTS = ("base_offset_joint", "base_link_cad_fixed")
EXPECTED_LEG_MOUNTS = {
    "lf_leg_mount_fixed": {
        "parent": "base_link",
        "child": "lf_leg_mount",
        "xyz": (-0.124375, -0.06, 0.0),
        "rpy": (math.pi / 2.0, 0.0, 0.0),
    },
    "lh_leg_mount_fixed": {
        "parent": "base_link",
        "child": "lh_leg_mount",
        "xyz": (0.122125, -0.06, 0.0),
        "rpy": (math.pi / 2.0, 0.0, 0.0),
    },
    "rh_leg_mount_fixed": {
        "parent": "base_link",
        "child": "rh_leg_mount",
        "xyz": (0.122125, 0.06, 0.0),
        "rpy": (math.pi / 2.0, 0.0, -math.pi),
    },
    "rf_leg_mount_fixed": {
        "parent": "base_link",
        "child": "rf_leg_mount",
        "xyz": (-0.119875, 0.06, 0.0),
        "rpy": (math.pi / 2.0, 0.0, -math.pi),
    },
}
EXPECTED_RAIL_JOINTS = {
    "lf_rail_joint": "lf_leg_mount",
    "lh_rail_joint": "lh_leg_mount",
    "rh_rail_joint": "rh_leg_mount",
    "rf_rail_joint": "rf_leg_mount",
}


def parse_vec(text: str) -> tuple[float, float, float]:
    parts = text.split()
    if len(parts) != 3:
        raise ValueError(f"Invalid vector format: {text!r}")
    return (float(parts[0]), float(parts[1]), float(parts[2]))


def is_close_vec(a: tuple[float, float, float], b: tuple[float, float, float], tol: float) -> bool:
    return all(math.isclose(x, y, abs_tol=tol) for x, y in zip(a, b))


def fail(msg: str) -> None:
    print(f"[FAIL] {msg}")
    raise SystemExit(1)


def run_xacro_to_urdf(xacro_path: Path) -> Path:
    with tempfile.NamedTemporaryFile(suffix=".urdf", delete=False) as tmp:
        urdf_path = Path(tmp.name)

    controllers_yaml = xacro_path.parents[1] / "config" / "ros2_controllers.yaml"
    cmd = [
        "xacro",
        str(xacro_path),
        f"controllers_yaml:={controllers_yaml}",
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


def assert_no_legacy_shift_tokens(xacro_path: Path) -> None:
    source = xacro_path.read_text(encoding="utf-8")
    if re.search(r'name="urdf_shift_[^"]+"', source) or "${urdf_shift_" in source:
        fail("Legacy urdf_shift_* properties/expressions are still present in the xacro source")


def get_joint_origin(robot: ET.Element, joint_name: str) -> tuple[float, float, float]:
    joint = get_joint(robot, joint_name)

    origin = joint.find("origin")
    if origin is None:
        fail(f"Joint {joint_name} has no origin")

    xyz = origin.attrib.get("xyz")
    if xyz is None:
        fail(f"Joint {joint_name} origin missing xyz")

    return parse_vec(xyz)

def get_joint_rpy(robot: ET.Element, joint_name: str) -> tuple[float, float, float]:
    joint = get_joint(robot, joint_name)
    origin = joint.find("origin")
    if origin is None:
        fail(f"Joint {joint_name} has no origin")
    return parse_vec(origin.attrib.get("rpy", "0 0 0"))


def get_joint_parent(robot: ET.Element, joint_name: str) -> str:
    joint = get_joint(robot, joint_name)
    parent = joint.find("parent")
    if parent is None:
        fail(f"Joint {joint_name} missing parent")
    link = parent.attrib.get("link")
    if not link:
        fail(f"Joint {joint_name} parent missing link attribute")
    return link


def get_joint_child(robot: ET.Element, joint_name: str) -> str:
    joint = get_joint(robot, joint_name)
    child = joint.find("child")
    if child is None:
        fail(f"Joint {joint_name} missing child")
    link = child.attrib.get("link")
    if not link:
        fail(f"Joint {joint_name} child missing link attribute")
    return link


def get_joint(robot: ET.Element, joint_name: str) -> ET.Element:
    joint = robot.find(f"./joint[@name='{joint_name}']")
    if joint is None:
        fail(f"Missing joint: {joint_name}")
    return joint


def assert_joint_absent(robot: ET.Element, joint_name: str) -> None:
    if robot.find(f"./joint[@name='{joint_name}']") is not None:
        fail(f"Unexpected legacy joint still present: {joint_name}")


def get_link(robot: ET.Element, link_name: str) -> ET.Element:
    link = robot.find(f"./link[@name='{link_name}']")
    if link is None:
        fail(f"Missing link: {link_name}")
    return link


def assert_link_absent(robot: ET.Element, link_name: str) -> None:
    if robot.find(f"./link[@name='{link_name}']") is not None:
        fail(f"Unexpected legacy link still present: {link_name}")


def get_root_links(robot: ET.Element) -> list[str]:
    link_names = {
        link.attrib.get("name")
        for link in robot.findall("./link")
        if link.attrib.get("name")
    }
    child_links = {
        child.attrib.get("link")
        for child in (joint.find("child") for joint in robot.findall("./joint"))
        if child is not None and child.attrib.get("link")
    }
    return sorted(link_names - child_links)


def assert_unique_root_link(robot: ET.Element, expected_root: str) -> None:
    roots = get_root_links(robot)
    if roots != [expected_root]:
        fail(f"URDF root mismatch: expected only {expected_root!r}, got {roots}")


def get_link_sub_origin(robot: ET.Element, link_name: str, tag_name: str) -> tuple[float, float, float]:
    link = get_link(robot, link_name)
    sub = link.find(tag_name)
    if sub is None:
        fail(f"Link {link_name} missing {tag_name}")

    origin = sub.find("origin")
    if origin is None:
        fail(f"Link {link_name} {tag_name} missing origin")

    xyz = origin.attrib.get("xyz")
    if xyz is None:
        fail(f"Link {link_name} {tag_name} origin missing xyz")

    return parse_vec(xyz)


def get_link_sub_geometry(robot: ET.Element, link_name: str, tag_name: str) -> ET.Element:
    link = get_link(robot, link_name)
    sub = link.find(tag_name)
    if sub is None:
        fail(f"Link {link_name} missing {tag_name}")

    geometry = sub.find("geometry")
    if geometry is None:
        fail(f"Link {link_name} {tag_name} missing geometry")
    return geometry


def get_link_sub_box_size(robot: ET.Element, link_name: str, tag_name: str) -> tuple[float, float, float]:
    geometry = get_link_sub_geometry(robot, link_name, tag_name)
    box = geometry.find("box")
    if box is None:
        fail(f"Link {link_name} {tag_name} geometry is not a box primitive")
    size = box.attrib.get("size")
    if size is None:
        fail(f"Link {link_name} {tag_name} box missing size")
    return parse_vec(size)


def assert_link_has_no_tag(robot: ET.Element, link_name: str, tag_name: str) -> None:
    link = get_link(robot, link_name)
    if link.find(tag_name) is not None:
        fail(f"Link {link_name} should not contain {tag_name} in the current trunk split")


def assert_close_vec(name: str, actual: tuple[float, float, float], expected: tuple[float, float, float], tol: float) -> None:
    if not is_close_vec(actual, expected, tol):
        fail(f"{name} mismatch: expected={expected}, got={actual}, tol={tol}")


def assert_joint_matches(
    robot: ET.Element,
    joint_name: str,
    *,
    parent: str,
    child: str,
    xyz: tuple[float, float, float],
    rpy: tuple[float, float, float],
    tol: float,
) -> None:
    if get_joint_parent(robot, joint_name) != parent:
        fail(f"{joint_name} parent mismatch: expected={parent}, got={get_joint_parent(robot, joint_name)}")
    if get_joint_child(robot, joint_name) != child:
        fail(f"{joint_name} child mismatch: expected={child}, got={get_joint_child(robot, joint_name)}")
    assert_close_vec(f"{joint_name} origin xyz", get_joint_origin(robot, joint_name), xyz, tol)
    assert_close_vec(f"{joint_name} origin rpy", get_joint_rpy(robot, joint_name), rpy, tol)


def get_link_inertial_origin(robot: ET.Element, link_name: str) -> tuple[float, float, float]:
    link = get_link(robot, link_name)
    inertial = link.find("inertial")
    if inertial is None:
        fail(f"Link {link_name} missing inertial")

    origin = inertial.find("origin")
    if origin is None:
        fail(f"Link {link_name} inertial missing origin")

    xyz = origin.attrib.get("xyz")
    if xyz is None:
        fail(f"Link {link_name} inertial origin missing xyz")

    return parse_vec(xyz)


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate dog2 URDF shift boundary invariants")
    parser.add_argument(
        "xacro_file",
        nargs="?",
        default="src/dog2_description/urdf/dog2.urdf.xacro",
        help="Path to dog2 xacro file",
    )
    parser.add_argument("--tolerance", "--tol", dest="tolerance", type=float, default=EPS,
                        help="Absolute tolerance (default: 1e-4)")
    parser.add_argument(
        "--strict",
        action="store_true",
        help="Enable strict tolerance mode (overrides tolerance to 1e-6)",
    )
    args = parser.parse_args()

    tol = STRICT_EPS if args.strict else args.tolerance

    xacro_path = Path(args.xacro_file).resolve()
    if not xacro_path.exists():
        fail(f"xacro file not found: {xacro_path}")

    urdf_path = run_xacro_to_urdf(xacro_path)

    try:
        assert_no_legacy_shift_tokens(xacro_path)
        root = ET.parse(urdf_path).getroot()

        assert_unique_root_link(root, "base_link")
        for link_name in FORBIDDEN_LINKS:
            assert_link_absent(root, link_name)
        for joint_name in FORBIDDEN_JOINTS:
            assert_joint_absent(root, joint_name)

        base_inertial = get_link_inertial_origin(root, "base_link")
        assert_close_vec("base_link inertial origin", base_inertial, EXPECTED_BASE_LINK_INERTIAL, tol)
        base_visual = get_link_sub_origin(root, "base_link", "visual")
        assert_close_vec("base_link visual origin", base_visual, EXPECTED_BASE_LINK_VISUAL, tol)
        base_collision = get_link_sub_origin(root, "base_link", "collision")
        assert_close_vec("base_link collision origin", base_collision, EXPECTED_BASE_LINK_COLLISION, tol)
        base_collision_box = get_link_sub_box_size(root, "base_link", "collision")
        assert_close_vec("base_link collision box size", base_collision_box, EXPECTED_BASE_LINK_COLLISION_BOX_SIZE, tol)

        for joint_name, expected in EXPECTED_LEG_MOUNTS.items():
            assert_joint_matches(root, joint_name, tol=tol, **expected)

        for joint_name, expected_parent in EXPECTED_RAIL_JOINTS.items():
            if get_joint_parent(root, joint_name) != expected_parent:
                fail(
                    f"{joint_name} parent mismatch: expected={expected_parent}, "
                    f"got={get_joint_parent(root, joint_name)}"
                )
            assert_close_vec(f"{joint_name} origin xyz", get_joint_origin(root, joint_name), ZERO_VEC, tol)
            assert_close_vec(f"{joint_name} origin rpy", get_joint_rpy(root, joint_name), ZERO_VEC, tol)

        mode = "strict" if args.strict else "normal"
        print(f"[PASS] URDF shift boundary checks passed (mode={mode}, tol={tol:g}).")
        return 0
    finally:
        try:
            urdf_path.unlink(missing_ok=True)
        except OSError:
            pass


if __name__ == "__main__":
    sys.exit(main())
