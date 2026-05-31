#!/usr/bin/env python3

"""
Validate dog2_symmetric URDF invariants: symmetric leg mounts, symmetric trunk
inertia, symmetric knee Z, symmetric rf hip, symmetric foot tip, and rail
joint topology.

This is the symmetric counterpart of check_urdf_shift_boundary.py.
"""

from __future__ import annotations

import math
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

EPS = 1e-4
ZERO_VEC = (0.0, 0.0, 0.0)

# symmetric: lf X = -rf X, lh X = -rh X, Y pairs are mirrored
EXPECTED_LEG_MOUNTS = {
    "lf_leg_mount_fixed": {
        "parent": "base_link",
        "child": "lf_leg_mount",
        "xyz": (-0.122125, -0.06, 0.0),
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
        "xyz": (-0.122125, 0.06, 0.0),
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

    workspace_install = xacro_path.parents[3] / "install"
    setup_script = workspace_install / "setup.bash"
    if setup_script.exists():
        env_cmd = f"source {setup_script} && xacro {xacro_path} controllers_yaml:={controllers_yaml} -o {urdf_path}"
    else:
        env_cmd = f"xacro {xacro_path} controllers_yaml:={controllers_yaml} -o {urdf_path}"

    proc = subprocess.run(["bash", "-c", env_cmd], capture_output=True, text=True)
    if proc.returncode != 0:
        fail(
            "xacro expansion failed.\n"
            f"Command: {' '.join(cmd)}\n"
            f"stdout:\n{proc.stdout}\n"
            f"stderr:\n{proc.stderr}"
        )
    return urdf_path


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


def get_link(robot: ET.Element, link_name: str) -> ET.Element:
    link = robot.find(f"./link[@name='{link_name}']")
    if link is None:
        fail(f"Missing link: {link_name}")
    return link


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


def get_link_inertia(robot: ET.Element, link_name: str) -> tuple[float, float, float, float, float, float]:
    link = get_link(robot, link_name)
    inertial = link.find("inertial")
    if inertial is None:
        fail(f"Link {link_name} missing inertial")
    inertia = inertial.find("inertia")
    if inertia is None:
        fail(f"Link {link_name} missing inertia")
    return (
        float(inertia.attrib.get("ixx", "0")),
        float(inertia.attrib.get("ixy", "0")),
        float(inertia.attrib.get("ixz", "0")),
        float(inertia.attrib.get("iyy", "0")),
        float(inertia.attrib.get("iyz", "0")),
        float(inertia.attrib.get("izz", "0")),
    )


def get_joint_limit_lower_upper(robot: ET.Element, joint_name: str) -> tuple[str, str] | None:
    joint = get_joint(robot, joint_name)
    limit = joint.find("limit")
    if limit is None:
        return None
    return (limit.attrib.get("lower", ""), limit.attrib.get("upper", ""))


def get_joint_origin_xyz_str(robot: ET.Element, joint_name: str) -> str:
    joint = get_joint(robot, joint_name)
    origin = joint.find("origin")
    if origin is not None:
        return origin.attrib.get("xyz", "0 0 0")
    return "0 0 0"


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


def extract_foot_tip_xyz(robot: ET.Element, foot_joint_name: str) -> tuple[float, float, float]:
    """Extract foot_tip_xyz from a *_foot_fixed joint origin."""
    return get_joint_origin(robot, foot_joint_name)


def extract_hip_xyz(robot: ET.Element, axis_joint_name: str) -> tuple[float, float, float]:
    """Extract hip_xyz from a *_coxa_axis_fixed joint origin."""
    return get_joint_origin(robot, axis_joint_name)


def extract_knee_xyz(robot: ET.Element, axis_joint_name: str) -> tuple[float, float, float]:
    """Extract knee_xyz from a *_femur_axis_fixed joint origin."""
    return get_joint_origin(robot, axis_joint_name)


def main() -> int:
    xacro_path = Path(__file__).resolve().parents[1] / "urdf" / "dog2_symmetric.urdf.xacro"
    if not xacro_path.exists():
        fail(f"xacro file not found: {xacro_path}")

    urdf_path = run_xacro_to_urdf(xacro_path)

    try:
        root = ET.parse(urdf_path).getroot()

        roots = get_root_links(root)
        if roots != ["base_link"]:
            fail(f"URDF root mismatch: expected ['base_link'], got {roots}")

        # 1. symmetric leg mount positions
        for joint_name, expected in EXPECTED_LEG_MOUNTS.items():
            assert_joint_matches(root, joint_name, tol=EPS, **expected)

        # 2. rail joint parents
        for joint_name, expected_parent in EXPECTED_RAIL_JOINTS.items():
            if get_joint_parent(root, joint_name) != expected_parent:
                fail(f"{joint_name} parent mismatch: expected={expected_parent}, got={get_joint_parent(root, joint_name)}")
            assert_close_vec(f"{joint_name} origin xyz", get_joint_origin(root, joint_name), ZERO_VEC, EPS)
            assert_close_vec(f"{joint_name} origin rpy", get_joint_rpy(root, joint_name), ZERO_VEC, EPS)

        # 3. base_link inertial at origin
        base_inertial = get_link_inertial_origin(root, "base_link")
        assert_close_vec("base_link inertial origin", base_inertial, ZERO_VEC, EPS)

        # 4. base_link inertia pure diagonal (ixy=ixz=iyz=0)
        ixx, ixy, ixz, iyy, iyz, izz = get_link_inertia(root, "base_link")
        if abs(ixy) > EPS:
            fail(f"base_link ixy must be 0 (diagonal), got {ixy}")
        if abs(ixz) > EPS:
            fail(f"base_link ixz must be 0 (diagonal), got {ixz}")
        if abs(iyz) > EPS:
            fail(f"base_link iyz must be 0 (diagonal), got {iyz}")

        # 5. rf hip offset == (0.016, 0.0199, 0.055)
        rf_hip = extract_hip_xyz(root, "rf_coxa_axis_fixed")
        expected_rf_hip = (0.016, 0.0199, 0.055)
        assert_close_vec("rf coxa_axis_fixed xyz (hip)", rf_hip, expected_rf_hip, EPS)

        # 6. rh / rf knee Z == -0.0274
        rh_knee = extract_knee_xyz(root, "rh_femur_axis_fixed")
        rf_knee = extract_knee_xyz(root, "rf_femur_axis_fixed")
        expected_knee_z = -0.0274
        if abs(rh_knee[2] - expected_knee_z) > EPS:
            fail(f"rh knee Z mismatch: expected {expected_knee_z}, got {rh_knee[2]}")
        if abs(rf_knee[2] - expected_knee_z) > EPS:
            fail(f"rf knee Z mismatch: expected {expected_knee_z}, got {rf_knee[2]}")

        # 7. leg4 foot_tip == leg3 foot_tip
        leg3_foot = extract_foot_tip_xyz(root, "rh_foot_fixed")
        leg4_foot = extract_foot_tip_xyz(root, "rf_foot_fixed")
        assert_close_vec("leg4 foot_tip vs leg3 foot_tip", leg4_foot, leg3_foot, EPS)

        print("[PASS] Symmetric URDF checks passed (tol=1e-4).")
        return 0
    finally:
        try:
            urdf_path.unlink(missing_ok=True)
        except OSError:
            pass


if __name__ == "__main__":
    sys.exit(main())
