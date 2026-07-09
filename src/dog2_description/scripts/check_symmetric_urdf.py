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
            f"Command: {env_cmd}\n"
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


def get_inertial_element(robot: ET.Element, link_name: str) -> ET.Element:
    link = get_link(robot, link_name)
    inertial = link.find("inertial")
    if inertial is None:
        fail(f"Link {link_name} missing inertial")
    return inertial


def get_link_inertial_origin(robot: ET.Element, link_name: str) -> tuple[float, float, float]:
    inertial = get_inertial_element(robot, link_name)
    origin = inertial.find("origin")
    if origin is None:
        fail(f"Link {link_name} inertial missing origin")
    xyz = origin.attrib.get("xyz")
    if xyz is None:
        fail(f"Link {link_name} inertial origin missing xyz")
    return parse_vec(xyz)


def get_link_inertial_origin_rpy(robot: ET.Element, link_name: str) -> tuple[float, float, float]:
    inertial = get_inertial_element(robot, link_name)
    origin = inertial.find("origin")
    if origin is None:
        return (0.0, 0.0, 0.0)
    return parse_vec(origin.attrib.get("rpy", "0 0 0"))


def get_link_mass(robot: ET.Element, link_name: str) -> float:
    inertial = get_inertial_element(robot, link_name)
    mass = inertial.find("mass")
    if mass is None:
        fail(f"Link {link_name} inertial missing mass")
    return float(mass.attrib.get("value", "0"))


def get_link_inertia(robot: ET.Element, link_name: str) -> tuple[float, float, float, float, float, float]:
    inertial = get_inertial_element(robot, link_name)
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


def _extract_geometry_data(geom: ET.Element, link_name: str) -> tuple[str, str]:
    """Return (type, data).
    mesh   → filename
    sphere → radius
    box    → size
    cylinder → "radius length"
    """
    for child in geom:
        tag = child.tag
        if tag == "mesh":
            return (tag, child.attrib.get("filename", ""))
        if tag == "sphere":
            return (tag, child.attrib.get("radius", ""))
        if tag == "box":
            return (tag, child.attrib.get("size", ""))
        if tag == "cylinder":
            r = child.attrib.get("radius", "")
            l = child.attrib.get("length", "")
            return (tag, f"{r} {l}")
    fail(f"Link {link_name} geometry has no child")


def get_link_collision_geometry(robot: ET.Element, link_name: str) -> tuple[str, str, str, str]:
    """Return (type, data, xyz_str, rpy_str)."""
    link = get_link(robot, link_name)
    collision = link.find("collision")
    if collision is None:
        fail(f"Link {link_name} missing collision")
    origin = collision.find("origin")
    xyz = origin.attrib.get("xyz", "0 0 0") if origin is not None else "0 0 0"
    rpy = origin.attrib.get("rpy", "0 0 0") if origin is not None else "0 0 0"
    geom = collision.find("geometry")
    if geom is None:
        fail(f"Link {link_name} collision missing geometry")
    gtype, data = _extract_geometry_data(geom, link_name)
    return (gtype, data, xyz, rpy)


def get_link_visual_geometry(robot: ET.Element, link_name: str) -> tuple[str, str, str, str]:
    """Return (type, data, xyz_str, rpy_str)."""
    link = get_link(robot, link_name)
    visual = link.find("visual")
    if visual is None:
        fail(f"Link {link_name} missing visual")
    origin = visual.find("origin")
    xyz = origin.attrib.get("xyz", "0 0 0") if origin is not None else "0 0 0"
    rpy = origin.attrib.get("rpy", "0 0 0") if origin is not None else "0 0 0"
    geom = visual.find("geometry")
    if geom is None:
        fail(f"Link {link_name} visual missing geometry")
    gtype, data = _extract_geometry_data(geom, link_name)
    return (gtype, data, xyz, rpy)


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

        _leg_prefixes = ["lf", "lh", "rh", "rf"]

        # =====================================================================
        # 1. symmetric leg mount positions
        # =====================================================================
        for joint_name, expected in EXPECTED_LEG_MOUNTS.items():
            assert_joint_matches(root, joint_name, tol=EPS, **expected)

        # =====================================================================
        # 2. rail joint parents
        # =====================================================================
        for joint_name, expected_parent in EXPECTED_RAIL_JOINTS.items():
            if get_joint_parent(root, joint_name) != expected_parent:
                fail(f"{joint_name} parent mismatch: expected={expected_parent}, got={get_joint_parent(root, joint_name)}")
            assert_close_vec(f"{joint_name} origin xyz", get_joint_origin(root, joint_name), ZERO_VEC, EPS)
            assert_close_vec(f"{joint_name} origin rpy", get_joint_rpy(root, joint_name), ZERO_VEC, EPS)

        # =====================================================================
        # 3. base_link inertial: origin at (0,0,0), rpy=(0,0,0)
        # =====================================================================
        base_xyz = get_link_inertial_origin(root, "base_link")
        assert_close_vec("base_link inertial origin xyz", base_xyz, ZERO_VEC, EPS)
        base_rpy = get_link_inertial_origin_rpy(root, "base_link")
        assert_close_vec("base_link inertial origin rpy", base_rpy, ZERO_VEC, EPS)

        # =====================================================================
        # 4. base_link mass == 6.0
        # =====================================================================
        mass = get_link_mass(root, "base_link")
        if not math.isclose(mass, 6.0, abs_tol=EPS):
            fail(f"base_link mass mismatch: expected 6.0, got {mass}")

        # =====================================================================
        # 5. base_link inertia: pure diagonal with exact expected values
        # =====================================================================
        ixx, ixy, ixz, iyy, iyz, izz = get_link_inertia(root, "base_link")
        if abs(ixy) > EPS:
            fail(f"base_link ixy must be 0 (diagonal), got {ixy}")
        if abs(ixz) > EPS:
            fail(f"base_link ixz must be 0 (diagonal), got {ixz}")
        if abs(iyz) > EPS:
            fail(f"base_link iyz must be 0 (diagonal), got {iyz}")
        if not math.isclose(ixx, 0.0153, abs_tol=EPS):
            fail(f"base_link ixx mismatch: expected 0.0153, got {ixx}")
        if not math.isclose(iyy, 0.044, abs_tol=EPS):
            fail(f"base_link iyy mismatch: expected 0.044, got {iyy}")
        if not math.isclose(izz, 0.052, abs_tol=EPS):
            fail(f"base_link izz mismatch: expected 0.052, got {izz}")

        # =====================================================================
        # 6. base_link collision: box primitive at origin
        # =====================================================================
        col_type, col_size, col_xyz, col_rpy = get_link_collision_geometry(root, "base_link")
        if col_type != "box":
            fail(f"base_link collision geometry type: expected 'box', got '{col_type}'")
        assert_close_vec("base_link collision origin xyz", parse_vec(col_xyz), ZERO_VEC, EPS)
        assert_close_vec("base_link collision origin rpy", parse_vec(col_rpy), ZERO_VEC, EPS)
        expected_col_size = parse_vec("0.342 0.160 0.100333")
        assert_close_vec("base_link collision box size", parse_vec(col_size), expected_col_size, EPS)

        # =====================================================================
        # 7. base_link visual: box primitive at origin (same as collision)
        # =====================================================================
        vis_type, vis_data, vis_xyz, vis_rpy = get_link_visual_geometry(root, "base_link")
        if vis_type != "box":
            fail(f"base_link visual geometry type: expected 'box', got '{vis_type}'")
        assert_close_vec("base_link visual origin xyz", parse_vec(vis_xyz), ZERO_VEC, EPS)
        assert_close_vec("base_link visual origin rpy", parse_vec(vis_rpy), ZERO_VEC, EPS)
        assert_close_vec("base_link visual box size", parse_vec(vis_data), expected_col_size, EPS)

        # =====================================================================
        # 8. No base_link.STL references in symmetric URDF
        # =====================================================================
        urdf_text = ET.tostring(root, encoding="unicode")
        if "base_link.STL" in urdf_text or "base_link.stl" in urdf_text:
            fail("Symmetric URDF must not reference base_link.STL (trunk is primitive box)")

        # =====================================================================
        # 9. rf hip offset == (0.016, 0.0199, 0.055)
        # =====================================================================
        rf_hip = extract_hip_xyz(root, "rf_coxa_axis_fixed")
        expected_rf_hip = (0.016, 0.0199, 0.055)
        assert_close_vec("rf coxa_axis_fixed xyz (hip)", rf_hip, expected_rf_hip, EPS)

        # =====================================================================
        # 10. rh / rf knee Z == -0.0274
        # =====================================================================
        rh_knee = extract_knee_xyz(root, "rh_femur_axis_fixed")
        rf_knee = extract_knee_xyz(root, "rf_femur_axis_fixed")
        expected_knee_z = -0.0274
        if abs(rh_knee[2] - expected_knee_z) > EPS:
            fail(f"rh knee Z mismatch: expected {expected_knee_z}, got {rh_knee[2]}")
        if abs(rf_knee[2] - expected_knee_z) > EPS:
            fail(f"rf knee Z mismatch: expected {expected_knee_z}, got {rf_knee[2]}")

        # =====================================================================
        # 11. foot_tip symmetry: leg1 == leg2, leg3 == leg4, left/right x sign mirror
        # =====================================================================
        leg1_foot = extract_foot_tip_xyz(root, "lf_foot_fixed")
        leg2_foot = extract_foot_tip_xyz(root, "lh_foot_fixed")
        leg3_foot = extract_foot_tip_xyz(root, "rh_foot_fixed")
        leg4_foot = extract_foot_tip_xyz(root, "rf_foot_fixed")

        # leg1 == leg2 (both left side)
        assert_close_vec("leg1 foot_tip vs leg2 foot_tip", leg1_foot, leg2_foot, EPS)
        # leg3 == leg4 (both right side)
        assert_close_vec("leg4 foot_tip vs leg3 foot_tip", leg3_foot, leg4_foot, EPS)
        # left vs right: X sign mirror, Y/Z identical
        if not math.isclose(leg1_foot[0], -leg3_foot[0], abs_tol=EPS):
            fail(f"left/right foot_tip X not mirrored: leg1 X={leg1_foot[0]}, leg3 X={leg3_foot[0]}")
        if not math.isclose(leg1_foot[1], leg3_foot[1], abs_tol=EPS):
            fail(f"left/right foot_tip Y mismatch: leg1 Y={leg1_foot[1]}, leg3 Y={leg3_foot[1]}")
        if not math.isclose(leg1_foot[2], leg3_foot[2], abs_tol=EPS):
            fail(f"left/right foot_tip Z mismatch: leg1 Z={leg1_foot[2]}, leg3 Z={leg3_foot[2]}")

        # =====================================================================
        # 12. Leg link inertial symmetry — mass, COM, inertia
        # =====================================================================
        # Each spec: (link_suffix, com_grouping)
        #   "all_equal"         → all 4 legs identical
        #   "left_right_pairs"  → left pair (lf,lh) match, right pair (rh,rf) match
        #   "diagonal_pairs"    → (lf,rh) match, (lh,rf) match
        LEG_LINK_SPECS = [
            ("rail_link", "diagonal_pairs"),
            ("coxa_link", "all_equal"),
            ("femur_link", "left_right_pairs"),
            ("tibia_link", "left_right_pairs"),
            ("foot_link", "all_equal"),
        ]
        for link_suffix, com_grouping in LEG_LINK_SPECS:
            _masses = [get_link_mass(root, f"{p}_{link_suffix}") for p in _leg_prefixes]
            if not all(math.isclose(m, _masses[0], abs_tol=EPS) for m in _masses):
                fail(f"{link_suffix} mass not uniform: {dict(zip(_leg_prefixes, _masses))}")

            _inertias = [get_link_inertia(root, f"{p}_{link_suffix}") for p in _leg_prefixes]
            for i in range(1, 4):
                if not all(math.isclose(_inertias[i][j], _inertias[0][j], abs_tol=EPS) for j in range(6)):
                    fail(f"{link_suffix} inertia mismatch {_leg_prefixes[i]} vs lf: "
                         f"lf={_inertias[0]}, {_leg_prefixes[i]}={_inertias[i]}")

            _coms = [get_link_inertial_origin(root, f"{p}_{link_suffix}") for p in _leg_prefixes]
            if com_grouping == "all_equal":
                for i in range(1, 4):
                    if not all(math.isclose(_coms[i][j], _coms[0][j], abs_tol=EPS) for j in range(3)):
                        fail(f"{link_suffix} COM not uniform: lf={_coms[0]}, {_leg_prefixes[i]}={_coms[i]}")
            elif com_grouping == "left_right_pairs":
                if not all(math.isclose(_coms[i][j], _coms[0][j], abs_tol=EPS) for i in (0, 1) for j in range(3)):
                    fail(f"{link_suffix} COM mismatch between left legs: lf={_coms[0]}, lh={_coms[1]}")
                if not all(math.isclose(_coms[i][j], _coms[2][j], abs_tol=EPS) for i in (2, 3) for j in range(3)):
                    fail(f"{link_suffix} COM mismatch between right legs: rh={_coms[2]}, rf={_coms[3]}")
            elif com_grouping == "diagonal_pairs":
                # (lf, rh) and (lh, rf) are diagonal pairs
                if not all(math.isclose(_coms[0][j], _coms[2][j], abs_tol=EPS) for j in range(3)):
                    fail(f"{link_suffix} COM mismatch: lf({_coms[0]}) != rh({_coms[2]})")
                if not all(math.isclose(_coms[1][j], _coms[3][j], abs_tol=EPS) for j in range(3)):
                    fail(f"{link_suffix} COM mismatch: lh({_coms[1]}) != rf({_coms[3]})")

        # =====================================================================
        # 13. Femur/tibia inertia cross terms must be zero (symmetric dynamics)
        # =====================================================================
        for link_suffix in ("femur_link", "tibia_link"):
            for prefix in _leg_prefixes:
                _inertia = get_link_inertia(root, f"{prefix}_{link_suffix}")
                if abs(_inertia[1]) > EPS:
                    fail(f"{prefix}_{link_suffix} ixy={_inertia[1]} must be 0")
                if abs(_inertia[2]) > EPS:
                    fail(f"{prefix}_{link_suffix} ixz={_inertia[2]} must be 0")
                if abs(_inertia[4]) > EPS:
                    fail(f"{prefix}_{link_suffix} iyz={_inertia[4]} must be 0")

        # =====================================================================
        # 14. Visual & collision geometry: rail/coxa use mesh, femur/tibia use
        #     cylinder primitives (use_primitive_links:=true).
        # =====================================================================
        _mesh_link_suffixes = ["rail_link", "coxa_link"]
        for link_suffix in _mesh_link_suffixes:
            for prefix in _leg_prefixes:
                link_name = f"{prefix}_{link_suffix}"

                # Visual mesh
                vis_type, vis_data, _, _ = get_link_visual_geometry(root, link_name)
                if vis_type == "mesh":
                    if "meshes_symmetric" not in vis_data:
                        fail(f"{link_name} visual mesh path missing meshes_symmetric: {vis_data}")
                    if "meshes/" in vis_data and "meshes_symmetric" not in vis_data:
                        fail(f"{link_name} visual mesh uses old meshes/ path: {vis_data}")
                # Collision mesh
                col_type, col_data, _, _ = get_link_collision_geometry(root, link_name)
                if col_type == "mesh":
                    if "meshes_symmetric" not in col_data:
                        fail(f"{link_name} collision mesh path missing meshes_symmetric: {col_data}")
                    if "meshes/" in col_data and "meshes_symmetric" not in col_data:
                        fail(f"{link_name} collision mesh uses old meshes/ path: {col_data}")

        # Femur/tibia use cylinder primitives
        _primitive_link_suffixes = ["femur_link", "tibia_link"]
        for link_suffix in _primitive_link_suffixes:
            for prefix in _leg_prefixes:
                link_name = f"{prefix}_{link_suffix}"

                vis_type, vis_data, _, _ = get_link_visual_geometry(root, link_name)
                if vis_type != "cylinder":
                    fail(f"{link_name} visual geometry type: expected 'cylinder', got '{vis_type}'")

                col_type, col_data, _, _ = get_link_collision_geometry(root, link_name)
                if col_type != "cylinder":
                    fail(f"{link_name} collision geometry type: expected 'cylinder', got '{col_type}'")

                # Parse "radius length" and sanity-check
                vis_parts = vis_data.split()
                col_parts = col_data.split()
                if len(vis_parts) != 2 or len(col_parts) != 2:
                    fail(f"{link_name} cylinder data format: expected 'radius length', got vis='{vis_data}' col='{col_data}'")

                vis_radius, vis_length = float(vis_parts[0]), float(vis_parts[1])
                col_radius, col_length = float(col_parts[0]), float(col_parts[1])

                if not math.isclose(vis_radius, col_radius, abs_tol=EPS):
                    fail(f"{link_name} visual radius ({vis_radius}) != collision radius ({col_radius})")
                if not math.isclose(vis_length, col_length, abs_tol=EPS):
                    fail(f"{link_name} visual length ({vis_length}) != collision length ({col_length})")

                # Femur: radius ~0.018, length ~0.2; Tibia: radius ~0.014, length ~0.327
                if link_suffix == "femur_link":
                    if not math.isclose(vis_radius, 0.018, abs_tol=1e-3):
                        fail(f"{link_name} femur radius unexpected: {vis_radius}")
                elif link_suffix == "tibia_link":
                    if not math.isclose(vis_radius, 0.014, abs_tol=1e-3):
                        fail(f"{link_name} tibia radius unexpected: {vis_radius}")

        # No base_link.STL anywhere in the URDF
        urdf_text = ET.tostring(root, encoding="unicode")
        if "base_link.STL" in urdf_text or "base_link.stl" in urdf_text:
            fail("Symmetric URDF must not reference base_link.STL (trunk is primitive box)")

        # =====================================================================
        # 15. Foot primitive: sphere radius = 0.020, consistent across legs
        # =====================================================================
        _foot_radii = []
        for prefix in _leg_prefixes:
            link_name = f"{prefix}_foot_link"
            col_type, col_data, col_xyz, col_rpy = get_link_collision_geometry(root, link_name)
            if col_type != "sphere":
                fail(f"{link_name} collision geometry type: expected 'sphere', got '{col_type}'")
            assert_close_vec(f"{link_name} collision origin xyz", parse_vec(col_xyz), ZERO_VEC, EPS)
            assert_close_vec(f"{link_name} collision origin rpy", parse_vec(col_rpy), ZERO_VEC, EPS)
            _radius = float(col_data)
            _foot_radii.append(_radius)

            vis_type, vis_data, vis_xyz, vis_rpy = get_link_visual_geometry(root, link_name)
            if vis_type != "sphere":
                fail(f"{link_name} visual geometry type: expected 'sphere', got '{vis_type}'")
            assert_close_vec(f"{link_name} visual origin xyz", parse_vec(vis_xyz), ZERO_VEC, EPS)
            assert_close_vec(f"{link_name} visual origin rpy", parse_vec(vis_rpy), ZERO_VEC, EPS)
            if abs(float(vis_data) - _radius) > EPS:
                fail(f"{link_name} visual radius ({vis_data}) != collision radius ({_radius})")

        if not all(math.isclose(r, 0.020, abs_tol=EPS) for r in _foot_radii):
            fail(f"foot sphere radii not 0.020: {dict(zip(_leg_prefixes, _foot_radii))}")

        for prefix in _leg_prefixes:
            ixx, ixy, ixz, iyy, iyz, izz = get_link_inertia(root, f"{prefix}_foot_link")
            expected_sphere_inertia = 8.0e-06
            inertia_tol = 1e-9
            if not math.isclose(ixx, expected_sphere_inertia, abs_tol=inertia_tol):
                fail(f"{prefix}_foot_link ixx={ixx} expected {expected_sphere_inertia}")
            if not math.isclose(iyy, expected_sphere_inertia, abs_tol=inertia_tol):
                fail(f"{prefix}_foot_link iyy={iyy} expected {expected_sphere_inertia}")
            if not math.isclose(izz, expected_sphere_inertia, abs_tol=inertia_tol):
                fail(f"{prefix}_foot_link izz={izz} expected {expected_sphere_inertia}")
            if abs(ixy) > inertia_tol or abs(ixz) > inertia_tol or abs(iyz) > inertia_tol:
                fail(f"{prefix}_foot_link inertia cross terms must be zero: {(ixy, ixz, iyz)}")

        # =====================================================================
        # 16. Python leg_parameters_symmetric aligns with URDF
        # =====================================================================
        sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "dog2_motion_control"))
        try:
            from dog2_motion_control.leg_parameters_symmetric import LEG_PARAMETERS_SYMMETRIC  # noqa: F811
        except ImportError:
            fail("Could not import LEG_PARAMETERS_SYMMETRIC from dog2_motion_control")

        _expected_mounts = {
            "lf": (-0.122125, -0.06, 0.0),
            "lh": (0.122125, -0.06, 0.0),
            "rh": (0.122125, 0.06, 0.0),
            "rf": (-0.122125, 0.06, 0.0),
        }
        for leg_id, expected_xyz in _expected_mounts.items():
            params = LEG_PARAMETERS_SYMMETRIC.get(leg_id)
            if params is None:
                fail(f"Python LEG_PARAMETERS_SYMMETRIC missing key: {leg_id}")
            actual = params.base_position
            if not is_close_vec(tuple(actual.tolist()), expected_xyz, EPS):
                fail(f"Python {leg_id}.base_position mismatch: expected {expected_xyz}, got {tuple(actual.tolist())}")

            # Verify hip_offset matches *_coxa_axis_fixed origin
            hip_actual = tuple(params.hip_offset.tolist())
            hip_expected = get_joint_origin(root, f"{leg_id}_coxa_axis_fixed")
            if not is_close_vec(hip_actual, hip_expected, EPS):
                fail(f"Python {leg_id}.hip_offset ({hip_actual}) != URDF {leg_id}_coxa_axis_fixed ({hip_expected})")

            # Verify knee_offset matches *_femur_axis_fixed origin
            knee_actual = tuple(params.knee_offset.tolist())
            knee_expected = get_joint_origin(root, f"{leg_id}_femur_axis_fixed")
            if not is_close_vec(knee_actual, knee_expected, EPS):
                fail(f"Python {leg_id}.knee_offset ({knee_actual}) != URDF {leg_id}_femur_axis_fixed ({knee_expected})")

            # Verify foot_tip_offset_tibia matches *_foot_fixed origin
            foot_actual = tuple(params.foot_tip_offset_tibia.tolist())
            foot_expected = get_joint_origin(root, f"{leg_id}_foot_fixed")
            if not is_close_vec(foot_actual, foot_expected, EPS):
                fail(f"Python {leg_id}.foot_tip_offset_tibia ({foot_actual}) != URDF {leg_id}_foot_fixed ({foot_expected})")

        print("[PASS] Symmetric URDF checks passed (tol=1e-4).")
        return 0
    finally:
        try:
            urdf_path.unlink(missing_ok=True)
        except OSError:
            pass


if __name__ == "__main__":
    sys.exit(main())
