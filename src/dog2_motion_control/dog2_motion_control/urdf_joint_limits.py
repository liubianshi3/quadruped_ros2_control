"""Load joint limits from the authoritative dog2 xacro/URDF description."""

from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
import math
from pathlib import Path
from typing import Dict, Tuple
import xml.etree.ElementTree as ET

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
import xacro


RAIL_JOINT_BY_LEG = {
    "lf": "lf_rail_joint",
    "lh": "lh_rail_joint",
    "rh": "rh_rail_joint",
    "rf": "rf_rail_joint",
}

REVOLUTE_JOINT_BY_ROLE = {
    "coxa": "coxa_joint",
    "femur": "femur_joint",
    "tibia": "tibia_joint",
}


@dataclass(frozen=True)
class Dog2UrdfJointLimits:
    """Expanded dog2 xacro joint limits."""

    source_path: str
    rail_by_leg: Dict[str, Tuple[float, float]]
    revolute_by_role: Dict[str, Tuple[float, float]]


def _resolve_dog2_description_paths() -> tuple[Path, Path]:
    workspace_src_dir = Path(__file__).resolve().parents[2] / "dog2_description"
    workspace_xacro = workspace_src_dir / "urdf" / "dog2.urdf.xacro"
    workspace_controllers = workspace_src_dir / "config" / "ros2_controllers.yaml"
    if workspace_xacro.exists() and workspace_controllers.exists():
        return workspace_xacro, workspace_controllers

    try:
        share_dir = Path(get_package_share_directory("dog2_description"))
    except PackageNotFoundError as exc:
        raise FileNotFoundError(
            "Unable to locate dog2_description xacro source from workspace or installed share directory."
        ) from exc

    xacro_path = share_dir / "urdf" / "dog2.urdf.xacro"
    controllers_yaml = share_dir / "config" / "ros2_controllers.yaml"
    if not xacro_path.exists() or not controllers_yaml.exists():
        raise FileNotFoundError(
            f"dog2_description share directory is missing required files: xacro={xacro_path}, "
            f"controllers_yaml={controllers_yaml}"
        )
    return xacro_path, controllers_yaml


def _parse_joint_limit(joint: ET.Element) -> tuple[float, float]:
    limit = joint.find("limit")
    if limit is None:
        raise RuntimeError(f"Joint {joint.attrib.get('name', '<unknown>')} is missing <limit>.")
    lower = limit.attrib.get("lower")
    upper = limit.attrib.get("upper")
    if lower is None or upper is None:
        raise RuntimeError(
            f"Joint {joint.attrib.get('name', '<unknown>')} is missing lower/upper limit attributes."
        )
    return float(lower), float(upper)


def _limits_match(a: tuple[float, float], b: tuple[float, float], tol: float = 1e-9) -> bool:
    return all(math.isclose(x, y, abs_tol=tol) for x, y in zip(a, b))


@lru_cache(maxsize=1)
def load_dog2_urdf_joint_limits() -> Dog2UrdfJointLimits:
    """Expand dog2 xacro and parse the authoritative joint limits."""

    xacro_path, controllers_yaml = _resolve_dog2_description_paths()
    document = xacro.process_file(
        str(xacro_path),
        mappings={"controllers_yaml": str(controllers_yaml)},
    )
    root = ET.fromstring(document.toxml())

    rail_by_leg: Dict[str, Tuple[float, float]] = {}
    revolute_entries: Dict[str, list[tuple[str, tuple[float, float]]]] = {
        role: [] for role in REVOLUTE_JOINT_BY_ROLE
    }

    for joint in root.findall("./joint"):
        joint_name = joint.attrib.get("name", "")
        for leg_id, rail_joint_name in RAIL_JOINT_BY_LEG.items():
            if joint_name == rail_joint_name:
                rail_by_leg[leg_id] = _parse_joint_limit(joint)
        for role, joint_suffix in REVOLUTE_JOINT_BY_ROLE.items():
            if joint_name.endswith(joint_suffix):
                revolute_entries[role].append((joint_name, _parse_joint_limit(joint)))

    missing_rails = sorted(set(RAIL_JOINT_BY_LEG) - set(rail_by_leg))
    if missing_rails:
        raise RuntimeError(f"Missing rail limits for legs: {missing_rails}")

    revolute_by_role: Dict[str, Tuple[float, float]] = {}
    for role, entries in revolute_entries.items():
        if not entries:
            raise RuntimeError(f"Missing revolute joint limits for role '{role}'.")
        canonical = entries[0][1]
        mismatched = [
            (joint_name, limits)
            for joint_name, limits in entries[1:]
            if not _limits_match(limits, canonical)
        ]
        if mismatched:
            raise RuntimeError(
                f"Role '{role}' has inconsistent limits across legs: "
                f"canonical={canonical}, mismatched={mismatched}"
            )
        revolute_by_role[role] = canonical

    return Dog2UrdfJointLimits(
        source_path=str(xacro_path),
        rail_by_leg=rail_by_leg,
        revolute_by_role=revolute_by_role,
    )


def clear_dog2_urdf_joint_limits_cache() -> None:
    """Force the next load to re-expand the xacro source."""

    load_dog2_urdf_joint_limits.cache_clear()
