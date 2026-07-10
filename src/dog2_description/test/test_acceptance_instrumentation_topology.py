"""Acceptance contact instrumentation must not alter Dog2 mechanics."""

from __future__ import annotations

import os
import subprocess
import xml.etree.ElementTree as ET
from copy import deepcopy
from pathlib import Path

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = PACKAGE_ROOT.parents[1]
XACRO_FILES = ("dog2.urdf.xacro", "dog2_symmetric.urdf.xacro")
MOVABLE_TYPES = {"prismatic", "revolute", "continuous"}
EXPECTED_ACCEPTANCE_SENSORS = {
    "base_acceptance_contact",
    "lf_tibia_acceptance_contact",
    "lh_tibia_acceptance_contact",
    "rh_tibia_acceptance_contact",
    "rf_tibia_acceptance_contact",
}


def _expand(filename: str, enabled: bool) -> ET.Element:
    xacro_path = PACKAGE_ROOT / "urdf" / filename
    controllers_yaml = PACKAGE_ROOT / "config" / "ros2_controllers.yaml"
    env = os.environ.copy()
    install_path = WORKSPACE_ROOT / "install"
    if install_path.exists():
        prefix = env.get("AMENT_PREFIX_PATH", "")
        env["AMENT_PREFIX_PATH"] = (
            f"{install_path}:{prefix}" if prefix else str(install_path)
        )
    result = subprocess.run(
        [
            "xacro",
            str(xacro_path),
            f"controllers_yaml:={controllers_yaml}",
            f"enable_acceptance_contact_sensors:={'true' if enabled else 'false'}",
        ],
        check=True,
        capture_output=True,
        text=True,
        env=env,
    )
    return ET.fromstring(result.stdout)


def _joint_signature(root: ET.Element) -> list[str]:
    return [_canonical_xml(joint) for joint in root.findall("joint") if joint.get("type") in MOVABLE_TYPES]


def _link_signature(root: ET.Element) -> list[str]:
    return [_canonical_xml(link) for link in root.findall("link")]


def _canonical_xml(element: ET.Element) -> str:
    clone = deepcopy(element)
    clone.tail = None
    return ET.tostring(clone, encoding="unicode")


@pytest.mark.parametrize("filename", XACRO_FILES)
def test_acceptance_sensors_preserve_topology_and_link_physics(filename: str) -> None:
    disabled = _expand(filename, enabled=False)
    enabled = _expand(filename, enabled=True)

    assert _joint_signature(enabled) == _joint_signature(disabled)
    assert _link_signature(enabled) == _link_signature(disabled)

    movable = [
        joint for joint in enabled.findall("joint") if joint.get("type") in MOVABLE_TYPES
    ]
    assert len(movable) == 16
    assert sum(joint.get("type") == "prismatic" for joint in movable) == 4
    assert sum(joint.get("type") == "revolute" for joint in movable) == 12
    assert {
        joint.get("name") for joint in movable if joint.get("type") == "prismatic"
    } == {
        "lf_rail_joint",
        "lh_rail_joint",
        "rh_rail_joint",
        "rf_rail_joint",
    }

    disabled_sensors = {
        sensor.get("name")
        for sensor in disabled.findall(".//sensor")
        if "acceptance_contact" in sensor.get("name", "")
    }
    enabled_sensors = {
        sensor.get("name")
        for sensor in enabled.findall(".//sensor")
        if "acceptance_contact" in sensor.get("name", "")
    }
    assert disabled_sensors == set()
    assert enabled_sensors == EXPECTED_ACCEPTANCE_SENSORS
