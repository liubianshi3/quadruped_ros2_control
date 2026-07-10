"""Acceptance contact instrumentation must not alter Dog2 mechanics."""

from __future__ import annotations

import os
import subprocess
import tempfile
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
EXPECTED_CONTACT_TOPICS = {
    "/dog2/gz_contact/base",
    *{
        f"/dog2/gz_contact/{leg}_{kind}"
        for leg in ("lf", "lh", "rh", "rf")
        for kind in ("foot", "tibia")
    },
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


def _canonical_numeric_xml(element: ET.Element) -> str:
    clone = deepcopy(element)
    for node in clone.iter():
        node.tail = None
        if not (node.text or "").strip():
            node.text = None
        text = (node.text or "").strip()
        if text:
            try:
                values = [float(value) for value in text.split()]
            except ValueError:
                pass
            else:
                node.text = " ".join(
                    "0" if abs(value) < 1e-12 else f"{value:.12g}"
                    for value in values
                )
    return ET.tostring(clone, encoding="unicode")


def _to_sdf(root: ET.Element) -> ET.Element:
    with tempfile.TemporaryDirectory(prefix="dog2_acceptance_sdf_") as directory:
        urdf_path = Path(directory) / "robot.urdf"
        urdf_path.write_text(
            ET.tostring(root, encoding="unicode"), encoding="utf-8"
        )
        result = subprocess.run(
            ["ign", "sdf", "-p", str(urdf_path)],
            check=True,
            capture_output=True,
            text=True,
        )
    return ET.fromstring(result.stdout)


def _sdf_mechanical_signature(root: ET.Element) -> tuple[list[str], list[str]]:
    links = []
    for link in root.findall(".//model/link"):
        clone = deepcopy(link)
        for child in list(clone):
            if child.tag not in {"inertial", "collision"}:
                clone.remove(child)
        for collision in clone.findall("collision"):
            surface = collision.find("surface")
            if surface is not None and all(
                not node.attrib and not (node.text or "").strip()
                for node in surface.iter()
            ):
                # libsdformat materializes an empty default ODE surface when a
                # sensor is added to an otherwise untouched collision. It is
                # semantically identical to the implicit default.
                collision.remove(surface)
        links.append(_canonical_numeric_xml(clone))
    joints = [
        _canonical_numeric_xml(joint) for joint in root.findall(".//model/joint")
    ]
    return links, joints


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

    disabled_sdf = _to_sdf(disabled)
    enabled_sdf = _to_sdf(enabled)
    assert _sdf_mechanical_signature(enabled_sdf) == _sdf_mechanical_signature(
        disabled_sdf
    )

    contact_topics = set()
    for link in enabled_sdf.findall(".//model/link"):
        collision_names = {
            str(collision.get("name")) for collision in link.findall("collision")
        }
        for sensor in link.findall("sensor"):
            if sensor.get("type") != "contact":
                continue
            selector = sensor.findtext("./contact/collision")
            topic = sensor.findtext("./contact/topic")
            assert selector in collision_names, (
                f"{filename}: sensor {sensor.get('name')} selects missing "
                f"collision {selector} on {link.get('name')}"
            )
            assert selector != "__default__"
            assert topic and topic != "__default_topic__"
            contact_topics.add(topic)
    assert contact_topics == EXPECTED_CONTACT_TOPICS
