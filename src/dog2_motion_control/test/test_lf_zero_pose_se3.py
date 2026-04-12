import subprocess
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pinocchio as pin


EXPECTED_LF_ZERO_POSE = {
    "lf_coxa_link": {
        "translation": np.array([-0.140375, -0.115, 0.0199], dtype=float),
        "rotation": np.array(
            [
                [0.0, -1.0, 0.0],
                [0.0, 0.0, -1.0],
                [1.0, 0.0, 0.0],
            ],
            dtype=float,
        ),
    },
    "lf_femur_link": {
        "translation": np.array([-0.085375, -0.1424, -0.0034], dtype=float),
        "rotation": np.array(
            [
                [0.0, 0.0, 1.0],
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
            ],
            dtype=float,
        ),
    },
    "lf_tibia_link": {
        "translation": np.array([0.044595, -0.1424, -0.15541], dtype=float),
        "rotation": np.array(
            [
                [0.0, 0.0, 1.0],
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
            ],
            dtype=float,
        ),
    },
    "lf_foot_link": {
        "translation": np.array([-0.097573, -0.1184, -0.448589], dtype=float),
        "rotation": np.array(
            [
                [0.0, 0.0, 1.0],
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
            ],
            dtype=float,
        ),
    },
}

EXPECTED_LH_ZERO_POSE = {
    "lh_coxa_link": {
        "translation": np.array([0.138125, -0.115, 0.0199], dtype=float),
        "rotation": np.array(
            [
                [0.0, -1.0, 0.0],
                [0.0, 0.0, -1.0],
                [1.0, 0.0, 0.0],
            ],
            dtype=float,
        ),
    },
    "lh_femur_link": {
        "translation": np.array([0.193125, -0.1424, -0.0034], dtype=float),
        "rotation": np.array(
            [
                [0.0, 0.0, 1.0],
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
            ],
            dtype=float,
        ),
    },
    "lh_tibia_link": {
        "translation": np.array([0.323095, -0.1424, -0.15541], dtype=float),
        "rotation": np.array(
            [
                [0.0, 0.0, 1.0],
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
            ],
            dtype=float,
        ),
    },
    "lh_foot_link": {
        "translation": np.array([0.180927, -0.1184, -0.448589], dtype=float),
        "rotation": np.array(
            [
                [0.0, 0.0, 1.0],
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
            ],
            dtype=float,
        ),
    },
}

EXPECTED_RH_ZERO_POSE = {
    "rh_coxa_link": {
        "translation": np.array([0.138125, 0.115, 0.0199], dtype=float),
        "rotation": np.array(
            [
                [0.0, -1.0, 0.0],
                [0.0, 0.0, -1.0],
                [1.0, 0.0, 0.0],
            ],
            dtype=float,
        ),
    },
    "rh_femur_link": {
        "translation": np.array([0.193125, 0.1404, -0.0034], dtype=float),
        "rotation": np.array(
            [
                [0.0, 0.0, 1.0],
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
            ],
            dtype=float,
        ),
    },
    "rh_tibia_link": {
        "translation": np.array([0.323095, 0.1404, -0.15541], dtype=float),
        "rotation": np.array(
            [
                [0.0, 0.0, 1.0],
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
            ],
            dtype=float,
        ),
    },
    "rh_foot_link": {
        "translation": np.array([0.180927, 0.1164, -0.448589], dtype=float),
        "rotation": np.array(
            [
                [0.0, 0.0, 1.0],
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
            ],
            dtype=float,
        ),
    },
}

EXPECTED_RF_ZERO_POSE = {
    "rf_coxa_link": {
        "translation": np.array([-0.131475, 0.115, 0.0199], dtype=float),
        "rotation": np.array(
            [
                [0.0, -1.0, 0.0],
                [0.0, 0.0, -1.0],
                [1.0, 0.0, 0.0],
            ],
            dtype=float,
        ),
    },
    "rf_femur_link": {
        "translation": np.array([-0.076475, 0.1404, -0.0034], dtype=float),
        "rotation": np.array(
            [
                [0.0, 0.0, 1.0],
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
            ],
            dtype=float,
        ),
    },
    "rf_tibia_link": {
        "translation": np.array([0.053495, 0.1404, -0.15541], dtype=float),
        "rotation": np.array(
            [
                [0.0, 0.0, 1.0],
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
            ],
            dtype=float,
        ),
    },
    "rf_foot_link": {
        "translation": np.array([-0.088626, 0.1164, -0.448612], dtype=float),
        "rotation": np.array(
            [
                [0.0, 0.0, 1.0],
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
            ],
            dtype=float,
        ),
    },
}


def _workspace_root() -> Path:
    return Path(__file__).resolve().parents[3]


def _expand_dog2_xacro() -> Path:
    workspace_root = _workspace_root()
    xacro_path = workspace_root / "src" / "dog2_description" / "urdf" / "dog2.urdf.xacro"
    controllers_yaml = workspace_root / "src" / "dog2_description" / "config" / "ros2_controllers.yaml"
    with tempfile.NamedTemporaryFile(suffix=".urdf", delete=False) as tmp:
        urdf_path = Path(tmp.name)
    cmd = ["xacro", str(xacro_path), f"controllers_yaml:={controllers_yaml}", "-o", str(urdf_path)]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0:
        raise RuntimeError(
            "xacro expansion failed\n"
            f"cmd: {' '.join(cmd)}\n"
            f"stdout:\n{proc.stdout}\n"
            f"stderr:\n{proc.stderr}"
        )
    return urdf_path


def _assert_zero_pose_frames(expected_frames: dict[str, dict[str, np.ndarray]]) -> None:
    urdf_path = _expand_dog2_xacro()
    try:
        root = ET.parse(urdf_path).getroot()
        xml_text = ET.tostring(root, encoding="unicode")
        model = pin.buildModelFromXML(xml_text, pin.JointModelFreeFlyer())
        data = model.createData()

        q = np.zeros(model.nq, dtype=float)
        q[:7] = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0], dtype=float)
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)

        trunk_frame_id = int(model.getFrameId("base_link"))
        assert trunk_frame_id < int(model.nframes), "base_link frame not found"
        trunk_pose = data.oMf[trunk_frame_id]

        for frame_name, expected in expected_frames.items():
            frame_id = int(model.getFrameId(frame_name))
            assert frame_id < int(model.nframes), f"{frame_name} frame not found"
            trunk_to_frame = trunk_pose.inverse() * data.oMf[frame_id]
            assert np.allclose(
                np.asarray(trunk_to_frame.translation, dtype=float),
                expected["translation"],
                atol=1e-9,
            ), f"{frame_name} translation drifted"
            assert np.allclose(
                np.asarray(trunk_to_frame.rotation, dtype=float),
                expected["rotation"],
                atol=1e-9,
            ), f"{frame_name} rotation drifted"
    finally:
        urdf_path.unlink(missing_ok=True)


def test_lf_zero_pose_frames_match_regression() -> None:
    _assert_zero_pose_frames(EXPECTED_LF_ZERO_POSE)


def test_lh_zero_pose_frames_match_regression() -> None:
    _assert_zero_pose_frames(EXPECTED_LH_ZERO_POSE)


def test_rh_zero_pose_frames_match_regression() -> None:
    _assert_zero_pose_frames(EXPECTED_RH_ZERO_POSE)


def test_rf_zero_pose_frames_match_regression() -> None:
    _assert_zero_pose_frames(EXPECTED_RF_ZERO_POSE)
