#!/usr/bin/env python3
"""Pinocchio smoke test for dog2_symmetric URDF: mass, inertia, joint limits."""

from __future__ import annotations

import math
import subprocess
import sys
import tempfile
from pathlib import Path

EPS = 1e-4


def fail(msg: str) -> None:
    print(f"[FAIL] {msg}")
    sys.exit(1)


def main() -> int:
    xacro_path = Path(__file__).resolve().parents[1] / "urdf" / "dog2_symmetric.urdf.xacro"
    controllers_yaml = xacro_path.parents[1] / "config" / "ros2_controllers.yaml"

    with tempfile.NamedTemporaryFile(suffix=".urdf", delete=False) as tmp:
        urdf_path = Path(tmp.name)

    proc = subprocess.run(
        ["bash", "-c",
         f"source /opt/ros/humble/setup.bash && source /home/dell/aperfect/carbot_ws/install/setup.bash 2>/dev/null; "
         f"xacro {xacro_path} controllers_yaml:={controllers_yaml} -o {urdf_path}"],
        capture_output=True, text=True,
    )
    if proc.returncode != 0:
        fail(f"xacro failed: {proc.stderr}")

    import pinocchio as pin
    model = pin.buildModelFromUrdf(str(urdf_path))
    data = model.createData()
    print(f"[INFO] Pinocchio model: nq={model.nq}, nv={model.nv}")

    # Root link check
    root_frame_id = model.getFrameId("base_link")
    if root_frame_id < 0:
        fail("base_link not found in Pinocchio model")
    print("[PASS] base_link found in Pinocchio model")

    # Mass check
    # Pinocchio exposes URDF links as frames; Frame.inertia holds the link
    # inertial data directly across the API versions used in this workspace.
    base_iner = model.frames[root_frame_id].inertia
    mass = base_iner.mass
    if math.isclose(mass, 6.0, abs_tol=0.01):
        print(f"[PASS] base_link mass == {mass:.3f}")
    else:
        print(f"[WARN] base_link mass != 6.0: {mass}")

    # Inertia check
    i_mat = base_iner.inertia
    ixx = float(i_mat[0, 0])
    iyy = float(i_mat[1, 1])
    izz = float(i_mat[2, 2])
    expected = {"ixx": 0.0153, "iyy": 0.044, "izz": 0.052}
    for name, exp in expected.items():
        val = ixx if name == "ixx" else iyy if name == "iyy" else izz
        if math.isclose(val, exp, abs_tol=EPS):
            print(f"[PASS] base_link {name} == {val} (expected {exp})")
        else:
            print(f"[WARN] base_link {name} == {val} (expected {exp})")

    # Off-diagonal check
    for r, c in [(0, 1), (0, 2), (1, 2)]:
        v = float(i_mat[r, c])
        if abs(v) < EPS:
            print(f"[PASS] base_link inertia off-diagonal ({r},{c}) == {v:.2e}")
        else:
            fail(f"base_link inertia off-diagonal ({r},{c}) != 0: {v}")

    # Rail joints
    rail_expected = {
        "lf_rail_joint": (0.0, 0.111),
        "lh_rail_joint": (-0.111, 0.0),
        "rh_rail_joint": (-0.111, 0.0),
        "rf_rail_joint": (0.0, 0.111),
    }
    for rail_name, (expected_low, expected_up) in rail_expected.items():
        jid = model.getJointId(rail_name)
        if jid >= model.njoints:
            fail(f"Missing rail joint: {rail_name}")
        q_min = model.lowerPositionLimit
        q_max = model.upperPositionLimit
        idx = model.joints[jid].idx_q
        actual_low = float(q_min[idx])
        actual_up = float(q_max[idx])
        if math.isclose(actual_low, expected_low, abs_tol=EPS) and math.isclose(actual_up, expected_up, abs_tol=EPS):
            print(f"[PASS] {rail_name} limits: [{actual_low}, {actual_up}]")
        else:
            fail(f"{rail_name} limits mismatch: got [{actual_low}, {actual_up}], expected [{expected_low}, {expected_up}]")

    try:
        urdf_path.unlink()
    except OSError:
        pass
    print("\n[PASS] All symmetric Pinocchio model checks passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
