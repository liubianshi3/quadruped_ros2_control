#!/usr/bin/env python3
"""Stage 2+/leg-mount acceptance: rail install topology + stand-pose world geometry vs golden.

Hard checks (exit non-zero on failure):
  1) Expanded URDF: lf/lh/rh/rf_rail_joint parent link is the matching *_leg_mount
  2–3) Pinocchio FK at canonical stand q: coxa_link and foot_link world positions match golden

Usage:
  python3 verify_stage2_rail_geometry.py [--xacro PATH] [--golden PATH]
  python3 verify_stage2_rail_geometry.py --write-golden [--xacro PATH] [--golden OUT_PATH]
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np

RAIL_JOINTS = ("lf_rail_joint", "lh_rail_joint", "rh_rail_joint", "rf_rail_joint")
LEG_ORDER = ("lf", "lh", "rh", "rf")

# Canonical stand (matches KinematicsSolver._standing_angles + rail=0; free-flyer matches offline tests)
Q_FREE_FLYER = [0.0, 0.0, 0.4, 0.0, 0.0, 0.0, 1.0]
STAND_COXA_FEMUR_TIBIA = (0.0, 0.3, -0.5)


def _default_paths(ws_src: Path | None) -> Tuple[Path, Path]:
    try:
        from ament_index_python.packages import get_package_share_directory

        share = Path(get_package_share_directory("dog2_description"))
        return share / "urdf" / "dog2.urdf.xacro", share / "config" / "migration_stage2_stand.json"
    except Exception:
        if ws_src is None:
            raise
    xacro = ws_src / "dog2_description" / "urdf" / "dog2.urdf.xacro"
    golden = ws_src / "dog2_description" / "config" / "migration_stage2_stand.json"
    return xacro, golden


def _resolve_ws_src() -> Path | None:
    """.../src when running from source tree; None if layout unknown (use ament_index)."""
    here = Path(__file__).resolve()
    parts = here.parts
    if "src" in parts:
        i = parts.index("src")
        return Path(*parts[: i + 1])
    return None


def run_xacro(xacro_path: Path, controllers_yaml: Path | None) -> Path:
    with tempfile.NamedTemporaryFile(suffix=".urdf", delete=False) as tmp:
        out = Path(tmp.name)
    cmd = ["xacro", str(xacro_path), "-o", str(out)]
    if controllers_yaml is not None and controllers_yaml.is_file():
        cmd.append(f"controllers_yaml:={controllers_yaml}")
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0:
        sys.stderr.write(proc.stderr or proc.stdout or "xacro failed\n")
        raise SystemExit(1)
    return out


EXPECTED_RAIL_PARENTS = {
    "lf_rail_joint": "lf_leg_mount",
    "lh_rail_joint": "lh_leg_mount",
    "rh_rail_joint": "rh_leg_mount",
    "rf_rail_joint": "rf_leg_mount",
}


def check_rail_parents(urdf_path: Path) -> None:
    root = ET.parse(urdf_path).getroot()
    for jn in RAIL_JOINTS:
        joint = root.find(f"./joint[@name='{jn}']")
        if joint is None:
            raise SystemExit(f"[FAIL] missing joint {jn}")
        parent = joint.find("parent")
        if parent is None:
            raise SystemExit(f"[FAIL] {jn} has no parent")
        pl = parent.attrib.get("link", "")
        expected = EXPECTED_RAIL_PARENTS[jn]
        if pl != expected:
            raise SystemExit(f"[FAIL] {jn} parent is '{pl}', expected {expected}")


def _controllers_yaml(ws_src: Path | None) -> Path | None:
    try:
        from ament_index_python.packages import get_package_share_directory

        p = Path(get_package_share_directory("dog2_motion_control")) / "config" / "effort_controllers.yaml"
        if p.is_file():
            return p
    except Exception:
        pass
    if ws_src is not None:
        p = ws_src / "dog2_motion_control" / "config" / "effort_controllers.yaml"
        if p.is_file():
            return p
        p2 = ws_src / "dog2_description" / "config" / "ros2_controllers.yaml"
        if p2.is_file():
            return p2
    return None


def _make_q_stand(model) -> np.ndarray:
    import pinocchio as pin

    nq = int(model.nq)
    q = np.zeros(nq, dtype=float)
    q[:7] = np.array(Q_FREE_FLYER, dtype=float)
    leg_js: Dict[str, Tuple[float, float, float, float]] = {
        leg: (0.0,) + STAND_COXA_FEMUR_TIBIA for leg in LEG_ORDER
    }
    for leg_id, (rail, coxa, femur, tibia) in leg_js.items():
        prefix = f"{leg_id}_"
        for joint_name, value in (
            (prefix + "rail_joint", rail),
            (prefix + "coxa_joint", coxa),
            (prefix + "femur_joint", femur),
            (prefix + "tibia_joint", tibia),
        ):
            jid = int(model.getJointId(joint_name))
            if jid == 0:
                raise SystemExit(f"joint not found: {joint_name}")
            joint = model.joints[jid]
            if isinstance(joint, pin.JointModelFreeFlyer):
                raise SystemExit(f"unexpected free flyer for {joint_name}")
            idx_q = int(joint.idx_q)
            if idx_q < 7 or idx_q >= nq:
                raise SystemExit(f"{joint_name} idx_q={idx_q} invalid for nq={nq}")
            q[idx_q] = float(value)
    return q


def compute_stand_world(model, data) -> Tuple[Dict[str, List[float]], Dict[str, List[float]]]:
    import pinocchio as pin

    q = _make_q_stand(model)
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    hip_w: Dict[str, List[float]] = {}
    foot_w: Dict[str, List[float]] = {}
    for leg in LEG_ORDER:
        hid = int(model.getFrameId(f"{leg}_coxa_link"))
        fid = int(model.getFrameId(f"{leg}_foot_link"))
        if hid >= model.nframes or fid >= model.nframes:
            raise SystemExit(f"missing frames for {leg}")
        hip_w[leg] = [float(x) for x in data.oMf[hid].translation]
        foot_w[leg] = [float(x) for x in data.oMf[fid].translation]
    return hip_w, foot_w


def write_golden(xacro_path: Path, golden_out: Path, controllers_yaml: Path | None) -> None:
    import pinocchio as pin

    urdf = run_xacro(xacro_path, controllers_yaml)
    try:
        root = ET.parse(urdf).getroot()
        xml_str = ET.tostring(root, encoding="unicode")
        model = pin.buildModelFromXML(xml_str, pin.JointModelFreeFlyer())
        data = model.createData()
        hip_w, foot_w = compute_stand_world(model, data)
    finally:
        urdf.unlink(missing_ok=True)

    payload = {
        "schema": "dog2_stage2_stand_v1",
        "description": "World positions (m) at canonical stand q; rail parent matches explicit leg_mount topology",
        "q_freeflyer": Q_FREE_FLYER,
        "leg_js": {leg: [0.0, STAND_COXA_FEMUR_TIBIA[0], STAND_COXA_FEMUR_TIBIA[1], STAND_COXA_FEMUR_TIBIA[2]] for leg in LEG_ORDER},
        "tol_m": 1.0e-5,
        "hip_world": hip_w,
        "foot_world": foot_w,
    }
    golden_out.parent.mkdir(parents=True, exist_ok=True)
    golden_out.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(f"[OK] wrote golden {golden_out}")


def verify(xacro_path: Path, golden_path: Path, controllers_yaml: Path | None) -> None:
    import pinocchio as pin

    if not golden_path.is_file():
        raise SystemExit(f"[FAIL] golden file missing: {golden_path}")

    golden = json.loads(golden_path.read_text(encoding="utf-8"))
    tol = float(golden.get("tol_m", 1e-5))

    urdf = run_xacro(xacro_path, controllers_yaml)
    try:
        check_rail_parents(urdf)
        print("[PASS] all *_rail_joint parents match explicit *_leg_mount topology")
        root = ET.parse(urdf).getroot()
        xml_str = ET.tostring(root, encoding="unicode")
        model = pin.buildModelFromXML(xml_str, pin.JointModelFreeFlyer())
        data = model.createData()
        hip_w, foot_w = compute_stand_world(model, data)
    finally:
        urdf.unlink(missing_ok=True)

    gh = golden["hip_world"]
    gf = golden["foot_world"]
    for leg in LEG_ORDER:
        hw = np.array(hip_w[leg], dtype=float)
        fw = np.array(foot_w[leg], dtype=float)
        he = np.linalg.norm(hw - np.array(gh[leg], dtype=float))
        fe = np.linalg.norm(fw - np.array(gf[leg], dtype=float))
        if he > tol or fe > tol:
            raise SystemExit(
                f"[FAIL] {leg} stand geometry mismatch hip_err={he:.3e} foot_err={fe:.3e} tol={tol}\n"
                f"  hip  got={hip_w[leg]} golden={gh[leg]}\n"
                f"  foot got={foot_w[leg]} golden={gf[leg]}"
            )
    print(f"[PASS] stand pose hip/foot world match golden (tol={tol:g} m)")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--xacro", type=Path, default=None, help="dog2.urdf.xacro path")
    ap.add_argument("--golden", type=Path, default=None, help="migration_stage2_stand.json path")
    ap.add_argument("--write-golden", action="store_true", help="Write golden JSON from current xacro")
    args = ap.parse_args()

    ws_src = _resolve_ws_src()
    dx, dg = _default_paths(ws_src)
    xacro_path = args.xacro or dx
    golden_path = args.golden or dg
    cy = _controllers_yaml(ws_src)

    if not xacro_path.is_file():
        print(f"[FAIL] xacro not found: {xacro_path}", file=sys.stderr)
        return 1

    if args.write_golden:
        write_golden(xacro_path, golden_path, cy)
        return 0

    verify(xacro_path, golden_path, cy)
    return 0


if __name__ == "__main__":
    sys.exit(main())
