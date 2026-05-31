#!/usr/bin/env python3
"""Stage 4 trot in place.

Rail joints remain locked by RailLockController.  No Raibert planner, no WBC,
no MPC.  Swing legs follow a conservative vertical swing trajectory; stance
legs keep their feet anchored in world frame.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import time
from typing import Any, Dict, Mapping, Optional, Sequence, Tuple
import xml.etree.ElementTree as ET

import yaml

try:
    import xacro
except Exception:  # pragma: no cover
    xacro = None

PROJECT_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = PROJECT_ROOT.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from controllers.gait_scheduler import GaitScheduler
from controllers.joint_pd_controller import JointPDController
from controllers.rail_lock_controller import RailLockController
from controllers.safety_checker import SafetyChecker
from controllers.stance_controller import StanceController
from controllers.swing_trajectory import SwingTrajectory
from kinematics.numerical_ik import NumericalIK
from utils.contact_filter import enable_foot_only_ground_contact, ground_contact_link_names


LEG_ORDER = ("FL", "FR", "RL", "RR")


def resolve_path(raw_path: str, base_dir: Path) -> Path:
    path = Path(raw_path).expanduser()
    if not path.is_absolute():
        for root in (base_dir, WORKSPACE_ROOT):
            candidate = (root / path).resolve()
            if candidate.exists():
                return candidate
    return path.resolve()


def load_yaml(path: Path) -> Dict[str, Any]:
    if not path.exists():
        return {}
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def local_package_map(workspace_root: Path) -> Dict[str, Path]:
    package_map: Dict[str, Path] = {}
    for package_xml in workspace_root.glob("src/*/package.xml"):
        try:
            package_name = (ET.parse(package_xml).getroot().findtext("name") or package_xml.parent.name).strip()
        except ET.ParseError:
            package_name = package_xml.parent.name
        package_map[package_name] = package_xml.parent.resolve()
    return package_map


def expand_model_to_urdf(model_path: Path, robot_config: Mapping[str, Any], config_dir: Path) -> str:
    if model_path.suffix.lower() == ".xacro":
        if xacro is None:
            raise RuntimeError("xacro is required to expand .xacro robot models.")
        raw_args = robot_config.get("robot", {}).get("xacro_args", {}) or {}
        mappings = {key: str(resolve_path(str(value), config_dir)) for key, value in raw_args.items()}
        return xacro.process_file(str(model_path), mappings=mappings).toprettyxml(indent="  ")
    return model_path.read_text(encoding="utf-8")


def replace_package_urls(urdf_text: str, package_map: Mapping[str, Path]) -> str:
    out = urdf_text
    for package_name, package_dir in package_map.items():
        out = out.replace(f"package://{package_name}", str(package_dir))
    return out


def pybullet_base_alignment_quat(p: Any, robot_config: Mapping[str, Any]) -> Tuple[float, float, float, float]:
    pybullet_cfg = ((robot_config.get("simulation") or {}).get("pybullet") or {})
    base_rpy = pybullet_cfg.get("base_orientation_rpy", [0.0, 0.0, 0.0])
    return tuple(p.getQuaternionFromEuler([float(v) for v in base_rpy]))


def semantic_base_pose(
    p: Any,
    robot_config: Mapping[str, Any],
    body_id: int,
) -> Tuple[Sequence[float], Sequence[float]]:
    base_pos, base_urdf_quat = p.getBasePositionAndOrientation(body_id)
    align_quat = pybullet_base_alignment_quat(p, robot_config)
    _, inv_align = p.invertTransform([0.0, 0.0, 0.0], align_quat)
    _, base_sem_quat = p.multiplyTransforms([0.0, 0.0, 0.0], base_urdf_quat, [0.0, 0.0, 0.0], inv_align)
    return base_pos, base_sem_quat


def semantic_base_rpy(p: Any, robot_config: Mapping[str, Any], body_id: int) -> Tuple[float, float, float]:
    _, base_sem_quat = semantic_base_pose(p, robot_config, body_id)
    return tuple(p.getEulerFromQuaternion(base_sem_quat))


def world_to_semantic_base(
    p: Any,
    robot_config: Mapping[str, Any],
    body_id: int,
    point_world: Sequence[float],
) -> Tuple[float, float, float]:
    base_pos, base_sem_quat = semantic_base_pose(p, robot_config, body_id)
    inv_pos, inv_quat = p.invertTransform(base_pos, base_sem_quat)
    pos, _ = p.multiplyTransforms(inv_pos, inv_quat, point_world, [0.0, 0.0, 0.0, 1.0])
    return tuple(float(v) for v in pos)


def semantic_base_to_world(
    p: Any,
    robot_config: Mapping[str, Any],
    body_id: int,
    point_base: Sequence[float],
) -> Tuple[float, float, float]:
    base_pos, base_sem_quat = semantic_base_pose(p, robot_config, body_id)
    pos, _ = p.multiplyTransforms(base_pos, base_sem_quat, point_base, [0.0, 0.0, 0.0, 1.0])
    return tuple(float(v) for v in pos)


def semantic_vector_to_world(
    p: Any,
    robot_config: Mapping[str, Any],
    body_id: int,
    vector_base: Sequence[float],
) -> Tuple[float, float, float]:
    _, base_sem_quat = semantic_base_pose(p, robot_config, body_id)
    vec_world, _ = p.multiplyTransforms(
        [0.0, 0.0, 0.0],
        base_sem_quat,
        vector_base,
        [0.0, 0.0, 0.0, 1.0],
    )
    return tuple(float(v) for v in vec_world)


def apply_debug_base_attitude_assist(
    p: Any,
    body_id: int,
    robot_config: Mapping[str, Any],
    controller_config: Mapping[str, Any],
) -> None:
    controller = controller_config.get("controller") or {}
    if not bool(controller.get("debug_base_attitude_assist", False)):
        return
    kp = float(controller.get("debug_base_attitude_kp", 4.0))
    kd = float(controller.get("debug_base_attitude_kd", 0.35))
    yaw_kp = float(controller.get("debug_base_yaw_kp", 0.0))
    yaw_kd = float(controller.get("debug_base_yaw_kd", 0.0))
    base_rpy = semantic_base_rpy(p, robot_config, body_id)
    _, base_ang_vel_world = p.getBaseVelocity(body_id)
    _, inv_sem_quat = p.invertTransform([0.0, 0.0, 0.0], semantic_base_pose(p, robot_config, body_id)[1])
    base_ang_vel_sem, _ = p.multiplyTransforms(
        [0.0, 0.0, 0.0],
        inv_sem_quat,
        base_ang_vel_world,
        [0.0, 0.0, 0.0, 1.0],
    )
    torque_sem = (
        -kp * float(base_rpy[0]) - kd * float(base_ang_vel_sem[0]),
        -kp * float(base_rpy[1]) - kd * float(base_ang_vel_sem[1]),
        -yaw_kp * float(base_rpy[2]) - yaw_kd * float(base_ang_vel_sem[2]),
    )
    torque_world = semantic_vector_to_world(p, robot_config, body_id, torque_sem)
    p.applyExternalTorque(body_id, -1, torque_world, p.WORLD_FRAME)


def leg_joint_names(robot_config: Mapping[str, Any]) -> Dict[str, Sequence[str]]:
    return {
        leg: [str(name) for name in ((robot_config.get("legs") or {}).get(leg, {}) or {}).get("joint_names", [])]
        for leg in LEG_ORDER
    }


def default_stand_targets(
    robot_config: Mapping[str, Any],
    pose_key: str = "default_stand_q",
) -> Dict[str, float]:
    targets: Dict[str, float] = {}
    for leg in LEG_ORDER:
        leg_cfg = ((robot_config.get("legs") or {}).get(leg, {}) or {})
        for index, joint_name in enumerate(leg_cfg.get("joint_names", []) or []):
            default_q = leg_cfg.get(pose_key, leg_cfg.get("default_stand_q", [])) or []
            targets[str(joint_name)] = float(default_q[index]) if index < len(default_q) else 0.0
    return targets


def configured_rail_positions(robot_config: Mapping[str, Any]) -> Dict[str, float]:
    rail_cfg = robot_config.get("rail") or {}
    joint_names = [str(name) for name in (rail_cfg.get("joint_names") or [])]
    single = str(rail_cfg.get("joint_name", "") or "").strip()
    if single and single not in joint_names:
        joint_names.append(single)
    source = str(rail_cfg.get("lock_position_source", "initial"))
    position_default = float(rail_cfg.get("lock_position", 0.0))
    position_map = {
        str(name): float(value)
        for name, value in (rail_cfg.get("lock_position_map", {}) or {}).items()
    }
    positions: Dict[str, float] = {}
    for joint_name in joint_names:
        positions[joint_name] = position_map.get(joint_name, position_default) if source == "configured" else 0.0
    return positions


def row_json(values: Mapping[str, Any]) -> str:
    return json.dumps(values, sort_keys=True, separators=(",", ":"))


def joint_dict_from_pybullet(p: Any, body_id: int, joint_name_to_index: Mapping[str, int]) -> Dict[str, Tuple[float, float]]:
    return {
        joint_name: tuple(float(v) for v in p.getJointState(body_id, joint_index)[:2])
        for joint_name, joint_index in joint_name_to_index.items()
    }


def pybullet_maps(p: Any, body_id: int) -> Tuple[Dict[str, int], Dict[str, int], Dict[str, Tuple[float, float]], Dict[str, float]]:
    joint_name_to_index: Dict[str, int] = {}
    link_name_to_index: Dict[str, int] = {}
    joint_limits: Dict[str, Tuple[float, float]] = {}
    torque_limits: Dict[str, float] = {}
    for joint_index in range(p.getNumJoints(body_id)):
        info = p.getJointInfo(body_id, joint_index)
        joint_name = info[1].decode("utf-8")
        link_name = info[12].decode("utf-8")
        joint_name_to_index[joint_name] = joint_index
        link_name_to_index[link_name] = joint_index
        if info[2] != p.JOINT_FIXED:
            lower = float(info[8])
            upper = float(info[9])
            joint_limits[joint_name] = (lower, upper) if lower <= upper else (None, None)
            torque_limits[joint_name] = float(info[10]) if info[10] > 0 else math.inf
    return joint_name_to_index, link_name_to_index, joint_limits, torque_limits


def reset_initial_pose(
    p: Any,
    body_id: int,
    joint_name_to_index: Mapping[str, int],
    link_name_to_index: Mapping[str, int],
    robot_config: Mapping[str, Any],
    base_height: float,
    pose_key: str = "default_stand_q",
) -> None:
    base_quat = pybullet_base_alignment_quat(p, robot_config)
    p.resetBasePositionAndOrientation(body_id, [0.0, 0.0, base_height], base_quat)
    p.resetBaseVelocity(body_id, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
    for rail_name, rail_q in configured_rail_positions(robot_config).items():
        if rail_name in joint_name_to_index:
            p.resetJointState(body_id, joint_name_to_index[rail_name], rail_q, 0.0)
    for joint_name, q in default_stand_targets(robot_config, pose_key=pose_key).items():
        if joint_name in joint_name_to_index:
            p.resetJointState(body_id, joint_name_to_index[joint_name], q, 0.0)

    foot_z = []
    for leg in LEG_ORDER:
        foot_link = ((robot_config.get("legs") or {}).get(leg, {}) or {}).get("foot_link", "")
        if foot_link in link_name_to_index:
            foot_z.append(p.getLinkState(body_id, link_name_to_index[str(foot_link)])[0][2])
    if foot_z:
        shift = 0.014 - min(foot_z)
        base_pos, base_quat = p.getBasePositionAndOrientation(body_id)
        p.resetBasePositionAndOrientation(body_id, [base_pos[0], base_pos[1], base_pos[2] + shift], base_quat)
        p.resetBaseVelocity(body_id, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])


def apply_zero_default_motors(p: Any, body_id: int) -> None:
    for joint_index in range(p.getNumJoints(body_id)):
        info = p.getJointInfo(body_id, joint_index)
        if info[2] != p.JOINT_FIXED:
            p.setJointMotorControl2(body_id, joint_index, p.VELOCITY_CONTROL, targetVelocity=0.0, force=0.0)


def actuator_mode(controller_config: Mapping[str, Any]) -> str:
    return str((controller_config.get("controller") or {}).get("actuator_mode", "torque_pd"))


def apply_leg_actuators(
    p: Any,
    body_id: int,
    joint_name_to_index: Mapping[str, int],
    pd_commands: Mapping[str, Any],
    joint_pd: JointPDController,
    controller_config: Mapping[str, Any],
) -> None:
    controller = controller_config.get("controller") or {}
    mode = actuator_mode(controller_config)
    if mode == "pybullet_position":
        position_gain = float(controller.get("pybullet_position_gain", 0.35))
        velocity_gain = float(controller.get("pybullet_velocity_gain", 0.08))
        for command in pd_commands.values():
            force_limit = joint_pd.joint_configs[command.joint_name].torque_limit
            p.setJointMotorControl2(
                body_id,
                joint_name_to_index[command.joint_name],
                p.POSITION_CONTROL,
                targetPosition=command.q_des,
                targetVelocity=command.dq_des,
                positionGain=position_gain,
                velocityGain=velocity_gain,
                force=force_limit,
            )
        return

    for command in pd_commands.values():
        p.setJointMotorControl2(
            body_id,
            joint_name_to_index[command.joint_name],
            p.TORQUE_CONTROL,
            force=command.tau,
        )


def apply_rail_actuators(
    p: Any,
    body_id: int,
    joint_name_to_index: Mapping[str, int],
    rail_commands: Mapping[str, Any],
    rail_controller: RailLockController,
    controller_config: Mapping[str, Any],
) -> None:
    controller = controller_config.get("controller") or {}
    mode = actuator_mode(controller_config)
    if mode == "pybullet_position":
        position_gain = float(controller.get("rail_position_gain", 0.8))
        velocity_gain = float(controller.get("rail_velocity_gain", 0.2))
        for command in rail_commands.values():
            p.setJointMotorControl2(
                body_id,
                joint_name_to_index[command.joint_name],
                p.POSITION_CONTROL,
                targetPosition=command.q0,
                targetVelocity=0.0,
                positionGain=position_gain,
                velocityGain=velocity_gain,
                force=rail_controller.config.max_force,
            )
        return

    for command in rail_commands.values():
        p.setJointMotorControl2(
            body_id,
            joint_name_to_index[command.joint_name],
            p.TORQUE_CONTROL,
            force=command.force,
        )


def disable_body_collision_and_visual(p: Any, body_id: int) -> None:
    for link_index in range(-1, p.getNumJoints(body_id)):
        p.setCollisionFilterGroupMask(body_id, link_index, 0, 0)
        try:
            p.changeVisualShape(body_id, link_index, rgbaColor=[1.0, 1.0, 1.0, 0.0])
        except Exception:
            pass


class PyBulletLegKinematics:
    """Fixed-base FK backend approximating semantic base-frame leg kinematics."""

    def __init__(
        self,
        p: Any,
        body_id: int,
        robot_config: Mapping[str, Any],
        joint_name_to_index: Mapping[str, int],
        link_name_to_index: Mapping[str, int],
    ):
        self.p = p
        self.body_id = body_id
        self.robot_config = robot_config
        self.joint_name_to_index = dict(joint_name_to_index)
        self.link_name_to_index = dict(link_name_to_index)

    def set_pose(self, joint_targets: Mapping[str, float]) -> None:
        self.p.resetBasePositionAndOrientation(
            self.body_id,
            [0.0, 0.0, 0.0],
            pybullet_base_alignment_quat(self.p, self.robot_config),
        )
        for rail_name, rail_q in configured_rail_positions(self.robot_config).items():
            if rail_name in self.joint_name_to_index:
                self.p.resetJointState(self.body_id, self.joint_name_to_index[rail_name], rail_q, 0.0)
        for joint_name, q in joint_targets.items():
            if joint_name in self.joint_name_to_index:
                self.p.resetJointState(self.body_id, self.joint_name_to_index[joint_name], float(q), 0.0)

    def foot_position(self, leg_name: str, q_model: Sequence[float]) -> Tuple[float, float, float]:
        leg_cfg = ((self.robot_config.get("legs") or {}).get(leg_name, {}) or {})
        joint_names = [str(name) for name in leg_cfg.get("joint_names", [])]
        if len(joint_names) != 3:
            raise ValueError(f"{leg_name} must have three configured joints.")
        for joint_name, q in zip(joint_names, q_model):
            self.p.resetJointState(self.body_id, self.joint_name_to_index[joint_name], float(q), 0.0)
        foot_link = str(leg_cfg.get("foot_link", ""))
        pos = self.p.getLinkState(self.body_id, self.link_name_to_index[foot_link])[0]
        return tuple(float(v) for v in pos)


def add_marker(
    p: Any,
    pos: Sequence[float],
    color: Sequence[float],
    label: str = "",
    size: float = 0.035,
    life_time: float = 0.3,
) -> None:
    x, y, z = pos
    p.addUserDebugLine([x - size, y, z], [x + size, y, z], color, lineWidth=4, lifeTime=life_time)
    p.addUserDebugLine([x, y - size, z], [x, y + size, z], color, lineWidth=4, lifeTime=life_time)
    p.addUserDebugLine([x, y, z - size], [x, y, z + size], color, lineWidth=4, lifeTime=life_time)
    if label:
        p.addUserDebugText(label, [x, y, z + 0.04], textColorRGB=color[:3], textSize=1.2, lifeTime=life_time)


def apply_leg_visuals(
    p: Any,
    body_id: int,
    link_name_to_index: Mapping[str, int],
    gait_states: Mapping[str, Mapping[str, Any]],
) -> None:
    prefix_map = {"FL": "lf", "FR": "rf", "RL": "lh", "RR": "rh"}
    for leg_name, prefix in prefix_map.items():
        state = str(gait_states[leg_name]["state"])
        if state == "swing":
            rgba = [1.0, 0.72, 0.10, 1.0]
        else:
            rgba = [0.35, 0.55, 1.0, 1.0]
        for link_name, link_index in link_name_to_index.items():
            if link_name.startswith(prefix + "_"):
                try:
                    p.changeVisualShape(body_id, link_index, rgbaColor=rgba)
                except Exception:
                    pass


def recover_to_default_stand_posture(
    p: Any,
    body_id: int,
    joint_name_to_index: Mapping[str, int],
    link_name_to_index: Mapping[str, int],
    robot_config: Mapping[str, Any],
    controller_config: Mapping[str, Any],
    rail_controller: RailLockController,
) -> None:
    default_q = default_stand_targets(robot_config, pose_key="default_stand_q")
    base_height = float((controller_config.get("controller") or {}).get("initial_base_height", 0.42))
    reset_initial_pose(
        p,
        body_id,
        joint_name_to_index,
        link_name_to_index,
        robot_config,
        base_height,
        pose_key="default_stand_q",
    )
    p.resetBaseVelocity(body_id, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
    for _ in range(300):
        for joint_name, q_des in default_q.items():
            p.setJointMotorControl2(
                body_id,
                joint_name_to_index[joint_name],
                p.POSITION_CONTROL,
                targetPosition=q_des,
                targetVelocity=0.0,
                positionGain=0.35,
                velocityGain=0.08,
                force=80.0,
            )
        for rail_name, rail_q in rail_controller.lock_positions.items():
            p.setJointMotorControl2(
                body_id,
                joint_name_to_index[rail_name],
                p.POSITION_CONTROL,
                targetPosition=rail_q,
                targetVelocity=0.0,
                positionGain=0.8,
                velocityGain=0.2,
                force=rail_controller.config.max_force,
            )
        p.stepSimulation()


def preview_default_stand_pose(
    p: Any,
    body_id: int,
    joint_name_to_index: Mapping[str, int],
    link_name_to_index: Mapping[str, int],
    robot_config: Mapping[str, Any],
    controller_config: Mapping[str, Any],
    rail_controller: RailLockController,
    duration_s: float,
) -> None:
    if duration_s <= 0.0:
        return
    dt = float((controller_config.get("controller") or {}).get("control_dt", 0.002))
    default_q = default_stand_targets(robot_config, pose_key="default_stand_q")
    base_height = float((controller_config.get("controller") or {}).get("initial_base_height", 0.42))
    reset_initial_pose(
        p,
        body_id,
        joint_name_to_index,
        link_name_to_index,
        robot_config,
        base_height,
        pose_key="default_stand_q",
    )
    preview_steps = max(1, int(duration_s / max(dt, 1e-6)))
    for _ in range(preview_steps):
        for joint_name, q_des in default_q.items():
            p.setJointMotorControl2(
                body_id,
                joint_name_to_index[joint_name],
                p.POSITION_CONTROL,
                targetPosition=q_des,
                targetVelocity=0.0,
                positionGain=0.35,
                velocityGain=0.08,
                force=80.0,
            )
        for rail_name, rail_q in rail_controller.lock_positions.items():
            p.setJointMotorControl2(
                body_id,
                joint_name_to_index[rail_name],
                p.POSITION_CONTROL,
                targetPosition=rail_q,
                targetVelocity=0.0,
                positionGain=0.8,
                velocityGain=0.2,
                force=rail_controller.config.max_force,
            )
        p.stepSimulation()
        time.sleep(dt)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=str(PROJECT_ROOT / "config" / "robot_config.yaml"))
    parser.add_argument("--rail-config", default=str(PROJECT_ROOT / "config" / "rail_config.yaml"))
    parser.add_argument("--controller-config", default=str(PROJECT_ROOT / "config" / "controller_config.yaml"))
    parser.add_argument("--gait-config", default=str(PROJECT_ROOT / "config" / "gait_config.yaml"))
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument(
        "--demo-step-height",
        type=float,
        default=None,
        help="Override swing step height for the fixed-base GUI demo.",
    )
    parser.add_argument(
        "--dynamic",
        action="store_true",
        help="Run free-base dynamics. Default is fixed-base gait visualization and target verification.",
    )
    parser.add_argument(
        "--no-hold-on-stop",
        action="store_true",
        help="Do not keep the PyBullet GUI open after a safety stop.",
    )
    parser.add_argument(
        "--initial-visible-hold",
        type=float,
        default=1.2,
        help="GUI dynamic mode: hold the more natural default stand pose before switching to the locomotion pose.",
    )
    args = parser.parse_args()

    try:
        import pybullet as p
        import pybullet_data
    except Exception as exc:
        print(f"[ERROR] pybullet is required for trot_in_place: {exc}")
        return 2

    robot_config = load_yaml(Path(args.config).resolve())
    rail_config = load_yaml(Path(args.rail_config).resolve())
    controller_config = load_yaml(Path(args.controller_config).resolve())
    gait_config = load_yaml(Path(args.gait_config).resolve())
    config_dir = Path(args.config).resolve().parent

    dt = float((controller_config.get("controller") or {}).get("control_dt", 0.002))
    base_height = float((controller_config.get("controller") or {}).get("initial_base_height", 0.42))
    swing_cfg = gait_config.get("swing_trajectory", {}) or {}
    configured_step_height = float(swing_cfg.get("step_height", 0.03))
    max_step_height = float(swing_cfg.get("max_step_height", 0.06))
    if args.demo_step_height is not None:
        visual_step_height = min(max(float(args.demo_step_height), 0.0), max_step_height)
    elif args.dynamic:
        visual_step_height = configured_step_height
    else:
        visual_step_height = min(max(configured_step_height, 0.05), max_step_height)

    model_path = resolve_path(robot_config["robot"]["model_path"], config_dir)
    urdf_text = expand_model_to_urdf(model_path, robot_config, config_dir)
    urdf_text = replace_package_urls(urdf_text, local_package_map(WORKSPACE_ROOT))
    with tempfile.NamedTemporaryFile("w", suffix=".urdf", delete=False, encoding="utf-8") as f:
        f.write(urdf_text)
        temp_urdf = f.name

    logs_dir = PROJECT_ROOT / "logs"
    logs_dir.mkdir(parents=True, exist_ok=True)
    log_path = logs_dir / "trot_in_place_log.csv"

    physics_client = p.connect(p.GUI if args.gui else p.DIRECT)
    try:
        if args.gui:
            p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
            p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
            p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0.0, 0.0, -9.81)
        p.setTimeStep(dt)
        p.setPhysicsEngineParameter(numSolverIterations=120, numSubSteps=2)
        plane_id = p.loadURDF("plane.urdf")
        body_id = p.loadURDF(temp_urdf, useFixedBase=not args.dynamic, flags=p.URDF_USE_INERTIA_FROM_FILE)
        kin_body_id = p.loadURDF(temp_urdf, useFixedBase=True, flags=p.URDF_USE_INERTIA_FROM_FILE)
        disable_body_collision_and_visual(p, kin_body_id)

        joint_name_to_index, link_name_to_index, joint_limits, _ = pybullet_maps(p, body_id)
        kin_joint_name_to_index, kin_link_name_to_index, _, _ = pybullet_maps(p, kin_body_id)
        enable_foot_only_ground_contact(
            p,
            body_id,
            plane_id,
            link_name_to_index,
            robot_config,
            keep_base_collision=True,
        )
        rail_controller = RailLockController.from_config_dicts(robot_config, rail_config)
        joint_pd = JointPDController.from_config_dicts(
            robot_config,
            controller_config,
            excluded_joints=rail_controller.rail_joint_names,
        )
        rail_controller.validate_leg_joint_exclusion(leg_joint_names(robot_config))
        gait_scheduler = GaitScheduler.from_config_dict(gait_config)
        swing_trajectory = SwingTrajectory.from_config_dict(gait_config)
        stance_controller = StanceController.from_config_dict(gait_config)
        safety_checker = SafetyChecker.from_config_dict(controller_config)

        visible_default_q = default_stand_targets(robot_config, pose_key="default_stand_q")
        locomotion_q = default_stand_targets(robot_config, pose_key="locomotion_stand_q")
        if args.dynamic and args.gui:
            preview_default_stand_pose(
                p,
                body_id,
                joint_name_to_index,
                link_name_to_index,
                robot_config,
                controller_config,
                rail_controller,
                duration_s=float(max(args.initial_visible_hold, 0.0)),
            )
        reset_initial_pose(
            p,
            body_id,
            joint_name_to_index,
            link_name_to_index,
            robot_config,
            base_height,
            pose_key="locomotion_stand_q" if args.dynamic else "default_stand_q",
        )
        apply_zero_default_motors(p, body_id)
        joint_state = joint_dict_from_pybullet(p, body_id, joint_name_to_index)
        rail_controller.initialize({name: joint_state[name] for name in rail_controller.rail_joint_names})
        joint_pd.reset_targets(joint_state)

        kin = PyBulletLegKinematics(p, kin_body_id, robot_config, kin_joint_name_to_index, kin_link_name_to_index)
        kin.set_pose(locomotion_q)
        ik = NumericalIK.from_robot_config(robot_config, kin.foot_position, tolerance=0.01)

        if args.gui:
            p.resetDebugVisualizerCamera(
                cameraDistance=1.05,
                cameraYaw=42.0,
                cameraPitch=-23.0,
                cameraTargetPosition=[0.0, 0.0, 0.18],
            )

        nominal_foot_pos_base: Dict[str, Tuple[float, float, float]] = {}
        for leg_name in LEG_ORDER:
            foot_link = str(((robot_config.get("legs") or {}).get(leg_name, {}) or {}).get("foot_link", ""))
            nominal_foot_pos_base[leg_name] = world_to_semantic_base(
                p,
                robot_config,
                kin_body_id,
                p.getLinkState(kin_body_id, kin_link_name_to_index[foot_link])[0],
            )

        swing_memory: Dict[str, Dict[str, Optional[Tuple[float, float, float]]]] = {
            leg_name: {"start_world": None, "target_world": None, "prev_des_world": None}
            for leg_name in LEG_ORDER
        }
        last_state = {leg_name: "stance" for leg_name in LEG_ORDER}
        last_good_q = {
            leg: tuple(locomotion_q[joint_name] for joint_name in leg_joint_names(robot_config)[leg])
            for leg in LEG_ORDER
        }
        pd_torque_limits = {name: cfg.torque_limit for name, cfg in joint_pd.joint_configs.items()}

        headers = [
            "time",
            "base_position",
            "base_rpy",
            "rail_error",
            "gait_states",
            "foot_target_world",
            "foot_actual_world",
            "ik_error",
            "safety_status",
        ]

        success = True
        debug_draw_interval = max(1, int(0.10 / dt))
        with log_path.open("w", newline="", encoding="utf-8") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=headers)
            writer.writeheader()

            steps = int(float(args.duration) / dt)
            for step in range(steps):
                sim_time = step * dt
                joint_state = joint_dict_from_pybullet(p, body_id, joint_name_to_index)
                gait_states = gait_scheduler.update(sim_time)
                blend = gait_scheduler.transition_blend(sim_time)
                in_initial_stand = sim_time < gait_scheduler.config.initial_stand_time

                foot_actual_world: Dict[str, Tuple[float, float, float]] = {}
                foot_target_world: Dict[str, Tuple[float, float, float]] = {}
                ik_success: Dict[str, bool] = {}
                ik_error: Dict[str, float] = {}
                foot_target_jumps: Dict[str, float] = {}
                stand_progress = 1.0 if gait_scheduler.config.initial_stand_time <= 1e-9 else min(
                    max(sim_time / gait_scheduler.config.initial_stand_time, 0.0),
                    1.0,
                )
                stand_q = {
                    joint_name: (1.0 - stand_progress) * visible_default_q[joint_name]
                    + stand_progress * locomotion_q[joint_name]
                    for joint_name in visible_default_q
                }
                q_des_all = dict(locomotion_q if args.dynamic else (stand_q if in_initial_stand else locomotion_q))

                for leg_name in LEG_ORDER:
                    foot_link = str(((robot_config.get("legs") or {}).get(leg_name, {}) or {}).get("foot_link", ""))
                    foot_actual_world[leg_name] = tuple(
                        float(v) for v in p.getLinkState(body_id, link_name_to_index[foot_link])[0]
                    )

                for leg_name in LEG_ORDER:
                    state = gait_states[leg_name]["state"]
                    swing_phase = float(gait_states[leg_name]["swing_phase"])
                    actual_world = foot_actual_world[leg_name]
                    nominal_world = semantic_base_to_world(p, robot_config, body_id, nominal_foot_pos_base[leg_name])
                    previous_target = swing_memory[leg_name]["prev_des_world"]

                    if in_initial_stand:
                        foot_target_world[leg_name] = actual_world
                        foot_target_jumps[leg_name] = 0.0 if previous_target is None else math.dist(actual_world, previous_target)
                        swing_memory[leg_name]["start_world"] = None
                        swing_memory[leg_name]["target_world"] = None
                        swing_memory[leg_name]["prev_des_world"] = actual_world
                        ik_success[leg_name] = True
                        ik_error[leg_name] = 0.0
                        continue

                    if state == "stance":
                        stance_target = stance_controller.update_leg(leg_name, state, actual_world)
                        foot_target_world[leg_name] = stance_target.target_world
                        swing_memory[leg_name]["start_world"] = None
                        swing_memory[leg_name]["target_world"] = None
                    else:
                        stance_controller.update_leg(leg_name, state, actual_world)
                        entering_swing = last_state[leg_name] != "swing"
                        if entering_swing or swing_memory[leg_name]["start_world"] is None:
                            swing_memory[leg_name]["start_world"] = actual_world
                            target_world = tuple(
                                (1.0 - blend) * actual_world[i] + blend * nominal_world[i]
                                for i in range(3)
                            )
                            swing_memory[leg_name]["target_world"] = target_world
                            swing_memory[leg_name]["prev_des_world"] = actual_world
                        sample = swing_trajectory.evaluate(
                            swing_memory[leg_name]["start_world"],
                            swing_memory[leg_name]["target_world"],
                            swing_phase,
                            step_height=visual_step_height * blend,
                            swing_duration=gait_scheduler.swing_time,
                            previous_desired_foot_pos_world=swing_memory[leg_name]["prev_des_world"],
                        )
                        foot_target_world[leg_name] = sample.position_world

                    foot_target_jumps[leg_name] = 0.0 if previous_target is None else math.dist(
                        foot_target_world[leg_name], previous_target
                    )
                    swing_memory[leg_name]["prev_des_world"] = foot_target_world[leg_name]

                    target_base = world_to_semantic_base(p, robot_config, body_id, foot_target_world[leg_name])
                    q_current_leg = tuple(joint_state[joint_name][0] for joint_name in leg_joint_names(robot_config)[leg_name])
                    result = ik.solve(
                        leg_name,
                        target_base,
                        last_good_q[leg_name],
                        max_iters=30,
                        damping=1e-3,
                        step_limit=0.05,
                    )
                    ik_success[leg_name] = bool(result.success or result.error_norm <= 0.02)
                    ik_error[leg_name] = float(result.error_norm)
                    if ik_success[leg_name]:
                        last_good_q[leg_name] = result.q
                    for joint_name, q in zip(leg_joint_names(robot_config)[leg_name], last_good_q[leg_name]):
                        q_des_all[joint_name] = q

                if args.dynamic:
                    pd_commands = joint_pd.compute(joint_state, q_des_all)
                    torques = {name: command.tau for name, command in pd_commands.items()}

                    rail_commands = rail_controller.compute(
                        {name: joint_state[name] for name in rail_controller.rail_joint_names}
                    )
                    apply_leg_actuators(p, body_id, joint_name_to_index, pd_commands, joint_pd, controller_config)
                    apply_rail_actuators(p, body_id, joint_name_to_index, rail_commands, rail_controller, controller_config)
                    apply_debug_base_attitude_assist(p, body_id, robot_config, controller_config)
                else:
                    for joint_name, q_des in q_des_all.items():
                        p.resetJointState(body_id, joint_name_to_index[joint_name], float(q_des), 0.0)
                    for rail_name in rail_controller.rail_joint_names:
                        p.resetJointState(
                            body_id,
                            joint_name_to_index[rail_name],
                            float(rail_controller.lock_positions[rail_name]),
                            0.0,
                        )
                    joint_state = joint_dict_from_pybullet(p, body_id, joint_name_to_index)
                    rail_commands = rail_controller.compute(
                        {name: joint_state[name] for name in rail_controller.rail_joint_names}
                    )
                    torques = {name: 0.0 for name in joint_pd.joint_names}

                p.stepSimulation()
                if args.gui and bool((controller_config.get("controller") or {}).get("gui_sleep", True)):
                    time.sleep(dt)

                base_pos, _ = p.getBasePositionAndOrientation(body_id)
                base_rpy = semantic_base_rpy(p, robot_config, body_id)
                safety = safety_checker.check(
                    base_position=base_pos,
                    base_rpy=base_rpy,
                    joint_state={
                        name: joint_state[name]
                        for name in set(joint_pd.joint_names) | set(rail_controller.rail_joint_names)
                    },
                    joint_limits=joint_limits,
                    torques=torques,
                    torque_limits=pd_torque_limits,
                    rail_errors={name: command.error for name, command in rail_commands.items()},
                    gait_states={leg_name: gait_states[leg_name]["state"] for leg_name in LEG_ORDER},
                    ik_success=ik_success,
                    foot_target_jumps=foot_target_jumps,
                )
                safety_status = "OK" if safety.ok else "STOP"
                if not safety.ok:
                    print(f"[SAFETY STOP] t={sim_time:.3f}: {'; '.join(safety.reasons)}")
                    success = False

                if args.gui and step % debug_draw_interval == 0:
                    p.removeAllUserDebugItems()
                    apply_leg_visuals(p, body_id, link_name_to_index, gait_states)
                    p.addUserDebugText(
                        f"Trot in place ({'dynamic' if args.dynamic else 'fixed-base demo'}) | t={sim_time:.2f}s | blend={blend:.2f}",
                        [0.18, -0.18, 0.34],
                        textColorRGB=[1.0, 1.0, 1.0],
                        textSize=1.4,
                        lifeTime=0.0,
                    )
                    p.addUserDebugText(
                        f"Blue leg = stance, Orange leg = swing | step_height={visual_step_height * 100.0:.1f} cm",
                        [0.18, -0.18, 0.30],
                        textColorRGB=[0.9, 0.9, 0.9],
                        textSize=1.1,
                        lifeTime=0.0,
                    )
                    if sim_time < gait_scheduler.config.initial_stand_time:
                        p.addUserDebugText(
                            f"Initial stand + posture blend: {gait_scheduler.config.initial_stand_time - sim_time:.1f}s remaining",
                            [0.18, -0.18, 0.26],
                            textColorRGB=[1.0, 0.9, 0.4],
                            textSize=1.05,
                            lifeTime=0.0,
                        )
                    else:
                        p.addUserDebugText(
                            "Green cross = actual foot | Blue/Orange cross = current foot target",
                            [0.18, -0.18, 0.26],
                            textColorRGB=[0.9, 1.0, 0.9],
                            textSize=1.05,
                            lifeTime=0.0,
                        )
                    if args.dynamic:
                        contact_names = sorted(ground_contact_link_names(p, body_id, plane_id))
                        p.addUserDebugText(
                            "Ground contacts: " + ", ".join(contact_names),
                            [0.18, -0.18, 0.22],
                            textColorRGB=[0.65, 1.0, 1.0],
                            textSize=1.0,
                            lifeTime=0.0,
                        )
                    for leg_name in LEG_ORDER:
                        state = gait_states[leg_name]["state"]
                        actual = foot_actual_world[leg_name]
                        target = foot_target_world[leg_name]
                        anchor = stance_controller.get_anchor_world(leg_name)
                        target_color = [0.1, 0.4, 1.0] if state == "stance" else [1.0, 0.55, 0.1]
                        add_marker(p, actual, [0.1, 1.0, 0.1], f"{leg_name} actual", size=0.022, life_time=0.35)
                        add_marker(p, target, target_color, f"{leg_name} {state}", size=0.028, life_time=0.35)
                        p.addUserDebugLine(actual, target, [1.0, 0.0, 0.0], lineWidth=3, lifeTime=0.35)
                        if anchor is not None:
                            add_marker(p, anchor, [0.5, 0.2, 1.0], f"{leg_name} anchor", size=0.018, life_time=0.35)

                if step % 10 == 0:
                    writer.writerow(
                        {
                            "time": f"{sim_time:.6f}",
                            "base_position": row_json({"x": base_pos[0], "y": base_pos[1], "z": base_pos[2]}),
                            "base_rpy": row_json({"r": base_rpy[0], "p": base_rpy[1], "y": base_rpy[2]}),
                            "rail_error": row_json({name: command.error for name, command in rail_commands.items()}),
                            "gait_states": row_json({leg: gait_states[leg]["state"] for leg in LEG_ORDER}),
                            "foot_target_world": row_json(
                                {leg: {"x": foot_target_world[leg][0], "y": foot_target_world[leg][1], "z": foot_target_world[leg][2]} for leg in LEG_ORDER}
                            ),
                            "foot_actual_world": row_json(
                                {leg: {"x": foot_actual_world[leg][0], "y": foot_actual_world[leg][1], "z": foot_actual_world[leg][2]} for leg in LEG_ORDER}
                            ),
                            "ik_error": row_json(ik_error),
                            "safety_status": safety_status,
                        }
                    )

                last_state = {leg_name: gait_states[leg_name]["state"] for leg_name in LEG_ORDER}
                if not safety.ok:
                    break

        print(f"[INFO] Trot-in-place log: {log_path}")
        print("[PASS] Trot in place completed." if success else "[FAIL] Trot in place stopped on safety.")
        if args.gui and not success and not args.no_hold_on_stop:
            recover_to_default_stand_posture(
                p,
                body_id,
                joint_name_to_index,
                link_name_to_index,
                robot_config,
                controller_config,
                rail_controller,
            )
            print("[INFO] GUI hold after safety stop. Press Ctrl+C in this terminal to close.")
            try:
                while p.isConnected(physics_client):
                    contact_names = sorted(ground_contact_link_names(p, body_id, plane_id))
                    p.addUserDebugText(
                        "SAFETY STOP - inspect pose, then press Ctrl+C",
                        [0.18, -0.18, 0.38],
                        textColorRGB=[1.0, 0.1, 0.1],
                        textSize=1.25,
                        lifeTime=0.3,
                    )
                    p.addUserDebugText(
                        "Ground contacts: " + (", ".join(contact_names) if contact_names else "none"),
                        [0.18, -0.18, 0.34],
                        textColorRGB=[0.65, 1.0, 1.0],
                        textSize=1.0,
                        lifeTime=0.3,
                    )
                    time.sleep(0.25)
            except KeyboardInterrupt:
                pass
        return 0 if success else 1
    finally:
        p.disconnect(physics_client)
        try:
            os.unlink(temp_urdf)
        except OSError:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
