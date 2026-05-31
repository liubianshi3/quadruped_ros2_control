#!/usr/bin/env python3
"""Stage 5 low-speed quadruped walking with Raibert foot placement.

Default mode is a fixed-base visualization/verification preview.  Use
--dynamic for free-base PyBullet dynamics after stand/trot-in-place dynamics
are stable enough.  Rail joints are locked independently and never enter
Raibert, IK, gait scheduling, or leg PD.
"""

from __future__ import annotations

import argparse
import csv
import importlib.util
import math
import os
from pathlib import Path
import sys
import tempfile
import time
from typing import Any, Dict, Mapping, Optional, Sequence, Tuple


PROJECT_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = PROJECT_ROOT.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from controllers.gait_scheduler import GaitScheduler
from controllers.joint_pd_controller import JointPDController
from controllers.raibert_planner import RaibertPlanner
from controllers.rail_lock_controller import RailLockController
from controllers.safety_checker import SafetyChecker
from controllers.stance_controller import StanceController
from controllers.swing_trajectory import SwingTrajectory
from kinematics.numerical_ik import NumericalIK
from utils.contact_filter import enable_foot_only_ground_contact, ground_contact_link_names


LEG_ORDER = ("FL", "FR", "RL", "RR")


def load_stage4_helpers() -> Any:
    stage4_path = PROJECT_ROOT / "scripts" / "04_trot_in_place.py"
    spec = importlib.util.spec_from_file_location("stage4_trot_in_place_helpers", stage4_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Cannot load helpers from {stage4_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


H = load_stage4_helpers()


def body_velocity_from_world(
    p: Any,
    robot_config: Mapping[str, Any],
    body_id: int,
    linear_velocity_world: Sequence[float],
    angular_velocity_world: Sequence[float],
) -> Tuple[Tuple[float, float, float], float]:
    _, base_sem_quat = H.semantic_base_pose(p, robot_config, body_id)
    _, inv_quat = p.invertTransform([0.0, 0.0, 0.0], base_sem_quat)
    lin_body, _ = p.multiplyTransforms(
        [0.0, 0.0, 0.0],
        inv_quat,
        linear_velocity_world,
        [0.0, 0.0, 0.0, 1.0],
    )
    ang_body, _ = p.multiplyTransforms(
        [0.0, 0.0, 0.0],
        inv_quat,
        angular_velocity_world,
        [0.0, 0.0, 0.0, 1.0],
    )
    return tuple(float(v) for v in lin_body), float(ang_body[2])


def desired_velocity_from_config(gait_config: Mapping[str, Any], args: argparse.Namespace) -> Tuple[float, float, float]:
    cfg = gait_config.get("desired_velocity", {}) or {}
    vx = float(args.vx if args.vx is not None else cfg.get("vx", 0.03))
    vy = float(args.vy if args.vy is not None else cfg.get("vy", 0.0))
    yaw_rate = float(args.yaw_rate if args.yaw_rate is not None else cfg.get("yaw_rate", 0.0))
    return vx, vy, yaw_rate


def apply_debug_base_velocity_assist(
    p: Any,
    body_id: int,
    robot_config: Mapping[str, Any],
    controller_config: Mapping[str, Any],
    desired_velocity_body: Sequence[float],
    current_velocity_body: Sequence[float],
) -> None:
    controller = controller_config.get("controller") or {}
    if not bool(controller.get("debug_base_velocity_assist", False)):
        return
    kp = float(controller.get("debug_base_velocity_kp", 20.0))
    max_force = max(float(controller.get("debug_base_velocity_max_force", 3.0)), 0.0)
    force_body = [
        kp * (float(desired_velocity_body[0]) - float(current_velocity_body[0])),
        kp * (float(desired_velocity_body[1]) - float(current_velocity_body[1])),
        0.0,
    ]
    norm_xy = math.hypot(force_body[0], force_body[1])
    if max_force > 0.0 and norm_xy > max_force:
        scale = max_force / max(norm_xy, 1e-9)
        force_body[0] *= scale
        force_body[1] *= scale
    force_world = H.semantic_vector_to_world(p, robot_config, body_id, force_body)
    base_pos, _ = p.getBasePositionAndOrientation(body_id)
    p.applyExternalForce(body_id, -1, force_world, base_pos, p.WORLD_FRAME)


def add_landing_marker(
    p: Any,
    leg_name: str,
    target_world: Sequence[float],
    life_time: float = 0.35,
) -> None:
    H.add_marker(p, target_world, [0.65, 0.15, 1.0], f"{leg_name} Raibert", size=0.032, life_time=life_time)


def apply_contact_dynamics(
    p: Any,
    body_id: int,
    plane_id: int,
    link_name_to_index: Mapping[str, int],
    robot_config: Mapping[str, Any],
    controller_config: Mapping[str, Any],
) -> None:
    """Set repeatable foot/ground friction for leg-driven walking bring-up."""
    contact_cfg = controller_config.get("contact", {}) or {}
    plane_lateral = float(contact_cfg.get("plane_lateral_friction", 1.0))
    foot_lateral = float(contact_cfg.get("foot_lateral_friction", plane_lateral))
    foot_spinning = float(contact_cfg.get("foot_spinning_friction", 0.0))
    foot_rolling = float(contact_cfg.get("foot_rolling_friction", 0.0))
    base_linear_damping = float(contact_cfg.get("base_linear_damping", 0.0))
    base_angular_damping = float(contact_cfg.get("base_angular_damping", 0.0))
    p.changeDynamics(
        body_id,
        -1,
        linearDamping=base_linear_damping,
        angularDamping=base_angular_damping,
    )
    p.changeDynamics(plane_id, -1, lateralFriction=plane_lateral)
    for leg_cfg in (robot_config.get("legs") or {}).values():
        foot_link = str((leg_cfg or {}).get("foot_link", ""))
        if foot_link in link_name_to_index:
            p.changeDynamics(
                body_id,
                link_name_to_index[foot_link],
                lateralFriction=foot_lateral,
                spinningFriction=foot_spinning,
                rollingFriction=foot_rolling,
            )


def apply_soft_cap_base_brake(
    p: Any,
    robot_config: Mapping[str, Any],
    body_id: int,
    controller_config: Mapping[str, Any],
    current_velocity_body: Sequence[float],
) -> None:
    """Damp residual forward velocity after the walking bring-up distance.

    This brake is deliberately one-way: it only removes positive forward speed
    after leg-driven walking has already produced the validation distance.  It
    never pulls the base forward.
    """
    contact_cfg = controller_config.get("contact", {}) or {}
    brake_kp = max(float(contact_cfg.get("soft_cap_brake_kp", 0.0)), 0.0)
    max_force = max(float(contact_cfg.get("soft_cap_brake_max_force", 0.0)), 0.0)
    vx = max(float(current_velocity_body[0]), 0.0)
    if brake_kp <= 0.0 or max_force <= 0.0 or vx <= 0.0:
        return
    force_body = (-min(brake_kp * vx, max_force), 0.0, 0.0)
    force_world = H.semantic_vector_to_world(p, robot_config, body_id, force_body)
    base_pos, _ = p.getBasePositionAndOrientation(body_id)
    p.applyExternalForce(body_id, -1, force_world, base_pos, p.WORLD_FRAME)


def compute_stance_push_body(
    desired_velocity_body: Sequence[float],
    stance_elapsed: float,
    gain: float,
    max_push_m: float,
    ramp_time: float,
) -> Tuple[Tuple[float, float, float], float]:
    """Return a small backward body-frame stance push target offset.

    Positive desired body velocity commands a backward stance-foot target.  In
    contact, the position servo reacts through ground friction and produces the
    first measurable forward body motion while the full WBC/MPC path is still
    being brought up.
    """
    elapsed = max(float(stance_elapsed), 0.0)
    push_x = -float(desired_velocity_body[0]) * elapsed * float(gain)
    push_y = -float(desired_velocity_body[1]) * elapsed * float(gain)
    push_norm = math.hypot(push_x, push_y)
    max_push = max(float(max_push_m), 0.0)
    if max_push > 0.0 and push_norm > max_push:
        scale = max_push / max(push_norm, 1e-9)
        push_x *= scale
        push_y *= scale
        push_norm = max_push

    ramp = 1.0
    if ramp_time > 0.0:
        ramp = min(elapsed / float(ramp_time), 1.0)
    push_x *= ramp
    push_y *= ramp
    push_norm *= ramp
    return (push_x, push_y, 0.0), float(push_norm)


def stance_drive_velocity_body(
    desired_velocity_body: Sequence[float],
    current_velocity_body: Sequence[float],
    max_drive_velocity_mps: float = 0.0,
) -> Tuple[float, float, float]:
    """Return the body-frame velocity error that should still drive stance legs.

    The bootstrap stance drive is closed-loop: once the body is already moving
    at the commanded speed, stance feet stop asking for additional backward
    sweep.  That keeps the gait from turning a successful first push into a
    joint-limit or IK runaway.
    """

    drive_xy = []
    for desired, current in zip(desired_velocity_body[:2], current_velocity_body[:2]):
        desired = float(desired)
        current = float(current)
        if desired > 0.0:
            drive_xy.append(max(desired - current, 0.0))
        elif desired < 0.0:
            drive_xy.append(min(desired - current, 0.0))
        else:
            drive_xy.append(0.0)
    max_drive = max(float(max_drive_velocity_mps), 0.0)
    if max_drive > 0.0:
        drive_norm = math.hypot(drive_xy[0], drive_xy[1])
        if drive_norm > max_drive:
            scale = max_drive / max(drive_norm, 1e-9)
            drive_xy[0] *= scale
            drive_xy[1] *= scale
    return (drive_xy[0], drive_xy[1], 0.0)


def apply_stance_push_target(
    p: Any,
    robot_config: Mapping[str, Any],
    body_id: int,
    anchor_world: Sequence[float],
    desired_velocity_body: Sequence[float],
    stance_elapsed: float,
    gain: float,
    max_push_m: float,
    ramp_time: float,
) -> Tuple[Tuple[float, float, float], float]:
    """Apply the bootstrap stance push in world coordinates.

    Bootstrap stance push:
    The stance foot anchor is the real touchdown point.  We command the desired
    foot slightly backward from that anchor so the position servo produces a
    forward ground reaction through contact.  This is intentionally small and
    clipped; it is a bring-up tool, not the final locomotion controller.
    """
    push_body, push_norm = compute_stance_push_body(
        desired_velocity_body,
        stance_elapsed,
        gain,
        max_push_m,
        ramp_time,
    )
    push_world = H.semantic_vector_to_world(p, robot_config, body_id, push_body)
    target_world = tuple(float(anchor_world[i]) + float(push_world[i]) for i in range(3))
    return target_world, push_norm


def apply_stance_leveling_target(
    target_world: Sequence[float],
    nominal_foot_pos_body: Sequence[float],
    base_pitch: float,
    gain: float,
    max_z_m: float,
) -> Tuple[Tuple[float, float, float], float]:
    """Trim stance foot height targets to counter pitch through the legs.

    This is a leg-side stabilizer: it changes stance foot targets, not the base
    pose.  With the current semantic frame, front feet have negative body-x and
    rear feet positive body-x.  For positive pitch, `pitch * x` lowers the front
    stance targets slightly so the support polygon pushes back through contact.
    """
    raw_z = float(base_pitch) * float(nominal_foot_pos_body[0]) * float(gain)
    max_z = max(float(max_z_m), 0.0)
    z_offset = max(-max_z, min(max_z, raw_z))
    leveled = (float(target_world[0]), float(target_world[1]), float(target_world[2]) + z_offset)
    return leveled, z_offset


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=str(PROJECT_ROOT / "config" / "robot_config.yaml"))
    parser.add_argument("--rail-config", default=str(PROJECT_ROOT / "config" / "rail_config.yaml"))
    parser.add_argument("--controller-config", default=str(PROJECT_ROOT / "config" / "controller_config.yaml"))
    parser.add_argument("--gait-config", default=str(PROJECT_ROOT / "config" / "gait_config.yaml"))
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--vx", type=float, default=None)
    parser.add_argument("--vy", type=float, default=None)
    parser.add_argument("--yaw-rate", type=float, default=None)
    parser.add_argument(
        "--demo-speed-scale",
        type=float,
        default=4.0,
        help="Fixed-base preview scale so low-speed Raibert footholds are visible.",
    )
    parser.add_argument(
        "--dynamic",
        action="store_true",
        help="Run free-base dynamics. Default is fixed-base Raibert target preview.",
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
        print(f"[ERROR] pybullet is required for trot_forward_raibert: {exc}")
        return 2

    robot_config = H.load_yaml(Path(args.config).resolve())
    rail_config = H.load_yaml(Path(args.rail_config).resolve())
    controller_config = H.load_yaml(Path(args.controller_config).resolve())
    gait_config = H.load_yaml(Path(args.gait_config).resolve())
    config_dir = Path(args.config).resolve().parent
    desired_vx, desired_vy, desired_yaw_rate = desired_velocity_from_config(gait_config, args)

    dt = float((controller_config.get("controller") or {}).get("control_dt", 0.002))
    base_height = float((controller_config.get("controller") or {}).get("initial_base_height", 0.42))
    swing_cfg = gait_config.get("swing_trajectory", {}) or {}
    step_height = float(swing_cfg.get("step_height", 0.03))
    if not args.dynamic:
        step_height = min(max(step_height, 0.05), float(swing_cfg.get("max_step_height", 0.06)))
    bootstrap_cfg = gait_config.get("walking_bootstrap", {}) or {}
    bootstrap_enabled = bool(bootstrap_cfg.get("enabled", False))
    stance_push_gain = float(bootstrap_cfg.get("stance_push_gain", 0.85))
    max_stance_push_m = float(bootstrap_cfg.get("max_stance_push_m", 0.035))
    stance_push_ramp_time = float(bootstrap_cfg.get("stance_push_ramp_time", 0.35))
    max_stance_drive_velocity_mps = float(bootstrap_cfg.get("max_stance_drive_velocity_mps", 0.0))
    forward_distance_soft_cap_m = float(bootstrap_cfg.get("forward_distance_soft_cap_m", 0.0))
    stance_level_pitch_gain = float(bootstrap_cfg.get("stance_level_pitch_gain", 0.35))
    max_stance_level_z_m = float(bootstrap_cfg.get("max_stance_level_z_m", 0.012))
    min_dynamic_forward_distance_m = float(bootstrap_cfg.get("min_dynamic_forward_distance_m", 0.02))
    validation_duration_s = float(bootstrap_cfg.get("validation_duration_s", 6.0))

    model_path = H.resolve_path(robot_config["robot"]["model_path"], config_dir)
    urdf_text = H.expand_model_to_urdf(model_path, robot_config, config_dir)
    urdf_text = H.replace_package_urls(urdf_text, H.local_package_map(WORKSPACE_ROOT))
    with tempfile.NamedTemporaryFile("w", suffix=".urdf", delete=False, encoding="utf-8") as f:
        f.write(urdf_text)
        temp_urdf = f.name

    logs_dir = PROJECT_ROOT / "logs"
    logs_dir.mkdir(parents=True, exist_ok=True)
    log_path = logs_dir / "trot_forward_log.csv"

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
        H.disable_body_collision_and_visual(p, kin_body_id)

        joint_name_to_index, link_name_to_index, joint_limits, _ = H.pybullet_maps(p, body_id)
        kin_joint_name_to_index, kin_link_name_to_index, _, _ = H.pybullet_maps(p, kin_body_id)
        enable_foot_only_ground_contact(
            p,
            body_id,
            plane_id,
            link_name_to_index,
            robot_config,
            keep_base_collision=True,
        )
        apply_contact_dynamics(p, body_id, plane_id, link_name_to_index, robot_config, controller_config)
        rail_controller = RailLockController.from_config_dicts(robot_config, rail_config)
        joint_pd = JointPDController.from_config_dicts(
            robot_config,
            controller_config,
            excluded_joints=rail_controller.rail_joint_names,
        )
        rail_controller.validate_leg_joint_exclusion(H.leg_joint_names(robot_config))
        gait_scheduler = GaitScheduler.from_config_dict(gait_config)
        swing_trajectory = SwingTrajectory.from_config_dict(gait_config)
        stance_controller = StanceController.from_config_dict(gait_config)
        raibert_planner = RaibertPlanner.from_config_dict(gait_config)
        safety_checker = SafetyChecker.from_config_dict(controller_config)

        visible_default_q = H.default_stand_targets(robot_config, pose_key="default_stand_q")
        locomotion_q = H.default_stand_targets(robot_config, pose_key="locomotion_stand_q")
        if args.dynamic and args.gui:
            H.preview_default_stand_pose(
                p,
                body_id,
                joint_name_to_index,
                link_name_to_index,
                robot_config,
                controller_config,
                rail_controller,
                duration_s=float(max(args.initial_visible_hold, 0.0)),
            )
        H.reset_initial_pose(
            p,
            body_id,
            joint_name_to_index,
            link_name_to_index,
            robot_config,
            base_height,
            pose_key="locomotion_stand_q" if args.dynamic else "default_stand_q",
        )
        H.apply_zero_default_motors(p, body_id)
        joint_state = H.joint_dict_from_pybullet(p, body_id, joint_name_to_index)
        rail_controller.initialize({name: joint_state[name] for name in rail_controller.rail_joint_names})
        joint_pd.reset_targets(joint_state)

        kin = H.PyBulletLegKinematics(p, kin_body_id, robot_config, kin_joint_name_to_index, kin_link_name_to_index)
        kin.set_pose(locomotion_q)
        ik = NumericalIK.from_robot_config(robot_config, kin.foot_position, tolerance=0.01)

        if args.gui:
            p.resetDebugVisualizerCamera(
                cameraDistance=1.12,
                cameraYaw=35.0,
                cameraPitch=-24.0,
                cameraTargetPosition=[0.02, 0.0, 0.17],
            )

        nominal_foot_pos_body: Dict[str, Tuple[float, float, float]] = {}
        for leg_name in LEG_ORDER:
            foot_link = str(((robot_config.get("legs") or {}).get(leg_name, {}) or {}).get("foot_link", ""))
            nominal_foot_pos_body[leg_name] = H.world_to_semantic_base(
                p,
                robot_config,
                body_id,
                p.getLinkState(body_id, link_name_to_index[foot_link])[0],
            )

        swing_memory: Dict[str, Dict[str, Optional[Tuple[float, float, float]]]] = {
            leg_name: {
                "start_world": None,
                "target_world": None,
                "landing_world": None,
                "prev_des_world": None,
                "landing_body": None,
            }
            for leg_name in LEG_ORDER
        }
        last_state = {leg_name: "stance" for leg_name in LEG_ORDER}
        stance_entry_time = {leg_name: gait_scheduler.config.initial_stand_time for leg_name in LEG_ORDER}
        last_good_q = {
            leg: tuple(locomotion_q[joint_name] for joint_name in H.leg_joint_names(robot_config)[leg])
            for leg in LEG_ORDER
        }
        pd_torque_limits = {name: cfg.torque_limit for name, cfg in joint_pd.joint_configs.items()}
        initial_base_x = float(p.getBasePositionAndOrientation(body_id)[0][0])

        headers = [
            "time",
            "base_x",
            "base_y",
            "base_z",
            "base_roll",
            "base_pitch",
            "base_yaw",
            "base_vx_body",
            "base_vy_body",
            "base_wz_body",
            "base_dx",
            "avg_vx_world",
            "desired_vx",
            "desired_vy",
            "rail_error",
            "gait_states",
            "foot_actual_world",
            "foot_target_world",
            "stance_push_norm",
            "stance_level_z",
            "raibert_landing_body",
            "ik_error",
            "safety_status",
        ]

        success = True
        final_base_dx = 0.0
        final_avg_vx_world = 0.0
        max_abs_roll = 0.0
        max_abs_pitch = 0.0
        max_stance_push_seen = 0.0
        debug_draw_interval = max(1, int(0.10 / dt))
        with log_path.open("w", newline="", encoding="utf-8") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=headers)
            writer.writeheader()

            steps = int(float(args.duration) / dt)
            for step in range(steps):
                sim_time = step * dt
                joint_state = H.joint_dict_from_pybullet(p, body_id, joint_name_to_index)
                gait_states = gait_scheduler.update(sim_time)
                blend = gait_scheduler.transition_blend(sim_time)
                in_initial_stand = sim_time < gait_scheduler.config.initial_stand_time

                base_pos, base_quat = p.getBasePositionAndOrientation(body_id)
                base_rpy_before_step = H.semantic_base_rpy(p, robot_config, body_id)
                base_lin_vel_world, base_ang_vel_world = p.getBaseVelocity(body_id)
                measured_vel_body, measured_yaw_rate = body_velocity_from_world(
                    p,
                    robot_config,
                    body_id,
                    base_lin_vel_world,
                    base_ang_vel_world,
                )
                if args.dynamic:
                    current_vel_body = measured_vel_body
                    current_yaw_rate = measured_yaw_rate
                    desired_vel_body = (desired_vx, desired_vy, 0.0)
                    yaw_rate_des = desired_yaw_rate
                else:
                    desired_vel_body = (desired_vx * args.demo_speed_scale, desired_vy * args.demo_speed_scale, 0.0)
                    current_vel_body = desired_vel_body
                    current_yaw_rate = desired_yaw_rate * args.demo_speed_scale
                    yaw_rate_des = current_yaw_rate
                current_base_dx = float(base_pos[0]) - initial_base_x
                soft_cap_active = (
                    args.dynamic
                    and bootstrap_enabled
                    and forward_distance_soft_cap_m > 0.0
                    and current_base_dx >= forward_distance_soft_cap_m
                )
                planning_desired_vel_body = desired_vel_body
                planning_current_vel_body = current_vel_body
                if soft_cap_active:
                    planning_desired_vel_body = (0.0, desired_vel_body[1], 0.0)
                    planning_current_vel_body = (0.0, 0.0, 0.0)

                foot_actual_world: Dict[str, Tuple[float, float, float]] = {}
                foot_target_world: Dict[str, Tuple[float, float, float]] = {}
                landing_body: Dict[str, Optional[Tuple[float, float, float]]] = {}
                ik_success: Dict[str, bool] = {}
                ik_error: Dict[str, float] = {}
                foot_target_jumps: Dict[str, float] = {}
                stance_push_norm = {leg_name: 0.0 for leg_name in LEG_ORDER}
                stance_level_z = {leg_name: 0.0 for leg_name in LEG_ORDER}
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
                    previous_target = swing_memory[leg_name]["prev_des_world"]

                    if in_initial_stand:
                        foot_target_world[leg_name] = actual_world
                        landing_body[leg_name] = None
                        stance_entry_time[leg_name] = gait_scheduler.config.initial_stand_time
                        foot_target_jumps[leg_name] = 0.0 if previous_target is None else math.dist(actual_world, previous_target)
                        swing_memory[leg_name]["start_world"] = None
                        swing_memory[leg_name]["target_world"] = None
                        swing_memory[leg_name]["landing_world"] = None
                        swing_memory[leg_name]["landing_body"] = None
                        swing_memory[leg_name]["prev_des_world"] = actual_world
                        ik_success[leg_name] = True
                        ik_error[leg_name] = 0.0
                        continue

                    if state == "stance":
                        if last_state[leg_name] != "stance":
                            stance_entry_time[leg_name] = sim_time
                        stance_target = stance_controller.update_leg(leg_name, state, actual_world)
                        foot_target_world[leg_name] = stance_target.target_world
                        if soft_cap_active:
                            foot_target_world[leg_name] = actual_world
                        elif args.dynamic and bootstrap_enabled:
                            desired_drive_body = planning_desired_vel_body
                            drive_velocity_body = stance_drive_velocity_body(
                                desired_drive_body,
                                current_vel_body,
                                max_stance_drive_velocity_mps,
                            )
                            pushed_target, push_norm = apply_stance_push_target(
                                p,
                                robot_config,
                                body_id,
                                stance_target.target_world,
                                drive_velocity_body,
                                sim_time - stance_entry_time[leg_name],
                                stance_push_gain,
                                max_stance_push_m,
                                stance_push_ramp_time,
                            )
                            foot_target_world[leg_name] = pushed_target
                            stance_push_norm[leg_name] = push_norm
                            max_stance_push_seen = max(max_stance_push_seen, push_norm)
                            if stance_level_pitch_gain > 0.0 and max_stance_level_z_m > 0.0:
                                leveled_target, level_z = apply_stance_leveling_target(
                                    foot_target_world[leg_name],
                                    nominal_foot_pos_body[leg_name],
                                    base_rpy_before_step[1],
                                    stance_level_pitch_gain,
                                    max_stance_level_z_m,
                                )
                                foot_target_world[leg_name] = leveled_target
                                stance_level_z[leg_name] = level_z
                        landing_body[leg_name] = swing_memory[leg_name]["landing_body"]
                        swing_memory[leg_name]["start_world"] = None
                        swing_memory[leg_name]["target_world"] = None
                        swing_memory[leg_name]["landing_world"] = None
                    else:
                        stance_controller.update_leg(leg_name, state, actual_world)
                        entering_swing = last_state[leg_name] != "swing"
                        if entering_swing or swing_memory[leg_name]["start_world"] is None:
                            result = raibert_planner.plan(
                                leg_name=leg_name,
                                nominal_foot_pos_body=nominal_foot_pos_body[leg_name],
                                desired_base_velocity_body=planning_desired_vel_body,
                                current_base_velocity_body=planning_current_vel_body,
                                stance_time=gait_scheduler.stance_time,
                                swing_time=gait_scheduler.swing_time,
                                yaw_rate_desired=yaw_rate_des,
                                current_yaw_rate=current_yaw_rate,
                                previous_target_body=swing_memory[leg_name]["landing_body"],
                            )
                            landing_body[leg_name] = result.foot_target_body
                            landing_world = H.semantic_base_to_world(p, robot_config, body_id, result.foot_target_body)
                            target_world = tuple(
                                (1.0 - blend) * actual_world[i] + blend * landing_world[i]
                                for i in range(3)
                            )
                            swing_memory[leg_name]["start_world"] = actual_world
                            swing_memory[leg_name]["target_world"] = target_world
                            swing_memory[leg_name]["landing_world"] = landing_world
                            swing_memory[leg_name]["landing_body"] = result.foot_target_body
                            swing_memory[leg_name]["prev_des_world"] = actual_world
                        else:
                            landing_body[leg_name] = swing_memory[leg_name]["landing_body"]

                        sample = swing_trajectory.evaluate(
                            swing_memory[leg_name]["start_world"],
                            swing_memory[leg_name]["target_world"],
                            swing_phase,
                            step_height=step_height * blend,
                            swing_duration=gait_scheduler.swing_time,
                            previous_desired_foot_pos_world=swing_memory[leg_name]["prev_des_world"],
                        )
                        foot_target_world[leg_name] = sample.position_world

                    foot_target_jumps[leg_name] = 0.0 if previous_target is None else math.dist(
                        foot_target_world[leg_name], previous_target
                    )
                    if soft_cap_active:
                        foot_target_jumps[leg_name] = 0.0
                    swing_memory[leg_name]["prev_des_world"] = foot_target_world[leg_name]

                    target_base = H.world_to_semantic_base(p, robot_config, body_id, foot_target_world[leg_name])
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
                    for joint_name, q in zip(H.leg_joint_names(robot_config)[leg_name], last_good_q[leg_name]):
                        q_des_all[joint_name] = q

                if args.dynamic:
                    pd_commands = joint_pd.compute(joint_state, q_des_all)
                    torques = {name: command.tau for name, command in pd_commands.items()}
                    rail_commands = rail_controller.compute(
                        {name: joint_state[name] for name in rail_controller.rail_joint_names}
                    )
                    H.apply_leg_actuators(p, body_id, joint_name_to_index, pd_commands, joint_pd, controller_config)
                    H.apply_rail_actuators(p, body_id, joint_name_to_index, rail_commands, rail_controller, controller_config)
                    H.apply_debug_base_attitude_assist(p, body_id, robot_config, controller_config)
                    if soft_cap_active:
                        apply_soft_cap_base_brake(
                            p,
                            robot_config,
                            body_id,
                            controller_config,
                            current_vel_body,
                        )
                    apply_debug_base_velocity_assist(
                        p,
                        body_id,
                        robot_config,
                        controller_config,
                        desired_vel_body,
                        current_vel_body,
                    )
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
                    joint_state = H.joint_dict_from_pybullet(p, body_id, joint_name_to_index)
                    rail_commands = rail_controller.compute(
                        {name: joint_state[name] for name in rail_controller.rail_joint_names}
                    )
                    torques = {name: 0.0 for name in joint_pd.joint_names}

                p.stepSimulation()
                if args.gui and bool((controller_config.get("controller") or {}).get("gui_sleep", True)):
                    time.sleep(dt)

                base_pos, _ = p.getBasePositionAndOrientation(body_id)
                base_rpy = H.semantic_base_rpy(p, robot_config, body_id)
                final_base_dx = float(base_pos[0]) - initial_base_x
                final_avg_vx_world = final_base_dx / max(sim_time, 1e-9)
                max_abs_roll = max(max_abs_roll, abs(float(base_rpy[0])))
                max_abs_pitch = max(max_abs_pitch, abs(float(base_rpy[1])))
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
                    H.apply_leg_visuals(p, body_id, link_name_to_index, gait_states)
                    p.addUserDebugText(
                        f"Raibert gait ({'dynamic' if args.dynamic else 'fixed-base preview'}) | t={sim_time:.2f}s",
                        [0.18, -0.18, 0.36],
                        textColorRGB=[1.0, 1.0, 1.0],
                        textSize=1.35,
                        lifeTime=0.0,
                    )
                    p.addUserDebugText(
                        f"desired vx={desired_vx:.3f} m/s | preview scale={args.demo_speed_scale if not args.dynamic else 1.0:.1f}",
                        [0.18, -0.18, 0.32],
                        textColorRGB=[0.9, 0.9, 0.9],
                        textSize=1.05,
                        lifeTime=0.0,
                    )
                    if args.dynamic:
                        contact_names = sorted(ground_contact_link_names(p, body_id, plane_id))
                        p.addUserDebugText(
                            "Ground contacts: " + ", ".join(contact_names),
                            [0.18, -0.18, 0.28],
                            textColorRGB=[0.65, 1.0, 1.0],
                            textSize=1.0,
                            lifeTime=0.0,
                        )
                    for leg_name in LEG_ORDER:
                        state = gait_states[leg_name]["state"]
                        actual = foot_actual_world[leg_name]
                        target = foot_target_world[leg_name]
                        target_color = [0.1, 0.4, 1.0] if state == "stance" else [1.0, 0.55, 0.1]
                        H.add_marker(p, actual, [0.1, 1.0, 0.1], f"{leg_name} actual", size=0.020, life_time=0.35)
                        H.add_marker(p, target, target_color, f"{leg_name} {state}", size=0.027, life_time=0.35)
                        p.addUserDebugLine(actual, target, [1.0, 0.0, 0.0], lineWidth=3, lifeTime=0.35)
                        if swing_memory[leg_name]["landing_world"] is not None:
                            add_landing_marker(p, leg_name, swing_memory[leg_name]["landing_world"], life_time=0.35)

                if step % 10 == 0:
                    writer.writerow(
                        {
                            "time": f"{sim_time:.6f}",
                            "base_x": f"{base_pos[0]:.6f}",
                            "base_y": f"{base_pos[1]:.6f}",
                            "base_z": f"{base_pos[2]:.6f}",
                            "base_roll": f"{base_rpy[0]:.6f}",
                            "base_pitch": f"{base_rpy[1]:.6f}",
                            "base_yaw": f"{base_rpy[2]:.6f}",
                            "base_vx_body": f"{measured_vel_body[0]:.6f}",
                            "base_vy_body": f"{measured_vel_body[1]:.6f}",
                            "base_wz_body": f"{measured_yaw_rate:.6f}",
                            "base_dx": f"{final_base_dx:.6f}",
                            "avg_vx_world": f"{final_avg_vx_world:.6f}",
                            "desired_vx": f"{desired_vx:.6f}",
                            "desired_vy": f"{desired_vy:.6f}",
                            "rail_error": H.row_json({name: command.error for name, command in rail_commands.items()}),
                            "gait_states": H.row_json({leg: gait_states[leg]["state"] for leg in LEG_ORDER}),
                            "foot_actual_world": H.row_json(
                                {leg: {"x": foot_actual_world[leg][0], "y": foot_actual_world[leg][1], "z": foot_actual_world[leg][2]} for leg in LEG_ORDER}
                            ),
                            "foot_target_world": H.row_json(
                                {leg: {"x": foot_target_world[leg][0], "y": foot_target_world[leg][1], "z": foot_target_world[leg][2]} for leg in LEG_ORDER}
                            ),
                            "stance_push_norm": H.row_json(stance_push_norm),
                            "stance_level_z": H.row_json(stance_level_z),
                            "raibert_landing_body": H.row_json(
                                {
                                    leg: None
                                    if landing_body.get(leg) is None
                                    else {
                                        "x": landing_body[leg][0],
                                        "y": landing_body[leg][1],
                                        "z": landing_body[leg][2],
                                    }
                                    for leg in LEG_ORDER
                                }
                            ),
                            "ik_error": H.row_json(ik_error),
                            "safety_status": safety_status,
                        }
                    )

                last_state = {leg_name: gait_states[leg_name]["state"] for leg_name in LEG_ORDER}
                if not safety.ok:
                    break

        print(f"[INFO] Trot-forward Raibert log: {log_path}")
        print(f"[SUMMARY] dynamic forward distance: {final_base_dx:.4f} m")
        print(f"[SUMMARY] avg vx: {final_avg_vx_world:.4f} m/s")
        print(
            "[SUMMARY] max abs roll/pitch: "
            f"{math.degrees(max_abs_roll):.2f}/{math.degrees(max_abs_pitch):.2f} deg"
        )
        print(f"[SUMMARY] stance push max: {max_stance_push_seen:.4f} m")
        if (
            args.dynamic
            and success
            and float(args.duration) >= validation_duration_s
            and final_base_dx < min_dynamic_forward_distance_m
        ):
            print(
                "[FAIL] Dynamic gait did not produce enough forward motion: "
                f"dx={final_base_dx:.4f} m < {min_dynamic_forward_distance_m:.4f} m"
            )
            success = False
        elif (
            args.dynamic
            and success
            and float(args.duration) < validation_duration_s
            and final_base_dx < min_dynamic_forward_distance_m
        ):
            print(
                "[WARN] Dynamic gait forward distance is below validation threshold, "
                "but duration is shorter than validation_duration_s."
            )
        print("[PASS] Raibert gait preview completed." if success else "[FAIL] Raibert gait stopped on safety.")
        if args.gui and not success and not args.no_hold_on_stop:
            H.recover_to_default_stand_posture(
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
                        [0.18, -0.18, 0.40],
                        textColorRGB=[1.0, 0.1, 0.1],
                        textSize=1.25,
                        lifeTime=0.3,
                    )
                    p.addUserDebugText(
                        "Ground contacts: " + (", ".join(contact_names) if contact_names else "none"),
                        [0.18, -0.18, 0.36],
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
