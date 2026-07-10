"""Pure acceptance decisions for Dog2 locomotion validation.

This module deliberately has no ROS imports.  The live node is responsible for
collecting ground truth; this oracle is responsible for deterministic verdicts
that can be exercised with replay and fault-injection tests.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Mapping, Optional, Sequence

from dog2_bringup.acceptance_metrics import JointLimit


PROTOCOL_STAGE_ORDER = (
    "WAIT_READY",
    "WAIT_SETTLE",
    "STAND",
    "OUTBOUND",
    "OUTBOUND_STOP",
    "RETURN",
    "RETURN_STOP",
    "TURN",
    "FINAL_STOP",
)


@dataclass(frozen=True)
class GateFailure:
    code: str
    message: str
    measured: object = None
    limit: object = None
    provenance: str = "PROJECT_SAFETY"
    infrastructure: bool = False


@dataclass(frozen=True)
class MotionObservation:
    x_m: float
    y_m: float
    z_m: float
    yaw_rad: float
    tilt_rad: float
    up_z: float
    vx_body_mps: float
    vy_body_mps: float
    vz_mps: float
    yaw_rate_radps: float
    angular_speed_radps: float
    lateral_error_m: float
    rail_error_m: float
    base_contact: bool
    tibia_contact_legs: tuple[str, ...] = ()


@dataclass(frozen=True)
class MotionGateLimits:
    max_tilt_rad: float
    min_up_z: float
    body_height_min_m: float
    body_height_max_m: float
    max_rail_error_m: float
    corridor_half_width_m: float
    enforce_tibia_contact: bool = True


@dataclass(frozen=True)
class StabilityLimits:
    max_planar_speed_mps: float
    max_vertical_speed_mps: float
    max_angular_speed_radps: float


@dataclass(frozen=True)
class JointGateEvaluation:
    failure: Optional[GateFailure]
    minimum_position_margin: Optional[float]
    maximum_velocity_ratio: float
    maximum_effort_ratio: float


@dataclass(frozen=True)
class TurnAssessment:
    signed_progress_rad: float
    final_error_rad: float
    translation_m: float


def evaluate_motion_hard_gates(
    observation: MotionObservation,
    limits: MotionGateLimits,
    *,
    enforce_route_corridor: bool,
) -> Optional[GateFailure]:
    finite_values = (
        observation.x_m,
        observation.y_m,
        observation.z_m,
        observation.yaw_rad,
        observation.tilt_rad,
        observation.up_z,
        observation.vx_body_mps,
        observation.vy_body_mps,
        observation.vz_mps,
        observation.yaw_rate_radps,
        observation.angular_speed_radps,
        observation.lateral_error_m,
        observation.rail_error_m,
    )
    if not all(math.isfinite(value) for value in finite_values):
        return GateFailure(
            "NONFINITE_STATE",
            "non-finite Gazebo ground-truth state",
            measured=list(finite_values),
            provenance="PHYSICAL_HARD",
        )
    if observation.base_contact:
        return GateFailure(
            "BASE_CONTACT",
            "base_link made illegal contact",
            measured=True,
            limit=False,
            provenance="REFERENCE_HARD",
        )
    if limits.enforce_tibia_contact and observation.tibia_contact_legs:
        return GateFailure(
            "TIBIA_CONTACT",
            f"{observation.tibia_contact_legs[0]}_tibia_link made illegal contact",
            measured=list(observation.tibia_contact_legs),
            limit=[],
            provenance="PROJECT_SAFETY",
        )
    if observation.tilt_rad > limits.max_tilt_rad:
        return GateFailure(
            "TILT_LIMIT",
            "body tilt exceeded safety limit",
            measured=observation.tilt_rad,
            limit=limits.max_tilt_rad,
        )
    if observation.up_z < limits.min_up_z:
        return GateFailure(
            "UP_Z_LIMIT",
            "body +Z no longer points sufficiently upward",
            measured=observation.up_z,
            limit=limits.min_up_z,
        )
    if not limits.body_height_min_m <= observation.z_m <= limits.body_height_max_m:
        return GateFailure(
            "BODY_HEIGHT_LIMIT",
            "body height left the accepted band",
            measured=observation.z_m,
            limit=[limits.body_height_min_m, limits.body_height_max_m],
        )
    if observation.rail_error_m > limits.max_rail_error_m:
        return GateFailure(
            "RAIL_LOCK_ERROR",
            "one or more prismatic rails left the zero-lock tolerance",
            measured=observation.rail_error_m,
            limit=limits.max_rail_error_m,
            provenance="MISSION_REQUIREMENT",
        )
    if enforce_route_corridor and abs(observation.lateral_error_m) > limits.corridor_half_width_m:
        return GateFailure(
            "ROUTE_CORRIDOR",
            "body left the fixed-route lateral corridor",
            measured=abs(observation.lateral_error_m),
            limit=limits.corridor_half_width_m,
            provenance="TASK_PROTOCOL",
        )
    return None


def evaluate_joint_hard_gates(
    limits: Mapping[str, JointLimit],
    positions: Mapping[str, float],
    velocities: Mapping[str, float],
    efforts: Mapping[str, float],
    *,
    required_effort_names: Sequence[str] = (),
    epsilon: float = 1e-6,
) -> JointGateEvaluation:
    missing_positions = sorted(set(limits) - set(positions))
    missing_velocities = sorted(set(limits) - set(velocities))
    missing_efforts = sorted(set(required_effort_names) - set(efforts))
    if missing_positions or missing_velocities or missing_efforts:
        return JointGateEvaluation(
            GateFailure(
                "MISSING_JOINT_STATE",
                "required joint state or command channels are absent",
                measured={
                    "positions": missing_positions,
                    "velocities": missing_velocities,
                    "efforts": missing_efforts,
                },
                provenance="TEST_INFRASTRUCTURE",
                infrastructure=True,
            ),
            None,
            0.0,
            0.0,
        )

    minimum_margin = math.inf
    maximum_velocity_ratio = 0.0
    maximum_effort_ratio = 0.0
    for name, limit in limits.items():
        position = float(positions[name])
        velocity = abs(float(velocities[name]))
        effort = abs(float(efforts[name])) if name in efforts else 0.0
        if not all(math.isfinite(value) for value in (position, velocity, effort)):
            return JointGateEvaluation(
                GateFailure(
                    "NONFINITE_JOINT_STATE",
                    f"{name} has a non-finite state or command",
                    measured=[position, velocity, effort],
                    provenance="PHYSICAL_HARD",
                ),
                None,
                maximum_velocity_ratio,
                maximum_effort_ratio,
            )

        if limit.lower is not None:
            minimum_margin = min(minimum_margin, position - limit.lower)
            if position < limit.lower - epsilon:
                return JointGateEvaluation(
                    GateFailure(
                        "JOINT_POSITION_LIMIT",
                        f"{name} is below its URDF hard limit",
                        measured=position,
                        limit=limit.lower,
                        provenance="PHYSICAL_HARD",
                    ),
                    minimum_margin,
                    maximum_velocity_ratio,
                    maximum_effort_ratio,
                )
        if limit.upper is not None:
            minimum_margin = min(minimum_margin, limit.upper - position)
            if position > limit.upper + epsilon:
                return JointGateEvaluation(
                    GateFailure(
                        "JOINT_POSITION_LIMIT",
                        f"{name} is above its URDF hard limit",
                        measured=position,
                        limit=limit.upper,
                        provenance="PHYSICAL_HARD",
                    ),
                    minimum_margin,
                    maximum_velocity_ratio,
                    maximum_effort_ratio,
                )

        if limit.velocity is not None and limit.velocity > 0.0:
            ratio = velocity / limit.velocity
            maximum_velocity_ratio = max(maximum_velocity_ratio, ratio)
            if ratio > 1.0 + epsilon:
                return JointGateEvaluation(
                    GateFailure(
                        "JOINT_VELOCITY_LIMIT",
                        f"{name} exceeded its URDF velocity limit",
                        measured=velocity,
                        limit=limit.velocity,
                        provenance="PHYSICAL_HARD",
                    ),
                    None if math.isinf(minimum_margin) else minimum_margin,
                    maximum_velocity_ratio,
                    maximum_effort_ratio,
                )

        if name in efforts and limit.effort is not None and limit.effort > 0.0:
            ratio = effort / limit.effort
            maximum_effort_ratio = max(maximum_effort_ratio, ratio)
            if ratio > 1.0 + epsilon:
                return JointGateEvaluation(
                    GateFailure(
                        "JOINT_EFFORT_LIMIT",
                        f"{name} command exceeded its URDF effort limit",
                        measured=effort,
                        limit=limit.effort,
                        provenance="PHYSICAL_HARD",
                    ),
                    None if math.isinf(minimum_margin) else minimum_margin,
                    maximum_velocity_ratio,
                    maximum_effort_ratio,
                )

    return JointGateEvaluation(
        None,
        None if math.isinf(minimum_margin) else minimum_margin,
        maximum_velocity_ratio,
        maximum_effort_ratio,
    )


def is_stable(
    observation: MotionObservation,
    motion_limits: MotionGateLimits,
    stability_limits: StabilityLimits,
) -> bool:
    if evaluate_motion_hard_gates(
        observation, motion_limits, enforce_route_corridor=False
    ) is not None:
        return False
    return (
        math.hypot(observation.vx_body_mps, observation.vy_body_mps)
        <= stability_limits.max_planar_speed_mps
        and abs(observation.vz_mps) <= stability_limits.max_vertical_speed_mps
        and observation.angular_speed_radps
        <= stability_limits.max_angular_speed_radps
    )


def stand_drift_failure(
    origin_xy: tuple[float, float],
    current_xy: tuple[float, float],
    maximum_drift_m: float,
) -> Optional[GateFailure]:
    drift = math.hypot(
        float(current_xy[0]) - float(origin_xy[0]),
        float(current_xy[1]) - float(origin_xy[1]),
    )
    if drift <= maximum_drift_m:
        return None
    return GateFailure(
        "STAND_DRIFT",
        "stationary stand translated beyond its allowed radius",
        measured=drift,
        limit=maximum_drift_m,
        provenance="TASK_PROTOCOL",
    )


def assess_turn(
    start_xy: tuple[float, float],
    current_xy: tuple[float, float],
    start_unwrapped_yaw_rad: float,
    current_unwrapped_yaw_rad: float,
    command_yaw_rate_radps: float,
    target_yaw_rad: float,
) -> TurnAssessment:
    direction = -1.0 if command_yaw_rate_radps < 0.0 else 1.0
    progress = direction * (
        float(current_unwrapped_yaw_rad) - float(start_unwrapped_yaw_rad)
    )
    return TurnAssessment(
        signed_progress_rad=progress,
        final_error_rad=progress - abs(float(target_yaw_rad)),
        translation_m=math.hypot(
            float(current_xy[0]) - float(start_xy[0]),
            float(current_xy[1]) - float(start_xy[1]),
        ),
    )


def completion_outcome(
    calibration_status: str,
    quality_failures: Sequence[GateFailure],
) -> tuple[str, bool, bool]:
    """Return status, final-pass flag, and provisional-pass flag."""

    if quality_failures:
        return "FAIL_LOCOMOTION", False, False
    if calibration_status.strip().lower() != "calibrated":
        return "PASS_SAFETY_ROUTE_PROVISIONAL", False, True
    return "PASS_LOCOMOTION_BASELINE", True, False


def is_valid_stage_transition(current: str, requested: str) -> bool:
    try:
        return (
            PROTOCOL_STAGE_ORDER.index(requested)
            == PROTOCOL_STAGE_ORDER.index(current) + 1
        )
    except ValueError:
        return False


def latch_illegal_contact(
    existing: Optional[dict],
    *,
    active: bool,
    scored: bool,
    kind: str,
    leg: str,
    force_n: Optional[float],
    wrench_available: bool,
    wall_time_sec: float,
    sim_time_sec: Optional[float],
    stage: str,
) -> Optional[dict]:
    """Latch the first scored illegal contact; later empty samples cannot erase it."""

    if existing is not None or not active or not scored or kind not in {"base", "tibia"}:
        return existing
    return {
        "kind": kind,
        "leg": leg,
        "force_n": force_n if wrench_available else None,
        "wrench_available": bool(wrench_available),
        "wall_time_sec": float(wall_time_sec),
        "sim_time_sec": sim_time_sec,
        "stage": stage,
    }
