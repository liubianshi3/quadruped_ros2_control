"""Pure metric helpers for Dog2 headless locomotion acceptance."""

from __future__ import annotations

import json
import hashlib
import math
import os
import tempfile
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, Mapping, Optional, Sequence, Tuple


LEG_ORDER = ("lf", "lh", "rh", "rf")


@dataclass
class ScalarStats:
    """Online population statistics with RMS."""

    count: int = 0
    total: float = 0.0
    total_sq: float = 0.0
    minimum: float = math.inf
    maximum: float = -math.inf

    def update(self, value: float) -> None:
        value = float(value)
        if not math.isfinite(value):
            return
        self.count += 1
        self.total += value
        self.total_sq += value * value
        self.minimum = min(self.minimum, value)
        self.maximum = max(self.maximum, value)

    @property
    def mean(self) -> Optional[float]:
        if self.count == 0:
            return None
        return self.total / self.count

    @property
    def rms(self) -> Optional[float]:
        if self.count == 0:
            return None
        return math.sqrt(max(0.0, self.total_sq / self.count))

    @property
    def std(self) -> Optional[float]:
        if self.count == 0:
            return None
        mean = self.total / self.count
        return math.sqrt(max(0.0, self.total_sq / self.count - mean * mean))

    def as_dict(self) -> dict:
        return {
            "count": self.count,
            "mean": self.mean,
            "std": self.std,
            "rms": self.rms,
            "min": None if self.count == 0 else self.minimum,
            "max": None if self.count == 0 else self.maximum,
        }


@dataclass(frozen=True)
class JointLimit:
    name: str
    joint_type: str
    lower: Optional[float]
    upper: Optional[float]
    velocity: Optional[float]
    effort: Optional[float]


@dataclass(frozen=True)
class RobotModelMetadata:
    """Mechanical values derived from the exact expanded URDF under test."""

    total_mass_kg: float
    body_length_m: float
    body_width_m: float
    foot_radius_m: float


@dataclass(frozen=True)
class PoseKinematics:
    """Pose-difference velocity, independent of an odometry twist convention."""

    vx_world_mps: float
    vy_world_mps: float
    vz_world_mps: float
    yaw_rate_radps: float
    angular_speed_radps: float


def parse_joint_limits(robot_description: str) -> Dict[str, JointLimit]:
    """Read movable-joint hard limits from expanded URDF XML."""

    root = ET.fromstring(robot_description)
    limits: Dict[str, JointLimit] = {}
    for joint in root.findall("joint"):
        joint_type = str(joint.get("type", ""))
        if joint_type not in {"prismatic", "revolute", "continuous"}:
            continue
        name = str(joint.get("name", ""))
        limit = joint.find("limit")
        if not name or limit is None:
            continue

        def _optional_float(key: str) -> Optional[float]:
            raw = limit.get(key)
            return None if raw is None else float(raw)

        limits[name] = JointLimit(
            name=name,
            joint_type=joint_type,
            lower=_optional_float("lower"),
            upper=_optional_float("upper"),
            velocity=_optional_float("velocity"),
            effort=_optional_float("effort"),
        )
    return limits


def parse_robot_model_metadata(robot_description: str) -> RobotModelMetadata:
    """Derive mass, trunk dimensions, and foot radius from expanded URDF."""

    root = ET.fromstring(robot_description)
    total_mass = 0.0
    foot_radii: list[float] = []
    body_length = None
    body_width = None
    for link in root.findall("link"):
        inertial_mass = link.find("./inertial/mass")
        if inertial_mass is not None and inertial_mass.get("value") is not None:
            total_mass += float(inertial_mass.get("value", "0"))

        name = str(link.get("name", ""))
        if name == "base_link":
            box = link.find("./collision/geometry/box")
            if box is None or box.get("size") is None:
                raise ValueError("base_link collision must contain a sized box")
            size = [float(value) for value in str(box.get("size")).split()]
            if len(size) != 3 or not all(math.isfinite(value) for value in size):
                raise ValueError("base_link collision box size is invalid")
            body_length, body_width = size[0], size[1]
        elif name.endswith("_foot_link"):
            sphere = link.find("./collision/geometry/sphere")
            if sphere is None or sphere.get("radius") is None:
                raise ValueError(f"{name} collision must contain a sized sphere")
            foot_radii.append(float(sphere.get("radius", "nan")))

    if not math.isfinite(total_mass) or total_mass <= 0.0:
        raise ValueError("URDF total mass must be finite and positive")
    if body_length is None or body_width is None:
        raise ValueError("URDF base_link dimensions are unavailable")
    if len(foot_radii) != len(LEG_ORDER):
        raise ValueError(f"expected {len(LEG_ORDER)} foot collision spheres")
    if not all(math.isfinite(radius) and radius > 0.0 for radius in foot_radii):
        raise ValueError("foot radii must be finite and positive")
    if max(foot_radii) - min(foot_radii) > 1e-9:
        raise ValueError(f"inconsistent foot radii: {foot_radii}")
    return RobotModelMetadata(
        total_mass_kg=total_mass,
        body_length_m=body_length,
        body_width_m=body_width,
        foot_radius_m=foot_radii[0],
    )


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def transform_point(
    parent_xyz: Tuple[float, float, float],
    parent_quaternion_xyzw: Tuple[float, float, float, float],
    child_xyz: Tuple[float, float, float],
) -> Tuple[float, float, float]:
    """Express a child-frame point in the parent transform's world frame."""

    qx, qy, qz, qw = parent_quaternion_xyzw
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm < 1e-12:
        return (math.nan, math.nan, math.nan)
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    vx, vy, vz = child_xyz
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    rx = vx + qw * tx + qy * tz - qz * ty
    ry = vy + qw * ty + qz * tx - qx * tz
    rz = vz + qw * tz + qx * ty - qy * tx
    return (parent_xyz[0] + rx, parent_xyz[1] + ry, parent_xyz[2] + rz)


def level_from_quaternion(
    x: float, y: float, z: float, w: float
) -> Tuple[float, float, float, float]:
    """Return roll-like, pitch-like, tilt and body-up-z without Euler wrap.

    The tuple contains ``(roll_like, pitch_like, tilt, up_z)``.  The annotation
    deliberately stays broad for Python 3.10 compatibility with ROS 2 Humble.
    """

    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        return (math.nan, math.nan, math.nan, math.nan)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    body_z_x = 2.0 * (x * z + w * y)
    body_z_y = 2.0 * (y * z - w * x)
    up_z = max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))
    tilt = math.atan2(math.hypot(body_z_x, body_z_y), up_z)
    denom = max(0.15, abs(up_z))
    roll_like = math.atan2(-body_z_y, denom)
    pitch_like = math.atan2(body_z_x, denom)
    return roll_like, pitch_like, tilt, up_z


def body_velocity_from_world(
    vx_world: float, vy_world: float, yaw: float
) -> Tuple[float, float]:
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    return (
        cy * vx_world + sy * vy_world,
        -sy * vx_world + cy * vy_world,
    )


def route_coordinates(
    x: float,
    y: float,
    origin_x: float,
    origin_y: float,
    axis_x: float,
    axis_y: float,
) -> Tuple[float, float]:
    """Return signed along-route projection and lateral displacement."""

    dx = x - origin_x
    dy = y - origin_y
    return dx * axis_x + dy * axis_y, -dx * axis_y + dy * axis_x


def percentile(values: Sequence[float], quantile: float) -> Optional[float]:
    """Return a linearly interpolated quantile for finite values."""

    finite = sorted(float(value) for value in values if math.isfinite(float(value)))
    if not finite:
        return None
    q = max(0.0, min(1.0, float(quantile)))
    position = q * (len(finite) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return finite[lower]
    weight = position - lower
    return finite[lower] * (1.0 - weight) + finite[upper] * weight


def pose_kinematics(
    previous_xyz: Tuple[float, float, float],
    previous_quaternion_xyzw: Tuple[float, float, float, float],
    current_xyz: Tuple[float, float, float],
    current_quaternion_xyzw: Tuple[float, float, float, float],
    dt_sec: float,
) -> PoseKinematics:
    """Compute linear and angular speed from two ground-truth poses."""

    dt = float(dt_sec)
    if not math.isfinite(dt) or dt <= 0.0:
        raise ValueError("pose kinematics requires a positive finite dt")

    def _normalized(
        quaternion: Tuple[float, float, float, float]
    ) -> Tuple[float, float, float, float]:
        x, y, z, w = (float(value) for value in quaternion)
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if not math.isfinite(norm) or norm < 1e-12:
            raise ValueError("pose kinematics received an invalid quaternion")
        return x / norm, y / norm, z / norm, w / norm

    px, py, pz, pw = _normalized(previous_quaternion_xyzw)
    cx, cy, cz, cw = _normalized(current_quaternion_xyzw)
    # q_delta = conjugate(q_previous) * q_current.
    dx = pw * cx - px * cw - py * cz + pz * cy
    dy = pw * cy + px * cz - py * cw - pz * cx
    dz = pw * cz - px * cy + py * cx - pz * cw
    dw = pw * cw + px * cx + py * cy + pz * cz
    if dw < 0.0:
        dx, dy, dz, dw = -dx, -dy, -dz, -dw
    vector_norm = math.sqrt(dx * dx + dy * dy + dz * dz)
    angle = 2.0 * math.atan2(vector_norm, max(0.0, min(1.0, dw)))
    previous_yaw = yaw_from_quaternion(px, py, pz, pw)
    current_yaw = yaw_from_quaternion(cx, cy, cz, cw)
    return PoseKinematics(
        vx_world_mps=(float(current_xyz[0]) - float(previous_xyz[0])) / dt,
        vy_world_mps=(float(current_xyz[1]) - float(previous_xyz[1])) / dt,
        vz_world_mps=(float(current_xyz[2]) - float(previous_xyz[2])) / dt,
        yaw_rate_radps=wrap_angle(current_yaw - previous_yaw) / dt,
        angular_speed_radps=angle / dt,
    )


@dataclass
class _FootState:
    previous_position: Optional[Tuple[float, float, float]] = None
    previous_contact: bool = False
    slip_distance_m: float = 0.0
    swing_active: bool = False
    swing_peak_m: float = -math.inf
    swing_peaks_m: list[float] = field(default_factory=list)
    mismatch_sec: float = 0.0
    mismatch_streak_sec: float = 0.0
    mismatch_streak_max_sec: float = 0.0
    observed_sec: float = 0.0
    stance_episode_active: bool = False
    stance_episode_slip_m: float = 0.0
    stance_episode_slips_m: list[float] = field(default_factory=list)
    stance_speed_samples_mps: list[float] = field(default_factory=list)
    planned_swing_contact_sec: float = 0.0
    actual_contact_observed: bool = False
    actual_no_contact_observed: bool = False


class FootMetricsAccumulator:
    """Integrate stance slip, planned-swing clearance, and contact mismatch."""

    def __init__(
        self,
        legs: Sequence[str] = LEG_ORDER,
        *,
        ground_z_m: float = 0.0,
        foot_radius_m: float = 0.012,
    ) -> None:
        self._states = {leg: _FootState() for leg in legs}
        self._ground_z_m = float(ground_z_m)
        self._foot_radius_m = float(foot_radius_m)
        self._last_stamp_sec: Optional[float] = None

    def update(
        self,
        stamp_sec: float,
        positions: Mapping[str, Tuple[float, float, float]],
        actual_contacts: Mapping[str, bool],
        planned_stance: Mapping[str, bool],
    ) -> None:
        dt = 0.0
        if self._last_stamp_sec is not None:
            candidate = float(stamp_sec) - self._last_stamp_sec
            if 0.0 < candidate <= 0.25:
                dt = candidate
        self._last_stamp_sec = float(stamp_sec)

        for leg, state in self._states.items():
            position = positions.get(leg)
            if position is None:
                continue
            actual = bool(actual_contacts.get(leg, False))
            stance = bool(planned_stance.get(leg, False))
            state.actual_contact_observed |= actual
            state.actual_no_contact_observed |= not actual

            if (
                dt > 0.0
                and actual
                and state.previous_contact
                and state.previous_position is not None
            ):
                displacement = math.hypot(
                    position[0] - state.previous_position[0],
                    position[1] - state.previous_position[1],
                )
                state.slip_distance_m += displacement
                state.stance_episode_slip_m += displacement
                if dt > 0.0:
                    state.stance_speed_samples_mps.append(displacement / dt)

            if actual and not state.previous_contact:
                state.stance_episode_active = True
                state.stance_episode_slip_m = 0.0
            elif not actual and state.previous_contact and state.stance_episode_active:
                state.stance_episode_slips_m.append(state.stance_episode_slip_m)
                state.stance_episode_active = False
                state.stance_episode_slip_m = 0.0

            clearance = position[2] - self._ground_z_m - self._foot_radius_m
            if not stance:
                if not state.swing_active:
                    state.swing_active = True
                    state.swing_peak_m = clearance
                else:
                    state.swing_peak_m = max(state.swing_peak_m, clearance)
            elif state.swing_active:
                if math.isfinite(state.swing_peak_m):
                    state.swing_peaks_m.append(state.swing_peak_m)
                state.swing_active = False
                state.swing_peak_m = -math.inf

            if dt > 0.0:
                mismatch = actual != stance
                state.observed_sec += dt
                if not stance and actual:
                    state.planned_swing_contact_sec += dt
                if mismatch:
                    state.mismatch_sec += dt
                    state.mismatch_streak_sec += dt
                    state.mismatch_streak_max_sec = max(
                        state.mismatch_streak_max_sec,
                        state.mismatch_streak_sec,
                    )
                else:
                    state.mismatch_streak_sec = 0.0

            state.previous_position = position
            state.previous_contact = actual

    def finish(self) -> None:
        for state in self._states.values():
            if state.swing_active and math.isfinite(state.swing_peak_m):
                state.swing_peaks_m.append(state.swing_peak_m)
            state.swing_active = False
            if state.stance_episode_active:
                state.stance_episode_slips_m.append(state.stance_episode_slip_m)
            state.stance_episode_active = False

    def as_dict(self) -> dict:
        per_leg = {}
        total_slip = 0.0
        total_mismatch = 0.0
        total_observed = 0.0
        all_swing_peaks: list[float] = []
        for leg, state in self._states.items():
            peaks = list(state.swing_peaks_m)
            all_swing_peaks.extend(peaks)
            total_slip += state.slip_distance_m
            total_mismatch += state.mismatch_sec
            total_observed += state.observed_sec
            per_leg[leg] = {
                "stance_slip_distance_m": state.slip_distance_m,
                "stance_episode_count": len(state.stance_episode_slips_m),
                "stance_episode_slip_max_m": (
                    max(state.stance_episode_slips_m)
                    if state.stance_episode_slips_m
                    else None
                ),
                "stance_speed_p95_mps": percentile(
                    state.stance_speed_samples_mps, 0.95
                ),
                "swing_count": len(peaks),
                "swing_peak_clearance_min_m": min(peaks) if peaks else None,
                "swing_peak_clearance_mean_m": (
                    sum(peaks) / len(peaks) if peaks else None
                ),
                "swing_peak_clearance_max_m": max(peaks) if peaks else None,
                "contact_mismatch_sec": state.mismatch_sec,
                "contact_mismatch_ratio": (
                    state.mismatch_sec / state.observed_sec
                    if state.observed_sec > 0.0
                    else None
                ),
                "contact_mismatch_streak_max_sec": state.mismatch_streak_max_sec,
                "planned_swing_contact_sec": state.planned_swing_contact_sec,
                "actual_contact_observed": state.actual_contact_observed,
                "actual_no_contact_observed": state.actual_no_contact_observed,
            }
        return {
            "per_leg": per_leg,
            "stance_slip_distance_total_m": total_slip,
            "swing_peak_clearance_min_m": (
                min(all_swing_peaks) if all_swing_peaks else None
            ),
            "contact_mismatch_ratio": (
                total_mismatch / total_observed if total_observed > 0.0 else None
            ),
        }


def atomic_write_json(path: str | Path, payload: object) -> None:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        mode="w",
        encoding="utf-8",
        dir=target.parent,
        prefix=f".{target.name}.",
        suffix=".tmp",
        delete=False,
    ) as handle:
        json.dump(payload, handle, indent=2, sort_keys=True, ensure_ascii=False)
        handle.write("\n")
        temp_name = handle.name
    os.replace(temp_name, target)


def sha256_text(value: str) -> str:
    return hashlib.sha256(value.encode("utf-8")).hexdigest()


def sha256_file(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def finite_numbers(values: Iterable[object]) -> list[float]:
    result = []
    for value in values:
        if isinstance(value, (int, float)) and not isinstance(value, bool):
            numeric = float(value)
            if math.isfinite(numeric):
                result.append(numeric)
    return result
