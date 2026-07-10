"""Pure metric helpers for Dog2 headless locomotion acceptance."""

from __future__ import annotations

import json
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

            if (
                dt > 0.0
                and actual
                and state.previous_contact
                and state.previous_position is not None
            ):
                state.slip_distance_m += math.hypot(
                    position[0] - state.previous_position[0],
                    position[1] - state.previous_position[1],
                )

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


def finite_numbers(values: Iterable[object]) -> list[float]:
    result = []
    for value in values:
        if isinstance(value, (int, float)) and not isinstance(value, bool):
            numeric = float(value)
            if math.isfinite(numeric):
                result.append(numeric)
    return result
