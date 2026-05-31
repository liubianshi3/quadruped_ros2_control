#!/usr/bin/env python3
"""Phase-based quadruped gait scheduler.

This module only decides stance/swing timing.  It does not compute footholds,
IK, or torque commands, and it never includes rail joints in any state.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Mapping, Tuple


LEG_ORDER = ("FL", "FR", "RL", "RR")


def _float(value: Any, default: float) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


@dataclass(frozen=True)
class GaitSchedulerConfig:
    gait_type: str = "trot"
    period: float = 1.0
    duty_factor: float = 0.70
    initial_stand_time: float = 2.0
    transition_duration: float = 0.50
    min_stance_legs: int = 2
    phase_offset: Tuple[Tuple[str, float], ...] = (
        ("FL", 0.0),
        ("RR", 0.0),
        ("FR", 0.5),
        ("RL", 0.5),
    )


@dataclass(frozen=True)
class LegPhaseState:
    phase: float
    state: str
    swing_phase: float

    def to_dict(self) -> Dict[str, Any]:
        return {
            "phase": self.phase,
            "state": self.state,
            "swing_phase": self.swing_phase,
        }


class GaitScheduler:
    """Simple phase scheduler with an initial all-stance stabilization period."""

    def __init__(self, config: GaitSchedulerConfig):
        self.config = config
        self._phase_offset = dict(config.phase_offset)
        self._validate_config()

    @classmethod
    def from_config_dict(cls, gait_config: Mapping[str, Any]) -> "GaitScheduler":
        gait = gait_config.get("gait", {}) or {}
        phase_offset_cfg = gait.get("phase_offset", {}) or {}
        offsets = []
        for leg_name in LEG_ORDER:
            offsets.append((leg_name, _float(phase_offset_cfg.get(leg_name), 0.0)))
        return cls(
            GaitSchedulerConfig(
                gait_type=str(gait.get("type", "trot")),
                period=max(_float(gait.get("period"), 1.0), 1e-6),
                duty_factor=min(max(_float(gait.get("duty_factor"), 0.70), 1e-3), 0.999),
                initial_stand_time=max(_float(gait.get("initial_stand_time"), 2.0), 0.0),
                transition_duration=max(_float(gait.get("transition_duration"), 0.50), 0.0),
                min_stance_legs=max(int(_float(gait.get("min_stance_legs"), 2)), 1),
                phase_offset=tuple(offsets),
            )
        )

    def _validate_config(self) -> None:
        gait_type = self.config.gait_type.lower()
        if gait_type not in {"trot", "crawl", "walk"}:
            raise ValueError(
                f"Supported staged gaits are 'trot', 'crawl', and 'walk', got {self.config.gait_type!r}."
            )
        missing = [leg for leg in LEG_ORDER if leg not in self._phase_offset]
        if missing:
            raise ValueError(f"Missing phase_offset entries for legs: {missing}")
        if gait_type == "trot":
            if self.config.duty_factor <= 0.5:
                raise ValueError("Trot duty_factor must stay above 0.5 so at least two legs remain in stance.")
            if abs((self._phase_offset["FL"] - self._phase_offset["RR"]) % 1.0) > 1e-9:
                raise ValueError("FL and RR must stay in phase for trot.")
            if abs((self._phase_offset["FR"] - self._phase_offset["RL"]) % 1.0) > 1e-9:
                raise ValueError("FR and RL must stay in phase for trot.")
            diff = (self._phase_offset["FR"] - self._phase_offset["FL"]) % 1.0
            if abs(diff - 0.5) > 1e-9:
                raise ValueError("Diagonal groups must differ by 0.5 phase for trot.")
        else:
            if self.config.min_stance_legs < 3:
                raise ValueError("Crawl/walk bring-up must require at least three stance legs.")
            if self.config.duty_factor <= 0.75:
                raise ValueError("Crawl/walk duty_factor must stay above 0.75 to avoid two-leg support gaps.")

        # Validate the configured phase offsets over one cycle.  This keeps the
        # scheduler generic while still refusing phase tables that briefly drop
        # below the requested support polygon size.
        for i in range(400):
            phase_time = (i / 400.0) * self.config.period
            states = [self.leg_state(leg_name, self.config.initial_stand_time + phase_time) for leg_name in LEG_ORDER]
            stance_count = sum(1 for state in states if state.state == "stance")
            if stance_count < self.config.min_stance_legs:
                raise ValueError(
                    "Configured gait drops below "
                    f"{self.config.min_stance_legs} stance legs near phase {i / 400.0:.3f}."
                )

    @property
    def swing_time(self) -> float:
        return self.config.period * (1.0 - self.config.duty_factor)

    @property
    def stance_time(self) -> float:
        return self.config.period * self.config.duty_factor

    def transition_blend(self, time_s: float) -> float:
        time_s = max(float(time_s), 0.0)
        if time_s < self.config.initial_stand_time:
            return 0.0
        if self.config.transition_duration <= 0.0:
            return 1.0
        return min((time_s - self.config.initial_stand_time) / self.config.transition_duration, 1.0)

    def leg_phase(self, leg_name: str, time_s: float) -> float:
        if leg_name not in self._phase_offset:
            raise KeyError(f"Unknown leg {leg_name}")
        time_s = max(float(time_s), 0.0)
        if time_s < self.config.initial_stand_time:
            return float(self._phase_offset[leg_name] % 1.0)
        gait_time = time_s - self.config.initial_stand_time
        return float((gait_time / self.config.period + self._phase_offset[leg_name]) % 1.0)

    def leg_state(self, leg_name: str, time_s: float) -> LegPhaseState:
        phase = self.leg_phase(leg_name, time_s)
        if time_s < self.config.initial_stand_time:
            return LegPhaseState(phase=phase, state="stance", swing_phase=0.0)
        if phase < self.config.duty_factor:
            return LegPhaseState(phase=phase, state="stance", swing_phase=0.0)
        swing_phase = (phase - self.config.duty_factor) / max(1.0 - self.config.duty_factor, 1e-9)
        return LegPhaseState(phase=phase, state="swing", swing_phase=float(min(max(swing_phase, 0.0), 1.0)))

    def update(self, time_s: float) -> Dict[str, Dict[str, Any]]:
        states = {leg_name: self.leg_state(leg_name, time_s) for leg_name in LEG_ORDER}
        stance_count = sum(1 for state in states.values() if state.state == "stance")
        if stance_count < self.config.min_stance_legs:
            raise RuntimeError(f"Invalid gait state at t={time_s:.3f}: only {stance_count} stance legs.")
        return {leg_name: state.to_dict() for leg_name, state in states.items()}


__all__ = ["GaitScheduler", "GaitSchedulerConfig", "LegPhaseState", "LEG_ORDER"]
