#!/usr/bin/env python3
"""Tests for bootstrap stance-push walking convention."""

from __future__ import annotations

import importlib.util
import math
from pathlib import Path
import sys


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_PATH = PROJECT_ROOT / "scripts" / "05_trot_forward_raibert.py"


def load_stage5_module():
    spec = importlib.util.spec_from_file_location("stage5_trot_forward_raibert", SCRIPT_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Cannot load {SCRIPT_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


stage5 = load_stage5_module()


# This test protects the bootstrap walking convention: positive desired body
# velocity commands a backward stance-foot target, which creates forward body
# reaction through contact.
def test_positive_forward_velocity_pushes_stance_foot_backward():
    push_body, push_norm = stage5.compute_stance_push_body(
        desired_velocity_body=(0.04, 0.0, 0.0),
        stance_elapsed=0.5,
        gain=0.85,
        max_push_m=0.035,
        ramp_time=0.35,
    )

    assert push_body[0] < 0.0
    assert push_body[1] == 0.0
    assert push_body[2] == 0.0
    assert push_norm > 0.0


def test_stance_push_is_clipped_to_max_norm():
    push_body, push_norm = stage5.compute_stance_push_body(
        desired_velocity_body=(1.0, 1.0, 0.0),
        stance_elapsed=10.0,
        gain=1.0,
        max_push_m=0.035,
        ramp_time=0.0,
    )

    assert math.hypot(push_body[0], push_body[1]) <= 0.035 + 1e-12
    assert push_norm <= 0.035 + 1e-12


def test_stance_push_ramps_in_smoothly():
    full_push, _ = stage5.compute_stance_push_body(
        desired_velocity_body=(0.04, 0.0, 0.0),
        stance_elapsed=0.35,
        gain=1.0,
        max_push_m=1.0,
        ramp_time=0.35,
    )
    early_push, _ = stage5.compute_stance_push_body(
        desired_velocity_body=(0.04, 0.0, 0.0),
        stance_elapsed=0.05,
        gain=1.0,
        max_push_m=1.0,
        ramp_time=0.35,
    )

    assert abs(early_push[0]) < abs(full_push[0])


def test_zero_desired_velocity_produces_zero_push():
    push_body, push_norm = stage5.compute_stance_push_body(
        desired_velocity_body=(0.0, 0.0, 0.0),
        stance_elapsed=1.0,
        gain=1.0,
        max_push_m=0.035,
        ramp_time=0.35,
    )

    assert push_body == (0.0, 0.0, 0.0)
    assert push_norm == 0.0


def test_stance_push_never_commands_vertical_offset():
    push_body, _ = stage5.compute_stance_push_body(
        desired_velocity_body=(0.04, -0.02, 99.0),
        stance_elapsed=0.5,
        gain=1.0,
        max_push_m=0.035,
        ramp_time=0.35,
    )

    assert push_body[2] == 0.0


def test_stance_drive_stops_when_forward_speed_is_reached():
    drive = stage5.stance_drive_velocity_body(
        desired_velocity_body=(0.035, 0.0, 0.0),
        current_velocity_body=(0.040, 0.0, 0.0),
    )

    assert drive == (0.0, 0.0, 0.0)


def test_stance_drive_uses_remaining_forward_velocity_error():
    drive = stage5.stance_drive_velocity_body(
        desired_velocity_body=(0.035, 0.0, 0.0),
        current_velocity_body=(0.010, 0.0, 0.0),
    )

    assert 0.0 < drive[0] < 0.035
    assert drive[1] == 0.0
    assert drive[2] == 0.0


def test_stance_drive_velocity_can_be_capped():
    drive = stage5.stance_drive_velocity_body(
        desired_velocity_body=(0.10, 0.10, 0.0),
        current_velocity_body=(0.0, 0.0, 0.0),
        max_drive_velocity_mps=0.02,
    )

    assert math.hypot(drive[0], drive[1]) <= 0.02 + 1e-12
