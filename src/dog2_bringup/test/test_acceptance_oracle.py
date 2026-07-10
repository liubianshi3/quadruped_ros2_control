import math
from dataclasses import replace

import pytest

from dog2_bringup.acceptance_metrics import JointLimit
from dog2_bringup.acceptance_oracle import (
    PROTOCOL_STAGE_ORDER,
    MotionGateLimits,
    MotionObservation,
    StabilityLimits,
    assess_turn,
    completion_outcome,
    evaluate_joint_hard_gates,
    evaluate_motion_hard_gates,
    is_stable,
    is_valid_stage_transition,
    latch_illegal_contact,
    stand_drift_failure,
)


def _observation() -> MotionObservation:
    return MotionObservation(
        x_m=0.0,
        y_m=0.0,
        z_m=0.2,
        yaw_rad=0.0,
        tilt_rad=0.05,
        up_z=0.998,
        vx_body_mps=0.0,
        vy_body_mps=0.0,
        vz_mps=0.0,
        yaw_rate_radps=0.0,
        angular_speed_radps=0.0,
        lateral_error_m=0.0,
        rail_error_m=0.0,
        base_contact=False,
    )


def _limits() -> MotionGateLimits:
    return MotionGateLimits(
        max_tilt_rad=0.55,
        min_up_z=math.cos(0.55),
        body_height_min_m=0.12,
        body_height_max_m=0.4,
        max_rail_error_m=0.005,
        corridor_half_width_m=0.08,
    )


@pytest.mark.parametrize(
    ("changed", "code"),
    [
        ({"tilt_rad": 0.56}, "TILT_LIMIT"),
        ({"up_z": 0.1}, "UP_Z_LIMIT"),
        ({"z_m": 0.1}, "BODY_HEIGHT_LIMIT"),
        ({"rail_error_m": 0.006}, "RAIL_LOCK_ERROR"),
        ({"base_contact": True}, "BASE_CONTACT"),
        ({"tibia_contact_legs": ("lf",)}, "TIBIA_CONTACT"),
        ({"lateral_error_m": 0.09}, "ROUTE_CORRIDOR"),
        ({"vx_body_mps": math.nan}, "NONFINITE_STATE"),
    ],
)
def test_motion_fault_injection_has_exact_failure_code(changed, code) -> None:
    failure = evaluate_motion_hard_gates(
        replace(_observation(), **changed),
        _limits(),
        enforce_route_corridor=True,
    )
    assert failure is not None
    assert failure.code == code


def test_route_corridor_is_not_applied_during_turn() -> None:
    failure = evaluate_motion_hard_gates(
        replace(_observation(), lateral_error_m=0.2),
        _limits(),
        enforce_route_corridor=False,
    )
    assert failure is None


def test_stability_rejects_vertical_and_three_axis_angular_motion() -> None:
    stability = StabilityLimits(0.05, 0.05, 0.15)
    assert is_stable(_observation(), _limits(), stability)
    assert not is_stable(
        replace(_observation(), vz_mps=0.051), _limits(), stability
    )
    assert not is_stable(
        replace(_observation(), angular_speed_radps=0.151), _limits(), stability
    )


def test_stand_drift_and_turn_final_error_are_net_pose_checks() -> None:
    assert stand_drift_failure((0.0, 0.0), (0.019, 0.0), 0.02) is None
    assert (
        stand_drift_failure((0.0, 0.0), (0.021, 0.0), 0.02).code
        == "STAND_DRIFT"
    )

    turn = assess_turn(
        (0.0, 0.0),
        (0.01, 0.0),
        0.0,
        math.pi / 2.0 + 0.2,
        0.15,
        math.pi / 2.0,
    )
    assert turn.signed_progress_rad == pytest.approx(math.pi / 2.0 + 0.2)
    assert turn.final_error_rad == pytest.approx(0.2)
    assert turn.translation_m == pytest.approx(0.01)


def test_joint_oracle_rejects_missing_and_over_limit_channels() -> None:
    limits = {
        "joint": JointLimit("joint", "revolute", -1.0, 1.0, 2.0, 3.0)
    }
    missing = evaluate_joint_hard_gates(
        limits, {}, {}, {}, required_effort_names=("joint",)
    )
    assert missing.failure is not None
    assert missing.failure.code == "MISSING_JOINT_STATE"
    assert missing.failure.infrastructure

    position = evaluate_joint_hard_gates(
        limits,
        {"joint": 1.1},
        {"joint": 0.0},
        {"joint": 0.0},
        required_effort_names=("joint",),
    )
    assert position.failure.code == "JOINT_POSITION_LIMIT"

    velocity = evaluate_joint_hard_gates(
        limits,
        {"joint": 0.0},
        {"joint": 2.1},
        {"joint": 0.0},
        required_effort_names=("joint",),
    )
    assert velocity.failure.code == "JOINT_VELOCITY_LIMIT"

    effort = evaluate_joint_hard_gates(
        limits,
        {"joint": 0.0},
        {"joint": 0.0},
        {"joint": 3.1},
        required_effort_names=("joint",),
    )
    assert effort.failure.code == "JOINT_EFFORT_LIMIT"


def test_uncalibrated_completion_can_never_claim_formal_pass() -> None:
    assert completion_outcome("provisional", []) == (
        "PASS_SAFETY_ROUTE_PROVISIONAL",
        False,
        True,
    )
    assert completion_outcome("calibrated", []) == (
        "PASS_LOCOMOTION_BASELINE",
        True,
        False,
    )


def test_protocol_stage_machine_allows_only_the_fixed_sequence() -> None:
    for current, requested in zip(PROTOCOL_STAGE_ORDER, PROTOCOL_STAGE_ORDER[1:]):
        assert is_valid_stage_transition(current, requested)
    assert not is_valid_stage_transition("STAND", "RETURN")
    assert not is_valid_stage_transition("TURN", "OUTBOUND")
    assert not is_valid_stage_transition("FINAL_STOP", "WAIT_READY")


def test_illegal_contact_short_pulse_remains_latched() -> None:
    event = latch_illegal_contact(
        None,
        active=True,
        scored=True,
        kind="base",
        leg="base",
        force_n=0.0,
        wrench_available=False,
        wall_time_sec=1.0,
        sim_time_sec=2.0,
        stage="OUTBOUND",
    )
    event = latch_illegal_contact(
        event,
        active=False,
        scored=True,
        kind="base",
        leg="base",
        force_n=0.0,
        wrench_available=False,
        wall_time_sec=1.1,
        sim_time_sec=2.1,
        stage="OUTBOUND",
    )
    assert event is not None
    assert event["kind"] == "base"
    assert event["force_n"] is None
