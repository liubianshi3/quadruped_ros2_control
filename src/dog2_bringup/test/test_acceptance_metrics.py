import math

import pytest

from dog2_bringup.acceptance_metrics import (
    FootMetricsAccumulator,
    ScalarStats,
    body_velocity_from_world,
    foot_contact_startup_evidence_missing,
    level_from_quaternion,
    parse_robot_model_metadata,
    parse_joint_limits,
    percentile,
    pose_kinematics,
    route_coordinates,
    transform_point,
)


def test_contact_health_requires_per_foot_startup_evidence() -> None:
    missing = foot_contact_startup_evidence_missing(
        message_seen={"lf": False, "lh": True, "rh": True, "rf": True},
        active_seen={"lf": False, "lh": False, "rh": True, "rf": True},
    )

    assert missing == [
        "foot_contact_lf_no_messages",
        "foot_contact_lh_never_active",
    ]


def test_contact_health_accepts_silence_after_startup_evidence() -> None:
    # Message age is intentionally absent from this event-stream contract:
    # after initial proof, silence means no contact rather than failed health.
    assert (
        foot_contact_startup_evidence_missing(
            message_seen={leg: True for leg in ("lf", "lh", "rh", "rf")},
            active_seen={leg: True for leg in ("lf", "lh", "rh", "rf")},
        )
        == []
    )


def test_scalar_stats_reports_population_values() -> None:
    stats = ScalarStats()
    for value in (1.0, 2.0, 3.0):
        stats.update(value)

    result = stats.as_dict()
    assert result["count"] == 3
    assert result["mean"] == pytest.approx(2.0)
    assert result["std"] == pytest.approx(math.sqrt(2.0 / 3.0))
    assert result["rms"] == pytest.approx(math.sqrt(14.0 / 3.0))
    assert result["min"] == 1.0
    assert result["max"] == 3.0


def test_level_metric_is_wrap_safe() -> None:
    _, _, tilt, up_z = level_from_quaternion(0.0, 0.0, 0.0, 1.0)
    assert tilt == pytest.approx(0.0)
    assert up_z == pytest.approx(1.0)

    _, _, inverted_tilt, inverted_up_z = level_from_quaternion(
        1.0, 0.0, 0.0, 0.0
    )
    assert inverted_tilt == pytest.approx(math.pi)
    assert inverted_up_z == pytest.approx(-1.0)


def test_body_velocity_and_route_projection_use_body_heading() -> None:
    vx_body, vy_body = body_velocity_from_world(0.0, 1.0, math.pi / 2.0)
    assert vx_body == pytest.approx(1.0)
    assert vy_body == pytest.approx(0.0)

    along, lateral = route_coordinates(
        x=-1.0,
        y=0.2,
        origin_x=0.0,
        origin_y=0.0,
        axis_x=-1.0,
        axis_y=0.0,
    )
    assert along == pytest.approx(1.0)
    assert lateral == pytest.approx(-0.2)


def test_transform_point_applies_parent_rotation_and_translation() -> None:
    half_angle = math.pi / 4.0
    point = transform_point(
        (1.0, 2.0, 3.0),
        (0.0, 0.0, math.sin(half_angle), math.cos(half_angle)),
        (0.1, 0.0, -0.2),
    )

    assert point == pytest.approx((1.0, 2.1, 2.8))


def test_parse_joint_limits_reads_physical_hard_limits() -> None:
    limits = parse_joint_limits(
        """
        <robot name="test">
          <joint name="rail" type="prismatic">
            <parent link="a"/><child link="b"/>
            <limit lower="-0.1" upper="0.2" velocity="3" effort="40"/>
          </joint>
          <joint name="fixed" type="fixed">
            <parent link="b"/><child link="c"/>
          </joint>
        </robot>
        """
    )
    assert set(limits) == {"rail"}
    assert limits["rail"].lower == pytest.approx(-0.1)
    assert limits["rail"].upper == pytest.approx(0.2)
    assert limits["rail"].velocity == pytest.approx(3.0)
    assert limits["rail"].effort == pytest.approx(40.0)


def test_robot_metadata_comes_from_exact_urdf() -> None:
    links = "".join(
        f"""
        <link name="{leg}_foot_link">
          <inertial><mass value="0.05"/></inertial>
          <collision><geometry><sphere radius="0.02"/></geometry></collision>
        </link>
        """
        for leg in ("lf", "lh", "rh", "rf")
    )
    metadata = parse_robot_model_metadata(
        f"""
        <robot name="test">
          <link name="base_link">
            <inertial><mass value="6.0"/></inertial>
            <collision><geometry><box size="0.342 0.160 0.1"/></geometry></collision>
          </link>
          {links}
        </robot>
        """
    )
    assert metadata.total_mass_kg == pytest.approx(6.2)
    assert metadata.body_length_m == pytest.approx(0.342)
    assert metadata.body_width_m == pytest.approx(0.160)
    assert metadata.foot_radius_m == pytest.approx(0.02)


def test_pose_kinematics_uses_pose_difference_not_reported_twist() -> None:
    half_quarter_turn = math.pi / 4.0
    result = pose_kinematics(
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
        (0.5, -0.25, 0.1),
        (0.0, 0.0, math.sin(half_quarter_turn), math.cos(half_quarter_turn)),
        0.5,
    )
    assert result.vx_world_mps == pytest.approx(1.0)
    assert result.vy_world_mps == pytest.approx(-0.5)
    assert result.vz_world_mps == pytest.approx(0.2)
    assert result.yaw_rate_radps == pytest.approx(math.pi)
    assert result.angular_speed_radps == pytest.approx(math.pi)


def test_percentile_interpolates_finite_samples() -> None:
    assert percentile([0.0, 1.0, 2.0], 0.75) == pytest.approx(1.5)
    assert percentile([], 0.95) is None


def test_foot_metrics_integrate_stance_slip_and_swing_peak() -> None:
    tracker = FootMetricsAccumulator(legs=("lf",), foot_radius_m=0.012)
    tracker.update(
        0.0,
        {"lf": (0.0, 0.0, 0.012)},
        {"lf": True},
        {"lf": True},
    )
    tracker.update(
        0.1,
        {"lf": (0.01, 0.0, 0.012)},
        {"lf": True},
        {"lf": True},
    )
    tracker.update(
        0.2,
        {"lf": (0.01, 0.0, 0.032)},
        {"lf": False},
        {"lf": False},
    )
    tracker.update(
        0.3,
        {"lf": (0.01, 0.0, 0.052)},
        {"lf": False},
        {"lf": False},
    )
    tracker.update(
        0.4,
        {"lf": (0.01, 0.0, 0.012)},
        {"lf": True},
        {"lf": True},
    )
    tracker.finish()

    result = tracker.as_dict()
    assert result["stance_slip_distance_total_m"] == pytest.approx(0.01)
    assert result["per_leg"]["lf"]["stance_episode_count"] == 2
    assert result["per_leg"]["lf"]["stance_speed_p95_mps"] == pytest.approx(0.1)
    assert result["swing_peak_clearance_min_m"] == pytest.approx(0.04)
    assert result["contact_mismatch_ratio"] == pytest.approx(0.0)
    assert result["per_leg"]["lf"]["actual_contact_observed"] is True
    assert result["per_leg"]["lf"]["actual_no_contact_observed"] is True
