import numpy as np

from dog2_gait_planner.swing_target_node import (
    body_z_for_world_height,
    contact_event_is_active,
    contact_synchronized_swing_phase,
    foothold_velocity_offset,
    swing_bezier,
    touchdown_height,
    world_height,
)


def test_swing_bezier_starts_and_ends_at_footholds():
    p0 = np.array([0.0, 0.0, -0.25])
    pf = np.array([0.1, 0.0, -0.25])

    start, start_vel = swing_bezier(p0, pf, 0.0, 0.4, 0.08)
    end, end_vel = swing_bezier(p0, pf, 1.0, 0.4, 0.08)
    mid, _ = swing_bezier(p0, pf, 0.5, 0.4, 0.08)

    assert np.allclose(start, p0)
    assert np.allclose(end, pf)
    assert np.allclose(start_vel, np.zeros(3))
    assert np.allclose(end_vel, np.zeros(3))
    assert np.isclose(mid[2], p0[2] + 0.08)


def test_swing_bezier_clears_ground_before_horizontal_transfer():
    p0 = np.array([0.03, -0.14, -0.18])
    pf = np.array([0.00, -0.11, -0.20])

    lift, lift_vel = swing_bezier(p0, pf, 0.20, 0.5, 0.05)
    transfer, _ = swing_bezier(p0, pf, 0.50, 0.5, 0.05)
    lower, lower_vel = swing_bezier(p0, pf, 0.85, 0.5, 0.05)

    assert np.allclose(lift[:2], p0[:2])
    assert lift[2] > p0[2]
    assert np.allclose(lift_vel[:2], np.zeros(2))
    assert np.isclose(transfer[2], p0[2] + 0.05)
    assert np.allclose(transfer[:2], 0.5 * (p0[:2] + pf[:2]))
    assert np.allclose(lower[:2], pf[:2])
    assert lower_vel[2] < 0.0


def test_swing_target_payload_shape_contract():
    mask = np.zeros(4)
    pos = np.zeros((4, 3))
    vel = np.zeros((4, 3))

    payload = mask.tolist() + pos.reshape(-1).tolist() + vel.reshape(-1).tolist()

    assert len(payload) == 28


def test_crawl_foothold_leads_command_within_workspace_limit():
    offset = foothold_velocity_offset(
        np.array([-0.05, 0.0]),
        np.array([-0.05, 0.0]),
        gait="crawl",
        raibert_k=0.03,
        crawl_lead_sec=1.2,
        maximum=0.06,
    )

    assert np.allclose(offset, [-0.06, 0.0])


def test_trot_foothold_retains_feedback_only():
    offset = foothold_velocity_offset(
        np.array([-0.03, 0.01]),
        np.array([-0.05, 0.0]),
        gait="trot",
        raibert_k=0.03,
        crawl_lead_sec=1.2,
        maximum=0.06,
    )

    assert np.allclose(offset, [0.0006, 0.0003])


def test_touchdown_search_is_relative_to_measured_liftoff_height():
    assert np.isclose(touchdown_height(-0.177, 0.005), -0.182)


def test_contact_event_timeout_detects_gazebo_silent_release():
    assert contact_event_is_active(1.19, True, 1.0, 0.2)
    assert not contact_event_is_active(1.21, True, 1.0, 0.2)
    assert not contact_event_is_active(1.05, False, 1.0, 0.2)


def test_touchdown_height_stays_fixed_in_world_as_body_rises_and_tilts():
    liftoff_row = np.array([0.0, 0.0, 1.0])
    liftoff_foot = np.array([-0.156, -0.118, -0.175])
    target_world_z = world_height(0.195, liftoff_row, liftoff_foot) - 0.005

    touchdown_row = np.array([0.114, -0.051, 0.992])
    touchdown_xy = np.array([-0.092, -0.118])
    touchdown_z = body_z_for_world_height(
        target_world_z,
        0.212,
        touchdown_row,
        touchdown_xy,
        -0.180,
    )
    touchdown = np.array([touchdown_xy[0], touchdown_xy[1], touchdown_z])

    assert touchdown_z < -0.19
    assert np.isclose(
        world_height(0.212, touchdown_row, touchdown),
        target_world_z,
    )


def test_contact_synchronized_phase_holds_lift_until_release():
    assert np.isclose(
        contact_synchronized_swing_phase(0.4, 0.0, 0.5, None),
        0.3,
    )
    assert np.isclose(
        contact_synchronized_swing_phase(0.2, 0.0, 0.5, 0.2),
        0.3,
    )
    assert np.isclose(
        contact_synchronized_swing_phase(0.45, 0.0, 0.5, 0.2),
        0.65,
    )
    assert np.isclose(
        contact_synchronized_swing_phase(0.7, 0.0, 0.5, 0.2),
        1.0,
    )


def test_contact_synchronized_phase_does_not_jump_on_early_release():
    assert np.isclose(
        contact_synchronized_swing_phase(0.05, 0.0, 0.5, 0.05),
        0.1,
    )
    assert np.isclose(
        contact_synchronized_swing_phase(0.55, 0.0, 0.5, 0.05),
        1.0,
    )


def test_contact_synchronization_disabled_matches_nominal_timing():
    assert np.isclose(
        contact_synchronized_swing_phase(0.25, 0.0, 0.5, 0.0),
        0.5,
    )
