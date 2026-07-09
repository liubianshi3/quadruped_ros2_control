import numpy as np

from dog2_gait_planner.swing_target_node import swing_bezier


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


def test_swing_target_payload_shape_contract():
    mask = np.zeros(4)
    pos = np.zeros((4, 3))
    vel = np.zeros((4, 3))

    payload = mask.tolist() + pos.reshape(-1).tolist() + vel.reshape(-1).tolist()

    assert len(payload) == 28
