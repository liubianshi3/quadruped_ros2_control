import numpy as np

from dog2_bringup.wbc_effort_mux import WBCEffortMux, _blend_effort


def test_compose_effort_command_keeps_controller_joint_order():
    joint = np.tile(np.array([0.0, -7.029, -9.362]), 4)
    rail = np.zeros(4)

    assert WBCEffortMux._compose_effort_command(joint, rail) == [
        0.0,
        0.0,
        -7.029,
        -9.362,
        0.0,
        0.0,
        -7.029,
        -9.362,
        0.0,
        0.0,
        -7.029,
        -9.362,
        0.0,
        0.0,
        -7.029,
        -9.362,
    ]


def test_blend_effort_is_linear_and_clamped():
    hold = np.zeros(4)
    target = np.array([10.0, -10.0, 4.0, 2.0])

    np.testing.assert_allclose(_blend_effort(hold, target, 0.25), [2.5, -2.5, 1.0, 0.5])
    np.testing.assert_allclose(_blend_effort(hold, target, -1.0), hold)
    np.testing.assert_allclose(_blend_effort(hold, target, 2.0), target)
