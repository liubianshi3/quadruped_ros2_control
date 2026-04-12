"""Verify motion-control joint limits stay synchronized with dog2 xacro."""

from dog2_motion_control.leg_parameters import LEG_PARAMETERS
from dog2_motion_control.urdf_joint_limits import load_dog2_urdf_joint_limits


def test_leg_parameters_limits_match_urdf():
    urdf_limits = load_dog2_urdf_joint_limits()

    for leg_id, params in LEG_PARAMETERS.items():
        assert params.joint_limits["rail"] == urdf_limits.rail_by_leg[leg_id]
        for role in ("coxa", "femur", "tibia"):
            assert params.joint_limits[role] == urdf_limits.revolute_by_role[role]
