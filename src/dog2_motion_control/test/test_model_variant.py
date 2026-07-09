"""Tests for dog2_motion_control.model_variant."""

import pytest

from dog2_motion_control.model_variant import (
    get_leg_parameters,
    get_urdf_xacro_filename,
    normalize_model_variant,
    reload_leg_parameter_joint_limits,
)


def test_normalize_real():
    assert normalize_model_variant("real") == "real"
    assert normalize_model_variant("Real") == "real"
    assert normalize_model_variant("  real  ") == "real"


def test_normalize_symmetric():
    assert normalize_model_variant("symmetric") == "symmetric"


def test_normalize_invalid():
    with pytest.raises(ValueError):
        normalize_model_variant("invalid")
    with pytest.raises(ValueError):
        normalize_model_variant("")


def test_get_urdf_xacro_filename():
    assert get_urdf_xacro_filename("real") == "dog2.urdf.xacro"
    assert get_urdf_xacro_filename("symmetric") == "dog2_symmetric.urdf.xacro"


def test_get_leg_parameters_real():
    params = get_leg_parameters("real")
    assert sorted(params) == ["lf", "lh", "rf", "rh"]


def test_get_leg_parameters_symmetric():
    import numpy as np

    params = get_leg_parameters("symmetric")
    assert sorted(params) == ["lf", "lh", "rf", "rh"]
    assert np.allclose(params["rf"].base_position, [-0.122125, 0.06, 0.0])
    assert np.allclose(
        params["rf"].foot_tip_offset_tibia,
        params["rh"].foot_tip_offset_tibia,
    )


def test_reload_joint_limits_for_variant_uses_selected_urdf():
    assert reload_leg_parameter_joint_limits("real").endswith("dog2.urdf.xacro")
    assert reload_leg_parameter_joint_limits("symmetric").endswith("dog2_symmetric.urdf.xacro")
