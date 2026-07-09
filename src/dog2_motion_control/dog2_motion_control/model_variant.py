"""
Dog2 model variant selector.

Provides a single entry point for choosing between the real (asymmetric) and
symmetric simulation models.  Every consumer that needs a different URDF or
LegParameters dict should go through this module.
"""

from __future__ import annotations

ALLOWED_VARIANTS = ("real", "symmetric")


def normalize_model_variant(value: str) -> str:
    """Canonicalize a user-supplied variant name.

    Args:
        value: raw string, e.g.  "real" / "Real" / "symmetric"

    Returns:
        Lower-cased canonical variant name ("real" or "symmetric").

    Raises:
        ValueError: the value is not one of the allowed variants.
    """
    cleaned = str(value).strip().lower()
    if cleaned not in ALLOWED_VARIANTS:
        raise ValueError(
            f"Invalid model_variant: {value!r}. Allowed: {ALLOWED_VARIANTS}"
        )
    return cleaned


def get_urdf_xacro_filename(model_variant: str) -> str:
    """Return the xacro filename for the given variant."""
    v = normalize_model_variant(model_variant)
    if v == "real":
        return "dog2.urdf.xacro"
    return "dog2_symmetric.urdf.xacro"


def get_leg_parameters(model_variant: str):
    """Return the LEG_PARAMETERS dict for the given variant.

    Returns:
        real    -> dog2_motion_control.leg_parameters.LEG_PARAMETERS
        symmetric -> dog2_motion_control.leg_parameters_symmetric.LEG_PARAMETERS_SYMMETRIC
    """
    v = normalize_model_variant(model_variant)
    if v == "real":
        from dog2_motion_control.leg_parameters import LEG_PARAMETERS
        return LEG_PARAMETERS
    from dog2_motion_control.leg_parameters_symmetric import LEG_PARAMETERS_SYMMETRIC
    return LEG_PARAMETERS_SYMMETRIC


def reload_leg_parameter_joint_limits(model_variant: str, force_reload: bool = False) -> str:
    """Refresh the LegParameters joint limits for the selected model variant."""
    v = normalize_model_variant(model_variant)
    if v == "real":
        from dog2_motion_control.leg_parameters import reload_leg_parameter_joint_limits_from_urdf
    else:
        from dog2_motion_control.leg_parameters_symmetric import (
            reload_leg_parameter_joint_limits_from_urdf,
        )
    return reload_leg_parameter_joint_limits_from_urdf(force_reload=force_reload)
