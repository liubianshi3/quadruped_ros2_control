"""Shared Dog2 joint semantic contract.

This module centralizes the local-axis and fixed-origin conventions that the
URDF and motion-control geometry are expected to share.

**Authoritative topology description** lives in the xacro comment block
``Leg kinematic contract`` inside ``dog2_description/urdf/dog2.urdf.xacro``
(P – R_z – R_y – R_y: rail +q along base_link +X; at q=0, coxa +q is yaw about
base_link −Z; femur and tibia +q are pitch about base_link −Y, parallel at nominal pose).

Before changing axes or limits, expand the xacro and run
``dog2_description/scripts/check_joint_semantics.py``.
"""

from __future__ import annotations

import math
import numpy as np


HALF_PI = math.pi / 2.0

# Fixed leg-root orientation used by the normalized rail joints.
LEG_BASE_RPY = np.array([HALF_PI, 0.0, 0.0], dtype=float)

# Fixed revolute joint origin rotations shared by all four legs.
COXA_ORIGIN_RPY = np.array([0.0, 0.0, HALF_PI], dtype=float)
FEMUR_ORIGIN_RPY = np.array([HALF_PI, HALF_PI, 0.0], dtype=float)
TIBIA_ORIGIN_RPY = np.array([0.0, 0.0, 0.0], dtype=float)

# Unit axes in each joint’s URDF joint frame (same strings as <axis xyz> in
# dog2.urdf.xacro). KinematicsSolver composes them after fixed origins; base
# semantics are documented in the xacro “Leg kinematic contract” block.
JOINT_AXIS_LOCAL = {
    "rail": np.array([1.0, 0.0, 0.0], dtype=float),
    "coxa": np.array([-1.0, 0.0, 0.0], dtype=float),
    "femur": np.array([-1.0, 0.0, 0.0], dtype=float),
    "tibia": np.array([-1.0, 0.0, 0.0], dtype=float),
}

# Per-leg prismatic limits: +q slides rail_link along base_link +X (URDF mirrors
# local axis on right legs so R_rail * axis_joint = +X; see xacro leg_rail_axis_*).
RAIL_LIMITS_BY_LEG = {
    "lf": (0.0, 0.111),
    "lh": (-0.111, 0.0),
    "rh": (0.0, 0.111),
    "rf": (-0.111, 0.0),
}

# URDF prismatic joint-frame axis (matches dog2.urdf.xacro rail_axis macro arg).
RAIL_AXIS_JOINT_LOCAL_BY_LEG = {
    "lf": np.array([1.0, 0.0, 0.0], dtype=float),
    "lh": np.array([1.0, 0.0, 0.0], dtype=float),
    "rh": np.array([-1.0, 0.0, 0.0], dtype=float),
    "rf": np.array([-1.0, 0.0, 0.0], dtype=float),
}

# Human-readable contract for code review / logging / future regression checks.
JOINT_SEMANTICS = {
    "rail": "+q translates along base_link +X (prismatic leg shift)",
    "coxa": "hip yaw (R_z family): +q about −Z in base_link at q=0 (all legs)",
    "femur": "thigh pitch (R_y family): +q about −Y in base_link at q=0",
    "tibia": "knee pitch (R_y family): +q about −Y in base_link at q=0; axis parallel to femur nominally",
}
