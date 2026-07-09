"""Pipeline leg-order guard.

Asserts that every 4-element leg-prefix array in MPC/WBC/gait/mux/viz/state-est
matches ``LEG_ORDER = ('lf','lh','rh','rf')``. Legacy position-mode controller
and Dog2Model::FOOT_NAMES intentionally use a different order and are out of
scope (whitelisted below).
"""

from __future__ import annotations

import re
from pathlib import Path

# Inlined to avoid pulling in the ROS-dependent package __init__ during pytest.
# Must stay in sync with dog2_motion_control.joint_names.LEG_ORDER.
LEG_ORDER = ("lf", "lh", "rh", "rf")

REPO_SRC = Path(__file__).resolve().parents[2]

# Pipeline files that MUST match LEG_ORDER. Anything not on this list is
# unchecked — keep the list explicit so additions are deliberate.
PIPELINE_FILES = [
    "dog2_mpc/src/mpc_node_complete.cpp",
    "dog2_wbc/src/wbc_controller.cpp",
    "dog2_wbc/src/wbc_node_complete.cpp",
    "dog2_wbc/test/test_wbc_urdf_jacobian.cpp",
    "dog2_mpc/test/test_mpc_urdf_model_data.cpp",
    "dog2_gait_planner/dog2_gait_planner/swing_target_node.py",
    "dog2_gait_planner/dog2_gait_planner/gait_scheduler_node.py",
    "dog2_state_estimation/dog2_state_estimation/sim_state_estimator_node.py",
    "dog2_visualization/dog2_visualization/visualization_node.py",
    "dog2_bringup/dog2_bringup/wbc_effort_mux.py",
    "dog2_bringup/dog2_bringup/mpc_debug_adapter.py",
    "dog2_bringup/launch/effort_research_sim.launch.py",
    "dog2_motion_control/dog2_motion_control/mpc_robot_controller.py",
    "dog2_motion_control/launch/spider_gazebo_mpc.launch.py",
]

# Match {"lf"...,"rf"} or ("lf",...,"rf") with the four leg prefixes in any
# order. Quote style and whitespace are tolerated; suffix _foot_link, _joint,
# etc. are also matched as long as the prefix sequence is exposed.
_QUOTE = r"['\"]"
_PREFIX = r"(lf|lh|rh|rf)"
_SUFFIX = r"(?:_[a-z_]+)?"
_ITEM = rf"{_QUOTE}{_PREFIX}{_SUFFIX}{_QUOTE}"
_SEP = r"\s*,\s*"
LEG_ARRAY_RE = re.compile(
    rf"{_ITEM}{_SEP}{_ITEM}{_SEP}{_ITEM}{_SEP}{_ITEM}",
    re.MULTILINE,
)


def test_pipeline_leg_order_matches_canonical():
    canonical = list(LEG_ORDER)
    violations = []
    for rel in PIPELINE_FILES:
        path = REPO_SRC / rel
        assert path.exists(), f"missing pipeline file: {rel}"
        text = path.read_text()
        for match in LEG_ARRAY_RE.finditer(text):
            order = list(match.groups())
            if order != canonical:
                line_no = text[: match.start()].count("\n") + 1
                violations.append(f"{rel}:{line_no}  got {order}  expected {canonical}")
    assert not violations, "leg-order mismatches on pipeline path:\n  " + "\n  ".join(violations)


if __name__ == "__main__":
    test_pipeline_leg_order_matches_canonical()
    print("[PASS] pipeline leg order matches LEG_ORDER")
