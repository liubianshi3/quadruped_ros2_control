"""IK/FK symmetric roundtrip and mirror-invariance tests for symmetric model."""

from __future__ import annotations

import numpy as np
import pytest

from dog2_motion_control.kinematics_solver import create_kinematics_solver
from dog2_motion_control.leg_parameters_symmetric import LEG_PARAMETERS_SYMMETRIC

EPS = 1e-4


@pytest.fixture
def sym_solver():
    return create_kinematics_solver(model_variant="symmetric")


@pytest.fixture
def real_solver():
    return create_kinematics_solver(model_variant="real")


def test_symmetric_solver_created():
    solver = create_kinematics_solver(model_variant="symmetric")
    assert solver is not None


def test_real_solver_default():
    solver = create_kinematics_solver()
    assert solver is not None


def test_symmetric_base_positions():
    params = LEG_PARAMETERS_SYMMETRIC
    assert np.allclose(params["lf"].base_position, [-0.122125, -0.06, 0.0], atol=EPS)
    assert np.allclose(params["lh"].base_position, [0.122125, -0.06, 0.0], atol=EPS)
    assert np.allclose(params["rh"].base_position, [0.122125, 0.06, 0.0], atol=EPS)
    assert np.allclose(params["rf"].base_position, [-0.122125, 0.06, 0.0], atol=EPS)


def _safe_joints():
    return (0.0, 0.0, 0.30, -0.50)


def test_fk_all_legs(sym_solver):
    joints = _safe_joints()
    for leg_id in ("lf", "lh", "rh", "rf"):
        pos = sym_solver.solve_fk(leg_id, joints)
        assert len(pos) == 3
        assert all(isinstance(v, float) for v in pos)


def test_ik_roundtrip_all_legs(sym_solver):
    joints = _safe_joints()
    for leg_id in ("lf", "lh", "rh", "rf"):
        fk_pos = sym_solver.solve_fk(leg_id, joints)
        ik_result = sym_solver.solve_ik(leg_id, fk_pos, rail_offset=0.0)
        assert ik_result is not None, f"{leg_id}: IK returned None at {fk_pos}"
        rail_m, hr, hp, kp = ik_result
        fk2_pos = sym_solver.solve_fk(leg_id, (rail_m, hr, hp, kp))
        dist = float(np.linalg.norm(np.array(fk_pos) - np.array(fk2_pos)))
        assert dist < 0.01, f"{leg_id}: FK(IK(FK(q))) roundtrip error {dist:.6f}"


def test_mirror_symmetry_lf_vs_rf(sym_solver):
    sym_params = LEG_PARAMETERS_SYMMETRIC
    lf_base = sym_params["lf"].base_position
    rf_base = sym_params["rf"].base_position
    assert abs(lf_base[0] - rf_base[0]) < EPS, f"lf/rf X mismatch"
    assert abs(lf_base[1] + rf_base[1]) < EPS, f"lf/rf Y not mirrored"

    lh_base = sym_params["lh"].base_position
    rh_base = sym_params["rh"].base_position
    assert abs(lh_base[0] - rh_base[0]) < EPS, f"lh/rh X mismatch"
    assert abs(lh_base[1] + rh_base[1]) < EPS, f"lh/rh Y not mirrored"

    assert abs(lf_base[0] + lh_base[0]) < EPS, f"front/rear X not mirrored"

    joints = _safe_joints()
    lf_pos = sym_solver.solve_fk("lf", joints)
    rf_pos = sym_solver.solve_fk("rf", joints)
    print(f"[INFO] lf foot: ({lf_pos[0]:.4f}, {lf_pos[1]:.4f}, {lf_pos[2]:.4f})")
    print(f"[INFO] rf foot: ({rf_pos[0]:.4f}, {rf_pos[1]:.4f}, {rf_pos[2]:.4f})")
    # All foot Z should be the same (level ground at q=0)
    assert abs(lf_pos[2] - rf_pos[2]) < 0.01, f"lf/rf foot Z mismatch"
    # X positions should be reasonably close (same rail=0, same joint config)
    assert abs(lf_pos[0] - rf_pos[0]) < 0.06, f"lf/rf foot X too far: {lf_pos[0]:.4f} vs {rf_pos[0]:.4f}"


def test_symmetric_foot_tip_leg3_leg4_identical():
    sym_params = LEG_PARAMETERS_SYMMETRIC
    f3 = sym_params["rh"].foot_tip_offset_tibia
    f4 = sym_params["rf"].foot_tip_offset_tibia
    assert np.allclose(f3, f4, atol=EPS), f"leg3/leg4 foot_tip differ: {f3} vs {f4}"
