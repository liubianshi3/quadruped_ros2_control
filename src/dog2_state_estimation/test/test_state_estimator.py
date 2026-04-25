from dog2_state_estimation.sim_state_estimator_node import (
    build_snapshot,
    infer_contact_state,
)


def test_infer_contact_state_defaults_true_with_inputs():
    assert infer_contact_state("all_true", object(), object()) == [True, True, True, True]


def test_infer_contact_state_without_inputs():
    assert infer_contact_state("all_true", None, None) == [False, False, False, False]


def test_build_snapshot_counts_joint_names():
    class FakeJointState:
        name = ["lf_rail_joint", "lf_coxa_joint"]

    snapshot = build_snapshot(
        "sim_ground_truth",
        "gazebo_ground_truth",
        [True] * 4,
        FakeJointState(),
    )
    assert snapshot.estimator_mode == "sim_ground_truth"
    assert snapshot.source == "gazebo_ground_truth"
    assert snapshot.joint_count == 2
    assert snapshot.contact_state == [True, True, True, True]
