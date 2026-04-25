from dog2_gait_planner.gait_scheduler_node import compute_phase_array
from dog2_interfaces.msg import ContactPhase


def test_stationary_all_stance():
    assert compute_phase_array("trot", 0.1, False) == [ContactPhase.STANCE] * 4


def test_trot_alternates_pairs():
    assert compute_phase_array("trot", 0.1, True) == [
        ContactPhase.STANCE,
        ContactPhase.SWING,
        ContactPhase.STANCE,
        ContactPhase.SWING,
    ]
