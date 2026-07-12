import pytest

from dog2_gait_planner.gait_scheduler_node import (
    ContactAwareCrawl,
    compute_phase_array,
    crawl_body_shift,
)
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


def test_crawl_never_swings_more_than_one_foot():
    for phase in [index / 40.0 for index in range(40)]:
        phases = compute_phase_array("crawl", phase, True, duty=0.75)
        assert phases.count(ContactPhase.SWING) == 1
        assert phases.count(ContactPhase.STANCE) == 3


def test_crawl_body_shift_moves_away_from_unloaded_corner():
    lf_shift = crawl_body_shift(0)
    lh_shift = crawl_body_shift(1)
    rh_shift = crawl_body_shift(2)
    rf_shift = crawl_body_shift(3)

    assert lf_shift[0] > 0.0 and lf_shift[1] > 0.0
    assert lh_shift[0] < 0.0 and lh_shift[1] > 0.0
    assert rh_shift[0] < 0.0 and rh_shift[1] < 0.0
    assert rf_shift[0] > 0.0 and rf_shift[1] < 0.0


def make_crawl(**overrides) -> ContactAwareCrawl:
    parameters = dict(
        pre_shift_sec=0.4,
        swing_sec=0.4,
        settle_sec=0.2,
        max_swing_sec=1.0,
        max_shift_sec=8.0,
        max_settle_sec=3.0,
        body_shift_scale=0.65,
        contact_aware=True,
    )
    parameters.update(overrides)
    return ContactAwareCrawl(**parameters)


def test_contact_aware_crawl_requires_release_then_touchdown():
    crawl = make_crawl()

    shifted = crawl.update(0.4, True, [True] * 4)
    assert shifted.state == "SWING"
    assert shifted.active_leg == 0  # front feet centre the support first

    # A clock edge alone cannot declare touchdown if the foot never released.
    still_swing = crawl.update(0.5, True, [True] * 4)
    assert still_swing.state == "SWING"

    crawl.update(0.1, True, [False, True, True, True])
    touchdown = crawl.update(0.1, True, [True] * 4)
    assert touchdown.state == "SETTLE"
    assert touchdown.body_shift_x == pytest.approx(shifted.body_shift_x)
    assert touchdown.body_shift_y == pytest.approx(shifted.body_shift_y)

    next_leg = crawl.update(0.2, True, [True] * 4)
    assert next_leg.state == "SHIFT"
    assert next_leg.active_leg == 3
    assert next_leg.body_shift_x == pytest.approx(touchdown.body_shift_x)
    assert next_leg.body_shift_y == pytest.approx(touchdown.body_shift_y)


def test_swing_past_max_swing_is_forced_into_settle():
    crawl = make_crawl(
        pre_shift_sec=0.1, swing_sec=0.2, settle_sec=0.1, max_swing_sec=0.3
    )
    crawl.update(0.1, True, [True] * 4)
    crawl.update(0.1, True, [False, True, True, True])
    forced = crawl.update(0.3, True, [False, True, True, True])

    assert forced.state == "SETTLE"
    assert forced.event == "forced_settle"
    # The commanded phases return the leg to stance so the MPC/WBC ground
    # search presses it down instead of the swing PD holding it in the air.
    assert forced.phases.count(ContactPhase.SWING) == 0


def test_forced_settle_still_requires_all_contact_before_next_shift():
    crawl = make_crawl(
        pre_shift_sec=0.1, swing_sec=0.2, settle_sec=0.1, max_swing_sec=0.3
    )
    crawl.update(0.1, True, [True] * 4)
    crawl.update(0.1, True, [False, True, True, True])
    crawl.update(0.3, True, [False, True, True, True])

    hovering = crawl.update(0.2, True, [False, True, True, True])
    assert hovering.state == "SETTLE"
    assert hovering.event == ""

    grounded = crawl.update(0.05, True, [True] * 4)
    assert grounded.state == "SHIFT"
    assert grounded.active_leg == 3


def test_shift_timeout_latches_safe_standing_fault():
    crawl = make_crawl(pre_shift_sec=0.1, max_shift_sec=0.5)
    held = crawl.update(0.4, True, [True] * 4, shift_ready=False)
    assert held.state == "SHIFT"

    faulted = crawl.update(0.2, True, [True] * 4, shift_ready=False)
    assert faulted.state == "FAULT"
    assert faulted.event == "shift_timeout_fault"
    assert faulted.phases == [ContactPhase.STANCE] * 4
    # The body-shift command freezes rather than snapping to zero, which
    # previously caused a 2-3 cm lateral reference reversal.
    assert faulted.body_shift_x == pytest.approx(held.body_shift_x)
    assert faulted.body_shift_y == pytest.approx(held.body_shift_y)

    # FAULT is absorbing while the walk command persists, even if the gate
    # later reports ready.
    still = crawl.update(5.0, True, [True] * 4, shift_ready=True)
    assert still.state == "FAULT"
    assert still.event == ""
    assert still.body_shift_x == pytest.approx(held.body_shift_x)

    # A stop command is the only reset path.
    stood = crawl.update(0.1, False, [True] * 4)
    assert stood.state == "STAND"
    restarted = crawl.update(0.2, True, [True] * 4, shift_ready=True)
    assert restarted.state == "SWING"


def test_settle_timeout_latches_safe_standing_fault():
    crawl = make_crawl(
        pre_shift_sec=0.1,
        swing_sec=0.2,
        settle_sec=0.1,
        max_swing_sec=0.3,
        max_settle_sec=0.5,
    )
    crawl.update(0.1, True, [True] * 4)
    crawl.update(0.1, True, [False, True, True, True])
    crawl.update(0.3, True, [False, True, True, True])

    faulted = crawl.update(0.6, True, [False, True, True, True])
    assert faulted.state == "FAULT"
    assert faulted.event == "settle_timeout_fault"
    assert faulted.phases == [ContactPhase.STANCE] * 4


def test_shift_ready_at_deadline_prefers_progress_over_fault():
    crawl = make_crawl(pre_shift_sec=0.1, max_shift_sec=0.5)
    crawl.update(0.4, True, [True] * 4, shift_ready=False)
    progressed = crawl.update(0.2, True, [True] * 4, shift_ready=True)
    assert progressed.state == "SWING"


def test_contact_aware_crawl_waits_for_measured_body_shift():
    crawl = make_crawl(
        pre_shift_sec=0.1, swing_sec=0.2, settle_sec=0.1, max_swing_sec=0.3
    )
    held = crawl.update(0.2, True, [True] * 4, shift_ready=False)
    assert held.state == "SHIFT"

    released = crawl.update(0.01, True, [True] * 4, shift_ready=True)
    assert released.state == "SWING"


def test_contact_array_shape_is_checked():
    crawl = make_crawl(
        pre_shift_sec=0.1, swing_sec=0.2, settle_sec=0.1, max_swing_sec=0.3
    )
    with pytest.raises(ValueError):
        crawl.update(0.1, True, [True, True])
